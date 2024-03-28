from binascii import hexlify
from typing import Dict


DLC = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64]
STUFFBIT_COUNT_CODING = [0x0, 0x03, 0x6, 0x05, 0xc, 0xf, 0xa, 0x9]
class CANFDFrame:
    """
    Class for calculating the bitstream of a CAN FD frame.
    """
    FIELD_NAMES = ['sof',
                   'ida',
                   'srr',
                   'ide',
                   'idb',
                   'rrs',
                   'fdf',
                   'res',
                   'brs',
                   'esi',
                   'dlc',
                   'data',
                   'crc',
                   'crc_delimiter',
                   'ack',
                   'eof',
                   'ifs']

    RED = "\x1b[31m"
    GREEN = "\x1b[32m"
    YELLOW = "\x1b[33m"
    BLUE = "\x1b[34m"
    MAGENTA = "\x1b[35m"
    CYAN = "\x1b[36m"
    WHITE = "\x1b[37m"
    RESET = "\x1b[0m"

    STUFFED_FIELDS = ['sof', 'ida', 'idb', 'ide', 'srr', 'rrs', 'fdf', 'res', 'brs', 'esi', 'dlc', 'data']
    FIELD_COLORS = {
        'sof': WHITE,
        'ida': MAGENTA,
        'srr': MAGENTA,
        'idb': MAGENTA,
        'rrs': MAGENTA,
        'ide': MAGENTA,
        'fdf': GREEN,
        'res': GREEN,
        'brs': GREEN,
        'esi': GREEN,
        'dlc': GREEN,
        'data': CYAN,
        'crc': WHITE,
        'crc_delimiter': WHITE,
        'ack': YELLOW,
        'ack_delimiter': YELLOW,
        'eof': GREEN,
        'ifs': BLUE
    }
    FIELD_DESCRIPTIONS = {
        'sof': "Start of frame",
        'ida': "11 bit ID",
        'srr': "SRR",
        'idb': "18 bit ID extension",
        'rrs': "RRS",
        'ide': "IDE",
        'res': "Reserved",
        'fdf': "FDF",
        'brs': "BRS",
        'esi': "ESI",
        'dlc': "DLC",
        'data': "Data",
        'crc': "CRC",
        'crc_delimiter': "CRC delimiter",
        'ack': "ACK",
        'ack_delimiter': "ACK delimiter",
        'eof': "EOF",
        'ifs': "IFS"
    }

    def __init__(self, id_a: int, data: bytes = bytes([]), ide=False, rtr=False, fdf=False, brs=False, esi=False, id_b=0, ack=0):
        """
        :param id_a: The 11-bit part of a CAN ID
        :param id_b: The 18-bit extended part of the ID if an extended ID frame
        :param data: 0-64 bytes for the CAN payload, the DLC is automatically set to the length of the data field if not a remote frame
        :param ide: True if the CAN ID is 29 bits, false if 11 bits
        :param rtr: True if a CAN remote frame
        :param fdf: True if FD format framne
        :param brs: True if bit rate switch
        :param esi: True if error state
        :param ack: The value of the ACK field (defaults to 0 to emulate another controller acknowledging the frame)
        """
        self.id_a = id_a & 0x7ff
        self.id_b = id_b & 0x3ffff
        self.ack = ack

        if len(data) not in DLC:
            raise ValueError("Data field must be 0..8, 12, 16, 20, 24, 32, 48 or 64 bytes")

        self.ide = int(ide)
        if fdf == True:            
            self.rrs = 0 # CAN FD does not support Remote Frames
        else:
            self.rrs = int(rtr)
            if rtr and len(data) != 0:
                raise ValueError("Cannot have remote frames with data")
        
        self.fdf = int(fdf)
        self.brs = int(brs)
        self.esi = int(esi)
        self.data = data
        self.fields_position = {}
        self.fields = {}

        self.dlc = len(data)

        self._stuff_bit_count = 0
        self._1_bits_in_row = 0
        self._0_bits_in_row = 0

        self.unstuffed_bitstream = ''
        self.stuffed_bitstream = ''
        self.frame_bits = []

        # CRC polynomial seed
        if len(data) <= 16:
            self._crc_rg = 0x10000
        else:
            self._crc_rg = 0x100000

        # Compute the bitstream
        self._to_bitstream()

    def print_field_str(self, fieldname: str) -> str:
        """
        Prints a field of the CAN FD frame.

        :param fieldname: name of the field
        :return:
        """
        start_finish = self.fields_position[fieldname]  # type: range
        description = self.FIELD_DESCRIPTIONS[fieldname]
        value = ""
        if fieldname == 'crc':
            value = "(0x{:04x})".format(self._crc_rg)
        if fieldname == 'data':
            if len(self.fields['data']) == 0:
                description = "(No data field)"
            else:
                value = "({})".format(self.data)
        if fieldname == 'dlc':
            value = "({})".format(self.dlc)
        if fieldname == 'ide':
            value = "(1=Extended ID)" if self.ide else "(0=Standard ID)"
        if fieldname == 'ida':
            value = "(0x{:03x})".format(self.id_a)
        if fieldname == 'idb':
            value = "(0x{:05x}, full ID=0x{:08x})".format(self.id_b, self.id_a << 18 | self.id_b)
        if fieldname == 'rtr':
            value = "(1=Remote frame)" if self.rrs else "(0=Data frame)"
        if fieldname == 'fdf':
            value = "(1=CAN FD)" if self.fdf else "(0=CAN CC)"
        if fieldname == 'brs':
            value = "(1=Data transferred using data baudrate)" if self.brs else "(0=Data transferred using arbitartion baudrate)"
        if fieldname == 'esi':
            value = "(1=Error passive)" if self.esi else "(0=Error active)"

        result = "{}{} {} {}".format(" " * start_finish.start, "^" * len(start_finish), description, value)
        return result

    def _crc17_bit(self, bit: str):
        """
        Calculate the CRC17 from each bit of the frame.
        """
        if bit not in ['0', '1']:
            raise AssertionError("Illegal bit value")
        nxtbit = 1 if bit == '1' else 0
        bit_16 = (self._crc_rg & (1 << 16)) >> 16
        crcnxt = nxtbit ^ bit_16
        assert crcnxt in [0, 1]
        self._crc_rg <<= 1
        self._crc_rg &= 0x1ffff

        if crcnxt == 1:
            self._crc_rg ^= 0x1685b

        assert self._crc_rg <= 0x1ffff

    def _crc21_bit(self, bit: str):
        """
        Calculate the CRC21 from each bit of the frame.
        """
        if bit not in ['0', '1']:
            raise AssertionError("Illegal bit value")
        nxtbit = 1 if bit == '1' else 0
        bit_20 = (self._crc_rg & (1 << 20)) >> 20
        crcnxt = nxtbit ^ bit_20
        assert crcnxt in [0, 1]
        self._crc_rg <<= 1
        self._crc_rg &= 0x1fffff

        if crcnxt == 1:
            self._crc_rg ^= 0x102899

        assert self._crc_rg <= 0x1fffff

    def _add_stuff_bit(self, bit: str):
        if bit == '0':
            self._0_bits_in_row += 1
            self._1_bits_in_row = 0
        else:
            self._1_bits_in_row += 1
            self._0_bits_in_row = 0

        if self._0_bits_in_row == 5:
            self.frame_bits.append(True)
            self._calc_crc(bit='1')
            self.stuffed_bitstream += self.RED + '1' + self.RESET
            self._1_bits_in_row = 1
            self._0_bits_in_row = 0
            self._stuff_bit_count = (self._stuff_bit_count + 1) % 8
        if self._1_bits_in_row == 5:
            self.frame_bits.append(False)
            self._calc_crc(bit='0')
            self.stuffed_bitstream += self.RED + '0' + self.RESET
            self._0_bits_in_row = 1
            self._1_bits_in_row = 0
            self._stuff_bit_count = (self._stuff_bit_count + 1) % 8

    def _calc_crc(self, bit: str):
        """
        Calculate the CRC from each bit of the frame.
        """
        if len(self.data) <= 16:
            self._crc17_bit(bit)
        else:
            self._crc21_bit(bit)

    def _add_fixed_stuff_bits(self, unstuffed_bitstream : str) -> str:
        result = ''
        
        last_data_bit = self.frame_bits[-1]

        if last_data_bit == True:
            self.frame_bits.append(False)
            result = self.RED + '0' + self.RESET
        else:
            self.frame_bits.append(True)
            result = self.RED + '1' + self.RESET

        chunks, chunk_size = len(unstuffed_bitstream), 4
        for chunk in [ unstuffed_bitstream[i:i+chunk_size] for i in range(0, chunks, chunk_size) ]:
            for bit in range(0, len(chunk)):
                self.frame_bits.append(True if chunk[bit] == '1' else False)

            if len(chunk) == 4:
                if chunk[-1] == '0':
                    chunk += self.RED + '1' + self.RESET
                    self.frame_bits.append(True)
                else:
                    chunk += self.RED + '0' + self.RESET
                    self.frame_bits.append(False)

            result += chunk

        
        return result

    def _to_bitstream(self):
        self.fields = {
            'sof': '0',
            'rrs': '1' if self.rrs else '0',
            'fdf': '1' if self.fdf else '0',
            'res': '0',
            'brs': '1' if self.brs else '0',
            'esi': '1' if self.esi else '0',
            'dlc': "{:04b}".format(self.dlc),
            'data': "".join(["{:08b}".format(byte) for byte in self.data]),
            'crc_delimiter': '1',
            'ack': '1' if self.ack == 1 else '0',
            'ack_delimiter': '1',
            'eof': '1111111',
            'ifs': '111'
        }

        if self.ide:
            self.fields['ida'] = "{:011b}".format(self.id_a)
            self.fields['srr'] = '1'
            self.fields['ide'] = '1'
            self.fields['idb'] = "{:018b}".format(self.id_b)
            self.fieldnames = ['sof', 'ida', 'srr', 'ide', 'idb', 'rrs']
        else:
            self.fields['ida'] = "{:011b}".format(self.id_a)
            self.fields['ide'] = '0'
            self.fieldnames = ['sof', 'ida', 'rrs', 'ide']
        self.fieldnames += ['fdf', 'res', 'brs', 'esi', 'dlc', 'data', 'crc', 'crc_delimiter', 'ack', 'ack_delimiter', 'eof', 'ifs']

        for fieldname in self.fieldnames:

            start = len(self.frame_bits)

            if fieldname == 'crc':
                _crc_unstuffed_bitstream = "{:04b}".format(STUFFBIT_COUNT_CODING[self._stuff_bit_count])
                # CRC includes the SBC (https://www.mikrocontroller.net/topic/420643)
                for bit in range(0, len(_crc_unstuffed_bitstream)):
                    self._calc_crc(_crc_unstuffed_bitstream[bit])

                if len(self.data) <= 16:
                    _crc_unstuffed_bitstream += "{:017b}".format(self._crc_rg)
                else:
                    _crc_unstuffed_bitstream += "{:021b}".format(self._crc_rg)

                self.unstuffed_bitstream += _crc_unstuffed_bitstream

                _crc_stuffed_bitstream = self._add_fixed_stuff_bits(_crc_unstuffed_bitstream)
                self.stuffed_bitstream += _crc_stuffed_bitstream

            else:
                color = self.FIELD_COLORS[fieldname]
                stuffed = fieldname in self.STUFFED_FIELDS

                self.fields_position[fieldname] = [len(self.frame_bits), 0]
                for bit in self.fields[fieldname]:
                    self.unstuffed_bitstream += color + bit + self.RESET
                    self.stuffed_bitstream += color + bit + self.RESET
                    self.frame_bits.append(True if bit == '1' else False)
                    if stuffed:
                        self._calc_crc(bit=bit)
                        self._add_stuff_bit(bit=bit)
            
            self.fields_position[fieldname] = range(start, len(self.frame_bits))

    def print(self, detailed=False):
        """
        Prints the CAN FD frame. Different colours are used for the frame fields.
        Stuff bits are marked in red.

        :param detailed: if set to True then the fields are described
        :return:
        """
        print(self.print_str(detailed=detailed))
    
    def print_str(self, detailed=False) -> str:
        result = self.stuffed_bitstream + '\n'
        if detailed:
            for fieldname in self.fieldnames:
                result += self.print_field_str(fieldname=fieldname) + '\n'
        return result

    def bitseq(self) -> str:
        return "".join(['1' if bit else '0' for bit in self.frame_bits])

    def __repr__(self):
        return self.print_str(detailed=True)

    @staticmethod
    def from_bitseq(bitseq: str) -> Dict:
        """
        Decode a sequence of bits and return a CANFDFrame object
        :param bitseq: string containing a sequence of '0' and '1'
        :return: a dictionary of fields that have been decoded, and a CANFrame from the bit sequence
        """
        frame = {
            'ida': '',
            'srr': '',
            'ide': '',
            'idb': '',
            'rrs': '',
            'fdf': '',
            'res': '',
            'brs': '',
            'esi': '',
            'dlc': '',
            'data': '',
            'crc': '',
            'crc_delimiter': '',
            'ack': '',
            'ack_delimiter': '',
            'eof': '',
            'ifs': ''
        }
        decoded = False
        stuffing = False
        byte_count = 0
        field = 'sof'
        last_6 = ''
        for bit_num in range(len(bitseq)):
            bit = bitseq[bit_num]
            last_6 = last_6[-5:] + bit
            if stuffing:
                if last_6 == '111111' or last_6 == '000000':
                    frame['stuff_error'] = bit_num
                    break
                elif last_6 == '111110' or last_6 == '000001':
                    continue
            # Look for SOF bit
            if field == 'sof' and bit == '0':
                stuffing = True
                field = 'ida'
            elif field == 'ida':
                frame[field] += bit
                if len(frame[field]) == 11:
                    field = 'srr'
            elif field == 'srr':
                frame[field] = bit
                field = 'ide'
            elif field == 'ide':
                frame[field] = bit
                if bit == '1':
                    field = 'idb'
                else:
                    field = 'fdf'
                    frame['rrs'] = frame['srr']
            elif field == 'idb':
                frame[field] += bit
                if len(frame[field]) == 18:
                    field = 'rrs'
            elif field == 'rrs':
                frame[field] = bit
                field = 'fdf'
            elif field == 'fdf':
                frame[field] = bit
                field = 'res'
            elif field == 'res':
                frame[field] = bit
                field = 'brs'
            elif field == 'brs':
                frame[field] = bit
                field = 'esi'
            elif field == 'esi':
                frame[field] = bit
                field = 'dlc'
            elif field == 'dlc':
                frame[field] += bit
                if len(frame[field]) == 4:
                    byte_count = DLC[int(frame[field], base=2)]
                    if byte_count == 0:
                        field = 'crc'
                    else:
                        field = 'data'
            elif field == 'data':
                frame[field] += bit
                if len(frame[field]) == byte_count * 8:
                    stuffing = False
                    field = 'crc'
            elif field == 'crc':
                frame[field] += bit
                if (byte_count <= 16) and len(frame[field]) == 17 + 6 + 4:
                    field = 'crc_delimiter'
                if (byte_count > 20) and len(frame[field]) == 21 + 7 + 4:
                    field = 'crc_delimiter'
            elif field == 'crc_delimiter':
                frame[field] = bit
                field = 'ack'
            elif field == 'ack':
                frame[field] = bit
                field = 'ack_delimiter'
            elif field == 'ack_delimiter':
                frame[field] = bit
                field = 'eof'
            elif field == 'eof':
                frame[field] += bit
                if len(frame[field]) == 7:
                    field = 'ifs'
            elif field == 'ifs':
                if frame[field] == '11':
                    if bit == '1':
                        frame[field] += bit
                    decoded = True
                    break
                else:
                    frame[field] += bit

        if decoded:
            id_a = int(frame['ida'], base=2)
            ide = True if frame['ide'] == '1' else False
            fdf = True if frame['fdf'] == '1' else False
            brs = True if frame['brs'] == '1' else False
            esi = True if frame['esi'] == '1' else False
            dlc = int(frame['dlc'], base=2)
            id_b = 0 if not ide else int(frame['idb'], base=2)
            num_bytes = DLC[dlc]

            data = []
            data_str = frame['data']
            for i in range(num_bytes):
                data.append(int(data_str[:8], base=2))
                data_str = data_str[8:]
            frame['can_frame'] = CANFDFrame(id_a=id_a, ide=ide, fdf=fdf, brs=brs, esi=esi, id_b=id_b, data=bytes(data))
        return frame


if __name__ == '__main__':
    f = CANFDFrame(id_a=0x560, fdf=1, data=bytes([0xA6,0x59,0x6C,0xDF,0xB2,0xE5]))
    f.print(detailed=True)

    frame = CANFDFrame.from_bitseq(f.bitseq())
    frame['can_frame'].print(detailed=True)

