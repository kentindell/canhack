from binascii import hexlify
from typing import Dict


class CANFrame:
    """
    Class for calculating the bitstream of a CAN frame.
    """
    FIELD_NAMES = ['sof',
                   'ida',
                   'idb',
                   'ide',
                   'srr',
                   'rtr',
                   'r0',
                   'r1',
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
    RESET = "\x1b[0m"

    STUFFED_FIELDS = ['sof', 'ida', 'idb', 'ide', 'srr', 'rtr', 'r0', 'r1', 'dlc', 'data', 'crc']
    CRCD_FIELDS = ['sof', 'ida', 'idb', 'ide', 'srr', 'rtr', 'r0', 'r1', 'dlc', 'data']
    FIELD_COLORS = {
        'sof': BLUE,
        'ida': GREEN,
        'idb': GREEN,
        'srr': YELLOW,
        'rtr': CYAN,
        'ide': MAGENTA,
        'r0': YELLOW,
        'r1': CYAN,
        'dlc': BLUE,
        'data': GREEN,
        'crc': MAGENTA,
        'crc_delimiter': YELLOW,
        'ack': CYAN,
        'ack_delimiter': MAGENTA,
        'eof': GREEN,
        'ifs': YELLOW
    }
    FIELD_DESCRIPTIONS = {
        'sof': "Start of frame",
        'ida': "11 bit ID",
        'idb': "18 bit ID extension",
        'srr': "SRR",
        'rtr': "RTR",
        'ide': "IDE",
        'r0': "Reserved 0",
        'r1': "Reserved 1",
        'dlc': "DLC",
        'data': "Data",
        'crc': "CRC",
        'crc_delimiter': "CRC delimiter",
        'ack': "ACK",
        'ack_delimiter': "ACK delimiter",
        'eof': "EOF",
        'ifs': "IFS"
    }

    def __init__(self, id_a: int, data: bytes = bytes([]), ide=False, rtr=False, dlc=0, id_b=0, ack=0):
        """
        :param id_a: The 11-bit part of a CAN ID
        :param id_b: The 18-bit extended part of the ID if an extended ID frame
        :param data: 0-8 bytes for the CAN payload
        :param ide: True if the CAN ID is 29 bits, false if 11 bits
        :param rtr: True if a CAN remote frame
        :param dlc: The DLC is automatically set to the length of the data field if not a remote frame
        :param ack: The value of the ACK field (defaults to 0 to emulate another controller acknowledging the frame)
        """
        self.id_a = id_a & 0x7ff
        self.id_b = id_b & 0x3ffff
        self.ack = ack

        if rtr and len(data) != 0:
            raise ValueError("Cannot have remote frames with data")
        if len(data) not in range(9):
            raise ValueError("Data field must be 0..8 bytes")

        self.ide = int(ide)
        self.rtr = int(rtr)
        self.data = data
        self.fields_position = {}
        self.fields = {}

        if rtr:
            self.dlc = dlc & 0xf
        else:
            self.dlc = len(data)

        self._1_bits_in_row = 0
        self._0_bits_in_row = 0

        self.unstuffed_bitstream = ''
        self.stuffed_bitstream = ''
        self.frame_bits = []
        self._crc_rg = 0

        # Compute the bitstream
        self._to_bitstream()

    def print_field_str(self, fieldname: str) -> str:
        """
        Prints a field of the CAN frame.

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
            if self.rtr:
                value = "({} bytes)".format(self.dlc)
            else:
                value = "({})".format(self.dlc)
        if fieldname == 'ide':
            value = "(1=Extended ID)" if self.ide else "(0=Standard ID)"
        if fieldname == 'ida':
            value = "(0x{:03x})".format(self.id_a)
        if fieldname == 'idb':
            value = "(0x{:05x}, full ID=0x{:08x})".format(self.id_b, self.id_a << 18 | self.id_b)
        if fieldname == 'rtr':
            value = "(1=Remote frame)" if self.rtr else "(0=Data frame)"

        result = "{}{} {} {}".format(" " * start_finish.start, "^" * len(start_finish), description, value)
        return result

    def _crc_bit(self, bit: str):
        """
        Calculate the CRC from each bit of the frame.
        """
        if bit not in ['0', '1']:
            raise AssertionError("Illegal bit value")
        self.unstuffed_bitstream += bit
        nxtbit = 1 if bit == '1' else 0
        bit_14 = (self._crc_rg & (1 << 14)) >> 14
        crcnxt = nxtbit ^ bit_14
        assert crcnxt in [0, 1]
        self._crc_rg <<= 1
        self._crc_rg &= 0x7fff

        if crcnxt == 1:
            self._crc_rg ^= 0x4599

        assert self._crc_rg <= 0x7fff

    def _add_stuff_bit(self, bit: str):
        if bit == '0':
            self._0_bits_in_row += 1
            self._1_bits_in_row = 0
        else:
            self._1_bits_in_row += 1
            self._0_bits_in_row = 0

        if self._0_bits_in_row == 5:
            self.frame_bits.append(True)
            self.stuffed_bitstream += self.RED + '1' + self.RESET
            self._1_bits_in_row = 1
            self._0_bits_in_row = 0
        if self._1_bits_in_row == 5:
            self.frame_bits.append(False)
            self.stuffed_bitstream += self.RED + '0' + self.RESET
            self._0_bits_in_row = 1
            self._1_bits_in_row = 0

    def _to_bitstream(self):
        self.fields = {
            'sof': '0',
            'rtr': '1' if self.rtr else '0',
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
            self.fields['r1'] = '0'
            self.fields['r0'] = '0'
            self.fieldnames = ['sof', 'ida', 'srr', 'ide', 'idb', 'rtr', 'r1', 'r0']
        else:
            self.fields['ida'] = "{:011b}".format(self.id_a)
            self.fields['ide'] = '0'
            self.fields['r0'] = '0'
            self.fieldnames = ['sof', 'ida', 'rtr', 'ide', 'r0']
        self.fieldnames += ['dlc', 'data', 'crc', 'crc_delimiter', 'ack', 'ack_delimiter', 'eof', 'ifs']

        for fieldname in self.fieldnames:
            color = self.FIELD_COLORS[fieldname]
            stuffed = fieldname in self.STUFFED_FIELDS
            crcd = fieldname in self.CRCD_FIELDS

            start = len(self.frame_bits)

            self.fields_position[fieldname] = [len(self.frame_bits), 0]
            for bit in self.fields[fieldname]:
                self.unstuffed_bitstream += color + bit + self.RESET
                self.stuffed_bitstream += color + bit + self.RESET
                self.frame_bits.append(True if bit == '1' else False)
                if stuffed:
                    self._add_stuff_bit(bit=bit)
                if crcd:
                    self._crc_bit(bit=bit)
                    self.fields['crc'] = "{:015b}".format(self._crc_rg)
            self.fields_position[fieldname] = range(start, len(self.frame_bits))

    def print(self, detailed=False):
        """
        Prints the CAN frame. Different colours are used for the frame fields.
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
        Decode a sequence of bits and return a CANFrame object
        :param bitseq: string containing a sequence of '0' and '1'
        :return: a dictionary of fields that have been decoded, and a CANFrame from the bit sequence
        """
        frame = {
            'ida': '',
            'srr': '',
            'ide': '',
            'idb': '',
            'rtr': '',
            'r1': '',
            'r0': '',
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
            if len(frame['crc']) == 15:
                stuffing = False
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
                    field = 'r0'
                    frame['rtr'] = frame['srr']
            elif field == 'idb':
                frame[field] += bit
                if len(frame[field]) == 18:
                    field = 'rtr'
            elif field == 'rtr':
                frame[field] = bit
                field = 'r1'
            elif field == 'r1':
                frame[field] = bit
                field = 'r0'
            elif field == 'r0':
                frame[field] = bit
                field = 'dlc'
            elif field == 'dlc':
                frame[field] += bit
                if len(frame[field]) == 4:
                    byte_count = int(frame[field], base=2)
                    if frame['rtr'] == '1' or byte_count == 0:
                        field = 'crc'
                    else:
                        if byte_count > 8:
                            byte_count = 8
                        field = 'data'
            elif field == 'data':
                frame[field] += bit
                if len(frame[field]) == byte_count * 8:
                    field = 'crc'
            elif field == 'crc':
                frame[field] += bit
                if len(frame[field]) == 15:
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
            rtr = True if frame['rtr'] == '1' else False
            dlc = int(frame['dlc'], base=2)
            id_b = 0 if not ide else int(frame['idb'], base=2)
            num_bytes = 0 if rtr or dlc == 0 else min(dlc, 8)

            data = []
            if not rtr:
                data_str = frame['data']
                for i in range(num_bytes):
                    data.append(int(data_str[:8], base=2))
                    data_str = data_str[8:]
            frame['can_frame'] = CANFrame(id_a=id_a, ide=ide, rtr=rtr, id_b=id_b, dlc=dlc, data=bytes(data))
        return frame


def is_janus(payload_a, payload_b, id_a, id_b=0, ide=False) -> bool:
    f_a = CANFrame(id_a=id_a, id_b=id_b, ide=ide, data=payload_a)
    f_b = CANFrame(id_a=id_a, id_b=id_b, ide=ide, data=payload_b)

    bitseq_a = f_a.bitseq()
    bitseq_b = f_b.bitseq()

    # Check 1: both frames must be the same length
    if len(bitseq_a) != len(bitseq_b):
        return False

    # Check 2: if 10 then must be the same until back in sync
    synced = True
    for i in range(len(bitseq_a)):
        if synced:
            if bitseq_a[i] == '1' and bitseq_b[i] == '0':
                synced = False
        else:
            if bitseq_a[i] != bitseq_b[i]:
                return False
            if bitseq_a[i] == '1':
                synced = True

    return True


if __name__ == '__main__':
    f = CANFrame(id_a=0x14, data=bytes([0x01]))
    f.print(detailed=True)

    frame = CANFrame.from_bitseq(f.bitseq())
    frame['can_frame'].print(detailed=True)

    payload_a = bytes([0xde, 0xad, 0xbe, 0xef, 0xca, 0xfe, 0xf0, 0x0d])
    for b in range(256):
        for b_dash in range(256):
            payload_b = bytes([0xde, 0xad, 0xbe, 0xef, b_dash, b, 0xf0, 0x0d])
            if payload_a != payload_b and is_janus(id_a=0x123, payload_a=payload_a, payload_b=payload_b):
                print("Match: Payload A={}".format(hexlify(payload_a)))
                print("       Payload B={}".format(hexlify(payload_b)))
