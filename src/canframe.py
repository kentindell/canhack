class CANFrame:
    """
    Class for calculating the bitstream of a CAN frame.
    """
    FIELD_NAMES = ['sof', 'ida', 'idb', 'ide', 'srr', 'rtr', 'r0', 'r1', 'dlc', 'data', 'crc', 'crc_delimiter', 'ack', 'eof', 'ifs']

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
        'srr': "SRR (Substitute Remote Request)",
        'rtr': "RTR (Remote Transmit Request)",
        'ide': "IDE (Extended ID)",
        'r0': "Reserved 0",
        'r1': "Reserved 1",
        'dlc': "DLC (Data Length Code)",
        'data': "Data",
        'crc': "CRC",
        'crc_delimiter': "CRC delimiter",
        'ack': "ACK",
        'ack_delimiter': "ACK delimiter",
        'eof': "EOF (End Of Frame)",
        'ifs': "IFS (Inter Frame Space)"
    }

    def __init__(self, can_id: int, data: bytes = bytes([]), extended=False, remote=False, dlc=0):
        """

        :param can_id: CAN identifier (either 11 bits or 29 bits)
        :param data: 0-8 bytes for the CAN payload
        :param extended: True if the CAN ID is 29 bits, false if 11 bits
        :param remote: True if a CAN remote frame
        :param dlc: The DLC is set automatically if the frame is a data frame, but can be set directly if a remote frame
        """
        if extended:
            self.id_a = (can_id >> 18) & 0x7ff
            self.id_b = can_id & 0x3ffff
        else:
            self.id_a = can_id & 0x7ff
            self.id_b = 0

        if len(data) not in range(9):
            raise ValueError("Data field must be 0..8 bytes")

        self.ide = extended
        self.rtr = remote
        self.data = data
        self.fields_position = {}
        self.fields = {}

        if remote:
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

    def print_field(self, fieldname: str):
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
        if fieldname == 'data' and len(self.fields['data']) == 0:
            description = "(No data field)"
        print("{}{} {} {}".format(" " * start_finish.start, "^" * len(start_finish), description, value))

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
            'ack': '0',
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
        print(self.stuffed_bitstream)
        if detailed:
            for fieldname in self.fieldnames:
                self.print_field(fieldname=fieldname)


if __name__ == '__main__':
    f = CANFrame(can_id=0x14, data=bytes([0x01]))
    f.print(detailed=True)
