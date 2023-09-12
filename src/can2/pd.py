##
## This file is part of the libsigrokdecode project.
##
## Copyright (C) 2020-2023 Canis Automotive Labs <info@canislabs.com>
##
## This program is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 2 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program; if not, see <http://www.gnu.org/licenses/>.
##
from binascii import hexlify
from copy import copy
import json
from statistics import mean, median
import struct
import string
from collections import OrderedDict, namedtuple
import sigrokdecode as srd


class CANPCAPNG:
    """
    Generates a binary capture file in pcapng format. This is documented:

    https://tools.ietf.org/id/draft-tuexen-opsawg-pcapng-01.html

    The file produces:

    - SHB (Section header block)
    - IDB (Interface description block)
    - Multiple EPBs (Extended packet blocks)

    The SHB defines the file size and endianness.

    The IDB defines the link type, which will be passed through into protocol decoders.
        - The option if_tsresol should be set to select nanosecond timestamp resolution (since 1st Jan 1970)

    The EPBs define the CAN frames. The timestamp is 64-bit. The interface ID matches the one in the IBD.

    The link type is 227 (see http://www.tcpdump.org/linktypes.html), and the packet format is:

        +---------------------------+
        |      CAN ID and flags     |
        |         (4 Octets)        |
        +---------------------------+
        |    Frame payload length   |
        |         (1 Octet)         |
        +---------------------------+
        |          Padding          |
        |         (1 Octet)         |
        +---------------------------+
        |      Reserved/Padding     |
        |         (1 Octet)         |
        +---------------------------+
        |      Reserved/Padding     |
        |         (1 Octet)         |
        +---------------------------+
        |           Payload         |
        .                           .
        .                           .
        .                           .

        Description

        The field containing the CAN ID and flags is in network byte order (big-endian). The bottom 29 bits contain the CAN ID of the frame. The remaining bits are:

            0x20000000 - set if the frame is an error message rather than a data frame.
            0x40000000 - set if the frame is a remote transmission request frame.
            0x80000000 - set if the frame is an extended 29-bit frame rather than a standard 11-bit frame. frame.

        The SocketCAN dissector in wireshark might be able to handle a larger payload than 8 bytes.

        # TODO create a new link layer definition with a LUA dissector that can handle more information
    """

    @staticmethod
    def get_shb() -> bytes:
        return b"".join(struct.pack('>I', i) for i in [0x0a0d0d0a, 28, 0x1a2b3c4d, 0x00010000, 0xffffffff, 0xffffffff, 28])

    @staticmethod
    def get_idb() -> bytes:
        # Link type is SocketCAN, 227 (see http://www.tcpdump.org/linktypes.html)
        # TODO set timestamps to be in nanoseconds, rather than microseconds, by adding the if_tsresol option
        # TODO option type: 9
        # TODO option length: 1
        # TODO option value: 9 (+ 3 pad bytes)
        # TODO this will add 8 bytes to the block length
        return b"".join(struct.pack('>I', i) for i in [0x00000001, 20, 227 << 16, 0, 20])

    @staticmethod
    def get_epb(ida: int, idb: int, ide: int, rtr: int, data: bytes, timestamp: int, interface_id=0, error_frame=False) -> bytes:
        assert ide in [1, 0]
        assert rtr in [1, 0]
        assert ida < (1 << 11)
        assert idb < (1 << 18)
        assert len(data) <= 8

        if ide:
            canid = ida << 18 | idb | 0x80000000
        else:
            canid = ida
        if rtr:
            canid |= 0x40000000
        if error_frame:
            canid = 0x20000000
            data = bytes([8] * 8)  # Error reports need to be 8 bytes of data
        padded_data = (data + bytes([0] * 8))[:8]

        data_h = struct.unpack('>I', padded_data[:4])[0]
        data_l = struct.unpack('>I', padded_data[4:])[0]

        return b"".join(struct.pack('>I', i) for i in [0x00000006,
                                                       48,
                                                       interface_id,
                                                       timestamp >> 32,
                                                       timestamp & 0xffffffff,
                                                       16,
                                                       16,
                                                       canid,
                                                       len(data) << 24,
                                                       data_h,
                                                       data_l,
                                                       48])


class SerialCRC:
    def __init__(self, initial=0, polynomial=0x4599, size=15):
        self.initial = initial
        self.polynomial = polynomial
        self.size = size
        self.crc = initial
        self.mask = (1 << size) - 1

    def add_bit(self, bit: str):
        d = int(bit)
        crc_xor = ((self.crc << 1) ^ self.polynomial) & self.mask
        crc_shift = (self.crc << 1) & self.mask
        crc_next = crc_xor if d ^ self.crc >> (self.size - 1) else crc_shift
        self.crc = crc_next

    def reset(self):
        self.crc = self.initial


class CANBit:
    sampling = None  # type: bool
    next_event_samplenum = None  # type: int
    bit_time_samples = None  # type: int
    sample_point_samples = None  # type: int
    sample_to_end_samples = None  # type: int
    canrx_sample = None  # type: str
    synced = None  # type: bool

    def __init__(self, start_samplenum: int):
        self.start_samplenum = start_samplenum
        self.end_samplenum = None  # type: int
        self.value = None  # type: str
        self.stuffbit = False

    @classmethod
    def canhg_start(cls, samplenum: int):
        pass

    @classmethod
    def canhg_stop(cls):
        pass

    @classmethod
    def bitstream(cls, samplenum: int, falling_edge: bool, canrx: str, canbit: 'CANBit') -> bool:
        """
        This function is called from a falling edge or when the next CAN event is due

        :param samplenum: the current time
        :param falling_edge: there is a falling edge in the CAN signal
        :param canrx: the CAN signal at the event
        :param canbit: the current CAN bit
        :return: True if the bit has ended
        """
        assert canrx in ['1', '0']
        bit_end = False

        # Wait for a falling edge after a '1' sample or for the sample point of the next bit, whichever comes first
        if cls.sampling:
            if not cls.synced and falling_edge and cls.canrx_sample == '1':  # Can only sync once per bit
                # Soft/hard sync here (we ignore SJW for simplicity), sample point moved out
                # and we stay in the sampling state, this is the new start of the bit
                cls.next_event_samplenum = samplenum + cls.sample_point_samples
                # The bit re-starts here but we don't mark that: prefer to show a stretched bit
                cls.synced = True
            elif samplenum >= cls.next_event_samplenum:
                cls.synced = False  # Next bit, syncing re-enabled
                canbit.value = canrx
                cls.canrx_sample = canrx

                field = CANField.get_current_field()
                if field.name == 'r0' and canrx == '0':
                    cls.canhg_start(samplenum=samplenum)
                if field.name == 'crc-delimiter':
                    cls.canhg_stop()
                cls.next_event_samplenum = samplenum + cls.sample_to_end_samples
                cls.sampling = False
        else:
            # Waiting for the end of a bit, having already taken the sample
            if not cls.synced and falling_edge and cls.canrx_sample == '1':
                cls.next_event_samplenum = samplenum + cls.sample_point_samples
                cls.sampling = True
                cls.synced = True
                bit_end = True
            elif samplenum >= cls.next_event_samplenum:
                # This is the end of the bit, record this and set the future sampling time
                cls.next_event_samplenum = samplenum + cls.sample_point_samples
                cls.sampling = True
                bit_end = True
        if bit_end:
            assert canbit.value in ['1', '0']
        return bit_end

    @classmethod
    def reset(cls,
              bit_time_samples: int,
              sample_point_samples: int,
              sample_to_end_samples: int):
        cls.bit_time_samples = bit_time_samples
        cls.sample_point_samples = sample_point_samples
        cls.sample_to_end_samples = sample_to_end_samples
        cls.sampling = True
        cls.next_event_samplenum = cls.bit_time_samples


class CANField:
    Info = namedtuple('Info', 'annotation canbit descriptions')

    fields = OrderedDict()
    stuffing = False  # type: bool
    crcing = False  # type: bool
    crc = SerialCRC()  # Defaults to CAN's 15-bit CRC
    bus_integration_count = None  # type: int
    byte_count = None  # type: int
    last_6_str = ''
    data_bytes = []  # type: List['CANField']
    info = []  # type: List[Info]
    rx_ok = False
    recessive_count = 0

    def __init__(self, name: str, add_to_data=False):
        self.name = name
        self.canbits = []  # type: List[CANBit]
        if add_to_data:
            self.data_bytes.append(self)
        else:
            # Make sure the new entry goes to the end of the ordered dictionary
            if name in self.fields:
                del self.fields[name]
            self.fields[name] = self
        self.info = []

    def get_value(self) -> int:
        """
        Returns the value of the field as an integer
        :return:
        """
        value = 0
        for canbit in self.canbits:
            if not canbit.stuffbit:
                value <<= 1
                value |= 1 if canbit.value == '1' else 0
        return value

    def get_len(self) -> int:
        """
        Returns the number of non stuff bits currently in the field
        """
        return len([canbit for canbit in self.canbits if not canbit.stuffbit])

    def get_descriptions(self):
        """
        :return: A tuple of start/end sample num and a list of descriptions
        """
        value = self.get_value()
        # An empty list of descriptions will cause the field not to be displayed
        if self.name == 'bus-integration':
            return self.canbits[-1].start_samplenum, self.canbits[-1].end_samplenum, []
        elif self.name == 'idle':
            # The idle field can be 0 bits long so never return description if it's a zero-length field
            if len(self.canbits) > 0:
                return self.canbits[-1].start_samplenum, self.canbits[-1].end_samplenum, []
            else:
                return 0, 0, []
        elif self.name == 'sof':
            return self.canbits[-1].start_samplenum, self.canbits[-1].end_samplenum, ["SOF"]
        elif self.name == 'ide':
            descriptions = ["IDE=Extended" if value else "IDE=Standard",
                            "IDE={}".format(value),
                            "IDE",
                            "I"]
        elif self.name == 'rtr':
            descriptions = ["RTR=Remote" if value else "RTR=Data",
                            "RTR={}".format(value),
                            "RTR",
                            "R"]
        elif self.name == 'srr':
            if 'ide' in self.fields and self.fields['ide'].get_value() == 0:
                descriptions = ["RTR=Remote" if value else "RTR=Data",
                                "RTR={}".format(value),
                                "RTR",
                                "R"]
            else:
                descriptions = ["SRR={}".format(value)]
        elif self.name == 'r1':
            descriptions = ["r1={}".format(value), "r1"]
        elif self.name == 'r0':
            descriptions = ["r0={}".format(value), "r0"]
        elif self.name == 'crc-delimiter':
            descriptions = ["CRC delimiter"]
        elif self.name == 'ack':
            descriptions = ["ACK"]
        elif self.name == 'ack-delimiter':
            descriptions = ["ACK delimiter"]
        elif self.name == 'eof':
            descriptions = ["EOF"]
        elif self.name == 'ifs':
            descriptions = ["IFS"]
        elif self.name == 'ida':
            descriptions = ["ID A 0x{:03x}".format(value), "ID A", "A"]
        elif self.name == 'idb':
            descriptions = ["ID B 0x{:05x}".format(value), "ID B", "B"]
        elif self.name == 'dlc':
            descriptions = ["DLC 0x{:x}".format(value), "DLC"]
        elif self.name == 'data':
            # An empty list of descriptions will cause the field not to be displayed
            return 0, 0, []
        elif self.name == 'crc':
            descriptions = ["CRC 0x{:04x}".format(value), "CRC"]
        elif self.name == 'superposition':
            descriptions = ["Error flag", "EF", "E", ""]
        elif self.name == 'error-delimiter':
            descriptions = ["Error delimiter", "ED", "E", ""]
        elif self.name.startswith('databyte'):
            b = self.name[-1]
            descriptions = ["DATA{}=0x{:02x}".format(b, value), "0x{:02x}".format(value)]
        else:
            descriptions = ["{}?".format(self.name)]
        descriptions.append("")

        return self.canbits[0].start_samplenum, self.canbits[-1].end_samplenum, descriptions

    @classmethod
    def is_remote(cls) -> bool:
        if cls.fields['ide'].get_value() == 1:
            return True if cls.fields['rtr'].get_value() == 1 else False
        else:
            return True if cls.fields['srr'].get_value() == 1 else False

    @classmethod
    def add_data_bit(cls, canbit: CANBit):
        cls.data_bytes[-1].canbits.append(canbit)

    @classmethod
    def reset(cls):
        cls.fields = OrderedDict()
        CANField('bus-integration')
        cls.bus_integration_count = 0
        cls.stuffing = False
        cls.last_6_str = ''
        cls.recessive_count = 0

    @classmethod
    def sof(cls):
        cls.stuffing = True
        cls.crcing = True
        cls.crc = SerialCRC()
        cls.byte_count = None  # type: int
        cls.data_bytes = []
        CANField('ida')

    @classmethod
    def get_current_field(cls) -> 'CANField':
        if len(cls.fields) > 0:
            return cls.fields[next(reversed(cls.fields))]
        else:
            cls.reset()
            return cls.fields[0]

    @classmethod
    def add_bit(cls, canbit: CANBit):
        """
        Adds a CAN bit to the current field
        :param canbit: the CAN bit (might be a stuff bit)
        :return:
        """
        current_field = cls.get_current_field()
        current_field.canbits.append(canbit)

    @classmethod
    def crc_id_bit(cls, canbit: CANBit):
        pass

    @classmethod
    def state_machine(cls, canbit: CANBit) -> 'CANField':
        """
        Decodes CAN bits into fields. If after this call the new field is 'ida' then a new frame
        started; the caller must display any undisplayed parts of the old frame and reset the frame.

        :param canbit: the current CAN bit
        :return: Field just finished, or None if none
        """
        assert canbit.value in ['1', '0']

        field = cls.get_current_field()

        if field.name == 'data':
            cls.data_bytes[-1].canbits.append(canbit)

        cls.last_6_str = cls.last_6_str[-5:] + canbit.value
        # Add to the current field; may have to remove this bit later
        cls.add_bit(canbit=canbit)

        if canbit.value == '1':
            cls.recessive_count += 1
        else:
            cls.recessive_count = 0

        if cls.stuffing:
            if cls.last_6_str == '111111' or cls.last_6_str == '000000':
                cls.info.append(cls.Info('can-warning', canbit, ['Stuff error', 'Error', 'E', ''], ))
                CANField(name='superposition')
                cls.stuffing = False
                return field
            elif cls.last_6_str == '111110' or cls.last_6_str == '000001':
                canbit.stuffbit = True

        if field.name in ['ida', 'idb', 'ide', 'rtr', 'srr'] and not canbit.stuffbit:
            cls.crc_id_bit(canbit=canbit)

        if cls.crcing and not canbit.stuffbit:
            cls.crc.add_bit(bit=canbit.value)

        # Look for SOF at end of frame
        if field.name == 'ifs' and cls.get_current_field().get_len() == 3 and canbit.value == '0':
            # This is treated as SOF
            cls.sof()
            return field

        # SOF at end of frame case is handled above
        if field.name in ['crc-delimiter', 'ack-delimiter', 'eof', 'ifs', 'error-delimiter'] and canbit.value == '0':
            if field.name == 'ifs':
                cls.info.append(cls.Info('can-warning', canbit, ['Overload', 'O', ''], ))
            elif field.name == 'eof' and field.get_len() == 7:
                cls.info.append(cls.Info('can-warning', canbit, ['Double receive', 'Double', 'D', ''], ))
            else:
                cls.info.append(cls.Info('can-warning', canbit, ['Form error', 'Error', 'E', ''], ))
            CANField(name='superposition')
            cls.stuffing = False
            return field

        ########### MAIN STATE MACHINE ############
        if cls.stuffing and cls.last_6_str[1:] == '11111' or cls.last_6_str[1:] == '00000':
            # There is a stuff bit due next so do not process this bit until that bit occurs
            pass
        elif field.name == 'bus-integration':
            if canbit.value == '0':
                cls.bus_integration_count = 0
            else:
                cls.bus_integration_count += 1
                if cls.bus_integration_count == 11:
                    cls.bus_integration_count = 0
                    CANField('idle')
                    return field
        elif field.name == 'idle':
            if canbit.value == '0':
                # Actually this is an SOF bit, so rewrite the field
                del field.canbits[-1]
                sof = CANField('sof')
                sof.add_bit(canbit=canbit)
                cls.sof()
                return field
        elif field.name == 'ida':
            if field.get_len() == 11:
                CANField('srr')
                return field
        elif field.name == 'srr':
            if field.get_len() == 1:
                CANField('ide')
                return field
        elif field.name == 'ide':
            if field.get_len() == 1:
                if field.get_value() == 1:
                    CANField('idb')
                    return field
                else:
                    CANField('r0')
                    return field
        elif field.name == 'idb':
            if field.get_len() == 18:
                CANField('rtr')
                return field
        elif field.name == 'rtr':
            if field.get_len() == 1:
                CANField('r1')
                return field
        elif field.name == 'r1':
            if field.get_len() == 1:
                CANField('r0')
                return field
        elif field.name == 'r0':
            if field.get_len() == 1:
                if field.get_value() == 1:
                    # If r0 is 1 then this indicates that the frame is not a valid CAN frame and the decoder
                    # should skip this frame and look for the next one
                    cls.info.append(cls.Info('can-warning', canbit, ['r0=1', 'r0', '']))
                    CANField('bus-integration')
                    cls.stuffing = False
                    return field
                else:
                    CANField('dlc')
                    return field
        elif field.name == 'dlc':
            if field.get_len() == 4:
                cls.byte_count = field.get_value()
                if cls.byte_count > 8:
                    cls.byte_count = 8
                if cls.is_remote() or cls.byte_count == 0:
                    cls.crcing = False
                    CANField('crc')
                    return field
                else:
                    CANField('data')
                    CANField('databyte0', add_to_data=True)
                    return field
        elif field.name == 'data':
            if field.get_len() == cls.byte_count * 8:
                CANField('crc')
                cls.crcing = False
                return field
            elif cls.data_bytes[-1].get_len() == 8:
                CANField('databyte{}'.format(field.get_len() // 8), add_to_data=True)
        elif field.name == 'crc':
            if field.get_len() == 15:
                cls.stuffing = False
                if field.get_value() != cls.crc.crc:
                    cls.info.append(cls.Info('can-warning',
                                             canbit,
                                             ["CRC mismatch (0x{:04x})".format(cls.crc.crc),
                                              "CRC mismatch",
                                              "C",
                                              ""]))
                CANField('crc-delimiter')
                return field
        elif field.name == 'crc-delimiter':
            CANField('ack')
            return field
        elif field.name == 'ack':
            if canbit.value == '1':
                cls.info.append(cls.Info('can-warning', canbit, ['ACK error', 'A', '']))
            CANField('ack-delimiter')
            return field
        elif field.name == 'ack-delimiter':
            CANField('eof')
            return field
        elif field.name == 'eof':
            if field.get_len() == 6:
                cls.info.append(cls.Info('can-info', canbit, ['Received OK', 'RX OK', 'RX', '']))
                cls.rx_ok = True
            if field.get_len() == 7:
                cls.info.append(cls.Info('can-info', canbit, ['Transmitted OK', 'TX OK', 'TX', '']))
                CANField('ifs')
                return field
        elif field.name == 'ifs':
            if field.get_len() == 3:
                # Note there is a special case of IFS0=0 meaning SOF, handled earlier
                CANField('idle')
                return field
        elif field.name == 'superposition':
            if canbit.value == '1':
                CANField("error-delimiter")
                return field
        elif field.name == 'error-delimiter':
            if canbit.value == '0':
                CANField("superposition")
            elif field.get_len() == 7:
                CANField('ifs')
                return field

        return None


class Annotation:
    annotations = OrderedDict()

    def __init__(self, name: str, description: str):
        if name in self.annotations:
            raise KeyError("Annotation '{}' already used".format(name))
        self.name = name
        self.description = description
        self.index = len(self.annotations)
        self.annotations[name] = self
        self.used = False

    @classmethod
    def lookup(cls, name: str) -> int:
        if name not in cls.annotations:
            raise KeyError("Annotation '{}' not known".format(name))
        if not cls.annotations[name].used:
            raise KeyError("Annotation '{}' not used in a row".format(name))
        return cls.annotations[name].index

    @classmethod
    def get_annotation_row(cls, names):
        for name in names:
            if name not in cls.annotations:
                raise KeyError("Annotation '{}' not known".format(name))
            cls.annotations[name].used = True
        return tuple(cls.lookup(name) for name in names)

    @classmethod
    def get_annotations(cls):
        return tuple([(a.name, a.description) for a in cls.annotations.values()])
        # return tuple(cls.annotations.items())


class Node:
    nodes = []
    
    def __init__(self, first_ns: float, last_ns: float, name: str=None):
        self.first_ns = first_ns
        self.last_ns = last_ns
        if name is None:
            name = "Node%d" % len(self.nodes)
        self.name = name

        # Adds into nodes
        self.nodes.append(self)

    def to_dict(self):
        return {
            "first_ns": self.first_ns,
            "last_ns": self.last_ns,
            "name": self.name,
        }

    @classmethod
    def to_json(cls):
        data = [node.to_dict() for node in cls.nodes]

        with open('can2nodes.json', 'w') as f:
            json.dump(data, f)

    @classmethod
    def from_json(cls):
        try:
            # FIXME use a dot file in the home directory
            with open('can2nodes.json', 'r') as f:
                data = json.load(f)
        except:
            # Couldn't open the file
            data = []

        # Create all the nodes from the imported JSON (if the open failed then will be empty)
        cls.nodes = []
        for node_dict in data:
            Node(first_ns=node_dict['first_ns'], last_ns=node_dict['last_ns'], name=node_dict['name'])

    @classmethod
    def get_node(cls, delta_ns, sample_period_ns: float):
        # Tries to find the node, a list in which the item fits
        nodes = []

        # Find a list of nodes into which this distortion delta fits, with the distance to the mean
        for n in cls.nodes:
            if n.first_ns - sample_period_ns <= delta_ns <= n.last_ns + sample_period_ns:
                mean_ns = int((n.first_ns + n.last_ns) / 2.0)
                distance_ns = abs(n.first_ns - mean_ns)
                nodes.append((distance_ns, n, ))
        nodes.sort(key=lambda node: node[0])

        ordered_nodes = [n[1] for n in nodes]

        return ordered_nodes
 

class Decoder(srd.Decoder):
    api_version = 3
    id = 'can2'
    name = 'CAN 2.0'
    longname = 'Controller Area Network 2.0'
    desc = 'CAN protocol'
    license = 'gplv2+'
    inputs = ['logic']
    outputs = ['can2']
    tags = ['Automotive']
    channels = (
        {'id': 'canrx', 'name': 'CAN RX', 'desc': 'Digital CAN bus signal'},
    )
    optional_channels = ()
    options = (
        {'id': 'can-bitrate', 'desc': 'CAN bit rate (Hz)', 'default': 500000},
        {'id': 'can-samplepoint', 'desc': 'Sample point (%)', 'default': 75},
        {'id': 'can-datadisplay', 'desc': 'Data display', 'default': 'Hex', 'values': ('Hex', 'Hex and ASCII')},
        {'id': 'can-nodes-write', 'desc': 'Write nodes file', 'default': 'No', 'values': ('No', 'Yes')},
    )
    binary = (
        ('pcapng', 'The pcapng packet capture format used by Wireshark'),
    )
    _ = (
        Annotation('data', 'Payload'),
        Annotation('sof', 'Start of frame'),
        Annotation('eof', 'End-of-frame'),
        Annotation('ida', '11-bit identifier'),
        Annotation('idb', '18-bit identifier extension'),
        Annotation('ide', 'IDE'),
        Annotation('r0', 'r0'),
        Annotation('r1', 'r1'),
        Annotation('rtr', 'RTR'),
        Annotation('srr', 'RTR/SRR'),
        Annotation('dlc', 'DLC'),
        Annotation('crc', 'CRC'),
        Annotation('crc-delimiter', 'CRC delimiter'),
        Annotation('ack', 'ACK'),
        Annotation('ack-delimiter', 'ACK delimiter'),
        Annotation('can-warning', 'Warning'),
        Annotation('can-delta', 'Pulse timing'),
        Annotation('can-frame-delta', 'Frame timing'),
        Annotation('can-info', 'Info'),
        Annotation('overload', 'Overload flag'),
        Annotation('bus-integration', 'Bus integration'),
        Annotation('superposition', 'Error flag'),
        Annotation('error-delimiter', 'Error delimiter'),
        Annotation('ifs', 'IFS'),
        Annotation('idle', 'Idle'),
        Annotation('stuffbit', 'Stuff bit'),
        Annotation('bit', 'Bit'),
        Annotation('can-payload', 'CAN Payload'),
        Annotation('can-id', 'CAN ID'),
    )

    annotations = Annotation.get_annotations()
    annotation_rows = (
        ('row-bits', "Bits", Annotation.get_annotation_row(['bit', 'stuffbit'])),
        ('row-can-fields', "Fields", Annotation.get_annotation_row(['can-payload',
                                                                    'sof',
                                                                    'eof',
                                                                    'ida',
                                                                    'idb',
                                                                    'ide',
                                                                    'r0',
                                                                    'r1',
                                                                    'rtr',
                                                                    'srr',
                                                                    'dlc',
                                                                    'crc',
                                                                    'crc-delimiter',
                                                                    'ack',
                                                                    'ack-delimiter',
                                                                    'overload',
                                                                    'bus-integration',
                                                                    'superposition',
                                                                    'error-delimiter',
                                                                    'ifs',
                                                                    'idle'])),
        ('row-can-payload', "Payload", Annotation.get_annotation_row(['can-id', 'data'])),
        ('row-can-delta', "Pulse", Annotation.get_annotation_row(['can-delta', 'can-delta'])),
        ('row-can-warning', "Info", Annotation.get_annotation_row(['can-warning', 'can-info'])),
        ('row-can-frame-delta', "Timing", Annotation.get_annotation_row(['can-frame-delta', 'can-frame-delta'])),
    )

    def __init__(self, **kwargs):
        self.display_hex = True
        self.display_ascii = None  # type: bool
        self.can_bit_time_ns = None  # type: float
        self.can_sample_point = None  # type: float
        # This is the logic analyzer / oscilloscope sample rate
        self.sample_rate = None  # type: int
        self.sample_period_ns = None  # type: float
        self.out_ann = None
        self.out_binary = None
        self.printables = set(string.printable)
        self.last_rising_edge_ns = None  # type: float
        self.last_falling_edge_ns = None  # type: float
        self.last_recessive_pulse_width_ns = None  # type: float
        self.last_dominant_pulse_width_ns = None  # type: float
        self.shortenings_ns = []  # List of nanosecond shortenings for the current frame

        # Import the node descriptions (if they exist)
        Node.from_json()

    def metadata(self, key, value):
        if key == srd.SRD_CONF_SAMPLERATE:
            self.sample_rate = value
            self.sample_period_ns = 1000000000.0 / float(value)

    def time_ns_to_num_samples(self, time_ns: float) -> int:
        return int(time_ns / self.sample_period_ns)

    def num_samples_to_time_ns(self, numsamples: int) -> float:
        return self.sample_period_ns * numsamples

    def now_ns(self) -> float:
        return self.num_samples_to_time_ns(self.samplenum)

    def reset(self):
        # TODO This is called when the Run button is hit, so is a good place to send a command over MIN to set an advanced trigger
        self.display_hex = True
        self.display_ascii = self.options['can-datadisplay'] != 'Hex'
        self.nodes_write = self.options['can-nodes-write'] == 'Yes'
        self.can_bit_time_ns = 1000000000 / self.options['can-bitrate']
        self.can_sample_point = self.options['can-samplepoint'] / 100
        CANField.reset()
        CANBit.reset(bit_time_samples=self.time_ns_to_num_samples(self.can_bit_time_ns),
                     sample_point_samples=self.time_ns_to_num_samples(self.can_bit_time_ns * self.can_sample_point),
                     sample_to_end_samples=self.time_ns_to_num_samples(
                         self.can_bit_time_ns * (1.0 - self.can_sample_point)))

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)
        self.out_binary = self.register(srd.OUTPUT_BINARY)

    def decode_events(self, canbit: CANBit, falling_edge: bool, rising_edge: bool, canrx: str) -> CANBit:
        assert canrx in ['1', '0']
        end_of_canbit = CANBit.bitstream(samplenum=self.samplenum,
                                         falling_edge=falling_edge,
                                         canrx=canrx,
                                         canbit=canbit)

        cf = CANField.get_current_field()

        if cf is not None and cf.name in ['r0']:
            # Use the r0 field to reset the pulse width measurements
            self.last_rising_edge_ns = None
            self.last_falling_edge_ns = None
            self.last_dominant_pulse_width_ns = None
            self.last_recessive_pulse_width_ns = None
            self.shortenings_ns = []

        if rising_edge or falling_edge:
            if cf is not None and cf.name in ['dlc', 'data', 'crc']:
                # Keep track of pulses entirely within the part of the frame transmitted by the node
                # Error frames in the middle of this sequence will throw the data off: it's only valid the frame is received OK
                # TODO output the data for a frame only in the "received OK" event handler
                if rising_edge:
                    self.last_rising_edge_ns = self.now_ns()
                    if self.last_falling_edge_ns is not None:
                        self.last_dominant_pulse_width_ns = self.last_rising_edge_ns - self.last_falling_edge_ns
                if falling_edge:
                    self.last_falling_edge_ns = self.now_ns()
                    if self.last_rising_edge_ns is not None:
                        self.last_recessive_pulse_width_ns = self.last_falling_edge_ns - self.last_rising_edge_ns

        if end_of_canbit:
            # Finish the old bit, start the new bit

            canbit.end_samplenum = self.samplenum
            field = CANField.state_machine(canbit=canbit)
            # At this point the CAN bit will have been identified as a stuff bit or not
            self.put_can_bit(canbit=canbit)

            # If a CAN frame has been received OK then can output it to the pcapng binary out
            if CANField.rx_ok:
                self.put_pcapng_epb(canbit=canbit)

                if len(self.shortenings_ns) > 2:
                    # Output the mean pulse shortenings for the whole frame alond with its ID into a separate information field
                    # TODO eliminate outlying results, maybe used median
                    self.shortenings_ns.sort()
                    # Remove first and last data points
                    # s = s[1:]
                    # s = s[:-1]
                    mean_shortening_ns = mean(self.shortenings_ns)

                    # Try and determine the node(s) that the frame can have come from
                    nodes = Node.get_node(delta_ns=mean_shortening_ns, sample_period_ns=self.sample_period_ns)
                    # Must be an unknown node
                    if len(nodes) > 0:
                        pass
                    else:
                        # If the option is set to write back the nodes file then do so
                        if self.nodes_write:
                            # No nodes: must be a new node, so create it
                            nodes = [Node(first_ns = self.shortenings_ns[0], last_ns=self.shortenings_ns[-1])]
                            # Save all the nodes into the JSON file to keep track of this one
                            Node.to_json()

                    display_str = "/".join([node.name for node in nodes])
    
                    sof_samplenum = CANField.fields['ida'].canbits[0].start_samplenum
                    frame_duration = CANBit(start_samplenum=sof_samplenum)
                    frame_duration.end_samplenum = self.samplenum

                    CANField.info.append(CANField.Info('can-frame-delta', frame_duration, ['%s (%.1f ns)' % (display_str, mean_shortening_ns)], ))

                CANField.rx_ok = False
            if field is not None and field.name == 'superposition':
                self.put_pcapng_epb(canbit=canbit, error_frame=True)

            # Work out what to display. If the current field is ID A, then the previous field is SOF
            # and the IFS, in which case display it then reset the fields (except for the current SOF)
            # The previous field may be idle, but we do not display idle or bus integration.
            if field is not None:
                # Display the previous field
                self.put_can_field(field=field)
                if field.name == 'idle' and CANField.get_current_field().name == 'ida':
                    self.put_can_field(field=CANField.fields['sof'])
                if field.name == 'data':
                    self.put_can_payloads()
                if field.name == 'crc':
                    self.put_can_payload()
                if field.name == 'r0':
                    self.put_can_id()
                # Now erase the fields except for the current one
                if field.name in ['idle', 'ifs']:
                    # The current field may be idle or ida, depending on what happened
                    current_field = CANField.get_current_field()
                    CANField.fields = OrderedDict()
                    CANField.fields[current_field.name] = current_field
                while len(CANField.info) > 0:
                    info = CANField.info[0]
                    self.put_can_info(canbit=info.canbit,
                                      descriptions=info.descriptions,
                                      annotation=info.annotation)
                    del CANField.info[0]

            # Move on to next bit
            canbit = CANBit(start_samplenum=self.samplenum + 1)

        # Want to note pulse durations for recessive pulses within the frame when the measurement is good
        if falling_edge and self.last_recessive_pulse_width_ns is not None and self.last_dominant_pulse_width_ns is not None and cf is not None and cf.name in ['data', 'crc']:
            # Recessive pulse from non-arbitration fields
            # TODO parameterize the duration of the dominant and recessive pulses for which measurements will be taken
            # Create a record indicating the shortening from an expected duration based on the number of CAN bits
            expected_duration_ns = CANField.recessive_count * self.can_bit_time_ns
            shortening_ns = int(expected_duration_ns - self.last_recessive_pulse_width_ns)

            self.shortenings_ns.append(shortening_ns)
            # Display rounded to nearest nanosecond
            pulse = CANBit(start_samplenum=self.time_ns_to_num_samples(self.last_rising_edge_ns))
            pulse.end_samplenum = self.time_ns_to_num_samples(self.last_falling_edge_ns)

            # Try and determine the node(s) that the frame can have come from
            nodes = Node.get_node(delta_ns=shortening_ns, sample_period_ns=self.sample_period_ns)
            # Must be an unknown node
            if len(nodes) > 0:
                nodes_str = "/".join([node.name for node in nodes])
            else:
                nodes_str = "?"

            CANField.info.append(CANField.Info('can-delta', pulse, ['%s (%s ns)' % (nodes_str, shortening_ns)], ))


        return canbit

    def decode(self):
        """
        This is the main driver of the decoder: it is fed a set of logic analyzer samples
        """
        self.reset()

        self.put_pcapng_init()

        canbit = CANBit(start_samplenum=self.samplenum)

        while True:
            # Wait for a falling edge or the next can event

            (pin,) = self.wait([{0: 'f'}, {0: 'r'}, {'skip': CANBit.next_event_samplenum - self.samplenum}])

            # events = [{0: 'f'},
                    #   {0: 'r'},
                     #  {'skip': CANBit.next_event_samplenum - self.samplenum},
                    #   ]

            canrx = '1' if pin == 1 else '0'

            if isinstance(self.matched, int):
                falling_edge = True if self.matched & 0x1 else False
                rising_edge = True if self.matched & 0x2 else False
            else:
                falling_edge = self.matched[0]
                rising_edge = self.matched[1]

            canbit = self.decode_events(canbit=canbit, falling_edge=falling_edge, rising_edge=rising_edge, canrx=canrx)

    def put_can_bit(self, canbit: CANBit):
        if canbit.stuffbit:
            data = [Annotation.lookup('stuffbit'), ["Stuff bit={}".format(canbit.value), canbit.value, '']]
        else:
            data = [Annotation.lookup('bit'), [canbit.value, '']]
        self.put(canbit.start_samplenum, canbit.end_samplenum, self.out_ann, data)

    def put_can_payloads(self):
        for i in range(len(CANField.data_bytes)):
            databyte = CANField.data_bytes[i]
            b = databyte.get_value()
            if self.display_ascii and chr(b) in self.printables:
                a = str(bytes([b]))[1:]
                data = [Annotation.lookup('can-payload'), ["DATA{}=0x{:02x} {}".format(i, b, a),
                                                           "0x{:02x} {}".format(b, a),
                                                           ""]]
            else:
                data = [Annotation.lookup('can-payload'), ["DATA{}=0x{:02x}".format(i, b),
                                                           "0x{:02x}".format(b),
                                                           "{:02x}".format(b),
                                                           ""]]
            self.put(databyte.canbits[0].start_samplenum, databyte.canbits[-1].end_samplenum, self.out_ann, data)

    def put_can_payload(self):
        bs = list()
        for i in range(len(CANField.data_bytes)):
            bs.append(CANField.data_bytes[i].get_value())
        if self.display_ascii:
            cs = "'" + ''.join(chr(i) if chr(i) in self.printables else '.' for i in bs) + "'"
            bs = hexlify(bytes(bs)).decode("ascii")  # bytes.hex() isn't available
            data = [Annotation.lookup('data'), ["DATA=0x{} {}".format(bs, cs),
                                                "0x{} {}".format(bs, cs),
                                                "DATA",
                                                "D",
                                                ""]]
        else:
            bs = hexlify(bytes(bs)).decode("ascii")
            data = [Annotation.lookup('data'), ["DATA=0x{}".format(bs),
                                                "0x{}".format(bs),
                                                "{}".format(bs),
                                                "DATA",
                                                "D",
                                                ""]]
        if len(CANField.data_bytes) > 0:
            self.put(CANField.data_bytes[0].canbits[0].start_samplenum, CANField.data_bytes[-1].canbits[-1].end_samplenum, self.out_ann, data)

    def put_can_id(self):
        ide = CANField.fields['ide']  # type: CANField
        ida = CANField.fields['ida']  # type: CANField
        id_start_samplenum = ida.canbits[0].start_samplenum
        if ide.get_value() == 1:
            idb = CANField.fields['idb']  # type: CANField
            id_end_samplenum = idb.canbits[-1].end_samplenum
            can_id = ida.get_value() << 18 | idb.get_value()
            can_id_strs = ["ID=0x{:08x} (Extended)".format(can_id),
                           "ID=0x{:08x} (Ext)".format(can_id),
                           "ID=0x{:08x}E".format(can_id),
                           "ID={:08x}".format(can_id),
                           "{:08x}".format(can_id),
                           "ID(Ext)",
                           "ID",
                           ""]
        else:
            id_end_samplenum = ida.canbits[-1].end_samplenum
            can_id = ida.get_value()
            can_id_strs = ["ID=0x{:03x} (Standard)".format(can_id),
                           "ID=0x{:03x} (Std)".format(can_id),
                           "ID=0x{:03x}".format(can_id),
                           "ID={:03x}".format(can_id),
                           "{:03x}".format(can_id),
                           "ID(Std)",
                           "ID(S)",
                           "ID",
                           ""]
        data = [Annotation.lookup('can-id'), can_id_strs]
        self.put(id_start_samplenum, id_end_samplenum, self.out_ann, data)

    def put_can_field(self, field: CANField):
        start_samplenum, end_samplenum, descriptions = field.get_descriptions()
        if len(descriptions) > 0:
            data = [Annotation.lookup(field.name), descriptions]
            self.put(start_samplenum, end_samplenum, self.out_ann, data)

    def put_can_info(self, canbit: CANBit, descriptions, annotation):
        data = [Annotation.lookup(annotation), descriptions]
        self.put(canbit.start_samplenum, canbit.end_samplenum, self.out_ann, data)

    def put_pcapng_init(self):
        self.put(0, 0, self.out_binary, [0, CANPCAPNG.get_shb()])
        self.put(0, 0, self.out_binary, [0, CANPCAPNG.get_idb()])

    def put_pcapng_epb(self, canbit: CANBit, error_frame=False):
        timestamp_ns = self.num_samples_to_time_ns(canbit.end_samplenum)
        timestamp = int(timestamp_ns / 1000.0)

        if error_frame:
            pcapng_epg = CANPCAPNG.get_epb(ida=0, idb=0, ide=0, rtr=0, data=bytes(), timestamp=timestamp, error_frame=error_frame)
        else:
            ida = CANField.fields['ida'].get_value()
            ide = CANField.fields['ide'].get_value()
            idb = CANField.fields['idb'].get_value() if ide else 0
            rtr = CANField.fields['rtr'].get_value() if ide else CANField.fields['srr'].get_value()
            dlc = CANField.fields['dlc'].get_value()

            if rtr or dlc == 0:
                length = 0
            elif dlc > 8:
                length = 8
            else:
                length = dlc
            data = bytes([CANField.data_bytes[i].get_value() for i in range(length)])

            pcapng_epg = CANPCAPNG.get_epb(ida=ida, idb=idb, ide=ide, rtr=rtr, data=data, timestamp=timestamp, error_frame=error_frame)
            
        self.put(0, 0, self.out_binary, [0, pcapng_epg])
