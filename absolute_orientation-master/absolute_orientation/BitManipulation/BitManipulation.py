"""
This module contains some functions to help with bit arithmetic.
"""

from bitarray import bitarray
from rov_logger.logger import get_rov_logger as __get_rov_logger

log = __get_rov_logger()


def to_hex(num):
    return hex(num)


def to_bin(num):
    return bin(num)


def bin_to_int(s):
    return int(s, 2)


# def hex_to_int(s):
#     return int(s, 16)


def to_bitarray(num):
    """ Converts '0b1111' or int(15) to bitarray('1111') format. """
    if isinstance(num, int):
        return to_bitarray(to_bin(num))
    if isinstance(num, str):
        return bitarray(num[2:])


def bitarray_to_int(ba: bitarray):
    return bin_to_int('0b' + ba.to01())


def leftshift(ba: bitarray, count):
    return ba[count:] + (bitarray('0') * count)


def rightshift(ba: bitarray, count):
    return (bitarray('0') * count) + ba[:-count]


def bitarray_fill_8bit(ba: bitarray) -> bitarray:
    """
    Fills a bitarray object to match 8bit for easier operations.

    E.g., ('11') -> ('0000 0011')
    """
    len_ = ba.length()
    if len_ >= 8:
        return ba
    else:
        aux = bitarray('0')
        aux.extend(ba)
        return bitarray_fill_8bit(aux)


def insert_bit_mask(com, register: int,
                    value: bitarray, mask: bitarray, shift: int):
    """
    Insert a defined amount of bits in a register preserving the remaining
    register values.

    :param com: An i2c communication interface.
    :param register: Actual register position in i2c device.
    :param value: Value to insert as bitarray object. E.g., bitarray('10')
    :param mask: Mask in the actual register as bitarray. E.g., bitarray('00110000')
    :param shift: Bitshift included in mask. E.g., 4
    :return: Turns a register ('1000 0011') into ('1010 0011')
    """
    register_value = com.read_register(register)
    register_value = to_bitarray(register_value)
    register_value = bitarray_fill_8bit(register_value)

    # clean register bits needed in mask
    mask_complement = ~mask
    register_value_clean = register_value & mask_complement

    # shift bits data
    value = bitarray_fill_8bit(value)
    value_shifted = leftshift(value, shift)

    # insert bits in complement mask and then in existent register value
    new_register_value = register_value_clean | value_shifted

    # >> Log register value change for debug.
    log.debug('Updating register %s from value %s to new value %s.' %
              (to_hex(register), register_value, new_register_value))

    com.write(register, bitarray_to_int(new_register_value))


def twos_comp(val, bits) -> int:
    """
    Compute the 2's complement of int value val.

    In python when a register is read it's assumed as an int. This help in cases
    where a two complement number is expected.
    """

    if (val & (1 << (bits - 1))) != 0:  # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)  # compute negative value
    return val



