# -*- coding: utf-8 -*-
# @Author: lorenzo
# @Date:   2019-07-11 10:24:47
# @Last Modified by:   l.rizzello
# @Last Modified time: 2019-08-12 12:05:56

"""
.. module:: cxd5602gnss

************************
Sony CXD5602 GNSS Module
************************

This module implements the Zerynth driver for the Sony CXD5602 GNSS.
It allows configuring and retrieving data from the GNSS module mounted on the Sony Spresense board.

    """

SAT_GPS     = 1   # GPS
SAT_GLONASS = 2   # Glonass
SAT_SBAS    = 4   # SBAS
SAT_QZ_L1CA = 8   # L1CA
SAT_IMES    = 16  # IMES
SAT_QZ_L1S  = 32  # L1S
SAT_BEIDOU  = 64  # BeiDou
SAT_GALILEO = 128 # Galileo

STMOD_COLD       = 0 # Cold Start
STMOD_WARM       = 1 # Warm Start
STMOD_WARM_ACC2  = 2 # Warm Start, better accuracy, less TTFF than WARM
STMOD_HOT        = 3 # Hot Start
STMOD_HOT_ACC    = 4 # Hot Start, better accuracy, less TTFF than HOT
STMOD_HOT_ACC2   = 5 # Hot Start, better accuracy, less TTFF than ACC
STMOD_HOT_ACC3   = 6 # Optimized hot start, better TTFF than HOT

@native_c("_cxd5602gnss_init", 
    [
        "csrc/cxd5602gnss_ifc.c"
    ],
    [
        "VHAL_GNSS"
    ], 
    []
)
def init(start_mode=STMOD_HOT, sat_set=SAT_GPS|SAT_GLONASS, cycle=1000):
    """
.. function:: init(start_mode=STMOD_HOT, sat_set=SAT_GPS|SAT_GLONASS, cycle=1000)

    :param start_mode: start mode
    :param sat_set: set of GNSS satellites
    :param cycle: positioning data evaluation cycle, must be multiple of 1000

    Initializes and starts the GNSS system.
    ``start_mode`` can be one of (with ``TTFF`` meaning ``Time To First Fix``):

        * ``STMOD_COLD``: cold Start;
        * ``STMOD_WARM``: warm Start;
        * ``STMOD_WARM_ACC2``: warm Start, better accuracy, less ``TTFF`` than ``WARM``;
        * ``STMOD_HOT``: hot Start;
        * ``STMOD_HOT_ACC``: hot Start, better accuracy, less ``TTFF`` than ``HOT``;
        * ``STMOD_HOT_ACC2``: hot Start, better accuracy, less ``TTFF`` than ``ACC``;
        * ``STMOD_HOT_ACC3``: optimized hot start, better ``TTFF`` than ``HOT``;

    For more details on the start mode to choose refer to the `official documentation <https://developer.sony.com/develop/spresense/developer-tools/get-started-using-nuttx/nuttx-developer-guide#_gnss>`_.

    ``sat_set`` is a bitmap containing the set of satellites to derive positioning data from.
    Supported satellites are:

        * ``SAT_GPS``: GPS
        * ``SAT_GLONASS``: Glonass
        * ``SAT_SBAS``: SBAS
        * ``SAT_QZ_L1CA``: L1CA
        * ``SAT_IMES``: IMES
        * ``SAT_QZ_L1S``: L1S
        * ``SAT_BEIDOU``: BeiDou
        * ``SAT_GALILEO``: Galileo

    :raise UnsupportedError: when cycle is not a multiple of 1000
    :raise IOError: when initialization fails
    """
    pass

@native_c("_cxd5602gnss_deinit", [])
def deinit():
    """
.. function:: deinit()

    Stops and de-initializes the GNSS system.

    :raise IOError: when de-initialization fails
    """
    pass

@native_c("_cxd5602gnss_wait", [])
def wait():
    """
.. function:: wait()

    Waits for updated GNSS data to become available.

    :raise IOError: when the operation fails
    """
    pass

@native_c("_cxd5602gnss_read", [])
def _read(read_filter):
    pass


FILTER_TIMESTAMP = 1
FILTER_RECEIVER_SATS = 2
FILTER_RECEIVER_POSITION = 4
FILTER_RECEIVER_POSITION_PRECISION = 8
FILTER_RECEIVER_DATETIME = 16
FILTER_RECEIVER = (FILTER_RECEIVER_SATS | FILTER_RECEIVER_POSITION_PRECISION | FILTER_RECEIVER_DATETIME)
FILTER_SATS_DATA = 32

class SatelliteData:

    def __init__(self):
        self.type = None
        self.svid = None
        self.stat = None
        self.elevation = None
        self.azimuth = None
        self.siglevel = None

class ReceiverSats:

    def __init__(self):
        self.numsv = None
        self.numsv_tracking = None
        self.numsv_calcpos = None
        self.numsv_calcvel = None
        self.svtype = None
        self.pos_svtype = None
        self.vel_svtype = None

class DOP:

    def __init__(self, pdop, hdop, vdop, tdop, ewdop, nsdop, majdop, mindop, oridop):
        self.pdop = pdop
        self.hdop = hdop
        self.vdop = vdop
        self.tdop = tdop
        self.ewdop = ewdop
        self.nsdop = nsdop
        self.majdop = majdop
        self.mindop = mindop
        self.oridop = oridop

class Variance:

    def __init__(self, hvar, vvar):
        self.hvar = hvar
        self.vvar = vvar

class ReceiverPositionPrecision:

    def __init__(self):
        self.pos_dop = None
        self.vel_idx = None
        self.pos_accuracy = None

class ReceiverPosition:

    def __init__(self):
        self.type = None
        self.dgps = None
        self.pos_fixmode = None
        self.vel_fixmode = None
        self.assist = None
        self.pos_dataexist = None
        self.possource = None
        self.tcxo_offset = None
        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.geoid = None
        self.velocity = None
        self.direction = None
        self.precision = None


class Date:

    def __init__(self, year, month, day):
        self.year = year
        self.month = month
        self.day = day

class Time:

    def __init__(self, hour, minute, sec, usec):
        self.hour = hour
        self.minute = minute
        self.sec = sec
        self.usec = usec

class ReceiverDatetime:

    def __init__(self):
        self.date = None
        self.time = None
        self.gpsdate = None
        self.gpstime = None
        self.receivetime = None

class Receiver:

    def __init__(self):
        self.sats = None
        self.position = None
        self.datetime = None

class GNSSData:
    def __init__(self):
        self.timestamp = None
        self.receiver = None
        self.sats = None

def read(read_filter=FILTER_RECEIVER_POSITION | FILTER_RECEIVER_DATETIME):
    """
.. function:: read(read_filter=FILTER_RECEIVER_POSITION | FILTER_RECEIVER_DATETIME)

    :param read_filter: bitmap to filter data to be read to save RAM

    Reads GNSS data. 
    To be called after :func:`wait` to be sure of reading updated data.

    Returns a :class:`GNSSData` object filled according to selected filters.
    Available filters are:

        * ``FILTER_TIMESTAMP``: fills ``GNSSData.timestamp``
        * ``FILTER_RECEIVER_SATS``: fills ``GNSSData.receiver.sats``
        * ``FILTER_RECEIVER_POSITION``: fills ``GNSSData.receiver.position``, except for ``GNSSData.receiver.position.precision``
        * ``FILTER_RECEIVER_POSITION_PRECISION``: fills ``GNSSData.receiver.position``, including ``GNSSData.receiver.position.precision``
        * ``FILTER_RECEIVER_DATETIME``: fills ``GNSSData.receiver.datetime``
        * ``FILTER_RECEIVER``: fills ``GNSSData.receiver``
        * ``FILTER_SATS_DATA``: fills ``GNSSData.sats``

    :raise IOError: when read fails, for example if :func:`read` is called immediately after \
        :func:`init` without waiting proper initialization time calling :func:`wait` function.


.. class:: GNSSData()

    Class to store GNSS data retrieved by :func:`read` calls.

    **N.B.** Each attribute might be filled depending on selected read filter.

    List of attributes:

        * :attr:`GNSSData.timestamp`: integer timestamp
        * :attr:`GNSSData.receiver`: :class:`Receiver` instance
        * :attr:`GNSSData.sats`: tuple of :class:`SatelliteData` instances

.. class:: Receiver()

    Class to store Receiver info retrieved by :func:`read` calls.

    **N.B.** Each attribute might be filled depending on selected read filter.

    List of attributes:

        * :attr:`Receiver.sats`: :class:`ReceiverSats` instance
        * :attr:`Receiver.position`: :class:`ReceiverPosition` instance
        * :attr:`Receiver.datetime`: :class:`ReceiverDatetime` instance


.. class:: ReceiverSats()

    Class to store Receiver Satellites info retrieved by :func:`read` calls.

    **N.B.** Each attribute might be filled depending on selected read filter.

    List of attributes:

        * :attr:`ReceiverSats.numsv`: number of visible satellites
        * :attr:`ReceiverSats.numsv_tracking`: number of tracking satellites
        * :attr:`ReceiverSats.numsv_calcpos`: number of satellites to calculate the position
        * :attr:`ReceiverSats.numsv_calcvel`: number of satellites to calculate the velocity
        * :attr:`ReceiverSats.svtype`: used sv system, bitfield. \
                                ``bit0:GPS``, ``bit1:GLONASS``, ``bit2:SBAS``, \
                                ``bit3:QZSS_L1CA``, ``bit4:IMES``, \
                                ``bit5:QZSS_L1SAIF``, ``bit6:Beidu``, \
                                ``bit7:Galileo``
        * :attr:`ReceiverSats.pos_svtype`: used sv system to calculate position, bitfield
        * :attr:`ReceiverSats.vel_svtype`: used sv system to calculate velocity, bitfield

.. class:: ReceiverPosition()

    Class to store Receiver Position info retrieved by :func:`read` calls.

    **N.B.** Each attribute might be filled depending on selected read filter.

    List of attributes:

        * :attr:`ReceiverPosition.type`: position type. ``0:Invalid``, ``1:GNSS``, \
                               ``2:IMES``, ``3:user set``, ``4:previous``
        * :attr:`ReceiverPosition.dgps`: ``0:SGPS``, ``1:DGPS``
        * :attr:`ReceiverPosition.pos_fixmode`: ``1:Invalid``, ``2:2D``, ``3:3D``
        * :attr:`ReceiverPosition.vel_fixmode`: ``1:Invalid``, ``2:2D VZ``, ``3:2D Offset``, \
                               ``4:3D``, ``5:1D``, ``6:PRED``
        * :attr:`ReceiverPosition.assist`: bit field ``[7..5] Reserved`` ``[4] AEP Velocity`` \
                               ``[3] AEP Position`` ``[2] CEP Velocity`` \
                               ``[1] CEP Position``, ``[0] user set``
        * :attr:`ReceiverPosition.pos_dataexist`: ``0:none``, ``1:exist``
        * :attr:`ReceiverPosition.possource`: position source. ``0:Invalid``, ``1:GNSS``, \
                               ``2:IMES``, ``3:user set``, ``4:previous``
        * :attr:`ReceiverPosition.tcxo_offset`: TCXO offset ``[Hz]``
        * :attr:`ReceiverPosition.latitude`: latitude ``[degree]``
        * :attr:`ReceiverPosition.longitude`: longitude ``[degree]``
        * :attr:`ReceiverPosition.altitude`: altitude ``[m]``
        * :attr:`ReceiverPosition.geoid`: geoid height ``[m]``
        * :attr:`ReceiverPosition.velocity`: velocity ``[m/s]``
        * :attr:`ReceiverPosition.direction`: direction ``[degree]``
        * :attr:`ReceiverPosition.precision`: :class:`ReceiverPositionPrecision` instance

.. class:: ReceiverPositionPrecision()

        * :attr:`ReceiverPositionPrecision.pos_dop`: :class:`DOP` instance
        * :attr:`ReceiverPositionPrecision.vel_idx`: :class:`DOP` instance
        * :attr:`ReceiverPositionPrecision.pos_accuracy`: :class:`Variance` instance

.. class:: DOP()

    Class to store Dilution of Precision.

        * :attr:`Dop.pdop`: position DOP
        * :attr:`Dop.hdop`: horizontal DOP
        * :attr:`Dop.vdop`: vertical DOP
        * :attr:`Dop.tdop`: time DOP
        * :attr:`Dop.ewdop`: East-West DOP
        * :attr:`Dop.nsdop`: North-South DOP
        * :attr:`Dop.majdop`: Stdev of semi-major axis
        * :attr:`Dop.mindop`: Stdev of semi-minor axis
        * :attr:`Dop.oridop`: orientation of semi-major axis ``[deg]``

.. class:: Variance()

    Class to store Variance.

        * :attr:`Variance.hvar`: horizontal variance
        * :attr:`Variance.vvar`: vertical variance

    """
    raw_data = _read(read_filter)
    data = GNSSData()
    data.timestamp = raw_data[0]

    if (read_filter & (FILTER_RECEIVER_POSITION | FILTER_RECEIVER_POSITION_PRECISION |
                    FILTER_RECEIVER_DATETIME | FILTER_RECEIVER_SATS)):
        data.receiver = Receiver()

        if raw_data[1][0]: # has sats

            data.receiver.sats = ReceiverSats()
            data.receiver.sats.numsv = raw_data[1][0][0]
            data.receiver.sats.numsv_tracking = raw_data[1][0][1]
            data.receiver.sats.numsv_calcpos = raw_data[1][0][2]
            data.receiver.sats.numsv_calcvel = raw_data[1][0][3]
            data.receiver.sats.svtype = raw_data[1][0][4]
            data.receiver.sats.pos_svtype = raw_data[1][0][5]
            data.receiver.sats.vel_svtype = raw_data[1][0][6]

        if raw_data[1][1]: # has position

            data.receiver.position = ReceiverPosition()
            data.receiver.position.type = raw_data[1][1][0]
            data.receiver.position.dgps = raw_data[1][1][1]
            data.receiver.position.pos_fixmode = raw_data[1][1][2]
            data.receiver.position.vel_fixmode = raw_data[1][1][3]
            data.receiver.position.assist = raw_data[1][1][4]
            data.receiver.position.pos_dataexist = raw_data[1][1][5]
            data.receiver.position.possource = raw_data[1][1][6]
            data.receiver.position.tcxo_offset = raw_data[1][1][7]
            data.receiver.position.latitude = raw_data[1][1][8]
            data.receiver.position.longitude = raw_data[1][1][9]
            data.receiver.position.altitude = raw_data[1][1][10]
            data.receiver.position.geoid = raw_data[1][1][11]
            data.receiver.position.velocity = raw_data[1][1][12]
            data.receiver.position.direction = raw_data[1][1][13]


            if raw_data[1][1][14]: # has position precision
                data.receiver.position.precision = ReceiverPositionPrecision()

                data.receiver.position.precision.pos_dop = DOP(*raw_data[1][1][14][0:9])

                data.receiver.position.precision.vel_idx = DOP(*raw_data[1][1][14][9:18])
                data.receiver.position.precision.pos_accuracy = Variance(raw_data[1][1][14][18], raw_data[1][1][14][19])

        if raw_data[1][2]: # has datetime

            data.receiver.datetime = ReceiverDatetime()

            data.receiver.datetime.date = Date(*raw_data[1][2][0:3])
            data.receiver.datetime.time = Time(*raw_data[1][2][3:7])
            data.receiver.datetime.gpsdate = Date(*raw_data[1][2][7:10])
            data.receiver.datetime.gpstime = Time(*raw_data[1][2][10:14])
            data.receiver.datetime.receivetime = Time(*raw_data[1][2][14:18])

    if read_filter & FILTER_SATS_DATA and len(raw_data[2]):
        data.sats = [None] * len(raw_data[2])
        for sat_i in range(len(raw_data[2])):
            data.sats[sat_i] = SatelliteData()

            data.sats[sat_i].type = raw_data[2][sat_i][0]
            data.sats[sat_i].svid = raw_data[2][sat_i][1]
            data.sats[sat_i].stat = raw_data[2][sat_i][2]
            data.sats[sat_i].elevation = raw_data[2][sat_i][3]
            data.sats[sat_i].azimuth = raw_data[2][sat_i][4]
            data.sats[sat_i].siglevel = raw_data[2][sat_i][5]

    return data

@native_c("_cxd5602gnss_double_to_dmf", [])
def double_to_dmf(x):
    """

Utils
#####

.. function:: double_to_dmf(x)

    :param x: double to convert    

    Converts from double format to degree-minute-frac format.

    Returns a tuple of four elements: ``(sign, degree, minute, frac)``.
    """
    pass
