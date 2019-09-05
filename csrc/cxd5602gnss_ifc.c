/*
* @Author: lorenzo
* @Date:   2019-07-11 10:25:44
* @Last Modified by:   l.rizzello
* @Last Modified time: 2019-07-26 09:57:24
*/

#include <sdk/config.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <errno.h>
#include <stdio.h>
#include <fcntl.h>
#include <poll.h>
#include <arch/chip/gnss.h>
#define ZERYNTH_PRINTF
#include "zerynth.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GNSS_POLL_FD_NUM          1
#define GNSS_POLL_TIMEOUT_FOREVER -1
#define MY_GNSS_SIG               18

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int gnss_fd;
struct cxd56_gnss_signal_setting_s gnss_setting;
sigset_t gnss_sigmask;

static int gnss_disable(void) {
    int ret;

    /* GNSS firmware needs to disable the signal after positioning. */
    gnss_setting.enable = 0;
    ret = ioctl(gnss_fd, CXD56_GNSS_IOCTL_SIGNAL_SET, (unsigned long)&gnss_setting);
    if (ret < 0) {
        printf("signal error\n");
    }

    sigprocmask(SIG_UNBLOCK, &gnss_sigmask, NULL);

    /* Release GNSS file descriptor. */
    ret = close(gnss_fd);
    if (ret < 0) {
      printf("close error %i\n", errno);
    }

    return ret;
}

static int gnss_setparams(uint32_t set_satellite, uint32_t cycle) {
    int      ret = 0;
    struct cxd56_gnss_ope_mode_param_s set_opemode;

    /* Set the GNSS operation interval. */
    set_opemode.mode     = 1;     /* Operation mode:Normal(default). */
    set_opemode.cycle    = cycle;  /* Position notify cycle(msec step). */

    ret = ioctl(gnss_fd, CXD56_GNSS_IOCTL_SET_OPE_MODE, (uint32_t)&set_opemode);
    if (ret < 0) {
        printf("ioctl(CXD56_GNSS_IOCTL_SET_OPE_MODE) NG!!\n");
        goto _err;
    }

    ret = ioctl(gnss_fd, CXD56_GNSS_IOCTL_SELECT_SATELLITE_SYSTEM, set_satellite);
    if (ret < 0) {
        printf("ioctl(CXD56_GNSS_IOCTL_SELECT_SATELLITE_SYSTEM) NG!!\n");
        goto _err;
    }

_err:
    return ret;
}

C_NATIVE(_cxd5602gnss_init) {
    NATIVE_UNWARN();

    int      ret;
    uint32_t set_satellite, cycle, start_mode;

    if (parse_py_args("iii", nargs, args, &start_mode, &set_satellite, &cycle) != 3) {
        return ERR_TYPE_EXC;
    }

    if (cycle % 1000 != 0) {
        // cycle must be multiple of 1000
        return ERR_UNSUPPORTED_EXC;
    } 

    /* Get file descriptor to control GNSS. */
    gnss_fd = open("/dev/gps", O_RDONLY);
    if (gnss_fd < 0) {
        return ERR_IOERROR_EXC;
    }

    /* Configure mask to notify GNSS signal. */
    sigemptyset(&gnss_sigmask);
    sigaddset(&gnss_sigmask, MY_GNSS_SIG);
    ret = sigprocmask(SIG_BLOCK, &gnss_sigmask, NULL);
    if (ret != OK) {
        goto _err;
    }

    /* Set the signal to notify GNSS events. */
    gnss_setting.fd      = gnss_fd;
    gnss_setting.enable  = 1;
    gnss_setting.gnsssig = CXD56_GNSS_SIG_GNSS;
    gnss_setting.signo   = MY_GNSS_SIG;
    gnss_setting.data    = NULL;

    ret = ioctl(gnss_fd, CXD56_GNSS_IOCTL_SIGNAL_SET, (unsigned long)&gnss_setting);
    if (ret < 0) {
        goto _err;
    }

    /* Set GNSS parameters. */
    ret = gnss_setparams(set_satellite, cycle);
    if (ret != OK) {
      goto _err;
    }

    ret = ioctl(gnss_fd, CXD56_GNSS_IOCTL_START, start_mode);
    if (ret < 0) {
        printf("start GNSS ERROR %i\n", errno);
        goto _err;
    }

    return ERR_OK;

_err:
    gnss_disable();
    return ERR_IOERROR_EXC;
}

C_NATIVE(_cxd5602gnss_wait) {
    NATIVE_UNWARN();

    int ret;
    RELEASE_GIL();
    ret = sigwaitinfo(&gnss_sigmask, NULL);
    ACQUIRE_GIL();

    if (ret != MY_GNSS_SIG) {
      return ERR_IOERROR_EXC;
    }

    return ERR_OK;
}

C_NATIVE(_cxd5602gnss_double_to_dmf) {
    NATIVE_UNWARN();

    int    b, d, m;
    double f, t;

    double x;
    if (parse_py_args("f", nargs, args, &x) != 1) {
        return ERR_TYPE_EXC;
    }

    if (x < 0) {
        b = 1;
        x = -x;
    }
    else {
        b = 0;
    }

    d = (int)x; /* = floor(x), x is always positive */
    t = (x - d) * 60;
    m = (int)t; /* = floor(t), t is always positive */
    f = (t - m) * 10000;

    *res = ptuple_new(4, NULL);
    PTUPLE_SET_ITEM(*res, 0, PSMALLINT_NEW(b));
    PTUPLE_SET_ITEM(*res, 0, PSMALLINT_NEW(d));
    PTUPLE_SET_ITEM(*res, 0, PSMALLINT_NEW(m));
    PTUPLE_SET_ITEM(*res, 0, PSMALLINT_NEW(f));

    return ERR_OK;
}

#define FILTER_TIMESTAMP                    1
#define FILTER_RECEIVER_SATS                2
#define FILTER_RECEIVER_POSITION            4
#define FILTER_RECEIVER_POSITION_PRECISION  8
#define FILTER_RECEIVER_DATETIME            16
#define FILTER_SATS_DATA                    32

C_NATIVE(_cxd5602gnss_read) {
    NATIVE_UNWARN();

    int ret;
    struct cxd56_gnss_positiondata_s pos_dat;

    uint32_t read_filter;
    if (parse_py_args("i", nargs, args, &read_filter) != 1) {
        return ERR_TYPE_EXC;
    } 

    /* Read POS data. */
    ret = read(gnss_fd, &pos_dat, sizeof(pos_dat));
    if (ret < 0 || ret != sizeof(pos_dat)) {
        return ERR_IOERROR_EXC;
    }

    *res = ptuple_new(3, NULL);

    // timestamp
    if (read_filter & FILTER_TIMESTAMP) {
        PTUPLE_SET_ITEM(*res, 0, pinteger_new(pos_dat.data_timestamp));
    }
    else {
        PTUPLE_SET_ITEM(*res, 0, MAKE_NONE());   
    }

    // receiver
    PTuple *receiver = ptuple_new(3, NULL);
    PTuple *sats, *position, *position_precision, *date;

    // receiver sats
    if (read_filter & FILTER_RECEIVER_SATS) {
        sats = ptuple_new(7, NULL);

        PTUPLE_SET_ITEM(sats, 0, PSMALLINT_NEW(pos_dat.receiver.numsv)); 
        PTUPLE_SET_ITEM(sats, 1, PSMALLINT_NEW(pos_dat.receiver.numsv_tracking));
        PTUPLE_SET_ITEM(sats, 2, PSMALLINT_NEW(pos_dat.receiver.numsv_calcpos));
        PTUPLE_SET_ITEM(sats, 3, PSMALLINT_NEW(pos_dat.receiver.numsv_calcvel));
        PTUPLE_SET_ITEM(sats, 4, PSMALLINT_NEW(pos_dat.receiver.svtype));
        PTUPLE_SET_ITEM(sats, 5, PSMALLINT_NEW(pos_dat.receiver.pos_svtype));
        PTUPLE_SET_ITEM(sats, 6, PSMALLINT_NEW(pos_dat.receiver.vel_svtype));
    }
    else {
        sats = ptuple_new(0, NULL);
    }
    PTUPLE_SET_ITEM(receiver, 0, sats);

    // receiver position
    if (read_filter & FILTER_RECEIVER_POSITION || 
        read_filter & FILTER_RECEIVER_POSITION_PRECISION) {

        position = ptuple_new(15, NULL);

        PTUPLE_SET_ITEM(position, 0, PSMALLINT_NEW(pos_dat.receiver.type));
        PTUPLE_SET_ITEM(position, 1, PSMALLINT_NEW(pos_dat.receiver.dgps));
        PTUPLE_SET_ITEM(position, 2, PSMALLINT_NEW(pos_dat.receiver.pos_fixmode));
        PTUPLE_SET_ITEM(position, 3, PSMALLINT_NEW(pos_dat.receiver.vel_fixmode));
        PTUPLE_SET_ITEM(position, 4, PSMALLINT_NEW(pos_dat.receiver.assist));
        PTUPLE_SET_ITEM(position, 5, PSMALLINT_NEW(pos_dat.receiver.pos_dataexist));
        PTUPLE_SET_ITEM(position, 6, PSMALLINT_NEW(pos_dat.receiver.possource));
        PTUPLE_SET_ITEM(position, 7, PSMALLINT_NEW(pos_dat.receiver.tcxo_offset));
        PTUPLE_SET_ITEM(position, 8, pfloat_new((FLOAT_TYPE) pos_dat.receiver.latitude));
        PTUPLE_SET_ITEM(position, 9, pfloat_new((FLOAT_TYPE) pos_dat.receiver.longitude));
        PTUPLE_SET_ITEM(position, 10, pfloat_new((FLOAT_TYPE) pos_dat.receiver.altitude));
        PTUPLE_SET_ITEM(position, 11, pfloat_new((FLOAT_TYPE) pos_dat.receiver.geoid));
        PTUPLE_SET_ITEM(position, 12, pfloat_new((FLOAT_TYPE) pos_dat.receiver.velocity));
        PTUPLE_SET_ITEM(position, 13, pfloat_new((FLOAT_TYPE) pos_dat.receiver.direction));

        if (read_filter & FILTER_RECEIVER_POSITION_PRECISION) {
            position_precision = ptuple_new(20, NULL);

            PTUPLE_SET_ITEM(position_precision, 0, pfloat_new(pos_dat.receiver.pos_dop.pdop));
            PTUPLE_SET_ITEM(position_precision, 1, pfloat_new(pos_dat.receiver.pos_dop.hdop));
            PTUPLE_SET_ITEM(position_precision, 2, pfloat_new(pos_dat.receiver.pos_dop.vdop));
            PTUPLE_SET_ITEM(position_precision, 3, pfloat_new(pos_dat.receiver.pos_dop.tdop));
            PTUPLE_SET_ITEM(position_precision, 4, pfloat_new(pos_dat.receiver.pos_dop.ewdop));
            PTUPLE_SET_ITEM(position_precision, 5, pfloat_new(pos_dat.receiver.pos_dop.nsdop));
            PTUPLE_SET_ITEM(position_precision, 6, pfloat_new(pos_dat.receiver.pos_dop.majdop));
            PTUPLE_SET_ITEM(position_precision, 7, pfloat_new(pos_dat.receiver.pos_dop.mindop));
            PTUPLE_SET_ITEM(position_precision, 8, pfloat_new(pos_dat.receiver.pos_dop.oridop));
            PTUPLE_SET_ITEM(position_precision, 9, pfloat_new(pos_dat.receiver.vel_idx.pdop));
            PTUPLE_SET_ITEM(position_precision, 10, pfloat_new(pos_dat.receiver.vel_idx.hdop));
            PTUPLE_SET_ITEM(position_precision, 11, pfloat_new(pos_dat.receiver.vel_idx.vdop));
            PTUPLE_SET_ITEM(position_precision, 12, pfloat_new(pos_dat.receiver.vel_idx.tdop));
            PTUPLE_SET_ITEM(position_precision, 13, pfloat_new(pos_dat.receiver.vel_idx.ewdop));
            PTUPLE_SET_ITEM(position_precision, 14, pfloat_new(pos_dat.receiver.vel_idx.nsdop));
            PTUPLE_SET_ITEM(position_precision, 15, pfloat_new(pos_dat.receiver.vel_idx.majdop));
            PTUPLE_SET_ITEM(position_precision, 16, pfloat_new(pos_dat.receiver.vel_idx.mindop));
            PTUPLE_SET_ITEM(position_precision, 17, pfloat_new(pos_dat.receiver.vel_idx.oridop));
            PTUPLE_SET_ITEM(position_precision, 18, pfloat_new(pos_dat.receiver.pos_accuracy.hvar));
            PTUPLE_SET_ITEM(position_precision, 19, pfloat_new(pos_dat.receiver.pos_accuracy.vvar));

            PTUPLE_SET_ITEM(position, 14, position_precision);
        }
        else {
            PTUPLE_SET_ITEM(position, 14, MAKE_NONE());
        }
    }
    else {
        position = ptuple_new(0, NULL);
    }
    PTUPLE_SET_ITEM(receiver, 1, position);

    // receiver date
    if (read_filter & FILTER_RECEIVER_DATETIME) {
        date = ptuple_new(18, NULL);

        PTUPLE_SET_ITEM(date, 0, PSMALLINT_NEW(pos_dat.receiver.date.year));
        PTUPLE_SET_ITEM(date, 1, PSMALLINT_NEW(pos_dat.receiver.date.month));
        PTUPLE_SET_ITEM(date, 2, PSMALLINT_NEW(pos_dat.receiver.date.day));
        PTUPLE_SET_ITEM(date, 3, PSMALLINT_NEW(pos_dat.receiver.time.hour));
        PTUPLE_SET_ITEM(date, 4, PSMALLINT_NEW(pos_dat.receiver.time.minute));
        PTUPLE_SET_ITEM(date, 5, PSMALLINT_NEW(pos_dat.receiver.time.sec));
        PTUPLE_SET_ITEM(date, 6, PSMALLINT_NEW(pos_dat.receiver.time.usec));
        PTUPLE_SET_ITEM(date, 7, PSMALLINT_NEW(pos_dat.receiver.gpsdate.year));
        PTUPLE_SET_ITEM(date, 8, PSMALLINT_NEW(pos_dat.receiver.gpsdate.month));
        PTUPLE_SET_ITEM(date, 9, PSMALLINT_NEW(pos_dat.receiver.gpsdate.day));
        PTUPLE_SET_ITEM(date, 10, PSMALLINT_NEW(pos_dat.receiver.gpstime.hour));
        PTUPLE_SET_ITEM(date, 11, PSMALLINT_NEW(pos_dat.receiver.gpstime.minute));
        PTUPLE_SET_ITEM(date, 12, PSMALLINT_NEW(pos_dat.receiver.gpstime.sec));
        PTUPLE_SET_ITEM(date, 13, PSMALLINT_NEW(pos_dat.receiver.gpstime.usec));
        PTUPLE_SET_ITEM(date, 14, PSMALLINT_NEW(pos_dat.receiver.receivetime.hour));
        PTUPLE_SET_ITEM(date, 15, PSMALLINT_NEW(pos_dat.receiver.receivetime.minute));
        PTUPLE_SET_ITEM(date, 16, PSMALLINT_NEW(pos_dat.receiver.receivetime.sec));
        PTUPLE_SET_ITEM(date, 17, PSMALLINT_NEW(pos_dat.receiver.receivetime.usec));
    }
    else {
        date = ptuple_new(0, NULL);
    }
    PTUPLE_SET_ITEM(receiver, 2, date);

    PTUPLE_SET_ITEM(*res, 1, receiver);

    // sats data

    PTuple *sats_data;
    if (read_filter & FILTER_SATS_DATA) {
        sats_data = ptuple_new(pos_dat.svcount, NULL);

        PTuple *sat_data;
        for (uint32_t svcount_i = 0; svcount_i < pos_dat.svcount; svcount_i++) {
            sat_data = ptuple_new(6, NULL); // single satellite data
            PTUPLE_SET_ITEM(sat_data, 0, PSMALLINT_NEW(pos_dat.sv[svcount_i].type));
            PTUPLE_SET_ITEM(sat_data, 1, PSMALLINT_NEW(pos_dat.sv[svcount_i].svid));
            PTUPLE_SET_ITEM(sat_data, 2, PSMALLINT_NEW(pos_dat.sv[svcount_i].stat));
            PTUPLE_SET_ITEM(sat_data, 3, PSMALLINT_NEW(pos_dat.sv[svcount_i].elevation));
            PTUPLE_SET_ITEM(sat_data, 4, PSMALLINT_NEW(pos_dat.sv[svcount_i].azimuth));
            PTUPLE_SET_ITEM(sat_data, 5, pfloat_new(pos_dat.sv[svcount_i].siglevel));

            PTUPLE_SET_ITEM(sats_data, svcount_i, sat_data);
        }
    }
    else {
        sats_data = ptuple_new(0, NULL);
    }

    PTUPLE_SET_ITEM(*res, 2, sats_data);

    return ERR_OK;
}

C_NATIVE(_cxd5602gnss_deinit) {
    NATIVE_UNWARN();

    int ret;
    /* Stop GNSS. */

    ret = ioctl(gnss_fd, CXD56_GNSS_IOCTL_STOP, 0);
    if (ret < 0) {
        return ERR_IOERROR_EXC;
    }
    ret = gnss_disable();
    if (ret < 0) {
        return ERR_IOERROR_EXC;
    }

    return ERR_OK;
}
