#ifndef UBLOX_H
#define UBLOX_H

#include <stdlib.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/poll.h>

#include "core.h"
#include "tcp.h"
#include "serial.h"

/**
 * UBX Class IDs
 * -------------
 * NAV 0x01 Navigation Results: Position, Speed, Time, Acceleration, Heading,
 * DOP, SVs used RXM 0x02 Receiver Manager: Satellite Status, RTC Status INF
 * 0x04 Information: Printf-Style Messages, with IDs such as Error, Warning,
 * Notice ACK 0x05 Ack/Nak: Acknowledge or Reject messages to UBX-CFG input
 * messages CFG 0x06 Configuration Input: Set Dynamic Model, Set DOP Mask, Set
 * Baud Rate, etc. UPD 0x09 Firmware Update: Memory/Flash erase/write, Reboot,
 * Flash identification, etc. MON 0x0A Monitoring: Communication Status, CPU
 * Load, Stack Usage, Task Status TIM 0x0D Timing: Time Pulse Output, Time Mark
 * Results MGA 0x13 Multiple GNSS Assistance: Assistance data for various GNSS
 * LOG 0x21 Logging: Log creation, deletion, info and retrieval
 * SEC 0x27 Security Feature
 */
#define UBX_NAV 0x01
#define UBX_RXM 0x02
#define UBX_INF 0x04
#define UBX_ACK 0x05
#define UBX_CFG 0x06
#define UBX_UPD 0x09
#define UBX_MON 0x0A
#define UBX_TIM 0x0D
#define UBX_MGA 0x13
#define UBX_LOG 0x21
#define UBX_SEC 0x27

/**
 * UBX Class CFG
 * -------------
 * ACK-ACK 0x05 0x01 2 Output Message Acknowledged
 * ACK-NAK 0x05 0x00 2 Output Message Not-Acknowledged
 */
#define UBX_ACK_ACK 0x01
#define UBX_ACK_NAK 0x00

/**
 * UBX Class CFG
 * -------------
 * CFG-VALDEL 0x06 0x8C 4 + 4*N Set Deletes values corresponding to...
 * CFG-VALGET 0x06 0x8B 4 + 4*N Poll Request Get Configuration Items
 * CFG-VALSET 0x06 0x8A 4 + 1*N Set Sets values corresponding to provided...
 */
#define UBX_CFG_VALDEL 0x8C
#define UBX_CFG_VALGET 0x8B
#define UBX_CFG_VALSET 0x8A

/**
 * UBX Class MON Monitoring Messages
 * ---------------------------------
 * MON-COMMS 0x0A 0x36 8 + 40*nPorts Periodic/Polled Comm port information
 * MON-GNSS 0x0A 0x28 8 Polled Information message major GNSS...
 * MON-HW2 0x0A 0x0B 28 Periodic/Polled Extended Hardware Status
 * MON-HW3 0x0A 0x37 22 + 6*nPins Periodic/Polled HW I/O pin information
 * MON-HW 0x0A 0x09 60 Periodic/Polled Hardware Status
 * MON-IO 0x0A 0x02 0 + 20*N Periodic/Polled I/O Subsystem Status
 * MON-MSGPP 0x0A 0x06 120 Periodic/Polled Message Parse and Process Status
 * MON-PATCH 0x0A 0x27 4 + 16*nEntries Polled Output information about installed...
 * MON-RF 0x0A 0x38 4 + 24*nBlocks Periodic/Polled RF information
 * MON-RXBUF 0x0A 0x07 24 Periodic/Polled Receiver Buffer Status
 * MON-RXR 0x0A 0x21 1 Output Receiver Status Information
 * MON-TXBUF 0x0A 0x08 28 Periodic/Polled Transmitter Buffer Status
 * MON-VER 0x0A 0x04 40 + 30*N Polled Receiver/Software Version
 */
#define UBX_MON_COMMS 0x36
#define UBX_MON_GNSS 0x28
#define UBX_MON_HW2 0x0B
#define UBX_MON_HW3 0x37
#define UBX_MON_HW 0x09
#define UBX_MON_IO 0x02
#define UBX_MON_MSGPP 0x06
#define UBX_MON_PATCH 0x27
#define UBX_MON_RF 0x38
#define UBX_MON_RXBUF 0x07
#define UBX_MON_RXR 0x21
#define UBX_MON_TXBUF 0x08
#define UBX_MON_VER 0x04

/**
 * UBX Class NAV Navigation Results Messages
 * -----------------------------------------
 * NAV-CLOCK 0x01 0x22 20 Periodic/Polled Clock Solution
 * NAV-DOP 0x01 0x04 18 Periodic/Polled Dilution of precision
 * NAV-EOE 0x01 0x61 4 Periodic End Of Epoch
 * NAV-GEOFENCE 0x01 0x39 8 + 2*numFe... Periodic/Polled Geofencing status
 * NAV-HPPOSECEF 0x01 0x13 28 Periodic/Polled High Precision Position Solution
 * in ECEF NAV-HPPOSLLH 0x01 0x14 36 Periodic/Polled High Precision Geodetic
 * Position Solution NAV-ODO 0x01 0x09 20 Periodic/Polled Odometer Solution
 * NAV-ORB 0x01 0x34 8 + 6*numSv Periodic/Polled GNSS Orbit Database Info
 * NAV-POSECEF 0x01 0x01 20 Periodic/Polled Position Solution in ECEF
 * NAV-POSLLH 0x01 0x02 28 Periodic/Polled Geodetic Position Solution
 * NAV-PVT 0x01 0x07 92 Periodic/Polled Navigation Position Velocity Time...
 * NAV-RELPOSNED 0x01 0x3C 64 Periodic/Polled Relative Positioning Information
 * in... NAV-RESETODO 0x01 0x10 0 Command Reset odometer NAV-SAT 0x01 0x35 8 +
 * 12*numSvs Periodic/Polled Satellite Information NAV-SIG 0x01 0x43 8 +
 * 16*numSi... Periodic/Polled Signal Information NAV-STATUS 0x01 0x03 16
 * Periodic/Polled Receiver Navigation Status NAV-SVIN 0x01 0x3B 40
 * Periodic/Polled Survey-in data NAV-TIMEBDS 0x01 0x24 20 Periodic/Polled BDS
 * Time Solution NAV-TIMEGAL 0x01 0x25 20 Periodic/Polled Galileo Time Solution
 * NAV-TIMEGLO 0x01 0x23 20 Periodic/Polled GLO Time Solution
 * NAV-TIMEGPS 0x01 0x20 16 Periodic/Polled GPS Time Solution
 * NAV-TIMELS 0x01 0x26 24 Periodic/Polled Leap second event information
 * NAV-TIMEUTC 0x01 0x21 20 Periodic/Polled UTC Time Solution
 * NAV-VELECEF 0x01 0x11 20 Periodic/Polled Velocity Solution in ECEF
 * NAV-VELNED 0x01 0x12 36 Periodic/Polled Velocity Solution in NED
 */
#define UBX_NAV_CLOCK 0x22
#define UBX_NAV_DOP 0x04
#define UBX_NAV_EOE 0x61
#define UBX_NAV_GEOFENCE 0x39
#define UBX_NAV_HPPOSECEF 0x13
#define UBX_NAV_HPPOSLLH 0x14
#define UBX_NAV_ODO 0x09
#define UBX_NAV_ORB 0x34
#define UBX_NAV_POSECEF 0x01
#define UBX_NAV_POSLLH 0x02
#define UBX_NAV_PVT 0x07
#define UBX_NAV_RELPOSNED 0x3C
#define UBX_NAV_RESETODO 0x10
#define UBX_NAV_SAT 0x35
#define UBX_NAV_SIG 0x43
#define UBX_NAV_STATUS 0x03
#define UBX_NAV_SVIN 0x3B
#define UBX_NAV_TIMEBDS 0x24
#define UBX_NAV_TIMEGAL 0x25
#define UBX_NAV_TIMEGLO 0x23
#define UBX_NAV_TIMEGPS 0x20
#define UBX_NAV_TIMELS 0x26
#define UBX_NAV_TIMEUTC 0x21
#define UBX_NAV_VELECEF 0x11
#define UBX_NAV_VELNED 0x12

/**
 * UBX Class RXM Receiver Manager Messages
 * ---------------------------------------
 * RXM-MEASX 0x02 0x14 44 + 24*num... Periodic/Polled Satellite Measurements for
 * RRLP RXM-PMREQ 0x02 0x41 8 Command Requests a Power Management task RXM-PMREQ
 * 0x02 0x41 16 Command Requests a Power Management task RXM-RAWX 0x02 0x15 16 +
 * 32*num... Periodic/Polled Multi-GNSS Raw Measurement Data RXM-RLM 0x02 0x59
 * 16 Output Galileo SAR Short-RLM report RXM-RLM 0x02 0x59 28 Output Galileo
 * SAR Long-RLM report RXM-RTCM 0x02 0x32 8 Output RTCM input status RXM-SFRBX
 * 0x02 0x13 8 + 4*numW... Output
 */
#define UBX_RXM_MEASX 0x14
#define UBX_RXM_PMREQ 0x41
#define UBX_RXM_RAWX 0x15
#define UBX_RXM_RLM 0x59
#define UBX_RXM_RTCM 0x32
#define UBX_RXM_SFRBX 0x13

#define CFG_SIGNAL_GPS_ENA 0x1031001f
#define CFG_SIGNAL_GPS_L1CA_ENA 0x10310001
#define CFG_SIGNAL_QZSS_ENA 0x10310024
#define CFG_SIGNAL_BDS_B2_ENA 0x1031000e

#define CFG_RATE_MEAS 0x30210001
#define CFG_UART1_BAUDRATE 0x40520001
#define CFG_USBOUTPROT_NMEA 0x10780002

#define CFG_MSGOUT_RTCM_3X_TYPE1005_USB 0x209102c0
#define CFG_MSGOUT_RTCM_3X_TYPE1077_USB 0x209102cf
#define CFG_MSGOUT_RTCM_3X_TYPE1087_USB 0x209102d4
#define CFG_MSGOUT_RTCM_3X_TYPE1097_USB 0x2091031b
#define CFG_MSGOUT_RTCM_3X_TYPE1127_USB 0x209102d9
#define CFG_MSGOUT_RTCM_3X_TYPE1230_USB 0x20910306

#define CFG_MSGOUT_UBX_NAV_CLOCK_USB 0x20910068
#define CFG_MSGOUT_UBX_NAV_DOP_USB 0x2091003b
#define CFG_MSGOUT_UBX_NAV_EOE_USB 0x20910162
#define CFG_MSGOUT_UBX_NAV_HPPOSEECF_USB 0x20910031
#define CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB 0x20910036
#define CFG_MSGOUT_UBX_NAV_RELPOSNED_USB 0x20910090
#define CFG_MSGOUT_UBX_NAV_STATUS_USB 0x2091001d
#define CFG_MSGOUT_UBX_NAV_SVIN_USB 0x2091008b
#define CFG_MSGOUT_UBX_NAV_PVT_USB 0x20910009
#define CFG_MSGOUT_UBX_NAV_VELNED_USB 0x20910045
#define CFG_MSGOUT_UBX_MON_RF_USB 0x2091035c
#define CFG_MSGOUT_UBX_RXM_RTCM_USB 0x2091026b

#define CFG_TMODE_MODE 0x20030001
#define CFG_TMODE_SVIN_MIN_DUR 0x40030010
#define CFG_TMODE_SVIN_ACC_LIMIT 0x40030011

#define CFG_NAVSPG_DYNMODEL 0x20110021

/*****************************************************************************
 * UBX Message
 ****************************************************************************/

#define UBX_MON_RF_MAX_BLOCKS 100

typedef struct ubx_msg_t {
  uint8_t ok;

  uint8_t msg_class;
  uint8_t msg_id;
  uint16_t payload_length;
  uint8_t payload[1024];
  uint8_t ck_a;
  uint8_t ck_b;
} ubx_msg_t;

typedef struct ubx_nav_dop_t {
  uint32_t itow;
  uint16_t gdop;
  uint16_t pdop;
  uint16_t tdop;
  uint16_t vdop;
  uint16_t hdop;
  uint16_t ndop;
  uint16_t edop;
} ubx_nav_dop_t;

typedef struct ubx_nav_eoe_t {
  uint32_t itow;
} ubx_nav_eoe_t;

typedef struct ubx_nav_hpposllh_t {
  uint8_t version;
  uint32_t itow;
  int32_t lon;
  int32_t lat;
  int32_t height;
  int32_t hmsl;
  int8_t lon_hp;
  int8_t lat_hp;
  int8_t height_hp;
  int8_t hmsl_hp;
  uint32_t hacc;
  uint32_t vacc;
} ubx_nav_hpposllh_t;

typedef struct ubx_nav_pvt_t {
  uint32_t itow;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t valid;
  uint32_t tacc;
  int32_t nano;
  uint8_t fix_type;
  uint8_t flags;
  uint8_t flags2;
  uint8_t num_sv;
  uint32_t lon;
  uint32_t lat;
  uint32_t height;
  uint32_t hmsl;
  int32_t hacc;
  int32_t vacc;
  int32_t veln;
  int32_t vele;
  int32_t veld;
  int32_t gspeed;
  int32_t headmot;
  uint32_t sacc;
  uint32_t headacc;
  uint16_t pdop;
  int32_t headveh;
  int16_t magdec;
  uint16_t magacc;
} ubx_nav_pvt_t;

typedef struct ubx_nav_status_t {
  uint32_t itow;
  uint8_t fix;
  uint8_t flags;
  uint8_t fix_status;
  uint8_t flags2;
  uint32_t ttff;
  uint32_t msss;
} ubx_nav_status_t;

typedef struct ubx_nav_svin_t {
  uint32_t itow;
  uint32_t dur;
  int32_t mean_x;
  int32_t mean_y;
  int32_t mean_z;
  int8_t mean_xhp;
  int8_t mean_yhp;
  int8_t mean_zhp;
  uint32_t mean_acc;
  uint32_t obs;
  uint8_t valid;
  uint8_t active;
} ubx_nav_svin_t;

typedef struct ubx_nav_velned_t {
  uint32_t itow;
  int32_t veln;
  int32_t vele;
  int32_t veld;
  uint32_t speed;
  uint32_t gspeed;
  int32_t heading;
  uint32_t sacc;
  uint32_t cacc;
} ubx_nav_velned_t;

typedef struct ubx_rxm_rtcm_t {
  uint8_t flags;
  uint16_t sub_type;
  uint16_t ref_station;
  uint16_t msg_type;
} ubx_rxm_rtcm_t;

typedef struct ubx_mon_rf_t {
  uint8_t version;
  uint32_t nblocks;

  uint8_t block_id[UBX_MON_RF_MAX_BLOCKS];
  uint8_t flags[UBX_MON_RF_MAX_BLOCKS];
  uint8_t ant_status[UBX_MON_RF_MAX_BLOCKS];
  uint8_t ant_power[UBX_MON_RF_MAX_BLOCKS];
  uint32_t post_status[UBX_MON_RF_MAX_BLOCKS];
  uint16_t noise_per_ms[UBX_MON_RF_MAX_BLOCKS];
  uint16_t agc_cnt[UBX_MON_RF_MAX_BLOCKS];
  uint8_t jam_ind[UBX_MON_RF_MAX_BLOCKS];
  int8_t ofs_i[UBX_MON_RF_MAX_BLOCKS];
  uint8_t mag_i[UBX_MON_RF_MAX_BLOCKS];
  int8_t ofs_q[UBX_MON_RF_MAX_BLOCKS];
  uint8_t mag_q[UBX_MON_RF_MAX_BLOCKS];
} ubx_mon_rf_t;

void ubx_msg_init(ubx_msg_t *msg);
void ubx_msg_checksum(const uint8_t msg_class,
                      const uint8_t msg_id,
                      const uint16_t payload_length,
                      const uint8_t *payload,
                      uint8_t *ck_a,
                      uint8_t *ck_b);
uint8_t ubx_msg_is_valid(const ubx_msg_t *msg);
void ubx_msg_build(ubx_msg_t *msg,
                   const uint8_t msg_class,
                   const uint8_t msg_id,
                   const uint16_t length,
                   const uint8_t *payload);
void ubx_msg_parse(ubx_msg_t *msg, const uint8_t *data);
void ubx_msg_serialize(const ubx_msg_t *msg,
                       uint8_t *frame,
                       size_t *frame_size);
void ubx_msg_print(const ubx_msg_t *msg);

ubx_nav_dop_t ubx_nav_dop(const ubx_msg_t *msg);
ubx_nav_eoe_t ubx_nav_eoe(const ubx_msg_t *msg);
ubx_nav_hpposllh_t ubx_nav_hpposllh(const ubx_msg_t *msg);
ubx_nav_pvt_t ubx_nav_pvt(const ubx_msg_t *msg);
ubx_nav_status_t ubx_nav_status(const ubx_msg_t *msg);
ubx_nav_svin_t ubx_nav_svin(const ubx_msg_t *msg);
ubx_nav_velned_t ubx_nav_velned(const ubx_msg_t *msg);
ubx_rxm_rtcm_t ubx_rxm_rtcm(const ubx_msg_t *msg);

void print_ubx_nav_hpposllh(const ubx_nav_hpposllh_t *msg);
void print_ubx_nav_pvt(const ubx_nav_pvt_t *msg);
void print_ubx_nav_status(const ubx_nav_status_t *msg);
void print_ubx_nav_svin(const ubx_nav_svin_t *msg);
void print_ubx_rxm_rtcm(const ubx_rxm_rtcm_t *msg);


/*****************************************************************************
 * UBX Stream Parser
 ****************************************************************************/

/**
 * UBX Stream Parser States
 */
#define SYNC_1 0
#define SYNC_2 1
#define MSG_CLASS 2
#define MSG_ID 3
#define PAYLOAD_LENGTH_LOW 4
#define PAYLOAD_LENGTH_HI 5
#define PAYLOAD_DATA 6
#define CK_A 7
#define CK_B 8

/**
 * UBX Stream Parser
 */
typedef struct ubx_parser_t {
  uint8_t state;
  uint8_t buf_data[9046];
  size_t buf_pos;
  ubx_msg_t msg;
} ubx_parser_t;

void ubx_parser_init(ubx_parser_t *parser);
void ubx_parser_reset(ubx_parser_t *parser);
int ubx_parser_update(ubx_parser_t *parser, uint8_t data);

/*****************************************************************************
 * RTCM3 Stream Parser
 ****************************************************************************/

/**
 * RTCM3 Stream Parser
 */
typedef struct rtcm3_parser_t {
  uint8_t buf_data[9046];
  size_t buf_pos;
  size_t msg_len;
  size_t msg_type;
} rtcm3_parser_t;

void rtcm3_parser_init(rtcm3_parser_t *parser);
void rtcm3_parser_reset(rtcm3_parser_t *parser);

/**
 * RTCM 3.2 Frame
 * --------------
 * Byte 0: Always 0xD3
 * Byte 1: 6-bits of zero
 * Byte 2: 10-bits of length of this packet including the first two-ish header
 *         bytes, + 6.
 * byte 3 + 4: Msg type 12 bits
 *
 * Example [Msg type 1087]:
 *
 *   D3 00 7C 43 F0 ...
 *
 * Where 0x7C is the payload size = 124
 * = 124 + 6 [header]
 * = 130 total bytes in this packet
 */
int rtcm3_parser_update(rtcm3_parser_t *parser, uint8_t data);

/*****************************************************************************
 * UBlox
 ****************************************************************************/

#define UBLOX_MAX_CONNS 10
#define UBLOX_READY 0
#define UBLOX_PARSING_UBX 1
#define UBLOX_PARSING_RTCM3 2

typedef struct ublox_t ublox_t;
typedef void (*ubx_msg_callback)(ublox_t *ublox);
typedef void (*rtcm3_msg_callback)(ublox_t *ublox);

/**
 * UBlox
 */
typedef struct ublox_t {
  int state;
  uint8_t ok;
  serial_t serial;

  int sockfd;
  int conns[UBLOX_MAX_CONNS];
  size_t nb_conns;

  ubx_parser_t ubx_parser;
  ubx_msg_callback ubx_cb;

  rtcm3_parser_t rtcm3_parser;
  rtcm3_msg_callback rtcm3_cb;

} ublox_t;

void ublox_init(ublox_t *ublox);
void ublox_reset(ublox_t *ublox);
int ublox_connect(ublox_t *ublox);
void ublox_disconnect(ublox_t *ublox);

int ubx_write(const ublox_t *ublox,
              uint8_t msg_class,
              uint8_t msg_id,
              uint16_t length,
              uint8_t *payload);
int ubx_poll(const ublox_t *ublox,
             const uint8_t msg_class,
             const uint8_t msg_id,
             uint16_t *payload_length,
             uint8_t *payload,
             const uint8_t expect_ack,
             const int retry);
int ubx_read_ack(const ublox_t *ublox, uint8_t msg_class, uint8_t msg_id);
int ubx_val_get(const ublox_t *ublox,
                const uint8_t layer,
                const uint32_t key,
                uint32_t *val);
int ubx_val_set(const ublox_t *ublox,
                const uint8_t layer,
                const uint32_t key,
                const uint32_t val,
                const uint8_t val_size);

void ublox_version(const ublox_t *ublox);
int ublox_parse_ubx(ublox_t *ublox, uint8_t data);
int ublox_parse_rtcm3(ublox_t *ublox, uint8_t data);

int ublox_base_station_config(ublox_t *base);
void ublox_base_station_loop(ublox_t *base);
int ublox_base_station_run(ublox_t *base, const int port);

int ublox_rover_config(ublox_t *rover);
void ublox_rover_loop(ublox_t *rover);
int ublox_rover_run(ublox_t *rover, const char *base_ip, const int base_port);

#endif /* UBLOX_H */
