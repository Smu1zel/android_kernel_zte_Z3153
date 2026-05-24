/*
 * GTPTouchScreen oem config.
 */

#ifndef _GTX8_CONFIG_H_
#define _GTX8_CONFIG_H_

#define GTP_REPORT_BY_ZTE_ALGO
#ifdef GTP_REPORT_BY_ZTE_ALGO
#define gtp_left_edge_limit_v			10
#define gtp_right_edge_limit_v		10
#define gtp_left_edge_limit_h			10
#define gtp_right_edge_limit_h		10
#define gtp_left_edge_long_pess_v		30
#define gtp_right_edge_long_pess_v	30
#define gtp_left_edge_long_pess_h	50
#define gtp_right_edge_long_pess_h	30
#define gtp_long_press_max_count	80
#define gtp_edge_long_press_check	1
#endif

#define GTP_MODULE_NUM	2
#define GTP_VENDOR_ID_0 0
#define GTP_VENDOR_ID_1 1
#define GTP_VENDOR_ID_2 2
#define GTP_VENDOR_ID_3 3

#define GTP_VENDOR_0_NAME                 "GVO_60HZ"
#define GTP_VENDOR_1_NAME                 "GVO_90HZ"
#define GTP_VENDOR_2_NAME                 "unknown"
#define GTP_VENDOR_3_NAME                 "unknown"

#endif

