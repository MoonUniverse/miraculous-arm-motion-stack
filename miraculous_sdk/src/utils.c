/**
 * @file    utils.c
 * @brief   SDK 工具函数
 */

#include "miraculous_internal.h"

const char* mrc_strerror(int err)
{
    switch (err) {
    case MRC_SUCCESS:               return "Success";
    case MRC_ERROR_UNKNOWN:         return "Unknown error";
    case MRC_ERROR_INVALID_PARAM:   return "Invalid parameter";
    case MRC_ERROR_NOT_INIT:        return "Not initialized";
    case MRC_ERROR_TIMEOUT:         return "Operation timed out";
    case MRC_ERROR_NOT_SUPPORTED:   return "Operation not supported";
    case MRC_ERROR_CAN_OPEN:        return "CAN socket open failed";
    case MRC_ERROR_CAN_SEND:        return "CAN frame send failed";
    case MRC_ERROR_CAN_RECV:        return "CAN frame receive failed";
    case MRC_ERROR_CAN_SOCKET:      return "Socket operation failed";
    case MRC_ERROR_CAN_IOCTL:       return "ioctl call failed";
    case MRC_ERROR_CAN_BIND:        return "Socket bind failed";
    case MRC_ERROR_CAN_TX_FULL:     return "CAN TX buffer full";
    case MRC_ERROR_CO_SDO_UPLOAD_TIMEOUT:   return "SDO upload timeout";
    case MRC_ERROR_CO_SDO_DOWNLOAD_TIMEOUT: return "SDO download timeout";
    case MRC_ERROR_CO_SDO_ABORT:     return "SDO aborted by slave";
    case MRC_ERROR_CO_SDO_TOGGLE:    return "SDO toggle bit error";
    case MRC_ERROR_CO_NODE_NOT_FOUND: return "CANopen node not found";
    case MRC_ERROR_CO_NMT_FAILED:    return "NMT command failed";
    case MRC_ERROR_CO_PDO_CONFIG:    return "PDO configuration failed";
    case MRC_ERROR_CO_HEARTBEAT_LOST: return "Heartbeat lost";
    case MRC_ERROR_CO_WRONG_NMT_STATE: return "Wrong NMT state for operation";
    case MRC_ERROR_MOTION_STATE_TRANSITION: return "Motion state transition failed";
    case MRC_ERROR_MOTION_FAULT:     return "Motor in fault state";
    case MRC_ERROR_MOTION_MODE_REJECTED: return "Motion mode rejected";
    case MRC_ERROR_MOTION_NOT_ENABLED: return "Motor not enabled";
    case MRC_ERROR_MOTION_QUICK_STOP: return "Motor in quick stop";
    case MRC_ERROR_MOTION_TARGET_TIMEOUT: return "Target position/velocity timeout";
    default:                        return "Unknown error code";
    }
}
