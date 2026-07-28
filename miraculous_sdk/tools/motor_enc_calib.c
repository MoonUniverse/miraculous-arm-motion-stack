/**
 * @file    motor_enc_calib.c
 * @brief   编码器校准参数工具 — 读取/保存/写入 0x2007 + 0x2008
 *
 * OD 0x2007: 编码器零点位置 (int32_t[11], sub1~11)
 * OD 0x2008: 编码器零点 Ki 值 (float[11], sub1~11)
 * sub0 = 有效极对数 + 1
 *
 * 文件格式 (文本, 每行一个值):
 *   poles=11
 *   zero_point[0]=12345
 *   zero_point[1]=...
 *   zero_ki[0]=0.001234
 *   zero_ki[1]=...
 *
 * 用法:
 *   ./motor_enc_calib [can_if] [node_id]              # 读取并显示
 *   ./motor_enc_calib [can_if] [node_id] save <file>  # 读取并保存到文件
 *   ./motor_enc_calib [can_if] [node_id] load <file>  # 从文件加载并写入电机
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "miraculous_sdk.h"

#define MAX_POLES 12

static int read_from_motor(MiraMotor *motor,
                           int *poles, int32_t *zero_point, float *zero_ki)
{
    /* 读取极对数 (sub0) */
    uint8_t poles_sub0 = 0;
    uint8_t len = 1;
    int ret = miraculous_motor_sdo_read(motor, CIA402_OD_ENC_ZERO_POINT, 0,
                                         &poles_sub0, &len);
    if (ret < 0) {
        fprintf(stderr, "Failed to read poles count: %s\n", mrc_strerror(ret));
        return ret;
    }
    *poles = (int)poles_sub0 - 1;
    if (*poles < 1 || *poles > MAX_POLES) {
        fprintf(stderr, "Invalid poles count: %d (sub0=%d)\n", *poles, poles_sub0);
        return MRC_ERROR_INVALID_PARAM;
    }

    /* 读取零点位置 0x2007 sub1~poles */
    for (int i = 0; i <= *poles; i++) {
        len = 4;
        ret = miraculous_motor_sdo_read(motor, CIA402_OD_ENC_ZERO_POINT,
                                         (uint8_t)(i + 1),
                                         &zero_point[i], &len);
        if (ret < 0) {
            fprintf(stderr, "Failed to read zero_point[%d]: %s\n",
                    i, mrc_strerror(ret));
            return ret;
        }
    }

    /* 读取零点 Ki 值 0x2008 sub1~poles */
    for (int i = 0; i <= *poles; i++) {
        len = 4;
        ret = miraculous_motor_sdo_read(motor, CIA402_OD_ENC_ZERO_KI,
                                         (uint8_t)(i + 1),
                                         &zero_ki[i], &len);
        if (ret < 0) {
            fprintf(stderr, "Failed to read zero_ki[%d]: %s\n",
                    i, mrc_strerror(ret));
            return ret;
        }
    }
    return MRC_SUCCESS;
}

static int write_to_motor(MiraMotor *motor,
                          int poles, int32_t *zero_point, float *zero_ki)
{
    int ret;

    /* 写入零点位置 0x2007 sub1~poles */
    for (int i = 0; i <= poles; i++) {
        ret = miraculous_motor_sdo_write(motor, CIA402_OD_ENC_ZERO_POINT,
                                          (uint8_t)(i + 1),
                                          &zero_point[i], 4);
        if (ret < 0) {
            fprintf(stderr, "Failed to write zero_point[%d]: %s\n",
                    i, mrc_strerror(ret));
            return ret;
        }
    }

    /* 写入零点 Ki 值 0x2008 sub1~poles */
    for (int i = 0; i <= poles; i++) {
        ret = miraculous_motor_sdo_write(motor, CIA402_OD_ENC_ZERO_KI,
                                          (uint8_t)(i + 1),
                                          &zero_ki[i], 4);
        if (ret < 0) {
            fprintf(stderr, "Failed to write zero_ki[%d]: %s\n",
                    i, mrc_strerror(ret));
            return ret;
        }
    }

    return MRC_SUCCESS;
}

static int store_to_persist(MiraMotor *motor)
{
    return miraculous_motor_save_config(motor);
}

static int save_to_file(const char *filename, int poles,
                         int32_t *zero_point, float *zero_ki)
{
    FILE *fp = fopen(filename, "w");
    if (!fp) {
        perror("fopen");
        return MRC_ERROR_INVALID_PARAM;
    }

    fprintf(fp, "poles=%d\n", poles);
    for (int i = 0; i <= poles; i++)
        fprintf(fp, "zero_point[%d]=%d\n", i, zero_point[i]);
    for (int i = 0; i <= poles; i++) {
        uint32_t raw;
        memcpy(&raw, &zero_ki[i], 4);
        fprintf(fp, "zero_ki[%d]=0x%08X  (%g)\n", i, raw, zero_ki[i]);
    }

    fclose(fp);
    printf("Saved to %s (%d poles)\n", filename, poles);
    return MRC_SUCCESS;
}

static int load_from_file(const char *filename, int *poles,
                           int32_t *zero_point, float *zero_ki)
{
    FILE *fp = fopen(filename, "r");
    if (!fp) {
        perror("fopen");
        return MRC_ERROR_INVALID_PARAM;
    }

    char line[128];
    int loaded_poles = 0;
    int pt_count = 0, ki_count = 0;
    uint32_t raw_ki[MAX_POLES] = {0};

    while (fgets(line, sizeof(line), fp)) {
        /* 去掉末尾换行 */
        size_t len = strlen(line);
        if (len > 0 && line[len - 1] == '\n') line[len - 1] = '\0';

        if (sscanf(line, "poles=%d", &loaded_poles) == 1) {
            if (loaded_poles < 1 || loaded_poles > MAX_POLES) {
                fprintf(stderr, "Invalid poles=%d in file\n", loaded_poles);
                fclose(fp);
                return MRC_ERROR_INVALID_PARAM;
            }
            *poles = loaded_poles;
        } else if (sscanf(line, "zero_point[%*d]=%d", &zero_point[pt_count]) == 1) {
            pt_count++;
        } else if (sscanf(line, "zero_ki[%*d]=0x%x", &raw_ki[ki_count]) == 1) {
            ki_count++;
        }
    }
    fclose(fp);

    if (*poles < 1 || pt_count < *poles || ki_count < *poles) {
        fprintf(stderr, "Incomplete data in %s: poles=%d pts=%d kis=%d\n",
                filename, *poles, pt_count, ki_count);
        return MRC_ERROR_INVALID_PARAM;
    }

    /* 将 uint32 原始字节转为 float */
    for (int i = 0; i <= *poles; i++)
        memcpy(&zero_ki[i], &raw_ki[i], 4);

    printf("Loaded from %s (%d poles)\n", filename, *poles);
    return MRC_SUCCESS;
}

static void print_data(int poles, int32_t *zero_point, float *zero_ki)
{
    printf("Poles: %d\n\n", poles);
    printf("  # | zero_point    | zero_ki (hex)    | zero_ki\n");
    printf("----|---------------|------------------|------------\n");
    for (int i = 0; i <= poles; i++) {
        uint32_t raw;
        memcpy(&raw, &zero_ki[i], 4);
        printf("  %2d | %11d | 0x%08X | %.6f\n",
               i, zero_point[i], raw, zero_ki[i]);
    }
    printf("\n");
}

int main(int argc, char **argv)
{
    if (argc < 3) {
        fprintf(stderr, "Usage: %s <can_if> <node_id> [save|load <file>]\n", argv[0]);
        return -1;
    }

    const char *ifname = argv[1];
    int node_id = atoi(argv[2]);
    const char *cmd = (argc >= 4) ? argv[3] : "show";
    const char *filename = (argc >= 5) ? argv[4] : "enc_calib.txt";

    MiraMotor *motor = miraculous_motor_open(ifname, 0, node_id);
    if (!motor) { fprintf(stderr, "Failed to open motor\n"); return -1; }

    if (miraculous_motor_bootstrap(motor, 3000) < 0) {
        fprintf(stderr, "Motor not responding!\n");
        goto cleanup;
    }

    if (strcmp(cmd, "save") == 0) {
        /* 从电机读取 → 保存到文件 */
        int poles;
        int32_t zero_point[MAX_POLES];
        float zero_ki[MAX_POLES];

        if (read_from_motor(motor, &poles, zero_point, zero_ki) < 0)
            goto cleanup;
        print_data(poles, zero_point, zero_ki);
        save_to_file(filename, poles, zero_point, zero_ki);

    } else if (strcmp(cmd, "load") == 0) {
        /* 从文件读取 → 写入电机 */
        int poles;
        int32_t zero_point[MAX_POLES];
        float zero_ki[MAX_POLES];

        if (load_from_file(filename, &poles, zero_point, zero_ki) < 0)
            goto cleanup;
        print_data(poles, zero_point, zero_ki);

        printf("Writing to node %d...\n", node_id);
        if (write_to_motor(motor, poles, zero_point, zero_ki) < 0)
            goto cleanup;
        store_to_persist(motor);
        printf("Waiting 3s for storage to complete...\n");
        sleep(3);
        printf("Write complete!\n");

    } else {
        /* 只显示 */
        int poles;
        int32_t zero_point[MAX_POLES];
        float zero_ki[MAX_POLES];

        if (read_from_motor(motor, &poles, zero_point, zero_ki) < 0)
            goto cleanup;
        printf("=== Encoder Calibration (Node %d) ===\n", node_id);
        print_data(poles, zero_point, zero_ki);
        printf("=== End ===\n");
    }

cleanup:
    miraculous_motor_close(motor);
    return 0;
}
