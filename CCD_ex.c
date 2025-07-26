#include "CCD.h"
#include "CCD_ex.h"
#include "ti/driverlib/m0p/dl_core.h"
#include "ti_msp_dl_config.h"

#define CCD_FILTER_WINDOW 3

// 窗口为5的均值滤波
// 计算方式：对每个像素点，取其前后各2个像素点的值（共5个），求平均值
// 适用于去除CCD数据中的噪声，平滑图像
// 只用 ccd_filtered[2] ~ ccd_filtered[125]，前2和后2个点不参与黑线识别。
void CCD_MeanFilter(void)
{
    uint32_t sum = 0;
    uint8_t half = CCD_FILTER_WINDOW / 2;
    for(uint8_t i = 0; i < 128; i++) {
        sum = 0;
        uint8_t count = 0;
        for(int8_t j = -half; j <= half; j++) {
            int16_t idx = i + j;
            if(idx >= 0 && idx < 128) {
                sum += ccd_result[idx];
                count++;
            }
        }
        ccd_filtered[i] = (uint16_t)(sum / count);
    }
}

// 黑线中心查找算法（差分法）
// 返回黑线中心像素位置，404表示未找到
void CCD_FindBlackLine(void)
{
    int16_t dX[128];
    int16_t DxMax = -32768;
    int16_t DxMin = 32767;
    uint8_t MaxIdx = 0, MinIdx = 0;
    // 前和后各1个点不参与计算
    for(uint8_t j = 1; j < 123; j++) {
        dX[j] = ccd_filtered[j] - ccd_filtered[j + 3];
        if(dX[j] < DxMin) {
            DxMin = dX[j];
            MinIdx = j;
        }
        if(dX[j] > DxMax) {
            DxMax = dX[j];
            MaxIdx = j;
        }
    }
    // 黑线中心判定
    if(MaxIdx < MinIdx)
        TargetIdx = (MaxIdx + MinIdx) >> 1;
    else
        TargetIdx = 404; // 未找到有效黑线
}

