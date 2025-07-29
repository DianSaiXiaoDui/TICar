#include "CCD.h"
#include "CCD_ex.h"
#include "ti/driverlib/m0p/dl_core.h"
#include "ti_msp_dl_config.h"

extern uint16_t CCD_ADV_Origin[128];
extern uint16_t CCD_ADV_Filtered[128];
extern int16_t DxMax;
extern int16_t DxMin;
extern int16_t dX[128];
extern uint16_t MaxIdx;
extern uint16_t MinIdx;
extern int16_t CCD_TargetIdx;


//CCD原始像素序列中值滤波
void CCD_MedianFilter()
{
    // 对每个点计算左、中、右三个值的中位数并保存到 CCD_ADV_Filtered
    for (int j = 0; j < 128; j++)
    {
        int left_val, current_val, right_val;

        // 处理边界条件
        left_val = (j == 0) ? CCD_ADV_Origin[j] : CCD_ADV_Origin[j - 1];  // 左边值（j=0时取当前值）
        current_val = CCD_ADV_Origin[j];                       // 当前值
        right_val = (j == 127) ? CCD_ADV_Origin[j] : CCD_ADV_Origin[j + 1]; // 右边值（j=127时取当前值）

        // 计算三个值的中位数
        int a = left_val;
        int b = current_val;
        int c = right_val;

        // 手动计算最小值和最大值
        int min_val = a;
        if (b < min_val) min_val = b;
        if (c < min_val) min_val = c;

        int max_val = a;
        if (b > max_val) max_val = b;
        if (c > max_val) max_val = c;

        // 中位数 = 总和 - 最小值 - 最大值
        CCD_ADV_Filtered[j] = a + b + c - min_val - max_val;
    }
}

//CCD数据处理：中值滤波计算黑线中心
void CCD_DataProcess(void)
{
   uint8_t j;
   DxMax=0;
   DxMin=0;

    CCD_MedianFilter();


    for (j = 0; j < 125; j++)
    {
        dX[j] = CCD_ADV_Filtered[j] - CCD_ADV_Filtered[j + 3]; // 使用过滤后的数据，j+3 最大为 127（当 j=124）

        if (DxMin > dX[j])
        {
            DxMin = dX[j];
            MinIdx = j;
        }
        if (DxMax < dX[j])
        {
            DxMax = dX[j];
            MaxIdx = j;
        }
    }

   if (MinIdx-MaxIdx>10)
     CCD_TargetIdx = (MaxIdx+MinIdx)>>1;
   else
      CCD_TargetIdx=-1;//没看到黑线



}