#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <pthread.h>

#include <nuttx/gpio.h>
#include <arch/tsb/cdsi.h>
#include <arch/tsb/gpio.h>
#include <arch/board/cdsi1_offs_def.h>
#include <arch/board/ov5645.h>

#define CDSIRX_CLKEN_VAL                                0x00000001
#define CDSIRX_CLKSEL_VAL                               0x00000101
#define CDSIRX_MODE_CONFIG_VAL                          0x00000001
#define CDSIRX_LANE_ENABLE_VAL                          0x00000012
#define CDSIRX_VC_ENABLE_VAL                            0x0000000F
#define CDSIRX_LINE_INIT_COUNT_VAL                      0x000012C0
#define CDSIRX_HSRXTO_COUNT_VAL                         0xFFFFFFFF
#define CDSIRX_FUNC_ENABLE_VAL                          0x00070701
#define CDSIRX_DSI_LPTX_MODE_VAL                        0x00000001
#define CDSIRX_DSI_TATO_COUNT_VAL                       0xFFFFFFFF
#define CDSIRX_DSI_LPTXTO_COUNT_VAL                     0xFFFFFFFF
#define CDSIRX_FUNC_MODE_VAL                            0x00000000
#define CDSIRX_PPI_HSRX_CNTRL_VAL                       0x40000000
#define CDSIRX_PPI_HSRX_COUNT_VAL                       0x0400000A
#define CDSIRX_PPI_DPHY_DLYCNTRL_VAL                    0x00000000
#define CDSIRX_PPI_DPHY_LPRX_THSLD_VAL                  0x000002AA
#define CDSIRX_PPI_DPHY_LPTXTIMECNT_VAL                 0x00000FFF
#define CDSIRX_PPI_DSI_BTA_COUNT_VAL                    0x000407FF
#define CDSIRX_PPI_DSI_DPHYTX_ADJUST_VAL                0x00000002
#define CDSIRX_PPI_DPHY_HSRX_ADJUST_VAL                 0x000002AA
#define CDSIRX_PPI_DPHY_LPRXCALCNTRL_VAL                0x00190040
#define CDSIRX_LPRX_STATE_INT_MASK_VAL                  0x1F1F1F1D

#define AL_RX_BRG_MODE_VAL                              0x00000003
#define AL_RX_BRG_CSI_INFO_VAL                          0x00000000
#define AL_RX_BRG_CSI_DT0_VAL                           0x00000000
#define AL_RX_BRG_CSI_DT1_VAL                           0x00000000
#define AL_RX_BRG_CSI_DT2_VAL                           0x00000000
#define AL_RX_BRG_CSI_DT3_VAL                           0x00000000

static pthread_t g_camera_thread;

void ov5645_csi_init(struct cdsi_dev *dev)
{
    uint32_t rdata0;
    uint32_t rdata1;

    printf("ov5645_csi_init callback function for CSI-2 rx\n");

    cdsi_write(dev, CDSI1_AL_RX_BRG_MODE_OFFS, AL_RX_BRG_MODE_VAL);
    cdsi_write(dev, CDSI1_AL_RX_BRG_CSI_INFO_OFFS, AL_RX_BRG_CSI_INFO_VAL);
    cdsi_write(dev, CDSI1_AL_RX_BRG_CSI_DT0_OFFS, AL_RX_BRG_CSI_DT0_VAL);
    cdsi_write(dev, CDSI1_AL_RX_BRG_CSI_DT1_OFFS, AL_RX_BRG_CSI_DT1_VAL);
    cdsi_write(dev, CDSI1_AL_RX_BRG_CSI_DT2_OFFS, AL_RX_BRG_CSI_DT2_VAL);
    cdsi_write(dev, CDSI1_AL_RX_BRG_CSI_DT3_OFFS, AL_RX_BRG_CSI_DT3_VAL);

    cdsi_write(dev, CDSI1_CDSIRX_CLKEN_OFFS, CDSIRX_CLKEN_VAL);
    cdsi_write(dev, CDSI1_CDSIRX_FUNC_ENABLE_OFFS, CDSIRX_FUNC_ENABLE_VAL);
    cdsi_write(dev, CDSI1_CDSIRX_PPI_DPHY_LPRXCALCNTRL_OFFS,
               CDSIRX_PPI_DPHY_LPRXCALCNTRL_VAL);
    cdsi_write(dev, CDSI1_CDSIRX_PPI_DPHY_LPRX_THSLD_OFFS,
               CDSIRX_PPI_DPHY_LPRX_THSLD_VAL);
    cdsi_write(dev, CDSI1_CDSIRX_PPI_DPHY_LPRXAUTOCALST_OFFS, 0x00000001);
    cdsi_write(dev, CDSI1_CDSIRX_CLKSEL_OFFS, CDSIRX_CLKSEL_VAL);
    cdsi_write(dev, CDSI1_CDSIRX_MODE_CONFIG_OFFS, CDSIRX_MODE_CONFIG_VAL);
    cdsi_write(dev, CDSI1_CDSIRX_LANE_ENABLE_OFFS, CDSIRX_LANE_ENABLE_VAL);
    cdsi_write(dev, CDSI1_CDSIRX_VC_ENABLE_OFFS, CDSIRX_VC_ENABLE_VAL);
    cdsi_write(dev, CDSI1_CDSIRX_LINE_INIT_COUNT_OFFS,
               CDSIRX_LINE_INIT_COUNT_VAL);
    cdsi_write(dev, CDSI1_CDSIRX_HSRXTO_COUNT_OFFS, CDSIRX_HSRXTO_COUNT_VAL);
    cdsi_write(dev, CDSI1_CDSIRX_FUNC_MODE_OFFS, CDSIRX_FUNC_MODE_VAL);
    cdsi_write(dev, CDSI1_CDSIRX_PPI_DPHY_LPTXTIMECNT_OFFS,
               CDSIRX_PPI_DPHY_LPTXTIMECNT_VAL);
    cdsi_write(dev, CDSI1_CDSIRX_DSI_LPTX_MODE_OFFS,
               CDSIRX_DSI_LPTX_MODE_VAL);
    cdsi_write(dev, CDSI1_CDSIRX_PPI_DSI_BTA_COUNT_OFFS,
               CDSIRX_PPI_DSI_BTA_COUNT_VAL);
    cdsi_write(dev, CDSI1_CDSIRX_PPI_HSRX_CNTRL_OFFS,
               CDSIRX_PPI_HSRX_CNTRL_VAL);
    cdsi_write(dev, CDSI1_CDSIRX_PPI_HSRX_COUNT_OFFS,
               CDSIRX_PPI_HSRX_COUNT_VAL);
    cdsi_write(dev, CDSI1_CDSIRX_PPI_DPHY_POWERCNTRL_OFFS, 0x00000003);
    cdsi_write(dev, CDSI1_CDSIRX_PPI_DSI_DPHYTX_ADJUST_OFFS,
               CDSIRX_PPI_DSI_DPHYTX_ADJUST_VAL);
    cdsi_write(dev, CDSI1_CDSIRX_PPI_DPHY_HSRX_ADJUST_OFFS,
               CDSIRX_PPI_DPHY_HSRX_ADJUST_VAL);
    cdsi_write(dev, CDSI1_CDSIRX_PPI_DPHY_DLYCNTRL_OFFS,
               CDSIRX_PPI_DPHY_DLYCNTRL_VAL);
    cdsi_write(dev, CDSI1_CDSIRX_LPRX_STATE_INT_MASK_OFFS,
               CDSIRX_LPRX_STATE_INT_MASK_VAL);
    cdsi_write(dev, CDSI1_CDSIRX_DSI_RXTRIG_INT_MASK_OFFS, 0x00000000);
    cdsi_write(dev, CDSI1_CDSIRX_RXERR_INT_MASK_OFFS, 0x00000000);
    cdsi_write(dev, CDSI1_CDSIRX_TXERR_INT_MASK_OFFS, 0x00000000);
    cdsi_write(dev, CDSI1_CDSIRX_DSI_VC0_SH_INT_MASK_OFFS, 0x00000000);
    cdsi_write(dev, CDSI1_CDSIRX_DSI_VC1_SH_INT_MASK_OFFS, 0x00000000);
    cdsi_write(dev, CDSI1_CDSIRX_DSI_VC2_SH_INT_MASK_OFFS, 0x00000000);
    cdsi_write(dev, CDSI1_CDSIRX_DSI_VC3_SH_INT_MASK_OFFS, 0x00000000);
    cdsi_write(dev, CDSI1_CDSIRX_DSI_VC0_LN_INT_MASK_OFFS, 0x00000000);
    cdsi_write(dev, CDSI1_CDSIRX_DSI_VC1_LN_INT_MASK_OFFS, 0x00000000);
    cdsi_write(dev, CDSI1_CDSIRX_DSI_VC2_LN_INT_MASK_OFFS, 0x00000000);
    cdsi_write(dev, CDSI1_CDSIRX_DSI_VC3_LN_INT_MASK_OFFS, 0x00000000);
    cdsi_write(dev, CDSI1_CDSIRX_CSI2_VC0_SH_INT_MASK_OFFS, 0x00000000);
    cdsi_write(dev, CDSI1_CDSIRX_CSI2_VC1_SH_INT_MASK_OFFS, 0x00000000);
    cdsi_write(dev, CDSI1_CDSIRX_CSI2_VC2_SH_INT_MASK_OFFS, 0x00000000);
    cdsi_write(dev, CDSI1_CDSIRX_CSI2_VC3_SH_INT_MASK_OFFS, 0x00000000);
    cdsi_write(dev, CDSI1_CDSIRX_CSI2_VC0_LN0_INT_MASK_OFFS, 0x00000000);
    cdsi_write(dev, CDSI1_CDSIRX_CSI2_VC0_LN1_INT_MASK_OFFS, 0x00000000);
    cdsi_write(dev, CDSI1_CDSIRX_CSI2_VC1_LN0_INT_MASK_OFFS, 0x00000000);
    cdsi_write(dev, CDSI1_CDSIRX_CSI2_VC1_LN1_INT_MASK_OFFS, 0x00000000);
    cdsi_write(dev, CDSI1_CDSIRX_CSI2_VC2_LN0_INT_MASK_OFFS, 0x00000000);
    cdsi_write(dev, CDSI1_CDSIRX_CSI2_VC2_LN1_INT_MASK_OFFS, 0x00000000);
    cdsi_write(dev, CDSI1_CDSIRX_CSI2_VC3_LN0_INT_MASK_OFFS, 0x00000000);
    cdsi_write(dev, CDSI1_CDSIRX_CSI2_VC3_LN1_INT_MASK_OFFS, 0x00000000);
    cdsi_write(dev, CDSI1_CDSIRX_DSI_LPTXTO_COUNT_OFFS,
               CDSIRX_DSI_LPTXTO_COUNT_VAL);
    cdsi_write(dev, CDSI1_CDSIRX_DSI_TATO_COUNT_OFFS,
               CDSIRX_DSI_TATO_COUNT_VAL);
    cdsi_write(dev, CDSI1_CDSIRX_DSI_WAITBTA_COUNT_OFFS, 0x10000010);

    rdata0 = cdsi_read(dev, CDSI1_CDSIRX_LPRX_STATE_INT_STAT_OFFS);
    while ((rdata0 & 0x00000010) == 0x00000000) {
        rdata0 = cdsi_read(dev, CDSI1_CDSIRX_LPRX_STATE_INT_STAT_OFFS);
        //printf("%s: line = %d, rdata0 = %d\n", __func__, __LINE__, rdata0);
    }
    printf("First LPRX_STATE_INT: %d\n", rdata0);
    cdsi_write(dev, CDSI1_CDSIRX_START_OFFS, 0x00000001);

    rdata1 = cdsi_read(dev, CDSI1_CDSIRX_LPRX_STATE_INT_STAT_OFFS);
    ov5645_init(0);
    while ((rdata1 & 0x00000001) != 0x00000001) {
        rdata1 = cdsi_read(dev, CDSI1_CDSIRX_LPRX_STATE_INT_STAT_OFFS);
        //printf("%s: line = %d, rdata1 = %d\n", __func__, __LINE__, rdata1);
    }
    printf("Second LPRX_STATE_INT: %d\n", rdata1);

    cdsi_write(dev, CDSI1_CDSIRX_LPRX_STATE_INT_STAT_OFFS, 0x00000001);
    cdsi_write(dev, CDSI1_CDSIRX_DSI_LPTX_MODE_OFFS, CDSIRX_DSI_LPTX_MODE_VAL);
    cdsi_write(dev, CDSI1_CDSIRX_ADDRESS_CONFIG_OFFS, 0x00000000);
}

struct camera_sensor ov5645_sensor = {
    .cdsi_sensor_init = ov5645_csi_init,
};

static void *camera_fn(void *p_data)
{
    //ov5645_init(0);
    csi_initialize(&ov5645_sensor, TSB_CDSI1, TSB_CDSI_RX);
    return NULL;
}

int camera_init(void)
{
    return pthread_create(&g_camera_thread, NULL, camera_fn, NULL);
}
