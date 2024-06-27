#ifndef CAMERA_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define CAMERA_H

#define PIN_MOUSECAM_CS RPI_BPLUS_GPIO_J8_29
#define PIN1_MOUSECAM_CS RPI_BPLUS_GPIO_J8_31
#define PIN2_MOUSECAM_CS RPI_BPLUS_GPIO_J8_33
#define PIN3_MOUSECAM_CS RPI_BPLUS_GPIO_J8_35
#define PIN4_MOUSECAM_CS RPI_BPLUS_GPIO_J8_03
#define PIN5_MOUSECAM_CS RPI_BPLUS_GPIO_J8_05
#define PIN6_MOUSECAM_CS RPI_BPLUS_GPIO_J8_07
#define PIN7_MOUSECAM_CS RPI_BPLUS_GPIO_J8_12

#define electromagnet_1 RPI_BPLUS_GPIO_J8_32
#define electromagnet_2 RPI_BPLUS_GPIO_J8_36
#define electromagnet_3 RPI_BPLUS_GPIO_J8_38
#define electromagnet_4 RPI_BPLUS_GPIO_J8_40
#define electromagnet_5 RPI_BPLUS_GPIO_J8_37
#define electromagnet_6 RPI_BPLUS_GPIO_J8_11
#define electromagnet_7 RPI_BPLUS_GPIO_J8_13
#define electromagnet_8 RPI_BPLUS_GPIO_J8_15



#define ADNS3080_PIXELS_X                 30
#define ADNS3080_PIXELS_Y                 30

#define ADNS3080_PRODUCT_ID            0x00
#define ADNS3080_REVISION_ID           0x01
#define ADNS3080_MOTION                0x02
#define ADNS3080_DELTA_X               0x03
#define ADNS3080_DELTA_Y               0x04
#define ADNS3080_SQUAL                 0x05
#define ADNS3080_PIXEL_SUM             0x06
#define ADNS3080_MAXIMUM_PIXEL         0x07
#define ADNS3080_CONFIGURATION_BITS    0x0a
#define ADNS3080_EXTENDED_CONFIG       0x0b
#define ADNS3080_DATA_OUT_LOWER        0x0c
#define ADNS3080_DATA_OUT_UPPER        0x0d
#define ADNS3080_SHUTTER_LOWER         0x0e
#define ADNS3080_SHUTTER_UPPER         0x0f
#define ADNS3080_FRAME_PERIOD_LOWER    0x10
#define ADNS3080_FRAME_PERIOD_UPPER    0x11
#define ADNS3080_MOTION_CLEAR          0x12
#define ADNS3080_FRAME_CAPTURE         0x13
#define ADNS3080_SROM_ENABLE           0x14
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER      0x19
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER      0x1a
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_LOWER      0x1b
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_UPPER      0x1c
#define ADNS3080_SHUTTER_MAX_BOUND_LOWER           0x1e
#define ADNS3080_SHUTTER_MAX_BOUND_UPPER           0x1e
#define ADNS3080_SROM_ID               0x1f
#define ADNS3080_OBSERVATION           0x3d
#define ADNS3080_INVERSE_PRODUCT_ID    0x3f
#define ADNS3080_PIXEL_BURST           0x40
#define ADNS3080_MOTION_BURST          0x50
#define ADNS3080_SROM_LOAD             0x60

#define ADNS3080_PRODUCT_ID_VAL        0x17

#include <stdint.h>

struct MD
{
 int motion;
 signed char  dx, dy;
 int squal;
 int shutter;
 uint8_t max_pix;
};

struct MD ma;
struct MD mb;
struct MD mc;
struct MD md;
struct MD me;
struct MD mf;
struct MD mg;
struct MD mh;


void mousecam_reset();
int mousecam_read_reg(int reg);
void mousecam_write_reg(int reg, int val);
int mousecam_init();
void mousecam_read_motion(struct MD *p);

void mousecam1_reset();
int mousecam1_read_reg(int reg);
void mousecam1_write_reg(int reg, int val);
int mousecam1_init();
void mousecam1_read_motion(struct MD *p);

void mousecam2_reset();
int mousecam2_read_reg(int reg);
void mousecam2_write_reg(int reg, int val);
int mousecam2_init();
void mousecam2_read_motion(struct MD *p);

void mousecam3_reset();
int mousecam3_read_reg(int reg);
void mousecam3_write_reg(int reg, int val);
int mousecam3_init();
void mousecam3_read_motion(struct MD *p);

void mousecam4_reset();
int mousecam4_read_reg(int reg);
void mousecam4_write_reg(int reg, int val);
int mousecam4_init();
void mousecam4_read_motion(struct MD *p);

void mousecam5_reset();
int mousecam5_read_reg(int reg);
void mousecam5_write_reg(int reg, int val);
int mousecam5_init();
void mousecam5_read_motion(struct MD *p);

void mousecam6_reset();
int mousecam6_read_reg(int reg);
void mousecam6_write_reg(int reg, int val);
int mousecam6_init();
void mousecam6_read_motion(struct MD *p);

void mousecam7_reset();
int mousecam7_read_reg(int reg);
void mousecam7_write_reg(int reg, int val);
int mousecam7_init();
void mousecam7_read_motion(struct MD *p);



void setup();
void loop();

#endif
