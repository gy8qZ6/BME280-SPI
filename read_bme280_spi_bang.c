#include <bcm2835.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>

#include <time.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "bang_spi.h"


#define LOG_FILE "/home/pi/driver_dev/SPI/BME280.log"

/*
void bce280_write(uint8_t *out, uint8_t *in, uint8_t len)
{
    uint8_t outbuf[2] = { 0xd0 | 1 << 7, 0x00};
    uint8_t inbuf[2] = { 0x00, 0x00 };

    printf("writing: 0x%02x 0x%02x\n", outbuf[0], outbuf[1]);
    bcm2835_spi_transfernb(outbuf, inbuf, 2);
    printf("read: 0x%02x 0x%02x\n", inbuf[0], inbuf[1]);
}
*/

// TODO: check the correctnes of the pins again
RPiGPIOPin pwr = RPI_V2_GPIO_P1_37;
RPiGPIOPin sck = RPI_V2_GPIO_P1_35; 
RPiGPIOPin mosi = RPI_V2_GPIO_P1_33;
RPiGPIOPin cs = RPI_V2_GPIO_P1_31;  
RPiGPIOPin miso = RPI_V2_GPIO_P1_29;
/*
RPiGPIOPin sck = RPI_GPIO_P1_23;  //!< Version 1, Pin P1-23, CLK when SPI0 in use
RPiGPIOPin cs = RPI_GPIO_P1_24;   //!< Version 1, Pin P1-24, CE0 when SPI0 in use
RPiGPIOPin mosi = RPI_GPIO_P1_19; //!< Version 1, Pin P1-19, MOSI when SPI0 in use
RPiGPIOPin miso = RPI_GPIO_P1_21; //!< Version 1, Pin P1-21, MISO when SPI0 in use 
*/

#define DEBUG 0

#define WRITE(x) x & 0xff >> 1
#define READ(x) x | 1 << 7
#define MAX_BUFFER_LEN 9

#define BCE280_REG_STATUS 0xF3
#define BCE280_REG_CTRL_HUM 0xF2
#define BCE280_REG_CTRL_MEAS 0xF4
#define BCE280_REG_ID 0xD0
#define BCE280_CTRL_MEAS_CONF 0xB5
#define BCE280_SENSOR_CONF 0x05         // 101
#define BCE280_HUM_CONF_MASK 0x07       // 0111
#define BCE280_REG_MEAS_VALUES 0xF7

/* BME280 stuff stolen from BOSCH BME280 API github */
#define BME280_CONCAT_BYTES(msb,lsb) (uint16_t)msb << 8 | (uint16_t)lsb
static int32_t t_fine;

struct bme280_calib_data
{
    uint16_t dig_t1;
    int16_t dig_t2;
    int16_t dig_t3;
    uint16_t dig_p1;
    int16_t dig_p2;
    int16_t dig_p3;
    int16_t dig_p4;
    int16_t dig_p5;
    int16_t dig_p6;
    int16_t dig_p7;
    int16_t dig_p8;
    int16_t dig_p9;

    uint8_t dig_h1;
    int16_t dig_h2;
    uint8_t dig_h3;
    int16_t dig_h4;
    int16_t dig_h5;
    int8_t dig_h6;
    int32_t t_fine;
};

static struct bme280_calib_data calib_data;

/*!
 *  @brief This internal API is used to parse the temperature and
 *  pressure calibration data and store it in device structure.
 */
static void parse_temp_press_calib_data(const uint8_t *reg_data)
{
    //struct bme280_calib_data *calib_data = &dev->calib_data;

    calib_data.dig_t1 = BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
    calib_data.dig_t2 = (int16_t)BME280_CONCAT_BYTES(reg_data[3], reg_data[2]);
    calib_data.dig_t3 = (int16_t)BME280_CONCAT_BYTES(reg_data[5], reg_data[4]);
    calib_data.dig_p1 = BME280_CONCAT_BYTES(reg_data[7], reg_data[6]);
    calib_data.dig_p2 = (int16_t)BME280_CONCAT_BYTES(reg_data[9], reg_data[8]);
    calib_data.dig_p3 = (int16_t)BME280_CONCAT_BYTES(reg_data[11], reg_data[10]);
    calib_data.dig_p4 = (int16_t)BME280_CONCAT_BYTES(reg_data[13], reg_data[12]);
    calib_data.dig_p5 = (int16_t)BME280_CONCAT_BYTES(reg_data[15], reg_data[14]);
    calib_data.dig_p6 = (int16_t)BME280_CONCAT_BYTES(reg_data[17], reg_data[16]);
    calib_data.dig_p7 = (int16_t)BME280_CONCAT_BYTES(reg_data[19], reg_data[18]);
    calib_data.dig_p8 = (int16_t)BME280_CONCAT_BYTES(reg_data[21], reg_data[20]);
    calib_data.dig_p9 = (int16_t)BME280_CONCAT_BYTES(reg_data[23], reg_data[22]);
    calib_data.dig_h1 = reg_data[25];
}
static void parse_humidity_calib_data(const uint8_t *reg_data)
{
    int16_t dig_h4_lsb;
    int16_t dig_h4_msb;
    int16_t dig_h5_lsb;
    int16_t dig_h5_msb;

    calib_data.dig_h2 = (int16_t)BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
    calib_data.dig_h3 = reg_data[2];
    dig_h4_msb = (int16_t)(int8_t)reg_data[3] * 16;
    dig_h4_lsb = (int16_t)(reg_data[4] & 0x0F);
    calib_data.dig_h4 = dig_h4_msb | dig_h4_lsb;
    dig_h5_msb = (int16_t)(int8_t)reg_data[5] * 16;
    dig_h5_lsb = (int16_t)(reg_data[4] >> 4);
    calib_data.dig_h5 = dig_h5_msb | dig_h5_lsb;
    calib_data.dig_h6 = (int8_t)reg_data[6];
}
/* new
*/
void bce280_calibration_data_read()
{
    uint8_t cmd1[27];
    uint8_t buf1[27]; // calibzone1: 0x88 - 0xA1 = 26, 0xA0 is not needed!!
    uint8_t cmd2[8];
    uint8_t buf2[8]; // calibzone2: 0xE1 - 0xE7 = 7
    uint8_t bufall[32];

    cmd1[0] = READ(0x88);
    //bcm2835_spi_transfern(buf1, 27);
    bang_spi_transfer_n(cmd1, buf1, 27);
    
    parse_temp_press_calib_data(&buf1[1]);

    cmd2[0] = READ(0xE1);
    //bcm2835_spi_transfern(buf2, 8);
    bang_spi_transfer_n(cmd2, buf2, 8);
    
    parse_humidity_calib_data(&buf2[1]);

    if (DEBUG)
    {
        printf("\ndig_t1: %u 0x%04x\n", calib_data.dig_t1, calib_data.dig_t1);
        printf("dig_t2: %i 0x%04x\n", calib_data.dig_t2, calib_data.dig_t2);
        printf("dig_t3: %i 0x%04x\n", calib_data.dig_t3, calib_data.dig_t3);
        printf("dig_p1: %u 0x%04x\n", calib_data.dig_p1, calib_data.dig_p1);
        printf("dig_p2: %i 0x%04x\n", calib_data.dig_p2, calib_data.dig_p2);
        printf("dig_p3: %i 0x%04x\n", calib_data.dig_p3, calib_data.dig_p3);
        printf("dig_p4: %i 0x%04x\n", calib_data.dig_p4, calib_data.dig_p4);
        printf("dig_p5: %i 0x%04x\n", calib_data.dig_p5, calib_data.dig_p5);
        printf("dig_p6: %i 0x%04x\n", calib_data.dig_p6, calib_data.dig_p6);
        printf("dig_p7: %i 0x%04x\n", calib_data.dig_p7, calib_data.dig_p7);
        printf("dig_p8: %i 0x%04x\n", calib_data.dig_p8, calib_data.dig_p8);
        printf("dig_p9: %i 0x%02x\n", calib_data.dig_p9, calib_data.dig_p9);
        printf("dig_h1: %u 0x%02x\n", calib_data.dig_h1, calib_data.dig_h1);
        printf("dig_h2: %i 0x%04x\n", calib_data.dig_h2, calib_data.dig_h2);
        printf("dig_h3: %u 0x%02x\n", calib_data.dig_h3, calib_data.dig_h3);
        printf("dig_h4: %i 0x%04x\n", calib_data.dig_h4, calib_data.dig_h4);
        printf("dig_h5: %i 0x%04x\n", calib_data.dig_h5, calib_data.dig_h5);
        printf("dig_h6: %i 0x%02x\n", calib_data.dig_h6, calib_data.dig_h6);

        buf2[0] = READ(0xE1);
        bcm2835_spi_transfern(buf2, 2);
        printf("0xE1: 0x%02x ", buf2[1]);
        buf2[0] = READ(0xE2);
        bcm2835_spi_transfern(buf2, 2);
        printf("0xE2: 0x%02x ", buf2[1]);
        buf2[0] = READ(0xE3);
        bcm2835_spi_transfern(buf2, 2);
        printf("0xE3: 0x%02x ", buf2[1]);
        buf2[0] = READ(0xE4);
        bcm2835_spi_transfern(buf2, 2);
        printf("0xE4: 0x%02x ", buf2[1]);
        buf2[0] = READ(0xE5);
        bcm2835_spi_transfern(buf2, 2);
        printf("0xE5: 0x%02x ", buf2[1]);
        buf2[0] = READ(0xE6);
        bcm2835_spi_transfern(buf2, 2);
        printf("0xE6: 0x%02x ", buf2[1]);
        buf2[0] = READ(0xE7);
        bcm2835_spi_transfern(buf2, 2);
        printf("0xE7: 0x%02x \n", buf2[1]);
    }
}

/* old
void bce280_calibration_data_read()
{
    uint8_t buf1[27]; // calibzone1: 0x88 - 0xA1 = 26, 0xA0 is not needed!!
    uint8_t buf2[8]; // calibzone2: 0xE1 - 0xE7 = 7
    uint8_t bufall[32];

    buf1[0] = READ(0x88);
    bcm2835_spi_transfern(buf1, 27);
    memcpy(bufall, &buf1[1], 24);
    bufall[24] = buf1[26]; // 0xA0 contains useless data for us
    int i = 0;
    while (i < 27)
    {
        printf("0x%02x ", buf1[i]);
        i++;
    }
    printf("\n\n");

    buf2[0] = READ(0xE1);
    bcm2835_spi_transfern(buf2, 8);
    memcpy(bufall+25, &buf2[1], 7);
    i = 0;
    while (i < 8)
    {
        printf("0x%02x ", buf2[i]);
        i++;
    }
    printf("\n\n");

    i = 0;
    while (i < 32)
    {
        printf("0x%02x ", bufall[i]);
        i++;
    }
    cdata = (struct bme280_calib_data*)bufall;
    printf("\ndig_p9: %i 0x%02x\n", cdata->dig_p9, cdata->dig_p9);
    printf("dig_t1: %i 0x%04x\n", cdata->dig_t1, cdata->dig_t1);
    printf("dig_t2: %i 0x%04x\n", cdata->dig_t2, cdata->dig_t2);
    printf("dig_h1: %i 0x%04x\n", cdata->dig_h1, cdata->dig_h1);
    printf("dig_h2: %i 0x%04x\n", cdata->dig_h2, cdata->dig_h2);
    printf("dig_h3: %i 0x%02x\n", cdata->dig_h3, cdata->dig_h3);
    printf("dig_h4: %i 0x%04x\n", cdata->dig_h4, cdata->dig_h4);
    printf("dig_h5: %i 0x%04x\n", cdata->dig_h5, cdata->dig_h5);
    printf("dig_h6: %i 0x%02x\n", cdata->dig_h6, cdata->dig_h6);

    buf2[0] = READ(0xE1);
    bcm2835_spi_transfern(buf2, 2);
    printf("0xE1: 0x%02x ", buf2[1]);
    buf2[0] = READ(0xE2);
    bcm2835_spi_transfern(buf2, 2);
    printf("0xE2: 0x%02x ", buf2[1]);
    buf2[0] = READ(0xE6);
    bcm2835_spi_transfern(buf2, 2);
    printf("0xE6: 0x%02x ", buf2[1]);
    buf2[0] = READ(0xE7);
    bcm2835_spi_transfern(buf2, 2);
    printf("0xE7: 0x%02x ", buf2[1]);
}
*/

/*!
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in double data type.
 */
static double compensate_temperature(int32_t uncomp_data)
{
    double var1;
    double var2;
    double temperature;
    double temperature_min = -40;
    double temperature_max = 85;

    var1 = ((double)uncomp_data) / 16384.0 - ((double)calib_data.dig_t1) / 1024.0;
    var1 = var1 * ((double)calib_data.dig_t2);
    var2 = (((double)uncomp_data) / 131072.0 - ((double)calib_data.dig_t1) / 8192.0);
    var2 = (var2 * var2) * ((double)calib_data.dig_t3);
    calib_data.t_fine = (int32_t)(var1 + var2);
    temperature = (var1 + var2) / 5120.0;
    if (temperature < temperature_min)
    {
        temperature = temperature_min;
    }
    else if (temperature > temperature_max)
    {
        temperature = temperature_max;
    }

    return temperature;
}

/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in double data type.
 */
static double compensate_pressure(int32_t uncomp_data)
{
    double var1;
    double var2;
    double var3;
    double pressure;
    double pressure_min = 30000.0;
    double pressure_max = 110000.0;

    var1 = ((double)calib_data.t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double)calib_data.dig_p6) / 32768.0;
    var2 = var2 + var1 * ((double)calib_data.dig_p5) * 2.0;
    var2 = (var2 / 4.0) + (((double)calib_data.dig_p4) * 65536.0);
    var3 = ((double)calib_data.dig_p3) * var1 * var1 / 524288.0;
    var1 = (var3 + ((double)calib_data.dig_p2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double)calib_data.dig_p1);

    /* avoid exception caused by division by zero */
    if (var1 > (0.0))
    {
        pressure = 1048576.0 - (double) uncomp_data;
        pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
        var1 = ((double)calib_data.dig_p9) * pressure * pressure / 2147483648.0;
        var2 = pressure * ((double)calib_data.dig_p8) / 32768.0;
        pressure = pressure + (var1 + var2 + ((double)calib_data.dig_p7)) / 16.0;
        if (pressure < pressure_min)
        {
            pressure = pressure_min;
        }
        else if (pressure > pressure_max)
        {
            pressure = pressure_max;
        }
    }
    else /* Invalid case */
    {
        pressure = pressure_min;
    }

    return pressure;
}

/*!
 * @brief This internal API is used to compensate the raw humidity data and
 * return the compensated humidity data in double data type.
 */
static double compensate_humidity(int32_t uncomp_data)
{
    double humidity;
    double humidity_min = 0.0;
    double humidity_max = 100.0;
    double var1;
    double var2;
    double var3;
    double var4;
    double var5;
    double var6;

    var1 = ((double)calib_data.t_fine) - 76800.0;
    var2 = (((double)calib_data.dig_h4) * 64.0 + (((double)calib_data.dig_h5) / 16384.0) * var1);
    var3 = uncomp_data - var2;
    var4 = ((double)calib_data.dig_h2) / 65536.0;
    var5 = (1.0 + (((double)calib_data.dig_h3) / 67108864.0) * var1);
    var6 = 1.0 + (((double)calib_data.dig_h6) / 67108864.0) * var1 * var5;
    var6 = var3 * var4 * (var5 * var6);
    humidity = var6 * (1.0 - ((double)calib_data.dig_h1) * var6 / 524288.0);

    if (humidity > humidity_max)
    {
        humidity = humidity_max;
    }
    else if (humidity < humidity_min)
    {
        humidity = humidity_min;
    }

    return humidity;
}

void bce280_transfer(uint8_t len, uint8_t *outbuf, uint8_t *inbuf)
{
    if (len > MAX_BUFFER_LEN)
        exit(1);

    if (DEBUG)
    {
        printf("tx: ");
        int i = 0;
        while (i < len)
        {
            printf("0x%02x ", outbuf[i]);
            i++;
        }
    }

    memset(inbuf, 0x00, len);
    //bcm2835_spi_transfernb(outbuf, inbuf, len);
    bang_spi_transfer_n(outbuf, inbuf, len);
    memset(outbuf, 0x00, len);

    if (DEBUG)
    {
        printf("\nrx: ");
        int i = 0;
        while (i < len)
        {
            printf("0x%02x ", inbuf[i]);
            i++;
        }
        printf("\n");
    }
}

void read_sensors(uint8_t *inbuf, double *t, double *p, double *h)
{
    // read raw measured values
    int32_t press = inbuf[1] << 12 | inbuf[2] << 4 | inbuf[3] >> 4;
    //printf("press: %i 0x%08x\n", press, press); 
    int32_t temp  = inbuf[4] << 12 | inbuf[5] << 4 | inbuf[6] >> 4;
    //printf("temp: %i 0x%08x\n", temp, temp); 
    int32_t hum = inbuf[7] << 8 | inbuf[8];
    //printf("hum: %i 0x%08x\n", hum, hum); 

    // calculate actual values
    *t =compensate_temperature(temp);
    *p =compensate_pressure(press) / 100;
    *h =compensate_humidity(hum);
}

int main()
{
    time_t rawtime;
    struct tm * timeinfo;
    char time_str[20];

    time ( &rawtime );
    timeinfo = localtime ( &rawtime );

    strftime (time_str, 20, "%F %T", timeinfo);

    //printf ( "Current local time and date: %s\n", time_str);
    //printf ( "Current local time and date: %s", asctime (timeinfo) );

    bang_spi_init(sck, cs, mosi, miso);
    bang_spi_bit_order(BANG_SPI_BIT_ORDER_MSBFIRST);
    bang_spi_mode(0);
    //bang_spi_clock_divider(65535);
    bang_spi_clock_divider(5000);

    // bcm2835 was initialized by bang_spi_init()
    bcm2835_gpio_fsel(pwr, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_write(pwr, HIGH);

    //bang_spi_transfer_n(out, in, 2);
    
    uint8_t outbuf[9];
    uint8_t inbuf[9];

    // read out old values
    outbuf[0] = READ(BCE280_REG_MEAS_VALUES);
    bce280_transfer(9,outbuf,inbuf);
    bce280_calibration_data_read();

    double t_old,p_old,h_old;
    read_sensors(inbuf, &t_old, &p_old, &h_old);

    // --read out old values

    // make measurement
    // read ctrl_hum register (to only change 3bits of it when writing)
    outbuf[0] = READ(BCE280_REG_CTRL_HUM);
    bce280_transfer(2,outbuf,inbuf);
    //printf("HUM CTRL: 0x%02x\n", inbuf[1]);
    if ((inbuf[1] & BCE280_HUM_CONF_MASK) != BCE280_SENSOR_CONF)
    {
        // configure only if not already configured
        outbuf[0] = WRITE(BCE280_REG_CTRL_HUM); 
        outbuf[1] = (inbuf[1] & ~BCE280_HUM_CONF_MASK) | BCE280_SENSOR_CONF;// = 101 //0x01;
        //printf("writing HUM CTRL: 0x%02x\n", outbuf[1]);
        // activate hum measurement
        bce280_transfer(2,outbuf,inbuf);
        // check if write was effective
        //outbuf[0] = READ(BCE280_REG_CTRL_HUM);
        ////bce280_transfer(2,outbuf,inbuf);
    }
    /*
    outbuf[0] = READ(BCE280_REG_CTRL_HUM);
    bce280_transfer(2,outbuf,inbuf);
    printf("HUM CTRL: 0x%02x\n", inbuf[1]);
    */

    // configure ctrl_meas register
    // we must write this register to start measurement
    // so no check if already configured correctly
    //outbuf[0] = READ(BCE280_REG_CTRL_MEAS);
    //bce280_transfer(2,outbuf,inbuf);
    outbuf[0] = WRITE(BCE280_REG_CTRL_MEAS); 
    outbuf[1] = BCE280_CTRL_MEAS_CONF;// = 1011 0101 //0x25; = 0010 0101
    bce280_transfer(2,outbuf,inbuf);
    // read back changes to check effect
    //outbuf[0] = READ(BCE280_REG_CTRL_MEAS);
    //bce280_transfer(2,outbuf,inbuf);
    
    // wait for result via checking the sensor's status register
    //sleep(1);
    do 
    {
        usleep(50);
        outbuf[0] = READ(BCE280_REG_STATUS);
        bce280_transfer(2,outbuf,inbuf);
    } while ((inbuf[1] & 1 )|| (inbuf[1] >> 3 & 1));

    outbuf[0] = READ(BCE280_REG_MEAS_VALUES);
    bce280_transfer(9,outbuf,inbuf);
    // -- make measurement

    //TODO check filter configuration and datasheet

    //bce280_calibration_data_read();

    double t,p,h;
    read_sensors(inbuf, &t, &p, &h);

    /*
    printf("old values: temp: %.2f press: %.2f hum: %.2f\n", t_old, p_old, h_old);
    printf("new values: temp: %.2f press: %.2f hum: %.2f\n", t, p, h);
    */

    char out_str[40];
    int fd = open(LOG_FILE, O_WRONLY | O_APPEND | O_CREAT, S_IROTH);

    sprintf(out_str, "%s,%.2f,%.2f,%.2f\n", time_str, roundf(100*t)/100, roundf(100*p)/100, roundf(100*h)/100);
    printf("%s", out_str);
    write(fd, out_str, strlen(out_str));
    
    close(fd);

    bcm2835_gpio_fsel(pwr, BCM2835_GPIO_FSEL_INPT);
    bang_spi_exit();
    return 0;
    
}
