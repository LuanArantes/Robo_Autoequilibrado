#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/multicore.h"
#include "hardware/pwm.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

static int addr = 0x68;

const int PWR_MGMT_1   = 0x6B;
const int SMPLRT_DIV   = 0x19;
const int CONFIG       = 0x1A;
const int GYRO_CONFIG  = 0x1B;
const int INT_ENABLE   = 0x38;
const int ACCEL_XOUT_H = 0x3B;
const int ACCEL_YOUT_H = 0x3D;
const int ACCEL_ZOUT_H = 0x3F;
const int GYRO_XOUT_H  = 0x43;
const int GYRO_YOUT_H  = 0x45;
const int GYRO_ZOUT_H  = 0x47;
const int TEMP_OUT_H   = 0X41;

float calib_x_accel = 0.0;
float calib_y_accel = 0.0;
float calib_z_accel = 0.0; 
float calib_x_gyro  = 0.0; 
float calib_y_gyro  = 0.0; 
float calib_z_gyro  = 0.0;

const uint  SDA = 12;
const uint  SCL = 1;

const float QAngle = 0.001;                
const float QBias = 0.003; 
const float RMeasure = 0.1;                    
float angle = 0.0;
float bias = 0.0;
float rate = 0.0;
float P[2][2]={{0.0,0.0},{0.0,0.0}};

const uint  A1 = 17;
const uint  B1 = 16;

const uint  M1A = 19;
const uint  M1B = 18;
const uint  M2A = 21;
const uint  M2B = 20;

const uint  ts = 2;
int j = 0;
const int tj = 1;
const uint T = ts*tj;
const float dt = ts/1000.0;
const uint  TEMPO = 20000;
const float VOLT_TO_PWM = (65535/5.99);



// xo(n) = Ao*xo(n-1)+Bo*u(n-1)+L1*y(n)+L2*y(n-1)
const float  M = 0.0; //M = 1.48 //R = p
const float ZERO = 0.0;
const float theta_offset = 0.8;
const float YZERO = 0.001;

const float k[4] = {        0.03098,          34.988,     0.21298,    6.1538};
const float Ao[2][2] = {{        0.73911,      -0.037159},{   0.024793,        0.67841}};
const float Bo[2] = {       -0.34683,    0.010322};
const float L1[2][2] = {{       59.487,    -6.5775},{    -10.679,    161.16}};
const float L2[2][2] = {{       -59.487,        6.5409},{  10.679,    -161.15}};   

int ang = 0;
const float conv = 2*M_PI/374.0;
float Y0[2] = {0.0,0.0};
float X0[2] = {0.0,0.0};

int tempo = 0;
bool TIMER = false;

float u = 0.0;

float DATA[6][10000];

float k_filtered_angle(float ay_angle, float Gy){

    rate = Gy-bias;

    angle = angle + dt*rate;

    P[0][0] = P[0][0] + dt * (dt*P[1][1] -P[0][1] - P[1][0] + QAngle);
    P[0][1] = P[0][1] - dt * P[1][1];
    P[1][0] = P[1][0] - dt * P[1][1];
    P[1][1] = P[1][1] + QBias * dt;

    float y = ay_angle - angle;

    float s = P[0][0] + RMeasure;

    float K[2]={0.0,0.0};
    K[0] = P[0][0]/s;
    K[1] = P[1][0]/s;

    angle = angle + K[0] * y;
    bias  = bias + K[1] * y;

    float P00Temp = P[0][0];
    float P01Temp = P[0][1];

    P[0][0] = P[0][0] - K[0] * P00Temp;
    P[0][1] = P[0][1] -  K[0] * P01Temp;
    P[1][0] = P[1][0] -  K[1] * P00Temp;
    P[1][1] = P[1][1] -  K[1] * P01Temp;
    return angle;
}
void send_cmd(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    i2c_write_blocking(i2c0, addr, buf, 2, false);
}

static void init_MPU() {
    send_cmd(SMPLRT_DIV, 0x07);
    send_cmd(PWR_MGMT_1, 0x00);
    send_cmd(CONFIG, 0x00);
    send_cmd(GYRO_CONFIG, 0x18);
    send_cmd(INT_ENABLE, 0x00);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t *gyro_y) {

    uint8_t buffer[6];
    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = ACCEL_XOUT_H;
    i2c_write_blocking(i2c0, addr, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c0, addr, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = GYRO_YOUT_H;
    i2c_write_blocking(i2c0, addr, &val, 1, true);
    i2c_read_blocking(i2c0, addr, buffer, 2, false);  // False - finished with bus
    *gyro_y = (buffer[0] << 8 | buffer[1]);
}

float read_mpu6050(){
    int16_t acceleration[3], gyro_y;
    mpu6050_read_raw(acceleration, &gyro_y);
    
    float Ax = acceleration[0]/16384.0;
    float Ay = acceleration[1]/16384.0;
    float Az = acceleration[2]/16384.0;

    float Gy = (gyro_y - calib_y_gyro)/131.0;
    float ay_angle = atan((-1*Ax)/sqrt(pow(Ay,2) + pow(Az, 2)))*(180/M_PI);
    float k_angle_y = k_filtered_angle(ay_angle, Gy);
    return k_angle_y;
}
void calibrate_sensors(){
    int16_t acceleration[3], gyro_y;
    float x_accel = 0;
    float y_accel = 0;
    float z_accel = 0;
    float y_gyro  = 0;
    mpu6050_read_raw(acceleration, &gyro_y);
    for(int i=0;i<10;i++){
        mpu6050_read_raw(acceleration, &gyro_y);
        x_accel += acceleration[0];
        y_accel += acceleration[1];
        z_accel += acceleration[2];
        y_gyro  += gyro_y;
        sleep_ms(100);
    }
    x_accel /= 10;
    y_accel /= 10;
    z_accel /= 10;
    y_gyro  /= 10;

    calib_x_accel = x_accel;
    calib_y_accel = y_accel;
    calib_z_accel = z_accel;
    calib_y_gyro  = y_gyro;
}

void control(){
    float X[2];
    float Y[2];
    float Ang = ang*conv;
    float Theta  = (read_mpu6050()-theta_offset)*M_PI/180;
    
    Y[0] = Ang;
    Y[1] = Theta;

    X[0] = Ao[0][0]*X0[0]+Ao[0][1]*X0[1]+Bo[0]*u+L1[0][0]*Y[0]+L1[0][1]*Y[1]+L2[0][0]*Y0[0]+L2[0][1]*Y0[1];
    X[1] = Ao[1][0]*X0[0]+Ao[1][1]*X0[1]+Bo[1]*u+L1[1][0]*Y[0]+L1[1][1]*Y[1]+L2[1][0]*Y0[0]+L2[1][1]*Y0[1];

    //xo(n) = Ao*xo(n-1)+Bo*u(n-1)+L1*y(n)+L2*y(n-1)
    

    Y0[0] = Y[0];
    Y0[1] = Y[1];

    X0[0] = X[0];
    X0[1] = X[1];
    u =-(k[0]*Y[0]+k[1]*Y[1]+k[2]*X[0]+k[3]*X[1]);
    
    DATA[0][tempo/T] = tempo;
    DATA[1][tempo/T] = Y[0];
    DATA[2][tempo/T] = Y[1];
    DATA[3][tempo/T] = X[0];
    DATA[4][tempo/T] = X[1];
    DATA[5][tempo/T] = u;
}
void datalog(){
    for(int i = 0;i<TEMPO/T;i++){
        printf("%f,%f,%f,%f,%f,%f\n",DATA[0][i],DATA[1][i],DATA[2][i],DATA[3][i],DATA[4][i],DATA[5][i]);
        sleep_ms(2);
    }
    sleep_ms(5000);
}
    
void core1_entry() {
	void gpio_callback(uint gpio, uint32_t events) {
		if(gpio_get(A1)){
		    ang = ang + 1;
		}else{
		    ang = ang - 1;
		}
	}
	gpio_init(A1);
    gpio_set_dir(A1, GPIO_IN);
    gpio_set_irq_enabled_with_callback(B1, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
}

bool repeating_timer_callback_0(struct repeating_timer *t) {
    float Ang = ang*conv;
    float Theta  = (read_mpu6050()-theta_offset)*M_PI/180;
    
    Y0[0] = Ang;
    Y0[1] = Theta;
    return true;
}

bool repeating_timer_callback(struct repeating_timer *t) {
    tempo = tempo + ts;
    if(tempo<TEMPO){
        j ++;
        if(j>=(tj)){
            control();
            j=0;
        }else{
            read_mpu6050();
        }
        
    }else{
        u = 0.0;
    }
    TIMER = true;
    return true;
}


int main() {
    stdio_init_all();

    i2c_init(i2c0, 400000);
    gpio_set_function(SDA, GPIO_FUNC_I2C);
    gpio_set_function(SCL, GPIO_FUNC_I2C);
    gpio_pull_up(SDA);
    gpio_pull_up(SCL);
    bi_decl(bi_2pins_with_func(SDA, SCL, GPIO_FUNC_I2C));

    init_MPU();

    calibrate_sensors();

    gpio_set_function(M1A, GPIO_FUNC_PWM);
    gpio_set_function(M1B, GPIO_FUNC_PWM);
    gpio_set_function(M2A, GPIO_FUNC_PWM);
    gpio_set_function(M2B, GPIO_FUNC_PWM);
    uint slice_numM1 = pwm_gpio_to_slice_num(M1A);
    uint slice_numM2 = pwm_gpio_to_slice_num(M2A);
    pwm_set_wrap(slice_numM1, 65534);
    pwm_set_wrap(slice_numM2, 65534);
    pwm_set_enabled(slice_numM1, true);
    pwm_set_enabled(slice_numM2, true);
    pwm_set_gpio_level(M1A, 0);
    pwm_set_gpio_level(M1B, 0);
    pwm_set_gpio_level(M2A, 0);
    pwm_set_gpio_level(M2B, 0);
    
	multicore_launch_core1(core1_entry);

    struct repeating_timer timer;
    add_repeating_timer_ms(-ts, repeating_timer_callback_0, NULL, &timer);
    sleep_ms(1000);
    cancel_repeating_timer(&timer);
    control();
	add_repeating_timer_ms(-ts, repeating_timer_callback, NULL, &timer);

    // Wait forever
    while (1){
        if(TIMER){
            if((Y0[1]>=YZERO)||(Y0[1]<=-YZERO)){
                if(u<-ZERO){
                    u = u - M;
                    if(-u>5.99){
                        u = -5.99;
                    }
                    pwm_set_gpio_level(M1A, 0);
                    pwm_set_gpio_level(M1B, -(int)(u*VOLT_TO_PWM));
                    pwm_set_gpio_level(M2A, 0);
                    pwm_set_gpio_level(M2B, -(int)(u*VOLT_TO_PWM));
                    u = u + M;
                }
                if(u>ZERO){
                    u = u + M;
                    if(u>5.99){
                        u = 5.99;
                    }
                    pwm_set_gpio_level(M1A, (int)(u*VOLT_TO_PWM));
                    pwm_set_gpio_level(M1B, 0);
                    pwm_set_gpio_level(M2A, (int)(u*VOLT_TO_PWM));
                    pwm_set_gpio_level(M2B, 0);
                    u = u - M;
                }
                if((u>=-ZERO)&(u<=ZERO)){
                    u = 0.0;
                    pwm_set_gpio_level(M1A, 0);
                    pwm_set_gpio_level(M1B, 0);
                    pwm_set_gpio_level(M2A, 0);
                    pwm_set_gpio_level(M2B, 0);
                }
            }else{
                pwm_set_gpio_level(M1A, 0);
                pwm_set_gpio_level(M1B, 0);
                pwm_set_gpio_level(M2A, 0);
                pwm_set_gpio_level(M2B, 0);
            }
            TIMER = false;
        }
        if(tempo>TEMPO){
            cancel_repeating_timer(&timer);
            datalog();
        }
    }
    return 0;
}