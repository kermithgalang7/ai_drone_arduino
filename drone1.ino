/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Blink
*/

#include <MPU9250_WE.h>
#include <Wire.h>
#define MPU9250_ADDR            0x68

struct command_queue_t {
  int command;
  int duration_in_millis;  
};

#define MOTION_CMD_IDLE         0
#define MOTION_CMD_FORWARD      1
#define MOTION_CMD_BACKWARD     2
#define MOTION_CMD_ROT_LEFT     3
#define MOTION_CMD_ROT_RIGHT    4

#define MOTION_CMD_STATPROX     5
#define MOTION_CMD_STATACC      6
#define MOTION_CMD_STATGYR      7
#define MOTION_CMD_STATMAG      8
#define MOTION_CMD_STATTEMP     9

#define MOTOR_FRONT_LEFT1       6
#define MOTOR_FRONT_LEFT2       5
#define MOTOR_FRONT_RIGHT1      4
#define MOTOR_FRONT_RIGHT2      3
#define MOTOR_REAR_LEFT1        A0
#define MOTOR_REAR_LEFT2        A1
#define MOTOR_REAR_RIGHT1       A2
#define MOTOR_REAR_RIGHT2       A3

#define PROX_ECHO               8
#define PROX_TRIGGER            7

#define MAX_COMMAND_QUEUE       10
#define MAX_SERIAL_BUFFER       100
#define MAX_SERIALTX_BUFFER     200
#define SERIAL_TX_DELAY         75

int motor_front_left_active = 0;
int motor_front_right_active = 0;
int motor_rear_left_active = 0;
int motor_rear_right_active = 0;

int pulse_duration = 0;
float distance_in_cm = 0;

char serial_command[MAX_SERIAL_BUFFER];
char serial_to_send[MAX_SERIALTX_BUFFER];
int serial_current_rx = 0;
int serial_current_tx = 0;
int serial_current_tosend = 0;

unsigned long ledServiceMillis = 0;
unsigned long motorServiceMillis = 0;
unsigned long commandQueueServiceMillis = 0;
unsigned long seialServiceMillis = 0;
unsigned long seialReceiveServiceMillis = 0;
unsigned long mpuServiceMillis = 0;

int ledCounter = 0;
//int motorCounter = 0;
int mpuCounter = 0;

int first_boot_flag = 0;
int mag_ok = 0;
int mpu_ok = 0;

MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);

struct command_queue_t command_queue[MAX_COMMAND_QUEUE];
#if 0
struct command_queue_t command_queue[MAX_COMMAND_QUEUE] = {
    { MOTION_CMD_IDLE, 500 },
    { MOTION_CMD_IDLE, 500 },
    { MOTION_CMD_FORWARD, 500 },
    { MOTION_CMD_BACKWARD, 500 },
    { MOTION_CMD_ROT_LEFT, 500 },
    { MOTION_CMD_ROT_RIGHT, 500 },
    { NULL, NULL },
};
#endif
struct command_queue_t temp_cmd;

int dummy_ctr = 0;
char temp_buffer[100];
char temp_float_to_string[50];

/******************** START **************************/

void send_to_server(char* msg)//discontinued
{
    int i;
    for(i = 0; msg[i] != NULL; i++)
    {
//        Serial.print(msg[i]);
//        delay(SERIAL_TX_DELAY);
    }
}
int send_to_server_ln(char* msg)
{
#if 0
    if(serial_to_send[serial_current_tx] == 0)
    {
        memset(serial_to_send, 0, MAX_SERIALTX_BUFFER);
        strcpy(serial_to_send, msg);
        serial_current_tx = 0;   
    }
    else
        return -1;
#endif
    sprintf(msg, "%s\r\n", msg);
    
    int i;
    for(i = 0; i < strlen(msg); i++)
    {
        serial_to_send[serial_current_tosend] = msg[i];
        
        if(serial_current_tosend < MAX_SERIALTX_BUFFER)
            serial_current_tosend++;
        else
            serial_current_tosend = 0;

        if(serial_current_tosend == serial_current_tx)
            return -1;
    }

    return 0;
}
int serialService()
{
#if 0
    if(serial_to_send[serial_current_tx] != 0)
    {
        Serial.print(serial_to_send[serial_current_tx]);
        serial_current_tx++;
    }
#endif    
    if(serial_current_tx != serial_current_tosend)
    {
        Serial.print(serial_to_send[serial_current_tx]);
        serial_to_send[serial_current_tx] = 0;//maybe no need
        
        if(serial_current_tx < MAX_SERIALTX_BUFFER)
            serial_current_tx++;
        else
            serial_current_tx = 0;
    }

    return 0;
}

int serialReceiveService()
{
    char rx_buff[10] = { 0 };
    //Serial command 
    if(Serial.available() > 0)
    {
        Serial.readBytes(rx_buff, 1);
        if(isPrintable(rx_buff[0]) == 1)
        {
            serial_command[serial_current_rx] = rx_buff[0];
            serial_current_rx++;            
        }
        else if(rx_buff[0] == '\r' )
        {
            if(parse_serial_command(serial_command) == 0)
            {                
                sprintf(temp_buffer, "RCV %s\n", serial_command);
                send_to_server_ln(temp_buffer);
            }
            else
            {
                sprintf(temp_buffer, "ERR %s\n", serial_command);
                send_to_server_ln(temp_buffer);
            }
            serial_current_rx = 0;
            memset(serial_command, 0, MAX_SERIAL_BUFFER);
        }
    }
    
    return 0;
}

int motor_emegency_stop(void)
{
    motor_front_left_active = 0;
    motor_front_right_active = 0;
    motor_rear_left_active = 0;
    motor_rear_right_active = 0;
    return 0;
}

int motor_forward(int duration_in_milli)
{
    motor_front_left_active = duration_in_milli;
    motor_front_right_active = duration_in_milli;
    motor_rear_left_active = duration_in_milli;
    motor_rear_right_active = duration_in_milli;
    return 0;
}
int motor_backward(int duration_in_milli)
{
    motor_front_left_active = -duration_in_milli;
    motor_front_right_active = -duration_in_milli;
    motor_rear_left_active = -duration_in_milli;
    motor_rear_right_active = -duration_in_milli;
    return 0;
}
int motor_rotate_left(int duration_in_milli)
{
    motor_front_left_active = -duration_in_milli;
    motor_front_right_active = duration_in_milli;
    motor_rear_left_active = -duration_in_milli;
    motor_rear_right_active = duration_in_milli;
    return 0;
}
int motor_rotate_right(int duration_in_milli)
{
    motor_front_left_active = duration_in_milli;
    motor_front_right_active = -duration_in_milli;
    motor_rear_left_active = duration_in_milli;
    motor_rear_right_active = -duration_in_milli;
    return 0;
}

int push_command_queue(int command, int duration_in_millis)
{
    int i;

    for(i = 0; i < MAX_COMMAND_QUEUE; i++)
    {
        if(command_queue[i].command == 0)
            if(command_queue[i].duration_in_millis == 0)
            {
                command_queue[i].command = command;
                command_queue[i].duration_in_millis = duration_in_millis;
                return 0;
            }
    }
    
    return -1; //if queue full
}

struct command_queue_t pop_command_queue(void)
{
    struct command_queue_t ret_cmd;
    int i;

    ret_cmd = command_queue[0];
    for(i = 1; i < MAX_COMMAND_QUEUE; i++)
    {
        command_queue[i - 1] = command_queue[i];
    }
    command_queue[MAX_COMMAND_QUEUE - 1].command = NULL;
    command_queue[MAX_COMMAND_QUEUE - 1].duration_in_millis = NULL;

    return ret_cmd;
}

void clear_command_queue(void)
{
    int i;
    for(i = 0; i < MAX_COMMAND_QUEUE; i++)
    {
        command_queue[MAX_COMMAND_QUEUE - 1].command = NULL;
        command_queue[MAX_COMMAND_QUEUE - 1].duration_in_millis = NULL;    
    }
}

int parse_serial_command(char* serial_command)
{
    int command = -1;
    int duration_in_millis = -1;
    char* token;
    char delim[] = ":";

    if(serial_command == NULL)
        return -1;

    token = strtok(serial_command, delim);
    if(token == NULL)
        return -1;
    if(strcmp(token, "MIDLE") == 0)
        command = MOTION_CMD_IDLE;
    if(strcmp(token, "MFORWARD") == 0)
        command = MOTION_CMD_FORWARD;
    if(strcmp(token, "MBACKWARD") == 0)
        command = MOTION_CMD_BACKWARD;
    if(strcmp(token, "MROTLEFT") == 0)
        command = MOTION_CMD_ROT_LEFT;
    if(strcmp(token, "MROTRIGHT") == 0)
        command = MOTION_CMD_ROT_RIGHT;

    if(strcmp(token, "PRX") == 0)
        command = MOTION_CMD_STATPROX;
    if(strcmp(token, "ACC") == 0)
        command = MOTION_CMD_STATACC;
    if(strcmp(token, "GYR") == 0)
        command = MOTION_CMD_STATGYR;
    if(strcmp(token, "MAG") == 0)
        command = MOTION_CMD_STATMAG;
    if(strcmp(token, "TEMP") == 0)
        command = MOTION_CMD_STATTEMP;

    if((command >= MOTION_CMD_STATPROX) && (command <= MOTION_CMD_STATTEMP))
    {
        mpuService(command);
        return 0;
    }

    //invalid command
    if(command == -1)
        return -1;

    token = strtok(NULL, delim);
    if(token == NULL)
        return -2;
    duration_in_millis = atoi(token);
    if(duration_in_millis > 1000) 
        duration_in_millis = 1000;
    push_command_queue(command, duration_in_millis);
    
    return 0;
}

int ledService()
{
    if(ledCounter == 0)
        digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)

    if(temp_cmd.command != MOTION_CMD_IDLE)
    {
        if (ledCounter >= 16)
            digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    }
    else 
    {
        if (ledCounter >= 2)
            digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    }
    
    if(ledCounter >= 20)
        ledCounter = 0;
    else
        ledCounter++;
        
    return 0;
}

int motorService()
{
    //motor control
    digitalWrite(MOTOR_FRONT_LEFT1, LOW);
    digitalWrite(MOTOR_FRONT_LEFT2, LOW);
    digitalWrite(MOTOR_FRONT_RIGHT1, LOW);
    digitalWrite(MOTOR_FRONT_RIGHT2, LOW);
    digitalWrite(MOTOR_REAR_LEFT1, LOW);
    digitalWrite(MOTOR_REAR_LEFT2, LOW);
    digitalWrite(MOTOR_REAR_RIGHT1, LOW);
    digitalWrite(MOTOR_REAR_RIGHT2, LOW);
    
    //forward
    if(motor_front_left_active > 0)
    {
        digitalWrite(MOTOR_FRONT_LEFT1, LOW);
        digitalWrite(MOTOR_FRONT_LEFT2, HIGH);
        motor_front_left_active--;
    }
    if(motor_front_right_active > 0) 
    {
        digitalWrite(MOTOR_FRONT_RIGHT1, LOW);
        digitalWrite(MOTOR_FRONT_RIGHT2, HIGH);
        motor_front_right_active--;
    }
    if(motor_rear_left_active > 0) 
    {
        digitalWrite(MOTOR_REAR_LEFT1, HIGH);
        digitalWrite(MOTOR_REAR_LEFT2, LOW);
        motor_rear_left_active--;
    }
    if(motor_rear_right_active > 0) 
    {
        digitalWrite(MOTOR_REAR_RIGHT1, HIGH);
        digitalWrite(MOTOR_REAR_RIGHT2, LOW);
        motor_rear_right_active--;
    }

    //reverse
    if(motor_front_left_active < 0) 
    {
        digitalWrite(MOTOR_FRONT_LEFT1, HIGH);
        digitalWrite(MOTOR_FRONT_LEFT2, LOW);
        motor_front_left_active++;
    }
    if(motor_front_right_active < 0)
    {
        digitalWrite(MOTOR_FRONT_RIGHT1, HIGH);
        digitalWrite(MOTOR_FRONT_RIGHT2, LOW);
        motor_front_right_active++;
    }
    if(motor_rear_left_active < 0) 
    {
        digitalWrite(MOTOR_REAR_LEFT1, LOW);
        digitalWrite(MOTOR_REAR_LEFT2, HIGH);
        motor_rear_left_active++;
    }
    if(motor_rear_right_active < 0) 
    {
        digitalWrite(MOTOR_REAR_RIGHT1, LOW);
        digitalWrite(MOTOR_REAR_RIGHT2, HIGH);
        motor_rear_right_active++;
    }

    return 0;
}

int commandQueueService()
{
    //1 second queue check
    temp_cmd = pop_command_queue();
    switch(temp_cmd.command)
    {
        case MOTION_CMD_IDLE:
        break;
        case MOTION_CMD_FORWARD:
            motor_forward(temp_cmd.duration_in_millis);
        break;
        case MOTION_CMD_BACKWARD:
            motor_backward(temp_cmd.duration_in_millis);
        break;
        case MOTION_CMD_ROT_LEFT:
            motor_rotate_left(temp_cmd.duration_in_millis);
        break;
        case MOTION_CMD_ROT_RIGHT:
            motor_rotate_right(temp_cmd.duration_in_millis);
        break;
        default:
            ;
    }
}

int mpuService(int cmd)
{
    if(cmd >= 0)
        mpuCounter = cmd;
#if 1
    if(mpuCounter == MOTION_CMD_STATPROX)
    {
            //check collision
            digitalWrite(PROX_TRIGGER, LOW);
            delayMicroseconds(2);
            digitalWrite(PROX_TRIGGER, HIGH);
            delayMicroseconds(10);
            digitalWrite(PROX_TRIGGER, LOW);
            pulse_duration = pulseIn(PROX_ECHO, HIGH);
            distance_in_cm = (pulse_duration * 0.343) / 2;
            //char *dtostrf(double val, signed char width, unsigned char prec, char *s)
            //convert float (not supported by arduino) to string
            dtostrf(distance_in_cm, 4, 4, temp_float_to_string);
            sprintf(temp_buffer, "DIST %s", temp_float_to_string);
    }
    else if(mpuCounter == MOTION_CMD_STATACC)
    {
            xyzFloat gValue = myMPU9250.getGValues();
            String gX = String(gValue.x);
            String gY = String(gValue.y);
            String gZ = String(gValue.z);
            float resultantG = myMPU9250.getResultantG(gValue);
            String gforce = String(resultantG);
            sprintf(temp_buffer, "ACC %s:%s:%s:%s", gX.c_str(), gY.c_str(), gZ.c_str(), gforce.c_str());
            
    }
    else if(mpuCounter == MOTION_CMD_STATGYR)
    {       
            xyzFloat gyr = myMPU9250.getGyrValues();
            String gyrX = String(gyr.x);
            String gyrY = String(gyr.y);
            String gyrZ = String(gyr.z);
            sprintf(temp_buffer, "GYR %s:%s:%s", gyrX.c_str(), gyrY.c_str(), gyrZ.c_str());
    }       
    else if(mpuCounter == MOTION_CMD_STATMAG)
    {
            xyzFloat magValue = myMPU9250.getMagValues();
            String magX = String(magValue.x);
            String magY = String(magValue.y);
            String magZ = String(magValue.z);
            sprintf(temp_buffer, "MAG %s:%s:%s", magX.c_str(), magY.c_str(), magZ.c_str());
    }
    else if(mpuCounter == MOTION_CMD_STATTEMP)
    {
            float temp = myMPU9250.getTemperature();    
            String temperature = String(temp);            
            sprintf(temp_buffer, "TEMP:%s:C", temperature.c_str());
    }        
#endif    

    send_to_server_ln(temp_buffer);

#if 0    
    if(mpuCounter < 4)
        mpuCounter++;
    else
        mpuCounter = 0;
#endif
    return 0;
}

// the setup function runs once when you press reset or power the board
void setup() {
    Serial.begin(9600); //serial writting should be done on main loop
    memset(serial_to_send, 0, MAX_SERIALTX_BUFFER);
//    send_to_server_ln("Start");
//    send_to_server("Drone");
//    send_to_server("Basic");
//    send_to_server_ln("FW 1.0");
    
    // initialize digital pin LED_BUILTIN as an output.
    pinMode(LED_BUILTIN, OUTPUT);

    pinMode(PROX_TRIGGER, OUTPUT);
    pinMode(PROX_ECHO, INPUT);
    
    pinMode(MOTOR_FRONT_LEFT1, OUTPUT);
    pinMode(MOTOR_FRONT_LEFT2, OUTPUT);
    pinMode(MOTOR_FRONT_RIGHT1, OUTPUT);
    pinMode(MOTOR_FRONT_RIGHT2, OUTPUT);
    pinMode(MOTOR_REAR_LEFT1, OUTPUT);
    pinMode(MOTOR_REAR_LEFT2, OUTPUT);
    pinMode(MOTOR_REAR_RIGHT1, OUTPUT);
    pinMode(MOTOR_REAR_RIGHT2, OUTPUT);

    //self diag
    push_command_queue(MOTION_CMD_IDLE, 300);
    push_command_queue(MOTION_CMD_IDLE, 300);
    push_command_queue(MOTION_CMD_FORWARD, 200);
    push_command_queue(MOTION_CMD_BACKWARD, 200);
    push_command_queue(MOTION_CMD_ROT_LEFT, 100);
    push_command_queue(MOTION_CMD_ROT_RIGHT, 100);

    memset(serial_command, 0, MAX_SERIAL_BUFFER);

    Wire.begin();
    if(!myMPU9250.init()){
        mpu_ok = 0;
    }
    else{
        mpu_ok = 1;
    }
    if(!myMPU9250.initMagnetometer()){
        mag_ok = 0;
    }
    else{
        mag_ok = 1;
    }

    //calibrate accelerometer
    myMPU9250.autoOffsets();
    delay(1000);
    myMPU9250.enableGyrDLPF();
    myMPU9250.setGyrDLPF(MPU9250_DLPF_6);
    myMPU9250.setSampleRateDivider(5);
    myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);
    myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);
    myMPU9250.enableAccDLPF(true);
    myMPU9250.setAccDLPF(MPU9250_DLPF_6);
    myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);
    delay(200);
}

// the loop function runs over and over again forever
void loop() {
    //all bootup serial messages here
    if(first_boot_flag == 0)
    {
        sprintf(temp_buffer, "\r\nDrone FW1.0");
        sprintf(temp_buffer, "%s\r\nMPU:%d MAG:%d", temp_buffer, mpu_ok, mag_ok);
        send_to_server_ln(temp_buffer);
        send_to_server_ln(temp_buffer);
        first_boot_flag = 1;
    }
    
    if((millis() - seialReceiveServiceMillis) > 250) 
    {
        seialReceiveServiceMillis = millis();
        serialReceiveService();
    }

#if 0
    if((millis() - mpuServiceMillis) > 5000) 
    {
        mpuServiceMillis = millis();
        mpuService();
    }
#endif    

    if((millis() - commandQueueServiceMillis) > 1100) 
    {
        commandQueueServiceMillis = millis();
        commandQueueService();
    }    

    if((millis() - motorServiceMillis) > 1) 
    {
        motorServiceMillis = millis();
        motorService();
    }

    //LED control
    if((millis() - ledServiceMillis) > 50) 
    {
        ledServiceMillis = millis();
        ledService();
    }

    if((millis() - seialServiceMillis) > 80) 
    {
        seialServiceMillis = millis();
        serialService();
    }

}
