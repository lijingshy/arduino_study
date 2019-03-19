/**
 * @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
 * @file         weixin_contorl.c
 * @author       liusen
 * @version      V1.0
 * @date         2017.08.02
 * @brief        微信控制BST_4WD
 * @details
 * @par History  见如下说明
 *
 */
#include <Servo.h>  //系统自带的头文件
#include "./IRremote.h"


#define ON  1       //使能LED
#define OFF 0       //禁止LED

#define run_car     '1'//按键前
#define back_car    '2'//按键后
#define left_car    '3'//按键左
#define right_car   '4'//按键右
#define stop_car    '0'//按键停

/*小车运行状态枚举*/
enum {
    enSTOP = 0,
    enRUN,
    enBACK,
    enLEFT,
    enRIGHT,
    enTLEFT,
    enTRIGHT
} enCarState;

Servo myservo;      //定义舵机对象myservo

//定义引脚
int LED_R = 11;     //LED_R接在arduino上的数字11口
int LED_G = 10;     //LED_G接在arduino上的数字10口
int LED_B = 9;      //LED_B接在arduino上的数字9口

int Left_motor_go = 8;        //左电机前进 AIN1
int Left_motor_back = 7;      //左电机后退 AIN2

int Right_motor_go = 2;       //右电机前进 BIN1
int Right_motor_back = 4;     //右电机后退 BIN2

int Left_motor_pwm = 6;       //左电机控速 PWMA
int Right_motor_pwm = 5;      //右电机控速 PWMB


int buzzer = A0;//设置控制蜂鸣器的数字IO脚
/*舵机*/
int servopin = 3;  //设置舵机驱动脚到数字口2

//红外遥控
int RECV_PIN = A5; // 红外一体化接收头连接到Arduino 11号引脚
IRrecv irrecv(RECV_PIN);
decode_results results; // 用于存储编码结果的对象
unsigned long last = millis();

/*灭火*/
int Fire = A5;
/*超声波*/
int Distance = 0;
int Echo = 12;                //定义回声脚为arduino上的数字口12
int Trig = 13;                //定义触发脚为arduino上的数字口13

/*协议用到变量*/
int control = 150;//PWM控制量
int g_carstate = enSTOP; //  1前2后3左4右0停止
int g_colorlight = 0;

/**
 * Function       setup
 * @author        liusen
 * @date          2017.08.02
 * @brief         初始化配置
 * @param[in]     void
 * @retval        void
 * @par History   无
 */
void setup()
{
    //初始化电机驱动IO为输出方式
    pinMode(Left_motor_go, OUTPUT); // PIN 5 (PWM)
    pinMode(Left_motor_back, OUTPUT); // PIN 9 (PWM)
    pinMode(Right_motor_go, OUTPUT); // PIN 6 (PWM)
    pinMode(Right_motor_back, OUTPUT); // PIN 10 (PWM)
    pinMode(buzzer, OUTPUT); //设置数字IO脚模式，OUTPUT为输出
    digitalWrite(buzzer, HIGH);    // 持续给触发脚低电
    pinMode(Echo, INPUT);    // 定义超声波输入脚
    pinMode(Trig, OUTPUT);   // 定义超声波输出脚

    Serial.begin(9600);	//波特率9600 （蓝牙通讯设定波特率）

    //pinMode(Fire, OUTPUT);   // 定义灭火输出

    //初始化RGB三色LED的IO口为输出方式
    pinMode(LED_R, OUTPUT);
    pinMode(LED_G, OUTPUT);
    pinMode(LED_B, OUTPUT);

    //设置舵机控制引脚为3
    myservo.attach(servopin);
    irrecv.enableIRIn(); // 初始化红外解码
    pinMode(RECV_PIN, INPUT_PULLUP);     //将2号管脚设置为输入并且内部上拉模式
}

/**
 * Function       Distance_test
 * @author        liusen
 * @date          2017.07.26
 * @brief         超声波测距
 * @param[in]     void
 * @param[out]    void
 * @retval        void
 * @par History   无
 */
void Distance_test()   // 量出前方距离
{
    digitalWrite(Trig, LOW);   // 给触发脚低电平2μs
    delayMicroseconds(2);
    digitalWrite(Trig, HIGH);  // 给触发脚高电平10μs，这里至少是10μs
    delayMicroseconds(10);
    digitalWrite(Trig, LOW);    // 持续给触发脚低电
    float Fdistance = pulseIn(Echo, HIGH);  // 读取高电平时间(单位：微秒)
    Fdistance = Fdistance / 58;    //为什么除以58等于厘米，  Y米=（X秒*344）/2
    Distance = Fdistance;
}

/**
 * Function       run
 * @author        Danny
 * @date          2017.07.26
 * @brief         小车前进
 * @param[in]     void
 * @param[out]    void
 * @retval        void
 * @par History   无
 */
void run()
{
    //左电机前进
    digitalWrite(Left_motor_go, HIGH);   //左电机前进使能
    digitalWrite(Left_motor_back, LOW);  //左电机后退禁止
    analogWrite(Left_motor_pwm, control);

    //右电机前进
    digitalWrite(Right_motor_go, HIGH);  //右电机前进使能
    digitalWrite(Right_motor_back, LOW); //右电机后退禁止
    analogWrite(Right_motor_pwm, control);
}

/**
 * Function       brake
 * @author        Danny
 * @date          2017.07.25
 * @brief         小车刹车
 * @param[in]     time
 * @param[out]    void
 * @retval        void
 * @par History   无
 */
//刹车
void brake()
{
    digitalWrite(Left_motor_go, LOW);
    digitalWrite(Left_motor_back, LOW);
    digitalWrite(Right_motor_go, LOW);
    digitalWrite(Right_motor_back, LOW);
}

/**
 * Function       left
 * @author        Danny
 * @date          2017.07.25
 * @brief         左转(左轮不动，右轮前进)
 * @param[in]     void
 * @param[out]    void
 * @retval        void
 * @par History   无
 */

void left()
{
    //左电机停止
    digitalWrite(Left_motor_go, LOW);    //左电机前进禁止
    digitalWrite(Left_motor_back, LOW);  //左电机后退禁止
    analogWrite(Left_motor_pwm, 0);

    //右电机前进
    digitalWrite(Right_motor_go, HIGH);  //右电机前进使能
    digitalWrite(Right_motor_back, LOW); //右电机后退禁止
    analogWrite(Right_motor_pwm, 150);
}
/**
 * Function       right
 * @author        Danny
 * @date          2017.07.25
 * @brief         小车右转
 * @param[in]     void
 * @param[out]    void
 * @retval        void
 * @par History   无
 */
//右转(右轮不动，左轮前进)
void right()
{
    //左电机前进
    digitalWrite(Left_motor_go, HIGH);   //左电机前进使能
    digitalWrite(Left_motor_back, LOW);  //左电机后退禁止
    analogWrite(Left_motor_pwm, 150);

    //右电机停止
    digitalWrite(Right_motor_go, LOW);   //右电机前进禁止
    digitalWrite(Right_motor_back, LOW); //右电机后退禁止
    analogWrite(Right_motor_pwm, 0);
}

/**
 * Function       spin_left
 * @author        Danny
 * @date          2017.07.25
 * @brief         小车原地左转
 * @param[in]     void
 * @param[out]    void
 * @retval        void
 * @par History   无
 */
//原地左转(左轮后退，右轮前进)
void spin_left()
{
    //左电机后退
    digitalWrite(Left_motor_go, LOW);     //左电机前进禁止
    digitalWrite(Left_motor_back, HIGH);  //左电机后退使能
    analogWrite(Left_motor_pwm, control);

    //右电机前进
    digitalWrite(Right_motor_go, HIGH);  //右电机前进使能
    digitalWrite(Right_motor_back, LOW); //右电机后退禁止
    analogWrite(Right_motor_pwm, control);

    //delay(time);
}

/**
 * Function       spin_right
 * @author        Danny
 * @date          2017.07.25
 * @brief         //原地右转(右轮后退，左轮前进)
 * @param[in]     time
 * @param[out]    void
 * @retval        void
 * @par History   无
 */

void spin_right()
{
    //左电机前进
    digitalWrite(Left_motor_go, HIGH);   //左电机前进使能
    digitalWrite(Left_motor_back, LOW);  //左电机后退禁止
    analogWrite(Left_motor_pwm, control);

    //右电机后退
    digitalWrite(Right_motor_go, LOW);    //右电机前进禁止
    digitalWrite(Right_motor_back, HIGH); //右电机后退使能
    analogWrite(Right_motor_pwm, control);

    //delay(time);
}


/**
 * Function       back
 * @author        Danny
 * @date          2017.07.25
 * @brief         小车后退
 * @param[in]     void
 * @param[out]    void
 * @retval        void
 * @par History   无
 */
//后退
void back()
{
    //左电机后退
    digitalWrite(Left_motor_go, LOW);     //左电机前进禁止
    digitalWrite(Left_motor_back, HIGH);  //左电机后退使能
    analogWrite(Left_motor_pwm, control);

    //右电机后退
    digitalWrite(Right_motor_go, LOW);     //右电机前进禁止
    digitalWrite(Right_motor_back, HIGH);  //右电机后退使能
    analogWrite(Right_motor_pwm, control);

}

/**
 * Function       whistle
 * @author        liusen
 * @date          2017.07.25
 * @brief         鸣笛
 * @param[in1]    void
 * @retval        void
 * @par History   无
 */

void whistle()   //旋转鸣笛
{
    int i;
    for (i = 0; i < 10; i++) //输出一个频率的声音
    {
        digitalWrite(buzzer, LOW); //发声音
        delay(10);//延时1ms
        digitalWrite(buzzer, HIGH); //不发声音
        delay(1);//延时ms
    }
    for (i = 0; i < 5; i++) //输出另一个频率的声音
    {
        digitalWrite(buzzer, LOW); //发声音
        delay(20);//延时2ms
        digitalWrite(buzzer, HIGH); //不发声音
        delay(2);//延时2ms
    }
}

/**
 * Function       color_led
 * @author        Danny
 * @date          2017.07.25
 * @brief         由R,G,B三色的不同组合形成7种不同的色彩
 * @param[in1]    Red开关
 * @param[in2]    Green开关
 * @param[in3]    Blue开关
 * @retval        void
 * @par History   无
 */

void corlor_led(int v_iRed, int v_iGreen, int v_iBlue)
{
    //红色LED
    if (v_iRed == ON)
    {
        digitalWrite(LED_R, HIGH);
    }
    else
    {
        digitalWrite(LED_R, LOW);
    }
    //绿色LED
    if (v_iGreen == ON)
    {
        digitalWrite(LED_G, HIGH);
    }
    else
    {
        digitalWrite(LED_G, LOW);
    }
    //蓝色LED
    if (v_iBlue == ON)
    {
        digitalWrite(LED_B, HIGH);
    }
    else
    {
        digitalWrite(LED_B, LOW);
    }
}
/**
 * Function       front_detection
 * @author        Danny
 * @date          2017.07.25
 * @brief         云台复位
 * @param[in]     void
 * @retval        void
 * @par History   无
 */
void front_detection()
{
    //此处循环次数减少，为了增加小车遇到障碍物的反应速度
    for (int i = 0; i <= 15; i++) //产生PWM个数，等效延时以保证能转到响应角度
    {
        //舵机旋转到90度,即前方
        myservo.write(90);
    }
}
/**
 * Function       left_detection
 * @author        Danny
 * @date          2017.07.25
 * @brief         云台左旋
 * @param[in]     void
 * @retval        void
 * @par History   无
 */
void left_detection()
{
    for (int i = 0; i <= 15; i++) //产生PWM个数，等效延时以保证能转到响应角度
    {
        //舵机旋转到180度,即左侧
        myservo.write(180);
    }
}
/**
 * Function       right_detection
 * @author        Danny
 * @date          2017.07.25
 * @brief         云台右旋
 * @param[in]     void
 * @retval        void
 * @par History   无
 */
void right_detection()
{
    for (int i = 0; i <= 15; i++) //产生PWM个数，等效延时以保证能转到响应角度
    {
        //舵机旋转到0度,即右侧,测距
        myservo.write(0);
    }
}


void IR_Deal()
{
    if (irrecv.decode(&results))
    {
        //Serial.println(results.value, HEX);


        //根据不同值来执行不同操作
        //  00FF00FF  开关
        //  00FF30CF   +
        //  00FF708F   -
        //  00FFA05F     beep
        //  00FF807F     前进
        //  00FF20DF     左转
        //  00FF00FF     停止
        //  00FF609F     右转
        //  00FF906F     后退
        //  00FF10EF     左旋
        //  00FF50AF     右旋
        //  00FFB04F     0
        //  00FF08F7     1
        //  00FF8877     2
        //  00FF48B7     3
        //  00FF28D7     4
        //  00FFA857     5
        //  00FF6897     6
        //  00FF18E7     7
        //  00FF9867     8
        //  00FF58A7     9    

        switch (results.value)
        {
        case 0X00FF00FF: g_carstate = enSTOP;  break;
        case 0x00FF40BF: g_colorlight++; if (g_colorlight > 9) g_colorlight = 0; break;
        case 0x00FF18E7: left_detection(); break;
        case 0x00FF9867: front_detection(); break;
        case 0x00FF58A7: right_detection(); break;
        default: break;
        }

        switch (results.value)
        {
        case 0x00FF30CF: control += 50; if (control > 255) control = 255; break;
        case 0x00FF708F: control -= 50; if (control < 50) control = 100; break;
        case 0x00FFA05F: whistle(); break;
        case 0x00FF807F:  g_carstate = enRUN; break;
        case 0x00FF20DF:  g_carstate = enLEFT; break;
        case 0x00FF00FF:  g_carstate = enSTOP; break;
        case 0x00FF609F:  g_carstate = enRIGHT; break;
        case 0x00FF906F:  g_carstate = enBACK; break;
        case 0x00FF10EF:  g_carstate = enTLEFT; break;
        case 0x00FF50AF:  g_carstate = enTRIGHT; break;
        default: break; //保持原来状态

        }


        last = millis();
        irrecv.resume(); // 接收下一个编码
    }
    else if (millis() - last > 120)
    {
        g_carstate = enSTOP;
        last = millis();
    }

}

void loop()
{
    IR_Deal();
    switch (g_colorlight)
    { 
    case 0: corlor_led(OFF, OFF, OFF); break;  
    case 1: corlor_led(ON, ON, ON); break;
    case 2: corlor_led(ON, OFF, OFF); break;
    case 3: corlor_led(OFF, ON, OFF); break;
    case 4: corlor_led(OFF, OFF, ON); break;
    case 5: corlor_led(OFF, ON, ON); break;
    case 6: corlor_led(ON, OFF, ON); break;
    case 7: corlor_led(ON, ON, OFF); break;
    case 8: corlor_led(OFF, OFF, OFF); break;
    }

    switch (g_carstate)
    {
    case enSTOP: brake(); break;
    case enRUN: run(); break;
    case enLEFT: left(); break;
    case enRIGHT: right(); break;
    case enBACK: back(); break;
    case enTLEFT:spin_left(); break;
    case enTRIGHT:spin_right();break;
    default: brake(); break;
    }
}


