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
int distance = 0;
/*超声波引脚及变量设置*/
int EchoPin = 12;         //Echo回声脚
int TrigPin = 13;         //Trig触发脚

//循迹红外引脚定义
//TrackSensorLeftPin1 TrackSensorLeftPin2 TrackSensorRightPin1 TrackSensorRightPin2
//      A2                  A1                  A3                   A4
const int TrackSensorLeftPin1  =  A2;  //定义左边第一个循迹红外传感器引脚为A2
const int TrackSensorLeftPin2  =  A1;  //定义左边第二个循迹红外传感器引脚为A1
const int TrackSensorRightPin1 =  A3;  //定义右边第一个循迹红外传感器引脚为A3
const int TrackSensorRightPin2 =  A4;  //定义右边第二个循迹红外传感器引脚为A4

//定义各个循迹红外引脚采集的数据的变量
bool TrackSensorLeftValue1;
bool TrackSensorLeftValue2;
bool TrackSensorRightValue1;
bool TrackSensorRightValue2;

/*避障红外传感器引脚及变量设置*/
const int AvoidSensorLeft =  A3;   //定义左边避障的红外传感器引脚为A3
const int AvoidSensorRight = A1;   //定义右边避障的红外传感器引脚为A1
const int FollowSensorLeft =  A3;   //定义左边跟随的红外传感器引脚为A3
const int FollowSensorRight = A1;   //定义右边跟随的红外传感器引脚为A1

int LeftSensorValue ;              //定义变量来保存红外传感器采集的数据大小
int RightSensorValue ;

/*定义光敏电阻引脚及变量设置*/
const int LdrSensorLeft =  A4;   //定义左边光敏电阻引脚为A4
const int LdrSensorRight = A2;   //定义右边光敏电阻引脚为A2

int LdrSersorLeftValue ;         //定义变量来保存光敏电阻采集的数据大小
int LdrSersorRightValue ;

/*颜色识别引脚及变量设置*/
int LDR_pin = A5;

//第二位 1:左90 ; 2: 右90; 3: 180度  4:直行
char mapLocation[38][2] = {
    {0, 2},  {1, 1},  {2, 2}, {3, 3},
    {4, 2},  {5, 4},  {6, 3},
    {7, 2},  {8, 2},  {9, 2}, {10, 3},
    {11, 4}, {12, 4}, {13, 3},
    {14, 2}, {15, 1}, {16, 1}, {17, 3},
    {18, 2}, {19, 1}, {20, 3},
    {21, 1}, {22, 1}, {23, 1}, {24, 2}, {25, 3},
    {26, 2}, {27, 2}, {28, 3},
    {29, 2}, {30, 2}, {31, 3},
    {32, 2}, {33, 1}, {34, 3},
    {35, 1}, {36, 2}, {37, 3}
};
int position = 0; //位置定义


/*协议用到变量*/
int CarSpeedControl = 150;//PWM控制量
int g_carstate = enSTOP; //  1前2后3左4右0停止
int g_colorlight = 0;
int g_modeSelect = 0;  //0是默认状态;  1:红外遥控 2:巡线模式 3:超声波避障 4: 七彩探照 5: 寻光模式 6: 红外跟踪

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
    pinMode(EchoPin, INPUT);   //定义超声波输入脚
    pinMode(TrigPin, OUTPUT);  //定义超声波输出脚

    Serial.begin(9600);	//波特率9600 （蓝牙通讯设定波特率）

    //pinMode(Fire, OUTPUT);   // 定义灭火输出

    //初始化RGB三色LED的IO口为输出方式
    pinMode(LED_R, OUTPUT);
    pinMode(LED_G, OUTPUT);
    pinMode(LED_B, OUTPUT);
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, LOW);

    //定义四路循迹红外传感器为输入接口
    pinMode(TrackSensorLeftPin1, INPUT);
    pinMode(TrackSensorLeftPin2, INPUT);
    pinMode(TrackSensorRightPin1, INPUT);
    pinMode(TrackSensorRightPin2, INPUT);

    //定义红外寻光管脚为输入模式
    pinMode(AvoidSensorLeft, INPUT);
    pinMode(AvoidSensorRight, INPUT);
    pinMode(FollowSensorLeft, INPUT);
    pinMode(FollowSensorRight, INPUT);
    pinMode(LdrSensorLeft, INPUT);
    pinMode(LdrSensorRight, INPUT);

    //设置舵机控制引脚为3
    myservo.attach(servopin);
    irrecv.enableIRIn(); // 初始化红外解码
    pinMode(RECV_PIN, INPUT_PULLUP);     //将2号管脚设置为输入并且内部上拉模式
}

/**
 * Function       Distance_test
 * @author        Danny
 * @date          2017.07.26
 * @brief         超声波测距
 * @param[in]     void
 * @param[out]    void
 * @retval        void
 * @par History   无
 */
void Distance_test()
{
    digitalWrite(TrigPin, LOW);               //给触发脚低电平2μs
    delayMicroseconds(2);
    digitalWrite(TrigPin, HIGH);              //给触发脚高电平10μs，这里至少是10μs
    delayMicroseconds(10);
    digitalWrite(TrigPin, LOW);
    float Fdistance = pulseIn(EchoPin, HIGH); // 读取高电平时间(单位：微秒)
    Fdistance = Fdistance / 58;
    //  Serial.print("Distance:");            //输出距离（单位：厘米）
    //  Serial.print(Fdistance);              //显示距离
    //  Serial.println("cm");
    distance = Fdistance;
    return;
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
    analogWrite(Left_motor_pwm, CarSpeedControl);

    //右电机前进
    digitalWrite(Right_motor_go, HIGH);  //右电机前进使能
    digitalWrite(Right_motor_back, LOW); //右电机后退禁止
    analogWrite(Right_motor_pwm, CarSpeedControl);
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
    analogWrite(Left_motor_pwm, CarSpeedControl);

    //右电机前进
    digitalWrite(Right_motor_go, HIGH);  //右电机前进使能
    digitalWrite(Right_motor_back, LOW); //右电机后退禁止
    analogWrite(Right_motor_pwm, CarSpeedControl);

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
    analogWrite(Left_motor_pwm, CarSpeedControl);

    //右电机后退
    digitalWrite(Right_motor_go, LOW);    //右电机前进禁止
    digitalWrite(Right_motor_back, HIGH); //右电机后退使能
    analogWrite(Right_motor_pwm, CarSpeedControl);

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
    analogWrite(Left_motor_pwm, CarSpeedControl);

    //右电机后退
    digitalWrite(Right_motor_go, LOW);     //右电机前进禁止
    digitalWrite(Right_motor_back, HIGH);  //右电机后退使能
    analogWrite(Right_motor_pwm, CarSpeedControl);

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
 * Function       color_led_pwm
 * @author        Danny
 * @date          2017.07.25
 * @brief         七彩灯亮指定的颜色
 * @param[in1]    v_iRed:指定的颜色值（0-255）
 * @param[in2]    v_iGreen:指定的颜色值（0-255）
 * @param[in3]    v_iBlue:指定的颜色值（0-255）
 * @param[out]    void
 * @retval        void
 * @par History   无
 */
void color_led_pwm(int v_iRed, int v_iGreen, int v_iBlue)
{
    analogWrite(LED_R, v_iRed);
    analogWrite(LED_G, v_iGreen);
    analogWrite(LED_B, v_iBlue);
    delay(100);
    return;
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

/**
 * Function       color_test
 * @author        Danny
 * @date          2017.07.26
 * @brief         颜色识别测试
 * @param[in]     void
 * @param[out]    void
 * @retval        iIntensity:灰度值
 * @par History   无
 */
int color_test()
{
    int iIntensity;
    pinMode(LDR_pin, INPUT_PULLUP);          //灭火引脚和灰度传感器引脚复用A5口,需通过引脚设置进行分时复用
    iIntensity = analogRead(LDR_pin); //读取模拟口A5的值，存入Intensity变量
    //  Serial.print("Intensity = "); //串口输出"Intensity = "
    //  Serial.println(iIntensity);
    pinMode(LDR_pin, OUTPUT);
    return iIntensity;
}

/**
 * Function       color_dis
 * @author        liusen
 * @date          2017.08.26
 * @brief         颜色识别测试
 * @param[in]     void
 * @param[out]    void
 * @retval        void
 * @par History   无
 */
void color_dis()
{
    int Color = 0;
    Color = color_test();

    if (Color >= 370 && Color <= 410)
    {
        color_led_pwm(0, 0, 255);
    }
    else if (Color >= 300 && Color <= 320)
    {
        color_led_pwm(0, 255, 0);
    }
    else if (Color >= 345 && Color <= 365)
    {
        color_led_pwm(255, 0, 0);
    }
    else if (Color >= 330 && Color < 345)
    {
        color_led_pwm(255, 0, 255);
    }
    else if (Color >= 270 && Color <= 290)
    {
        color_led_pwm(0, 255, 255);
    }
    else if (Color >= 250 && Color < 270)
    {
        color_led_pwm(255, 255, 0);
    }
    else
    {
        color_led_pwm(0, 0, 0);
    }
}
/**
 * Function       track_get_value
 * @author        liusen
 * @date          2017.07.26
 * @brief         循迹模式引脚测试
 * @param[in]     void
 * @param[out]    void
 * @retval        void
 * @par History   无
 */
void track_get_value()
{
    TrackSensorLeftValue1  = digitalRead(TrackSensorLeftPin1);
    TrackSensorLeftValue2  = digitalRead(TrackSensorLeftPin2);
    TrackSensorRightValue1 = digitalRead(TrackSensorRightPin1);
    TrackSensorRightValue2 = digitalRead(TrackSensorRightPin2);
}

/********************************************************************************************************/
/*模式2 巡线*/
/**
 * Function       Tracking_Mode
 * @author        Danny
 * @date          2017.07.25
 * @brief         巡线
 * @param[in1]    void
 * @param[out]    void
 * @retval        void
 * @par History   无
 */
void Tracking_Mode()
{
    //检测到黑线时循迹模块相应的指示灯亮，端口电平为LOW
    //未检测到黑线时循迹模块相应的指示灯灭，端口电平为HIGH
    TrackSensorLeftValue1  = digitalRead(TrackSensorLeftPin1);
    TrackSensorLeftValue2  = digitalRead(TrackSensorLeftPin2);
    TrackSensorRightValue1 = digitalRead(TrackSensorRightPin1);
    TrackSensorRightValue2 = digitalRead(TrackSensorRightPin2);

    //四路循迹引脚电平状态
    // 0 0 X 0
    // 1 0 X 0
    // 0 1 X 0
    //以上6种电平状态时小车原地右转，速度为250,延时80ms
    //处理右锐角和右直角的转动
    if ( (TrackSensorLeftValue1 == LOW || TrackSensorLeftValue2 == LOW) &&  TrackSensorRightValue2 == LOW)
    {
        CarSpeedControl = 250;
        spin_right();
        delay(80);
    }
    //四路循迹引脚电平状态
    // 0 X 0 0
    // 0 X 0 1
    // 0 X 1 0
    //处理左锐角和左直角的转动
    else if ( TrackSensorLeftValue1 == LOW && (TrackSensorRightValue1 == LOW ||  TrackSensorRightValue2 == LOW))
    {
        CarSpeedControl = 250;
        spin_left();
        delay(80);
    }
    // 0 X X X
    //最左边检测到
    else if ( TrackSensorLeftValue1 == LOW)
    {
        CarSpeedControl = 150;
        spin_left();
        //delay(10);
    }
    // X X X 0
    //最右边检测到
    else if ( TrackSensorRightValue2 == LOW )
    {
        CarSpeedControl = 150;
        spin_right();
        //delay(10);
    }
    //四路循迹引脚电平状态
    // X 0 1 X
    //处理左小弯
    else if ( TrackSensorLeftValue2 == LOW && TrackSensorRightValue1 == HIGH)
    {
        CarSpeedControl = 220;
        left();
    }
    //四路循迹引脚电平状态
    // X 1 0 X
    //处理右小弯
    else if (TrackSensorLeftValue2 == HIGH && TrackSensorRightValue1 == LOW)
    {
        CarSpeedControl = 220;
        right();
    }
    //四路循迹引脚电平状态
    // X 0 0 X
    //处理直线
    else if (TrackSensorLeftValue2 == LOW && TrackSensorRightValue1 == LOW)
    {
        CarSpeedControl = 250;
        run();
    }
}
/********************************************************************************************************/
/*模式3:超声波避障模式*/
/**
 * Function       bubble
 * @author        Danny
 * @date          2017.07.26
 * @brief         超声波测五次的数据进行冒泡排序
 * @param[in1]    a:超声波数组首地址
 * @param[in2]    n:超声波数组大小
 * @param[out]    void
 * @retval        void
 * @par History   无
 */
void bubble(unsigned long *a, int n)

{
    int i, j, temp;
    for (i = 0; i < n - 1; i++)
    {
        for (j = i + 1; j < n; j++)
        {
            if (a[i] > a[j])
            {
                temp = a[i];
                a[i] = a[j];
                a[j] = temp;
            }
        }
    }
}
/**
 * Function       Distance
 * @author        Danny
 * @date          2017.07.26
 * @brief         超声波测五次，去掉最大值,最小值,
 *                取平均值,提高测试准确性
 * @param[in]     void
 * @param[out]    void
 * @retval        void
 * @par History   无
 */
void Distance()
{
    unsigned long ultrasonic[5] = {0};
    int num = 0;
    while (num < 5)
    {
        Distance_test();
        //过滤掉测试距离中出现的错误数据大于500
        while (distance >= 500)
        {
            brake();
            Distance_test();
        }
        ultrasonic[num] = distance;
        num++;
    }
    num = 0;
    bubble(ultrasonic, 5);
    distance = (ultrasonic[1] + ultrasonic[2] + ultrasonic[3]) / 3;
    return;
}
/**
 * Function       Ultrasonic_avoidMode
 * @author        Danny
 * @date          2017.07.26
 * @brief         超声波避障模式
 * @param[in]     void
 * @param[out]    void
 * @retval        void
 * @par History   无
 */
void Ultrasonic_avoidMode()
{
    Distance();
    printf("$4WD,CSB%d,PV8.3,GS000,LF0000,HW00,GM00#", distance);
    if (distance > 50)
    {
        CarSpeedControl = 200;
        run();      //当距离障碍物较远时全速前进
    }
    else if (distance >= 25 && distance <= 50)
    {
        CarSpeedControl = 100;
        run();      //当快靠近障碍物时慢速前进
    }
    else if (distance < 25)
    {
        CarSpeedControl = 160;
        spin_right();    //当靠近障碍物时原地右转大约90度
        delay(350);
        brake();
        delay(100);
        Distance();    //再次测试判断前方距离
        if (distance >= 25)
        {
            CarSpeedControl = 100;
            run();    //转弯后当前方距离大于25cm时前进
        }
        else if (distance < 25)
        {
            CarSpeedControl = 160;
            spin_left();    //转弯后前方距离小于25cm时向左原地转弯180度
            delay(700);
            brake();
            delay(100);
            Distance(); //再次测试判断前方距离
            if (distance >= 25)
            {
                CarSpeedControl = 100;
                run(); //转弯后当前方距离大于25cm时前进
            }
            else if (distance < 25)
            {
                CarSpeedControl = 160;
                spin_left();//转弯后前方距离小于25cm时向左原地转弯90度
                delay(350);
                brake();
                delay(100);
            }
        }
    }
}

/**
 * Function       BeepOnOffMode
 * @author        Danny
 * @date          2017.08.17
 * @brief         模式切换长鸣
 * @param[in]     void
 * @param[out]    void
 * @retval        void
 * @par History   无
 */
void BeepOnOffMode()
{
    pinMode(buzzer, OUTPUT);
    digitalWrite(buzzer, LOW);   //发声音
    delay(1000);                  //延时100ms
    digitalWrite(buzzer, HIGH);  //不发声音
}

/********************************************************************************************************/
/*模式:4  七彩颜色*/

void FindColor_Mode()
{
    track_get_value();
    if ( ((TrackSensorLeftValue1 == LOW || TrackSensorLeftValue2 == LOW) &&  TrackSensorRightValue2 == LOW) || ( TrackSensorLeftValue1 == LOW && (TrackSensorRightValue1 == LOW ||  TrackSensorRightValue2 == LOW)))
    {
        CarSpeedControl = 120;
        run();
        delay(60);
        brake();
        delay(200);
        //ModeBEEP(position);
        switch (mapLocation[position][1])
        {
        case 0: brake(); break;
        case 1: //左转90
                {
                    CarSpeedControl = 150;
                    spin_left();
                    delay(200);
                    while (1)
                    {
                        CarSpeedControl = 150;
                        spin_left();
                        track_get_value();
                        if (TrackSensorLeftValue2 == LOW ||  TrackSensorRightValue1 == LOW)
                        {
                            brake();
                            break;
                        }
                    }
                } break;
        case 2://右转90
                {
                    CarSpeedControl = 150;
                    spin_right();
                    delay(200);
                    while (1)
                    {
                        spin_right();
                        track_get_value();
                        if (TrackSensorLeftValue2 == LOW  ||  TrackSensorRightValue1 == LOW)
                        {
                            brake();
                            break;
                        }
                    }

                } break;
        case 3:
                {

                    brake();
                    delay(200);
                    BeepOnOffMode();
                    color_dis();
                    if (position == 37)
                    {
                        BeepOnOffMode();
                        position = 0;
                        //自动退出此模式 
                        g_modeSelect = 0;
                    }
                    CarSpeedControl = 150;
                    spin_left();
                    delay(200);
                    while (1)
                    {
                        spin_left();
                        track_get_value();
                        if (TrackSensorLeftValue2 == LOW ||  TrackSensorRightValue1 == LOW)
                        {
                            brake();
                            break;
                        }
                    }

                } break;
        case 4: CarSpeedControl = 160; run(); delay(50); break;//直行
        }
        position++;
        //while(1);
    }
    // 0 X X X
    //最左边检测到
    else if ( TrackSensorLeftValue1 == LOW)
    {
        CarSpeedControl = 150;
        spin_left();
    }
    // X X X 0
    //最右边检测到
    else if ( TrackSensorRightValue2 == LOW )
    {
        CarSpeedControl = 150;
        spin_right();
    }
    //四路循迹引脚电平状态
    // X 0 1 X
    //处理左小弯
    else if ( TrackSensorLeftValue2 == LOW && TrackSensorRightValue1 == HIGH)
    {
        CarSpeedControl = 150;
        left();
    }
    //四路循迹引脚电平状态
    // X 1 0 X
    //处理右小弯
    else if (TrackSensorLeftValue2 == HIGH && TrackSensorRightValue1 == LOW)
    {
        CarSpeedControl = 150;
        right();
    }
    else if (TrackSensorLeftValue2 == LOW && TrackSensorRightValue1 == LOW)
    {
        CarSpeedControl = 120;
        run();
    }
}



/********************************************************************************************************/
/*模式5:  寻光模式*/
void LightSeeking_Mode()
{
    //遇到光线,寻光模块的指示灯灭,端口电平为HIGH
    //未遇光线,寻光模块的指示灯亮,端口电平为LOW
    LdrSersorRightValue = digitalRead(LdrSensorRight);
    LdrSersorLeftValue  = digitalRead(LdrSensorLeft);
    CarSpeedControl = 250;
    printf("$4WD,CSB120,PV8.3,GS000,LF0000,HW00,GM%d%d#", LdrSersorLeftValue, LdrSersorRightValue);
    if (LdrSersorLeftValue == HIGH && LdrSersorRightValue == HIGH)
    {
        run();   //两侧均有光时信号为HIGH，光敏电阻指示灯灭,小车前进
    }
    else if (LdrSersorLeftValue == HIGH && LdrSersorRightValue == LOW)
    {
        left(); //左边探测到有光，有信号返回，向左转
    }
    else if (LdrSersorRightValue == HIGH && LdrSersorLeftValue == LOW)
    {
        right();//右边探测到有光，有信号返回，向右转
    }
    else
    {
        brake();//均无光，停止
    }
}
/********************************************************************************************************/
/*模式6: 红外跟随模式*/

void Ir_flow_Mode()
{
    //遇到跟随物,红外跟随模块的指示灯亮,端口电平为LOW
    //未遇到跟随物,红外跟随模块的指示灯灭,端口电平为HIGH
    LeftSensorValue  = digitalRead(FollowSensorLeft);
    RightSensorValue = digitalRead(FollowSensorRight);
    CarSpeedControl = 200;

    printf("$4WD,CSB120,PV8.3,GS000,LF0000,HW%d%d,GM00", LeftSensorValue, RightSensorValue);
    if (LeftSensorValue == LOW && RightSensorValue == LOW)
    {
        run();        //当两侧均检测到跟随物时调用前进函数
    }
    else if (LeftSensorValue == LOW && RightSensorValue == HIGH)
    {
        spin_left(); //左边探测到有跟随物，有信号返回，原地向左转
    }
    else if (RightSensorValue == LOW && LeftSensorValue == HIGH)
    {
        spin_right();//右边探测到有跟随物，有信号返回，原地向右转
    }
    else
    {
        brake();     //当两侧均未检测到跟随物时停止
    }
}

/**
 * Function       ModeBEEP
 * @author        Danny
 * @date          2017.08.17
 * @brief         模式显示函数
 * @param[in]     void
 * @param[out]    void
 * @retval        void
 * @par History   无
 */
void ModeBEEP(int mode)
{
    pinMode(buzzer, OUTPUT);
    for (int i = 0; i < mode + 1; i++)
    {
        digitalWrite(buzzer, LOW); //鸣
        delay(100);
        digitalWrite(buzzer, HIGH); //不鸣
        delay(100);
    }
    delay(100);
    digitalWrite(buzzer, HIGH); //不鸣
}

/**
 * Function       careRun
 * @author        lijing
 * @date          2019.03.19
 * @brief         小车前进(谨慎的，遇到障碍停止并转向)
 * @param[in]     void
 * @param[out]    void
 * @retval        void
 * @par History   无
 */
void careRun()
{
    front_detection(); //front
    Distance();

    if (distance > 20) run();
    else
    {
        right_detection(); //right
        Distance();
        if (distance > 20)
        {
            front_detection();
            spin_right();
            delay(200);
        }
        else
        {
            left_detection(); //left
            Distance();
            if (distance > 20)
            {
                front_detection();
                spin_left();
                delay(200);
            }
            else
                brake();
        }
    }
}


/**
 * Function       IR_Deal
 * @author        liusen
 * @date          2017.08.17
 * @brief         红外接收处理
 * @param[in]     void
 * @param[out]    void
 * @retval        void
 * @par History   无
 */
void IR_Deal()
{
    if (irrecv.decode(&results))
    {
        //Serial.println(results.value, HEX);


        //根据不同值来执行不同操作
        //  00FF00FF  开关
        //  00FF30CF   +
        //  00FF708F   -
        //  00FF40BF   点灯
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
        case 0X00FF00FF: g_carstate = enSTOP; g_modeSelect = 0; position = 0; break;
        case 0x00FF40BF: g_colorlight++; if (g_colorlight > 7) g_colorlight = 0; break;
        case 0x00FF18E7: left_detection(); break;
        case 0x00FF9867: front_detection(); break;
        case 0x00FF58A7: right_detection(); break;

        case 0x00FF08F7: g_modeSelect = 1; ModeBEEP(1); BeepOnOffMode();break; //模式1 默认红外遥控模式
        case 0x00FF8877: g_modeSelect = 2; ModeBEEP(2); BeepOnOffMode();break; //模式2 巡线模式
        case 0x00FF48B7: g_modeSelect = 3; ModeBEEP(3); BeepOnOffMode();break; //模式3: 避障模式
        case 0x00FF28D7: g_modeSelect = 4; ModeBEEP(4); BeepOnOffMode();break; //模式4: 七彩探照(游园赛道)
        case 0x00FFA857: g_modeSelect = 5; ModeBEEP(5); BeepOnOffMode();break; //模式5: 寻光模式
        case 0x00FF6897: g_modeSelect = 6; ModeBEEP(6); BeepOnOffMode();break; //模式6: 跟随模式

        case 0x00FF30CF: CarSpeedControl += 50; if (CarSpeedControl > 255) CarSpeedControl = 255; break; //加速速度控制
        case 0x00FF708F: CarSpeedControl -= 50; if (CarSpeedControl < 50) CarSpeedControl = 100; break;  //减速速度控制
        case 0x00FFA05F: whistle(); break;  //鸣笛
        case 0x00FF807F:  g_carstate = enRUN; break;   //前
        case 0x00FF20DF:  g_carstate = enLEFT; break;  //左

        case 0x00FF609F:  g_carstate = enRIGHT; break; //右
        case 0x00FF906F:  g_carstate = enBACK; break;  //后
        case 0x00FF10EF:  g_carstate = enTLEFT; break; //左
        case 0x00FF50AF:  g_carstate = enTRIGHT; break;//右
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

    // 切换不同功能模式, 功能模式显示
    if(g_modeSelect != 0)
    {
        if (g_modeSelect == 1) g_modeSelect = 0;
        else Ultrasonic_avoidMode();
#if 0
        switch (g_modeSelect)
        {
        case 1: g_modeSelect = 0; break; //红外遥控
        case 2: Tracking_Mode(); break; //巡线模式
        case 3: Ultrasonic_avoidMode();  break;  //超声波避障模式
        case 4: FindColor_Mode(); break;  //七彩颜色识别模式
        case 5: LightSeeking_Mode(); break;  //寻光模式
        case 6: Ir_flow_Mode(); break;  //跟随模式
        }
#endif
    }

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

    if(g_modeSelect == 0)
    {
        switch (g_carstate)
        {
        case enSTOP: brake(); break;
        case enRUN: careRun(); break;
        case enLEFT: left(); break;
        case enRIGHT: right(); break;
        case enBACK: back(); break;
        case enTLEFT:spin_left(); break;
        case enTRIGHT:spin_right();break;
        default: brake(); break;
        }
    }
}

