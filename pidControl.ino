// PID 控制参数 (!!! 这是需要你花时间耐心整定的关键 !!!)
double Kp = 150  ;     // 比例系数 -- 先调我
double Ki = 1;     // 积分系数 -- 再调我
double Kd = 70 ;      // 微分系数 -- 最后调我

// PID 核心变量
double error = 0;
double last_error = 0;
double integral = 0;
double derivative = 0;
double correction = 0;

// 传感器引脚定义
int sensorPins[] = {A0, A1, A2, A3, A4};
int sensorValues[5];

// 小车基础速度 (0-255)
int baseSpeed = 200 ;

void setup() {
  // put your setup code here, to run once:
  pinMode(A0, INPUT);//定义从左往右第一个红外循迹传感器模块引脚为A0
  pinMode(A1, INPUT);//定义从左往右第二个红外循迹传感器模块引脚为A1
  pinMode(A2, INPUT);//定义从左往右第三个红外循迹传感器模块引脚为A2
  pinMode(A3, INPUT);//定义从左往右第四个红外循迹传感器模块引脚为A3
  pinMode(A4, INPUT);//定义从左往右第五个红外循迹传感器模块引脚为A4

  pinMode(7, OUTPUT);//定义数字引脚7为左电机方向控制脚
  pinMode(4, OUTPUT);//定义数字引脚4为右电机方向控制脚
}

void loop() {
  // put your main code here, to run repeatedly:
  // --- 输入模型 ---
  // 1. 读取传感器数据并计算误差
  calculate_error();

 // --- 处理模型 ---
  // 2. 进行PID计算，得到修正量
  calculate_pid();

  // --- 输出模型 ---
  // 3. 根据修正量控制电机转速
  control_motors();

  // 打印调试信息
  // Serial.print("Error: "); Serial.print(error);
  // Serial.print("  Correction: "); Serial.println(correction);

  delay(10); // 控制循环频率
}

void calculate_error() {
  // 读取5个传感器的值 (黑线为1, 白地为0)
  int totalWeight = 0;
  int activeSensors = 0;
  int weights[] = {-2, -1, 0, 1, 2}; // 传感器的权重

  for (int i = 0; i < 5; i++) {
    sensorValues[i] = digitalRead(sensorPins[i]);
    if (sensorValues[i] == HIGH) { // 只计算在黑线上的传感器
      totalWeight += weights[i];
      activeSensors++;
    }
  }

  // 计算误差
  if (activeSensors > 0) {
    // 加权平均法
    error = (double)totalWeight / activeSensors;
    last_error = error; // 只有找到线的时候才更新last_error
  } else {
    // 丢线情况: 根据上一次的误差方向来决定转向
    if (last_error > 0) {
      error = 2; // 上次在右边丢线，强制一个大的右偏误差，让车左转找线
    } else {
      error = -2; // 上次在左边丢线，强制一个大的左偏误差，让车右转找线
    }
  }
}

void calculate_pid() {
  // P - 比例项
  double proportional = Kp * error;

  // I - 积分项 (带积分限幅，防止饱和)
  integral += error;
  if (integral > 200) integral = 200;
  if (integral < -200) integral = -200;
  
  // D - 微分项
  derivative = error - last_error;

  // 最终修正量
  correction = proportional + (Ki * integral) + (Kd * derivative);
}

void control_motors() {
  // 计算左右轮的最终速度
  // 当 error > 0 (偏右) 时, correction > 0, 需要左转 (左轮减速, 右轮加速)
  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  // 速度限制，确保在 0-255 范围内
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // 应用速度到电机
  analogWrite(5, leftSpeed);
  digitalWrite(7,HIGH);//设定左边电机为高电平，前进
  analogWrite(6, rightSpeed);
  digitalWrite(4,HIGH);//设定右边电机为高电平，前进
}