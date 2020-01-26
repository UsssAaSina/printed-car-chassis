// tachanka ver.0.0.5
// Target - Arduino Uno (ATMega328P)
/*  Система динамической стабилизации четырехколесного шасси с независимыми или зависимые через осевой дифференциал колесами. 
 *  Корректировка частот вращения колес происходит тормозной системы. Контроль частот вращения колес осуществляется датчиками 
 *  вращения (Датчик Холла TLE4945 или другим способом).
 *  Корректировка вращения колес, происходи из расчёта, что угловые скорости движения колесам по дугам радиусу равны.
 *  Алгоритм предусматривает само калибровку системы. 
 *  В версии 0.0.5
 *  Отличается полностью от предыдущей. И переписывается заново.
*/

// блок констант переменных счётчиков вращения колес и сигналов ШИМ
//==========================================================
volatile int32_t  rotSensor_back_left_Timer   =  0; //таймер датчика вращения колеса заднего левого    ¦
volatile int32_t  rotSensor_back_right_Timer  =  0; //таймер датчика вращения колеса заднего правого   ¦
volatile int32_t  rotSensor_front_left_Timer  =  0; //таймер датчика вращения колеса переднего левого  ¦
volatile int32_t  rotSensor_front_right_Timer =  0; //таймер датчика вращения колеса переднего правого ¦

volatile int32_t  PWR_TIMER = 0; //таймер длительность сигнала ШИМ                                     ¦
volatile int32_t  PWR_Pulse = 0; //длительность частоты сигнала ШИМ
volatile int32_t  PWR_Duty = 0;  // длительность сигнала ШИМ
int8_t            PPM_status = 0;
//==========================================================

//блок переменных характеризующие состояние шасси размерность частот сек/оборот
//========================================================
volatile float   tWbl                  = 0; // фактическое время одного оборота заднего левого  колеса   ¦
volatile float   tWbr                  = 0; // фактическое время одного оборота  заднего правого колеса   ¦
volatile float   tWfl                  = 0; // фактическое время одного оборота переднего левого колеса  ¦
volatile float   tWfr                  = 0; // фактическое время одного оборота  переднего правого колеса ¦
int16_t /*float R */   R               = 65535; //65535; // заданный радиус поворота радиус 0 бесконечный радиус          ¦
boolean LR                             = true; // направления поворота где true правый, false левый          ¦
// При int16_tРадиус поворота в метрах (целое число от 65535м до 1 метра)                           ¦
// можно изменить тип данных переменных, но это повлечет увеличение объема данных скорость работы                   ¦
//========================================================

// блок переменных функции rotationSenaor
//========================================================¬
// bit 4: FR, bit 5: FL, bit 6: RR, bit 7: RL
volatile uint8_t DataRotSensor = 0b00000000; // 8 битная переменная. Куда с порта ввода D (пин с 7-0) считываем¦
volatile uint8_t N;  // Переменная для чтения битов в  DataRotSensor                                                                  ¦
//========================================================

// блок констант настройки пинов входов от дачиков вращения колес
//========================================================
const int8_t  rotSensor_back_left_Pin   =  7; // Пин датчика вращения колеса заднего левого 
const int8_t  rotSensor_back_right_Pin  =  6; // Пин датчика вращения колеса заднего правого
const int8_t  rotSensor_front_left_Pin  =  5; // Пин датчика вращения колеса переднего левого
const int8_t  rotSensor_front_right_Pin =  4; // Пин датчика вращения колеса переднего правого
const int8_t  rotSensor_All             =  2; // Пин 2 со всех датчиков сигнал дублируется на него
const int8_t  PWRDAT_Pin                =  3; // пин 3 подключение ШИМ от сервы

//========================================================
int16_t d = 0;
int16_t b = 0;

void initPins() {//инициализачция входов от дачиков вращения колес
  pinMode (rotSensor_back_left_Pin, INPUT); //инициализация входов от датчиков вращения колес
  pinMode (rotSensor_back_right_Pin, INPUT); //инициализация входов от датчиков вращения колес
  pinMode (rotSensor_front_left_Pin, INPUT); //инициализация входов от датчиков вращения колес
  pinMode (rotSensor_front_right_Pin, INPUT); //инициализация входов от датчиков вращения колес
  pinMode (rotSensor_All, INPUT); //инициализация пина прерывания со всех датчиков
  pinMode (PWRDAT_Pin, INPUT); //инициализация пина ввода ШИМ
  //включение потягивающего резистра
  digitalWrite (rotSensor_back_left_Pin, HIGH);
  digitalWrite  (rotSensor_back_right_Pin, HIGH);
  digitalWrite  (rotSensor_front_left_Pin, HIGH);
  digitalWrite  (rotSensor_front_right_Pin, HIGH);
  digitalWrite  (rotSensor_All, HIGH);
  digitalWrite  (PWRDAT_Pin, HIGH);
} // initPins

void setup() {
  Serial.begin(230400);
  initPins(); //функция инициализации Pin
  cli();//отключаем прерывание

  //настройка таймера 2
  TCCR2A = 0; //обнулили
  TCCR2B = 0; //обнулили
  TIMSK2 = (1 << TOIE1);
  TCCR2B |= (1 << CS10); //настроили без деления 255*6.25е-8 сек.
  // TODO: применить https://playground.arduino.cc/Main/PinChangeInterrupt/https://playground.arduino.cc/Main/PinChangeInterrupt/
  attachInterrupt(digitalPinToInterrupt(rotSensor_All), isr_wheel, RISING);// актевирует срабатывания функции rotationSenaor на пине 2 изменение с 0, на 1
  attachInterrupt(digitalPinToInterrupt(PWRDAT_Pin), isr_ppm, CHANGE);// актевирует срабатывания функции PWRDAT_H на пине 3 изменение с 0, на 1
  
  sei();//разрешили прерывание
  PPM_status = digitalRead(PWRDAT_Pin); // текущее состояние берём из состояния пина
}

// Проверка 
void isr_ppm() {
  if (PPM_status & 1) { // 
    PWR_Pulse = PWR_TIMER; // запоминаем время между импульсами
    PWR_TIMER = 0; // обнуляем таймер    
  } else {              // 
    PWR_Duty = PWR_TIMER; //запоминаем время продолжительности импульса  
  }
  
  PPM_status ^ 1;       // 0b00000001     // применяем побитовый XOR, 1 такт 
}

/* void PWRDAT_H () {//обработка начала импульса ШИМ
  PWR_Pulse = PWR_TIMER; // запоминаем время между импульсами
  PWR_TIMER = 0; // обнуляем таймер
  attachInterrupt(1, PWRDAT_L, FALLING);// активирует срабатывания функции PWRDAT_L на пине 3 изменение с 0, на 1
}

void PWRDAT_L () {//обработка спада импульса ШИМ
  PWR_Duty = PWR_TIMER; //запоминаем время продолжительности импульса
  attachInterrupt(1, PWRDAT_H, RISING);// активирует срабатывания функции PWRDAT_H на пине 3 изменение с 1, на 0
} */

void isr_wheel () {//функция работы с датчиками скоростей вращения колес по прерыванию на пине 2 (прерывание 0 )
  DataRotSensor = PIND;
  switch (DataRotSensor & (1 << N)) {
    case 7: tWbl = rotSensor_back_left_Timer; rotSensor_back_left_Timer = 0;
      break;
    case 6: tWbr = rotSensor_back_right_Timer; rotSensor_back_right_Timer = 0;
      break;
    case 5: tWfl = rotSensor_front_left_Timer; rotSensor_front_left_Timer = 0;
      break;
    case 4: tWfr = rotSensor_front_right_Timer; rotSensor_front_right_Timer = 0;
      break;  
  }
}

ISR (TIMER2_OVF_vect) { //по переполнению таймера срабатывает прерывание каждые 15 мкс
  rotSensor_back_left_Timer++ ;//таймер датчика вращения колеса заднего левого
  rotSensor_back_right_Timer++;//таймер датчика вращения колеса заднего правого
  rotSensor_front_left_Timer++;//таймер датчика вращения колеса переднего левого
  rotSensor_front_right_Timer++;//таймер датчика вращения колеса переднего правого
  PWR_TIMER++;//таймер длительность сигнала ШИМ
}

void loop() {
  Serial.print (b-b);
}
