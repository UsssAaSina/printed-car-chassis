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
#define ERROR_bl  Serial.println("ERROR_bl");
#define ERROR_br  Serial.println("ERROR_br");
#define ERROR_fl  Serial.println("ERROR_br");
#define ERROR_fr  Serial.println("ERROR_fr");
// дописать функции обработки ошибок

// блок констант характерезующий параметры шасси данные в мм.
//════════════════════════════════════════════════════════╗
const int16_t  weelbase            = 300; // колесная база                    (по ТЗ bPfP)          ║
const int16_t  backtrack           = 200; // задняя колея                     (по ТЗ blfr)          ║
uint16_t  rdWhell_back_left   =  25; // радиус колеса заднего левого     (по ТЗ rWbl)          ║
uint16_t  rdWhell_back_right  =  25; // радиус колеса заднего правого    (по ТЗ rWbr)          ║
uint16_t  rdWhell_front_left  =  25; // радиус колеса переднего левого   (по ТЗ rWfl)          ║
uint16_t  rdWhell_front_right =  25; // радиус колеса переднего правого  (по ТЗ rWfr)          ║
uint16_t  rdWhell             =  25; // средний радиус колес                    (по ТЗ rW)           ║
//float weelbaseSQ; //квадрат длинны базы. Вычисляется автоматически.                                 ║
//════════════════════════════════════════════════════════╝

// блок констант переменных счётчиков вращения колес и сигналов ШИМ
//==========================================================
volatile uint32_t  rotSensor_back_left_Timer   =  0; //таймер датчика вращения колеса заднего левого    ¦
volatile uint32_t  rotSensor_back_right_Timer  =  0; //таймер датчика вращения колеса заднего правого   ¦
volatile uint32_t  rotSensor_front_left_Timer  =  0; //таймер датчика вращения колеса переднего левого  ¦
volatile uint32_t  rotSensor_front_right_Timer =  0; //таймер датчика вращения колеса переднего правого ¦

volatile uint32_t  PWR_TIMER = 0; //таймер длительность сигнала ШИМ                                     ¦
volatile uint32_t  PWR_Pulse = 0; //длительность частоты сигнала ШИМ
volatile uint32_t  PWR_Duty = 0;  // длительность сигнала ШИМ
int8_t            PPM_status = 0;
//==========================================================

//блок переменных характеризующие состояние шасси размерность частот сек/оборот
//========================================================
// (0- 65535)размерность (0,01х об/сек)
uint16_t   Wbl                  = 0; // фактическая частота вращения заднего левого  колеса   ║
uint16_t   Wbr                  = 0; // фактическая частота вращения заднего правого колеса   ║
uint16_t   Wfl                  = 0; // фактическая частота вращения переднего левого колеса  ║
uint16_t   Wfr                  = 0; // фактическая частота вращения переднего правого колеса ║

volatile uint32_t   tWbl                  = 0; // фактическое время одного оборота заднего левого  колеса   ¦
volatile uint32_t   tWbr                  = 0; // фактическое время одного оборота  заднего правого колеса   ¦
volatile uint32_t   tWfl                  = 0; // фактическое время одного оборота переднего левого колеса  ¦
volatile uint32_t   tWfr                  = 0; // фактическое время одного оборота  переднего правого колеса ¦
int16_t /*float R */   R               = 65535; //65535; // заданный радиус поворота радиус           ¦
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


//Функция чтения радиуса поворота по соотношению периодов вращения read_radius()
//возвращает радиуса в величинах backtrack /1000 по умолчанию в м, и переключает
//флаг левый/правый текущего поворота
//пока без учета поправок на не идеальность колес!!!!
uint16_t read_radius() {
  int32_t Rv;
  Rv = tWbr - tWbl; // вычитаем
  if (Rv == 0) {
    Rv = 1;  //проверка на нол.
    return 65535;
  }
  if (Rv > 0) {
    Rv = tWbl / Rv * backtrack; LR = true;
  }
  else {
    Rv = Rv * (-1);
    Rv = tWbr / Rv * backtrack; LR = false;
  }
  Rv = (Rv + backtrack / 2) / 1000;
  if (Rv > 65535) {
    return (65535);
  };
  return (int16_t(Rv));
}


//
// (62745 тиков-примерно одна секунда)
// если мы ориентируемся примерно на 1сек, то порядок частот примерно от 0-70 оборотов в секунду
// то uint16_t, 0- 65535 вполне может характеризовать наши частоты, от 0 до 655,35 это об/сек максимальное 39321 об/мин
// Предполагается, что колеса вращаются не с постоянной частотой. Если они решат крутится с постоянной частотой то это фиаско.
void CalibrationWhels () { //взаимная калибровка радиусов, колес по соотношению частот вращения при движении по прямой.
  const uint16_t   oneSecond = 62745; //число тиков в одной секунде
  uint8_t    TimeCalibration = 20;    // число оборотов на основании которых вычисляется соотношение колес
  int8_t Flag_1 = 4;                  // флаг подсчета изменения оборотов
  const int32_t ForFlaf=2000000;// ожидание колес
  int32_t Flag_2 = ForFlaf;           // ожидание колес
  uint32_t   SumWbl                  = 0; // Сумма частот вращения заднего левого  колеса
  uint32_t   SumWbr                  = 0; // Сумма частот вращения заднего правого колеса
  uint32_t   SumWfl                  = 0; // Сумма частот вращения переднего левого колеса
  uint32_t   SumWfr                  = 0; // Сумма частот вращения переднего правого колеса
  //ждем, пока все колеса не начнут крутится с частотой более 1герц
  
  while (tWbl < oneSecond ) {
    if (!Flag_2) {
      ERROR_bl
    };
    --Flag_2;
  }
  Flag_2 = ForFlaf;
  while (tWbr < oneSecond ) {
    if (!Flag_2) {
      ERROR_br
    }
    --Flag_2;
  }
  Flag_2 = ForFlaf;
  while (tWfl < oneSecond ) {
    if (!Flag_2) {
      ERROR_fl
    }
    --Flag_2;
  }
  Flag_2 = ForFlaf;
  while (tWfr < oneSecond ) {
    if (!Flag_2) {
      ERROR_fr
    }
    --Flag_2;
  }
  tWbl = 0;
  tWbr = 0;
  tWfl = 0;
  tWfr = 0;
  uint32_t   OldtWbl                  = tWbl; // фактическое время одного оборота заднего левого  колеса
  uint32_t   OldtWbr                  = tWbr; // фактическое время одного оборота  заднего правого колеса
  uint32_t   OldtWfl                  = tWfl; // фактическое время одного оборота переднего левого колеса
  uint32_t   OldtWfr                  = tWfr; // фактическое время одного оборота  переднего правого колеса

  while (TimeCalibration) {//наблюдаем за изменением периода это гарантирует, что мы не накопим одно измерение периода
    if (OldtWbl - tWbl) {
      Flag_1--;
    }
    if (OldtWbr - tWbr) {
      Flag_1--;
    }
    if (OldtWbl - tWbl) {
      Flag_1--;
    }
    if (OldtWbr - tWbr) {
      Flag_1--;
    }
    if (!Flag_1) {
      SumWbl += oneSecond / tWbl;
      SumWbr += oneSecond / tWbr;
      SumWfl += oneSecond / tWfl;
      SumWfr += oneSecond / tWfr;
      TimeCalibration--;
    }

  }
  uint32_t Sum = (SumWbl + SumWbr + SumWfl + SumWfr) * rdWhell / 4;
  if (!SumWbl) {
    Wbl = Sum / SumWbl;
  } else {
    ERROR_bl
  }
  if (!SumWbr) {
    Wbr = Sum / SumWbr;
  } else {
    ERROR_br
  }
  if (!SumWfl) {
    Wfl = Sum / SumWfl;
  } else {
    ERROR_fl
  }
  if (!SumWfr) {
    Wfr = Sum / SumWfr;
  } else {
    ERROR_fr
  }
}



void loop() {
  
}
