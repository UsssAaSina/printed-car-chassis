// tachanka ver.0.0.6
// Target - Arduino Uno (ATMega328P)
/*  Система динамической стабилизации четырехколесного шасси с независимыми или зависимые через осевой дифференциал колесами.
    Корректировка частот вращения колес происходит тормозной системы. Контроль частот вращения колес осуществляется датчиками
    вращения (Датчик Холла TLE4945 или другим способом).
    Корректировка вращения колес, происходи из расчёта, что угловые скорости движения колесам по дугам радиусу равны.
    Алгоритм предусматривает само калибровку системы.
    В версии 0.0.5
    Отличается полностью от предыдущей. И переписывается заново.
*/
#define TEST
#define ERROR_bl  Serial.println("ERROR_bl");
#define ERROR_br  Serial.println("ERROR_br");
#define ERROR_fl  Serial.println("ERROR_br");
#define ERROR_fr  Serial.println("ERROR_fr");
#define ERROR_NaN  Serial.println("ERROR_NaN");//
#define ERROR_wheel Serial.println("ERROR_wheel");
// дописать функции обработки ошибок

// блок констант характерезующий параметры шасси данные в мм.Поправки к радиусам размерность х100
//════════════════════════════════════════════════════════╗
const int16_t  weelbase            = 300; // колесная база                    (по ТЗ bPfP)          ║
const int16_t  backtrack           = 200; // задняя колея                     (по ТЗ blfr)          ║
int16_t HalfBacktrack = 100; //половина задней колеи
uint8_t  rdWhell_back_left   =  100; // поправка к радиусу  колеса заднего левого     (по ТЗ rWbl)          ║
uint8_t  rdWhell_back_right  =  100; // поправка к радиусу  колеса заднего правого    (по ТЗ rWbr)          ║
uint8_t  rdWhell_front_left  =  100; // поправка к радиусу  колеса переднего левого   (по ТЗ rWfl)          ║
uint8_t  rdWhell_front_right =  100; // поправка к радиусу  колеса переднего правого  (по ТЗ rWfr)          ║
uint16_t  rdWhell             =  25; // средний радиус колес размерноть ХЗ                    (по ТЗ rW)           ║
uint32_t weelbaseSQ = 90000; //квадрат длинны базы. Вычисляется автоматически.                                 ║
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
const int16_t RANGE_R = INT16_MAX - backtrack ;
int16_t /*float R */   R               = RANGE_R; //RANGE_R ; // заданный радиус поворота радиус           ¦
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
  HalfBacktrack = backtrack / 2;

  weelbaseSQ =  uint32_t (weelbase) * (weelbase);
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

  PPM_status ^= 1;       // 0b00000001     // применяем побитовый XOR, 1 такт
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
//возвращает радиуса в величинах backtrack  по умолчанию в мм, и переключает
//флаг левый/правый текущего поворота
//пока без учета поправок на не идеальность колес!!!!
uint16_t read_radius() {
  int32_t Rv;
  Rv = tWbr - tWbl; // вычитаем
  if (Rv == 0) {
    //проверка на нол.
    return RANGE_R;
  }
  if (Rv > 0) {
    Rv = tWbl / Rv * backtrack; LR = true;
  }
  else {
    Rv = Rv * (-1);
    Rv = tWbr / Rv * backtrack; LR = false;
  }
  Rv = (Rv + backtrack / 2) ;
  if (Rv > RANGE_R) {
    return (RANGE_R);
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
  const int32_t ForFlaf = 2000000; // ожидание колес
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
  /* tWbl = 0;
    tWbr = 0;
    tWfl = 0;
    tWfr = 0;*/
  uint32_t   OldtWbl                  = 0; // фактическое время одного оборота заднего левого  колеса
  uint32_t   OldtWbr                  = 0; // фактическое время одного оборота  заднего правого колеса
  //  uint32_t   OldtWfl                  = 0; // фактическое время одного оборота переднего левого колеса
  //  uint32_t   OldtWfr                  = 0; // фактическое время одного оборота  переднего правого колеса

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
  uint32_t Sum = (SumWbl + SumWbr + SumWfl + SumWfr)  / 4 * 100;
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

//unint16_t (0-65535) DutyCycle =PWR_Duty/PWR_Pulse*100 (0-10000)
uint16_t DutyCycle() {
  return uint16_t((PWR_Duty * 100) / PWR_Pulse );
  // порядок вычисление именно такой.
}//

int16_t A = 0; //кожфициенты уровнения 1000000/(x*a+b) ВНИМАНИЕ ОЦЕНИТЬ ПОВТОРНО РАЗРЯДНОСТЬ ЧИСЕЛ
int16_t B = 0; //кожфициенты уровнения 1000000/(x*a+b) ВНИМАНИЕ ОЦЕНИТЬ ПОВТОРНО РАЗРЯДНОСТЬ ЧИСЕЛ
uint16_t NoR_MAX = 0;//недопустимое значения  DutyCycle при которых результат превышает int16_t
uint16_t NoR_MIN = 0;//недопустимое значения  DutyCycle при которых результат превышает int16_t
// данные для проверки
int16_t AS[] = {500, 510, 520, 530, 540, 550, 560, 570, 580, 590, 600, 610, 620, 630, 640, 650, 660, 670, 680, 690, 700, 710, 720, 730, 740, 750, 760, 770, 780, 790, 800, 810, 820, 830, 840, 850, 860, 870, 880, 890, 900, 910, 920, 930, 940, 950, 960, 970, 980, 990, 1000};
int16_t ASR[] = {177, 185, 194, 204, 215, 227, 240, 255, 272, 291, 313, 338, 367, 402, 443, 493, 555, 633, 735, 874, 1072, 1377, 1901, 2995, 6515, 14170, 6515, 2995, 1901, 1377, 1072, 874, 735, 633, 555, 493, 443, 402, 367, 338, 313, 291, 272, 255, 240, 227, 215, 204, 194, 185, 177};
boolean ASB[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

void CalibrationTurn () {// намеренно выбран тип переменных, что бы замедлить процесс выполнения
  int8_t    TimeCalibration = 50;//число изменений
  int32_t Flag_1 = 2000000; //время ожидания
  float SumDC = 0; //сумма входящих Duty Cycle (DC)
  float SumRdForLR = 0; // сумма R^-1
  float SumDCsqForLR = 0; //сумма DC^2
  float SumDCtoRForLR = 0; //сумма (DC/R)
  uint16_t N = 0; //число замеров
  uint16_t oldR = 0; //предыдущее значение радиуса
  uint16_t DC = 0;
  float a = 0;
  float b = 0;

  while (TimeCalibration) {//набор стат данных в случаи изменения радиуса
    R = read_radius(); //включить
    DC = DutyCycle();//включить
#ifdef TEST
    R = ASR[TimeCalibration];//отключить
    DC = AS[TimeCalibration];//отключить
    LR = ASB[TimeCalibration];//отключить
#endif
    if ((R - oldR)) {
      SumDC += float (DC); //сумма входящих Duty Cycle (DC)
      SumDCsqForLR += float(DC) * float(DC); //сумма DC^2
      if (LR) {
        SumRdForLR += float(1.00 / float(R)); // сумма R^-1
        SumDCtoRForLR += float(DC) / float(R); //сумма (DC/R)
      }
      else {
        SumRdForLR -= float(1.00 / float(R)); // сумма R^-1
        SumDCtoRForLR -= float(DC ) / float(R); //сумма (DC/R)
      }
      N++;//число замеров
      TimeCalibration--;
      oldR = R;
      Flag_1++;
    }
    Flag_1--;
    if (!Flag_1) {
      ERROR_wheel
    }  //сообщаем, что не получаемм данных DC
  }
  b = (SumRdForLR * SumDCsqForLR - SumDC * SumDCtoRForLR) * 1000000 / (N * SumDCsqForLR - SumDC * SumDC);
  a  = ( SumDCtoRForLR - SumDC  * b) / SumDCsqForLR;
  A = int16_t (lrint(a));
  B = int16_t (lrint(b));
  int16_t NoR = 0; // недопустимое значения  DutyCycle при которых ДЕЛО
  if (A == 0) {
    ERROR_NaN
  } else {
    NoR = -B / A;
    int16_t Add = (1000000 / (RANGE_R) / A);
    NoR_MAX = NoR + Add;
    NoR_MIN = NoR - Add;
  }
}

void DCtoR() {//вычисление радуса поворота
  uint16_t DC = DutyCycle();
  if ( DC >= NoR_MIN && DC <= NoR_MAX ) {
    R = RANGE_R ;
    LR = true;
  }
  else {
    R = 1000000 / (int16_t(DC) * A + B);
    if (R < 0) {
      LR = false;
      R = abs(R);
    }
    else {
      LR = true;
    }
  }
}

//блок переменных характерезующие расчетные параметры траектории в мм
//═════════════════════════════════
uint32_t  Rbl   = 0; // расчтеный радиус траектории внешнего заднего колеса
uint32_t  Rbr   = 0; // расчетный радиус траектории внутренного заднего колеса
uint32_t  Rfl  = 0; // расчтеный радиус траектории внешнего переднего колеса
uint32_t  Rfr  = 0; // расчтеный радиус траектории внутренного переднего колеса
uint32_t  Rmm = 0;
//========================================================                                                                                              ║
void radiusOfWeelPaths () { //вычисления Радиусов траекторий колес при заданном Радиусе попворота
  Rmm = uint32_t  (R);
  if (LR) {

    Rbr = Rmm - HalfBacktrack;
    Rbl = Rmm + HalfBacktrack;
  }
  else {
    Rbl = Rmm - HalfBacktrack;
    Rbr = Rmm + HalfBacktrack;
  }

  Rfr = sqrt( (sq(Rbr)) + (weelbaseSQ));
  Rfl = sqrt( (sq(Rbl)) + (weelbaseSQ));
  
}

void loop() {
 CalibrationTurn ();
  PWR_Duty = 510;//510
  while (PWR_Duty < 1000) {//1000
    PWR_Pulse = 100;
    DCtoR();
    //Serial.println (R);
    PWR_Duty++;
 delay(100);
    radiusOfWeelPaths ();
  }
  delay(100000);

}
