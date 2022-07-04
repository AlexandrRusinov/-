#include <NewPing.h> //Библиотека Ультразвукового датчика (нужно установить)
#include <Servo.h>
#include<FastLED.h>

#define DIR_RR 6 // управлять направлением вращения правого мотора будем с контакта 2
#define DIR_FR 5 // управлять разрешением вращения и скоростью вращения правого //мотора будем с контакта 3
#define DIR_RL 3 //управлять направлением вращения левого мотора будем с контакта 4
#define DIR_FL 11 // управлять разрешением вращения и скоростью вращения левого //мотора будем с контакта 5
// для оптического датчика
const int prx_pin = 4;
byte v;
bool light = false;
// How many leds in your strip?
#define NUM_LEDS 7
#define LED_PIN 10
// Define the array of leds
CRGB leds[NUM_LEDS];

//
//Пины ультразвукового датчика
#define trig_pin 7 //Аналоговый вход 1
#define echo_pin 8 //Аналоговый вход 2
#define maximum_distance 200
char income_symbol;
int i = 0; //Случайная переменная, назначенная циклам
int j = 0; //Случайная переменная, назначенная циклам
int state; //Переменная сигнала от устройства Bluetooth
int vSpeed = 255; // Стандартная скорость может принимать значение от 0-255
boolean goesForward = false;
int distance = 100;
int value;
//В этой части кода больше не будем задавать ни каких параметров
NewPing sonar(trig_pin, echo_pin, maximum_distance); //sensor function
Servo servo_motor; //our servo name
// приступаем ко второй части программы. Мы знаем, что в этой части кода команды //исполняются только один раз


void setup()
{
    Serial.begin(19200);
    servo_motor.attach(9); //Пин подключения сервомотора
    servo_motor.write(90);
    FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
    // яркость светодиодов (макс значение 255) менять можно в любом месте программы
    FastLED.setBrightness(50);
    pinMode(DIR_RR, OUTPUT); // Драйвер управляется выходными сигналами с Ардуино.
    //Поэтому мы определяем все контакты , как OUTPUT
    pinMode(DIR_FR, OUTPUT);
    pinMode(DIR_RL, OUTPUT);
    pinMode(DIR_FL, OUTPUT);
    pinMode(trig_pin, OUTPUT);
    pinMode(echo_pin, INPUT);
}

void loop()
{
    //servo_motor.write(90);
    lightCheck();
    //readPing(); // чтобы расстояние чекалось всегда
    while (Serial.available() > 0) { // если что-то вводится

        income_symbol = Serial.read(); // считывание и инициализация в переменную
        Serial.println(income_symbol); // выводим считанный символ в монитор

        if (income_symbol == 'L') { Left(); return; }
        else if (income_symbol == 'R') { Right(); return; }
        else if (income_symbol == 'RevRight') { TurnRight(); return; }
        else if (income_symbol == 'RevLeftt') { TurnLeft(); return; }
        else if (income_symbol == 'F') { Forward(); return; }
        else if (income_symbol == 'B') { Reverse(); return; }
    }
}


int readPing() {
    int duration, cm; // назначаем переменную "cm" и "duration" для показаний датчика
    digitalWrite(trig_pin, LOW); // изначально датчик не посылает сигнал
    delay(20); // ставим задержку в 2 ммикросекунд

    digitalWrite(trig_pin, HIGH); // посылаем сигнал
    delay(100); // ставим задержку в 10 микросекунд
    digitalWrite(trig_pin, LOW); // выключаем сигнал

    duration = pulseIn(echo_pin, HIGH); // включаем прием сигнала

    cm = duration / 58; // вычисляем расстояние в сантиметрах

    Serial.print(cm); // выводим расстояние в сантиметрах
    Serial.println(" cm");

    delay(1000); // ставим паузу в 1 секунду
    return cm;

}

void moveStop() {

    digitalWrite(DIR_FR, LOW);
    digitalWrite(DIR_FL, LOW);
    digitalWrite(DIR_RR, LOW);
    digitalWrite(DIR_RL, LOW);
    Serial.println("stop");
}

void lightCheck() {

    // открываем порт для оптического датчика
    pinMode(prx_pin, INPUT);
    // принимаем данные
    v = digitalRead(prx_pin);
    // условие на наличие тупика
    if (v == 1) {
        // сюда заходит, есс
        // тут должен светодиод гореть
        // первый пиксель (мб который по середине) будет синий
        leds[0] = CHSV(0, 0, 0);
        leds[2] = CHSV(0, 0, 0);
        leds[1] = CHSV(0, 0, 0);
        leds[4] = CHSV(0, 0, 0);
        leds[6] = CHSV(0, 0, 0);
        // отправляем информациюю на ленту
        FastLED.show();
        light = false;
    }
    else {
        light = true;
        leds[0] = CHSV(0, 0, 255);
        leds[2] = CHSV(0, 0, 255);
        leds[1] = CHSV(0, 0, 255);

        leds[4] = CHSV(0, 0, 255);
        leds[6] = CHSV(0, 0, 255);
        FastLED.show();

    }

}

void Forward() {
    Serial.println("forward");
    for (int i = 70; i <= vSpeed; i++)
    {
        lightCheck();
        if (light)
        {
            moveStop();
            break;
        }
        analogWrite(DIR_FR, i);
        analogWrite(DIR_FL, i);
        delay(30);

    }
    digitalWrite(DIR_FL, LOW);
    digitalWrite(DIR_FR, LOW);
}

void Reverse() {
    Serial.println("Back");
    digitalWrite(DIR_RL, HIGH);
    digitalWrite(DIR_RR, HIGH);
    delay(750);
    digitalWrite(DIR_RL, LOW);
    digitalWrite(DIR_RR, LOW);
}

void Right() {
    Serial.println("Right");
    digitalWrite(DIR_FR, HIGH);
    digitalWrite(DIR_FL, LOW);
    delay(500);
    digitalWrite(DIR_FR, LOW);
    digitalWrite(DIR_FL, LOW);
}

void Left() {
    Serial.println("Left");
    digitalWrite(DIR_FL, HIGH);
    digitalWrite(DIR_FR, LOW);
    delay(500);
    digitalWrite(DIR_FR, LOW);
    digitalWrite(DIR_FL, LOW);
}

void TurnRight() {
    digitalWrite(DIR_FL, HIGH);
    digitalWrite(DIR_RR, HIGH);
    delay(385);
    digitalWrite(DIR_FL, LOW);
    digitalWrite(DIR_RR, LOW);
}

void TurnLeft() {
    digitalWrite(DIR_RL, HIGH);
    digitalWrite(DIR_FR, HIGH);
    delay(310);
    digitalWrite(DIR_RL, LOW);
    digitalWrite(DIR_FR, LOW);
}


