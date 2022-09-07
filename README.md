# Работа с сервоприводом SG90 на STM32 F411 discovery
____
  Сервопривод SG90 управляется ШИМ сигналом, параметры которого определяют положение ротора. В случае сервопривода SG90 период ШИМ равен 20 милисекунд соответственно частота 50 Гц. Длина импульса примерно 500-2700 микросекунд, где 500 микросекунд соответсвует 0 градусов, а 2700 мс - 180 градусов.
  
  ![servo](https://user-images.githubusercontent.com/73960471/188959963-2e7a01a5-a23a-4160-a630-79065c58d188.png)


### Настройка ШИМ в CubeMX
____ 
  Настраиваем ШИМ исходя из требуемых параметров. Для работы с ШИМ в STM32 используются таймеры через которые можно генерировать ШИМ сигнал. 
Выбираем любой из встроенных таймеров, выбираем канал, и находим к какой ножке он подключен. В моем случае таймер TIM1 канал CH2, подключен к ножке PE11

![image](https://user-images.githubusercontent.com/73960471/188961364-de5d5a39-368c-4beb-afca-f5606d5acc10.png)

  Далее необходимо определить исходную частоту таймера, для этого заходим в datasheet нашего микроконтроллера и находим блок-схему. Здесь мы видим, что наш таймер подключен к шине APB2
  
  ![image](https://user-images.githubusercontent.com/73960471/188962259-1aa2e6e8-994a-4355-b07c-c1dbde184b14.png)

Далее заходим в CubeMX и смотрим частоту на шине APB2 - 96 МГц

![image](https://user-images.githubusercontent.com/73960471/188962500-4d463900-f32c-4e33-a0ee-3f2c64e61c79.png)
____ 

Теперь определим параметры таймера. Его частота определяется по формуле Frequency = timer clock / prescaler * counter period.
timer clock мы узнали  - 96 МГц, counter period для удобства можно взять 20000, частота нам нужна 20 Гц, тогда prescaler = timer clock / Frequency * counter period
считаем и получаем prescaler = 96. Но в CubeMX нужно записать значение PSC - 1, то есть 95

![image](https://user-images.githubusercontent.com/73960471/188964024-63ec14a7-aa63-4af5-a3d0-2464536421f5.png)

Генерируем код и открываем main.c здесь после инициализации нужно вызвать функцию:
```С
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
```
Далее в регистр канала можно записывать число, определяющее скважность в диапазоне от 0 до 20000. Для удобного управления сервоприводом написал следующую функцию:
```С
int Set_Servo_Angle(uint8_t Angle) // from 0 to 180 degrees
{
  uint16_t Pulse_length = 500;
  if (Angle > 0) {
      Pulse_length += (2700-500)/180 * Angle;
  }
  
  TIM1->CCR2=Pulse_length;
  return 0;
}
```




