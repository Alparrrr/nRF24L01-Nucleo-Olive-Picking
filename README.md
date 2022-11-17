# nRF24L01-Nucleo
PROJECT SUMMARY :

   This system is designed for a Olive Picking Machine. Firstly , we are reading 12x analog variable like a potantiometer. They give us position information of steering wheel and beaters. After reading the values, we have 16x PWM output to control the lenght of machine , steering wheels, position of the beaters and the machine according to the trees. Secondly, we are choose using a RF remote controller to control the machine parts separately. In RC(Remote Controller), we are planing to send 2 digital signal in addition to the data package. One of is Overwrite Signal to make an overwrite for the values that comes from the machine parts and the other one is Voltage-Detect Signal for understanding if the RC is opened or not. Lastly, we recoded that when the tranciever part is starting to run in begining, the values and data packeges are coming randomly for a moment. Hence, We can put some delay but we are going to use 2 more digital signals on tranciever side for the same reasons with the RC part and to prevent random machine movements. As it is seen above, we are using analog pins , timer pins for pwm, SPI pins for RF Controller.To give one more little information for the machine, it consists of only electrical and hydraulical systems.
   
   How to Use : 
   
![image](https://user-images.githubusercontent.com/70060259/202440002-ebf6e4ef-6710-4a84-a548-7e4e312a3843.png)

This figure shows us pinout, connectivities , I/Os and timer pins of master nucleo.

For Analog Read with Polling method :

![image](https://user-images.githubusercontent.com/70060259/202443992-4f841dae-44bc-45b8-8416-4d41a7c209b7.png)

After doing pin configuration, write select functions which have polling parameters for all ADC pins.

![image](https://user-images.githubusercontent.com/70060259/202444356-ec6dc64d-3ee5-43b8-b2e1-e7f5a2476f3e.png)

Write one more READ function to read analog value separately. The continuous conversion mode must be disabled, because we want to read analog values sequentially and don't want them to get mixed up when reading 12 values. Just select the ADC channel, start adc read, write poll for per conversation function, get the value that you want and stop the adc read.






For nRF Configuration :

![image](https://user-images.githubusercontent.com/70060259/202441793-84e26366-b4e8-43e0-bb7c-66462d769d2e.png)   ![image](https://user-images.githubusercontent.com/70060259/202442646-756b91be-5f5f-408a-8cae-f005e26ae262.png)


SPI cofiguration is here. The baudrate is crucial for both RX and TX parts. Hence, baudrate must be the same both RX and TX, even if you use different clock frequencies. You need 2 more output pins for chip select and chip enable. You can see the figures above.
