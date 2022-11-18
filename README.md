# nRF24L01-Nucleo
PROJECT SUMMARY :

   This system is designed for a Olive Picking Machine. Firstly , I am reading 12x software and hardware filtered analog variable comes from like a multi-turn potantiometer. They give us position information of steering wheel and beaters. After reading the values, we have 16x High or Low side PWM output with diagnostics to control the lenght of machine , steering wheels, position of the beaters and position of the machine itself according to the trees. Secondly, I choose to use a DIY RF controller to control the machine parts separately. In RC(Remote Controller), I am planing to send 2 digital signal in addition to the data package. One of is Overwrite Signal to make an overwrite for the values that comes from the machine parts and the other one is Voltage-Detect Signal for understanding if the RC is opened or not. Lastly, I recorded that when the tranciever part is starting to run in begining, the values and data packeges are coming randomly for a moment. Hence, I can put some delay but I am going to use 2 more digital signals on tranciever side for the same reasons with the RC part and to prevent random machine movements. 
   As it is seen above, I am using analog pins , timer pins for pwm, SPI pins for RF Controller. To give one more little information for the machine, it consists of only electrical and hydraulical systems.
   
   How to Use : 
   
![image](https://user-images.githubusercontent.com/70060259/202440002-ebf6e4ef-6710-4a84-a548-7e4e312a3843.png)

This figure shows us pinout, connectivities , I/Os and timer pins of master nucleo.

For Analog Read with Polling Method :

![image](https://user-images.githubusercontent.com/70060259/202443992-4f841dae-44bc-45b8-8416-4d41a7c209b7.png)

After doing pin configuration, write select functions which have polling parameters for all ADC pins.

![image](https://user-images.githubusercontent.com/70060259/202444356-ec6dc64d-3ee5-43b8-b2e1-e7f5a2476f3e.png)

Write one more READ function to read analog value separately. The continuous conversion mode must be disabled, because we want to read analog values sequentially and don't want them to get mixed up when reading 12 values which are different each others. Just select the ADC channel, start adc read, write poll for per conversation function, get the value that you want and stop the ADC read. In order to this method, the maximum ADC channels of the microprocessor can be read without any mess. 

IIR FILTER : 


![image](https://user-images.githubusercontent.com/70060259/202680050-323401ef-40e8-4595-adf1-2c3fba4ce295.png)

To apply IIR filter to the analog value that comes from potantiometer, you can use http://www.winfilter.20m.com.



The figure that below the 3rd order IIR FILTER code that generates by winfilter program. You can create more function like this. In this case, you can filter all of your variables by one by.

![image](https://user-images.githubusercontent.com/70060259/202813790-e1f87e63-2067-4343-b97c-c2e5efc17ae9.png)





For nRF Configuration :

![image](https://user-images.githubusercontent.com/70060259/202441793-84e26366-b4e8-43e0-bb7c-66462d769d2e.png)   ![image](https://user-images.githubusercontent.com/70060259/202442646-756b91be-5f5f-408a-8cae-f005e26ae262.png)


SPI cofiguration is here. The baudrate is crucial for both RX and TX parts. Hence, baudrate must be the same both RX and TX, even if you use different clock frequencies. You need 2 more output pins for chip select and chip enable. You can see the figures above.



Write the init and necessary functions. For this release , RF's are using 1 pipe.

![image](https://user-images.githubusercontent.com/70060259/202816711-24f20107-9bab-48b9-bd5e-06e1757dc779.png)
![image](https://user-images.githubusercontent.com/70060259/202816967-aff36336-2ed4-41b0-8541-30d0487266af.png)


In infinite loop, we can start our functions and RX-TX process like that. You can think to use a struct and a object to perform dynamic payloads. The figure below shows how to use this structure.

![image](https://user-images.githubusercontent.com/70060259/202817323-65bccb2f-f1c2-43c6-af7a-e9fa0cd56f9c.png)

I have only told the transmitter part until here. RX part is very similar to TX part. Hence, I am just sharing the files for now.


NOTE:

This is not the whole project. I am dealing with issues migrating the project to FREERTOS. For this project, hardware implementation and PCB work are in progress. After finishing the hardware design, I will share in here.


For all questions or further contact : alialper97@gmail.com



