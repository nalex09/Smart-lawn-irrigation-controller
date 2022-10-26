# Smart-lawn-irrigation-controller
The realized project involves the design and implementation of a solution for lawn irrigation
controller based on AVR MCU ATmega328P. The project involved the integration of
hardware components with software to get a working system. In the making of this project it was
used hardware components like microcontroller, relays, soil moisture sensors, liquid crystal display, HC-06 Bluetooth module,
mechanical swiches, etc. To integrate this hardware modules with software it was used
components like Analog-Digital converter, Digital Input/Outpus interface and UART protocol.  
  This lawn irrigation controller allow control of three dinstinct irrigation areas. Also, the sistem allows three operating modes: automatic mode, manual mode and Bluetooth mode.   
   In the automatic mode, the sistem works on the behalf of soil-resistant moisture sensors. In the case of the soil moisture detected in one of the areas is lower than the reference value implemented in the software, the system will decide to start irrigation in that area. If not, such as in an area the value of soil moisture is higher than the reference value, the system will decide the stop of the irrigation.  
   In manual operating mode, the user can turn on the irrigation separately, on each zone. This   operation is done through the use of some switches placed on the system's control panel.   
   Compared to the other operating modes, Bluetooth mode allows the control of the status of the mobile phone by the Bluetooth communication. This operating mode is similar to the manual control mode, the difference being that in the Bluetooth mode the control of the irrigation zones is done from an smartphone-based application.   
   The system owns too a display panel consisting of an LCD display on which the status of the system can be monitorized. On this display is showed the data about how the system works and the status of each irrigation area. The resulting system is a simple system, easy to implement and not expensive compared to the systems on the market, while also allowing further developments. 
## Proteus wiring diagram
![proteus wiring diagram](https://user-images.githubusercontent.com/57752680/198001166-ea8e64c5-f652-4f0e-bcec-6045b21ec465.png)
## Circuit testing on breadboard
![image](https://user-images.githubusercontent.com/57752680/198001465-d1467dd4-6809-41f6-8475-00ed25baeec1.png)
![image](https://user-images.githubusercontent.com/57752680/198001555-50d15205-0c33-4ad5-876c-0cdae77f8548.png)
![image](https://user-images.githubusercontent.com/57752680/198001580-611a0015-c424-46a6-bf8c-8f03954a0a4f.png)
![image](https://user-images.githubusercontent.com/57752680/198001596-0a27e209-8ede-47f6-a5b7-d44a00e4a036.png)
## Final circuit
![image](https://user-images.githubusercontent.com/57752680/198001646-89ee3547-a2d2-4cd7-a2f7-1c4d2fe1b82a.png)

