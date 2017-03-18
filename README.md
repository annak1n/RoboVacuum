# RoboVacuum
Project which aims to use a raspberry pi to control a robot vacuum cleaner. The base chasis is a Roboclean next generation, there are many generic robots using the same model and can be picked up quite cheap second hand. The majority of electronics were stripped and replaced with the Raspbery Pi 3, some i2c hardware, and two new batteries.

## Part lists:
- Roboclean
- raspberry PI 3 model B
- Adafruit BNO055
- 2 X Adafruit motor and stepper hat
- ADC PI Plus (ABE electonics)
- Sharp IR (10-80cm) distance sensor
- 3000mAh battery (JST-XH-2P) for RPI
- 5000mAh battery (Hyperion G5 50C 3S 5000mAh LiPo Battery) for motors
- PowerBoost 1000 Charger


![Robo Clean](images/robo_clean.jpg?raw=true "Roboclean next generation")

After disassembly via removing all screws on bottom of robot and then removing the lid. All cirtcuit boards should be removed. From the "main" circuit board the contacts for the dustbin need to be cut off and stored. Finally the front IR proximity sensor in the lid should be removed.

![Robo Clean](images/original_guts.jpg?raw=true "Roboclean next generation")

## Structural modification:

The 5000mAh battery is about the same volume as the original 2400mAh. Unfortunatly it is not the same in dimension, so a section of the battery bay of the chasis needs to be removed, aswell as the square pannel which did nothing. The pannel could be glued back on place, the screws are what interfere with the battery. This side was chosed as it did not interfere with the side brush motor.

![Robo Clean](images/structure/battery_cut1.jpg?raw=true "Roboclean next generation")


![Robo Clean](images/structure/battery_cut2.jpg?raw=true "Roboclean next generation")

Three modifications are required for the lid, a large rectangular hole should be cut for the RPI and two small slots for slide switches. The location of the slide switch holes are not important, but the chosen location is close to the battery terminals. The lid is made of a softer plastic compared to the chasis (PE?), driling holes and then cutting with a stanley knife was suffient. Also marking with a knife and bending until it breaks was useful for adjusting the square hole size.

![Robo Clean](images/structure/lid_cut.jpg?raw=true "Roboclean next generation")

The location where the lipo's charger cable is should be determined aproixamlty and a hole cut in the lid to allow it through. I got lucky as for me it was under the handle, so when the charger is not in use I can hide the hole and the cable.




## Assembly
    After removing the main board and cutting all the holes the eletronics can now be added. This can be seperated into two stages, the lid and the chasis.
### Lid
The lid first needs to have the Sharp IR sensor added, in the place of the original fron sensor. There is a little bit of adjustment required to get it flush with the front. As the plastic is soft you can do this with pliers.

![Robo Clean](images/assembly/dist_sensor1.jpg?raw=true "Roboclean next generation")

![Robo Clean](images/assembly/dist_sensor2.jpg?raw=true "Roboclean next generation")

The stored contacts from the main board need some wires soldered onto them and can be mounted again in the same possition. The wires need to be long enough to get to a motor contoller.

![Robo Clean](images/assembly/dustbin_contact.jpg?raw=true "Roboclean next generation")

Two slide switched need to be glued or screwed into the cut slots.

### chasis
The 5000mAh battery can be slid into place, I found that some doublesided foam tape was very practical for restraining the battery. Along the back of the chasis the raspberry pi plus papy(i?)rus modules are screwed in on the left, then the motor controller and the ADC going along to the right. The BNO055 is stuck on some foam tape in what by eye looked like the center of the robot. All devices had 5 pin cables attached for 5v, 3v3, gnd, SCL and, SDA of the I2C port, this all went to a single bus containing all of these connectors. The Papyrus was also conected to the bus and and was the link between the Pi and the rest. 


![Robo Clean](images/assembly/chasis1.jpg?raw=true "Roboclean next generation")

The switches in the lid were attached to some prototyping cable (i.e. those used with a breadboard) on the battery side. The pins were bent back around forming a very crude plug which could be inserted into the battery sockets.

### general tips
I tried to put detachable connections between all devices connecting between the lid and the chassis. These are the following pairs:
- The ADC and IR sensor
- The Dustbin motor contact and motor controller
- The Motor contrllers and switched
- the battery and switches

As this is all a work in progress some new modifications can be added, also some old ones might become redundant.