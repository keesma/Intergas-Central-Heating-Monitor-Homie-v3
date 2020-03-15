# Intergas-Central-Heating-Monitor-Homie-v3
Intergas Central heating monitor for homie version 3

This is an update of the previous Intergas Central Heating Monitor for homie version 2. The functionality is almost the same and now supports auto discovery. The software is monitoring my Intergas Prestige CW6 without any issues for many years.

* This program can read the the status of a Central heating from Intergas.
  It has been built for an esp8266. The central heating status is sent through MQTT to a central system (MQTT broker).
  
* Configuration is done using the configuration mode Homie. The wifi network identification and password is entered at first startup or when the device is reset.

* How to connect the esp8266 to the Intergas (is still the same as previous version)

  I have included a diagram how to connect the optocouplers (and a picture of the prototype):
  https://github.com/keesma/Intergas-Central-Heating-Monitor-Homie/blob/master/Intergas%20reader%20optocoupler.jpg

  To connect the central heating to the esp8266.
  It is best to use an optocoupler to connect the esp8266 to the Intergas.
  E.g. an 4n25 can be used (take two 4n25s to protect both tx and rx).
  I got good results by using a 220 Ohm resistor for the input and a 1k Ohm resistor in the output.

  Default config on the esp8266 is:
  - pin 4: Rx
  - pin 5: Tx
  - pin 12: LED. The LED is on during initialization and while sending data to the Intergas.
  The communication speed is 9600 baud.

  The intergas has a 4 pin plug with: Vcc, ground, Tx and Rx.
  
  Not that there is an alternative scheme which is slightly different.

* Dependencies
  - Homie 3.0: https://github.com/homieiot/homie-esp8266
 
   Homie has great features and is working very well.
  
* Openhab: I have connected the esp8266 through MQTT to openhab. Openhab can display the data, save it and create nice graphs. The item definitions are included. The rules are required for translating the status bytes to (bit) values.
https://github.com/keesma/Intergas-Central-Heating-Monitor-Homie/tree/master/openhab

  Openhab now also does auto discovery of homie devices since support for homie version 3.
  In the subdirectory openhab an example item file is included and a sample rules file. These have to be tuned to your needs (which fields do you need, will extract the fields in the device or in the server.

* Power: I now use a D1 mini pro and a simple USB power supply.

* Changes compared to previous version (based on Homie) v2:
  - Each property has a descriptive label and type and if appropriate also a unit.
  - More status bits are send in seaparate flags. You can of course comment the properties you do not need.
  

