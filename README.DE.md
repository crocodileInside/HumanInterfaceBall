# HumanInterfaceBall
Ein Soft-Ball mit einer **Bosch BNO055** IMU und einem **Bosch BME280** Umweltsensor zur Maussteuerung via Bluetooth 4.0LE.  


## Hardware  
Basiert auf dem RedBearLabs BLE Nano v2 (**nRF52832**) als Microcontroller und Bluetooth-Modul.

## Software  
Entwickelt in Arduino. Anleitung zur Einrichtung der Entwicklungsumgebung, siehe unten.


### Installationsanleitung 
Um mit Arduino den Controller programmieren zu können, muss mit folenden Schritten die Entwicklungsumgebung eingerichtet werden:

1. Herunterladen und installieren der [Arduino IDE](https://www.arduino.cc/en/Main/Software) >=1.6.12 (getestet mit 1.6.12)  
2. Die Erweiterung `arduino-nRF5` nach Anleitung installieren: https://github.com/sandeepmistry/arduino-nRF5
3. Den Inhalt des Ordners `Arduino/libraries/` in den eigenen Arduino-Library Ordner kopieren (normalerweise unter `Dokumente/Arduino/libraries`)
4. Die originale Pinbelegung der arduino-nRF5 Erweiterung passt nicht mit unseren Platinen zusammen, also muss die originale `variant.h` mit der aus `Arduino/variant.h` ersetzt werden.  
Die originale variant.h befindet sich normalerweise unter `C:\Users\<Benutzername>\AppData\Local\Arduino15\packages\sandeepmistry\hardware\nRF5\0.4.0\variants\RedBear_BLENano2`
5. Den Programmieradapter (DAPLink) mit dem Computer verbinden
6. Den [mbed Serial Driver](https://os.mbed.com/handbook/Windows-serial-configuration) installieren
7. Die Arduino IDE öffnen (oder neu starten, wenn bereits geöffnet)
8. *Werkzeuge -> Board -> RedBear BLE Nano 2* auswählen
9. *Werkzeuge -> Softdevice -> S132* auswählen
10. Den korrekten Anschluss des Programmieradapters auswählen unter *Werzeuge -> Port* (wird manchmal als *BBC:microBit* angezeigt)
11. *Werkzeuge -> Programmer -> CMSIS-DAP* auswählen
12. **wenn Fabrikneues Board**: *Werkzeuge -> nRF5 Flash SoftDevice* auswählen, und auf Abschluss warten.
13. Software compilieren und hochladen (grüner Haken links oben, oder STRG-U)
14. Fertig!




*Erstellt 01/2018 von C. Locher, M. Schuster, S. Oechslein, N. Rieth, T. Windberg*

