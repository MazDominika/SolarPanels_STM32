System tworzony w ramach pracy inżynierskiej sterujący położeniem paneli fotowoltaicznych:
* wyznaczanie położenie na podstawie różnicy rezystancji fotorezystorów
* manualne sterowanie silnikami  pozycję zadaną przesyłaną przez łącze szeregowe z płytki ESP32 komunikującej się z WebServerem za pomocą protokołu MQTT
* sterowanie silnikami dwóch osi - horyzonralnej oraz wertykalnej
* obsługa wyłączników krańcowych
* wysyłanie aktualnej pozycji do płytki ESP32C3 
