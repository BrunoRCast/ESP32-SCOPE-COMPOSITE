# ESP32-SCOPE-COMPOSITE
Brincadeira de transformar um esp32 em um osciloscópio de 4 canais claramente inspirado no tds420 da tektronix e utilizando uma tv de tubo (CRT) como display, porque toda pessoa deve ter uma tv de tubo em casa(afirmação)... E um osciloscópio!  

(⌐⊙_⊙)  

Pino 25 gera sinal de vídeo composto, não é necessário colocar resistor, comigo funcionou sem em 3 TVs e 2 Leitores de VHS.  

Pinos 39(VP), 36(VN), 34, 35 são os canais do osciloscópio.  

//------------------------------  

Compilado na IDE Arduino  

Utilizado ESP32 Wroom  

Biblioteca necessária:  

https://github.com/Roger-random/ESP_8_BIT_composite  


Pode ser instalada direto da IDE ESP_8_BIT_composite Video by Roger Cheng  

Em Boards Manager a versão de esp32 by Espressif System precisa ser no máximo 2.0.17, as mais atuais não compilam, mudou muita coisa.  

Sim, utilizei IA, mas mesmo assim levei dois dias pra fazer funcionar e de momento vai ficar assim.  

Explore o código, utilize o terminal serial para os comandos, digite help e ele irá lhe fornecer a lista de comandos.  

um core é responsável por coletar os sinais analógicos enquanto o outro se fica com a geração de sinais de vídeo. 

No mais...  

Haduuukeen ༼つಠ益ಠ༽つ ─=≡ΣO))  

<img width="1599" height="777" alt="image" src="https://github.com/user-attachments/assets/a9451119-3ee5-4029-acc1-05e854da6430" />
