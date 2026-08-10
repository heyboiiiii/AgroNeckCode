> AgroNomad 2026

# AgroNeckCode

Código del AgroNeck para la ESP32. Incluye librerías para:

- LoRa
- GPS
- MPU6050
- MLX90614
- LM35

## Funcionando:

- Comunicacion con MPU6050(Aceletometro y giroscopo) y MLX90614(Sensor Temp. Interna). -> (I2C)

- Lectura Sensor LM35(Revisar que la medicion sea adecuada).

## Pendientes:

- Comunicacion GPS. -> UART(Posible problema fisico entre pines de la placa).

- Comunicacion LoRA. -> SPI(Revisar codigo de la libreria y probar con RX).
  
- Programar Interrupciones en el MPU6050 para activar la TX de datos solo cuando se detecte movimiento del animal. (software realizado, falta confirmar su funcionamiento)

## Luego de realizar Pendientes:

- Plantear como se va a estructurar el paquete que se debe enviar al receptor.(Ej. {"lat":"dato_lat","long","dato_long",etc...}).

- Buscar un metodo de encriptacion para enviar la informacion de forma segura.
