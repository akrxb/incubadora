# Incubadora Neonatal    
Proyecto relacionado con el reto UPCT de Instrumentación en el que tratamos de replicar los sensores de una incubadora neonatal de uso hospitalario. Esto incluye:      
Calidad del aire: comprobación de que los niveles de CO2 sean los correctos.      
Control de temperatura: se añaden dos sensores que miden la temperatura del interior de la incubadora, por diseño, están ajustados para que los dos aporten información significativa, ya que como uno está más cerca de las resistencias calefactoras, se podría dar una falsa estimación de la TªC (no es el caso). Este control se hace mediante un PID, regulando la potencia que se le administra a dos resistencias calefactoras a partir de MOSFETs. Se pueden desactivar, e incluso tener un setpoint estable de TªC (máx. 37ºC).    
Peso: con dos HX711 calibrados obtenemos el peso del neonato.      
Iluminación: incluimos tiras led de dos colores, azul y rojo. Esto se debe a que las azules contribuyen al tratamiento de la ictericia, y la luz roja, permite en un entorno real, permite poder inspeccionar al neonato sin que este se vea alterado por la luz, ya que no captan esa longitud de onda. Se puede controlar la intensidad lumínica así como encendido y apagado.      

Todo esto se trabaja sobre una ESP32S3 y además para facilitar su uso, mostramos los controles por una pantalla que hace de interfaz gráfica. También, disponemos de una dashboard web, en el que tendríamos todos los parámetros.      

<img width="1134" height="2016" alt="WhatsApp Image 2026-09-19 at 11 07 26" src="https://github.com/user-attachments/assets/0f7c2bd4-eee6-4e1a-8036-ce8ce5ff3716" />

<img width="1296" height="848" alt="WhatsApp Image 2026-09-19 at 11 08 10" src="https://github.com/user-attachments/assets/fcd63ef7-2cc7-4d6d-92ac-8834e09928bd" />


