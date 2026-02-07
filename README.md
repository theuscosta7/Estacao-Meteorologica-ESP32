<h1> Estação Meteorológica </h1>
<h3> Aluno: Mateus Costa </h3>

<h3> Este projeto implementa uma estação meteorológica utilizando o ESP32, capaz de coletar dados ambientais e enviá-los para a nuvem por meio do protocolo MQTT.
O sistema realiza leituras periódicas de temperatura, pressão atmosférica e umidade, utilizando o sensor BME280, e publica essas informações em formato JSON em um broker MQTT hospedado na plataforma HiveMQ. </h3>

<h3> Funcionalidades: </h3>

<h4> 
  <ul> 1. Conexão Wi-Fi </ul>
  <ul> 2. Comunicação segura via MQTT (TLS) </ul>
  <ul> 3. Leitura de temperatura, pressão e umidade </ul>
  <ul> 4. Comunicação I2C </ul>
  <ul> 5. Envio periódico de dados a cada 1 minuto </ul>
  <ul> 6. Dados no formato JSON </ul>
</h4>

<h3> Tecnologias Utilizadas: </h3>

<h4>
  <ul> 1. ESP32 </ul>
  <ul> 2. ESP-IDF </ul>
  <ul> 3. MQTT </ul>
  <ul> 4. HiveMQ Cloud </ul>
  <ul> 5. I2C </ul>
  <ul> 6. BME280 </ul>
  <ul> 7. JSON </ul>
</h4>

<h3> Funcionamento do Sistema: </h3>

<h4>
  <ul> 1. O ESP32 inicializa a memória NVS </ul>
  <ul> 2. Conecta-se à rede Wi-Fi configurada </ul>
  <ul> 3. Inicializa o barramento I2C </ul>
  <ul> 4. Estabelece conexão segura com o broker MQTT </ul>
  <ul> 5. Inicializa o sensor ambiental </ul>
  <ul> 6. Uma task do FreeRTOS executa continuamente </ul>
</h4>

<h3> Hardware necessário: </h3>

<h4>
  <ul> 1. ESP32 </ul>
  <ul> 2. Sensor BME280 </ul>
  <ul> 3. Jumpers </ul>
  <ul> 4. Fonte de alimentação / cabo USB </ul>
</h4>

