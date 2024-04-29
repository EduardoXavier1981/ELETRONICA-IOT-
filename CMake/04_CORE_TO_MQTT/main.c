#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "bsp/board.h"
#include "hardware/adc.h"

#include "lwip/apps/mqtt.h"
 
#define WIFI_SSID "Claudinei-2G"
#define WIFI_PASSWORD "C21R28i20t27r22"
#define MQTT_SERVER "54.146.113.169"
 

struct mqtt_connect_client_info_t mqtt_client_info=
{
  "CORERasp/pico_w",  /*client id*/
  NULL,               /* user */
  NULL,               /* pass */
  0,                  /* keep alive */
  NULL,               /* will_topic */
  NULL,               /* will_msg */
  0,                  /* will_qos */
  0                   /* will_retain */
#if LWIP_ALTCP && LWIP_ALTCP_TLS
  , NULL
#endif
};

static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    printf("data: %s\n",data);    
}
  
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
  printf("topic %s\n", topic);
}
 
static void mqtt_request_cb(void *arg, err_t err) { 
  printf(("MQTT client request cb: err %d\n", (int)err));
}
 
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
  printf(("MQTT client connection cb: status %d\n",  (int)status));z
}
 
int main()
{
  stdio_init_all();           //Configuração inicial da placa
  printf("Iniciando...\n");   //Informativo

  adc_init();
  adc_set_temp_sensor_enabled(true);
  adc_select_input(4);

  //Iniciando o chip WiFi
  if (cyw43_arch_init()){
    printf("Falha ao Iniciar chip WiFi\n");
    return 1;
  }

  //Configurando a placa como cliente
  cyw43_arch_enable_sta_mode();
  printf("Conectando ao %s\n",WIFI_SSID);

  //Conectando ao roteador
  if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)){
    printf("Falha ao conectar\n");
    return 1;
  }
  printf("Conectado ao WiFi %s\n",WIFI_SSID);

  //Conectando ao MQTT
  ip_addr_t addr;
  if(!ip4addr_aton(MQTT_SERVER, &addr)){
    printf("ip error\n");
    return 1;
  }
  printf("Conectando ao MQTT\n");

  mqtt_client_t* cliente_mqtt = mqtt_client_new();    //Criando novo cliente
  mqtt_set_inpub_callback(cliente_mqtt, &mqtt_incoming_publish_cb, &mqtt_incoming_data_cb, NULL);   //Lincando as callbacks de envio e recebimento de dados
  err_t erro = mqtt_client_connect(cliente_mqtt, &addr, 1883, &mqtt_connection_cb, NULL, &mqtt_client_info);
  if(erro != ERR_OK){
    printf("Erro de conexão\n");
    return 1;
  }

  printf("Conectado ao MQTT!\n");

  while(true){
    uint16_t raw = adc_read();
    const float conversion_factor = 3.3f / (1<<12);
    float result = raw * conversion_factor;
    float temp = 27 - (result -0.706)/0.001721;

    char temp_str[20];
    snprintf(temp_str, sizeof(temp_str), "%f", temp);
    int bytes = strlen(temp_str);

    if (board_button_read()){
      mqtt_publish(cliente_mqtt, "CORE/button", temp_str, bytes, 0, false, &mqtt_request_cb, NULL);
    }

    sleep_ms(1000);
  }
}