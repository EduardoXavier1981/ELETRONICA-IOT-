#include <stdio.h>
#include "btstack.h"
#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"
#include "pico/stdlib.h"
#include "MPU_6050_GATT.h"
#include "haw/MPU6050.h"

#define HEARTBEAT_PERIOD_MS 50      //Pulso do Bluetooth (50 ms para cada rotina)

static btstack_timer_source_t heartbeat;                                            //Timer do Bluetooth
static btstack_packet_callback_registration_t hci_event_callback_registration;      //Registro de callback

#define APP_AD_FLAGS 0x06           //Configuração da notificação

//Campos
static uint8_t adv_data[] = {
    // Flags general discoverable
    0x02, BLUETOOTH_DATA_TYPE_FLAGS, APP_AD_FLAGS,                                                      //Tamanho em bits do parámetro
    // Name
    0x0B, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, 'P', 'i', 'c', 'o', ' ', 'W', ' ', 'I', 'M', 'U',
    0x03, BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS, 0x1a, 0x18,
};
static const uint8_t adv_data_len = sizeof(adv_data);

int le_notification_enabled;
hci_con_handle_t con_handle;
static float acel[3] = {0,0,0}, giro[3] = {0,0,0},  temperatura = 0;
uint16_t tx_temp = 0;
mpu6050_t mpu6050;
bool initialized = false;

void init_imu_mpu_6050();
void poll_imu_mpu_6050();
void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static uint16_t att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size);
static int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size);

void init_imu_mpu_6050() {
    gpio_init(PICO_DEFAULT_I2C_SDA_PIN);                        //Pino SDA padrão da placa
    gpio_init(PICO_DEFAULT_I2C_SCL_PIN);                        //Pino SCL padrão da placa 

    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C); //Seta a função I2C
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C); //Seta a função I2C

    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);                     //Habilita resistor de PULLUP do SDA
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);                     //Habilita resistor de PULLUP do SCL
    
    mpu6050 = mpu6050_init(i2c_default, MPU6050_ADDRESS_A0_GND);

    if (!mpu6050_begin(&mpu6050)){
        while(true){
            printf("ERRO AO INICIAR O SENSOR!\n");
            sleep_ms(500);
        }
    }

    mpu6050_set_scale(&mpu6050, MPU6050_SCALE_2000DPS);
    mpu6050_set_range(&mpu6050, MPU6050_RANGE_16G);

    mpu6050_set_temperature_measuring(&mpu6050, true);
    mpu6050_set_gyroscope_measuring(&mpu6050, true);
    mpu6050_set_accelerometer_measuring(&mpu6050, true);
    
    mpu6050_set_int_free_fall(&mpu6050, false);
    mpu6050_set_int_motion(&mpu6050, false);
    mpu6050_set_int_zero_motion(&mpu6050, false);
    
    mpu6050_set_motion_detection_threshold(&mpu6050, 2);
    mpu6050_set_motion_detection_duration(&mpu6050, 5);

    mpu6050_set_zero_motion_detection_threshold(&mpu6050, 4);
    mpu6050_set_zero_motion_detection_duration(&mpu6050, 2);
}

void poll_imu_mpu_6050(){
    mpu6050_event(&mpu6050);    //Lê todos os sensores
    
    mpu6050_vectorf_t *acel_temp = mpu6050_get_accelerometer(&mpu6050);  //Armazena a leitura do acelerometro
    mpu6050_vectorf_t *giro_temp = mpu6050_get_gyroscope(&mpu6050);      //Armazena a leitura do giroscopio

    //Armazenando os valores para leitura global
    temperatura = mpu6050_get_temperature_c(&mpu6050);

    acel[0] = acel_temp -> x;
    acel[1] = acel_temp -> y;
    acel[2] = acel_temp -> z;

    giro[0] = giro_temp -> x;
    giro[1] = giro_temp -> y;
    giro[2] = giro_temp -> z;
}

static void heartbeat_handler(struct btstack_timer_source *ts) {
    static uint32_t contador = 0;
    contador ++;

    if (contador%1 == 0){
        poll_imu_mpu_6050();
        if (le_notification_enabled) {
            att_server_request_can_send_now_event(con_handle);

        }
    }

    static int led = true;
    led = !led;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led);

    btstack_run_loop_set_timer(ts, HEARTBEAT_PERIOD_MS);
    btstack_run_loop_add_timer(ts);
}

void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    bd_addr_t local_addr;
    if ( packet_type != HCI_EVENT_PACKET) return;
 
    uint8_t tipo_evento = hci_event_packet_get_type(packet);
 
    switch(tipo_evento) {
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) return;
            gap_local_bd_addr(local_addr);
            printf("BTStack rodando no endereço: %s\n", bd_addr_to_str(local_addr));
        break;
 
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            le_notification_enabled = 0;
        break;
 
        case ATT_EVENT_CAN_SEND_NOW:
            tx_temp = temperatura * 100;
            att_server_notify(con_handle, ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_TEMPERATURE_01_VALUE_HANDLE,
            (uint8_t *) &tx_temp, sizeof(tx_temp));
        break;
    }
}

static uint16_t att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size) {
    switch (att_handle){
        case ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_TEMPERATURE_01_VALUE_HANDLE:
            tx_temp = temperatura*100;
            return att_read_callback_handle_blob((const uint8_t *)&tx_temp, sizeof(tx_temp), offset, buffer, buffer_size);
        break;
    }
    return 0;
}

static int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size) {
    switch (att_handle){
        case ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_TEMPERATURE_01_CLIENT_CONFIGURATION_HANDLE:
            le_notification_enabled = little_endian_read_16(buffer, 0) == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
        break;
    }

    if (le_notification_enabled){
        con_handle = connection_handle;
        att_server_request_can_send_now_event(con_handle);
    }

    return 0;
}



int main(){
    stdio_init_all();

    //Inicialização do Bluetooth

    //Verificação de falhas do chip wireless
    if (cyw43_arch_init()) {
        printf("Falha ao iniciar CYW43\n");
        return -1;
    }

    l2cap_init();
    sm_init();
    init_imu_mpu_6050();

    //Pacotes de configuração da biblioteca
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    att_server_register_packet_handler(packet_handler);

    //Configuração do timer periódico do bluetooth
    heartbeat.process = &heartbeat_handler;
    btstack_run_loop_set_timer(&heartbeat, HEARTBEAT_PERIOD_MS);
    btstack_run_loop_add_timer(&heartbeat);

    //Inicializar o servidor de atributos
    att_server_init(profile_data, att_read_callback, att_write_callback);

    uint16_t desc_min = 800;    //tempo mínimo disponível para conexão
    uint16_t desc_max = 800;    //tempo maximo disponível para conexão
    //como min = max o serviço fica disponível o tempo todo
    uint8_t desc_type = 0;
    bd_addr_t endereco;
    memset(endereco, 0, 6);

    gap_advertisements_set_params(desc_min, desc_max, desc_type, 0, endereco, 0x07, 0x00);
    gap_advertisements_set_data(adv_data_len, (uint8_t*) adv_data);
    gap_advertisements_enable(1);

    hci_power_control(HCI_POWER_ON);
    btstack_run_loop_execute();

    /***************************/

    /*
    while(true){
        poll_imu_mpu_6050();
        printf("%f m/s^2 no x, %f m/s^2 no y, %f m/s^2 no z\n", acel[0], acel[1], acel[2]);
        sleep_ms(500);
    }
    */
}
