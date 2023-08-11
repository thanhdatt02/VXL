#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"

#include "bme280.h"
#include "sdkconfig.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "esp_spi_flash.h"
#include <lwip/api.h>
#include <lwip/sys.h>
#include <lwip/netdb.h>
#include <esp_http_server.h>
#include <esp_http_client.h>

#define TAG_BME280 "BME280"

#define BME_SDA_PIN GPIO_NUM_21
#define BME_SCL_PIN GPIO_NUM_22

#define I2C_MASTER_ACK 0
#define I2C_MASTER_NACK 1


#define RELAY_PIN GPIO_NUM_18
//************WIFI
static EventGroupHandle_t s_wifi_event_group;
static const char *TAG = "esp32 webserver";

#define ESP_WIFI_SSID "Linhlinh"
#define ESP_WIFI_PASSWORD "password"
#define ESP_MAXIMUM_RETRY 5

#define TAG_BME280 "BME280"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
static int s_retry_num = 0;
int wifi_connect_status = 0;
double temp;
double press;
double hum;
double percent;
char html_page[] = "<!DOCTYPE HTML><html>\n"
                   "<head>\n"
                   "  <title>TEAM17 - SMART GARDEN</title>\n"
                   "  <meta http-equiv=\"refresh\" content=\"10\">\n"
                   "  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n"
                   "  <link rel=\"stylesheet\" href=\"https://use.fontawesome.com/releases/v5.7.2/css/all.css\" integrity=\"sha384-fnmOCqbTlWIlj8LyTjo7mOUStjsKC4pOpQbqyi7RrhN7udi9RwhKkMHpvLbHG9Sr\" crossorigin=\"anonymous\">\n"
                   "  <link rel=\"icon\" href=\"data:,\">\n"
                   "  <style>\n"
                   "    html {font-family: Arial; display: inline-block; text-align: center;}\n"
                   "    p {  font-size: 1.2rem;}\n"
                   "    body {  margin: 0;}\n"
                   "    .topnav { overflow: hidden; background-color: #0e7c7b; color: white; font-size: 1.7rem; }\n"
                   "    .content { padding: 20px; }\n"
                   "    .card { background-color: white; box-shadow: 2px 2px 12px 1px rgba(140,140,140,.5); }\n"
                   "    .cards { max-width: 700px; margin: 0 auto; display: grid; grid-gap: 2rem; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); }\n"
                   "    .reading { font-size: 2.8rem; }\n"
                   "    .card.temperature { color: #17bebb; }\n"
                   "    .card.humidity { color: #17bebb; }\n"
                   "    .card.pressure { color: #17bebb; }\n"
                   "    .card.percent { color: #17bebb; }\n"
                   "  </style>\n"
                   "</head>\n"
                   "<body>\n"
                   "  <div class=\"topnav\">\n"
                   "    <h3>TEAM17 - SMART GARDEN</h3>\n"
                   "  </div>\n"
                   "  <div class=\"content\">\n"
                   "    <div class=\"cards\">\n"
                   "      <div class=\"card temperature\">\n"
                   "        <h4> TEMPERATURE</h4><p><span class=\"reading\">%.2f&deg;C</span></p>\n"
                   "      </div>\n"
                   "      <div class=\"card humidity\">\n"
                   "        <h4> HUMIDITY</h4><p><span class=\"reading\">%.2f</span> &percnt;</span></p>\n"
                   "      </div>\n"
                   "      <div class=\"card pressure\">\n"
                   "        <h4> PRESSURE</h4><p><span class=\"reading\">%.2fhPa</span></p>\n"
                   "      </div>\n"
                   "      <div class=\"card pressure\">\n"
                   "        <h4> soil</h4><p><span class=\"reading\">%.2f%%</span></p>\n"
                   "      </div>\n"
                   "    </div>\n"
                   "  </div>\n"
                   "</body>\n"
                   "</html>";

// Initialize I2C communication parameters: khởi tạo giao tiếp i2c
void i2c_master_init()
{ // tạo biến cấu trúc i2c_config: chứa thông tin về cấu hình giao tiếp i2c
	i2c_config_t i2c_config = {
		.mode = I2C_MODE_MASTER,   // thiết lập chế độ giao tiếp là chế độ master
		.sda_io_num = BME_SDA_PIN, // thiết lập chân GPIO là chân dữ liệu SDA
		.scl_io_num = BME_SCL_PIN,
		.sda_pullup_en = GPIO_PULLUP_ENABLE, // kích hoạt điện trở kéo lên pullup để đảm bảo tín hiệu ổn định
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 1000000};						 // thiết lập tốc độ truyền dữ liệu là 1Mhz
	i2c_param_config(I2C_NUM_0, &i2c_config);				 // cấu hình tham số i2c cho controller
	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0); // cài đặt driver i2c cho i2c controller
}

// BME280 I2C write function
s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) // hàm trả về giá trị kiểu s8
// địa chỉ thiết bị, địa chỉ thanh ghi, con trỏ tới dữ liệu, số lượng byte cần gửi:cnt
{
	s32 iError = BME280_INIT_VALUE;
	esp_err_t espRc;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create(); // Tạo hàm handle 'cmd' xây dựng chuỗi lệnh gửi đến bme280
	i2c_master_start(cmd);						  // thêm lệnh khởi động truyền dữ liệu, bắt đầu với TH start
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
	// thêm lệnh gửi địa chỉ thiết bị bằng cách dịch trái dev_addr 1 để thêm bit R/W
	i2c_master_write_byte(cmd, reg_addr, true); // thêm lệnh gửi địa chỉ thanh ghi
	i2c_master_write(cmd, reg_data, cnt, true); // thêm lệnh gửi dữ liệu con trỏ
	i2c_master_stop(cmd);						// thêm lệnh kết thúc truyền

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
	// thực thi chuỗi lệnh i2c controller, thời gian chờ là 10ms
	if (espRc == ESP_OK)
	{
		iError = SUCCESS;
	}
	else
	{
		iError = FAIL;
	}
	i2c_cmd_link_delete(cmd);

	return (s8)iError;
}

// BME280 I2C read function
s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BME280_INIT_VALUE;
	esp_err_t espRc;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, reg_addr, true);

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
	// kiểm tra số lượng byte cần đọc, nếu >1 thì đọc các byte dữ liệu trừ byte cuối
	if (cnt > 1)
	{
		i2c_master_read(cmd, reg_data, cnt - 1, I2C_MASTER_ACK);
	}
	i2c_master_read_byte(cmd, reg_data + cnt - 1, I2C_MASTER_NACK);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
	if (espRc == ESP_OK)
	{
		iError = SUCCESS;
	}
	else
	{
		iError = FAIL;
	}

	i2c_cmd_link_delete(cmd);

	return (s8)iError;
}

// BME280 I2C delay function: tạo độ trễ theo thời gian được chỉ định, đv là mili giây
void BME280_delay_msek(u32 msek) // nhận đối số msek
{
	vTaskDelay(msek / portTICK_PERIOD_MS);
}

// BME280 I2C task: task chạy trên freeRTOS, đọc dữ liệu từ bme280 và in ra màn hình
void Publisher_Task(void *params)
{
	// BME280 I2C communication structure
	struct bme280_t bme280 = {
		.bus_write = BME280_I2C_bus_write, // con trở tới hàm ghi
		.bus_read = BME280_I2C_bus_read,
		.dev_addr = BME280_I2C_ADDRESS1,  // địa chỉ i2c của bme280
		.delay_msec = BME280_delay_msek}; // con trỏ tới hàm tạo độ trễ

	s32 com_rslt; // giá trị trả về
	// khai báo các biến cục bộ chưa được xử lý
	s32 v_uncomp_pressure_s32;
	s32 v_uncomp_temperature_s32;
	s32 v_uncomp_humidity_s32;

	// Initialize BME280 sensor and set internal parameters: thiết lập cấu hình ban đầu
	com_rslt = bme280_init(&bme280);
	printf("com_rslt %d\n", com_rslt);

	com_rslt += bme280_set_oversamp_pressure(BME280_OVERSAMP_16X);
	com_rslt += bme280_set_oversamp_temperature(BME280_OVERSAMP_2X);
	com_rslt += bme280_set_oversamp_humidity(BME280_OVERSAMP_1X);
	// thiết lập thời gian chờ giữa các lần đọc dữ liệu từ cảm biến
	com_rslt += bme280_set_standby_durn(BME280_STANDBY_TIME_1_MS);
	// thiết lập bộ lọc dữ liệu cho cảm biến
	com_rslt += bme280_set_filter(BME280_FILTER_COEFF_16);
	// thiết lập chế độ hoạt động cho cảm biến: normal
	com_rslt += bme280_set_power_mode(BME280_NORMAL_MODE);

	// đọc cảm biến độ ẩm đất
	adc1_config_width(ADC_WIDTH_BIT_10);       // Cấu hình độ phân giải ADC 10 bit (0-1023)
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);  // Cấu hình kênh ADC và mức giảm áp (chân GPIO 32)
    esp_adc_cal_characteristics_t *adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_10, 1100, adc_chars);
    

	if (com_rslt == SUCCESS)
	{
		while (true)
		{
			vTaskDelay(10000 / portTICK_PERIOD_MS);
			
			
			// Read BME280 data
			com_rslt = bme280_read_uncomp_pressure_temperature_humidity(
				&v_uncomp_pressure_s32, &v_uncomp_temperature_s32, &v_uncomp_humidity_s32);
			// hàm xử lý dữ liệu uncomp/raw
			double temp = bme280_compensate_temperature_double(v_uncomp_temperature_s32);
			char temperature[12];
			sprintf(temperature, "%.2f degC", temp);

			double press = bme280_compensate_pressure_double(v_uncomp_pressure_s32) / 100; // Pa -> hPa
			char pressure[12];
			sprintf(pressure, "%.2f hPa", press);

			double hum = bme280_compensate_humidity_double(v_uncomp_humidity_s32);
			char humidity[12];
			sprintf(humidity, "%.2f %%", hum);

			float value = adc1_get_raw(ADC1_CHANNEL_4);  // Đọc giá trị từ cảm biến (chân GPIO 32)
			 percent = 100 - (value * 100 /1023); // Chuyển đổi giá trị thành phần trăm
			//Điều khiển relay dựa trên giá trị độ ẩm đất
			if (percent > 70 )
			{
				gpio_set_level(RELAY_PIN, 0); // Tắt relay
				printf("turn off relay\n");
				
			}
			else
			{
				gpio_set_level(RELAY_PIN, 1); 
				printf("turn on relay\n");// Bật relay
				
			}

			// Print BME data
			if (com_rslt == SUCCESS)
			{
				printf("Soil: %.2f%%\n", percent);
				printf("Temperature %s, Humidity %s\n", temperature, humidity);
			}
			else
			{
				ESP_LOGE(TAG_BME280, "measure error. code: %d", com_rslt);
			}
	
		}
	}
	else
	{
		ESP_LOGE(TAG_BME280, "init or setting error. code: %d", com_rslt);
	}
}
//***********WIFI 

//hàm xử lý wifi và ip
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)// bắt đầu chế độ station
    {
        esp_wifi_connect();//hàm kết nối wifi
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)// gọi khi kết nối wifi thất bại
    {   //xử lý khi kết nối wifi bị ngắt
        if (s_retry_num < ESP_MAXIMUM_RETRY)// thử kết nối lại nếu số lần thử chưa vượt quá ngưỡng
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        wifi_connect_status = 0; //đánh dấu trạng thái kết nối
        ESP_LOGI(TAG, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) // khi nhận được địa chị ip
    {   //xử lý khi nhận địa chỉ IP
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        wifi_connect_status = 1;
    }
}

//thiết lập kết nối wifi cho esp32
void connect_wifi(void)
{   //khởi tạo nhóm sự kiện để theo dõi trạng thái và kết nối wifi
    s_wifi_event_group = xEventGroupCreate();
    //khởi tạo cấu trúc mạng, tạo một giao diện wifi vào chế độ station
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());//tạo vòng loop để xử lý các sự kiện hệ thống
    esp_netif_create_default_wifi_sta();// tạo giao diện mạng vào chế độ station

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();// cấu hình mặc định cho wifi
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    //đăng ký các trình xử lý sự kiện và wifi
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));//đặt chế độ wifi là station
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));//đặt cấu hình wifi đã thiết lập vào chế độ station
    ESP_ERROR_CHECK(esp_wifi_start());// bắt đàu kết nối wifi

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    //đợi cho đến khi các bit được đặt trong sự kiện wait bits
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    //kiểm tra kết quả kết nối
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 ESP_WIFI_SSID, ESP_WIFI_PASSWORD);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 ESP_WIFI_SSID, ESP_WIFI_PASSWORD);
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
    vEventGroupDelete(s_wifi_event_group); //xóa nhóm sự kiện
}
//hàm gửi dữ liệu trang web động về một yêu cầu http
esp_err_t send_web_page(httpd_req_t *req)
{

    int response;
    Publisher_Task(NULL);
    char response_data[sizeof(html_page) + 50];
    memset(response_data, 0, sizeof(response_data));// gán các biến bằng 0 trước khi gán dữ liệu vào
    sprintf(response_data, html_page, temp, hum, percent);
    response = httpd_resp_send(req, response_data, HTTPD_RESP_USE_STRLEN);// gửi dữ liệu từ mảng thông qua yêu cầu http

    return response;
}
//hàn xử lý yêu cầu http dạng get
esp_err_t get_req_handler(httpd_req_t *req)
{
    return send_web_page(req);
}
//cấu trúc đại diện cho uri(gốc) trên máy chủ http
httpd_uri_t uri_get = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = get_req_handler,
    .user_ctx = NULL};
//khởi tạo http sử dụng cấu hình, đăng ký xử lý uri_get ở trên
httpd_handle_t setup_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK)
    {
        httpd_register_uri_handler(server, &uri_get);
    }

    return server;
}

void app_main(void) // khởi động esp
{
	// Initialize memory: khởi tạo bộ nhớ non volatile storage/ bộ nhớ điện tĩnh
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		ESP_ERROR_CHECK(nvs_flash_erase()); // hàm xóa toàn bộ dữ liệu trong nvs
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret); // kiểm tra kết quả khởi tạo nvs

	// Initialize I2C parameters: khởi tạo thông số
	i2c_master_init();
	//khởi tạo gpio cho relay
	gpio_pad_select_gpio(RELAY_PIN);				 // Khai báo chân GPIO để sử dụng cho relay
	gpio_set_direction(RELAY_PIN, GPIO_MODE_OUTPUT); // Đặt chế độ OUTPUT cho chân GPIO
	gpio_set_level(RELAY_PIN, 0);	
	ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    connect_wifi();

    if (wifi_connect_status)
    {
        setup_server();
        ESP_LOGI(TAG, "BME280 Web Server is up and running\n");
    }
    else
        ESP_LOGI(TAG, "Failed to connected with Wi-Fi, check your network Credentials\n");

	// khởi động task freeRTOS, với kích thước 5kb, độ ưu tiên (configMAX_PRIORITIES-1) là 5
	xTaskCreate(&Publisher_Task, "Publisher_Task", 1024 * 5, NULL, 5, NULL); // NULL được truyền cho đối số params
}
