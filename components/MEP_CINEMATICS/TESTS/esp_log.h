
#include <stdio.h>
#include <time.h>




#define LOG_COLOR_BLACK   "30"
#define LOG_COLOR_RED     "31"
#define LOG_COLOR_GREEN   "32"
#define LOG_COLOR_BROWN   "33"
#define LOG_COLOR_BLUE    "34"
#define LOG_COLOR_PURPLE  "35"
#define LOG_COLOR_CYAN    "36"
#define LOG_COLOR(COLOR)  "\033[0;" COLOR "m"
#define LOG_BOLD(COLOR)   "\033[1;" COLOR "m"
#define LOG_RESET_COLOR   "\033[0m"
#define LOG_COLOR_E       LOG_COLOR(LOG_COLOR_RED)
#define LOG_COLOR_W       LOG_COLOR(LOG_COLOR_BROWN)
#define LOG_COLOR_I       LOG_COLOR(LOG_COLOR_GREEN)
#define LOG_COLOR_D
#define LOG_COLOR_V



#define ESP_LOGE(TAG, FORMAT, ... ) (printf(LOG_COLOR_E"E (%lld) %s: " FORMAT "\n\r" LOG_RESET_COLOR, (long long int)time(0), TAG, ##__VA_ARGS__))
#define ESP_LOGW(TAG, FORMAT, ... ) (printf(LOG_COLOR_W"W (%lld) %s: " FORMAT "\n\r" LOG_RESET_COLOR, (long long int)time(0), TAG, ##__VA_ARGS__))
#define ESP_LOGI(TAG, FORMAT, ... ) (printf(LOG_COLOR_I"I (%lld) %s: " FORMAT "\n\r" LOG_RESET_COLOR, (long long int)time(0), TAG, ##__VA_ARGS__))
#define ESP_LOGD(TAG, FORMAT, ... ) (printf(LOG_COLOR_D"D (%lld) %s: " FORMAT "\n\r" LOG_RESET_COLOR, (long long int)time(0), TAG, ##__VA_ARGS__))
#define ESP_LOGV(TAG, FORMAT, ... ) (printf(LOG_COLOR_V"V (%lld) %s: " FORMAT "\n\r" LOG_RESET_COLOR, (long long int)time(0), TAG, ##__VA_ARGS__))