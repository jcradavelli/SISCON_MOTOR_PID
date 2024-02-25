#ifndef TWRAPP_H
#define TWRAPP_H

/**
 * Indexador de erros
*/
typedef enum sgcp_err_
{
    SGCP_ERR_NONE = 0,

    SGCP_ERR_FAIL,


    SGCP_ERR_UNKNOW,
} sgcp_err_t;


/**
 * LOG wrappers
*/

#define __ANSI_COLLOR_RED__    "\033[0;31m"
#define __ANSI_COLLOR_YELLOW__ "\033[0;33m"
#define __ANSI_COLLOR_GREEN__  "\033[0;32m" 
#define __ANSI_COLLOR_WHITE__  "\033[0;37m"
#define __ANSI_COLLOR_RESET__  __ANSI_COLLOR_WHITE__


#define ESP_LOGE( tag, format, ... )    printf(__ANSI_COLLOR_RED__   "E (%d) [%s]: "format __ANSI_COLLOR_RESET__ "\n", GetTickCount(), tag, ##__VA_ARGS__ ) ///Error log       - LOWEST
#define ESP_LOGW( tag, format, ... )    printf(__ANSI_COLLOR_YELLOW__"W (%d) [%s]: "format __ANSI_COLLOR_RESET__ "\n", GetTickCount(), tag, ##__VA_ARGS__ ) //Warning log
#define ESP_LOGI( tag, format, ... )    printf(__ANSI_COLLOR_GREEN__ "I (%d) [%s]: "format __ANSI_COLLOR_RESET__ "\n", GetTickCount(), tag, ##__VA_ARGS__ ) //Information log
#define ESP_LOGD( tag, format, ... )    printf(__ANSI_COLLOR_WHITE__ "D (%d) [%s]: "format __ANSI_COLLOR_RESET__ "\n", GetTickCount(), tag, ##__VA_ARGS__ ) //Debug LOG
#define ESP_LOGV( tag, format, ... )    printf(__ANSI_COLLOR_RESET__ "V (%d) [%s]: "format __ANSI_COLLOR_RESET__ "\n", GetTickCount(), tag, ##__VA_ARGS__ ) //verbose log     - HIGHEST

// /**
//  * quewes wrappers
// */
// #include "freertos/FreeRTOS.h"
// #include "freertos/queue.h"
// #include "freertos/semphr.h"

// typedef QueueHandle_t twrap_QueueHandle_t;

// #define twrap_xQueueSendFromISR( xQueue, pvItemToQueue, pxHigherPriorityTaskWoken ) xQueueSendFromISR( xQueue, pvItemToQueue, pxHigherPriorityTaskWoken )
// #define twrap_xQueueReceive( xQueue, pvBuffer, xTicksToWait )                       xQueueReceive( xQueue, pvBuffer, xTicksToWait )
// #define twrap_xQueueCreate( uxQueueLength, uxItemSize )                             xQueueCreate( uxQueueLength, uxItemSize )
// #define trwap_vQueueDelete( xQueue )                                                vQueueDelete( xQueue )
// #define twrap_xQueueSend( xQueue, pvItemToQueue, pxHigherPriorityTaskWoken )        xQueueSend( xQueue, pvItemToQueue, pxHigherPriorityTaskWoken )
// //xQueuePeek
// //xQueuePeekFromISR



// /**
//  * List wrappers
// */
// #define TRWAP_USE_FREERTOS_LIST


// #ifdef TRWAP_USE_FREERTOS_LIST

// #include "freertos/list.h"

// typedef List_t trwap_List_t;
// typedef ListItem_t trwap_ListItem_t;

// #define twrap_vListInitialise( pxList ) vListInitialise(  pxList )
// #define twrap_vListInitialiseItem( pxItem ) vListInitialiseItem( pxItem )
// #define twrap_vListInsertEnd( pxList, pxNewListItem ) vListInsertEnd(  pxList, pxNewListItem )
// #define trwap_uxListRemove( pxItemToRemove ) uxListRemove( pxItemToRemove ) //Recebe um ponteiro para entregar o item e devolve a quantidade de elementos disponivel
// #define trwap_listGET_HEAD_ENTRY(pxList) listGET_HEAD_ENTRY(pxList)
// #define twrap_listLIST_IS_INITIALISED( pxList ) listLIST_IS_INITIALISED( pxList )   
// #define twrap_listSET_LIST_ITEM_OWNER( pxListItem, pxOwner )  listSET_LIST_ITEM_OWNER( pxListItem, pxOwner ) 
// #define twrap_listSET_LIST_ITEM_VALUE( pxListItem, xValue ) listSET_LIST_ITEM_VALUE( pxListItem, xValue ) 
// #define twrap_listGET_LIST_ITEM_VALUE(pxListItem) listGET_LIST_ITEM_VALUE(pxListItem)
// #define trwap_listLIST_IS_INITIALISED( pxList ) listLIST_IS_INITIALISED( pxList )
// #define trwap_listLIST_IS_EMPTY( pxList )   listLIST_IS_EMPTY( pxList )  
// #define trwap_listGET_OWNER_OF_HEAD_ENTRY( pxList ) listGET_OWNER_OF_HEAD_ENTRY( pxList )
// #define trwap_listGET_OWNER_OF_NEXT_ENTRY( pxTCB, pxList ) listGET_OWNER_OF_NEXT_ENTRY( pxTCB, pxList )
// #define trwap_listGET_LIST_ITEM_OWNER( pxListItem ) listGET_LIST_ITEM_OWNER( pxListItem ) 
// #define trwap_listCURRENT_LIST_LENGTH( pxList ) listCURRENT_LIST_LENGTH( pxList ) 


// #endif
// #ifdef TWRAP_USE_MYLIST
// #include "myList.h"

// typedef xMyList trwap_List_t;
// typedef xMyListItem trwap_ListItem_t;

// #define twrap_vListInitialise( pxList ) vMyListInitialise(  pxList )
// #define twrap_vListInitialiseItem( pxItem ) vMyListInitialiseItem( pxItem )
// #define twrap_vListInsertEnd( pxList, pxNewListItem ) vMyListInsertEnd(  pxList, pxNewListItem )
// #define trwap_uxListRemove( pxItemToRemove ) vMyListRemove( pxItemToRemove ) //Recebe um ponteiro para entregar o item e devolve a quantidade de elementos disponivel
// #define trwap_listGET_HEAD_ENTRY(pxList) myListGET_HEAD_ENTRY(pxList)
// #define twrap_listLIST_IS_INITIALISED( pxList ) myListLIST_IS_INITIALISED( pxList )   
// #define twrap_listSET_LIST_ITEM_OWNER( pxListItem, pxOwner )  myListSET_LIST_ITEM_OWNER( pxListItem, pxOwner ) 
// #define twrap_listSET_LIST_ITEM_VALUE( pxListItem, xValue ) myListSET_LIST_ITEM_VALUE( pxListItem, xValue ) 
// #define twrap_listGET_LIST_ITEM_VALUE(pxListItem) (uint32_t) myListGET_LIST_ITEM_VALUE(pxListItem)
// #define trwap_listLIST_IS_INITIALISED( pxList ) myListLIST_IS_INITIALISED( pxList )
// #define trwap_listLIST_IS_EMPTY( pxList )   myListLIST_IS_EMPTY( pxList )  
// #define trwap_listGET_OWNER_OF_HEAD_ENTRY( pxList ) myListGET_OWNER_OF_HEAD_ENTRY( pxList )


// #endif






// /**
//  * Radio wrappers 
// */
// #include "drv_radio.h"


// /**
//  * Generic operations 
// */

// //#define NEW_INSTANCE(_instance_type_t_) malloc(sizeof(_instance_type_t_))//pvPortMalloc(sizeof(_instance_type_t_))
// #define NEW_INSTANCE(_instance_type_t_) heap_caps_malloc(sizeof(_instance_type_t_), MALLOC_CAP_8BIT)
// #define DEL_INSTANCE(_px_) heap_caps_free(_px_)
// // #define NEW_INSTANCE(_instance_type_t_) malloc(sizeof(_instance_type_t_))
// // #define DEL_INSTANCE(_px_) free(_px_)


// #include "esp_timer.h"


#endif