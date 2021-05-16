/*****************************************************************************/
/*                                   main                                    */
/*****************************************************************************/
#include "mbed.h"
#include "mbed_rpc.h"                       // rpc 函式庫
#include "stm32l475e_iot01_accelero.h"      // 加速度函式庫

static BufferedSerial pc(STDIO_UART_TX, STDIO_UART_RX);     // PC
static BufferedSerial xbee(D1, D0);                         // Xbee

EventQueue queue(32 * EVENTS_EVENT_SIZE);                   // queue執行接收rpc指令function
Thread t;                                                   // thread跑queue接收rpc指令

void getAcc(Arguments *in, Reply *out);                     // 取加速度的function
RPCFunction rpcAcc(&getAcc, "getAcc");                      // 自定義RPC指令，當接收到"getAcc"，跑getAcc函式

void xbee_rx_interrupt(void);                               // 接收到XBee訊息interrupt
void xbee_rx(void);                                         // 讀取XBee訊息並回傳訊息
void reply_messange(char *xbee_reply, char *messange);      // 讀下指令後的回覆
void check_addr(char *xbee_reply, char *messenger);         // 讀設定的MY與DL

int main() 
{
    BSP_ACCELERO_Init();                                    // 加速度計初始化

    pc.set_baud(9600);

    char xbee_reply[4];

    xbee.set_baud(9600);
    xbee.write("+++", 3);                                   // Xbee設定
    xbee.read(&xbee_reply[0], sizeof(xbee_reply[0]));       // 接收回傳
    xbee.read(&xbee_reply[1], sizeof(xbee_reply[1]));
    if(xbee_reply[0] == 'O' && xbee_reply[1] == 'K') {      // 若回傳是OK
        printf("enter AT mode.\r\n");
        xbee_reply[0] = '\0';
        xbee_reply[1] = '\0';
    }
    xbee.write("ATMY 0x230\r\n", 12);                       // 設定MY是230
    reply_messange(xbee_reply, "setting MY : 0x230");
    xbee.write("ATDL 0x130\r\n", 12);                       // 設定DL是130
    reply_messange(xbee_reply, "setting DL : 0x130");
    xbee.write("ATID 0x1\r\n", 10);                         // 設定ID是1
    reply_messange(xbee_reply, "setting PAN ID : 0x1");
    xbee.write("ATWR\r\n", 6);                              // 儲存變更
    reply_messange(xbee_reply, "write config");
    xbee.write("ATMY\r\n", 6);                              // 檢查MY
    check_addr(xbee_reply, "MY");
    xbee.write("ATDL\r\n", 6);                              // 檢查DL
    check_addr(xbee_reply, "DL");
    xbee.write("ATCN\r\n", 6);                              // 結束
    reply_messange(xbee_reply, "exit AT mode");

    while(xbee.readable()){                                 // 把不要的輸入讀掉
        char *k = new char[1];
        xbee.read(k, 1);
        printf("clear\r\n");
    }

    printf("start\r\n");                                    // 開始接收輸入
    t.start(callback(&queue, &EventQueue::dispatch_forever));
    xbee.set_blocking(false);
    xbee.sigio(mbed_event_queue()->event(xbee_rx_interrupt));
}

void xbee_rx_interrupt(void) {                              // 若有輸入就interrupt執行xbee_rx
    queue.call(&xbee_rx);
}

void xbee_rx(void) {                                        // 接收輸入訊號並執行對應RPC function
    char buf[200] = {0};                                    // 存輸入指令
    char outbuf[200] = {0};                                 // 存輸出訊息

    while(xbee.readable()) {                                // 若還有輸入就繼續讀
        for (int i = 0; i < 200; i++) {                     // 一次存一個字，最多存200個字
            char *recv = new char[1];                       // 一個char存一個字
            xbee.read(recv, 1);                             // 讀一個字到recv
            buf[i] = *recv;                                 // 存入buf的第i位
            //printf("|%s|", recv);                         // 檢驗存入資料用
            if (*recv == '\r') {                            // 若是指令最後一位'\r'
                break;                                      // 結束，跳出迴圈
            }
        }
        //printf("\n%s\n", buf);                            // 檢驗輸入指令
        RPC::call(buf, outbuf);                             // 呼叫RPC function，回傳outbuf

        printf("%s\r\n", outbuf);                           // 印出回傳outbuf，為加速度量值
        xbee.write(outbuf, sizeof(outbuf));                 // 傳給PC回傳的加速度
        ThisThread::sleep_for(1s);                          // 停頓一秒，可能是要等outbuf傳出，也可能不需要
    }
}

void reply_messange(char *xbee_reply, char *messange) {     // 讀下指令後的回覆
    xbee.read(&xbee_reply[0], 1);                           // 三個char讀回復
    xbee.read(&xbee_reply[1], 1);
    xbee.read(&xbee_reply[2], 1);
    if(xbee_reply[1] == 'O' && xbee_reply[2] == 'K'){       // 若是回覆為OK
        printf("%s\r\n", messange);                         // 設定成功，顯示設定內容
        xbee_reply[0] = '\0';                               // 歸0
        xbee_reply[1] = '\0';
        xbee_reply[2] = '\0';
    }
}

void check_addr(char *xbee_reply, char *messenger) {        // 讀設定的MY與DL
    xbee.read(&xbee_reply[0], 1);                           // 四個char讀回復
    xbee.read(&xbee_reply[1], 1);
    xbee.read(&xbee_reply[2], 1);
    xbee.read(&xbee_reply[3], 1);                           // 輸出MY或DL的數值
    printf("%s = %c%c%c\r\n", messenger, xbee_reply[1], xbee_reply[2], xbee_reply[3]);
    xbee_reply[0] = '\0';                                   // 歸0
    xbee_reply[1] = '\0';
    xbee_reply[2] = '\0';
    xbee_reply[3] = '\0';
}

void getAcc(Arguments *in, Reply *out) {                    // 讀加速度的function
    int16_t pDataXYZ[3] = {0};                              // 存加速度
    char buffer[200];                                       // 存回傳的字串
    BSP_ACCELERO_AccGetXYZ(pDataXYZ);                       // 讀加速度
    sprintf(buffer, "Accelerometer values: (%d, %d, %d)\n", pDataXYZ[0], pDataXYZ[1], pDataXYZ[2]);
    out->putData(buffer);                                   // 將字串寫入buffer並回傳
}