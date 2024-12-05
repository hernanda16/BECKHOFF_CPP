#include <arpa/inet.h>
#include <chrono>
#include <iostream>
#include <soem/ethercat.h>
#include <strings.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <thread>
#include <unistd.h>

#define EC_TIMEOUTMON 5000

#define EL1809_ID 0x07113052
#define EL3104_ID 0x0c203052
#define EL3204_ID 0x0c843052
#define EL5152_ID 0x14203052
#define EL6751_ID 0x1a5f3052
#define EL2008_ID 0x07d83052
#define EL6021_ID 0x17853052

#define SLAVE_RECV_PORT_UDP 8888
#define MASTER_RECV_PORT_UDP 44444
#define MASTER_RECV_PORT_TCP 12345
#define RECV_UDP
#define SEND_UDP

//*=================================================================================================
//*                                         LOGGER
//*=================================================================================================
/**
 * @note log - print a message with a color based on the log level
 * @note LogLevel - enum class for log levels
 * @note INFO - print a message with green color
 * @note WARN - print a message with yellow color
 * @note ERR - print a message with red color
 * @note Hernanda - hernanda16
 */
enum LogLevel {
    INFO,
    WARN,
    ERR
};

void log(LogLevel level, const std::string& message)
{
    switch (level) {
    case INFO:
        std::cout << "\033[32m" << message << "\033[0m" << std::endl;
        break;
    case WARN:
        std::cout << "\033[33m" << message << "\033[0m" << std::endl;
        break;
    case ERR:
        std::cout << "\033[31m" << message << "\033[0m" << std::endl;
        break;
    }
}
//*=================================================================================================
//*                                     END OF LOGGER
//*=================================================================================================
//*=================================================================================================
//*                                     CHRONO TIMER
//*=================================================================================================
class ChronoTimer {
public:
    ChronoTimer()
    {
    }

    void startTimer(float precision = 0.2)
    {
        this->precision = precision;
        start_timer = std::chrono::high_resolution_clock::now();
    }

    void sleep(int ms)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(ms - (int64_t)elapsed()));
        real_time_timer = elapsed();

        if (elapsed() * 1000 > ms + precision * ms) {
            log(WARN, "Error: Elapsed time is more than " + std::to_string(ms) + " ms -> " + std::to_string(elapsed()) + " s");
        }
    }

    double getElapsed()
    {
        return std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start_timer).count();
    }

private:
    float precision;
    double real_time_timer;
    std::chrono::high_resolution_clock::time_point start_timer;

    double elapsed()
    {
        return std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start_timer).count();
    }
};
//*=================================================================================================
//*                                   END OF CHRONO TIMER
//*=================================================================================================
//*=================================================================================================
//*                                     UDP CONNECTION
//*=================================================================================================
/**
 * @note socket - return a file descriptor for the pc new socket
 * @note bind - give socket FD the local address ADDR (which is LEN bytes long)
 * @author Hernanda - hernanda16
 */
int socket_udp;
struct sockaddr_in server_udp; // PC Master Address (Need to configure to send)
struct sockaddr_in any_addr_udp; // Any Address sending to this PC
socklen_t any_addr_len_udp = sizeof(any_addr_udp); // Lenght of Any Address
struct sockaddr_in client_udp; // This PC Address (Need to configure to recv)

char recv_buffer_udp[64];
char send_buffer_udp[64] = "its";

void initUDP()
{
    socket_udp = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_udp < 0) {
        log(ERR, "UDP ERROR : Failed to create socket");
        return;
    }

    // Clean data and set PC Master configuration
    bzero(&server_udp, sizeof(server_udp));
    server_udp.sin_family = AF_INET;
    server_udp.sin_port = htons(MASTER_RECV_PORT_UDP);
    server_udp.sin_addr.s_addr = inet_addr("169.254.183.100");

    // Clean data and set This PC configuration
    bzero(&client_udp, sizeof(client_udp));
    client_udp.sin_family = AF_INET;
    client_udp.sin_port = htons(SLAVE_RECV_PORT_UDP);
    client_udp.sin_addr.s_addr = INADDR_ANY;

    // set udp timeout
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 2000;
    setsockopt(socket_udp, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

    // Bind socket and checking is binding success
    bind(socket_udp, (struct sockaddr*)&client_udp, sizeof(client_udp));
    if (socket_udp < 0) {
        log(ERR, "UDP ERROR : Binding Failed");
        return;
    }
}

void sendUDP()
{
    //=================================================================

    //=================================================================
    sendto(socket_udp, send_buffer_udp, sizeof(send_buffer_udp), 0, (struct sockaddr*)&server_udp, sizeof(server_udp));
}

void recvUDP()
{
    int8_t recv_len = recvfrom(socket_udp, recv_buffer_udp, 64, 0, (struct sockaddr*)&any_addr_udp, &any_addr_len_udp);
    if (recv_len > 0 && recv_buffer_udp[0] == 'i' && recv_buffer_udp[1] == 't' && recv_buffer_udp[2] == 's') {
        //=================================================================

        //=================================================================
    }
}
//*=================================================================================================
//*                                 END OF UDP CONNECTION
//*=================================================================================================
//*=================================================================================================
//*                                     TCP CONNECTION
//*=================================================================================================
/**
 * @note socket - return a file descriptor for the pc new socket
 * @note bind - give socket FD the local address ADDR (which is LEN bytes long)
 * @author Hernanda - hernanda16
 */
int socket_tcp, confd;
struct sockaddr_in server_tcp;
struct sockaddr_in client_tcp;
socklen_t client_len_tcp = sizeof(client_udp); // Lenght of Any Address

char recv_buffer_tcp[64];
char send_buffer_tcp[64];

void initTCP()
{
    socket_tcp = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_tcp < 0) {
        log(ERR, "TCP ERROR : Failed to create socket");
        return;
    }

    // Clean data and set PC Master configuration
    bzero(&client_tcp, sizeof(client_tcp));
    client_tcp.sin_family = AF_INET;
    client_tcp.sin_port = htons(MASTER_RECV_PORT_TCP);
    client_tcp.sin_addr.s_addr = htonl(INADDR_ANY);

    // Bind socket and checking is binding success
    bind(socket_tcp, (struct sockaddr*)&client_tcp, sizeof(client_tcp));
    if (socket_tcp < 0) {
        log(ERR, "TCP ERROR : Binding Failed");
        return;
    }

    // Now server is ready to listen and verification
    if ((listen(socket_tcp, 5)) != 0) {
        log(ERR, "TCP ERROR : Listen Failed");
        return;
    } else
        printf("Server listening..\n");

    // Accept the data packet from client and verification
    confd = accept(socket_tcp, (struct sockaddr*)&client_tcp, &client_len_tcp);
    if (confd < 0) {
        log(ERR, "TCP ERROR : Server Accept Failed");
        return;
    } else
        printf("Server accept the client...\n");

    // if (connect(socket_tcp, (struct sockaddr*)&server_tcp, sizeof(server_tcp)) != 0) {
    //     log(ERR, "Connection with the TCP server failed...");
    // } else
    //     log(INFO, "Connected to TCP Server");
}

void commTCP()
{
    bzero(recv_buffer_tcp, sizeof(recv_buffer_tcp));
    read(confd, recv_buffer_tcp, sizeof(recv_buffer_tcp));
    printf("From client: %s\n", recv_buffer_tcp);
}

//*=================================================================================================
//*                                 END OF TCP CONNECTION
//*=================================================================================================
//*=================================================================================================
//*                                        BECKHOFF
//*=================================================================================================
// Configs
// =======================================================
std::string if_name;
int po2so_config = 0;

// Vars
// =======================================================
uint16_t slave_canopen_id = 255;
int expectedWKC = 0;
uint8 IOmap[4096]; // I/O map for PDOs

/**
 * Scan and save to EEPROM the CANopen slaves configuration
 */
int scan_CANopen_Slaves(uint16_t slave)
{
    uint8_t _f002_1[2] = { 0x01, 0x00 };
    ec_SDOwrite(slave, 0xf002, 0x01, FALSE, sizeof(_f002_1), &_f002_1, EC_TIMEOUTRXM);

    printf("Wait...\n");

    while (1) {
        uint8_t _f002_2;
        static uint8_t prev_f002_2;
        int _f002_2_s = sizeof(_f002_2);
        int _f002_2_wkc = ec_SDOread(slave, 0xf002, 0x02, FALSE, &_f002_2_s, &_f002_2, EC_TIMEOUTRXM);
        if (_f002_2_wkc > 0) {
            if (prev_f002_2 != _f002_2) {
                if (100 == _f002_2)
                    printf("f002_2: %d\n", _f002_2);
                else if (130 == _f002_2)
                    printf("f002_2: %d\n", _f002_2);
                else if (180 == _f002_2)
                    printf("f002_2: %d\n", _f002_2);
            }

            if (_f002_2 <= 3)
                break;
        }

        prev_f002_2 = _f002_2;

        usleep(10000);
    }

    uint8_t _9000[200] = { 0x00 };
    int _9000_s = sizeof(_9000);
    int _9000_wkc = ec_SDOread(slave, 0x9000, 0x00, TRUE, &_9000_s, &_9000, 10 * EC_TIMEOUTRXM);
    if (_9000_wkc > 0 && _9000_s > 2) {
        printf("CO slave0: ");
        for (int k = 0; k < _9000_s; k++) {
            printf("0x%x ", _9000[k]);
        }
        printf("\n");
    }

    uint8_t _9010[200] = { 0x00 };
    int _9010_s = sizeof(_9010);
    int _9010_wkc = ec_SDOread(slave, 0x9010, 0x00, TRUE, &_9010_s, &_9010, 10 * EC_TIMEOUTRXM);
    if (_9010_wkc > 0 && _9010_s > 2) {
        printf("CO slave1: ");
        for (int k = 0; k < _9010_s; k++) {
            printf("0x%x ", _9010[k]);
        }
        printf("\n");
    }

    uint8_t _9020[200] = { 0x00 };
    int _9020_s = sizeof(_9020);
    int _9020_wkc = ec_SDOread(slave, 0x9020, 0x00, TRUE, &_9020_s, &_9020, 10 * EC_TIMEOUTRXM);
    if (_9020_wkc > 0 && _9020_s > 2) {
        printf("CO slave2: ");
        for (int k = 0; k < _9020_s; k++) {
            printf("0x%x ", _9020[k]);
        }
        printf("\n");
    }

    uint8_t _9030[200] = { 0x00 };
    int _9030_s = sizeof(_9030);
    int _9030_wkc = ec_SDOread(slave, 0x9030, 0x00, TRUE, &_9030_s, &_9030, 10 * EC_TIMEOUTRXM);
    if (_9030_wkc > 0 && _9030_s > 2) {
        printf("CO slave3: ");
        for (int k = 0; k < _9030_s; k++) {
            printf("0x%x ", _9030[k]);
        }
        printf("\n");
    }

    uint8_t _9040[200] = { 0x00 };
    int _9040_s = sizeof(_9040);
    int _9040_wkc = ec_SDOread(slave, 0x9040, 0x00, TRUE, &_9040_s, &_9040, 10 * EC_TIMEOUTRXM);
    if (_9040_wkc > 0 && _9040_s > 2) {
        printf("CO slave4: ");
        for (int k = 0; k < _9040_s; k++) {
            printf("0x%x ", _9040[k]);
        }
        printf("\n");
    }

    // Save ke EEPROM
    uint32_t _1010 = 0x65766173;
    ec_SDOwrite(slave, 0x1010, 0x01, TRUE, sizeof(_1010), &_1010, EC_TIMEOUTRXM);

    while (ec_iserror()) {
        printf("%s", ec_elist2string());
    }

    return 1;
}

int remove_CAN_eeprom(uint16_t slave)
{
    uint32_t _1011 = 0x64616F6C; // Delete
    ec_SDOwrite(slave, 0x1011, 0x01, TRUE, sizeof(_1011), &_1011, EC_TIMEOUTRXM);

    while (EcatError)
        printf("%s", ec_elist2string());

    return 1;
}

int init_CAN_Startup(uint16_t slave)
{
    uint8_t _8003[68] = {
        0x08, 0x00,
        0x02, 0x1A,
        0x00,
        0x01, 0x00,
        0x00,

        0x02, 0x1A,
        0x01,
        0x04, 0x00,
        0x20, 0x00, 0x64, 0x60,

        0x02, 0x1A,
        0x02,
        0x04, 0x00,
        0x20, 0x00, 0x69, 0x60,

        0x02, 0x1A,
        0x00,
        0x01, 0x00,
        0x02,

        0x02, 0x16,
        0x00,
        0x01, 0x00,
        0x00,

        0x02, 0x16,
        0x01,
        0x04, 0x00,
        0x20, 0x00, 0xff, 0x60,

        0x02, 0x16,
        0x02,
        0x04, 0x00,
        0x10, 0x00, 0x71, 0x60,

        0x02, 0x16,
        0x00,
        0x01, 0x00,
        0x02,

        0x60, 0x60,
        0x00,
        0x01, 0x00,
        0x03
    };
    ec_SDOwrite(slave, 0x8003, 0x00, TRUE, sizeof(_8003), &_8003, EC_TIMEOUTRXM);

    while (EcatError)
        printf("%s", ec_elist2string());

    return 1;
}

int8_t init_beckhoff()
{
    log(INFO, "+--------------------------------------------------+");
    log(INFO, "|             Initializing Beckhoff...             |");
    log(INFO, "+--------------------------------------------------+");
    if_name = "enp1s0";
    if (ec_init(if_name.c_str()) > 0) {
        if (ec_config_init(FALSE) > 0) {
            while (EcatError)
                printf("%s", ec_elist2string());

            log(INFO, "| ✓ " + std::to_string(ec_slavecount) + " slaves found and configured                  |");

            for (uint8_t slave = 1; slave <= ec_slavecount; slave++) {
                switch (ec_slave[slave].eep_id) {
                case EL6751_ID:
                    slave_canopen_id = slave;
                    log(INFO, "| ✓ Found EL6751 slave with Canopen ID: " + std::to_string(slave_canopen_id) + "          |");
                    break;
                }
            }

            if (slave_canopen_id == 255) {
                log(ERR, "+--------------------------------------------------+");
                log(ERR, "|          No slave found with Canopen ID          |");
                log(ERR, "+--------------------------------------------------+");

                return 4;
            }

            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            log(INFO, "| ✓ Calculated workcounter: " + std::to_string(expectedWKC) + "                      |");

            // ============= CONFIGURE STARTUP STATE =============
            if (1 == po2so_config) {
                ec_slave[slave_canopen_id].PO2SOconfig = &scan_CANopen_Slaves;
            } else if (2 == po2so_config) {
                ec_slave[slave_canopen_id].PO2SOconfig = &remove_CAN_eeprom;
            } else if (3 == po2so_config) {
                ec_slave[slave_canopen_id].PO2SOconfig = &init_CAN_Startup;
            }
            // ===================================================

            ec_config_map(&IOmap);
            ec_configdc();

            ec_slave[0].state = EC_STATE_SAFE_OP;
            ec_writestate(0);

            while (ec_iserror()) {
                printf("%s", ec_elist2string());
            }

            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 3);
            if (ec_slave[0].state != EC_STATE_SAFE_OP) {
                printf("Not all slaves reached safe operational state.\n");
                ec_readstate();
                for (int i = 1; i <= ec_slavecount; i++) {
                    if (ec_slave[i].state != EC_STATE_SAFE_OP) {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n", i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }

            ec_readstate();
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            ec_writestate(0);

            if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
                log(INFO, "| ✓ Operational state reached for all slaves.      |");
                log(INFO, "+--------------------------------------------------+");
                log(INFO, "|        Beckhoff initialized successfully         |");
                log(INFO, "+--------------------------------------------------+");

                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
            }

            return 0;
        } else {
            log(ERR, "+--------------------------------------------------+");
            log(ERR, "|        Could not configure beckhoff config       |");
            log(ERR, "+--------------------------------------------------+");
            return 2;
        }
    } else {
        log(ERR, "+--------------------------------------------------+");
        log(ERR, "|      Could not configure beckhoff interface      |");
        log(ERR, "+--------------------------------------------------+");
        return 1;
    }
}
//*=================================================================================================
//*                                    END OF BECKHOFF
//*=================================================================================================

int main()
{
    ChronoTimer timer;

    //===================== Initialize Beckhoff =====================
    uint8_t ret_beckhoff = init_beckhoff();
    //===============================================================
    //=================== Initialize TCP Client =====================
    initTCP();
    //===============================================================

    commTCP();

    while (1) {
        timer.startTimer();
        //===========================================================
        for (size_t i = 0; i < 1000; i++) {
            for (size_t j = 0; j < 1000; j++) {
                for (size_t j = 0; j < 1000; j++) {
                    /* code */
                }
            }
        }

        //===========================================================
        timer.sleep(10);
    }

    return 0;
}