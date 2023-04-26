#include "tm4c123gh6pm.h"
#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"
#include "fpu.h"

#define NVIC_CPAC               0xE000ED88  // Coprocessor Access Control
#define HWREG(x)                (*((volatile uint32_t *)(x)))

void FPUEnable(void);
void UART0_Init(void);
void PortF_Init(void);
uint8_t UART0_available(void);
char UART0_read(void);
void UART0_write(char c);
void printStr(char *str);
void parse(void);
void getCoordinates(void);
void getCommand(char*str);
char* substring(char *destination, const char *source, int beg, int n);

char latitude[100], longitude[100], command[100];
char lat_dir, long_dir;
int lat_deg, long_deg;
float lat_coordinate, long_coordinate;
int flag, len;

int main()
{
    FPUEnable();
    UART0_Init();
    PortF_Init();

    while(1)
    {
        printStr("Enter command: \n");
        getCommand(command);
        getCoordinates();
        UART0_write('\n');
    }
}

void UART0_Init(void)
{
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;    //enable UART clock
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;    //enable GPIO clock

    UART0_CTL_R &= ~UART_CTL_UARTEN;            //disable UART CTL
    
    UART0_IBRD_R = 104;     //calculate BRD: (16*10^6)/(16*9600) = 104.16667
    UART0_FBRD_R = 11;      //0.16667*64 + 0.5

    UART0_LCRH_R = (UART_LCRH_WLEN_8 | UART_LCRH_FEN);  //enable uart parameters such as data length & FIFO
    UART0_CTL_R |= (UART_CTL_UARTEN | UART_CTL_RXE | UART_CTL_TXE);             //enable UART CTL

    GPIO_PORTA_AFSEL_R |= 0x03;
    GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R & ~0xFF) | (GPIO_PCTL_PA0_U0RX | GPIO_PCTL_PA1_U0TX);
    GPIO_PORTA_DEN_R |= 0x03;
}

void PortF_Init(void)
{
    SYSCTL_RCGCGPIO_R |= 0X20;                      //start clock
    while(!(SYSCTL_PRGPIO_R & 0x20));               //check clock status
    GPIO_PORTF_DEN_R |= 0x0E;                       //enable digital
    GPIO_PORTF_AMSEL_R &= ~0x0E;                    //disable analog
    GPIO_PORTF_AFSEL_R &= ~0x0E;                    //no alternative function
    GPIO_PORTF_PCTL_R &= ~0xFFF0;                   //disable PCTL
    GPIO_PORTF_DIR_R |= 0x0E;                       //3 output LEDs
    GPIO_PORTF_DATA_R &= 0X0E;                      //leds are off initially
}

uint8_t UART0_available(void)
{
    return ((UART0_FR_R & UART_FR_RXFE) == UART_FR_RXFE) ? 0 : 1;
}

char UART0_read(void)
{
    while (UART0_available() != 1);
    return UART0_DR_R & 0xFF;
}

void getCommand(char*str)
{
    char c;
    int i;
    for (i = 0; i < 100; i++)
    {
        c = UART0_read();
        if(c == '\n' || c == '\r')
        {
            len = i;
            break;
        }
        else
            str[i] = c; 
        UART0_write(c);
    }
}

void parse(void)
{
    char check[100];
    int i, j; 
    flag = 0;

    //CHECK
    for (i = 0; i < 6; i++)
        check[i] = command[i];
    if (((strcmp(check, "$GPRMC")) != 0) || len < 10 || command[16] != 'A')
    {
        flag = 1;
        return;
    }

    j = 0;
    printStr("\nlatitude: ");
    for (i = 19; i <= 29; i++)
    {
        UART0_write(command[i]);
        latitude[j] = command[i];
        j += 1;
    }
    printStr("\nlongitude: ");
    j = 0;
    for (i = 32; i <= 43; i++)
    {
        UART0_write(command[i]);
        longitude[j] = command[i];
        j += 1;
    }
}

void getCoordinates(void)
{
    char str[15];
    int l1, l2;

    l1 = strlen(latitude);
    l2 = strlen(longitude);

    parse();
    if (flag == 1)
        return;

    //degrees
    substring(str, latitude, 0, 2);
    lat_deg = atoi(str);
    substring(str, longitude, 0, 3);
    long_deg = atoi(str);

    //coordinates
    substring(str, latitude, 2, 6);
    UART0_write('\n');
    printStr(str);
    lat_coordinate = atof(str);

    substring(str, longitude, 3, 6);
    UART0_write('\n');
    printStr(str);
    long_coordinate = atof(str);

    //direction
    lat_dir = latitude[l1-1];
    long_dir = longitude[l2-1];

}

void UART0_write(char c)
{
    while ((UART0_FR_R & UART_FR_TXFF) != 0);
    UART0_DR_R = c;
}

void printStr(char *str)
{
    uint8_t i = 0;
    while(str[i])
    {
        UART0_write(str[i]);
        i++;
    }
}

char* substring(char *destination, const char *source, int beg, int n)
{
    while (n > 0)
    {
        *destination = *(source + beg);
 
        destination++;
        source++;
        n--;
    }
    *destination = '\0';
    return destination;
}

void FPUEnable(void)
{
    //
    // Enable the coprocessors used by the floating-point unit.
    //
    HWREG(NVIC_CPAC) = ((HWREG(NVIC_CPAC) &
                         ~(NVIC_CPAC_CP10_M | NVIC_CPAC_CP11_M)) |
                        NVIC_CPAC_CP10_FULL | NVIC_CPAC_CP11_FULL);
}


//total distance
//< 5 meter take 2 points & calc distance
//leds
//floating point
//uart check
