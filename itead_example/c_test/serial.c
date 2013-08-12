#include <stdio.h>
#include <itead.h>
void main(void)
{
        SerialBegin(UART0,115200);
        SerialPrintln(UART0,"hello pengge");
        SerialPrint(UART0,"hello print\n");
        SerialWrite(UART0,0x38);

}
