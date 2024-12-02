/*
 * Copyright (c) 2006-2020 Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 */
#include "mbed.h"
#include <functional>

Ticker BuscaLixo;
DigitalIn Leitura(PTB0);
DigitalOut Escrita(PTB1);
char FourBit, EightBit = 0;
int cont = 0, contador = 0, cont1 = 0;
char mask = 15;
int StartBit = 0;

void rx()
{  
    

    if(cont == 4){
    if(FourBit == 0){
        EightBit = EightBit << 1;
        cont = 0;
        cont1 = 0;
    }
    else if(cont1 >= 2){
        EightBit = EightBit << 1;
        EightBit = (EightBit|0x01);
        cont = 0;
        cont1 = 0;
    }
    else{
        cont1 = 0;
        cont = 0;
    }
   printf("%d ", EightBit);
    
   if(EightBit == 15 && StartBit == 0){
       printf("\nAchei o start!\n");
        StartBit = 1;
   }
   if(EightBit == 116 && StartBit == 1){
       printf("\nAchei a palavra!\n");
        StartBit = 2;
   }
   if(EightBit == 14 && StartBit == 2){
       printf("\nAchei o ID!\n");
        StartBit = 3;
   }
   if(EightBit == 15 && StartBit == 3){
       printf("\nAchei o STOP!\n");
        StartBit = 0;
    
       printf("\n\n");
   }
   
   


}
    FourBit = FourBit << 1;
    if(Leitura.read() == 1){
        cont1++;
    }
    FourBit = (FourBit | Leitura.read());
    FourBit = FourBit & mask;
    cont ++;
}

void printar(int tempbuff, int temp) {
    for (int i = 0; i < 8; i++) {
        if ((tempbuff & 0b10000000) == 0b10000000) {
            Escrita = 1;
        } else{
            Escrita = 0;
        }
        tempbuff = tempbuff << 1;
        thread_sleep_for(temp);
    }

}


int main()
{
    BuscaLixo.attach(&rx, 1ms); // the address of the function to be attached (flip) and the interval (2 seconds)
    int start = 0b00001111;
    int palavra = 0b01110100; // Palavra a ser enviada
    char MagId = 0b00001110; // Identificação
    char stop = 0b00001111; // StopBITs


    // spin in a main loop. BuscaLixo will interrupt it to call flip
    while (1) {
        printar(start, 4);
        printar(palavra, 4);
        printar(MagId, 4);
        printar(stop, 4);
    }
}
