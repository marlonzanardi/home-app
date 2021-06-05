/****************************************Informação do Arquivo********************************************
** Nome do Arquivo:          Pinagem.h
** Data Ultima Modificação:  08-01-19
** Ultima Versão:            Sim
** Descrição:                Biblioteca do controle dos pinos I/O utilizados.                 
**------------------------------------------------------------------------------------------------------
** Criado por:          Marlon Zanardi <marlon.zanardi95@hotmail.com>
** Data de Criação:     02-03-18     
********************************************************************************************************/

#ifndef Pinagem_h
#define Pinagem_h
#include <Arduino.h>

// Define apelidos para os botoes
#define BT_AVANCA 53
#define BT_VOLTA 49
#define BT_SELECIONA 51

// Leds
#define LED_VERD 47
#define LED_VERM 43
#define LED_AMAR 45

// Define as constantes para os pinos da ponte H e motores
#define RELE_LAMP_TROFEU 2
#define RELE_LED_TV 3
#define RELE_LUZ_QUARTO 4
#define IN4 5
#define ENA 3
#define ENB A3

//Pino BUZZER
#define PINO_BUZZER A5 
//Delay do buzzer
#define DELAY_BUZZER 80

#endif
