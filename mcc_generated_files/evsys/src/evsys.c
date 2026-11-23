///**
// *
// * @file evsys.c
// *
// * @ingroup evsys_driver
// *
// * @brief This file contains the API implementation for the EVSYS driver.
// *
// * @version EVSYS Driver Version 1.1.3
// */
//
///*
//© [2026] Microchip Technology Inc. and its subsidiaries.
//
//    Subject to your compliance with these terms, you may use Microchip 
//    software and any derivatives exclusively with Microchip products. 
//    You are responsible for complying with 3rd party license terms  
//    applicable to your use of 3rd party software (including open source  
//    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
//    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
//    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
//    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
//    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
//    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
//    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
//    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
//    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
//    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
//    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
//    THIS SOFTWARE.
//*/
//
//#include "../evsys.h"
//
//int8_t EVSYS_Initialize()
//{
//    //CHANNEL0 AC0_OUT; 
//    EVSYS.CHANNEL0 = (uint8_t)0x20;
//    //CHANNEL1 OFF; 
//    EVSYS.CHANNEL1 = (uint8_t)0x0;
//    //CHANNEL2 OFF; 
//    EVSYS.CHANNEL2 = (uint8_t)0x0;
//    //CHANNEL3 OFF; 
//    EVSYS.CHANNEL3 = (uint8_t)0x0;
//    //CHANNEL4 OFF; 
//    EVSYS.CHANNEL4 = (uint8_t)0x0;
//    //CHANNEL5 OFF; 
//    EVSYS.CHANNEL5 = (uint8_t)0x0;
//    //USER OFF; 
//    EVSYS.USERADC0START = (uint8_t)0x0;
//    //USER OFF; 
//    EVSYS.USERCCLLUT0A = (uint8_t)0x0;
//    //USER OFF; 
//    EVSYS.USERCCLLUT0B = (uint8_t)0x0;
//    //USER OFF; 
//    EVSYS.USERCCLLUT1A = (uint8_t)0x0;
//    //USER OFF; 
//    EVSYS.USERCCLLUT1B = (uint8_t)0x0;
//    //USER OFF; 
//    EVSYS.USERCCLLUT2A = (uint8_t)0x0;
//    //USER OFF; 
//    EVSYS.USERCCLLUT2B = (uint8_t)0x0;
//    //USER OFF; 
//    EVSYS.USERCCLLUT3A = (uint8_t)0x0;
//    //USER OFF; 
//    EVSYS.USERCCLLUT3B = (uint8_t)0x0;
//    //USER OFF; 
//    EVSYS.USEREVSYSEVOUTA = (uint8_t)0x0;
//    //USER OFF; 
//    EVSYS.USEREVSYSEVOUTB = (uint8_t)0x0;
//    //USER OFF; 
//    EVSYS.USEREVSYSEVOUTC = (uint8_t)0x0;
//    //USER OFF; 
//    EVSYS.USEREVSYSEVOUTD = (uint8_t)0x0;
//    //USER OFF; 
//    EVSYS.USEREVSYSEVOUTF = (uint8_t)0x0;
//    //USER OFF; 
//    EVSYS.USERTCA0CNTA = (uint8_t)0x0;
//    //USER OFF; 
//    EVSYS.USERTCA0CNTB = (uint8_t)0x0;
//    //USER OFF; 
//    EVSYS.USERTCB0CAPT = (uint8_t)0x0;
//    //USER OFF; 
//    EVSYS.USERTCB0COUNT = (uint8_t)0x0;
//    //USER OFF; 
//    EVSYS.USERTCB1CAPT = (uint8_t)0x0;
//    //USER OFF; 
//    EVSYS.USERTCB1COUNT = (uint8_t)0x0;
//    //USER OFF; 
//    EVSYS.USERUSART0IRDA = (uint8_t)0x0;
//    //USER OFF; 
//    EVSYS.USERUSART1IRDA = (uint8_t)0x0;
//    //CHANNEL6 OFF; 
//    EVSYS.CHANNEL6 = (uint8_t)0x0;
//    //CHANNEL7 OFF; 
//    EVSYS.CHANNEL7 = (uint8_t)0x0;
//    //USER OFF; 
//    EVSYS.USERPTCSTART = (uint8_t)0x0;
//    //USER OFF; 
//    EVSYS.USERTCB2CAPT = (uint8_t)0x0;
//    //USER OFF; 
//    EVSYS.USERTCB2COUNT = (uint8_t)0x0;
//    //USER CHANNEL0; 
//    EVSYS.USERTCD0INPUTA = (uint8_t)0x1;
//    //USER OFF; 
//    EVSYS.USERTCD0INPUTB = (uint8_t)0x0;
//    //USER OFF; 
//    EVSYS.USERUSART2IRDA = (uint8_t)0x0;
//    //CHANNEL8 OFF; 
//    EVSYS.CHANNEL8 = (uint8_t)0x0;
//    //CHANNEL9 OFF; 
//    EVSYS.CHANNEL9 = (uint8_t)0x0;
//    //USER OFF; 
//    EVSYS.USERCCLLUT4A = (uint8_t)0x0;
//    //USER OFF; 
//    EVSYS.USERCCLLUT4B = (uint8_t)0x0;
//    //USER OFF; 
//    EVSYS.USERCCLLUT5A = (uint8_t)0x0;
//    //USER OFF; 
//    EVSYS.USERCCLLUT5B = (uint8_t)0x0;
//    //USER OFF; 
//    EVSYS.USEREVSYSEVOUTE = (uint8_t)0x0;
//    //USER OFF; 
//    EVSYS.USERTCA1CNTA = (uint8_t)0x0;
//    //USER OFF; 
//    EVSYS.USERTCA1CNTB = (uint8_t)0x0;
//    //USER OFF; 
//    EVSYS.USERTCB3CAPT = (uint8_t)0x0;
//    //USER OFF; 
//    EVSYS.USERTCB3COUNT = (uint8_t)0x0;
//    //USER OFF; 
//    EVSYS.USERUSART3IRDA = (uint8_t)0x0;
//    //USER OFF; 
//    EVSYS.USERUSART4IRDA = (uint8_t)0x0;
//
//
//    return 0;
//}
//
//void EVSYS_SoftwareEventASet(uint8_t channel){
//    EVSYS.SWEVENTA = channel;
//} 
//
//void EVSYS_SoftwareEventBSet(uint8_t channel){
//    EVSYS.SWEVENTB = channel;
//}
