#ifndef MAIN_DATA_H
#define MAIN_DATA_H

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "SoftwareSerial.h"

// Simulationsindikator
//#define SIMULATION 1

/////////////////////////////////////////////////////////////////////
// Pin-Defines 
/////////////////////////////////////////////////////////////////////
static const unsigned char PIN_IN_Ident		= 0; // High = Alpha, Low = Omega
// Status-LED
static const unsigned char PIN_OUT_LED_R = 22;// 8;	// RGB LED Rot
static const unsigned char PIN_OUT_LED_G = 24;// 9;	// RGB LED Grün
static const unsigned char PIN_OUT_LED_B = 26;// 10;	// RGB LED Blau											 
// FWeg
static const unsigned char PIN_OUT_Motoren_STEP	= 5;	// Step Motor rechts und links (bekommen das selbe PWM)
static const unsigned char PIN_OUT_Motor_R_DIR	= 4;	// Richtung Motor rechts
static const unsigned char PIN_OUT_Motor_L_DIR = 2;		// Richtung Motor links
static const unsigned char PIN_IN_Motoren_STEP = 20;	// Richtung Motor links
// Komm
static const unsigned char PIN_IN_RX			= 10; // Input BT Modul RX
static const unsigned char PIN_IN_TX			= 11; // Input BT Modul TX
// UBG
static const unsigned char PIN_OUT_SERVO = 37; // Input BT Modul 2

/////////////////////////////////////////////////////////////////////
// Enums
/////////////////////////////////////////////////////////////////////
// Enum für Main-State-Machine
typedef enum
{
	Main_Init = 0,	// LED Weiß
	Main_Idle = 1,	// LED Gelb
	Main_Drive = 2,	// LED Blau
	Main_UBG = 3,	// LED Lila
	Main_End = 4,	// LED Grün
	Main_Error = 5,	// LED Rot
} eMainState;

// Enum für Roboterart
typedef enum
{
	NOT_INIT = 0,
	ALPHA = 1,
	OMEGA = 2,
} eName;

/////////////////////////////////////////////////////////////////////
// Liste für Route
/////////////////////////////////////////////////////////////////////
// Struct für verkettete Liste
struct sWegPunkt{
	double m_strecke;
	double m_RelWinkel;
	struct sWegPunkt* next;
};

typedef sWegPunkt WegPunkt;

// Funktionen für verkettete Liste
void add_list(int strecke, int RelWinkel);
void delete_list();
void output_list();

/////////////////////////////////////////////////////////////////////
// Globale Variablen und Funktionen
/////////////////////////////////////////////////////////////////////
extern WegPunkt *g_Route;	// Speichert Wegpunkte ab
extern bool g_UBG_Flag;		// wird bei Übergabeszenario auf True gesetzt
extern eName g_Roboter_Name;
extern SoftwareSerial *g_btSerial;

extern int g_Step_cnt;
extern double g_StreckeProStep;
extern double g_DrehwinkelProStep;

// Funktionen für Kommunikation und Fahren
bool GetData();
bool Drive(double winkel, double strecke);
void LED(eMainState State);
bool move_via_step(int step);
double calc_step(double weg);
double calc_deg(double deg);
void Step_CNT();
void Calc_factors_for_motordrive();

#endif