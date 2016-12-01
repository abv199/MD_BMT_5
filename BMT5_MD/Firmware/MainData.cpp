#include"MainData.h"
#include"Arduino.h"
#include "TimerThree.h"
#include "math.h"
#include "SoftwareSerial.h"
#include <stdio.h>
#include <stdlib.h>

using namespace std;
////////////////////////////////////////////////////////////////////
// Globale Variablen definieren
////////////////////////////////////////////////////////////////////
WegPunkt *g_Route = NULL;
bool g_UBG_Flag = false;
bool g_ERROR_FLAG = false;
eName g_Roboter_Name = NOT_INIT;
int g_Step_cnt = 0;
TimerThree timer_Motor;
double g_StreckeProStep = 0;
double g_DrehwinkelProStep = 0;
SoftwareSerial *g_btSerial = NULL;

////////////////////////////////////////////////////////////////////

// Funktion zum berechenen der Faktoren für die Calc_step und Calc_deg Funktionen
void Calc_factors_for_motordrive()
{
	double D_Rad = 77; // mm
	int GesStepsProUmdrehung = 200;
	double Radabstand = 145;//mm
	g_StreckeProStep = PI*D_Rad / GesStepsProUmdrehung;
	g_DrehwinkelProStep = (360 * D_Rad) / (Radabstand*GesStepsProUmdrehung);
	// Einstellungen für Betriebsart des Treibers: 
	// MS1	 MS2   MS3    Anzahl Steps
	//  L     L     L     4
	//  H     L     L     8
	//  L     H     L     16
	//  H     H     L     32
	//  H     H     H     64
}

// erstellt neue Liste, wenn noch keine vorhanden und sonst wird sie mit den übergebenen Werten 
// erweitert (ptr->next bekommt wert zugewiesen)

// strecke in mm
// RelWinkel in Grad
void add_list(int strecke, int RelWinkel) {
	WegPunkt *ptr;
	if (g_Route == NULL)
	{
		g_Route = (WegPunkt *)malloc(sizeof(*g_Route));				//Zeiger auf das erste Element des reservierten Speichers
		g_Route->m_RelWinkel = RelWinkel;
		g_Route->m_strecke = strecke;
		g_Route->next = NULL;
	}
	else
	{
		ptr = g_Route;
		while (ptr->next != NULL)
			ptr = ptr->next;				// ptr->next zeigt auf das erste Element der nächsten Struktur!!! 
											// ->ptr ist somit der Zeiger auf die nächste Struktur 
		ptr->next = (WegPunkt *)malloc(sizeof(*ptr));
		ptr = ptr->next;
		ptr->m_RelWinkel = RelWinkel;
		ptr->m_strecke = strecke;
		ptr->next = NULL;
	}
}

/*Liste wird vom ersten Element bis zum letzten Element gelöscht*/
void delete_list() {
	WegPunkt *ptr, *ptrA;
	//Überprüfung, ob nächste Adresse einem Speicher zugewiesen ist
	if (g_Route->next != NULL) { 
		//Wenn ja, dann wird temp. ptr auf aktuelles g_Route verwiesen
		ptr = g_Route;									
		/*Solange jetzt das nächste Element noch eine Adresse hat,
		wird diese Adresse zwischengespeichert und aktueller Speicher
		gelöscht*/
		while (ptr->next != NULL)
		{
			ptrA = ptr;
			ptr = ptr->next;
			free(ptrA);
		}
		//Beim letzten Speicher ist der Pointer auf next == NULL, deswegen muss er nochmal
		//explizit freigegeben werden
		free(ptr);				
	}							
	//Um auch g_Route wieder zurückzusetzen, wird er mit NULL initialisiert
	g_Route = NULL;
}


bool GetData()
{
	// empfängt Strecke und speichert sie in Strecke[] ab
	// empfängt Übergabebit und setzt demnach UBG_Flag
	// TRUE, wenn erfolgreich, bei Fehler FALSE
	static int data[20][3];
	static int i = 0;
	static int j = 0;
	String btData = "";

	// löscht alte Strecke
	if (g_Route != NULL) {
		delete_list();
	}
#ifndef SIMULATION
	btData = g_btSerial->read();
	data[i][j] = (btData.toInt());
	i++;
	if (i == 20)
	{
		i = 0;
		j++;
	}
	if (j == 3)
	{
		// Umrechnung und zur liste hinzu fügen
		for (int n = 0; n < 20; n++)
		{
			add_list(data[n][0], data[n][1] - data[n][2]);	// Daten in Liste speichern
			//Serial.print("Weg: ");
			//Serial.print(data[n][0]);
			//Serial.print("Winkel: ");
			//Serial.println(data[n][1] - data[n][2] +100);
		}
		i = 0;
		j = 0;
		btData = "";
		return true;
		//druck();
	}
	return  false;
#endif
#ifdef SIMULATION
	// bei simulation Quadrat mit 1000mm * 1000mm abfahren
	add_list(255,-90);
	add_list(255,-90);
	add_list(255,-90);
	add_list(255,-90);
	add_list(255,-90);
	add_list(255,-90);
	// Simulation
	delay(1000);
	return true;
#endif
}

bool Drive(double winkel, double strecke)
{
	// true wenn ziel erreicht
	// false, wenn durch stopp-Bit angehalten
	// Simulation
	double step = calc_step(strecke);
	double deg = calc_deg(abs(winkel));
	
	if(winkel > 0){					//Linksdrehung
		digitalWrite(PIN_OUT_Motor_R_DIR, LOW);
		digitalWrite(PIN_OUT_Motor_L_DIR, HIGH);
		move_via_step(deg);
		delay(100);				//Pause nach Drehung
	}else if(winkel < 0){			//Rechtsdrehung
		digitalWrite(PIN_OUT_Motor_R_DIR, HIGH);
		digitalWrite(PIN_OUT_Motor_L_DIR, LOW);
		move_via_step(deg);
		delay(100);				//Pause nach Drehung
	}								//Geradeausfahren
	digitalWrite(PIN_OUT_Motor_R_DIR, HIGH);
	digitalWrite(PIN_OUT_Motor_L_DIR, HIGH);
	move_via_step(step);
	delay(100);				//Pause nach Drehung
	//Auswertung des übergabebits
	if (Main_State == Main_Idle) {
		return false;
	}
	else {
		return true;
	}
}

double calc_step(double weg){
	//int Ks = 1;			//Fahrzeugabängiger Wert für das Schritte/Weg Verhältniss
	//Serial.print("g_StreckeProStep: ");
	//Serial.println(g_StreckeProStep);
	return weg / g_StreckeProStep;
}

double calc_deg(double deg){
	//int Kw = 1;			//Fahrzeugabängiger Wert für den Betrag der Drehung
	//Serial.print("g_DrehwinkelProStep: ");
	//Serial.println(g_DrehwinkelProStep);
	return deg / g_DrehwinkelProStep;
}

void move_via_step(int step){	//Schritte auf den Motortreiber übergeben
	////Serial.print("Step: ");
	//Serial.println(step);
	// Timer für die steps initialisieren
	timer_Motor.initialize(5000); // 5000
	timer_Motor.pwm(PIN_OUT_Motoren_STEP, 512);
	attachInterrupt(digitalPinToInterrupt(PIN_IN_Motoren_STEP), Step_CNT, RISING);
	// Interrupt zum zählen der steps initialisieren
	g_Step_cnt = 0;
	// Warten bis Steps erreicht und kontinuierlich das Bluetooth modul abfragen
	while (g_Step_cnt < step) {
		delayMicroseconds(10);
		////Serial.print("Step_CNT: ");
		////Serial.println(g_Step_cnt);
		if (g_btSerial->available())
		{
			timer_Motor.pwm(PIN_OUT_Motoren_STEP, 0);
			detachInterrupt(digitalPinToInterrupt(PIN_IN_Motoren_STEP));
			g_Step_cnt = step + 1;
			Main_State = Main_Idle;
		}//Bluetooth
	}
	detachInterrupt(digitalPinToInterrupt(PIN_IN_Motoren_STEP));
	// PWM signal auf 0 setzen
	timer_Motor.pwm(PIN_OUT_Motoren_STEP, 0);
}

void Step_CNT() // zählt steps anhand von steigenden flanken des PWM signals
{
	g_Step_cnt++;
}

void LED(eMainState State)
{
	// Hier wurde mit Common-Kathode LED gearbeitet (siehe http://propmakergen.blogspot.de/2016/03/rgb-led-tutorial.html)
	// Demnach HIGH-Activ!!
	// ACHTUNG!! Bei genutzter LED sind im gegensatz zu der in dem Link die Pins Grün und Blau vertauscht!
	// Reset
	digitalWrite(PIN_OUT_LED_R, LOW);
	digitalWrite(PIN_OUT_LED_G, LOW);
	digitalWrite(PIN_OUT_LED_B, LOW);
	// neue Farbe einstellen
	switch (State)
	{
		// Initialisierung -> Weiß
	case Main_Init:
	{
		digitalWrite(PIN_OUT_LED_R, HIGH);
		digitalWrite(PIN_OUT_LED_G, HIGH);
		digitalWrite(PIN_OUT_LED_B, HIGH);
		break;
	}
	// Fahrweg empfangen -> Gelb
	case Main_Idle:
	{
		digitalWrite(PIN_OUT_LED_R, HIGH);
		digitalWrite(PIN_OUT_LED_G, HIGH);
		break;
	}
	// Fahrweg abfahren und überprüfen, ob noch auf Kurs -> Blau
	case Main_Drive:
	{			
		digitalWrite(PIN_OUT_LED_B, HIGH);
		break;
	}
	// Am Ziel angekommen -> Grün
	case Main_End:
	{
		digitalWrite(PIN_OUT_LED_G, HIGH);
		break;
	}
	// Übergabe durchführen -> Lila
	case Main_UBG:
	{
		digitalWrite(PIN_OUT_LED_R, HIGH);
		digitalWrite(PIN_OUT_LED_B, HIGH);
		break;
	}
	// Fehler -> 	Rot blinkend bei gesetzter ERROR-Flag; 
	//				Rot-gelb blinkend bei anderen Fehlern(leerer Routen-Liste)
	case Main_Error:
	{
		digitalWrite(PIN_OUT_LED_R, HIGH);
		if (g_ERROR_FLAG){
			delay(500);
			digitalWrite(PIN_OUT_LED_R, LOW);
			delay(500);
		}
		else{
			delay(500);
			digitalWrite(PIN_OUT_LED_G, HIGH);
			delay(500);
			digitalWrite(PIN_OUT_LED_G, LOW);
		}
		break;
	}
	default:
		digitalWrite(PIN_OUT_LED_R, LOW);
		break;
	}
}
