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
eName g_Roboter_Name = NOT_INIT;
int g_Step_cnt = 0;
double g_StreckeProStep = 0;
double g_DrehwinkelProStep = 0;
SoftwareSerial *g_btSerial = NULL;

////////////////////////////////////////////////////////////////////
// Funktion zum berechenen der Faktoren f�r die Calc_step und Calc_deg Funktionen
void Calc_factors_for_motordrive()
{
	double D_Rad = 77; // mm
	int GesStepsProUmdrehung = 200;
	double Radabstand = 145;//mm
	g_StreckeProStep = PI*D_Rad / GesStepsProUmdrehung;
	g_DrehwinkelProStep = (360 * D_Rad) / (Radabstand*GesStepsProUmdrehung);
	// Einstellungen f�r Betriebsart des Treibers: 
	// MS1	 MS2   MS3    Anzahl Steps
	//  L     L     L     4
	//  H     L     L     8
	//  L     H     L     16
	//  H     H     L     32
	//  H     H     H     64
}

////////////////////////////////////////////////////////////////////
// erstellt neue Liste, wenn noch keine vorhanden und sonst wird sie mit den �bergebenen Werten 
// erweitert (ptr->next bekommt wert zugewiesen)
void add_list(int strecke, int RelWinkel) {//strecke in mm | RelWinkel in Grad
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
			ptr = ptr->next;				// ptr->next zeigt auf das erste Element der n�chsten Struktur!!! 
											// ->ptr ist somit der Zeiger auf die n�chste Struktur 
		ptr->next = (WegPunkt *)malloc(sizeof(*ptr));
		ptr = ptr->next;
		ptr->m_RelWinkel = RelWinkel;
		ptr->m_strecke = strecke;
		ptr->next = NULL;
	}
}

////////////////////////////////////////////////////////////////////
//Liste wird vom ersten Element bis zum letzten Element gel�scht
void delete_list() {
	WegPunkt *ptr, *ptrA;
	//�berpr�fung, ob n�chste Adresse einem Speicher zugewiesen ist
	if (g_Route->next != NULL) { 
		//Wenn ja, dann wird temp. ptr auf aktuelles g_Route verwiesen
		ptr = g_Route;									
		/*Solange jetzt das n�chste Element noch eine Adresse hat,
		wird diese Adresse zwischengespeichert und aktueller Speicher
		gel�scht*/
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
	//Um auch g_Route wieder zur�ckzusetzen, wird er mit NULL initialisiert
	g_Route = NULL;
}

////////////////////////////////////////////////////////////////////
// empf�ngt Strecke und speichert sie in mit add_list() ab
// TRUE, wenn erfolgreich, bei Fehler FALSE
bool GetData()
{
	static int data[20][3];
	static int i = 0;
	static int j = 0;
	String btData = "";

	Serial.print("GetData: i=");
	Serial.print(i);
	Serial.print("  j=");
	Serial.print(j);
	Serial.print("  btData=");
	Serial.println(btData);

	// l�scht alte Strecke
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
		// Umrechnung und zur liste hinzu f�gen
		for (int n = 0; n < 20; n++)
		{
			add_list(data[n][0], data[n][1] - data[n][2]);		// Daten in Liste speichern
			Serial.print("Weg: ");
			Serial.print(data[n][0]);
			Serial.print("Winkel: ");
			Serial.println(data[n][1] - data[n][2]);
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
	add_list(255,0);
	add_list(255,90);
	add_list(255,90);
	add_list(255,90);
	add_list(255,90);
	add_list(255,90);
	// Simulation
	delay(1000);
	return true;
#endif
}

////////////////////////////////////////////////////////////////////
// F�hrt �bergebenen Wegpunkt an
// true wenn ziel erreicht
// false, wenn durch stopp-Bit angehalten
bool Drive(double winkel, double strecke)
{
	double step = 0; // Speicher f�r die Anzahl der zu fahrenden steps

	// Setzen der Direction-Pins und Berechnung der zu fahrenden Steps
	if(winkel > 0)					//Linksdrehung
	{
		step = calc_deg(winkel);
		digitalWrite(PIN_OUT_Motor_R_DIR, LOW);
		digitalWrite(PIN_OUT_Motor_L_DIR, HIGH);
	}
	else if(winkel < 0)			//Rechtsdrehung
	{
		step = calc_deg(abs(winkel));
		digitalWrite(PIN_OUT_Motor_R_DIR, HIGH);
		digitalWrite(PIN_OUT_Motor_L_DIR, LOW);
	}
	if (step != 0) // Falls Drehung n�tig, diese Abfahren
	{
		delay(100); // Kurz warten bevor weiter gefahren wird damit �berg�nge nicht zu hart
		if (!move_via_step(step)) // Drehung ausf�hren, falls Fehler, dann diesen zur�ck geben und fahrt unterbrechen
			return false;
	}

	step = calc_step(strecke); //Geradeausfahren
	digitalWrite(PIN_OUT_Motor_R_DIR, HIGH);
	digitalWrite(PIN_OUT_Motor_L_DIR, HIGH);
	
	delay(100); // Kurz warten bevor weiter gefahren wird damit �berg�nge nicht zu hart
	return move_via_step(step);
}
////////////////////////////////////////////////////////////////////
// Berechnet Anzahl der Steps welche f�r den �bergebenen weg ben�tigt werden
double calc_step(double weg){
	Serial.print("g_StreckeProStep: ");
	Serial.println(g_StreckeProStep);
	return weg / g_StreckeProStep;
}
////////////////////////////////////////////////////////////////////
// Berechnet Anzahl der Steps welche f�r den �bergebenen winkel ben�tigt werden
double calc_deg(double deg){
	Serial.print("g_DrehwinkelProStep: ");
	Serial.println(g_DrehwinkelProStep);
	return deg / g_DrehwinkelProStep;
}
////////////////////////////////////////////////////////////////////
// Steuert Motortreiber mit der �bergebenen Anzahl an Steps an
// Kontrolliert w�hrend der Fahrt, ob Stop-Bit empfangen wird, falls ja stoppt die Ansteuerung des Motortreibers
// True wenn erfolgreich, false wenn durch stopp-bis angehalten
bool move_via_step(int step)
{	
	Serial.print("Step: ");
	Serial.println(step);
	TimerThree timer_Motor;
	// Timer f�r die steps initialisieren
	timer_Motor.initialize(5000); // 5000
	timer_Motor.pwm(PIN_OUT_Motoren_STEP, 512);
	attachInterrupt(digitalPinToInterrupt(PIN_IN_Motoren_STEP), Step_CNT, RISING);
	// Interrupt zum z�hlen der steps initialisieren
	g_Step_cnt = 0;
	// Warten bis Steps erreicht und kontinuierlich das Bluetooth modul abfragen
	while (g_Step_cnt < step) {
		delayMicroseconds(10);
		//Serial.print("Step_CNT: ");
		//Serial.println(g_Step_cnt);
		if (g_btSerial->available()) //Bluetooth Stop-Bit
		{
			timer_Motor.pwm(PIN_OUT_Motoren_STEP, 0);
			detachInterrupt(digitalPinToInterrupt(PIN_IN_Motoren_STEP));
			return false; // Alles Abbrechen, False zur�ck geben als Zeichen, dass Fahrt gescheitert
		}
	}
	detachInterrupt(digitalPinToInterrupt(PIN_IN_Motoren_STEP)); // Interrupt f�r Step-Z�hler deaktivieren
	timer_Motor.pwm(PIN_OUT_Motoren_STEP, 0);// PWM signal auf 0 setzen
	return true; // Fahrt erfolgreich
}

////////////////////////////////////////////////////////////////////
// z�hlt steps des Motortreibers anhand von steigenden Flanken des PWM signals
void Step_CNT() 
{
	g_Step_cnt++;
}

////////////////////////////////////////////////////////////////////
// Setzt LED Farbe je nach �bergebenen Status
void LED(eMainState State)
{
	// Hier wurde mit Common-Kathode LED gearbeitet (siehe http://propmakergen.blogspot.de/2016/03/rgb-led-tutorial.html)
	// Demnach HIGH-Activ!!
	// ACHTUNG!! Bei genutzter LED sind im gegensatz zu der in dem Link die Pins Gr�n und Blau vertauscht!
	// Reset
	digitalWrite(PIN_OUT_LED_R, LOW);
	digitalWrite(PIN_OUT_LED_G, LOW);
	digitalWrite(PIN_OUT_LED_B, LOW);
	// neue Farbe einstellen
	switch (State)
	{
	// Initialisierung -> Wei�
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
	// Fahrweg abfahren und �berpr�fen, ob noch auf Kurs -> Blau
	case Main_Drive:
	{			
		digitalWrite(PIN_OUT_LED_B, HIGH);
		break;
	}
	// Am Ziel angekommen -> Gr�n
	case Main_End:
	{
		digitalWrite(PIN_OUT_LED_G, HIGH);
		break;
	}
	// �bergabe durchf�hren -> Lila
	case Main_UBG:
	{
		digitalWrite(PIN_OUT_LED_R, HIGH);
		digitalWrite(PIN_OUT_LED_B, HIGH);
		break;
	}
	case Main_Error:
	default:
		digitalWrite(PIN_OUT_LED_R, LOW);
		break;
	}
}
