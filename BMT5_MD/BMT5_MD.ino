/*
 Name:		BMT5_MD.ino
 Created:	26.09.2016 18:04:53
 Author:	Eike
*/
#include "Firmware\MainData.h"
#include <Servo.h>
using namespace std;

eMainState Main_State;

void setup()
{
	// globale Variablen initialisieren
	g_Step_cnt		= 0;
	g_Roboter_Name	= ALPHA;
	Main_State		= Main_Init;

	// Faktoren für Motortreiber berechnen
	Calc_factors_for_motordrive();

	// PinMode Init-Pin
	pinMode(PIN_IN_Ident, INPUT);
	// PinMode Status LED
	pinMode(PIN_OUT_LED_R, OUTPUT);
	pinMode(PIN_OUT_LED_G, OUTPUT);
	pinMode(PIN_OUT_LED_B, OUTPUT);
	// PinMode Motoren
	pinMode(PIN_OUT_Motoren_STEP, OUTPUT);
	pinMode(PIN_OUT_Motor_R_DIR, OUTPUT);
	pinMode(PIN_OUT_Motor_L_DIR, OUTPUT);
	// PinMode Bluetooth-Module

	// PinMode Taster UBG
	pinMode(PIN_OUT_SERVO, OUTPUT);

	g_btSerial = new SoftwareSerial(PIN_IN_RX, PIN_IN_TX);

	g_btSerial->begin(38400);
	Serial.begin(9600);
}

void loop() 
{
	LED(Main_State); // setzt status LED
	//Serial.println(Main_State);
	switch (Main_State)
	{
		/////////////////////////////////////////////////////////////////////////////////////////
		// Initialisierung
		// LED Weiß
		case Main_Init:
		{
			Serial.println("MainInit");
			// Servo auf 180 grad drehen
			//myservo.write(180);

			delay(1000);
			// Roboternamen ermitteln
			if (digitalRead(PIN_IN_Ident) == HIGH)
				g_Roboter_Name = ALPHA;
			else
				g_Roboter_Name = OMEGA;
			Main_State = Main_Idle;
			break;
		}
		/////////////////////////////////////////////////////////////////////////////////////////
		// Fahrweg empfangen
		// LED Gelb
		case Main_Idle: 
		{
			Serial.println("MainIdle");
			
			while (g_btSerial->available())
			{
				if (GetData()) // Daten erfolgreich empfangen
					Main_State = Main_Drive;
			}
			break;
		}
		/////////////////////////////////////////////////////////////////////////////////////////
		// Fahrweg abfahren und überprüfen, ob noch auf Kurs
		// LED Blau
		case Main_Drive:
		{
			bool Drive_Success = false; // Merker, ob fahrt erfolgreich
			Serial.println("MainDrive");
			WegPunkt *ptr = g_Route;
			if ((ptr == NULL) /*|| (g_ERROR_Flag) */) {
				//Fehler, da keine route gespeichtert
				//oder vom Rechner unterbrochen wegen Routenabweichung
				//Fehlerausgabe LED
				Main_State = Main_Error;
			}
			else //Durchlaufen der gespeicherten Route
			{			
				do 
				{
					//Serial.println(ptr->m_RelWinkel);
					//Serial.println(ptr->m_strecke);
					Drive_Success = Drive(ptr->m_RelWinkel, ptr->m_strecke);
					if (!Drive_Success) // Fahrt abbrechen wenn Drive aufgrund von Stop-Befehl anhält
						break;
					ptr = ptr->next;
				} while (ptr->next != NULL);
			}
			if (Drive_Success) // Fahrt erfolgreich, also am Ziel angekommen
			{
				if (g_UBG_Flag) // Übergabe durchführen
					Main_State = Main_UBG;
				else
					Main_State = Main_End;
			}
			else // Fahrt nicht erfolgreich, also durch Hauptrechner Stop-Befehl bekommen -> Neue Route empfangen
				Main_State = Main_Idle;

			break;
		}
		/////////////////////////////////////////////////////////////////////////////////////////
		// Am Ziel angekommen
		// LED Grün
		case Main_End:
		{
			Serial.println("MainEnd");

			// 5 Sekunden Warten, danach wieder in Idle -> kann neue Strecke empfangen
			delay(5000);
			Main_State = Main_Idle;
			break;
		}
		/////////////////////////////////////////////////////////////////////////////////////////
		// Übergabe durchführen
		// LED Lila
		case Main_UBG:
		{
			Serial.println("MainUBG");
			Servo myservo;
			myservo.attach(PIN_OUT_SERVO);

			for (int pos = 180; pos > 90; pos--)
			{
				myservo.write(pos);
				delay(15);
			}

			myservo.detach();

			delay(1000);

			// Übergabebit auf 0 setzen und senden
			Main_State = Main_Idle;

			break;
		}
		/////////////////////////////////////////////////////////////////////////////////////////
		// Error
		// LED Rot
		case Main_Error:
		{
			Serial.println("MainError");

			//5 Sekunden warten, danach wieder in Idle um neue Strecke zu empfangen
			delay(5000);
			Main_State = Main_Idle;
			break;
		}
		
		/////////////////////////////////////////////////////////////////////////////////////////
		/*
		case Main_Reverse:
		{
			hier vllt noch case einbringen, der im Falle von annaehender Kollision einfach die letzten Motorbewegungen
			wieder zurückfährt? Dann wäre Abstand gewährleistet und die Routenberechnung kann eine
			neue Ideallinie berechnen --> Könnte aber auch in den Main_Error miteingebaut werden (dann einfach if-else in ABhängigkeit 
			von g_ERROR_Flag)
		}
		
		*/
		
		default:
		break;
	}
}