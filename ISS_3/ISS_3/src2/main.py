import threading
from time import sleep

import serial
import time

from utils import paczkowanie, wybierz_port, wybierz_baud
from menu import *
from commands import commandsHandler, saveLog


restart_event = threading.Event()

class Arduino:
    def __init__(self, port=None, baud=None):
        self.port = port
        self.baud = baud
        self.lock = threading.Lock()
        self.ser = None
        if port and baud:
            self.connect()

    def connect(self):
        
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
                time.sleep(1)  
        except Exception as e:
            print("[Connect] Błąd przy zamykaniu starego portu:", e)

        try:
            if self.port and self.baud:
                self.ser = serial.Serial(self.port, self.baud, timeout=2)
                time.sleep(3)  # daj Arduino czas na inicjalizację
                print(f"[Connect] Połączono z {self.port} {self.baud}")
        except Exception as e:
            print("[Connect] Błąd połączenia:", e)
            self.ser = None

    def reconnect(self):
        print("[Arduino] Próba ponownego połączenia...")
        self.connect()


arduinoOBJ = Arduino(None, None)
watchdog_thread = None
watchdog_stop_event = threading.Event()


def watchdog_loop(arduinoOBJ):
    while not watchdog_stop_event.is_set():
        time.sleep(8)
        with arduinoOBJ.lock:
            
            if not arduinoOBJ.ser or not arduinoOBJ.ser.is_open:
                print("[Watchdog]: Port nieotwarty, czekam na reconnect...")
                continue
            try:
                arduinoOBJ.ser.write(paczkowanie("PING", info=False))
                saveLog("[Watchdog]: Wysłano PING")
                resp = arduinoOBJ.ser.readline().decode().strip()
                if resp:
                    saveLog("[Watchdog]: Arduino OK")
                else:
                    saveLog("[Watchdog]: Brak odpowiedzi!")
                    print("[Watchdog]: Brak odpowiedzi! Ustawiam restart...")
                    arduinoOBJ.reconnect()
                    
            except Exception as e:
                print("[Watchdog] Błąd:", e)
                print("[Watchdog]: Brak odpowiedzi! Ustawiam restart...")
                arduinoOBJ.reconnect()
                


def start_watchdog():
    global watchdog_thread
    if watchdog_thread is None or not watchdog_thread.is_alive():
        watchdog_stop_event.clear()
        watchdog_thread = threading.Thread(target=watchdog_loop, args=(arduinoOBJ,), daemon=True)
        watchdog_thread.start()


def stop_watchdog():
    watchdog_stop_event.set()
    if watchdog_thread:
        watchdog_thread.join()


def main():
    global arduinoOBJ
    
    
    arduinoOBJ.port = wybierz_port()
    arduinoOBJ.baud = wybierz_baud()
    arduinoOBJ.connect()
    

    start_watchdog()

    try:
        while True:
            
            if restart_event.is_set():
                print("[Main] Restart event ustawiony — wychodzę z main()...")
                break

            pokaz_menu(MENU_GLOWNE, "MENU GŁÓWNE")
            try:

                choice = int(input("Wybierz numer: "))
            except ValueError:
                print("Niepoprawny wybór")
                continue

            if choice not in MENU_GLOWNE:
                print("Niepoprawny numer komendy")
                continue

            desc, cmd = MENU_GLOWNE[choice]
            if cmd == "q":
                break

            if cmd == "sterowanie":
                while True:
                    pokaz_menu(MENU_STEROWANIE, "STEROWANIE ROBOTEM")
                    try:

                        sub_choice = int(input("Wybierz numer: "))
                    except ValueError:
                        print("Niepoprawny wybór")
                        continue

                    if sub_choice not in MENU_STEROWANIE:
                        print("Niepoprawny numer komendy")
                        continue

                    sub_desc, sub_cmd = MENU_STEROWANIE[sub_choice]
                    if sub_cmd == "back":
                        break

                    
                    if arduinoOBJ.ser and arduinoOBJ.ser.is_open:
                        commandsHandler(sub_cmd, arduinoOBJ.ser, arduinoOBJ)
                    else:
                        print("[Main] Port nieotwarty, komenda pominięta")
                        
            
            
            elif cmd == "pidmenu":
                while True:
                    pokaz_menu(MENU_PID, "USTAWIENIA PID I DYSTANSU")
                    try:

                        pid_choice = int(input("Wybierz numer: "))
                    except ValueError:
                        print("Niepoprawny wybor")
                        continue

                    if pid_choice not in MENU_PID:
                        print("Niepoprawny numer komendy")
                        continue

                    pid_desc, pid_cmd = MENU_PID[pid_choice]
                    if pid_cmd == "back":
                        break

                    if arduinoOBJ.ser and arduinoOBJ.ser.is_open:
                        commandsHandler(pid_cmd, arduinoOBJ.ser, arduinoOBJ)
                    else:
                        print("[Main] Port nieotwarty, komenda pominieta")

            elif cmd == "lfMenu":
                while True:
                    pokaz_menu(MENU_LF, "ŚLEDZENIE LINII")
                    try:
                        lf_choice = int(input("Wybierz numer: "))
                    except ValueError:
                        print("Niepoprawny wybor")
                        continue

                    if lf_choice not in MENU_LF:
                        print("Niepoprawny numer komendy")
                        continue

                    lf_desc, lf_cmd = MENU_LF[lf_choice]

                    if lf_cmd == "back":
                        break
                    if arduinoOBJ.ser and arduinoOBJ.ser.is_open:
                        commandsHandler(lf_cmd, arduinoOBJ.ser, arduinoOBJ)
                    else:
                        print("[Main] Port nieotwarty, komenda pominieta")
            
            
            else:
                
                if arduinoOBJ.ser and arduinoOBJ.ser.is_open:
                    commandsHandler(cmd, arduinoOBJ.ser, arduinoOBJ)
                else:
                    print("[Main] Port nieotwarty, komenda pominięta")

    except KeyboardInterrupt:
        print("Zakończenie programu.")
    finally:
        stop_watchdog()
        with arduinoOBJ.lock:
            if arduinoOBJ.ser and arduinoOBJ.ser.is_open:
                arduinoOBJ.ser.close()
        print("Rozłączono.")


if __name__ == "__main__":
    while True:
        restart_event.clear()
        main()

        if restart_event.is_set():
            print("[System] Restartuję Arduino i watchdog...\n")
            arduinoOBJ.reconnect()  
            restart_event.clear()
            continue
        else:
            print("[System] Program zakończony.")
            break
