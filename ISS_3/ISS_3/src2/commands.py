import time
from utils import paczkowanie, send_with_backoff


import threading
import keyboard
import time

command_history = []
nasluchiwanieTestu = False



def start_key_listener(ard):

    def listener(arduino=ard):
        global nasluchiwanieTestu

        while True:
            if keyboard.is_pressed("s"):
                arduino.write(paczkowanie("STOP"))
                nasluchiwanieTestu = False
                break
            time.sleep(0.05)

    t = threading.Thread(target=listener, daemon=True)
    t.start()






def commandsHandler(cmd, arduino, arduinoOBJ) -> str:
    global command_history
    full_command = cmd
    
    arduinoOBJ.lock.acquire(blocking=True)
    
    match cmd:
        case "help":
            from menu import opis_komend
            opis_komend()
            

        case "history":
            print("Cała historia komend:")
            if not command_history:
                print("(pusta)")
            for i, c in enumerate(command_history, start=1):
                print(f"{i}: {c}")
            

        
        case "saveLogs":
            try:
                with open("commands_log.txt", "r", encoding="utf-8") as f:
                    print(f.read())
            except FileNotFoundError:
                print("Brak zapisanych logów.")
            
        
        case "usunsavelogs":
            try:
                open("commands_log.txt", "w").close()
                print("Logi zostały wyczyszczone.")
            except Exception as e:
                print("Błąd przy czyszczeniu logów:", e)
            
        
        
        case "status":
            saveLog(full_command)
            send_with_backoff(arduino, paczkowanie("STATUS"))

        case "v":
            while True:
                param = input("Podaj prędkość (0-255): ")
                if param.isdigit() and 0 <= int(param) <= 255:
                    break
                print("Błąd: podaj liczbę z zakresu 0–255.")
            full_command += f" {param}"
            saveLog(full_command)
            send_with_backoff(arduino, paczkowanie("V", param))

        case "przod":
            while True:
                param = input("Podaj odległość: ")
                if param.isdigit():
                    break
                print("Błąd: podaj liczbę.")
            full_command += f" {param}"
            saveLog(full_command)
            send_with_backoff(arduino, paczkowanie("PRZOD", param))

        case "tyl":
            while True:
                param = input("Podaj odległość: ")
                if param.isdigit():
                    break
                print("Błąd: podaj liczbę.")
            full_command += f" {param}"
            saveLog(full_command)
            send_with_backoff(arduino, paczkowanie("TYL", param))

        case "lewo":
            while True:
                param = input("Podaj krok obrotu: ")
                if param.isdigit():
                    break
                print("Błąd: podaj liczbę.")
            full_command += f" {param}"
            saveLog(full_command)
            send_with_backoff(arduino, paczkowanie("LEWO", param))

        case "prawo":
            while True:
                param = input("Podaj krok obrotu: ")
                if param.isdigit():
                    break
                print("Błąd: podaj liczbę.")
            full_command += f" {param}"
            saveLog(full_command)
            send_with_backoff(arduino, paczkowanie("PRAWO", param))

        case "stop":
            saveLog(full_command)
            send_with_backoff(arduino, paczkowanie("STOP"))

        case "b":
            saveLog(full_command)
            send_with_backoff(arduino, paczkowanie("B"))

        case "i":
            saveLog(full_command)
            send_with_backoff(arduino, paczkowanie("I"))
        
        case "dist":
            param = input("Ustaw docelowa odleglosc (TARGET): ")
            full_command += f" {param}"
            saveLog(full_command)
            send_with_backoff(arduino, paczkowanie("DIST", param))

        case "ustaw_p":
            param = input("Ustaw wartosc P: ")
            full_command += f" {param}"
            saveLog(full_command)
            send_with_backoff(arduino, paczkowanie("P", param))

        case "ustaw_i":
            param = input("Ustaw wartosc I: ")
            full_command += f" {param}"
            saveLog(full_command)
            send_with_backoff(arduino, paczkowanie("I", param))

        case "ustaw_d":
            param = input("Ustaw wartosc D: ")
            full_command += f" {param}"
            saveLog(full_command)
            send_with_backoff(arduino, paczkowanie("D", param))

        case "zero":
            param = input("Ustaw pozycje ZERO serwa: ")
            full_command += f" {param}"
            saveLog(full_command)
            send_with_backoff(arduino, paczkowanie("ZERO", param))

        case "lfjazda":
            saveLog(full_command)
            send_with_backoff(arduino, paczkowanie("LFJAZDA"))

        case "test":
            saveLog(full_command)
            send_with_backoff(arduino, paczkowanie("TEST"))

            nasluchiwanieTestu = True
            start_key_listener(arduino)

            while nasluchiwanieTestu:
                if arduino.in_waiting > 0:
                    line = arduino.readline().decode(errors='ignore').strip()

                    if "ENDENDEND" in line:
                        break

                    if line:
                        print("Odpowiedz Arduino:", line)
                        saveLog("Odpowiedz Arduino: " + line)


                else:
                    time.sleep(0.01)

            # while True:
            #     subcmd = input("Wpisz 'q' aby wyjsc z trybu testowego PID: ").strip()
            #     if subcmd == "q":
            #         commandsHandler("STOPTEST", arduino, arduinoOBJ)
            #         break

        case "run":
            saveLog(full_command)
            send_with_backoff(arduino, paczkowanie("RUN"))


        case "STOPTEST":
            saveLog(full_command)
            send_with_backoff(arduino, paczkowanie("STOPTEST"))


        


        

        case _:
            full_command = None

    
    arduinoOBJ.lock.release()

    if full_command:
        command_history.append(full_command)
        return full_command
        
    
    arduinoOBJ.lock.release()
    # if full_command and cmd not in ("history", "help", "saveLogs", "usunsavelogs"):
    #     command_history.append(full_command)
    #     

def saveLog(line, filename="commands_log.txt"):
    from datetime import datetime
    try:
        with open(filename, "a", encoding="utf-8") as f:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            f.write(f"{timestamp}  {line}\n")
    except Exception as e:
        print("Błąd zapisu do pliku:", e)
