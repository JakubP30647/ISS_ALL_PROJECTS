import serial.tools.list_ports
import time

# ========================== PORT I BAUD ==========================

def wybierz_port():
    ports = [p.device for p in serial.tools.list_ports.comports()]
    if not ports:
        print("Brak portów. Użyto COM1")
        return "COM1"
    
    while True:
        print("Dostępne porty:")
        for i, p in enumerate(ports, start=1):
            print(f"{i}. {p}")
        try:
            idx = int(input("Wybierz numer portu: "))
            if 1 <= idx <= len(ports):
                return ports[idx - 1]
        except ValueError:
            pass
        print("Niepoprawny numer. Spróbuj ponownie.")

def wybierz_baud():
    while True:
        baud_input = input("Podaj prędkość (np. 9600): ")
        if baud_input.isdigit() and 0 < int(baud_input) <= 1000000:
            return int(baud_input)
        else:
            print("Niepoprawna prędkość baud. Spróbuj ponownie.")

# ========================== SUMA KONTROLNA I WYSYŁANIE ==========================

def ControlSUM(data: str) -> int:
    return sum(data.encode()) % 256

def paczkowanie(command: str, param: str = "", info=True) -> bytes:
    zawartosc = f"{command}:{param}" if param else command
    checksum = ControlSUM(zawartosc)
    frame = f"\x02{zawartosc}:{checksum}\x03"
    if info:
        print("wysyłam:", zawartosc, " CS:", checksum)
    
    return frame.encode()



def send_with_backoff(arduino, data, total_timeout=5, arduinoOBJ=None):
    
    
    import time
    from commands import saveLog
    
    delay = 1

    for attempt in range(5):
        try:
            arduino.reset_input_buffer()
            arduino.write(data)
            time.sleep(0.1)

            responses = []
            start_time = time.time()

            while True:
                if arduino.in_waiting > 0:
                    line = arduino.readline().decode(errors='ignore').strip()
                    
                    if "ENDENDEND" in line:
                        break
                    
                    if line:
                        
                        print("Odpowiedz Arduino:", line)
                        saveLog("Odpowiedz Arduino: " + line)
                        responses.append(line)
                        
                    start_time = time.time()  
                elif time.time() - start_time > total_timeout:
                    break
                else:
                    time.sleep(0.01) 

            if responses:
                return responses
            else:
                print(f"Brak odpowiedzi, próba {attempt+1}")

        except Exception as e:
            print("Błąd:", e)

        print(f"Czekam {delay} sekund przed kolejną próbą...")
        time.sleep(delay)
        delay *= 2

    print("Nie udało się połączyć po kilku próbach")
    return None


