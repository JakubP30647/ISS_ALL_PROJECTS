MENU_GLOWNE = {
    1: ("Pomoc (lista komend)", "help"),
    2: ("Pokaż historię komend", "history"),
    3: ("Przejdź do sterowania robotem", "sterowanie"),
    4: ("Odczytranie Save logs", "saveLogs"),
    5: ("Wyczyszczenie Save logs", "usunsavelogs"),
    6: ("Menu ustawień PID i dystansu", "pidmenu"),
    7: ("Menu LF", "lfMenu"),
    0: ("Zakończ program", "q")
}


MENU_STEROWANIE = {
    1: ("Status robota", "status"),
    2: ("Zmień prędkość", "v"),
    3: ("Jazda do przodu", "przod"),
    4: ("Jazda do tyłu", "tyl"),
    5: ("Skręt w lewo", "lewo"),
    6: ("Skręt w prawo", "prawo"),
    7: ("STOP", "stop"),
    8: ("Odczyt sonaru (cm)", "b"),
    9: ("Odczyt czujnika IR", "i"),
    0: ("Powrót do menu głównego", "back")
}

MENU_HELP = {
    1: ("Status robota", "status"),
    2: ("Zmień prędkość", "v"),
    3: ("Jazda do przodu", "przod"),
    4: ("Jazda do tyłu", "tyl"),
    5: ("Skręt w lewo", "lewo"),
    6: ("Skręt w prawo", "prawo"),
    7: ("STOP", "stop"),
    8: ("Odczyt sonaru (cm)", "b"),
    9: ("Odczyt czujnika IR", "i"),
    0: ("Powrót do menu głównego", "back")
}

MENU_PID = {
    1: ("Ustaw docelową odległość (TARGET)", "dist"),
    2: ("Ustaw wartość P", "ustaw_p"),
    3: ("Ustaw wartość I", "ustaw_i"),
    4: ("Ustaw wartość D", "ustaw_d"),
    5: ("Ustaw pozycję ZERO serwa", "zero"),
    6: ("Tryb testowy PID (telemetria)", "test"),
    7: ("Tryb zaliczeniowy (10s + MAE)", "run"),
    8: ("STOP", "STOPTEST"),
    0: ("Powrót do menu głównego", "back")
}


MENU_LF = {
    1: ("Rozpocznij śledzenie linii", "lfjazda"),
    0: ("Powrót do menu głównego", "back")
}




def pokaz_menu(menu, tytul):
    print("\n╔══════════════════════════════════════════╗")
    print(f"║    {tytul.center(36)}  ║")
    print("╠══════════════════════════════════════════╣")
    for num, (desc, _) in menu.items():
        print(f"║ {str(num).rjust(2)} - {desc.ljust(34)}  ║")
    print("╚══════════════════════════════════════════╝")


def opis_komend():
    print("     ╔" + "═" * 99 + "╗")
    print("     ║" + " OPIS DOSTĘPNYCH KOMEND ROBOTA ".center(99) + "║")
    print("     ╚" + "═" * 99 + "╝")
    print("╔════╦════════════════════════════╦════════════╦════════════════════════════════════════════════════════════════╗")
    print("║ Nr ║ Komenda                    ║ W SYSTEMIE ║ Opis                                                           ║")
    print("╠════╬════════════════════════════╬════════════╬════════════════════════════════════════════════════════════════╣")
    print("║ 1. ║ Status robota              ║ 'status'   ║ Wyświetla aktualny stan robota (prędkość, kierunek, czujniki)  ║")
    print("║ 2. ║ Zmień prędkość             ║ 'v'        ║ Umożliwia ustawienie nowej prędkości jazdy robota              ║")
    print("║ 3. ║ Jazda do przodu            ║ 'przod'    ║ Uruchamia ruch robota do przodu                                ║")
    print("║ 4. ║ Jazda do tyłu              ║ 'tyl'      ║ Uruchamia ruch robota do tyłu                                  ║")
    print("║ 5. ║ Skręt w lewo               ║ 'lewo'     ║ Obraca robota w lewo                                           ║")
    print("║ 6. ║ Skręt w prawo              ║ 'prawo'    ║ Obraca robota w prawo                                          ║")
    print("║ 7. ║ STOP                       ║ 'stop'     ║ Natychmiast zatrzymuje robota                                  ║")
    print("║ 8. ║ Odczyt sonaru (cm)         ║ 'b'        ║ Wyświetla pomiar z czujnika odległości (sonaru)                ║")
    print("║ 9. ║ Odczyt czujnika IR         ║ 'i'        ║ Wyświetla dane z czujnika podczerwieni (IR)                    ║")
    print("║10. ║ Powrót do menu głównego    ║ 'back'     ║ Powraca do głównego ekranu sterowania                          ║")
    print("╚════╩════════════════════════════╩════════════╩════════════════════════════════════════════════════════════════╝")

    

    