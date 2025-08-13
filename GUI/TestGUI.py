import tkinter as tk
from tkinter import ttk
import sv_ttk
import csv, serial, time, threading

calc_win = None

# --------------- SERIAL MIT SERVO

def write_serial(gesamtV, gesamtW, stop_event, on_finish):
    try:
        ser = serial.Serial('COM3', 115200, timeout=2)
        daten = []
        time.sleep(1)
        ser.write(f"SETV:{gesamtV}\n".encode())
        time.sleep(0.2)
        ser.write(f"SETW:{gesamtW}\n".encode())
        time.sleep(0.2)
        print("Sende: GO") #debug
        ser.write(b"GO\n")

        while True:
            if stop_event.is_set():
                ser.write(b"STOP\n")
                break

            line = ser.readline().decode('utf-8').strip()
            print("Empfangen:", line) #debug
            if line == 'READY':
                break
            elif line == 'CANCEL':
                break
            if line:
                try:
                    parts = line.split(";")
                    sollwinkel = float(parts[0].split(":")[1])
                    sollspannung = float(parts[1].split(":")[1])
                    winkel = float(parts[2].split(":")[1])
                    spannung = float(parts[3].split(":")[1])
                    fehler_abs = float(parts[4].split(":")[1])
                    fehler_rel = float(parts[5].split(":")[1])
                    daten.append([sollwinkel, sollspannung, winkel, spannung, fehler_abs, fehler_rel])
                except Exception as e:
                    print("Fehler beim Parsen:", e) #debug
                    continue

        ser.close()

        if not stop_event.is_set():
            with open("Results.csv", "w", newline="") as file:
                writer = csv.writer(file, delimiter=';')
                writer.writerow(["Soll-Winkel", "Soll-Spannung", "Winkel", "Spannung", "Fehler-Absolut", "Fehler-Relativ"])
                for sollwinkel, sollspannung, winkel, spannung, fehler_abs, fehler_rel in daten:
                    writer.writerow([
                        f"{sollwinkel:.1f}".replace('.', ','), 
                        f"{sollspannung:.5f}".replace('.', ','),
                        f"{winkel:.1f}".replace('.', ','), 
                        f"{spannung:.5f}".replace('.', ','),
                        f"{fehler_abs:.5f}".replace('.', ','),
                        f"{fehler_rel:.1f}".replace('.', ',')]) 
    except Exception as e:
        print("Fehler bei Serial: ", e) #debug

    root.after(0, on_finish)

def close_window():
    root.destroy()

# --------------- CALC BUTTON

def open_calc_win():
    def close_wait_results():
        wait_win.destroy()

        global calc_win
        if calc_win is not None and calc_win.winfo_exists():
            calc_win.destroy()

        calc_win = tk.Toplevel(root)
        calc_win.title("New window")
        calc_win.geometry(f"{scrwid//4}x{scrhei//4}+{scrwid//2}+{scrhei//2}")
        calc_win.grid_rowconfigure(0, weight=1)
        calc_win.grid_rowconfigure(1, weight=1)
        calc_win.grid_columnconfigure(0, weight=1)
        
        ttk.Label(calc_win, text="Messung erfolgreich!").grid(row=0, column=0)
        ok_button = ttk.Button(calc_win, text="OK", command=calc_win.destroy)
        ok_button.grid(row=1, column=0, pady=(0, 20), ipadx=20)
        ok_button.focus_set()  
        calc_win.bind("<Return>", lambda event: ok_button.invoke())

    try:
        txtSoll = float(txt1.get().strip().replace(',', '.'))
        txtWinkel = float(txt6.get().strip().replace(',', '.'))
        txtDead11 = float(txt2.get().strip().replace(',', '.'))
        txtDead12 = float(txt3.get().strip().replace(',', '.'))
        txtDead21 = float(txt4.get().strip().replace(',', '.'))
        txtDead22 = float(txt5.get().strip().replace(',', '.'))
        
    except ValueError:
        error_win = tk.Toplevel(root)
        error_win.title("Falsche Eingabe!")
        error_win.geometry(f"{scrwid//8}x{scrhei//8}+{scrwid//2}+{scrhei//2}")
        error_win.resizable(False, False)
        error_win.transient(root)
        error_win.grab_set()
        error_win.grid_rowconfigure(0, weight=1)
        error_win.grid_rowconfigure(1, weight=1)
        error_win.grid_columnconfigure(0, weight=1)
        error_win.bell()
        ttk.Label(error_win, text="Leeres Feld gefunden!").grid(row=0, column=0)
        ok_button = ttk.Button(error_win, text="OK", command=error_win.destroy)
        ok_button.grid(row=1, column=0, ipadx=20)
        ok_button.focus_set()
        error_win.bind("<Return>", lambda event: ok_button.invoke())
        return
        
    wait_win = tk.Toplevel(root)
    wait_win.title("Datenmessung")
    wait_win.geometry(f"{scrwid//8}x{scrhei//8}+{scrwid//2}+{scrhei//2}")
    wait_win.transient(root)
    wait_win.grab_set()
    wait_win.resizable(False, False)
    ttk.Label(wait_win, text="Bitte warten...").pack(pady=30)

    stop_event = threading.Event()    
    def cancel_close():
        stop_event.set()
        wait_win.destroy()
    wait_win.protocol("WM_DELETE_WINDOW", cancel_close)
    threading.Thread(target=write_serial, args=(txtSoll, txtWinkel, stop_event, close_wait_results), daemon=True).start()

# --------------- GUI

root = tk.Tk()
scrwid = root.winfo_screenwidth()
scrhei = root.winfo_screenheight()
root.geometry(f"{scrwid}x{scrhei}+0+0")
root.title("Test window")
root.resizable(False, False)

root.grid_rowconfigure(0, weight=1)
root.grid_columnconfigure(0, weight=1)

main_frame = ttk.Frame(root)
main_frame.grid(row=0, column=0)

vcmd = (root.register(lambda P: (P.count(',') <= 1 and all(ch.isdigit() or ch == ',' for ch in P))), "%P")

ttk.Label(main_frame, text="Sollspannung:").grid(row=0, column=0, sticky="w", pady=(0, 2))
txt1 = ttk.Entry(main_frame, width=20, validate="key", validatecommand=vcmd)
txt1.grid(row=1, column=0, pady=(0, 10))
txt1.focus_set()

ttk.Label(main_frame, text="Gesamtwinkel:").grid(row=0, column=1, sticky="w", pady=(0, 2),ipadx=20)
txt6 = ttk.Entry(main_frame, width=20, validate="key", validatecommand=vcmd)
txt6.grid(row=1, column=1, pady=(0, 10))

ttk.Label(main_frame, text="Anfang Deadzone 1:").grid(row=2, column=0, sticky="w", pady=(0, 2))
txt2 = ttk.Entry(main_frame, width=20, validate="key", validatecommand=vcmd)
txt2.grid(row=3, column=0, pady=(0, 10))

ttk.Label(main_frame, text="Ende Deadzone 1:").grid(row=2, column=1, sticky="w", pady=(0, 2),ipadx=20)
txt3 = ttk.Entry(main_frame, width=20, validate="key", validatecommand=vcmd)
txt3.grid(row=3, column=1, pady=(0, 10))

ttk.Label(main_frame, text="Anfang Deadzone 2:").grid(row=4, column=0, sticky="w", pady=(0, 2))
txt4 = ttk.Entry(main_frame, width=20, validate="key", validatecommand=vcmd)
txt4.grid(row=5, column=0, pady=(0, 10))

ttk.Label(main_frame, text="Ende Deadzone 2:").grid(row=4, column=1, sticky="w", pady=(0, 2),ipadx=20)
txt5 = ttk.Entry(main_frame, width=20, validate="key", validatecommand=vcmd)
txt5.grid(row=5, column=1, pady=(0, 10))

ttk.Button(main_frame, text="OK", command=close_window).grid(row=6, column=1, pady=(0, 5), ipadx=20)
ttk.Button(main_frame, text="Calc", command=open_calc_win).grid(row=7, column=1, pady=5, ipadx=10)

txt1.bind("<Return>", lambda event: open_calc_win())
txt2.bind("<Return>", lambda event: open_calc_win())
txt3.bind("<Return>", lambda event: open_calc_win())
root.bind("<Escape>", lambda event: close_window())

sv_ttk.set_theme("dark")
root.mainloop()