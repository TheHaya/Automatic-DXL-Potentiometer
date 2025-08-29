import tkinter as tk
from tkinter import ttk
import sv_ttk
import serial, time, threading
from itertools import cycle
import pandas as pd
import xlsxwriter
calc_win = None

# --------------- SERIAL MIT SERVO

labels = [  "Mittelposition",
            "Start aktiver Bereich CW (Drehrichtung-)",
            "50° CW (25% aktiver Bereich)(Drehrichtung-)",
            "75° CW (50% aktiver Bereich)(Drehrichtung-)",
            "100° CW (75% aktiver Bereich)(Drehrichtung-)",
            "Ende aktiver Bereich CW (Drehrichtung-)(11) 10V",
            "Mechanisches Ende CW (Drehrichtung-)",
            "Start aktiver Bereich CCW (Drehrichtung+)",
            "50° CCW (25% aktiver Bereich)(Drehrichtung+)",
            "75° CCW (50% aktiver Bereich)(Drehrichtung+)",
            "100° CCW (75% aktiver Bereich)(Drehrichtung+)",
            "Ende aktiver Bereich CCW (Drehrichtung+)(13) 0V",
            "Mechanisches Ende CCW (Drehrichtung+)"]
labels_iter = cycle(labels)

def open_first_available(ports=("COM6", "COM3"), baud=115200, timeout=2):
    last = None
    for p in ports:
        try:
            ser = serial.Serial(p, baudrate=baud, timeout=timeout)
            print(f"[SERIAL] Verbunden: {p}")
            time.sleep(1)
            return ser
        except Exception as e:
            last = e
    raise RuntimeError(f"Kein Port aus {ports} verfügbar: {last}")

def write_serial(gesamtV, gesamtW, d11, d12, d21, d22, d31, d32, stop_event, on_finish):
    try:
        ser = open_first_available(("COM6","COM3"), baud=115200, timeout=5)
        daten = []
        time.sleep(1)
        ser.write(f"SETV:{gesamtV}\n".encode())
        time.sleep(0.2)
        ser.write(f"SETW:{gesamtW}\n".encode())
        time.sleep(0.2)
        ser.write(f"dead11:{d11}\n".encode())
        time.sleep(0.2)
        ser.write(f"dead12:{d12}\n".encode())
        time.sleep(0.2)
        ser.write(f"dead21:{d21}\n".encode())
        time.sleep(0.2)
        ser.write(f"dead22:{d22}\n".encode())
        time.sleep(0.2)
        ser.write(f"dead31:{d31}\n".encode())
        time.sleep(0.2)
        ser.write(f"dead32:{d32}\n".encode())
        time.sleep(0.2)
        print("Sende: GO") #debug
        #print(d11, d12, d21, d22, d31, d32)
        ser.write(b"GO\n")

        ser.timeout = 0.1
        while True:
            if stop_event.is_set():
                ser.write(b"STOP\n")
                ser.flush()
                time.sleep(0.05)
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
                    istspannung = float(parts[2].split(":")[1])
                    istwinkel = float(parts[3].split(":")[1])
                    linear = float(parts[4].split(":")[1])
                    daten.append([sollwinkel, sollspannung, istspannung, istwinkel, linear])
                except Exception as e:
                    print("Fehler beim Parsen:", e) #debug
                    continue

        ser.close()

        if not stop_event.is_set():
            rows = []
            for(sollwinkel, sollspannung, istspannung, istwinkel, linear), label in zip(daten, labels_iter):
                rows.append({
                    " ": label,
                    "Soll-Winkel [°]": round(sollwinkel, 1),
                    "Soll-Spannung [V]": round(sollspannung, 2),
                    "Ist-Spannung [V]": round(istspannung, 3),
                    "Ist-Winkel [°]": round(istwinkel, 1),
                    "Linearität":  float(linear)
                })
            df = pd.DataFrame(rows)

            with pd.ExcelWriter("Alwin-RMTest-"+txt9.get()+".xlsx", engine="xlsxwriter") as writer:
                sheet = "Messung"
                df.to_excel(writer, index=False, sheet_name=sheet)
                wb = writer.book
                ws = writer.sheets[sheet]

                format_percent = wb.add_format({'num_format': '0.00%','align': 'center'})
                format_degree = wb.add_format({'num_format': '0.0°','align': 'center'})
                format_volt2 = wb.add_format({'num_format': '0.00','align': 'center'})
                format_volt3 = wb.add_format({'num_format': '0.000','align': 'center'})

                ws.set_column('A:A', 44)
                ws.set_column('B:B', 20, format_degree)
                ws.set_column('C:C', 20, format_volt2)
                ws.set_column('D:D', 20, format_volt3)
                ws.set_column('E:E', 20, format_degree)
                ws.set_column('F:F', 20, format_percent)

                ws.freeze_panes(1,0)
    except Exception as e:
        print("Fehler bei Serial: ", e) #debug

    root.after(0, on_finish)

def close_window():
    root.destroy()

# --------------- CALC BUTTON

def open_calc_win():
    def close_wait_results():
        wait_win.destroy()

        if stop_event.is_set():
            global cancelled_win
            
            cancelled_win = tk.Toplevel(root)
            cancelled_win.title("Abbruch")
            cancelled_win.geometry(f"{scrwid//4}x{scrhei//4}+{scrwid//2}+{scrhei//2}")
            cancelled_win.grid_rowconfigure(0, weight=1)
            cancelled_win.grid_rowconfigure(1, weight=1)
            cancelled_win.grid_columnconfigure(0, weight=1)
            
            ttk.Label(cancelled_win, text="Vorgang wurde abgebrochen.").grid(row=0, column=0)
            ok_button = ttk.Button(cancelled_win, text="OK", command=cancelled_win.destroy)
            ok_button.grid(row=1, column=0, pady=(0, 20), ipadx=20)
            ok_button.focus_set()  
            cancelled_win.bind("<Return>", lambda event: ok_button.invoke())
        else:
            global calc_win
            if calc_win is not None and calc_win.winfo_exists():
                calc_win.destroy()

            calc_win = tk.Toplevel(root)
            calc_win.title("Fertig")
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
        txtDead31 = float(txt7.get().strip().replace(',', '.'))
        txtDead32 = float(txt8.get().strip().replace(',', '.'))
        
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
    threading.Thread(target=write_serial, args=(txtSoll, txtWinkel, txtDead11, txtDead12,
                                                 txtDead21, txtDead22, txtDead31, txtDead32,
                                                   stop_event, close_wait_results), daemon=True).start()

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
txt1.insert(0, "10")
txt1.focus_set()

ttk.Label(main_frame, text="Gesamtwinkel:").grid(row=0, column=1, sticky="w", pady=(0, 2),ipadx=20)
txt6 = ttk.Entry(main_frame, width=20, validate="key", validatecommand=vcmd)
txt6.grid(row=1, column=1, pady=(0, 10))
txt6.insert(0, "330")

ttk.Label(main_frame, text="Anfang Deadzone 1:").grid(row=2, column=0, sticky="w", pady=(0, 2))
txt2 = ttk.Entry(main_frame, width=20, validate="key", validatecommand=vcmd)
txt2.grid(row=3, column=0, pady=(0, 10))
txt2.insert(0, "0")

ttk.Label(main_frame, text="Ende Deadzone 1:").grid(row=2, column=1, sticky="w", pady=(0, 2),ipadx=20)
txt3 = ttk.Entry(main_frame, width=20, validate="key", validatecommand=vcmd)
txt3.grid(row=3, column=1, pady=(0, 10))
txt3.insert(0, "40")

ttk.Label(main_frame, text="Anfang Deadzone 2:").grid(row=4, column=0, sticky="w", pady=(0, 2))
txt4 = ttk.Entry(main_frame, width=20, validate="key", validatecommand=vcmd)
txt4.grid(row=5, column=0, pady=(0, 10))
txt4.insert(0, "140")

ttk.Label(main_frame, text="Ende Deadzone 2:").grid(row=4, column=1, sticky="w", pady=(0, 2),ipadx=20)
txt5 = ttk.Entry(main_frame, width=20, validate="key", validatecommand=vcmd)
txt5.grid(row=5, column=1, pady=(0, 10))
txt5.insert(0, "190")

ttk.Label(main_frame, text="Anfang Deadzone 3:").grid(row=6, column=0, sticky="w", pady=(0, 2))
txt7 = ttk.Entry(main_frame, width=20, validate="key", validatecommand=vcmd)
txt7.grid(row=7, column=0, pady=(0, 10))
txt7.insert(0, "290")

ttk.Label(main_frame, text="Ende Deadzone 3:").grid(row=6, column=1, sticky="w", pady=(0, 2),ipadx=20)
txt8 = ttk.Entry(main_frame, width=20, validate="key", validatecommand=vcmd)
txt8.grid(row=7, column=1, pady=(0, 10))
txt8.insert(0, "330")

ttk.Label(main_frame, text="Name Teil:").grid(row=8, column=0, sticky="w", pady=(0, 2),ipadx=20)
txt9 = ttk.Entry(main_frame, width=20)
txt9.grid(row=9, column=0, pady=(0, 10))
txt9.insert(0, "T107357")

ttk.Button(main_frame, text="OK", command=close_window).grid(row=10, column=0, pady=(0, 5), ipadx=20)
ttk.Button(main_frame, text="Calc", command=open_calc_win).grid(row=10, column=1, pady=5, ipadx=10)

txt1.bind("<Return>", lambda event: open_calc_win())
txt2.bind("<Return>", lambda event: open_calc_win())
txt3.bind("<Return>", lambda event: open_calc_win())
txt4.bind("<Return>", lambda event: open_calc_win())
txt5.bind("<Return>", lambda event: open_calc_win())
txt6.bind("<Return>", lambda event: open_calc_win())
txt7.bind("<Return>", lambda event: open_calc_win())
txt8.bind("<Return>", lambda event: open_calc_win())
root.bind("<Escape>", lambda event: close_window())

sv_ttk.set_theme("dark")
root.mainloop()