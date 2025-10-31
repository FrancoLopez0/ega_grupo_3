import serial
import csv
import matplotlib.pyplot as plt
import signal
import sys

PORT = "COM8"
BAUDRATE = 9600
CSV_FILE = 'raw.csv'
TIMEOUT = 1         # Tiempo de espera para lectura (segundos)

data = {
    "lux":[],
    "bh1750":[],
    "temt6000":[],
}

def write_csv(file_name, col, row):
    with open(file_name, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(col)
        writer.writerows(row)

def signal_handler(sig, frame):
    """Maneja la señal de interrupción (Ctrl+C) para terminar el programa."""
    global running
    print("\nDeteniendo la adquisición...")
    running = False

if __name__ == "__main__":
    global running

    running = True

    # Configurar el manejador de señales (Ctrl+C)
    signal.signal(signal.SIGINT, signal_handler)

    # Inicializar conexión serial
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT)
        print(f"Conectado a {ser.name}. Presiona Ctrl+C para detener y graficar.")
    except serial.SerialException as e:
        print(f"Error al abrir el puerto serial: {e}")
        sys.exit(1)


     # Leer datos hasta que se presione Ctrl+C
    try:
        time = 0
        while running:

            line = ser.readline().decode('utf-8').strip()
            
            row = line.split(",")

            data["lux"].append(row[0])
            data["bh1750"].append(row[1]) 
            data["temt6000"].append(row[2]) 

            if line:  # Ignorar líneas vacías
                try:
                    print(f"Valor leído: {data}")  # Opcional: mostrar progreso
                    time += 1
                    if time == 2000: 
                        break
                except ValueError:
                    print(f"Dato no válido: {line}")
    finally:
        ser.close()
        write_csv(CSV_FILE, list(data.keys()), list(data.values()))
