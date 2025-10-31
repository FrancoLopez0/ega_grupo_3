import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Parámetros
fs = 800  # Frecuencia de muestreo en Hz

# Cargar el archivo CSV
df = pd.read_csv("raw.csv")

# Extraer la columna 'Value'
signal = df['Value'].values
N = len(signal)

# Eliminar componente continua (DC) y normalizar
signal = signal - np.mean(signal)       # Quitar la media
signal = signal / np.max(np.abs(signal))  # Normalización [-1, 1]

# Calcular la FFT
fft_values = np.fft.fft(signal)
fft_magnitude = np.abs(fft_values) / N  # Normalización por número de muestras
frequencies = np.fft.fftfreq(N, d=1/fs)

# Tomar solo la mitad positiva del espectro
half_N = N // 2
frequencies = frequencies[:half_N]
fft_magnitude = fft_magnitude[:half_N]

# Graficar
plt.figure(figsize=(10, 6))
plt.plot(frequencies, fft_magnitude)
plt.title("Espectro de Magnitud (FFT) - Normalizado y sin DC")
plt.xlabel("Frecuencia (Hz)")
plt.ylabel("Magnitud")
plt.grid(True)
plt.show()