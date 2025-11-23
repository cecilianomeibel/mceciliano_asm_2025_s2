# Importa las librerías necesarias
import numpy as np
import matplotlib.pyplot as plt

# Clase para modulación y demodulación FSK
class FSKModulatorDemodulator:
    def __init__(self, freq_bit0, freq_bit1, sampling_rate=10000):
        """
        Inicializa el modulador/demodulador FSK.
        freq_bit0: Frecuencia para el bit 0 (Hz)
        freq_bit1: Frecuencia para el bit 1 (Hz)
        sampling_rate: Frecuencia de muestreo (Hz)
        """
        self.freq_bit0 = freq_bit0
        self.freq_bit1 = freq_bit1
        self.sampling_rate = sampling_rate


    def modulate(self, bit_sequence, bit_duration):
        """
        Modula una secuencia de bits usando FSK.
        bit_sequence: lista de bits a transmitir
        bit_duration: duración de cada bit (segundos)
        Devuelve: vector de tiempo y señal modulada
        """
        t = np.arange(0, bit_duration * len(bit_sequence), 1 / self.sampling_rate)  # Vector de tiempo
        signal = np.zeros_like(t)  # Señal inicializada en cero

        for i, bit in enumerate(bit_sequence):
            freq = self.freq_bit1 if bit == 1 else self.freq_bit0  # Selecciona frecuencia según el bit
            start = int(i * bit_duration * self.sampling_rate)  # Índice de inicio
            end = int((i + 1) * bit_duration * self.sampling_rate)  # Índice de fin
            signal[start:end] = np.cos(2 * np.pi * freq * t[start:end])  # Genera la onda para el bit

        return t, signal


    def demodulate(self, modulated_signal, bit_duration):
        """
        Demodula una señal FSK y recupera la secuencia de bits.
        modulated_signal: señal FSK recibida
        bit_duration: duración de cada bit (segundos)
        Devuelve: lista de bits recuperados
        """
        num_bits = int(len(modulated_signal) / (bit_duration * self.sampling_rate))  # Número de bits
        bit_sequence = []  # Lista para los bits recuperados

        for i in range(num_bits):
            start = int(i * bit_duration * self.sampling_rate)  # Índice de inicio
            end = int((i + 1) * bit_duration * self.sampling_rate)  # Índice de fin
            segment = modulated_signal[start:end]  # Segmento de la señal para un bit

            # FFT para determinar la frecuencia dominante en el segmento
            freqs = np.fft.fftfreq(len(segment), 1 / self.sampling_rate)
            fft_magnitude = np.abs(np.fft.fft(segment))

            dominant_freq = freqs[np.argmax(fft_magnitude[:len(freqs)//2])]  # Frecuencia dominante
            bit = 1 if np.isclose(dominant_freq, self.freq_bit1, atol=10) else 0  # Decisión de bit
            bit_sequence.append(bit)

        return bit_sequence


if __name__ == "__main__":
    # Parámetros de la simulación
    freq_bit0 = 1000  # Frecuencia para bit 0 (Hz)
    freq_bit1 = 2000  # Frecuencia para bit 1 (Hz)
    sampling_rate = 10000  # Frecuencia de muestreo (Hz)
    bit_duration = 0.1  # Duración de cada bit (s)
    bit_sequence = [1, 0, 1, 1, 0]  # Secuencia de bits a transmitir
   
    # Crear instancia del sistema modulador/demodulador
    system = FSKModulatorDemodulator(freq_bit0, freq_bit1, sampling_rate)

    # Modulación de la secuencia de bits
    t, modulated_signal = system.modulate(bit_sequence, bit_duration)

    # Demodulación de la señal FSK
    demodulated_bits = system.demodulate(modulated_signal, bit_duration)

    # Graficar resultados
    plt.figure(figsize=(12, 6))

    plt.subplot(2, 1, 1)
    plt.plot(t, modulated_signal)
    plt.title("Señal Modulada (FSK)")
    plt.xlabel("Tiempo [s]")
    plt.ylabel("Amplitud")

    plt.subplot(2, 1, 2)
    plt.stem(range(len(demodulated_bits)), demodulated_bits)
    plt.title("Bits Demodulados")
    plt.xlabel("Índice del bit")
    plt.ylabel("Valor del bit")

    plt.tight_layout()
    plt.show()