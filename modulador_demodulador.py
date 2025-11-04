import numpy as np
import matplotlib.pyplot as plt

class FSKModulatorDemodulator:
    def __init__(self, freq_bit0, freq_bit1, sampling_rate=10000):
        self.freq_bit0 = freq_bit0
        self.freq_bit1 = freq_bit1
        self.sampling_rate = sampling_rate

    def modulate(self, bit_sequence, bit_duration):
        """Modula una secuencia de bits usando FSK."""
        t = np.arange(0, bit_duration * len(bit_sequence), 1 / self.sampling_rate)
        signal = np.zeros_like(t)

        for i, bit in enumerate(bit_sequence):
            freq = self.freq_bit1 if bit == 1 else self.freq_bit0
            start = int(i * bit_duration * self.sampling_rate)
            end = int((i + 1) * bit_duration * self.sampling_rate)
            signal[start:end] = np.cos(2 * np.pi * freq * t[start:end])

        return t, signal

    def demodulate(self, modulated_signal, bit_duration):
        """Demodula una señal FSK y recupera la secuencia de bits."""
        num_bits = int(len(modulated_signal) / (bit_duration * self.sampling_rate))
        bit_sequence = []

        for i in range(num_bits):
            start = int(i * bit_duration * self.sampling_rate)
            end = int((i + 1) * bit_duration * self.sampling_rate)
            segment = modulated_signal[start:end]

            # FFT para determinar la frecuencia dominante
            freqs = np.fft.fftfreq(len(segment), 1 / self.sampling_rate)
            fft_magnitude = np.abs(np.fft.fft(segment))

            dominant_freq = freqs[np.argmax(fft_magnitude[:len(freqs)//2])]
            bit = 1 if np.isclose(dominant_freq, self.freq_bit1, atol=10) else 0
            bit_sequence.append(bit)

        return bit_sequence

if __name__ == "__main__":
    # Parámetros
    freq_bit0 = 1000  # Frecuencia para bit 0 (Hz)
    freq_bit1 = 2000  # Frecuencia para bit 1 (Hz)
    sampling_rate = 10000  # Frecuencia de muestreo (Hz)
    bit_duration = 0.1  # Duración de cada bit (s)
    bit_sequence = [1, 0, 1, 1, 0]  # Secuencia de bits
   
    # Crear instancia del sistema
    system = FSKModulatorDemodulator(freq_bit0, freq_bit1, sampling_rate)

    # Modulación
    t, modulated_signal = system.modulate(bit_sequence, bit_duration)

    # Demodulación
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