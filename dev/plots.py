import os
import sys
import numpy as np
import matplotlib.pyplot as plt

RECV_FILE = sys.argv[1] if len(sys.argv) > 1 else "rxdata.pcm"
SAMPLE_RATE = 1e6

MODULATION = "qpsk"              
MESSAGE = "PRIVET_SDR_2026"      
SPS = 10                         
HEADER_SAMPLES = 6               
BLOCKS_PER_BURST = 3             

SKIP_START = 200_000             
MAX_SAMPLES = 5_000_000          
REMOVE_CFO = True                
CFO_MAX_HZ = 10_000


def read_iq(name):
    if not os.path.exists(name):
        print(f"Ошибка: файл '{name}' не найден")
        sys.exit(1)
    raw = np.fromfile(name, dtype=np.int16)
    raw = raw[: len(raw) // 2 * 2].astype(np.float32)
    return raw[0::2], raw[1::2]


def estimate_cfo(x, fs, m, max_hz):
    n = len(x)
    nfft = 1 << int(np.ceil(np.log2(n)) + 2)
    spec = np.abs(np.fft.fftshift(np.fft.fft(x ** m * np.hamming(n), nfft)))
    freqs = np.fft.fftshift(np.fft.fftfreq(nfft, d=1 / fs))
    band = np.abs(freqs) < m * max_hz
    return freqs[band][np.argmax(spec[band])] / m


def gardner_ted(x, y, k, Ts, tau_hat):
    prev = int((k - 1) * Ts + tau_hat)
    mid = int((k - 0.5) * Ts + tau_hat)
    curr = int(k * Ts + tau_hat)
    return x[mid] * (x[prev] - x[curr]) + y[mid] * (y[prev] - y[curr])


qpsk = MODULATION.lower() == "qpsk"
n_bits = 8 * len(MESSAGE)
n_sym = n_bits // 2 if qpsk else n_bits
BLOCK = HEADER_SAMPLES + n_sym * SPS          
SHOW = BLOCKS_PER_BURST * BLOCK               

rx_i, rx_q = read_iq(RECV_FILE)
rx = (rx_i + 1j * rx_q)
rx = rx[SKIP_START: SKIP_START + MAX_SAMPLES]
rx = rx - np.mean(rx)
print(f"Файл {RECV_FILE}: {len(rx)} отсчётов в просмотре, блок {BLOCK}, пакет {SHOW}")

p = np.abs(rx) ** 2
cs = np.concatenate(([0.0], np.cumsum(p)))
win = cs[SHOW:] - cs[:-SHOW]
start = int(np.argmax(win))
start = max(start, SPS - 1)
seg = slice(start, start + SHOW)
print(f"Пакет найден: отсчёт {start} ({start / SAMPLE_RATE * 1e3:.2f} мс от начала просмотра)")

ext = rx[start - (SPS - 1): start + SHOW].copy()   
if REMOVE_CFO:
    m = 4 if qpsk else 2
    df = estimate_cfo(ext, SAMPLE_RATE, m, CFO_MAX_HZ)
    ext *= np.exp(-2j * np.pi * df * np.arange(len(ext)) / SAMPLE_RATE)
    print(f"Сдвиг несущей: {df:.1f} Гц")

h = np.ones(SPS)
y_eval = np.convolve(ext, h, 'valid')                

plt.figure(figsize=(6, 6))
plt.scatter(rx.real[seg], rx.imag[seg], color="blue", alpha=0.6, s=6, label="I/Q")
plt.title("I/Q Созвездие (до фильтра)")
plt.xlabel("I")
plt.ylabel("Q")
plt.axhline(0, color="black", linewidth=0.8, linestyle="--")
plt.axvline(0, color="black", linewidth=0.8, linestyle="--")
plt.grid(True)
plt.legend(loc="upper right")
plt.tight_layout()

plt.figure(figsize=(6, 6))
plt.scatter(y_eval.real, y_eval.imag, color="blue", alpha=0.6, s=6, label="Filtered I/Q")
plt.title("I/Q Созвездие (после свёртки)")
plt.xlabel("I")
plt.ylabel("Q")
plt.axhline(0, color="black", linewidth=0.8, linestyle="--")
plt.axvline(0, color="black", linewidth=0.8, linestyle="--")
plt.grid(True)
plt.legend(loc="upper right")
plt.tight_layout()

fig, axes = plt.subplots(2, 5, figsize=(18, 7), sharex=True, sharey=True)
axes = axes.flatten()
for phase in range(SPS):
    z_phase = y_eval[phase::SPS]
    ax = axes[phase]
    ax.scatter(z_phase.real, z_phase.imag, color="crimson", alpha=0.6, s=15)
    ax.set_title(f"Фаза: {phase}")
    ax.set_xlabel("I")
    ax.set_ylabel("Q")
    ax.axhline(0, color="black", linewidth=0.8, linestyle="--")
    ax.axvline(0, color="black", linewidth=0.8, linestyle="--")
    ax.grid(True)
plt.suptitle("Созвездия по фазам стробирования", fontsize=14)
plt.tight_layout()
x = y_eval.real
y_iq = y_eval.imag
num_symbols = len(y_eval) // SPS - 2

e = []
for phase in range(SPS):
    errors = []
    for k in range(1, num_symbols):
        errors.append(gardner_ted(x, y_iq, k, SPS, phase))
    e.append(np.mean(errors))

plt.figure(figsize=(7, 5))
plt.plot(range(SPS), e, marker="o", color="blue")
plt.title("Ошибка Гарднера от фазы стробирования")
plt.xlabel("Фаза (сдвиг отсчета)")
plt.ylabel("Средняя ошибка e")
plt.xticks(range(SPS))
plt.axhline(0, color="black", linestyle="--")
plt.grid(True)
plt.tight_layout()

mags = [np.mean(np.abs(y_eval[ph::SPS])) for ph in range(SPS)]
print(f"Фаза с максимальным средним модулем символов: {int(np.argmax(mags))}")
print(f"Фаза с минимальной |ошибкой Гарднера|: {int(np.argmin(np.abs(e)))}")

plt.show()