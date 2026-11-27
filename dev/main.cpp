#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>

#include <algorithm>
#include <complex>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

using cfloat = std::complex<float>;

constexpr double SAMPLE_RATE = 1e6;
constexpr double FREQUENCIES_HZ[] = {800e6, 850e6};
constexpr size_t FREQUENCY_COUNT =
    sizeof(FREQUENCIES_HZ) / sizeof(FREQUENCIES_HZ[0]);

constexpr size_t SPS = 10;               
constexpr float TX_AMPLITUDE = 2000.0f;
constexpr double TX_GAIN_DB = -10.0;
constexpr double RX_GAIN_DB = 40.0;
constexpr size_t HEADER_SAMPLES = 6;     
constexpr size_t MAX_BLOCKS_PER_BUFFER = 3;
constexpr size_t RX_ITERATIONS = 2000;
constexpr long STREAM_TIMEOUT_US = 1000000;

const std::string MESSAGE = "PRIVET_SDR_2026";   

enum class Mode { Tx, Rx };
enum class Modulation { Bpsk, Qpsk };

volatile std::sig_atomic_t g_stop = 0;
void on_stop(int) { g_stop = 1; }

void print_usage(const char *program) {
  std::printf("Использование: %s <Tx|Rx> --freq <1|2> [bpsk|qpsk]\n"
              "  Tx / --tx     передача сообщения \"%s\" пакетами\n"
              "  Rx / --rx     приём I/Q-сигнала в rxdata.pcm\n"
              "  --freq 1      %.0f МГц\n"
              "  --freq 2      %.0f МГц\n"
              "  bpsk|qpsk     модуляция (только для Tx, по умолчанию bpsk)\n",
              program, MESSAGE.c_str(), FREQUENCIES_HZ[0] / 1e6,
              FREQUENCIES_HZ[1] / 1e6);
}

bool parse_arguments(int argc, char *argv[], Mode *mode, double *frequency,
                     Modulation *modulation) {
  if ((argc != 4 && argc != 5) || std::strcmp(argv[2], "--freq") != 0)
    return false;

  if (std::strcmp(argv[1], "Tx") == 0 || std::strcmp(argv[1], "--tx") == 0)
    *mode = Mode::Tx;
  else if (std::strcmp(argv[1], "Rx") == 0 || std::strcmp(argv[1], "--rx") == 0)
    *mode = Mode::Rx;
  else
    return false;

  char *end = nullptr;
  const long selected = std::strtol(argv[3], &end, 10);
  if (*argv[3] == '\0' || *end != '\0' || selected < 1 ||
      selected > static_cast<long>(FREQUENCY_COUNT))
    return false;
  *frequency = FREQUENCIES_HZ[selected - 1];

  *modulation = Modulation::Bpsk;
  if (argc == 5) {
    if (std::strcmp(argv[4], "bpsk") == 0)
      *modulation = Modulation::Bpsk;
    else if (std::strcmp(argv[4], "qpsk") == 0)
      *modulation = Modulation::Qpsk;
    else
      return false;
  }
  return true;
}

SoapySDRDevice *make_device() {
  SoapySDRKwargs args = {};
  SoapySDRKwargs_set(&args, "driver", "plutosdr");
  SoapySDRKwargs_set(&args, "uri", "usb:");
  SoapySDRKwargs_set(&args, "loopback", "0");
  SoapySDRDevice *sdr = SoapySDRDevice_make(&args);
  SoapySDRKwargs_clear(&args);
  return sdr;
}

std::vector<bool> string_to_bits(const std::string &str) {
  std::vector<bool> bits;
  bits.reserve(str.size() * 8);
  for (unsigned char c : str)
    for (int i = 7; i >= 0; --i)
      bits.push_back((c >> i) & 1);
  return bits;
}


std::vector<cfloat> modulate(const std::vector<bool> &bits, Modulation mod) {
  std::vector<cfloat> sym;
  if (mod == Modulation::Bpsk) {
    sym.reserve(bits.size());
    for (bool bit : bits)
      sym.emplace_back(bit ? -1.0f : 1.0f, 0.0f);
  } else {
    constexpr float scale = 0.70710678f;
    sym.reserve(bits.size() / 2);
    for (size_t i = 0; i + 1 < bits.size(); i += 2)
      sym.emplace_back(bits[i] ? -scale : scale, bits[i + 1] ? -scale : scale);
  }
  return sym;
}

std::vector<cfloat> upsample(const std::vector<cfloat> &symbols, size_t factor) {
  std::vector<cfloat> samples(symbols.size() * factor, cfloat{});
  for (size_t i = 0; i < symbols.size(); ++i)
    samples[i * factor] = symbols[i];
  return samples;
}


std::vector<cfloat> convolve_rect(const std::vector<cfloat> &x, size_t len) {
  std::vector<cfloat> y(x.size());
  for (size_t n = 0; n < x.size(); ++n) {
    cfloat sum{};
    for (size_t k = 0; k < len && k <= n; ++k)
      sum += x[n - k];
    y[n] = sum;
  }
  return y;
}

void save_bits(const std::vector<bool> &bits, const char *filename) {
  FILE *f = std::fopen(filename, "w");
  if (!f)
    return;
  for (bool b : bits)
    std::fprintf(f, "%d\n", b ? 1 : 0);
  std::fclose(f);
}


void write_header(int16_t *p, long long tx_time) {
  p[0] = static_cast<int16_t>(0xffff);
  p[1] = static_cast<int16_t>(0xffff);
  for (int i = 0; i < 8; ++i) {
    const uint8_t byte = static_cast<uint8_t>((tx_time >> (8 * i)) & 0xff);
    p[2 + i] = static_cast<int16_t>(byte << 4);
  }
  p[10] = static_cast<int16_t>(0xffff);
  p[11] = static_cast<int16_t>(0xffff);
}

int run_tx(SoapySDRDevice *sdr, double frequency, Modulation modulation) {
  const size_t channel = 0;
  if (SoapySDRDevice_setSampleRate(sdr, SOAPY_SDR_TX, channel, SAMPLE_RATE) !=
          0 ||
      SoapySDRDevice_setFrequency(sdr, SOAPY_SDR_TX, channel, frequency,
                                  nullptr) != 0 ||
      SoapySDRDevice_setGain(sdr, SOAPY_SDR_TX, channel, TX_GAIN_DB) != 0) {
    std::printf("Не удалось настроить TX: %s\n", SoapySDRDevice_lastError());
    return EXIT_FAILURE;
  }

  SoapySDRStream *stream = SoapySDRDevice_setupStream(
      sdr, SOAPY_SDR_TX, SOAPY_SDR_CS16, &channel, 1, nullptr);
  if (!stream) {
    std::printf("setupStream TX: %s\n", SoapySDRDevice_lastError());
    return EXIT_FAILURE;
  }

  const size_t mtu = SoapySDRDevice_getStreamMTU(sdr, stream);

  const auto bits = string_to_bits(MESSAGE);
  save_bits(bits, "tx_bits.txt");
  const auto sym = modulate(bits, modulation);
  const auto signal = convolve_rect(upsample(sym, SPS), SPS);

  const size_t sig_samples = signal.size();
  const size_t block_samples = HEADER_SAMPLES + sig_samples;
  if (block_samples > mtu) {
    std::printf("Сообщение слишком длинное: блок %zu отсчётов, MTU %zu\n",
                block_samples, mtu);
    SoapySDRDevice_closeStream(sdr, stream);
    return EXIT_FAILURE;
  }
  const size_t blocks = std::min(MAX_BLOCKS_PER_BUFFER, mtu / block_samples);

  std::vector<int16_t> active_buffer(2 * mtu, 0);
  std::vector<int16_t> zero_buffer(2 * mtu, 0);
  for (size_t b = 0; b < blocks; ++b) {
    const size_t sig_start = b * block_samples + HEADER_SAMPLES;
    for (size_t s = 0; s < sig_samples; ++s) {
      active_buffer[2 * (sig_start + s)] =
          static_cast<int16_t>(signal[s].real() * TX_AMPLITUDE);
      active_buffer[2 * (sig_start + s) + 1] =
          static_cast<int16_t>(signal[s].imag() * TX_AMPLITUDE);
    }
  }

  const long long buf_ns =
      static_cast<long long>(static_cast<double>(mtu) / SAMPLE_RATE * 1e9);
  const long long step_ns = 2 * buf_ns + 200000;   

  std::printf("TX: %.0f МГц, %s, MTU: %zu, сообщение \"%s\" (%zu бит, %zu симв.), "
              "блоков в пакете: %zu\n",
              frequency / 1e6, modulation == Modulation::Bpsk ? "BPSK" : "QPSK",
              mtu, MESSAGE.c_str(), bits.size(), sym.size(), blocks);

  int result = EXIT_SUCCESS;
  if (SoapySDRDevice_activateStream(sdr, stream, 0, 0, 0) != 0) {
    std::printf("activateStream TX: %s\n", SoapySDRDevice_lastError());
    result = EXIT_FAILURE;
  } else {
    const long long base_time =
        SoapySDRDevice_getHardwareTime(sdr, nullptr) + 10LL * 1000 * 1000;

    for (size_t iteration = 0; !g_stop; ++iteration) {
      const long long tx_time =
          base_time + static_cast<long long>(iteration) * step_ns;

      for (size_t b = 0; b < blocks; ++b)
        write_header(&active_buffer[2 * b * block_samples], tx_time);

      if (iteration == 0) {                         
        FILE *f = std::fopen("txdata.pcm", "wb");
        if (f) {
          std::fwrite(active_buffer.data(), sizeof(int16_t),
                      active_buffer.size(), f);
          std::fclose(f);
        }
      }

      const void *active_buffers[] = {active_buffer.data()};
      int flags = SOAPY_SDR_HAS_TIME;
      int written = SoapySDRDevice_writeStream(
          sdr, stream, active_buffers, mtu, &flags, tx_time, STREAM_TIMEOUT_US);
      if (written < 0) {
        std::printf("TX ошибка (пакет): %d (%s)\n", written,
                    SoapySDRDevice_lastError());
        result = EXIT_FAILURE;
        break;
      }

      const long long zero_time = tx_time + buf_ns;   
      const void *zero_buffers[] = {zero_buffer.data()};
      flags = SOAPY_SDR_HAS_TIME;
      written = SoapySDRDevice_writeStream(
          sdr, stream, zero_buffers, mtu, &flags, zero_time, STREAM_TIMEOUT_US);
      if (written < 0) {
        std::printf("TX ошибка (тишина): %d (%s)\n", written,
                    SoapySDRDevice_lastError());
        result = EXIT_FAILURE;
        break;
      }

      if (iteration % 250 == 0)
        std::printf("Отправлено пакетов: %zu\n", iteration);
    }
    std::printf("Остановка передачи...\n");
    SoapySDRDevice_deactivateStream(sdr, stream, 0, 0);
  }
  SoapySDRDevice_closeStream(sdr, stream);
  return result;
}

int run_rx(SoapySDRDevice *sdr, double frequency) {
  const size_t channel = 0;
  if (SoapySDRDevice_setSampleRate(sdr, SOAPY_SDR_RX, channel, SAMPLE_RATE) !=
          0 ||
      SoapySDRDevice_setFrequency(sdr, SOAPY_SDR_RX, channel, frequency,
                                  nullptr) != 0 ||
      SoapySDRDevice_setGain(sdr, SOAPY_SDR_RX, channel, RX_GAIN_DB) != 0) {
    std::printf("Не удалось настроить RX: %s\n", SoapySDRDevice_lastError());
    return EXIT_FAILURE;
  }

  SoapySDRStream *stream = SoapySDRDevice_setupStream(
      sdr, SOAPY_SDR_RX, SOAPY_SDR_CS16, &channel, 1, nullptr);
  if (!stream) {
    std::printf("setupStream RX: %s\n", SoapySDRDevice_lastError());
    return EXIT_FAILURE;
  }

  const size_t mtu = SoapySDRDevice_getStreamMTU(sdr, stream);
  std::vector<int16_t> buffer(2 * mtu);
  FILE *file = std::fopen("rxdata.pcm", "wb");
  if (!file) {
    std::printf("Не удалось открыть rxdata.pcm\n");
    SoapySDRDevice_closeStream(sdr, stream);
    return EXIT_FAILURE;
  }

  int result = EXIT_SUCCESS;
  size_t total_samples = 0;
  if (SoapySDRDevice_activateStream(sdr, stream, 0, 0, 0) != 0) {
    std::printf("activateStream RX: %s\n", SoapySDRDevice_lastError());
    result = EXIT_FAILURE;
  } else {
    std::printf("RX: %.0f МГц, MTU: %zu\n", frequency / 1e6, mtu);
    for (size_t iteration = 0; iteration < RX_ITERATIONS && !g_stop;
         ++iteration) {
      void *buffers[] = {buffer.data()};
      int flags = 0;
      long long time_ns = 0;
      const int received = SoapySDRDevice_readStream(
          sdr, stream, buffers, mtu, &flags, &time_ns, STREAM_TIMEOUT_US);
      if (received > 0) {
        std::fwrite(buffer.data(), sizeof(int16_t),
                    static_cast<size_t>(received) * 2, file);
        total_samples += static_cast<size_t>(received);
        if (iteration % 250 == 0)
          std::printf("Буфер %zu/%zu, всего отсчётов: %zu\n", iteration + 1,
                      RX_ITERATIONS, total_samples);
      } else if (received < 0 && received != SOAPY_SDR_TIMEOUT) {
        std::printf("RX ошибка: %d (%s)\n", received,
                    SoapySDR_errToStr(received));
      }
    }
    SoapySDRDevice_deactivateStream(sdr, stream, 0, 0);
  }
  std::fclose(file);
  std::printf("Приём завершён, записано %zu комплексных отсчётов в rxdata.pcm\n",
              total_samples);
  SoapySDRDevice_closeStream(sdr, stream);
  return result;
}

} 

int main(int argc, char *argv[]) {
  Mode mode;
  double frequency = 0;
  Modulation modulation;
  if (!parse_arguments(argc, argv, &mode, &frequency, &modulation)) {
    print_usage(argv[0]);
    return EXIT_FAILURE;
  }

  std::signal(SIGINT, on_stop);
  std::signal(SIGTERM, on_stop);

  SoapySDRDevice *sdr = make_device();
  if (!sdr) {
    std::printf("SoapySDRDevice_make: %s\n", SoapySDRDevice_lastError());
    return EXIT_FAILURE;
  }

  const int result = mode == Mode::Tx ? run_tx(sdr, frequency, modulation)
                                      : run_rx(sdr, frequency);
  SoapySDRDevice_unmake(sdr);
  return result;
}