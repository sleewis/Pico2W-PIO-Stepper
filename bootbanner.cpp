#include "bootBanner.h"
#include "SerialDevice.h"

namespace bootBanner {

// ANSI sequences
#define ANSI_ESC        "\x1b["
#define ANSI_RESET      "\x1b[0m"
#define ANSI_BOLD       "\x1b[1m"
#define ANSI_DIM        "\x1b[2m"

#define FG_BRIGHT_RED     "\x1b[91m"
#define FG_BRIGHT_GREEN   "\x1b[92m"
#define FG_BRIGHT_YELLOW  "\x1b[93m"
#define FG_BRIGHT_BLUE    "\x1b[94m"
#define FG_BRIGHT_MAGENTA "\x1b[95m"
#define FG_BRIGHT_CYAN    "\x1b[96m"
#define FG_BRIGHT_WHITE   "\x1b[97m"

static inline void println_P(const __FlashStringHelper* s) { SD.println(s); }

void print_boot_banner(bool useAnsi, const char* fwVersion) {
  const char* R  = useAnsi ? ANSI_RESET          : "";
  const char* B  = useAnsi ? ANSI_BOLD           : "";
  const char* D  = useAnsi ? ANSI_DIM            : "";
  const char* CY = useAnsi ? FG_BRIGHT_CYAN      : "";
  const char* YL = useAnsi ? FG_BRIGHT_YELLOW    : "";
  const char* GR = useAnsi ? FG_BRIGHT_GREEN     : "";
  const char* RD = useAnsi ? FG_BRIGHT_RED       : "";
  const char* BL = useAnsi ? FG_BRIGHT_BLUE      : "";
  const char* WH = useAnsi ? FG_BRIGHT_WHITE     : "";

  if (useAnsi) SD.print("\x1b[2J\x1b[H"); // clear + home

  // Top border
  SD.print(D); println_P(F("+------------------------------------------------------------------------------+")); SD.print(R);

  // Big “HELP” (H=CYAN, E=YELLOW, L=GREEN, P=RED)
  SD.print("|  "); SD.print(CY); SD.print(B);  SD.print(" _    _ "); SD.print(R);
  SD.print("  "); SD.print(YL); SD.print(B);  SD.print(" ______ "); SD.print(R);
  SD.print("  "); SD.print(GR); SD.print(B);  SD.print(" _      "); SD.print(R);
  SD.print("  "); SD.print(RD); SD.print(B);  SD.print(" _____  "); SD.print(R);
  SD.println("                                      |");

  SD.print("|  "); SD.print(CY); SD.print(B);  SD.print("| |  | |"); SD.print(R);
  SD.print("  "); SD.print(YL); SD.print(B);  SD.print("|  ____|"); SD.print(R);
  SD.print("  "); SD.print(GR); SD.print(B);  SD.print("| |     "); SD.print(R);
  SD.print("  "); SD.print(RD); SD.print(B);  SD.print("|  __ \\ "); SD.print(R);
  SD.println("                                      |");

  SD.print("|  "); SD.print(CY); SD.print(B);  SD.print("| |__| |"); SD.print(R);
  SD.print("  "); SD.print(YL); SD.print(B);  SD.print("| |__   "); SD.print(R);
  SD.print("  "); SD.print(GR); SD.print(B);  SD.print("| |     "); SD.print(R);
  SD.print("  "); SD.print(RD); SD.print(B);  SD.print("| |__) |"); SD.print(R);
  SD.println("                                      |");

  SD.print("|  "); SD.print(CY); SD.print(B);  SD.print("|  __  |"); SD.print(R);
  SD.print("  "); SD.print(YL); SD.print(B);  SD.print("|  __|  "); SD.print(R);
  SD.print("  "); SD.print(GR); SD.print(B);  SD.print("| |     "); SD.print(R);
  SD.print("  "); SD.print(RD); SD.print(B);  SD.print("|  ___/ "); SD.print(R);
  SD.println("                                      |");

  SD.print("|  "); SD.print(CY); SD.print(B);  SD.print("| |  | |"); SD.print(R);
  SD.print("  "); SD.print(YL); SD.print(B);  SD.print("| |____ "); SD.print(R);
  SD.print("  "); SD.print(GR); SD.print(B);  SD.print("| |____ "); SD.print(R);
  SD.print("  "); SD.print(RD); SD.print(B);  SD.print("| |     "); SD.print(R);
  SD.println("                                      |");

  SD.print("|  "); SD.print(CY); SD.print(B);  SD.print("|_|  |_|"); SD.print(R);
  SD.print("  "); SD.print(YL); SD.print(B);  SD.print("|______|"); SD.print(R);
  SD.print("  "); SD.print(GR); SD.print(B);  SD.print("|______|"); SD.print(R);
  SD.print("  "); SD.print(RD); SD.print(B);  SD.print("|_|     "); SD.print(R);
  SD.println("                                      |");

  // Subtitle
  SD.print(D); println_P(F("+------------------------------------------------------------------------------+")); SD.print(R);
  SD.print("|  "); SD.print(BL); SD.print("Pico PIO/DMA Stepper Controller"); SD.print(R);
  SD.print("  \xE2\x80\xA2  "); SD.print(WH); SD.print("Real-time position"); SD.print(R);
  SD.print("  \xE2\x80\xA2  "); SD.print(WH); SD.print("DMA"); SD.print(R);
  SD.print("  \xE2\x80\xA2  "); SD.print(WH); SD.println("PIO SM   |");

  // Firmware line
  SD.print("|  Firmware: "); SD.print(YL); SD.print(fwVersion); SD.print(R);
  SD.println("                                                            |");

  // Pins / PIO timing
  SD.print(D); println_P(F("+------------------------------------------------------------------------------+")); SD.print(R);
  SD.println("| Pins: DIR X/Y/Z = GPIO 0/1/2  \xE2\x80\xA2  STEP X/Y/Z = GPIO 3/4/5  \xE2\x80\xA2  3.3V logic      |");
  SD.println("| PIO: STEP high = 20us  \xE2\x80\xA2  DIR setup = 5us  \xE2\x80\xA2  Extra delay encoded per word   |");

  // Commands
  SD.print(D); println_P(F("+------------------------------------------------------------------------------+")); SD.print(R);
  SD.println("| Commands:                                                                    |");
  SD.println("|   G0 X.. Y.. Z..            ; rapid                                          |");
  SD.println("|   G1 X.. Y.. Z.. F..        ; linear, F in current units/s                   |");
  SD.println("|   W... ... / C2             ; GS232 commands                                 |");
  SD.println("|   G28                       ; home to origin (0,0,0 steps)                   |");
  SD.println("|   G92 X.. Y.. Z..           ; set current position (in current units)        |");
  SD.println("|   M114 / M0 / M2 / M110     ; report / stop / stop(keep) / clear queue       |");
  SD.println("|   M111 / M120 / M121        ; queue status / homing on / homing off          |");
  SD.println("|   G101 / G102 / G103        ; units: steps / mm / deg                        |");
  SD.println("|   DEBUG / HELP              ; Show debug information / Show this information |");

  // Footer
  SD.print(D); println_P(F("+------------------------------------------------------------------------------+")); SD.print(R);
  SD.println();
}

} // namespace bootBanner
