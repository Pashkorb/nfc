# NFC (STM32F401 + RC522)

Проект для **STM32F401** (bare-metal, без HAL/RTOS), который работает с **MFRC522 (RC522)** по SPI и выполняет базовый сценарий работы с NFC Type 4 Tag.

## 1) Что делает прошивка

- Инициализирует тактирование GPIO/SPI и интерфейс RC522.
- Выполняет обмен с меткой (REQA → anticollision → select → RATS).
- Читает/обновляет NDEF-данные через APDU-команды.
- Поддерживает режимы аутентификации/программирования через compile-time флаги.

Основная логика находится в `src/main.c`.

---

## 2) Подключение RC522 к STM32F401

В проекте используется **SPI1** и следующие пины `GPIOA`:

| STM32F401 | Назначение | RC522 |
|---|---|---|
| PA5 | SPI1_SCK | SCK |
| PA6 | SPI1_MISO | MISO |
| PA7 | SPI1_MOSI | MOSI |
| PA4 | CS (NSS, вручную через GPIO) | SDA / SS |
| PA10 | RST RC522 | RST |
| 3.3V | Питание | 3.3V |
| GND | Земля | GND |

> Важно: RC522 должен работать от **3.3V**. Не подавайте 5V на логические линии.

### Примечания по железу

- IRQ RC522 в текущем примере не используется.
- Если у вас другая плата/распиновка, скорректируйте инициализацию GPIO и SPI в `hw_init_periph()`.

---

## 3) Зависимости

Нужны инструменты:

- `cmake` 3.20+
- `make` (или другой генератор CMake)
- ARM toolchain:
  - `arm-none-eabi-gcc`
  - `arm-none-eabi-objcopy`
  - `arm-none-eabi-size`

Проверка:

```bash
cmake --version
arm-none-eabi-gcc --version
```

---

## 4) Сборка

### Debug

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
```

### Release

```bash
cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release
cmake --build build-release
```

После сборки появляются артефакты:

- `f401_bare.elf` — ELF для отладки
- `f401_bare.hex` — HEX для программаторов
- `f401_bare.bin` — BIN для прямой прошивки

---

## 5) Прошивка

### Вариант A: ST-Link + st-flash

```bash
st-flash write build/f401_bare.bin 0x08000000
```

### Вариант B: OpenOCD

Пример (интерфейс/target при необходимости поменяйте под вашу плату):

```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg
```

В другом терминале:

```bash
arm-none-eabi-gdb build/f401_bare.elf
```

Внутри GDB:

```gdb
target extended-remote localhost:3333
monitor reset halt
load
monitor reset init
continue
```

---

## 6) Отладка (что смотреть в debugger)

В `src/main.c` есть набор глобальных переменных для диагностики состояния:

- `g_stage` — текущий этап сценария
- `g_err` — код ошибки (0 = OK)
- `g_rc522_ver` — версия RC522
- `g_last_errorreg`, `g_last_commirq`, `g_last_fifolevel` — низкоуровневые регистры RC522
- `g_last_resp`, `g_last_resp_len`, `g_last_sw1`, `g_last_sw2` — последний ответ/APDU-статус

Удобно поставить breakpoint в `main()` и наблюдать эти переменные в Watch.

### Коды ошибок (g_err)

- `0` — `ERR_OK`
- `1` — `ERR_TIMEOUT`
- `2` — `ERR_RC522`
- `3` — `ERR_PROTO`
- `4` — `ERR_SW`
- `5` — `ERR_CC_PARSE`
- `6` — `ERR_NOT_IMPLEMENTED`
- `7` — `ERR_SIZE`
- `8` — `ERR_UID`
- `9` — `ERR_CHAINING_UNSUPPORTED`

Если ошибка возникает, прошивка входит в бесконечный цикл — это нормальное поведение для текущего debug-friendly сценария.

---

## 7) Настройка режимов через compile-time флаги

В коде есть режимы:

- `AUTH_MODE_URL` / `AUTH_MODE_KEY`
- `PROGRAM_MODE_URL_ONLY` / `PROGRAM_MODE_URL_THEN_KEYS`

По умолчанию используются:

- `AUTH_MODE_URL`
- `PROGRAM_MODE_URL_ONLY`

Можно переопределить через `target_compile_definitions(...)` в `CMakeLists.txt` или через флаги компилятора.

---

## 8) Частые проблемы

1. **`arm-none-eabi-gcc: No such file or directory`**  
   Установите ARM GNU Toolchain и проверьте `PATH`.

2. **RC522 не отвечает (`g_rc522_ver` = 0x00 или 0xFF)**  
   Проверьте питание 3.3V, CS/RST, и корректность линий SPI.

3. **Таймауты обмена (`ERR_TIMEOUT`)**  
   Проверьте качество контактов, частоту SPI, антенну/метку и расстояние.

4. **Прошивка записалась, но “тишина”**  
   Проверьте, что подключили именно те пины, которые ожидает код (PA4/5/6/7/10).

---

## 9) Структура проекта (минимум)

- `src/main.c` — логика NFC/RC522
- `src/system_stm32f4xx.c` — системная инициализация STM32F4
- `mcu_core/startup/startup_stm32f401xe.s` — startup
- `mcu_core/ld/STM32F401XE.ld` — linker script
- `CMakeLists.txt` — сборка

