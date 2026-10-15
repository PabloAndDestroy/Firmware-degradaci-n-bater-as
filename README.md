# ADS1115 data acquisition GUI

The GUI in [`src/interfaz.py`](src/interfaz.py) provides:

- serial-port discovery and connection at 115200 baud;
- measurement sequencing with `Start measurement` and `Stop`;
- elapsed-time display from the start of the acquisition;
- live plots and a table for four temperature channels, voltage, and current;
- per-sensor visibility selection;
- CSV export with an `elapsed_ms` column.

## Run the GUI

From this directory:

```text
python -m pip install -r requirements.txt
python src/interfaz.py
```

The firmware should be uploaded to the STM32 before connecting. Select the
board's serial port and click **Connect**. **Start measurement** clears the
current view and sends `START`; the board then timestamps records relative to
that command and writes them to `temperature_log.csv` on the SD card. The
previous `log.csv` is left untouched so resistance and temperature records
are not mixed under the same CSV header. **Stop** sends
`STOP` and prevents further SD writes until the next start.

## Serial protocol

The firmware emits one record per second:

```text
DATA,elapsed_ms,T0,T1,T2,T3,voltage,current
```

The four thermistor channels are converted from resistance to degrees Celsius
using the resistance-temperature table in the firmware. Temperatures between
the table's 1 °C points are calculated by linear interpolation; values outside
the calibrated range (-40 °C to 200 °C) are recorded as `nan`. The
`temperature_log.csv` file on the SD card labels these channels `T1_C` through
`T4_C`.

It also accepts newline-terminated `START`, `STOP`, and `STATUS` commands.
The GUI retains a parser for the previous human-readable `T:... | t:...`
format, so it can still display data from older firmware builds.

## Windows application

A standalone executable is available at:

```text
dist\ADS1115_Acquisition.exe
```

Double-click it to run the GUI; Python is not required on the target computer.
To rebuild it after changing the source, double-click `build_windows.bat` or run
it from this directory. The build uses PyInstaller's one-file, windowed mode.
