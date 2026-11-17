# Universal Log

Universal Log is a lightweight Python utility for recording structured state updates from a drone or similar system. It validates each log entry as a JSON object, writes a timestamped message to a log file, and can optionally echo the same output to the terminal.

## What it does

Each log entry is expected to be a dictionary with the following shape:

```python
{
    "State": "TRACKING",
    "Timestamp": "2026-10-09T16:00:00Z",
    "Details": {
        "altitude": 12.4,
        "battery": 88,
        "mode": "autonomous"
    }
}
```

The logger then creates a message in this format:

```text
[timestamp] STATE: {"altitude":12.4,"battery":88,"mode":"autonomous"}
```

It stores the entry in a per-run log file and optionally prints it to the console. If the `State` is one of the known values, it logs at `INFO`; otherwise it logs at `ERROR`.

## Supported states

```text
IDLE
TRACKING
HOVERING
DOCKED
DOCKING_INIT
DOCKING_APPROACH
ABORT
EMERGENCY_LAND
```

Any other state is treated as an unexpected/error condition.

## Requirements

- Python 3.8+
- Standard library only (`json`, `logging`, `datetime`, `pathlib`)

## Usage

Import the class and create log entries from your application:

```python
from universal_log.Universal_log import UniversalLog

UniversalLog(
    {
        "State": "TRACKING",
        "Timestamp": "2026-10-09T16:00:00Z",
        "Details": {
            "altitude": 12.4,
            "battery": 88,
            "speed": 3.6,
        },
    },
    print_to_terminal=True,
)
```

The bundled sample entries are in `test_states.json`. The `main()` sample
runner at the bottom of `Universal_log.py` is commented out, so running that
file does not execute the sample.

When an application creates a `UniversalLog` entry, the logger will:

- validate the input as a JSON object
- ensure `Details` is a dictionary
- write the log entry to `universal_log/logs/logged_states_<date>_<time>.log`
- print the message to the terminal if `print_to_terminal=True`

The `logs` directory is created automatically when the first log entry is
written. Its generated log files are ignored by Git.

## Error handling

The class raises `TypeError` if:

- the incoming log entry is not a dictionary
- the `Details` field is not a dictionary

This makes it easy to catch malformed log input early.

## Notes

This project is designed as a reusable logging helper. Applications can import
`UniversalLog` and write entries through the same interface.