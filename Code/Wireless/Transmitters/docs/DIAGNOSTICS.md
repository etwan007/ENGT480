# TX1 timeout diagnostics

Flash `TX1.ino` onto the existing TX1 board using its normal board selection and
pins. Open TX1's own USB serial monitor at **115200 baud**. Keep the motor off
for the first check, then save this output during the next run alongside the
Pi log. The startup line is `TX1_CONFIG`; subsequent lines are `TX1_DIAG`.
If startup fails, `TX1_SETUP_ERROR` names the operation and its error code.

The diagnostics retain channel 6, the receiver MAC, the 200 Hz target schedule,
400 kHz I²C, the existing I²C timeout, and the three-byte radio packet. Receiver
and Pi updates are not required for this TX1 change. TX2 is unchanged.
A failed encoder read still transmits `0xFFFF`.

## Finding the cause

Compare successive summaries. Counters are cumulative since TX1 boot, so use
their differences to identify new failures. Maxima reset after each queued
summary; `window_ms` gives that interval's length (normally about 1000 ms).
Times ending in `_ms` are TX1 uptime, which differs from receiver and Pi uptime.
Capture both logs together and align the observed gaps; the radio payload has
no TX1 sequence or timestamp, so these summaries do not identify an exact
receiver frame or attribute every callback to an individual angle packet.

| Observation around a receiver timeout | What it indicates |
| --- | --- |
| `read_max_us` near 50000 and a similar `gap_max_us` | TX1 spent about 50 ms reading the encoder before its next send attempt. |
| `write_max_us` near 50000, increasing `write_fail` | The register-selection transaction stalled/failed. `last_i2c_error` records its error. |
| `request_max_us` near 50000, increasing `short_read` | The two-byte read transaction stalled/returned insufficient data. |
| Normal encoder duration, increasing `send_error` | The ESP-NOW API rejected a send attempt. Inspect `last_send_error`. |
| Normal encoder duration, increasing `delivery_fail` | Queued transmissions subsequently failed at the Wi-Fi MAC layer. |
| `queued` advances while callback totals stop advancing and `delivery_ok_age_us` grows | Callback/delivery activity is lagging or stalled. Counts alone do not locate the fault. |
| Large `gap_max_us` and normal encoder duration | TX1 paused elsewhere; check `send_max_us` and scheduler lateness. |

Delivery success is a **MAC-layer result**, not proof that the receiver sketch
accepted the payload. This distinction is documented in
[Espressif's ESP-NOW guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/network/esp_now.html#send-esp-now-data).
Mixed encoder and wireless faults can occur together. A slow read can succeed,
so look at its duration even when `read_fail` stays unchanged.

## Fields

| Fields | Meaning |
| --- | --- |
| `reads`, `read_fail` | Encoder read attempts and failed attempts. |
| `write_fail`, `short_read` | Failure during register selection, or insufficient bytes from the read. |
| `slow_read` | Reads taking at least the 5000 µs send period, including successful reads. |
| `read_max_us`, `read_max_ms` | Longest complete encoder read in this reporting interval, and uptime at its completion. |
| `write_max_us`, `request_max_us` | Longest `endTransmission(false)` and `requestFrom` calls in this interval. |
| `last_read_fail_ms`, `last_i2c_error`, `last_read_bytes` | Most recent failed read's uptime, register-write error, and returned byte count. `-1` means the write failed before the request. These values persist through subsequent good reads; ignore until `read_fail` is nonzero. |
| `attempts`, `queued`, `send_error` | Send calls, calls returning `ESP_OK`, and calls returning an error. `queued` does not mean delivered. |
| `last_send_error`, `last_send_error_ms` | Most recent nonzero API error and its uptime. Both persist through good sends. Error values come from the installed ESP-IDF; consult its `esp_now.h`/`esp_err.h`. |
| `send_max_us` | Longest send API call in this interval. |
| `gap_max_us`, `gap_max_ms` | Longest gap between send-call starts in this interval, and uptime at the later attempt. |
| `gaps_50ms` | Number of gaps between send attempts of at least 50000 µs. This threshold is a diagnostic indicator, not the receiver's exact timeout test. |
| `late_max_us` | Largest lateness relative to the scheduled read/send slot. Original catch-up behavior is retained. |
| `delivery_ok`, `delivery_fail` | Cumulative asynchronous MAC delivery results. Their sum can lag `queued`. No per-packet matching is assumed. |
| `delivery_ok_age_us` | Time since last successful delivery callback; `4294967295` means none has occurred. This age uses a 32-bit microsecond clock and wraps after about 71.6 minutes without success. |
| `last_delivery_fail_ms` | Uptime of the most recent failed callback; ignore until `delivery_fail` is nonzero. |
| `log_deferred` | Reporting opportunities deferred because the previous summary has not drained, or formatting failed. |

The startup line reports the actual `Wire.getTimeOut()` value. Espressif
[documents a 50 ms default](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/i2c.html#settimeout);
this change observes the current setting rather than shortening it. Each
transaction can wait independently, so a complete read need not be bounded
by just one timeout.

## Logging and validation

The callback only updates protected counters. Summaries are formatted in the
main loop, with a fixed 1024-byte buffer, then written in chunks of at most 64
bytes and only when serial reports room. Backpressure defers summaries and
retains maxima instead of waiting for serial space. Logging still adds some
CPU work; this is a diagnostic build. Counters and uptime wrap naturally.
The send callback supports the MAC-address signature used before ESP-IDF 5.5
and the send-info signature used starting in 5.5.

From the repository root, run:

```sh
python3 -m unittest discover -s Code/Wireless/tests -p 'test_tx_diagnostics.py'
```

The host tests include the actual sketch and inject a 50 ms write timeout,
a read timeout, short responses, a slow successful read, send-call rejection,
success/failure callbacks, serial backpressure, and clock wrap. They compile
both callback signatures and check that maximum-width summaries fit.
These checks do not replace compiling and uploading for the actual TX1 board,
or collecting hardware logs. No firmware has been flashed by this change.
