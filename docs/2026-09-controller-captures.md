# Original Que controller observations, September 2026

These observations supersede earlier assumptions that reg 2 low byte 0x23
means Auto, reg 3 bit 1 alone means running, and reg 4 low byte is always 0x23.

| Original setting / observation | Slave 3 reg 2 | reg 3 | reg 4 (zone 7 only) |
|---|---|---|---|
| Fan-only Low | 0800 | 0928 | 4000 |
| Fan-only Medium | 0800 | 113D | 4000 |
| Fan-only High | 0800 | 2159 | 4000 |
| Fan-only Auto, airflow confirmed | 0800 | 0300 | 4000 |
| Heat High 26, airflow confirmed | 0164 | 2159 | 4023 |
| Heat Auto 26, standby | 0100 | 0100 | 4023 |
| Heat Auto 26, LCD fast heating (fan still Auto) | 015E then 0164 | 2159 | 4023 |
| Cool Auto 20, physical cooling confirmed | 0243 then 0248 | varies | 4023 |

LCD Auto operating mode with comfort range 26–28 reported Heat/26 on the
bus. Changing the range to 20–22 produced Heat/20 then Cool/22 after a delay;
the LCD was in standby with room temperature about 22. This supports Auto
being a wall-controller algorithm, not a distinct on-wire mode. The fan
register likewise describes the controller's output, not necessarily the
user's selected fan preference. These fields do not independently measure
compressor operation, RPM, or airflow.

## Implemented and remaining work

Fan-only responder encoding now matches the four captured speeds, and passive
thermal decoding uses the reg 2 low byte as activity/demand rather than an
Auto selector. Physical fan-only takeover needs a follow-up test after the
original controller is disconnected.

Heat/Cool demand generation, Auto comfort-band logic, hysteresis, timing,
and automatic thermal fan selection remain unimplemented. Thermal responder
commands still emit standby demand (zero). The previous Auto 0223 placeholder
is removed; it would otherwise be interpreted as a fixed cooling demand.
Do not treat this patch as working replacement thermal control. Do not infer
an exact demand control law from these few snapshots or replay fixed maximum
demand as a thermostat implementation.
