Run `./test/host/run.sh` with a C++17 compiler (Clang on macOS). The suite
compiles the real library with AddressSanitizer and UndefinedBehaviorSanitizer.
Only hardware/clock/logging are stubbed; CRC-framed original-controller register
responses enter the production serial parser. No LAN or physical HVAC needed.

Fixtures reproduce September 2026 fan-only Low/Medium/High/Auto and thermal
standby/activity captures. The 0x0223 fixture is a regression against treating
an arbitrary cooling demand value as the user's Auto preference; it is not a
newly observed original-controller frame. Tests verify passive operation emits
no bytes. These tests establish encoding/decoding, not physical operation or a
complete thermal/Auto control algorithm.
