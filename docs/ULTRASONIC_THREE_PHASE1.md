# Three ultrasonic sensors: observation-only upgrade

2026-09-20. Front PC0/PB6(TIM4_CH1), left PC1/PD13(CH2), right PC2/PD14(CH3).
Use common ground, regulated module supply and 3.3V-compatible ECHO signals.
LCD/ETH/FSMC must remain disabled. PG12 (active-low SRAM CE) is held high;
an external 10k pull-up to 3.3V is recommended during reset/programming.

80ms slots: front,left,front,right. Front period160ms; side period320ms.
Timeout70ms. CS100A ~66ms no-target pulses use an elapsed-ms guard against
the 16-bit timer wrap. ISR only captures timestamps and changes edge polarity.

Old 0x05 remains16bytes. New 0x08 has36bytes: three12-byte records in front,
left,right order. Each is float32 distance_m, uint32 age_ms, uint16 sequence,
uint8 status, uint8 reserved0, little endian. Status0=uninitialized,
1=echo,2=no echo/out of range,3=fault. No echo does NOT prove free space.
UART4 adds US3 lines without changing legacy TEL fields.

New side readings do not affect existing safety flags or motion decisions.
Existing front protection remains active. Updated CubeMX .ioc accompanies
the manually updated HAL files; CubeMX regeneration itself was not run.

Keil build_us3.log: zero errors/warnings. Portable pulse test passed on RK.
Programmed HEX SHA256: ff1d802de7d11acf29ded9e5befd23604c40e5542af7d1d42c3d149b93495d0c
OpenOCD verified flash. Passive6s capture:121 new frames,zero CRC errors,
zero wheel ticks. Side modules are not installed/validated yet.

Full wiring and joint acceptance procedure:
G:/work/ROS2/Docs2/三超声波接线与静态验收_20260920.md
