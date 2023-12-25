# Estimación Carga Baterías

*Puerto Montt, 14 de julio de 2020, Nicolás Hasbún A.*

## Resumen Posibilidades

- Usando sistema actual:
  - Medición Voltaje/Potencia y Análisis de Datos
  - Medición Energía utilizada usando ArduSub
- Uso de Smart Batteries:
  - Necesario uso de bus CAN

## Esquema Actual

- Baterías Li-Ion de 14.8V ~21Ah proveedor Herculesi (4 serie 7 parallel)
  - [Herculesi Website][herculesi_website]
- Medición Voltaje/Corriente usando [Power Sense Module][power_sense_module_bluerobotics] de BlueRobotics
- Estimación es posible usando Voltaje/Corriente + Filtros de Kalman (u otros)
  - Ejemplo [larchuto](https://github.com/larchuto)/**[Battery-Kalman](https://github.com/larchuto/Battery-Kalman)**:
    <img src=".\img\battery_kalman_estimation.png" alt="battery_kalman_estimation" style="zoom:60%;" />
  - Ejemplo completo y didáctico para la estimación de baterías (pega lista)
    - [State of Charge Estimation with a Kalman Filter][soc_kalman_estimation_jackogrady]
    - Repositorio [jogrady23](https://github.com/jogrady23)/**[kalman-filter-battery-soc](https://github.com/jogrady23/kalman-filter-battery-soc)**

## Usando ArduSub

- [Voltage and Current Measurement Setup - BlueROV2 Software Setup][bluerobotic_current_measurement_setup]
- Sistema ArduSub lleva registro interno de uso de energía
  - Requiere lógica/intervención para detectar recargas de baterías o estados intermedios (esto puede resultar complejo o dejar espacio al error humano del piloto)
- Otros sistemas de power modules al parecer funcionan similar
  - [Interfacing smart battery over UAVCAN - Ardupilot forum](https://discuss.ardupilot.org/t/interfacing-smart-battery-over-uavcan/44017/1)

## Cambio al Sistema de Baterías

- Smart Batteries
  - [Spektrum Smart LiPO 5000mAh 4S 14.8V](https://www.spektrumrc.com/Products/Default.aspx?ProdID=SPMX50004S100H5)
- Battery Management System
  - [Lion Battery Management System](https://www.lionsmart.com/en/batterymanagementsystem/)

## Relevante - Uso de Bus Can

- [Can Transceiver DIY][diy_can_transceiver]
  - IC [Microchip MCP2551][mcp2551_site] High-speed CAN transceiver, costo <1 USD
- [CAN Bus Electrical Specification and Architecture][can_bus_wikipedia]
- [CAN FD data communication protocol][can_fd_wikipedia]

---

## Referencias

- Artículos sobre medición estado de carga:
  - [The State of Charge Estimating Methods for Battery: A Review](https://www.hindawi.com/journals/isrn/2013/953792/)
  - [Modeling of Lithium-ion Battery for Charging/Discharging Characteristics Based on Circuit Model](https://doi.org/10.3991/ijoe.v13i06.6799)
  - [Constrained generalized predictive control of battery charging process based on a coupled thermoelectric model](https://sci-hub.tw/https://doi.org/10.1016/j.jpowsour.2017.02.039)
- Proyectos para estimación de batería:
    - [rlogiacco/BatterySense][batterysense]
    - [Battery State of Charge Estimation Using Kalman Filter][battery_soc_estimation]
    - [A simple and naive battery modelisation + Kalman filter for state of charge (SoC) estimation][battery-kalman]
- Explicaciones de Filtros de Kalman:
    - [State of Charge Estimation with a Kalman Filter][soc_kalman_estimation_jackogrady]







[bluerobotic_current_measurement_setup]: https://bluerobotics.com/learn/bluerov2-software-setup/#voltage-and-current-measurement-setup
[power_sense_module_bluerobotics]: https://bluerobotics.com/store/comm-control-power/elec-packages/psm-asm-r2-rp/
[herculesi_website]: http://www.herculesi.com/
[diy_can_transceiver]: https://www.electroschematics.com/diy-can-transceiver/
[mcp2551_site]: https://www.microchip.com/wwwproducts/en/en010405
[can_fd_wikipedia]: https://en.wikipedia.org/wiki/CAN_FD
[can_bus_wikipedia]: https://en.wikipedia.org/wiki/CAN_bus
[batterysense]: https://github.com/rlogiacco/BatterySense
[battery_soc_estimation]: https://github.com/AlterWL/Battery_SOC_Estimation
[battery-kalman]: https://github.com/larchuto/Battery-Kalman

[soc_kalman_estimation_jackogrady]: https://www.jackogrady.me/battery-management-system/state-of-charge





