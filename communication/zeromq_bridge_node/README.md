# `zeromq_bridge_node` ROS 2 Package

This ROS 2 package provides a ZeroMQ-based communication bridge for structured air traffic data exchange using a custom SDU format.

---

## 📦 Message Structure: `V2VB_PL0_MSG_T`

Below is the detailed structure of the `V2VB_PL0_MSG_T` payload used in the ZeroMQ communication.

| Field Name               | Type      | Description                        |
|-------------------------|-----------|------------------------------------|
| `payload_type_code`     | `uint8_t` | Payload type code                  |
| `address_qualifier`     | `uint8_t` | Address qualifier                  |
| `icao`                  | `uint32_t`| ICAO address                       |
| `security_mode`         | `uint8_t` | Security mode                      |
| `latitude_wgs84`        | `double`  | Latitude (WGS84)                   |
| `longitude_wgs84`       | `double`  | Longitude (WGS84)                  |
| `true_altitude_ft`      | `double`  | True altitude (feet)               |
| `pressure_altitude_ft`  | `double`  | Pressure altitude (feet)           |
| `absolute_altitude_ft`  | `double`  | Absolute altitude (feet)           |
| `ags`                   | `uint8_t` | AGS flag                           |
| `vst`                   | `uint8_t` | VST flag                           |
| `hvlc_ns_dir`           | `uint8_t` | North/South direction              |
| `hvlc_ns_velocity`      | `double`  | North/South velocity               |
| `hvlc_ew_dir`           | `uint8_t` | East/West direction                |
| `hvlc_ew_velocity`      | `double`  | East/West velocity                 |
| `vvlc_source`           | `uint8_t` | Vertical velocity source           |
| `vvlc_up_down`          | `uint8_t` | Up/Down direction                  |
| `vvlc_velocity`         | `double`  | Vertical velocity                  |
| `utc_coupled`           | `uint8_t` | UTC coupled                        |
| `nic`                   | `uint8_t` | NIC                                |
| `nacp`                  | `uint8_t` | NACp                               |
| `nacv`                  | `uint8_t` | NACv                               |
| `gva`                   | `uint8_t` | GVA                                |
| `sil`                   | `uint8_t` | SIL                                |
| `sda`                   | `uint8_t` | SDA                                |
| `talt_head_valid`       | `uint8_t` | TALT head valid                    |
| `talt_type`             | `uint8_t` | TALT type                          |
| `talt_height`           | `double`  | TALT height                        |
| `talt_pressure_value`   | `uint16_t`| TALT pressure value                |
| `head_valid`            | `uint8_t` | Heading valid                      |
| `head_dir_sign`         | `uint8_t` | Heading direction sign             |
| `head_dir_value`        | `double`  | Heading direction value            |
| `fmc_st`                | `uint8_t` | FMC status                         |
| `fmc_ap`                | `uint8_t` | FMC autopilot                      |
| `fmc_vnav`              | `uint8_t` | FMC VNAV                           |
| `fmc_alt`               | `uint8_t` | FMC altitude                       |
| `fmc_app`               | `uint8_t` | FMC approach                       |
| `fmc_lnav`              | `uint8_t` | FMC LNAV                           |
| `sign`                  | `uint32_t`| Signature                          |
| `reserved_bits`         | `uint32_t`| Reserved bits                      |

---

## 🧩 ROS 2 Parameters

You can configure the ZeroMQ endpoints using the ROS 2 parameter file.

### `config/config.yaml`

```yaml
/**:
  ros__parameters:
    pub_connection_address: "tcp://*:5566"
    sub_connection_address: "tcp://192.168.1.62:5566"
```

These parameters define the publisher binding address and the subscriber connection target.

---

## 📂 Directory Structure

```bash
zeromq_bridge_node/
├── config/
│   └── config.yaml
├── launch/
│   └── zeromq_bridge.launch.py
├── src/
│   ├── zeromq_bridge_pub.cpp
│   └── zeromq_bridge_sub.cpp
├── include/
│   └── zeromq_bridge_node/
│       ├── zeromq_bridge_pub.hpp
│       └── zeromq_bridge_sub.hpp
└── CMakeLists.txt
```

---

## 🧪 Dependencies

Make sure the following libraries are installed:

```bash
sudo apt install libzmq3-dev libczmq-dev
```

---

## 🚀 Example Launch

```bash
ros2 launch zeromq_bridge_node zeromq_bridge.launch.py
```

This will launch both the publisher and subscriber nodes using the parameters provided in `config.yaml`.
