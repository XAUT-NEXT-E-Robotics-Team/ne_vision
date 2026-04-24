# 串口通信协议

## 帧格式

```
[0xAA][0x55][CMD][PAYLOAD...][CRC16_LO][CRC16_HI]
```

| 字段 | 长度 | 说明 |
|------|------|------|
| Header | 2 bytes | 固定 `0xAA 0x55` |
| CMD | 1 byte | 包类型标识 |
| Payload | N bytes | 数据体，直接 memcpy |
| CRC16 | 2 bytes | 小端，覆盖 Header+CMD+Payload |

CRC16 算法：poly `0x8408`（reflected），初始值 `0xFFFF`。

---

## CMD 定义

| CMD | 方向 | 结构体 |
|-----|------|--------|
| `0x01` | MCU → Vision | `GimbalInputProtocol_t` |
| `0x02` | Vision → MCU | `GimbalControlProtocol_t` |

---

## CMD 0x01 — GimbalInputProtocol_t

Payload 大小：44 bytes（`#pragma pack(1)`）

```cpp
struct GimbalInputProtocol_t
{
  float   q[4];           // 四元数 [w, x, y, z]
  float   gyro[3];        // 角速度 [x, y, z]，rad/s
  float   acc[3];         // 加速度 [x, y, z]，m/s²
  char    out_color;      // 我方颜色：0=红，1=蓝
  float   bullet_speed;  // 弹速，m/s
};
```

坐标系：x 向前，y 向左，z 向上（右手系），与 IMU 坐标系一致。

---

## CMD 0x02 — GimbalControlProtocol_t

Payload 大小：17 bytes（`#pragma pack(1)`）

```cpp
struct GimbalControlProtocol_t
{
  float   yaw;             // 期望 yaw，rad，相对 IMU 系
  float   pitch;           // 期望 pitch，rad，相对 IMU 系
  float   yaw_v;           // 期望 yaw 速度，rad/s
  float   pitch_v;         // 期望 pitch 速度，rad/s
  uint8_t hit_probability; // 命中概率 0~255，0 表示无目标（未实装）
};
```

注：未考虑 ROLL 补偿。

---

## 波特率

默认 `115200`，可配置（支持 9600 / 57600 / 115200 / 230400 / 460800 / 921600）。

---

## 示例帧（CMD 0x01）

```
AA 55 01 [44字节payload] [CRC16_LO] [CRC16_HI]
总长：49 bytes
```
