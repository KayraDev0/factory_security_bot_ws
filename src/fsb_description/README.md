# fsb_description

**Amaç:** Fabrika güvenlik robotunun URDF/xacro modeli, mesh'leri ve `<ros2_control>` tanımı.
Fiziksel parametrelerin **tek kaynağı** bu paketteki xacro property dosyasıdır (`urdf/fsb_properties.xacro`, Aşama 2).

**Durum:** Aşama 2 tamam — URDF, ros2_control, RViz.

## Dizinler
- `urdf/` — `fsb_robot.urdf.xacro`, `fsb_properties.xacro`, `fsb_ros2_control.xacro`, `fsb_sensors.xacro`
- `meshes/`, `rviz/`, `launch/`, `config/`

## Launch
```bash
ros2 launch fsb_description display.launch.py                      # RSP + joint_state_publisher_gui + RViz
ros2 launch fsb_description display.launch.py gui:=false rviz:=false
```

## Argümanlar
| Argüman | Varsayılan | Açıklama |
|---|---|---|
| `use_sim` | `false` | xacro'ya iletilir: `true` → `gz_ros2_control/GazeboSimSystem` + Gazebo eklentisi, `false` → `fsb_hardware/FsbSystemHardware` (placeholder, Aşama 7) |
| `use_sim_time` | `false` | Tüm düğümlere iletilir |
| `gui` | `true` | `joint_state_publisher_gui` / `joint_state_publisher` |
| `rviz` | `true` | `rviz/fsb_description.rviz` ile RViz |

## TF ağacı
`base_footprint → base_link → {left_wheel_link, right_wheel_link, caster_link, lidar_link, camera_link, imu_link}`
`base_link` orijini tekerlek milinin orta noktasıdır (dönme merkezi), zeminden `wheel_radius` yukarıda.

## Test
`colcon test --packages-select fsb_description` → `test/test_params_consistency.py`:
URDF ↔ `fsb_control/config/controllers.yaml` (yarıçap, ayrım), zemin teması (tekerlek + caster alt noktası z=0),
tekerlek eksenleri, LiDAR 180°, IMU dönme ekseninde, TF ağacı, atalet geçerliliği, `use_sim` eklenti seçimi.

## Onaylanmış fiziksel değerler (Aşama 2 girdisi)

| Değer | Değer | Kaynak |
|---|---|---|
| Tekerlek yarıçapı | 0.0331 m | — |
| Tekerlek genişliği | 0.025 m | — |
| wheel_separation | 0.128 m | 2×(0.038279+0.025721) |
| Gövde (x,y,z), kütle | 0.25×0.1×0.05 m, 0.8 kg | — |
| Tekerlek kütlesi | 0.032 kg | — |
| LiDAR konumu | (0.0666, 0, 0.084808), rpy 0 0 π | — |
| Kamera konumu | (0.098, 0, 0.025) | — |
| IMU | (0, 0, gövde üstü) — dönme ekseni üzerinde; kütle 0.005 kg VARSAYIM | kullanıcı kararı 2026-11-15 |

Kararlar (kullanıcı onaylı, 2026-11-15):
- TF: `base_footprint → base_link` (andino'nun tersi).
- Zemin teması: tekerlek alt noktası `base_footprint` z=0'a denk gelecek; ofset hesabı URDF'e yorum olarak yazılır.
- Andino controller yaml'ındaki 0.035 / 0.137 değerleri KULLANILMAZ (URDF ile çelişiyor). Gerçek kalibrasyon Aşama 7.
