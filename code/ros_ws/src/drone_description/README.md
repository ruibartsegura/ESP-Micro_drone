# drone_description

Paquete ROS 2 (probado mentalmente para **ROS 2 Jazzy + Gazebo Harmonic**, vía `ros_gz`).
Si usas Humble + Gazebo Fortress/Garden cambia `ros_gz_sim`/`ros_gz_bridge` por
`ros_gz_sim`/`ros_gz_bridge` de esa distro (los nombres de paquete son iguales,
pero los plugins de gz cambian de `gz-sim-*` a `ignition-gazebo-*`, revisa versión con `gz sim --version`).

## 1. Estructura de carpetas

```
drone_ws/
└── src/
    └── drone_description/
        ├── package.xml
        ├── CMakeLists.txt
        ├── urdf/
        │   ├── drone.urdf.xacro     <- robot completo (chasis + 4 rotores + IMU)
        │   └── pcb_link.xacro       <- aquí pegas tu bloque de <collision> de la PCB
        ├── meshes/                  <- aquí van TODOS los .dae que exportó tu CAD
        ├── worlds/
        │   └── drone_world.sdf
        ├── config/
        │   └── bridge.yaml          <- mapeo de topics ROS2 <-> Gazebo
        └── launch/
            └── gz_sim.launch.py
```

## 2. Completar lo que falta

1. Pega tu listado completo de `<collision>` (las ~500 entradas del PCB/ESP32/IMU GY-87)
   dentro de `urdf/pcb_link.xacro`, en el hueco marcado.
2. Copia todos los `.dae` referenciados (`pcb_9cbb.dae`, `esp32s2v3.dae`,
   `gy87_10dof_imu_pad*.dae`, `pads*.dae`, etc.) a `meshes/`.
3. Revisa/ajusta masas, brazo del dron (`arm_length`) y constantes de motor
   (`motorConstant`, `momentConstant`) en `drone.urdf.xacro` según tu dron real.

## 3. Dependencias del sistema

```bash
sudo apt install ros-$ROS_DISTRO-ros-gz ros-$ROS_DISTRO-xacro \
                  ros-$ROS_DISTRO-robot-state-publisher \
                  ros-$ROS_DISTRO-actuator-msgs
```

## 4. Build

```bash
cd ~/drone_ws
colcon build --symlink-install
source install/setup.bash
```

## 5. Lanzar la simulación

```bash
ros2 launch drone_description gz_sim.launch.py
```

Esto:
- Arranca Gazebo (gz sim) con el mundo `drone_world.sdf`.
- Publica el URDF con `robot_state_publisher`.
- Spawnea el dron en Gazebo.
- Levanta el bridge ROS2↔Gazebo (IMU y motores).

## 6. Mandar potencia a los motores

El plugin `MulticopterMotorModel` de cada rotor escucha velocidades de motor
(rad/s) en `/model/drone/command/motor_speed` (tipo `gz.msgs.Actuators`),
puenteado a ROS2 como `actuator_msgs/msg/Actuators` en `/drone/motor_speed`.

Ejemplo, para hacer despegar el dron (4 motores a la misma velocidad):

```bash
ros2 topic pub /drone/motor_speed actuator_msgs/msg/Actuators \
  "{velocity: [650.0, 650.0, 650.0, 650.0]}" -r 20
```

- El orden del array es `[fl, fr, bl, br]` (motor_number 0,1,2,3 definidos en el xacro).
- Sube o baja los 4 valores por igual para subir/bajar altitud.
- Diferencia valores entre motores para roll/pitch, y usa la alternancia
  cw/ccw (ya configurada) para yaw.

Para un control más cómodo (mezcla thrust+roll+pitch+yaw en vez de velocidad
por motor), lo habitual es añadir un nodo intermedio en Python que suscriba a
`geometry_msgs/Twist` o similar y calcule las 4 velocidades de motor (mezcla
de cuadricóptero en cruz), y publique en `/drone/motor_speed`. Puedo
escribirte ese nodo si quieres control por velocidad/altitud en vez de por
motor individual.

## 7. Leer la IMU

```bash
ros2 topic echo /drone/imu
```

Publica `sensor_msgs/msg/Imu` (orientación, velocidad angular, aceleración lineal).

## 8. Comprobaciones rápidas si algo no aparece

```bash
gz topic -l | grep drone     # ver topics de Gazebo del modelo
ros2 topic list | grep drone # ver topics ROS2 puenteados
ros2 topic echo /robot_description --once  # confirma que el xacro compila bien
```
