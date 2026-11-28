# Install the Kinisi Mini on a planar chassis

Protocol 2.24 provides board-center setup and readback shared by IMU-only tracking
and fused tracking on the Kinisi Mini v0.4.0. Set it with the robot stopped and
disarmed. The SDK checks the READY board identity, protocol and `imu_mounting` capability.

## Measure the installation

Use the platform's tracking origin, such as the midpoint of a differential axle
or the center of the omni or mecanum wheel geometry. Measure **from this point to
the center of the PCB outline on its underside**, opposite the components.
Use chassis X forward, Y left and Z up, even with a vertical or inverted board. A
board center 100 mm right of the origin has Y = −0.100 m at every board orientation.

Board +X points away from the USB-C edge (J501), board +Y points toward the long
edge nearest U601, and board +Z points out of the component face. At zero angles
these axes align with chassis X, Y and Z. Use right-handed angles in degrees.
The rotation from board to chassis for a column vector is
`Rz(yaw) Ry(pitch) Rx(roll)`. Positive yaw turns board +X left. A +90° roll
points board +Y up and the component face to the right. A 180° roll points the
component face down. Firmware accounts for the sensing axes and the fixed displacement from this point to U601.

## Configure and verify a vertical right-side installation

```cpp
if (controller.supportsBoardMounting()) {
    // Only after stopping and disarming the chassis.
    KinisiBoardMountGeometry mount{};
    if (controller.setBoardMountGeometry(90, 0, 0, 0, -0.100, 0) &&
        controller.getBoardMountGeometry(mount) && mount.configured) {
        Serial.println(mount.y_m, 3);  // -0.100
    }
}
```

Both helpers return false on failure. Inspect `lastError()` for a local validation
error or the board's refusal. The complete readback requires at least 23 Wire bytes.
For exact integer readback on AVR, use the raw interface below.

The six arguments are roll, pitch, yaw in degrees and X, Y, Z in metres.
Limits are inclusive: ±180° roll and yaw, ±90° pitch and ±10 m per axis.
For the raw interface, `set_board_mount(roll_cdeg, pitch_cdeg, yaw_cdeg, x_um, y_um, z_um)`
uses signed centidegrees and micrometres. `get_board_mount()` returns the applied
integer values and `configured`. The example is `(9000, 0, 0, 0, -100000, 0)`.
Call `controller.save_configuration()` after verifying the pose to retain it across
power cycles. On reconnect, read the board-center pose again. A changed mount or reboot
requires fresh tracking initialization. Perform stationary calibration at true rest.
Motion targets, bias and tracking origins are volatile.

## Existing installations

Protocol 2.23 introduced the now-deprecated chip-reference methods
`set_imu_mount` and `get_imu_mount` and their metric helpers `setImuMountGeometry` and `getImuMountGeometry`.
Their coordinates always locate the BMI088 chip. On protocol 2.24, the board-center
getter also reads installations originally entered at the chip. Firmware converts
between the two references. A derived readback can reach ±10.031098 m per axis.
A fresh board returns zero values and `configured=false`. Firmware 2.23 supports
only the chip-reference interface.

Without an explicit mount, IMU-only yaw refers to **sensor +X** and fusion yaw
refers to **board +X**. For a level installation, keep the fusion yaw or add 180°
to the IMU-only sensor yaw and wrap to [−180°, +180°]. Enter the measured PCB
underside-center position. Zero offsets put that point at the tracking origin.
The explicit board rotation then takes precedence over both legacy yaw arguments.

The Mini hardware profile exposes the board-frame displacement to the sensing
package as `controller.boardProfile()->imu_geometry.underside_center_to_imu_um` when `imu_geometry.present` is true. Firmware applies it when deriving the chip position.
