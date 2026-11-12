# ROS 2 RUKA3 на Raspberry Pi

Развёрнуто 2026-11-11 на `pi@192.168.1.121` (ARM64, ROS 2 Jazzy).

- Workspace: `/home/pi/ws_ruka3`.
- Исходники: `/home/pi/ws_ruka3/src/RUKA3/ros2`.
- Зависимость DSDL/libcanard: `/home/pi/ws_ruka3/src/RUKA3/Boards/g474/Drivers/libcxxcanard/libs` из текущих исходников RUKA3.
- Сборка: Release, три пакета, `--symlink-install`.
- Совпадение всех 40 ROS source-файлов с локальными подтверждено SHA-256: `/home/pi/ws_ruka3/ros2.sources.sha256`, результат `/home/pi/ws_ruka3/source-check.log`.

В каждом терминале на Pi:

```bash
source ~/ws_ruka3/install/setup.bash
```

Знакомая пара запусков VRHAND:

```bash
# Контроллеры без MoveIt; включает реальные приводы и задаёт нулевые цели.
ros2 launch ruka3_moveit_config with_cont.launch.py

# MoveIt/RViz к уже работающим контроллерам.
ros2 launch ruka3_moveit_config without_cont.launch.py
```

Для mock к обеим командам добавить `use_mock_hardware:=true`. Для headless MoveIt добавить `use_rviz:=false`. Назначение launch-файлов совпадает со старым VR: `with_cont` запускает только control, `without_cont` запускает MoveIt/RViz.

Проверено непосредственно на Pi:

- Сборка всех трёх пакетов.
- Тесты: 5, ошибок и отказов 0. Включена загрузка ARM64 hardware plugin и создание его диагностических publishers без обращения к физическому CAN (заведомо отсутствующий интерфейс).
- `with_cont` с mock поднимает `arm_controller`.
- `without_cont` без RViz подключается к нему, планирует и исполняет цель joint_1=0.03 рад; фактическая конечная позиция mock 0.0301385 рад.

Логи: `build.log`, `tests.log`, `smoke.log`, `log/ruka3-smoke/` в workspace. Для повторения launch-проверки:

```bash
cd ~/ws_ruka3
ROS_DOMAIN_ID=96 python3 src/RUKA3/ros2/tests/launch_smoke.py
```

Все запущенные для проверки ROS-процессы остановлены. Новые пакеты не запускались с физическими приводами. CAN-профиль, systemd, shell startup, прошивка и старый `/home/pi/ws_vr/src/vr` не менялись. Запуск по умолчанию `with_cont` является реальным управлением; mock требует явного параметра.

В установленном ROS на Pi при завершении mock-прогона также наблюдалось зависание/падение штатных `ros2_control_node`/`move_group` после SIGINT. Проверочные процессы были завершены. Этот дефект системного ROS не исправлялся в рамках развёртывания.
