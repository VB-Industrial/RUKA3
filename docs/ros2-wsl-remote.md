# WSL → Raspberry Pi: RUKA3

Развёртывание проверено 2026-11-12.

- WSL-сборка: `/home/robot/projects/RUKA3/build/ros2-install`.
- Локальный запуск: `/home/robot/projects/RUKA3/ros2/moveit_wsl.bash`.
- Ссылка на запуск: `/home/robot/.local/bin/ruka3-moveit`.
- Для ручного ROS CLI: `source /home/robot/projects/RUKA3/ros2/wsl_setup.bash`.
- Discovery: Fast DDS SUPER_CLIENT, `192.168.1.121:11811`, domain 1, как в существующем `/home/pi/.ros_params`.
- MoveIt и RViz — WSL; controller_manager и robot_state_publisher — Pi.

Сборка трёх пакетов прошла. RViz запущен через WSLg (OpenGL 4.6), модель RUKA3 и planning group загружены. Сквозная сетевая проверка выполнена с mock-контроллерами на Pi: MoveIt передал FollowJointTrajectory, Pi исполнила цель joint_1=0.03 рад; конечное измеренное mock-положение в WSL 0.02902725 рад, MoveIt сообщил успешное исполнение. Физический CAN не использовался.

Логи: `build/ruka3-wsl-build.log`, `build/ruka3-wsl-remote-moveit.log`, на Pi — `/home/pi/ws_ruka3/log/wsl-remote/mock-control.log`.

Рабочие команды и режимы приведены в `ros2/README.md`. Глобальные shell startup и сетевые настройки WSL/Windows не менялись. Локальный системный дефект завершения MoveIt по SIGINT не исправлялся.
