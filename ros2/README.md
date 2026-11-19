# ROS 2 / MoveIt для RUKA3

ROS 2 Jazzy. Исходники находятся в этом репозитории; старый пакет `vr` на Pi не используется и не изменяется.

- `ruka3_description`: геометрия VRHAND, шесть осей `joint_1`…`joint_6`, две mimic-связи параллелограмма.
- `ruka3_control`: Cyphal hardware plugin, mock, JointTrajectoryController, диагностика и возврат из зоны soft-limit.
- `ruka3_moveit_config`: группа `ruka3_arm`, OMPL/KDL, MoveIt и RViz.

## Сборка

Из корня RUKA3, после получения submodules прошивки:

```bash
git submodule update --init --recursive
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths ros2 --ignore-src -r -y
colcon --log-base build/ros2-log build --base-paths ros2 \
  --build-base build/ros2 --install-base build/ros2-install --symlink-install
source build/ros2-install/local_setup.bash
```

`--base-paths ros2` отделяет ROS-сборку от embedded CMake. Плагин компилирует имеющийся C-core libcanard и использует DSDL-заголовки прошивки G474; вторая копия транспорта не требуется.

## Запуск

```bash
# Только модель и ручная визуализация
ros2 launch ruka3_description display.launch.py

# Контроллеры на mock, без MoveIt
ros2 launch ruka3_control mock.launch.py use_rviz:=false

# MoveIt + mock-контроллеры; RViz опционален
ros2 launch ruka3_moveit_config demo.launch.py use_rviz:=false

# Реальные контроллеры без MoveIt: включает приводы и задаёт нулевые цели
ros2 launch ruka3_control real.launch.py can_interface:=can0

# Реальные контроллеры вместе с MoveIt
ros2 launch ruka3_moveit_config demo.launch.py \
  use_mock_hardware:=false can_interface:=can0

# MoveIt к уже запущенным контроллерам, локально или на Pi
ros2 launch ruka3_moveit_config demo.launch.py \
  start_control:=false use_mock_hardware:=false use_rviz:=false

# Отдельные move_group и RViz
ros2 launch ruka3_moveit_config move_group.launch.py use_mock_hardware:=false
ros2 launch ruka3_moveit_config rviz.launch.py use_mock_hardware:=false
```

Для раздельных хостов необходимы общий `ROS_DOMAIN_ID` и доступная DDS-связь. `controller_manager` и `robot_state_publisher` запускаются только на стороне control. `move_group` публикует world→base_link; не запускайте второй такой publisher. `demo` уже включает `move_group`, отдельно его запускать не нужно.

## Развёртывание на Raspberry Pi

Пакеты собраны на `pi@192.168.1.121` в `/home/pi/ws_ruka3`. Исходники: `/home/pi/ws_ruka3/src/RUKA3/ros2`. Старый `/home/pi/ws_vr/src/vr` сохранён отдельно и не изменяется.

В каждом терминале на Pi:

```bash
source ~/ws_ruka3/install/setup.bash
```

После этого доступны launch-файлы из следующего раздела. На рабочем компьютере перед запуском `without_cont` нужно подключить локальный `build/ros2-install/local_setup.bash`; DDS domain должен совпадать с Pi.

Проверка развёрнутой пары на mock без графики и без физического CAN:

```bash
cd ~/ws_ruka3
source install/setup.bash
ROS_DOMAIN_ID=96 python3 src/RUKA3/ros2/tests/launch_smoke.py
```

Логи сборки/тестов: `~/ws_ruka3/build.log`, `tests.log`, `smoke.log`; логи launch-проверки: `~/ws_ruka3/log/ruka3-smoke/`.

## MoveIt из WSL к удалённой руке

WSL-настройка от 2026-11-12: Fast DDS Discovery Server на Pi `192.168.1.121:11811`, ROS domain `1`. Сетевой профиль: `ros2/config/fastdds-wsl.xml`.

На Pi в терминале контроллеров:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_ruka3/install/local_setup.bash
source ~/.ros_params
ros2 launch ruka3_moveit_config with_cont.launch.py
```

`~/.ros_params` подключает существующий сетевой профиль Pi (domain 1, discovery server на localhost). Реальный `with_cont` включает приводы и задаёт нулевые цели.

В WSL одной командой:

```bash
~/projects/RUKA3/ros2/moveit_wsl.bash
```

Скрипт запускает только MoveIt и RViz через `without_cont.launch.py`. Контроллеры остаются на Pi. Он подключает локальную сборку RUKA3 без старых workspace, сохраняет WSLg и настройки сети из окружения. По умолчанию выбирается domain 1 и Fast DDS-профиль Pi; явно заданные `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION` и DDS-профиль сохраняются.

Для ручного запуска и ROS CLI:

```bash
cd ~/projects/RUKA3
source ros2/wsl_setup.bash
ros2 launch ruka3_moveit_config without_cont.launch.py
# Например, проверить удалённые контроллеры:
ros2 control list_controllers
```

Скрипту можно передавать обычные launch-аргументы, например `use_rviz:=false`. Для испытания на mock на Pi добавить `use_mock_hardware:=true` к `with_cont`, в WSL передать тот же параметр скрипту.

Проверено по сети: MoveIt/RViz в WSL → action-контроллер на Pi → исполнение mock-цели joint_1=0.03 рад → обратная связь в WSL. Реальные приводы не включались. Проверочные процессы остановлены. Прежняя проблема завершения системного MoveIt по SIGINT остаётся отдельной задачей.

## Привычные launch-файлы VRHAND

Семантика сохранена: `with_cont` запускает control без MoveIt, `without_cont` запускает MoveIt/RViz к уже работающему control. Оба находятся в `ruka3_moveit_config`; исходный `vr` не нужен.

```bash
# Терминал 1 на Pi: реальные контроллеры и нулевые цели
ros2 launch ruka3_moveit_config with_cont.launch.py

# Терминал 2 на рабочем компьютере: MoveIt и RViz
ros2 launch ruka3_moveit_config without_cont.launch.py

# Та же пара целиком на mock
ros2 launch ruka3_moveit_config with_cont.launch.py use_mock_hardware:=true
ros2 launch ruka3_moveit_config without_cont.launch.py use_mock_hardware:=true

# Без графики на Pi
ros2 launch ruka3_moveit_config without_cont.launch.py use_rviz:=false
```

## Включение и удержание

При активации hardware plugin ждёт шесть узлов, читает сохранённые лимиты, включает приводы через `arm` и задаёт **позицию 0, скорость 0** всем осям. Ноль должен входить в сохранённый soft-диапазон. При отказе включения снимаются arm-запросы текущей попытки.

Измеренное положение не заменяется нулём. `open_loop_control: true` у JointTrajectoryController сохраняет нулевые command interfaces при его активации; позиционное регулирование выполняет прошивка. Дальняя нулевая цель проходит через её существующий плавный подход. Перед армированием каждой оси hardware записывает стартовые `kp=10`, `kd=0`, `ki=0.1` в регистры и читает индивидуальные Kp/Kd обратно для потоковых команд. Токовые gains и torque feedforward в сообщении равны 0. `i_limit` не изменяется.

### Изменение коэффициентов во время работы

Реальный HW предоставляет `/ruka3/set_gains` стандартного типа `rcl_interfaces/srv/SetParameters`. Можно передать один коэффициент, несколько коэффициентов одного сустава или параметры нескольких суставов:

```bash
source ros2/wsl_setup.bash  # На Pi вместо этого подключить workspace и ~/.ros_params
ros2 service call /ruka3/set_gains rcl_interfaces/srv/SetParameters \
  '{parameters: [
    {name: joint_2.kp, value: {type: 3, double_value: 30.0}},
    {name: joint_2.kd, value: {type: 3, double_value: 0.0}},
    {name: joint_2.ki, value: {type: 3, double_value: 0.1}}
  ]}'
```

`type: 3` — стандартный `PARAMETER_DOUBLE`. Имена: `joint_1`…`joint_6`, суффиксы `.kp`, `.kd`, `.ki`. Неуказанные коэффициенты и суставы не меняются; цели положения/скорости и траектория сохраняются.

Коэффициенты записываются через CAN Register Access. Подтверждённые Kp/Kd сразу заменяют значения в последующих потоковых командах, Ki применяется регистром прошивки. Ответ содержит `successful`/`reason` для каждого параметра в порядке запроса. Применение последовательное, не атомарное; при ошибке уже подтверждённые значения сохраняются. При таймауте результат записи не подтверждён — проверьте регистр через Yakut перед повтором. Одновременно обслуживается один запрос; второй получает отказ с причиной.

Сервис работает через тот же неблокирующий CAN runtime: ожидание ответа не задерживает цикл команд. Доступен у реального hardware, а не `GenericSystem` mock. Записи в RAM, EEPROM не сохраняется. Следующая активация **hardware** снова задаёт 10/0/0.1; перезапуск одного `arm_controller` сохраняет текущие коэффициенты. Изменения Kp/Kd напрямую через Yakut после старта не обновляют host cache — во время ROS-управления используйте этот сервис.

При штатной деактивации hardware plugin отправляет удержание актуального положения, PWM не выключает. При потере feedback не отправляется старая измеренная позиция: локальный watchdog привода сам захватывает положение. Отключение мощности выполняется отдельным `arm 0`.

Контроллер доступен как `/arm_controller/follow_joint_trajectory`. Диагностика — `/diagnostics`.

## Лимиты и восстановление

Hardware plugin читает soft/hard-границы из EEPROM-калибровки через регистр `limits`. Обычные цели ограничиваются soft-диапазоном; наружная скорость у границы обнуляется. Физические защиты прошивки действуют независимо.

При измеренном выходе за soft-limit плагин перехватывает управление всей рукой. Остальные оси удерживают фактические позиции. Вышедшие оси возвращаются с ограничениями 0.1 рад/с и 0.2 рад/с² к границе с отступом 0.01 рад внутрь. Kp не уменьшается. Узел `recovery_supervisor.py` отменяет текущую action-цель; он запускается вместе с реальными контроллерами и не зависит от MoveIt. Состояние перехвата публикуется в `/ruka3/limit_recovery` (transient local).

После возврата старая траектория **не возобновляется**. Для продолжения явно перезапустите trajectory controller:

```bash
ros2 control switch_controllers --deactivate arm_controller
ros2 control switch_controllers --activate arm_controller
```

Повторная активация вновь задаёт нулевые цели. Пока возврат не завершён по измеренной позиции и скорости, активация отклоняется. Перезапускать весь hardware для этого не требуется.

ROS-профиль возврата требует работающего controller manager. При исчезновении хоста действует удержание прошивки по watchdog, а не автономный возврат ROS. Критический fault или потеря связи прерывают ROS-управление; обхода аппаратных защит нет.

Геометрические позиционные границы URDF перенесены из VRHAND. Они пока не заменены измеренными диапазонами всех осей RUKA3. Перед аппаратным вводом необходимо согласовать их с сохранённой калибровкой. Ограничение реальных команд уже использует EEPROM; при недостижении цели действуют допуски trajectory controller. Начальные ограничения планирования: 0.1 рад/с, 1 рад/с² с дополнительным масштабом ускорения 0.1. Они требуют настройки на механизме.

## CAN-контракт

| Ось | Joint | Node ID | Command subject |
|---|---|---:|---:|
| J1 | joint_1 | 101 | 1121 |
| J2 | joint_2 | 102 | 1122 |
| J3 | joint_3 | 103 | 1123 |
| J4 | joint_4 | 104 | 1124 |
| J5 | joint_5 | 105 | 1125 |
| J6 | joint_6 | 106 | 1126 |

Host node=100. Feedback subject=1001 (`voltbro.foc.state_simple.1.0`); command=`voltbro.foc.command.1.0`; heartbeat=7509; Register Access=384. SocketCAN CAN-FD: nominal 1 Мбит/с, data 4 Мбит/с. Настройка интерфейса остаётся в существующем `canstarter`.

Углы — радианы, скорости — знаковые рад/с, gains — публичный выходной контракт прошивки. Редукция и аппаратные знаки на хосте повторно не применяются. Типы/размеры внутренних регистров считаются контрактом; универсального валидатора нет. При активации RPC выполняются с ожиданием; во время работы RPC сервиса gains выполняются асинхронно в цикле `read/write` через неблокирующий SocketCAN.

## Проверки

```bash
colcon --log-base build/ros2-log test --base-paths ros2 --build-base build/ros2 \
  --install-base build/ros2-install
colcon --log-base build/ros2-log test-result --test-result-base build/ros2 --verbose

# При запущенном demo на mock: реальное планирование и исполнение короткой цели
python3 ros2/tests/mock_moveit_smoke.py

# Изолированные user/network namespaces: шесть виртуальных приводов, без физического CAN
unshare -Urn python3 ros2/tests/virtual_can_smoke.py
```

Последняя проверка требует поддержки user namespaces и модуля `vcan` в ядре. Она проверяет включение, начальную нулевую команду при ненулевой измеренной позиции, взаимодействие с JTC, soft-limit recovery, запрет возобновления старой цели и потерю feedback. Логи — `build/ros2-smoke/`.

Аппаратная отладка новых ROS-пакетов на реальной руке ещё не выполнена. Прошивка в рамках переноса не изменялась и не прошивалась. IMU и стабилизация старого VRHAND в эти три пакета пока не включены; шестью осями можно управлять независимо от них.

В текущей локальной установке Jazzy обнаружен SIGSEGV при завершении штатных `ros2_control_node` и `move_group` по SIGINT. Он воспроизводится также на GenericSystem, без загрузки RUKA3 hardware plugin; backtrace находится в деструкторе `rclcpp::Executor`. Рабочие сценарии выше проходят, но чистое завершение этой установки ROS пока не подтверждено. Системные ROS-пакеты в рамках переноса не обновлялись.
