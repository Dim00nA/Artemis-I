import krpc
import time
import csv
import matplotlib.pyplot as plt
import os
import pandas as pd

# ПУТЬ К ФАЙЛАМ
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CSV_FILE = os.path.join(BASE_DIR, "flight_log.csv")

# СОЕДИНЕНИЕ
conn = krpc.connect(name='ARTEMIS I Orbit')
vessel = conn.space_center.active_vessel
ap = vessel.auto_pilot

# НАСТРОЙКИ ПОЛЁТА
turn_start_alt = 250
turn_end_alt = 45000
target_ap = 150000
roll = 90
pitch0, pitch1 = 90, 0

# ПОТОКИ ТЕЛЕМЕТРИИ
flight = vessel.flight()
orbit = vessel.orbit
alt = conn.add_stream(getattr, flight, 'mean_altitude')
apo = conn.add_stream(getattr, orbit, 'apoapsis_altitude')
peri = conn.add_stream(getattr, orbit, 'periapsis_altitude')
speed = conn.add_stream(getattr, vessel.flight(vessel.orbit.body.reference_frame), 'speed')
mass = conn.add_stream(getattr, vessel, 'mass')
dyn_press = conn.add_stream(getattr, flight, 'dynamic_pressure')

# ПОДГОТОВКА ЛОГА
with open(CSV_FILE, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["time", "altitude", "apoapsis", "periapsis", "speed", "pitch", "throttle", "mass", "dynamic_pressure"])
start_time = time.time()

def log_data():
    now = time.time() - start_time
    with open(CSV_FILE, "a", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            round(now, 2),
            round(alt(), 2),
            round(apo(), 2),
            round(peri(), 2),
            round(speed(), 2),
            vessel.flight().pitch,
            vessel.control.throttle,
            round(mass(), 2),
            round(dyn_press(), 2)
        ])

# СТАРТ
vessel.control.sas = False
vessel.control.rcs = False
vessel.control.throttle = 1

ap.engage()
ap.target_pitch_and_heading(pitch0, roll)

print("Запуск")
vessel.control.activate_next_stage()
time.sleep(1)

# ПОВОРОТ
while True:
    h = alt()
    log_data()

    # Плавное уменьшение угла
    if turn_start_alt < h < turn_end_alt:
        k = (h - turn_start_alt) / (turn_end_alt - turn_start_alt)
        new_pitch = pitch0 - k * (pitch0 - pitch1)
        ap.target_pitch_and_heading(new_pitch, roll)
    
    # Фиксируем угол 0° после завершения поворота
    elif h >= turn_end_alt:
        ap.target_pitch_and_heading(0, roll)

    # Вышли на нужный апоцентр — уменьшаем тягу
    if apo() >= target_ap:
        vessel.control.throttle = 0
        break

    time.sleep(1)
# ОТДЕЛЕНИЕ СТУПЕНИ 
log_data()
time.sleep(2)

try:
    lf = vessel.resources_in_decouple_stage(
        vessel.control.current_stage, False
    ).amount("LiquidFuel")
except:
    lf = 0

if lf < 0.1:
    print("Отделение ступени")
    vessel.control.activate_next_stage()
    time.sleep(2)

# ПОДЪЁМ К АПОЦЕНТРУ
print("Подъём к апоцентру")
while alt() < apo() - 500:
    log_data()
    time.sleep(1)

# КРУГЛЕНИЕ ОРБИТЫ
print("Ожидание апоцентра")
while abs(apo() - alt()) > 300:
    log_data()
    time.sleep(0.5)

print("Начало кругления орбиты")

ap.target_pitch_and_heading(0, roll)
vessel.control.throttle = 1

# Круглим пока перицентр не выйдет на ~145 км
while peri() < 145000:
    log_data()
    time.sleep(1)

vessel.control.throttle = 0
print("Орбита успешно округлена!")

# # ОТДЕЛЕНИЕ СТУПЕНИ
# time.sleep(2)

# print("Отделяем ступень")
# vessel.control.activate_next_stage()
# time.sleep(2)

# # ВКЛЮЧЕНИЕ ДВИГАТЕЛЯ
# print("Включение двигателя")
# vessel.control.throttle = 1.0
# burn_time = 2  
# t0 = time.time()
# while time.time() - t0 < burn_time:
#     log_data()
#     time.sleep(1)

# # ОТКЛЮЧЕНИЕ ДВИГАТЕЛЯ
# vessel.control.throttle = 0
# print("Орбитальный модуль вышел на стабильную орбиту.")

for i in range(50):
    log_data()
    time.sleep(1)

time.sleep(1)
print("Миссия завершена.")

# ГРАФИКИ
df = pd.read_csv(CSV_FILE)

def save_plot(x, y, filename, xlabel, ylabel, title):
    plt.figure()
    plt.plot(df[x], df[y])
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.grid(True)
    plt.savefig(os.path.join(BASE_DIR, filename))
    plt.close()

save_plot("time", "altitude", "altitude.png", "Время (c)", "Высота (м)", "Высота vs Время")
save_plot("time", "speed", "speed.png", "Время (c)", "Скорость (м/c)", "Скорость vs Время")
save_plot("time", "pitch", "pitch.png", "Время (c)", "Угол (град)", "Угол vs Время")
save_plot("time", "apoapsis", "apoapsis.png", "Время (c)", "Апоцентр (м)", "Апоцентр vs Время")
save_plot("time", "periapsis", "periapsis.png", "Время (c)", "Перицентр (м)", "Перицентр vs Время")
save_plot("time", "mass", "mass.png", "Время (c)", "Масса (кг)", "Масса ракеты vs Время")
save_plot("time", "dynamic_pressure", "dynamic_pressure.png", "Время (c)", "Давление (Па)", "давление vs Время")

print("Графики сохранены в:", BASE_DIR)
