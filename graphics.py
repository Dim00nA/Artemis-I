import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
import math
import os

# 1. ЧТЕНИЕ ДАННЫХ ИЗ KSP

BASE_DIR = os.path.dirname(os.path.abspath('sample.ipynb'))
CSV_FILE = os.path.join(BASE_DIR, "flight_log.csv")

df = pd.read_csv(CSV_FILE)

t_ksp = df["time"].values       # Массив значений времени
h_ksp = df["altitude"].values  # Массив значений высоты
v_ksp = df["speed"].values      # Массив значений скорости

# Извлекаем угол тангажа из данных KSP
pitch_deg = df["pitch"].values  # Углы в градусах

# Преобразуем углы из градусов в радианы
pitch_rad = np.deg2rad(pitch_deg)

# Вычисляем горизонтальную координату для KSP
dt_ksp = np.diff(np.append(0, t_ksp))
v_x_ksp_est = v_ksp * np.cos(pitch_rad)
x_ksp = np.cumsum(v_x_ksp_est * dt_ksp)

# 2. ПАРАМЕТРЫ МАТЕМАТИЧЕСКОЙ МОДЕЛИ
# Физические константы для планеты Kerbin:
mu = 3.5316e12          # Гравитационный параметр
R = 600_000             # Радиус планеты Kerbin (метры)

# Параметры ракетного двигателя:
Isp = 295               # Удельный импульс
g0 = 9.8                # Стандартное ускорение свободного падения
T = 9.643*10**6             # Тяга двигателя

# Аэродинамические параметры:
C_d = 0.25              # Коэффициент лобового сопротивления
S = math.pi * 4         # Площадь поперечного сечения ракеты

# Параметры атмосферы:
rho0 = 1.225            # Плотность атмосферы на уровне моря
H = 5600            # Шкала высоты атмосферы

# 3. НАЧАЛЬНЫЕ УСЛОВИЯ
x0 = 0                  # Начальная горизонтальная координата
y0 = 600000             # Начальная вертикальная координата
v_x0 = 0                # Начальная горизонтальная скорость
v_y0 = 0                # Начальная вертикальная скорость
m0 = 614784             # Начальная масса ракеты

# 4. ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ

def pitch_func(h):
    if h <= 250:
        return np.deg2rad(90)
    elif h <= 45000:
        progress = (h - 250) / (45000 - 250)
        pitch = 90 - progress * 75  # От 80 до 15 градусов
        return np.deg2rad(pitch)
    else:
        return np.deg2rad(15)

def rho(x, y):
    # Функция расчета плотности атмосферы.
    r = math.sqrt(x**2 + y**2 + 1e-8)
    return rho0 * np.exp((R - r) / H)


# 5. ОСНОВНАЯ МАТЕМАТИЧЕСКАЯ МОДЕЛЬ

def model(t, state):
    x, y, v_x, v_y, m = state
    v = math.sqrt(v_x**2 + v_y**2 + 1e-8)
    r = math.sqrt(x**2 + y**2 + 1e-8)
    h = r - R
    Pitch = pitch_func(h)
    ρ = rho(x, y)
    F_drag = 0.5 * C_d * ρ * S * v**2
    g = mu / r**2

    # ДИФФЕРЕНЦИАЛЬНЫЕ УРАВНЕНИЯ:
    # 1. Производная горизонтальной координаты = горизонтальная скорость
    dxdt = v_x
    # 2. Производная вертикальной координаты = вертикальная скорость
    dydt = v_y
    # 3. Горизонтальное ускорение
    dv_xdt = (T * math.cos(Pitch) - F_drag * (v_x / v)) / m
    # 4. Вертикальное ускорение
    dv_ydt = (T * math.sin(Pitch) - F_drag * (v_y / v)) / m - g
    # 5. Расход массы
    dmdt = -T / (Isp * g0) if m > 150000 else 0
    return [dxdt, dydt, dv_xdt, dv_ydt, dmdt]


# 6. ЧИСЛЕННОЕ РЕШЕНИЕ ДИФФЕРЕНЦИАЛЬНЫХ УРАВНЕНИЙ
t_final = min(t_ksp[-1], 300)

# Начальное состояние системы
initial_state = [x0, y0, v_x0, v_y0, m0]
# Решаем систему дифференциальных уравнений
solution = solve_ivp(
    model,                   # Функция с уравнениями
    [0, t_final],            # Интервал времени: от 0 до t_final
    initial_state,           # Начальные условия
    method='RK45',           # Метод решения: Runge-Kutta 4/5 порядка
    max_step=0.1,            # Максимальный шаг интегрирования (секунды)
)


# ИЗВЛЕЧЕНИЕ РЕЗУЛЬТАТОВ
t_mod = solution.t

# Распаковываем результаты:
x_mod = solution.y[0]      # Горизонтальная координата
y_mod = solution.y[1]      # Вертикальная координата
v_x_mod = solution.y[2]    # Горизонтальная скорость
v_y_mod = solution.y[3]    # Вертикальная скорость
m_mod = solution.y[4]      # Масса

# Вычисляем полную скорость (скалярная величина)
v_mod = np.sqrt(v_x_mod**2 + v_y_mod**2)

h_mod = y_mod - 600000

# ПОДГОТОВКА ДАННЫХ ДЛЯ ГРАФИКОВ
mask_ksp_100 = t_ksp <= 120
mask_mod_100 = t_mod <= 120

max_x_limit = 65000

# Маски для фильтрации по горизонтальной координате:
mask_x_ksp = x_ksp <= max_x_limit  # Для данных KSP
mask_x_mod = x_mod <= max_x_limit  # Для данных модели

# Создаем отфильтрованные массивы для графика траектории:
x_ksp_filtered = x_ksp[mask_x_ksp]  # KSP данные
h_ksp_filtered = h_ksp[mask_x_ksp]  # KSP данные по высоте
x_mod_filtered = x_mod[mask_x_mod]  # Данные модели
h_mod_filtered = h_mod[mask_x_mod]  # Данные модели по высоте


# ПОСТРОЕНИЕ ГРАФИКОВ

plt.figure(figsize=(15, 4))
plt.subplot(1, 3, 1)
plt.plot(t_ksp[mask_ksp_100], h_ksp[mask_ksp_100],
         label="KSP", linewidth=2, color='blue')
plt.plot(t_mod[mask_mod_100], h_mod[mask_mod_100],
         label="Модель", linewidth=2, linestyle='--', color='red')
plt.xlabel("Время (с)")
plt.ylabel("Высота (м)")
plt.xlim(0, 120)
plt.grid(True, alpha=0.3)
plt.legend()
plt.subplot(1, 3, 2)
plt.plot(t_ksp[mask_ksp_100], v_ksp[mask_ksp_100],
         label="KSP", linewidth=2, color='blue')
plt.plot(t_mod[mask_mod_100], v_mod[mask_mod_100],
         label="Модель", linewidth=2, linestyle='--', color='red')
plt.xlabel("Время (с)")
plt.ylabel("Скорость (м/с)")
plt.xlim(0, 120)
plt.grid(True, alpha=0.3)
plt.legend()
plt.subplot(1, 3, 3)
plt.plot(x_ksp_filtered, h_ksp_filtered,
         label="KSP", linewidth=2, color='blue')
plt.plot(x_mod_filtered, h_mod_filtered,
         label="Модель", linewidth=2, linestyle='--', color='red')
plt.xlabel("Горизонтальная координата X (м)")
plt.ylabel("Высота (м)")
plt.xlim(0, max_x_limit)
plt.grid(True, alpha=0.3)
plt.legend()

plt.tight_layout()

plt.show()

