# Про планирования движения

В библиотеке roboticstoolbox-python имеется функционал для расчета движения робота.

Траектория движения может быть расчитана в различных пространствах:

- Пространство суставов (joint space): планирование траектории осуществляется в терминах угловых положений суставов.
- Рабочее пространство (cartesian space): задаются целевые позиции в ориентации конечного эффектора робота.

## Планирование траектория из точки A в точку B

Простую траекторию с одной точки до другой можно вычислить с помощью функции `jtraj`,
которая обеспечивает плавное движение робота, гарантируя непрерывность скорости и ускорения.

Функция `jtraj`:

- Использует полином пятого порядка (квинтический).
- Генерирует траектории с учетом начальных и конечных скоростей.
- Предоставляет не только положение суставов, но и их скорости и ускорения.

В примере по [ссылке](../example/use_jtraj.py) происходит следующее:

1. Определение начальной и конечной конфигурации:
    - Начальная конфигурация задается через `robot.qz`, что является нулевым положение каждого сустава робота.
    - Конечная конфигурация вычисляется с помощью обратной кинематики `ikine_LM`, где задается целевая позиция в рабочем пространстве.
2. Генерация временного вектора:
    - Вектор времени `t` задается с помощью функции `np.linspace` от 0 до 3 секунд.
3. Создание траектории:
    - Функция `jtraj` создает массив точек между начальной и конечной конфигурациями для каждого временного шага в `t`.
    - Начальная и конечная скорость по умолчанию равны нулю.

Команда для запуска примера с `jtraj`:

```bash
uv run -m tools.example.use_jtraj
```

~TODO: про движение с трапецеидальным профилем

## Генерация многосегментынх траекторий

Функция `mstraj` используется для создания многосегментной траектории, которая проходит через заданные точки. Гарантиоует плавность и скоординированность движения всех суставов робота.
