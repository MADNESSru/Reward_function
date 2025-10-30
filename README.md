## Варианты расчёта наведения

### A) Симуляция

Считаем вектор вперёд (F) из движка и вектор на цель (L).

Показатель наведения — косинус угла между ними:

$$ \cos\theta = \langle F, L\rangle $$

$$ F = \texttt{GetActorForwardVector()} $$

$$ L = \frac{p_{\text{target}} - p_{\text{drone}}}{\lVert p_{\text{target}} - p_{\text{drone}}\rVert} $$

Реальная дистанция и скорость сближения вдоль LOS:

$$ d = \lVert p_{\text{target}} - p_{\text{drone}}\rVert $$

$$ v_{\text{close}} = \langle v_{\text{drone}} - v_{\text{target}}, L \rangle $$

### B) IRL / только камера

Координаты не нужны — считаем всё по изображению (pinhole-модель).

Пусть \((u,v)\) — центр bbox, \((u_0,v_0)\) — центр кадра, размер кадра \(W\times H\), поля зрения HFOV/VFOV.

$$ \theta_x \approx \arctan\left(\tan\left(\frac{\mathrm{HFOV}}{2}\right) \cdot \frac{2(u - u_0)}{W}\right) $$

$$ \theta_y \approx \arctan\left(\tan\left(\frac{\mathrm{VFOV}}{2}\right) \cdot \frac{2(v - v_0)}{H}\right) $$

$$ \theta = \sqrt{\theta_x^2 + \theta_y^2} $$

$$ \cos\theta = \cos(\theta) $$

## Итоговая формула

$$ R = W_{\text{align}} \cdot R_{\text{align}} + W_{\text{prog}} \cdot R_{\text{prog}} + W_{\text{throttle}} \cdot R_{\text{throttle}} + W_{\text{smooth}} \cdot R_{\text{smooth}} + C_{\text{time}} + K_{\text{lost}} \cdot \mathbf{1}[\text{lost}] + R_{\text{collision}} $$

## Компоненты

### 1) Наведение (луч носа)

$$ R_{\text{align}} = \cos\theta $$

**Вариант A:**

$$ \cos\theta = \langle F, L\rangle $$

$$ F = \texttt{GetActorForwardVector()} $$

$$ L = \frac{p_{\text{target}} - p_{\text{drone}}}{\lVert p_{\text{target}} - p_{\text{drone}}\rVert} $$

**Вариант B:**

$\cos\theta$ из формул выше (через пиксельное смещение и HFOV/VFOV).

### 2) Прогресс по сближению

$$ R_{\text{prog}} = \min\left(\delta, \max\left(-\delta, d_{t-1} - d_t\right)\right) $$

**A:** $d_t = \lVert p_{\text{target}} - p_{\text{drone}}\rVert$

**B:** $d_t$ заменяем на прокси $\tilde{d} \propto 1/\sqrt{S_{\text{bbox}}}$ (только для «близко/далеко»).

### 3) Умный набор скорости (скорость закрытия, с гейтами)

$$
R_{\text{throttle}} =
\begin{cases}
\max(0, v_{\text{close}}), & \text{если } d > d_{\text{far}} \text{ и } \cos\theta > \tau, \\
-\alpha \cdot \max(0, v_{\text{close}}), & \text{если } d < d_{\text{near}} \text{ и } \cos\theta < \cos\theta_{\min}, \\
0, & \text{иначе}.
\end{cases}
$$

где $v_{\text{close}} = \langle v_{\text{drone}} - v_{\text{target}}, L\rangle$

(если $v_{\text{target}}$ неизвестна, берём $\langle v_{\text{drone}}, L\rangle$)

**A:** $d$ — реальная дистанция.

**B:** $d$ — прокси по площади bbox (порог «близко/далеко»).

### 4) Плавность управления

$$ R_{\text{smooth}} = \lVert a_t - a_{t-1}\rVert^2 $$

### 5) Штраф за время

$$ C_{\text{time}} = \text{малый постоянный штраф за каждый шаг} $$

### 6) Потеря цели

$$ K_{\text{lost}} \cdot \mathbf{1}[\text{lost}] $$

### 7) Коллизия (финальная награда)

$$
R_{\text{collision}} =
\begin{cases}
R_c, \text{ и } \texttt{done} = \texttt{True}, & \text{если зафиксирована коллизия}, \\
0, & \text{иначе}.
\end{cases}
$$
