#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
四旋翼巡航速度 / 续航时间 / 电流估算 — 可视化 GUI 版
（含：电池放电倍率约束、原始极限展示、终端表格化汇总）
Author: luli  + ChatGPT 协作

新增：
  - 在报告结尾输出两张等宽字符表：
      ① 基本参数表：重量、电池（容量/能量、倍率/电流上限）、桨尺寸&叶片数；
      ② 性能汇总表：空载悬停、满载悬停、最大巡航 的功率/电流/续航。
  - “满载”定义为：在悬停可达的最大增量载重 Δm_true 下的总重量（受 thrust/current 更严约束）。
  - 保留“原始极限”展示逻辑（V_thrust_raw / V_current_raw）。

依赖：仅 Python 标准库（tkinter 属于标准库；Ubuntu/Debian 需 apt 安装 python3-tk）。
"""

import io
import json
import math
import sys
from dataclasses import dataclass, asdict
from typing import Optional, Tuple, List

# =========================
# 业务计算模块
# =========================

@dataclass
class Config:
    # --- 质量与环境 ---
    mass_kg: float
    altitude_m: float
    ambient_temp_C: float

    # --- 螺旋桨/电机/几何 ---
    prop_diameter_in: float
    prop_pitch_in: float
    motor_kv: float
    motor_count: int = 4
    blade_count: int = 2

    # --- 电池 ---
    battery_cells_S: int = 6
    battery_voltage_full_per_cell: float = 4.20
    battery_voltage_nom_per_cell: float = 3.70
    battery_voltage_min_per_cell: float = 3.50
    battery_capacity_Ah: Optional[float] = None
    battery_energy_Wh: Optional[float] = None
    battery_usable_fraction: float = 0.8
    # 电池倍率/电流上限（两者二选一，若都给取更小值）
    battery_max_C: Optional[float] = None
    battery_max_current_A: Optional[float] = None

    # --- 效率参数 ---
    figure_of_merit_hover_base: float = 0.72
    motor_efficiency: float = 0.87
    esc_efficiency: float = 0.98
    loaded_rpm_fraction_of_kv: float = 0.90

    # --- Ct/Cp 基准（静态/低速区） ---
    prop_Ct0_base: float = 0.10
    prop_Cp0_base: float = 0.055

    # --- 桨叶数修正（经验） ---
    three_blade_Ct_multiplier: float = 1.10
    three_blade_Cp_multiplier: float = 1.12
    three_blade_FoM_delta: float = -0.03

    # --- 前进比经验模型 ---
    k_ct: float = 1.0
    k_cp: float = 0.6
    slip_fraction: float = 0.30

    # --- 巡航估算模式 ---
    mode: str = "drag_match"   # "drag_match" 或 "throttle"
    cruise_throttle_frac: float = 0.60

    # --- 气动阻力 ---
    CdA_m2: float = 0.06

    # --- 安装效应 ---
    install_Ct_multiplier: float = 1.0
    install_Cp_multiplier: float = 1.0

    # 曲线修正：J 与 k_T / k_P（可空）
    install_kT_curve_J: Optional[List[float]] = None
    install_kT_curve_val: Optional[List[float]] = None
    install_kP_curve_J: Optional[List[float]] = None
    install_kP_curve_val: Optional[List[float]] = None

    # --- 搜索上限（“原始极限”求解） ---
    J_cap_for_raw_search: float = 1.4

# 示例默认参数
DEFAULT_CONFIG = Config(
    mass_kg=3.0,
    altitude_m=400.0,
    ambient_temp_C=25.0,
    prop_diameter_in=8.0,
    prop_pitch_in=6.0,
    motor_kv=1100,
    motor_count=4,
    blade_count=3,
    battery_cells_S=6,
    battery_voltage_full_per_cell=4.20,
    battery_voltage_nom_per_cell=3.70,
    battery_voltage_min_per_cell=3.50,
    battery_capacity_Ah=5.3,
    battery_energy_Wh=None,
    battery_usable_fraction=0.80,
    battery_max_C=30.0,
    battery_max_current_A=None,
    figure_of_merit_hover_base=0.72,
    motor_efficiency=0.87,
    esc_efficiency=0.98,
    loaded_rpm_fraction_of_kv=0.90,
    prop_Ct0_base=0.10,
    prop_Cp0_base=0.055,
    three_blade_Ct_multiplier=1.10,
    three_blade_Cp_multiplier=1.12,
    three_blade_FoM_delta=-0.03,
    k_ct=1.0,
    k_cp=0.6,
    slip_fraction=0.30,
    mode="drag_match",
    cruise_throttle_frac=0.60,
    CdA_m2=0.06,
    install_Ct_multiplier=1.0,
    install_Cp_multiplier=1.0,
    install_kT_curve_J=None,
    install_kT_curve_val=None,
    install_kP_curve_J=None,
    install_kP_curve_val=None,
    J_cap_for_raw_search=1.4,
)

# ---------- 工具函数 ----------

def air_density_kg_m3(alt_m: float, ambient_temp_C: Optional[float] = None) -> Tuple[float,float,float,float,float,float,float]:
    T0 = 288.15; p0 = 101325.0; L = 0.0065; g = 9.80665; R = 287.058
    h = max(0.0, alt_m)
    if h <= 11000.0:
        T_isa = T0 - L*h
        p = p0 * (T_isa / T0) ** (g / (R*L))
        T_used = T_isa
    else:
        T_isa = T0 - L*11000.0
        p = p0 * (T_isa / T0) ** (g / (R*L))
        T_used = T_isa
    if ambient_temp_C is not None:
        T_used = ambient_temp_C + 273.15
    rho = p / (R * T_used)
    return rho, T0, p0, L, g, R, T_used

def in_to_m(inches: float) -> float:
    return inches * 0.0254

def m_to_in(meters: float) -> float:
    return meters / 0.0254

def interp1(x: float, xs: List[float], ys: List[float]) -> float:
    if xs is None or ys is None or len(xs) == 0 or len(xs) != len(ys):
        return 1.0
    n = len(xs)
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    lo, hi = 0, n-1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if xs[mid] <= x:
            lo = mid
        else:
            hi = mid
    x0, x1 = xs[lo], xs[hi]
    y0, y1 = ys[lo], ys[hi]
    t = (x - x0) / max(1e-12, (x1 - x0))
    return y0 + t * (y1 - y0)

def effective_coeffs_and_fom(cfg: Config) -> Tuple[float, float, float]:
    Ct0 = cfg.prop_Ct0_base; Cp0 = cfg.prop_Cp0_base; FoM = cfg.figure_of_merit_hover_base
    if cfg.blade_count == 3:
        Ct0 *= cfg.three_blade_Ct_multiplier
        Cp0 *= cfg.three_blade_Cp_multiplier
        FoM = max(0.30, FoM + cfg.three_blade_FoM_delta)
    elif cfg.blade_count != 2:
        raise ValueError("仅支持 2 或 3 叶桨")
    Ct0 *= cfg.install_Ct_multiplier
    Cp0 *= cfg.install_Cp_multiplier
    return Ct0, Cp0, FoM

def Ct_Cp_at_J(Ct0: float, Cp0: float, J: float, k_ct: float, k_cp: float, cfg: Config) -> Tuple[float, float]:
    Ct = max(0.02, Ct0 * (1.0 - k_ct * (J**2)))
    Cp = max(0.02, Cp0 * (1.0 + k_cp * (J**2)))
    if cfg.install_kT_curve_J and cfg.install_kT_curve_val:
        Ct *= interp1(J, cfg.install_kT_curve_J, cfg.install_kT_curve_val)
    if cfg.install_kP_curve_J and cfg.install_kP_curve_val:
        Cp *= interp1(J, cfg.install_kP_curve_J, cfg.install_kP_curve_val)
    return Ct, Cp

def hover_feasibility_block(cfg: Config, rho: float) -> Tuple[bool, float, float, float, float, float, float, float]:
    g = 9.80665; W = cfg.mass_kg * g
    V_nom = cfg.battery_cells_S * cfg.battery_voltage_nom_per_cell
    V_full = cfg.battery_cells_S * cfg.battery_voltage_full_per_cell
    rpm_nom_loaded  = cfg.loaded_rpm_fraction_of_kv * cfg.motor_kv * V_nom
    rpm_full_loaded = cfg.loaded_rpm_fraction_of_kv * cfg.motor_kv * V_full
    n_nom  = rpm_nom_loaded  / 60.0
    n_full = rpm_full_loaded / 60.0
    D_m = in_to_m(cfg.prop_diameter_in)
    Ct0, _, _ = effective_coeffs_and_fom(cfg)
    T_nom  = cfg.motor_count * Ct0 * rho * (n_nom**2)  * (D_m**4)
    T_full = cfg.motor_count * Ct0 * rho * (n_full**2) * (D_m**4)
    n_req_hover = math.sqrt(W / max(1e-12, (cfg.motor_count * Ct0 * rho * (D_m**4))))
    rpm_req_hover = 60.0 * n_req_hover
    kv_req_for_hover_nomV = rpm_req_hover / max(1e-9, (cfg.loaded_rpm_fraction_of_kv * V_nom))
    D_req_full_m = (W / max(1e-12, (cfg.motor_count * Ct0 * rho * (n_full**2)))) ** 0.25
    D_req_full_in = m_to_in(D_req_full_m)
    ok_hover_full = (T_full >= W)

    print("[可飞性自检（静态/低速 J≈0，用 Ct0；含安装效应修正）]")
    print("  公式：T_total = N · Ct0 · ρ · n² · D⁴（n = rpm/60）")
    print(f"  说明：此处 Ct0 已包含：叶片数修正、安装效应标量修正（install_Ct_multiplier = {cfg.install_Ct_multiplier:.3f}）")
    print(f"  代入（名义）：N={cfg.motor_count}, Ct0={Ct0:.3f}, ρ={rho:.3f}, D={D_m:.4f} m, rpm_nom={rpm_nom_loaded:.0f}")
    print(f"         → n_nom={n_nom:.3f} rps → T_nom = {T_nom:.2f} N")
    print(f"  代入（满电）：N={cfg.motor_count}, Ct0={Ct0:.3f}, ρ={rho:.3f}, D={D_m:.4f} m, rpm_full={rpm_full_loaded:.0f}")
    print(f"         → n_full={n_full:.3f} rps → T_full = {T_full:.2f} N")
    print("  对比重量：W = m·g")
    print(f"         → m={cfg.mass_kg:.3f} kg, g={g:.5f} → W = {W:.2f} N")
    if ok_hover_full:
        print("  结论：满电可悬停（T_full ≥ W） → 继续输出后续详细计算与报告")
    else:
        print("  结论：满电不可悬停（T_full < W） → 参数不可用，停止后续计算")
        print("  反推悬停所需转速（静态近似）")
        print("  公式：n_req = sqrt(W / (N·Ct0·ρ·D⁴))，rpm_req = 60·n_req")
        print(f"  代入：n_req={n_req_hover:.3f} rps → rpm_req = {rpm_req_hover:.0f} rpm")
        print("  若保持 6S 名义电压与负载系数，反推所需 KV（静态近似）")
        print("  公式：rpm_req = (负载系数) × KV × V_nom → KV_req = rpm_req / ((负载系数)·V_nom)")
        print(f"  代入：KV_req = {rpm_req_hover:.0f} / ({cfg.loaded_rpm_fraction_of_kv:.2f} × {V_nom:.1f}) → {kv_req_for_hover_nomV:.0f} KV")
        print("  若保持当前满电加载转速 rpm_full，不改 KV 与串数，仅靠增大桨径以实现悬停（静态近似）")
        print("  公式：W = N·Ct0·ρ·n_full²·D_req⁴ → D_req = ( W / (N·Ct0·ρ·n_full²) )^(1/4)")
        print(f"  代入：D_req = {D_req_full_in:.1f} in（当前 {cfg.prop_diameter_in:.1f} in）")
    return ok_hover_full, rpm_nom_loaded, rpm_full_loaded, T_nom, T_full, rpm_req_hover, kv_req_for_hover_nomV, D_req_full_in


def hover_power_elec_W_for_mass(cfg: Config, rho: float, mass_kg: float) -> float:
    """给定质量下的悬停电功率（W）。"""
    g = 9.80665; W = mass_kg * g
    D_m = in_to_m(cfg.prop_diameter_in)
    A_total = cfg.motor_count * math.pi * (D_m/2)**2
    P_ind = (W ** 1.5) / math.sqrt(2.0 * rho * A_total)
    _, _, FoM = effective_coeffs_and_fom(cfg)
    eta_elec = max(0.30, cfg.motor_efficiency * cfg.esc_efficiency)
    P_shaft = P_ind / max(0.30, FoM)
    return P_shaft / eta_elec

def hover_power_elec_W(cfg: Config, rho: float) -> float:
    return hover_power_elec_W_for_mass(cfg, rho, cfg.mass_kg)

def electricals_at_rpm_and_Cp(cfg: Config, rho: float, rpm: float, Cp_eff: float) -> Tuple[float, float, float]:
    D_m = in_to_m(cfg.prop_diameter_in); n = rpm / 60.0
    P_shaft = cfg.motor_count * Cp_eff * rho * (n**3) * (D_m**5)
    eta_elec = max(0.30, cfg.motor_efficiency * cfg.esc_efficiency)
    P_elec = P_shaft / eta_elec
    V_nom = cfg.battery_cells_S * cfg.battery_voltage_nom_per_cell
    I = P_elec / max(1.0, V_nom)
    return P_shaft, P_elec, I

def pack_max_current_A(cfg: Config) -> Optional[float]:
    """计算电池包最大持续电流（A），若信息不足返回 None。"""
    V_nom = cfg.battery_cells_S * cfg.battery_voltage_nom_per_cell
    Ah = cfg.battery_capacity_Ah
    if Ah is None and cfg.battery_energy_Wh is not None:
        Ah = cfg.battery_energy_Wh / max(1e-6, V_nom)
    I_by_C = None
    if cfg.battery_max_C is not None and Ah is not None:
        I_by_C = cfg.battery_max_C * Ah
    candidates = [x for x in [cfg.battery_max_current_A, I_by_C] if x is not None]
    if not candidates:
        return None
    return max(0.0, min(candidates))  # 若两者都给，取更严苛的较小值

def max_speed_at_rpm_with_constraints(cfg: Config, rho: float, rpm: float) -> Tuple[
    float, float, float, float, float, float, float, float, float, float, str]:
    """
    返回：
      V_pitch, V_thrust_raw, V_current_raw, V_thrust, V_current, V_true, J_at_true,
      Ct_eff_at_true, Cp_eff_at_true, I_at_true, limiter_code
      limiter_code in {"pitch", "thrust", "current", "none"}
    """
    D_m = in_to_m(cfg.prop_diameter_in)
    pitch_m = in_to_m(cfg.prop_pitch_in)
    n = rpm / 60.0
    V_pitch = (1.0 - cfg.slip_fraction) * pitch_m * n

    Ct0, Cp0, _ = effective_coeffs_and_fom(cfg)
    W = cfg.mass_kg * 9.80665; N = cfg.motor_count
    I_max = pack_max_current_A(cfg)

    def T_avail_at(V: float) -> Tuple[float, float, float]:
        J = V / max(1e-9, n * D_m)
        Ct_eff, Cp_eff = Ct_Cp_at_J(Ct0, Cp0, J, cfg.k_ct, cfg.k_cp, cfg)
        T_avail = N * Ct_eff * rho * (n**2) * (D_m**4)
        return J, Ct_eff, T_avail

    def I_at(V: float) -> Tuple[float, float, float, float]:
        J = V / max(1e-9, n * D_m)
        Ct_eff, Cp_eff = Ct_Cp_at_J(Ct0, Cp0, J, cfg.k_ct, cfg.k_cp, cfg)
        _, _, I = electricals_at_rpm_and_Cp(cfg, rho, rpm, Cp_eff)
        return J, Ct_eff, Cp_eff, I

    # 起步可达性检查
    _, _, T0_avail = T_avail_at(0.0)
    if T0_avail < W:
        Jt, Ct_eff_true, Cp_eff_true, It = I_at(0.0)
        return V_pitch, 0.0, 0.0, 0.0, 0.0, 0.0, Jt, Ct_eff_true, Cp_eff_true, It, "thrust"

    # 原始搜索上限（不受 V_pitch 截断）
    V_hi_raw = n * D_m * max(0.6, cfg.J_cap_for_raw_search)

    # 推力原始极限
    lo, hi = 0.0, V_hi_raw
    V_thrust_raw = 0.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        _, _, T_av = T_avail_at(mid)
        Dm = 0.5 * rho * max(1e-6, cfg.CdA_m2) * mid * mid
        T_req = math.sqrt(W * W + Dm * Dm)
        if T_av >= T_req:
            V_thrust_raw = mid; lo = mid
        else:
            hi = mid

    # 电流原始极限
    if I_max is None or I_max <= 0:
        V_current_raw = float('inf')
    else:
        lo, hi = 0.0, V_hi_raw
        V_current_raw = 0.0
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            _, _, _, Im = I_at(mid)
            if Im <= I_max:
                V_current_raw = mid; lo = mid
            else:
                hi = mid

    # 截断后的极限（用于最终 V_true）
    V_thrust = min(V_pitch, V_thrust_raw if V_thrust_raw > 0 else 0.0)
    V_current = min(V_pitch, V_current_raw if math.isfinite(V_current_raw) else float('inf'))

    V_candidates = [V_pitch, V_thrust, V_current]
    V_true = min(V for V in V_candidates if not (isinstance(V, float) and math.isinf(V)))
    Jt, Ct_eff_true, Cp_eff_true, It = I_at(V_true)

    if abs(V_true - V_pitch) < 1e-6:
        limiter = "pitch"
    elif abs(V_true - V_thrust) < 1e-6:
        limiter = "thrust"
    elif isinstance(V_current, float) and not math.isinf(V_current) and abs(V_true - V_current) < 1e-6:
        limiter = "current"
    else:
        limiter = "none"

    return (V_pitch, V_thrust_raw, V_current_raw, V_thrust, V_current,
            V_true, Jt, Ct_eff_true, Cp_eff_true, It, limiter)

def solve_cruise(cfg: Config, rho: float):
    """综合扫描 rpm（依据模式），返回最优速度点及电参。"""
    V_nom = cfg.battery_cells_S * cfg.battery_voltage_nom_per_cell
    rpm_base = cfg.loaded_rpm_fraction_of_kv * cfg.motor_kv * V_nom

    best = None
    if cfg.mode == "throttle":
        rpm_grid = [max(0.0, min(1.0, cfg.cruise_throttle_frac)) * rpm_base]
    elif cfg.mode == "drag_match":
        rpm_grid = [ (i/50) * rpm_base for i in range(15, 51) ]  # 0.30~1.00×RPM_base
    else:
        raise ValueError("mode 只能为 'drag_match' 或 'throttle'")

    for rpm in rpm_grid:
        (V_pitch, Vth_raw, Vcur_raw, Vth, Vcur,
         Vtrue, J, Ct_eff, Cp_eff, I_true, limiter) = max_speed_at_rpm_with_constraints(cfg, rho, rpm)
        P_shaft, P_elec, I = electricals_at_rpm_and_Cp(cfg, rho, rpm, Cp_eff)
        theta_deg = 0.0
        if Vtrue > 0:
            W = cfg.mass_kg * 9.80665
            D_drag = 0.5 * rho * max(1e-6, cfg.CdA_m2) * Vtrue * Vtrue
            theta_deg = math.degrees(math.atan2(D_drag, W))
        rec = (Vtrue, rpm, V_pitch, Vth_raw, Vcur_raw, Vth, Vcur, J,
               Ct_eff, Cp_eff, P_shaft, P_elec, I, theta_deg, limiter)
        if best is None or Vtrue > best[0]:
            best = rec

    (V_true, rpm, Vp, Vth_raw, Vcur_raw, Vth, Vcur, J, Ct_eff, Cp_eff,
     P_shaft, P_elec, I, theta_deg, limiter) = best
    return rpm, Vp, Vth_raw, Vcur_raw, Vth, Vcur, V_true, J, theta_deg, P_shaft, P_elec, I, limiter

def endurance_minutes(cfg: Config, P_elec_W: float) -> float:
    V_nom = cfg.battery_cells_S * cfg.battery_voltage_nom_per_cell
    if cfg.battery_energy_Wh is not None:
        E_Wh = cfg.battery_energy_Wh
    elif cfg.battery_capacity_Ah is not None:
        E_Wh = cfg.battery_capacity_Ah * V_nom
    else:
        raise ValueError("需要提供 battery_energy_Wh 或 battery_capacity_Ah 之一。")
    E_use = E_Wh * max(0.10, min(1.0, cfg.battery_usable_fraction))
    t_h = E_use / max(1e-6, P_elec_W)
    return t_h * 60.0

def max_payload_hover(cfg: Config, rho: float, T_full_N: float) -> Tuple[float, Optional[float], float, Optional[float]]:
    """基于悬停能力的最大增量载重（Δm，kg）。"""
    g = 9.80665
    # 1) 推力上限
    m_max_thrust = max(0.0, T_full_N / g)
    d_m_thrust = max(0.0, m_max_thrust - cfg.mass_kg)
    # 2) 电流上限（若存在）
    I_max = pack_max_current_A(cfg)
    if I_max is None or I_max <= 0:
        return d_m_thrust, None, m_max_thrust, None
    V_nom = cfg.battery_cells_S * cfg.battery_voltage_nom_per_cell
    D_m = in_to_m(cfg.prop_diameter_in)
    A_total = cfg.motor_count * math.pi * (D_m/2)**2
    _, _, FoM = effective_coeffs_and_fom(cfg)
    eta_elec = max(0.30, cfg.motor_efficiency * cfg.esc_efficiency)
    term = I_max * V_nom * FoM * eta_elec * math.sqrt(2.0 * rho * max(1e-12, A_total))
    W_max_current = max(0.0, term) ** (2.0/3.0)
    m_max_current = W_max_current / g
    d_m_current = max(0.0, m_max_current - cfg.mass_kg)
    return d_m_thrust, d_m_current, m_max_thrust, m_max_current

def line():
    print("-" * 96)

def _fmt_row(cols, widths, aligns=None):
    if aligns is None: aligns = ["<"] * len(cols)
    parts = []
    for i, v in enumerate(cols):
        w = widths[i]
        a = aligns[i]
        s = f"{v}"
        if a == "<":
            parts.append(s.ljust(w))
        elif a == ">":
            parts.append(s.rjust(w))
        else:
            parts.append(s.center(w))
    return " | ".join(parts)

def _print_kv_table(title: str, items: List[Tuple[str, str]]):
    print(title)
    key_w = max(6, max(len(k) for k,_ in items))
    val_w = max(6, max(len(v) for _,v in items))
    print(_fmt_row(["项目", "值"], [key_w, val_w]))
    print(_fmt_row(["-"*key_w, "-"*val_w], [key_w, val_w]))
    for k, v in items:
        print(_fmt_row([k, v], [key_w, val_w]))
    line()

def _print_perf_table(rows: List[Tuple[str, float, float, float, float, str]]):
    """
    rows: 每行 (场景, 质量kg, 功率W, 电流A, 续航min, 备注)
    """
    headers = ["场景", "质量(kg)", "功率(W)", "电流(A)", "续航(min)", "备注"]
    # 计算列宽
    str_rows = [[
        r[0],
        f"{r[1]:.2f}",
        f"{r[2]:.0f}",
        f"{r[3]:.1f}",
        f"{r[4]:.1f}",
        r[5]
    ] for r in rows]
    widths = [max(len(headers[i]), max(len(sr[i]) for sr in str_rows)) for i in range(len(headers))]
    print(_fmt_row(headers, widths))
    print(_fmt_row(["-"*w for w in widths], widths))
    for sr in str_rows:
        print(_fmt_row(sr, widths))
    line()

def run_report(cfg: Config) -> str:
    buf = io.StringIO(); _stdout = sys.stdout
    try:
        sys.stdout = buf
        print("四旋翼巡航/续航/电流估算 — 详细报告（含电池倍率约束 & 原始极限展示 & 表格汇总）")
        line()
        rho, *_ = air_density_kg_m3(cfg.altitude_m, cfg.ambient_temp_C)
        print("[环境密度]")
        print(f"  海拔={cfg.altitude_m:.0f} m，温度={cfg.ambient_temp_C:.1f} °C → ρ = {rho:.3f} kg/m³")
        line()

        ok_hover_full, rpm_nom_loaded, rpm_full_loaded, T_nom, T_full, rpm_req_hover, kv_req_for_hover_nomV, D_req_full_in = hover_feasibility_block(cfg, rho)
        line()
        if not ok_hover_full:
            print("【参数不可用（不可悬停）— 停止后续计算】")
            return buf.getvalue()

        # 悬停功率/电流/续航（空载=当前质量）
        P_elec_hover_empty = hover_power_elec_W(cfg, rho)
        V_nom = cfg.battery_cells_S * cfg.battery_voltage_nom_per_cell
        I_hover_empty = P_elec_hover_empty / max(1.0, V_nom)
        t_hover_empty_min = endurance_minutes(cfg, P_elec_hover_empty)
        print("[悬停功率/电流/续航（空载=当前质量）]")
        print(f"  P_elec ≈ {P_elec_hover_empty:.0f} W，I ≈ {I_hover_empty:.1f} A，续航 ≈ {t_hover_empty_min:.1f} 分钟")
        line()

        # 最大载重（悬停可达）
        print("[最大载重（悬停可达）— 推导全过程]")
        d_m_thrust, d_m_current, m_max_thrust, m_max_current = max_payload_hover(cfg, rho, T_full)
        if d_m_current is None:
            d_m_true = d_m_thrust; limiter_payload = "thrust"
            m_full = m_max_thrust
        else:
            d_m_true = min(d_m_thrust, d_m_current)
            limiter_payload = "thrust" if d_m_true == d_m_thrust else "current"
            m_full = cfg.mass_kg + d_m_true
        print(f"  结论：最大增量载重 Δm_true = {d_m_true:.3f} kg（受 {limiter_payload} 约束）")
        print(f"        → 满载重量 m_full = {m_full:.3f} kg")
        line()

        # 满载悬停功率/电流/续航（若 m_full == mass 则与空载相同）
        P_elec_hover_full = hover_power_elec_W_for_mass(cfg, rho, m_full)
        I_hover_full = P_elec_hover_full / max(1.0, V_nom)
        t_hover_full_min = endurance_minutes(cfg, P_elec_hover_full)

        # 电池电流上限
        I_max = pack_max_current_A(cfg)
        if I_max is None:
            print("[电池电流上限] 未提供倍率/电流上限；报告继续（不作为约束）。")
        else:
            src = "battery_max_current_A" if cfg.battery_max_current_A is not None else "battery_max_C×Ah"
            print(f"[电池电流上限] I_max = {I_max:.1f} A（来源：{src}）")
        line()

        # 巡航/最大速度求解
        (rpm, Vp, Vth_raw, Vcur_raw, Vth, Vcur, Vtrue, J, theta_deg,
         P_shaft_cruise, P_elec_cruise, I_cruise, limiter) = solve_cruise(cfg, rho)
        print("[巡航/最大速度求解（综合约束）]")
        print(f"  RPM ≈ {rpm:.0f} rpm")
        print(f"  V_pitch = {Vp:.2f} m/s；V_thrust_raw = {Vth_raw:.2f} m/s；"
              f"V_current_raw = {'N/A' if not math.isfinite(Vcur_raw) else f'{Vcur_raw:.2f}'} m/s")
        if math.isfinite(Vth):
            print(f"  V_thrust = min(V_pitch, V_thrust_raw) = {Vth:.2f} m/s")
        else:
            print("  V_thrust = +inf")
        if I_max is None:
            print("  V_current = N/A（未设 I_max）")
        else:
            print(f"  V_current = min(V_pitch, V_current_raw) = {Vcur:.2f} m/s")
        print(f"  → V_true = {Vtrue:.2f} m/s（束缚项：{limiter}），J={J:.3f}，姿态角≈{theta_deg:.1f}°")
        print(f"  巡航电参：P_elec ≈ {P_elec_cruise:.0f} W，I ≈ {I_cruise:.1f} A")
        if I_max is not None and I_cruise > I_max + 1e-6:
            print("  [警告] 当前最大速度点电流超过电池持续上限！应降低 rpm/速度或更换电池/并联。")
        line()

        # 巡航续航
        t_cruise_min = endurance_minutes(cfg, P_elec_cruise)
        print("[等功率巡航续航估算]")
        print(f"  以 P_elec_cruise ≈ {P_elec_cruise:.0f} W 计，续航 ≈ {t_cruise_min:.1f} 分钟")
        line()

        # =========================
        # 终端表格化汇总
        # =========================
        # —— 基本参数表 ——
        V_full_pack = cfg.battery_cells_S * cfg.battery_voltage_full_per_cell
        V_nom_pack  = cfg.battery_cells_S * cfg.battery_voltage_nom_per_cell
        cap_Ah = cfg.battery_capacity_Ah
        if cap_Ah is None and cfg.battery_energy_Wh is not None:
            cap_Ah = cfg.battery_energy_Wh / max(1e-6, V_nom_pack)
        cap_Wh = cfg.battery_energy_Wh if cfg.battery_energy_Wh is not None else (cfg.battery_capacity_Ah or 0) * V_nom_pack
        battery_desc = f"{cfg.battery_cells_S}S，{(cap_Ah or 0):.1f}Ah / {cap_Wh:.0f}Wh，η可用={cfg.battery_usable_fraction:.2f}"
        if cfg.battery_max_C is not None:
            battery_desc += f"，C={cfg.battery_max_C:.1f}"
        if cfg.battery_max_current_A is not None:
            battery_desc += f"，I_max(A)={cfg.battery_max_current_A:.0f}"
        if I_max is not None:
            battery_desc += f"，I_max(实际)={I_max:.0f}A"
        prop_desc = f"{cfg.prop_diameter_in:.1f}×{cfg.prop_pitch_in:.1f} in，{cfg.blade_count}叶×{cfg.motor_count}桨"

        _print_kv_table("【汇总 - 基本参数】", [
            ("当前重量(kg)", f"{cfg.mass_kg:.3f}"),
            ("满载重量(kg)", f"{m_full:.3f}"),
            ("电池", battery_desc),
            ("桨/叶片", prop_desc),
        ])

        # —— 性能汇总表 ——
        perf_rows = [
            ("空载悬停", cfg.mass_kg, P_elec_hover_empty, I_hover_empty, t_hover_empty_min, "当前质量"),
            ("满载悬停", m_full, P_elec_hover_full, I_hover_full, t_hover_full_min, f"受{limiter_payload}约束"),
            ("最大巡航", cfg.mass_kg, P_elec_cruise, I_cruise, t_cruise_min, f"V≈{Vtrue:.1f}m/s, 限制:{limiter}"),
        ]
        print("【汇总 - 性能表】")
        _print_perf_table(perf_rows)

        # —— 文字汇总保留（用于 GUI 区右侧文本）——
        print("[汇总（文字版）]")
        print(f"  悬停（空载）：P≈{P_elec_hover_empty:.0f}W，I≈{I_hover_empty:.1f}A，t≈{t_hover_empty_min:.1f}min")
        print(f"  悬停（满载）：P≈{P_elec_hover_full:.0f}W，I≈{I_hover_full:.1f}A，t≈{t_hover_full_min:.1f}min（满载={m_full:.2f}kg）")
        vc_note = f"V_max≈{Vtrue:.1f}m/s（受{limiter}限制）" if Vtrue>0 else "V_max≈0（不可达）"
        print(f"  巡航：{vc_note}；P≈{P_elec_cruise:.0f}W，I≈{I_cruise:.1f}A，t≈{t_cruise_min:.1f}min")
        return buf.getvalue()
    finally:
        sys.stdout = _stdout

# =========================
# GUI（Tkinter）
# =========================
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

class LabeledEntry(ttk.Frame):
    def __init__(self, master, label, var, width=12, unit=None, tooltip=None):
        super().__init__(master)
        self.var = var
        self.label = ttk.Label(self, text=label)
        self.entry = ttk.Entry(self, textvariable=var, width=width)
        self.unit_label = ttk.Label(self, text=unit or "")
        self.label.grid(row=0, column=0, sticky="w")
        self.entry.grid(row=0, column=1, padx=(4,4))
        self.unit_label.grid(row=0, column=2, sticky="w")
        if tooltip:
            create_tooltip(self.label, tooltip)
            create_tooltip(self.entry, tooltip)
    def mark_error(self, is_err):
        self.entry.configure(foreground=("red" if is_err else "black"))

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("四旋翼巡航/续航/电流等估算")
        self.geometry("1280x720"); self.minsize(1024, 640)
        try: self.tk.call("tk", "scaling", 1.2)
        except Exception: pass
        self.cfg_vars = {}
        self._build_layout()
        self._load_from_config(DEFAULT_CONFIG)

    def _build_layout(self):
        toolbar = ttk.Frame(self); toolbar.pack(side=tk.TOP, fill=tk.X)
        ttk.Button(toolbar, text="载入预设 JSON", command=self.on_load_json).pack(side=tk.LEFT, padx=4, pady=4)
        ttk.Button(toolbar, text="保存当前预设", command=self.on_save_json).pack(side=tk.LEFT, padx=4, pady=4)
        ttk.Separator(toolbar, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=6)
        ttk.Button(toolbar, text="运行计算 (Ctrl+R)", command=self.on_run).pack(side=tk.LEFT, padx=4, pady=4)
        ttk.Button(toolbar, text="清空输出", command=self.on_clear_output).pack(side=tk.LEFT, padx=4, pady=4)
        ttk.Button(toolbar, text="保存报告 TXT", command=self.on_save_report).pack(side=tk.LEFT, padx=4, pady=4)
        ttk.Separator(toolbar, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=6)
        ttk.Button(toolbar, text="恢复示例参数", command=lambda: self._load_from_config(DEFAULT_CONFIG)).pack(side=tk.LEFT, padx=4, pady=4)

        main = ttk.Panedwindow(self, orient=tk.HORIZONTAL); main.pack(fill=tk.BOTH, expand=True)
        left = ttk.Frame(main); main.add(left, weight=1)
        canvas = tk.Canvas(left, highlightthickness=0)
        vsb = ttk.Scrollbar(left, orient="vertical", command=canvas.yview)
        self.param_frame = ttk.Frame(canvas)
        self.param_frame.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=self.param_frame, anchor="nw")
        canvas.configure(yscrollcommand=vsb.set)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        vsb.pack(side=tk.RIGHT, fill=tk.Y)

        right = ttk.Frame(main); main.add(right, weight=2)
        self.text = tk.Text(right, wrap=tk.NONE, font=("Consolas", 11))
        self.text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        ysb = ttk.Scrollbar(right, orient=tk.VERTICAL, command=self.text.yview)
        xsb = ttk.Scrollbar(right, orient=tk.HORIZONTAL, command=self.text.xview)
        self.text.configure(yscrollcommand=ysb.set, xscrollcommand=xsb.set)
        ysb.pack(side=tk.RIGHT, fill=tk.Y); xsb.pack(side=tk.BOTTOM, fill=tk.X)
        self.bind_all("<Control-r>", lambda e: self.on_run())
        self._build_param_inputs(self.param_frame)

    def _build_param_inputs(self, root):
        r = 0
        def add(label, key, default, unit=None, tip=None):
            nonlocal r
            var = tk.StringVar(value=str(default) if default is not None else "")
            self.cfg_vars[key] = var
            w = LabeledEntry(root, label, var, unit=unit, tooltip=tip)
            w.grid(row=r, column=0, sticky="w", padx=8, pady=4)
            r += 1

        ttk.Label(root, text="— 质量与环境 —", font=("Arial", 11, "bold")).grid(row=r, column=0, sticky="w", padx=6, pady=(8,2)); r+=1
        add("整机起飞质量 mass_kg", 'mass_kg', DEFAULT_CONFIG.mass_kg, "kg")
        add("海拔 altitude_m", 'altitude_m', DEFAULT_CONFIG.altitude_m, "m")
        add("环境温度 ambient_temp_C", 'ambient_temp_C', DEFAULT_CONFIG.ambient_temp_C, "°C")

        ttk.Label(root, text="— 螺旋桨/电机/几何 —", font=("Arial", 11, "bold")).grid(row=r, column=0, sticky="w", padx=6, pady=(8,2)); r+=1
        add("桨径 prop_diameter_in", 'prop_diameter_in', DEFAULT_CONFIG.prop_diameter_in, "英寸(in)")
        add("桨距 prop_pitch_in", 'prop_pitch_in', DEFAULT_CONFIG.prop_pitch_in, "英寸(in)")
        add("电机 KV motor_kv", 'motor_kv', DEFAULT_CONFIG.motor_kv, "rpm/V")
        add("电机数量 motor_count", 'motor_count', DEFAULT_CONFIG.motor_count)
        add("桨叶数 blade_count(2/3)", 'blade_count', DEFAULT_CONFIG.blade_count)

        ttk.Label(root, text="— 电池 —", font=("Arial", 11, "bold")).grid(row=r, column=0, sticky="w", padx=6, pady=(8,2)); r+=1
        add("S 数 battery_cells_S", 'battery_cells_S', DEFAULT_CONFIG.battery_cells_S, "S")
        add("单节满电 V_full", 'battery_voltage_full_per_cell', DEFAULT_CONFIG.battery_voltage_full_per_cell, "V")
        add("单节名义 V_nom", 'battery_voltage_nom_per_cell', DEFAULT_CONFIG.battery_voltage_nom_per_cell, "V")
        add("单节最低 V_min", 'battery_voltage_min_per_cell', DEFAULT_CONFIG.battery_voltage_min_per_cell, "V")
        add("容量 Ah（或留空）", 'battery_capacity_Ah', DEFAULT_CONFIG.battery_capacity_Ah, "Ah")
        add("总能量 Wh（或留空）", 'battery_energy_Wh', DEFAULT_CONFIG.battery_energy_Wh, "Wh")
        add("可用比例 usable_fraction", 'battery_usable_fraction', DEFAULT_CONFIG.battery_usable_fraction)
        add("最大持续倍率 battery_max_C（可空）", 'battery_max_C', DEFAULT_CONFIG.battery_max_C, tip="连续 C 值；若留空将尝试使用 battery_max_current_A")
        add("最大持续电流 battery_max_current_A（A，可空）", 'battery_max_current_A', DEFAULT_CONFIG.battery_max_current_A, "A", tip="直接给 A 值；若与倍率同时给，将取二者较小")

        ttk.Label(root, text="— 效率/系数 —", font=("Arial", 11, "bold")).grid(row=r, column=0, sticky="w", padx=6, pady=(8,2)); r+=1
        add("悬停 FoM figure_of_merit_hover_base", 'figure_of_merit_hover_base', DEFAULT_CONFIG.figure_of_merit_hover_base)
        add("电机效率 motor_efficiency", 'motor_efficiency', DEFAULT_CONFIG.motor_efficiency)
        add("电调效率 esc_efficiency", 'esc_efficiency', DEFAULT_CONFIG.esc_efficiency)
        add("负载转速系数 loaded_rpm_fraction", 'loaded_rpm_fraction_of_kv', DEFAULT_CONFIG.loaded_rpm_fraction_of_kv)
        add("Ct0 基准 prop_Ct0_base", 'prop_Ct0_base', DEFAULT_CONFIG.prop_Ct0_base)
        add("Cp0 基准 prop_Cp0_base", 'prop_Cp0_base', DEFAULT_CONFIG.prop_Cp0_base)
        add("三叶 Ct 乘子", 'three_blade_Ct_multiplier', DEFAULT_CONFIG.three_blade_Ct_multiplier)
        add("三叶 Cp 乘子", 'three_blade_Cp_multiplier', DEFAULT_CONFIG.three_blade_Cp_multiplier)
        add("三叶 FoM Δ", 'three_blade_FoM_delta', DEFAULT_CONFIG.three_blade_FoM_delta)

        ttk.Label(root, text="— 前进比经验 & 巡航模式 —", font=("Arial", 11, "bold")).grid(row=r, column=0, sticky="w", padx=6, pady=(8,2)); r+=1
        add("k_ct", 'k_ct', DEFAULT_CONFIG.k_ct)
        add("k_cp", 'k_cp', DEFAULT_CONFIG.k_cp)
        add("滑差 slip_fraction", 'slip_fraction', DEFAULT_CONFIG.slip_fraction)
        ttk.Label(root, text="巡航模式 mode").grid(row=r, column=0, sticky="w", padx=8)
        mode_var = tk.StringVar(value=DEFAULT_CONFIG.mode)
        self.cfg_vars['mode'] = mode_var
        mode_cb = ttk.Combobox(root, textvariable=mode_var, values=["drag_match", "throttle"], state="readonly", width=14)
        mode_cb.grid(row=r, column=0, sticky="e", padx=8); r += 1
        add("throttle 模式油门", 'cruise_throttle_frac', DEFAULT_CONFIG.cruise_throttle_frac)
        add("CdA_m2（阻力×面积）", 'CdA_m2', DEFAULT_CONFIG.CdA_m2, "m²")
        add("J_cap_for_raw_search（原始极限搜索上限）", 'J_cap_for_raw_search', DEFAULT_CONFIG.J_cap_for_raw_search, tip="用于推力/电流“原始极限”求解的 J 上界，默认 1.4")

        ttk.Label(root, text="— 安装效应修正（可选） —", font=("Arial", 11, "bold")).grid(row=r, column=0, sticky="w", padx=6, pady=(8,2)); r+=1
        add("install_Ct_multiplier", 'install_Ct_multiplier', DEFAULT_CONFIG.install_Ct_multiplier)
        add("install_Cp_multiplier", 'install_Cp_multiplier', DEFAULT_CONFIG.install_Cp_multiplier)
        ttk.Label(root, text="曲线修正：J 列表（如 0,0.2,0.4）").grid(row=r, column=0, sticky="w", padx=8); r+=1
        self.cfg_vars['install_kT_curve_J'] = tk.StringVar(value="")
        self.cfg_vars['install_kP_curve_J'] = tk.StringVar(value="")
        ttk.Entry(root, textvariable=self.cfg_vars['install_kT_curve_J'], width=40).grid(row=r, column=0, sticky="w", padx=8, pady=2); r+=1
        ttk.Label(root, text="k_T(J) 值列表（如 0.8,0.85,0.9）").grid(row=r, column=0, sticky="w", padx=8); r+=1
        self.cfg_vars['install_kT_curve_val'] = tk.StringVar(value="")
        ttk.Entry(root, textvariable=self.cfg_vars['install_kT_curve_val'], width=40).grid(row=r, column=0, sticky="w", padx=8, pady=2); r+=1
        ttk.Label(root, text="k_P(J) 值列表（如 0.9,0.95,1.0）").grid(row=r, column=0, sticky="w", padx=8); r+=1
        self.cfg_vars['install_kP_curve_val'] = tk.StringVar(value="")
        ttk.Entry(root, textvariable=self.cfg_vars['install_kP_curve_val'], width=40).grid(row=r, column=0, sticky="w", padx=8, pady=2); r+=1
        ttk.Label(root, text="（注：k_P 使用同一 J 列表；为空则不启用曲线修正）").grid(row=r, column=0, sticky="w", padx=8, pady=(0,8)); r+=1

    # ---------- 事件 ----------
    def on_clear_output(self):
        self.text.delete("1.0", tk.END)

    def on_save_report(self):
        content = self.text.get("1.0", tk.END)
        if not content.strip():
            messagebox.showinfo("保存报告", "没有可保存的内容。请先运行计算。")
            return
        fp = filedialog.asksaveasfilename(defaultextension=".txt", filetypes=[("Text", "*.txt"), ("All", "*.*")])
        if fp:
            with open(fp, "w", encoding="utf-8") as f:
                f.write(content)
            messagebox.showinfo("保存报告", f"已保存到\n{fp}")

    def on_load_json(self):
        fp = filedialog.askopenfilename(filetypes=[("JSON", "*.json"), ("All", "*.*")])
        if not fp: return
        try:
            with open(fp, "r", encoding="utf-8") as f:
                data = json.load(f)
            cfg = DEFAULT_CONFIG.__class__(**{**asdict(DEFAULT_CONFIG), **data})
            self._load_from_config(cfg)
        except Exception as e:
            messagebox.showerror("载入失败", f"读取 JSON 出错：\n{e}")

    def on_save_json(self):
        try:
            cfg = self._gather_config()
        except ValueError as e:
            messagebox.showerror("保存失败", str(e)); return
        fp = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON", "*.json"), ("All", "*.*")])
        if not fp: return
        with open(fp, "w", encoding="utf-8") as f:
            json.dump(asdict(cfg), f, ensure_ascii=False, indent=2)
        messagebox.showinfo("保存预设", f"已保存到\n{fp}")

    def on_run(self):
        for child in self.param_frame.winfo_children():
            if isinstance(child, LabeledEntry): child.mark_error(False)
        try:
            cfg = self._gather_config()
        except ValueError as e:
            messagebox.showerror("输入有误", str(e)); return
        try:
            report = run_report(cfg)
            self.text.delete("1.0", tk.END)
            self.text.insert(tk.END, report)
        except Exception as e:
            messagebox.showerror("运行失败", f"计算过程中发生错误：\n{e}")

    # ---------- 辅助 ----------
    def _load_from_config(self, cfg: Config):
        for k, v in asdict(cfg).items():
            if k in ('install_kT_curve_J','install_kT_curve_val','install_kP_curve_J','install_kP_curve_val'):
                continue
            var = self.cfg_vars.get(k)
            if var is None:
                if k == 'mode': self.cfg_vars['mode'].set(v)
                continue
            var.set("" if v is None else str(v))
        self.cfg_vars['install_kT_curve_J'].set("")
        self.cfg_vars['install_kT_curve_val'].set("")
        self.cfg_vars['install_kP_curve_val'].set("")

    def _parse_list(self, s: str) -> Optional[List[float]]:
        s = (s or "").strip()
        if not s: return None
        arr = []
        for x in s.split(','):
            x = x.strip()
            if x: arr.append(float(x))
        return arr

    def _maybe_float(self, s: str):
        s = (s or "").strip()
        if s == "" or s.lower() == "none": return None
        return float(s)

    def _getf_with_default(self, key, allow_none=False):
        var = self.cfg_vars.get(key)
        if var is None:
            return getattr(DEFAULT_CONFIG, key)
        s = var.get().strip()
        if allow_none and (s == "" or s.lower() == "none"):
            return None
        try:
            return float(s) if ('.' in s or 'e' in s.lower()) else float(s)
        except Exception:
            raise ValueError(f"参数 {key} 需为数字，当前输入：{s!r}")

    def _gather_config(self) -> Config:
        def getf(key, allow_none=False):
            return self._getf_with_default(key, allow_none=allow_none)
        def geti(key):
            var = self.cfg_vars.get(key)
            if var is None:
                return int(getattr(DEFAULT_CONFIG, key))
            s = var.get().strip()
            try:
                return int(float(s))
            except Exception:
                raise ValueError(f"参数 {key} 需为整数，当前输入：{s!r}")
        def getfrac(key):
            v = float(self.cfg_vars[key].get().strip()) if key in self.cfg_vars else float(getattr(DEFAULT_CONFIG, key))
            if not (0.0 < v <= 1.0): 
                raise ValueError(f"参数 {key} 应在 (0,1] 区间")
            return v

        cfg = Config(
            mass_kg=getf('mass_kg'),
            altitude_m=getf('altitude_m'),
            ambient_temp_C=getf('ambient_temp_C'),
            prop_diameter_in=getf('prop_diameter_in'),
            prop_pitch_in=getf('prop_pitch_in'),
            motor_kv=getf('motor_kv'),
            motor_count=geti('motor_count'),
            blade_count=geti('blade_count'),
            battery_cells_S=geti('battery_cells_S'),
            battery_voltage_full_per_cell=getf('battery_voltage_full_per_cell'),
            battery_voltage_nom_per_cell=getf('battery_voltage_nom_per_cell'),
            battery_voltage_min_per_cell=getf('battery_voltage_min_per_cell'),
            battery_capacity_Ah=self._maybe_float(self.cfg_vars['battery_capacity_Ah'].get()) if 'battery_capacity_Ah' in self.cfg_vars else DEFAULT_CONFIG.battery_capacity_Ah,
            battery_energy_Wh=self._maybe_float(self.cfg_vars['battery_energy_Wh'].get()) if 'battery_energy_Wh' in self.cfg_vars else DEFAULT_CONFIG.battery_energy_Wh,
            battery_usable_fraction=getfrac('battery_usable_fraction'),
            battery_max_C=self._maybe_float(self.cfg_vars['battery_max_C'].get()) if 'battery_max_C' in self.cfg_vars else DEFAULT_CONFIG.battery_max_C,
            battery_max_current_A=self._maybe_float(self.cfg_vars['battery_max_current_A'].get()) if 'battery_max_current_A' in self.cfg_vars else DEFAULT_CONFIG.battery_max_current_A,
            figure_of_merit_hover_base=getf('figure_of_merit_hover_base'),
            motor_efficiency=getf('motor_efficiency'),
            esc_efficiency=getf('esc_efficiency'),
            loaded_rpm_fraction_of_kv=getf('loaded_rpm_fraction_of_kv'),
            prop_Ct0_base=getf('prop_Ct0_base'),
            prop_Cp0_base=getf('prop_Cp0_base'),
            three_blade_Ct_multiplier=getf('three_blade_Ct_multiplier'),
            three_blade_Cp_multiplier=getf('three_blade_Cp_multiplier'),
            three_blade_FoM_delta=getf('three_blade_FoM_delta'),
            k_ct=getf('k_ct'),
            k_cp=getf('k_cp'),
            slip_fraction=getf('slip_fraction'),
            mode=self.cfg_vars['mode'].get() if 'mode' in self.cfg_vars else DEFAULT_CONFIG.mode,
            cruise_throttle_frac=getf('cruise_throttle_frac'),
            CdA_m2=getf('CdA_m2'),
            install_Ct_multiplier=getf('install_Ct_multiplier'),
            install_Cp_multiplier=getf('install_Cp_multiplier'),
            install_kT_curve_J=self._parse_list(self.cfg_vars['install_kT_curve_J'].get()) if 'install_kT_curve_J' in self.cfg_vars else None,
            install_kT_curve_val=self._parse_list(self.cfg_vars['install_kT_curve_val'].get()) if 'install_kT_curve_val' in self.cfg_vars else None,
            install_kP_curve_J=self._parse_list(self.cfg_vars['install_kT_curve_J'].get()) if 'install_kT_curve_J' in self.cfg_vars and self._parse_list(self.cfg_vars['install_kT_curve_J'].get()) else None,
            install_kP_curve_val=self._parse_list(self.cfg_vars['install_kP_curve_val'].get()) if 'install_kP_curve_val' in self.cfg_vars else None,
            J_cap_for_raw_search=getf('J_cap_for_raw_search'),
        )
        if cfg.battery_energy_Wh is None and cfg.battery_capacity_Ah is None:
            raise ValueError("电池容量（Ah）与总能量（Wh）至少填一个；若两者都填，将优先使用 Wh。")
        return cfg

# ---------- 简易 Tooltip ----------
class ToolTip:
    def __init__(self, widget, text):
        self.widget = widget; self.text = text; self.tipwindow = None
        widget.bind("<Enter>", self.show); widget.bind("<Leave>", self.hide)
    def show(self, event=None):
        if self.tipwindow or not self.text: return
        x = self.widget.winfo_rootx() + 20; y = self.widget.winfo_rooty() + self.widget.winfo_height() + 1
        self.tipwindow = tw = tk.Toplevel(self.widget)
        tw.wm_overrideredirect(True); tw.wm_geometry(f"+{x}+{y}")
        label = tk.Label(tw, text=self.text, justify=tk.LEFT, relief=tk.SOLID, borderwidth=1, font=("Arial", 9), background="#ffffe0")
        label.pack(ipadx=4, ipady=2)
    def hide(self, event=None):
        if self.tipwindow:
            self.tipwindow.destroy(); self.tipwindow = None

def create_tooltip(widget, text):
    return ToolTip(widget, text)

# =========================
# 入口
# =========================
if __name__ == "__main__":
    try:
        app = App(); app.mainloop()
    except tk.TclError:
        sys.stderr.write("\n[错误] 无法初始化图形界面（可能缺少 python3-tk）。\n")
        sys.stderr.write("请安装： sudo apt-get install -y python3-tk \n\n")
        sys.stderr.write("改用命令行生成一次示例报告：\n\n")
        print(run_report(DEFAULT_CONFIG))
