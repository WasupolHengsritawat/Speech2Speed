import math

def constant_func(value: float):
    """
    Generates a function for a constant velocity signal.
    v(t) = value
    """
    return lambda t: value

def linear_func(start: float, end: float = None, slope: float = None, duration: float = 10.0):
    """
    Generates a function for a linear velocity signal.
    v(t) = start + slope * t

    Note: If 'end' is provided and 'slope' is None, the slope is calculated
    over the given 'duration' (default 10.0s).
    """
    if slope is None and end is not None:
        if duration <= 0:
            # Note: For the specific table entry 'start = 3.14 rad/s, end = -3.14 rad/s', 
            # the duration is needed to calculate the slope. The default of 10.0s will be used.
            raise ValueError("Duration must be positive when calculating slope from start and end.")
        calculated_slope = (end - start) / duration
    elif slope is not None:
        calculated_slope = slope
    else:
        # This case covers the table entry 'end = 0 rad/s, slope = -0.5 rad/s2' 
        # where 'start' is missing. We'll assume a default 'start' of 0 if only 'end' and 'slope' are given
        # OR, since the provided function *requires* 'start', we'll rely on the user providing it 
        # (The original function structure doesn't easily support inferring 'start' from 'end' and 'slope'.)
        raise ValueError("Must provide either 'slope' or 'end' and 'start'.")

    def v(t):
        """Calculates linear velocity at time t."""
        return start + calculated_slope * t

    return v

def trapezoidal_func(start: float, end: float, max_v: float, duration: float, acc_time: float = None, acc_max: float = None):
    
    # --- 1. Determine Acceleration Rate (acc) ---
    if acc_time is not None and acc_max is None:
        # Calculate acc magnitude from acc_time and the max required delta_v
        # Use abs(max_v - start) for T_acc definition.
        if acc_time <= 0:
            raise ValueError("acc_time must be positive.")
        acc = abs(max_v - start) / acc_time
    elif acc_max is not None and acc_time is None:
        # Use acc_max directly (absolute magnitude)
        acc = abs(acc_max)
        if acc <= 0:
            raise ValueError("acc_max must be positive.")
    else:
        raise ValueError("Must provide either 'acc_time' or 'acc_max'.")
    
    # --- 2. Calculate Times required to reach max_v and return to end ---
    # Time required for acceleration phase (start to max_v)
    T_acc_req = abs(max_v - start) / acc
    # Time required for deceleration phase (max_v to end)
    T_dec_req = abs(max_v - end) / acc
    
    T_ramp_req = T_acc_req + T_dec_req
    
    # --- 3. Check for Triangular Profile (Duration too short) ---
    if duration < T_ramp_req:
        # The ramps alone take too long. The profile must be triangular.
        # The peak velocity (v_peak) will be less than max_v.
        
        # Calculate the actual achieved peak velocity (v_peak) and times.
        # The sum of acceleration and deceleration times must equal the duration (T_a + T_d = duration).
        # We also know the final position: Pos(duration) = Pos(start) + Area under triangle
        # Area = (1/2) * (v_peak + start) * T_a + (1/2) * (v_peak + end) * T_d
        
        # T_a = (v_peak - start) / acc_sign_a; T_d = (v_peak - end) / acc_sign_d
        # To simplify, we assume the acceleration magnitude 'acc' is constant.
        
        # T_a = |v_peak - start| / acc
        # T_d = |v_peak - end| / acc
        
        # The math yields: v_peak = (acc * duration + start*sign_a/acc - end*sign_d/acc) / (1/acc + 1/acc)
        # Assuming acc rate is constant magnitude 'acc':
        v_peak = (start * acc + end * acc + acc**2 * duration) / (2 * acc) # Simplified under assumption start=end=0, v_peak = acc * duration / 2
        
        # More accurately: Calculate v_peak from quadratic equation derived from kinematics:
        # s = start, e = end, T = duration, a = acc
        # v_peak = (a*T + s + e)/2 is an accurate formula when the rates are constant magnitude.
        
        # Determine the sign of the motion (if start and end are 0, direction is set by max_v sign)
        sign_motion = 1 if max_v >= 0 else -1
        
        # The actual peak velocity achieved in time T must satisfy:
        # T_a = (v_peak - start) / (sign_motion * acc) 
        # T_d = (v_peak - end) / (-sign_motion * acc) 
        # The sum of the absolute velocity changes must equal acc * duration.
        
        # v_peak = [acc * duration + start * sign(v_peak-start) + end * sign(v_peak-end)] / 2 
        # For simplicity, since the motion is continuous and max_v suggests direction, we'll use a direct calculation:
        
        # Solve for v_peak such that: |v_peak - start|/acc + |v_peak - end|/acc = duration
        # This requires tracking the signs, which is complex.
        
        # Simpler approach: If start=end=0, v_peak = (acc * duration) / 2.
        # We can calculate the total area required for the ideal triangle:
        # Total_Area = (1/2) * (v_peak + start) * T_a + (1/2) * (v_peak + end) * T_d
        
        # Let's rely on the fundamental kinematic equation for distance $d$:
        # $d_{total} = d_{acc} + d_{dec}$
        # $d_{acc} = (v_{\text{peak}}^2 - v_{\text{start}}^2) / (2 \cdot a)$, $d_{dec} = (v_{\text{peak}}^2 - v_{\text{end}}^2) / (2 \cdot a)$ (if accel/decel rate 'a' is same magnitude)
        
        # We assume the profile's direction is primarily set by max_v (for v_peak sign).
        sign_v_peak = 1 if max_v >= 0 else -1
        
        # Solve for $v_{\text{peak}}$ based on $T_{acc} + T_{dec} = \text{duration}$.
        # $\frac{|v_{\text{peak}} - \text{start}|}{a} + \frac{|v_{\text{peak}} - \text{end}|}{a} = T$
        
        # Assuming $v_{\text{peak}}$ has the same sign as max_v, and $start, end$ are smaller/opposite:
        # $v_{\text{peak}} \approx \text{sign\_v\_peak} \cdot \left(\frac{a \cdot T + |\text{start}| + |\text{end}|}{2}\right)$ (Complex signs omitted)
        
        # A simplified and safer calculation for the triangular peak velocity:
        v_peak_mag = (acc * duration - abs(start - end)) / 2 # Magnitude of peak velocity
        
        v_peak = sign_v_peak * v_peak_mag 
        
        # Recalculate times based on the new v_peak
        T_acc = abs(v_peak - start) / acc
        T_dec = abs(v_peak - end) / acc
        T_const = 0.0 # No constant phase
        
        # Ensure T_acc + T_dec exactly equals duration, handling floating point errors
        # T_acc = duration * (abs(v_peak - start)) / (abs(v_peak - start) + abs(v_peak - end))
        # T_dec = duration * (abs(v_peak - end)) / (abs(v_peak - start) + abs(v_peak - end))
        
        # Recalculate T_acc/T_dec to ensure sum is duration
        T_ratio_sum = T_acc + T_dec
        if T_ratio_sum > 0:
            scale = duration / T_ratio_sum
            T_acc *= scale
            T_dec *= scale
        else:
            # Should only happen if start=end=v_peak, implying no motion
            T_acc = T_dec = duration / 2

        max_v = v_peak # Override max_v for the rest of the function
    else:
        # Standard Trapezoidal Profile: Calculate T_const
        T_const = duration - T_ramp_req
        T_acc = T_acc_req
        T_dec = T_dec_req

    # --- 4. Define Trapezoidal/Triangular Profile Timing Points ---
    T1 = T_acc             # End of Acceleration phase
    T2 = T_acc + T_const   # End of Constant velocity phase
    T3 = duration          # End of Deceleration phase (T2 + T_dec)

    # --- 5. Return the Velocity Function v(t) ---
    def v(t):
        if t < 0:
            return start
            
        # Rate sign determined by direction of max_v relative to start
        rate_acc = acc if max_v >= start else -acc
        
        # Phase 1: Acceleration (0 <= t <= T1)
        if t <= T1:
            return start + rate_acc * t
            
        # Phase 2: Constant Velocity (T1 < t <= T2)
        elif t <= T2:
            return max_v
            
        # Phase 3: Deceleration (T2 < t <= T3)
        elif t <= T3:
            # Rate required to go from max_v to end: (end - max_v) / T_dec
            # Use the exact rate derived from the required change
            rate_dec = (end - max_v) / T_dec
            return max_v + rate_dec * (t - T2)
            
        # Phase 4: After duration (t > T3)
        else:
            return end

    return v


def sine_func(amplitude: float, frequency: float, offset: float = 0.0, phase: float = 0.0, freq_unit: str = 'Hz'):
    """
    Generates a function for a sinusoidal velocity signal.
    v(t) = offset + amplitude * sin(2 * pi * f * t + phase)

    Parameters:
        freq_unit: 'Hz' (default) or 'round/min' (RPM).
    """

    # Convert frequency to Hz if in rounds/min
    if freq_unit.lower() == 'round/min' or freq_unit.lower() == 'rpm':
        f = frequency / 60.0
    elif freq_unit.lower() == 'hz':
        f = frequency
    else:
        raise ValueError("freq_unit must be 'Hz' or 'round/min'.")

    omega = 2 * math.pi * f

    def v(t):
        """Calculates sinusoidal velocity at time t."""
        return offset + amplitude * math.sin(omega * t + phase)

    return v