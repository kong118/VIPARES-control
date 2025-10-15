class Plant1st:
    def __init__(self, tau=0.4, k=3.0, Ts=0.02, umin=0.0, umax=1.0, du_max=0.05):
        self.Ts = Ts # Sample time
        self.tau = tau # System time constant
        self.k = k # System gain
        self.umin = umin # <- Actuator saturation
        self.umax = umax # <-/ 
        self.du_max = du_max # Max rate of change
        self.v = 0.0 # Current plant output (speed)
        self.u_prev = 0.0 # Last applied control input
        self._update_ab()
    def _update_ab(self):
        import math
        self.a = math.exp(-self.Ts/self.tau)
        self.b = (1.0 - self.a)*self.k
    def reset(self, v0=0.0):
        self.v = v0; self.u_prev = 0.0; return [self.v]  # ← 統一回傳 state list
    def step(self, u_cmd):
        du = max(min(u_cmd - self.u_prev, self.du_max), -self.du_max)
        u = max(self.umin, min(self.umax, self.u_prev + du))
        v_next = self.a*self.v + self.b*u
        self.v, self.u_prev = max(0.0, v_next), u
        y = self.v
        return [self.v], y

class Plant2nd:
    def __init__(self, m=25., ku=30., c0=2., c1=1.2, c2=0.25, tau_a=0.1, Ts=0.02,
                 umin=0.0, umax=1.0, du_max=0.05, disturbance=lambda t:0.0):
        self.m, self.ku, self.c0, self.c1, self.c2 = m, ku, c0, c1, c2
        self.tau_a, self.Ts = tau_a, Ts
        self.umin, self.umax, self.du_max = umin, umax, du_max
        self.v, self.ua, self.u_prev, self.t = 0.0, 0.0, 0.0, 0.0
        self.disturb = disturbance
    def reset(self, v0=0.0):
        self.v, self.ua, self.u_prev, self.t = v0, 0.0, 0.0, 0.0
        return [self.v, self.ua]
    def step(self, u_cmd):
        Ts = self.Ts
        # rate limit + saturation on command
        du = max(min(u_cmd - self.u_prev, self.du_max), -self.du_max)
        u = max(self.umin, min(self.umax, self.u_prev + du))
        # actuator lag (Euler)
        dua = (u - self.ua)/self.tau_a
        ua_next = self.ua + Ts*dua
        # longitudinal dynamics
        drag = self.c0 + self.c1*self.v + self.c2*self.v*self.v
        dv = (self.ku*ua_next - drag)/self.m + self.disturb(self.t)
        v_next = max(0.0, self.v + Ts*dv)
        # update states, time
        self.v, self.ua, self.u_prev, self.t = v_next, ua_next, u, self.t + Ts
        y = self.v
        return [self.v, self.ua], y
        