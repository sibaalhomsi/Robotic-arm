import numpy as np

RAD = np.pi/180.0
DEG = 180.0/np.pi

# Joint Limits (degrees)
JOINT_LIMITS = [
    (-90, 90),
    (-90, 90),
    (-90, 90),
    (-90, 90)
]

def check_joint_limits(angles_deg):
    for i, angle in enumerate(angles_deg):
        if i >= len(JOINT_LIMITS):
            break
        low, high = JOINT_LIMITS[i]
        if angle < low or angle > high:
            return False
    return True
#حساب مصفوفة التحويل 
def dh_transform(a, alpha, d, theta):
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    # الخرج مصفوفة رباعية
    return np.array([
        [ct, -st*ca, st*sa, a*ct],
        [st, ct*ca, -ct*sa, a*st],
        [0, sa, ca, d],
        [0,0,0,1]
    ])

class Kinematics:
    def __init__(self, dh_table):
        self.dh = np.array(dh_table, dtype=float)
        self.n = self.dh.shape[0] # عدد الصفوف بكل جدول

    def forward(self, thetas_rad):
        T = np.eye(4)
        positions = [(0.0,0.0)]
        for i in range(self.n):
            a, alpha, d, theta_off = self.dh[i]
            theta = thetas_rad[i] + theta_off
            T = T @ dh_transform(a, alpha, d, theta)
            positions.append((T[0,3], T[1,3]))
        return positions, T
# دالة استخراج موقع النهاية فقط
    def end_effector(self, thetas_rad):
        _, T = self.forward(thetas_rad)
        return np.array([T[0,3], T[1,3]])

    def clamp_workspace(self, xy, limits):
        x, y = xy
        return limits['x'][0] <= x <= limits['x'][1] and limits['y'][0] <= y <= limits['y'][1]

    # Jacobian-based IK 
    def jacobian(self, thetas_rad, delta=1e-6):
        J = np.zeros((2, self.n))
        f0 = self.end_effector(thetas_rad)
        for i in range(self.n):
            dtheta = np.zeros(self.n)
            dtheta[i] = delta
            f1 = self.end_effector(thetas_rad + dtheta)
            J[:,i] = (f1 - f0) / delta
        return J

    def inverse_jacobian(self, target_xy, initial_guess=None, tol=1e-3, max_iter=200):
        if initial_guess is None:
            thetas = np.zeros(self.n)
        else:
            thetas = np.array(initial_guess, dtype=float)
        for _ in range(max_iter):
            ee = self.end_effector(thetas)
            error = target_xy - ee
            if np.linalg.norm(error) < tol:
                return thetas
            J = self.jacobian(thetas)
            try:
                dtheta = np.linalg.pinv(J) @ error
            except np.linalg.LinAlgError:
                return None
            thetas += dtheta
            # enforce joint limits in radians
            for i in range(self.n):
                low, high = JOINT_LIMITS[i]
                thetas[i] = np.clip(thetas[i], low*RAD, high*RAD)
        return thetas
