import tkinter as tk
from tkinter import ttk, messagebox, simpledialog
import numpy as np
import matplotlib
matplotlib.use('TkAgg') #عرض الرسوم ضمن  نافذة الtk للتواصل
import matplotlib.pyplot as plt #استيراد واجهة الرسم
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg #دمج شبكة الاحداثيات

from kinematics import Kinematics, check_joint_limits
from arduino_serial import ArduinoComm

WORKSPACE = {'x': (-40.0, 40.0), 'y': (0.0, 80.0)}

# Default servo mapping: direction (1 or -1) and offset (deg)
SERVO_DIRECTION = [1, 1, 1, 1]
SERVO_OFFSET = [10.0, 0.0, 4.0, 10.0]

DH_EXAMPLES = {
    2: [[4,0,0,0],[8,0,0,0]],
    3: [[4,0,0,0],[8,0,0,0],[8,0,0,0]],
    4: [[4,0,0,0],[8,0,0,0],[8,0,0,0],[8,0,0,0]]
}
#
class App(tk.Tk):
    def __init__(self): # استدعي الكونستراكتر لهيء النافذة وعين القيم اللازمة
        super().__init__() 
        self.title('Robotic Arm — Tuned')
        self.geometry('1000x700')
        self.dof_var = tk.IntVar(value=3)
        self.serial_port_var = tk.StringVar(value='COM11')
        self.simulate_var = tk.BooleanVar(value=True)
        self.dh_entries = []
        self.current_kin = None
        self.current_angles = None
        self.create_widgets()
#انشاء ويدجت
    def create_widgets(self):
        frm_left = ttk.Frame(self)
        frm_left.pack(side='left', fill='y', padx=8, pady=8)

        ttk.Label(frm_left, text='Degrees of Freedom (2-4):').pack()
        spin = ttk.Spinbox(frm_left, from_=2, to=4, textvariable=self.dof_var, width=5)
        spin.pack()

        ttk.Button(frm_left, text='Generate DH Table', command=self.generate_dh_table).pack(pady=4)
        self.table_frame = ttk.Frame(frm_left)
        self.table_frame.pack()
        ttk.Button(frm_left, text='Save DH Table', command=self.save_dh).pack(pady=4)

        ttk.Separator(frm_left, orient='horizontal').pack(fill='x', pady=6)

        ttk.Label(frm_left, text='Servo mapping (direction & offset deg):').pack()
        self.dir_entries = []
        self.off_entries = []
        for i in range(4):
            f = ttk.Frame(frm_left)
            f.pack(anchor='w')
            ttk.Label(f, text=f'Joint {i+1}').pack(side='left')
            d = ttk.Entry(f, width=4)
            d.insert(0, str(SERVO_DIRECTION[i]))
            d.pack(side='left', padx=4)
            o = ttk.Entry(f, width=6)
            o.insert(0, str(SERVO_OFFSET[i]))
            o.pack(side='left', padx=4)
            self.dir_entries.append(d)
            self.off_entries.append(o)

        ttk.Button(frm_left, text='Apply Mapping', command=self.apply_mapping).pack(pady=4)
        ttk.Button(frm_left, text='Send Test Angles', command=self.send_test_angles).pack(pady=4)

        ttk.Separator(frm_left, orient='horizontal').pack(fill='x', pady=6)

        ttk.Button(frm_left, text='Forward Kinematics', command=self.forward_kin).pack(pady=2)
        ttk.Button(frm_left, text='Inverse Kinematics', command=self.inverse_kin).pack(pady=2)

        ttk.Checkbutton(frm_left, text='Simulation Mode (no Arduino)', variable=self.simulate_var).pack()
        ttk.Label(frm_left, text='Serial Port:').pack()
        ttk.Entry(frm_left, textvariable=self.serial_port_var).pack()

        #ttk.Button(frm_left, text='Record Real Endpoint', command=self.record_real_endpoint).pack(pady=4)

        frm_right = ttk.Frame(self)
        frm_right.pack(side='right', fill='both', expand=True)

        self.fig, self.ax = plt.subplots(figsize=(6,6))
        self.ax.set_aspect('equal')
        self.canvas = FigureCanvasTkAgg(self.fig, master=frm_right)
        self.canvas.get_tk_widget().pack(fill='both', expand=True)

        self.generate_dh_table()
        self.draw_workspace()

    def apply_mapping(self):
        global SERVO_DIRECTION, SERVO_OFFSET
        for i in range(4):
            try:
                SERVO_DIRECTION[i] = int(self.dir_entries[i].get())
            except:
                SERVO_DIRECTION[i] = 1
            try:
                SERVO_OFFSET[i] = float(self.off_entries[i].get())
            except:
                SERVO_OFFSET[i] = 0.0
        messagebox.showinfo('Mapping', f'Applied directions: {SERVO_DIRECTION}\n offsets: {SERVO_OFFSET}')

    def send_test_angles(self):
        # simple test pattern: [0,30,0,0]
        if self.current_kin is None:
            messagebox.showerror('Error','Save DH table first')
            return
        test = [0.0]*self.current_kin.n
        if self.current_kin.n>=2:
            test[1]=30.0
        self._send_angles_deg(test)

    def generate_dh_table(self):
        for child in self.table_frame.winfo_children():
            child.destroy()
        self.dh_entries = []
        n = int(self.dof_var.get())
        headers = ['a', 'alpha(deg)', 'd', 'theta_offset(deg)']
        for j, h in enumerate(headers):
            ttk.Label(self.table_frame, text=h).grid(row=0, column=j)
        dh_pre = DH_EXAMPLES.get(n, [[10,0,0,0]]*n)
        for i in range(n):
            row_entries = []
            for j in range(4):
                e = ttk.Entry(self.table_frame, width=8)
                e.grid(row=i+1, column=j, padx=2, pady=2)
                e.insert(0, str(dh_pre[i][j]))
                row_entries.append(e)
            self.dh_entries.append(row_entries)

    def save_dh(self):
        try:
            dh = []
            for row in self.dh_entries:
                a = float(row[0].get())
                alpha = float(row[1].get())*np.pi/180.0
                d = float(row[2].get())
                theta_off = float(row[3].get())*np.pi/180.0
                dh.append([a, alpha, d, theta_off])
            self.current_kin = Kinematics(dh)
            messagebox.showinfo('Saved','DH table saved and kinematics initialized.')
            self.current_angles = np.zeros(self.current_kin.n)
            self.draw_robot(self.current_angles)
        except Exception as e:
            messagebox.showerror('Error', f'Failed to save DH table: {e}')

    def map_to_servo_angles(self, angles_deg):
        # angles_deg: list math angles in degrees (e.g. -90..90)
        servo_angles = []
        for i, ang in enumerate(angles_deg):
            dir_i = SERVO_DIRECTION[i] if i < len(SERVO_DIRECTION) else 1
            off_i = SERVO_OFFSET[i] if i < len(SERVO_OFFSET) else 0.0
            a = dir_i * ang + off_i
            # convert from math range (-90..90) to servo 0..180 by adding 90
            servo_val = float(np.clip(a + 90.0, 0.0, 180.0))
            servo_angles.append(servo_val)
        return servo_angles

    def _send_angles_deg(self, angles_deg):
        # print and send to Arduino after mapping
        print('[SEND] computed angles (deg):', angles_deg)
        servo_angles = self.map_to_servo_angles(angles_deg)
        print('[SEND] servo-mapped angles (deg):', servo_angles)
        simulate = self.simulate_var.get()
        port = None if simulate else self.serial_port_var.get()
        comm = ArduinoComm(port=port, simulate=simulate)
        try:
            success = comm.send_angles(servo_angles)
            if not success:
                messagebox.showerror('Serial Error','Failed to send angles to Arduino')
            else:
                messagebox.showinfo('Sent','Angles sent to Arduino (or simulated).')
        except Exception as e:
            messagebox.showerror('Serial Error', str(e))
        finally:
            comm.close()

    def forward_kin(self):
        if self.current_kin is None:
            messagebox.showerror('Error','Please save DH table first.')
            return
        ans = simpledialog.askstring('Forward', f'Enter {self.current_kin.n} joint angles (deg) separated by commas')
        if ans is None:
            return
        try:
            angles_deg = [float(x.strip()) for x in ans.split(',')]
            if len(angles_deg) != self.current_kin.n:
                raise ValueError('Incorrect number of angles')
            if not check_joint_limits(angles_deg):
                messagebox.showerror('Error','Joint angles out of limits')
                return
            angles_rad = np.array(angles_deg)*np.pi/180.0
        except Exception as e:
            messagebox.showerror('Error', f'Invalid input: {e}')
            return
        ee = self.current_kin.end_effector(angles_rad)
        if not self.current_kin.clamp_workspace(ee, WORKSPACE):
            messagebox.showerror('Error', f'End-effector outside workspace: {ee}')
            return
        self.current_angles = angles_rad
        self.draw_robot(angles_rad)
        self._send_angles_deg(angles_deg)

    def inverse_kin(self):
        if self.current_kin is None:
            messagebox.showerror('Error','Please save DH table first.')
            return
        try:
            tx = float(simpledialog.askstring('Inverse','Enter Target X'))
            ty = float(simpledialog.askstring('Inverse','Enter Target Y'))
        except:
            return
        target = np.array([tx, ty])
        init = self.current_angles if self.current_angles is not None else np.zeros(self.current_kin.n)
        sol = self.current_kin.inverse_jacobian(target, initial_guess=init)
        if sol is None:
            messagebox.showerror('IK failed', 'Inverse kinematics failed to converge')
            return
        angles_rad = sol
        angles_deg = angles_rad*180.0/np.pi
        if not check_joint_limits(angles_deg):
            messagebox.showerror('Error','Joint angles out of limits')
            return
        ee = self.current_kin.end_effector(angles_rad)
        if not self.current_kin.clamp_workspace(ee, WORKSPACE):
            messagebox.showerror('Error','Target outside workspace')
            return
        self.current_angles = angles_rad
        self.draw_robot(angles_rad)
        self._send_angles_deg(angles_deg)

    def draw_workspace(self):
        self.ax.clear()
        self.ax.set_xlim(WORKSPACE['x'])
        self.ax.set_ylim(WORKSPACE['y'])
        self.ax.axvline(0, linewidth=0.5)
        self.ax.axhline(0, linewidth=0.5)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_title('Robot Visualization')
        self.canvas.draw()

    def draw_robot(self, angles_rad, real_point=None):
        self.ax.clear()
        self.ax.set_xlim(WORKSPACE['x'])
        self.ax.set_ylim(WORKSPACE['y'])
        positions, _ = self.current_kin.forward(angles_rad)
        xs = [p[0] for p in positions]
        ys = [p[1] for p in positions]
        self.ax.plot(xs, ys, marker='o')
        ex, ey = xs[-1], ys[-1]
        self.ax.plot(ex, ey, 'ro')
        if real_point is not None:
            self.ax.plot(real_point[0], real_point[1], 'gx', markersize=10)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.canvas.draw()

    def record_real_endpoint(self):
        # allow user to type measured real endpoint and compare
        try:
            x_real = float(simpledialog.askstring('Record','Enter measured X (cm)'))
            y_real = float(simpledialog.askstring('Record','Enter measured Y (cm)'))
        except:
            return
        if self.current_angles is None:
            messagebox.showerror('Error','No current pose to compare')
            return
        sim_xy = self.current_kin.end_effector(self.current_angles)
        self.draw_robot(self.current_angles, real_point=(x_real, y_real))
        dx = x_real - sim_xy[0]
        dy = y_real - sim_xy[1]
        messagebox.showinfo('Comparison', f'Simulated: {sim_xy}\nMeasured: ({x_real},{y_real})\nDelta: (dx={dx:.2f}, dy={dy:.2f})')

    def reset_all(self):
        self.generate_dh_table()
        self.current_kin = None
        self.current_angles = None
        self.draw_workspace()

if __name__ == '__main__':
    app = App()
    app.mainloop()
