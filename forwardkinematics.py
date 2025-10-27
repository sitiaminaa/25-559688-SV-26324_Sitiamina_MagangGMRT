import math
import numpy as np
import matplotlib.pyplot as plt

def create_trans_matrix(theta_deg, length):
    theta_rad = math.radians(theta_deg)
    cos_t = np.cos(theta_rad)
    sin_t = np.sin(theta_rad)
    return np.array([
        [cos_t, -sin_t, length * cos_t],
        [sin_t,  cos_t, length * sin_t],
        [    0,      0,             1]
    ])
def forward_kinematics_matrix(L1, L2, theta1_deg, theta2_deg):
    T1 = create_trans_matrix(theta1_deg, L1)
    T2 = create_trans_matrix(theta2_deg, L2)
    T_final = np.dot(T1, T2)
    x1 = T1[0, 2]
    y1 = T1[1, 2]
    x2 = T_final[0, 2]
    y2 = T_final[1, 2]
    return (x1, y1), (x2, y2)
def inverse_kinematics(x_target, y_target, L1, L2):
    return theta1_deg, theta2_deg
def plot_arm(x1, y1, x2, y2, target_x=None, target_y=None, L1=0, L2=0):
    origin_x, origin_y = 0, 0
    total_length = L1 + L2 + 2
    plt.figure()
    plt.plot([origin_x, x1, x2], [origin_y, y1, y2], 'o-', color='blue', linewidth=3, markersize=8, label='Lengan Robot')
    plt.plot(origin_x, origin_y, 'o', color='red', markersize=10, label='Sendi 1 (Origin)')
    plt.plot(x1, y1, 'o', color='red', markersize=10, label='Sendi 2 (Siku)')
    plt.plot(x2, y2, 'o', color='green', markersize=10, label='End-Effector')
    if target_x is not None and target_y is not None:
        plt.plot(target_x, target_y, 'rx', markersize=15, label='Target (x,y)')
        plt.axis('equal')
    plt.xlim(-total_length, total_length)
    plt.ylim(-total_length, total_length)
    plt.grid()
    plt.legend()
    plt.title("Visualisasi 2-DoF Arm")
    plt.savefig("hasil.png")
    plt.show()

if __name__ == "__main__":
    L1 = 88.0   
    L2 = 24.0   
    theta1_deg = 40.0  
    theta2_deg = 30.0  
    print(f"Menghitung FK Study Case")
    print(f"Input:")
    print(f"L1 (femur)={L1}")
    print(f"L2 (tibia)={L2}")
    print(f"Theta 1={theta1_deg} derajat")
    print(f"Theta 2={theta2_deg} derajat")
    (siku_x, siku_y), (ujung_x, ujung_y) = forward_kinematics_matrix(
        L1, L2, theta1_deg, theta2_deg
    )
    print(f"\nOutput:")
    print(f"Koordinat Siku (Joint 2): (x={siku_x:.2f}, y={siku_y:.2f})")
    print(f"KOORDINAT TITIK AKHIR (End-Effector):(x={ujung_x:.2f}, y={ujung_y:.2f})")
    print("\nMenampilkan visualisasi...")
    plot_arm (siku_x, siku_y, ujung_x, ujung_y, None, None, L1, L2)
    
