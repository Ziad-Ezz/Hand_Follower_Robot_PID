import math
import numpy as np

def calculate_ik_3dof(target, lengths):
    x, y, z = target
    L0, L1, L2 = lengths

    # 1. Base Angle (Yaw - Theta 1)
    theta1 = math.atan2(y, x)

    # 2. Distance from shoulder to target in the planar projection
    r = math.sqrt(x**2 + y**2)
    z_adj = z - L0
    D = math.sqrt(r**2 + z_adj**2)

    # Check reachability and clamp if the target is too far
    max_reach = L1 + L2 - 1e-5
    if D > max_reach:
        scale = max_reach / D
        r *= scale
        z_adj *= scale
        D = max_reach

    # 3. Elbow Angle (Pitch - Theta 3) using Law of Cosines
    cos_theta3 = (D**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta3 = max(-1.0, min(1.0, cos_theta3))
    theta3 = math.acos(cos_theta3)

    # 4. Shoulder Angle (Pitch - Theta 2)
    alpha = math.atan2(z_adj, r)
    cos_beta = (L1**2 + D**2 - L2**2) / (2 * L1 * D)
    cos_beta = max(-1.0, min(1.0, cos_beta))
    beta = math.acos(cos_beta)
    theta2 = alpha + beta

    # 5. Forward Kinematics (FK) for visualization
    p0 = np.array([0.0, 0.0, 0.0]) # Base
    p1 = np.array([0.0, 0.0, L0])  # Shoulder
    
    p2 = np.array([ # Elbow
        L1 * math.cos(theta2) * math.cos(theta1),
        L1 * math.cos(theta2) * math.sin(theta1),
        L0 + L1 * math.sin(theta2)
    ])
    
    p3 = np.array([ # Wrist (End Effector)
        p2[0] + L2 * math.cos(theta2 - theta3) * math.cos(theta1),
        p2[1] + L2 * math.cos(theta2 - theta3) * math.sin(theta1),
        p2[2] + L2 * math.sin(theta2 - theta3)
    ])

    joints = [p0, p1, p2, p3]
    angles_deg = [math.degrees(theta1), math.degrees(theta2), math.degrees(theta3)]

    return joints, angles_deg