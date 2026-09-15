import math
def get_dist(A, B):
    return math.sqrt((A[0] - B[0])**2 + ( A[1] - B[1] )**2)

def rotate_point_at_pivot(P, pv, theta):
    theta = theta * math.pi / 180
    x = ( (P[0] - pv[0]) * math.cos(theta) - (P[1] - pv[1]) * math.sin(theta) ) + pv[0]
    y = ( (P[0] - pv[0]) * math.sin(theta) + (P[1] - pv[1]) * math.cos(theta) ) + pv[1]
    return (x, y)