import math
def get_dist(A, B):
    return math.sqrt((A[0] - B[0])**2 + ( A[1] - B[1] )**2)

def rotate_point_at_pivot(P, pv, theta):
    theta = theta * math.pi / 180
    x = ( (P[0] - pv[0]) * math.cos(theta) - (P[1] - pv[1]) * math.sin(theta) ) + pv[0]
    y = ( (P[0] - pv[0]) * math.sin(theta) + (P[1] - pv[1]) * math.cos(theta) ) + pv[1]
    return (x, y)

def get_line_intersection(A1, A2, B1, B2):
    l1 = get_line_equation(A1, A2)
    l2 = get_line_equation(B1, B2)
    x = 0
    # y = l1.a * x + l1.b = l2.a * x + l2.b
    if( l1[0] != l2[0] ):
        x = ( l2[1] - l1[1] ) / ( l1[0] - l2[0] )
    y = l1[0] * x + l1[1]
    return (x, y)

def get_line_equation(A, B):
    # A.y = a * A.x + b
    # B.y = b * B.x + b
    a = 0
    if( A[0] != B[0] ):
        a = (A[1] - B[1]) / (A[0] - B[0])
    b = A[1] - A[0] * a
    return (a, b)

