from numpy import sqrt, sin, cos

sqrt_d = {}
sqrt_d[2/3] = sqrt(2/3)
sqrt_d[2] = sqrt(2)
sqrt_d[3] = sqrt(3)
sqrt_d[6] = sqrt(6)


def dq0(a, b, c, theta):
    x, y, z = clarke(a, b, c)
    return park(x, y, theta)


def dq0_inverse(d, q, theta):
    x, y = park_inverse(d, q, theta)
    return clarke_inverse(x, y, 0)


def clarke(a, b, c):
    x = (2*a - b - c)*(1/sqrt_d[6])
    y = (b - c)*(1/sqrt_d[2])
    z = (a + b + c)*(1/sqrt_d[3])

    return x, y, z


def clarke_inverse(x, y, z):
    a = (1/sqrt_d[3])*z
    b = a - (1/sqrt_d[6])*x
    c = b - (1/sqrt_d[2])*y
    b += (1/sqrt_d[2])*y
    a += (sqrt_d[2/3])*x

    return a, b, c


def clarke_pv(a, b, c):
    x = (2*a - b - c)*(1/3)
    y = (b - c)*(1/sqrt_d[3])
    z = (a + b + c)*(sqrt_d[2]/3)

    return x, y, z


def clarke_pv_inverse(x, y, z):
    a = (1/sqrt_d[2])*z
    b = a - (1/2)*x
    c = b - (sqrt_d[3]/2)*y
    b += (sqrt_d[3]/2)*y
    a += x

    return a, b, c


def park(x, y, theta):
    co = cos(theta)
    si = sin(theta)

    d = co*x + si*y
    q = co*y - si*x

    return d, q


def park_inverse(d, q, theta):
    co = cos(theta)
    si = sin(theta)

    x = co*d - si*q
    y = si*d + co*q

    return x, y
