# Histogram implementation of a bayes filter - combines
# convolution and multiplication of distributions, for the
# movement and measurement steps.
# 06_d_histogram_filter
# Claus Brenner, 28 NOV 2012
from pylab import plot, show, ylim
from distribution import *

def move(distribution, delta):
    """Returns a Distribution that has been moved (x-axis) by the amount of
       delta."""
    return Distribution(distribution.offset + delta, distribution.values)

def convolve(a, b):
    """Convolve distribution a and b and return the resulting new distribution."""
    # print(a)
    # print(b)
    # --->>> Put your code here.
    dist_list = []
    offset = a.offset + b.offset     # this command works to displace the distribution by the given 'move'
    for a_value in a.values:
        chotu_dist = []
        for b_value in b.values:
            chotu_dist.append(a_value*b_value)
        dist_list.append(Distribution(offset, chotu_dist))
        offset += 1    # offset for the next chotu_dist
    a_conv = Distribution.sum(dist_list)
    return a_conv

def multiply(a, b):
    """Multiply two distributions and return the resulting distribution."""

    # --->>> Put your code here.
    start = min(a.start(), b.start())
    stop = max(a.stop(), b.stop())
    mult_values = []
    for i in range(start, stop):
        mult_values.append(a.value(i)*b.value(i))
    mult_dist = Distribution(start, mult_values)
    mult_dist.normalize()
    return mult_dist



if __name__ == '__main__':
    arena = (0,220)

    # Start position. Exactly known - a unit pulse.
    start_position = 10
    position = Distribution.unit_pulse(start_position)
    plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
         linestyle='steps')

    # Movement data.
    controls  =    [ 20 ] * 10

    # Measurement data. Assume (for now) that the measurement data
    # is correct. - This code just builds a cumulative list of the controls,
    # plus the start position.
    p = start_position
    measurements = []
    for c in controls:
        p += c
        measurements.append(p)

    # This is the filter loop.
    for i in range(len(controls)):
        # Move, by convolution. Also termed "prediction".
        control = Distribution.triangle(controls[i], 10)
        position = convolve(position, control)
        plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
             color='b', linestyle='steps')

        # Measure, by multiplication. Also termed "correction".
        measurement = Distribution.triangle(measurements[i], 10)
        position = multiply(position, measurement)
        plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
             color='r', linestyle='steps')

    show()
