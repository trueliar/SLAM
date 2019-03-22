# Instead of moving a distribution, move (and modify) it using a convolution.
# 06_b_convolve_distribution
# Claus Brenner, 26 NOV 2012
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


if __name__ == '__main__':
    arena = (0,1000)

    # Move 3 times by 20.
    moves = [20] * 50

    # Start with a known position: probability 1.0 at position 10.
    position = Distribution.unit_pulse(10)
    plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
         linestyle='steps')

    # Now move and plot.
    for m in moves:
        move_distribution = Distribution.triangle(m, 2)
        position = convolve(position, move_distribution)
        plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
             linestyle='steps')

    ylim(0.0, 1.1)
    show()
