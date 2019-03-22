# Compute the derivative of a scan.
# 03_b_scan_derivative
# Claus Brenner, 09 NOV 2012
from pylab import *
from lego_robot import *

# Find the derivative in scan data, ignoring invalid measurements.
def compute_derivative(scan, min_dist):
    jumps = [ 0 ]
    for i in range(1, len(scan) - 1):
        # --->>> Insert your code here.
        if scan[i+1] > min_dist and scan[i-1] > min_dist:
            jumps.append((scan[i+1] - scan[i-1])/2) # derivative of depth
        else:
            jumps.append(0) # append zero if erroneous scan values encountered

    jumps.append(0)
    return jumps


if __name__ == '__main__':

    minimum_valid_distance = 20.0

    # Read the logfile which contains all scans.
    logfile = LegoLogfile()
    logfile.read("robot4_scan.txt")

    # Pick one scan.
    scan_no = 8
    scan = logfile.scan_data[scan_no]

    # Compute derivative, (-1, 0, 1) mask.
    der = compute_derivative(scan, minimum_valid_distance)

    # Plot scan and derivative.
    title("Plot of scan %d" % scan_no)
    plot(scan)
    plot(der)
    show()
