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
    distributions = []
    # --->>> Put your code here.
    for i in xrange(a.start(),a.stop()):

        vals = [a.value(i)*bvalue for bvalue in b.values]
              
        dis = Distribution(i+b.offset, vals)
        distributions.append(dis)
            
    return a.sum(distributions)# Replace this by your own result.
# --->>> Copy your convolve(a, b) and multiply(a, b) functions here.
def multiply(a, b):
    """Multiply two distributions and return the resulting distribution."""

    # --->>> Put your code here.
    pX=a
    pzGivenX=b
    vals=[]
    refvals=[]
 
    for i in xrange(0,999):
        x=pX.value(i)*pzGivenX.value(i)
        refvals.append(x)
        if x>0:
            vals.append(x)
   
    offset=0
    for i in xrange(len(refvals)):

        if refvals[i]!=0:
            offset = i
            break
            
    dis = Distribution(offset,vals)
    dis.normalize()
    
    return dis  # Modify this to return your result.




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
    for i in xrange(len(controls)):
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
