from math import sin, cos, pi
from numpy import matrix, array, identity
from control.matlab import *
from nose.tools import assert_equal, ok_, assert_almost_equal, assert_in

# lqr tests
def test_lqr(fn):
	# compute gain vector k
	

# constrain tests
def test_constrain(fn):
	assert_almost_equal(fn(0), 0.0)
	assert_almost_equal(fn(pi/4), 0.7853981633974483)
	assert_almost_equal(fn(pi/2), 1.5707963267948966)
	assert_almost_equal(fn(3*pi/4), 2.356194490192345)
	assert_almost_equal(fn(pi), 3.141592653589793)
	assert_almost_equal(fn(5*pi/4), -2.356194490192345)
	assert_almost_equal(fn(3*pi/2), -1.5707963267948966)
	assert_almost_equal(fn(7*pi/4), -0.7853981633974483)
	assert_almost_equal(fn(2*pi), 0.0)
	# Test Negative
	assert_almost_equal(fn(-3*pi), 3.141592653589793)
	assert_almost_equal(fn(-4*pi), 0.0)
	# Loop around 2*pi
	assert_almost_equal(fn((2*pi)+(pi/4)), 0.7853981633974483)
	assert_almost_equal(fn((2*pi)+(pi/2)), 1.5707963267948966)
	assert_almost_equal(fn((2*pi)+(3*pi/4)), 2.356194490192345)
	assert_almost_equal(fn((2*pi)+(pi)), 3.141592653589793)
	assert_almost_equal(fn((2*pi)+(5*pi/4)), -2.356194490192345)
	assert_almost_equal(fn((2*pi)+(3*pi/2)), -1.5707963267948966)
	assert_almost_equal(fn((2*pi)+(7*pi/4)), -0.7853981633974483)

