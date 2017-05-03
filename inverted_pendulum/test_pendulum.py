from math import sin, cos, pi
from numpy import matrix, array, identity, allclose
from control.matlab import *
import scipy.linalg
from nose.tools import assert_equal, ok_, assert_almost_equal, assert_in
import pendulum

# lqr tests
def test_lqr(fn):
	# compute gain vector k
	A = matrix([[0,1],[0,0]])
	B = matrix([[0],[1]])
	Q = matrix([[1,0],[0,0]])
	R = matrix([[1]])
	Ke = matrix([[1., 1.41421356]])
	Xe = matrix([[ 1.41421356, 1.], [1., 1.41421356]])
	(K, X) = fn(A,B,Q,R)
	ok_(allclose(K, Ke))
	ok_(allclose(X, Xe))
	
	Q1 = matrix([[1,0],[0,1]])
	R1 = matrix([[3]])
	Ke1 = array([[ 0.57735027,  1.21984994]])
	Xe1 = array([[ 2.11284207,  1.73205081],[ 1.73205081,  3.65954981]])
	(K, X) = fn(A,B,Q1,R1)
	ok_(allclose(K, Ke1))
	ok_(allclose(X, Xe1))
		
	Q2 = matrix([[1000,0],[0,1000]])
	R2 = matrix([[10]])
	Ke2 = array([[ 10.        ,  10.95445115]])
	Xe2 = array([[ 1095.44511501,   100.        ],
       [  100.        ,   109.5445115 ]])
	(K, X) = fn(A,B,Q2,R2)
	ok_(allclose(K, Ke2))
	ok_(allclose(X, Xe2))

	Q3 = matrix([[1,0],[0,1]])
	R3 = matrix([[0.3]])
	Ke3 = array([[ 1.82574186,  2.64288045]])
	Xe3 = array([[ 1.44756524,  0.54772256],
       [ 0.54772256,  0.79286413]])
	(K, X) = fn(A,B,Q3,R3)
	ok_(allclose(K, Ke3))
	ok_(allclose(X, Xe3))
	print "Passed!!"


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


# test weighted average approximation. Not testing
# def test_average(fn):

# def test_step_function(fn):


def test_end_at_theta_goal(final_pose):
	theta = final_pose[2]
	success = abs(pendulum.constrain(theta)) < 0.25
	return success
	
def test_end_at_x_goal(final_pose):
	x = final_pose[0]
	success = abs(pendulum.constrain(x)) < 0.25
	return success

def test_end_close_to_goal(data):
	final_pose = data[len(data)-1]
	theta_success = test_end_at_theta_goal(final_pose)
	x_success = test_end_at_x_goal(final_pose)
	dtheta_success = final_pose[3] < 0.1


# run all tests

def do_lqr(A,B,Q,R):
    X = matrix(scipy.linalg.solve_continuous_are(A, B, Q, R))
    K = matrix(scipy.linalg.inv(R)*(B.T*X))
    return K, X


test_lqr(do_lqr)




