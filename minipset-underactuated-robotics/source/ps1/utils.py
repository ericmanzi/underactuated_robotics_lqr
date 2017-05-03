from math import sin, cos, pi
from numpy import matrix, array, identity, allclose
from control.matlab import *
import scipy.linalg
from nose.tools import assert_equal, ok_, assert_almost_equal, assert_in
from IPython.display import display, HTML, clear_output, display_html, Javascript
import pendulum
import simulate as sim


def video(fname, mimetype):
    from IPython.display import HTML
    video_encoded = open(fname, "rb").read().encode("base64")
    video_tag = '<video controls alt="test" src="data:video/{0};base64,{1}">'.format(mimetype, video_encoded)
    return HTML(data=video_tag)

def play_local_video(fname):
	return HTML("""
	<video width="620" height="420" controls>
	  <source src="./media/{0}.mp4" type="video/mp4">
	</video>
	""".format(fname))

def test_ok():
    """ If execution gets to this point, print out a happy message """
    try:
        from IPython.display import display_html
        display_html("""<div class="alert alert-success">
        <strong>Tests passed!!</strong>
        </div>""", raw=True)
    except:
        print("Tests passed!!")

# lqr tests
def check_lqr_solver(fn):
	# compute gain vector k
	A = matrix([[0,1],[0,0]])
	B = matrix([[0],[1]])
	Q = matrix([[1,0],[0,0]])
	R = matrix([[1]])
	Ke = matrix([[1., 1.41421356]])
	# Xe = matrix([[ 1.41421356, 1.], [1., 1.41421356]])
	K = fn(A,B,Q,R)
	assert allclose(K, Ke)

	Q1 = matrix([[1,0],[0,1]])
	R1 = matrix([[3]])
	Ke1 = array([[ 0.57735027,  1.21984994]])
	# Xe1 = array([[ 2.11284207,  1.73205081],[ 1.73205081,  3.65954981]])
	K1 = fn(A,B,Q1,R1)
	assert allclose(K1, Ke1)
		
	Q2 = matrix([[1000,0],[0,1000]])
	R2 = matrix([[10]])
	Ke2 = array([[ 10.        ,  10.95445115]])
	# Xe2 = array([[ 1095.44511501,   100.        ],
       # [  100.        ,   109.5445115 ]])
	K2 = fn(A,B,Q2,R2)
	assert allclose(K2, Ke2)
	# ok_(allclose(X, Xe2))

	Q3 = matrix([[1,0],[0,1]])
	R3 = matrix([[0.3]])
	Ke3 = array([[ 1.82574186,  2.64288045]])
	# Xe3 = array([[ 1.44756524,  0.54772256],
       # [ 0.54772256,  0.79286413]])
	K3 = fn(A,B,Q3,R3)
	assert allclose(K3, Ke3)

	test_ok()



"""
Test that after 10 seconds (10000 iterations), goal has been reached
"""
x_0 = 0.
dx_0 = 0.
th_0 = 2*pi/8
dth_0 = 0.

def test_end_at_theta_goal(final_pose):
	theta = final_pose[3]
	success = abs(pendulum.constrain(theta)) < 0.25
	if not success:
		print("Goal pose (theta ~= 0) was not reached. theta = " + str(final_pose[3]))
	return success
	
def test_end_at_x_goal(final_pose):
	x = final_pose[1]
	success = abs(pendulum.constrain(x)) < 1
	if not success:
		print("Goal pose (x ~= 0) was not reached. x = " + str(final_pose[1]))
	return success

def test_end_close_to_goal(final_pose):
	theta_success = test_end_at_theta_goal(final_pose)
	x_success = test_end_at_x_goal(final_pose)
	dtheta_success = final_pose[4] < 0.1
	if not dtheta_success:
		print("Goal pose (dtheta ~= 0) was not reached. dtheta (angular speed) = " + str(final_pose[3]))
	return theta_success and x_success and dtheta_success

def control_with_manual_gain(K_in):
	K = matrix(K_in)
	final_pose = sim.simulate(x_0, dx_0, 2*pi/8, dth_0, K, "manual_K", False)
	assert test_end_close_to_goal(final_pose)
	test_ok()

def tune_cost_functions(solve_lqr, Q_in, R_in):
	global K, Q, R
	Q = matrix(Q_in)
	R = matrix(R_in)
	K = solve_lqr(pendulum.A, pendulum.B, Q, R)
	final_pose = sim.simulate(x_0, dx_0, 2*pi/8, dth_0, K, "tune_cost_fn", False)
	assert test_end_close_to_goal(final_pose)
	test_ok()

def test_threshold(neg_threshold_in, pos_threshold_in):
	K = matrix([[ -1.,  -2., -40.,  -7.]])
	print("Trying "+str(neg_threshold_in))
	final_pose = sim.simulate(x_0, dx_0, neg_threshold_in, dth_0, K, "threshold_neg", False)
	assert test_end_close_to_goal(final_pose)
	print("OK: "+str(neg_threshold_in))
	print("Trying "+str(pos_threshold_in))
	final_pose2 = sim.simulate(x_0, dx_0, pos_threshold_in, dth_0, K, "threshold_pos", False)
	assert test_end_close_to_goal(final_pose2)
	print("OK: "+str(pos_threshold_in))
	# Should have returned here if goal not reached
	# If goal reached, check that threshold is atleast pi/5 
	assert neg_threshold_in <= -pi/5 and pos_threshold_in >= pi/5
	test_ok()

def simulate_with_swingup(solve_lqr, Q_in, R_in):
	Q = matrix(Q_in)
	R = matrix(R_in)
	K = solve_lqr(pendulum.A, pendulum.B, Q, R)
	final_pose = sim.simulate(x_0, dx_0, pi/2, dth_0, K, "swingup", True)
	assert test_end_close_to_goal(final_pose)
	test_ok()




