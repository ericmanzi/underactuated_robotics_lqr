import os
from math import sin, cos, pi

import matplotlib.pyplot as plt

import pendulum

pendulum = pendulum.Pendulum(
    .001,
    # 10, # dtheta
    # [0, 0., -pi/2, 0.],
    [0, 0., 2*pi, 0.],
    10,
)
data = pendulum.integrate()

fig = plt.figure(0)
fig.suptitle("Pendulum on Cart")

cart_time_line = plt.subplot2grid(
    (12, 12),
    (9, 0),
    colspan=12,
    rowspan=3
)
cart_time_line.axis([
    0,
    10,
    min(data[:,1])*1.1,
    max(data[:,1])*1.1+.1,
])
cart_time_line.set_xlabel('time (s)')
cart_time_line.set_ylabel('x (m)')
cart_time_line.plot(data[:,0], data[:,1],'r-')

pendulum_time_line = cart_time_line.twinx()
pendulum_time_line.axis([
    0,
    10,
    min(data[:,3])*1.1-.1,
    max(data[:,3])*1.1
])
pendulum_time_line.set_ylabel('theta (rad)')
pendulum_time_line.plot(data[:,0], data[:,3],'g-')

cart_plot = plt.subplot2grid(
    (12,12),
    (0,0),
    rowspan=8,
    colspan=12
)
cart_plot.axes.get_yaxis().set_visible(False)

def video(fname, mimetype):
    from IPython.display import HTML
    video_encoded = open(fname, "rb").read().encode("base64")
    video_tag = '<video controls alt="test" src="data:video/{0};base64,{1}">'.format(mimetype, video_encoded)
    return HTML(data=video_tag)

time_bar, = cart_time_line.plot([0,0], [10, -10], lw=3)
def draw_point(point):
    time_bar.set_xdata([t, t])
    cart_plot.cla()
    cart_plot.axis([-1.1,1.1,-.8,.8])
    cart_plot.plot([point[1]-.1,point[1]+.1],[0,0],'r-',lw=5)
    cart_plot.plot([point[1],point[1]+.4*sin(point[3])],[0,.4*cos(point[3])],'g-', lw=4)
t = 0
fps = 25.
frame_number = 1
# Create image output directory if it doesn't exist
try:
    os.makedirs('./img')
except OSError:
    pass
# Delete pendulum output video if it already exists
try:
    os.remove('./pendulum_lqr.mp4')
except OSError:
    pass
    
for point in data:
    if point[0] >= t + 1./fps or not t:
        draw_point(point)
        t = point[0]
        fig.savefig('img/_tmp%03d.png' % frame_number)
        frame_number += 1

print os.system("ffmpeg -framerate 25 -i img/_tmp%03d.png  -c:v libx264 -r 30 -pix_fmt yuv420p pendulum_lqr.mp4")

# In the next cell, run
# video("pendulum_lqr.mp4", "mp4")








