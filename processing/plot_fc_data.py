"""jupyter notebook-style cells below allow for easy plotting of data 
   collected during feedback control on drone flight
"""

#%%
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from pathlib import Path
import os

#----------PARAMS------------------#
date = '2022-09-12'
run_num = 13

vars = ['Pitch','AboveObject']
# vars = ['Alt','GPS_x','GPS_y','MoveUp','AboveObject','Pitch','Size_error','OptFlow_On','Flow_x','Flow_y','Vspeed','Fspeed','Hspeed']
#----------------------------------#

#%%
username = os.getlogin( )
maindir = Path('/home/%s/1FeedbackControl' % username)
fc_data = pd.read_csv(maindir.joinpath('%s_run%02d_fc-data' % (date,run_num),'Feedbackdata.csv'))
# fc_data = pd.read_csv('/Users/nate/UMN/FeedbackControl_2022-08-26_goodRuns/FeedbackControl_2022-08-25__10-41-59/data/Feedbackdata.txt')
#%% 2D plots for comparing variables
# %matplotlib qt

n = len(vars)
fig,ax = plt.subplots(nrows = n,dpi=90)
if n > 1:
    for ii in range(n):
        ax[ii].plot(fc_data.Timestamp,fc_data[vars[ii]])
        ax[ii].set_ylabel(vars[ii])
        ax[ii].set_xlabel('Timestamp')
else:
    ax.plot(fc_data.Timestamp,fc_data[vars[0]])
    ax.set_ylabel(vars[0])
    ax.set_xlabel('Timestamp')
plt.tight_layout()
plt.show()
#%% 3D PLOTS for visualizing flight
# plt.close('all')
# %matplotlib qt
fig = plt.figure(figsize=(5,5),dpi=90)
ax = fig.add_subplot(projection='3d')
tmp = ax.scatter(fc_data.GPS_x,fc_data.GPS_y,fc_data.Alt,c=fc_data.Timestamp)
fig.colorbar(tmp,label='Timestamp',shrink=0.5)
ax.set_xlabel('East [m]')
ax.set_ylabel('North [m]')
ax.set_zlabel('Altitude [m]')
plt.show()

for ii in range(n):
    fig = plt.figure(figsize=(5,5),dpi=90)
    ax = fig.add_subplot(projection='3d')
    tmp = ax.scatter(fc_data.GPS_x,fc_data.GPS_y,fc_data.Alt,c=fc_data[vars[ii]])
    fig.colorbar(tmp,label=vars[ii],shrink=0.5)
    ax.set_xlabel('East [m]')
    ax.set_ylabel('North [m]')
    ax.set_zlabel('Altitude [m]')

plt.show()