import pickle
import numpy as np
import matplotlib.pyplot as plt
import os

import fly_plot_lib.plot as fpl

import data_fit.models

def open_data_file(filename):
    f = open(filename)
    data = pickle.load(f)
    pos_list = data['flydra_position']
    for i, pos in enumerate(pos_list):
        pos_list[i] = get_tip_pos_from_led_pos(pos)
    data['flydra_position'] = pos_list
    return data
    

def get_tip_pos_from_led_pos(pos):
    led_pos = np.array([0.706909511404, -0.0480302411391, 0.0385864020082])
    tip_pos = np.array([.5936, -.0715, -.0261])
    return pos - led_pos + tip_pos
    
    
def get_all_data_filenames(path, string='pid_flydra_data'):
    cmd = 'ls ' + path
    ls = os.popen(cmd).read()
    all_filelist = ls.split('\n')
    try:
        all_filelist.remove('')
    except:
        pass
    filenames = []
    for filename in all_filelist:
        f = os.path.join(path, filename)
        if string in filename:
            filenames.append(f)
    return filenames

def open_all_data_files(path):
    filenames = get_all_data_filenames(path)
    data = []
    for filename in filenames:
        data.append(open_data_file(filename))
    return data
    

def get_binned_data(positions, values):
    nbins = 600
    binned_values = [[] for i in range(nbins)]
    bins = np.linspace(-.15, .15, nbins) # 2mm bins
    
    
    for i, pos in enumerate(positions):
        bin_index = np.argmin(np.abs(bins-pos))
        binned_values[bin_index].append(values[i])
    
    mean_binned_values = np.nan_to_num([np.mean(vals) for vals in binned_values])
    
    return bins, mean_binned_values
    
    
def get_means_of_data_files(path):
    all_data = open_all_data_files(path)
    zpositions = []
    ypositions = []
    xpositions = []
    pid_vals = []
    
    for i, data in enumerate(all_data):
        all_positions = []
        all_pid_vals = []
    
        pos = np.vstack(data['flydra_position'])
        vals = np.array(data['pid'])
        indices_where_no_plume = np.where(pos[:,1]<0)[0]
        vals = vals - np.mean(vals[indices_where_no_plume])
        
        all_positions.extend(pos[:,1].tolist())
        all_pid_vals.extend(vals)
        
        
        bins, mean_binned_values = get_binned_data(all_positions, all_pid_vals)
        
        non_nan_indices = np.where(np.isnan(pos[:,2]) == False)[0]
        zpos = np.mean(pos[:,2][non_nan_indices])
        non_nan_indices = np.where(np.isnan(pos[:,0]) == False)[0]
        xpos = np.mean(pos[:,0][non_nan_indices])
        
        zpositions.append(np.ones_like(bins)*zpos)
        xpositions.append(np.ones_like(bins)*xpos)
        ypositions.append(bins)
        pid_vals.append(mean_binned_values)
        
    return xpositions, ypositions, zpositions, pid_vals
    

def compile_data_to_list(data):
    new_data = []
    for d in data:
        if type(d) is not list:
            new_data.extend(d.tolist())
        else:
            new_data.extend(d)
    
    return np.array(new_data)
    

def get_gaussian_model_of_data_files(path):
    
    xpositions, ypositions, zpositions, pid_vals = get_means_of_data_files(path)
    
    ypositions = compile_data_to_list(ypositions)
    zpositions = compile_data_to_list(zpositions)
    pid_vals = compile_data_to_list(pid_vals)

    gm = data_fit.models.GaussianModel2D()
    gm.fit_with_guess(pid_vals, [ypositions, zpositions])
    
    #gm.parameters['magnitude'] = 200
    
    if 1:
    
        fig = plt.figure()
        ax = fig.add_subplot(111, rasterized=True)
        
        im, extent = gm.get_array_2d([-.15, .15], [-.15,.15], 0.001)
        ax.imshow(im, extent=extent, origin='lower')
        #ax.plot(x,y,'b.')
    
        fpl.scatter(ax, ypositions, zpositions, pid_vals, colormap='jet', colornorm=[0,gm.parameters['magnitude']], radius=0.001, use_ellipses=False)
        
        ax.set_xlim([-.15,.15])
        ax.set_ylim([-.15,.15])
        ax.set_xlabel('crosswind position, meters')
        ax.set_ylabel('altitude position, meters')
        ax.set_aspect('equal')
        #plt.show()
        
        filename = 'odor_plume_plot_2d.pdf'
        filename_with_path = os.path.join(path, filename)
        
        fig.savefig(filename_with_path, format='pdf')
    
    return gm
    
    
    

def plot_all_data_files(path, colors=None):
    all_data = open_all_data_files(path)
    fig = plt.figure(figsize=(8,8))
    ax = fig.add_subplot(111)
    ax.set_rasterization_zorder(0)
    
    if colors is None:
        colors = ['red' for i in range(len(all_data))]
    
    for i, data in enumerate(all_data):
        all_positions = []
        all_pid_vals = []
    
        pos = np.vstack(data['flydra_position'])
        vals = np.array(data['pid'])
        indices_where_no_plume = np.where(pos[:,1]<-.04)[0]
        print np.mean(vals[indices_where_no_plume])
        vals = vals - np.mean(vals[indices_where_no_plume])
        print np.mean(vals[indices_where_no_plume])
        #ax.plot(pos[:,1], vals, color='gray')
        
        all_positions.extend(pos[:,1].tolist())
        all_pid_vals.extend(vals)
        
        
        bins, mean_binned_values = get_binned_data(all_positions, all_pid_vals)
        
        print i
        try:
            ax.plot(bins, mean_binned_values, color=colors[i], zorder=10)
        except:
            ax.plot(bins, mean_binned_values, color='pink', zorder=10)
        
    
    ax.set_xlim(-.05, .1)
    fpl.adjust_spines(ax, ['left', 'bottom'])
    
    ax.set_xlabel('crosswind position, m')
    ax.set_ylabel('pid value')
    
    filename = 'odor_plume_plot.pdf'
    filename_with_path = os.path.join(path, filename)
    
    fig.savefig(filename_with_path, format='pdf')


def get_mean_x(path):
    xpositions, ypositions, zpositions, pid_vals = get_means_of_data_files(path)
    
    xpositions = compile_data_to_list(xpositions)
   
   
    return np.mean(xpositions)
        
