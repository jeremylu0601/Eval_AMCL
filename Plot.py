#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import csv 
from scipy.spatial.transform import Rotation as R

def process_raw_data(data_path,gt_path):
    #Calculate the error and running time on a travel
    
    data=np.loadtxt(data_path,dtype=np.float,delimiter=',')
    gt=np.loadtxt(gt_path,dtype=np.float,delimiter=',')
    output=np.zeros((data.shape[0]-1,4))
    # The firt data from AMCL is discarded because it is stored when Save_Data.py is executed.
    # If we used it to calculate the running time, the mean running time would be boosted.
    # In other words, the time from executing Save_Data.py to clicking "2D Navi Goal" is taken into account.
    
    start=data[1,0]
    j=0
    rt_ult=np.zeros((2,1))
    # It is used to stored the running time
    
    for i in range(data.shape[0]-1):
        index=i+1
        while gt[j,0]>data[index,0]:
            index=index+1
        while gt[j+1,0]<data[index,0] :
            j=j+1
        # Interpolation is used to calculate the error, so we have to make sure that index-th AMCL data happened between j-th and (j+1)-th Gazebo data
        # i.e., gt[j,0]<data[index,0]<gt[j+1,0]

        last_gt=R.from_quat([0,0,gt[j,3],gt[j,4]])
        next_gt=R.from_quat([0,0,gt[j+1,3],gt[j+1,4]])
        a=last_gt.as_euler('xyz')
        b=next_gt.as_euler('xyz')
        data_angle=R.from_quat([0,0,data[index,3],data[index,4]])
        c=data_angle.as_euler('xyz')
        gap=b[2]-a[2]
        if gap>np.pi:
            gap=gap-2*np.pi
        if gap<-1*np.pi:
            gap=gap+2*np.pi
        gt_angle=a[2]+gap*((data[index,0]-gt[j,0])/(gt[j+1,0]-gt[j,0]))
        if gt_angle>np.pi:
            gt_angle=gt_angle-2*np.pi
        if gt_angle<-1*np.pi:
            gt_angle=gt_angle+2*np.pi
            
        x_gt=gt[j,1]+(gt[j+1,1]-gt[j,1])*((data[index,0]-gt[j,0])/(gt[j+1,0]-gt[j,0]))
        y_gt=gt[j,2]+(gt[j+1,2]-gt[j,2])*((data[index,0]-gt[j,0])/(gt[j+1,0]-gt[j,0]))
        
        output[i,0]=np.abs(data[index,0]-start) #the time started from the first AMCL data after clicking "2D Navi Goal"
        output[i,1]=np.abs(data[index,1]-x_gt)
        output[i,2]=np.abs(data[index,2]-y_gt)

        error=c[2]-gt_angle
        if error>np.pi:
            error=error-2*np.pi
        if error<-1*np.pi:
            error=error+2*np.pi

        output[i,3]=np.abs(error)
        if index>1:
            rt_ult[0,0]=rt_ult[0,0]+1
            rt_ult[1,0]=rt_ult[1,0]+data[index,0]-data[index-1,0]

    rt=rt_ult[1,0]/rt_ult[0,0]
    return output,rt



def eval_cg(path,output,i):
    # Put the mean error and mean running time in the output matrix for a specific number of particles
    
    data_path=path+"/cpu/1/amcl.csv"
    gt_path=path+"/cpu/1/gt.csv"
    output_c,rt=process_raw_data(data_path,gt_path)
    output[i,1]=rt
    output[i,7]=np.mean(output_c,axis=0)[1]
    output[i,9]=np.mean(output_c,axis=0)[2]
    output[i,11]=np.mean(output_c,axis=0)[3]
    # calculate the mean error and mean running time on travel 1 while AMCL is on CPU
    
    data_path=path+"/gpu/1/amcl.csv"
    gt_path=path+"/gpu/1/gt.csv"
    output_g,rt=process_raw_data(data_path,gt_path)
    output[i,8]=np.mean(output_g,axis=0)[1]
    output[i,10]=np.mean(output_g,axis=0)[2]
    output[i,12]=np.mean(output_g,axis=0)[3]
    output[i,2]=rt
    # calculate the mean error and mean running time on travel 1 while AMCL is on GPU



    data_path=path+"/cpu/2/amcl.csv"
    gt_path=path+"/cpu/2/gt.csv"
    output_c,rt=process_raw_data(data_path,gt_path)
    output[i,13]=np.mean(output_c,axis=0)[1]
    output[i,15]=np.mean(output_c,axis=0)[2]
    output[i,17]=np.mean(output_c,axis=0)[3]
    output[i,3]=rt
    # calculate the mean error and mean running time on travel 2 while AMCL is on CPU

    data_path=path+"/gpu/2/amcl.csv"
    gt_path=path+"/gpu/2/gt.csv"
    output_g,rt=process_raw_data(data_path,gt_path)
    output[i,14]=np.mean(output_g,axis=0)[1]
    output[i,16]=np.mean(output_g,axis=0)[2]
    output[i,18]=np.mean(output_g,axis=0)[3]
    output[i,4]=rt
    # calculate the mean error and mean running time on travel 2 while AMCL is on GPU


    data_path=path+"/cpu/3/amcl.csv"
    gt_path=path+"/cpu/3/gt.csv"
    output_c,rt=process_raw_data(data_path,gt_path)
    output[i,19]=np.mean(output_c,axis=0)[1]
    output[i,21]=np.mean(output_c,axis=0)[2]
    output[i,23]=np.mean(output_c,axis=0)[3]
    output[i,5]=rt
    # calculate the mean error and mean running time on travel 3 while AMCL is on CPU
    
    data_path=path+"/gpu/3/amcl.csv"
    gt_path=path+"/gpu/3/gt.csv"
    output_g,rt=process_raw_data(data_path,gt_path)
    output[i,20]=np.mean(output_g,axis=0)[1]
    output[i,22]=np.mean(output_g,axis=0)[2]
    output[i,24]=np.mean(output_g,axis=0)[3]
    output[i,6]=rt
    # calculate the mean error and mean running time on travel 3 while AMCL is on GPU
    
    return 0

if __name__ == "__main__":

    #--------------------------
    # feel free to change the value here
    # make sure the names of folders match the values here
    # like there is folder named as "500"
    output=np.zeros((7,25))
    output[0,0]=500
    output[1,0]=1000
    output[2,0]=2000
    output[3,0]=4000
    output[4,0]=6000
    output[5,0]=8000
    output[6,0]=10000
    #--------------------------
    

    for i in range(output.shape[0]):
        num=int(output[i,0])
        path=str(num)        # define the path as the number of particles, so we can search the correponding folder
        eval_cg(path,output,i)  # Put the mean error and mean running time in the output matrix for a specific number of particles
    

    plt.figure()
    plt.subplot(3,1,1)
    plt.plot(output[:,0], output[:,1],'b*--',output[:,0], output[:,2],'r*--')
    plt.title('Running Time(s) V.S. Number of Particles')
    plt.ylabel('Travel 1')
    plt.subplot(3,1,2)
    plt.plot(output[:,0], output[:,3],'b*--',output[:,0], output[:,4],'r*--')
    plt.ylabel('Travel 2')
    plt.subplot(3,1,3)
    plt.plot(output[:,0], output[:,5],'b*--',output[:,0], output[:,6],'r*--')
    plt.ylabel('Travel 3')
    plt.xlabel('Number of Particles')
    plt.legend(('cpu','gpu'))
    plt.savefig('Running_Time.png')

    plt.figure()
    plt.subplot(3,1,1)
    plt.plot(output[:,0], output[:,7],'b*--',output[:,0], output[:,8],'r*--')
    plt.title('Mean Error in Travel 1 V.S. Number of Particles')
    plt.ylabel('Error in X')
    plt.subplot(3,1,2)
    plt.plot(output[:,0], output[:,9],'b*--',output[:,0], output[:,10],'r*--')
    plt.ylabel('Error in Y')
    plt.subplot(3,1,3)
    plt.plot(output[:,0], output[:,11],'b*--',output[:,0], output[:,12],'r*--')
    plt.ylabel('Error in Theta')
    plt.xlabel('Number of Particles')
    plt.legend(('cpu','gpu'))
    plt.savefig('Travel_1.png')


    plt.figure()
    plt.subplot(3,1,1)
    plt.plot(output[:,0], output[:,13],'b*--',output[:,0], output[:,14],'r*--')
    plt.title('Mean Error in Travel 2 V.S. Number of Particles')
    plt.ylabel('Error in X')
    plt.subplot(3,1,2)
    plt.plot(output[:,0], output[:,15],'b*--',output[:,0], output[:,16],'r*--')
    plt.ylabel('Error in Y')
    plt.subplot(3,1,3)
    plt.plot(output[:,0], output[:,17],'b*--',output[:,0], output[:,18],'r*--')
    plt.ylabel('Error in Theta')
    plt.xlabel('Number of Particles')
    plt.legend(('cpu','gpu'))
    plt.savefig('Travel_2.png')


    plt.figure()
    plt.subplot(3,1,1)
    plt.plot(output[:,0], output[:,19],'b*--',output[:,0], output[:,20],'r*--')
    plt.title('Mean Error in Travel 3 V.S. Number of Particles')
    plt.ylabel('Error in X')
    plt.subplot(3,1,2)
    plt.plot(output[:,0], output[:,21],'b*--',output[:,0], output[:,22],'r*--')
    plt.ylabel('Error in Y')
    plt.subplot(3,1,3)
    plt.plot(output[:,0], output[:,23],'b*--',output[:,0], output[:,24],'r*--')
    plt.ylabel('Error in Theta')
    plt.xlabel('Number of Particles')
    plt.legend(('cpu','gpu'))
    plt.savefig('Travel_3.png')
