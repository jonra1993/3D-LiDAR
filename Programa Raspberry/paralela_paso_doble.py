# -*- coding: utf-8 -*-
"""
Created on Wed Dec 21 13:29:45 2016

@author: jonathan
"""

"""
IMPLEMENTACION DE GRAFICAS EN 3D Y FUNCION DE CALCULO DENTRO DE LA INTERFAZ DE VISUALIZACION 
"""

#Libreria LASER
import time 
import Hokuyolib
 
from multiprocessing import Process, Pool, Lock, Array 
# Libreria de Operaciones matematicas
import numpy as np

### Libreria NUBE DE PUNTOS
import pcl
from funciones_creadas import abrir_puerto, puntos_mas_cercanos, medianapuntocercano, coordenadas_r_e_a
from serial import Serial
   
def calculo_trigo(lim_min_servo,lim_max_servo,separ_angulos,paso_min_laser,paso_max_laser):
    #Creacion de un vector para representar cada grado segun el movimiento del servo
    # Grados_SERVO =-LIMITES +225 
    rango_r=np.arange(lim_min_servo,lim_max_servo+1,separ_angulos)
    t_ra=np.linspace(lim_min_servo+225,-lim_max_servo+225,rango_r.size)
    t_ro=np.linspace(lim_min_servo-135,lim_max_servo-135,rango_r.size)         
    ##LASER: Transformacion de pasos a grados
    # Los valores ingresados para el laser son pasos por lo q es necesario trans a grados
    ang_laser_min=np.floor(0.3522*paso_min_laser-45.44)
    ang_laser_max=np.floor(0.3522*paso_max_laser-45.44)
    # Se crea una vecrtor con cada grado  del barrido del Laser
    t_l=np.radians(np.linspace(ang_laser_max,ang_laser_min,paso_max_laser-paso_min_laser+1))   
    ######### Tabla de Senos y Cosenos
    ## Radio
    seno_ra=np.sin(np.radians(t_ra))
    coseno_ra=np.cos(np.radians(t_ra))
    # Rotacion 
    seno_ro=np.sin(np.radians(t_ro))
    coseno_ro=np.cos(np.radians(t_ro))
    ## Laser
    seno_laser=np.sin(t_l)
    coseno_laser=np.cos(t_l)   
    return seno_ra,coseno_ra,seno_ro,coseno_ro, seno_laser,coseno_laser,t_ra.size,t_l.size 

def crear_matrix_almacenamiento(rango_r,rango_l):
    puntos_x_graf3=np.ones([rango_r,rango_l],dtype=np.float32)
    puntos_y_graf3=np.ones(puntos_x_graf3.shape,dtype=np.float32)
    puntos_z_graf3=np.ones(puntos_x_graf3.shape,dtype=np.float32)
    return puntos_x_graf3, puntos_y_graf3 ,puntos_z_graf3

#2.3) TRANSFORMACION DE COORDENADAS (Izquierda-Derecha).................................................      
def trans_espacio(sentido,arduino,S_laser,filas,columnas,lim_min_servo,separ_angulos,paso_min_laser,paso_max_laser,umbral,puntos_x_graf3,puntos_y_graf3,puntos_z_graf3):
    #MOV. SERVO PLANO -X-Y
    if sentido==False: #de la DR
        m=np.flipud(np.arange(0,filas))
    else:
        m=np.arange(0,filas)  #IZ
            
    for angr in m:
        arduino.write(chr(lim_min_servo+angr*separ_angulos))
        dato1=S_laser.get_scan(paso_min_laser,paso_max_laser)  
        w=coseno_ro[angr]
        v=seno_ro[angr]
        tx=ra*coseno_ra[angr]
        ty=-ra*seno_ra[angr]
        Tran_H=np.matrix([[w,-v,0,tx],[v,w,0,ty],[0,0,1,0],[0,0,0,1]])
        # MOV. LASER PLANO -XZ
        for angl in np.arange(0,columnas):
            dis=dato1[1][angl]
            ## Delimitacion de datos                   
            if dis<umbral:
                dis=5000;
            else:
                dis=dis
                
            z_laser=dis*np.sin(np.radians(dato1[0][angl]))
            x_laser=-dis*np.cos(np.radians(dato1[0][angl])) #Se encuentra en el eje - de la x
            p_medidos=np.matrix([[x_laser],[0],[z_laser],[1]])
            puntos_real=Tran_H*p_medidos
            #print puntos_real
            puntos_x_graf3[angr,angl]=puntos_real[0]
            puntos_y_graf3[angr,angl]=puntos_real[1]
            puntos_z_graf3[angr,angl]=puntos_real[2]
                    
                    
def Agrupacion_en_sola_mtx(puntos_x_graf3,puntos_y_graf3,puntos_z_graf3):
    x_pc=np.resize(puntos_x_graf3,(1,puntos_x_graf3.size))
    y_pc=np.resize(puntos_y_graf3,(1,puntos_y_graf3.size))
    z_pc=np.resize(puntos_z_graf3,(1,puntos_z_graf3.size))
    pc_matriz=np.concatenate((x_pc,y_pc,z_pc))
    pc_matriz1=pc_matriz.T
    return pc_matriz1

    
def Procesamiento_pointcloud(pc_matriz1,recorte,mult_thrs,seg_eps_ang,seg_max_inter,seg_dist_thres):
    cloud=pcl.PointCloud()
    cloud.from_array(pc_matriz1)
    pcl.save(cloud,'cloud.ply')        
    ######################## Recorte
    fil_r = cloud.make_passthrough_filter()
    fil_r.set_filter_field_name("x")
    fil_r.set_filter_limits(-recorte, 0)
    cloud_filtered = fil_r.filter()
    pcl.save(cloud_filtered,'recorte.ply')            
    ###############DownSampling        
    fil = cloud_filtered.make_statistical_outlier_filter()
    fil.set_mean_k(50)
    fil.set_std_dev_mul_thresh(mult_thrs)
    cloud_down=fil.filter()
    pcl.save(cloud_down,'downs.ply')     
    ############## Identificacion Suelo obtaculos
    seg = cloud_down.make_segmenter_normals(ksearch=50)
    seg.set_optimize_coefficients(True)
    seg.set_model_type(pcl.SACMODEL_PERPENDICULAR_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_axis(0.0,0.0,1.0) #Plano Perpendicular al que se desea eliminar (Z)
    seg.set_eps_angle(seg_eps_ang) #the maximum allowed difference between the model normal and the given axis in radians
    seg.set_max_iterations(seg_max_inter) #numero de interacion alto para evidencia Suelo
    seg.set_distance_threshold(seg_dist_thres)
    indicesS,model=seg.segment()
    #Identificacion de Obstaculos
    cloud_obstc=cloud_down.extract(indicesS, negative=True)
    pcl.save(cloud_obstc,'obtcs.ply')
   
    #ingresa la nube de obstaculos y regresa el punto
    a=puntos_mas_cercanos(cloud_obstc,50)
    elmascercano=medianapuntocercano(a)
    coor=coordenadas_r_e_a(elmascercano)    
    np.savetxt('punto.cvs',elmascercano,delimiter=",")  #guarda las coordenas x,y,z
    np.savetxt('punto_rea.cvs',coor,delimiter=",")  #guarda las coordenas esfericas
    #return coor


###########################################################################
def in_arduino():
    puertoAr=abrir_puerto('arduino')  
    bd = 115200                                                             
    arduino=Serial(port=puertoAr, baudrate=bd, timeout=0.5)
    if(arduino.isOpen() == False):
        arduino.open()
    return arduino

#Inicializacion de puerto LASER
def in_laser():
    uart_port = abrir_puerto('laser')  
    uart_speed = 19200       
    laser_serial = Serial(port=uart_port, baudrate=uart_speed, timeout=0.5)                                                      
    if(laser_serial.isOpen() == False):
        laser_serial.open()
    return Hokuyolib.Hokuyo(laser_serial)   
    

def shared_array(shape):    
    shared_array_base = Array(ctypes.c_double, shape[0]*shape[1])
    shared_array = np.ctypeslib.as_array(shared_array_base.get_obj())
    shared_array = shared_array.reshape(*shape)
    return shared_array


############################################################################
### PROGRAMA PRINCIPAL
#variables de inicializacion
ra=37 #[mm] distancia desde el eje fijo de trans hasta centro laser
lim_min_servo=105
lim_max_servo=165
separar_angulos=1

paso_min_laser=300
paso_max_laser=470
        
umbral=50 #[mm] umbral de puntos muy cercanos
        
#Parametros Filtrado inicial
recorte=2000        
mult_thrs=0.01
seg_max_inter=370
seg_dist_thres=18
seg_eps_ang=0.5

w=[lim_min_servo,lim_max_servo,separar_angulos,paso_min_laser,paso_max_laser]
seno_ra,coseno_ra,seno_ro,coseno_ro, seno_laser,coseno_laser,filas,columnas=calculo_trigo(w[0],w[1],w[2],w[3],w[4])

#puntos_x_graf3, puntos_y_graf3 ,puntos_z_graf3=crear_matrix_almacenamiento(filas,columnas)

puntos_x_graf3 = shared_array((filas,columnas))
puntos_y_graf3 = shared_array((filas,columnas))
puntos_z_graf3 = shared_array((filas,columnas))
 
S_laser=in_laser()
arduino=in_arduino()

S_laser.laser_on() #enciende el laser  
a=0
sentido=True

while (a<3):
    if sentido==True:
        trans_espacio(sentido,arduino,S_laser,filas,columnas,lim_min_servo,separar_angulos,paso_min_laser,paso_max_laser,umbral,puntos_x_graf3,puntos_y_graf3,puntos_z_graf3)
        sentido=False
    else:
        trans_espacio(sentido,arduino,S_laser,filas,columnas,lim_min_servo,separar_angulos,paso_min_laser,paso_max_laser,umbral,puntos_x_graf3,puntos_y_graf3,puntos_z_graf3)
        sentido=True
    
        # Creacion de Nube de Puntos y filtrado
    pc_matriz1=Agrupacion_en_sola_mtx(puntos_x_graf3,puntos_y_graf3,puntos_z_graf3)
    Procesamiento_pointcloud(pc_matriz1,recorte,mult_thrs,seg_eps_ang,seg_max_inter,seg_dist_thres)        
    a=a+1      

S_laser.laser_off()    