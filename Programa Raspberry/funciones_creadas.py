# -*- coding: utf-8 -*-
"""
Created on Tue Oct  4 12:19:04 2016

@author: diego
"""

import pcl
import numpy as np

#%% CONVERSION A SISTEMA AZIMUT Y ELEVACION
def abrir_puerto(y):
    if y=='arduino':
        c='USB'
    elif y=='laser':
        c='ACM'
    
    import serial
    # Varialbe para saber si hemos encontrado el puerto o no
    bEncontrado = False
    # Hacemos un bulce para recorrer los puertos que queremos comprobar
    for iPuerto in range(0, 4):
        try:
            # Puerto que vamos a probar
            PUERTO = '/dev/tty'+c+ str(iPuerto)
            # Velocidad
            VELOCIDAD = '19200'
            # Probamos ha abrir el puerto
            Sensor = serial.Serial(PUERTO, VELOCIDAD)
            # si no se ha producido un error, cerramos el puerto
            Sensor.close()
            # cambiamos el estado del la variable para saber si lo hemos encontrado
            bEncontrado = True
            # Salimos del bucle
            break
        except:
            # Si hay error, no hacemos nada y continuamos con la busqueda
            pass
 # Si encontramos el puerto?
    if bEncontrado:
         print('el puerto del sensor es: ' + '/dev/tty'+c + str(iPuerto))
         return '/dev/tty'+c+str(iPuerto)
    else:
         print('No se ha encontrado Ningun Dispositivo')
         
         
def puntos_mas_cercanos(nube,numpuntos):
    c = nube.to_array()
    o=np.zeros((c.shape),dtype=np.float32)
    origen= pcl.PointCloud()
    origen.from_array(o) #nube
    #el punto mas cercano de p1 al primero elemetno de p2 es
    kd = pcl.KdTreeFLANN(nube)
    indices, sqr_distances = kd.nearest_k_search_for_cloud(origen, numpuntos)
    #guarda los puntos mas cercannos
    cercanos=np.zeros((numpuntos,3),dtype=np.float32)
    for i in range (numpuntos):
        cercanos[i]=nube[indices[0,i]] 
    pcercanos= pcl.PointCloud()
    pcercanos.from_array(cercanos) #nube
    return pcercanos
    
def medianapuntocercano(nube):
    nub = nube.to_array()
    xm=np.median(nub[:,0])
    ym=np.median(nub[:,1])
    zm=np.median(nub[:,2])
    c=np.array((xm,ym,zm))   
    c=c.T
    return c

def coordenadas_r_e_a(punto):
    punto=punto.T
    radio=np.sqrt(np.power(punto[0], 2)+np.power(punto[1], 2)+np.power(punto[2], 2))
    ##solo segundo y tercer cuadrante respecto a x- nuestro norte
    if punto[0]==0:
        azimut=0
    else:
        azimut=-np.arctan(punto[1]/punto[0]) * 180 / np.pi
        
    rr=np.sqrt(np.power(punto[0], 2)+np.power(punto[1], 2))    
    if punto[2]>0:
        theta=(np.pi/2.0)-np.arctan(rr/punto[2])
    elif punto[2]==0:
        theta=0
    else:
        theta=-((np.pi/2.0)+np.arctan(rr/punto[2]))
    theta=theta * 180 / np.pi #en grados    
    out=np.array((radio,theta,azimut))     
    return out
           
def circle(punto,r):
    # theta goes from 0 to 2pi
    theta = np.linspace(0, 2*np.pi, 100)
    # the radius of the circle
    r = np.sqrt(r)
    # compute x1 and x2
    x0 = punto[0]*np.ones(100)
    x1 = r*np.cos(theta)+punto[1]
    x2 = r*np.sin(theta)+punto[2]
    cir=np.array((x0,x1,x2)) 
    cir=cir.T
    return cir
