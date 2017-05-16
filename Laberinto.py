# -*- coding: utf-8 -*-
"""
Created on Sat May  6 18:30:04 2017

POLARITY_INVERSED = ‘inversed’
With inversed polarity, a positive duty cycle will cause the motor to rotate counter-clockwise.

POLARITY_NORMAL = ‘normal’
With normal polarity, a positive duty cycle will cause the motor to rotate clockwise.



@author: jonathan steven franco g
"""
import numpy as np
from ev3dev.auto import *
import time
from time import sleep

numero_filas = 50
numero_columnas = 50
#Creacion de la matriz mapeada al laberinto con las posicion derecha-izquierda y arriba-abajo
# 0 =izq  2 =der 1 =arib  3 =abajo
direccion = [0] * 4       
matriz = [direccion] * numero_filas
for i in range(numero_filas):
    matriz[i] = [direccion] * numero_columnas

class orientacion:
    abajo = 3
    arriba = 1
    der = 2
    izq = 0
    
    def calcBest(self, orien, posicion, actual):
        #direcion de la rotacion, numero de rotaciones, nueva orientacion
        mejor = np.array[0, 0, orien]
        for i in range(4):
            temp = (matriz[self.posicion[0]][self.posicion[1]])[i]
            if temp == 0:
                if (orien - i) == 1 and orien - i > 0:
                    return  np.array[0 , 1, i]
                if (orien - i == 1) and orien - i < 0:
                     return  np.array[1 , 1, i]
                else:
                     return  np.array[0 , 2, i]
            if temp == 1 and actual != 1:
                if (orien - i) == 1 and orien - i > 0:
                    mejor = np.array[0 , 1, i]
                if (orien - i == 1) and orien - i < 0:
                     mejor = np.array[1 , 1, i]
                if (mejor[1] != 1):
                     mejor = np.array[1, 2, i]
                     
        #fin del for
        
        return mejor
                

    
class robot:
    #MOTORS
    motorD =  Motor('outA')
    motorI =  Motor('outD')
    
    # COLOR SENSOR MODE REFLECT
    clR = UltrasonicSensor()
    assert clR.connected

    
     # COLOR SENSOR MODE COLOR
    cl = ColorSensor() 
    assert cl.connected
    cl.mode='COL-COL'   

    #una aproximacion de rotacion por segundo
    rotacionMsec = 1
    Nrotaciones = 1
    ortent =  oritancio = orientacion.abajo
    posicion = []
    
    #rotacion de robot sobre su propio eje 
    def rotar(self,movimiento):
        if(movimiento[0]== 0):
            self.motorD.inversed()
        else:
            self.motorA.inversed            
        self.motorD.run_timed(time_sp=movimiento[2]*1000, speed_sp=self.rotacionMsec)
        self.motorA.run_timed(time_sp=movimiento[2]*1000, speed_sp=self.rotacionMsec)
        self.motorD.normal
        self.motorA.normal
        
    
    def start(self,):
        while self.cl.value() != 5:
            if (matriz[self.posicion[0]][self.posicion[1]])[self.oritancio] == 0:
                while self.clR.value() > 13:
                    #n rotaciones de los motores equivalentes a una casilla
                    self.motorD.run_timed(time_sp=self.Nrotaciones, speed_sp=self.rotacionMsec)
                    self.motorA.run_timed(time_sp=self.Nrotaciones, speed_sp=self.rotacionMsec)
                    # se va marcando el laberinto
                    sleep(self.Nrotaciones-500)
                    if self.oritancio == 2:
                        (matriz[self.posicion[0]][self.posicion[1]])[2] = 1
                        (matriz[self.posicion[0]][self.posicion[1]])[0] = 1
                        self.posicion[0] = self.posicion[0] + 1        
                                                              
                    if self.oritancio == 0:
                        (matriz[self.posicion[0]][self.posicion[1]])[0] = 1
                        (matriz[self.posicion[0]][self.posicion[1]])[2] = 1
                        self.posicion[0] = self.posicion[0] - 1
                    if self.oritancio == 1:
                        (matriz[self.posicion[0]][self.posicion[1]])[1] = 1
                        (matriz[self.posicion[0]][self.posicion[1]])[3] = 1
                        self.posicion[1] =  self.posicion[1] - 1                   
                    else:
                        (matriz[self.posicion[0]][self.posicion[1]])[3] = 1
                        (matriz[self.posicion[0]][self.posicion[1]])[1] = 1
                        self.posicion[1] =  self.posicion[1] + 1  
                #fin del while
                    
            if (matriz[self.posicion[0]][self.posicion[1]])[self.oritancio] == 1:
                mejor =orientacion.calcBest(self.oritancio, self.posicion, 1)
                self.rotar(mejor);
                self.oritancio = mejor[2]
                if self.clR.value() < 13:
                    #quiere decir que ay un muro y lo marcamos como si ruta
                    matriz[self.posicion[0]][self.posicion[1]][self.oritancio] = -1
                else:
                    #avanzamos una casilla en la dirrecion mejor[2]                     
                    #n rotaciones de los motores equivalentes a una casilla
                    self.motorD.run_timed(time_sp=self.Nrotaciones, speed_sp=self.rotacionMsec)
                    self.motorA.run_timed(time_sp=self.Nrotaciones, speed_sp=self.rotacionMsec)
                    
                    if matriz[self.posicion[0]][self.posicion[1]][self.oritancio] == 1:
                        #como la anterior estaba en 1 marcamos arriba y abajo como  -1
                        if self.oritancio == 1 or self.oritancio == 3:
                            (matriz[self.posicion[0]][self.posicion[1]])[1] = -1
                            (matriz[self.posicion[0]][self.posicion[1]])[3] = -1
                        #como la anterior estaba en 1 marcamos derecha e izquierda como  -1
                        else:
                            (matriz[self.posicion[0]][self.posicion[1]])[0] = -1
                            (matriz[self.posicion[0]][self.posicion[1]])[2] = -1          
                                
            else:
                mejor =orientacion.calcBest(self.oritancio, self.posicion, -1)
                self.rotar(mejor);
                self.oritancio = mejor[2]
                if self.clR.value() < 13:
                    #quiere decir que ay un muro y lo marcamos como si ruta
                    matriz[self.posicion[0]][self.posicion[1]][self.oritancio] = -1
                else:
                    #avanzamos una casilla en la dirrecion mejor[2]                     
                    #n rotaciones de los motores equivalentes a una casilla
                    self.motorD.run_timed(time_sp=self.Nrotaciones, speed_sp=self.rotacionMsec)
                    self.motorA.run_timed(time_sp=self.Nrotaciones, speed_sp=self.rotacionMsec)
                    
                    if matriz[self.posicion[0]][self.posicion[1]][self.oritancio] == 1:
                        #como la anterior estaba en 1 marcamos arriba y abajo como  -1
                        if self.oritancio == 1 or self.oritancio == 3:
                            (matriz[self.posicion[0]][self.posicion[1]])[1] = -1
                            (matriz[self.posicion[0]][self.posicion[1]])[3] = -1
                        #como la anterior estaba en 1 marcamos derecha e izquierda como  -1
                        else:
                            (matriz[self.posicion[0]][self.posicion[1]])[0] = -1
                            (matriz[self.posicion[0]][self.posicion[1]])[2] = -1    
                
           #self.motorD.run_forever(speed_sp=self.rotacionMsec)           
    def robotp(self, Nro):
        self.posicion = [0] * 2
        self.posicion[1] = 25
        (matriz[0][25])[2] = -1
        self.rotacionMsec = 357 
        self.Nrotaciones = Nro
        self.start
        
        
robot.robotp(3000)

      
       
        
