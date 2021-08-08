#!/usr/bin/env python
# -*- coding: utf-8 -*-

class PositionPID:
    def __init__(self, p=0.0, i=0.0, d=0.0):
        self.__kp, self.__ki, self.__kd = p, i, d
        self.__error, self.__last_error = 0.0, 0.0
        self.__err_integral = 0.0
        self.__integral_min, self.__integral_max = -1000, 1000

    def __integral_limiter(self, integral):
        integral = self.__integral_max if integral > self.__integral_max else integral
        integral = self.__integral_min if integral < self.__integral_min else integral
        return integral

    def set_parameter(self, p, i, d):
        self.__kp, self.__ki, self.__kd = p, i, d

    def set_integral_limiter(self, iMin, iMax):
        self.__integral_min, self.__integral_max = iMin, iMax

    def clear_integral(self):
        self.__err_integral = 0

    def run(self, error):
        self.__error = error
        self.__err_integral = self.__err_integral + self.__error
        self.__err_integral = self.__integral_limiter(self.__err_integral)
        out = self.__kp*self.__error + self.__ki*self.__err_integral + self.__kd*(self.__error-self.__last_error)
        self.__last_error = self.__error
        return out
