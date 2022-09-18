#Import bibliotek
from asyncore import write
from cProfile import label
from cgitb import text
from curses import window
from distutils.cmd import Command
from email import message
from multiprocessing.connection import wait
from operator import truediv
from optparse import Values
from pickle import FALSE, GLOBAL, TRUE
from re import X
from sre_parse import State
import string
from tkinter import DISABLED, StringVar, ttk
from turtle import delay, forward, position, title, width
from pandas import DataFrame
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib.backend_bases import key_press_handler
import pigpio
import numpy as np
import time
import spidev
import tkinter
import os
import copy

class PWM:
   _MAX_GPIO=32

   def __init__(self, pi, frequency=1000):
      self.pi = pi

      self.frequency = frequency
      self.micros = 1000000.0 / frequency

      self.used = [False]*self._MAX_GPIO
      self.pS = [0.0]*self._MAX_GPIO
      self.pL = [0.0]*self._MAX_GPIO
      self.old_wid = None

      self.stop = False

   def set_frequency(self, frequency):
      self.frequency = float(frequency)
      self.micros = 1000000.0 / self.frequency

   def set_cycle_time(self, micros):
      self.micros = float(micros)
      self.frequency = 1000000.0 / self.micros

   def get_frequency(self):
      return self.frequency

   def get_cycle_length(self):
      return self.micros

   def set_pulse_length_in_micros(self, gpio, length):
      length %= self.micros

      self.pL[gpio] = length / self.micros

      if not self.used[gpio]:
         self.pi.set_mode(gpio, pigpio.OUTPUT)
         self.used[gpio] = True

   def set_pulse_length_in_fraction(self, gpio, length):
      self.set_pulse_length_in_micros(gpio, self.micros * length)

   def set_pulse_start_in_micros(self, gpio, start):
      start %= self.micros

      self.pS[gpio] = start / self.micros

      if not self.used[gpio]:
         self.pi.set_mode(gpio, pigpio.OUTPUT)
         self.used[gpio] = True

   def set_pulse_start_in_fraction(self, gpio, start):
      self.set_pulse_start_in_micros(gpio, self.micros * start)

   def set_pulse_start_and_length_in_micros(self, gpio, start, length):
      self.set_pulse_start_in_micros(gpio, start)
      self.set_pulse_length_in_micros(gpio, length)

   def set_pulse_start_and_length_in_fraction(self, gpio, start, length):
      self.set_pulse_start_in_fraction(gpio, start)
      self.set_pulse_length_in_fraction(gpio, length)

   def get_GPIO_settings(self, gpio):
      if self.used[gpio]:
         return (True, self.pS[gpio], self.pL[gpio])
      else:
         return (False, 0.0, 0.0)

   def update(self):

      null_wave = True

      for g in range(self._MAX_GPIO):

         if self.used[g]:

            null_wave = False

            on = int(self.pS[g] * self.micros)
            length = int(self.pL[g] * self.micros)

            micros = int(self.micros)

            if length <= 0:
               self.pi.wave_add_generic([pigpio.pulse(0, 1<<g, micros)])
            elif length >= micros:
               self.pi.wave_add_generic([pigpio.pulse(1<<g, 0, micros)])
            else:
               off = (on + length) % micros
               if on < off:
                  self.pi.wave_add_generic([
                     pigpio.pulse(   0, 1<<g,           on),
                     pigpio.pulse(1<<g,    0,     off - on),
                     pigpio.pulse(   0, 1<<g, micros - off),
                     ])
               else:
                  self.pi.wave_add_generic([
                     pigpio.pulse(1<<g,    0,         off),
                     pigpio.pulse(   0, 1<<g,    on - off),
                     pigpio.pulse(1<<g,    0, micros - on),
                     ])

      if not null_wave:
         if not self.stop:
            new_wid = self.pi.wave_create()
            if self.old_wid is not None:

               self.pi.wave_send_using_mode(
                  new_wid, pigpio.WAVE_MODE_REPEAT_SYNC)
               while self.pi.wave_tx_at() != new_wid:

                  pass
               self.pi.wave_delete(self.old_wid)

            else:

               self.pi.wave_send_repeat(new_wid)

            self.old_wid = new_wid

   def cancel(self):
      self.stop = True

      self.pi.wave_tx_stop()

      if self.old_wid is not None:
         self.pi.wave_delete(self.old_wid)

#Zmienne
global current_value, force_value, number_of_digit_entry_prad, value_read, number_of_digit_entry_czaspoczatkowy, number_of_digit_entry_czasimpulsu, number_of_digit_entry_czaskoncowy, number_of_digit_entry_czaspomiedzy
global first_clear_prad, first_clear_czaspoczatkowy, first_clear_czasimpulsu, first_clear_czaspomiedzy, first_clear_czaskoncowy, first_clear_number_of_pulses, number_of_digit_number_of_pulses
global time_of_process, start_time, end_time, impulse_time, between_impulse_time, amount_of_pulses, elapsed_time_var, pulses, number_of_digit_entry_sila_nacisku, first_clear_sila_nacisku, bool_current_on
global break_loop
break_loop = False
first_clear_prad = True
first_clear_czaspoczatkowy = True
first_clear_czasimpulsu = True
first_clear_czaspomiedzy = True
first_clear_czaskoncowy = True
first_clear_number_of_pulses = True
first_clear_sila_nacisku = True
bool_current_on = 1
number_of_digit_entry_czaspoczatkowy = 0
number_of_digit_entry_czasimpulsu = 0
number_of_digit_entry_czaskoncowy = 0
number_of_digit_entry_czaspomiedzy = 0
number_of_digit_entry_prad = 0
number_of_digit_number_of_pulses = 0
number_of_digit_entry_sila_nacisku = 0
current_value = '1000'
force_value = '0 - 300'
time_of_process = 0
start_time = 'Czas początkowy'
end_time = 'Czas końcowy'
impulse_time = 'Czas impulsu'
between_impulse_time = 'Czas pomiędzy imp.'
amount_of_pulses = '2'

#Wyświetlacz LED
if os.environ.get('DISPLAY','') == '':
    os.environ.__setitem__('DISPLAY', ':0.0')

#Włączenie PWM
pi = pigpio.pi()
#Włączenie komunikacji SPI
spi = spidev.SpiDev(0,0)
spi.max_speed_hz = 	250000

#Deklarowane zmienne
plot1_x = list()
plot1_y = list()
data2 = {'Czas [ms]': plot1_x,
         'Prąd [kA]': plot1_y
    }
df2 = DataFrame(data2,columns=['Czas [ms]','Prąd [kA]'])

#Konfiguracja klawiatury
row1 = 27
row2 = 3
row3 = 24
row4 = 21
column1 = 23
column2 = 26
column3 = 20

pi.set_mode(row1, pigpio.OUTPUT)
pi.set_mode(row2, pigpio.OUTPUT)
pi.set_mode(row3, pigpio.OUTPUT)
pi.set_mode(row4, pigpio.OUTPUT)
pi.set_mode(column1, pigpio.INPUT)
pi.set_mode(column2, pigpio.INPUT)
pi.set_mode(column3, pigpio.INPUT)

#Konfiguracja timerów
pwm = PWM(pi)
fill_factor_pwm_1 = 0   #wsp. wypełnienia PWM0
fill_factor_pwm_2 = 0   #wsp. wypełnienia PWM1

#Konfiguracja pozostałych PINów
dac_2_0 = 16
dac_2_1 = 19
dac_2_2 = 5
pi.set_mode(dac_2_0, pigpio.OUTPUT)   #DAC 2^0
pi.set_mode(dac_2_1, pigpio.OUTPUT)   #DAC 2^1
pi.set_mode(dac_2_2, pigpio.OUTPUT)   #DAC 2^2

#Wyprowadzenia na STM32
pressure_sensor = 4             #LE8 - 8
delete_errors_pin = 6           #LE6 - 7

pi.set_mode(pressure_sensor, pigpio.OUTPUT)
pi.set_mode(delete_errors_pin, pigpio.OUTPUT)

#Tworzenie Layoutu GUI-----------------------------------------------------------------------
window_app = tkinter.Tk()
window_app.title("Program do obsługi zgrzewarki")
window_app.geometry("800x400")
#--------------------------------------------------------------------------------------------
#Wywoływanie zdarzeń
def close():
    pi.write(row4,1)
    if(pi.read(column1)==True):
        try:
            #Zakończenie generowania PWM
            pwm.cancel()
            pi.write(pressure_sensor,0)
            #Zakończenie komunikacji przez SPI
            spi.close()
            window_app.destroy()
            global break_loop
            break_loop = True
        except:
            pass
    pi.write(row4,0)


#Wpisywanie wartości liczbowych w odpowiednie miejsca
def keypad_4x4_window_app():
    global number_of_digit_entry_prad, first_clear_prad, number_of_digit_entry_sila_nacisku, first_clear_sila_nacisku
    try:
        widget = window_app.focus_get()

        #Czyszczenie okna przy pierwszym wejsciu w nie
        if(widget == entry_prad and first_clear_prad == True):
            entry_prad.delete(0,'end')
            first_clear_prad = False
        if(widget == entry_sila_nacisku and first_clear_sila_nacisku == True):
            entry_sila_nacisku.delete(0,'end')
            first_clear_sila_nacisku = False

        #Wpisywanie cyfr w odpowiednie miejsca
        pi.write(row1,1)
        if(pi.read(column1)==True):
            if(widget == entry_prad):
                entry_prad.insert(number_of_digit_entry_prad,'1')
                number_of_digit_entry_prad = number_of_digit_entry_prad + 1
            if(widget == entry_sila_nacisku):
                entry_sila_nacisku.insert(number_of_digit_entry_sila_nacisku,'1')
                number_of_digit_entry_sila_nacisku = number_of_digit_entry_sila_nacisku + 1
        if(pi.read(column2)==True):
            if(widget == entry_prad):
                entry_prad.insert(number_of_digit_entry_prad,'2')
                number_of_digit_entry_prad = number_of_digit_entry_prad + 1
            if(widget == entry_sila_nacisku):
                entry_sila_nacisku.insert(number_of_digit_entry_sila_nacisku,'2')
                number_of_digit_entry_sila_nacisku = number_of_digit_entry_sila_nacisku + 1
        if(pi.read(column3)==True):
            if(widget == entry_prad):
                entry_prad.insert(number_of_digit_entry_prad,'3')
                number_of_digit_entry_prad = number_of_digit_entry_prad + 1
            if(widget == entry_sila_nacisku):
                entry_sila_nacisku.insert(number_of_digit_entry_sila_nacisku,'3')
                number_of_digit_entry_sila_nacisku = number_of_digit_entry_sila_nacisku + 1

        pi.write(row1,0)
        pi.write(row2,1)
        if(pi.read(column1)==True):
            if(widget == entry_prad):
                entry_prad.insert(number_of_digit_entry_prad,'4')
                number_of_digit_entry_prad = number_of_digit_entry_prad + 1
            if(widget == entry_sila_nacisku):
                entry_sila_nacisku.insert(number_of_digit_entry_sila_nacisku,'4')
                number_of_digit_entry_sila_nacisku = number_of_digit_entry_sila_nacisku + 1
        if(pi.read(column2)==True):
            if(widget == entry_prad):
                entry_prad.insert(number_of_digit_entry_prad,'5')
                number_of_digit_entry_prad = number_of_digit_entry_prad + 1
            if(widget == entry_sila_nacisku):
                entry_sila_nacisku.insert(number_of_digit_entry_sila_nacisku,'5')
                number_of_digit_entry_sila_nacisku = number_of_digit_entry_sila_nacisku + 1
        if(pi.read(column3)==True):
            if(widget == entry_prad):
                entry_prad.insert(number_of_digit_entry_prad,'6')
                number_of_digit_entry_prad = number_of_digit_entry_prad + 1
            if(widget == entry_sila_nacisku):
                entry_sila_nacisku.insert(number_of_digit_entry_sila_nacisku,'6')
                number_of_digit_entry_sila_nacisku = number_of_digit_entry_sila_nacisku + 1

        pi.write(row2,0)
        pi.write(row3,1)    
        if(pi.read(column1)==True):
            if(widget == entry_prad):
                entry_prad.insert(number_of_digit_entry_prad,'7')
                number_of_digit_entry_prad = number_of_digit_entry_prad + 1
            if(widget == entry_sila_nacisku):
                entry_sila_nacisku.insert(number_of_digit_entry_sila_nacisku,'7')
                number_of_digit_entry_sila_nacisku = number_of_digit_entry_sila_nacisku + 1
        if(pi.read(column2)==True):
            if(widget == entry_prad):
                entry_prad.insert(number_of_digit_entry_prad,'8')
                number_of_digit_entry_prad = number_of_digit_entry_prad + 1
            if(widget == entry_sila_nacisku):
                entry_sila_nacisku.insert(number_of_digit_entry_sila_nacisku,'8')
                number_of_digit_entry_sila_nacisku = number_of_digit_entry_sila_nacisku + 1
        if(pi.read(column3)==True):
            if(widget == entry_prad):
                entry_prad.insert(number_of_digit_entry_prad,'9')
                number_of_digit_entry_prad = number_of_digit_entry_prad + 1
            if(widget == entry_sila_nacisku):
                entry_sila_nacisku.insert(number_of_digit_entry_sila_nacisku,'9')
                number_of_digit_entry_sila_nacisku = number_of_digit_entry_sila_nacisku + 1

        pi.write(row3,0)
        pi.write(row4,1)
        if(pi.read(column1)==True):
            pi.write(delete_errors_pin,1)
            time.sleep(0.1)
            pi.write(delete_errors_pin,0)
        if(pi.read(column2)==True):
            if(widget == entry_prad):
                entry_prad.insert(number_of_digit_entry_prad,'0')
                number_of_digit_entry_prad = number_of_digit_entry_prad + 1
            if(widget == entry_sila_nacisku):
                entry_sila_nacisku.insert(number_of_digit_entry_sila_nacisku,'0')
                number_of_digit_entry_sila_nacisku = number_of_digit_entry_sila_nacisku + 1
        if(pi.read(column3)==True):
            if(widget == entry_prad):
                entry_prad.delete(number_of_digit_entry_prad-1,'end')
                number_of_digit_entry_prad = number_of_digit_entry_prad -1
            if(widget == entry_sila_nacisku):
                entry_sila_nacisku.delete(number_of_digit_entry_sila_nacisku-1,'end')
                number_of_digit_entry_sila_nacisku = number_of_digit_entry_sila_nacisku -1
        pi.write(row4,0)
        #Spowodowanie aby klawiatura działała non-stop
        window_app.after(300, keypad_4x4_window_app)
    except:
        pass

def read_values():
    global current_value, force_value
    if(entry_prad.get()=='' or entry_sila_nacisku.get()=='' or entry_sila_nacisku.get()=='0 - 300' or int(entry_prad.get())>=15000 or int(entry_prad.get())<1000 or int(entry_sila_nacisku.get())>=300 or int(entry_sila_nacisku.get())==0):
        error_msg = tkinter.messagebox.showerror(parent=window_app ,title='Błąd',message='Wpisz odpowiednie wartości siły i prądu.')
        window_app.after(300, keypad_4x4_window_app)
    else:
        current_value = entry_prad.get()
        force_value = entry_sila_nacisku.get()
        entry_prad.config(state=tkinter.DISABLED)
        entry_sila_nacisku.config(state=tkinter.DISABLED)
        button_zatwierdz.config(state=tkinter.DISABLED)
        button_dalej.config(state=tkinter.NORMAL)

def reset_all():
    global number_of_digit_entry_prad, start_time, end_time, impulse_time, between_impulse_time, first_clear_prad, first_clear_sila_nacisku, first_clear_czaspoczatkowy, first_clear_czasimpulsu, first_clear_czaspomiedzy, first_clear_czaskoncowy, first_clear_number_of_pulses
    number_of_digit_entry_prad = 0

    #Reset stanów przycisków i pól do wpisywania danych
    button_pomiar.config(state=tkinter.DISABLED)
    button_zatwierdz.config(state=tkinter.NORMAL)
    button_dalej.config(state=tkinter.DISABLED)
    entry_prad.config(state=tkinter.NORMAL)
    entry_sila_nacisku.config(state=tkinter.NORMAL)

    #Reset pól do wpisywania danych
    start_time = 'Czas początkowy'
    end_time = 'Czas końcowy'
    impulse_time = 'Czas impulsu'
    between_impulse_time = 'Czas pomiędzy imp.'
    first_clear_prad = True
    first_clear_sila_nacisku = True
    first_clear_czaspoczatkowy = True
    first_clear_czasimpulsu = True
    first_clear_czaspomiedzy = True
    first_clear_czaskoncowy = True
    first_clear_number_of_pulses = True

def process_new():
    global number_of_digit_entry_prad, start_time, end_time, impulse_time, between_impulse_time, first_clear_prad, first_clear_sila_nacisku, first_clear_czaspoczatkowy, first_clear_czasimpulsu, first_clear_czaspomiedzy, first_clear_czaskoncowy, first_clear_number_of_pulses
    number_of_digit_entry_prad = 0

    #Reset stanów przycisków i pól do wpisywania danych
    button_pomiar.config(state=tkinter.DISABLED)
    button_zatwierdz.config(state=tkinter.NORMAL)
    button_dalej.config(state=tkinter.DISABLED)
    entry_prad.config(state=tkinter.NORMAL)
    entry_sila_nacisku.config(state=tkinter.NORMAL)

    #Reset pól do wpisywania danych
    first_clear_prad = True
    first_clear_sila_nacisku = True
    first_clear_czaspoczatkowy = True
    first_clear_czasimpulsu = True
    first_clear_czaspomiedzy = True
    first_clear_czaskoncowy = True
    first_clear_number_of_pulses = True

def start_process():
    #Deklaracja zmiennych
    global time_of_process, start_time, end_time, impulse_time, between_impulse_time, amount_of_pulses, break_loop
    pwm = PWM(pi)
    plot1_x = list()
    plot1_y = list()
    voltage_values = list()
    vref = 3.3
    current_val = int(entry_prad.get())
    force_val = (int(entry_sila_nacisku.get()))     #zakres 0-300 [kG]
    i=0
    j=0
    first_cycle = True
    second_cycle = True
    fill_factor_pwm_1p = 0
    fill_factor_pwm_2p = 0
    fill_factor_pwm_1_last = 0
    fill_factor_pwm_2_last = 0
    error = 0
    prev_error = 0
    sum_errors = 0

    #Nastawy regulatora
    k_proportional = 0.00005
    k_derivative = 0.000001
    k_integral = 0.000005
    simulated_resistance = 0
    simulated_current = 0
    avarage_value = 0
    
    lower_limit_adc0 = 0.6           # 0 [A]
    upper_limit_adc0 = 3.15          # 15 [kA]
    limit_adc = upper_limit_adc0 - lower_limit_adc0

    if(int(bool_current_on) == 1):
        #Czekanie na zaciśnięcie szczęk
        pi.write(pressure_sensor,1)
        time.sleep(1)

        #Obliczanie siły nacisku i ustawianie wyjść (skok siły nacisku co 50 [kG])
        if(force_val==0):
            pi.write(dac_2_2,0)
            pi.write(dac_2_1,0)
            pi.write(dac_2_0,0)
        if(force_val<43 and force_val>0):
            pi.write(dac_2_2,0)
            pi.write(dac_2_1,0)
            pi.write(dac_2_0,1)
        if(force_val>=43 and force_val<86):
            pi.write(dac_2_2,0)
            pi.write(dac_2_1,1)
            pi.write(dac_2_0,0)
        if(force_val>=86 and force_val<129):
            pi.write(dac_2_2,0)
            pi.write(dac_2_1,1)
            pi.write(dac_2_0,1)
        if(force_val>=129 and force_val<172):
            pi.write(dac_2_2,1)
            pi.write(dac_2_1,0)
            pi.write(dac_2_0,0)
        if(force_val>=172 and force_val<215):
            pi.write(dac_2_2,1)
            pi.write(dac_2_1,0)
            pi.write(dac_2_0,1)
        if(force_val>=215 and force_val<258):
            pi.write(dac_2_2,1)
            pi.write(dac_2_1,1)
            pi.write(dac_2_0,0)
        if(force_val>=258):
            pi.write(dac_2_2,1)
            pi.write(dac_2_1,1)
            pi.write(dac_2_0,1)

        pwm.set_frequency(10000)
        start_time_process = time.time()
        elapsed_time = round(time.time() - start_time_process,3)
        while elapsed_time < time_of_process:
            close()
            if(break_loop == True):
                break_loop = False
                break
        #Odczytywanie wartości z ADC
            #Kanał 0
            byte_1 = 0b00000001
            byte_2 = 0b10000000
            byte_3 = 0b00000000
            msg = [byte_1, byte_2, byte_3]
            reply_ch0 = spi.xfer2(msg)

            adc_value = 0
            if(reply_ch0[1]>=8):
                reply_ch0[1] = reply_ch0[1] - 8
                adc_value = 2048
            if(reply_ch0[1]>=4):
                reply_ch0[1] = reply_ch0[1] - 4
                adc_value = adc_value + 1024
            if(reply_ch0[1]>=2):
                reply_ch0[1] = reply_ch0[1] - 2
                adc_value = adc_value + 512
            if(reply_ch0[1]>=1):
                reply_ch0[1] = reply_ch0[1] - 1
                adc_value = adc_value + 256
            
            adc_value = adc_value + reply_ch0[2]
            read_voltage_ch0 = (adc_value*vref)/4096
            if (i<=10):
                voltage_values.append((current_val/15000)*3.3)
                read_voltage_ch0 = 0.01

            #if(read_voltage_ch0<=0.1 and i>10):
            #    continue

            if (read_voltage_ch0>0.1 and i>10):
                voltage_values.append(read_voltage_ch0)
                avarage_value = 0
                avarage_value = np.average(voltage_values[-10:])

                if(read_voltage_ch0<=(avarage_value*0.9)):
                    read_voltage_ch0 = avarage_value

            read_voltage_ch0 = read_voltage_ch0-lower_limit_adc0+(vref-upper_limit_adc0)

            #Kanał 1
            byte_1 = 0b00000001
            byte_2 = 0b11000000
            byte_3 = 0b00000000
            msg = [byte_1, byte_2, byte_3]
            reply_ch1 = spi.xfer2(msg)

            adc_value = 0
            if(reply_ch1[1]>=8):
                reply_ch1[1] = reply_ch1[1] - 8
                adc_value = 2048
            if(reply_ch1[1]>=4):
                reply_ch1[1] = reply_ch1[1] - 4
                adc_value = adc_value + 1024
            if(reply_ch1[1]>=2):
                reply_ch1[1] = reply_ch1[1] - 2
                adc_value = adc_value + 512
            if(reply_ch1[1]>=1):
                reply_ch1[1] = reply_ch1[1] - 1
                adc_value = adc_value + 256
            
            adc_value = adc_value + reply_ch1[2]
            read_voltage_ch1 = (vref*adc_value)/4096

            #Regulator PID
            resistance = read_voltage_ch1/((read_voltage_ch0*15000)/3.3)
            if(i == 0):
                simulated_resistance = 1/current_val
            error = abs(read_voltage_ch1/current_val - resistance)
            adjustment = (error * k_proportional) + (sum_errors * k_integral) + (prev_error * k_derivative)
            if(resistance <= read_voltage_ch1/current_val):
                simulated_resistance = abs(simulated_resistance + adjustment)
            else:
                simulated_resistance = abs(simulated_resistance - adjustment)

            simulated_current = read_voltage_ch1/simulated_resistance

            if(elapsed_time>=(start_time/1000) and elapsed_time<=(start_time+impulse_time)/1000 and amount_of_pulses>=1 or 
            elapsed_time>=(start_time+impulse_time+between_impulse_time)/1000 and elapsed_time<=(start_time+2*impulse_time+between_impulse_time)/1000 and amount_of_pulses>=2 or
            elapsed_time>=(start_time+2*(impulse_time+between_impulse_time))/1000 and elapsed_time<=(start_time+ 3*impulse_time+2*between_impulse_time)/1000 and amount_of_pulses>=3):
                #Przeliczenie wartości odczytanej na wsp. wypełnienia
                fill_factor_pwm_1 = round((0.47*(current_val/15000)),3)
                fill_factor_pwm_2 = round((0.47*(current_val/15000)),3)
                #fill_factor_pwm_1 = round((0.47*(simulated_current/15000)),3)
                #fill_factor_pwm_2 = round((0.47*(simulated_current/15000)),3)
                simulated_value = round((read_voltage_ch0*15000/3.3), 3)
                plot1_y.append(simulated_value)

                #Stopniowy narost wartości współczynnika wypełnienia
                if(first_cycle == True):
                    fill_factor_pwm_1p = fill_factor_pwm_1 / 4
                    fill_factor_pwm_2p = fill_factor_pwm_2 / 4

                if(second_cycle == True and first_cycle == False):
                    fill_factor_pwm_1p = fill_factor_pwm_1 / 2
                    fill_factor_pwm_2p = fill_factor_pwm_2 / 2

                #Zebezpieczenie przed przekroczeniem zakresu górnego i dolnego
                if(fill_factor_pwm_1 >= 0.47):
                    fill_factor_pwm_1 = 0.47
                if(fill_factor_pwm_2 >= 0.47):
                    fill_factor_pwm_2 = 0.47  
                if(fill_factor_pwm_1 <= 0.03):
                    fill_factor_pwm_1 = 0.03     
                if(fill_factor_pwm_2 <= 0.03):
                    fill_factor_pwm_2 = 0.03   
            else:
                fill_factor_pwm_1 = 0 
                fill_factor_pwm_2 = 0
                w_symulowane = 0
                plot1_y.append(w_symulowane)

            #Ustawianie wyjść timerów
            if(first_cycle == False and second_cycle == False and fill_factor_pwm_1!=fill_factor_pwm_1_last and fill_factor_pwm_2!=fill_factor_pwm_2_last):
                pwm.set_pulse_start_and_length_in_fraction(12, 0.03, fill_factor_pwm_1)
                pwm.set_pulse_start_and_length_in_fraction(13, 0.53, fill_factor_pwm_2)
                pwm.update()
                fill_factor_pwm_1_last = fill_factor_pwm_1
                fill_factor_pwm_2_last = fill_factor_pwm_2
            if(first_cycle == True or second_cycle == True):
                pwm.set_pulse_start_and_length_in_fraction(12, 0.03, fill_factor_pwm_1p)
                pwm.set_pulse_start_and_length_in_fraction(13, 0.53, fill_factor_pwm_2p)
                pwm.update() 
            if(first_cycle == False):
                second_cycle = False
            first_cycle = False
            sum_errors = sum_errors + error
            prev_error = error  #PID całka
            i = i + 1
            elapsed_time = round(time.time() - start_time_process,3)
            plot1_x.append(elapsed_time)

        pi.write(dac_2_2,0)
        pi.write(dac_2_1,0)
        pi.write(dac_2_0,0)

        pwm.set_frequency(1)
        pwm.set_pulse_start_and_length_in_fraction(12, 0, 0)
        pwm.set_pulse_start_and_length_in_fraction(13, 0, 0)
        pwm.cancel()
        pi.write(12,0)
        pi.write(13,0)

        pi.write(pressure_sensor,0)

        ax2.cla()
        ax2.set_ylabel('Prąd [A]')
        ax2.set_xlabel('Czas [ms]')
        data2 = {'Czas [ms]': plot1_x,
            'Prąd [kA]': plot1_y
            }
        df2 = DataFrame(data2,columns=['Czas [ms]','Prąd [kA]'])
        df2 = df2[['Czas [ms]','Prąd [kA]']].groupby('Czas [ms]').sum()
        df2.plot(kind='line', legend=False, ax=ax2, color='r', fontsize=10).grid()
        figure1.canvas.draw()

    #Zgrzewanie bez prądu
    else:
        pi.write(pressure_sensor,1)
        time.sleep(1)

        #Obliczanie siły nacisku i ustawianie wyjść (skok siły nacisku co 50 [kG])
        if(force_val==0):
            pi.write(dac_2_2,0)
            pi.write(dac_2_1,0)
            pi.write(dac_2_0,0)
        if(force_val<43 and force_val>0):
            pi.write(dac_2_2,0)
            pi.write(dac_2_1,0)
            pi.write(dac_2_0,1)
        if(force_val>=43 and force_val<86):
            pi.write(dac_2_2,0)
            pi.write(dac_2_1,1)
            pi.write(dac_2_0,0)
        if(force_val>=86 and force_val<129):
            pi.write(dac_2_2,0)
            pi.write(dac_2_1,1)
            pi.write(dac_2_0,1)
        if(force_val>=129 and force_val<172):
            pi.write(dac_2_2,1)
            pi.write(dac_2_1,0)
            pi.write(dac_2_0,0)
        if(force_val>=172 and force_val<215):
            pi.write(dac_2_2,1)
            pi.write(dac_2_1,0)
            pi.write(dac_2_0,1)
        if(force_val>=215 and force_val<258):
            pi.write(dac_2_2,1)
            pi.write(dac_2_1,1)
            pi.write(dac_2_0,0)
        if(force_val>=258):
            pi.write(dac_2_2,1)
            pi.write(dac_2_1,1)
            pi.write(dac_2_0,1)
        print('Rozpoczęto proces')
        start_time_process = time.time()
        elapsed_time = round(time.time() - start_time_process,3)
        while elapsed_time < time_of_process:
            close()
            if(break_loop == True):
                break_loop = False
                break
            elapsed_time = round(time.time() - start_time_process,3)
        pi.write(pressure_sensor,0)
        ax2.cla()
        print('Zakończono po: ',elapsed_time)

    #Reset GUI przed kolejnym cyklem
    process_new()

#Budowanie okna pomocniczego
def dalej():
    #Zmienne
    current_welding_bool = StringVar()
    current_value_var = StringVar()
    force_value_var = StringVar()

    #Przypisania zmiennych
    current_welding_bool.set(bool_current_on)
    current_value_var.set(current_value)
    force_value_var.set(force_value)
    button_pomiar.config(state=tkinter.NORMAL)

    window_next = tkinter.Toplevel()
    window_next.geometry('800x400')

    def keypad_4x4_window_next():
        global number_of_digit_entry_czaspoczatkowy, number_of_digit_entry_czasimpulsu, number_of_digit_entry_czaskoncowy, number_of_digit_entry_czaspomiedzy, pulses, number_of_digit_number_of_pulses
        global first_clear_czaspoczatkowy, first_clear_czasimpulsu, first_clear_czaspomiedzy, first_clear_czaskoncowy, first_clear_number_of_pulses
        try:
            widget = window_next.focus_get()

            #Czyszczenie przy pierwszym wejsciu w okno wpisywania wartosci
            if(widget == entry_czaspoczatkowy and first_clear_czaspoczatkowy == True):
                entry_czaspoczatkowy.delete(0,'end')
                first_clear_czaspoczatkowy = False
            if(widget == entry_czasimpulsu and first_clear_czasimpulsu == True):
                entry_czasimpulsu.delete(0,'end')
                first_clear_czasimpulsu = False
            if(widget == entry_czaspomiedzy and first_clear_czaspomiedzy == True):
                entry_czaspomiedzy.delete(0,'end')
                first_clear_czaspomiedzy = False
            if(widget == entry_czaskoncowy and first_clear_czaskoncowy == True):
                entry_czaskoncowy.delete(0,'end')
                first_clear_czaskoncowy = False
            if(widget == number_of_pulses and first_clear_number_of_pulses == True):
                number_of_pulses.delete(0,'end')
                first_clear_number_of_pulses = False

            #Sprawdzenie czy przekroczono więcej niż 3 impulsów
            if(widget != number_of_pulses):
                if(int(number_of_pulses.get())>=3):
                    number_of_pulses.delete(0,'end')
                    number_of_pulses.insert(0,3)

            #Wpisywanie cyfr w odpowiednie miejsca
            pi.write(row1,1)
            if(pi.read(column1)==True):
                if(widget == entry_czaspoczatkowy):
                    entry_czaspoczatkowy.insert(number_of_digit_entry_czaspoczatkowy,'1')
                    number_of_digit_entry_czaspoczatkowy = number_of_digit_entry_czaspoczatkowy + 1
                if(widget == entry_czasimpulsu):
                    entry_czasimpulsu.insert(number_of_digit_entry_czasimpulsu,'1')
                    number_of_digit_entry_czasimpulsu = number_of_digit_entry_czasimpulsu + 1
                if(widget == entry_czaspomiedzy):
                    entry_czaspomiedzy.insert(number_of_digit_entry_czaspomiedzy,'1')
                    number_of_digit_entry_czaspomiedzy = number_of_digit_entry_czaspomiedzy + 1
                if(widget == entry_czaskoncowy):
                    entry_czaskoncowy.insert(number_of_digit_entry_czaskoncowy,'1')
                    number_of_digit_entry_czaskoncowy = number_of_digit_entry_czaskoncowy + 1
                if(widget == number_of_pulses):
                    number_of_pulses.insert(number_of_digit_number_of_pulses,'1')
                    number_of_digit_number_of_pulses = number_of_digit_number_of_pulses + 1
            if(pi.read(column2)==True):
                if(widget == entry_czaspoczatkowy):
                    entry_czaspoczatkowy.insert(number_of_digit_entry_czaspoczatkowy,'2')
                    number_of_digit_entry_czaspoczatkowy = number_of_digit_entry_czaspoczatkowy + 1
                if(widget == entry_czasimpulsu):
                    entry_czasimpulsu.insert(number_of_digit_entry_czasimpulsu,'2')
                    number_of_digit_entry_czasimpulsu = number_of_digit_entry_czasimpulsu + 1
                if(widget == entry_czaspomiedzy):
                    entry_czaspomiedzy.insert(number_of_digit_entry_czaspomiedzy,'2')
                    number_of_digit_entry_czaspomiedzy = number_of_digit_entry_czaspomiedzy + 1
                if(widget == entry_czaskoncowy):
                    entry_czaskoncowy.insert(number_of_digit_entry_czaskoncowy,'2')
                    number_of_digit_entry_czaskoncowy = number_of_digit_entry_czaskoncowy + 1
                if(widget == number_of_pulses):
                    number_of_pulses.insert(number_of_digit_number_of_pulses,'2')
                    number_of_digit_number_of_pulses = number_of_digit_number_of_pulses + 1
            if(pi.read(column3)==True):
                if(widget == entry_czaspoczatkowy):
                    entry_czaspoczatkowy.insert(number_of_digit_entry_czaspoczatkowy,'3')
                    number_of_digit_entry_czaspoczatkowy = number_of_digit_entry_czaspoczatkowy + 1
                if(widget == entry_czasimpulsu):
                    entry_czasimpulsu.insert(number_of_digit_entry_czasimpulsu,'3')
                    number_of_digit_entry_czasimpulsu = number_of_digit_entry_czasimpulsu + 1
                if(widget == entry_czaspomiedzy):
                    entry_czaspomiedzy.insert(number_of_digit_entry_czaspomiedzy,'3')
                    number_of_digit_entry_czaspomiedzy = number_of_digit_entry_czaspomiedzy + 1
                if(widget == entry_czaskoncowy):
                    entry_czaskoncowy.insert(number_of_digit_entry_czaskoncowy,'3')
                    number_of_digit_entry_czaskoncowy = number_of_digit_entry_czaskoncowy + 1
                if(widget == number_of_pulses):
                    number_of_pulses.insert(number_of_digit_number_of_pulses,'3')
                    number_of_digit_number_of_pulses = number_of_digit_number_of_pulses + 1

            pi.write(row1,0)
            pi.write(row2,1)
            if(pi.read(column1)==True):
                if(widget == entry_czaspoczatkowy):
                    entry_czaspoczatkowy.insert(number_of_digit_entry_czaspoczatkowy,'4')
                    number_of_digit_entry_czaspoczatkowy = number_of_digit_entry_czaspoczatkowy + 1
                if(widget == entry_czasimpulsu):
                    entry_czasimpulsu.insert(number_of_digit_entry_czasimpulsu,'4')
                    number_of_digit_entry_czasimpulsu = number_of_digit_entry_czasimpulsu + 1
                if(widget == entry_czaspomiedzy):
                    entry_czaspomiedzy.insert(number_of_digit_entry_czaspomiedzy,'4')
                    number_of_digit_entry_czaspomiedzy = number_of_digit_entry_czaspomiedzy + 1
                if(widget == entry_czaskoncowy):
                    entry_czaskoncowy.insert(number_of_digit_entry_czaskoncowy,'4')
                    number_of_digit_entry_czaskoncowy = number_of_digit_entry_czaskoncowy + 1
                if(widget == number_of_pulses):
                    number_of_pulses.insert(number_of_digit_number_of_pulses,'4')
                    number_of_digit_number_of_pulses = number_of_digit_number_of_pulses + 1
            if(pi.read(column2)==True):
                if(widget == entry_czaspoczatkowy):
                    entry_czaspoczatkowy.insert(number_of_digit_entry_czaspoczatkowy,'5')
                    number_of_digit_entry_czaspoczatkowy = number_of_digit_entry_czaspoczatkowy + 1
                if(widget == entry_czasimpulsu):
                    entry_czasimpulsu.insert(number_of_digit_entry_czasimpulsu,'5')
                    number_of_digit_entry_czasimpulsu = number_of_digit_entry_czasimpulsu + 1
                if(widget == entry_czaspomiedzy):
                    entry_czaspomiedzy.insert(number_of_digit_entry_czaspomiedzy,'5')
                    number_of_digit_entry_czaspomiedzy = number_of_digit_entry_czaspomiedzy + 1
                if(widget == entry_czaskoncowy):
                    entry_czaskoncowy.insert(number_of_digit_entry_czaskoncowy,'5')
                    number_of_digit_entry_czaskoncowy = number_of_digit_entry_czaskoncowy + 1
                if(widget == number_of_pulses):
                    number_of_pulses.insert(number_of_digit_number_of_pulses,'5')
                    number_of_digit_number_of_pulses = number_of_digit_number_of_pulses + 1
            if(pi.read(column3)==True):
                if(widget == entry_czaspoczatkowy):
                    entry_czaspoczatkowy.insert(number_of_digit_entry_czaspoczatkowy,'6')
                    number_of_digit_entry_czaspoczatkowy = number_of_digit_entry_czaspoczatkowy + 1
                if(widget == entry_czasimpulsu):
                    entry_czasimpulsu.insert(number_of_digit_entry_czasimpulsu,'6')
                    number_of_digit_entry_czasimpulsu = number_of_digit_entry_czasimpulsu + 1
                if(widget == entry_czaspomiedzy):
                    entry_czaspomiedzy.insert(number_of_digit_entry_czaspomiedzy,'6')
                    number_of_digit_entry_czaspomiedzy = number_of_digit_entry_czaspomiedzy + 1
                if(widget == entry_czaskoncowy):
                    entry_czaskoncowy.insert(number_of_digit_entry_czaskoncowy,'6')
                    number_of_digit_entry_czaskoncowy = number_of_digit_entry_czaskoncowy + 1
                if(widget == number_of_pulses):
                    number_of_pulses.insert(number_of_digit_number_of_pulses,'6')
                    number_of_digit_number_of_pulses = number_of_digit_number_of_pulses + 1

            pi.write(row2,0)
            pi.write(row3,1)
            if(pi.read(column1)==True):
                if(widget == entry_czaspoczatkowy):
                    entry_czaspoczatkowy.insert(number_of_digit_entry_czaspoczatkowy,'7')
                    number_of_digit_entry_czaspoczatkowy = number_of_digit_entry_czaspoczatkowy + 1
                if(widget == entry_czasimpulsu):
                    entry_czasimpulsu.insert(number_of_digit_entry_czasimpulsu,'7')
                    number_of_digit_entry_czasimpulsu = number_of_digit_entry_czasimpulsu + 1
                if(widget == entry_czaspomiedzy):
                    entry_czaspomiedzy.insert(number_of_digit_entry_czaspomiedzy,'7')
                    number_of_digit_entry_czaspomiedzy = number_of_digit_entry_czaspomiedzy + 1
                if(widget == entry_czaskoncowy):
                    entry_czaskoncowy.insert(number_of_digit_entry_czaskoncowy,'7')
                    number_of_digit_entry_czaskoncowy = number_of_digit_entry_czaskoncowy + 1
                if(widget == number_of_pulses):
                    number_of_pulses.insert(number_of_digit_number_of_pulses,'7')
                    number_of_digit_number_of_pulses = number_of_digit_number_of_pulses + 1
            if(pi.read(column2)==True):
                if(widget == entry_czaspoczatkowy):
                    entry_czaspoczatkowy.insert(number_of_digit_entry_czaspoczatkowy,'8')
                    number_of_digit_entry_czaspoczatkowy = number_of_digit_entry_czaspoczatkowy + 1
                if(widget == entry_czasimpulsu):
                    entry_czasimpulsu.insert(number_of_digit_entry_czasimpulsu,'8')
                    number_of_digit_entry_czasimpulsu = number_of_digit_entry_czasimpulsu + 1
                if(widget == entry_czaspomiedzy):
                    entry_czaspomiedzy.insert(number_of_digit_entry_czaspomiedzy,'8')
                    number_of_digit_entry_czaspomiedzy = number_of_digit_entry_czaspomiedzy + 1
                if(widget == entry_czaskoncowy):
                    entry_czaskoncowy.insert(number_of_digit_entry_czaskoncowy,'8')
                    number_of_digit_entry_czaskoncowy = number_of_digit_entry_czaskoncowy + 1
                if(widget == number_of_pulses):
                    number_of_pulses.insert(number_of_digit_number_of_pulses,'8')
                    number_of_digit_number_of_pulses = number_of_digit_number_of_pulses + 1
            if(pi.read(column3)==True):
                if(widget == entry_czaspoczatkowy):
                    entry_czaspoczatkowy.insert(number_of_digit_entry_czaspoczatkowy,'9')
                    number_of_digit_entry_czaspoczatkowy = number_of_digit_entry_czaspoczatkowy + 1
                if(widget == entry_czasimpulsu):
                    entry_czasimpulsu.insert(number_of_digit_entry_czasimpulsu,'9')
                    number_of_digit_entry_czasimpulsu = number_of_digit_entry_czasimpulsu + 1
                if(widget == entry_czaspomiedzy):
                    entry_czaspomiedzy.insert(number_of_digit_entry_czaspomiedzy,'9')
                    number_of_digit_entry_czaspomiedzy = number_of_digit_entry_czaspomiedzy + 1
                if(widget == entry_czaskoncowy):
                    entry_czaskoncowy.insert(number_of_digit_entry_czaskoncowy,'9')
                    number_of_digit_entry_czaskoncowy = number_of_digit_entry_czaskoncowy + 1
                if(widget == number_of_pulses):
                    number_of_pulses.insert(number_of_digit_number_of_pulses,'9')
                    number_of_digit_number_of_pulses = number_of_digit_number_of_pulses + 1

            pi.write(row3,0)
            pi.write(row4,1)
            if(pi.read(column1)==True):
                pi.write(delete_errors_pin,1)
                time.sleep(0.1)
                pi.write(delete_errors_pin,0)
            if(pi.read(column2)==True):
                if(widget == entry_czaspoczatkowy):
                    entry_czaspoczatkowy.insert(number_of_digit_entry_czaspoczatkowy,'0')
                    number_of_digit_entry_czaspoczatkowy = number_of_digit_entry_czaspoczatkowy + 1
                if(widget == entry_czasimpulsu):
                    entry_czasimpulsu.insert(number_of_digit_entry_czasimpulsu,'0')
                    number_of_digit_entry_czasimpulsu = number_of_digit_entry_czasimpulsu + 1
                if(widget == entry_czaspomiedzy):
                    entry_czaspomiedzy.insert(number_of_digit_entry_czaspomiedzy,'0')
                    number_of_digit_entry_czaspomiedzy = number_of_digit_entry_czaspomiedzy + 1
                if(widget == entry_czaskoncowy):
                    entry_czaskoncowy.insert(number_of_digit_entry_czaskoncowy,'0')
                    number_of_digit_entry_czaskoncowy = number_of_digit_entry_czaskoncowy + 1
                if(widget == number_of_pulses):
                    number_of_pulses.insert(number_of_digit_number_of_pulses,'0')
                    number_of_digit_number_of_pulses = number_of_digit_number_of_pulses + 1
            if(pi.read(column3)==True):
                if(widget == entry_czaspoczatkowy):
                    entry_czaspoczatkowy.delete(number_of_digit_entry_czaspoczatkowy-1,'end')
                    number_of_digit_entry_czaspoczatkowy = number_of_digit_entry_czaspoczatkowy -1
                if(widget == entry_czasimpulsu):
                    entry_czasimpulsu.delete(number_of_digit_entry_czasimpulsu-1,'end')
                    number_of_digit_entry_czasimpulsu = number_of_digit_entry_czasimpulsu -1
                if(widget == entry_czaspomiedzy):
                    entry_czaspomiedzy.delete(number_of_digit_entry_czaspomiedzy-1,'end')
                    number_of_digit_entry_czaspomiedzy = number_of_digit_entry_czaspomiedzy -1
                if(widget == entry_czaskoncowy):
                    entry_czaskoncowy.delete(number_of_digit_entry_czaskoncowy-1,'end')
                    number_of_digit_entry_czaskoncowy = number_of_digit_entry_czaskoncowy -1
                if(widget == number_of_pulses):
                    number_of_pulses.delete(number_of_digit_number_of_pulses-1,'end')
                    number_of_digit_number_of_pulses = number_of_digit_number_of_pulses -1

            pi.write(row4,0)
            window_next.after(300, keypad_4x4_window_next)
        except:
            pass

    def close_dalej():
        global time_of_process, start_time, end_time, impulse_time, between_impulse_time, amount_of_pulses, elapsed_time_var, bool_current_on
        if(entry_czaspoczatkowy.get()=='' or entry_czasimpulsu.get()=='' or entry_czaspomiedzy.get()=='' or entry_czaskoncowy.get()=='' or number_of_pulses.get()=='' or
        entry_czaspoczatkowy.get()=='Czas początkowy' or entry_czasimpulsu.get()=='Czas impulsu' or entry_czaspomiedzy.get()=='Czas pomiędzy imp.' or entry_czaskoncowy.get()=='Czas końcowy' or
        entry_czasimpulsu.get()=='0' or int(entry_czaspoczatkowy.get())<=499 or int(entry_czaskoncowy.get())<=499 or entry_czaspomiedzy.get()=='0'):
            error_msg = tkinter.messagebox.showerror(parent=window_next ,title='Błąd',message='Uzupełnij wszystkie wartości czasów. Czas początkowy i końcowy musi być >= 500.')
            window_next.after(300, keypad_4x4_window_next)
        else:
            #Zczytanie danych do zmiennych
            bool_current_on = current_welding_bool.get()
            start_time = int(entry_czaspoczatkowy.get())
            end_time = int(entry_czaskoncowy.get())
            impulse_time = int(entry_czasimpulsu.get())
            between_impulse_time = int(entry_czaspomiedzy.get())
            amount_of_pulses = int(number_of_pulses.get())

            #Obliczenie czasu trwania procesu zgrzewania
            time_of_process = round((start_time + end_time + amount_of_pulses*(impulse_time + between_impulse_time)-between_impulse_time)/1000,3)
            elapsed_time_var.set(time_of_process)
            window_next.destroy()

    def plot_update():
        if(entry_czaspoczatkowy.get()=='' or entry_czasimpulsu.get()=='' or entry_czaspomiedzy.get()=='' or entry_czaskoncowy.get()=='' or number_of_pulses.get()=='' or 
        entry_czaspoczatkowy.get()=='Czas początkowy' or entry_czasimpulsu.get()=='Czas impulsu' or entry_czaspomiedzy.get()=='Czas pomiędzy imp.' or entry_czaskoncowy.get()=='Czas końcowy'):
            pass
        else:
            global current_value, force_value

            start_time = int(entry_czaspoczatkowy.get())/1000
            end_time = int(entry_czaskoncowy.get())/1000
            impulse_time = int(entry_czasimpulsu.get())/1000
            between_impulse_time = int(entry_czaspomiedzy.get())/1000
            amount_of_pulses = int(number_of_pulses.get())
            time_of_process = round(start_time + end_time + amount_of_pulses*(impulse_time + between_impulse_time),3)

            plot2_x_time1 = [0,(time_of_process-between_impulse_time)/15,(time_of_process-between_impulse_time)*(14/15),(time_of_process-between_impulse_time)]
            plot2_y_force = [0,int(force_value),int(force_value),0]
            raw_data_fig2_force = {'Czas [s]': plot2_x_time1,
                'Prąd [A]': plot2_y_force
                }

            if(amount_of_pulses == 1):
                plot2_x_time2 = [0,start_time,start_time+0.001,start_time+impulse_time-0.001, start_time+impulse_time ,start_time+impulse_time+end_time]
                plot2_y_current = [0,0.001,int(current_value)+0.001,int(current_value),0.001,0]
                raw_data_fig2_current = {'Czas [s]': plot2_x_time2,
                    'Prąd [A]': plot2_y_current
                    }
            if(amount_of_pulses == 2):
                plot2_x_time2 = [0,start_time,start_time+0.001,start_time+impulse_time-0.001, start_time+impulse_time, start_time+impulse_time+between_impulse_time,start_time+impulse_time+between_impulse_time + 0.001
                ,start_time+2*impulse_time+between_impulse_time,start_time+2*impulse_time+between_impulse_time+0.001,start_time+2*impulse_time+between_impulse_time + end_time]
                plot2_y_current = [0,0.001,int(current_value)+0.001,int(current_value),0.001,0,int(current_value),int(current_value)+0.001,0.001,0]
                raw_data_fig2_current = {'Czas [s]': plot2_x_time2,
                    'Prąd [A]': plot2_y_current
                    }
            if(amount_of_pulses == 3):
                plot2_x_time2 = [0,start_time,start_time+0.001,start_time+impulse_time-0.001, start_time+impulse_time, start_time+impulse_time+between_impulse_time,start_time+impulse_time+between_impulse_time + 0.001
                ,start_time+2*impulse_time+between_impulse_time,   start_time+2*impulse_time+between_impulse_time+0.001 ,start_time+2*impulse_time+2*between_impulse_time,start_time+2*impulse_time+2*between_impulse_time+0.001
                ,start_time+3*impulse_time+2*between_impulse_time, start_time+3*impulse_time+2*between_impulse_time+0.001,start_time+3*impulse_time+2*between_impulse_time+end_time]
                plot2_y_current = [0,0.001,int(current_value)+0.001,int(current_value),0.001,0,int(current_value),int(current_value)+0.001,0.001,0, int(current_value), int(current_value)+0.001,0.001,0]
                raw_data_fig2_current = {'Czas [s]': plot2_x_time2,
                    'Prąd [A]': plot2_y_current
                    }

            ax2.cla()
            ax2.set_xlabel('Czas [s]')

            figure2_rawdata = DataFrame(raw_data_fig2_force,columns=['Czas [s]','Prąd [A]'])
            figure2_rawdata = figure2_rawdata[['Czas [s]','Prąd [A]']].groupby('Czas [s]').sum()
            figure2_rawdata.plot(kind='line', legend=False, ax=ax2, color='r', fontsize=10).grid()
            figure2.canvas.draw()

            figure2_rawdata = DataFrame(raw_data_fig2_current,columns=['Czas [s]','Prąd [A]'])
            figure2_rawdata = figure2_rawdata[['Czas [s]','Prąd [A]']].groupby('Czas [s]').sum()
            figure2_rawdata.plot(kind='line', legend=False, ax=ax2, color='b', fontsize=10).grid()
            figure2.canvas.draw()

    figure2 = plt.Figure(figsize=(8,2), dpi=100, linewidth=1)
    figure2.subplots_adjust(bottom=0.2)
    ax2 = figure2.add_subplot(111)
    ax2.set_xlabel('Czas [s]')
    plt.ion()
    fig_canvas = FigureCanvasTkAgg(figure2, master=window_next)
    fig_canvas.get_tk_widget().place(x=0,y=0)
    fig_canvas.draw()
    plot2_x_time1 = [0]
    plot2_y_force = [0]
    raw_data_fig2_force = {'Czas [ms]': plot2_x_time1,
         'Prąd [kA]': plot2_y_force
        }
    figure2_rawdata = DataFrame(raw_data_fig2_force,columns=['Czas [ms]','Prąd [kA]'])
    figure2_rawdata = figure2_rawdata[['Czas [ms]','Prąd [kA]']].groupby('Czas [ms]').sum()
    figure2_rawdata.plot(kind='line', legend=False, ax=ax2, color='r', fontsize=10).grid()

    plot2_x_time2 = [0] 
    plot2_y_current = [0]
    raw_data_fig2_zgrzewanie = {'Czas [ms]': plot2_x_time2,
         'Prąd [kA]': plot2_y_current
        }
    figure2_rawdata2 = DataFrame(raw_data_fig2_zgrzewanie,columns=['Czas [ms]','Prąd [kA]'])
    figure2_rawdata2 = figure2_rawdata2[['Czas [ms]','Prąd [kA]']].groupby('Czas [ms]').sum()
    figure2_rawdata2.plot(kind='line', legend=False, ax=ax2, color='b', fontsize=10).grid()

    label_list_of_numbers = tkinter.Label(window_next, text="Liczba impulsów", width = 16, height = 2, relief=tkinter.SUNKEN, font=("Arial",12))
    label_list_of_numbers.place(x=20, y=210)

    number_of_pulses = tkinter.Entry(window_next, width=16, font=("Arial",12))
    number_of_pulses.insert(0,amount_of_pulses)
    number_of_pulses.place(x=20, y=260)

    label_list_of_numbers = tkinter.Label(window_next, text="Czasy:", width = 16, height = 2, relief=tkinter.SUNKEN, font=("Arial",12))
    label_list_of_numbers.place(x=180, y=210)

    entry_czaspoczatkowy=tkinter.Entry(window_next, width=16, font=("Arial",12))
    entry_czaspoczatkowy.insert(0,start_time)
    entry_czaspoczatkowy.place(x=180, y=260)

    entry_czasimpulsu=tkinter.Entry(window_next, width=16, font=("Arial",12))
    entry_czasimpulsu.insert(0,impulse_time)
    entry_czasimpulsu.place(x=180, y=295)

    entry_czaspomiedzy=tkinter.Entry(window_next, width=16, font=("Arial",12))
    entry_czaspomiedzy.insert(0,between_impulse_time)
    entry_czaspomiedzy.place(x=180, y=330)

    entry_czaskoncowy=tkinter.Entry(window_next, width=16, font=("Arial",12))
    entry_czaskoncowy.insert(0,end_time)
    entry_czaskoncowy.place(x=180, y=365)

    check_button_zgrzewanie = tkinter.Checkbutton(window_next, text = "Zgrzewać z prądem?", variable = current_welding_bool, font=("Arial",12))
    check_button_zgrzewanie.place(x=600, y=220)

    button_exit_dalej = tkinter.Button(window_next, text='Powrót', command=close_dalej, height=4, width=16,bg="red", activebackground="red", font=("Arial",12))
    button_exit_dalej.place(x=600,y=280)

    button_aktualizuj_wykres = tkinter.Button(window_next, text='Aktualizuj wykres', command=plot_update, height=2, width=14, font=("Arial",12))
    button_aktualizuj_wykres.place(x=360,y=320)

    label_prad = tkinter.Label(window_next, text="Natężenie prądu [A]:", width = 16, height = 2, relief=tkinter.SUNKEN, font=("Arial",12))
    label_prad.place(x=360, y=210)

    label_prad_wartosc = tkinter.Label(window_next, textvariable=current_value_var, width = 6, height = 2, font=("Arial",12))
    label_prad_wartosc.place(x=510, y=210)

    label_sila_nacisku = tkinter.Label(window_next, text="Siła nacisku [kG]:", width = 16, height = 2, relief=tkinter.SUNKEN, font=("Arial",12))
    label_sila_nacisku.place(x=360, y=265)

    label_sila_nacisku_wartosc = tkinter.Label(window_next, textvariable=force_value_var, width = 6, height = 2, font=("Arial",12))
    label_sila_nacisku_wartosc.place(x=510, y=265)

    keypad_4x4_window_next()    #Wywołanie działania klawiatury

#Elementy głównego okienka
elapsed_time_var = StringVar()
elapsed_time_var.set(time_of_process)

figure1 = plt.Figure(figsize=(8,2.5), dpi=100, linewidth=1)
ax2 = figure1.add_subplot(111)
ax2.set_ylabel('Prąd [A]')
ax2.set_xlabel('Czas [ms]')
plt.ion()
fig_canvas = FigureCanvasTkAgg(figure1, master=window_app)
fig_canvas.get_tk_widget().place(x=0,y=0)
fig_canvas.draw()
fig_toolbarframe = tkinter.Frame(master=window_app)
fig_toolbarframe.place(x=570,y=250)
fig_toolbar = NavigationToolbar2Tk(fig_canvas, fig_toolbarframe)

df2 = df2[['Czas [ms]','Prąd [kA]']].groupby('Czas [ms]').sum()
df2.plot(kind='line', legend=False, ax=ax2, color='r', fontsize=10)

label_prad = tkinter.Label(window_app, text = "Natężenie prądu [A]", width = 16, height = 2, relief=tkinter.SUNKEN, font=("Arial",12))
label_prad.place(x=20, y=255)

entry_prad=tkinter.Entry(window_app, width=16, font=("Arial",12))
entry_prad.insert(0,current_value)
entry_prad.place(x=20, y=305)

label_sila_nacisku = tkinter.Label(window_app, text="Siła Nacisku [kG]", width = 16, height = 2, relief=tkinter.SUNKEN, font=("Arial",12))
label_sila_nacisku.place(x=195, y=255)

label_czas_calkowity = tkinter.Label(window_app, text="Czas całkowity [s]:", width = 16, height = 2, relief=tkinter.SUNKEN, font=("Arial",12))
label_czas_calkowity.place(x=575, y=290)

label_czas_calkowity_wartosc = tkinter.Label(window_app, textvariable=elapsed_time_var, width = 6, height = 2, font=("Arial",12))
label_czas_calkowity_wartosc.place(x=725, y=290)

entry_sila_nacisku=tkinter.Entry(window_app, width=16, font=("Arial",12))
entry_sila_nacisku.insert(0,force_value)
entry_sila_nacisku.place(x=195, y=305)

button_zatwierdz = tkinter.Button(window_app, text= 'Zatwierdź wartości', command= read_values, height=2, width=33,font=("Arial",12))
button_zatwierdz.place(x=20, y=340)

button_pomiar= tkinter.Button(window_app, text= 'Start', command= start_process, height=2, width=12, state=tkinter.DISABLED, font=("Arial",12))
button_pomiar.place(x=390, y=255)

button_reset = tkinter.Button(window_app, text='Reset', command=reset_all, height=2, width=13, state=tkinter.NORMAL, font=("Arial",12))
button_reset.place(x = 575, y= 340)

#Przejście do okna pomocniczego
button_dalej = tkinter.Button(window_app, text='Dalej', command=dalej, height=2, state=tkinter.DISABLED, width=12,font=("Arial",12))
button_dalej.place(x=390, y=340)
#-------------------------------------------------------------------------------------------------------------------------------------------------
#Nieskończona pętla działania okna programowego
window_app.after(300, keypad_4x4_window_app)
window_app.mainloop()