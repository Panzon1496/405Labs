'''!
    @file closedloop.py
    @brief Closed loop controller containing methods to control an arbitraries motor duty cycle
    @details Controller uses the difference of reference and current values to create an error variable. the time 
             difference, and magnitude of error are then used in the update method to return a duty for the motor to be run.
    @author John Bennett
    @author Daniel Munic
    @author Rodolfo Diaz
    
    @date   January 25, 2022
'''

import utime

class ClosedLoop:
    '''!@brief                  Interface with closed loop controller
        @details                Contains all methods that will be used in task_hardware to set the duty cycle based on closed
                                loop control.
    '''
        
    def __init__ (self, Kp, setpoint = 0, Ki = 0, Kd = 0, satLim = [-100,100]):
        '''!@brief Constructs a closed loop controller
            @details Sets saturation limits to what is determined by task_hardware and instantiates error variables.
            @param satLim is a list containing the upper and lower bounds of saturation      
        '''
        ## @brief Instantiates gains and setpoint
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.setpoint = setpoint
        # PID Kp(*%/rad)Ki(*%/rad)Kd(*%s2/rad)
        ## @brief Instantiates duty saturation upper and lower bounds
        self.satLim = satLim
        
        ## @brief Sum of error over a difference in time
        self.esum = 0
        ## @brief Previous error
        self.laste = 0
        
        '''@ creates an object that will keep track of time and the rate of time it takes to count each tick
        Also creates two lists, so that position and time can be measured and put into a graph in order
        to be able to see the PID controller in a visual sense. Create a recording object and initiate to false
        '''
        self.Time = utime.ticks_ms
        self.to = self.Time()
        self.times = []
        self.positions = []
        self.recording = False

    def update (self, read, tdif):
        ''' @brief Constructs a closed loop controller
            @details Sets saturation limits to what is determined by task_hardware and instantiates error variables.
            @param satLim is a list containing the upper and lower bounds of saturation    
            @return Sends back saturated duty value using sat method.
        '''
        ## @brief Error signal which is the difference between the expected setpoint and the measured value [read].
        e = self.setpoint - read
        #Updates sum of error (area under curve)
        self.esum += (self.laste+e)*tdif/2
        ## @brief Delta error calculated by taking difference in error values over a time difference
        dele = (e - self.laste)/tdif
        # Updates last error
        self.laste = e
        
        ## @brief Actuation signal (in duty cycle) calculation using gains and error values
        actuation_signal = self.Kp*(e) + self.Ki*(self.esum) + self.Kd*(dele)
        
        
        ''' A loop that that records the current time and position of the motor 100 times.
            After each recording, it adds the newest postion and time into the lists and once
            100 recordings have beeen achieved, it prints all the values of these lists then resets
            recording object back to false. Then prints the word STOP.
        '''
        if(len(self.times) < 100 and self.recording):
            self.times.append(utime.ticks_diff(self.Time(),self.to))
            self.positions.append(read)
        elif (self.recording):
            self.print_values()
            self.recording = False
            print("#STOP#")
        ## Returns the actuation signal which tells the motor how much duty it should give for the next cycle 
        return self.sat(actuation_signal)
                
    def sat(self,duty):
        ''' @brief Saturation functionallity
            @details Controls if a duty is too large from what is calculated in update method.
            @param duty is the value sent by what is calculated in update method.
            @return Sends back either the saturated limit if duty is too high or original duty based on bounds.
        '''
        if duty<self.satLim[0]:
            return self.satLim[0]
        elif duty>self.satLim[1]:
            return self.satLim[1]
        return duty


    ## Function that sets the current postion and time as the starting point and time,
    ## and then begins to record the new values.
    def set_setpoint(self, point):
        self.setpoint = point
        self.to = self.Time()
        self.record()

    ##  Function that set the gain in controller
    def set_control_gain(self, gain):
        self.Kp = gain
        
        
    ## Function that clears the values in the lists, time and position, and set the recording object to true
    ## which allows the code to begin recording the new values of the positon and time
    def record(self):
        self.times = []
        self.positions = []
        self.recording = True
        
        
    ##Function that prints the values of the two lists, postion and time
    def print_values(self):
        for i in range(len(self.times)):
            print(str(self.times[i]) + "," + str(self.positions[i]))