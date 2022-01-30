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
            @param satLim is a list containing the upper and lower bounds of saturation, between -100 and 100    
        '''
        ## @brief Instantiates gains and setpoint
        
        ## Kp is the proportional gain
        self.Kp = Kp
        ## Kd is the derivitive gain
        self.Kd = Kd
        ## Kd is the integral gain
        self.Ki = Ki
        ## Setpoint is the goal point, the point which the motors attempts to reach
        self.setpoint = setpoint
        ## @brief Instantiates duty saturation upper and lower bounds
        self.satLim = satLim
        ## @brief Sum of error over a difference in time
        self.esum = 0
        ## @brief Previous error
        self.laste = 0
        ## A counter using ms
        self.Time = utime.ticks_ms
        ## A value that keeps the current time
        self.to = self.Time()
        ## A list that keeps track of the times
        self.times = []
        ## A list that keep tracks of the poisitons
        self.positions = []
        ## A flag that determines whether or not the function is recording, set to false or not recording
        self.recording = False

    def update (self, read, tdif):
        '''! @brief Constructs a closed loop controller and records the time and postion to the motor
            @details Sets saturation limits to what is determined by task_hardware and instantiates error variables.
            Uses a loop to record the position and time of the motor 100 times as it moves.
            Once it has finished the 100th recording, prints the values of all 100 positons and time.
            Resets the recording flag back to false, waiting for the next set
            @param read is the motor reading the new values from the encoder.
            @param tdif is the difference of time between recordings.
            @return Sends back actuation signal value using sat method.
        '''
        # Error signal which is the difference between the expected setpoint and the measured value [read].
        e = self.setpoint - read
        #Updates sum of error (area under curve)
        self.esum += (self.laste+e)*tdif/2
        #  Delta error calculated by taking difference in error values over a time difference
        dele = (e - self.laste)/tdif
        # Updates last error
        self.laste = e
        # Actuation signal (in duty cycle) calculation using gains and error values
        actuation_signal = self.Kp*(e) + self.Ki*(self.esum) + self.Kd*(dele)
        
        
       
        if(len(self.times) < 100 and self.recording):
            self.times.append(utime.ticks_diff(self.Time(),self.to))
            self.positions.append(read)
        elif (self.recording):
            self.print_values()
            self.recording = False
            print("#STOP#")
        return self.sat(actuation_signal)
                
    def sat(self,duty):
        '''! @brief Saturation functionallity
            @details Controls if a duty is too large from what is calculated in update method.
            @param duty is the value sent by what is calculated in update method.
            @return Sends back either the saturated limit if duty is too high or original duty based on bounds.
        '''
        if duty<self.satLim[0]:
            return self.satLim[0]
        elif duty>self.satLim[1]:
            return self.satLim[1]
        return duty


    
    
    def set_setpoint(self, point):
        '''! @brief Function that resets the setpoint and begins to record the current time.
            @details Also resets the lists for positions and times
        '''    
        self.setpoint = point
        self.to = self.Time()
        self.record()



    def set_control_gain(self, gain):
        '''! @brief Function that sets the controller gains
        ''' 
        self.Kp = gain
        
        
    
    def record(self):
        '''! @brief Function that rests the the values in the lists for positions and times
            @details Also changes the recording flag to true so that it allows the computer to begin recording values
        '''         
        self.times = []
        self.positions = []
        self.recording = True
        
        
   
    def print_values(self):
        '''! @brief Function that prints the values in the lists for positions and times
        '''         
        for i in range(len(self.times)):
            print(str(self.times[i]) + "," + str(self.positions[i]))