from time import sleep
from math import log
from pynq.lib.pmod import PMOD_GROVE_G3
from pynq.lib.pmod import PMOD_GROVE_G4
from pynq.lib import Pmod_IIC

class AHT20(Pmod_IIC):
    """This class controls the grove AHT20 humidity/temperature sensor.
    
    This class inherits from the PMODIIC class.
    
    Attributes
    ----------
    iop : _IOP
        The _IOP object returned from the DevMode.
    scl_pin : int
        The SCL pin number.
    sda_pin : int
        The SDA pin number.
    iic_addr : int
        The IIC device address.
    
    """
    def __init__(self, pmod_id, gr_pins): 
        """Return a new instance of a grove AHT20 object. 
        
        Parameters
        ----------
        pmod_id : int
            The PMOD ID (1, 2) corresponding to (PMODA, PMODB).
        gr_pins: list
            The group pins on Grove Adapter. G3 or G4 is valid.
        model : string
            Temperature sensor model (can be found on the device).
            
        """
        if gr_pins in [PMOD_GROVE_G3, PMOD_GROVE_G4]:
            [scl_pin,sda_pin] = gr_pins
        else:
            raise ValueError("Valid group numbers are G3 and G4.")
            
        super().__init__(pmod_id, scl_pin, sda_pin, 0x38)
        
        status = self.get_status()[0]
        if(not(bin(status)[-4])):
            raise ValueError("Device not calibrated")
            
        self.initialise()

    def get_status(self):
        """Read the device state.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        byte
            bit[7]   - Busy Indication
                          1 - Busy in measurement 
                          0 - Free in dormant state
            bit[6:5] - Mode Status
                         00 - NOR mode
                         01 - CYC mode
                         1x - CMD mode
            bit[4]   - Remained
            bit[3]   - Cal Enable
                          1 - Calibrated
                          0 - Uncalibrated
            bit[2:0] - Remained
        
        """
        self.send([0x71])
        return self.receive(1)

    def initialise(self):
        """Sends command to initialise the AHT20 sensor.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        None
        
        """
        self.send([0xBE, 0x08, 0x00])

    def trigger_measurement(self):
        """Sends command to trigger the device to record a measurement for both
        the humidity and temperature.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        None
        
        """
        self.send([0xAC, 0x33, 0x00])
        
    def recv_measurements(self):
        """Receives last recorded humidity and temperature measurements.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        bytes
            byte[0]      - State
            byte[1]      - Humidity (msb)
            byte[2]      - Humidity
            byte[3][7:4] - Humidity (lsb)
            byte[3][3:0] - Temperature (msb)
            byte[4]      - Temperature
            byte[5]      - Temperature (lsb)
        
        """
        self.send([0x71])
        bytes = self.receive(6)
        return bytes
    
    def get_humidity(self):
        """Read last measurements and calculate the relative humidity (RH) as a 
        percentage from the relative humidity signal (S_RH).
        
        Parameters
        ----------
        None
        
        Returns
        -------
        float
            Humidity reading as percentage.
        
        """
        self.trigger_measurement()
        sleep(0.075)
        bytes = self.recv_measurements()
        S_RH = (bytes[1] << 12) + (bytes[2] << 4) + (bytes[3] & 0xF0)
        RH = (S_RH / 2**20)*100
        return RH
        
    def get_temp(self):
        """Read last measurements and calculate the temperature (T) in celsius
        from the temperature output signal (S_T).
        
        Parameters
        ----------
        None
        
        Returns
        -------
        float
            Temperature reading in Celsius.
        
        """
        self.trigger_measurement()
        sleep(0.075)
        bytes = self.recv_measurements()
        S_T = ((bytes[3] & 0x0F) << 16) + (bytes[4] << 8) + bytes[5]
        T = (S_T / 2**20) * 200 - 50
        return T
        
    def soft_reset(self):
        """This command is used to restart the sensor system without having to
        turn off and turn on the power to the board. 
        
        Parameters
        ----------
        None
        
        Returns
        -------
        None
        
        """
        self.send([0xBA])
