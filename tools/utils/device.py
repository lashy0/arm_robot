import logging
import time

import serial


class SerialDevice:
    """Class to manage a serial connection to a device.

    Attributes
    ----------
    port: str 
        The COM port for the serial connection.
    
    baudrate: int
        The baud rate for the serial connection.
    
    timeout: float
        The read timeout for the serial connection.
    
    _serial: serial.Serial
        The pySerial instance representing the connection.
    """
    def __init__(self, port: str = None, baudrate: int = 9600, timeout: float = 1.0):
        """Initialize the SerialDevice.

        Args
        ----
        port:
            The COM port to connect.
        
        baudrate:
            The baud rate the connection.
        
        timeout:
            The timeout for reading from the serial connection.
        """
        self._serial = None
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

        self._logger = logging.getLogger("SerialDevice")
    
    def is_connected(self) -> bool:
        """Check if the serial connection is open

        Returns
        -------
        result: bool
            True if the connection is open, False otherwise.
        """
        return self._serial is not None and self._serial.is_open
    
    def connect(self, port: str = None) -> None:
        """Connects to the serial device.

        Args
        ----
        port: str, optional
            The COM port to connect to.
        
        Raises
        ------
        serial.SerialException:
            If the connection to the serial port fails.
        """
        if port:
            self.port = port
        
        if not self.port:
            self._logger.error("No COM port specified")
            return
        
        if not self.is_connected():
            try:
                self._serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
                self._logger.info(f"Connected to {self.port} at {self.baudrate} baudrate")
            except serial.SerialException as e:
                self._logger.error(f"Failed to connect to {self.port}: {e}")
                raise
    
    def disconnect(self) -> None:
        """Disconnects from the serial device if connected."""
        if self.is_connected():
            try:
                self._serial.close()
                self._serial = None
                self._logger.info(f"Disconnected from {self.port}")
            except serial.SerialException as e:
                self._logger.error(f"Failed to disconnect from {self.port}: {e}")
    
    def reconnect(self, retries: int = 3, delay: float = 1.0) -> None:
        """Attempts to reconnect to the serial device.

        Args
        ----
        retries: int
            Number of reconnection attempts.
        
        delay: float
            Delay between reconnection attempts is seconds.
        """
        for attempt in range(retries):
            self._logger.info(f"Reconnection attempt {attempt + 1}...")
            self.disconnect()
            time.sleep(delay)
            try:
                self.connect()

                if self.is_connected():
                    self._logger.info(f"Reconnected to {self.port} on attempt {attempt + 1}")
                    return
            except serial.SerialException as e:
                self._logger.error(f"Reconnection attempt {attempt + 1} failed: {e}")
            
        self._logger.error(f"Failed to reconnect to {self.port} after {retries} attempts")
        raise serial.SerialException(f"Failed to reconnect to {self.port} after {retries} attempts")
    
    def _set_timeout(self, timeout: float) -> float:
        previous_timeout = self._serial.timeout
        if timeout:
            self._serial.timeout = timeout
        return previous_timeout
    
    def _restore_timeout(self, previous_timeout: float) -> None:
        self._serial.timeout = previous_timeout

    def write_data(self, data: str, timeout: float = None) -> None:
        """Writes data to the serial device.
        
        Args
        ----
        data: str
            The data to be written to the serial device.
        
        timeout: float, optional
            Timeout for writing data.
        
        Raises
        ------
        serial.SerialException:
            If writing to the serial port fails.
        """
        if not self.is_connected():
            self._logger.error(f"Device {self.port} is not connected")
            return
        
        try:
            previous_timeout = self._set_timeout(timeout)

            self._serial.write(data.encode('utf-8'))
            self._logger.debug(f"Sent to {self.port}: {data}")
            if timeout:
                self._serial.timeout = previous_timeout
        except serial.SerialException as e:
            self._logger.error(f"Failed to write data to {self.port}: {e}")
            raise
        finally:
            self._restore_timeout(previous_timeout)

    def read_data(self, timeout: float = None) -> str:
        """Reads data from the serial device.

        Args
        ----
        timeout: float, optional
            Timeout for reading data.
        
        Returns
        -------
        data: str
            The data read from the serial devic, or an empty string if the read fails.
        
        Raises
        ------
        serial.SerialException:
            If reading from the serial port fails.
        """
        if not self.is_connected():
            self._logger.error(f"Device {self.port} is not connected")
            return ""
        
        try:
            previous_timeout = self._set_timeout(timeout)
            
            data = self._serial.readline().decode('utf-8').strip()
            if data:
                self._logger.debug(f"Received from {self.port}: {data}")
            else:
                self._logger.warning(f"No data received from {self.port}")
            
            return data
        except serial.SerialTimeoutException as e:
            self._logger.error(f"Timeout while rading from {self.port}: {e}")
            return ""
        except serial.SerialException as e:
            self._logger.error(f"Failed to read data from {self.port}: {e}")
            raise
        finally:
            self._restore_timeout(previous_timeout)
            
    def flush_input(self) -> None:
        """Flushes the input buffer of the serial connection.

        Raises
        ------
        serial.SerialException:
            If flushing the input buffer fails.
        """
        if self.is_connected():
            try:
                self._serial.reset_input_buffer()
                self._logger.info(f"Input buffer reset to for {self.port}")
            except serial.SerialException as e:
                self._logger.error(f"Failed to flush input buffer for {self.port}: {e}")
                raise
    
    def flush_output(self) -> None:
        """Flushes the output buffer of the serial connection.

        Raises
        ------
        serial.SerialException:
            If flushing the output buffer fails.
        """
        if self.is_connected():
            try:
                self._serial.reset_output_buffer()
                self._logger.info(f"Output buffer reset for {self.port}")
            except serial.SerialException as e:
                self._logger.error(f"Failed to flush output buffer for {self.port}: {e}")
                raise
