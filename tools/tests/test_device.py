from unittest.mock import MagicMock

import pytest
from serial import SerialException

from tools.utils import SerialDevice


def test_connect_success(mocker):
    """Test the successful connection to the serial device.
    
    Test mocks the 'serial.Serial' class to simulate a successsful
    connection to the device and checks if the device reports
    being connected after calling 'connect()'.
    """
    mock_serial = mocker.patch('serial.Serial', autospec=True)
    mock_serial.return_value.is_open = True
    device = SerialDevice(port="COM1")
    
    device.connect()
    
    assert device.is_connected() == True
    mock_serial.assert_called_once_with("COM1", 9600, timeout=1.0)

def test_disconnect_success(mocker):
    """Test the successful disconnection from the serial device.
    
    Test mocks the 'serial.Serial' class, establishes a connection,
    and the disconnects, ensuring that the 'disconnect()' method
    correctly closes the connection and updates the device state.
    """
    mock_serial = mocker.patch('serial.Serial', autospec=True)
    mock_serial.return_value.is_open = True
    device = SerialDevice(port="COM1")
    
    device.connect()
    device.disconnect()
    
    assert device.is_connected() == False
    mock_serial.return_value.close.assert_called_once()

def test_connect_failure(mocker):
    """Test the failure to connect to the serial device.
    
    Test mocks the 'serial.Serial' class to simulate a failure
    during the connection attempt (raises a 'SerialException'),
    ensuring that the 'connect()' method correctly handles the
    exception and raises it.
    """
    mock_serial = mocker.patch('serial.Serial', autospec=True)
    mock_serial.side_effect = SerialException("Connection failed")
    device = SerialDevice(port="COM1")
    
    with pytest.raises(SerialException):
        device.connect()

def test_write_data_success(mocker):
    """Test successful data write to the serial device.
    
    Test mocks the 'serial.Serial' class, establishes a connection,
    and calls the 'write_data()' method. It ensures that the data is
    correctly written to the serial device.
    """
    mock_serial = mocker.patch('serial.Serial', autospec=True)
    mock_serial.return_value.is_open = True
    mock_serial.return_value.write = MagicMock()
    device = SerialDevice(port="COM1")
    
    device.connect()
    device.write_data("Hello, Device!")
    
    mock_serial.return_value.write.assert_called_once_with(b"Hello, Device!")

def test_read_data_success(mocker):
    """Test successful data read from the serial device.
    
    Test mocks the 'serial.Serial' class, establishes a connection, and
    simulates reading data from the serial device. It verifies that the
    correct data is returned by the 'read_data()' method.
    """
    mock_serial = mocker.patch('serial.Serial', autospec=True)
    mock_serial.return_value.is_open = True
    mock_serial.return_value.readline = MagicMock(return_value=b"Response from device")
    device = SerialDevice(port="COM1")
    
    device.connect()
    data = device.read_data()
    
    assert data == "Response from device"
    mock_serial.return_value.readline.assert_called_once()

def test_reconnect_success(mocker):
    """Test successful reconnection to the serial device.
    
    Test mocks the 'serial.Serial' class, establishes a connection,
    and then simulates the reconnection process using the 'reconnect()'
    method. It ensures that the device reconnects successfully.
    """
    mock_serial = mocker.patch('serial.Serial', autospec=True)
    mock_serial.return_value.is_open = True
    device = SerialDevice(port="COM1")
    
    device.connect()
    device.reconnect(retries=2, delay=1.0)
    
    assert device.is_connected() == True

def test_reconnect_failure(mocker):
    """Test the failure to reconnect to the serial device.
    
    Test mocks the 'serial.Serial' class to simulate a failure during
    the reconnection attempt (raises a 'SerialException'), ensuring
    that the 'reconnect()' method correctly handles the exception
    and raises it.
    """
    mock_serial = mocker.patch('serial.Serial', autospec=True)
    mock_serial.side_effect = SerialException("Connection failed")
    device = SerialDevice(port="COM1")
    
    with pytest.raises(SerialException):
        device.reconnect(retries=2, delay=1.0)

def test_flush_input(mocker):
    """Test the flushing of the input buffer of the serial device.
    
    Test mocks the 'serial.Serial' class, establishes a connection,
    and calls the 'flush_input()' method. It ensures that the input
    buffer is correctly reset.
    """
    mock_serial = mocker.patch('serial.Serial', autospec=True)
    mock_serial.return_value.is_open = True
    device = SerialDevice(port="COM1")
    
    device.connect()
    device.flush_input()
    
    mock_serial.return_value.reset_input_buffer.assert_called_once()

def test_flush_output(mocker):
    """Test the flushing of the output buffer of the serial device.
    
    Test mocks the 'serial.Serial' class, establishes a connection,
    and calls the 'flush_output()' method. It ensures that the output
    buffer is correctly reset.
    """
    mock_serial = mocker.patch('serial.Serial', autospec=True)
    mock_serial.return_value.is_open = True
    device = SerialDevice(port="COM1")
    
    device.connect()
    device.flush_output()
    
    mock_serial.return_value.reset_output_buffer.assert_called_once()