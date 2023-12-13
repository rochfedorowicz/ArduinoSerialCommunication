import serial
import time
from serial.tools import list_ports as ports_list
import struct
from datetime import datetime
import asyncio
import numpy as np
import scipy.io as sio
from numpy.core.records import fromarrays

START_MESSAGE = b'SOS\n'
RECEIVED_MESSAGE = b'R\n'

async def hanlde_signal_receiving_and_writing(port, port_data):
    print(f'[INFO] Started collecting signals from {port.name}')
    matlab_data = {'info': {'measurement_date': datetime.today().__str__()}}
    matlab_data.update({'movements': {}})
    number_of_signals = port_data.get(port.name).get('number_of_signals')
    singal_data = np.empty((number_of_signals, 0))
    begin_date = datetime.now()
    matlab_data.get('movements').update({'time_begin': 0})
    while True:
        if port.in_waiting:
            signal_data_row = []
            received_bytes = port.read(2)
            if (received_bytes == RECEIVED_MESSAGE):
                for i in range(number_of_signals):
                    received_bytes = port.read(4)
                    received_signal = struct.unpack('f', received_bytes)[0]
                    signal_data_row.append(received_signal)
                singal_data = np.append(singal_data, np.expand_dims(signal_data_row, axis=1), axis=1)
                port.write(RECEIVED_MESSAGE)
            else: break
    end_date = datetime.now()
    time_elapsed = (end_date - begin_date).total_seconds()
    matlab_data.get('movements').update({'time_end': time_elapsed})
    matlab_data.get('movements').update({'sources': {}})
    matlab_data.get('movements').get('sources').update({'name': 'Arduino'})
    matlab_data.get('movements').get('sources').update({'signals': {}})
    count = singal_data.shape[1]
    frequency = count * 1.0 / time_elapsed
    current_name = port_data.get(port.name).get('signals').get(f'signal_1').get('name')
    j = 1
    last_i = 0
    for i in range(number_of_signals):
        signal_data = port_data.get(port.name).get('signals').get(f'signal_{i + 1}')
        if (current_name != signal_data.get('name')):
            needed_data = [singal_data[k, :] for k in range(last_i, i)]
            needed_coord = [port_data.get(port.name).get('signals').get(f'signal_{k + 1}').get('axis') for k in range(last_i, i)]
            signal_matlab_data = fromarrays(needed_data, names = needed_coord)
            matlab_data.get('movements').get('sources').get('signals').get(f'signal_{j}').update({'data': signal_matlab_data})
            current_name = signal_data.get('name')
            j = j + 1
            last_i = i
        if f'signal_{j}' not in matlab_data.get('movements').get('sources').get('signals'):
            matlab_data.get('movements').get('sources').get('signals').update({f'signal_{j}': {}})
            matlab_data.get('movements').get('sources').get('signals').get(f'signal_{j}').update({'type': 'signal'})
            matlab_data.get('movements').get('sources').get('signals').get(f'signal_{j}').update({'name': signal_data.get('name')})
            matlab_data.get('movements').get('sources').get('signals').get(f'signal_{j}').update({'units': signal_data.get('units')})
            matlab_data.get('movements').get('sources').get('signals').get(f'signal_{j}').update({'frequency': frequency})
            matlab_data.get('movements').get('sources').get('signals').get(f'signal_{j}').update({'count': count})
    i = number_of_signals
    needed_data = [singal_data[k, :] for k in range(last_i, i)]
    needed_coord = [port_data.get(port.name).get('signals').get(f'signal_{k + 1}').get('axis') for k in range(last_i, i)]
    signal_matlab_data = fromarrays(needed_data, names = needed_coord)
    matlab_data.get('movements').get('sources').get('signals').get(f'signal_{j}').update({'data': signal_matlab_data})
    sio.savemat(f'{datetime.now().__str__().replace(":", "_").replace(".", "_").replace(" ", "_").replace("-", "_")}.mat',
                {f'record_{datetime.now().__str__().replace(":", "_").replace(".", "_").replace(" ", "_").replace("-", "_")}': matlab_data})
    print(f'[INFO] Ended collecting signals from {port.name}')

def main():
    print('[INFO] Program starts')
    is_connection_established = {}
    are_signals_negotiated = {}
    serial_ports = {}
    port_data = {}

    for port in ports_list.comports():
        serial_ports.update({port.name: serial.Serial()})
        serial_ports.get(port.name).baudrate = 230400
        serial_ports.get(port.name).bytesize = 8
        serial_ports.get(port.name).parity = serial.PARITY_NONE
        serial_ports.get(port.name).port = port.name
        serial_ports.get(port.name).setDTR(False)
        serial_ports.get(port.name).open()
        is_connection_established.update({port.name: False})
        are_signals_negotiated.update({port.name: False})
        port_data.update({port.name: {}})

    for port_name in is_connection_established:
        for i in range(5):
            if not is_connection_established[port_name]:
                print(f'[INFO] Trying to establish connection on {port_name}')
                port = serial_ports.get(port_name)
                waiting_counter = 0
                port.write(START_MESSAGE)
                while not port.in_waiting:
                    time.sleep(0.05)
                    waiting_counter = waiting_counter + 1
                    if (waiting_counter == 200):
                        print(f'[INFO] Failed to establish connection on {port_name}')
                        break
                if port.in_waiting:
                    received_message = port.read(2)
                    if (received_message == RECEIVED_MESSAGE):
                        is_connection_established[port_name] = True
                        print(f'[INFO] Established connection on {port_name}')
        if not is_connection_established[port_name]:
            are_signals_negotiated.pop(port_name)
            port_data.pop(port_name)

    for port_name in are_signals_negotiated:
        if not are_signals_negotiated[port_name]:
            print(f'[INFO] Trying to negotiate signals on {port_name}')
            port = serial_ports.get(port_name)
            if port.in_waiting:
                received_bytes = port.read()
                number_of_signals = int.from_bytes(received_bytes, byteorder='big')
                port_data.get(port.name).update({'number_of_signals': number_of_signals})
                received_message = port.read().decode('utf')
                if (received_message == '\n'):
                    port.write(RECEIVED_MESSAGE)
                port_data.get(port.name).update({'signals': {}})
                for i in range(number_of_signals):
                    port_data.get(port.name).get('signals').update({f'signal_{i + 1}': {}})
                    while not port.in_waiting:
                        time.sleep(0.005)
                    received_bytes = port.read_until()
                    if (received_bytes[0] == 0):
                        received_bytes = received_bytes[1:]
                    received_message = received_bytes.decode('utf')
                    port_data.get(port.name).get('signals').get(f'signal_{i + 1}').update({'name': received_message[:-1]})
                    received_message = port.read_until().decode('utf')
                    port_data.get(port.name).get('signals').get(f'signal_{i + 1}').update({'axis': received_message[:-1]})
                    received_message = port.read_until().decode('utf')
                    port_data.get(port.name).get('signals').get(f'signal_{i + 1}').update({'units': received_message[:-1]})
                port.write(RECEIVED_MESSAGE)
                received_message = port.read_until()
                if (received_message[-2:] == RECEIVED_MESSAGE):
                    are_signals_negotiated[port_name] = True
                    print(f'[INFO] Negotiated signals on {port_name}')
                    print(f'[INFO] Device reported {number_of_signals} signals to be transmitted on {port_name}:')
                    for i in range(number_of_signals):
                        signal_name = port_data.get(port.name).get('signals').get(f'signal_{i + 1}').get('name')
                        signal_axis = port_data.get(port.name).get('signals').get(f'signal_{i + 1}').get('axis')
                        signal_units = port_data.get(port.name).get('signals').get(f'signal_{i + 1}').get('units')
                        print(f'    > Signal {signal_name}_{signal_axis} measured in {signal_units}')

    for port_name in are_signals_negotiated:
        if are_signals_negotiated[port_name]:
            asyncio.run(hanlde_signal_receiving_and_writing(
                            serial_ports.get(port_name),
                            port_data
                        ))

if __name__ == "__main__":
    main()