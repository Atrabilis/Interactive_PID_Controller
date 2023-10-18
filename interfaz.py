import sys
import serial
import time
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
import collections
import threading

SERIAL_PORT = 'COM3'
BAUD_RATE = 9600
MAX_POINTS = 50

class RealTimeGraph:
    def __init__(self):
        self.app = QtWidgets.QApplication(sys.argv)
        self.win = pg.GraphicsLayoutWidget(show=True)
        self.win.setWindowTitle('Real Time Graph')
        self.plot = self.win.addPlot()
        self.plot.setLabels(left='Position', bottom='Time', title='Reference vs Actual Position')
        self.plot.addLegend()

        self.ref_curve = self.plot.plot(pen='r', name='Reference Position')
        self.actual_curve = self.plot.plot(pen='b', name='Actual Position')

        self.data_ref = collections.deque(maxlen=MAX_POINTS)
        self.data_actual = collections.deque(maxlen=MAX_POINTS)
        self.time_stamps = collections.deque(maxlen=MAX_POINTS)

        # Configurar timer para actualizar gráfico
        self.timer = pg.QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(50)

        self.serial = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        self.serial.flushInput()

        # Thread para leer datos
        self.serial_thread = threading.Thread(target=self.read_from_port, args=(self.serial,))
        self.serial_thread.daemon = True
        self.serial_thread.start()

        # Variables para guardar los datos leídos
        self.latest_ref = None
        self.latest_actual = None
        self.lock = threading.Lock()

    def read_from_port(self, ser):
        while True:
            line = ser.readline().decode('utf-8').strip()
            if line:
                data = line.split(" ")
                if len(data) == 2:
                    with self.lock:
                        self.latest_ref, self.latest_actual = map(float, data)

    def update(self):
        with self.lock:
            if self.latest_ref is not None and self.latest_actual is not None:
                current_time = time.time()
                
                self.data_ref.append(self.latest_ref)
                self.data_actual.append(self.latest_actual)
                self.time_stamps.append(current_time)

                self.ref_curve.setData(self.time_stamps, self.data_ref)
                self.actual_curve.setData(self.time_stamps, self.data_actual)

                self.latest_ref, self.latest_actual = None, None  # Resetear los últimos datos leídos

    def start(self):
        if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
            QtWidgets.QApplication.instance().exec_()

    def close(self):
        self.serial.close()

# Crear y ejecutar la aplicación
if __name__ == '__main__':
    graph = RealTimeGraph()
    try:
        graph.start()
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        graph.close()
