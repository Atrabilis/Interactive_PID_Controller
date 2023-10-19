import sys
import serial
import time
import collections
import threading
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

SERIAL_PORT = 'COM3'
BAUD_RATE = 9600
MAX_POINTS = 50

class RealTimeGraph(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        # Configuración de la interfaz gráfica de usuario
        self.init_ui()

        # Inicialización de la conexión serial y el thread de lectura
        self.serial = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Espera para la inicialización del puerto serial
        self.serial.flushInput()

        self.serial_thread = threading.Thread(target=self.read_from_port, args=(self.serial,))
        self.serial_thread.daemon = True
        self.serial_thread.start()

        # Variables para guardar los datos leídos
        self.latest_data = None
        self.lock = threading.Lock()

    def init_ui(self):
        self.layout = QtWidgets.QVBoxLayout(self)

        # Parámetros PID
        self.pid_layout = QtWidgets.QHBoxLayout()
        self.p_input = QtWidgets.QLineEdit(self)
        self.i_input = QtWidgets.QLineEdit(self)
        self.d_input = QtWidgets.QLineEdit(self)
        self.windup_checkbox = QtWidgets.QCheckBox('Anti-windup Enable', self)  # Checkbox para el anti-windup
        self.pid_button = QtWidgets.QPushButton('Set PID', self)
        self.pid_button.clicked.connect(self.update_pid)

        self.pid_layout.addWidget(QtWidgets.QLabel("P:"))
        self.pid_layout.addWidget(self.p_input)
        self.pid_layout.addWidget(QtWidgets.QLabel("I:"))
        self.pid_layout.addWidget(self.i_input)
        self.pid_layout.addWidget(QtWidgets.QLabel("D:"))
        self.pid_layout.addWidget(self.d_input)
        self.pid_layout.addWidget(self.windup_checkbox)  # Añadir checkbox al layout
        self.pid_layout.addWidget(self.pid_button)
        self.layout.addLayout(self.pid_layout)

        # Gráficos
        self.win = pg.GraphicsLayoutWidget(show=True)
        self.plot = self.win.addPlot(row=1, col=1)
        self.error_plot = self.win.addPlot(row=2, col=1)  # Nuevo gráfico para los errores
        self.plot.setLabels(left='Position', bottom='Time', title='Reference vs Actual Position')
        self.error_plot.setLabels(left='Error', bottom='Time', title='Errors Over Time')  # Etiquetas para el gráfico de errores
        self.plot.addLegend()
        self.error_plot.addLegend()  # Leyenda para el gráfico de errores
        self.layout.addWidget(self.win)

        # Curvas para el gráfico de posición
        self.ref_curve = self.plot.plot(pen='r', name='Reference Position')
        self.actual_curve = self.plot.plot(pen='b', name='Actual Position')

        # Curvas para el gráfico de errores
        self.error_curve = self.error_plot.plot(pen='g', name='Error')
        self.dedt_curve = self.error_plot.plot(pen='y', name='Error derivative')
        self.eintegral_curve = self.error_plot.plot(pen='c', name='Error integral')

        # Deques para almacenar datos
        self.data_ref = collections.deque(maxlen=MAX_POINTS)
        self.data_actual = collections.deque(maxlen=MAX_POINTS)
        self.data_error = collections.deque(maxlen=MAX_POINTS)  # Datos de error
        self.data_dedt = collections.deque(maxlen=MAX_POINTS)  # Datos de la derivada del error
        self.data_eintegral = collections.deque(maxlen=MAX_POINTS)  # Datos del error integral
        self.time_stamps = collections.deque(maxlen=MAX_POINTS)

        # Configurar timer para actualizar gráfico
        self.timer = pg.QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(50)

    def update_pid(self):
        p = self.p_input.text()
        i = self.i_input.text()
        d = self.d_input.text()
        windup = '1' if self.windup_checkbox.isChecked() else '0'  # Estado del checkbox anti-windup
        command = f"P{p},I{i},D{d},W{windup}\n"  # Incluir estado del anti-windup en el comando
        self.serial.write(command.encode())

    def read_from_port(self, ser):
        while True:
            line = ser.readline().decode('utf-8').strip()
            if line:
                data = line.split(" ")
                if len(data) == 5:  # Asegurarse de que hay cinco elementos: referencia, actual, error, dedt, eintegral
                    with self.lock:
                        self.latest_data = list(map(float, data))  # Almacenar los últimos datos leídos

    def update(self):
        with self.lock:
            if self.latest_data is not None:
                ref, actual, error, dedt, eintegral = self.latest_data
                current_time = time.time()

                # Añadir nuevos puntos de datos a los deques
                self.data_ref.append(ref)
                self.data_actual.append(actual)
                self.data_error.append(error)  # Error
                self.data_dedt.append(dedt)  # Derivada del error
                self.data_eintegral.append(eintegral)  # Error integral
                self.time_stamps.append(current_time)

                # Actualizar curvas con nuevos datos
                self.ref_curve.setData(self.time_stamps, self.data_ref)
                self.actual_curve.setData(self.time_stamps, self.data_actual)
                self.error_curve.setData(self.time_stamps, self.data_error)  # Curva de error
                self.dedt_curve.setData(self.time_stamps, self.data_dedt)  # Curva de la derivada del error
                self.eintegral_curve.setData(self.time_stamps, self.data_eintegral)  # Curva del error integral

                self.latest_data = None  # Resetear los últimos datos leídos

    def closeEvent(self, event):
        self.serial.close()  # Cerrar conexión serial cuando se cierra la aplicación

def main():
    app = QtWidgets.QApplication(sys.argv)
    main = RealTimeGraph()
    main.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
