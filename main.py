import os
import sys
import numpy as np
import math
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication, QMainWindow, QGridLayout, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit
from PyQt5.QtCore import Qt
from PyQt5 import QtGui

# Check operating system
CURR_OS = 0
if os.name == "nt":
    CURR_OS = 1

class GraphWidget(QWidget):
    def __init__(self, parent=None):
        super(GraphWidget, self).__init__(parent)

        self._load_start()

    def _load_start(self):
        # Set initial layout for reading file
        self._layout = QGridLayout(self)
        # self._layout.setRowStretch(3, 1)
        
        # TODO: Load fenix Logo at center
        self.parent().setWindowTitle("BancadaInterface")

        # Receive filepath for csv file
        self._selected_file = ""
        self._is_selected = False
        self.filepath_widget = QLineEdit(self)
        self.filepath_widget.setPlaceholderText("Please insert path to data file")
        self.filepath_widget.textChanged.connect(self._check_file)
        self._layout.addWidget(self.filepath_widget, 0, 0)
        self.error_message = QLabel(self)
        self.error_message.setText(None)
        self._layout.addWidget(self.error_message, 1, 0)

        self.confirm_button = QPushButton("Confirm", self)
        self.confirm_button.setMaximumWidth(100)
        self.confirm_button.clicked.connect(self._confirm_file)
        self._layout.addWidget(self.confirm_button, 0, 1)

    def _check_file(self, filepath: str):
        '''
        Updates the current entered filepath in the search bar
        '''
        if not os.path.abspath(".") in filepath:
            if CURR_OS == 0:
                filepath = os.path.abspath(".") + "/" + filepath.replace("\\", "/").strip("/")
            elif CURR_OS == 1:
                filepath = os.path.abspath(".") + "\\" + filepath.replace("/", "\\").strip("\\")

        if self._is_selected:
            self._new_filepath = filepath
        else:
            self._selected_file = filepath

    def _set_error_message(self, msg: str):
        '''
        Sets the content in the label for error reporting
        '''
        self.error_message.setText(msg)
    
    def _confirm_file(self):
        '''
        Checks filename for errors in path and extension.
        '''
        checking_file = self._selected_file
        if self._is_selected:
            checking_file = self._new_filepath
 
        if len(checking_file) < 1:
            self._set_error_message("Please enter a file path")
        elif not os.path.isfile(checking_file):
            self._set_error_message("Please enter a valid file path")
        elif (not checking_file.lower().endswith(".txt")) and (not checking_file.lower().endswith(".csv")):
            self._set_error_message("Please enter a valid file")
        else:
            self._selected_file = checking_file
            self._load_graphs()
            self._is_selected = True
            
    def _read_header(self, filepath: str):
        '''
        Gets the header content from the data files.
        Expected format:
        ; TEST NAME
        CLASSE Dext COMPRIMENTO P MASSA_PROPELENTE(kg) MASSA_MOTOR_COMPLETO(kg) FENIX
        '''
        with open(filepath, 'r') as report:
            header = report.readline()
            self._test_name = header.strip("; \n")
            self.parent().setWindowTitle(f"BancadaInterface - {self._test_name}")

            self._model_info = report.readline().split()

            self._columns = list(filter(None, report.readline().split()))

    def _read_data(self, filepath: str):
        '''
        Reads the columns data and stores in a dictionary.
        Each key corresponds to a column and contains a list of values
        for that metric.
        '''
        self._values = dict.fromkeys(self._columns, [])
        with open(filepath, 'r') as report:
            skip_columns = True
            for line in report.readlines()[2:]:
                row = list(filter(None, line.split()))
                if skip_columns:
                    skip_columns = False
                    continue                
                for i, val in enumerate(row):
                    self._values[self._columns[i]].append(float(val))

    def _load_visualization(self):
        # self._graph_layout = QGridLayout(self)
        # self._layout.addLayout(self._graph_layout, 2, 0, 1, 0)
        # Plot the test information including the test name
        self._info_display = QVBoxLayout(self)
        
        # Plot one graph for each column excepting the time
        self._plotwidgets = []
        self._plots = []
        for idx, column in enumerate(self._columns[1:]):
            # Create widget in grid layout
            self._plotwidgets.append(pg.PlotWidget(self))
            self._plotwidgets[idx].setTitle(f"{column} x {self._columns[0]}")
            self._plotwidgets[idx].setLabel('left', column)
            self._plotwidgets[idx].setLabel('bottom', self._columns[0])

            # Display graph
            # TODO change plot style (really bad plot too many points)
            self._plots.append(self._plotwidgets[idx].plot(pen='#3d2163', symbolBrush='#3d2163', symbolPen='w'))
            self._plots[idx].setData(np.array(self._values[self._columns[0]]), np.array(self._values[column]))
            self._layout.addWidget(self._plotwidgets[idx], 2 + math.floor(idx/2), idx%2)

            # Add statistics for graph

    def _load_graphs(self):
        print(f"Loading graphs from file {self._selected_file}")
        self._read_header(self._selected_file)
        self._read_data(self._selected_file)
        self._load_visualization()


if __name__ == "__main__":
    print("Inicializando bancada de testes...")
        
    app = QApplication(sys.argv)
    main_window = QMainWindow()
    graph_widget = GraphWidget(main_window)
    main_window.setCentralWidget(graph_widget)
    main_window.setGeometry(100, 100, 800, 600)
    main_window.show()

    sys.exit(app.exec_())