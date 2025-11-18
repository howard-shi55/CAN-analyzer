import sys
import csv
import os
import numpy as np

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QWidget,
    QHBoxLayout, QCheckBox, QLineEdit, QMessageBox, QFileDialog,
    QTreeWidget, QTreeWidgetItem, QProgressBar, QPushButton
)
from PyQt5.QtGui import QIcon, QFont
from PyQt5.QtWebEngineWidgets import QWebEngineView
import plotly.graph_objects as go
from plotly.graph_objects import Scattergl
import plotly.express as px      # <-- add for palettes
from PyQt5.QtCore import QThread, pyqtSignal

class Data:
    def __init__(self):
        self.DC_Voltages = {"ts": [], "val": []}
        self.Output_Voltages = {"ts": [], "val": []}
        self.DC_Currents = {"ts": [], "val": []}
        self.DC_Powers = {"ts": [], "val": []}
        self.DC_Discharge_Comulative_Powers = {"ts": [], "val": []}
        self.DC_Regeneration_Comulative_Powers = {"ts": [], "val": []}
        self.DC_Comulative_Powers = {"ts": [], "val": []}
        self.A_Currents = {"ts": [], "val": []}
        self.B_Currents = {"ts": [], "val": []}
        self.C_Currents = {"ts": [], "val": []}
        self.Calculated_Currents = {"ts": [], "val": []}
        self.Feedback_Torques = {"ts": [], "val": []}
        self.Command_Torques = {"ts": [], "val": []}
        self.VCU_Command_Torques = {"ts": [], "val": []}
        self.VCU_Command_Directions = {"ts": [], "val": []}
        self.APPS1 = {"ts": [], "val": []}
        self.APPS2 = {"ts": [], "val": []}
        self.BSE = {"ts": [], "val": []}
        self.Distance = {"ts": [], "val": []}
        self.Speed = {"ts": [], "val": []}
        self.RPM = {"ts": [], "val": []}
        self.Motor_Temperature = {"ts": [], "val": []}
        self.Gate_Driver_Temperature = {"ts": [], "val": []}
        self.Module_A_Temperature = {"ts": [], "val": []}
        self.Module_B_Temperature = {"ts": [], "val": []}
        self.Module_C_Temperature = {"ts": [], "val": []}
        self.Control_Board_Temperature = {"ts": [], "val": []}
        self.Delta_Resolver = {"ts": [], "val": []}
        self.Gyro_Acc_x = {"ts": [], "val": []}
        self.Gyro_Acc_y = {"ts": [], "val": []}
        self.Gyro_Acc_z = {"ts": [], "val": []}
        self.Gyro_Ang_x = {"ts": [], "val": []}
        self.Gyro_Ang_y = {"ts": [], "val": []}
        self.Gyro_Ang_z = {"ts": [], "val": []}
        self.BMS_Pack = {"ts": [], "val": []}
        self.BMS_Segments = {"ts": [[] for _ in range(10)], "val": [[] for _ in range(10)]}
        self.BMS_Cell = {"ts": [[] for _ in range(120)], "val": [[] for _ in range(120)]}
        self.NTC_Segments = {"ts": [[] for _ in range(10)], "val": [[] for _ in range(10)]}
        self.NTC_Cell = {"ts": [[] for _ in range(50)], "val": [[] for _ in range(50)]}
        self.Steering_Angle = {"ts": [], "val": []}
        self.Steering_Speed = {"ts": [], "val": []}
        self.Front_Left_Linear = {"ts": [], "val": []}
        self.Front_Right_Linear = {"ts": [], "val": []}
        self.Rear_Left_Linear = {"ts": [], "val": []}
        self.Rear_Right_Linear = {"ts": [], "val": []}
        self.Front_Left_Wheel_Speed = {"ts": [], "val": []}
        self.Front_Right_Wheel_Speed = {"ts": [], "val": []}
        self.Rear_Left_Wheel_Speed = {"ts": [], "val": []}
        self.Rear_Right_Wheel_Speed = {"ts": [], "val": []}
        

def parse_bms_data(data):
    """Parse BMS message data"""
    module_id = int(data[0:2], 16)
    value1 = int.from_bytes(bytes.fromhex(data[4:8]), byteorder='little')
    value2 = int.from_bytes(bytes.fromhex(data[8:12]), byteorder='little')
    value3 = int.from_bytes(bytes.fromhex(data[12:16]), byteorder='little')
    
    return module_id, value1, value2, value3

def load_dataset(filepath):
    DATA = Data()
    if not os.path.isfile(filepath):
        return DATA
    
    with open(filepath, mode="r", encoding="utf-8", newline='') as f:
        reader = csv.DictReader(f, fieldnames=["Timestamp", "ID", "Extended", "Length", "Data"])
        next(reader)
        base_dt = None
        for row in reader:
            ID = row["ID"]
            data = row["Data"]
            elapsed_ms = int(row["Timestamp"])       # 把秒跟毫秒拆開
            if ID == "0x0A0":
                a_t = int.from_bytes(bytes.fromhex(data[0:4]), byteorder="little", signed=True) / 10
                b_t = int.from_bytes(bytes.fromhex(data[4:8]), byteorder="little", signed=True) / 10
                c_t = int.from_bytes(bytes.fromhex(data[8:12]), byteorder="little", signed=True) / 10
                gate_t = int.from_bytes(bytes.fromhex(data[12:16]), byteorder="little", signed=True) / 10
                DATA.Module_A_Temperature['ts'].append(elapsed_ms)
                DATA.Module_A_Temperature['val'].append(a_t)
                DATA.Module_B_Temperature['ts'].append(elapsed_ms)
                DATA.Module_B_Temperature['val'].append(b_t)
                DATA.Module_C_Temperature['ts'].append(elapsed_ms)
                DATA.Module_C_Temperature['val'].append(c_t)
                DATA.Gate_Driver_Temperature['ts'].append(elapsed_ms)
                DATA.Gate_Driver_Temperature['val'].append(gate_t)

            if ID == "0x0A1":
                value = int.from_bytes(bytes.fromhex(data[0:4]), byteorder="little", signed=True) / 10
                DATA.Control_Board_Temperature['ts'].append(elapsed_ms)
                DATA.Control_Board_Temperature['val'].append(value)

            if ID == "0x0A2":
                motor_t = int.from_bytes(bytes.fromhex(data[8:12]), byteorder="little", signed=True) / 10
                DATA.Motor_Temperature['ts'].append(elapsed_ms)
                DATA.Motor_Temperature['val'].append(motor_t)

            if ID == "0x0A5":
                rpm = int.from_bytes(bytes.fromhex(data[4:8]), byteorder="little", signed=True) * -1
                delta_resolver = int.from_bytes(bytes.fromhex(data[12:16]), byteorder="little", signed=True)
                speed = rpm * 0.52 * 3.14159 / 3 / 60 * 3.6  # Convert to km/h
                try:
                    distance = speed * (elapsed_ms - DATA.Speed['ts'][-1]) / 3600 + DATA.Distance['val'][-1]
                except Exception as e:
                    distance = 0
                DATA.RPM['ts'].append(elapsed_ms)
                DATA.RPM['val'].append(rpm)
                DATA.Delta_Resolver['ts'].append(elapsed_ms)
                DATA.Delta_Resolver['val'].append(delta_resolver)
                DATA.Speed['ts'].append(elapsed_ms)
                DATA.Speed['val'].append(speed)
                DATA.Distance['ts'].append(elapsed_ms)
                DATA.Distance['val'].append(distance)


            if ID == "0x0A7":
                dc = int.from_bytes(bytes.fromhex(data[0:4]), byteorder="little")
                output = int.from_bytes(bytes.fromhex(data[4:8]), byteorder="little")
                DATA.DC_Voltages['ts'].append(elapsed_ms)
                DATA.DC_Voltages['val'].append(dc / 10)
                DATA.Output_Voltages['ts'].append(elapsed_ms)
                DATA.Output_Voltages['val'].append(output / 10)

            if ID == "0x0A6":
                a_current = int.from_bytes(bytes.fromhex(data[0:4]), byteorder="little", signed=True) / 10
                b_current = int.from_bytes(bytes.fromhex(data[4:8]), byteorder="little", signed=True) / 10
                c_current = int.from_bytes(bytes.fromhex(data[8:12]), byteorder="little", signed=True) / 10
                dc_current = int.from_bytes(bytes.fromhex(data[12:16]), byteorder="little", signed=True) / 10
                try:
                    calculated_current = DATA.RPM['val'][-1] * DATA.Feedback_Torques['val'][-1] / 9550 * 1000 / DATA.DC_Voltages['val'][-1]
                except Exception as e:
                    calculated_current = 0
                try:
                    dc_power = DATA.DC_Voltages['val'][-1] * dc_current / 1000  # kW
                except Exception as e:
                    dc_power = 0
                try:
                    dc_comulative_power = DATA.DC_Comulative_Powers['val'][-1] + dc_power * (elapsed_ms - DATA.DC_Comulative_Powers['ts'][-1]) / 3600  # Wh
                except Exception as e:
                    dc_comulative_power = 0
                DATA.A_Currents['ts'].append(elapsed_ms)
                DATA.A_Currents['val'].append(a_current)
                DATA.B_Currents['ts'].append(elapsed_ms)
                DATA.B_Currents['val'].append(b_current)
                DATA.C_Currents['ts'].append(elapsed_ms)
                DATA.C_Currents['val'].append(c_current)
                DATA.DC_Currents['ts'].append(elapsed_ms)
                DATA.DC_Currents['val'].append(dc_current)
                DATA.Calculated_Currents['ts'].append(elapsed_ms)
                DATA.Calculated_Currents['val'].append(calculated_current)
                DATA.DC_Powers['ts'].append(elapsed_ms)
                DATA.DC_Powers['val'].append(dc_power)
                DATA.DC_Comulative_Powers['ts'].append(elapsed_ms)
                DATA.DC_Comulative_Powers['val'].append(dc_comulative_power)
                try:
                    if dc_power >= 0:
                        dc_discharge_comulative_power = DATA.DC_Discharge_Comulative_Powers['val'][-1] + dc_power * (elapsed_ms - DATA.DC_Discharge_Comulative_Powers['ts'][-1]) / 3600
                        dc_regeneration_comulative_power = DATA.DC_Regeneration_Comulative_Powers['val'][-1]
                    else:
                        dc_regeneration_comulative_power = DATA.DC_Regeneration_Comulative_Powers['val'][-1] + (-dc_power) * (elapsed_ms - DATA.DC_Regeneration_Comulative_Powers['ts'][-1]) / 3600
                        dc_discharge_comulative_power = DATA.DC_Discharge_Comulative_Powers['val'][-1]
                except Exception as e:
                    dc_discharge_comulative_power = 0
                    dc_regeneration_comulative_power = 0
                
                DATA.DC_Discharge_Comulative_Powers['ts'].append(elapsed_ms)
                DATA.DC_Discharge_Comulative_Powers['val'].append(dc_discharge_comulative_power)
                DATA.DC_Regeneration_Comulative_Powers['ts'].append(elapsed_ms)
                DATA.DC_Regeneration_Comulative_Powers['val'].append(dc_regeneration_comulative_power)

            if ID == "0x0C0":
                command_torque = int.from_bytes(bytes.fromhex(data[0:4]), byteorder="little")
                command_direction = int.from_bytes(bytes.fromhex(data[8:10]), byteorder="little")
                DATA.VCU_Command_Torques['ts'].append(elapsed_ms)
                DATA.VCU_Command_Torques['val'].append(command_torque / 10)
                DATA.VCU_Command_Directions['ts'].append(elapsed_ms)
                DATA.VCU_Command_Directions['val'].append(command_direction * 100)

            if ID == "0x0AC":
                command_torque = int.from_bytes(bytes.fromhex(data[0:4]), byteorder="little", signed=True)
                feedback_torque = int.from_bytes(bytes.fromhex(data[4:8]), byteorder="little", signed=True) * -1
                DATA.Command_Torques['ts'].append(elapsed_ms)
                DATA.Command_Torques['val'].append(command_torque / 10)
                DATA.Feedback_Torques['ts'].append(elapsed_ms)
                DATA.Feedback_Torques['val'].append(feedback_torque / 10)

            if ID == "0x000075A1":
                value = int.from_bytes(bytes.fromhex(data[0:4]), byteorder="little")
                ratio = int.from_bytes(bytes.fromhex(data[4:8]), byteorder="little")
                DATA.APPS1['ts'].append(elapsed_ms)
                DATA.APPS1['val'].append(value)

            if ID == "0x000075A2":
                value = int.from_bytes(bytes.fromhex(data[0:4]), byteorder="little")
                ratio = int.from_bytes(bytes.fromhex(data[4:8]), byteorder="little")
                DATA.APPS2['ts'].append(elapsed_ms)
                DATA.APPS2['val'].append(value)

            if ID == "0x000075B0":
                value = int.from_bytes(bytes.fromhex(data[0:4]), byteorder="little")
                ratio = int.from_bytes(bytes.fromhex(data[4:8]), byteorder="little")
                DATA.BSE['ts'].append(elapsed_ms)
                DATA.BSE['val'].append(value)

            if ID == "0x4EC":
                if DATA.Gyro_Ang_x['ts'] and elapsed_ms - DATA.Gyro_Ang_x['ts'][-1] < 100:
                    continue
                acc_x = int.from_bytes(bytes.fromhex(data[0:4]), byteorder="big", signed=True) / 10
                acc_y = int.from_bytes(bytes.fromhex(data[4:8]), byteorder="big", signed=True) / 10
                acc_z = int.from_bytes(bytes.fromhex(data[8:12]), byteorder="big", signed=True) / 10
                DATA.Gyro_Ang_x['ts'].append(elapsed_ms)
                DATA.Gyro_Ang_x['val'].append(acc_x)
                DATA.Gyro_Ang_y['ts'].append(elapsed_ms)
                DATA.Gyro_Ang_y['val'].append(acc_y)
                DATA.Gyro_Ang_z['ts'].append(elapsed_ms)
                DATA.Gyro_Ang_z['val'].append(acc_z)

            if ID == "0x4ED":
                if DATA.Gyro_Acc_x['ts'] and elapsed_ms - DATA.Gyro_Acc_x['ts'][-1] < 100:
                    continue
                ang_x = int.from_bytes(bytes.fromhex(data[0:4]), byteorder="big", signed=True) / 100
                ang_y = int.from_bytes(bytes.fromhex(data[4:8]), byteorder="big", signed=True) / 100
                ang_z = int.from_bytes(bytes.fromhex(data[8:12]), byteorder="big", signed=True) / 100
                DATA.Gyro_Acc_x['ts'].append(elapsed_ms)
                DATA.Gyro_Acc_x['val'].append(ang_x)
                DATA.Gyro_Acc_y['ts'].append(elapsed_ms)
                DATA.Gyro_Acc_y['val'].append(ang_y)
                DATA.Gyro_Acc_z['ts'].append(elapsed_ms)
                DATA.Gyro_Acc_z['val'].append(ang_z)

            NUM_CMU_MODULE = 10  # Number of CMU modules
            NUM_CELL_PER_CMU = 12  # Cells per CMU
            NUM_NTC_PER_CMU = 5  # NTC sensors per CMU
            BMS_CV_ID = [
                "0x12905301",
                "0x12905381",
                "0x12905401",
                "0x12905481",
                "0x12905501",
                "0x12905581",
            ]
            if ID in BMS_CV_ID:
                module_id, cell_a_mv, cell_b_mv, cell_c_mv = parse_bms_data(data)
                if module_id < NUM_CMU_MODULE:
                    
                    # Calculate cell indices based on message ID
                    msg_idx = BMS_CV_ID.index(ID)
                    cell_a_idx = msg_idx * 3
                    cell_b_idx = msg_idx * 3 + 1
                    cell_c_idx = msg_idx * 3 + 2

                    
                    # Update cell voltages
                    if cell_a_idx < NUM_CELL_PER_CMU:
                        DATA.BMS_Cell['ts'][module_id*NUM_CELL_PER_CMU + cell_a_idx].append(elapsed_ms)
                        DATA.BMS_Cell['val'][module_id*NUM_CELL_PER_CMU + cell_a_idx].append(cell_a_mv)
                    if cell_b_idx < NUM_CELL_PER_CMU:
                        DATA.BMS_Cell['ts'][module_id*NUM_CELL_PER_CMU + cell_b_idx].append(elapsed_ms)
                        DATA.BMS_Cell['val'][module_id*NUM_CELL_PER_CMU + cell_b_idx].append(cell_b_mv)
                    if cell_c_idx < NUM_CELL_PER_CMU:
                        DATA.BMS_Cell['ts'][module_id*NUM_CELL_PER_CMU + cell_c_idx].append(elapsed_ms)
                        DATA.BMS_Cell['val'][module_id*NUM_CELL_PER_CMU + cell_c_idx].append(cell_c_mv)
                    try:
                        # print([i[-1] for i in DATA.BMS_Cell['val']])
                        total_pack_voltage = sum([i[-1] for i in DATA.BMS_Cell['val']]) / 1000  # Convert mV to V
                        # print(total_pack_voltage)
                    except Exception as e:
                        total_pack_voltage = 0
                    DATA.BMS_Pack['ts'].append(elapsed_ms)
                    DATA.BMS_Pack['val'].append(total_pack_voltage)
                    try:
                        total_segment_voltage = sum([i[-1] for i in DATA.BMS_Cell['val'][msg_idx // 4 * 12 : msg_idx // 4 * 12 + 12]]) / 1000
                    except Exception as e:
                        total_segment_voltage = 0
                    DATA.BMS_Segments['ts'][module_id].append(elapsed_ms)
                    DATA.BMS_Segments['val'][module_id].append(total_segment_voltage)


            # # Process temperature messages
            BMS_NTC_ID = [
                "0x12905601",
                "0x12905681"
            ]
            if ID in BMS_NTC_ID:
                module_id, ntc_a, ntc_b, ntc_c = parse_bms_data(data)
                if module_id < NUM_CMU_MODULE:
                    # Calculate NTC indices
                    msg_idx = BMS_NTC_ID.index(ID)
                    ntc_a_idx = msg_idx * 3
                    ntc_b_idx = msg_idx * 3 + 1
                    ntc_c_idx = msg_idx * 3 + 2
                    
                    # Convert to Celsius (from 0.1 Kelvin units)
                    ntc_a_deg_c = 0.1 * float(ntc_a) - 273.15
                    ntc_b_deg_c = 0.1 * float(ntc_b) - 273.15
                    ntc_c_deg_c = 0.1 * float(ntc_c) - 273.15
                    
                    # Update NTC temperatures, bypass -273.15 (invalid data)
                    if ntc_a_idx < NUM_NTC_PER_CMU and ntc_a_deg_c != -273.15:
                        DATA.NTC_Cell['ts'][module_id * NUM_NTC_PER_CMU + ntc_a_idx].append(elapsed_ms)
                        DATA.NTC_Cell['val'][module_id * NUM_NTC_PER_CMU + ntc_a_idx].append(ntc_a_deg_c)
                    if ntc_b_idx < NUM_NTC_PER_CMU and ntc_b_deg_c != -273.15:
                        DATA.NTC_Cell['ts'][module_id * NUM_NTC_PER_CMU + ntc_b_idx].append(elapsed_ms)
                        DATA.NTC_Cell['val'][module_id * NUM_NTC_PER_CMU + ntc_b_idx].append(ntc_b_deg_c)
                    if ntc_c_idx < NUM_NTC_PER_CMU and ntc_c_deg_c != -273.15:
                        DATA.NTC_Cell['ts'][module_id * NUM_NTC_PER_CMU + ntc_c_idx].append(elapsed_ms)
                        DATA.NTC_Cell['val'][module_id * NUM_NTC_PER_CMU + ntc_c_idx].append(ntc_c_deg_c)
                    
                    try:
                        avg_ntc_segment_temp = sum([v[-1] for v in DATA.NTC_Cell['val'][module_id * NUM_NTC_PER_CMU : (module_id + 1) * NUM_NTC_PER_CMU] if v]) / NUM_NTC_PER_CMU
                    except Exception as e:
                        avg_ntc_segment_temp = 0
                    DATA.NTC_Segments['ts'][module_id].append(elapsed_ms)
                    DATA.NTC_Segments['val'][module_id].append(avg_ntc_segment_temp)
                    
            if ID == "0x2B0":
                data_bytes = bytes.fromhex(data)
                angle = int.from_bytes(data_bytes[0:2], byteorder="little", signed=True) / 10
                speed = int.from_bytes(data_bytes[2:3], byteorder="little", signed=True)

                DATA.Steering_Angle['ts'].append(elapsed_ms)
                DATA.Steering_Angle['val'].append(angle)
                DATA.Steering_Speed['ts'].append(elapsed_ms)
                DATA.Steering_Speed['val'].append(speed)
                
            if ID == "0x200":
                data_bytes = bytes.fromhex(data)
                front_left_linear = int.from_bytes(data_bytes[0:2], byteorder="big", signed=False) / 10
                front_right_linear = int.from_bytes(data_bytes[2:4], byteorder="big", signed=False) / 10
                DATA.Front_Left_Linear['ts'].append(elapsed_ms)
                DATA.Front_Left_Linear['val'].append(front_left_linear)
                DATA.Front_Right_Linear['ts'].append(elapsed_ms)
                DATA.Front_Right_Linear['val'].append(front_right_linear)
            
            if ID == "0x300":
                data_bytes = bytes.fromhex(data)
                rear_left_linear = int.from_bytes(data_bytes[0:2], byteorder="big", signed=False) / 10
                rear_right_linear = int.from_bytes(data_bytes[2:4], byteorder="big", signed=False) / 10
                DATA.Rear_Left_Linear['ts'].append(elapsed_ms)
                DATA.Rear_Left_Linear['val'].append(rear_left_linear)
                DATA.Rear_Right_Linear['ts'].append(elapsed_ms)
                DATA.Rear_Right_Linear['val'].append(rear_right_linear)
            
            if ID == "0x710":
                data_bytes = bytes.fromhex(data)
                front_left_wheel_speed = int.from_bytes(data_bytes[0:2], byteorder="big", signed=False) / 100
                front_right_wheel_speed = int.from_bytes(data_bytes[2:4], byteorder="big", signed=False) / 100
                DATA.Front_Left_Wheel_Speed['ts'].append(elapsed_ms)
                DATA.Front_Left_Wheel_Speed['val'].append(front_left_wheel_speed)
                DATA.Front_Right_Wheel_Speed['ts'].append(elapsed_ms)
                DATA.Front_Right_Wheel_Speed['val'].append(front_right_wheel_speed)
                
            if ID == "0x702":
                data_bytes = bytes.fromhex(data)
                rear_left_wheel_speed = int.from_bytes(data_bytes[0:2], byteorder="big", signed=False) / 100
                rear_right_wheel_speed = int.from_bytes(data_bytes[2:4], byteorder="big", signed=False) / 100
                DATA.Rear_Left_Wheel_Speed['ts'].append(elapsed_ms)
                DATA.Rear_Left_Wheel_Speed['val'].append(rear_left_wheel_speed)
                DATA.Rear_Right_Wheel_Speed['ts'].append(elapsed_ms)
                DATA.Rear_Right_Wheel_Speed['val'].append(rear_right_wheel_speed)
                

    # Convert all lists to numpy arrays before returning
    for key, obj in DATA.__dict__.items():
        if isinstance(obj, dict):
            for subkey, val in obj.items():
                try:
                    obj[subkey] = np.array(val)
                except Exception as e:
                    obj[subkey] = [np.array(v) for v in val]
    return DATA

class FileLineEdit(QLineEdit):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAcceptDrops(True)
        self.setPlaceholderText("Drag a csv file or click here")

    def dragEnterEvent(self, event):
        urls = event.mimeData().urls()
        if urls and urls[0].scheme() == 'file':
            event.acceptProposedAction()

    def dragMoveEvent(self, event):
        urls = event.mimeData().urls()
        if urls and urls[0].scheme() == 'file':
            event.acceptProposedAction()

    def dropEvent(self, event):
        urls = event.mimeData().urls()
        if not (urls and urls[0].scheme() == 'file'):
            return
        path = urls[0].toLocalFile()
        if not path.lower().endswith('.csv'):
            QMessageBox.warning(self, "Error: Invalid File",
                                "Only .csv files are accepted")
            return
        self.setText(path)
        top = self.window()
        if hasattr(top, "load_file"):
            top.load_file(path)
    
    def mousePressEvent(self, event):
        file_path, _ = QFileDialog.getOpenFileName(self, "Select CSV File", "", "CSV Files (*.csv)")
        if file_path:
            self.setText(file_path)
            top = self.window()
            if hasattr(top, "load_file"):
                top.load_file(file_path)
        else:
            super().mousePressEvent(event)

class LoadThread(QThread):
    result = pyqtSignal(object)
    error  = pyqtSignal(str)

    def __init__(self, filepath):
        super().__init__()
        self.filepath = filepath

    def run(self):
        try:
            data = load_dataset(self.filepath)
            self.result.emit(data)
        except Exception as e:
            self.error.emit(str(e))

class PlotlyWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("我不想加班qwq")
        self.resize(900, 700)
        self.setWindowIcon(QIcon("icon.ico"))

        widget = QWidget()
        main_layout = QHBoxLayout(widget)

        sidebar = QWidget()
        sidebar_layout = QVBoxLayout(sidebar)

        # add our drag-drop line-edit
        self.file_input = FileLineEdit(self)
        self.file_input.setMinimumHeight(40)  # <-- Make the input box larger
        self.file_input.setMinimumWidth(200)  # <-- Optionally, make it wider
        sidebar_layout.addWidget(self.file_input)

        # --- Collapsible checkbox groups using QTreeWidget ---
        self.tree = QTreeWidget()
        self.tree.setHeaderHidden(True)
        self.tree.setStyleSheet("""
            QTreeWidget { 
                border: none; 
                background: palette(window); 
            }
        """)  # <-- Updated to use the origin gray bg
        sidebar_layout.addWidget(self.tree)
        # sidebar_layout.addStretch()
        main_layout.addWidget(sidebar)

        # Define categories and their checkboxes
        categories = {
            "Voltage": [
                "DC Voltages(V)",
                "Output Voltages(V)",
            ],
            "Current": [
                "DC Currents(A)",
                "A Currents(A)",
                "B Currents(A)",
                "C Currents(A)",
                "Calculated Currents(A)",
            ],
            "Power": [
                "DC Powers(kW)",
                "DC Comulative Powers(kWh)",
                "DC Discharge Comulative Powers(kWh)",
                "DC Regeneration Comulative Powers(kWh)",
            ],
            "Torque": [
                "Feedback Torques(Nm)",
                "Command Torques(Nm)",
                "VCU Command Torques(Nm)",
                "VCU Command Directions",
            ],
            "Pedal": [
                "APPS1(mV)",
                "APPS2(mV)",
                "BSE(mV)",
            ],
            "Motor": [
                "RPM(rpm)",
                "Motor Temperature(°C)",
                "Gate Driver Temperature(°C)",
                "Module A Temperature(°C)",
                "Module B Temperature(°C)",
                "Module C Temperature(°C)",
                "Control Board Temperature(°C)",
                "Delta Resolver",
            ],
            "Gyro": [
                "Gyro Acc x(°/s²)",
                "Gyro Acc y(°/s²)",
                "Gyro Acc z(°/s²)",
                "Gyro Ang x(°/s²)",
                "Gyro Ang y(°/s²)",
                "Gyro Ang z(°/s²)",
            ],
            "Motion": [
                "Speed(km/h)",
                "Distance(km)",
                "Steering Angle(°)",
                "Steering Speed(°/s)",
            ],
            "Suspension": [
                "Front Left Linear(mm)",
                "Front Right Linear(mm)",
                "Rear Left Linear(mm)",
                "Rear Right Linear(mm)",
                "Front Left Wheel Speed(rps)",
                "Front Right Wheel Speed(rps)",
                "Rear Left Wheel Speed(rps)",
                "Rear Right Wheel Speed(rps)",
            ],
            "BMS": [
                "BMS Pack(V)",
                "BMS Segments(V)",
                "BMS Segment 1(mV)",
                "BMS Segment 2(mV)",
                "BMS Segment 3(mV)",
                "BMS Segment 4(mV)",
                "BMS Segment 5(mV)",
                "BMS Segment 6(mV)",
                "BMS Segment 7(mV)",
                "BMS Segment 8(mV)",
                "BMS Segment 9(mV)",
                "BMS Segment 10(mV)"
            ],
            "NTC" : [
                "NTC Segments(°C)",       # ← aggregated avg-NTC per module
                "NTC Segment 1(°C)",      # ← individual NTC sensors in module 1
                "NTC Segment 2(°C)",
                "NTC Segment 3(°C)",
                "NTC Segment 4(°C)",
                "NTC Segment 5(°C)",
                "NTC Segment 6(°C)",
                "NTC Segment 7(°C)",
                "NTC Segment 8(°C)",
                "NTC Segment 9(°C)",
                "NTC Segment 10(°C)",
            ],
        }

        self.CheckBoxes = []
        for cat, items in categories.items():
            cat_item = QTreeWidgetItem([cat])
            self.tree.addTopLevelItem(cat_item)
            for label in items:
                cb = QCheckBox(label)
                cb.stateChanged.connect(self.plot)
                self.tree.setItemWidget(QTreeWidgetItem(cat_item), 0, cb)
                self.CheckBoxes.append(cb)

        # --- Plot area ---
        self.webview = QWebEngineView()
        main_layout.addWidget(self.webview, 1)

        self.setCentralWidget(widget)

        self.fig = go.Figure()
        self.data = load_dataset('')

        self.loaded = []
        
        # self.export_btn = QPushButton("Export")
        # sidebar_layout.addWidget(self.export_btn)
        # self.export_btn.setFixedHeight(30)
        # self.export_btn.setFlat(True)

        # add status‐bar progress indicator
        self.progress = QProgressBar(self)
        self.progress.setVisible(False)
        sidebar_layout.addWidget(self.progress)

    def load_file(self, filepath):
        """Called by FileLineEdit when a .csv is dropped."""
        # 1) show busy‐indicator
        self.progress.setRange(0, 0)   # indeterminate
        self.progress.setVisible(True)
        QApplication.processEvents()

        # 2) start background loading
        self.loader = LoadThread(filepath)
        self.loader.result.connect(self.on_data_loaded)
        self.loader.error.connect(self.on_load_error)
        self.loader.start()

    def on_data_loaded(self, data):
        # replace old data + reset UI
        self.data = data
        self.fig = go.Figure()
        self.loaded.clear()
        for cb in self.CheckBoxes:
            cb.setChecked(False)
        self.webview.setHtml(self.fig.to_html(include_plotlyjs='cdn'))

        # hide progress bar
        self.progress.setVisible(False)
        self.loader = None

    def on_load_error(self, message):
        self.progress.setVisible(False)
        QMessageBox.warning(self, "Load Error", message)
        self.loader = None

    def plot(self):
        cb = self.sender()
        text = cb.text()
        text = text.find('(') != -1 and text[:text.find('(')] or text
        

        # --- BMS segment N (12 cells) with per-cell shading ---
        if text.startswith("BMS Segment ") and text != "BMS Segments":
            seg = int(text.split()[-1]) - 1
            # clear old traces
            self.fig.data = tuple(
                tr for tr in self.fig.data
                if not tr.name.startswith(f"Seg{seg+1}_Cell")
            )
            if cb.isChecked():
                base_col = px.colors.qualitative.Plotly[seg % len(px.colors.qualitative.Plotly)]
                for c in range(12):
                    ts = self.data.BMS_Cell['ts'][seg*12 + c]
                    val = self.data.BMS_Cell['val'][seg*12 + c]
                    color = shade_color(base_col, c, 12)
                    self.fig.add_trace(
                        Scattergl(
                            x=ts, y=val, mode="lines",
                            name=f"Seg{seg+1}_Cell{c+1}",
                            line=dict(color=color)
                        )
                    )
            self.webview.setHtml(self.fig.to_html(include_plotlyjs='cdn'))
            return

        # --- BMS Segments (10 module volt averages) ---
        if text == "BMS Segments":
            self.fig.data = tuple(
                tr for tr in self.fig.data
                if not tr.name.startswith("Module ")
            )
            if cb.isChecked():
                num_mod = len(self.data.BMS_Segments['val'])
                for m in range(num_mod):
                    ts = self.data.BMS_Segments['ts'][m]
                    val = self.data.BMS_Segments['val'][m]
                    self.fig.add_trace(
                        Scattergl(x=ts, y=val, mode="lines",
                                  name=f"Module {m+1}")
                    )
            self.webview.setHtml(self.fig.to_html(include_plotlyjs='cdn'))
            return

        # --- NTC segment N (5 sensors) with per-sensor shading ---
        if text.startswith("NTC Segment ") and text != "NTC Segments":
            seg = int(text.split()[-1]) - 1
            # clear old traces
            self.fig.data = tuple(
                tr for tr in self.fig.data
                if not tr.name.startswith(f"Seg{seg+1}_NTC")
            )
            if cb.isChecked():
                base_col = px.colors.qualitative.Plotly[seg % len(px.colors.qualitative.Plotly)]
                for i in range(5):
                    ts = self.data.NTC_Cell['ts'][seg*5 + i]
                    val = self.data.NTC_Cell['val'][seg*5 + i]
                    color = shade_color(base_col, i, 5)
                    self.fig.add_trace(
                        Scattergl(
                            x=ts, y=val, mode="lines",
                            name=f"Seg{seg+1}_NTC{i+1}",
                            line=dict(color=color)
                        )
                    )
            self.webview.setHtml(self.fig.to_html(include_plotlyjs='cdn'))
            return

        # --- NTC Segments (avg-NTC per module) ---
        if text == "NTC Segments":
            self.fig.data = tuple(
                tr for tr in self.fig.data
                if not tr.name.startswith("NTC Module ")
            )
            if cb.isChecked():
                num_mod = len(self.data.NTC_Segments['val'])
                for m in range(num_mod):
                    ts = self.data.NTC_Segments['ts'][m]
                    val = self.data.NTC_Segments['val'][m]
                    self.fig.add_trace(
                        Scattergl(x=ts, y=val, mode="lines",
                                  name=f"NTC Module {m+1}")
                    )
            self.webview.setHtml(self.fig.to_html(include_plotlyjs='cdn'))
            return

        # --- all other signals (single trace) ---
        key = text.replace(" ", "_")
        if cb.isChecked():
            if text in self.loaded:
                for tr in self.fig.data:
                    if tr.name == text:
                        tr.visible = True
            else:
                _D = getattr(self.data, key)
                self.fig.add_trace(
                    Scattergl(x=_D['ts'], y=_D['val'],
                              mode='lines', name=text)
                )
                self.loaded.append(text)
        else:
            for tr in self.fig.data:
                if tr.name == text:
                    tr.visible = False

        self.webview.setHtml(self.fig.to_html(include_plotlyjs='cdn'))

# helper to darken/lighten a hex color
def darken_color(hex_str, factor):
    h = hex_str.lstrip('#')
    r, g, b = (int(h[i:i+2], 16) for i in (0, 2, 4))
    r, g, b = [int(c * factor) for c in (r, g, b)]
    return f"rgb({r},{g},{b})"

# compute a shade between full color and min_factor-dark version
def shade_color(hex_str, idx, total, min_factor=0.3):
    if total < 2:
        return hex_str
    factor = 1 - (idx / (total - 1)) * (1 - min_factor)
    return darken_color(hex_str, factor)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setFont(QFont("Segoe UI", 9))
    win = PlotlyWindow()
    win.show()
    sys.exit(app.exec_())
