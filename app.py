import time
import os
import smbus2
import paho.mqtt.client as mqtt
from prometheus_client import start_http_server, Gauge

# --- CONFIGURATION ---
MQTT_BROKER = os.getenv('MQTT_BROKER', '192.168.0.175')
MQTT_PORT = int(os.getenv('MQTT_PORT', 1883))
MQTT_TOPIC = os.getenv('MQTT_TOPIC', 'dzicave/pi3_BMP180')
UPDATE_INTERVAL = int(os.getenv('UPDATE_INTERVAL', 15))
I2C_BUS = 1
BMP180_ADDRESS = 0x77

# --- PROMETHEUS METRICS ---
prom_temp_cpu = Gauge('pi3_cpu_temperature_celsius', 'Temperature of the Raspberry Pi CPU')
prom_temp_sensor = Gauge('pi3_temperature_celsius', 'Ambient temperature from BMP180')
prom_pressure = Gauge('pi3_pressure_pa', 'Barometric pressure from BMP180')

# --- BMP180 DRIVER CLASS ---
class BMP180:
    def __init__(self, bus_num=1, address=0x77):
        self.bus = smbus2.SMBus(bus_num)
        self.address = address
        # Read calibration data
        self.cal = {}
        self.load_calibration()

    def read_word(self, reg):
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        val = (high << 8) + low
        return val - 65536 if val >= 32768 else val

    def read_unsigned_word(self, reg):
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        return (high << 8) + low

    def load_calibration(self):
        # 11 words of calibration data
        cal_regs = {
            'AC1': 0xAA, 'AC2': 0xAC, 'AC3': 0xAE, 'AC4': 0xB0,
            'AC5': 0xB2, 'AC6': 0xB4, 'B1': 0xB6, 'B2': 0xB8,
            'MB': 0xBA, 'MC': 0xBC, 'MD': 0xBE
        }
        for name, reg in cal_regs.items():
            if name in ['AC4', 'AC5', 'AC6']:
                self.cal[name] = self.read_unsigned_word(reg)
            else:
                self.cal[name] = self.read_word(reg)

    def read_raw_temp(self):
        self.bus.write_byte_data(self.address, 0xF4, 0x2E)
        time.sleep(0.005)
        return self.read_unsigned_word(0xF6)

    def read_raw_pressure(self):
        # Mode 1 (standard)
        self.bus.write_byte_data(self.address, 0xF4, 0x34 + (1 << 6))
        time.sleep(0.014) # Wait for conversion
        msb = self.bus.read_byte_data(self.address, 0xF6)
        lsb = self.bus.read_byte_data(self.address, 0xF7)
        xlsb = self.bus.read_byte_data(self.address, 0xF8)
        return ((msb << 16) + (lsb << 8) + xlsb) >> (8 - 1)

    def read_data(self):
        # Temperature calculation
        UT = self.read_raw_temp()
        X1 = ((UT - self.cal['AC6']) * self.cal['AC5']) / 32768.0
        X2 = (self.cal['MC'] * 2048.0) / (X1 + self.cal['MD'])
        B5 = X1 + X2
        temp = (B5 + 8.0) / 16.0 / 10.0

        # Pressure calculation
        UP = self.read_raw_pressure()
        B6 = B5 - 4000
        X1 = (self.cal['B2'] * (B6 * B6 / 4096.0)) / 2048.0
        X2 = (self.cal['AC2'] * B6) / 2048.0
        X3 = X1 + X2
        B3 = (((self.cal['AC1'] * 4 + X3) * 2) + 2) / 4.0
        X1 = (self.cal['AC3'] * B6) / 8192.0
        X2 = (self.cal['B1'] * (B6 * B6 / 4096.0)) / 65536.0
        X3 = ((X1 + X2) + 2) / 4.0
        B4 = (self.cal['AC4'] * (X3 + 32768)) / 32768.0
        B7 = (UP - B3) * (50000 >> 1)
        
        if B7 < 0x80000000:
            p = (B7 * 2) / B4
        else:
            p = (B7 / B4) * 2
            
        X1 = (p / 256.0) * (p / 256.0)
        X1 = (X1 * 3038) / 65536.0
        X2 = (-7357 * p) / 65536.0
        pressure = p + (X1 + X2 + 3791) / 16.0
        
        return temp, pressure

# --- HELPER FUNCTIONS ---
def get_cpu_temp():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            return float(f.read()) / 1000.0
    except:
        return 0.0

def on_connect(client, userdata, flags, rc):
    print(f"Connected to MQTT Broker with result code {rc}")

# --- MAIN ---
if __name__ == "__main__":
    print("Starting Sensor Exporter...")
    
    # Init Sensors
    bmp = BMP180(bus_num=I2C_BUS)
    
    # Init MQTT
    mqtt_client = mqtt.Client()
    mqtt_client.on_connect = on_connect
    try:
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        mqtt_client.loop_start()
    except Exception as e:
        print(f"Failed to connect to MQTT: {e}")

    # Init Prometheus
    start_http_server(8000)
    print("Prometheus metrics available on port 8000")

    while True:
        try:
            # Read Data
            cpu_t = get_cpu_temp()
            bmp_t, bmp_p = bmp.read_data()
            
            # Update Prometheus
            prom_temp_cpu.set(cpu_t)
            prom_temp_sensor.set(bmp_t)
            prom_pressure.set(bmp_p)
            
            # Publish MQTT
            payload = f'{{"cpu_temperature": {cpu_t:.2f}, "temperature": {bmp_t:.2f}, "pressure": {bmp_p:.2f}}}'
            mqtt_client.publish(MQTT_TOPIC, payload)
            
            print(f"Update: CPU={cpu_t:.1f}C, Sensor={bmp_t:.1f}C, Pressure={bmp_p:.1f}Pa")
            
        except Exception as e:
            print(f"Error reading sensor: {e}")
            
        time.sleep(UPDATE_INTERVAL)
