import canopen
import time

# --- Basic setup ---
CAN_INTERFACE = "can0"
BAUDRATE = 1_000_000
NODE_ID = 1

print("Connecting to CANopen network...")
network = canopen.Network()
network.connect(bustype="socketcan", channel=CAN_INTERFACE, bitrate=BAUDRATE)

# Create a RemoteNode without an EDS
node = canopen.RemoteNode(NODE_ID, None)
network.add_node(node)

try:
    node.nmt.wait_for_bootup(5)
except canopen.nmt.NmtError:
    print("No boot-up seen; continuing anyway.")
node.nmt.state = "OPERATIONAL"

# --- Helper to write SDOs (index, subindex, value, size) ---
def write_sdo(index, subindex, value, size):
    node.sdo.download(index, subindex, value.to_bytes(size, byteorder="little", signed=True))

# --- Set mode of operation = Profile Velocity (3) ---
write_sdo(0x6060, 0x00, 3, 1)
print("Mode of operation set to Profile Velocity (3).")

# --- Enable operation (drive state machine sequence) ---
for value in (0x0006, 0x0007, 0x000F):
    write_sdo(0x6040, 0x00, value, 2)
    time.sleep(0.05)
print("Drive enabled for operation.")

# --- Jog counter-clockwise ---
target_velocity = 1000  # negative = CCW
write_sdo(0x60FF, 0x00, target_velocity, 4)

write_sdo(0x6040, 0x00, 0x003F, 2)  # 0x000F + 0x0030
time.sleep(0.05)
write_sdo(0x6040, 0x00, 0x002F, 2)  # clear bit 4 again

write_sdo(0x6040, 0x00, 0x001F, 2)
print(f"Jogging counter-clockwise at {target_velocity} internal units...")

time.sleep(3)

# --- Stop motion ---
write_sdo(0x60FF, 0x00, 0, 4)
write_sdo(0x6040, 0x00, 0x0000, 2)
print("Motion stopped.")

network.disconnect()
print("Disconnected cleanly.")
