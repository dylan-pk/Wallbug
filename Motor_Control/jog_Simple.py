import canopen
import time
import datetime
# --- Configuration ---
CAN_INTERFACE = "can0"
BAUDRATE = 1_000_000
NODE_ID = 1

# --- Connect to CAN network ---
print("Connecting to CAN network...")
network = canopen.Network()
network.connect(bustype="socketcan", channel=CAN_INTERFACE, bitrate=BAUDRATE)

# --- Add node without EDS ---
node = canopen.RemoteNode(NODE_ID, None)
network.add_node(node)

# --- Wait for boot-up (skip if already running) ---
try:
    node.nmt.wait_for_bootup(5)
except canopen.nmt.NmtError:
    print("No boot-up message seen; continuing anyway.")
node.nmt.state = "OPERATIONAL"
print("Node in OPERATIONAL state.")

# --- Helper to write SDOs ---
def write_sdo(index, subindex, value, size):
    node.sdo.download(index, subindex, value.to_bytes(size, byteorder="little", signed=True))

# --- Set the Mode of Operation to Job ---
write_sdo(0x6060, 0x00, -1, 1)
print('here')
time.sleep(0.5)

# --- Setting reference values ---
print('setting method')
write_sdo(0x4453, 0x00, 0, 2)
print('setting time')
write_sdo(0x4451, 0x00, 1000000, 4)
print('max Profile Velocity')
write_sdo(0x607F, 0x00, 5000000, 4)
print('Jog step')
write_sdo(0x4452, 0x00, 100000, 4)
time.sleep(0.5)

status = int.from_bytes(node.sdo.upload(0x6041, 0x00), "little")
prot_bit = (status >> 12) & 0x01
if prot_bit:
    print("WARNING: Drive is in PROT (Protected) mode. Cannot jog via CANopen.")
else:
    print("Drive in REMOTE/CAN mode. Ready for jogging.")

# --- Starting Movement ---
write_sdo(0x6040, 0x00, 80, 2)
print('starting movement')
while(1):
    status = int.from_bytes(node.sdo.upload(0x6041, 0x00), "little")
    print(f"Final Statusword: 0x{status:04X}")
    if status & 0x0400:  # bit10 = Target reached
        print("Motion complete")
    time.sleep(0.1)



#timenow = datetime.datetime.now

