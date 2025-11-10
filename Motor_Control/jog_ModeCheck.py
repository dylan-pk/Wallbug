import canopen
import time

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

# --- Read Statusword ---
status = int.from_bytes(node.sdo.upload(0x6041, 0x00), "little")
print(f"Statusword: 0x{status:04X}")

# --- Check if motor is in PROT mode ---
# On LXM28A, bit 12 (Operation mode specific) = 1 → PROT mode
prot_bit = (status >> 12) & 0x01
if prot_bit:
    print("WARNING: Drive is in PROT (Protected) mode. Cannot jog via CANopen.")
    network.disconnect()
    exit(1)
else:
    print("Drive in REMOTE/CAN mode. Ready for jogging.")

# --- Set Profile Velocity Mode (0x6060 = 3) ---
write_sdo(0x6060, 0x00, 3, 1)
print("Mode of operation: Profile Velocity (3)")

## --- Drive state machine: enable operation ---
#for value in (0x0006, 0x0007, 0x000F):
#    write_sdo(0x6040, 0x00, value, 2)
#    time.sleep(0.05)
#
## Optional: check statusword
#status = int.from_bytes(node.sdo.upload(0x6041, 0x00), "little")
#print(f"Statusword after enable: 0x{status:04X}")
#
## --- Set target velocity (negative = counterclockwise) ---
#target_velocity = -1000  # adjust units to your drive
#write_sdo(0x60FF, 0x00, target_velocity, 4)
#print(f"Target velocity set to {target_velocity}")
#
## --- Start motion: CiA-402 new set-point handshake ---
#write_sdo(0x6040, 0x00, 0x003F, 2)  # bits for new set-point + change immediately
#time.sleep(0.05)
#write_sdo(0x6040, 0x00, 0x002F, 2)  # clear new set-point bit
#
#print("Motor should now be jogging counterclockwise...")
#
## --- Run for 3 seconds ---
#time.sleep(3)
#
## --- Stop motion ---
#write_sdo(0x60FF, 0x00, 0, 4)
#write_sdo(0x6040, 0x00, 0x000F, 2)
#print("Motion stopped.")
#
## Optional: check final statusword
#status = int.from_bytes(node.sdo.upload(0x6041, 0x00), "little")
#print(f"Final Statusword: 0x{status:04X}")
#
## --- Disconnect ---
#network.disconnect()
#print("Disconnected cleanly.")
