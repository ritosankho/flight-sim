import jsbsim
import pygame
import asyncio
import websockets
import json

# ===== JSBSIM INIT =====
fdm = jsbsim.FGFDMExec(None)
fdm.load_model("c172p")

fdm["ic/h-sl-ft"] = 3000
fdm["ic/u-fps"] = 150
fdm["ic/phi-deg"] = 0
fdm["ic/theta-deg"] = 0
fdm["ic/psi-deg"] = 0

fdm.run_ic()

# ===== PYGAME INIT =====
pygame.init()
screen = pygame.display.set_mode((400, 300))
pygame.display.set_caption("Flight Control Test")
clock = pygame.time.Clock()

# ===== CONTROL STATE =====
elevator_smooth = 0.0
aileron_smooth = 0.0
rudder_smooth = 0.0
throttle_cmd = 0.6

# ===== PARAMETERS =====
SMOOTH_ELEV = 0.15
SMOOTH_AIL = 0.2
SMOOTH_RUD = 0.25

# ===== WEBSOCKET CLIENTS =====
clients = set()

# ===== HELPERS =====
def smooth(current, target, rate):
    return current + rate * (target - current)

def expo(x, e=0.4):
    return x * (1 - e) + (x**3) * e

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

# ===== MAIN LOOP =====
async def main_loop():
    global elevator_smooth, aileron_smooth, rudder_smooth, throttle_cmd

    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        keys = pygame.key.get_pressed()

        # ===== TARGET INPUT =====
        # Elevator
        if keys[pygame.K_UP]:
            target_elevator = 0.7
        elif keys[pygame.K_DOWN]:
            target_elevator = -0.7
        else:
            target_elevator = 0.0

        # Aileron
        if keys[pygame.K_RIGHT]:
            target_aileron = 0.7
        elif keys[pygame.K_LEFT]:
            target_aileron = -0.7
        else:
            target_aileron = 0.0

        # Rudder
        if keys[pygame.K_d]:
            target_rudder = 0.5
        elif keys[pygame.K_a]:
            target_rudder = -0.5
        else:
            target_rudder = 0.0

        # Throttle
        if keys[pygame.K_w]:
            throttle_cmd += 0.01
        if keys[pygame.K_s]:
            throttle_cmd -= 0.01

        throttle_cmd = clamp(throttle_cmd, 0, 1)

        # ===== SMOOTHING =====
        elevator_smooth = smooth(elevator_smooth, target_elevator, SMOOTH_ELEV)
        aileron_smooth  = smooth(aileron_smooth,  target_aileron,  SMOOTH_AIL)
        rudder_smooth   = smooth(rudder_smooth,   target_rudder,   SMOOTH_RUD)

        # ===== APPLY EXPO =====
        elev = expo(elevator_smooth)
        ail  = expo(aileron_smooth)
        rud  = expo(rudder_smooth)

        # ===== SEND TO JSBSIM =====
        fdm["fcs/elevator-cmd-norm"] = elev
        fdm["fcs/aileron-cmd-norm"]  = ail
        fdm["fcs/rudder-cmd-norm"]   = rud
        fdm["fcs/throttle-cmd-norm"] = throttle_cmd

        # STEP SIM
        fdm.run()

        # DEBUG
        print(f"P:{fdm['attitude/theta-deg']:6.2f} "
              f"R:{fdm['attitude/phi-deg']:6.2f} "
              f"Y:{fdm['attitude/psi-deg']:6.2f} "
              f"ALT:{fdm['position/h-sl-ft']:7.1f}")

        clock.tick(60)
        await asyncio.sleep(0)

    pygame.quit()

# ===== WEBSOCKET SERVER =====
async def ws_handler(websocket):
    global clients
    clients.add(websocket)
    try:
        while True:
            await asyncio.sleep(1)
    finally:
        clients.remove(websocket)

# ===== BROADCAST LOOP =====
async def ws_broadcast():
    global clients

    while True:
        if clients:
            data = json.dumps({
                "pitch": fdm["attitude/theta-deg"],
                "roll": fdm["attitude/phi-deg"],
                "heading": fdm["attitude/psi-deg"],
                "speed": fdm["velocities/vc-kts"],
                "altitude": fdm["position/h-sl-ft"],
                "vs": fdm["velocities/v-down-fps"]
            })

            dead = set()

            for client in clients:
                try:
                    await client.send(data)
                except:
                    dead.add(client)

            clients -= dead

        await asyncio.sleep(1/60)

# ===== RUN EVERYTHING =====
async def run_all():
    server = await websockets.serve(ws_handler, "localhost", 8765)

    await asyncio.gather(
        main_loop(),
        ws_broadcast()
    )

if __name__ == "__main__":
    asyncio.run(run_all())