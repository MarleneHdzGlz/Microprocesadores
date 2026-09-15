from controller import Supervisor
import math

TIME_STEP = 32
CRUISE_SPEED = 5.0
TURN_SPEED = 3.0
BACK_SPEED = -3.0
DISTANCIA_MAXIMA = 0.15
ARRANQUE = 30
TOTAL_BASURAS = 7       

supervisor = Supervisor()
robot = supervisor

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

prox = []
for i in range(8):
    s = robot.getDevice(f'ps{i}')
    if s:
        s.enable(TIME_STEP)
    prox.append(s)

bumper = robot.getDevice('bumper')
bumper.enable(TIME_STEP)

basura_sensor = robot.getDevice('basura_sensor')
basura_sensor.enable(TIME_STEP)

self_node = supervisor.getSelf()
eliminadas = []

estado = 0
t = 0
paso_inicial = 0

def get_pos(nodo):
    return nodo.getField('translation').getSFVec3f()

def recolectar():
    try:
        pr = get_pos(self_node)
    except:
        return False
    mejor_nodo = None
    mejor_nombre = None
    mejor_dist = 999.0
    for i in range(1, 31):
        nombre = f"bas{i}"
        if nombre in eliminadas:
            continue
        n = supervisor.getFromDef(nombre)
        if n is None:
            continue
        try:
            pb = get_pos(n)
        except:
            continue
        dx = pr[0] - pb[0]
        dy = pr[1] - pb[1]
        d = math.sqrt(dx*dx + dy*dy)
        if d < mejor_dist:
            mejor_dist = d
            mejor_nodo = n
            mejor_nombre = nombre

    if mejor_nodo is not None and mejor_dist < DISTANCIA_MAXIMA:
        mejor_nodo.remove()
        eliminadas.append(mejor_nombre)
        print(f"✔ Eliminada {mejor_nombre} (dist={mejor_dist:.3f}) — "
              f"{len(eliminadas)}/{TOTAL_BASURAS}")
        return True
    return False

# --- Bucle principal ---
while supervisor.step(TIME_STEP) != -1:
    paso_inicial += 1

    # CONDICIÓN DE PARADA
    if len(eliminadas) >= TOTAL_BASURAS:
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)

        if not hasattr(supervisor, '_msg_final'):
            print("¡TODAS LAS BASURAS RECOLECTADAS! Aspiradora detenida.")
            supervisor._msg_final = True
        continue

    pv = [s.getValue() if s else 0.0 for s in prox]
    vb = basura_sensor.getValue() if paso_inicial > ARRANQUE else 0.0
    bumper_val = bumper.getValue()

    frente = pv[0] > 100 or pv[7] > 100

    if bumper_val == 1.0:
        if recolectar():
            estado = 0
            t = 0
        else:
            if frente:
                estado = 1
                t = 0

    elif vb > 400:
        if recolectar():
            estado = 0
            t = 0

    # --- Máquina de estados ---
    if estado == 0:
        if frente and bumper_val != 1.0:
            estado = 1
            t = 0
        else:
            left_motor.setVelocity(CRUISE_SPEED)
            right_motor.setVelocity(CRUISE_SPEED)

    elif estado == 1:
        t += 1
        left_motor.setVelocity(BACK_SPEED)
        right_motor.setVelocity(BACK_SPEED)
        if t >= 12:
            estado = 2
            t = 0

    elif estado == 2:
        t += 1
        left_motor.setVelocity(-TURN_SPEED)
        right_motor.setVelocity(TURN_SPEED)
        if (not frente and t > 15) or t > 45:
            estado = 0
            t = 0