#!/usr/bin/env python3
"""
estacion_tierra.py - Estacion en tierra para el proyecto Black Bird
Marco Antonio Beltran Perez - mabeltranpe@unal.edu.co

Manda al avion las teclas activas cada 50 ms (todas juntas, para poder
controlar los tres ejes a la vez) y dibuja los testigos de vuelo con
la telemetria que devuelve el avion.

TRAMA QUE ENVIA (bytes sueltos, los que ya interpreta cambio_fsm):
    "awu"   = alabeo izq + cabeceo arriba + modo crucero

TRAMA QUE ESPERA RECIBIR (una linea, legible en minicom):
    ROLL=-12 PITCH=5 ALT=34 THR=1550

Uso:   python3 estacion_tierra.py
Salir: cerrar la ventana, o Ctrl+Q
"""

import math
import re
import sys
import tkinter as tk
import serial

# ----------------------------------------------------------------------
# CONFIGURACION
# ----------------------------------------------------------------------

# Dos enlaces posibles. Se elige por argumentos al arrancar:
#
#   python3 estacion_tierra.py                     -> XBee   (por defecto)
#   python3 estacion_tierra.py cable               -> cable USB de la Nucleo
#   python3 estacion_tierra.py /dev/ttyUSB1 9600   -> puerto y baudios a mano
#
ENLACES = {
    'xbee':  ("/dev/ttyUSB0", 19200),    # XBee por adaptador FTDI
    'cable': ("/dev/ttyACM0", 19200),   # VCP del ST-Link
}

_arg = sys.argv[1] if len(sys.argv) > 1 else 'xbee'
if _arg in ENLACES:
    PUERTO, BAUDIOS = ENLACES[_arg]
else:
    PUERTO  = _arg
    BAUDIOS = int(sys.argv[2]) if len(sys.argv) > 2 else 19200

PERIODO_MS        = 50        # cada cuanto se manda el estado al avion
PERIODO_DIBUJO_MS = 100       # cada cuanto se repintan los instrumentos
ANTIRREBOTE_MS = 40           # filtro de la auto-repeticion de X11

ALT_MAX = 100                 # fondo de escala del altimetro, en metros

# tecla -> (eje, signo), segun el firmware
TECLAS_EJES = {
    'a': ('roll',  '+'),  'd': ('roll',  '-'),
    'w': ('pitch', '+'),  's': ('pitch', '-'),
    'l': ('yaw',   '+'),  'j': ('yaw',   '-'),
}

TECLAS_MODOS = {
    'p': ('p', 'ARMADO'),
    'i': ('i', 'DESPEGUE'),
    'u': ('u', 'CRUCERO'),
    'o': ('o', 'ATERRIZAJE'),
    ' ': (' ', 'DESARMADO'),
}

TECLA_PANICO = 'Escape'

# ----------------------------------------------------------------------
# COLORES
# ----------------------------------------------------------------------

FONDO   = "#141414"
CARA    = "#0d0d0d"
BORDE   = "#8a8a8a"
CIELO   = "#2e6fb7"
TIERRA  = "#7a4a1e"
AGUJA   = "#ffcc00"
TEXTO   = "#e8e8e8"
SUAVE   = "#8a8a8a"
VERDE   = "#3fa34d"
ROJO    = "#c0392b"

# ----------------------------------------------------------------------
# ESTADO
# ----------------------------------------------------------------------

ejes = {'roll': '0', 'pitch': '0', 'yaw': '0'}
modo = TECLAS_MODOS[' ']
pendientes = {}

tele = {'ROLL': 0.0, 'PITCH': 0.0, 'ALT': 0.0, 'THR': 1000.0}
modo_avion = "-"              # el modo que reporta el avion por telemetria

# el avion puede nombrar los campos de varias formas: se aceptan todas
ALIAS = {
    'ROLL': 'ROLL',
    'PITCH': 'PITCH',
    'ALT': 'ALT', 'ALTURA': 'ALT', 'ALTITUD': 'ALT',
    'THR': 'THR', 'THROTTLE': 'THR', 'POTENCIA': 'THR',
}
ALIAS_MODO = ('MODE', 'MODO', 'MODO_OPERACION')

buffer_rx = ""
puerto = None
ultima_linea = "(sin datos)"

TECLA_DE = {}
for _t, (_e, _s) in TECLAS_EJES.items():
    TECLA_DE[(_e, _s)] = _t


# ----------------------------------------------------------------------
# SERIAL
# ----------------------------------------------------------------------

def abrir_puerto():
    global puerto
    try:
        puerto = serial.Serial(PUERTO, BAUDIOS, timeout=0)
    except Exception as e:
        print("No se pudo abrir %s: %s" % (PUERTO, e))
        print("La estacion arranca sin enlace.")
        puerto = None


def enviar():
    """Manda los bytes de todas las teclas activas, juntos.
    El firmware los procesa uno a uno, asi que "awu" pone alabeo,
    cabeceo y modo en el mismo instante."""
    salida = ""
    for eje in ('roll', 'pitch', 'yaw'):
        if ejes[eje] != '0':
            salida += TECLA_DE[(eje, ejes[eje])]
    salida += modo[0]
    if puerto:
        try:
            puerto.write(salida.encode())
        except Exception as e:
            print("Error al transmitir:", e)
    return salida


def recibir():
    """Parsea lineas de pares clave=valor. Acepta, por ejemplo:

        ROLL=-12 PITCH=5 ALT=34 THR=1550
        ROLL=-12 ° PITCH=5 ° THROTTLE=1550 MODO_OPERACION=CRUCERO

    Los trozos sueltos sin '=' (como el simbolo de grados) se ignoran,
    y las claves desconocidas tambien: asi el avion puede mezclar
    mensajes de depuracion con la telemetria sin romper nada.
    """
    global buffer_rx, ultima_linea, modo_avion
    if not puerto:
        return
    try:
        datos = puerto.read(1024).decode(errors='ignore')
    except Exception:
        return
    if not datos:
        return
    buffer_rx += datos
    while "\n" in buffer_rx:
        linea, buffer_rx = buffer_rx.split("\n", 1)
        linea = linea.strip()
        if not linea:
            continue
        ultima_linea = linea
        for campo in linea.split():
            if "=" not in campo:
                continue
            clave, _, valor = campo.partition("=")
            clave = clave.strip().upper()
            valor = valor.strip()

            if clave in ALIAS_MODO:
                modo_avion = valor
                continue

            destino = ALIAS.get(clave)
            if destino is None:
                continue
            # tolera basura pegada al numero (unidades, grados, etc.)
            m = re.match(r'[-+]?\d+(\.\d+)?', valor)
            if m:
                tele[destino] = float(m.group(0))


# ----------------------------------------------------------------------
# TECLADO
# ----------------------------------------------------------------------

def caracter(evento):
    c = evento.char if evento.char else evento.keysym
    return ' ' if c == 'space' else c


def al_presionar(evento):
    global modo
    if evento.keysym == TECLA_PANICO:
        panico()
        return
    if evento.keysym in pendientes:
        raiz.after_cancel(pendientes.pop(evento.keysym))
    c = caracter(evento)
    if c in TECLAS_EJES:
        eje, signo = TECLAS_EJES[c]
        ejes[eje] = signo
    elif c in TECLAS_MODOS:
        modo = TECLAS_MODOS[c]


def al_soltar(evento):
    c = caracter(evento)
    if c not in TECLAS_EJES:
        return
    tecla = evento.keysym

    def confirmar():
        pendientes.pop(tecla, None)
        ejes[TECLAS_EJES[c][0]] = '0'

    if tecla in pendientes:
        raiz.after_cancel(pendientes[tecla])
    pendientes[tecla] = raiz.after(ANTIRREBOTE_MS, confirmar)


def panico():
    global modo
    modo = TECLAS_MODOS[' ']
    for e in ejes:
        ejes[e] = '0'
    for t in list(pendientes):
        raiz.after_cancel(pendientes.pop(t))


# ----------------------------------------------------------------------
# INSTRUMENTOS  (los tres son redondos, como en un panel de verdad)
# ----------------------------------------------------------------------

def bisel(cx, cy, r, titulo):
    cv.create_oval(cx - r - 6, cy - r - 6, cx + r + 6, cy + r + 6,
                   fill="#2a2a2a", outline="#3a3a3a")
    cv.create_oval(cx - r, cy - r, cx + r, cy + r, fill=CARA, outline=BORDE, width=2)
    cv.create_text(cx, cy + r + 26, text=titulo, fill=TEXTO,
                   font=("DejaVu Sans", 11, "bold"))


def horizonte(cx, cy, r, roll, pitch):
    """Horizonte artificial. El avion (simbolo amarillo) esta fijo y el
    mundo gira detras. Todo se dibuja DENTRO del circulo calculando la
    interseccion exacta del horizonte con la esfera: sin mascaras.

    Escalas de grados en los dos ejes:
      - cabeceo: escalera de marcas cada 10 grados que sube y baja
      - alabeo:  arco fijo con marcas, y un puntero que gira con el avion
    """
    a  = math.radians(roll)
    esc = r / 45.0                          # pixeles por grado de cabeceo
    d  = max(-0.95 * r, min(0.95 * r, pitch * esc))
    ux, uy = math.cos(a), math.sin(a)       # a lo largo del horizonte
    nx, ny = -math.sin(a), math.cos(a)      # perpendicular, hacia la tierra

    # ---- cielo: el circulo completo
    cv.create_oval(cx - r, cy - r, cx + r, cy + r, fill=CIELO, outline="")

    # ---- tierra: segmento circular exacto bajo la linea del horizonte
    half = math.sqrt(r * r - d * d)         # semicuerda del horizonte
    hx, hy = cx + nx * d, cy + ny * d
    p1 = (hx - ux * half, hy - uy * half)
    p2 = (hx + ux * half, hy + uy * half)

    def puntos_arco(a_ini, a_fin):
        while a_fin < a_ini:
            a_fin += 2.0 * math.pi
        pts = []
        for i in range(1, 24):
            t = a_ini + (a_fin - a_ini) * i / 24.0
            pts.append((cx + r * math.cos(t), cy + r * math.sin(t)))
        return pts

    ang1 = math.atan2(p1[1] - cy, p1[0] - cx)
    ang2 = math.atan2(p2[1] - cy, p2[0] - cx)
    arco = puntos_arco(ang1, ang2)
    mid = arco[len(arco) // 2]
    lado = (mid[0] - cx) * nx + (mid[1] - cy) * ny
    if lado < d:                            # ese arco era el del cielo
        arco = puntos_arco(ang2, ang1)
        poligono = [p2] + arco + [p1]
    else:
        poligono = [p1] + arco + [p2]
    cv.create_polygon(poligono, fill=TIERRA, outline="")
    cv.create_line(p1, p2, fill="white", width=2)

    # ---- escalera de cabeceo: una marca cada 10 grados
    for m in (-30, -20, -10, 10, 20, 30):
        dm = d - m * esc                    # posicion de la marca m
        if abs(dm) > r - 20:
            continue
        bx, by = cx + nx * dm, cy + ny * dm
        hl = 16 + abs(m) * 0.45             # las marcas crecen con el angulo
        cv.create_line(bx - ux * hl, by - uy * hl,
                       bx + ux * hl, by + uy * hl,
                       fill="white", width=2)
        cv.create_text(bx + ux * (hl + 12), by + uy * (hl + 12),
                       text=str(abs(m)), fill="white",
                       font=("DejaVu Sans", 7))

    # ---- escala de alabeo: marcas fijas en el borde superior del reloj
    for t in (-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60):
        ta = math.radians(t)
        sx, sy = math.sin(ta), -math.cos(ta)        # t=0 arriba, + a la derecha
        largo = 12 if t in (0, -30, 30, -60, 60) else 7
        cv.create_line(cx + sx * (r - largo), cy + sy * (r - largo),
                       cx + sx * r, cy + sy * r,
                       fill="white", width=2 if largo == 12 else 1)

    # puntero de alabeo: triangulo que gira con el avion
    ta = math.radians(roll)
    sx, sy = math.sin(ta), -math.cos(ta)
    px, py = math.cos(ta), math.sin(ta)             # perpendicular al radio
    tip  = (cx + sx * (r - 14), cy + sy * (r - 14))
    b1   = (cx + sx * (r - 27) + px * 6, cy + sy * (r - 27) + py * 6)
    b2   = (cx + sx * (r - 27) - px * 6, cy + sy * (r - 27) - py * 6)
    cv.create_polygon(tip, b1, b2, fill=AGUJA, outline="")

    # ---- bisel y simbolo del avion (fijo)
    cv.create_oval(cx - r - 6, cy - r - 6, cx + r + 6, cy + r + 6,
                   outline="#2a2a2a", width=10)
    cv.create_oval(cx - r, cy - r, cx + r, cy + r, outline=BORDE, width=2)

    cv.create_line(cx - 46, cy, cx - 16, cy, fill=AGUJA, width=4)
    cv.create_line(cx + 16, cy, cx + 46, cy, fill=AGUJA, width=4)
    cv.create_line(cx - 16, cy, cx - 8, cy + 7, fill=AGUJA, width=4)
    cv.create_line(cx + 16, cy, cx + 8, cy + 7, fill=AGUJA, width=4)
    cv.create_oval(cx - 4, cy - 4, cx + 4, cy + 4, fill=AGUJA, outline="")

    cv.create_text(cx, cy + r + 26, text="ACTITUD", fill=TEXTO,
                   font=("DejaVu Sans", 11, "bold"))
    cv.create_text(cx, cy + r + 46,
                   text="alabeo %+.0f°   cabeceo %+.0f°" % (roll, pitch),
                   fill=SUAVE, font=("DejaVu Sans", 9))


def dial(cx, cy, r, titulo, valor, vmin, vmax, unidad, decimales=0):
    """Reloj de aguja: barre 270 grados, empezando abajo a la izquierda."""
    bisel(cx, cy, r, titulo)

    ini, barrido = 225.0, -270.0           # grados, sentido horario
    frac = (valor - vmin) / float(vmax - vmin)
    frac = max(0.0, min(1.0, frac))

    # marcas cada 10% y numeros cada 20%
    for i in range(11):
        ang = math.radians(ini + barrido * (i / 10.0))
        ca, sa = math.cos(ang), -math.sin(ang)
        largo = 14 if i % 2 == 0 else 8
        cv.create_line(cx + ca * (r - largo), cy + sa * (r - largo),
                       cx + ca * (r - 2), cy + sa * (r - 2),
                       fill=BORDE, width=2 if i % 2 == 0 else 1)
        if i % 2 == 0:
            v = vmin + (vmax - vmin) * (i / 10.0)
            cv.create_text(cx + ca * (r - 30), cy + sa * (r - 30),
                           text="%g" % v, fill=SUAVE,
                           font=("DejaVu Sans", 8))

    # arco de valor
    cv.create_arc(cx - r + 8, cy - r + 8, cx + r - 8, cy + r - 8,
                  start=ini, extent=barrido * frac,
                  style=tk.ARC, outline=AGUJA, width=5)

    # aguja
    ang = math.radians(ini + barrido * frac)
    ca, sa = math.cos(ang), -math.sin(ang)
    cv.create_line(cx, cy, cx + ca * (r - 22), cy + sa * (r - 22),
                   fill=AGUJA, width=4)
    cv.create_oval(cx - 7, cy - 7, cx + 7, cy + 7, fill="#3a3a3a", outline=BORDE)

    cv.create_text(cx, cy + r + 46,
                   text=("%." + str(decimales) + "f %s") % (valor, unidad),
                   fill=SUAVE, font=("DejaVu Sans", 9))


# ----------------------------------------------------------------------
# DIBUJO
# ----------------------------------------------------------------------

ANCHO, ALTO = 900, 560
CY, R = 190, 115

def redibujar(enviado):
    cv.delete("all")

    horizonte(160, CY, R, tele['ROLL'], tele['PITCH'])
    dial(450, CY, R, "ALTURA", tele['ALT'], 0, ALT_MAX, "m")
    dial(740, CY, R, "POTENCIA", (tele['THR'] - 1000) / 10.0, 0, 100, "%")

    y = 366
    cv.create_line(30, y, ANCHO - 30, y, fill="#2a2a2a")

    # --- fila 1: modo enviado vs modo que reporta el avion
    y += 26
    cv.create_text(30, y, anchor="w", text="ENVIADO:", fill=SUAVE,
                   font=("DejaVu Sans", 11))
    cv.create_text(125, y, anchor="w", text=modo[1], fill=AGUJA,
                   font=("DejaVu Sans", 14, "bold"))

    # si el modo reportado no coincide con el enviado, algo no llego
    coincide = modo_avion.upper().startswith(modo[1][:4].upper())
    cv.create_text(400, y, anchor="w", text="AVION:", fill=SUAVE,
                   font=("DejaVu Sans", 11))
    cv.create_text(475, y, anchor="w", text=modo_avion,
                   fill=VERDE if coincide else ROJO,
                   font=("DejaVu Sans", 14, "bold"))

    # --- fila 2: estado de los tres ejes
    y += 28
    estado = "alabeo %s    cabeceo %s    guinada %s" % (
        ejes['roll'], ejes['pitch'], ejes['yaw'])
    cv.create_text(30, y, anchor="w", text=estado, fill=TEXTO,
                   font=("DejaVu Sans Mono", 12))
    cv.create_text(ANCHO - 30, y, anchor="e",
                   text="enlace: %s" % (PUERTO if puerto else "SIN CONEXION"),
                   fill=VERDE if puerto else ROJO, font=("DejaVu Sans", 10))

    # --- fila 3 y 4: trafico serial
    y += 26
    cv.create_text(30, y, anchor="w", text="TX  >>  %r" % enviado,
                   fill=SUAVE, font=("DejaVu Sans Mono", 10))
    y += 20
    cv.create_text(30, y, anchor="w", text="RX  <<  %s" % ultima_linea[:96],
                   fill=SUAVE, font=("DejaVu Sans Mono", 10))

    # --- ayuda
    y += 30
    cv.create_text(30, y, anchor="w",
                   text="a/d alabeo    w/s cabeceo    j/l guinada",
                   fill="#666666", font=("DejaVu Sans", 9))
    y += 18
    cv.create_text(30, y, anchor="w",
                   text="p armar    i despegue    u crucero    o aterrizaje    "
                        "espacio desarmar    ESC panico    Ctrl+Q salir",
                   fill="#666666", font=("DejaVu Sans", 9))


ultimo_enviado = ""

def ciclo_serial():
    """Enlace con el avion. Va rapido y NO dibuja: el control no puede
    depender de lo que tarde el canvas en repintarse."""
    global ultimo_enviado
    recibir()
    ultimo_enviado = enviar()
    raiz.after(PERIODO_MS, ciclo_serial)


def ciclo_dibujo():
    """Repintado de los instrumentos, a su propio ritmo."""
    redibujar(ultimo_enviado)
    raiz.after(PERIODO_DIBUJO_MS, ciclo_dibujo)


# ----------------------------------------------------------------------

abrir_puerto()

raiz = tk.Tk()
raiz.title("Black Bird - Estacion en tierra")
raiz.configure(bg=FONDO)
raiz.resizable(False, False)

cv = tk.Canvas(raiz, width=ANCHO, height=ALTO, bg=FONDO, highlightthickness=0)
cv.pack()

raiz.bind("<KeyPress>", al_presionar)
raiz.bind("<KeyRelease>", al_soltar)
raiz.bind("<Control-q>", lambda e: raiz.destroy())

ciclo_serial()
ciclo_dibujo()
raiz.mainloop()

if puerto:
    puerto.close()
