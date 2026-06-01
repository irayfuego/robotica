# Datos de los encoders de los 4 motores TT
std_msgs/Header header
float32[4] position    # posición en radianes [FL, FR, RL, RR]
float32[4] velocity    # velocidad en rad/s [FL, FR, RL, RR]
int32[4]   ticks       # ticks de encoder raw [FL, FR, RL, RR]
