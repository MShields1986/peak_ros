Header header

# LTPA Configuration
int32 dof
int32 gate_start
int32 gate_end
int32 ascan_length
int32 num_ascans

# Reconstruction Configuration
# TODO: Consider sending this as a separate one time latched message rather than repeated here
int32                           digitisation_rate     # MHz
int32                           n_elements
float64                         element_pitch         # mm
float64                         inter_element_spacing # mm
float64                         element_width         # mm
float64                         vel_wedge             # m/s
float64                         vel_couplant          # m/s
float64                         vel_material          # m/s
float64                         wedge_angle           # degrees
float64                         wedge_depth           # mm
float64                         couplant_depth        # mm
float64                         specimen_depth        # mm

# LTPA Data
peak_ros/Ascan[] ascans

int32 max_amplitude
