# stewart_platform
6-DoF Gough-Stewart platform model

Implented a stewart platform state calculator which performs
direct calculations of the 6-DoF platform state and servo leg
angles by provided tranlsation vector and rotation parameters
(roll, pitch, yaw).

Stewart platform model geometry should be specified in a YAML
file. See `configs/platform_shuayk.yml` as an example.

# UI

PyQt5+OpenGL based UI can be used to visualise and control
platform. If a provided configuration is impossible the model
stucks in last correct state (UI highlights the affected servos
as well).

Type `python qglui.py` to run UI.

# Dependencies

Python3, PyYaml, PyQt5, PyOpenGL, numpy.