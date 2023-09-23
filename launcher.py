	
import subprocess
import gi

gi.require_version("Gtk", "3.0")
from gi.repository import Gtk


# Bash commands executed in each terminal
terminal1 = """
usuario=$USER
sudo -E su -c "
cd /home/$usuario/PX4-Autopilot;
export PX4_HOME_LAT=-22.001333;
export PX4_HOME_LON=-47.934152;
export PX4_HOME_ALT=847.142652;
make px4_sitl gazebo;
" root
"""

terminal2 = """
usuario=$USER
sudo -E su -c "
cd /home/$usuario/harpia_test;
source devel/setup.bash;
roslaunch harpia.launch;
" root
"""

terminal3 = """
usuario=$USER
sudo -E su -c "
cd /home/$usuario/harpia_test;
source devel/setup.bash;
rosrun mission_planning test_client.py {mission_id} {map_id} {drone_id};
" root
"""

class GridWindow(Gtk.Window):
    def __init__(self):

        super().__init__(title="Harpia")

        button_initSim = Gtk.Button(label="Iniciar simulação")
        button_initSim.connect("clicked", self.on_click_init_sim)

        button_initHarpia = Gtk.Button(label="Iniciar harpia")
        button_initHarpia.connect("clicked", self.on_click_init_harpia)

        button_initMission = Gtk.Button(label="Iniciar missão")
        button_initMission.connect("clicked", self.on_click_init_mission)

        self.entry_missionID = Gtk.Entry()
        self.entry_missionID.set_text("ID Missão")
        
        self.entry_mapID = Gtk.Entry()
        self.entry_mapID.set_text("ID Mapa")

        self.entry_droneID = Gtk.Entry()
        self.entry_droneID.set_text("ID Drone")

        grid = Gtk.Grid()

        grid.add(button_initSim)
        grid.attach_next_to(button_initHarpia, button_initSim, Gtk.PositionType.BOTTOM, 1, 1)
        grid.attach_next_to(button_initMission, button_initHarpia, Gtk.PositionType.BOTTOM, 1, 1)
        grid.attach(self.entry_missionID, 1, 0, 1, 1)
        grid.attach(self.entry_mapID, 1, 1, 1, 1)
        grid.attach(self.entry_droneID, 1, 2, 1, 1)
        

        self.add(grid)
        
    # Events of every button click
    def on_click_init_sim(self, button):
        subprocess.call(['gnome-terminal', '--', 'bash', '-c', terminal1])

    def on_click_init_harpia(self, button):
        subprocess.call(['gnome-terminal', '--', 'bash', '-c', terminal2])

    def on_click_init_mission(self, button): 
        mission_id = self.entry_missionID.get_text()
        map_id = self.entry_mapID.get_text()
        drone_id = self.entry_droneID.get_text()

        terminal3_updated = terminal3.format(mission_id=mission_id, map_id=map_id, drone_id=drone_id)
        subprocess.call(['gnome-terminal', '--', 'bash', '-c', terminal3_updated])

win = GridWindow()
win.connect("destroy", Gtk.main_quit)
win.set_position(1)
win.show_all()
win.resize(300, 100)

Gtk.main()
