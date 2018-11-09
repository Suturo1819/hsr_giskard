#!/usr/bin/env python

import rospy

# Input-Wrapper fuer einen 3d-Punkt
from giskardpy.input_system import Point3Input

# Giskards Robotermodell
from giskardpy.symengine_robot import Robot

# Giskard-Regler
from giskardpy.symengine_controller import SymEngineController

# Hilfsfunktionen
from giskardpy.symengine_wrappers import *

# Softconstraint
from giskardpy.qp_problem_builder import SoftConstraint

from sensor_msgs.msg import JointState as JointStateMsg
from geometry_msgs.msg import Point as PointMsg

# Konvertiert Liste l in ein Symbol
def to_expr(l):
    return Symbol(''.join(l))

# Ermittelt freie Variablen und benoetigte Gelenke aus constraints und Roboter. Filter kann genutzt werden, um bestimmte Gelenke auszuschliessen.
def configure_controller(controller, robot, soft_cs, filter=set()):
    free_symbols = set()
    
    # Freie Variablen in Softconstraints finden
    for sc in soft_cs.values():
        for f in sc:
            if hasattr(f, 'free_symbols'):
                free_symbols = free_symbols.union(f.free_symbols)
    
    # Kontrollierbare Gelenke auswaehlen
    controller.set_controlled_joints([j for j in robot.get_joint_names() if robot.joint_to_symbol_map[j] in free_symbols and j not in filter])

    # Freie Variablen in Jointconstraints finden
    for jc in controller.joint_constraints.values():
        for f in jc:
            if hasattr(f, 'free_symbols'):
                free_symbols = free_symbols.union(f.free_symbols)
    
    # Freie Variablen in Hardconstraints finden
    for hc in controller.hard_constraints.values():
        for f in hc:
            if hasattr(f, 'free_symbols'):
                free_symbols = free_symbols.union(f.free_symbols)

    # Constraint(s) und freie Variablen an den Regler ueberreichen
    controller.update_soft_constraints(soft_cs, free_symbols)

    return free_symbols



class TestController(object):
    """docstring for TestController"""
    def __init__(self, urdf_string):
        super(TestController, self).__init__()
        
        # Roboter aus URDF erzeugen
        self.robot = Robot(urdf_string)
        self.robot.parse_urdf()

        # Regler fuer den Roboter anlegen
        self.controller = SymEngineController(self.robot, '.controller_cache/')

        # Transformation vom Greifer zur Basis des Roboters erzeugen
        gripper_pose = self.robot.get_fk_expression('base_link', 'hand_palm_link')

        # Parameterisierbaren Punkt erzeugen 
        self.point_input = Point3Input(to_expr, 'goal')

        # Mathematischen Punkt vom Punkt-Input erzeugen lassen
        goal_point = self.point_input.get_expression()

        # Distanz zwischen Zielpunkt und Greiferposition
        d = norm(goal_point - position_of(gripper_pose))

        a = dot(unitX, gripper_pose * unitZ)

        c_dist_angular = SoftConstraint(1-a, 1-a, 1, a)

        # Constraint, der die Distanz Richtung 0 zwingt
        c_dist = SoftConstraint(-d, -d, 1, d)

        free_symbols = configure_controller(self.controller, self.robot, {'dist': c_dist, 'dist_angular': c_dist_angular})

        # Dictionary fuer die derzeitige Variablenbelegung. Initial sind alle Variablen 0 
        self.cur_subs = {s: 0 for s in free_symbols}

        # Publisher fuer Kommandos
        self.pub_cmd = rospy.Publisher('/hsr/commands/joint_velocities', JointStateMsg, queue_size=1)

        # Abonent fuer Zielpunkt
        self.sub_point = rospy.Subscriber('/goal_point', PointMsg, self.cb_point, queue_size=1)

        # Abonent fuer Gelenkpositionen
        self.sub_js = rospy.Subscriber('/hsr/joint_states', JointStateMsg, self.cb_js, queue_size=1)


    def cb_point(self, msg):
        # Werte der Nachricht in Variablenbelegung uebertragen
        self.cur_subs[self.point_input.x] = msg.x
        self.cur_subs[self.point_input.y] = msg.y
        self.cur_subs[self.point_input.z] = msg.z


    def cb_js(self, msg):
        # Map von Jointnamen auf Variablen
        robot_joint_symbols = self.robot.joint_to_symbol_map

        # ueber alle Felder iterieren
        for x in range(len(msg.name)):
            # Wenn Joint tatsaechlich im Roboter definiert ist
            if msg.name[x] in robot_joint_symbols:
                # Variablenwert aktualisieren mit neuer Position
                self.cur_subs[robot_joint_symbols[msg.name[x]]] = msg.position[x]

        # Kommandos generieren lassen. Variablenbelegung muss als {str: float} erfolgen.
        cmd = self.controller.get_cmd({str(s): p for s, p in self.cur_subs.items()})

        # Kommandonachricht anlegen
        cmd_msg = JointStateMsg()

        # Kommandozeitstempel setzen
        cmd_msg.header.stamp = rospy.Time.now()

        # Positionen und Kraefte auf 0 setzen
        cmd_msg.position = [0]*len(cmd)
        cmd_msg.effort   = [0]*len(cmd)

        # Kommando in Nachricht schreiben
        for j, v in cmd.items():
            cmd_msg.name.append(j)
            cmd_msg.velocity.append(v)

        # Kommando abschicken
        self.pub_cmd.publish(cmd_msg)

if __name__ == '__main__':

    # An ROS anmelden
    rospy.init_node('first_controller')

    # Reglungsknoten instanziieren und Roboterbeschreibung vom Parameterserver holen
    node = TestController(rospy.get_param('hsr/robot_description'))

    # Programm am Beenden hindern
    while not rospy.is_shutdown():
        pass


