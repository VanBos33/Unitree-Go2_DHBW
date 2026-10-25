#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def main():
    # ROS 2 initialisieren
    rclpy.init()
    
    # Der BasicNavigator ist unser "Wrapper" für Nav2
    navigator = BasicNavigator()

    print("Warte darauf, dass der Nav2 Stack hochfährt...")
    navigator.waitUntilNav2Active()
    print("Nav2 ist aktiv! Bereite Ziel vor...")

    # Zielkoordinate definieren (Diese Werte kommen später vom Dashboard/Student 1)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    
    # X und Y Koordinaten auf der Karte in Metern
    goal_pose.pose.position.x = 2.0 
    goal_pose.pose.position.y = 1.0 
    # w=1.0 bedeutet: Der Roboter schaut in Fahrtrichtung (keine Drehung auf der Stelle)
    goal_pose.pose.orientation.w = 1.0 

    # Roboter losschicken
    print(f"Sende Roboter zu X: {goal_pose.pose.position.x}, Y: {goal_pose.pose.position.y}")
    navigator.goToPose(goal_pose)

    # Warten, bis der Roboter ankommt, und Feedback geben
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            # Gibt alle paar Sekunden die verbleibende Distanz aus
            print(f'Entfernung zum Ziel: {feedback.distance_remaining:.2f} Meter', end='\r')

    # Endergebnis prüfen (Schnittstelle für Student 4)
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('\n[ERFOLG] Ziel erreicht! State Machine kann nun Bilderkennung (Student 2) starten.')
        # Hier würde die State Machine getriggert werden
    elif result == TaskResult.CANCELED:
        print('\n[ABBRUCH] Navigation wurde abgebrochen.')
    elif result == TaskResult.FAILED:
        print('\n[FEHLER] Navigation fehlgeschlagen! Hindernis im Weg?')

    # Beenden
    rclpy.shutdown()

if _name_ == '_main_':
    main()
