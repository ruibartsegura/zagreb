#!/usr/bin/env python3
import rospy
from std_srvs.srv import Empty

def call_services():
    rospy.init_node('service_caller', anonymous=True)
    
    # Esperar a que el servicio /kill esté disponible
    rospy.wait_for_service('/kill')
    try:
        # Crear el proxy para el servicio /kill
        kill_service = rospy.ServiceProxy('/kill', Empty)
        # Llamar al servicio kill sin argumentos
        kill_service()
        rospy.loginfo("Turtle1 killed")

        # Esperar a que el servicio /spawn esté disponible
        rospy.wait_for_service('/spawn')
        spawn_service = rospy.ServiceProxy('/spawn', Empty)
        # Llamar al servicio spawn con los argumentos correctos
        spawn_service(1.0, 1.0, 1.57, 'Turtle1')
        rospy.loginfo("Turtle1 spawned at (1.0, 1.0)")

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == '__main__':
    try:
        call_services()
    except rospy.ROSInterruptException:
        pass
