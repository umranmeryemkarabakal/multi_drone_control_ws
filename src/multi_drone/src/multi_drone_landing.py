#!/usr/bin/env python
import rospy
from mavros_msgs.srv import CommandTOL
import threading

def land_drone(namespace):
    rospy.loginfo(f"[{namespace}] İniş işlemi başlatılıyor...")

    land_srv = f"/{namespace}/mavros/cmd/land"

    try:
        rospy.wait_for_service(land_srv, timeout=10)
    except rospy.ROSException:
        rospy.logerr(f"[{namespace}] {land_srv} servisine bağlanılamadı.")
        return

    land_client = rospy.ServiceProxy(land_srv, CommandTOL)

    try:
        land_resp = land_client(min_pitch=0.0, yaw=0.0, latitude=0.0, longitude=0.0, altitude=0.0)
        if land_resp.success:
            rospy.loginfo(f"[{namespace}] Landing komutu gönderildi.")
        else:
            rospy.logerr(f"[{namespace}] Landing başarısız!")
            return

        rospy.sleep(30)

    except rospy.ServiceException as e:
        rospy.logerr(f"[{namespace}] Servis çağrısı hatası: {e}")

if __name__ == "__main__":
    rospy.init_node("multi_drone_landing_node")

    drone_namespaces = ["drone1", "drone2", "drone3"]

    threads = []
    for ns in drone_namespaces:
        t = threading.Thread(target=land_drone, args=(ns,))
        t.start()
        threads.append(t)

    for t in threads:
        t.join()
