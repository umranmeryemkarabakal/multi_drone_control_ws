#!/usr/bin/env python
import rospy
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
import threading
import time

def wait_until_service_ready(service_name, srv_type, timeout=15.0):
    start_time = time.time()
    while not rospy.is_shutdown():
        try:
            rospy.wait_for_service(service_name, timeout=1.0)
            # Servise bağlantı kurmayı da dene
            proxy = rospy.ServiceProxy(service_name, srv_type)
            proxy.wait_for_service(timeout=1.0)
            return proxy
        except (rospy.ServiceException, rospy.ROSException):
            if time.time() - start_time > timeout:
                rospy.logerr(f"[{service_name}] servis zaman aşımına uğradı.")
                return None
            rospy.sleep(0.5)
def takeoff_drone(namespace, altitude=10.0):
    rospy.loginfo(f"[{namespace}] İşlem başlatıldı.")

    set_mode_srv = f"/{namespace}/mavros/set_mode"
    arming_srv = f"/{namespace}/mavros/cmd/arming"
    takeoff_srv = f"/{namespace}/mavros/cmd/takeoff"

    set_mode_client = wait_until_service_ready(set_mode_srv, SetMode)
    arming_client = wait_until_service_ready(arming_srv, CommandBool)
    takeoff_client = wait_until_service_ready(takeoff_srv, CommandTOL)

    if not all([set_mode_client, arming_client, takeoff_client]):
        rospy.logerr(f"[{namespace}] Servisler hazır değil, işlem iptal edildi.")
        return

    try:
        mode_resp = set_mode_client(custom_mode="GUIDED")
        if hasattr(mode_resp, 'mode_sent') and mode_resp.mode_sent:
            rospy.loginfo(f"[{namespace}] GUIDED modu aktif.")
        else:
            rospy.logerr(f"[{namespace}] Mod ayarlanamadı.")
            return

        arm_resp = arming_client(True)
        if arm_resp.success:
            rospy.loginfo(f"[{namespace}] Motorlar çalıştı.")
        else:
            rospy.logerr(f"[{namespace}] Motorlar çalıştırılamadı.")
            return

        takeoff_resp = takeoff_client(
            altitude=altitude, latitude=0.0, longitude=0.0, min_pitch=0.0, yaw=0.0
        )
        if takeoff_resp.success:
            rospy.loginfo(f"[{namespace}] Takeoff başlatıldı ({altitude} m).")
        else:
            rospy.logerr(f"[{namespace}] Takeoff başarısız.")

    except rospy.ServiceException as e:
        rospy.logerr(f"[{namespace}] Servis hatası: {e}")

if __name__ == "__main__":
    rospy.init_node("multi_drone_takeoff_node")

    # MAVROS için namespace'ler
    drone_namespaces = ["drone1", "drone2", "drone3"]

    threads = []
    for ns in drone_namespaces:
        t = threading.Thread(target=takeoff_drone, args=(ns, 10.0))
        t.start()
        threads.append(t)

    for t in threads:
        t.join()
