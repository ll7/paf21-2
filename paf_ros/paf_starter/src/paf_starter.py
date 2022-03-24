#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from paf_messages.msg import PafRoutingRequest

# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """
    Waits until config/competition manager/subscribers are ready
    and then starts the competition.
    """
    rospy.init_node("paf_starter_node", anonymous=True)
    rospy.loginfo("[paf_starter] PAF_START RUNNING")

    # Wait until config yaml is loaded
    while not rospy.has_param("competition/traffic_rules"):
        rospy.sleep(1)
        rospy.loginfo_throttle(2, "[paf_starter] Waiting for the config yaml to be loaded.")

    # Publish the rule settings to the planner
    traffic_rules = rospy.get_param("competition/traffic_rules")
    rospy.logwarn(f"[paf_starter] Publishing rule settings. Rules enabled = {traffic_rules}")
    rules_publisher: rospy.Publisher = rospy.Publisher("/paf/paf_local_planner/rules_enabled", Bool, queue_size=1)

    # Wait until rules subscribers are initialized
    while rules_publisher.get_num_connections() < 1:
        rospy.sleep(1)

    rules_publisher.publish(Bool(traffic_rules))

    # Wait until the competition manager wants to start
    rospy.logwarn("[paf_starter] Waiting for the competition manager to start the competition...")
    ready_for_ego = False
    while not rospy.is_shutdown() and not ready_for_ego:
        if rospy.has_param("competition/ready_for_ego"):
            ready_for_ego = rospy.get_param("competition/ready_for_ego")
            if ready_for_ego:
                rospy.logwarn("[paf_starter] The competition manager is ready.")
            else:
                rospy.logwarn_throttle(2, "[paf_starter] The competition manager is NOT ready, yet.")
        else:
            ready_for_ego = False
            rospy.logwarn_throttle(2, "[paf_starter] ready_for_ego=PARAMETER NOT FOUND")

    # Now we received the start signal from the competition manager
    # Publish the routing request to our planner
    goal_x = rospy.get_param("competition/goal/position/x")
    goal_y = rospy.get_param("competition/goal/position/y")

    rospy.logwarn(f"[paf_starter] Publishing goal settings. Coordinates = ({goal_x},{goal_y})")
    goal_publisher: rospy.Publisher = rospy.Publisher(
        "/paf/paf_local_planner/routing_request", PafRoutingRequest, queue_size=1
    )

    routing_request = PafRoutingRequest()
    routing_request.target = [goal_x, goal_y]
    routing_request.rules_enabled = traffic_rules

    # Wait until routing subscribers are initialized
    while goal_publisher.get_num_connections() < 1:
        rospy.sleep(1)

    goal_publisher.publish(routing_request)


if __name__ == "__main__":
    main()
