import rclpy

from turtlebot4_navigation.turtlebot4_navigation import CreateBotDirections, CreatBotNavigator


def main():
    rclpy.init()

    navigator = CreatBotNavigator()

    # Start on dock
    #if not navigator.getDockedStatus():
    #    navigator.info('Docking before intialising pose')
    #    navigator.dock()

    # Set initial pose
    initial_pose = navigator.getPoseStamped([0.0, 0.0], CreateBotDirections.NORTH)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Set goal poses
    goal_pose = navigator.getPoseStamped([13.0, 5.0], CreateBotDirections.EAST)

    # Undock
    navigator.undock()

    # Go to each goal pose
    navigator.startToPose(goal_pose)

    rclpy.shutdown()


if __name__ == '__main__':
    main()