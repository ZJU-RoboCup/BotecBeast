import rospy
from hollow_out import Hollow
from landmine import Landmine
from door import Door
from narrow_bridge import NarrowBridge
from kick_ball import KickBall
from hurdle import Hurdle
from image_manager import ImageManager
from ball2stairs import Ball2Stairs
from up_stairs import UpStairs
from up2down import Up2Down
from down_stairs import DownStairs


if __name__ == "__main__":
    rospy.init_node("all_levels")
    manager = ImageManager() 
    Hollow(manager)
    Landmine(manager)
    Hurdle().main()
    Door(manager)
    NarrowBridge(manager)
    KickBall(manager)
    Ball2Stairs(manager)
    UpStairs().main()
    Up2Down()
    DownStairs().main()



