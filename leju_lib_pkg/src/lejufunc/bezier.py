from motion.TrajectoryPlan import TrajectoryPlanning
from motion.bezierPlan import MultiBeizerPlan
from motion.motionControl import SendJointCommand, WaitTrajectoryExecute, GetJointAngle, SetBodyhubTo_setStatus
import rospy
MIN_INTERVAL = 10.0


def points_2_frames(points, interval=MIN_INTERVAL):
    results = [[] for _ in range(len(points[0]))]
    for point in points:
        for i in range(len(point)):
            results[i].append(point[i].y)
    return results


def get_bezier_frames(p0, p1, p2, act_time, frame_time=MIN_INTERVAL):
    tpObject = TrajectoryPlanning(len(p0), frame_time)
    tpObject.setInterval(act_time)
    tpObject.planningBegin(p0, p1)
    if p2 == None:
        results = points_2_frames(tpObject.planningEnd())
    else:
        results = points_2_frames(tpObject.planning(p2))
    return results


def generator_bezier(poseList):
    tpObject = TrajectoryPlanning(22, 10.0)
    interval = poseList[1][1] if poseList[1][1] > MIN_INTERVAL else MIN_INTERVAL
    tpObject.setInterval(interval)
    tpObject.planningBegin(poseList[0][0], poseList[1][0])
    for poseIndex in range(2, len(poseList)):
        interval = poseList[poseIndex][1] if poseList[poseIndex][1] > MIN_INTERVAL else MIN_INTERVAL
        tpObject.setInterval(interval)
        trajectoryPoint = tpObject.planning(poseList[poseIndex][0])
        yield points_2_frames(trajectoryPoint), poseList[poseIndex-1][2] / 1000.0, False

    trajectoryPoint = tpObject.planningEnd()
    yield points_2_frames(trajectoryPoint), poseList[-1][2] / 1000.0, True


def get_custom_points(act_frames, need_first_frame=True):

    act_frames = { int(key): act_frames[key] for key in act_frames }
    joint_ids = act_frames.keys()
    joint_ids.sort()

    def get_valid_value():
        min_values = [-160, -75, -75, -100, -43, -28, -160, -30, -70,
                      0, -62, -28, -160, -85, -90, -90, -28,  0, -35,  0, -85, -20]
        max_values = [160,  30,  70,    0,  62,  28,  160,  75,  75,
                      100,  43,  28,   90,  28,   0, 160,  85, 90,   0, 35,  85,  30]
        angleLimitSet = [ (min_values[i - 1],max_values[i - 1]) for i in joint_ids ]
        return angleLimitSet

    joint_current_angles = GetJointAngle(joint_ids)
    frames = [[] for _ in joint_ids]
    for joint_index, key in enumerate(joint_ids):
        for index, value in enumerate(act_frames[key]):
            if need_first_frame and index == 0 and value[0][0] != 0:
                current_angle = joint_current_angles[joint_index]
                frames[joint_index].append(((0, current_angle), (0, 0), (0, 0)))
            (abs_time, angle), left_point, right_point = act_frames[key][index]
            current_frame = (abs_time, angle), left_point, right_point
            frames[joint_index].append(current_frame)

    def get_point(points):
        if not points:
            return
        for i in range(len(max(points, key=len))):
            joint_position = []
            for point in points:
                joint_position.append(point[min(i, len(point) - 1)].y)
            yield joint_position
            # SendJointCommand(2, joint_ids, joint_position)

    plan_obj = MultiBeizerPlan(len(joint_ids), MIN_INTERVAL, frames, get_valid_value())
    while not rospy.is_shutdown():
        plan_obj.planing(200)
        points = plan_obj.getTrajectory(100)
        for p in get_point(points):
            yield p
        # WaitTrajectoryExecute()
        if plan_obj.sendComplete():
            # WaitTrajectoryExecute(True)
            break



def send_custom_bezier(act_frames, need_first_frame=True):
    act_frames = { int(key): act_frames[key] for key in act_frames }

    joint_ids = act_frames.keys()
    joint_ids.sort()
    for points in get_custom_points(act_frames, need_first_frame):
        SendJointCommand(2, joint_ids, points)
        WaitTrajectoryExecute()
    WaitTrajectoryExecute(True)



# auto bezier example code:
if __name__ == '__main__':
    # poseList = [
    #     [0,-1,21,-40,-18,-1,0,1,-21,40,18,1,-16,-58,-21,16,58,21,0,0,0,0],
    #     [0,-1,21,-40,-18,-1,0,1,-21,40,18,1,-16,-58,-21,16,58,21,0,0,0,0],
    # ]
    # start = [0,-1,21,-40,-18,-1,0,1,-21,40,18,1,-16,-58,-21,16,58,21,0,0,0,0]
    # stop = [0,-1,21,-40,-18,-1,0,1,-21,40,18,1,-16,-58,-21,16,58,21,0,0,0,0]
    # for pos in get_bezier_frames(start, stop, 100):
    #     print(pos)
    rospy.init_node('bezierPlan_node', anonymous=True)
    import time
    time.sleep(0.2)
    # rospy.on_shutdown(rosShutdownHook)
    rospy.loginfo('node runing...')

    print(SetBodyhubTo_setStatus(2))
    act_frames = {
        1: [
            ((500, 50), (-200, 0), (200, 0)),
            ((1000, 100), (-200, 0), (200, 0))
        ]
    }    
    send_custom_bezier(act_frames)
    # print(rospy.is_shutdown())
