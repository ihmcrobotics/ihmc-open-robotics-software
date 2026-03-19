package us.ihmc.avatar.drcRobot;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.robotModels.FullRobotModelTestTools.RandomFullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class ROS2HumanoidFramesTest
{
   private static final double EPSILON = 1E-7;

   @Test
   public void testManyManyUpdates()
   {
      Random random = new Random();
      RandomFullHumanoidRobotModel fullRobotModel = new RandomFullHumanoidRobotModel(random);
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);

      ArrayList<OneDoFJointBasics> joints = new ArrayList<>();
      fullRobotModel.getOneDoFJoints(joints);

      ROS2HumanoidFrames ros2HumanoidFrames = new ROS2HumanoidFrames(referenceFrames, fullRobotModel);

      for (int i = 0; i < 10000; ++i)
      {
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, -0.5 * Math.PI, 0.5 * Math.PI, joints);
         referenceFrames.updateFrames();
         assertDoesNotThrow(ros2HumanoidFrames::update);

         if (referenceFrames.getHeadFrame() != null)
            EuclidCoreTestTools.assertEquals(referenceFrames.getHeadFrame().getTransformToRoot(),
                                             ros2HumanoidFrames.getGazeFrame().getTransformToRoot(),
                                             EPSILON);
         if (referenceFrames.getChestFrame() != null)
            EuclidCoreTestTools.assertEquals(referenceFrames.getChestFrame().getTransformToRoot(),
                                             ros2HumanoidFrames.getTorsoFrame().getTransformToRoot(),
                                             EPSILON);
         if (referenceFrames.getPelvisFrame() != null)
            EuclidCoreTestTools.assertEquals(referenceFrames.getPelvisFrame().getTransformToRoot(),
                                             ros2HumanoidFrames.getBaseLinkFrame().getTransformToRoot(),
                                             EPSILON);

         for (RobotSide side : RobotSide.values)
         {
            if (fullRobotModel.getHand(side) != null)
               EuclidCoreTestTools.assertEquals(fullRobotModel.getHand(side).getParentJoint().getFrameAfterJoint().getTransformToRoot(),
                                                ros2HumanoidFrames.getWristFrame(side).getTransformToRoot(),
                                                EPSILON);
            if (referenceFrames.getHandFrame(side) != null)
               EuclidCoreTestTools.assertEquals(referenceFrames.getHandFrame(side).getTransformToRoot(),
                                                ros2HumanoidFrames.getGripperFrame(side).getTransformToRoot(),
                                                EPSILON);
            if (referenceFrames.getFootFrame(side) != null)
               EuclidCoreTestTools.assertEquals(referenceFrames.getFootFrame(side).getTransformToRoot(),
                                                ros2HumanoidFrames.getAnkleFrame(side).getTransformToRoot(),
                                                EPSILON);
            if (referenceFrames.getSoleFrame(side) != null)
               EuclidCoreTestTools.assertEquals(referenceFrames.getSoleFrame(side).getTransformToRoot(),
                                                ros2HumanoidFrames.getSoleFrame(side).getTransformToRoot(),
                                                EPSILON);
         }
      }

      ros2HumanoidFrames.remove();
   }
}
