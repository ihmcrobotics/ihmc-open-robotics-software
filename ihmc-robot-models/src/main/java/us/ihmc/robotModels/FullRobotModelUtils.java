package us.ihmc.robotModels;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;

import java.util.ArrayList;
import java.util.zip.CRC32;

public class FullRobotModelUtils
{
   public static OneDoFJointBasics[] getAllJointsExcludingHands(FullHumanoidRobotModel model)
   {
      ArrayList<OneDoFJointBasics> joints = new ArrayList<>();
      getAllJointsExcludingHands(joints, model);
      return joints.toArray(new OneDoFJointBasics[joints.size()]);
   }

   public static void getAllJointsExcludingHands(ArrayList<OneDoFJointBasics> jointsToPack, FullHumanoidRobotModel model)
   {
      model.getOneDoFJoints(jointsToPack);
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics hand = model.getHand(robotSide);
         if (hand != null)
         {
            OneDoFJointBasics[] fingerJoints = MultiBodySystemTools.filterJoints(MultiBodySystemTools.collectSubtreeJoints(hand), OneDoFJointBasics.class);
            for (OneDoFJointBasics fingerJoint : fingerJoints)
            {
               jointsToPack.remove(fingerJoint);
            }
         }
      }
   }
   public static int calculateJointNameHash(OneDoFJointBasics[] joints, ForceSensorDefinition[] forceSensorDefinitions, IMUDefinition[] imuDefinitions)
   {
      CRC32 crc = new CRC32();
      for (OneDoFJointBasics joint : joints)
      {
         crc.update(joint.getName().getBytes());
      }

      for (ForceSensorDefinition forceSensorDefinition : forceSensorDefinitions)
      {
         crc.update(forceSensorDefinition.getSensorName().getBytes());
      }

      for (IMUDefinition imuDefinition : imuDefinitions)
      {
         crc.update(imuDefinition.getName().getBytes());
      }

      return (int) crc.getValue();
   }

   public static void checkJointNameHash(int expectedJointNameHash, int actualJointNameHash)
   {
      if (expectedJointNameHash != actualJointNameHash)
      {
         throw new RuntimeException("Joint names do not match");
      }
   }
}
