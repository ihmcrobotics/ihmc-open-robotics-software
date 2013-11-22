package us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers;

import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.ArmJointName;
import us.ihmc.darpaRoboticsChallenge.handControl.iRobot.iRobotHandModel.iRobotFingerJointName;
import us.ihmc.darpaRoboticsChallenge.handControl.iRobot.iRobotHandModel.iRobotFingerName;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.net.ObjectConsumer;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

public class HandJointAngleProvider implements ObjectConsumer<HandJointAnglePacket>
{
   private final SideDependentList<AtomicReference<HandJointAnglePacket>> packets = new SideDependentList<AtomicReference<HandJointAnglePacket>>();
   
   protected final SideDependentList<EnumMap<iRobotFingerName, double[]>> fingerJointPositions = new SideDependentList<EnumMap<iRobotFingerName, double[]>>();

   public HandJointAngleProvider()
   {
//      for (RobotSide robotSide : RobotSide.values)
//      {
//         packets.put(robotSide, new AtomicReference<HandJointAnglePacket>());
//         
//         EnumMap<iRobotFingerJointName, Double> jointAngles = new EnumMap<iRobotFingerJointName, Double>(iRobotFingerJointName.class);
//
//         for(iRobotFingerJointName jointName : iRobotFingerJointName.values())
//         {
//            jointAngles.put(jointName, 0.0);
//         }
//
//         fingerJointPositions.put(robotSide, jointAngles);
//      }
   }

   public boolean checkForNewArmJointPositions(RobotSide robotSide)
   {
//      HandJointAnglePacket object = packets.get(robotSide).getAndSet(null);
//      if(object != null)
//      {
//         EnumMap<iRobotFingerJointName, Double> jointAngles = fingerJointPositions.get(robotSide);
//
//         jointAngles.put(iRobotFingerJointName.FLEXIBLE_JOINT_FLEX_FROM_1_TO_2, object.getIndexJoints()[0]);
//         return true;
//      }
//      else
//      {
//         return false;
//      }
      return false;
   }

   public Map<OneDoFJoint, Double> getDesiredArmJointPositions(SDFFullRobotModel fullRobotModel, RobotSide robotSide)
   {
      Map<OneDoFJoint, Double> jointPositions = new LinkedHashMap<OneDoFJoint, Double>();
//      EnumMap<ArmJointName, Double> jointAngles = armJointPositions.get(robotSide);
//
//      for (ArmJointName armJointName : jointAngles.keySet())
//      {
//         OneDoFJoint armJoint = fullRobotModel.getArmJoint(robotSide, armJointName);
//         Double jointAngle = jointAngles.get(armJointName);
//
//         jointPositions.put(armJoint, jointAngle);
//      }

      return jointPositions;
   }

   public void consumeObject(HandJointAnglePacket object)
   {
      packets.get(object.getRobotSide()).set(object);
   }

   public EnumMap<ArmJointName, Double> peekAtDesiredArmJointPostionEnumMap(RobotSide robotSide)
   {
      return null;// armJointPositions.get(robotSide);
   }

   public EnumMap<ArmJointName, Double> getDesiredArmJointPostionEnumMap(RobotSide robotSide)
   {
      return null;//armJointPositions.get(robotSide);
   }

}
