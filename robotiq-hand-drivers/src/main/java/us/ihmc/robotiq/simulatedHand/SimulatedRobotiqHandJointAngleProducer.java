package us.ihmc.robotiq.simulatedHand;

import java.util.EnumMap;

import controller_msgs.msg.dds.HandJointAnglePacket;
import us.ihmc.avatar.handControl.packetsAndConsumers.HandJointAngleCommunicator;
import us.ihmc.avatar.handControl.packetsAndConsumers.HandSensorData;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotiq.model.RobotiqHandModel.RobotiqHandJointNameMinimal;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;

public class SimulatedRobotiqHandJointAngleProducer
{
   private final SideDependentList<EnumMap<RobotiqHandJointNameMinimal, OneDegreeOfFreedomJoint>> handJoints = SideDependentList.createListOfEnumMaps(RobotiqHandJointNameMinimal.class);

   private final SideDependentList<Boolean> hasRobotiqHand = new SideDependentList<Boolean>(false, false);

   private final SideDependentList<HandJointAngleCommunicator> jointAngleCommunicators = new SideDependentList<>();

   public SimulatedRobotiqHandJointAngleProducer(IHMCRealtimeROS2Publisher<HandJointAnglePacket> jointAnglePublisher, FloatingRootJointRobot simulatedRobot)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         jointAngleCommunicators.put(robotSide, new HandJointAngleCommunicator(robotSide, jointAnglePublisher));

         for (RobotiqHandJointNameMinimal jointEnum : RobotiqHandJointNameMinimal.values)
         {
            OneDegreeOfFreedomJoint fingerJoint = simulatedRobot.getOneDegreeOfFreedomJoint(jointEnum.getJointName(robotSide));

            if (fingerJoint != null)
               hasRobotiqHand.put(robotSide, true);
            handJoints.get(robotSide).put(jointEnum, fingerJoint);
         }
      }
   }

   public void sendHandJointAnglesPacket()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (hasRobotiqHand.get(robotSide))
         {
            final double[] joints = new double[RobotiqHandJointNameMinimal.values.length];

            for (RobotiqHandJointNameMinimal jointEnum : RobotiqHandJointNameMinimal.values)
            {
               joints[jointEnum.getIndex(robotSide)] = handJoints.get(robotSide).get(jointEnum).getQ();
            }

            jointAngleCommunicators.get(robotSide).updateHandAngles(new HandSensorData()
            {
               @Override
               public double[] getFingerJointAngles(RobotSide robotSide)
               {
                  return joints;
               }

               @Override
               public boolean isCalibrated()
               {
                  return true;
               }

               @Override
               public boolean isConnected()
               {
                  return true;
               }
            });
            jointAngleCommunicators.get(robotSide).write();
         }
      }
   }

   public void cleanup()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         jointAngleCommunicators.get(robotSide).cleanup();
      }
   }
}
