package us.ihmc.valkyrie.fingers;

import java.util.EnumMap;

import controller_msgs.msg.dds.HandJointAnglePacket;
import us.ihmc.avatar.handControl.packetsAndConsumers.HandJointAngleCommunicator;
import us.ihmc.avatar.handControl.packetsAndConsumers.HandSensorData;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;

public class SimulatedValkyrieFingerJointAngleProducer
{
   private final SideDependentList<EnumMap<ValkyrieHandJointName, OneDegreeOfFreedomJoint>> handJoints = SideDependentList.createListOfEnumMaps(ValkyrieHandJointName.class);

   private final SideDependentList<HandJointAngleCommunicator> jointAngleCommunicators = new SideDependentList<>();

   public SimulatedValkyrieFingerJointAngleProducer(IHMCRealtimeROS2Publisher<HandJointAnglePacket> jointAnglePublisher, FloatingRootJointRobot simulatedRobot)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         jointAngleCommunicators.put(robotSide, new HandJointAngleCommunicator(robotSide, jointAnglePublisher));

         for (ValkyrieHandJointName jointEnum : ValkyrieHandJointName.values)
         {
            OneDegreeOfFreedomJoint fingerJoint = simulatedRobot.getOneDegreeOfFreedomJoint(jointEnum.getJointName(robotSide));

            handJoints.get(robotSide).put(jointEnum, fingerJoint);
         }
      }
   }

   public void sendHandJointAnglesPacket()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         final double[] joints = new double[ValkyrieHandJointName.values.length];

         for (ValkyrieHandJointName jointEnum : ValkyrieHandJointName.values)
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

   public void cleanup()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         jointAngleCommunicators.get(robotSide).cleanup();
      }
   }
}