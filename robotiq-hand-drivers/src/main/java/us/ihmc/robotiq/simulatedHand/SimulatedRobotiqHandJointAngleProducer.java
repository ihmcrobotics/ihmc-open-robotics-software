package us.ihmc.robotiq.simulatedHand;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.avatar.handControl.packetsAndConsumers.HandJointAngleCommunicator;
import us.ihmc.avatar.handControl.packetsAndConsumers.HandSensorData;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotiq.model.RobotiqHandModel.RobotiqHandJointNameMinimal;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;

public class SimulatedRobotiqHandJointAngleProducer
{
   private final SideDependentList<List<OneDegreeOfFreedomJoint>> indexJoints = new SideDependentList<>();
   private final SideDependentList<List<OneDegreeOfFreedomJoint>> middleJoints = new SideDependentList<>();
   private final SideDependentList<List<OneDegreeOfFreedomJoint>> thumbJoints = new SideDependentList<>();

   private final SideDependentList<Boolean> hasRobotiqHand = new SideDependentList<Boolean>(false, false);
   
   private final SideDependentList<HandJointAngleCommunicator> jointAngleCommunicators = new SideDependentList<>();

   public SimulatedRobotiqHandJointAngleProducer(HumanoidGlobalDataProducer dataProducer, FloatingRootJointRobot simulatedRobot, CloseableAndDisposableRegistry closeableAndDisposableRegistry)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         indexJoints.put(robotSide, new ArrayList<OneDegreeOfFreedomJoint>());
         middleJoints.put(robotSide, new ArrayList<OneDegreeOfFreedomJoint>());
         thumbJoints.put(robotSide, new ArrayList<OneDegreeOfFreedomJoint>());
         
         jointAngleCommunicators.put(robotSide, new HandJointAngleCommunicator(robotSide, dataProducer, closeableAndDisposableRegistry));

         for (RobotiqHandJointNameMinimal jointEnum : RobotiqHandJointNameMinimal.values)
         {
            OneDegreeOfFreedomJoint fingerJoint = simulatedRobot.getOneDegreeOfFreedomJoint(jointEnum.getJointName(robotSide));

            if (fingerJoint != null)
               hasRobotiqHand.put(robotSide, true);

            switch (jointEnum.getFinger(robotSide))
            {
               case INDEX:
                  indexJoints.get(robotSide).add(fingerJoint);
                  break;

               case MIDDLE:
                  middleJoints.get(robotSide).add(fingerJoint);
                  break;

               case THUMB:
                  thumbJoints.get(robotSide).add(fingerJoint);
                  break;

               default:
                  break;
            }
         }
      }
   }

   public void sendHandJointAnglesPacket()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (hasRobotiqHand.get(robotSide))
         {
            final double[] indexJoints = new double[this.indexJoints.get(robotSide).size()];
            for (int i = 0; i < this.indexJoints.get(robotSide).size(); i++)
               indexJoints[i] = this.indexJoints.get(robotSide).get(i).getQYoVariable().getDoubleValue();

            final double[] middleJoints = new double[this.middleJoints.get(robotSide).size()];
            for (int i = 0; i < this.middleJoints.get(robotSide).size(); i++)
               middleJoints[i] = this.middleJoints.get(robotSide).get(i).getQYoVariable().getDoubleValue();

            final double[] thumbJoints = new double[this.thumbJoints.get(robotSide).size()];
            for (int i = 0; i < this.thumbJoints.get(robotSide).size(); i++)
               thumbJoints[i] = this.thumbJoints.get(robotSide).get(i).getQYoVariable().getDoubleValue();

            jointAngleCommunicators.get(robotSide).updateHandAngles(new HandSensorData()
            {
               @Override
               public double[][] getFingerJointAngles(RobotSide robotSide)
               {
                  return new double[][]{indexJoints, middleJoints, thumbJoints};
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
}
