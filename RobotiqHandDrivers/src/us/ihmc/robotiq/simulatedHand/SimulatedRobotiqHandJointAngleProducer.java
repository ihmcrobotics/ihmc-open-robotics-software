package us.ihmc.robotiq.simulatedHand;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.communication.packets.manipulation.HandJointAnglePacket;
import us.ihmc.robotiq.model.RobotiqHandModel.RobotiqHandJointNameMinimal;
import us.ihmc.utilities.io.streamingData.GlobalDataProducer;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;

import com.yobotics.simulationconstructionset.OneDegreeOfFreedomJoint;

public class SimulatedRobotiqHandJointAngleProducer
{
   private final GlobalDataProducer dataProducer;

   private final SideDependentList<List<OneDegreeOfFreedomJoint>> indexJoints = new SideDependentList<>();
   private final SideDependentList<List<OneDegreeOfFreedomJoint>> middleJoints = new SideDependentList<>();
   private final SideDependentList<List<OneDegreeOfFreedomJoint>> thumbJoints = new SideDependentList<>();

   private final SideDependentList<Boolean> hasRobotiqHand = new SideDependentList<Boolean>(false, false);

   public SimulatedRobotiqHandJointAngleProducer(GlobalDataProducer dataProducer, SDFRobot simulatedRobot)
   {
      this.dataProducer = dataProducer;

      for (RobotSide robotSide : RobotSide.values)
      {
         indexJoints.put(robotSide, new ArrayList<OneDegreeOfFreedomJoint>());
         middleJoints.put(robotSide, new ArrayList<OneDegreeOfFreedomJoint>());
         thumbJoints.put(robotSide, new ArrayList<OneDegreeOfFreedomJoint>());

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
            double[] indexJoints = new double[this.indexJoints.get(robotSide).size()];
            for (int i = 0; i < this.indexJoints.get(robotSide).size(); i++)
               indexJoints[i] = this.indexJoints.get(robotSide).get(i).getQ().getDoubleValue();

            double[] middleJoints = new double[this.middleJoints.get(robotSide).size()];
            for (int i = 0; i < this.middleJoints.get(robotSide).size(); i++)
               middleJoints[i] = this.middleJoints.get(robotSide).get(i).getQ().getDoubleValue();

            double[] thumbJoints = new double[this.thumbJoints.get(robotSide).size()];
            for (int i = 0; i < this.thumbJoints.get(robotSide).size(); i++)
               thumbJoints[i] = this.thumbJoints.get(robotSide).get(i).getQ().getDoubleValue();

            HandJointAnglePacket handJointAnglePacket = new HandJointAnglePacket(robotSide, true, indexJoints, middleJoints, thumbJoints);
            dataProducer.queueDataToSend(handJointAnglePacket);
         }
      }
   }
}
