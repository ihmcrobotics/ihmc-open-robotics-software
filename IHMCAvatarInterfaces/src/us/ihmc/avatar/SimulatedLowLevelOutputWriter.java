package us.ihmc.avatar;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.sensorProcessing.outputData.LowLevelJointDataReadOnly;
import us.ihmc.sensorProcessing.outputData.LowLevelOneDoFJointDesiredDataHolderList;
import us.ihmc.sensorProcessing.outputData.LowLevelOutputWriter;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.tools.lists.PairList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SimulatedLowLevelOutputWriter implements LowLevelOutputWriter
{

   protected final FloatingRootJointRobot robot;
   protected final PairList<OneDegreeOfFreedomJoint, LowLevelJointDataReadOnly> revoluteJoints = new PairList<OneDegreeOfFreedomJoint, LowLevelJointDataReadOnly>();

   public SimulatedLowLevelOutputWriter(FloatingRootJointRobot robot)
   {
      this.robot = robot;

   }

   @Override
   public void setLowLevelOneDoFJointDesiredDataHolderList(LowLevelOneDoFJointDesiredDataHolderList lowLevelDataHolder)
   {
      revoluteJoints.clear();

      for (int i = 0; i < lowLevelDataHolder.getNumberOfJointsWithLowLevelData(); i++)
      {
         OneDoFJoint revoluteJoint = lowLevelDataHolder.getOneDoFJoint(i);
         LowLevelJointDataReadOnly data = lowLevelDataHolder.getLowLevelJointData(i);

         String name = revoluteJoint.getName();
         OneDegreeOfFreedomJoint oneDoFJoint = robot.getOneDegreeOfFreedomJoint(name);

         ImmutablePair<OneDegreeOfFreedomJoint, LowLevelJointDataReadOnly> jointPair = new ImmutablePair<OneDegreeOfFreedomJoint, LowLevelJointDataReadOnly>(oneDoFJoint, data);
         this.revoluteJoints.add(jointPair);
      }

   }

   @Override
   public void setForceSensorDataHolder(ForceSensorDataHolderReadOnly forceSensorDataHolderForEstimator)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void initialize()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void write()
   {
      for (int i = 0; i < revoluteJoints.size(); i++)
      {

         OneDegreeOfFreedomJoint pinJoint = revoluteJoints.first(i);
         LowLevelJointDataReadOnly data = revoluteJoints.second(i);

         double tau = data.getDesiredTorque();

         pinJoint.setTau(tau);
         if (data.hasKp())
         {
            pinJoint.setKp(data.getKp());
         }
         if (data.hasKd())
         {
            pinJoint.setKd(data.getKd());
         }
         if (data.hasDesiredPosition())
         {
            pinJoint.setqDesired(data.getDesiredPosition());
         }
         if (data.hasDesiredVelocity())
         {
            pinJoint.setQdDesired(data.getDesiredVelocity());
         }

      }
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return null;
   }

}
