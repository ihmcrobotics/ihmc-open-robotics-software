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
   protected final boolean writeBeforeEstimatorTick;
   protected final PairList<OneDegreeOfFreedomJoint, LowLevelJointDataReadOnly> revoluteJoints = new PairList<OneDegreeOfFreedomJoint, LowLevelJointDataReadOnly>();

   public SimulatedLowLevelOutputWriter(FloatingRootJointRobot robot, boolean writeBeforeEstimatorTick)
   {
      this.robot = robot;
      this.writeBeforeEstimatorTick = writeBeforeEstimatorTick;

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

   }

   @Override
   public void initialize()
   {

   }
   
   protected void write()
   {
      for (int i = 0; i < revoluteJoints.size(); i++)
      {

         OneDegreeOfFreedomJoint pinJoint = revoluteJoints.first(i);
         LowLevelJointDataReadOnly data = revoluteJoints.second(i);



         if(data.hasDesiredTorque())
         {
            pinJoint.setTau(data.getDesiredTorque());
         }
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
   public void writeBefore(long timestamp)
   {
      if(writeBeforeEstimatorTick)
      {
         write();
      }
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return null;
   }

   @Override
   public void writeAfter()
   {  
      if(!writeBeforeEstimatorTick)
      {
         write();
      }
   }

}
