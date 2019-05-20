package us.ihmc.avatar;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.tools.lists.PairList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SimulatedLowLevelOutputWriter implements JointDesiredOutputWriter
{

   protected final FloatingRootJointRobot robot;
   protected final boolean writeBeforeEstimatorTick;
   protected final PairList<OneDegreeOfFreedomJoint, JointDesiredOutputReadOnly> revoluteJoints = new PairList<OneDegreeOfFreedomJoint, JointDesiredOutputReadOnly>();

   public SimulatedLowLevelOutputWriter(FloatingRootJointRobot robot, boolean writeBeforeEstimatorTick)
   {
      this.robot = robot;
      this.writeBeforeEstimatorTick = writeBeforeEstimatorTick;

   }

   @Override
   public void setJointDesiredOutputList(JointDesiredOutputListBasics lowLevelDataHolder)
   {
      revoluteJoints.clear();

      for (int i = 0; i < lowLevelDataHolder.getNumberOfJointsWithDesiredOutput(); i++)
      {
         OneDoFJointBasics revoluteJoint = lowLevelDataHolder.getOneDoFJoint(i);
         JointDesiredOutputReadOnly data = lowLevelDataHolder.getJointDesiredOutput(i);

         String name = revoluteJoint.getName();
         OneDegreeOfFreedomJoint oneDoFJoint = robot.getOneDegreeOfFreedomJoint(name);

         ImmutablePair<OneDegreeOfFreedomJoint, JointDesiredOutputReadOnly> jointPair = new ImmutablePair<OneDegreeOfFreedomJoint, JointDesiredOutputReadOnly>(oneDoFJoint, data);
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
         JointDesiredOutputReadOnly data = revoluteJoints.second(i);



         if(data.hasDesiredTorque())
         {
            pinJoint.setTau(data.getDesiredTorque());
         }
         if (data.hasStiffness())
         {
            pinJoint.setKp(data.getStiffness());
         }
         if (data.hasDamping())
         {
            pinJoint.setKd(data.getDamping());
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
