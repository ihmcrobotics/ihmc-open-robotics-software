package us.ihmc.avatar;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.commons.robotics.outputData.JointDesiredOutputListBasics;
import us.ihmc.commons.robotics.outputData.JointDesiredOutputReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.commons.lists.PairList;
import us.ihmc.yoVariables.registry.YoRegistry;

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
         OneDoFJointReadOnly revoluteJoint = lowLevelDataHolder.getOneDoFJoint(i);
         JointDesiredOutputReadOnly data = lowLevelDataHolder.getJointDesiredOutput(i);

         String name = revoluteJoint.getName();
         OneDegreeOfFreedomJoint oneDoFJoint = robot.getOneDegreeOfFreedomJoint(name);

         ImmutablePair<OneDegreeOfFreedomJoint, JointDesiredOutputReadOnly> jointPair = new ImmutablePair<OneDegreeOfFreedomJoint, JointDesiredOutputReadOnly>(oneDoFJoint, data);
         this.revoluteJoints.add(jointPair);
      }

   }

   protected void write()
   {
      for (int i = 0; i < revoluteJoints.size(); i++)
      {

         OneDegreeOfFreedomJoint pinJoint = revoluteJoints.first(i);

         if (pinJoint == null)
            continue; // Can happen for loop closures

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
   public YoRegistry getYoVariableRegistry()
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
