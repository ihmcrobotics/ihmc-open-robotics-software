package us.ihmc.avatar.factory;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.commons.robotics.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.commons.robotics.outputData.JointDesiredOutputReadOnly;
import us.ihmc.simulationconstructionset.DummyOneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJointHolder;

public class DefaultSimulatedHandOutputWriter implements SimulatedHandOutputWriter
{
   private final OneDegreeOfFreedomJointHolder robot;

   public DefaultSimulatedHandOutputWriter(OneDegreeOfFreedomJointHolder robot)
   {
      this.robot = robot;
   }

   @Override
   public void write(JointDesiredOutputListReadOnly jointDesiredOutputList)
   {
      for (int i = 0; i < jointDesiredOutputList.getNumberOfJointsWithDesiredOutput(); i++)
      {
         OneDoFJointReadOnly joint = jointDesiredOutputList.getOneDoFJoint(i);
         JointDesiredOutputReadOnly jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(i);

         if (!jointDesiredOutput.hasControlMode())
            throw new IllegalStateException("The joint " + joint.getName() + " has no control mode.");

         OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint = robot.getOneDegreeOfFreedomJoint(joint.getName());

         switch (jointDesiredOutput.getControlMode())
         {
            case POSITION:
               writeJointPosition(jointDesiredOutput, oneDegreeOfFreedomJoint);
               break;
            case EFFORT:
               writeJointEffort(jointDesiredOutput, oneDegreeOfFreedomJoint);
               writeJointDesiredPosition(jointDesiredOutput, oneDegreeOfFreedomJoint);
               writeJointDesiredVelocity(jointDesiredOutput, oneDegreeOfFreedomJoint);
               writeJointStiffness(jointDesiredOutput, oneDegreeOfFreedomJoint);
               writeJointDamping(jointDesiredOutput, oneDegreeOfFreedomJoint);
               break;
            default:
               throw new UnsupportedOperationException("Unsupported control mode: " + jointDesiredOutput.getControlMode());
         }
      }
   }

   private void writeJointPosition(JointDesiredOutputReadOnly jointDesiredOutput, OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint)
   {
      if (!jointDesiredOutput.hasDesiredPosition())
         return;

      if (oneDegreeOfFreedomJoint instanceof DummyOneDegreeOfFreedomJoint)
         oneDegreeOfFreedomJoint.getQYoVariable().set(jointDesiredOutput.getDesiredPosition());
      else
         oneDegreeOfFreedomJoint.setQ(jointDesiredOutput.getDesiredPosition());
   }

   private void writeJointEffort(JointDesiredOutputReadOnly jointDesiredOutput, OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint)
   {
      if (!jointDesiredOutput.hasDesiredTorque())
         return;

      if (oneDegreeOfFreedomJoint instanceof DummyOneDegreeOfFreedomJoint)
         oneDegreeOfFreedomJoint.getTauYoVariable().set(jointDesiredOutput.getDesiredTorque());
      else
         oneDegreeOfFreedomJoint.setTau(jointDesiredOutput.getDesiredTorque());
   }

   private void writeJointDesiredPosition(JointDesiredOutputReadOnly jointDesiredOutput, OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint)
   {
      if (!jointDesiredOutput.hasDesiredPosition())
         return;

      oneDegreeOfFreedomJoint.setqDesired(jointDesiredOutput.getDesiredPosition());
   }

   private void writeJointDesiredVelocity(JointDesiredOutputReadOnly jointDesiredOutput, OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint)
   {
      if (!jointDesiredOutput.hasDesiredVelocity())
         return;

      oneDegreeOfFreedomJoint.setQdDesired(jointDesiredOutput.getDesiredVelocity());
   }

   private void writeJointStiffness(JointDesiredOutputReadOnly jointDesiredOutput, OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint)
   {
      if (!jointDesiredOutput.hasStiffness())
         return;

      oneDegreeOfFreedomJoint.setKp(jointDesiredOutput.getStiffness());
   }

   private void writeJointDamping(JointDesiredOutputReadOnly jointDesiredOutput, OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint)
   {
      if (!jointDesiredOutput.hasDamping())
         return;

      oneDegreeOfFreedomJoint.setKd(jointDesiredOutput.getDamping());
   }
}
