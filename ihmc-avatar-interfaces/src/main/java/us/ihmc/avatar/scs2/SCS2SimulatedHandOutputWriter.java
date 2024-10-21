package us.ihmc.avatar.scs2;

import us.ihmc.avatar.factory.SimulatedHandOutputWriter;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.scs2.definition.state.interfaces.OneDoFJointStateBasics;
import us.ihmc.commons.robotics.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.commons.robotics.outputData.JointDesiredOutputReadOnly;

public class SCS2SimulatedHandOutputWriter implements SimulatedHandOutputWriter
{
   private final ControllerInput controllerInput;
   private final ControllerOutput controllerOutput;

   public SCS2SimulatedHandOutputWriter(ControllerInput controllerInput, ControllerOutput controllerOutput)
   {
      this.controllerInput = controllerInput;
      this.controllerOutput = controllerOutput;
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

         OneDoFJointReadOnly jointInput = (OneDoFJointReadOnly) controllerInput.getInput().findJoint(joint.getName());
         OneDoFJointStateBasics jointOutput = controllerOutput.getOneDoFJointOutput(joint);

         switch (jointDesiredOutput.getControlMode())
         {
            case POSITION:
               if (jointDesiredOutput.hasDesiredPosition())
                  jointOutput.setConfiguration(jointDesiredOutput.getDesiredPosition());
               break;
            case EFFORT:
               double tau = 0.0;

               if (jointDesiredOutput.hasDesiredTorque())
                  tau = (jointDesiredOutput.getDesiredTorque());

               double q_d = 0.0;
               double qd_d = 0.0;
               double kp = 0.0;
               double kd = 0.0;

               if (jointDesiredOutput.hasDesiredPosition())
                  q_d = jointDesiredOutput.getDesiredPosition();
               if (jointDesiredOutput.hasDesiredVelocity())
                  qd_d = jointDesiredOutput.getDesiredVelocity();
               if (jointDesiredOutput.hasStiffness())
                  kp = jointDesiredOutput.getStiffness();
               if (jointDesiredOutput.hasDamping())
                  kd = jointDesiredOutput.getDamping();

               tau += kp * (q_d - jointInput.getQ()) + kd * (qd_d - jointInput.getQd());

               jointOutput.setEffort(tau);
               break;
            default:
               throw new UnsupportedOperationException("Unsupported control mode: " + jointDesiredOutput.getControlMode());
         }
      }
   }
}
