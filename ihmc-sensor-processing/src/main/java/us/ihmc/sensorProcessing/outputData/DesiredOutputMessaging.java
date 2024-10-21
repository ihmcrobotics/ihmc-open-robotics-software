package us.ihmc.sensorProcessing.outputData;

import controller_msgs.msg.dds.JointDesiredOutputMessage;
import controller_msgs.msg.dds.RobotDesiredConfigurationData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.robotics.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.commons.robotics.outputData.JointDesiredOutputReadOnly;

public class DesiredOutputMessaging
{
   public static void copyToMessage(JointDesiredOutputListReadOnly outputListToCopy, RobotDesiredConfigurationData desiredConfigurationDataToPack)
   {
      RecyclingArrayList<JointDesiredOutputMessage> jointDesiredOutputList = desiredConfigurationDataToPack.getJointDesiredOutputList();
      jointDesiredOutputList.clear();

      for (int i = 0; i < outputListToCopy.getNumberOfJointsWithDesiredOutput(); i++)
      {
         JointDesiredOutputMessage jointDesiredOutputMessage = jointDesiredOutputList.add();

         String jointName = outputListToCopy.getOneDoFJoint(i).getName();
         jointDesiredOutputMessage.setJointName(jointName);
         copyToMessage(outputListToCopy.getJointDesiredOutput(i), jointDesiredOutputMessage);
      }
   }

   /**
    * Copies the contents of this object to {@link JointDesiredOutputMessage}
    */
   public static void copyToMessage(JointDesiredOutputReadOnly outputToCopy, JointDesiredOutputMessage jointDesiredOutputMessageToPack)
   {
      jointDesiredOutputMessageToPack.setControlMode(outputToCopy.hasControlMode() ? outputToCopy.getControlMode().toByte() : (byte) 255);

      jointDesiredOutputMessageToPack.setHasDesiredTorque(outputToCopy.hasDesiredTorque());
      jointDesiredOutputMessageToPack.setHasDesiredPosition(outputToCopy.hasDesiredPosition());
      jointDesiredOutputMessageToPack.setHasDesiredVelocity(outputToCopy.hasDesiredVelocity());
      jointDesiredOutputMessageToPack.setHasDesiredAcceleration(outputToCopy.hasDesiredAcceleration());
      jointDesiredOutputMessageToPack.setHasStiffness(outputToCopy.hasStiffness());
      jointDesiredOutputMessageToPack.setHasDamping(outputToCopy.hasDamping());
      jointDesiredOutputMessageToPack.setHasMasterGain(outputToCopy.hasMasterGain());
      jointDesiredOutputMessageToPack.setHasVelocityScaling(outputToCopy.hasVelocityScaling());
      jointDesiredOutputMessageToPack.setHasPositionIntegrationBreakFrequency(outputToCopy.hasPositionIntegrationBreakFrequency());
      jointDesiredOutputMessageToPack.setHasVelocityIntegrationBreakFrequency(outputToCopy.hasVelocityIntegrationBreakFrequency());
      jointDesiredOutputMessageToPack.setHasPositionIntegrationMaxError(outputToCopy.hasPositionIntegrationMaxError());
      jointDesiredOutputMessageToPack.setHasVelocityFeedbackMaxError(outputToCopy.hasVelocityFeedbackMaxError());

      jointDesiredOutputMessageToPack.setDesiredTorque(outputToCopy.hasDesiredTorque() ? outputToCopy.getDesiredTorque() : 0.0);
      jointDesiredOutputMessageToPack.setDesiredPosition(outputToCopy.hasDesiredPosition() ? outputToCopy.getDesiredPosition() : 0.0);
      jointDesiredOutputMessageToPack.setDesiredVelocity(outputToCopy.hasDesiredVelocity() ? outputToCopy.getDesiredVelocity() : 0.0);
      jointDesiredOutputMessageToPack.setDesiredAcceleration(outputToCopy.hasDesiredAcceleration() ? outputToCopy.getDesiredAcceleration() : 0.0);
      jointDesiredOutputMessageToPack.setStiffness(outputToCopy.hasStiffness() ? outputToCopy.getStiffness() : 0.0);
      jointDesiredOutputMessageToPack.setDamping(outputToCopy.hasDamping() ? outputToCopy.getDamping() : 0.0);
      jointDesiredOutputMessageToPack.setMasterGain(outputToCopy.hasMasterGain() ? outputToCopy.getMasterGain() : 0.0);
      jointDesiredOutputMessageToPack.setVelocityScaling(outputToCopy.hasVelocityScaling() ? outputToCopy.getVelocityScaling() : 0.0);
      jointDesiredOutputMessageToPack.setPositionIntegrationBreakFrequency(outputToCopy.hasPositionIntegrationBreakFrequency() ?
                                                                                 outputToCopy.getPositionIntegrationBreakFrequency() :
                                                                                 0.0);
      jointDesiredOutputMessageToPack.setVelocityIntegrationBreakFrequency(outputToCopy.hasVelocityIntegrationBreakFrequency() ?
                                                                                 outputToCopy.getVelocityIntegrationBreakFrequency() :
                                                                                 0.0);
      jointDesiredOutputMessageToPack.setPositionIntegrationMaxError(outputToCopy.hasPositionIntegrationMaxError() ?
                                                                           outputToCopy.getPositionIntegrationMaxError() :
                                                                           0.0);
      jointDesiredOutputMessageToPack.setVelocityFeedbackMaxError(outputToCopy.hasVelocityFeedbackMaxError() ?
                                                                        outputToCopy.getVelocityFeedbackMaxError() :
                                                                        0.0);
   }

   public static void copyToMessage(RootJointDesiredConfigurationDataReadOnly outputToCopy, RobotDesiredConfigurationData desiredConfigurationDataToPack)
   {
      desiredConfigurationDataToPack.setHasDesiredRootJointPositionData(outputToCopy.hasDesiredConfiguration());
      if (outputToCopy.hasDesiredConfiguration())
      {
         desiredConfigurationDataToPack.getDesiredRootJointOrientation().set(0, outputToCopy.getDesiredConfiguration());
         desiredConfigurationDataToPack.getDesiredRootJointTranslation().set(4, outputToCopy.getDesiredConfiguration());
      }
      else
      {
         desiredConfigurationDataToPack.getDesiredRootJointTranslation().setToZero();
         desiredConfigurationDataToPack.getDesiredRootJointOrientation().setToZero();
      }

      desiredConfigurationDataToPack.setHasDesiredRootJointVelocityData(outputToCopy.hasDesiredVelocity());
      if (outputToCopy.hasDesiredVelocity())
      {
         desiredConfigurationDataToPack.getDesiredRootJointAngularVelocity().set(0, outputToCopy.getDesiredVelocity());
         desiredConfigurationDataToPack.getDesiredRootJointLinearVelocity().set(3, outputToCopy.getDesiredVelocity());
      }
      else
      {
         desiredConfigurationDataToPack.getDesiredRootJointLinearVelocity().setToZero();
         desiredConfigurationDataToPack.getDesiredRootJointAngularVelocity().setToZero();
      }

      desiredConfigurationDataToPack.setHasDesiredRootJointAccelerationData(outputToCopy.hasDesiredAcceleration());
      if (outputToCopy.hasDesiredAcceleration())
      {
         desiredConfigurationDataToPack.getDesiredRootJointAngularAcceleration().set(0, outputToCopy.getDesiredAcceleration());
         desiredConfigurationDataToPack.getDesiredRootJointLinearAcceleration().set(3, outputToCopy.getDesiredAcceleration());
      }
      else
      {
         desiredConfigurationDataToPack.getDesiredRootJointLinearAcceleration().setToZero();
         desiredConfigurationDataToPack.getDesiredRootJointAngularAcceleration().setToZero();
      }
   }
}
