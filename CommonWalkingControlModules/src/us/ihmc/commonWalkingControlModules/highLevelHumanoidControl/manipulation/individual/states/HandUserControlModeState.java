package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import us.ihmc.commonWalkingControlModules.controllerCore.command.SolverWeightLevels;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointspaceAccelerationCommand;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlMode;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ArmDesiredAccelerationsCommand;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class HandUserControlModeState extends HandControlState
{
   public static final double TIME_WITH_NO_MESSAGE_BEFORE_ABORT = 0.25;

   private final OneDoFJoint[] userControlledJoints;
   private final DoubleYoVariable[] userDesiredJointAccelerations;
   private final DoubleYoVariable timeOfLastUserMesage;
   private final DoubleYoVariable timeSinceLastUserMesage;
   private final BooleanYoVariable abortUserControlMode;
   private final DoubleYoVariable yoTime;
   private final JointspaceAccelerationCommand jointspaceAccelerationCommand = new JointspaceAccelerationCommand();

   public HandUserControlModeState(String namePrefix, OneDoFJoint[] userControlledJoints, HighLevelHumanoidControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry)
   {
      super(HandControlMode.USER_CONTROL_MODE);

      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);

      this.userControlledJoints = userControlledJoints;
      userDesiredJointAccelerations = new DoubleYoVariable[userControlledJoints.length];
      for (int i = 0; i < userControlledJoints.length; i++)
      {
         String jointName = userControlledJoints[i].getName();
         userDesiredJointAccelerations[i] = new DoubleYoVariable("qdd_d_user_" + jointName, registry);
         jointspaceAccelerationCommand.addJoint(userControlledJoints[i], Double.NaN);
      }

      jointspaceAccelerationCommand.setWeight(SolverWeightLevels.HIGH);

      timeOfLastUserMesage = new DoubleYoVariable(namePrefix + "TimeOfLastUserMesage", registry);
      timeSinceLastUserMesage = new DoubleYoVariable(namePrefix + "TimeSinceLastUserMesage", registry);
      abortUserControlMode = new BooleanYoVariable(namePrefix + "AbortUserControlMode", registry);
      yoTime = controllerToolbox.getYoTime();
   }

   public void setWeight(double weight)
   {
      jointspaceAccelerationCommand.setWeight(weight);
   }

   public boolean handleArmDesiredAccelerationsMessage(ArmDesiredAccelerationsCommand command)
   {
      for (int i = 0; i < userControlledJoints.length; i++)
         userDesiredJointAccelerations[i].set(command.getDesiredJointAcceleration(i));
      timeSinceLastUserMesage.set(0.0);
      timeOfLastUserMesage.set(yoTime.getDoubleValue());

      return true;
   }

   @Override
   public void doTransitionIntoAction()
   {
   }

   @Override
   public void doAction()
   {
      timeSinceLastUserMesage.set(yoTime.getDoubleValue() - timeOfLastUserMesage.getDoubleValue());

      if (timeSinceLastUserMesage.getDoubleValue() > TIME_WITH_NO_MESSAGE_BEFORE_ABORT)
      {
         abortUserControlMode.set(true);
         return;
      }

      for (int i = 0; i < userControlledJoints.length; i++)
      {
         jointspaceAccelerationCommand.setOneDoFJointDesiredAcceleration(i, userDesiredJointAccelerations[i].getDoubleValue());
      }
   }

   @Override
   public boolean isAbortRequested()
   {
      return abortUserControlMode.getBooleanValue();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      abortUserControlMode.set(false);
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return jointspaceAccelerationCommand;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return null;
   }

   @Override
   public boolean isDone()
   {
      return true;
   }
}
