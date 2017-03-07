package us.ihmc.commonWalkingControlModules.controlModules.head;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointspaceAccelerationCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.NeckDesiredAccelerationsCommand;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class HeadUserControlModeState extends HeadControlState
{
   public static final double TIME_WITH_NO_MESSAGE_BEFORE_ABORT = 0.25;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleYoVariable weight = new DoubleYoVariable("headUserControlModeWeight", registry);
   private final JointspaceAccelerationCommand jointspaceAccelerationCommand = new JointspaceAccelerationCommand();

   private final OneDoFJoint[] userControlledJoints;
   private final DoubleYoVariable[] userDesiredJointAccelerations;

   private final DoubleYoVariable timeOfLastUserMesage;
   private final DoubleYoVariable timeSinceLastUserMesage;
   private final BooleanYoVariable abortUserControlMode;
   private final DoubleYoVariable yoTime;

   public HeadUserControlModeState(OneDoFJoint[] userControlledJoints, DoubleYoVariable yoTime, YoVariableRegistry parentRegistry)
   {
      super(HeadControlMode.USER_CONTROL_MODE);

      this.userControlledJoints = userControlledJoints;
      this.yoTime = yoTime;

      userDesiredJointAccelerations = new DoubleYoVariable[userControlledJoints.length];
      for (int i = 0; i < userControlledJoints.length; i++)
      {
         String jointName = userControlledJoints[i].getName();
         userDesiredJointAccelerations[i] = new DoubleYoVariable("qdd_d_user_" + jointName, registry);
         jointspaceAccelerationCommand.addJoint(userControlledJoints[i], Double.NaN);
      }

      timeOfLastUserMesage = new DoubleYoVariable("HeadTimeOfsLastUserMesage", registry);
      timeSinceLastUserMesage = new DoubleYoVariable("HeadTimeSinceLastUserMesage", registry);
      abortUserControlMode = new BooleanYoVariable("HeadAbortUserControlMode", registry);

      parentRegistry.addChild(registry);
   }

   @Override
   public void setWeight(double weight)
   {
      this.weight.set(weight);
   }

   public void handleNeckDesiredAccelerationsMessage(NeckDesiredAccelerationsCommand command)
   {
      if (command.getNumberOfJoints() != userControlledJoints.length)
      {
         abortUserControlMode.set(true);
         return;
      }

      for (int i = 0; i < userControlledJoints.length; i++)
         userDesiredJointAccelerations[i].set(command.getDesiredJointAcceleration(i));
      timeSinceLastUserMesage.set(0.0);
      timeOfLastUserMesage.set(yoTime.getDoubleValue());
   }

   @Override
   public void doTransitionIntoAction()
   {
      abortUserControlMode.set(false);
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

   public boolean isAbortUserControlModeRequested()
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
}
