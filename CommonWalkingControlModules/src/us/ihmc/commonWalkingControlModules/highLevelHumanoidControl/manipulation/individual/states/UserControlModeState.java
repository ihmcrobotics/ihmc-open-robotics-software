package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmDesiredAccelerationsMessage;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.stateMachines.State;

public class UserControlModeState extends State<HandControlState>
{
   public static final double TIME_WITH_NO_MESSAGE_BEFORE_ABORT = 0.25;

   private final RobotSide robotSide;
   private final OneDoFJoint[] userControlledJoints;
   private final DoubleYoVariable[] userDesiredJointAccelerations;
   private final DoubleYoVariable timeOfLastUserMesage;
   private final DoubleYoVariable timeSinceLastUserMesage;
   private final BooleanYoVariable abortUserControlMode;
   private final DoubleYoVariable yoTime;
   private final MomentumBasedController momentumBasedController;

   public UserControlModeState(String namePrefix, RobotSide robotSide, OneDoFJoint[] userControlledJoints, MomentumBasedController momentumBasedController,
         YoVariableRegistry parentRegistry)
   {
      super(HandControlState.USER_CONTROL_MODE);

      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);

      this.robotSide = robotSide;
      this.userControlledJoints = userControlledJoints;
      userDesiredJointAccelerations = new DoubleYoVariable[userControlledJoints.length];
      for (int i = 0; i < userControlledJoints.length; i++)
      {
         String jointName = userControlledJoints[i].getName();
         userDesiredJointAccelerations[i] = new DoubleYoVariable("qdd_d_user_" + jointName, registry);
      }

      timeOfLastUserMesage = new DoubleYoVariable(namePrefix + "TimeOfsLastUserMesage", registry);
      timeSinceLastUserMesage = new DoubleYoVariable(namePrefix + "TimeSinceLastUserMesage", registry);
      abortUserControlMode = new BooleanYoVariable(namePrefix + "AbortUserControlMode", registry);
      yoTime = momentumBasedController.getYoTime();
      this.momentumBasedController = momentumBasedController;
   }

   public void handleArmDesiredAccelerationsMessage(ArmDesiredAccelerationsMessage message)
   {
      if (message.getNumberOfJoints() != userControlledJoints.length)
      {
         abortUserControlMode.set(true);
         return;
      }

      if (message.getRobotSide() != robotSide)
      {
         abortUserControlMode.set(true);
         return;
      }

      for (int i = 0; i < userControlledJoints.length; i++)
         userDesiredJointAccelerations[i].set(message.getArmDesiredJointAcceleration(i));
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
         OneDoFJoint joint = userControlledJoints[i];
         momentumBasedController.setOneDoFJointAcceleration(joint, userDesiredJointAccelerations[i].getDoubleValue());
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
}
