package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointspaceAccelerationCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.DesiredAccelerationCommand;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class RigidBodyUserControlState extends RigidBodyControlState
{
   public static final double TIME_WITH_NO_MESSAGE_BEFORE_ABORT = 0.25;

   private final JointspaceAccelerationCommand jointspaceAccelerationCommand = new JointspaceAccelerationCommand();

   private final OneDoFJoint[] jointsToControl;
   private final DoubleYoVariable[] userDesiredJointAccelerations;
   //   private final DoubleYoVariable[] weights;
   private final DoubleYoVariable weight;

   private final DoubleYoVariable timeOfLastUserMesage;
   private final DoubleYoVariable timeSinceLastUserMesage;
   private final BooleanYoVariable abortUserControlMode;
   private final DoubleYoVariable yoTime;

   public RigidBodyUserControlState(String bodyName, OneDoFJoint[] jointsToControl, DoubleYoVariable yoTime, YoVariableRegistry parentRegistry)
   {
      super(RigidBodyControlMode.USER, bodyName, yoTime);

      this.jointsToControl = jointsToControl;
      this.yoTime = yoTime;

      userDesiredJointAccelerations = new DoubleYoVariable[jointsToControl.length];
      //      weights = new DoubleYoVariable[jointsToControl.length];
      weight = new DoubleYoVariable(bodyName + "UserControlModeWeight", registry);

      for (int i = 0; i < jointsToControl.length; i++)
      {
         String jointName = jointsToControl[i].getName();
         userDesiredJointAccelerations[i] = new DoubleYoVariable(bodyName + "_qdd_d_user_" + jointName, registry);
         //         weights[i] = new DoubleYoVariable(bodyName + "_qdd_user_weight_" + jointName, registry);
         jointspaceAccelerationCommand.addJoint(jointsToControl[i], Double.NaN);
      }

      timeOfLastUserMesage = new DoubleYoVariable(bodyName + "TimeOfLastAccelerationMesage", registry);
      timeSinceLastUserMesage = new DoubleYoVariable(bodyName + "TimeSinceLastAccelerationMesage", registry);
      abortUserControlMode = new BooleanYoVariable(bodyName + "AbortUserControlMode", registry);
      parentRegistry.addChild(registry);
   }

   public boolean handleDesiredAccelerationsCommand(DesiredAccelerationCommand<?, ?> command)
   {
      if (command.getNumberOfJoints() != jointsToControl.length)
      {
         abortUserControlMode.set(true);
         return false;
      }

      for (int i = 0; i < jointsToControl.length; i++)
      {
         userDesiredJointAccelerations[i].set(command.getDesiredJointAcceleration(i));
         //         weights[i].set(command.getWeight(i));
      }
      timeSinceLastUserMesage.set(0.0);
      timeOfLastUserMesage.set(yoTime.getDoubleValue());
      return true;
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

      for (int i = 0; i < jointsToControl.length; i++)
      {
         jointspaceAccelerationCommand.setOneDoFJointDesiredAcceleration(i, userDesiredJointAccelerations[i].getDoubleValue());
         //         jointspaceAccelerationCommand.setWeight(i, weights[i].getDoubleValue());
         jointspaceAccelerationCommand.setWeight(weight.getDoubleValue());
      }
   }

   public void setWeight(double weight)
   {
      this.weight.set(weight);
   }

   public boolean isAbortUserControlModeRequested()
   {
      return abortUserControlMode.getBooleanValue();
   }

   @Override
   public void doTransitionIntoAction()
   {
      abortUserControlMode.set(false);
   }

   @Override
   public void doTransitionOutOfAction()
   {
      abortUserControlMode.set(false);
   }

   @Override
   public boolean isEmpty()
   {
      return false;
   }

   @Override
   public double getLastTrajectoryPointTime()
   {
      return 0;
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
