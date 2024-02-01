package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import java.util.Map;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointspaceAccelerationCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.DesiredAccelerationsCommand;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * A rigid body mode that takes a user's {@link DesiredAccelerationsCommand} and directly
 * feeds that into the whole body controller core as a {@link JointspaceAccelerationCommand}.
 */
public class RigidBodyUserControlState extends RigidBodyControlState
{
   public static final double TIME_WITH_NO_MESSAGE_BEFORE_ABORT = 0.25;

   private final JointspaceAccelerationCommand jointspaceAccelerationCommand;

   private final OneDoFJointBasics[] jointsToControl;
   private final int numberOfJoints;

   private final YoDouble[] userDesiredJointAccelerations;
   private final DoubleProvider[] weights;

   private final YoBoolean abortUserControlMode;
   private final YoBoolean hasWeights;

   public RigidBodyUserControlState(String bodyName, OneDoFJointBasics[] jointsToControl, YoDouble yoTime, YoRegistry parentRegistry)
   {
      super(RigidBodyControlMode.USER, bodyName, yoTime, parentRegistry);
      String prefix = bodyName + "UserMode";
      hasWeights = new YoBoolean(prefix + "HasWeights", registry);

      this.jointsToControl = jointsToControl;
      this.numberOfJoints = jointsToControl.length;

      jointspaceAccelerationCommand = new JointspaceAccelerationCommand();
      userDesiredJointAccelerations = new YoDouble[jointsToControl.length];
      weights = new DoubleProvider[jointsToControl.length];

      for (int i = 0; i < numberOfJoints; i++)
      {
         String jointName = jointsToControl[i].getName();
         userDesiredJointAccelerations[i] = new YoDouble(prefix + "_" + jointName + "_qdd_d", registry);
         jointspaceAccelerationCommand.addJoint(jointsToControl[i], Double.NaN);
      }

      abortUserControlMode = new YoBoolean(prefix + "Abort", registry);
   }

   public boolean handleDesiredAccelerationsCommand(DesiredAccelerationsCommand command)
   {
      if (!hasWeights.getBooleanValue())
      {
         LogTools.warn(warningPrefix + "Can not send joint desired accelerations. Do not have all weights set.");
         return false;
      }

      if (command.getNumberOfJoints() != jointsToControl.length)
      {
         LogTools.warn(warningPrefix + "Unexpected number of joints.");
         return false;
      }

      if (!handleCommandInternal(command))
         return false;

      for (int i = 0; i < numberOfJoints; i++)
         userDesiredJointAccelerations[i].set(command.getDesiredJointAcceleration(i));

      abortUserControlMode.set(false);
      return true;
   }

   @Override
   public void doAction(double timeInState)
   {
      if (getTimeInTrajectory() > TIME_WITH_NO_MESSAGE_BEFORE_ABORT)
      {
         abortUserControlMode.set(true);
         return;
      }

      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         double desiredAcceleration = userDesiredJointAccelerations[jointIdx].getDoubleValue();
         jointspaceAccelerationCommand.setOneDoFJointDesiredAcceleration(jointIdx, desiredAcceleration);
         jointspaceAccelerationCommand.setWeight(jointIdx, weights[jointIdx].getValue());
      }
   }

   public void setWeights(Map<String, DoubleProvider> weights)
   {
      hasWeights.set(true);
      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         OneDoFJointBasics joint = jointsToControl[jointIdx];
         if (weights.containsKey(joint.getName()))
            this.weights[jointIdx] = weights.get(joint.getName());
         else
            hasWeights.set(false);
      }
   }

   public void setWeight(DoubleProvider weight)
   {
      hasWeights.set(true);
      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
         this.weights[jointIdx] = weight;
   }

   @Override
   public void onEntry()
   {
   }

   @Override
   public void onExit(double timeInState)
   {
      abortUserControlMode.set(false);
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return jointspaceAccelerationCommand;
   }

   @Override
   public boolean abortState()
   {
      return abortUserControlMode.getBooleanValue();
   }

   @Override
   public boolean isEmpty()
   {
      // this control mode does not support command queuing
      return false;
   }

   @Override
   public double getLastTrajectoryPointTime()
   {
      // this control mode does not support command queuing
      return 0.0;
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      return null;
   }
}
