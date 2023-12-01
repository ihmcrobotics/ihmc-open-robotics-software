package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.EuclideanTrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.JointspaceTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SE3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SO3TrajectoryControllerCommand;
import us.ihmc.log.LogTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public abstract class RigidBodyTaskspaceControlState extends RigidBodyControlState
{
   public static final double timeEpsilonForInitialPoint = 0.05;
   public static final int maxPoints = 10000;
   public static final int maxPointsInGenerator = 5;

   public RigidBodyTaskspaceControlState(RigidBodyControlMode controlMode, String bodyName, YoDouble yoTime, YoRegistry parentRegistry)
   {
      super(controlMode, bodyName, yoTime, parentRegistry);
   }

   public abstract void goToPoseFromCurrent(FramePose3DReadOnly pose, double trajectoryTime);

   public abstract void goToPose(FramePose3DReadOnly pose, double trajectoryTime);

   public abstract void holdCurrent();

   public abstract void holdCurrentDesired();
   
   public abstract boolean isHybridModeActive();

   public RigidBodyOrientationControlHelper getOrientationControlHelper()
   {
      return null;
   }

   public boolean handleTrajectoryCommand(EuclideanTrajectoryControllerCommand command)
   {
      LogTools.warn("Handling of " + command.getClass().getSimpleName() + " not implemented for " + getClass().getSimpleName() + ".");
      return false;
   }

   public boolean handleTrajectoryCommand(SO3TrajectoryControllerCommand command)
   {
      LogTools.warn("Handling of " + command.getClass().getSimpleName() + " not implemented for " + getClass().getSimpleName() + ".");
      return false;
   }

   public boolean handleTrajectoryCommand(SE3TrajectoryControllerCommand command)
   {
      LogTools.warn("Handling of " + command.getClass().getSimpleName() + " not implemented for " + getClass().getSimpleName() + ".");
      return false;
   }

   public boolean handleHybridTrajectoryCommand(EuclideanTrajectoryControllerCommand command, JointspaceTrajectoryCommand jointspaceCommand,
                                                double[] initialJointPositions)
   {
      LogTools.warn("Handling of hybrid command " + command.getClass().getSimpleName() + " not implemented for " + getClass().getSimpleName() + ".");
      return false;
   }

   public boolean handleHybridTrajectoryCommand(SO3TrajectoryControllerCommand command, JointspaceTrajectoryCommand jointspaceCommand,
                                                double[] initialJointPositions)
   {
      LogTools.warn("Handling of hybrid command " + command.getClass().getSimpleName() + " not implemented for " + getClass().getSimpleName() + ".");
      return false;
   }

   public boolean handleHybridTrajectoryCommand(SE3TrajectoryControllerCommand command, JointspaceTrajectoryCommand jointspaceCommand,
                                                double[] initialJointPositions)
   {
      LogTools.warn("Handling of hybrid command " + command.getClass().getSimpleName() + " not implemented for " + getClass().getSimpleName() + ".");
      return false;
   }
}
