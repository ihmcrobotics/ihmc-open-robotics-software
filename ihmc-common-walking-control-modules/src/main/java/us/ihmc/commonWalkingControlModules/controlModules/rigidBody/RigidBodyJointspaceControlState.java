package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import java.util.Map;

import controller_msgs.msg.dds.JointspaceTrajectoryStatusMessage;
import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commonWalkingControlModules.controlModules.JointspaceTrajectoryStatusMessageHelper;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.JointspaceTrajectoryCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPIDGains;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class RigidBodyJointspaceControlState extends RigidBodyControlState
{
   public static final int maxPoints = 10000;
   public static final int maxPointsInGenerator = 5;

   private final RigidBodyJointControlHelper jointControlHelper;

   private final JointspaceTrajectoryStatusMessageHelper statusHelper;

   private final int numberOfJoints;
   private final double[] jointsHomeConfiguration;

   public RigidBodyJointspaceControlState(String bodyName, OneDoFJointBasics[] jointsToControl, TObjectDoubleHashMap<String> homeConfiguration,
         YoDouble yoTime, RigidBodyJointControlHelper jointControlHelper, YoVariableRegistry parentRegistry)
   {
      super(RigidBodyControlMode.JOINTSPACE, bodyName, yoTime, parentRegistry);
      this.jointControlHelper = jointControlHelper;

      numberOfJoints = jointsToControl.length;
      jointsHomeConfiguration = new double[numberOfJoints];

      statusHelper = new JointspaceTrajectoryStatusMessageHelper(jointsToControl);

      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         OneDoFJointBasics joint = jointsToControl[jointIdx];
         String jointName = joint.getName();
         if (!homeConfiguration.contains(jointName))
            throw new RuntimeException(warningPrefix + "Can not create control manager since joint home configuration is not defined.");
         jointsHomeConfiguration[jointIdx] = homeConfiguration.get(jointName);
      }
   }

   public void setDefaultWeights(Map<String, DoubleProvider> weights)
   {
      jointControlHelper.setDefaultWeights(weights);
   }

   public void setDefaultWeight(DoubleProvider weight)
   {
      jointControlHelper.setDefaultWeight(weight);
   }

   public void setGains(Map<String, PIDGainsReadOnly> gains)
   {
      jointControlHelper.setGains(gains);
   }

   public void setGains(YoPIDGains gains)
   {
      jointControlHelper.setGains(gains);
   }

   public void holdCurrent()
   {
      jointControlHelper.overrideTrajectory();
      jointControlHelper.setWeightsToDefaults();
      resetLastCommandId();
      setTrajectoryStartTimeToCurrentTime();

      jointControlHelper.queueInitialPointsAtCurrent();

      jointControlHelper.startTrajectoryExecution();
      trajectoryDone.set(false);
   }

   public void holdCurrentDesired()
   {
      jointControlHelper.overrideTrajectory();
      jointControlHelper.setWeightsToDefaults();
      resetLastCommandId();
      setTrajectoryStartTimeToCurrentTime();

      jointControlHelper.queueInitialPointsAtCurrentDesired();

      jointControlHelper.startTrajectoryExecution();
      trajectoryDone.set(false);
   }

   public void goHome(double trajectoryTime, double[] initialJointPositions)
   {
      jointControlHelper.overrideTrajectory();
      jointControlHelper.setWeightsToDefaults();
      resetLastCommandId();
      setTrajectoryStartTimeToCurrentTime();

      jointControlHelper.queuePointsAtTimeWithZeroVelocity(0.0, initialJointPositions);
      jointControlHelper.queuePointsAtTimeWithZeroVelocity(trajectoryTime, jointsHomeConfiguration);

      jointControlHelper.startTrajectoryExecution();
      trajectoryDone.set(false);
   }

   public void goHomeFromCurrent(double trajectoryTime)
   {
      jointControlHelper.overrideTrajectory();
      jointControlHelper.setWeightsToDefaults();
      resetLastCommandId();
      setTrajectoryStartTimeToCurrentTime();

      jointControlHelper.queueInitialPointsAtCurrent();
      jointControlHelper.queuePointsAtTimeWithZeroVelocity(trajectoryTime, jointsHomeConfiguration);

      jointControlHelper.startTrajectoryExecution();
      trajectoryDone.set(false);
   }

   @Override
   public void doAction(double timeInState)
   {
      double timeInTrajectory = getTimeInTrajectory();

      statusHelper.updateWithTimeInTrajectory(timeInTrajectory);

      trajectoryDone.set(jointControlHelper.doAction(timeInTrajectory));
   }

   public boolean handleTrajectoryCommand(JointspaceTrajectoryCommand command, double[] initialJointPositions)
   {
      if (!handleCommandInternal(command))
      {
         return false;
      }
      else if (jointControlHelper.handleTrajectoryCommand(command, initialJointPositions))
      {
         statusHelper.registerNewTrajectory(command);
         return true;
      }
      else
      {
         return false;
      }
   }

   @Override
   public double getLastTrajectoryPointTime()
   {
      return jointControlHelper.getLastTrajectoryPointTime();
   }

   @Override
   public boolean isEmpty()
   {
      return jointControlHelper.isEmpty();
   }

   public double getJointDesiredPosition(int jointIdx)
   {
      return jointControlHelper.getJointDesiredPosition(jointIdx);
   }

   public double getJointDesiredVelocity(int jointIdx)
   {
      return jointControlHelper.getJointDesiredVelocity(jointIdx);
   }

   @Override
   public void onEntry()
   {
   }

   @Override
   public void onExit()
   {
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return jointControlHelper.getJointspaceCommand();
   }

   @Override
   public JointspaceTrajectoryStatusMessage pollStatusToReport()
   {
      return statusHelper.pollStatusMessage(jointControlHelper.getJointspaceCommand());
   }
}
