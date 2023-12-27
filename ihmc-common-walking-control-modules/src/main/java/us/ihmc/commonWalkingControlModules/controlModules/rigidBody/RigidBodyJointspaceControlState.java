package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import java.util.Map;

import controller_msgs.msg.dds.JointspaceTrajectoryStatusMessage;
import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commonWalkingControlModules.controlModules.JointspaceTrajectoryStatusMessageHelper;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsOptimizationSettingsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParameters;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ArmTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.JointspaceTrajectoryCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPIDGains;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Manages control of a rigid body by controlling a number of joint ancestors
 * in joint space through jointspace feedback control commands.
 * <p>
 * When robot hardware requires direct position control instead of
 * torque control, this class supports disabling acceleration integration on those
 * joints and instead provides joint desired output data for those joints to
 * the controller core, which will bypass the optimizer. The user
 * specifies this via {@link ArmTrajectoryCommand.RequestedMode}.
 * </p>
 * <p>
 * This class is also responsible for commanding the "home" configuration
 * to the whole body controller when requested by the user.
 * </p>
 * <p>
 * Finally, this class reports status messages with the current and desired
 * joint positions.
 * </p>
 */
public class RigidBodyJointspaceControlState extends RigidBodyControlState
{
   public static final int maxPoints = 10000;
   public static final int maxPointsInGenerator = 5;

   private final RigidBodyJointControlHelper jointControlHelper;

   private final JointspaceTrajectoryStatusMessageHelper statusHelper;
   private final OneDoFJointBasics[] jointsToControl;

   private final int numberOfJoints;
   private final double[] jointsHomeConfiguration;
   private final JointDesiredOutputList jointDesiredOutputList;

   private final BooleanParameter[] defaultDirectPositionControlMode;
   private final YoBoolean[] directPositionControlMode;
   private final JointAccelerationIntegrationCommand accelerationIntegrationCommand = new JointAccelerationIntegrationCommand();
   private final InverseDynamicsOptimizationSettingsCommand activateJointsCommand = new InverseDynamicsOptimizationSettingsCommand();
   private final InverseDynamicsCommandList inverseDynamicsCommandList = new InverseDynamicsCommandList();

   public RigidBodyJointspaceControlState(String bodyName,
                                          OneDoFJointBasics[] jointsToControl,
                                          TObjectDoubleHashMap<String> homeConfiguration,
                                          YoDouble yoTime,
                                          RigidBodyJointControlHelper jointControlHelper,
                                          YoRegistry parentRegistry)
   {
      super(RigidBodyControlMode.JOINTSPACE, bodyName, yoTime, parentRegistry);
      this.jointControlHelper = jointControlHelper;
      this.jointsToControl = jointsToControl;

      defaultDirectPositionControlMode = new BooleanParameter[jointsToControl.length];
      directPositionControlMode = new YoBoolean[jointsToControl.length];

      for (int i = 0; i < jointsToControl.length; i++)
      {
         defaultDirectPositionControlMode[i] = new BooleanParameter(jointsToControl[i].getName() + "DefaultDirectPositionControlMode", parentRegistry, false);
         directPositionControlMode[i] = new YoBoolean(jointsToControl[i].getName() + "DirectPositionControlMode", parentRegistry);
      }

      jointDesiredOutputList = new JointDesiredOutputList(jointsToControl);

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

   @Override
   public JointDesiredOutputListReadOnly getJointDesiredData()
   {
      boolean containsPositionControlledJoint = false;

      for (int jointIdx = 0; jointIdx < jointDesiredOutputList.getNumberOfJointsWithDesiredOutput(); jointIdx++)
      {
         if (directPositionControlMode[jointIdx].getValue())
         {
            containsPositionControlledJoint = true;

            JointDesiredOutput lowLevelJointData = jointDesiredOutputList.getJointDesiredOutput(jointIdx);
            PIDGainsReadOnly lowLevelJointGain = jointControlHelper.getLowLevelJointGain(jointIdx);
            OneDoFJointReadOnly joint = jointDesiredOutputList.getOneDoFJoint(jointIdx);

            // Clamp desired position based on maximum feedback
            double desiredPosition = getJointDesiredPosition(jointIdx);
            double maximumFeedback = lowLevelJointGain.getMaximumFeedback();
            if (!Double.isNaN(maximumFeedback) && Double.isFinite(maximumFeedback) && lowLevelJointGain.getKp() > 1.0e-3)
            {
               double maxPositionError = maximumFeedback / lowLevelJointGain.getKp();
               desiredPosition = EuclidCoreTools.clamp(desiredPosition, joint.getQ() - maxPositionError, joint.getQ() + maxPositionError);
            }

            // Clamp desired velocity based on maximum feedback rate
            double desiredVelocity = getJointDesiredVelocity(jointIdx);
            double maximumFeedbackRate = lowLevelJointGain.getMaximumFeedbackRate();
            if (!Double.isNaN(maximumFeedbackRate) && Double.isFinite(maximumFeedbackRate) && lowLevelJointGain.getKd() > 1.0e-3)
            {
               double maxVelocityError = maximumFeedbackRate / lowLevelJointGain.getKd();
               desiredVelocity = EuclidCoreTools.clamp(desiredVelocity, joint.getQd() - maxVelocityError, joint.getQd() + maxVelocityError);
            }

            lowLevelJointData.setControlMode(JointDesiredControlMode.POSITION);
            lowLevelJointData.setDesiredPosition(desiredPosition);
            lowLevelJointData.setDesiredVelocity(desiredVelocity);
            lowLevelJointData.setStiffness(lowLevelJointGain.getKp());
            lowLevelJointData.setDamping(lowLevelJointGain.getKd());
         }
      }

      if (containsPositionControlledJoint)
         return jointDesiredOutputList;
      else
         return null;
   }

   public void setGains(Map<String, PIDGainsReadOnly> jointspaceHighLevelGains, Map<String, PIDGainsReadOnly> jointspaceLowLevelGains)
   {
      jointControlHelper.setGains(jointspaceHighLevelGains, jointspaceLowLevelGains);
   }

   public void setGains(YoPIDGains highLevelGains)
   {
      jointControlHelper.setHighLevelGains(highLevelGains);
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
      for (int jointIdx = 0; jointIdx < jointsToControl.length; jointIdx++)
      {
         directPositionControlMode[jointIdx].set(defaultDirectPositionControlMode[jointIdx].getValue() && jointControlHelper.hasLowLevelJointGains(jointIdx));
      }
   }

   @Override
   public void onExit(double timeInState)
   {
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      inverseDynamicsCommandList.clear();
      accelerationIntegrationCommand.clear();
      activateJointsCommand.getJointsToActivate().clear();

      for (int jointIdx = 0; jointIdx < jointDesiredOutputList.getNumberOfJointsWithDesiredOutput(); jointIdx++)
      {
         JointAccelerationIntegrationParameters jointParameters = accelerationIntegrationCommand.addJointToComputeDesiredPositionFor(jointsToControl[jointIdx]);
         jointParameters.setDisableAccelerationIntegration(directPositionControlMode[jointIdx].getValue());

         if (directPositionControlMode[jointIdx].getValue())
            activateJointsCommand.deactivateJoint(jointsToControl[jointIdx]);
         else
            activateJointsCommand.activateJoint(jointsToControl[jointIdx]);
      }

      inverseDynamicsCommandList.addCommand(activateJointsCommand);
      inverseDynamicsCommandList.addCommand(accelerationIntegrationCommand);

      return inverseDynamicsCommandList;
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

   public void setEnableDirectJointPositionControl(boolean enable)
   {
      for (int jointIdx = 0; jointIdx < jointsToControl.length; jointIdx++)
      {
         directPositionControlMode[jointIdx].set(enable && jointControlHelper.hasLowLevelJointGains(jointIdx));
      }
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      return null;
   }
}
