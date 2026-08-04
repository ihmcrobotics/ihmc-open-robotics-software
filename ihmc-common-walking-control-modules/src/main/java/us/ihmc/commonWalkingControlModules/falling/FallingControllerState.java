package us.ihmc.commonWalkingControlModules.falling;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.commons.MathTools;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.robotics.trajectories.yoVariables.YoPolynomial;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class FallingControllerState extends HighLevelControllerState
{
   private static final HighLevelControllerName controllerState = HighLevelControllerName.FALLING_STATE;
   private final YoDouble fallTransitionDuration;
   private final YoDouble fallVerticalLoweringDuration;
   private final YoDouble fallGainInterpolationDuration;
   private final YoDouble fallGainInterpolationAlpha;
   private final YoBoolean fallGainInterpolationActive;

   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder;
   private final JointDesiredOutputListReadOnly highLevelControllerOutput;
   private final YoPolynomial trajectory = new YoPolynomial("fallingTrajectory", 4, registry);
   private final YoPolynomial loweringTrajectory = new YoPolynomial("fallingLoweringTrajectory", 4, registry);
   private final YoDouble fallingLoweringAlpha = new YoDouble("fallingLoweringAlpha", registry);
   private final YoDouble fallingConfigurationAlpha = new YoDouble("fallingConfigurationAlpha", registry);
   private final YoEnum<FallingTrajectoryMode> fallingTrajectoryMode = new YoEnum<>("fallingTrajectoryMode", registry, FallingTrajectoryMode.class, false);
   private final YoBoolean enableFallingDampingMode = new YoBoolean("enableFallingDampingMode", registry);
   private final YoBoolean fallingDampingModeActive = new YoBoolean("fallingDampingModeActive", registry);
   private final YoEnum<FallingTrialConfiguration> fallingTrialConfiguration = new YoEnum<>("fallingTrialConfiguration",
                                                                                           registry,
                                                                                           FallingTrialConfiguration.class,
                                                                                           false);

   private final double[] initialJointPositions;
   private final double[] loweringJointPositions;
   private final double[] capturedJointPositions;
   private final double[] capturedJointVelocities;
   private final double[] initialJointStiffnesses;
   private final double[] initialJointDampings;
   private final boolean[] verticalLoweringJoints;
   private final WholeBodySetpointParameters fallingSetpoints;

   public FallingControllerState(CommandInputManager commandInputManager,
                                 StatusMessageOutputManager statusOutputManager,
                                 HighLevelControlManagerFactory managerFactory,
                                 HighLevelHumanoidControllerToolbox controllerToolbox,
                                 HighLevelControllerParameters highLevelControllerParameters,
                                 WalkingControllerParameters walkingControllerParameters)
   {
      this(commandInputManager,
           statusOutputManager,
           managerFactory,
           controllerToolbox,
           highLevelControllerParameters,
           walkingControllerParameters,
           highLevelControllerParameters.getFallingControllerParameters(),
           null);
   }

   public FallingControllerState(CommandInputManager commandInputManager,
                                 StatusMessageOutputManager statusOutputManager,
                                 HighLevelControlManagerFactory managerFactory,
                                 HighLevelHumanoidControllerToolbox controllerToolbox,
                                 HighLevelControllerParameters highLevelControllerParameters,
                                 WalkingControllerParameters walkingControllerParameters,
                                 WholeBodySetpointParameters fallingSetpoints)
   {
      this(commandInputManager,
           statusOutputManager,
           managerFactory,
           controllerToolbox,
           highLevelControllerParameters,
           walkingControllerParameters,
           fallingSetpoints,
           null);
   }

   public FallingControllerState(CommandInputManager commandInputManager,
                                 StatusMessageOutputManager statusOutputManager,
                                 HighLevelControlManagerFactory managerFactory,
                                 HighLevelHumanoidControllerToolbox controllerToolbox,
                                 HighLevelControllerParameters highLevelControllerParameters,
                                 WalkingControllerParameters walkingControllerParameters,
                                 WholeBodySetpointParameters fallingSetpoints,
                                 JointDesiredOutputListReadOnly highLevelControllerOutput)
   {
      super(controllerState, highLevelControllerParameters, controllerToolbox.getControlledOneDoFJoints());
      fallTransitionDuration = new YoDouble("fallTransitionDuration", registry);
      fallTransitionDuration.set(0.75);
      fallVerticalLoweringDuration = new YoDouble("fallVerticalLoweringDuration", registry);
      fallVerticalLoweringDuration.set(0.5);
      fallGainInterpolationDuration = new YoDouble("fallGainInterpolationDuration", registry);
      fallGainInterpolationDuration.set(0.5);
      fallGainInterpolationAlpha = new YoDouble("fallGainInterpolationAlpha", registry);
      fallGainInterpolationActive = new YoBoolean("fallGainInterpolationActive", registry);
      fallingTrajectoryMode.set(FallingTrajectoryMode.LOWER_BODY_PITCH_THEN_CONFIGURATION);
      lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder(controlledJoints.length);
      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledJoints);
      this.highLevelControllerOutput = highLevelControllerOutput;

      initialJointPositions = new double[controlledJoints.length];
      loweringJointPositions = new double[controlledJoints.length];
      capturedJointPositions = new double[controlledJoints.length];
      capturedJointVelocities = new double[controlledJoints.length];
      initialJointStiffnesses = new double[controlledJoints.length];
      initialJointDampings = new double[controlledJoints.length];
      verticalLoweringJoints = new boolean[controlledJoints.length];
      for (int i = 0; i < controlledJoints.length; i++)
      {
         verticalLoweringJoints[i] = isVerticalLoweringJoint(controlledJoints[i].getName());
      }
      this.fallingSetpoints = fallingSetpoints;
      fallingTrialConfiguration.set(FallingTrialConfiguration.DEFAULT);
   }

   @Override
   public void doAction(double timeInState)
   {
      double totalTransitionDuration = getTotalTransitionDuration();
      double loweringDuration = getLoweringDuration();
      double configurationDuration = totalTransitionDuration - loweringDuration;
      boolean useStagedTrajectory = isStagedFallingMode() && loweringDuration > 1.0e-3 && configurationDuration > 1.0e-3;
      boolean useDampingMode = enableFallingDampingMode.getBooleanValue() && timeInState >= totalTransitionDuration;
      fallingDampingModeActive.set(useDampingMode);

      double loweringAlphaPosition = 1.0;
      double loweringAlphaVelocity = 0.0;
      double configurationAlphaPosition = 1.0;
      double configurationAlphaVelocity = 0.0;

      if (useStagedTrajectory && timeInState < loweringDuration)
      {
         double timeInLowering = MathTools.clamp(timeInState, 0.0, loweringDuration);
         loweringTrajectory.compute(timeInLowering);
         loweringAlphaPosition = loweringTrajectory.getValue();
         loweringAlphaVelocity = loweringTrajectory.getVelocity();
         configurationAlphaPosition = 0.0;
         configurationAlphaVelocity = 0.0;
      }
      else if (useStagedTrajectory)
      {
         double timeInConfiguration = MathTools.clamp(timeInState - loweringDuration, 0.0, configurationDuration);
         trajectory.compute(timeInConfiguration);
         loweringAlphaPosition = 1.0;
         loweringAlphaVelocity = 0.0;
         configurationAlphaPosition = trajectory.getValue();
         configurationAlphaVelocity = trajectory.getVelocity();
      }
      else
      {
         double timeInTrajectory = MathTools.clamp(timeInState, 0.0, totalTransitionDuration);
         trajectory.compute(timeInTrajectory);
         loweringAlphaPosition = trajectory.getValue();
         loweringAlphaVelocity = trajectory.getVelocity();
         configurationAlphaPosition = loweringAlphaPosition;
         configurationAlphaVelocity = loweringAlphaVelocity;
      }

      fallingLoweringAlpha.set(loweringAlphaPosition);
      fallingConfigurationAlpha.set(configurationAlphaPosition);

      for (int i = 0; i < controlledJoints.length; i++)
      {
         controlledJoints[i].setTau(0.0);
         JointDesiredOutputBasics lowLevelJointData = lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(controlledJoints[i]);
         lowLevelJointData.clear();
         lowLevelJointData.setDesiredTorque(0.0);

         if (useDampingMode)
         {
            lowLevelJointData.setDesiredVelocity(0.0);
         }
         else
         {
            double desiredPosition;
            double desiredVelocity;

            if (useStagedTrajectory && timeInState < loweringDuration)
            {
               desiredPosition = interpolate(initialJointPositions[i], loweringJointPositions[i], loweringAlphaPosition);
               desiredVelocity = loweringAlphaVelocity * (loweringJointPositions[i] - initialJointPositions[i]);
            }
            else if (useStagedTrajectory)
            {
               desiredPosition = interpolate(loweringJointPositions[i], capturedJointPositions[i], configurationAlphaPosition);
               desiredVelocity = configurationAlphaVelocity * (capturedJointPositions[i] - loweringJointPositions[i]);
            }
            else
            {
               desiredPosition = interpolate(initialJointPositions[i], capturedJointPositions[i], configurationAlphaPosition);
               desiredVelocity = configurationAlphaVelocity * (capturedJointPositions[i] - initialJointPositions[i]);
            }

            lowLevelJointData.setDesiredPosition(desiredPosition);
            lowLevelJointData.setDesiredVelocity(desiredVelocity);
         }
      }

      JointDesiredOutputListReadOnly fallingJointSettings = getStateSpecificJointSettings();
      lowLevelOneDoFJointDesiredDataHolder.completeWith(fallingJointSettings);

      double gainInterpolationAlpha = computeGainInterpolationAlpha(timeInState);
      fallGainInterpolationAlpha.set(gainInterpolationAlpha);
      if (fallGainInterpolationActive.getBooleanValue())
      {
         for (int i = 0; i < controlledJoints.length; i++)
         {
            JointDesiredOutputBasics lowLevelJointData = lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(controlledJoints[i]);
            JointDesiredOutputReadOnly fallingJointData = fallingJointSettings.getJointDesiredOutput(controlledJoints[i]);

            if (fallingJointData.hasStiffness())
               lowLevelJointData.setStiffness(EuclidCoreTools.interpolate(initialJointStiffnesses[i], fallingJointData.getStiffness(), gainInterpolationAlpha));
            if (fallingJointData.hasDamping())
               lowLevelJointData.setDamping(EuclidCoreTools.interpolate(initialJointDampings[i], fallingJointData.getDamping(), gainInterpolationAlpha));
         }
      }
   }

   @Override
   public void onEntry()
   {
      double totalTransitionDuration = getTotalTransitionDuration();
      double loweringDuration = getLoweringDuration();
      double configurationDuration = totalTransitionDuration - loweringDuration;
      loweringTrajectory.setCubic(0.0, Math.max(1.0e-3, loweringDuration), 0.0, 0.0, 1.0, 0.0);
      trajectory.setCubic(0.0, Math.max(1.0e-3, configurationDuration), 0.0, 0.0, 1.0, 0.0);
      fallingLoweringAlpha.set(0.0);
      fallingConfigurationAlpha.set(0.0);
      fallingDampingModeActive.set(false);
      fallGainInterpolationActive.set(getPreviousHighLevelControllerName() == HighLevelControllerName.WALKING && highLevelControllerOutput != null);
      fallGainInterpolationAlpha.set(fallGainInterpolationActive.getBooleanValue() ? 0.0 : 1.0);
      FallingTrialConfiguration selectedFallingTrialConfiguration = fallingTrialConfiguration.getEnumValue();
      JointDesiredOutputListReadOnly fallingJointSettings = getStateSpecificJointSettings();

      for (int i = 0; i < controlledJoints.length; i++)
      {
         initialJointPositions[i] = controlledJoints[i].getQ();

         if (fallingSetpoints != null)
         {
            String jointName = controlledJoints[i].getName();
            if (fallingSetpoints instanceof FallingSetpointParameters)
               capturedJointPositions[i] = ((FallingSetpointParameters) fallingSetpoints).getSetpoint(jointName, selectedFallingTrialConfiguration);
            else
               capturedJointPositions[i] = fallingSetpoints.getSetpoint(jointName);
            capturedJointVelocities[i] = 0.0;
         }
         else
         {
            capturedJointPositions[i] = controlledJoints[i].getQ();
            capturedJointVelocities[i] = controlledJoints[i].getQd();
         }

         loweringJointPositions[i] = verticalLoweringJoints[i] ? capturedJointPositions[i] : initialJointPositions[i];

         JointDesiredOutputReadOnly previousJointData = highLevelControllerOutput != null ? highLevelControllerOutput.getJointDesiredOutput(controlledJoints[i]) : null;
         JointDesiredOutputReadOnly fallingJointData = fallingJointSettings.getJointDesiredOutput(controlledJoints[i]);
         initialJointStiffnesses[i] = previousJointData != null && previousJointData.hasStiffness() ? previousJointData.getStiffness() : fallingJointData.getStiffness();
         initialJointDampings[i] = previousJointData != null && previousJointData.hasDamping() ? previousJointData.getDamping() : fallingJointData.getDamping();
      }
   }

   @Override
   public void onExit(double timeInState)
   {
   }

   @Override
   public JointDesiredOutputListReadOnly getOutputForLowLevelController()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }

   private double getTotalTransitionDuration()
   {
      return Math.max(1.0e-3, fallTransitionDuration.getDoubleValue());
   }

   private double getLoweringDuration()
   {
      return MathTools.clamp(fallVerticalLoweringDuration.getDoubleValue(), 0.0, getTotalTransitionDuration());
   }

   private boolean isStagedFallingMode()
   {
      return fallingTrajectoryMode.getEnumValue() == FallingTrajectoryMode.LOWER_BODY_PITCH_THEN_CONFIGURATION;
   }

   private double computeGainInterpolationAlpha(double timeInState)
   {
      if (!fallGainInterpolationActive.getBooleanValue())
         return 1.0;

      double interpolationDuration = fallGainInterpolationDuration.getDoubleValue();
      return interpolationDuration <= 0.0 ? 1.0 : MathTools.clamp(timeInState / interpolationDuration, 0.0, 1.0);
   }

   private static double interpolate(double start, double end, double alpha)
   {
      return (1.0 - alpha) * start + alpha * end;
   }

   private static boolean isVerticalLoweringJoint(String jointName)
   {
      String lowerCaseJointName = jointName.toLowerCase();
      boolean legJoint = lowerCaseJointName.contains("hip") || lowerCaseJointName.contains("knee") || lowerCaseJointName.contains("ankle");
      boolean pitchJoint = lowerCaseJointName.contains("pitch") || lowerCaseJointName.endsWith("_y") || lowerCaseJointName.endsWith("y");
      boolean yawOrRollJoint = lowerCaseJointName.contains("yaw") || lowerCaseJointName.contains("roll") || lowerCaseJointName.endsWith("_x")
                               || lowerCaseJointName.endsWith("_z");

      return legJoint && pitchJoint && !yawOrRollJoint;
   }
}
