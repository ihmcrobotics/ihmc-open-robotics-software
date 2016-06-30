package us.ihmc.valkyrieRosControl;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import org.yaml.snakeyaml.Yaml;

import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager.StatusMessageListener;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HighLevelStateCommand;
import us.ihmc.humanoidRobotics.communication.packets.HighLevelStateChangeStatusMessage;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingControllerFailureStatusMessage;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.math.filters.GlitchFilteredBooleanYoVariable;
import us.ihmc.robotics.math.filters.SimpleMovingAverageFilteredYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.ForceSensorCalibrationModule;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.taskExecutor.Task;
import us.ihmc.tools.taskExecutor.TaskExecutor;
import us.ihmc.valkyrieRosControl.dataHolders.YoEffortJointHandleHolder;
import us.ihmc.valkyrieRosControl.dataHolders.YoPositionJointHandleHolder;
import us.ihmc.wholeBodyController.diagnostics.JointTorqueOffsetEstimator;

public class ValkyrieRosControlLowLevelController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final TimestampProvider timestampProvider;

   private enum ControlMode {WAITING, CALIBRATION, HIGH_LEVEL_CONTROL};

   private final AtomicBoolean resetIHMCControlRatioAndStandPrepRequested = new AtomicBoolean(false);

   private final ArrayList<ValkyrieRosControlEffortJointControlCommandCalculator> effortControlCommandCalculators = new ArrayList<>();
   private final LinkedHashMap<String, ValkyrieRosControlEffortJointControlCommandCalculator> effortJointToControlCommandCalculatorMap = new LinkedHashMap<>();
   private final ArrayList<ValkyrieRosControlPositionJointControlCommandCalculator> positionControlCommandCalculators = new ArrayList<>();
   private final LinkedHashMap<String, ValkyrieRosControlPositionJointControlCommandCalculator> positionJointToControlCommandCalculatorMap = new LinkedHashMap<>();

   private final DoubleYoVariable yoTime = new DoubleYoVariable("lowLevelControlTime", registry);
   private final DoubleYoVariable wakeUpTime = new DoubleYoVariable("lowLevelControlWakeUpTime", registry);

   private final DoubleYoVariable timeInStandprep = new DoubleYoVariable("timeInStandprep", registry);
   private final DoubleYoVariable standPrepStartTime = new DoubleYoVariable("standPrepStartTime", registry);

   private final DoubleYoVariable doIHMCControlRatio = new DoubleYoVariable("doIHMCControlRatio", registry);
   private final DoubleYoVariable standPrepRampDuration = new DoubleYoVariable("standPrepRampDuration", registry);
   private final DoubleYoVariable masterGain = new DoubleYoVariable("standPrepMasterGain", registry);

   private final DoubleYoVariable controlRatioRampDuration = new DoubleYoVariable("controlRatioRampDuration", registry);
   private final DoubleYoVariable calibrationDuration = new DoubleYoVariable("calibrationDuration", registry);
   private final DoubleYoVariable calibrationStartTime = new DoubleYoVariable("calibrationStartTime", registry);
   private final DoubleYoVariable timeInCalibration = new DoubleYoVariable("timeInCalibration", registry);

   private final BooleanYoVariable requestCalibration = new BooleanYoVariable("requestCalibration", registry);
   private final AtomicBoolean requestCalibrationAtomic = new AtomicBoolean(false);

   private final DoubleYoVariable minFootForceToSwitchToWalking = new DoubleYoVariable("minFootForceToSwitchToWalking", registry);
   private final BooleanYoVariable isRobotReadyToWalk = new BooleanYoVariable("isRobotReadyToWalk", registry);

   private final EnumYoVariable<ControlMode> currentControlMode = new EnumYoVariable<>("lowLevelControlMode", registry, ControlMode.class);

   private final ValkyrieTorqueHysteresisCompensator torqueHysteresisCompensator;
   private final ValkyrieAccelerationIntegration accelerationIntegration;

   private CommandInputManager commandInputManager;
   private final AtomicReference<HighLevelState> currentHighLevelState = new AtomicReference<HighLevelState>(null);

   private final HighLevelStateCommand highLevelStateCommand = new HighLevelStateCommand();

   private final TaskExecutor taskExecutor = new TaskExecutor();
   private final Task rampUpIHMCControlRatioTask;
   private final Task rampDownIHMCControlRatioTask;
   private final Task calibrationTask;

   private ForceSensorCalibrationModule forceSensorCalibrationModule;
   private JointTorqueOffsetEstimator jointTorqueOffsetEstimator;

   private final Wrench tempWrench = new Wrench();
   private SideDependentList<ForceSensorDataReadOnly> feetForceSensorData = new SideDependentList<>();
   private final SideDependentList<GlitchFilteredBooleanYoVariable> areFeetLoaded = new SideDependentList<>();
   private final SideDependentList<SimpleMovingAverageFilteredYoVariable> feetForceMagnitude = new SideDependentList<>();

   @SuppressWarnings("unchecked")
   public ValkyrieRosControlLowLevelController(TimestampProvider timestampProvider, final double updateDT, List<YoEffortJointHandleHolder> yoEffortJointHandleHolders,
         List<YoPositionJointHandleHolder> yoPositionJointHandleHolders, YoVariableRegistry parentRegistry)
   {
      this.timestampProvider = timestampProvider;

      standPrepRampDuration.set(3.0);
      controlRatioRampDuration.set(3.0);
      masterGain.set(0.3);
      calibrationDuration.set(10.0);
      minFootForceToSwitchToWalking.set(50.0);

      currentControlMode.set(ControlMode.WAITING);

      isRobotReadyToWalk.set(true);
      wakeUpTime.set(Double.NaN);
      standPrepStartTime.set(Double.NaN);
      calibrationStartTime.set(Double.NaN);
      
      torqueHysteresisCompensator = new ValkyrieTorqueHysteresisCompensator(yoEffortJointHandleHolders, yoTime, registry);
      accelerationIntegration = new ValkyrieAccelerationIntegration(yoEffortJointHandleHolders, updateDT, registry);

      double minDurationBeforeFootIsLoaded = 3.0;

      for (RobotSide robotSide : RobotSide.values)
      {
         String prefix = "standPrep" + robotSide.getCamelCaseNameForMiddleOfExpression();
         int windowSize = (int) (minDurationBeforeFootIsLoaded / updateDT);
         SimpleMovingAverageFilteredYoVariable footForceMagnitude = new SimpleMovingAverageFilteredYoVariable(prefix + "FootForceMag", windowSize, parentRegistry);
         feetForceMagnitude.put(robotSide, footForceMagnitude);
         GlitchFilteredBooleanYoVariable isFootLoaded = new GlitchFilteredBooleanYoVariable(prefix + "FootIsLoaded", registry, windowSize);
         areFeetLoaded.put(robotSide, isFootLoaded);
      }

      requestCalibration.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            if (requestCalibration.getBooleanValue())
               requestCalibrationAtomic.set(true);
         }
      });

      rampDownIHMCControlRatioTask = new AbstractLowLevelTask()
      {
         @Override
         public void doAction()
         {
            double newRatio = doIHMCControlRatio.getDoubleValue() - updateDT / controlRatioRampDuration.getDoubleValue();
            doIHMCControlRatio.set(MathTools.clipToMinMax(newRatio, 0.0, 1.0));
         }

         @Override
         public boolean isDone()
         {
            return doIHMCControlRatio.getDoubleValue() == 0.0;
         }
      };

      rampUpIHMCControlRatioTask = new AbstractLowLevelTask()
      {
         @Override
         public void doAction()
         {
            double newRatio = doIHMCControlRatio.getDoubleValue() + updateDT / controlRatioRampDuration.getDoubleValue();
            doIHMCControlRatio.set(MathTools.clipToMinMax(newRatio, 0.0, 1.0));
         }

         @Override
         public boolean isDone()
         {
            return doIHMCControlRatio.getDoubleValue() == 1.0;
         }
      };

      calibrationTask = new AbstractLowLevelTask()
      {
         @Override
         public void doAction()
         {
            timeInCalibration.set(yoTime.getDoubleValue() - calibrationStartTime.getDoubleValue());
            if (timeInCalibration.getDoubleValue() >= calibrationDuration.getDoubleValue() - 0.1)
            {
               if (jointTorqueOffsetEstimator != null)
                  jointTorqueOffsetEstimator.enableJointTorqueOffsetEstimationAtomic(false);
            }
         }

         @Override
         public void doTransitionIntoAction()
         {
            calibrationStartTime.set(yoTime.getDoubleValue());
            if (jointTorqueOffsetEstimator != null)
               jointTorqueOffsetEstimator.enableJointTorqueOffsetEstimationAtomic(true);
         }

         @Override
         public void doTransitionOutOfAction()
         {
            calibrationStartTime.set(Double.NaN);

            List<OneDoFJoint> oneDoFJoints = jointTorqueOffsetEstimator.getOneDoFJoints();

            for (int i = 0; i < oneDoFJoints.size(); i++)
            {
               OneDoFJoint joint = oneDoFJoints.get(i);
               if (jointTorqueOffsetEstimator.hasTorqueOffsetForJoint(joint))
               {
                  subtractTorqueOffset(joint, jointTorqueOffsetEstimator.getEstimatedJointTorqueOffset(joint));
                  jointTorqueOffsetEstimator.resetEstimatedJointTorqueOffset(joint);
               }
            }

            if (forceSensorCalibrationModule != null)
               forceSensorCalibrationModule.requestFootForceSensorCalibrationAtomic();
         }

         @Override
         public boolean isDone()
         {
            return timeInCalibration.getDoubleValue() >= calibrationDuration.getDoubleValue();
         }
      };

      Yaml yaml = new Yaml();
      InputStream gainStream = getClass().getClassLoader().getResourceAsStream("standPrep/gains.yaml");
      InputStream setpointsStream = getClass().getClassLoader().getResourceAsStream("standPrep/setpoints.yaml");
      InputStream offsetsStream;
      try
      {
         offsetsStream = new FileInputStream(new File(ValkyrieTorqueOffsetPrinter.IHMC_TORQUE_OFFSET_FILE));
      }
      catch (FileNotFoundException e1)
      {
         offsetsStream = null;
      }

      Map<String, Map<String, Double>> gainMap = (Map<String, Map<String, Double>>) yaml.load(gainStream);
      Map<String, Double> setPointMap = (Map<String, Double>) yaml.load(setpointsStream);
      Map<String, Double> offsetMap = null;
      if (offsetsStream != null)
         offsetMap = (Map<String, Double>) yaml.load(offsetsStream);

      try
      {
         gainStream.close();
         setpointsStream.close();
         if (offsetsStream != null)
            offsetsStream.close();
      }
      catch (IOException e)
      {
      }

      for (YoEffortJointHandleHolder effortJointHandleHolder : yoEffortJointHandleHolders)
      {
         String jointName = effortJointHandleHolder.getName();
         Map<String, Double> standPrepGains = gainMap.get(jointName);
         double torqueOffset = 0.0;
         if (offsetMap != null && offsetMap.containsKey(jointName))
            torqueOffset = offsetMap.get(jointName);

         double standPrepAngle = 0.0;
         if (setPointMap.containsKey(jointName))
         {
            standPrepAngle = setPointMap.get(jointName);
         }
         ValkyrieRosControlEffortJointControlCommandCalculator controlCommandCalculator = new ValkyrieRosControlEffortJointControlCommandCalculator(
               effortJointHandleHolder, standPrepGains, torqueOffset, standPrepAngle, updateDT, registry);
         effortControlCommandCalculators.add(controlCommandCalculator);

         effortJointToControlCommandCalculatorMap.put(jointName, controlCommandCalculator);
      }

      for (YoPositionJointHandleHolder positionJointHandleHolder : yoPositionJointHandleHolders)
      {
         String jointName = positionJointHandleHolder.getName();
         Map<String, Double> standPrepGains = gainMap.get(jointName);

         double standPrepAngle = 0.0;
         if (setPointMap.containsKey(jointName))
         {
            standPrepAngle = setPointMap.get(jointName);
         }
         ValkyrieRosControlPositionJointControlCommandCalculator controlCommandCalculator = new ValkyrieRosControlPositionJointControlCommandCalculator(
               positionJointHandleHolder, standPrepGains, standPrepAngle, updateDT, registry);
         positionControlCommandCalculators.add(controlCommandCalculator);

         positionJointToControlCommandCalculatorMap.put(jointName, controlCommandCalculator);
      }

      parentRegistry.addChild(registry);
   }

   public void setDoIHMCControlRatio(double controlRatio)
   {
      doIHMCControlRatio.set(MathTools.clipToMinMax(controlRatio, 0.0, 1.0));
   }

   public void requestCalibration()
   {
      requestCalibrationAtomic.set(true);
   }

   public void doControl()
   {
      long timestamp = timestampProvider.getTimestamp();

      if (wakeUpTime.isNaN())
         wakeUpTime.set(TimeTools.nanoSecondstoSeconds(timestamp));

      yoTime.set(TimeTools.nanoSecondstoSeconds(timestamp) - wakeUpTime.getDoubleValue());

      switch (currentControlMode.getEnumValue())
      {
      case WAITING:
         if (doIHMCControlRatio.getDoubleValue() > 1.0 - 1.0e-3 && currentHighLevelState.get() == HighLevelState.WALKING)
         {
            currentControlMode.set(ControlMode.HIGH_LEVEL_CONTROL);
            break;
         }

         if (requestCalibrationAtomic.getAndSet(false))
         {
            requestCalibration.set(false);
            highLevelStateCommand.setHighLevelState(HighLevelState.CALIBRATION);
            commandInputManager.submitCommand(highLevelStateCommand);
            taskExecutor.submit(rampUpIHMCControlRatioTask);
            taskExecutor.submit(calibrationTask);
            taskExecutor.submit(rampDownIHMCControlRatioTask);
            currentControlMode.set(ControlMode.CALIBRATION);
            break;
         }

         if (isRobotReadyToWalk.getBooleanValue())
         {
            boolean areBothFeetLoaded = true;
            
            for (RobotSide robotSide : RobotSide.values)
            {
               feetForceSensorData.get(robotSide).getWrench(tempWrench);
               SimpleMovingAverageFilteredYoVariable footForceMagnitude = feetForceMagnitude.get(robotSide);
               footForceMagnitude.update(Math.abs(tempWrench.getLinearPartZ()));
               GlitchFilteredBooleanYoVariable isFootLoaded = areFeetLoaded.get(robotSide);
               isFootLoaded.update(footForceMagnitude.getDoubleValue() > minFootForceToSwitchToWalking.getDoubleValue());
               areBothFeetLoaded &= isFootLoaded.getBooleanValue();
            }
            
            if (areBothFeetLoaded)
            {
               highLevelStateCommand.setHighLevelState(HighLevelState.WALKING);
               commandInputManager.submitCommand(highLevelStateCommand);
               taskExecutor.submit(rampUpIHMCControlRatioTask);
               currentControlMode.set(ControlMode.HIGH_LEVEL_CONTROL);
            }
         }
         break;

      case CALIBRATION:
         resetFeetForceData();
         ignoreCalibrationRequests();

         if (!taskExecutor.isDone())
            taskExecutor.doControl();
         else
         {
            highLevelStateCommand.setHighLevelState(HighLevelState.DO_NOTHING_BEHAVIOR);
            commandInputManager.submitCommand(highLevelStateCommand);
            currentControlMode.set(ControlMode.WAITING);
         }
         break;
      case HIGH_LEVEL_CONTROL:
         resetFeetForceData();
         ignoreCalibrationRequests();

         taskExecutor.doControl();

         if (resetIHMCControlRatioAndStandPrepRequested.getAndSet(false))
         {
            standPrepStartTime.set(Double.NaN);
            doIHMCControlRatio.set(0.0);
            taskExecutor.clear();
            currentControlMode.set(ControlMode.WAITING);
            isRobotReadyToWalk.set(false);
            break;
         }

         if (doIHMCControlRatio.getDoubleValue() < 1.0e-3 && taskExecutor.isDone())
         {
            currentControlMode.set(ControlMode.WAITING);
            isRobotReadyToWalk.set(false);
            break;
         }

         torqueHysteresisCompensator.compute();
         if (ValkyrieRosControlController.INTEGRATE_ACCELERATIONS_AND_CONTROL_VELOCITIES)
            accelerationIntegration.compute();
         break;
      default:
         break;
      }

      updateCommandCalculators();
   }

   private void resetFeetForceData()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         feetForceMagnitude.get(robotSide).set(0.0);
         areFeetLoaded.get(robotSide).set(false);
      }
   }

   public void ignoreCalibrationRequests()
   {
      requestCalibration.set(false);
      requestCalibrationAtomic.set(false);
   }

   public void updateCommandCalculators()
   {
      if (!standPrepStartTime.isNaN())
      {
         timeInStandprep.set(yoTime.getDoubleValue() - standPrepStartTime.getDoubleValue());

         double ramp = timeInStandprep.getDoubleValue() / standPrepRampDuration.getDoubleValue();
         ramp = MathTools.clipToMinMax(ramp, 0.0, 1.0);

         for (int i = 0; i < effortControlCommandCalculators.size(); i++)
         {
            ValkyrieRosControlEffortJointControlCommandCalculator commandCalculator = effortControlCommandCalculators.get(i);
            commandCalculator.computeAndUpdateJointTorque(ramp, doIHMCControlRatio.getDoubleValue(), masterGain.getDoubleValue());
         }

         for (int i = 0; i < positionControlCommandCalculators.size(); i++)
         {
            ValkyrieRosControlPositionJointControlCommandCalculator commandCalculator = positionControlCommandCalculators.get(i);
            commandCalculator.computeAndUpdateJointPosition(ramp, doIHMCControlRatio.getDoubleValue(), masterGain.getDoubleValue());
         }
      }
      else
      {
         standPrepStartTime.set(yoTime.getDoubleValue());

         for (int i = 0; i < effortControlCommandCalculators.size(); i++)
         {
            ValkyrieRosControlEffortJointControlCommandCalculator commandCalculator = effortControlCommandCalculators.get(i);
            commandCalculator.initialize();
         }

         for (int i = 0; i < positionControlCommandCalculators.size(); i++)
         {
            ValkyrieRosControlPositionJointControlCommandCalculator commandCalculator = positionControlCommandCalculators.get(i);
            commandCalculator.initialize();
         }
      }
   }

   public void subtractTorqueOffset(OneDoFJoint oneDoFJoint, double torqueOffset)
   {
      ValkyrieRosControlEffortJointControlCommandCalculator jointCommandCalculator = effortJointToControlCommandCalculatorMap.get(oneDoFJoint.getName());
      if (jointCommandCalculator != null)
         jointCommandCalculator.subtractTorqueOffset(torqueOffset);
      else
         PrintTools.error("Command calculator is NULL for the joint: " + oneDoFJoint.getName());
   }

   public void attachControllerAPI(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager)
   {
      this.commandInputManager = commandInputManager;

      StatusMessageListener<HighLevelStateChangeStatusMessage> highLevelStateChangeListener = new StatusMessageListener<HighLevelStateChangeStatusMessage>()
      {
         @Override
         public void receivedNewMessageStatus(HighLevelStateChangeStatusMessage statusMessage)
         {
            if (statusMessage != null)
               currentHighLevelState.set(statusMessage.endState);
         }
      };
      statusOutputManager.attachStatusMessageListener(HighLevelStateChangeStatusMessage.class, highLevelStateChangeListener);

      StatusMessageListener<WalkingControllerFailureStatusMessage> controllerFailureListener = new StatusMessageListener<WalkingControllerFailureStatusMessage>()
      {
         @Override
         public void receivedNewMessageStatus(WalkingControllerFailureStatusMessage statusMessage)
         {
            if (statusMessage != null)
               resetIHMCControlRatioAndStandPrepRequested.set(true);
         }
      };
      statusOutputManager.attachStatusMessageListener(WalkingControllerFailureStatusMessage.class, controllerFailureListener);
   }
   
   public void attachForceSensorCalibrationModule(DRCRobotSensorInformation sensorInformation, ForceSensorCalibrationModule forceSensorCalibrationModule)
   {
      this.forceSensorCalibrationModule = forceSensorCalibrationModule;
      SideDependentList<String> feetForceSensorNames = sensorInformation.getFeetForceSensorNames();
      ForceSensorDataHolderReadOnly forceSensorOutput = forceSensorCalibrationModule.getForceSensorOutput();
      for (RobotSide robotSide : RobotSide.values)
      {
         ForceSensorDataReadOnly footForceSensorData = forceSensorOutput.getByName(feetForceSensorNames.get(robotSide));
         feetForceSensorData.put(robotSide, footForceSensorData);
      }
   }

   public void attachJointTorqueOffsetEstimator(JointTorqueOffsetEstimator jointTorqueOffsetEstimator)
   {
      this.jointTorqueOffsetEstimator = jointTorqueOffsetEstimator;
   }

   private abstract class AbstractLowLevelTask implements Task
   {
      @Override
      public void doAction()
      {
      }

      @Override
      public void doTransitionIntoAction()
      {
      }

      @Override
      public void doTransitionOutOfAction()
      {
      }
   }
}
