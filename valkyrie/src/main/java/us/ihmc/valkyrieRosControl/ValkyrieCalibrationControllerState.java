package us.ihmc.valkyrieRosControl;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.MathTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.math.trajectories.yoVariables.YoPolynomial;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.ForceSensorCalibrationModule;
import us.ihmc.tools.lists.PairList;
import us.ihmc.valkyrie.ValkyrieCalibrationParameters;
import us.ihmc.wholeBodyController.diagnostics.CalibrationState;
import us.ihmc.wholeBodyController.diagnostics.JointTorqueOffsetEstimatorController;
import us.ihmc.wholeBodyController.diagnostics.JointTorqueOffsetEstimatorParameters;
import us.ihmc.wholeBodyController.diagnostics.TorqueOffsetPrinter;
import us.ihmc.yoVariables.variable.YoDouble;

public class ValkyrieCalibrationControllerState extends HighLevelControllerState
{
   private static final HighLevelControllerName controllerState = HighLevelControllerName.CALIBRATION;

   private enum CalibrationStates
   {
      ENTRY, CALIBRATE, EXIT
   }

   private static final double timeToMove = 3.0;

   private ForceSensorCalibrationModule forceSensorCalibrationModule;
   private final StateMachine<CalibrationStates, CalibrationState> stateMachine;

   private final PairList<OneDoFJointBasics, TrajectoryData> jointsData = new PairList<>();

   private final JointTorqueOffsetEstimatorController jointTorqueOffsetEstimatorController;
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();
   private final YoDouble timeToMoveForCalibration = new YoDouble("timeToMoveForCalibration", registry);
   private final YoDouble timeForEstimatingOffset = new YoDouble("timeForEstimatingOffset", registry);

   private final JointDesiredOutputListReadOnly highLevelControlOutput;

   public ValkyrieCalibrationControllerState(HighLevelHumanoidControllerToolbox highLevelControllerToolbox,
                                             HighLevelControllerParameters highLevelControllerParameters,
                                             JointDesiredOutputListReadOnly highLevelControlOutput,
                                             ValkyrieCalibrationParameters calibrationParameters, TorqueOffsetPrinter torqueOffsetPrinter)
   {
      super(controllerState, highLevelControllerParameters, MultiBodySystemTools.filterJoints(highLevelControllerToolbox.getControlledJoints(), OneDoFJoint.class));
      this.highLevelControlOutput = highLevelControlOutput;

      for (OneDoFJointBasics controlledJoint : controlledJoints)
      {
         String jointName = controlledJoint.getName();
         YoPolynomial trajectory = new YoPolynomial(jointName + "_CalibrationTrajectory", 4, registry);
         YoDouble initialPosition = new YoDouble(jointName + "_CalibrationInitialPosition", registry);
         jointsData.add(controlledJoint, new TrajectoryData(initialPosition, trajectory));
      }

      timeToMoveForCalibration.set(timeToMove);
      timeForEstimatingOffset.set(highLevelControllerParameters.getCalibrationDuration());

      boolean useArms = ValkyrieRosControlController.VERSION.hasBothArms();
      JointTorqueOffsetEstimatorParameters parameters = new JointTorqueOffsetEstimatorParameters();
      if (!useArms)
         parameters.setArmJointsToRun(null);
      jointTorqueOffsetEstimatorController = new JointTorqueOffsetEstimatorController(calibrationParameters, highLevelControllerToolbox, torqueOffsetPrinter, parameters);
      registry.addChild(jointTorqueOffsetEstimatorController.getYoRegistry());

      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledJoints);

      
      StateMachineFactory<CalibrationStates, CalibrationState> factory = new StateMachineFactory<>(CalibrationStates.class);
      factory.setNamePrefix("calibrationState").setRegistry(registry).buildYoClock(highLevelControllerToolbox.getYoTime());
      factory.addStateAndDoneTransition(CalibrationStates.ENTRY, new CalibrationEntry(), CalibrationStates.CALIBRATE);
      factory.addStateAndDoneTransition(CalibrationStates.CALIBRATE, new Calibration(), CalibrationStates.EXIT);
      factory.addState(CalibrationStates.EXIT, new CalibrationExit());
      stateMachine = factory.build(CalibrationStates.ENTRY);
   }

   public void attachForceSensorCalibrationModule(ForceSensorCalibrationModule forceSensorCalibrationModule)
   {
      this.forceSensorCalibrationModule = forceSensorCalibrationModule;
   }

   @Override
   public boolean isDone(double timeInState)
   {
      if (stateMachine.getCurrentStateKey() == CalibrationStates.EXIT)
         return stateMachine.getCurrentState().isDone(stateMachine.getTimeInCurrentState());
      else
         return false;
   }

   @Override
   public void onExit(double timeInState)
   {
      stateMachine.getCurrentState().onExit(timeInState);

   }

   @Override
   public void onEntry()
   {
      stateMachine.resetToInitialState();
   }

   @Override
   public void doAction(double timeInState)
   {
      stateMachine.doActionAndTransition();
   }

   @Override
   public JointDesiredOutputListReadOnly getOutputForLowLevelController()
   {
      return stateMachine.getCurrentState().getOutputForLowLevelController();
   }

   public JointTorqueOffsetEstimatorController getJointTorqueOffsetEstimatorController()
   {
      return jointTorqueOffsetEstimatorController;
   }

   private class CalibrationEntry implements CalibrationState
   {
      @Override
      public JointDesiredOutputListReadOnly getOutputForLowLevelController()
      {
         return lowLevelOneDoFJointDesiredDataHolder;
      }

      @Override
      public boolean isDone(double timeInState)
      {
         return timeInState > timeToMoveForCalibration.getDoubleValue();
      }

      @Override
      public void doAction(double timeInState)
      {
         double timeInTrajectory = MathTools.clamp(timeInState, 0.0, timeToMoveForCalibration.getDoubleValue());

         jointTorqueOffsetEstimatorController.doControl();

         for (int jointIndex = 0; jointIndex < jointsData.size(); jointIndex++)
         {
            OneDoFJointBasics joint = jointsData.get(jointIndex).getLeft();
            TrajectoryData trajectoryData = jointsData.get(jointIndex).getRight();

            YoPolynomial trajectory = trajectoryData.getTrajectory();

            trajectory.compute(timeInTrajectory);
            double desiredPosition = trajectory.getValue();

            JointDesiredOutputBasics lowLevelJointData = lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(joint);
            lowLevelJointData.clear();
            lowLevelJointData.setDesiredPosition(desiredPosition);
            lowLevelJointData.setDesiredVelocity(trajectory.getVelocity());
            lowLevelJointData.setDesiredAcceleration(trajectory.getAcceleration());

            JointDesiredOutputReadOnly estimatorOutput = jointTorqueOffsetEstimatorController.getOutputForLowLevelController().getJointDesiredOutput(joint);
            if (estimatorOutput != null && estimatorOutput.hasDesiredTorque())
            {
               double desiredTorque = estimatorOutput.getDesiredTorque();
               desiredTorque *= timeInTrajectory / timeToMoveForCalibration.getDoubleValue();
               lowLevelJointData.setDesiredTorque(desiredTorque);
            }
         }

         lowLevelOneDoFJointDesiredDataHolder.completeWith(getStateSpecificJointSettings());
      }

      @Override
      public void onEntry()
      {
         for (int i = 0; i < jointsData.size(); i++)
         {
            OneDoFJointBasics joint = jointsData.get(i).getLeft();
            TrajectoryData trajectoryData = jointsData.get(i).getRight();

            YoDouble initialPosition = trajectoryData.getInitialPosition();
            YoPolynomial trajectory = trajectoryData.getTrajectory();

            JointDesiredOutputReadOnly jointDesiredOutput = highLevelControlOutput.getJointDesiredOutput(joint);

            double startAngle = jointDesiredOutput != null && jointDesiredOutput.hasDesiredPosition() ? jointDesiredOutput.getDesiredPosition() : joint.getQ();
            double startVelocity = 0.0;

            double finalAngle = jointTorqueOffsetEstimatorController.getJointCalibrationPosition(joint);
            double finalVelocity = 0.0;

            initialPosition.set(startAngle);

            trajectory.setCubic(0.0, timeToMoveForCalibration.getDoubleValue(), startAngle, startVelocity, finalAngle, finalVelocity);
         }
         jointTorqueOffsetEstimatorController.initialize();
      }

      @Override
      public void onExit(double timeInState)
      {
      }
   }

   private class Calibration implements CalibrationState
   {
      @Override
      public void doAction(double timeInState)
      {
         jointTorqueOffsetEstimatorController.doControl();
      }

      @Override
      public void onEntry()
      {
         jointTorqueOffsetEstimatorController.estimateTorqueOffset(true);
      }

      @Override
      public void onExit(double timeInState)
      {
         jointTorqueOffsetEstimatorController.estimateTorqueOffset(false);
         jointTorqueOffsetEstimatorController.exportTorqueOffsets();

         if (forceSensorCalibrationModule != null)
            forceSensorCalibrationModule.requestFootForceSensorCalibrationAtomic();
      }

      @Override
      public boolean isDone(double timeInState)
      {
         return timeInState > timeForEstimatingOffset.getDoubleValue();
      }

      @Override
      public JointDesiredOutputListReadOnly getOutputForLowLevelController()
      {
         return jointTorqueOffsetEstimatorController.getOutputForLowLevelController();
      }
   }

   private class CalibrationExit implements CalibrationState
   {
      @Override
      public JointDesiredOutputListReadOnly getOutputForLowLevelController()
      {
         return lowLevelOneDoFJointDesiredDataHolder;
      }

      @Override
      public boolean isDone(double timeInState)
      {
         return timeInState > timeToMoveForCalibration.getDoubleValue();
      }

      @Override
      public void doAction(double timeInState)
      {
         double timeInTrajectory = MathTools.clamp(timeInState, 0.0, timeToMoveForCalibration.getDoubleValue());

         for (int jointIndex = 0; jointIndex < jointsData.size(); jointIndex++)
         {
            OneDoFJointBasics joint = jointsData.get(jointIndex).getLeft();
            TrajectoryData trajectoryData = jointsData.get(jointIndex).getRight();

            YoPolynomial trajectory = trajectoryData.getTrajectory();

            trajectory.compute(timeInTrajectory);
            double desiredPosition = trajectory.getValue();

            JointDesiredOutputBasics lowLevelJointData = lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(joint);
            lowLevelJointData.clear();
            lowLevelJointData.setDesiredPosition(desiredPosition);
            lowLevelJointData.setDesiredVelocity(trajectory.getVelocity());
            lowLevelJointData.setDesiredAcceleration(trajectory.getAcceleration());

            JointDesiredOutputReadOnly estimatorOutput = jointTorqueOffsetEstimatorController.getOutputForLowLevelController().getJointDesiredOutput(joint);
            if (estimatorOutput != null && estimatorOutput.hasDesiredTorque())
            {
               double desiredTorque = estimatorOutput.getDesiredTorque();
               desiredTorque *= 1.0 - timeInTrajectory / timeToMoveForCalibration.getDoubleValue();
               lowLevelJointData.setDesiredTorque(desiredTorque);
            }
         }

         lowLevelOneDoFJointDesiredDataHolder.completeWith(getStateSpecificJointSettings());
      }

      @Override
      public void onEntry()
      {
         for (int i = 0; i < jointsData.size(); i++)
         {
            OneDoFJointBasics joint = jointsData.get(i).getLeft();
            TrajectoryData trajectoryData = jointsData.get(i).getRight();

            YoDouble initialPosition = trajectoryData.getInitialPosition();
            YoPolynomial trajectory = trajectoryData.getTrajectory();

            double startAngle = jointTorqueOffsetEstimatorController.getJointCalibrationPosition(joint);
            double startVelocity = 0.0;

            double finalAngle = initialPosition.getDoubleValue();
            double finalVelocity = 0.0;

            trajectory.setCubic(0.0, timeToMoveForCalibration.getDoubleValue(), startAngle, startVelocity, finalAngle, finalVelocity);
         }
      }

      @Override
      public void onExit(double timeInState)
      {
      }
   }

   private class TrajectoryData
   {
      private final YoDouble initialPosition;
      private final YoPolynomial trajectory;

      public TrajectoryData(YoDouble initialPosition, YoPolynomial trajectory)
      {
         this.initialPosition = initialPosition;
         this.trajectory = trajectory;
      }

      public YoDouble getInitialPosition()
      {
         return initialPosition;
      }

      public YoPolynomial getTrajectory()
      {
         return trajectory;
      }
   }
}
