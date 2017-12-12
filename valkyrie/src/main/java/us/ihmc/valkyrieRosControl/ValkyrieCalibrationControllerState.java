package us.ihmc.valkyrieRosControl;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.MathTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.GenericStateMachine;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.ForceSensorCalibrationModule;
import us.ihmc.tools.lists.PairList;
import us.ihmc.valkyrie.ValkyrieCalibrationParameters;
import us.ihmc.wholeBodyController.diagnostics.CalibrationState;
import us.ihmc.wholeBodyController.diagnostics.JointTorqueOffsetEstimatorController;
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
   private final GenericStateMachine<CalibrationStates, CalibrationState<CalibrationStates>> stateMachine;

   private final PairList<OneDoFJoint, TrajectoryData> jointsData = new PairList<>();

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
      super(controllerState, highLevelControllerParameters, highLevelControllerToolbox);
      this.highLevelControlOutput = highLevelControlOutput;

      OneDoFJoint[] controlledJoints = highLevelControllerToolbox.getFullRobotModel().getOneDoFJoints();

      for (OneDoFJoint controlledJoint : controlledJoints)
      {
         String jointName = controlledJoint.getName();
         YoPolynomial trajectory = new YoPolynomial(jointName + "_CalibrationTrajectory", 4, registry);
         YoDouble calibrationPosition = new YoDouble(jointName + "_CalibrationPosition", registry);
         YoDouble initialPosition = new YoDouble(jointName + "_CalibrationInitialPosition", registry);

         calibrationPosition.set(calibrationParameters.getSetpoint(jointName));

         jointsData.add(controlledJoint, new TrajectoryData(initialPosition, calibrationPosition, trajectory));
      }

      timeToMoveForCalibration.set(timeToMove);
      timeForEstimatingOffset.set(highLevelControllerParameters.getCalibrationDuration());

      jointTorqueOffsetEstimatorController = new JointTorqueOffsetEstimatorController(calibrationParameters, highLevelControllerToolbox, torqueOffsetPrinter);
      registry.addChild(jointTorqueOffsetEstimatorController.getYoVariableRegistry());

      OneDoFJoint[] jointArray = ScrewTools.filterJoints(highLevelControllerToolbox.getControlledJoints(), OneDoFJoint.class);
      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(jointArray);

      CalibrationEntry calibrationEntry = new CalibrationEntry();
      Calibration calibration = new Calibration();
      CalibrationExit calibrationExit = new CalibrationExit();

      calibrationEntry.setDefaultNextState(CalibrationStates.CALIBRATE);
      calibration.setDefaultNextState(CalibrationStates.EXIT);

      stateMachine = new GenericStateMachine<>("calibrationState", "calibrationTime", CalibrationStates.class, highLevelControllerToolbox.getYoTime(),
                                               registry);

      stateMachine.addState(calibrationEntry);
      stateMachine.addState(calibration);
      stateMachine.addState(calibrationExit);

      stateMachine.setCurrentState(CalibrationStates.ENTRY);

   }

   public void attachForceSensorCalibrationModule(ForceSensorCalibrationModule forceSensorCalibrationModule)
   {
      this.forceSensorCalibrationModule = forceSensorCalibrationModule;
   }

   @Override
   public boolean isDone()
   {
      if (stateMachine.isCurrentState(CalibrationStates.EXIT))
         return stateMachine.getCurrentState().isDone();
      else
         return false;
   }

   @Override
   public void doTransitionOutOfAction()
   {
      stateMachine.getCurrentState().doTransitionOutOfAction();

   }

   @Override
   public void doTransitionIntoAction()
   {
      stateMachine.setCurrentState(CalibrationStates.ENTRY);
      stateMachine.getCurrentState().doTransitionIntoAction();
   }

   @Override
   public void doAction()
   {
      stateMachine.checkTransitionConditions();
      stateMachine.doAction();
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

   private class CalibrationEntry extends CalibrationState<CalibrationStates>
   {

      public CalibrationEntry()
      {
         super(CalibrationStates.ENTRY);
      }

      @Override
      public JointDesiredOutputListReadOnly getOutputForLowLevelController()
      {
         return lowLevelOneDoFJointDesiredDataHolder;
      }

      @Override
      public boolean isDone()
      {
         return getTimeInCurrentState() > timeToMoveForCalibration.getDoubleValue();
      }

      @Override
      public void doAction()
      {
         double timeInTrajectory = MathTools.clamp(getTimeInCurrentState(), 0.0, timeToMoveForCalibration.getDoubleValue());

         if (isDone())
            transitionToDefaultNextState();

         jointTorqueOffsetEstimatorController.doControl();

         for (int jointIndex = 0; jointIndex < jointsData.size(); jointIndex++)
         {
            OneDoFJoint joint = jointsData.get(jointIndex).getLeft();
            TrajectoryData trajectoryData = jointsData.get(jointIndex).getRight();

            YoPolynomial trajectory = trajectoryData.getTrajectory();

            trajectory.compute(timeInTrajectory);
            double desiredPosition = trajectory.getPosition();

            JointDesiredOutput lowLevelJointData = lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(joint);
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
      public void doTransitionIntoAction()
      {
         for (int i = 0; i < jointsData.size(); i++)
         {
            OneDoFJoint joint = jointsData.get(i).getLeft();
            TrajectoryData trajectoryData = jointsData.get(i).getRight();

            YoDouble initialPosition = trajectoryData.getInitialPosition();
            YoDouble calibrationPosition = trajectoryData.getCalibrationPosition();
            YoPolynomial trajectory = trajectoryData.getTrajectory();

            JointDesiredOutputReadOnly jointDesiredOutput = highLevelControlOutput.getJointDesiredOutput(joint);

            double startAngle = jointDesiredOutput != null && jointDesiredOutput.hasDesiredPosition() ? jointDesiredOutput.getDesiredPosition() : joint.getQ();
            double startVelocity = 0.0;

            double finalAngle = calibrationPosition.getDoubleValue();
            double finalVelocity = 0.0;

            initialPosition.set(startAngle);

            trajectory.setCubic(0.0, timeToMoveForCalibration.getDoubleValue(), startAngle, startVelocity, finalAngle, finalVelocity);
         }
         jointTorqueOffsetEstimatorController.initialize();
      }

      @Override
      public void doTransitionOutOfAction()
      {
      }
   }

   private class Calibration extends CalibrationState<CalibrationStates>
   {
      public Calibration()
      {
         super(CalibrationStates.CALIBRATE);
      }

      @Override
      public void doAction()
      {
         if (isDone())
            transitionToDefaultNextState();

         jointTorqueOffsetEstimatorController.doControl();
      }

      @Override
      public void doTransitionIntoAction()
      {
         jointTorqueOffsetEstimatorController.estimateTorqueOffset(true);
      }

      @Override
      public void doTransitionOutOfAction()
      {
         jointTorqueOffsetEstimatorController.estimateTorqueOffset(false);
         jointTorqueOffsetEstimatorController.exportTorqueOffsets();

         if (forceSensorCalibrationModule != null)
            forceSensorCalibrationModule.requestFootForceSensorCalibrationAtomic();
      }

      @Override
      public boolean isDone()
      {
         return getTimeInCurrentState() > timeForEstimatingOffset.getDoubleValue();
      }

      @Override
      public JointDesiredOutputListReadOnly getOutputForLowLevelController()
      {
         return jointTorqueOffsetEstimatorController.getOutputForLowLevelController();
      }
   }

   private class CalibrationExit extends CalibrationState<CalibrationStates>
   {
      public CalibrationExit()
      {
         super(CalibrationStates.EXIT);
      }

      @Override
      public JointDesiredOutputListReadOnly getOutputForLowLevelController()
      {
         return lowLevelOneDoFJointDesiredDataHolder;
      }

      @Override
      public boolean isDone()
      {
         return getTimeInCurrentState() > timeToMoveForCalibration.getDoubleValue();
      }

      @Override
      public void doAction()
      {
         double timeInTrajectory = MathTools.clamp(getTimeInCurrentState(), 0.0, timeToMoveForCalibration.getDoubleValue());

         for (int jointIndex = 0; jointIndex < jointsData.size(); jointIndex++)
         {
            OneDoFJoint joint = jointsData.get(jointIndex).getLeft();
            TrajectoryData trajectoryData = jointsData.get(jointIndex).getRight();

            YoPolynomial trajectory = trajectoryData.getTrajectory();

            trajectory.compute(timeInTrajectory);
            double desiredPosition = trajectory.getPosition();

            JointDesiredOutput lowLevelJointData = lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(joint);
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
      public void doTransitionIntoAction()
      {
         for (int i = 0; i < jointsData.size(); i++)
         {
            TrajectoryData trajectoryData = jointsData.get(i).getRight();

            YoDouble calibrationPosition = trajectoryData.getCalibrationPosition();
            YoDouble initialPosition = trajectoryData.getInitialPosition();
            YoPolynomial trajectory = trajectoryData.getTrajectory();

            double startAngle = calibrationPosition.getDoubleValue();
            double startVelocity = 0.0;

            double finalAngle = initialPosition.getDoubleValue();
            double finalVelocity = 0.0;

            trajectory.setCubic(0.0, timeToMoveForCalibration.getDoubleValue(), startAngle, startVelocity, finalAngle, finalVelocity);
         }
      }

      @Override
      public void doTransitionOutOfAction()
      {
      }
   }

   private class TrajectoryData
   {
      private final YoDouble initialPosition;
      private final YoDouble calibrationPosition;
      private final YoPolynomial trajectory;

      public TrajectoryData(YoDouble initialPosition, YoDouble calibrationPosition, YoPolynomial trajectory)
      {
         this.initialPosition = initialPosition;
         this.calibrationPosition = calibrationPosition;
         this.trajectory = trajectory;
      }

      public YoDouble getInitialPosition()
      {
         return initialPosition;
      }

      public YoDouble getCalibrationPosition()
      {
         return calibrationPosition;
      }

      public YoPolynomial getTrajectory()
      {
         return trajectory;
      }
   }
}
