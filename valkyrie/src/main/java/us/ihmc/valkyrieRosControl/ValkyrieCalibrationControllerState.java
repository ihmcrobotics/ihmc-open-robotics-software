package us.ihmc.valkyrieRosControl;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelController;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.GenericStateMachine;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.LowLevelOneDoFJointDesiredDataHolderReadOnly;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.ForceSensorCalibrationModule;
import us.ihmc.tools.lists.PairList;
import us.ihmc.valkyrie.ValkyrieCalibrationParameters;
import us.ihmc.wholeBodyController.diagnostics.CalibrationState;
import us.ihmc.wholeBodyController.diagnostics.JointTorqueOffsetEstimatorController;
import us.ihmc.wholeBodyController.diagnostics.TorqueOffsetPrinter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ValkyrieCalibrationControllerState extends HighLevelControllerState
{
   private enum CalibrationStates
   {
      ENTRY, CALIBRATE, EXIT
   }

   private static final double timeToMove = 3.0;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final HighLevelHumanoidControllerToolbox highLevelControllerToolbox;

   private ForceSensorCalibrationModule forceSensorCalibrationModule;
   private final GenericStateMachine<CalibrationStates, CalibrationState<CalibrationStates>> stateMachine;

   private final PairList<OneDoFJoint, TrajectoryData> jointsData = new PairList<>();

   private final JointTorqueOffsetEstimatorController jointTorqueOffsetEstimatorController;
   private final YoDouble timeToMoveForCalibration = new YoDouble("timeToMoveForCalibration", registry);

   public ValkyrieCalibrationControllerState(HighLevelHumanoidControllerToolbox highLevelControllerToolbox,
                                             HighLevelControllerParameters highLevelControllerParameters, ValkyrieCalibrationParameters calibrationParameters,
                                             TorqueOffsetPrinter torqueOffsetPrinter)
   {
      super(HighLevelController.CALIBRATION);

      this.highLevelControllerToolbox = highLevelControllerToolbox;

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

      jointTorqueOffsetEstimatorController = new JointTorqueOffsetEstimatorController(highLevelControllerToolbox, highLevelControllerParameters, torqueOffsetPrinter);
      registry.addChild(jointTorqueOffsetEstimatorController.getYoVariableRegistry());

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
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public void warmup(int iterations)
   {
   }

   @Override
   public LowLevelOneDoFJointDesiredDataHolderReadOnly getOutputForLowLevelController()
   {
      return stateMachine.getCurrentState().getOutputForLowLevelController();
   }

   public JointTorqueOffsetEstimatorController getJointTorqueOffsetEstimatorController()
   {
      return jointTorqueOffsetEstimatorController;
   }

   private class CalibrationEntry extends CalibrationState<CalibrationStates>
   {
      private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

      public CalibrationEntry()
      {
         super(CalibrationStates.ENTRY);

         OneDoFJoint[] jointArray = highLevelControllerToolbox.getFullRobotModel().getOneDoFJoints();
         lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(jointArray);
         lowLevelOneDoFJointDesiredDataHolder.setJointsControlMode(jointArray, JointDesiredControlMode.EFFORT);
      }

      @Override
      public LowLevelOneDoFJointDesiredDataHolderReadOnly getOutputForLowLevelController()
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

         for (int jointIndex = 0; jointIndex < jointsData.size(); jointIndex++)
         {
            OneDoFJoint joint = jointsData.get(jointIndex).getLeft();
            TrajectoryData trajectoryData = jointsData.get(jointIndex).getRight();

            YoPolynomial trajectory = trajectoryData.getTrajectory();

            trajectory.compute(timeInTrajectory);
            double desiredPosition = trajectory.getPosition();

            JointDesiredOutput lowLevelJointData = lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(joint);
            lowLevelJointData.setDesiredPosition(desiredPosition);
            lowLevelJointData.setDesiredVelocity(trajectory.getVelocity());
            lowLevelJointData.setDesiredAcceleration(trajectory.getAcceleration());
         }
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

            double currentAngle = joint.getQ();
            double currentVelocity = 0.0;

            double finalAngle = calibrationPosition.getDoubleValue();
            double finalVelocity = 0.0;

            initialPosition.set(currentAngle);

            trajectory.setCubic(0.0, timeToMoveForCalibration.getDoubleValue(), currentAngle, currentVelocity, finalAngle, finalVelocity);
         }
      }

      @Override
      public void doTransitionOutOfAction()
      {
      }
   }

   private class Calibration extends CalibrationState<CalibrationStates>
   {
      private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

      public Calibration()
      {
         super(CalibrationStates.CALIBRATE);

         OneDoFJoint[] jointArray = highLevelControllerToolbox.getFullRobotModel().getOneDoFJoints();
         lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(jointArray);
         lowLevelOneDoFJointDesiredDataHolder.setJointsControlMode(jointArray, JointDesiredControlMode.EFFORT);
      }

      @Override
      public void doAction()
      {
         if (isDone())
            transitionToDefaultNextState();
         
         jointTorqueOffsetEstimatorController.doAction();
      }

      @Override
      public void doTransitionIntoAction()
      {
         jointTorqueOffsetEstimatorController.doTransitionIntoAction();
      }

      @Override
      public void doTransitionOutOfAction()
      {
         jointTorqueOffsetEstimatorController.doTransitionOutOfAction();

         if (forceSensorCalibrationModule != null)
            forceSensorCalibrationModule.requestFootForceSensorCalibrationAtomic();
      }

      @Override
      public boolean isDone()
      {
         return jointTorqueOffsetEstimatorController.isDone();
      }

      @Override
      public LowLevelOneDoFJointDesiredDataHolderReadOnly getOutputForLowLevelController()
      {
         return lowLevelOneDoFJointDesiredDataHolder;
      }

   }

   private class CalibrationExit extends CalibrationState<CalibrationStates>
   {
      private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

      public CalibrationExit()
      {
         super(CalibrationStates.EXIT);

         OneDoFJoint[] jointArray = highLevelControllerToolbox.getFullRobotModel().getOneDoFJoints();
         lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(jointArray);
         lowLevelOneDoFJointDesiredDataHolder.setJointsControlMode(jointArray, JointDesiredControlMode.EFFORT);
      }

      @Override
      public LowLevelOneDoFJointDesiredDataHolderReadOnly getOutputForLowLevelController()
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
            lowLevelJointData.setDesiredPosition(desiredPosition);
            lowLevelJointData.setDesiredVelocity(trajectory.getVelocity());
            lowLevelJointData.setDesiredAcceleration(trajectory.getAcceleration());
         }
      }

      @Override
      public void doTransitionIntoAction()
      {
         for (int i = 0; i < jointsData.size(); i++)
         {
            OneDoFJoint joint = jointsData.get(i).getLeft();
            TrajectoryData trajectoryData = jointsData.get(i).getRight();

            YoDouble initialPosition = trajectoryData.getInitialPosition();
            YoPolynomial trajectory = trajectoryData.getTrajectory();

            double startAngle = joint.getQ();
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
