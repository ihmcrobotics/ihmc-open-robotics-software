package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controllerCore.command.SolverWeightLevels;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.JointspaceFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointspaceAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelJointControlMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolderReadOnly;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlMode;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.TaskspaceToJointspaceCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandComplianceControlParametersCommand;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.trajectories.PoseTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;
import us.ihmc.tools.FormattingTools;

public class TaskspaceToJointspaceHandPositionControlState extends FinishableState<HandControlMode>
{
   private final String name;
   private final YoVariableRegistry registry;

   private final DenseMatrix64F selectionMatrix = new DenseMatrix64F(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private PoseTrajectoryGenerator poseTrajectoryGenerator;

   private final FramePose desiredPose = new FramePose();
   private final FramePoint desiredPosition = new FramePoint(worldFrame);
   private final FrameVector desiredVelocity = new FrameVector(worldFrame);
   private final FrameVector desiredAcceleration = new FrameVector(worldFrame);

   private final FrameOrientation desiredOrientation = new FrameOrientation(worldFrame);
   private final FrameVector desiredAngularVelocity = new FrameVector(worldFrame);
   private final FrameVector desiredAngularAcceleration = new FrameVector(worldFrame);

   private TaskspaceToJointspaceCalculator taskspaceToJointspaceCalculator;
   private final JointspaceAccelerationCommand jointspaceAccelerationCommand;
   private final JointspaceFeedbackControlCommand jointspaceFeedbackControlCommand;
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelJointDesiredData;

   private final DoubleYoVariable percentOfTrajectoryWithOrientationBeingControlled;
   private final DoubleYoVariable startTimeInStateToIgnoreOrientation;
   private final DoubleYoVariable endTimeInStateToIgnoreOrientation;
   private final DoubleYoVariable currentOrientationControlFactor;
   private final DoubleYoVariable activeTrajectoryTime;

   private final DoubleYoVariable doneTrajectoryTime;
   private final DoubleYoVariable holdPositionDuration;

   private final BooleanYoVariable enableCompliantControl;
   private final HandCompliantControlHelper handCompliantControlHelper;

   private final boolean doPositionControl;

   private final DenseMatrix64F identityScaledWithOrientationControlFactor = CommonOps.identity(SpatialMotionVector.SIZE);
   private final DenseMatrix64F selectionMatrixWithReducedAngularControl = CommonOps.identity(SpatialMotionVector.SIZE);

   private final OneDoFJoint[] oneDoFJoints;

   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable initialTime;
   private final DoubleYoVariable currentTimeInState;

   public static TaskspaceToJointspaceHandPositionControlState createControlStateForForceControlledJoints(String namePrefix, RobotSide robotSide,
         HighLevelHumanoidControllerToolbox momentumBasedController, RigidBody base, RigidBody endEffector, YoPIDGains gains, YoVariableRegistry parentRegistry)
   {
      return new TaskspaceToJointspaceHandPositionControlState(namePrefix, robotSide, momentumBasedController, base, endEffector, false, gains, parentRegistry);
   }

   public static TaskspaceToJointspaceHandPositionControlState createControlStateForPositionControlledJoints(String namePrefix, RobotSide robotSide,
         HighLevelHumanoidControllerToolbox momentumBasedController, RigidBody base, RigidBody endEffector, YoVariableRegistry parentRegistry)
   {
      return new TaskspaceToJointspaceHandPositionControlState(namePrefix, robotSide, momentumBasedController, base, endEffector, true, null, parentRegistry);
   }

   private TaskspaceToJointspaceHandPositionControlState(String namePrefix, RobotSide robotSide, HighLevelHumanoidControllerToolbox momentumBasedController,
         RigidBody base, RigidBody endEffector, boolean doPositionControl, YoPIDGains gains, YoVariableRegistry parentRegistry)
   {
      super(HandControlMode.TASKSPACE);

      name = namePrefix + FormattingTools.underscoredToCamelCase(this.getStateEnum().toString(), true) + "State";
      registry = new YoVariableRegistry(name);

      parentRegistry.addChild(registry);

      yoTime = momentumBasedController.getYoTime();

      initialTime = new DoubleYoVariable(namePrefix + "InitialTime", registry);
      currentTimeInState = new DoubleYoVariable(namePrefix + "CurrentTimeInState", registry);

      doneTrajectoryTime = new DoubleYoVariable(namePrefix + "DoneTrajectoryTime", registry);
      holdPositionDuration = new DoubleYoVariable(namePrefix + "HoldPositionDuration", registry);

      this.doPositionControl = doPositionControl;

      percentOfTrajectoryWithOrientationBeingControlled = new DoubleYoVariable(namePrefix + "PercentOfTrajectoryWithOrientationBeingControlled", registry);
      percentOfTrajectoryWithOrientationBeingControlled.set(Double.NaN);

      activeTrajectoryTime = new DoubleYoVariable(namePrefix + "ActiveTrajectoryTime", registry);
      activeTrajectoryTime.set(Double.NaN);

      startTimeInStateToIgnoreOrientation = new DoubleYoVariable(namePrefix + "StartTimeInStateToIgnoreOrientation", registry);
      startTimeInStateToIgnoreOrientation.set(Double.NaN);

      endTimeInStateToIgnoreOrientation = new DoubleYoVariable(namePrefix + "EndTimeInStateToIgnoreOrientation", registry);
      endTimeInStateToIgnoreOrientation.set(Double.NaN);

      currentOrientationControlFactor = new DoubleYoVariable(namePrefix + "CurrentOrientationControlFactor", registry);
      currentOrientationControlFactor.set(Double.NaN);

      oneDoFJoints = ScrewTools.createOneDoFJointPath(base, endEffector);

      if (doPositionControl)
      {
         lowLevelJointDesiredData = new LowLevelOneDoFJointDesiredDataHolder(oneDoFJoints.length);
         lowLevelJointDesiredData.registerJointsWithEmptyData(oneDoFJoints);
         lowLevelJointDesiredData.setJointsControlMode(oneDoFJoints, LowLevelJointControlMode.POSITION_CONTROL);

         jointspaceAccelerationCommand = new JointspaceAccelerationCommand();
         jointspaceFeedbackControlCommand = null;

         for (int i = 0; i < oneDoFJoints.length; i++)
         {
            OneDoFJoint joint = oneDoFJoints[i];
            jointspaceAccelerationCommand.addJoint(joint, Double.NaN);
         }
      }
      else // Force control at the joints
      {
         lowLevelJointDesiredData = null;

         jointspaceAccelerationCommand = null;
         jointspaceFeedbackControlCommand = new JointspaceFeedbackControlCommand();
         jointspaceFeedbackControlCommand.setGains(gains);
         jointspaceFeedbackControlCommand.setWeightForSolver(SolverWeightLevels.ARM_JOINTSPACE_WEIGHT);

         for (int i = 0; i < oneDoFJoints.length; i++)
         {
            OneDoFJoint joint = oneDoFJoints[i];
            jointspaceFeedbackControlCommand.addJoint(joint, Double.NaN, Double.NaN, Double.NaN);
         }
      }

      enableCompliantControl = new BooleanYoVariable(namePrefix + "EnableCompliantControl", registry);
      if (momentumBasedController.getWristForceSensor(robotSide) != null)
      {
         handCompliantControlHelper = new HandCompliantControlHelper(namePrefix, robotSide, momentumBasedController, registry);
      }
      else
      {
         handCompliantControlHelper = null;
      }
   }

   public void setWeight(double weight)
   {
      if (jointspaceFeedbackControlCommand != null)
         jointspaceFeedbackControlCommand.setWeightForSolver(weight);
   }

   /** Either {@link #initializeWithCurrentJointAngles()} or {@link #initializeWithDesiredJointAngles()} needs to be called before {@link #doAction()} when this class is used as a standalone controller. */
   public void initializeWithDesiredJointAngles()
   {
      taskspaceToJointspaceCalculator.initializeFromDesiredJointAngles();
      doTransitionIntoAction();
   }

   /** Either {@link #initializeWithCurrentJointAngles()} or {@link #initializeWithDesiredJointAngles()} needs to be called before {@link #doAction()} when this class is used as a standalone controller. */
   public void initializeWithCurrentJointAngles()
   {
      taskspaceToJointspaceCalculator.initializeFromCurrentJointAngles();
      doTransitionIntoAction();
   }

   @Override
   public void doAction()
   {
      currentTimeInState.set(getTimeInCurrentState());

      if (poseTrajectoryGenerator.isDone())
         recordDoneTrajectoryTime();

      poseTrajectoryGenerator.compute(currentTimeInState.getDoubleValue());

      poseTrajectoryGenerator.getPose(desiredPose);
      poseTrajectoryGenerator.getLinearData(desiredPosition, desiredVelocity, desiredAcceleration);
      poseTrajectoryGenerator.getAngularData(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);

      ReferenceFrame controlFrame = taskspaceToJointspaceCalculator.getControlFrame();
      desiredVelocity.changeFrame(controlFrame);
      desiredAngularVelocity.changeFrame(controlFrame);

      decayAngularControl(currentTimeInState.getDoubleValue());

      if (enableCompliantControl.getBooleanValue())
         handCompliantControlHelper.doCompliantControl(desiredPosition, desiredOrientation);
      else
         handCompliantControlHelper.progressivelyCancelOutCorrection(desiredPosition, desiredOrientation);

      taskspaceToJointspaceCalculator.compute(desiredPosition, desiredOrientation, desiredVelocity, desiredAngularVelocity);
      taskspaceToJointspaceCalculator.getDesiredJointAnglesIntoOneDoFJoints(oneDoFJoints);
      taskspaceToJointspaceCalculator.getDesiredJointVelocitiesIntoOneDoFJoints(oneDoFJoints);
      taskspaceToJointspaceCalculator.getDesiredJointAccelerationsIntoOneDoFJoints(oneDoFJoints);

      if (doPositionControl)
      {
         enablePositionControl();
         for (int i = 0; i < oneDoFJoints.length; i++)
         {
            OneDoFJoint joint = oneDoFJoints[i];
            // The feed forward is really messy when controlling w.r.t. world frame, it's better to leave it to zero for now.
            double feedForwardAcceleration = 0.0;
            jointspaceAccelerationCommand.setOneDoFJointDesiredAcceleration(i, feedForwardAcceleration);
            lowLevelJointDesiredData.setDesiredJointPosition(joint, joint.getqDesired());
            lowLevelJointDesiredData.setDesiredJointVelocity(joint, joint.getQdDesired());
         }
      }
      else
      {
         for (int i = 0; i < oneDoFJoints.length; i++)
         {
            OneDoFJoint joint = oneDoFJoints[i];

            double qDesired = joint.getqDesired();
            double qdDesired = joint.getQdDesired();
            // TODO No feed-forward acceleration yet, needs to be implemented.
            double feedForwardAcceleration = 0.0;

            jointspaceFeedbackControlCommand.setOneDoFJoint(i, qDesired, qdDesired, feedForwardAcceleration);
         }
      }
   }

//   @Override
//   public double getTimeInCurrentState()
//   {
//      return yoTime.getDoubleValue() - initialTime.getDoubleValue();
//   }

   private void decayAngularControl(double time)
   {
      if (activeTrajectoryTime.isNaN() || percentOfTrajectoryWithOrientationBeingControlled.isNaN())
         return;

      if (time < startTimeInStateToIgnoreOrientation.getDoubleValue())
      {
         double alpha = 1.0 - time / startTimeInStateToIgnoreOrientation.getDoubleValue();
         alpha = MathTools.clamp(alpha, 0.0, 1.0);
         alpha *= alpha;
         currentOrientationControlFactor.set(alpha);
         applyAlphaFactorForOrientationControl(alpha);
      }
      else if (time > endTimeInStateToIgnoreOrientation.getDoubleValue())
      {
         double alpha = time - endTimeInStateToIgnoreOrientation.getDoubleValue();
         alpha /= activeTrajectoryTime.getDoubleValue() - endTimeInStateToIgnoreOrientation.getDoubleValue();
         alpha = MathTools.clamp(alpha, 0.0, 1.0);
         alpha *= alpha;
         currentOrientationControlFactor.set(alpha);
         applyAlphaFactorForOrientationControl(alpha);
      }
      else
      {
         setupSelectionMatrixForLinearControlOnly();
      }
   }

   @Override
   public void doTransitionIntoAction()
   {
      initialTime.set(yoTime.getDoubleValue());
      currentTimeInState.set(0.0);

      poseTrajectoryGenerator.showVisualization();
      poseTrajectoryGenerator.initialize();
      doneTrajectoryTime.set(Double.NaN);

      if (doPositionControl)
         enablePositionControl();

      taskspaceToJointspaceCalculator.setSelectionMatrix(selectionMatrix);

      selectionMatrixWithReducedAngularControl.reshape(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
      CommonOps.setIdentity(identityScaledWithOrientationControlFactor);
      CommonOps.setIdentity(selectionMatrixWithReducedAngularControl);

      if (activeTrajectoryTime.isNaN() || percentOfTrajectoryWithOrientationBeingControlled.isNaN())
      {
         startTimeInStateToIgnoreOrientation.set(Double.NaN);
         endTimeInStateToIgnoreOrientation.set(Double.NaN);
         currentOrientationControlFactor.set(Double.NaN);
      }
      else if (MathTools.epsilonEquals(percentOfTrajectoryWithOrientationBeingControlled.getDoubleValue(), 0.0, 1.0e-2))
      {
         activeTrajectoryTime.set(Double.NaN);
         percentOfTrajectoryWithOrientationBeingControlled.set(Double.NaN);
         startTimeInStateToIgnoreOrientation.set(Double.NaN);
         endTimeInStateToIgnoreOrientation.set(Double.NaN);
         currentOrientationControlFactor.set(0.0);
         setupSelectionMatrixForLinearControlOnly();
      }
      else
      {
         double timeToDecayOrientationControl = 0.5 * percentOfTrajectoryWithOrientationBeingControlled.getDoubleValue()
               * activeTrajectoryTime.getDoubleValue();
         startTimeInStateToIgnoreOrientation.set(timeToDecayOrientationControl);
         endTimeInStateToIgnoreOrientation.set(activeTrajectoryTime.getDoubleValue() - timeToDecayOrientationControl);
      }
   }

   private void setupSelectionMatrixForLinearControlOnly()
   {
      applyAlphaFactorForOrientationControl(0.0);
   }

   private void applyAlphaFactorForOrientationControl(double alpha)
   {
      CommonOps.setIdentity(identityScaledWithOrientationControlFactor);
      for (int i = 0; i < 3; i++)
         identityScaledWithOrientationControlFactor.set(i, i, alpha);

      selectionMatrixWithReducedAngularControl.reshape(selectionMatrix.getNumRows(), SpatialMotionVector.SIZE);
      CommonOps.mult(selectionMatrix, identityScaledWithOrientationControlFactor, selectionMatrixWithReducedAngularControl);

      MatrixTools.removeZeroRows(selectionMatrixWithReducedAngularControl, 1.0e-12);

      taskspaceToJointspaceCalculator.setSelectionMatrix(selectionMatrixWithReducedAngularControl);
   }

   @Override
   public void doTransitionOutOfAction()
   {
      holdPositionDuration.set(0.0);

      if (doPositionControl)
         disablePositionControl();

      startTimeInStateToIgnoreOrientation.set(Double.NaN);
      endTimeInStateToIgnoreOrientation.set(Double.NaN);
      currentOrientationControlFactor.set(Double.NaN);
   }

   private void enablePositionControl()
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         joint.setIntegrateDesiredAccelerations(false);
         joint.setUnderPositionControl(true);
      }
   }

   private void disablePositionControl()
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         joint.setUnderPositionControl(false);
      }
   }

   @Override
   public boolean isDone()
   {
      if (Double.isNaN(doneTrajectoryTime.getDoubleValue()))
         return false;

      return getTimeInCurrentState() > doneTrajectoryTime.getDoubleValue() + holdPositionDuration.getDoubleValue();
   }

   private void recordDoneTrajectoryTime()
   {
      if (Double.isNaN(doneTrajectoryTime.getDoubleValue()))
      {
         doneTrajectoryTime.set(getTimeInCurrentState());
      }
   }

   public void setHoldPositionDuration(double holdPositionDuration)
   {
      this.holdPositionDuration.set(holdPositionDuration);
   }

   public void setTrajectory(PoseTrajectoryGenerator poseTrajectoryGenerator)
   {
      setTrajectoryWithAngularControlQuality(poseTrajectoryGenerator, Double.NaN, Double.NaN);
   }

   public void setTrajectoryWithAngularControlQuality(PoseTrajectoryGenerator poseTrajectoryGenerator, double percentOfTrajectoryWithOrientationBeingControlled,
         double trajectoryTime)
   {
      this.poseTrajectoryGenerator = poseTrajectoryGenerator;
      percentOfTrajectoryWithOrientationBeingControlled = MathTools.clamp(percentOfTrajectoryWithOrientationBeingControlled, 0.0, 1.0);
      if (MathTools.epsilonEquals(percentOfTrajectoryWithOrientationBeingControlled, 1.0, 0.01))
      {
         this.percentOfTrajectoryWithOrientationBeingControlled.set(Double.NaN);
         this.activeTrajectoryTime.set(Double.NaN);
      }
      else
      {
         this.percentOfTrajectoryWithOrientationBeingControlled.set(percentOfTrajectoryWithOrientationBeingControlled);
         this.activeTrajectoryTime.set(trajectoryTime);
      }
   }

   public void setControlModuleForPositionControl(TaskspaceToJointspaceCalculator taskspaceToJointspaceCalculator)
   {
      if (handCompliantControlHelper != null)
      {
         this.taskspaceToJointspaceCalculator = taskspaceToJointspaceCalculator;
         handCompliantControlHelper.setCompliantControlFrame(this.taskspaceToJointspaceCalculator.getControlFrame());
      }
   }

   public void setCompliantControlFrame(ReferenceFrame compliantControlFrame)
   {
      handCompliantControlHelper.setCompliantControlFrame(compliantControlFrame);
   }

   public void setControlFrameFixedInEndEffector(ReferenceFrame controlFrame)
   {
   }

   public void getDesiredPose(FramePose desiredPoseToPack)
   {
      desiredPoseToPack.setIncludingFrame(desiredPose);
   }

   public ReferenceFrame getReferenceFrame()
   {
      return desiredPose.getReferenceFrame();
   }

   public ReferenceFrame getTrackingFrame()
   {
      return taskspaceToJointspaceCalculator.getControlFrame();
   }

   public void handleHandComplianceControlParametersMessage(HandComplianceControlParametersCommand message)
   {
      setEnableCompliantControl(message.isEnable());
      handCompliantControlHelper.handleHandComplianceControlParametersMessage(message);
   }

   public void setEnableCompliantControl(boolean enable)
   {
      enableCompliantControl.set(enable);
   }

   public void resetCompliantControl()
   {
      handCompliantControlHelper.reset();
   }

   public JointspaceAccelerationCommand getInverseDynamicsCommand()
   {
      return jointspaceAccelerationCommand;
   }

   public void setSelectionMatrix(DenseMatrix64F selectionMatrix)
   {
      this.selectionMatrix.reshape(selectionMatrix.getNumRows(), selectionMatrix.getNumCols());
      this.selectionMatrix.set(selectionMatrix);
   }

   public JointspaceFeedbackControlCommand getFeedbackControlCommand()
   {
      return jointspaceFeedbackControlCommand;
   }

   public LowLevelOneDoFJointDesiredDataHolderReadOnly getLowLevelJointDesiredData()
   {
      return lowLevelJointDesiredData;
   }
}
