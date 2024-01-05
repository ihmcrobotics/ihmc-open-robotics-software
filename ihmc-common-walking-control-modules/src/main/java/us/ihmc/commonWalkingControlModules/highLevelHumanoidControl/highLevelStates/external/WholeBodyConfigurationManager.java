package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.external;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.CrocoddylControlCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.CrocoddylSolverTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.CrocoddylStateCommand;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class WholeBodyConfigurationManager
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoDouble timeAtWholeBodyCommand = new YoDouble("timeAtWholeBodyCommand", registry);

   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   private final OneDoFJointReadOnly[] controlledJoints;
   private final FullRobotModel fullRobotModel;

   private final YoDouble[] currentFeedForwardTorque;
   private final YoDouble[] currentFeedBackTorque;
   private final YoDouble[] desiredTotalTorque;
   private final MultipleWaypointsTrajectoryGenerator[] jointTrajectories;
   private final MultipleWaypointsPoseTrajectoryGenerator basePoseTrajectory;

   private final RecyclingArrayList<DMatrixRMaj> feedForwardTorqueValues = new RecyclingArrayList<>(() -> new DMatrixRMaj(0, 0));
   private final RecyclingArrayList<DMatrixRMaj> feedbackGainMatrices = new RecyclingArrayList<>(() -> new DMatrixRMaj(0, 0));

   private final PoseReferenceFrame desiredPelvisFrame = new PoseReferenceFrame("DesiredPelvisFrame", ReferenceFrame.getWorldFrame());
   private final PoseReferenceFrame tempDesiredPelvisFrame = new PoseReferenceFrame("tempDesiredPelvisFrame", ReferenceFrame.getWorldFrame());
   private final FramePose3D desiredPelvisPose = new FramePose3D(ReferenceFrame.getWorldFrame());
   private final FrameVector3D desiredAngularRate = new FrameVector3D(ReferenceFrame.getWorldFrame());
   private final FrameVector3D desiredLinearRate = new FrameVector3D(ReferenceFrame.getWorldFrame());
   private final Twist pelvisTwist;

   private final FramePose3D currentPelvisPose = new FramePose3D();
   private final YoFrameVector3D rootTranslationError = new YoFrameVector3D("rootTranslationError", desiredPelvisFrame, registry);
   private final YoFrameVector3D rootOrientationError = new YoFrameVector3D("rootOrientationError", desiredPelvisFrame, registry);
   private final YoFrameVector3D rootLinearRateError = new YoFrameVector3D("rootLinearRateError", desiredPelvisFrame, registry);
   private final YoFrameVector3D rootAngularRateError = new YoFrameVector3D("rootAngularRateError", desiredPelvisFrame, registry);
   private final YoDouble[] jointPositionError;
   private final YoDouble[] jointRateError;

   private final FrameVector3D tempVector = new FrameVector3D();
   private final FrameVector3D rootLinearVelocity = new FrameVector3D();
   private final FrameVector3D rootAngularVelocity = new FrameVector3D();

   private final DoubleProvider time;

   private final DMatrixRMaj stateErrorVector = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj feedbackTorqueVector = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj feedForwardTorqueVector = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj totalTorqueVector = new DMatrixRMaj(0, 0);

   public WholeBodyConfigurationManager(DoubleProvider time,
                                        FullHumanoidRobotModel fullRobotModel,
                                        OneDoFJointReadOnly[] controlledJoints,
                                        YoRegistry parentRegistry)
   {
      this.time = time;
      this.controlledJoints = controlledJoints;
      this.fullRobotModel = fullRobotModel;

      pelvisTwist = new Twist(fullRobotModel.getRootBody().getBodyFixedFrame(), ReferenceFrame.getWorldFrame(), tempDesiredPelvisFrame);
      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledJoints);

      jointTrajectories = new MultipleWaypointsTrajectoryGenerator[controlledJoints.length];
      currentFeedBackTorque = new YoDouble[controlledJoints.length];
      currentFeedForwardTorque = new YoDouble[controlledJoints.length];
      desiredTotalTorque = new YoDouble[controlledJoints.length];
      jointPositionError = new YoDouble[controlledJoints.length];
      jointRateError = new YoDouble[controlledJoints.length];
      for (int i = 0; i < controlledJoints.length; i++)
      {
         OneDoFJointReadOnly joint = controlledJoints[i];
         jointTrajectories[i] = new MultipleWaypointsTrajectoryGenerator(joint.getName(), registry);
         currentFeedForwardTorque[i] = new YoDouble(joint.getName() + "FeedForwardTorque", registry);
         currentFeedBackTorque[i] = new YoDouble(joint.getName() + "FeedbackTorque", registry);
         desiredTotalTorque[i] = new YoDouble(joint.getName() + "DesiredTotalTorque", registry);

         jointPositionError[i] = new YoDouble(joint.getName() + "PositionError", registry);
         jointRateError[i] = new YoDouble(joint.getName() + "RateError", registry);
      }
      basePoseTrajectory = new MultipleWaypointsPoseTrajectoryGenerator("BaseTrajectory", 20, registry);
      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      timeAtWholeBodyCommand.set(time.getValue());
      for (int i = 0; i < jointTrajectories.length; i++)
      {
         jointTrajectories[i].clear();
         jointTrajectories[i].appendWaypointWithZeroAcceleration(0.0, controlledJoints[i].getQ(), 0.0);
         jointTrajectories[i].initialize();
      }

      desiredPelvisPose.setToZero(fullRobotModel.getRootBody().getBodyFixedFrame());
      desiredPelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
      desiredAngularRate.setToZero(ReferenceFrame.getWorldFrame());
      desiredLinearRate.setToZero(ReferenceFrame.getWorldFrame());

      basePoseTrajectory.clear(ReferenceFrame.getWorldFrame());
      basePoseTrajectory.appendPoseWaypoint(0.0, desiredPelvisPose, desiredLinearRate, desiredAngularRate);
      basePoseTrajectory.initialize();

      feedForwardTorqueValues.clear();
      feedbackGainMatrices.clear();

      DMatrixRMaj feedforwardTorques = feedForwardTorqueValues.add();
      feedforwardTorques.reshape(controlledJoints.length, 1);
      feedforwardTorques.zero();

      DMatrixRMaj gainMatrix = feedbackGainMatrices.add();
      gainMatrix.reshape(controlledJoints.length, 2 * (controlledJoints.length + 6));
      gainMatrix.zero();
   }

   public void doControl()
   {
      double currentTime = time.getValue() - timeAtWholeBodyCommand.getDoubleValue();

      for (MultipleWaypointsTrajectoryGenerator jointTrajectory : jointTrajectories)
         jointTrajectory.compute(currentTime);
      basePoseTrajectory.compute(currentTime);

      int currentIndex = basePoseTrajectory.getCurrentPositionWaypointIndex();
      populateStateErrorVector();
      populateFeedForwardTorqueVector(currentIndex);

      // make sure everything is sized properly
      feedbackTorqueVector.reshape(controlledJoints.length, 1);
      feedForwardTorqueVector.reshape(controlledJoints.length, 1);
      totalTorqueVector.reshape(controlledJoints.length, 1);

      CommonOps_DDRM.mult(feedbackGainMatrices.get(currentIndex), stateErrorVector, feedbackTorqueVector);
      CommonOps_DDRM.add(feedForwardTorqueVector, feedbackTorqueVector, totalTorqueVector);

      // populate the yo variables
      for (int i = 0; i < totalTorqueVector.getNumRows(); i++)
      {
         currentFeedForwardTorque[i].set(feedForwardTorqueVector.get(i, 0));
         currentFeedBackTorque[i].set(feedbackTorqueVector.get(i, 0));
         desiredTotalTorque[i].set(totalTorqueVector.get(i, 0));
      }
   }

   public JointDesiredOutputListReadOnly getControlOutput()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }

   private void populateStateErrorVector()
   {
      desiredPelvisFrame.setPoseAndUpdate(basePoseTrajectory.getPose());

      MovingReferenceFrame rootFrame = fullRobotModel.getRootBody().getBodyFixedFrame();

      currentPelvisPose.setToZero(rootFrame);
      currentPelvisPose.changeFrame(desiredPelvisFrame);

      rootTranslationError.set(currentPelvisPose.getTranslation());
      currentPelvisPose.getRotation().getRotationVector(rootOrientationError);

      tempVector.setReferenceFrame(ReferenceFrame.getWorldFrame());
      rootLinearVelocity.setIncludingFrame(rootFrame.getTwistOfFrame().getLinearPart());
      rootLinearVelocity.changeFrame(ReferenceFrame.getWorldFrame());
      tempVector.sub(basePoseTrajectory.getVelocity(), rootLinearVelocity);
      rootLinearRateError.setMatchingFrame(tempVector);

      rootLinearVelocity.setIncludingFrame(rootFrame.getTwistOfFrame().getAngularPart());
      rootLinearVelocity.changeFrame(ReferenceFrame.getWorldFrame());
      tempVector.sub(basePoseTrajectory.getAngularVelocity(), rootAngularVelocity);
      rootAngularRateError.setMatchingFrame(tempVector);

      // resize the vector
      stateErrorVector.reshape(2 * (controlledJoints.length + 6), 1);
      stateErrorVector.zero();

      rootTranslationError.get(stateErrorVector);
      rootOrientationError.get(3, stateErrorVector);
      rootLinearRateError.get(6 + controlledJoints.length, stateErrorVector);
      rootAngularRateError.get(9 + controlledJoints.length, stateErrorVector);

      for (int i = 0; i < controlledJoints.length; i++)
      {
         OneDoFJointReadOnly joint = controlledJoints[i];
         jointPositionError[i].set(jointTrajectories[i].getValue() - joint.getQ());
         jointRateError[i].set(jointTrajectories[i].getVelocity() - joint.getQd());

         stateErrorVector.set(i + 6, 0, jointPositionError[i].getDoubleValue());
         stateErrorVector.set(i + 12 + controlledJoints.length, 0, jointRateError[i].getDoubleValue());
      }
   }

   private void populateFeedForwardTorqueVector(int currentIndex)
   {
      feedForwardTorqueVector.set(feedForwardTorqueValues.get(currentIndex));
   }

   public void handleCrocoddylSolverTrajectoryCommand(CrocoddylSolverTrajectoryCommand command)
   {
      timeAtWholeBodyCommand.set(time.getValue());

      for (int jointIndex = 0; jointIndex < jointTrajectories.length; jointIndex++)
         jointTrajectories[jointIndex].clear();
      basePoseTrajectory.clear(ReferenceFrame.getWorldFrame());
      feedbackGainMatrices.clear();
      feedForwardTorqueValues.clear();

      double currentTime = 0.0;
      for (int knot = 0; knot < command.getNumberOfKnots(); knot++)
      {
         double duration = command.getTimeInterval(knot).getDuration();
         CrocoddylStateCommand state = command.getState(knot);
         DMatrixRMaj jointPositions = state.getJointPositions();
         DMatrixRMaj jointVelocities = state.getJointVelocities();

         for (int jointIndex = 0; jointIndex < jointTrajectories.length; jointIndex++)
            jointTrajectories[jointIndex].appendWaypointWithZeroAcceleration(currentTime, jointPositions.get(jointIndex, 0), jointVelocities.get(jointIndex, 0));

         tempDesiredPelvisFrame.setPoseAndUpdate(state.getBasePoseInWorld());
         pelvisTwist.setToZero(tempDesiredPelvisFrame);
         pelvisTwist.getLinearPart().set(state.getBaseLinearRateInBaseFrame());
         pelvisTwist.getAngularPart().set(state.getBaseAngularRateInBaseFrame());
         pelvisTwist.changeFrame(ReferenceFrame.getWorldFrame());


         desiredPelvisPose.set(state.getBasePoseInWorld());
         desiredAngularRate.setIncludingFrame(pelvisTwist.getAngularPart());
         desiredLinearRate.setIncludingFrame(pelvisTwist.getLinearPart());

         basePoseTrajectory.appendPoseWaypoint(currentTime, desiredPelvisPose, desiredLinearRate, desiredAngularRate);

         CrocoddylControlCommand control = command.getControl(knot);
         feedForwardTorqueValues.add().set(control.getControl());
         feedbackGainMatrices.add().set(control.getFeedbackGainCommand().getGainMatrix());

         currentTime += duration;
      }

      for (MultipleWaypointsTrajectoryGenerator jointTrajectory : jointTrajectories)
         jointTrajectory.initialize();
      basePoseTrajectory.initialize();
   }
}
