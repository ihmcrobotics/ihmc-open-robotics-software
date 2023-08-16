package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.external;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.CrocoddylControlCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.CrocoddylSolverTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.CrocoddylStateCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class WholeBodyConfigurationManager
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoDouble timeAtWholeBodyCommand = new YoDouble("timeAtWholeBodyCommand", registry);

   private final OneDoFJointReadOnly[] controlledJoints;
   private final YoDouble[] currentFeedForwardTorque;
   private final YoDouble[] currentFeedBackTorque;
   private final YoDouble[] desiredTotalTorque;
   private final MultipleWaypointsTrajectoryGenerator[] jointTrajectories;
   private final MultipleWaypointsPoseTrajectoryGenerator basePoseTrajectory;

   private final RecyclingArrayList<DMatrixRMaj> feedForwardTorqueValues = new RecyclingArrayList<>(DMatrixRMaj::new);
   private final RecyclingArrayList<DMatrixRMaj> feedbackGainMatrix = new RecyclingArrayList<>(DMatrixRMaj::new);

   private final PoseReferenceFrame desiredPelvisFrame = new PoseReferenceFrame("DesiredPelvisFrame", ReferenceFrame.getWorldFrame());
   private final FramePose3D desiredPelvisPose = new FramePose3D(ReferenceFrame.getWorldFrame());
   private final FrameVector3D desiredAngularRate = new FrameVector3D(ReferenceFrame.getWorldFrame());
   private final FrameVector3D desiredLinearRate = new FrameVector3D(ReferenceFrame.getWorldFrame());
   private final Twist pelvisTwist;

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

      pelvisTwist = new Twist(fullRobotModel.getRootBody().getBodyFixedFrame(), ReferenceFrame.getWorldFrame(), desiredPelvisFrame);

      jointTrajectories = new MultipleWaypointsTrajectoryGenerator[controlledJoints.length];
      currentFeedBackTorque = new YoDouble[controlledJoints.length];
      currentFeedForwardTorque = new YoDouble[controlledJoints.length];
      desiredTotalTorque = new YoDouble[controlledJoints.length];
      for (int i = 0; i < controlledJoints.length; i++)
      {
         OneDoFJointReadOnly joint = controlledJoints[i];
         jointTrajectories[i] = new MultipleWaypointsTrajectoryGenerator(joint.getName(), registry);
         currentFeedForwardTorque[i] = new YoDouble(joint.getName() + "FeedForwardTorque", registry);
         currentFeedBackTorque[i] = new YoDouble(joint.getName() + "FeedbackTorque", registry);
         desiredTotalTorque[i] = new YoDouble(joint.getName() + "DesiredTotalTorque", registry);
      }
      basePoseTrajectory = new MultipleWaypointsPoseTrajectoryGenerator("BaseTrajectory", 20, registry);
      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      timeAtWholeBodyCommand.set(Double.NaN);
      for (MultipleWaypointsTrajectoryGenerator jointTrajectory : jointTrajectories)
         jointTrajectory.clear();
      basePoseTrajectory.clear(ReferenceFrame.getWorldFrame());
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
      feedbackTorqueVector.reshape(controlledJoints.length, 0);
      feedForwardTorqueVector.reshape(controlledJoints.length, 0);
      totalTorqueVector.reshape(controlledJoints.length, 0);

      CommonOps_DDRM.mult(feedbackGainMatrix.get(currentIndex), stateErrorVector, feedbackTorqueVector);
      CommonOps_DDRM.add(feedForwardTorqueVector, feedbackTorqueVector, totalTorqueVector);

      // populate the yo variables
      for (int i = 0; i < totalTorqueVector.getNumRows(); i++)
      {
         currentFeedForwardTorque[i].set(feedForwardTorqueVector.get(i, 0));
         currentFeedBackTorque[i].set(feedbackTorqueVector.get(i, 0));
         desiredTotalTorque[i].set(totalTorqueVector.get(i, 0));
      }
   }

   private void populateStateErrorVector()
   {
      throw new RuntimeException("This needs to be done badly before we can do anythign else.");
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
      feedbackGainMatrix.clear();
      feedForwardTorqueValues.clear();

      double currentTime = 0.0;
      for (int knot = 0; knot < command.getNumberOfKnots(); knot++)
      {
         double duration = command.getTimeInterval(knot).getDuration();
         CrocoddylStateCommand state = command.getState(knot);
         DMatrixRMaj jointPositions = state.getJointPositions();
         DMatrixRMaj jointVelocities = state.getJointVelocities();

         for (int jointIndex = 0; jointIndex < jointTrajectories.length; jointIndex++)
            jointTrajectories[jointIndex].appendWaypoint(currentTime, jointPositions.get(jointIndex, 0), jointVelocities.get(jointIndex, 0));

         desiredPelvisFrame.setPoseAndUpdate(state.getBasePoseInWorld());
         pelvisTwist.setToZero(desiredPelvisFrame);
         pelvisTwist.getLinearPart().set(state.getBaseLinearRateInBaseFrame());
         pelvisTwist.getAngularPart().set(state.getBaseAngularRateInBaseFrame());
         pelvisTwist.changeFrame(ReferenceFrame.getWorldFrame());


         desiredPelvisPose.set(state.getBasePoseInWorld());
         desiredAngularRate.setIncludingFrame(pelvisTwist.getAngularPart());
         desiredLinearRate.setIncludingFrame(pelvisTwist.getLinearPart());

         basePoseTrajectory.appendPoseWaypoint(currentTime, desiredPelvisPose, desiredLinearRate, desiredAngularRate);

         CrocoddylControlCommand control = command.getControl(knot);
         feedForwardTorqueValues.add().set(control.getControl());
         feedbackGainMatrix.add().set(control.getFeedbackGainCommand().getGainMatrix());

         currentTime += duration;
      }

      for (MultipleWaypointsTrajectoryGenerator jointTrajectory : jointTrajectories)
         jointTrajectory.initialize();
      basePoseTrajectory.initialize();
   }
}
