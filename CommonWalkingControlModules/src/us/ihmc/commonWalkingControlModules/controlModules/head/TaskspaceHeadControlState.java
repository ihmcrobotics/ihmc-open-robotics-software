package us.ihmc.commonWalkingControlModules.controlModules.head;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HeadTrajectoryCommand;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

public class TaskspaceHeadControlState extends HeadControlState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final MultipleWaypointsOrientationTrajectoryGenerator trajectoryGenerator;

   private final DoubleYoVariable weight = new DoubleYoVariable("headTaskspaceWeight", registry);
   private final YoFrameQuaternion homeOrientation;
   private final YoOrientationPIDGainsInterface gains;
   private final OrientationFeedbackControlCommand feedbackControlCommand = new OrientationFeedbackControlCommand();

   private final FrameOrientation desiredOrientation = new FrameOrientation();
   private final FrameVector desiredAngularVelocity = new FrameVector();
   private final FrameVector feedForwardAngularAcceleration = new FrameVector();

   private final ReferenceFrame headFrame;
   private final ReferenceFrame chestFrame;

   public TaskspaceHeadControlState(RigidBody chest, RigidBody head, double[] homeYawPitchRoll, YoOrientationPIDGainsInterface gains, YoVariableRegistry parentRegistry)
   {
      super(HeadControlMode.TASKSPACE);

      this.gains = gains;
      feedbackControlCommand.set(chest, head);

      headFrame = head.getBodyFixedFrame();
      chestFrame = chest.getBodyFixedFrame();

      trajectoryGenerator = new MultipleWaypointsOrientationTrajectoryGenerator("head", true, worldFrame, registry);
      trajectoryGenerator.registerNewTrajectoryFrame(chestFrame);

      homeOrientation = new YoFrameQuaternion("headHomeOrientation", chestFrame, registry);
      homeOrientation.set(homeYawPitchRoll);

      parentRegistry.addChild(registry);
   }

   public void holdCurrentOrientation()
   {
      desiredOrientation.setToZero(headFrame);
      desiredOrientation.changeFrame(chestFrame);
      desiredAngularVelocity.setToZero(chestFrame);

      trajectoryGenerator.clear();
      trajectoryGenerator.switchTrajectoryFrame(chestFrame);
      trajectoryGenerator.appendWaypoint(0.0, desiredOrientation, desiredAngularVelocity);
   }

   public void handleGoHome(double trajectoryTime, FrameOrientation initialOrientation)
   {
      initialOrientation.changeFrame(chestFrame);
      homeOrientation.getFrameOrientationIncludingFrame(desiredOrientation);
      desiredAngularVelocity.setToZero(chestFrame);

      trajectoryGenerator.clear();
      trajectoryGenerator.switchTrajectoryFrame(chestFrame);
      trajectoryGenerator.appendWaypoint(0.0, initialOrientation, desiredAngularVelocity);
      trajectoryGenerator.appendWaypoint(trajectoryTime, desiredOrientation, desiredAngularVelocity);
   }

   public void handleHeadTrajectoryCommand(HeadTrajectoryCommand command, FrameOrientation initialOrientation)
   {
      if (command.getTrajectoryPoint(0).getTime() > 1.0e-5)
      {
         initialOrientation.changeFrame(worldFrame);
         desiredAngularVelocity.setToZero(worldFrame);

         trajectoryGenerator.switchTrajectoryFrame(worldFrame);
         trajectoryGenerator.clear();
         trajectoryGenerator.appendWaypoint(0.0, initialOrientation, desiredAngularVelocity);
      }
      else
      {
         trajectoryGenerator.switchTrajectoryFrame(worldFrame);
         trajectoryGenerator.clear();
      }

      trajectoryGenerator.appendWaypoints(command.getTrajectoryPointList());
      trajectoryGenerator.changeFrame(chestFrame);
   }

   @Override
   public void setWeight(double weight)
   {
      this.weight.set(weight);
   }

   @Override
   public void doTransitionIntoAction()
   {
      trajectoryGenerator.initialize();
   }

   @Override
   public void doAction()
   {
      trajectoryGenerator.compute(getTimeInCurrentState());
      trajectoryGenerator.getAngularData(desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);

      feedbackControlCommand.changeFrameAndSet(desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);
      feedbackControlCommand.setGains(gains);
      feedbackControlCommand.setWeightForSolver(weight.getDoubleValue());
   }

   @Override
   public void doTransitionOutOfAction()
   {

   }

   public void getDesiredOrientation(FrameOrientation desiredOrientationToPack)
   {
      trajectoryGenerator.getOrientation(desiredOrientationToPack);
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return null;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return feedbackControlCommand;
   }
}
