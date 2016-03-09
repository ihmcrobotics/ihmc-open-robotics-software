package us.ihmc.commonWalkingControlModules.controlModules.head;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.HeadOrientationControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.HeadTrajectoryControllerCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.InverseDynamicsCommand;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

public class HeadOrientationManager
{
   private static final double defaultTrajectoryTime = 1.0;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable receivedNewHeadOrientationTime = new DoubleYoVariable("receivedNewHeadOrientationTime", registry);
   private final MultipleWaypointsOrientationTrajectoryGenerator waypointOrientationTrajectoryGenerator;

   private final BooleanYoVariable isTrackingOrientation = new BooleanYoVariable("isTrackingOrientation", registry);

   private final ReferenceFrame headFrame;
   private final ReferenceFrame chestFrame;

   private final BooleanYoVariable hasBeenInitialized = new BooleanYoVariable("hasHeadOrientationManagerBeenInitialized", registry);

   private final FrameOrientation homeOrientation = new FrameOrientation();
   private final FrameOrientation desiredOrientation = new FrameOrientation();
   private final FrameVector desiredAngularVelocity = new FrameVector();
   private final FrameVector feedForwardAngularAcceleration = new FrameVector();

   private final OrientationFeedbackControlCommand orientationFeedbackControlCommand = new OrientationFeedbackControlCommand();

   public HeadOrientationManager(MomentumBasedController momentumBasedController, HeadOrientationControllerParameters headOrientationControllerParameters,
         YoOrientationPIDGainsInterface gains, double[] initialDesiredHeadYawPitchRoll, YoVariableRegistry parentRegistry)
   {
      yoTime = momentumBasedController.getYoTime();
      FullHumanoidRobotModel fullRobotModel = momentumBasedController.getFullRobotModel();

      RigidBody head = fullRobotModel.getHead();
      RigidBody chest = fullRobotModel.getChest();
      RigidBody elevator = fullRobotModel.getElevator();
      orientationFeedbackControlCommand.setWeightForSolver(10.0);
      orientationFeedbackControlCommand.set(elevator, head);
      orientationFeedbackControlCommand.setGains(gains);
      chestFrame = chest.getBodyFixedFrame();
      headFrame = head.getBodyFixedFrame();

      parentRegistry.addChild(registry);

      hasBeenInitialized.set(false);

      homeOrientation.setIncludingFrame(chestFrame, initialDesiredHeadYawPitchRoll);
      waypointOrientationTrajectoryGenerator = new MultipleWaypointsOrientationTrajectoryGenerator("head", true, chestFrame, registry);
      waypointOrientationTrajectoryGenerator.registerNewTrajectoryFrame(worldFrame);
   }

   public void initialize()
   {
      if (hasBeenInitialized.getBooleanValue())
         return;

      hasBeenInitialized.set(true);

      desiredOrientation.setToZero(headFrame);
      goToHome(defaultTrajectoryTime, desiredOrientation);
   }

   public void goToHomeFromCurrentDesired(double trajectoryTime)
   {
      waypointOrientationTrajectoryGenerator.getOrientation(desiredOrientation);
      goToHome(trajectoryTime, desiredOrientation);
   }

   public void goToHome(double trajectoryTime, FrameOrientation initialOrientation)
   {
      initialOrientation.changeFrame(chestFrame);
      desiredAngularVelocity.setToZero(chestFrame);

      waypointOrientationTrajectoryGenerator.switchTrajectoryFrame(chestFrame);
      waypointOrientationTrajectoryGenerator.appendWaypoint(0.0, initialOrientation, desiredAngularVelocity);
      waypointOrientationTrajectoryGenerator.appendWaypoint(trajectoryTime, homeOrientation, desiredAngularVelocity);
      waypointOrientationTrajectoryGenerator.initialize();

      receivedNewHeadOrientationTime.set(yoTime.getDoubleValue());
      isTrackingOrientation.set(true);
   }

   public void compute()
   {
      if (isTrackingOrientation.getBooleanValue())
      {
         double deltaTime = yoTime.getDoubleValue() - receivedNewHeadOrientationTime.getDoubleValue();
         waypointOrientationTrajectoryGenerator.compute(deltaTime);
         isTrackingOrientation.set(!waypointOrientationTrajectoryGenerator.isDone());
      }
      waypointOrientationTrajectoryGenerator.getOrientation(desiredOrientation);
      desiredAngularVelocity.setToZero(worldFrame);
      feedForwardAngularAcceleration.setToZero(worldFrame);
      orientationFeedbackControlCommand.changeFrameAndSet(desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);
   }

   public void handleHeadTrajectoryMessage(HeadTrajectoryControllerCommand message)
   {
      if (message == null)
         return;
      receivedNewHeadOrientationTime.set(yoTime.getDoubleValue());

      if (message.getTrajectoryPoint(0).getTime() > 1.0e-5)
      {
         waypointOrientationTrajectoryGenerator.getOrientation(desiredOrientation);
         desiredOrientation.changeFrame(worldFrame);
         desiredAngularVelocity.setToZero(worldFrame);

         waypointOrientationTrajectoryGenerator.switchTrajectoryFrame(worldFrame);
         waypointOrientationTrajectoryGenerator.clear();
         waypointOrientationTrajectoryGenerator.appendWaypoint(0.0, desiredOrientation, desiredAngularVelocity);
      }
      else
      {
         waypointOrientationTrajectoryGenerator.switchTrajectoryFrame(worldFrame);
         waypointOrientationTrajectoryGenerator.clear();
      }

      waypointOrientationTrajectoryGenerator.appendWaypoints(message);
      waypointOrientationTrajectoryGenerator.changeFrame(chestFrame);
      waypointOrientationTrajectoryGenerator.initialize();
      isTrackingOrientation.set(true);
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return null;
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return orientationFeedbackControlCommand;
   }
}
