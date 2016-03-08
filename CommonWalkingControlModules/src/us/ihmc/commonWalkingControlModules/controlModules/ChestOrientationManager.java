package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ChestTrajectoryControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.GoHomeControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.StopAllTrajectoryControllerCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.InverseDynamicsCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage.BodyPart;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

public class ChestOrientationManager
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final OrientationFeedbackControlCommand orientationFeedbackControlCommand = new OrientationFeedbackControlCommand();

   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable receivedNewChestOrientationTime = new DoubleYoVariable("receivedNewChestOrientationTime", registry);

   private final BooleanYoVariable isTrajectoryStopped = new BooleanYoVariable("isChestOrientationTrajectoryStopped", registry);
   private final BooleanYoVariable isTrackingOrientation = new BooleanYoVariable("isTrackingOrientation", registry);

   private final BooleanYoVariable hasBeenInitialized = new BooleanYoVariable("hasChestOrientationManagerBeenInitialized", registry);

   private final MultipleWaypointsOrientationTrajectoryGenerator waypointOrientationTrajectoryGenerator;
   private final ReferenceFrame pelvisZUpFrame;
   private final ReferenceFrame chestFrame;

   private final BooleanYoVariable initializeToCurrent;

   private final FrameOrientation desiredOrientation = new FrameOrientation();
   private final FrameVector desiredAngularVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector feedForwardAngularAcceleration = new FrameVector(ReferenceFrame.getWorldFrame());

   public ChestOrientationManager(MomentumBasedController momentumBasedController, YoOrientationPIDGainsInterface gains, double trajectoryTime,
         YoVariableRegistry parentRegistry)
   {
      this.yoTime = momentumBasedController.getYoTime();
      this.pelvisZUpFrame = momentumBasedController.getPelvisZUpFrame();

      FullHumanoidRobotModel fullRobotModel = momentumBasedController.getFullRobotModel();
      RigidBody chest = fullRobotModel.getChest();
      RigidBody elevator = fullRobotModel.getElevator();
      chestFrame = chest.getBodyFixedFrame();

      orientationFeedbackControlCommand.setWeightForSolver(10.0);
      orientationFeedbackControlCommand.set(elevator, chest);
      orientationFeedbackControlCommand.setGains(gains);

      boolean allowMultipleFrames = true;
      waypointOrientationTrajectoryGenerator = new MultipleWaypointsOrientationTrajectoryGenerator("chest", allowMultipleFrames, pelvisZUpFrame, registry);
      waypointOrientationTrajectoryGenerator.registerNewTrajectoryFrame(worldFrame);

      initializeToCurrent = new BooleanYoVariable("initializeChestOrientationToCurrent", registry);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      if (hasBeenInitialized.getBooleanValue())
         return;

      hasBeenInitialized.set(true);

      holdCurrentOrientation();
   }

   public void compute()
   {
      if (isTrackingOrientation.getBooleanValue())
      {
         if (!isTrajectoryStopped.getBooleanValue())
         {
            double deltaTime = yoTime.getDoubleValue() - receivedNewChestOrientationTime.getDoubleValue();
            waypointOrientationTrajectoryGenerator.compute(deltaTime);
         }
         boolean isTrajectoryDone = waypointOrientationTrajectoryGenerator.isDone();

         if (isTrajectoryDone)
            waypointOrientationTrajectoryGenerator.changeFrame(pelvisZUpFrame);

         isTrackingOrientation.set(!isTrajectoryDone);
      }

      waypointOrientationTrajectoryGenerator.getOrientation(desiredOrientation);
      desiredAngularVelocity.setToZero(worldFrame);
      feedForwardAngularAcceleration.setToZero(worldFrame);
      orientationFeedbackControlCommand.changeFrameAndSet(desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);
   }

   public void holdCurrentOrientation()
   {
      initializeToCurrent.set(false);

      receivedNewChestOrientationTime.set(yoTime.getDoubleValue());

      desiredOrientation.setToZero(chestFrame);
      desiredOrientation.changeFrame(pelvisZUpFrame);
      desiredAngularVelocity.setToZero(pelvisZUpFrame);
      waypointOrientationTrajectoryGenerator.changeFrame(pelvisZUpFrame);
      waypointOrientationTrajectoryGenerator.appendWaypoint(0.0, desiredOrientation, desiredAngularVelocity);
      waypointOrientationTrajectoryGenerator.initialize();
      isTrackingOrientation.set(true);
      isTrajectoryStopped.set(false);
   }

   public void handleChestTrajectoryMessage(ChestTrajectoryControllerCommand message)
   {
      receivedNewChestOrientationTime.set(yoTime.getDoubleValue());

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
      waypointOrientationTrajectoryGenerator.changeFrame(pelvisZUpFrame);
      waypointOrientationTrajectoryGenerator.initialize();
      isTrackingOrientation.set(true);
      isTrajectoryStopped.set(false);
   }

   public void handleStopAllTrajectoryMessage(StopAllTrajectoryControllerCommand message)
   {
      isTrajectoryStopped.set(message.isStopAllTrajectory());
   }

   public void handleGoHomeMessage(GoHomeControllerCommand message)
   {
      if (message.getRequest(BodyPart.CHEST))
         goToHomeFromCurrentDesired(message.getTrajectoryTime());
   }

   public void goToHomeFromCurrentDesired(double trajectoryTime)
   {
      receivedNewChestOrientationTime.set(yoTime.getDoubleValue());

      waypointOrientationTrajectoryGenerator.getOrientation(desiredOrientation);

      desiredOrientation.changeFrame(pelvisZUpFrame);
      waypointOrientationTrajectoryGenerator.clear();
      waypointOrientationTrajectoryGenerator.switchTrajectoryFrame(pelvisZUpFrame);
      waypointOrientationTrajectoryGenerator.appendWaypoint(0.0, desiredOrientation, desiredAngularVelocity);

      desiredOrientation.setToZero(pelvisZUpFrame);
      waypointOrientationTrajectoryGenerator.appendWaypoint(trajectoryTime, desiredOrientation, desiredAngularVelocity);
      waypointOrientationTrajectoryGenerator.initialize();

      isTrackingOrientation.set(true);
      isTrajectoryStopped.set(false);
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
