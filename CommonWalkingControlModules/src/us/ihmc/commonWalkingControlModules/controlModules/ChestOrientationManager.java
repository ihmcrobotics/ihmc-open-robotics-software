package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ChestTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.GoHomeCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage.BodyPart;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

import javax.vecmath.Vector3d;

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

   private final FrameOrientation desiredOrientation = new FrameOrientation();
   private final FrameVector desiredAngularVelocity = new FrameVector();
   private final FrameVector feedForwardAngularAcceleration = new FrameVector();

   private final YoFrameVector yoChestAngularWeight = new YoFrameVector("chestWeight", null, registry);
   private final Vector3d chestAngularWeight = new Vector3d();

   private final YoOrientationPIDGainsInterface gains;

   public ChestOrientationManager(MomentumBasedController momentumBasedController, YoOrientationPIDGainsInterface gains, Vector3d angularWeight,
         double trajectoryTime, YoVariableRegistry parentRegistry)
   {
      this.gains = gains;
      yoTime = momentumBasedController.getYoTime();
      pelvisZUpFrame = momentumBasedController.getPelvisZUpFrame();

      FullRobotModel fullRobotModel = momentumBasedController.getFullRobotModel();
      RigidBody chest = fullRobotModel.getChest();
      RigidBody elevator = fullRobotModel.getElevator();
      chestFrame = chest.getBodyFixedFrame();

      yoChestAngularWeight.set(angularWeight);
      yoChestAngularWeight.get(chestAngularWeight);
      orientationFeedbackControlCommand.setWeightsForSolver(chestAngularWeight);
      orientationFeedbackControlCommand.set(elevator, chest);
      orientationFeedbackControlCommand.setGains(gains);

      boolean allowMultipleFrames = true;
      waypointOrientationTrajectoryGenerator = new MultipleWaypointsOrientationTrajectoryGenerator("chest", allowMultipleFrames, pelvisZUpFrame, registry);
      waypointOrientationTrajectoryGenerator.registerNewTrajectoryFrame(worldFrame);

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
      orientationFeedbackControlCommand.setGains(gains);
      yoChestAngularWeight.get(chestAngularWeight);
      orientationFeedbackControlCommand.setWeightsForSolver(chestAngularWeight);
   }

   public void holdCurrentOrientation()
   {
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

   public void handleChestTrajectoryCommand(ChestTrajectoryCommand command)
   {
      receivedNewChestOrientationTime.set(yoTime.getDoubleValue());

      if (command.getTrajectoryPoint(0).getTime() > 1.0e-5)
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

      waypointOrientationTrajectoryGenerator.appendWaypoints(command);
      waypointOrientationTrajectoryGenerator.changeFrame(pelvisZUpFrame);
      waypointOrientationTrajectoryGenerator.initialize();
      isTrackingOrientation.set(true);
      isTrajectoryStopped.set(false);
   }

   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
      isTrajectoryStopped.set(command.isStopAllTrajectory());
   }

   public void handleGoHomeCommand(GoHomeCommand command)
   {
      if (command.getRequest(BodyPart.CHEST))
         goToHomeFromCurrentDesired(command.getTrajectoryTime());
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
