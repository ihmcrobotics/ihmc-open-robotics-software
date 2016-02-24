package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.packetConsumers.ChestTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.GoHomeMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.StopAllTrajectoryMessageSubscriber;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage.BodyPart;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

public class ChestOrientationManager
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final OrientationFeedbackControlCommand orientationFeedbackControlCommand = new OrientationFeedbackControlCommand();

   private final ChestTrajectoryMessageSubscriber chestTrajectoryMessageSubscriber;
   private final StopAllTrajectoryMessageSubscriber stopAllTrajectoryMessageSubscriber;
   private final GoHomeMessageSubscriber goHomeMessageSubscriber;
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

   public ChestOrientationManager(MomentumBasedController momentumBasedController, YoOrientationPIDGainsInterface gains,
         VariousWalkingProviders variousWalkingProviders, double trajectoryTime, YoVariableRegistry parentRegistry)
   {
      this.yoTime = momentumBasedController.getYoTime();
      this.chestTrajectoryMessageSubscriber = variousWalkingProviders.getChestTrajectoryMessageSubscriber();
      this.stopAllTrajectoryMessageSubscriber = variousWalkingProviders.getStopAllTrajectoryMessageSubscriber();
      this.goHomeMessageSubscriber = variousWalkingProviders.getGoHomeMessageSubscriber();
      this.pelvisZUpFrame = momentumBasedController.getPelvisZUpFrame();

      FullHumanoidRobotModel fullRobotModel = momentumBasedController.getFullRobotModel();
      RigidBody chest = fullRobotModel.getChest();
      RigidBody elevator = fullRobotModel.getElevator();
      chestFrame = chest.getBodyFixedFrame();

      orientationFeedbackControlCommand.set(elevator, chest);
      orientationFeedbackControlCommand.setGains(gains);

      boolean allowMultipleFrames = true;
      waypointOrientationTrajectoryGenerator = new MultipleWaypointsOrientationTrajectoryGenerator("chest", 15, allowMultipleFrames, pelvisZUpFrame, registry);
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
      handleStopAllTrajectoryMessage();
      handleChestTrajectoryMessages();
      handleGoHomeMessages();

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

   private void handleChestTrajectoryMessages()
   {
      if (chestTrajectoryMessageSubscriber == null || !chestTrajectoryMessageSubscriber.isNewTrajectoryMessageAvailable())
         return;

      ChestTrajectoryMessage message = chestTrajectoryMessageSubscriber.pollMessage();

      receivedNewChestOrientationTime.set(yoTime.getDoubleValue());

      if (message.getWaypoint(0).getTime() > 1.0e-5)
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

      waypointOrientationTrajectoryGenerator.appendWaypoints(message.getWaypoints());
      waypointOrientationTrajectoryGenerator.changeFrame(pelvisZUpFrame);
      waypointOrientationTrajectoryGenerator.initialize();
      isTrackingOrientation.set(true);
      isTrajectoryStopped.set(false);
   }

   private void handleStopAllTrajectoryMessage()
   {
      if (stopAllTrajectoryMessageSubscriber == null || !stopAllTrajectoryMessageSubscriber.pollMessage(this))
         return;

      isTrajectoryStopped.set(true);
   }

   private void handleGoHomeMessages()
   {
      if (goHomeMessageSubscriber == null)
         return;

      if (goHomeMessageSubscriber.isNewMessageAvailable(BodyPart.CHEST))
         goToHomeFromCurrentDesired(goHomeMessageSubscriber.pollMessage(BodyPart.CHEST));
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
