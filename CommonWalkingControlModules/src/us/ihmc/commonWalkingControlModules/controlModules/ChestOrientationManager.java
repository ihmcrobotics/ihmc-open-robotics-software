package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.packetConsumers.ChestOrientationProvider;
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
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.OrientationTrajectoryGeneratorInMultipleFrames;
import us.ihmc.robotics.math.trajectories.SimpleOrientationTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.TwistCalculator;

public class ChestOrientationManager
{
   private static final double defaultTrajectoryTime = 2.0;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;
   private final ChestOrientationControlModule chestOrientationControlModule;
   private final OrientationFeedbackControlCommand orientationFeedbackControlCommand = new OrientationFeedbackControlCommand();

   private final ChestTrajectoryMessageSubscriber chestTrajectoryMessageSubscriber;
   private final StopAllTrajectoryMessageSubscriber stopAllTrajectoryMessageSubscriber;
   private final GoHomeMessageSubscriber goHomeMessageSubscriber;
   private final ChestOrientationProvider chestOrientationProvider;
   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable receivedNewChestOrientationTime;

   private final BooleanYoVariable isTrajectoryStopped;
   private final BooleanYoVariable isTrackingOrientation;
   private final BooleanYoVariable isUsingWaypointTrajectory;
   private final YoFrameVector yoControlledAngularAcceleration;

   private OrientationTrajectoryGeneratorInMultipleFrames activeTrajectoryGenerator;
   private final SimpleOrientationTrajectoryGenerator simpleOrientationTrajectoryGenerator;
   private final MultipleWaypointsOrientationTrajectoryGenerator waypointOrientationTrajectoryGenerator;
   private final ReferenceFrame pelvisZUpFrame;
   private final ReferenceFrame chestFrame;

   private final BooleanYoVariable initializeToCurrent;

   public ChestOrientationManager(MomentumBasedController momentumBasedController, YoOrientationPIDGainsInterface chestControlGains,
         VariousWalkingProviders variousWalkingProviders, double trajectoryTime, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(getClass().getSimpleName());

      this.yoTime = momentumBasedController.getYoTime();
      this.chestTrajectoryMessageSubscriber = variousWalkingProviders.getChestTrajectoryMessageSubscriber();
      this.stopAllTrajectoryMessageSubscriber = variousWalkingProviders.getStopAllTrajectoryMessageSubscriber();
      this.chestOrientationProvider = variousWalkingProviders.getDesiredChestOrientationProvider();
      this.goHomeMessageSubscriber = variousWalkingProviders.getGoHomeMessageSubscriber();
      this.pelvisZUpFrame = momentumBasedController.getPelvisZUpFrame();

      FullHumanoidRobotModel fullRobotModel = momentumBasedController.getFullRobotModel();
      RigidBody chest = fullRobotModel.getChest();
      RigidBody elevator = fullRobotModel.getElevator();
      chestFrame = chest.getBodyFixedFrame();
      TwistCalculator twistCalculator = momentumBasedController.getTwistCalculator();
      double controlDT = momentumBasedController.getControlDT();
      chestOrientationControlModule = new ChestOrientationControlModule(pelvisZUpFrame, elevator, chest, twistCalculator, controlDT, chestControlGains,
            registry);

      orientationFeedbackControlCommand.set(elevator, chest);

      yoControlledAngularAcceleration = new YoFrameVector("controlledChestAngularAcceleration", chestFrame, registry);

      if (chestOrientationProvider != null || chestTrajectoryMessageSubscriber != null)
      {
         isTrajectoryStopped = new BooleanYoVariable("isChestOrientationTrajectoryStopped", registry);
         isTrackingOrientation = new BooleanYoVariable("isTrackingOrientation", registry);
         receivedNewChestOrientationTime = new DoubleYoVariable("receivedNewChestOrientationTime", registry);

         isUsingWaypointTrajectory = new BooleanYoVariable(getClass().getSimpleName() + "IsUsingWaypointTrajectory", registry);
         isUsingWaypointTrajectory.set(false);

         boolean allowMultipleFrames = true;
         simpleOrientationTrajectoryGenerator = new SimpleOrientationTrajectoryGenerator("chest", allowMultipleFrames, pelvisZUpFrame, parentRegistry);
         simpleOrientationTrajectoryGenerator.registerNewTrajectoryFrame(worldFrame);
         simpleOrientationTrajectoryGenerator.setTrajectoryTime(trajectoryTime);
         simpleOrientationTrajectoryGenerator.initialize();
         activeTrajectoryGenerator = simpleOrientationTrajectoryGenerator;

         waypointOrientationTrajectoryGenerator = new MultipleWaypointsOrientationTrajectoryGenerator("chest", 15, allowMultipleFrames, pelvisZUpFrame,
               registry);
         waypointOrientationTrajectoryGenerator.registerNewTrajectoryFrame(worldFrame);

         initializeToCurrent = new BooleanYoVariable("initializeChestOrientationToCurrent", registry);
      }
      else
      {
         isTrajectoryStopped = null;
         isTrackingOrientation = null;
         receivedNewChestOrientationTime = null;
         isUsingWaypointTrajectory = null;
         simpleOrientationTrajectoryGenerator = null;
         waypointOrientationTrajectoryGenerator = null;

         initializeToCurrent = null;
      }

      parentRegistry.addChild(registry);
   }

   private final FrameOrientation desiredOrientation = new FrameOrientation();
   private final FrameVector desiredAngularVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector feedForwardAngularAcceleration = new FrameVector(ReferenceFrame.getWorldFrame());

   private final FrameVector controlledAngularAcceleration = new FrameVector();

   public void compute()
   {
      if (isUsingWaypointTrajectory != null)
      {
         if (isUsingWaypointTrajectory.getBooleanValue())
            activeTrajectoryGenerator = waypointOrientationTrajectoryGenerator;
         else
            activeTrajectoryGenerator = simpleOrientationTrajectoryGenerator;
      }

      handleStopAllTrajectoryMessage();
      checkForNewDesiredOrientationInformation();
      handleChestTrajectoryMessages();
      handleGoHomeMessages();

      if (activeTrajectoryGenerator != null && isTrackingOrientation.getBooleanValue())
      {
         if (!isTrajectoryStopped.getBooleanValue())
         {
            double deltaTime = yoTime.getDoubleValue() - receivedNewChestOrientationTime.getDoubleValue();
            activeTrajectoryGenerator.compute(deltaTime);
         }
         boolean isTrajectoryDone = activeTrajectoryGenerator.isDone();

         if (isTrajectoryDone)
            activeTrajectoryGenerator.changeFrame(pelvisZUpFrame);

         activeTrajectoryGenerator.getOrientation(desiredOrientation);
         chestOrientationControlModule.setDesireds(desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);
         isTrackingOrientation.set(!isTrajectoryDone);
      }

      chestOrientationControlModule.compute();

      if (yoControlledAngularAcceleration != null)
      {
         SpatialAccelerationVector spatialAcceleration = chestOrientationControlModule.getSpatialAccelerationCommand().getSpatialAcceleration();
         if (spatialAcceleration.getExpressedInFrame() != null) // That happens when there is no joint to control.
         {
            spatialAcceleration.getAngularPart(controlledAngularAcceleration);
            yoControlledAngularAcceleration.set(controlledAngularAcceleration);
         }
      }
   }

   public void holdCurrentOrientation()
   {
      if (initializeToCurrent != null)
         initializeToCurrent.set(true);
   }

   private final FrameVector tempAngularVelocity = new FrameVector();

   private void handleChestTrajectoryMessages()
   {
      if (chestTrajectoryMessageSubscriber == null || !chestTrajectoryMessageSubscriber.isNewTrajectoryMessageAvailable())
         return;

      ChestTrajectoryMessage message = chestTrajectoryMessageSubscriber.pollMessage();

      receivedNewChestOrientationTime.set(yoTime.getDoubleValue());

      waypointOrientationTrajectoryGenerator.switchTrajectoryFrame(worldFrame);
      waypointOrientationTrajectoryGenerator.clear();

      if (message.getWaypoint(0).getTime() > 1.0e-5)
      {
         activeTrajectoryGenerator.changeFrame(worldFrame);
         activeTrajectoryGenerator.getOrientation(desiredOrientation);
         tempAngularVelocity.setToZero(worldFrame);

         waypointOrientationTrajectoryGenerator.appendWaypoint(0.0, desiredOrientation, tempAngularVelocity);
      }

      waypointOrientationTrajectoryGenerator.appendWaypoints(message.getWaypoints());
      waypointOrientationTrajectoryGenerator.changeFrame(pelvisZUpFrame);
      waypointOrientationTrajectoryGenerator.initialize();
      isUsingWaypointTrajectory.set(true);
      activeTrajectoryGenerator = waypointOrientationTrajectoryGenerator;
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
         goToHome(goHomeMessageSubscriber.pollMessage(BodyPart.CHEST));
   }

   private void checkForNewDesiredOrientationInformation()
   {
      if (chestOrientationProvider == null)
         return;

      if (initializeToCurrent.getBooleanValue())
      {
         initializeToCurrent.set(false);

         receivedNewChestOrientationTime.set(yoTime.getDoubleValue());

         simpleOrientationTrajectoryGenerator.changeFrame(pelvisZUpFrame);
         desiredOrientation.setToZero(chestFrame);
         desiredOrientation.changeFrame(pelvisZUpFrame);
         simpleOrientationTrajectoryGenerator.setInitialOrientation(desiredOrientation);
         simpleOrientationTrajectoryGenerator.setFinalOrientation(desiredOrientation);
         simpleOrientationTrajectoryGenerator.setTrajectoryTime(0.0);
         simpleOrientationTrajectoryGenerator.initialize();
         isUsingWaypointTrajectory.set(false);
         activeTrajectoryGenerator = simpleOrientationTrajectoryGenerator;
         isTrackingOrientation.set(true);
         isTrajectoryStopped.set(false);
      }
      else
      {
         if (chestOrientationProvider.checkForHomeOrientation())
         {
            double trajectoryTime = chestOrientationProvider.getTrajectoryTime();
            goToHome(trajectoryTime);
         }
         else if (chestOrientationProvider.checkForNewChestOrientation())
         {
            double trajectoryTime = chestOrientationProvider.getTrajectoryTime();
            receivedNewChestOrientationTime.set(yoTime.getDoubleValue());

            simpleOrientationTrajectoryGenerator.changeFrame(pelvisZUpFrame);
            activeTrajectoryGenerator.changeFrame(pelvisZUpFrame);
            activeTrajectoryGenerator.getOrientation(desiredOrientation);
            simpleOrientationTrajectoryGenerator.setInitialOrientation(desiredOrientation);
            FrameOrientation desiredChestOrientation = chestOrientationProvider.getDesiredChestOrientation();
            desiredChestOrientation.changeFrame(pelvisZUpFrame);
            simpleOrientationTrajectoryGenerator.setFinalOrientation(desiredChestOrientation);
            simpleOrientationTrajectoryGenerator.setTrajectoryTime(trajectoryTime);
            simpleOrientationTrajectoryGenerator.initialize();
            isUsingWaypointTrajectory.set(false);
            activeTrajectoryGenerator = simpleOrientationTrajectoryGenerator;
            isTrackingOrientation.set(true);
            isTrajectoryStopped.set(false);
         }
      }
   }

   public void goToHome()
   {
      goToHome(defaultTrajectoryTime);
   }

   public void goToHome(double trajectoryTime)
   {
      receivedNewChestOrientationTime.set(yoTime.getDoubleValue());

      simpleOrientationTrajectoryGenerator.changeFrame(pelvisZUpFrame);
      activeTrajectoryGenerator.changeFrame(pelvisZUpFrame);
      activeTrajectoryGenerator.getOrientation(desiredOrientation);
      simpleOrientationTrajectoryGenerator.setInitialOrientation(desiredOrientation);
      desiredOrientation.setToZero(pelvisZUpFrame);
      simpleOrientationTrajectoryGenerator.setFinalOrientation(desiredOrientation);
      simpleOrientationTrajectoryGenerator.setTrajectoryTime(trajectoryTime);
      simpleOrientationTrajectoryGenerator.initialize();
      isUsingWaypointTrajectory.set(false);
      activeTrajectoryGenerator = simpleOrientationTrajectoryGenerator;
      isTrackingOrientation.set(true);
      isTrajectoryStopped.set(false);
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return chestOrientationControlModule.getSpatialAccelerationCommand();
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return orientationFeedbackControlCommand;
   }
}
