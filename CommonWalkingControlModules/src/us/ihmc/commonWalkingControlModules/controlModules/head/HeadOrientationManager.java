package us.ihmc.commonWalkingControlModules.controlModules.head;

import us.ihmc.commonWalkingControlModules.configurations.HeadOrientationControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.packetConsumers.HeadOrientationProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.HeadTrajectoryMessageSubscriber;
import us.ihmc.humanoidRobotics.communication.packets.walking.HeadTrajectoryMessage;
import us.ihmc.robotics.controllers.YoOrientationPIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.OrientationInterpolationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.OrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.providers.YoQuaternionProvider;
import us.ihmc.robotics.math.trajectories.providers.YoVariableDoubleProvider;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class HeadOrientationManager
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;
   private final HeadOrientationControlModule headOrientationControlModule;
   private final MomentumBasedController momentumBasedController;
   private final HeadTrajectoryMessageSubscriber headTrajectoryMessageSubscriber;
   private final HeadOrientationProvider desiredHeadOrientationProvider;
   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable receivedNewHeadOrientationTime;
   private final DoubleYoVariable headOrientationTrajectoryTime;
   private OrientationTrajectoryGenerator activeTrajectoryGenerator;
   private final OrientationInterpolationTrajectoryGenerator simpleOrientationTrajectoryGenerator;
   private final MultipleWaypointsOrientationTrajectoryGenerator waypointOrientationTrajectoryGenerator;

   private final BooleanYoVariable isTrackingOrientation;
   private final BooleanYoVariable isUsingWaypointTrajectory;

   private final ReferenceFrame chestFrame;
   private final YoQuaternionProvider initialOrientationProvider;
   private final YoQuaternionProvider finalOrientationProvider;

   private final BooleanYoVariable hasBeenInitialized;

   public HeadOrientationManager(MomentumBasedController momentumBasedController, HeadOrientationControllerParameters headOrientationControllerParameters,
         YoOrientationPIDGains gains, HeadOrientationProvider desiredHeadOrientationProvider, HeadTrajectoryMessageSubscriber headTrajectoryMessageSubscriber,
         double trajectoryTime, double[] initialDesiredHeadYawPitchRoll, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(getClass().getSimpleName());

      this.momentumBasedController = momentumBasedController;
      this.yoTime = momentumBasedController.getYoTime();
      chestFrame = momentumBasedController.getFullRobotModel().getChest().getBodyFixedFrame();

      YoGraphicsListRegistry yoGraphicsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();
      this.headOrientationControlModule = new HeadOrientationControlModule(momentumBasedController, chestFrame, headOrientationControllerParameters, gains,
            registry, yoGraphicsListRegistry);
      this.desiredHeadOrientationProvider = desiredHeadOrientationProvider;
      this.headTrajectoryMessageSubscriber = headTrajectoryMessageSubscriber;
      parentRegistry.addChild(registry);

      if (desiredHeadOrientationProvider != null || headTrajectoryMessageSubscriber != null)
      {

         receivedNewHeadOrientationTime = new DoubleYoVariable("receivedNewHeadOrientationTime", registry);
         isTrackingOrientation = new BooleanYoVariable("isTrackingOrientation", registry);
         headOrientationTrajectoryTime = new DoubleYoVariable("headOrientationTrajectoryTime", registry);

         hasBeenInitialized = new BooleanYoVariable("hasHeadOrientationManagerBeenInitialized", registry);
         hasBeenInitialized.set(false);

         headOrientationTrajectoryTime.set(trajectoryTime);
         DoubleProvider trajectoryTimeProvider = new YoVariableDoubleProvider(headOrientationTrajectoryTime);
         initialOrientationProvider = new YoQuaternionProvider("headInitialOrientation", chestFrame, registry);
         finalOrientationProvider = new YoQuaternionProvider("headFinalOrientation", chestFrame, registry);
         desiredOrientation.setIncludingFrame(chestFrame, initialDesiredHeadYawPitchRoll);
         finalOrientationProvider.setOrientation(desiredOrientation);
         simpleOrientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator("headOrientation", chestFrame, trajectoryTimeProvider,
               initialOrientationProvider, finalOrientationProvider, registry);
         waypointOrientationTrajectoryGenerator = new MultipleWaypointsOrientationTrajectoryGenerator("headWaypoint", 15, true, chestFrame, registry);
         simpleOrientationTrajectoryGenerator.setContinuouslyUpdateFinalOrientation(true);

         isUsingWaypointTrajectory = new BooleanYoVariable(getClass().getSimpleName() + "IsUsingWaypointTrajectory", registry);
         isUsingWaypointTrajectory.set(false);

         activeTrajectoryGenerator = simpleOrientationTrajectoryGenerator;
      }
      else
      {
         receivedNewHeadOrientationTime = null;
         headOrientationTrajectoryTime = null;
         isTrackingOrientation = null;
         isUsingWaypointTrajectory = null;
         hasBeenInitialized = null;
         initialOrientationProvider = null;
         finalOrientationProvider = null;
         simpleOrientationTrajectoryGenerator = null;
         waypointOrientationTrajectoryGenerator = null;
      }
   }

   private void initialize()
   {
      if (hasBeenInitialized == null || hasBeenInitialized.getBooleanValue())
         return;

      hasBeenInitialized.set(true);

      ReferenceFrame headFrame = momentumBasedController.getFullRobotModel().getHead().getBodyFixedFrame();
      desiredOrientation.setToZero(headFrame);
      desiredOrientation.changeFrame(chestFrame);
      initialOrientationProvider.setOrientation(desiredOrientation);
      receivedNewHeadOrientationTime.set(yoTime.getDoubleValue());
      simpleOrientationTrajectoryGenerator.initialize();
      isTrackingOrientation.set(true);
      isUsingWaypointTrajectory.set(false);
      activeTrajectoryGenerator = simpleOrientationTrajectoryGenerator;
   }

   private final FrameOrientation desiredOrientation = new FrameOrientation();

   public void compute()
   {
      initialize();

      if (isUsingWaypointTrajectory != null)
      {
         if (isUsingWaypointTrajectory.getBooleanValue())
            activeTrajectoryGenerator = waypointOrientationTrajectoryGenerator;
         else
            activeTrajectoryGenerator = simpleOrientationTrajectoryGenerator;
      }

      checkForNewDesiredOrientationInformation();
      handleHeadTrajectoryMessages();

      if (desiredHeadOrientationProvider != null)
      {
         if (isTrackingOrientation.getBooleanValue())
         {
            double deltaTime = yoTime.getDoubleValue() - receivedNewHeadOrientationTime.getDoubleValue();
            activeTrajectoryGenerator.compute(deltaTime);
            isTrackingOrientation.set(!activeTrajectoryGenerator.isDone());
         }
         activeTrajectoryGenerator.get(desiredOrientation);
         headOrientationControlModule.setOrientationToTrack(desiredOrientation);
      }

      headOrientationControlModule.compute();
   }

   private final FrameOrientation tempOrientation = new FrameOrientation();
   private final FrameVector tempAngularVelocity = new FrameVector();

   private void handleHeadTrajectoryMessages()
   {
      if (headTrajectoryMessageSubscriber == null || !headTrajectoryMessageSubscriber.isNewTrajectoryMessageAvailable())
         return;

      HeadTrajectoryMessage message = headTrajectoryMessageSubscriber.pollMessage();

      waypointOrientationTrajectoryGenerator.switchTrajectoryFrame(worldFrame);
      waypointOrientationTrajectoryGenerator.clear();

      if (message.getWaypoint(0).getTime() > 1.0e-5)
      {
         tempOrientation.setToZero(chestFrame);
         tempOrientation.changeFrame(worldFrame);
         tempAngularVelocity.setToZero(worldFrame);

         waypointOrientationTrajectoryGenerator.appendWaypoint(0.0, tempOrientation, tempAngularVelocity);
      }

      waypointOrientationTrajectoryGenerator.appendWaypoints(message.getWaypoints());
      waypointOrientationTrajectoryGenerator.changeFrame(chestFrame);
      waypointOrientationTrajectoryGenerator.initialize();
      isUsingWaypointTrajectory.set(true);
      activeTrajectoryGenerator = waypointOrientationTrajectoryGenerator;
      isTrackingOrientation.set(true);
   }

   private void checkForNewDesiredOrientationInformation()
   {
      if (desiredHeadOrientationProvider == null)
         return;

      if (desiredHeadOrientationProvider.isNewHeadOrientationInformationAvailable())
      {
         simpleOrientationTrajectoryGenerator.get(desiredOrientation);
         initialOrientationProvider.setOrientation(desiredOrientation);
         FrameOrientation desiredHeadOrientation = desiredHeadOrientationProvider.getDesiredHeadOrientation();
         desiredHeadOrientation.changeFrame(chestFrame);
         finalOrientationProvider.setOrientation(desiredHeadOrientation);
         headOrientationTrajectoryTime.set(desiredHeadOrientationProvider.getTrajectoryTime());
         receivedNewHeadOrientationTime.set(yoTime.getDoubleValue());
         simpleOrientationTrajectoryGenerator.initialize();
         isTrackingOrientation.set(true);
         isUsingWaypointTrajectory.set(false);
         activeTrajectoryGenerator = simpleOrientationTrajectoryGenerator;
      }
      else if (desiredHeadOrientationProvider.isNewLookAtInformationAvailable())
      {
         simpleOrientationTrajectoryGenerator.get(desiredOrientation);
         initialOrientationProvider.setOrientation(desiredOrientation);
         headOrientationControlModule.setPointToTrack(desiredHeadOrientationProvider.getLookAtPoint());
         headOrientationControlModule.packDesiredFrameOrientation(desiredOrientation);
         desiredOrientation.changeFrame(chestFrame);
         finalOrientationProvider.setOrientation(desiredOrientation);
         headOrientationTrajectoryTime.set(desiredHeadOrientationProvider.getTrajectoryTime());
         receivedNewHeadOrientationTime.set(yoTime.getDoubleValue());
         simpleOrientationTrajectoryGenerator.initialize();
         isTrackingOrientation.set(true);
         isUsingWaypointTrajectory.set(false);
         activeTrajectoryGenerator = simpleOrientationTrajectoryGenerator;
      }
   }
}
