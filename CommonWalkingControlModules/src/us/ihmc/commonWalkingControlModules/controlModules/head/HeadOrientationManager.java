package us.ihmc.commonWalkingControlModules.controlModules.head;

import us.ihmc.commonWalkingControlModules.configurations.HeadOrientationControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.packetConsumers.HeadOrientationProvider;
import us.ihmc.robotics.controllers.YoOrientationPIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.math.trajectories.OrientationInterpolationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.providers.YoQuaternionProvider;
import us.ihmc.robotics.math.trajectories.providers.YoVariableDoubleProvider;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class HeadOrientationManager
{
   private final YoVariableRegistry registry;
   private final HeadOrientationControlModule headOrientationControlModule;
   private final MomentumBasedController momentumBasedController;
   private final HeadOrientationProvider desiredHeadOrientationProvider;
   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable receivedNewHeadOrientationTime;
   private final DoubleYoVariable headOrientationTrajectoryTime;
   private final OrientationInterpolationTrajectoryGenerator orientationTrajectoryGenerator;

   private final BooleanYoVariable isTrackingOrientation;

   private final ReferenceFrame chestFrame;
   private final YoQuaternionProvider initialOrientationProvider;
   private final YoQuaternionProvider finalOrientationProvider;

   private final BooleanYoVariable hasBeenInitialized;

   public HeadOrientationManager(MomentumBasedController momentumBasedController, HeadOrientationControllerParameters headOrientationControllerParameters,
         YoOrientationPIDGains gains, HeadOrientationProvider desiredHeadOrientationProvider, double trajectoryTime, double[] initialDesiredHeadYawPitchRoll,
         YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(getClass().getSimpleName());

      this.momentumBasedController = momentumBasedController;
      this.yoTime = momentumBasedController.getYoTime();
      chestFrame = momentumBasedController.getFullRobotModel().getChest().getBodyFixedFrame();

      YoGraphicsListRegistry yoGraphicsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();
      this.headOrientationControlModule = new HeadOrientationControlModule(momentumBasedController, chestFrame, headOrientationControllerParameters, gains,
            registry, yoGraphicsListRegistry);
      this.desiredHeadOrientationProvider = desiredHeadOrientationProvider;
      parentRegistry.addChild(registry);

      if (desiredHeadOrientationProvider != null)
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
         orientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator("headOrientation", chestFrame, trajectoryTimeProvider,
               initialOrientationProvider, finalOrientationProvider, registry);
         orientationTrajectoryGenerator.setContinuouslyUpdateFinalOrientation(true);
      }
      else
      {
         receivedNewHeadOrientationTime = null;
         headOrientationTrajectoryTime = null;
         isTrackingOrientation = null;
         hasBeenInitialized = null;
         initialOrientationProvider = null;
         finalOrientationProvider = null;
         orientationTrajectoryGenerator = null;
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
      orientationTrajectoryGenerator.initialize();
      isTrackingOrientation.set(true);
   }

   private final FrameOrientation desiredOrientation = new FrameOrientation();

   public void compute()
   {
      initialize();

      checkForNewDesiredOrientationInformation();

      if (desiredHeadOrientationProvider != null)
      {
         if (isTrackingOrientation.getBooleanValue())
         {
            double deltaTime = yoTime.getDoubleValue() - receivedNewHeadOrientationTime.getDoubleValue();
            orientationTrajectoryGenerator.compute(deltaTime);
            isTrackingOrientation.set(!orientationTrajectoryGenerator.isDone());
         }
         orientationTrajectoryGenerator.get(desiredOrientation);
         headOrientationControlModule.setOrientationToTrack(desiredOrientation);
      }

      headOrientationControlModule.compute();
   }

   private void checkForNewDesiredOrientationInformation()
   {
      if (desiredHeadOrientationProvider == null)
         return;

      if (desiredHeadOrientationProvider.isNewHeadOrientationInformationAvailable())
      {
         orientationTrajectoryGenerator.get(desiredOrientation);
         initialOrientationProvider.setOrientation(desiredOrientation);
         FrameOrientation desiredHeadOrientation = desiredHeadOrientationProvider.getDesiredHeadOrientation();
         desiredHeadOrientation.changeFrame(chestFrame);
         finalOrientationProvider.setOrientation(desiredHeadOrientation);
         headOrientationTrajectoryTime.set(desiredHeadOrientationProvider.getTrajectoryTime());
         receivedNewHeadOrientationTime.set(yoTime.getDoubleValue());
         orientationTrajectoryGenerator.initialize();
         isTrackingOrientation.set(true);
      }
      else if (desiredHeadOrientationProvider.isNewLookAtInformationAvailable())
      {
         orientationTrajectoryGenerator.get(desiredOrientation);
         initialOrientationProvider.setOrientation(desiredOrientation);
         headOrientationControlModule.setPointToTrack(desiredHeadOrientationProvider.getLookAtPoint());
         headOrientationControlModule.packDesiredFrameOrientation(desiredOrientation);
         desiredOrientation.changeFrame(chestFrame);
         finalOrientationProvider.setOrientation(desiredOrientation);
         headOrientationTrajectoryTime.set(desiredHeadOrientationProvider.getTrajectoryTime());
         receivedNewHeadOrientationTime.set(yoTime.getDoubleValue());
         orientationTrajectoryGenerator.initialize();
         isTrackingOrientation.set(true);
      }
   }
}
