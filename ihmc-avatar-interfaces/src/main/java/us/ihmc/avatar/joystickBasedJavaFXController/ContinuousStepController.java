package us.ihmc.avatar.joystickBasedJavaFXController;

import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.avatar.joystickBasedJavaFXController.JoystickStepParametersProperty.JoystickStepParameters;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.ContinuousStepGenerator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.DesiredTurningVelocityProvider;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.DesiredVelocityProvider;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootPoseProvider;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepMessenger;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.StopWalkingMessenger;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.collision.BoundingBoxCollisionDetector;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper;
import us.ihmc.footstepPlanning.simplePlanners.SnapAndWiggleSingleStep;
import us.ihmc.footstepPlanning.simplePlanners.SnapAndWiggleSingleStep.SnappingFailedException;
import us.ihmc.footstepPlanning.simplePlanners.SnapAndWiggleSingleStepParameters;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class ContinuousStepController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final ContinuousStepGenerator continuousStepGenerator = new ContinuousStepGenerator(registry);

   private final YoDouble turningVelocity = new YoDouble("turningVelocity", registry);
   private final YoDouble forwardVelocity = new YoDouble("forwardVelocity", registry);
   private final YoDouble lateralVelocity = new YoDouble("lateralVelocity", registry);

   private final AtomicReference<FootstepDataListMessage> footstepsToSendReference = new AtomicReference<>(null);

   private final JoystickStepParameters joystickStepParameters = new JoystickStepParameters();
   private final YoBoolean isWalking = new YoBoolean("isWalking", registry);
   private final YoBoolean hasSuccessfullyStoppedWalking = new YoBoolean("hasSuccessfullyStoppedWalking", registry);

   private final YoBoolean isLeftFootInSupport = new YoBoolean("isLeftFootInSupport", registry);
   private final YoBoolean isRightFootInSupport = new YoBoolean("isRightFootInSupport", registry);
   private final SideDependentList<YoBoolean> isFootInSupport = new SideDependentList<>(isLeftFootInSupport, isRightFootInSupport);
   private final BoundingBoxCollisionDetector collisionDetector;
   private final SteppingParameters steppingParameters;
   private final SnapAndWiggleSingleStepParameters snapAndWiggleParameters = new SnapAndWiggleSingleStepParameters();

   private final AtomicReference<PlanarRegionsList> planarRegionsList = new AtomicReference<>(null);
   private final SnapAndWiggleSingleStep snapAndWiggleSingleStep;
   private final SideDependentList<? extends ConvexPolygon2DReadOnly> footPolygons;
   private final ConvexPolygon2D footPolygonToWiggle = new ConvexPolygon2D();

   private boolean supportFootPosesInitialized = false;
   private final SideDependentList<FramePose3D> lastSupportFootPoses = new SideDependentList<>(null, null);

   private final AtomicReference<Boolean> walkingRequest = new AtomicReference<>(null);

   private StopWalkingMessenger stopWalkingMessenger;
   private FootPoseProvider footPoseProvider;

   public ContinuousStepController(WalkingControllerParameters walkingControllerParameters)
   {
      steppingParameters = walkingControllerParameters.getSteppingParameters();

      snapAndWiggleParameters.setFootLength(walkingControllerParameters.getSteppingParameters().getFootLength());
      snapAndWiggleSingleStep = new SnapAndWiggleSingleStep(snapAndWiggleParameters);

      continuousStepGenerator.setNumberOfTicksBeforeSubmittingFootsteps(0);
      continuousStepGenerator.setNumberOfFootstepsToPlan(10);
      continuousStepGenerator.setWalkInputProvider(isWalking);
      continuousStepGenerator.setDesiredTurningVelocityProvider(new DesiredTurningVelocityProvider()
      {
         @Override
         public double getTurningVelocity()
         {
            return forwardVelocity.getValue() < -1e-10 ? -turningVelocity.getValue() : turningVelocity.getValue();
         }

         @Override
         public boolean isUnitVelocity()
         {
            return true;
         }
      });
      continuousStepGenerator.setDesiredVelocityProvider(new DesiredVelocityProvider()
      {
         @Override
         public Vector2DReadOnly getDesiredVelocity()
         {
            return new Vector2D(forwardVelocity.getValue(), lateralVelocity.getValue());
         }

         @Override
         public boolean isUnitVelocity()
         {
            return true;
         }
      });
      continuousStepGenerator.configureWith(walkingControllerParameters);
      continuousStepGenerator.addFootstepAdjustment(this::adjustFootstep);
      continuousStepGenerator.setFootPoseProvider(robotSide -> lastSupportFootPoses.get(robotSide));
      continuousStepGenerator.addFootstepValidityIndicator(this::isStepSnappable);
      continuousStepGenerator.addFootstepValidityIndicator(this::isSafeDistanceFromObstacle);
      continuousStepGenerator.addFootstepValidityIndicator(this::isSafeStepHeight);

      SteppingParameters steppingParameters = walkingControllerParameters.getSteppingParameters();
      double footLength = steppingParameters.getFootLength();
      double toeWidth = steppingParameters.getToeWidth();
      double footWidth = steppingParameters.getFootWidth();
      ConvexPolygon2D footPolygon = new ConvexPolygon2D();
      footPolygon.addVertex(footLength / 2.0, toeWidth / 2.0);
      footPolygon.addVertex(footLength / 2.0, -toeWidth / 2.0);
      footPolygon.addVertex(-footLength / 2.0, -footWidth / 2.0);
      footPolygon.addVertex(-footLength / 2.0, footWidth / 2.0);
      footPolygon.update();
      footPolygons = new SideDependentList<>(footPolygon, footPolygon);

      double collisionBoxDepth = 0.65;
      double collisionBoxWidth = 1.15;
      double collisionBoxHeight = 1.0;
      collisionDetector = new BoundingBoxCollisionDetector();
      collisionDetector.setBoxDimensions(collisionBoxDepth, collisionBoxWidth, collisionBoxHeight);
   }

   public void setFootstepMessenger(FootstepMessenger footstepMessenger)
   {
      continuousStepGenerator.setFootstepMessenger(footstepMessenger);
   }

   public void setPauseWalkingPublisher(StopWalkingMessenger stopWalkingMessenger)
   {
      this.stopWalkingMessenger = stopWalkingMessenger;
      continuousStepGenerator.setStopWalkingMessenger(stopWalkingMessenger);
   }

   public void setFootPoseProviders(FootPoseProvider footPoseProvider)
   {
      this.footPoseProvider = footPoseProvider;
   }

   public void setContactState(boolean isLeftFootInSupport, boolean isRightFootInSupport)
   {
      this.isLeftFootInSupport.set(isLeftFootInSupport);
      this.isRightFootInSupport.set(isRightFootInSupport);
   }

   public void setJoystickStepParameters(JoystickStepParameters parameters)
   {
      joystickStepParameters.set(parameters);
   }

   public JoystickStepParameters getJoystickStepParameters()
   {
      return joystickStepParameters;
   }

   public boolean initialize()
   {
      if (supportFootPosesInitialized)
         return true;

      if (isLeftFootInSupport.getValue() && isRightFootInSupport.getValue())
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            if (footPoseProvider.getCurrentFootPose(robotSide) == null)
               return false;

            FramePose3D footPose = new FramePose3D(footPoseProvider.getCurrentFootPose(robotSide));
            footPose.changeFrame(worldFrame);
            lastSupportFootPoses.put(robotSide, footPose);
         }

         supportFootPosesInitialized = true;
      }

      return supportFootPosesInitialized;
   }

   public void update()
   {
      if (!initialize())
         return;

      for (RobotSide robotSide : RobotSide.values)
      {
         if (isFootInSupport.get(robotSide).getValue())
         { // Touchdown may not have been made with the foot properly settled, so we update the support foot pose if its current pose is lower.
            FramePose3D footPose = new FramePose3D(footPoseProvider.getCurrentFootPose(robotSide));
            footPose.changeFrame(worldFrame);
            if (!isWalking() || footPose.getZ() < lastSupportFootPoses.get(robotSide).getZ())
               lastSupportFootPoses.put(robotSide, footPose);
         }
      }

      Boolean newWalkingRequest = walkingRequest.getAndSet(null);

      if (newWalkingRequest != null)
      {
         if (newWalkingRequest)
            startWalking(true);
         else
            stopWalking(true);
      }

      continuousStepGenerator.setNumberOfFixedFootsteps(joystickStepParameters.getNumberOfFixedFootsteps());
      continuousStepGenerator.setFootstepTiming(joystickStepParameters.getSwingDuration(), joystickStepParameters.getTransferDuration());
      continuousStepGenerator.setStepTurningLimits(joystickStepParameters.getTurnMaxAngleInward(), joystickStepParameters.getTurnMaxAngleOutward());
      continuousStepGenerator.setStepWidths(joystickStepParameters.getDefaultStepWidth(),
                                            joystickStepParameters.getMinStepWidth(),
                                            joystickStepParameters.getMaxStepWidth());
      continuousStepGenerator.setMaxStepLength(joystickStepParameters.getMaxStepLength());
      continuousStepGenerator.update(Double.NaN);

      if (!isWalking.getValue())
      {
         if (!hasSuccessfullyStoppedWalking.getValue())
         { // Only send pause request if we think the command has not been executed yet. This is to be more robust in case packets are dropped.
            stopWalkingMessenger.submitStopWalkingRequest();
         }
         else
         { // Reset so the foot poses get updated.
            reset();
         }
      }
   }

   public void reset()
   {
      supportFootPosesInitialized = false;
      setContactState(false, false);
   }

   public boolean isInDoubleSupport()
   {
      return isLeftFootInSupport.getValue() && isRightFootInSupport.getValue();
   }

   public boolean isFootInSupport(RobotSide robotSide)
   {
      return isFootInSupport.get(robotSide).getValue();
   }

   public void submitWalkingRequest(Boolean request)
   {
      walkingRequest.set(request);
   }

   public void updateForwardVelocity(double alpha)
   {
      forwardVelocity.set(alpha);
   }

   public void updateLateralVelocity(double alpha)
   {
      lateralVelocity.set(alpha);
   }

   public void updateTurningVelocity(double alpha)
   {
      turningVelocity.set(alpha);
   }

   public void startWalking(boolean confirm)
   {
      if (confirm)
      {
         isWalking.set(true);
         hasSuccessfullyStoppedWalking.set(false);
      }
   }

   public void stopWalking(boolean confirm)
   {
      if (confirm)
      {
         isWalking.set(false);
         footstepsToSendReference.set(null);
      }
   }

   public void consumeFootstepStatus(FootstepStatusMessage footstepStatus)
   {
      continuousStepGenerator.consumeFootstepStatus(footstepStatus);

      if (footstepStatus.getFootstepStatus() == FootstepStatus.COMPLETED.toByte())
      {
         lastSupportFootPoses.put(RobotSide.fromByte(footstepStatus.getRobotSide()),
                                  new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                  footstepStatus.getActualFootPositionInWorld(),
                                                  footstepStatus.getActualFootOrientationInWorld()));
      }
   }

   public void consumePlanarRegionsListMessage(PlanarRegionsListMessage message)
   {
      if (message == null)
         return;

      PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(message);
      snapAndWiggleSingleStep.setPlanarRegions(planarRegionsList);
      collisionDetector.setPlanarRegionsList(new PlanarRegionsList(planarRegionsList.getPlanarRegionsAsList().stream()
                                                                                    .filter(region -> region.getConvexHull()
                                                                                                            .getArea() >= snapAndWiggleParameters.getMinPlanarRegionArea())
                                                                                    .collect(Collectors.toList())));
      this.planarRegionsList.set(planarRegionsList);
   }

   public void updateControllerMotionStatus(RobotMotionStatus newStatus)
   {
      // We only want to verify that the last PauseWalking sent has been successfully executed once.
      // Considering that the user may use a separate app to get the robot to walk, we do not want to interfere with the other app.
      if (hasSuccessfullyStoppedWalking.getValue() || isWalking.getValue())
         return;
      if (newStatus == null)
         return;
      if (newStatus != RobotMotionStatus.IN_MOTION)
         hasSuccessfullyStoppedWalking.set(true);
   }

   public boolean isWalking()
   {
      return isWalking.getValue();
   }

   public boolean hasSuccessfullyStoppedWalking()
   {
      return hasSuccessfullyStoppedWalking.getValue();
   }

   public void setupVisualization(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      continuousStepGenerator.setupVisualization(footPolygons.get(RobotSide.LEFT).getPolygonVerticesView(),
                                                 footPolygons.get(RobotSide.RIGHT).getPolygonVerticesView(),
                                                 yoGraphicsListRegistry);
   }

   private boolean adjustFootstep(FramePose3DReadOnly stanceFootPose, FramePose2DReadOnly footstepPose, RobotSide footSide, FootstepDataMessage adjustedFootstep)
   {
      FramePose3D adjustedBasedOnStanceFoot = new FramePose3D();
      adjustedBasedOnStanceFoot.getPosition().set(footstepPose.getPosition());
      adjustedBasedOnStanceFoot.setZ(continuousStepGenerator.getCurrentSupportFootPose().getZ());
      adjustedBasedOnStanceFoot.getOrientation().set(footstepPose.getOrientation());

      if (planarRegionsList.get() != null)
      {
         FramePose3D wiggledPose = new FramePose3D(adjustedBasedOnStanceFoot);
         footPolygonToWiggle.set(footPolygons.get(footSide));
         try
         {
            snapAndWiggleSingleStep.snapAndWiggle(wiggledPose, footPolygonToWiggle, forwardVelocity.getValue() > 0.0);
            if (wiggledPose.containsNaN())
            {
               adjustedFootstep.getLocation().set(adjustedBasedOnStanceFoot.getPosition());
               adjustedFootstep.getOrientation().set(adjustedBasedOnStanceFoot.getOrientation());
               return true;
            }
         }
         catch (SnappingFailedException e)
         {
            /*
             * It's fine if the snap & wiggle fails, can be because there no planar regions around the footstep.
             * Let's just keep the adjusted footstep based on the pose of the current stance foot.
             */
         }
         adjustedFootstep.getLocation().set(wiggledPose.getPosition());
         adjustedFootstep.getOrientation().set(wiggledPose.getOrientation());
         return true;
      }
      else
      {
         adjustedFootstep.getLocation().set(adjustedBasedOnStanceFoot.getPosition());
         adjustedFootstep.getOrientation().set(adjustedBasedOnStanceFoot.getOrientation());
         return true;
      }
   }

   private boolean isSafeStepHeight(FramePose3DReadOnly touchdownPose, FramePose3DReadOnly stancePose, RobotSide swingSide)
   {
      double heightChange = touchdownPose.getZ() - stancePose.getZ();
      return heightChange < steppingParameters.getMaxStepUp() && heightChange > -steppingParameters.getMaxStepDown();
   }

   private boolean isSafeDistanceFromObstacle(FramePose3DReadOnly touchdownPose, FramePose3DReadOnly stancePose, RobotSide swingSide)
   {
      if (planarRegionsList.get() == null)
         return true;

      double halfStanceWidth = 0.5 * steppingParameters.getInPlaceWidth();

      /** Shift box vertically by max step up, regions below this could be steppable */
      double heightOffset = steppingParameters.getMaxStepUp();

      double soleYaw = touchdownPose.getYaw();
      double lateralOffset = swingSide.negateIfLeftSide(halfStanceWidth);
      double offsetX = -lateralOffset * Math.sin(soleYaw);
      double offsetY = lateralOffset * Math.cos(soleYaw);
      collisionDetector.setBoxPose(touchdownPose.getX() + offsetX, touchdownPose.getY() + offsetY, touchdownPose.getZ() + heightOffset, soleYaw);

      return !collisionDetector.checkForCollision().isCollisionDetected();
   }

   private final ConvexPolygon2D footPolygon = new ConvexPolygon2D();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final PlanarRegion tempRegion = new PlanarRegion();

   private boolean isStepSnappable(FramePose3DReadOnly touchdownPose, FramePose3DReadOnly stancePose, RobotSide swingSide)
   {
      if (planarRegionsList.get() == null)
         return true;

      tempTransform.getTranslation().set(touchdownPose.getPosition().getX(), touchdownPose.getPosition().getY(), 0.0);
      tempTransform.getRotation().setToYawOrientation(touchdownPose.getYaw());

      footPolygon.set(footPolygons.get(swingSide));
      footPolygon.applyTransform(tempTransform, false);

      PlanarRegionsList planarRegionsList = this.planarRegionsList.get();

      return PlanarRegionsListPolygonSnapper.snapPolygonToPlanarRegionsList(footPolygon, planarRegionsList, Double.POSITIVE_INFINITY, tempRegion) != null;
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }
}
