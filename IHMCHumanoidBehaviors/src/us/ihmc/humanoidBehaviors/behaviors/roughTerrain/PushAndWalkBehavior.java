package us.ihmc.humanoidBehaviors.behaviors.roughTerrain;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatusMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatusMessage.Status;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;

public class PushAndWalkBehavior extends AbstractBehavior
{
   private final ConcurrentListeningQueue<CapturabilityBasedStatus> statusQueue;
   private final ConcurrentListeningQueue<WalkingStatusMessage> walkingStatusQueue;

   private final YoFramePoint2d yoDesiredCapturePoint = new YoFramePoint2d("DesiredICP", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint2d yoCapturePoint = new YoFramePoint2d("ICP", ReferenceFrame.getWorldFrame(), registry);

   private final YoGraphicPosition desiredCapturePointViz;
   private final YoGraphicPosition capturePointViz;

   private final YoBoolean walking = new YoBoolean("Walking", registry);
   private final YoDouble errorThreshold = new YoDouble("ErrorThreshold", registry);
   private final YoDouble errorFilterAlpha = new YoDouble("ErrorFilterAlpha", registry);
   private final AlphaFilteredYoVariable filteredError = new AlphaFilteredYoVariable("FilteredError", registry, errorFilterAlpha);

   private final HumanoidReferenceFrames referenceFrames;
   private final WalkingControllerParameters walkingControllerParameters;

   public PushAndWalkBehavior(CommunicationBridgeInterface communicationBridge, HumanoidReferenceFrames referenceFrames,
                              WalkingControllerParameters walkingControllerParameters, YoGraphicsListRegistry graphicsListRegistry)
   {
      super(communicationBridge);
      this.referenceFrames = referenceFrames;
      this.walkingControllerParameters = walkingControllerParameters;

      statusQueue = new ConcurrentListeningQueue<>(1);
      walkingStatusQueue = new ConcurrentListeningQueue<>(1);
      attachNetworkListeningQueue(statusQueue, CapturabilityBasedStatus.class);
      attachNetworkListeningQueue(walkingStatusQueue, WalkingStatusMessage.class);

      errorThreshold.set(0.02);
      errorFilterAlpha.set(0.95);

      if (graphicsListRegistry != null)
      {
         desiredCapturePointViz = new YoGraphicPosition("DesiredICP", yoDesiredCapturePoint, 0.05, YoAppearance.Yellow());
         capturePointViz = new YoGraphicPosition("ICP", yoCapturePoint, 0.05, YoAppearance.Blue());
         graphicsListRegistry.registerArtifact(getName(), desiredCapturePointViz.createArtifact());
         graphicsListRegistry.registerArtifact(getName(), capturePointViz.createArtifact());
      }
      else
      {
         desiredCapturePointViz = null;
         capturePointViz = null;
      }
   }

   @Override
   public void doControl()
   {
      if (walkingStatusQueue.isNewPacketAvailable())
      {
         WalkingStatusMessage latestPacket = walkingStatusQueue.getLatestPacket();
         Status walkingStatus = latestPacket.getWalkingStatus();
         walking.set(walkingStatus != Status.COMPLETED);
         walkingStatusQueue.clear();
      }

      if (statusQueue.isNewPacketAvailable())
      {
         CapturabilityBasedStatus latestPacket = statusQueue.getLatestPacket();
         FramePoint2d desiredCapturePoint = latestPacket.getDesiredCapturePoint();
         FramePoint2d capturePoint = latestPacket.getCapturePoint();

         boolean doubleSupport = true;
         for (RobotSide robotSide : RobotSide.values)
         {
            doubleSupport &= !latestPacket.getFootSupportPolygon(robotSide).isEmpty();
         }

         double error = desiredCapturePoint.distance(capturePoint);
         filteredError.update(error);
         boolean shouldWalk = filteredError.getDoubleValue() > errorThreshold.getDoubleValue();

         if (doubleSupport && shouldWalk && !walking.getBooleanValue())
         {
            Vector2D direction = new Vector2D();
            direction.sub(capturePoint.getPoint(), desiredCapturePoint.getPoint());
            direction.normalize();
            takeSteps(direction);
         }

         yoDesiredCapturePoint.setAndMatchFrame(desiredCapturePoint);
         yoCapturePoint.setAndMatchFrame(capturePoint);
         if (desiredCapturePointViz != null)
         {
            desiredCapturePointViz.update();
            capturePointViz.update();
         }
         statusQueue.clear();
      }
   }

   private void takeSteps(Vector2D direction2dInWorld)
   {
      referenceFrames.updateFrames();
      walking.set(true);

      FrameVector direction = new FrameVector(ReferenceFrame.getWorldFrame());
      direction.setXY(direction2dInWorld);
      RobotSide swingSide = findStepSide(direction);

      FramePoint location = computeSteppingLocation(direction, swingSide);

      MovingReferenceFrame stanceSoleFrame = referenceFrames.getSoleFrame(swingSide.getOppositeSide());
      FrameOrientation orientation = new FrameOrientation(stanceSoleFrame);
      orientation.changeFrame(ReferenceFrame.getWorldFrame());

      FootstepDataListMessage footsteps = new FootstepDataListMessage();
      FootstepDataMessage footstep = new FootstepDataMessage(swingSide, location.getPoint(), orientation.getQuaternion());
      footsteps.add(footstep);
      sendPacketToController(footsteps);
   }

   private RobotSide findStepSide(FrameVector direction)
   {
      double score = 0.0;
      RobotSide ret = null;

      // compare the two options:
      for (RobotSide stepSide : RobotSide.values)
      {
         FramePoint stepLocation = computeSteppingLocation(direction, stepSide);
         FramePoint stanceLocation = new FramePoint(referenceFrames.getSoleZUpFrame(stepSide.getOppositeSide()));

         stepLocation.changeFrame(ReferenceFrame.getWorldFrame());
         stanceLocation.changeFrame(ReferenceFrame.getWorldFrame());

         FramePoint midFeetPointAfterStep = new FramePoint();
         midFeetPointAfterStep.interpolate(stepLocation, stanceLocation, 0.5);

         MovingReferenceFrame midFeetFrame = referenceFrames.getMidFootZUpGroundFrame();
         FramePoint midFeetLocation = new FramePoint(midFeetFrame);
         midFeetLocation.changeFrame(ReferenceFrame.getWorldFrame());

         double progress = midFeetLocation.distance(midFeetPointAfterStep);

         if (progress > score)
         {
            score = progress;
            ret = stepSide;
         }
      }

      return ret;
   }

   private FramePoint computeSteppingLocation(FrameVector direction, RobotSide stepSide)
   {
      // reachable region in stance frame
      ConvexPolygon2D reachableRegion = new ConvexPolygon2D();
      reachableRegion.addVertex(0.25, stepSide.negateIfRightSide(0.18));
      reachableRegion.addVertex(0.15, stepSide.negateIfRightSide(0.35));
      reachableRegion.addVertex(-0.25, stepSide.negateIfRightSide(0.18));
      reachableRegion.addVertex(-0.15, stepSide.negateIfRightSide(0.35));
      reachableRegion.update();

      MovingReferenceFrame stanceSoleFrame = referenceFrames.getSoleZUpFrame(stepSide.getOppositeSide());
      FrameVector localDirection = new FrameVector(direction);
      localDirection.changeFrame(stanceSoleFrame);

      Point2D desiredLocation = new Point2D(localDirection.getX(), localDirection.getY());
      Point2D location2d = reachableRegion.orthogonalProjectionCopy(desiredLocation);

      FramePoint location = new FramePoint(stanceSoleFrame);
      location.setXY(location2d);
      location.changeFrame(ReferenceFrame.getWorldFrame());

      return location;
   }

   @Override
   public void onBehaviorEntered()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void onBehaviorAborted()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void onBehaviorPaused()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void onBehaviorResumed()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void onBehaviorExited()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public boolean isDone()
   {
      // TODO Auto-generated method stub
      return false;
   }

}
