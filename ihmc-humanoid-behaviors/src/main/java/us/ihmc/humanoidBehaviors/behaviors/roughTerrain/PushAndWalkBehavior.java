package us.ihmc.humanoidBehaviors.behaviors.roughTerrain;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
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
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

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
   
   private final YoDouble yawErrorThreshold = new YoDouble("YawErrorThreshold", registry);
   private final YoDouble yawErrorFilterAlpha = new YoDouble("YawErrorFilterAlpha", registry);
   private final AlphaFilteredYoVariable yawFilteredError = new AlphaFilteredYoVariable("YawFilteredError", registry, yawErrorFilterAlpha);
   private final YoDouble yawMaxAnglePerStep = new YoDouble("YawMaxAnglePerStep", registry);
   
   private final YoDouble[] footWorkSpaceVertex = new YoDouble[8];
   private final HumanoidReferenceFrames referenceFrames;
   private final WalkingControllerParameters walkingControllerParameters;
   private final FullHumanoidRobotModel fullRobotModel;

   public PushAndWalkBehavior(CommunicationBridgeInterface communicationBridge, HumanoidReferenceFrames referenceFrames,
		   FullHumanoidRobotModel fullRobotModel, WalkingControllerParameters walkingControllerParameters, YoGraphicsListRegistry graphicsListRegistry)
   {
      super(communicationBridge);
      this.referenceFrames = referenceFrames;
      this.walkingControllerParameters = walkingControllerParameters;
      this.fullRobotModel = fullRobotModel;
      
      statusQueue = new ConcurrentListeningQueue<>(1);
      walkingStatusQueue = new ConcurrentListeningQueue<>(1);
      attachNetworkListeningQueue(statusQueue, CapturabilityBasedStatus.class);
      attachNetworkListeningQueue(walkingStatusQueue, WalkingStatusMessage.class);
      
      
      footWorkSpaceVertex[0] = new YoDouble("FootWorkSpaceVertex1X", registry);
      footWorkSpaceVertex[1] = new YoDouble("FootWorkSpaceVertex1Y", registry);
      footWorkSpaceVertex[2] = new YoDouble("FootWorkSpaceVertex2X", registry);
      footWorkSpaceVertex[3] = new YoDouble("FootWorkSpaceVertex2Y", registry);
      footWorkSpaceVertex[4] = new YoDouble("FootWorkSpaceVertex3X", registry);
      footWorkSpaceVertex[5] = new YoDouble("FootWorkSpaceVertex3Y", registry);
      footWorkSpaceVertex[6] = new YoDouble("FootWorkSpaceVertex4X", registry);
      footWorkSpaceVertex[7] = new YoDouble("FootWorkSpaceVertex4Y", registry);
      
      footWorkSpaceVertex[0].set(0.25); footWorkSpaceVertex[1].set(0.18);
      footWorkSpaceVertex[2].set(0.15); footWorkSpaceVertex[3].set(0.35);
      footWorkSpaceVertex[4].set(-0.25); footWorkSpaceVertex[5].set(0.18);
      footWorkSpaceVertex[6].set(-0.15); footWorkSpaceVertex[7].set(0.35);
      
      errorThreshold.set(0.02);
      errorFilterAlpha.set(0.95);
      
      yawMaxAnglePerStep.set(Math.toRadians(10));
      yawErrorThreshold.set(Math.toRadians(2));
      yawErrorFilterAlpha.set(0.95);
      
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
         FramePoint2D desiredCapturePoint = latestPacket.getDesiredCapturePoint();
         FramePoint2D capturePoint = latestPacket.getCapturePoint();

         boolean doubleSupport = true;
         for (RobotSide robotSide : RobotSide.values)
         {
            doubleSupport &= !latestPacket.getFootSupportPolygon(robotSide).isEmpty();
         }

         double error = desiredCapturePoint.distance(capturePoint);
         filteredError.update(error);
         yawFilteredError.update(getSpineYawJointPositionError());
         
         boolean shouldWalk = filteredError.getDoubleValue() > errorThreshold.getDoubleValue();

         if (doubleSupport && shouldWalk && !walking.getBooleanValue())
         {
            Vector2D direction = new Vector2D();
            direction.sub(capturePoint, desiredCapturePoint);
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

   private double getSpineYawJointPositionError()
   {
	   return getSpineYawJointDesiredPosition() - getSpineYawJointCurrentPosition();
   }
   
   private double getSpineYawJointDesiredPosition()
   {
	   OneDoFJoint spineYaw = getSpineYawJoint();
	   return spineYaw.getqDesired();
   }
   
   private double getSpineYawJointCurrentPosition()
   {
	   OneDoFJoint spineYaw = getSpineYawJoint();
	   return spineYaw.getQ();
   }
   
   private OneDoFJoint getSpineYawJoint()
   {
	   SpineJointName[] spineJointName = fullRobotModel.getRobotSpecificJointNames().getSpineJointNames();
       int spineYawIndex = -1;
       for (int i = 0; i < spineJointName.length; i++) {
    	   if(spineJointName[i].getCamelCaseNameForStartOfExpression() == "spineYaw")
    	   {
    		   spineYawIndex = i;
    		   break;
    	   }
       }
       return fullRobotModel.getSpineJoint(spineJointName[spineYawIndex]);       
   }
   
   private void takeSteps(Vector2D direction2dInWorld)
   {
      referenceFrames.updateFrames();
      walking.set(true);

      FrameVector3D direction = new FrameVector3D(ReferenceFrame.getWorldFrame());
      direction.set(direction2dInWorld);
      RobotSide swingSide = findStepSide(direction);

      FramePoint3D location = computeSteppingLocation(direction, swingSide);

      MovingReferenceFrame stanceSoleFrame = referenceFrames.getSoleFrame(swingSide.getOppositeSide());
      FrameVector3D directionStanceFootFrame = new FrameVector3D(direction);
      directionStanceFootFrame.changeFrame(stanceSoleFrame);
      
      double yawAngleChange = Math.atan(directionStanceFootFrame.getY()/directionStanceFootFrame.getX());
      if(Math.abs(yawAngleChange) > yawMaxAnglePerStep.getDoubleValue())
    	  yawAngleChange = yawMaxAnglePerStep.getDoubleValue()*Math.signum(yawAngleChange);
      
      FrameQuaternion orientation = new FrameQuaternion(stanceSoleFrame, yawAngleChange, 0.0, 0.0);
      orientation.changeFrame(ReferenceFrame.getWorldFrame());

      FootstepDataListMessage footsteps = new FootstepDataListMessage();
      FootstepDataMessage footstep = new FootstepDataMessage(swingSide, location, orientation);
      footsteps.add(footstep);
      sendPacketToController(footsteps);
   }

   private RobotSide findStepSide(FrameVector3D direction)
   {
      double score = 0.0;
      RobotSide ret = null;

      // compare the two options:
      for (RobotSide stepSide : RobotSide.values)
      {
         FramePoint3D stepLocation = computeSteppingLocation(direction, stepSide);
         FramePoint3D stanceLocation = new FramePoint3D(referenceFrames.getSoleZUpFrame(stepSide.getOppositeSide()));
         
         stepLocation.changeFrame(ReferenceFrame.getWorldFrame());
         stanceLocation.changeFrame(ReferenceFrame.getWorldFrame());

         FramePoint3D midFeetPointAfterStep = new FramePoint3D();
         midFeetPointAfterStep.interpolate(stepLocation, stanceLocation, 0.5);

         MovingReferenceFrame midFeetFrame = referenceFrames.getMidFootZUpGroundFrame();
         FramePoint3D midFeetLocation = new FramePoint3D(midFeetFrame);
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

   private FramePoint3D computeSteppingLocation(FrameVector3D direction, RobotSide stepSide)
   {
      // reachable region in stance frame
      ConvexPolygon2D reachableRegion = new ConvexPolygon2D();
      reachableRegion.addVertex(footWorkSpaceVertex[0].getDoubleValue(), stepSide.negateIfRightSide(footWorkSpaceVertex[1].getDoubleValue()));
      reachableRegion.addVertex(footWorkSpaceVertex[2].getDoubleValue(), stepSide.negateIfRightSide(footWorkSpaceVertex[3].getDoubleValue()));
      reachableRegion.addVertex(footWorkSpaceVertex[4].getDoubleValue(), stepSide.negateIfRightSide(footWorkSpaceVertex[5].getDoubleValue()));
      reachableRegion.addVertex(footWorkSpaceVertex[6].getDoubleValue(), stepSide.negateIfRightSide(footWorkSpaceVertex[7].getDoubleValue()));
      reachableRegion.update();
      
      MovingReferenceFrame stanceSoleFrame = referenceFrames.getSoleZUpFrame(stepSide.getOppositeSide());
      //MovingReferenceFrame stanceSoleFrame = referenceFrames.getFootFrame(stepSide.getOppositeSide());
      FrameVector3D localDirection = new FrameVector3D(direction);
      localDirection.changeFrame(stanceSoleFrame);
      FramePoint3D stanceLocation = new FramePoint3D(stanceSoleFrame);
      FramePoint3D swingLocation = new FramePoint3D(referenceFrames.getFootFrame(stepSide));
      
      //System.out.println(swingLocation.toString());
      swingLocation.changeFrame(stanceSoleFrame);
      
      //Point2D desiredLocation = new Point2D(localDirection.getX(), localDirection.getY());
      //Point2D location2d = reachableRegion.orthogonalProjectionCopy(desiredLocation);
      
      //System.out.println(swingLocation.toString());      
      //System.out.println(localDirection.getX() + " " + localDirection.getY());
      //System.out.println(reachableRegion.toString());
      Line2D ray = new Line2D(swingLocation.getX(), swingLocation.getY(), localDirection.getX(), localDirection.getY());
      Point2D[] location2d = reachableRegion.intersectionWithRay(ray);
      
      int index = 0;
      if(location2d == null)
      {
    	  location2d = new Point2D[1];
    	  location2d[0] = new Point2D(swingLocation.getX(), swingLocation.getY());    	  
      }
      else if (location2d.length == 2)
      {
    	  Point2D swingLoc = new Point2D(swingLocation.getX(), swingLocation.getY());
    	  index = location2d[0].distance(swingLoc) > location2d[1].distance(swingLoc) ? 0:1;
      }
      
      FramePoint3D location = new FramePoint3D(stanceSoleFrame);
      location.set(location2d[index], 0.0);
      location.changeFrame(ReferenceFrame.getWorldFrame());

      return location;
   }

   @Override
   public void onBehaviorEntered()
   {

   }

   @Override
   public void onBehaviorAborted()
   {

   }

   @Override
   public void onBehaviorPaused()
   {

   }

   @Override
   public void onBehaviorResumed()
   {

   }

   @Override
   public void onBehaviorExited()
   {

   }

   @Override
   public boolean isDone()
   {
      return false;
   }
}
