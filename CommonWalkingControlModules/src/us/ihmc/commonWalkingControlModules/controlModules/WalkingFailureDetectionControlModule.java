package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.humanoidRobot.footstep.Footstep;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFramePoint2d;

public class WalkingFailureDetectionControlModule
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final FrameConvexPolygon2d combinedFootPolygon = new FrameConvexPolygon2d();
   private final FrameConvexPolygon2d combinedFootPolygonWithNextFootstep = new FrameConvexPolygon2d();
   private final SideDependentList<FrameConvexPolygon2d> footPolygons = new SideDependentList<>();
   private final FrameConvexPolygon2d nextFootstepPolygon = new FrameConvexPolygon2d();
   private final SideDependentList<FrameConvexPolygon2d> footPolygonsInWorldFrame = new SideDependentList<>();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanYoVariable isUsingNextFootstep;
   private final BooleanYoVariable isFallDetectionActivated;
   private final DoubleYoVariable icpDistanceFromFootPolygon;
   private final DoubleYoVariable icpDistanceFromFootPolygonThreshold;
   private final BooleanYoVariable isRobotFalling;
   private final FrameVector2d fallingDirection = new FrameVector2d();

   private final FramePoint2d capturePoint = new FramePoint2d();

   public WalkingFailureDetectionControlModule(SideDependentList<ContactablePlaneBody> contactableFeet, YoVariableRegistry parentRegistry)
   {

      for (RobotSide robotSide : RobotSide.values)
      {
         FrameConvexPolygon2d footPolygonInFootFrame = new FrameConvexPolygon2d(contactableFeet.get(robotSide).getContactPoints2d());
         footPolygons.put(robotSide, footPolygonInFootFrame);

         FrameConvexPolygon2d footPolygonInWorldFrame = new FrameConvexPolygon2d();
         footPolygonsInWorldFrame.put(robotSide, footPolygonInWorldFrame);
      }

      // Just for allocating memory for the nextFootstepPolygon
      nextFootstepPolygon.setIncludingFrameAndUpdate(footPolygons.get(RobotSide.LEFT));

      isUsingNextFootstep = new BooleanYoVariable("isFallDetectionUsingNextFootstep", registry);
      isUsingNextFootstep.set(false);

      updateCombinedPolygon();

      isFallDetectionActivated = new BooleanYoVariable("isFallDetectionActivated", registry);
      isFallDetectionActivated.set(true);

      icpDistanceFromFootPolygonThreshold = new DoubleYoVariable("icpDistanceFromFootPolygonThreshold", registry);
      icpDistanceFromFootPolygonThreshold.set(0.05);
      icpDistanceFromFootPolygon = new DoubleYoVariable("icpDistanceFromFootPolygon", registry);
      isRobotFalling = new BooleanYoVariable("isRobotFalling", registry);

      parentRegistry.addChild(registry);
   }

   private void updateCombinedPolygon()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         FrameConvexPolygon2d footPolygonInWorldFrame = footPolygonsInWorldFrame.get(robotSide);
         footPolygonInWorldFrame.setIncludingFrameAndUpdate(footPolygons.get(robotSide));
         footPolygonInWorldFrame.changeFrameAndProjectToXYPlane(worldFrame);
      }

      combinedFootPolygon.setIncludingFrameAndUpdate(footPolygonsInWorldFrame.get(RobotSide.LEFT), footPolygonsInWorldFrame.get(RobotSide.RIGHT));

      // If there is a nextFootstep, increase the polygon to include it.
      if (isUsingNextFootstep.getBooleanValue())
         combinedFootPolygonWithNextFootstep.setIncludingFrameAndUpdate(combinedFootPolygon, nextFootstepPolygon);
   }

   public void setNextFootstep(Footstep nextFootstep)
   {
      isUsingNextFootstep.set(nextFootstep != null);

      if (isUsingNextFootstep.getBooleanValue())
      {
         ReferenceFrame footstepSoleFrame = nextFootstep.getSoleReferenceFrame();
         ConvexPolygon2d footPolygon = footPolygons.get(nextFootstep.getRobotSide()).getConvexPolygon2d();
         nextFootstepPolygon.setIncludingFrameAndUpdate(footstepSoleFrame, footPolygon);
         nextFootstepPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      }
   }

   private final FrameVector2d tempFallingDirection = new FrameVector2d();
   public void checkIfRobotIsFalling(YoFramePoint currentCapturePoint, YoFramePoint2d desiredCapturePoint)
   {
      updateCombinedPolygon();

      currentCapturePoint.getFrameTuple2dIncludingFrame(capturePoint);
      if (isUsingNextFootstep.getBooleanValue())
         icpDistanceFromFootPolygon.set(combinedFootPolygonWithNextFootstep.distance(capturePoint));
      else
         icpDistanceFromFootPolygon.set(combinedFootPolygon.distance(capturePoint));
      // TODO need to investigate this method, seems to be buggy
      //      boolean isCapturePointCloseToFootPolygon = combinedFootPolygon.isPointInside(capturePoint, icpDistanceFromFootPolygonThreshold.getDoubleValue());
      boolean isCapturePointCloseToFootPolygon = icpDistanceFromFootPolygon.getDoubleValue() < icpDistanceFromFootPolygonThreshold.getDoubleValue();
      boolean isCapturePointCloseToDesiredCapturePoint = desiredCapturePoint.distance(capturePoint) < icpDistanceFromFootPolygonThreshold.getDoubleValue();
      isRobotFalling.set(!isCapturePointCloseToFootPolygon && !isCapturePointCloseToDesiredCapturePoint);

      if (isRobotFalling.getBooleanValue()){
         tempFallingDirection.set(capturePoint);
         FramePoint2d footCenter = combinedFootPolygon.getCentroid();
         tempFallingDirection.changeFrame(ReferenceFrame.getWorldFrame());
         footCenter.changeFrame(ReferenceFrame.getWorldFrame());
         tempFallingDirection.sub(footCenter);
         fallingDirection.set(tempFallingDirection);
      }
   }

   public boolean isRobotFalling()
   {
      if (!isFallDetectionActivated.getBooleanValue())
         return false;

      return isRobotFalling.getBooleanValue();
   }

   public FrameVector2d getFallingDirection(){
      return fallingDirection;
   }

   public FrameConvexPolygon2d getCombinedFootPolygon()
   {
      return combinedFootPolygon;
   }
}
