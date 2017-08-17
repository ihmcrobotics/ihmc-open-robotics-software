package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class WalkingFailureDetectionControlModule
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final FrameConvexPolygon2d combinedFootPolygon = new FrameConvexPolygon2d();
   private final FrameConvexPolygon2d combinedFootPolygonWithNextFootstep = new FrameConvexPolygon2d();
   private final SideDependentList<FrameConvexPolygon2d> footPolygons = new SideDependentList<>();
   private final FrameConvexPolygon2d nextFootstepPolygon = new FrameConvexPolygon2d();
   private final SideDependentList<FrameConvexPolygon2d> footPolygonsInWorldFrame = new SideDependentList<>();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoBoolean isUsingNextFootstep;
   private final YoBoolean isFallDetectionActivated;
   private final YoDouble icpDistanceFromFootPolygon;
   private final YoDouble icpDistanceFromFootPolygonThreshold;
   private final YoBoolean isRobotFalling;
   private final FrameVector2D fallingDirection = new FrameVector2D();

   public WalkingFailureDetectionControlModule(SideDependentList<? extends ContactablePlaneBody> contactableFeet, YoVariableRegistry parentRegistry)
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

      isUsingNextFootstep = new YoBoolean("isFallDetectionUsingNextFootstep", registry);
      isUsingNextFootstep.set(false);

      updateCombinedPolygon();

      isFallDetectionActivated = new YoBoolean("isFallDetectionActivated", registry);
      isFallDetectionActivated.set(true);

      icpDistanceFromFootPolygonThreshold = new YoDouble("icpDistanceFromFootPolygonThreshold", registry);
      icpDistanceFromFootPolygonThreshold.set(0.05);
      icpDistanceFromFootPolygon = new YoDouble("icpDistanceFromFootPolygon", registry);
      isRobotFalling = new YoBoolean("isRobotFalling", registry);

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
         ConvexPolygon2D footPolygon = footPolygons.get(nextFootstep.getRobotSide()).getConvexPolygon2d();
         nextFootstepPolygon.setIncludingFrameAndUpdate(footstepSoleFrame, footPolygon);
         nextFootstepPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      }
   }

   private final FrameVector2D tempFallingDirection = new FrameVector2D();

   public void checkIfRobotIsFalling(FramePoint2D capturePoint2d, FramePoint2D desiredCapturePoint2d)
   {
      updateCombinedPolygon();

      if (isUsingNextFootstep.getBooleanValue())
         icpDistanceFromFootPolygon.set(combinedFootPolygonWithNextFootstep.distance(capturePoint2d));
      else
         icpDistanceFromFootPolygon.set(combinedFootPolygon.distance(capturePoint2d));
      // TODO need to investigate this method, seems to be buggy
      //      boolean isCapturePointCloseToFootPolygon = combinedFootPolygon.isPointInside(capturePoint, icpDistanceFromFootPolygonThreshold.getDoubleValue());
      boolean isCapturePointCloseToFootPolygon = icpDistanceFromFootPolygon.getDoubleValue() < icpDistanceFromFootPolygonThreshold.getDoubleValue();
      boolean isCapturePointCloseToDesiredCapturePoint = desiredCapturePoint2d.distance(capturePoint2d) < icpDistanceFromFootPolygonThreshold.getDoubleValue();
      isRobotFalling.set(!isCapturePointCloseToFootPolygon && !isCapturePointCloseToDesiredCapturePoint);

      if (isRobotFalling.getBooleanValue())
      {
         tempFallingDirection.set(capturePoint2d);
         FramePoint2D footCenter = combinedFootPolygon.getCentroid();
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

   public FrameVector2D getFallingDirection()
   {
      return fallingDirection;
   }

   public FrameConvexPolygon2d getCombinedFootPolygon()
   {
      return combinedFootPolygon;
   }
}
