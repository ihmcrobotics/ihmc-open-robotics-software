package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex2DSupplier;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class WalkingFailureDetectionControlModule
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final FrameConvexPolygon2D combinedFootPolygon = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D combinedFootPolygonWithNextFootstep = new FrameConvexPolygon2D();
   private final SideDependentList<FrameConvexPolygon2D> footPolygons = new SideDependentList<>();
   private final FrameConvexPolygon2D nextFootstepPolygon = new FrameConvexPolygon2D();
   private final SideDependentList<FrameConvexPolygon2D> footPolygonsInWorldFrame = new SideDependentList<>();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoBoolean isUsingNextFootstep;
   private final YoBoolean isFallDetectionActivated;
   private final YoDouble icpDistanceFromFootPolygon;
   private final YoDouble icpDistanceFromFootPolygonThreshold;
   private final YoBoolean isRobotFalling;
   private final FrameVector2D fallingDirection2D = new FrameVector2D();
   private final FrameVector3D fallingDirection3D = new FrameVector3D();

   public WalkingFailureDetectionControlModule(SideDependentList<? extends ContactablePlaneBody> contactableFeet, YoRegistry parentRegistry)
   {

      for (RobotSide robotSide : RobotSide.values)
      {
         FrameConvexPolygon2D footPolygonInFootFrame = new FrameConvexPolygon2D(FrameVertex2DSupplier.asFrameVertex2DSupplier(contactableFeet.get(robotSide).getContactPoints2d()));
         footPolygons.put(robotSide, footPolygonInFootFrame);

         FrameConvexPolygon2D footPolygonInWorldFrame = new FrameConvexPolygon2D();
         footPolygonsInWorldFrame.put(robotSide, footPolygonInWorldFrame);
      }

      // Just for allocating memory for the nextFootstepPolygon
      nextFootstepPolygon.setIncludingFrame(footPolygons.get(RobotSide.LEFT));

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
         FrameConvexPolygon2D footPolygonInWorldFrame = footPolygonsInWorldFrame.get(robotSide);
         footPolygonInWorldFrame.setIncludingFrame(footPolygons.get(robotSide));
         footPolygonInWorldFrame.changeFrameAndProjectToXYPlane(worldFrame);
      }

      combinedFootPolygon.setIncludingFrame(footPolygonsInWorldFrame.get(RobotSide.LEFT), footPolygonsInWorldFrame.get(RobotSide.RIGHT));

      // If there is a nextFootstep, increase the polygon to include it.
      if (isUsingNextFootstep.getBooleanValue())
         combinedFootPolygonWithNextFootstep.setIncludingFrame(combinedFootPolygon, nextFootstepPolygon);
   }

   public void setNextFootstep(Footstep nextFootstep)
   {
      isUsingNextFootstep.set(nextFootstep != null);

      if (isUsingNextFootstep.getBooleanValue())
      {
         ReferenceFrame footstepSoleFrame = nextFootstep.getSoleReferenceFrame();
         nextFootstepPolygon.setIncludingFrame(footstepSoleFrame, footPolygons.get(nextFootstep.getRobotSide()));
         nextFootstepPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      }
   }

   private final FrameVector2D tempFallingDirection = new FrameVector2D();

   public void checkIfRobotIsFalling(FramePoint2DReadOnly capturePoint2d, FramePoint2DReadOnly desiredCapturePoint2d)
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
         FramePoint2DReadOnly footCenter = combinedFootPolygon.getCentroid();
         tempFallingDirection.changeFrame(ReferenceFrame.getWorldFrame());
         tempFallingDirection.sub(footCenter);
         fallingDirection2D.set(tempFallingDirection);
         fallingDirection3D.setIncludingFrame(fallingDirection2D, 0.0);
      }
   }

   public boolean isRobotFalling()
   {
      if (!isFallDetectionActivated.getBooleanValue())
         return false;

      return isRobotFalling.getBooleanValue();
   }

   public FrameVector2D getFallingDirection2D()
   {
      return fallingDirection2D;
   }

   public FrameVector3D getFallingDirection3D()
   {
      return fallingDirection3D;
   }

   public FrameConvexPolygon2D getCombinedFootPolygon()
   {
      return combinedFootPolygon;
   }

   public FrameConvexPolygon2DReadOnly getCombinedFootPolygonWithNextFootstep()
   {
      return combinedFootPolygonWithNextFootstep;
   }
}
