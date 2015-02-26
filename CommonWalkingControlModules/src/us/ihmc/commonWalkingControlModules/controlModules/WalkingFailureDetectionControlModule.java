package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.utilities.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFramePoint2d;

public class WalkingFailureDetectionControlModule
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final FrameConvexPolygon2d combinedFootPolygon = new FrameConvexPolygon2d();
   private final SideDependentList<FrameConvexPolygon2d> footPolygons = new SideDependentList<>();
   private final SideDependentList<FrameConvexPolygon2d> footPolygonsInWorldFrame = new SideDependentList<>();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanYoVariable isFallDetectionActivated;
   private final DoubleYoVariable icpDistanceFromFootPolygon;
   private final DoubleYoVariable icpDistanceFromFootPolygonThreshold;
   private final BooleanYoVariable isRobotFalling;

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
   }

   public void checkIfRobotIsFalling(YoFramePoint currentCapturePoint, YoFramePoint2d desiredCapturePoint)
   {
      updateCombinedPolygon();

      currentCapturePoint.getFrameTuple2dIncludingFrame(capturePoint);
      icpDistanceFromFootPolygon.set(combinedFootPolygon.distance(capturePoint));
      // TODO need to investigate this method, seems to be buggy
      //      boolean isCapturePointCloseToFootPolygon = combinedFootPolygon.isPointInside(capturePoint, icpDistanceFromFootPolygonThreshold.getDoubleValue());
      boolean isCapturePointCloseToFootPolygon = icpDistanceFromFootPolygon.getDoubleValue() < icpDistanceFromFootPolygonThreshold.getDoubleValue();
      boolean isCapturePointCloseToDesiredCapturePoint = desiredCapturePoint.distance(capturePoint) < icpDistanceFromFootPolygonThreshold.getDoubleValue();
      isRobotFalling.set(!isCapturePointCloseToFootPolygon && !isCapturePointCloseToDesiredCapturePoint);
   }

   public boolean isRobotFalling()
   {
      if (!isFallDetectionActivated.getBooleanValue())
         return false;

      return isRobotFalling.getBooleanValue();
   }
}
