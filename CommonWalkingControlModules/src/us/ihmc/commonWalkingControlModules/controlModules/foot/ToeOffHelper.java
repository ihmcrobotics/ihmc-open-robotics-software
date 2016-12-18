package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.List;

public class ToeOffHelper
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final SideDependentList<List<YoContactPoint>> contactPoints = new SideDependentList<>();

   private final FramePoint2d toeOffContactPoint2d = new FramePoint2d();
   private final FramePoint exitCMP = new FramePoint();
   private final FramePoint2d exitCMP2d = new FramePoint2d();
   private final FrameVector2d exitCMPRayDirection2d = new FrameVector2d();
   private final FrameLine2d rayThroughExitCMP = new FrameLine2d();

   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();
   private final FrameConvexPolygon2d footPolygon = new FrameConvexPolygon2d();

   private final DoubleYoVariable toeOffContactInterpolation;
   private final BooleanYoVariable hasComputedToeOffContactPoint;

   public ToeOffHelper(SideDependentList<YoPlaneContactState> contactStates, SideDependentList<? extends ContactablePlaneBody> feet,
                       WalkingControllerParameters walkingControllerParameters, YoVariableRegistry parentRegistry)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ReferenceFrame soleFrame = feet.get(robotSide).getSoleFrame();
         soleFrames.put(robotSide, soleFrame);

         contactPoints.put(robotSide, contactStates.get(robotSide).getContactPoints());
      }

      toeOffContactInterpolation = new DoubleYoVariable("toeOffContactInterpolation", registry);
      toeOffContactInterpolation.set(walkingControllerParameters.getToeOffContactInterpolation());

      hasComputedToeOffContactPoint = new BooleanYoVariable("hasComputedToeOffContactPoint", registry);

      parentRegistry.addChild(registry);
   }

   public void clear()
   {
      exitCMP2d.setToNaN();
      hasComputedToeOffContactPoint.set(false);
   }

   public void setExitCMP(FramePoint exitCMP, RobotSide trailingLeg)
   {
      ReferenceFrame soleFrame = soleFrames.get(trailingLeg);
      this.exitCMP.setIncludingFrame(exitCMP);
      this.exitCMP.changeFrame(soleFrame);
      exitCMP2d.setToZero(soleFrame);
      exitCMP2d.setByProjectionOntoXYPlaneIncludingFrame(this.exitCMP);
   }

   private final FramePoint2d interpolatedRayOrigin = new FramePoint2d();
   public void computeToeOffContactPoint(FramePoint2d desiredCMP, RobotSide trailingLeg)
   {
      ReferenceFrame soleFrame = soleFrames.get(trailingLeg);

      footPolygon.clear(soleFrame);

      for (int i = 0; i < contactPoints.get(trailingLeg).size(); i++)
      {
         contactPoints.get(trailingLeg).get(i).getPosition2d(toeOffContactPoint2d);
         footPolygon.addVertex(toeOffContactPoint2d);
      }

      footPolygon.update();

      FramePoint2d rayOrigin;
      if (!exitCMP2d.containsNaN() && footPolygon.isPointInside(exitCMP2d))
         rayOrigin = exitCMP2d;
      else
         rayOrigin = footPolygon.getCentroid();

      exitCMPRayDirection2d.setIncludingFrame(soleFrame, 1.0, 0.0);
      rayThroughExitCMP.setToZero(soleFrame);

      if (desiredCMP != null && !desiredCMP.containsNaN())
      {
         interpolatedRayOrigin.setToZero(soleFrame);
         desiredCMP.changeFrameAndProjectToXYPlane(soleFrame);
         interpolatedRayOrigin.interpolate(rayOrigin, desiredCMP, toeOffContactInterpolation.getDoubleValue());

         if (footPolygon.isPointInside(interpolatedRayOrigin))
            rayThroughExitCMP.set(interpolatedRayOrigin, exitCMPRayDirection2d);
         else
            rayThroughExitCMP.set(rayOrigin, exitCMPRayDirection2d);
      }
      else
      {
         rayThroughExitCMP.set(rayOrigin, exitCMPRayDirection2d);
      }

      FramePoint2d[] intersectionWithRay = footPolygon.intersectionWithRayCopy(rayThroughExitCMP);
      toeOffContactPoint2d.set(intersectionWithRay[0]);

      hasComputedToeOffContactPoint.set(true);
   }

   public void getToeOffContactPoint(FramePoint2d contactPointToPack, RobotSide trailingLeg)
   {
      if (!hasComputedToeOffContactPoint.getBooleanValue())
         computeToeOffContactPoint(null, trailingLeg);

      contactPointToPack.set(toeOffContactPoint2d);
   }
}
