package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import java.util.List;

public class ToeOffHelper
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   protected final HighLevelHumanoidControllerToolbox momentumBasedController;

   private final YoPlaneContactState contactState;
   private final List<YoContactPoint> contactPoints;

   private final FramePoint2d toeOffContactPoint2d = new FramePoint2d();
   private final FramePoint exitCMP = new FramePoint();
   private final FramePoint2d exitCMP2d = new FramePoint2d();
   private final FrameVector2d exitCMPRayDirection2d = new FrameVector2d();
   private final FrameLine2d rayThroughExitCMP = new FrameLine2d();

   private final ReferenceFrame soleFrame;
   private final FrameConvexPolygon2d footPolygon = new FrameConvexPolygon2d();

   private final DoubleYoVariable toeOffContactInterpolation;
   private final BooleanYoVariable hasComputedToeOffContactPoint;

   public ToeOffHelper(FootControlHelper footControlHelper, YoVariableRegistry parentRegistry)
   {
      momentumBasedController = footControlHelper.getMomentumBasedController();

      ContactableFoot contactableFoot = footControlHelper.getContactableFoot();
      soleFrame = contactableFoot.getSoleFrame();

      String namePrefix = contactableFoot.getName();

      contactState = momentumBasedController.getContactState(contactableFoot);
      contactPoints = contactState.getContactPoints();

      exitCMP2d.setToNaN(soleFrame);
      exitCMPRayDirection2d.setIncludingFrame(soleFrame, 1.0, 0.0);
      rayThroughExitCMP.setToNaN(soleFrame);

      toeOffContactInterpolation = new DoubleYoVariable(namePrefix + "ToeOffContactInterpolation", registry);
      toeOffContactInterpolation.set(footControlHelper.getWalkingControllerParameters().getToeOffContactInterpolation());

      hasComputedToeOffContactPoint = new BooleanYoVariable(namePrefix + "HasComputedToeOffContactPoint", registry);

      parentRegistry.addChild(registry);
   }

   public void clear()
   {
      exitCMP2d.setToNaN();
      hasComputedToeOffContactPoint.set(false);
   }

   public void setExitCMP(FramePoint exitCMP)
   {
      this.exitCMP.setIncludingFrame(exitCMP);
      this.exitCMP.changeFrame(soleFrame);
      exitCMP2d.setByProjectionOntoXYPlaneIncludingFrame(this.exitCMP);
   }

   private final FramePoint2d interpolatedRayOrigin = new FramePoint2d();
   public void computeToeOffContactPoint(FramePoint2d desiredCMP)
   {
      footPolygon.clear(soleFrame);

      for (int i = 0; i < contactPoints.size(); i++)
      {
         contactPoints.get(i).getPosition2d(toeOffContactPoint2d);
         footPolygon.addVertex(toeOffContactPoint2d);
      }

      footPolygon.update();

      FramePoint2d rayOrigin;
      if (!exitCMP2d.containsNaN() && footPolygon.isPointInside(exitCMP2d))
         rayOrigin = exitCMP2d;
      else
         rayOrigin = footPolygon.getCentroid();

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

   public boolean hasComputedToeOffContactPoint()
   {
      return hasComputedToeOffContactPoint.getBooleanValue();
   }

   public void getToeOffContactPoint(FramePoint2d contactPointToPack)
   {
      if (hasComputedToeOffContactPoint())
         computeToeOffContactPoint(null);

      contactPointToPack.set(toeOffContactPoint2d);
   }
}
