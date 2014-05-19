package us.ihmc.commonWalkingControlModules.captureRegion;

import java.util.List;

import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.ground.steppingStones.SteppingStones;

/**
 * Provides the function adjustFootstep which takes a footstep and a capture region
 * as arguments and projects the footstep into this region.
 * 
 * It is possible to provide stepping stones in this case the footstep will be projected
 * into the intersection between allowable region and stepping stones.
 * 
 * @author 
 *
 */
public class FootstepAdjustor
{
   private static final boolean VISUALIZE = true;
   private static final double SHRINK_TOUCHDOWN_POLYGON_FACTOR = 0.5;

   private final YoVariableRegistry registry = new YoVariableRegistry("FootstepAdjustor");

   private FootstepAdjusterVisualizer footstepAdjusterVisualizer = null;
   private SteppingStones steppingStones = null;

   public FootstepAdjustor(YoVariableRegistry parentRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      parentRegistry.addChild(registry);
      if (dynamicGraphicObjectsListRegistry != null && VISUALIZE)
      {
         footstepAdjusterVisualizer = new FootstepAdjusterVisualizer(this, dynamicGraphicObjectsListRegistry, registry);
      }
   }

   private final FrameConvexPolygon2d touchdownFootPolygon = new FrameConvexPolygon2d();
   private final FrameConvexPolygon2d desiredSteppingRegion = new FrameConvexPolygon2d();
   private final FrameConvexPolygon2d intersection = new FrameConvexPolygon2d();

   /**
    * This function takes a footstep and a captureRegion and if necessary projects the footstep
    * into the capture region. Returns true if the footstep was changed.
    */
   public boolean adjustFootstep(Footstep footstep, FrameConvexPolygon2d captureRegion)
   {
      boolean footstepChanged = false;

      // Check if there is a capture region
      if (captureRegion.isEmpty())
      {
         updateVisualizer();
         return footstepChanged;
      }

      // If there are stepping stones then intersect them with the capture region.
      if (steppingStones == null)
      {
         desiredSteppingRegion.setIncludingFrameAndUpdate(captureRegion);
      }
      else
      {
         updateDesiredSteppingRegion(captureRegion, steppingStones, footstep, desiredSteppingRegion);
      }

      // Check if the desired footstep intersects the capture region.
      calculateTouchdownFootPolygon(footstep, desiredSteppingRegion.getReferenceFrame(), touchdownFootPolygon);
      boolean nextStepInside = desiredSteppingRegion.intersectionWith(touchdownFootPolygon, intersection);

      if (nextStepInside)
      {
         updateVisualizer();
         return footstepChanged;
      }
      else
      {
         footstepChanged = true;
      }

      // No overlap between touch-down polygon and capture region.
      projectFootstepInCaptureRegion(footstep, desiredSteppingRegion);
      updateVisualizer();
      return footstepChanged;
   }

   private final FramePoint2d nextStep2d = new FramePoint2d();
   private final FrameConvexPolygon2d adjustedTouchdownFootPolygon = new FrameConvexPolygon2d();

   /** 
    * This function projects the footstep midpoint in the capture region.
    * Might be a bit conservative it should be sufficient to slightly overlap the capture region
    * and the touch-down polygon.
    */
   private void projectFootstepInCaptureRegion(Footstep footstep, FrameConvexPolygon2d captureRegion)
   {
      // new position is projection to the border of the capture region
//      footstep.getPosition().getFramePoint2d(nextStep2d);
//      nextStep2d.changeFrame(captureRegion.getReferenceFrame());
//      captureRegion.orthogonalProjection(nextStep2d);
//      nextStep2d.changeFrame(footstep.getReferenceFrame());
//      footstep.setPositionChangeOnlyXY(nextStep2d);

      // move the position of the footstep to the capture region centroid
      nextStep2d.setIncludingFrame(captureRegion.getCentroid());
      nextStep2d.changeFrame(footstep.getReferenceFrame());
      footstep.setPositionChangeOnlyXY(nextStep2d);

      calculateTouchdownFootPolygon(footstep, captureRegion.getReferenceFrame(), adjustedTouchdownFootPolygon);
   }

   private final FramePoint2d centroid = new FramePoint2d();

   /**
    * This function takes a footstep and calculates the touch-down polygon in the
    * desired reference frame
    */
   private void calculateTouchdownFootPolygon(Footstep footstep, ReferenceFrame desiredFrame, FrameConvexPolygon2d polygonToPack)
   {
      FramePoint centroid3d = footstep.getPosition();
      centroid.setIncludingFrame(centroid3d.getReferenceFrame(), centroid3d.getX(), centroid3d.getY());
      centroid.changeFrame(desiredFrame);

      List<FramePoint> expectedContactPoints = footstep.getExpectedContactPoints();
      polygonToPack.setIncludingFrameByProjectionOntoXYPlaneAndUpdate(desiredFrame, expectedContactPoints);
      // shrink the polygon for safety by pulling all the corner points towards the center
      polygonToPack.scale(centroid, SHRINK_TOUCHDOWN_POLYGON_FACTOR);
   }

   /**
    * This function intersects the capture region with the stepping stones and returns the best
    * intersecting polygon. Here 'best' means the one closest to the original footstep location.
    */
   private void updateDesiredSteppingRegion(FrameConvexPolygon2d captureRegion, SteppingStones steppingStones, Footstep footstep,
         FrameConvexPolygon2d polygonToPack)
   {
      // TODO
      //      polygonToPack.setIncludingFrameAndUpdate(captureRegion);
      throw new RuntimeException("Implement me!");
   }

   public void setSteppingStones(SteppingStones steppingStones)
   {
      this.steppingStones = steppingStones;
   }

   public FrameConvexPolygon2d getTouchdownFootPolygon()
   {
      return touchdownFootPolygon;
   }

   public FrameConvexPolygon2d getAdjustedFootPolygon()
   {
      return adjustedTouchdownFootPolygon;
   }

   public void updateVisualizer()
   {
      if (footstepAdjusterVisualizer != null)
      {
         footstepAdjusterVisualizer.update();
      }
   }

   public void hideTouchdownPolygons()
   {
      if (footstepAdjusterVisualizer != null)
      {
         footstepAdjusterVisualizer.hide();
      }
   }
}
