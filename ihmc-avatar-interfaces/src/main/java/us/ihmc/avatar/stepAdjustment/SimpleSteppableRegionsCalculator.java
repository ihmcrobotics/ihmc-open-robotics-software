package us.ihmc.avatar.stepAdjustment;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintListConverter;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

public class SimpleSteppableRegionsCalculator
{
   private static final double maxNormalAngleFromVertical = 0.4;
   private static final double minimumAreaToConsider = 0.01;
   private static final double defaultOrthogonalAngle = Math.toRadians(75.0);

   private static final Vector3D verticalAxis = new Vector3D(0.0, 0.0, 1.0);

   private final YoDouble maxAngleForSteppable;
   private final YoDouble minimumAreaForSteppable;
   private final YoDouble maximumStepReach;

   private final YoDouble orthogonalAngle;

   private final RecyclingArrayList<StepConstraintRegion> steppableRegions = new RecyclingArrayList<>(StepConstraintRegion::new);
   private final List<PlanarRegion> allPlanarRegions = new ArrayList<>();

   private final List<PlanarRegion> tooSmallRegions = new ArrayList<>();
   private final List<PlanarRegion> tooSteepRegions = new ArrayList<>();
   private final List<PlanarRegion> regionsValidForStepping = new ArrayList<>();
   
   private final FramePoint2D stanceFootPosition = new FramePoint2D();

   public SimpleSteppableRegionsCalculator(double maximumReach, YoRegistry registry)
   {
      maxAngleForSteppable = new YoDouble("maxAngleForSteppable", registry);
      minimumAreaForSteppable = new YoDouble("minimumAreaForSteppable", registry);
      maximumStepReach = new YoDouble("maximumStepReach", registry);
      orthogonalAngle = new YoDouble("orthogonalAngle", registry);

      maxAngleForSteppable.set(maxNormalAngleFromVertical);
      minimumAreaForSteppable.set(minimumAreaToConsider);
      maximumStepReach.set(maximumReach);
      orthogonalAngle.set(defaultOrthogonalAngle);
   }

   public void setPlanarRegions(List<PlanarRegion> planarRegions)
   {
      this.allPlanarRegions.clear();
      for (int i = 0; i < planarRegions.size(); i++)
         allPlanarRegions.add(planarRegions.get(i));
   }

   public void setStanceFootPosition(FramePoint3DReadOnly stanceFootPosition)
   {
      this.stanceFootPosition.set(stanceFootPosition);
   }

   public void setOrthogonalAngle(double orthogonalAngle)
   {
      this.orthogonalAngle.set(orthogonalAngle);
   }

   public List<StepConstraintRegion> computeSteppableRegions()
   {
      tooSmallRegions.clear();
      tooSteepRegions.clear();

      // first, filter out all the regions that are invalid for stepping
      regionsValidForStepping.clear();
      for (int i = 0; i < allPlanarRegions.size(); i++)
      {
         PlanarRegion region = allPlanarRegions.get(i);
         if (isRegionValidForStepping(region))
         {
            regionsValidForStepping.add(region);
         }
      }

      StepConstraintListConverter.convertPlanarRegionListToStepConstraintRegion(regionsValidForStepping, steppableRegions);

      return steppableRegions;
   }

   public List<PlanarRegion> getTooSmallRegions()
   {
      return tooSmallRegions;
   }

   public List<PlanarRegion> getTooSteepRegions()
   {
      return tooSteepRegions;
   }

   private boolean isRegionValidForStepping(PlanarRegion planarRegion)
   {
      double angle = planarRegion.getNormal().angle(verticalAxis);

      if (angle > maxAngleForSteppable.getValue())
      {
         tooSteepRegions.add(planarRegion);
         return false;
      }

      if (PlanarRegionTools.computePlanarRegionArea(planarRegion) < minimumAreaForSteppable.getValue())
      {
         tooSmallRegions.add(planarRegion);
         return false;
      }

      if (stanceFootPosition.containsNaN())
         return true;

      return isRegionWithinReach(stanceFootPosition, maximumStepReach.getDoubleValue(), planarRegion);
   }

   private final Point2D tempPoint = new Point2D();

   private boolean isRegionWithinReach(Point2DReadOnly point, double reach, PlanarRegion planarRegion)
   {
      // TODO do a check on the bounding box distance first

      planarRegion.getTransformToLocal().transform(point, tempPoint, false);
      if (planarRegion.getConvexHull().distance(tempPoint) > reach)
         return false;

      boolean closeEnough = false;
      for (int i = 0; i < planarRegion.getNumberOfConvexPolygons(); i++)
      {
         ConvexPolygon2DReadOnly convexPolygon = planarRegion.getConvexPolygon(i);
         if (convexPolygon.distance(tempPoint) < reach)
         {
            closeEnough = true;
            break;
         }
      }

      return closeEnough;
   }

}
