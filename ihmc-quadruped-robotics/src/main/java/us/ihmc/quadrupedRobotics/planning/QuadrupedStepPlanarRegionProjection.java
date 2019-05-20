package us.ihmc.quadrupedRobotics.planning;

import org.apache.commons.lang3.mutable.MutableBoolean;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class QuadrupedStepPlanarRegionProjection
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleParameter minimumDistanceToRegionEdge = new DoubleParameter("minimumDistanceToRegionEdge", registry, 0.03);

   private final RecyclingArrayList<PlanarRegion> planarRegions = new RecyclingArrayList<>(100, PlanarRegion::new);
   private final QuadrantDependentList<PlanarRegion> regionMap = new QuadrantDependentList<>();
   private final QuadrantDependentList<MutableBoolean> hasCheckedForRegion = new QuadrantDependentList<>(MutableBoolean::new);

   private final ConvexPolygon2D planarRegionPolygon = new ConvexPolygon2D();
   private final ConvexPolygonScaler scaler = new ConvexPolygonScaler();
   private final ConvexPolygon2D shrunkPolygon = new ConvexPolygon2D();

   private final Point2D tempPoint2D = new Point2D();
   private final Point3D tempPoint3D = new Point3D();
   private final Vector3D tempVector3D = new Vector3D();

   public QuadrupedStepPlanarRegionProjection(YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
   }

   public void project(FramePoint3D goalPosition, RobotQuadrant quadrant)
   {
      // regions are unavailable
      if (planarRegions.isEmpty())
      {
         return;
      }

      // at the start of the step, assign a region to the quadrant. subsequent projections will always keep the step inside this region
      if (regionMap.get(quadrant) == null && hasCheckedForRegion.get(quadrant).isFalse())
      {
         regionMap.put(quadrant, findTouchdownRegion(goalPosition));
         hasCheckedForRegion.get(quadrant).setTrue();
      }

      // if no region is found, do nothing
      if (regionMap.get(quadrant) == null)
      {
         return;
      }

      // project point into associated region
      doProjection(goalPosition, quadrant);
   }

   private void doProjection(FramePoint3D goalPosition, RobotQuadrant quadrant)
   {
      PlanarRegion region = regionMap.get(quadrant);

      tempPoint3D.set(goalPosition);
      region.transformFromWorldToLocal(tempPoint3D);
      tempPoint2D.set(tempPoint3D.getX(), tempPoint3D.getY());

      planarRegionPolygon.set(region.getConvexHull());
      double minimumDistanceToRegionEdge = this.minimumDistanceToRegionEdge.getValue();

      if (planarRegionPolygon.signedDistance(tempPoint2D) > - minimumDistanceToRegionEdge)
      {
         scaler.scaleConvexPolygon(planarRegionPolygon, minimumDistanceToRegionEdge, shrunkPolygon);
         shrunkPolygon.orthogonalProjection(tempPoint2D);

         tempVector3D.set(tempPoint2D.getX(), tempPoint2D.getY(), 0.0);
         tempVector3D.sub(tempPoint3D.getX(), tempPoint3D.getY(), 0.0);
         region.transformFromLocalToWorld(tempVector3D);

         goalPosition.add(tempVector3D);
      }
   }

   private PlanarRegion findTouchdownRegion(FramePoint3D goalPosition)
   {
      ReferenceFrame referenceFrame = goalPosition.getReferenceFrame();
      goalPosition.changeFrame(ReferenceFrame.getWorldFrame());

      PlanarRegion highestIntersectingRegion = null;
      double highestRegionHeight = Double.NEGATIVE_INFINITY;
      double x = goalPosition.getX();
      double y = goalPosition.getY();

      for (int i = 0; i < planarRegions.size(); i++)
      {
         PlanarRegion planarRegion = planarRegions.get(i);
         if (!isPointInsideByProjectionOntoXYPlane(planarRegion, x, y))
         {
            continue;
         }

         double height = planarRegion.getPlaneZGivenXY(x, y);
         if (height > highestRegionHeight)
         {
            highestRegionHeight = height;
            highestIntersectingRegion = planarRegion;
         }
      }

      if (highestIntersectingRegion == null)
      {
         return null;
      }
      else
      {
         goalPosition.changeFrame(referenceFrame);
         return highestIntersectingRegion;
      }
   }

   /**
    * Garbage free version of {@link PlanarRegion#isPointInsideByProjectionOntoXYPlane(double, double)}
    */
   private boolean isPointInsideByProjectionOntoXYPlane(PlanarRegion planarRegion, double x, double y)
   {
      tempPoint3D.setX(x);
      tempPoint3D.setY(y);
      tempPoint3D.setZ(planarRegion.getPlaneZGivenXY(x, y));
      planarRegion.transformFromWorldToLocal(tempPoint3D);

      for (int i = 0; i < planarRegion.getNumberOfConvexPolygons(); i++)
      {
         if(planarRegion.getConvexPolygon(i).isPointInside(tempPoint3D.getX(), tempPoint3D.getY()))
         {
            return true;
         }
      }

      return false;
   }

   public void completedStep(RobotQuadrant quadrant)
   {
      regionMap.put(quadrant, null);
      hasCheckedForRegion.get(quadrant).setFalse();
   }

   public void handlePlanarRegionsListCommand(PlanarRegionsListCommand command)
   {
      planarRegions.clear();
      for (int i = 0; i < command.getNumberOfPlanarRegions(); i++)
      {
         command.getPlanarRegionCommand(i).getPlanarRegion(planarRegions.add());
      }

      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         regionMap.put(quadrant, null);
         hasCheckedForRegion.get(quadrant).setFalse();
      }
   }

   // getter for tests
   protected double getMinimumDistanceToRegionEdge()
   {
      return minimumDistanceToRegionEdge.getValue();
   }
}
