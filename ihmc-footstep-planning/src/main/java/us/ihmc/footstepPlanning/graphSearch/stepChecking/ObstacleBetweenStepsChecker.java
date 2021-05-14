package us.ihmc.footstepPlanning.graphSearch.stepChecking;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ObstacleBetweenStepsChecker
{
   private static final boolean DEBUG = false;

   private PlanarRegionsList planarRegionsList;
   private final FootstepSnapAndWiggler snapper;
   private final BooleanSupplier checkForPathCollisions;
   private final DoubleSupplier idealFootstepWidth;
   private final DoubleSupplier heightOffset;
   private final DoubleSupplier heightExtrusion;

   public ObstacleBetweenStepsChecker(FootstepPlannerParametersReadOnly parameters, FootstepSnapAndWiggler snapper)
   {
      this(snapper, parameters::checkForPathCollisions, parameters::getIdealFootstepWidth, parameters::getBodyBoxBaseZ, parameters::getBodyBoxHeight);
   }

   public ObstacleBetweenStepsChecker(FootstepSnapAndWiggler snapper,
                                      BooleanSupplier checkForPathCollisions,
                                      DoubleSupplier idealFootstepWidth,
                                      DoubleSupplier heightOffset,
                                      DoubleSupplier heightExtrusion)
   {
      this.snapper = snapper;
      this.checkForPathCollisions = checkForPathCollisions;
      this.idealFootstepWidth = idealFootstepWidth;
      this.heightOffset = heightOffset;
      this.heightExtrusion = heightExtrusion;
   }

   public void setPlanarRegions(PlanarRegionsList planarRegions)
   {
      this.planarRegionsList = planarRegions;
   }

   boolean hasPlanarRegions()
   {
      return planarRegionsList != null && !planarRegionsList.isEmpty();
   }

   public boolean isFootstepValid(DiscreteFootstep footstep, DiscreteFootstep previousStep)
   {
      if (previousStep == null || !checkForPathCollisions.getAsBoolean() || !hasPlanarRegions())
         return true;

      FootstepSnapData snapData = snapper.snapFootstep(footstep);
      if (snapData == null)
      {
         return true;
      }

      RigidBodyTransform snapTransform = snapData.getSnapTransform();

      FootstepSnapData previousStepSnapData = snapper.snapFootstep(previousStep);
      RigidBodyTransform previousSnapTransform = previousStepSnapData.getSnapTransform();

      Point3D stepPosition = new Point3D(footstep.getOrComputeMidFootPoint(idealFootstepWidth.getAsDouble()));
      Point3D previousStepPosition = new Point3D(previousStep.getOrComputeMidFootPoint(idealFootstepWidth.getAsDouble()));

      snapTransform.transform(stepPosition);
      previousSnapTransform.transform(previousStepPosition);

      if (isObstacleBetweenSteps(stepPosition, previousStepPosition, planarRegionsList.getPlanarRegionsAsList()))
      {
         if (DEBUG)
         {
            LogTools.debug("Found a obstacle between the steps " + footstep + " and " + previousStep);
         }
         return false;
      }

      return true;
   }

   /**
    * This is meant to test if there is a wall that the body of the robot would run into when shifting
    * from one step to the next. It is not meant to eliminate swing overs.
    */
   private boolean isObstacleBetweenSteps(Point3D stepPosition, Point3D previousStepPosition, List<PlanarRegion> planarRegions)
   {
      double groundClearance = heightOffset.getAsDouble();
      double regionHeight = heightExtrusion.getAsDouble();

      try
      {
         PlanarRegion bodyPath = createBodyRegionFromSteps(stepPosition, previousStepPosition, groundClearance, regionHeight);

         for (PlanarRegion region : planarRegions)
         {
            List<LineSegment3D> intersections = region.intersect(bodyPath);
            if (!intersections.isEmpty())
            {
               return true;
            }
         }
      }
      catch(Exception e)
      {
         if (DEBUG)
         {
            e.printStackTrace();
         }
      }

      return false;
   }

   /**
    * Given two footstep position this will create a vertical planar region above the steps. The region
    * will be aligned with the vector connecting the steps. It's lower edge will be the specified
    * distance above the higher of the two steps and the plane will have the specified hight.
    */
   public static PlanarRegion createBodyRegionFromSteps(Point3DReadOnly stepPositionA, Point3DReadOnly stepPositionB, double clearance, double height)
   {
      double lowerZ = Math.max(stepPositionA.getZ(), stepPositionB.getZ()) + clearance;
      double higherZ = lowerZ + height;

      Point3D point0 = new Point3D(stepPositionA.getX(), stepPositionA.getY(), lowerZ);
      Point3D point1 = new Point3D(stepPositionA.getX(), stepPositionA.getY(), higherZ);
      Point3D point2 = new Point3D(stepPositionB.getX(), stepPositionB.getY(), lowerZ);
      Point3D point3 = new Point3D(stepPositionB.getX(), stepPositionB.getY(), higherZ);

      Vector3D xAxisInPlane = new Vector3D();
      xAxisInPlane.sub(point2, point0);
      xAxisInPlane.normalize();
      Vector3D yAxisInPlane = new Vector3D(0.0, 0.0, 1.0);
      Vector3D zAxis = new Vector3D();
      zAxis.cross(xAxisInPlane, yAxisInPlane);

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getRotation().set(xAxisInPlane.getX(), xAxisInPlane.getY(), xAxisInPlane.getZ(), yAxisInPlane.getX(), yAxisInPlane.getY(), yAxisInPlane.getZ(), zAxis.getX(), zAxis.getY(), zAxis.getZ());
      transform.getTranslation().set(point0);
      transform.getRotation().invert();

      point0.applyInverseTransform(transform);
      point1.applyInverseTransform(transform);
      point2.applyInverseTransform(transform);
      point3.applyInverseTransform(transform);

      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(point0.getX(), point0.getY());
      polygon.addVertex(point1.getX(), point1.getY());
      polygon.addVertex(point2.getX(), point2.getY());
      polygon.addVertex(point3.getX(), point3.getY());
      polygon.update();

      return new PlanarRegion(transform, polygon);
   }
}
