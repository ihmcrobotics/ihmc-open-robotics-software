package us.ihmc.footstepPlanning.graphSearch.collision;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstepTools;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticePoint;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersReadOnly;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.HashMap;
import java.util.function.DoubleSupplier;

public class PlanarRegionFootstepPlannerBodyCollisionDetector
{
   private final BoundingBoxCollisionDetector collisionDetector = new BoundingBoxCollisionDetector();
   private final HashMap<LatticePoint, BodyCollisionData> collisionDataHolder = new HashMap<>();

   private final DoubleSupplier bodyBoxDepth;
   private final DoubleSupplier bodyBoxWidth;
   private final DoubleSupplier bodyBoxHeight;

   private final DoubleSupplier bodyBoxBaseX;
   private final DoubleSupplier bodyBoxBaseY;
   private final DoubleSupplier bodyBoxBaseZ;

   public PlanarRegionFootstepPlannerBodyCollisionDetector(DefaultFootstepPlannerParametersReadOnly parameters)
   {
      this(parameters::getBodyBoxDepth,
           parameters::getBodyBoxWidth,
           parameters::getBodyBoxHeight,
           parameters::getBodyBoxBaseX,
           parameters::getBodyBoxBaseY,
           parameters::getBodyBoxBaseZ);
   }

   public PlanarRegionFootstepPlannerBodyCollisionDetector(DoubleSupplier bodyBoxDepth,
                                                           DoubleSupplier bodyBoxWidth,
                                                           DoubleSupplier bodyBoxHeight,
                                                           DoubleSupplier bodyBoxBaseX,
                                                           DoubleSupplier bodyBoxBaseY,
                                                           DoubleSupplier bodyBoxBaseZ)
   {
      this.bodyBoxDepth = bodyBoxDepth;
      this.bodyBoxWidth = bodyBoxWidth;
      this.bodyBoxHeight = bodyBoxHeight;
      this.bodyBoxBaseX = bodyBoxBaseX;
      this.bodyBoxBaseY = bodyBoxBaseY;
      this.bodyBoxBaseZ = bodyBoxBaseZ;
   }

   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      if(planarRegionsList != null)
         collisionDetector.setPlanarRegionsList(planarRegionsList);
      collisionDataHolder.clear();
   }

   public boolean checkForCollision(DiscreteFootstep footstep, DiscreteFootstep stanceStep, double snapHeight, double previousSnapHeight, int numberOfIntermediateChecks)
   {
      collisionDetector.setBoxDimensions(bodyBoxDepth.getAsDouble(), bodyBoxWidth.getAsDouble(), bodyBoxHeight.getAsDouble());

      // Start by checking next step
      LatticePoint latticePoint = createLatticePointForCollisionCheck(footstep);
      BodyCollisionData collisionData = checkForCollision(footstep, snapHeight);
      if (collisionData.isCollisionDetected())
      {
         return true;
      }
      else if (numberOfIntermediateChecks <= 0)
      {
         return false;
      }

      // If requested, interpolate between previous step
      LatticePoint previousLatticePoint = createLatticePointForCollisionCheck(stanceStep);

      for (int i = 0; i < numberOfIntermediateChecks; i++)
      {
         double alpha = (i + 1) / (numberOfIntermediateChecks + 1.0);
         LatticePoint interpolatedPoint = DiscreteFootstepTools.interpolate(latticePoint, previousLatticePoint, alpha);
         double interpolatedHeight = EuclidCoreTools.interpolate(snapHeight, previousSnapHeight, alpha);

         collisionData = checkForCollision(interpolatedPoint, interpolatedHeight);
         if (collisionData.isCollisionDetected())
         {
            return true;
         }
      }

      return false;
   }

   public BodyCollisionData checkForCollision(DiscreteFootstep footstep, double snappedHeight)
   {
      LatticePoint latticePoint = createLatticePointForCollisionCheck(footstep);
      return checkForCollision(latticePoint, snappedHeight);
   }

   private BodyCollisionData checkForCollision(LatticePoint latticePoint, double snappedStepHeight)
   {
      if (collisionDataHolder.containsKey(latticePoint))
      {
         return collisionDataHolder.get(latticePoint);
      }
      else
      {
         collisionDetector.setBoxPose(latticePoint.getX(), latticePoint.getY(), snappedStepHeight + bodyBoxBaseZ.getAsDouble(), latticePoint.getYaw());
         BodyCollisionData collisionData = collisionDetector.checkForCollision();
         collisionDataHolder.put(latticePoint, collisionData);
         return collisionData;
      }
   }

   private LatticePoint createLatticePointForCollisionCheck(DiscreteFootstep footstep)
   {
      double lateralOffsetSign = footstep.getRobotSide().negateIfLeftSide(1.0);
      double offsetX = bodyBoxBaseX.getAsDouble() * Math.cos(footstep.getYaw()) - lateralOffsetSign * bodyBoxBaseY.getAsDouble() * Math.sin(footstep.getYaw());
      double offsetY = bodyBoxBaseX.getAsDouble() * Math.sin(footstep.getYaw()) + lateralOffsetSign * bodyBoxBaseY.getAsDouble() * Math.cos(footstep.getYaw());
      return new LatticePoint(footstep.getX() + offsetX, footstep.getY() + offsetY, footstep.getYaw());
   }
}
