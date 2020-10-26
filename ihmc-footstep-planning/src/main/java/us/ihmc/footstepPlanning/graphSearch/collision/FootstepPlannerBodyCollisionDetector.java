package us.ihmc.footstepPlanning.graphSearch.collision;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstepTools;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticePoint;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.DoubleSupplier;

public class FootstepPlannerBodyCollisionDetector
{
   private final BoundingBoxCollisionDetector collisionDetector = new BoundingBoxCollisionDetector();
   private final HashMap<LatticePoint, BodyCollisionData> collisionDataHolder = new HashMap<>();

   private final DoubleSupplier bodyBoxDepth;
   private final DoubleSupplier bodyBoxWidth;
   private final DoubleSupplier bodyBoxHeight;
   private final DoubleSupplier xyProximityCheck;

   private final DoubleSupplier bodyBoxBaseX;
   private final DoubleSupplier bodyBoxBaseY;
   private final DoubleSupplier bodyBoxBaseZ;

   public FootstepPlannerBodyCollisionDetector(FootstepPlannerParametersReadOnly parameters)
   {
      this(parameters::getBodyBoxDepth,
           parameters::getBodyBoxWidth,
           parameters::getBodyBoxHeight,
           parameters::getMaximum2dDistanceFromBoundingBoxToPenalize,
           parameters::getBodyBoxBaseX,
           parameters::getBodyBoxBaseY,
           parameters::getBodyBoxBaseZ);
   }

   public FootstepPlannerBodyCollisionDetector(DoubleSupplier bodyBoxDepth,
                                               DoubleSupplier bodyBoxWidth,
                                               DoubleSupplier bodyBoxHeight,
                                               DoubleSupplier xyProximityCheck,
                                               DoubleSupplier bodyBoxBaseX,
                                               DoubleSupplier bodyBoxBaseY,
                                               DoubleSupplier bodyBoxBaseZ)
   {
      this.bodyBoxDepth = bodyBoxDepth;
      this.bodyBoxWidth = bodyBoxWidth;
      this.bodyBoxHeight = bodyBoxHeight;
      this.xyProximityCheck = xyProximityCheck;
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

   public List<BodyCollisionData> checkForCollision(DiscreteFootstep footstep, DiscreteFootstep stanceStep, double snapHeight, double previousSnapHeight, int numberOfCollisionChecks)
   {
      if(numberOfCollisionChecks < 1)
         return null;

      collisionDetector.setBoxDimensions(bodyBoxDepth.getAsDouble(), bodyBoxWidth.getAsDouble(), bodyBoxHeight.getAsDouble(), xyProximityCheck.getAsDouble());
      ArrayList<BodyCollisionData> collisionDataList = new ArrayList<>();

      LatticePoint previousLatticePoint = createLatticePointForCollisionCheck(stanceStep);
      LatticePoint latticePoint = createLatticePointForCollisionCheck(footstep);

      for (int i = 0; i < numberOfCollisionChecks; i++)
      {
         double alpha = i / ((double) numberOfCollisionChecks);
         LatticePoint interpolatedPoint = DiscreteFootstepTools.interpolate(latticePoint, previousLatticePoint, alpha);
         double interpolatedHeight = EuclidCoreTools.interpolate(snapHeight, previousSnapHeight, alpha);

         BodyCollisionData collisionData = checkForCollision(interpolatedPoint, interpolatedHeight);
         collisionDataList.add(collisionData);
      }

      return collisionDataList;
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
