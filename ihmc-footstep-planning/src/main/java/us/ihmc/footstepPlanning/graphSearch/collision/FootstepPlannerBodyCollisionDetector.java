package us.ihmc.footstepPlanning.graphSearch.collision;

import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstepTools;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticePoint;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.swing.HeightMapCollisionDetector;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import java.util.HashMap;
import java.util.function.DoubleSupplier;

public class FootstepPlannerBodyCollisionDetector
{
   private final BoundingBoxCollisionDetector collisionDetector = new BoundingBoxCollisionDetector();
   private final HashMap<LatticePoint, EuclidShape3DCollisionResult> collisionDataHolder = new HashMap<>();

   private HeightMapData heightMapData;
   private final FrameBox3D bodyBox = new FrameBox3D();

   private final DoubleSupplier bodyBoxDepth;
   private final DoubleSupplier bodyBoxWidth;
   private final DoubleSupplier bodyBoxHeight;

   private final DoubleSupplier bodyBoxBaseX;
   private final DoubleSupplier bodyBoxBaseY;
   private final DoubleSupplier bodyBoxBaseZ;

   public FootstepPlannerBodyCollisionDetector(DefaultFootstepPlannerParametersReadOnly parameters)
   {
      this(parameters::getBodyBoxDepth,
           parameters::getBodyBoxWidth,
           parameters::getBodyBoxHeight,
           parameters::getBodyBoxBaseX,
           parameters::getBodyBoxBaseY,
           parameters::getBodyBoxBaseZ);
   }

   public FootstepPlannerBodyCollisionDetector(DoubleSupplier bodyBoxDepth,
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

   public void setHeightMapData(HeightMapData heightMapData)
   {
      this.heightMapData = heightMapData;
      collisionDataHolder.clear();
   }

   public boolean checkForCollision(DiscreteFootstep footstep, DiscreteFootstep stanceStep, double snapHeight, double previousSnapHeight, int numberOfIntermediateChecks)
   {
      collisionDetector.setBoxDimensions(bodyBoxDepth.getAsDouble(), bodyBoxWidth.getAsDouble(), bodyBoxHeight.getAsDouble());

      // Start by checking next step
      LatticePoint latticePoint = createLatticePointForCollisionCheck(footstep);
      EuclidShape3DCollisionResult collisionData = checkForCollision(footstep, snapHeight);
      if (collisionData.areShapesColliding())
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
         if (collisionData.areShapesColliding())
         {
            return true;
         }
      }

      return false;
   }

   public EuclidShape3DCollisionResult checkForCollision(DiscreteFootstep footstep, double snappedHeight)
   {
      LatticePoint latticePoint = createLatticePointForCollisionCheck(footstep);
      return checkForCollision(latticePoint, snappedHeight);
   }

   private EuclidShape3DCollisionResult checkForCollision(LatticePoint latticePoint, double snappedStepHeight)
   {
      if (collisionDataHolder.containsKey(latticePoint))
      {
         return collisionDataHolder.get(latticePoint);
      }
      else
      {
         bodyBox.getSize().set(bodyBoxDepth.getAsDouble(), bodyBoxWidth.getAsDouble(), bodyBoxHeight.getAsDouble());
         bodyBox.getPose().getTranslation().set(latticePoint.getX(), latticePoint.getY(), snappedStepHeight + bodyBoxBaseZ.getAsDouble() + 0.5 * bodyBoxHeight.getAsDouble());
         bodyBox.getPose().getRotation().setYawPitchRoll(latticePoint.getYaw(), 0.0, 0.0);

         EuclidShape3DCollisionResult collisionResult = HeightMapCollisionDetector.evaluateCollision(bodyBox, heightMapData);
         collisionDetector.setBoxPose(latticePoint.getX(), latticePoint.getY(), snappedStepHeight + bodyBoxBaseZ.getAsDouble(), latticePoint.getYaw());
         collisionDataHolder.put(latticePoint, collisionResult);
         return collisionResult;
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
