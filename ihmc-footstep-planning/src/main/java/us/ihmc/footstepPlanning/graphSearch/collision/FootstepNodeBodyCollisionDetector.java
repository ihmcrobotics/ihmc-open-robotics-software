package us.ihmc.footstepPlanning.graphSearch.collision;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.DoubleSupplier;

public class FootstepNodeBodyCollisionDetector
{
   private final BoundingBoxCollisionDetector collisionDetector = new BoundingBoxCollisionDetector();
   private final HashMap<LatticeNode, BodyCollisionData> collisionDataHolder = new HashMap<>();

   private final DoubleSupplier bodyBoxDepth;
   private final DoubleSupplier bodyBoxWidth;
   private final DoubleSupplier bodyBoxHeight;
   private final DoubleSupplier xyProximityCheck;

   private final DoubleSupplier bodyBoxBaseX;
   private final DoubleSupplier bodyBoxBaseY;
   private final DoubleSupplier bodyBoxBaseZ;

   public FootstepNodeBodyCollisionDetector(FootstepPlannerParametersReadOnly parameters)
   {
      this(parameters::getBodyBoxDepth,
           parameters::getBodyBoxWidth,
           parameters::getBodyBoxHeight,
           parameters::getMaximum2dDistanceFromBoundingBoxToPenalize,
           parameters::getBodyBoxBaseX,
           parameters::getBodyBoxBaseY,
           parameters::getBodyBoxBaseZ);
   }

   public FootstepNodeBodyCollisionDetector(DoubleSupplier bodyBoxDepth,
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

   public List<BodyCollisionData> checkForCollision(FootstepNode footstepNode, FootstepNode previousNode, double snapHeight, double previousSnapHeight, int numberOfCollisionChecks)
   {
      if(numberOfCollisionChecks < 1)
         return null;

      collisionDetector.setBoxDimensions(bodyBoxDepth.getAsDouble(), bodyBoxWidth.getAsDouble(), bodyBoxHeight.getAsDouble(), xyProximityCheck.getAsDouble());
      ArrayList<BodyCollisionData> collisionDataList = new ArrayList<>();

      LatticeNode previousLatticeNode = createNodeForCollisionCheck(previousNode);
      LatticeNode latticeNode = createNodeForCollisionCheck(footstepNode);

      for (int i = 0; i < numberOfCollisionChecks; i++)
      {
         double alpha = i / ((double) numberOfCollisionChecks);
         LatticeNode interpolatedNode = FootstepNodeTools.interpolate(latticeNode, previousLatticeNode, alpha);
         double interpolatedHeight = EuclidCoreTools.interpolate(snapHeight, previousSnapHeight, alpha);

         BodyCollisionData collisionData = checkForCollision(interpolatedNode, interpolatedHeight);
         collisionDataList.add(collisionData);
      }

      return collisionDataList;
   }

   public BodyCollisionData checkForCollision(FootstepNode footstepNode, double snappedNodeHeight)
   {
      LatticeNode latticeNode = createNodeForCollisionCheck(footstepNode);
      return checkForCollision(latticeNode, snappedNodeHeight);
   }

   private BodyCollisionData checkForCollision(LatticeNode latticeNode, double snappedNodeHeight)
   {
      if (collisionDataHolder.containsKey(latticeNode))
      {
         return collisionDataHolder.get(latticeNode);
      }
      else
      {
         collisionDetector.setBoxPose(latticeNode.getX(), latticeNode.getY(), snappedNodeHeight + bodyBoxBaseZ.getAsDouble(), latticeNode.getYaw());
         BodyCollisionData collisionData = collisionDetector.checkForCollision();
         collisionDataHolder.put(latticeNode, collisionData);
         return collisionData;
      }
   }

   private LatticeNode createNodeForCollisionCheck(FootstepNode node)
   {
      double lateralOffsetSign = node.getRobotSide().negateIfLeftSide(1.0);
      double offsetX = bodyBoxBaseX.getAsDouble() * Math.cos(node.getYaw()) - lateralOffsetSign * bodyBoxBaseY.getAsDouble() * Math.sin(node.getYaw());
      double offsetY = bodyBoxBaseX.getAsDouble() * Math.sin(node.getYaw()) + lateralOffsetSign * bodyBoxBaseY.getAsDouble() * Math.cos(node.getYaw());
      return new LatticeNode(node.getX() + offsetX, node.getY() + offsetY, node.getYaw());
   }
}
