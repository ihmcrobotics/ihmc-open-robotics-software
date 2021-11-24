package us.ihmc.footstepPlanning.bodyPath;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.polygonSnapping.HeightMapPolygonSnapper;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

public class BodyPathCostCalculator
{
   private final FootstepPlannerParametersReadOnly parameters;
   private final Pose2D bodyPose = new Pose2D();
   private final Pose2D stepPose = new Pose2D();

   private final HeightMapPolygonSnapper snapper = new HeightMapPolygonSnapper();
   private final ConvexPolygon2D footPolygon;
   private HeightMapData heightMapData;

   private TDoubleArrayList xOffsets = new TDoubleArrayList();
   private TDoubleArrayList yOffsets = new TDoubleArrayList();
   private TDoubleArrayList yawOffsets = new TDoubleArrayList();

   public BodyPathCostCalculator(FootstepPlannerParametersReadOnly parameters, ConvexPolygon2D footPolygon)
   {
      this.parameters = parameters;
      this.footPolygon = footPolygon;

      xOffsets.add(0.0);
      xOffsets.add(0.05);
      xOffsets.add(-0.05);

      yOffsets.add(0.0);
      yOffsets.add(0.05);
      yOffsets.add(-0.05);

      yawOffsets.add(0.0);
      yawOffsets.add(Math.toRadians(20.0));
      yawOffsets.add(Math.toRadians(-20.0));
   }

   public void setHeightMap(HeightMapData heightMapData)
   {
      this.heightMapData = heightMapData;
   }

   public double computeCost(BodyPathLatticePoint node, BodyPathLatticePoint parentNode)
   {
      double yaw = Math.atan2(node.getY() - parentNode.getYIndex(), node.getX() - parentNode.getX());
      bodyPose.set(node.getX(), node.getY(), yaw);

      double leftTraversibilityCost = computeTraversibilityCost();

      stepPose.set(bodyPose);
      stepPose.appendTranslation(0.0, 0.5 * parameters.getIdealFootstepWidth());

      snapper.snapPolygonToHeightMap(footPolygon, heightMapData, 0.06);

      double rmsError = snapper.getRMSError();

      return 0.0;
   }
}
