package us.ihmc.footstepPlanning.bodyPath;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.polygonSnapping.HeightMapPolygonSnapper;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.HashMap;
import java.util.Map;

public class BodyPathCostCalculator
{
   private final FootstepPlannerParametersReadOnly parameters;
   private final Pose2D bodyPose = new Pose2D();
   private final Pose2D stepPose = new Pose2D();

   private final HeightMapPolygonSnapper snapper = new HeightMapPolygonSnapper();
   private final Map<BodyPathLatticePoint, Double> gridHeightMap;
   private final YoDouble xBody, yBody, yawBody;

   private final ConvexPolygon2D footPolygon;
   private HeightMapData heightMapData;

   private final TDoubleArrayList xOffsets = new TDoubleArrayList();
   private final TDoubleArrayList yOffsets = new TDoubleArrayList();
   private final TDoubleArrayList yawOffsets = new TDoubleArrayList();

   private final TDoubleArrayList rmsCosts = new TDoubleArrayList();
   private final TDoubleArrayList traversibilityCosts = new TDoubleArrayList();

   public BodyPathCostCalculator(FootstepPlannerParametersReadOnly parameters,
                                 ConvexPolygon2D footPolygon,
                                 Map<BodyPathLatticePoint, Double> gridHeightMap,
                                 YoRegistry registry)
   {
      this.parameters = parameters;
      this.footPolygon = footPolygon;
      this.gridHeightMap = gridHeightMap;

      xBody = new YoDouble("xBody", registry);
      yBody = new YoDouble("yBody", registry);
      yawBody = new YoDouble("yawBody", registry);

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
      double yaw = Math.atan2(node.getY() - parentNode.getY(), node.getX() - parentNode.getX());
      bodyPose.set(node.getX(), node.getY(), yaw);

      xBody.set(bodyPose.getX());
      yBody.set(bodyPose.getY());
      yawBody.set(bodyPose.getYaw());

      double parentHeight = gridHeightMap.get(parentNode);
      double leftStepCost = computeTraversibilityCost(RobotSide.LEFT, parentHeight);
      double rightStepCost = computeTraversibilityCost(RobotSide.RIGHT, parentHeight);

      return Math.min(leftStepCost, rightStepCost);
   }

   private double computeTraversibilityCost(RobotSide side, double parentHeight)
   {
      rmsCosts.clear();

      for (int xi = 0; xi < xOffsets.size(); xi++)
      {
         for (int yi = 0; yi < yOffsets.size(); yi++)
         {
            for (int ti = 0; ti < yawOffsets.size(); ti++)
            {
               stepPose.set(bodyPose);
               stepPose.appendTranslation(0.0, side.negateIfRightSide(0.5 * parameters.getIdealFootstepWidth()));
               stepPose.appendTranslation(xOffsets.get(xi), yOffsets.get(yi));
               stepPose.appendRotation(yawOffsets.get(ti));

               RigidBodyTransform transform = new RigidBodyTransform();
               stepPose.get(transform);

               ConvexPolygon2D footPolygon = new ConvexPolygon2D(this.footPolygon);
               footPolygon.applyTransform(transform);

               RigidBodyTransform snapTransform = snapper.snapPolygonToHeightMap(footPolygon,
                                                                                 heightMapData,
                                                                                 parameters.getHeightMapSnapThreshold(),
                                                                                 parentHeight - AStarBodyPathPlanner.maxStepUpDown);
               if (snapTransform == null)
               {
                  rmsCosts.add(Double.POSITIVE_INFINITY);
               }
               else
               {
                  rmsCosts.add(snapper.getRMSError());
               }
            }
         }
      }

      rmsCosts.sort();
      traversibilityCosts.clear();

      int samples = 5;
      for (int i = 0; i < samples; i++)
      {
         double rmsError = rmsCosts.get(i);
         double rmsAlpha = Math.max(0.0, (rmsError - parameters.getRMSMinErrorToPenalize()) / (parameters.getRMSErrorThreshold() - parameters.getRMSMinErrorToPenalize()));
         traversibilityCosts.add(rmsAlpha * 0.5);
      }

      return traversibilityCosts.sum() / traversibilityCosts.size();
   }
}
