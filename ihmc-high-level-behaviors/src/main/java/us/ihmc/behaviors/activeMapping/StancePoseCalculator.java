package us.ihmc.behaviors.activeMapping;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerEnvironmentHandler;
import us.ihmc.footstepPlanning.polygonSnapping.HeightMapPolygonSnapper;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;

public class StancePoseCalculator
{
   public static final float CONTACT_COST_WEIGHT = 10.0f;
   public static final float MAX_CONTACT_VALUE = 255.0f;
   public static final float NOMINAL_STANCE_DISTANCE = 0.3f;

   private final int windowSize = 5;
   private final double resolution = 0.08f;

   private final HeightMapPolygonSnapper heightMapPolygonSnapper;

   private final ArrayList<FramePose3D> leftPoses = new ArrayList<>();
   private final ArrayList<FramePose3D> rightPoses = new ArrayList<>();

   private final SideDependentList<Pose3D> bestPose3Ds = new SideDependentList<>(new Pose3D(), new Pose3D());
   private final SideDependentList<FramePose3D> bestFramePoses = new SideDependentList<>(new FramePose3D(), new FramePose3D());

   private final SideDependentList<ConvexPolygon2D> footPolygons;

   public StancePoseCalculator(SideDependentList<ConvexPolygon2D> footPolygons)
   {
      this.footPolygons = footPolygons;
      this.heightMapPolygonSnapper = new HeightMapPolygonSnapper();
   }

   public SideDependentList<FramePose3D> getStancePoses(FramePose3D pickPoint, TerrainMapData terrainMap, FootstepPlannerEnvironmentHandler environmentHandler)
   {
      reset();
      insertCandidatePoses(leftPoses, pickPoint, RobotSide.LEFT);
      insertCandidatePoses(rightPoses, pickPoint, RobotSide.RIGHT);
      searchForOptimalGoalStance(leftPoses, rightPoses, pickPoint, terrainMap);
      snapPosesToEnvironment(environmentHandler);
      return new SideDependentList<>(bestFramePoses.get(RobotSide.LEFT), bestFramePoses.get(RobotSide.RIGHT));
   }

   public void reset()
   {
      leftPoses.clear();
      rightPoses.clear();
      bestFramePoses.get(RobotSide.LEFT).setToZero();
      bestFramePoses.get(RobotSide.RIGHT).setToZero();
   }

   public void insertCandidatePoses(ArrayList<FramePose3D> poses, FramePose3D goalPose, RobotSide side)
   {
      poses.clear();
      double multiplier = side == RobotSide.LEFT ? -1 : 1;

      // Sample poses around the provided goal pose and check if they are valid in the height map
      for (int i = -windowSize; i < windowSize; i++)
      {
         for (int j = 1; j < windowSize; j++)
         {
            double x = i * resolution;
            double y = j * resolution * multiplier;

            FramePose3D pose = new FramePose3D(goalPose);
            pose.appendTranslation(x, y, 0);

            poses.add(pose);
         }
      }
   }

   public void searchForOptimalGoalStance(ArrayList<FramePose3D> leftPoses, ArrayList<FramePose3D> rightPoses, FramePose3D pickPoint, TerrainMapData terrainMap)
   {
      double minCost = Double.POSITIVE_INFINITY;
      double cost;

      for (FramePose3D leftPose : leftPoses)
      {
         for (FramePose3D rightPose : rightPoses)
         {
            double heightLeft = terrainMap.getHeightInWorld(leftPose.getPosition().getX32(), leftPose.getPosition().getY32());
            double heightRight = terrainMap.getHeightInWorld(rightPose.getPosition().getX32(), rightPose.getPosition().getY32());

            double contactCostLeft = Math.abs(MAX_CONTACT_VALUE - terrainMap.getContactScoreInWorld(leftPose.getPosition().getX32(), leftPose.getPosition().getY32()));
            double contactCostRight = Math.abs(MAX_CONTACT_VALUE - terrainMap.getContactScoreInWorld(rightPose.getPosition().getX32(), rightPose.getPosition().getY32()));

            cost = Math.abs(NOMINAL_STANCE_DISTANCE - leftPose.getPositionDistance(rightPose));
            cost += Math.abs((NOMINAL_STANCE_DISTANCE / 2) - leftPose.getPositionDistance(pickPoint));
            cost += Math.abs((NOMINAL_STANCE_DISTANCE / 2) - rightPose.getPositionDistance(pickPoint));
            cost += CONTACT_COST_WEIGHT * (contactCostLeft + contactCostRight);

            leftPose.setZ(heightLeft);
            rightPose.setZ(heightRight);

            if (cost < minCost)
            {
               minCost = cost;
               bestFramePoses.get(RobotSide.LEFT).set(leftPose);
               bestFramePoses.get(RobotSide.RIGHT).set(rightPose);
            }
         }
      }
   }

   public void snapPosesToEnvironment(FootstepPlannerEnvironmentHandler environmentHandler)
   {
      for (RobotSide side : RobotSide.values)
      {
         snapToEnvironment(environmentHandler, bestFramePoses.get(side), footPolygons.get(side));
      }
   }

   private void snapToEnvironment(FootstepPlannerEnvironmentHandler environmentHandler, FramePose3D poseToSnap, ConvexPolygon2D footPolygon)
   {
      footPolygon.applyTransform(poseToSnap);

      RigidBodyTransform snapTransform = heightMapPolygonSnapper.snapPolygonToHeightMap(footPolygon, environmentHandler, 0.05, Math.toRadians(45.0));

      if (snapTransform != null)
      {
         snapTransform.getTranslation().setZ(0);
         snapTransform.getRotation().setYawPitchRoll(0, snapTransform.getRotation().getPitch(), snapTransform.getRotation().getRoll());
         poseToSnap.getRotation().applyTransform(snapTransform);
      }
   }

   static RigidBodyTransform createTransformToMatchSurfaceNormalPreserveX(Vector3DReadOnly surfaceNormal)
   {
      RigidBodyTransform transformToReturn = new RigidBodyTransform();
      constructTransformToMatchSurfaceNormalPreserveX(surfaceNormal, transformToReturn);

      return transformToReturn;
   }

   static void constructTransformToMatchSurfaceNormalPreserveX(Vector3DReadOnly surfaceNormal, RigidBodyTransform transformToPack)
   {
      // xAxis = yAxis cross SurfaceNormal
      double xAxisX = surfaceNormal.getZ();
      double xAxisY = 0.0;
      double xAxisZ = -surfaceNormal.getX();

      double xNorm = EuclidCoreTools.norm(xAxisX, xAxisZ);

      xAxisX /= xNorm;
      xAxisZ /= xNorm;

      // yAxis = surfaceNormal cross xAxis
      double yAxisX = surfaceNormal.getY() * xAxisZ;
      double yAxisY = surfaceNormal.getZ() * xAxisX - surfaceNormal.getX() * xAxisZ;
      double yAxisZ = -surfaceNormal.getY() * xAxisX;

      transformToPack.getRotation().set(xAxisX, yAxisX, surfaceNormal.getX(), xAxisY, yAxisY, surfaceNormal.getY(), xAxisZ, yAxisZ, surfaceNormal.getZ());
   }

   public void snapPosesToTerrainMapData(TerrainMapData terrainMapData)
   {
      for (RobotSide side : RobotSide.values)
      {
         snapToTerrainMap(terrainMapData, bestFramePoses.get(side));
      }
   }

   private void snapToTerrainMap(TerrainMapData terrainMapData, FramePose3D poseToSnap)
   {
      UnitVector3DReadOnly normal = terrainMapData.computeSurfaceNormalInWorld((float) poseToSnap.getX(), (float) poseToSnap.getY(), 1);
      RigidBodyTransform snapTransform = createTransformToMatchSurfaceNormalPreserveX(normal);
      poseToSnap.applyTransform(snapTransform);
   }

   public ArrayList<FramePose3D> getLeftPoses()
   {
      return leftPoses;
   }

   public ArrayList<FramePose3D> getRightPoses()
   {
      return rightPoses;
   }

   public SideDependentList<Pose3D> getBestPoses()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         bestPose3Ds.set(robotSide, new Pose3D(bestFramePoses.get(robotSide)));
      }
      return bestPose3Ds;
   }

   public SideDependentList<FramePose3D> getBestFramePoses()
   {
      return bestFramePoses;
   }
}
