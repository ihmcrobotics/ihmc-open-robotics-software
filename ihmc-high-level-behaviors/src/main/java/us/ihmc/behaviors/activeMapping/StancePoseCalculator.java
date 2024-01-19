package us.ihmc.behaviors.activeMapping;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.footstepPlanning.polygonSnapping.HeightMapPolygonSnapper;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import java.util.ArrayList;

public class StancePoseCalculator
{
   private int windowSize = 5;
   private float resolution = 0.08f;
   private float maxWidth = 0.5f;
   private float maxLength = 0.5f;
   private float maxYaw = 0.1f;

   private HeightMapPolygonSnapper heightMapPolygonSnapper;

   private ArrayList<FramePose3D> leftPoses = new ArrayList<>();
   private ArrayList<FramePose3D> rightPoses = new ArrayList<>();

   private SideDependentList<Pose3D> bestPose3Ds = new SideDependentList<>(new Pose3D(), new Pose3D());
   private SideDependentList<FramePose3D> bestFramePoses = new SideDependentList<>(new FramePose3D(), new FramePose3D());

   public StancePoseCalculator(float maxWidth, float maxLength, float maxYaw)
   {
      this.maxWidth = maxWidth;
      this.maxLength = maxLength;
      this.maxYaw = maxYaw;
      this.heightMapPolygonSnapper = new HeightMapPolygonSnapper();
   }

   public SideDependentList<FramePose3D> getStancePoses(FramePose3D goalPose, TerrainMapData terrainMap, HeightMapData heightMapData)
   {
      insertCandidatePoses(leftPoses, goalPose, RobotSide.LEFT);
      insertCandidatePoses(rightPoses, goalPose, RobotSide.RIGHT);
      searchForOptimalGoalStance(leftPoses, rightPoses, goalPose, terrainMap);
      snapPosesToHeightMapData(heightMapData);
      return bestFramePoses;
   }

   public void insertCandidatePoses(ArrayList<FramePose3D> poses, FramePose3D goalPose, RobotSide side)
   {
      poses.clear();
      float multiplier = side == RobotSide.LEFT ? -1 : 1;

      // sample left and right poses around the provided goal pose and check if they are valid in the height map
      for (int i = -windowSize; i < windowSize; i++)
      {
         for (int j = 0; j < windowSize; j++)
         {
            float x = i * resolution;
            float y = j * resolution * multiplier;

            FramePose3D pose = new FramePose3D(goalPose);
            pose.appendTranslation(x, y, 0);

            poses.add(pose);
         }
      }
   }

   public void searchForOptimalGoalStance(ArrayList<FramePose3D> leftPoses, ArrayList<FramePose3D> rightPoses, FramePose3D goalPose, TerrainMapData terrainMap)
   {
      double minCost = Double.POSITIVE_INFINITY;
      double cost = 0.0f;

      for (FramePose3D leftPose : leftPoses)
      {
         for (FramePose3D rightPose : rightPoses)
         {
            float heightLeft = terrainMap.getHeightInWorld(leftPose.getPosition().getX32(), leftPose.getPosition().getY32());
            float heightRight = terrainMap.getHeightInWorld(rightPose.getPosition().getX32(), rightPose.getPosition().getY32());

            float contactCostLeft = Math.abs(255.0f - terrainMap.getContactScoreInWorld(leftPose.getPosition().getX32(), leftPose.getPosition().getY32()));
            float contactCostRight = Math.abs(255.0f - terrainMap.getContactScoreInWorld(rightPose.getPosition().getX32(), rightPose.getPosition().getY32()));

            cost = Math.abs(0.5f - leftPose.getPositionDistance(rightPose));
            cost += 10.0f * (contactCostLeft + contactCostRight);

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

   public void snapPosesToHeightMapData(HeightMapData heightMapData)
   {
      for (RobotSide side : RobotSide.values)
      {
         snapToHeightMap(heightMapData, bestFramePoses.get(side));
      }
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
      UnitVector3DBasics normal = terrainMapData.computeSurfaceNormalInWorld((float) poseToSnap.getX(), (float) poseToSnap.getY(), 1);
      RigidBodyTransform snapTransform = createTransformToMatchSurfaceNormalPreserveX(normal);
      poseToSnap.applyTransform(snapTransform);
   }

   private void snapToHeightMap(HeightMapData heightMapData, FramePose3D poseToSnap)
   {
      ConvexPolygon2D footPolygon = PlannerTools.createFootPolygon(0.25, 0.12, 0.8);
      footPolygon.applyTransform(poseToSnap);

      RigidBodyTransform snapTransform = heightMapPolygonSnapper.snapPolygonToHeightMap(footPolygon, heightMapData, 0.1);

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
