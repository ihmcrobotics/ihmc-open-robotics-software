package us.ihmc.behaviors.activeMapping;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerEnvironmentHandler;
import us.ihmc.footstepPlanning.polygonSnapping.HeightMapPolygonSnapper;
import us.ihmc.footstepPlanning.polygonSnapping.PolygonSnapperTools;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;

public class StancePoseCalculator
{
   private final StancePoseParameters stancePoseParameters = new StancePoseParameters();

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

   public SideDependentList<FramePose3D> getStancePoses(FramePose3DReadOnly midStancePose,
                                                        TerrainMapData terrainMap,
                                                        FootstepPlannerEnvironmentHandler environmentHandler)
   {
      reset();
      populationCandidatePoses(leftPoses, midStancePose, RobotSide.LEFT);
      populationCandidatePoses(rightPoses, midStancePose, RobotSide.RIGHT);
      searchForOptimalGoalStance(leftPoses, rightPoses, midStancePose, terrainMap);
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

   public void populationCandidatePoses(ArrayList<FramePose3D> posesToPack, FramePose3DReadOnly goalPose, RobotSide side)
   {
      posesToPack.clear();
      double multiplier = side == RobotSide.LEFT ? -1 : 1;

      // Sample poses around the provided goal pose and check if they are valid in the height map
      int windowSize = stancePoseParameters.getSearchWindowSize();
      for (int i = -windowSize; i < windowSize; i++)
      {
         for (int j = 1; j < windowSize + 1; j++)
         {
            double x = i * stancePoseParameters.getSearchWindowResolution();
            double y = j * stancePoseParameters.getSearchWindowResolution() * multiplier;

            FramePose3D pose = new FramePose3D(goalPose);
            pose.appendTranslation(x, y, 0);

            posesToPack.add(pose);
         }
      }
   }

   public void searchForOptimalGoalStance(ArrayList<FramePose3D> leftFootPoses,
                                          ArrayList<FramePose3D> rightFootPoses,
                                          FramePose3DReadOnly midStancePose,
                                          TerrainMapData terrainMap)
   {
      double minCost = Double.POSITIVE_INFINITY;
      double cost;

      for (FramePose3D leftPose : leftFootPoses)
      {
         for (FramePose3D rightPose : rightFootPoses)
         {
            double heightLeft = terrainMap.getHeightInWorld(leftPose.getPosition().getX32(), leftPose.getPosition().getY32());
            double heightRight = terrainMap.getHeightInWorld(rightPose.getPosition().getX32(), rightPose.getPosition().getY32());

            double contactCostLeft = Math.abs(stancePoseParameters.getMaxContactValue() - terrainMap.getContactScoreInWorld(leftPose.getPosition().getX32(),
                                                                                                                            leftPose.getPosition().getY32()));
            double contactCostRight = Math.abs(stancePoseParameters.getMaxContactValue() - terrainMap.getContactScoreInWorld(rightPose.getPosition().getX32(),
                                                                                                                             rightPose.getPosition().getY32()));

            cost = Math.abs(stancePoseParameters.getNominalStanceDistance() - leftPose.getPosition().distanceXY(rightPose.getPosition()));
            cost += Math.abs((stancePoseParameters.getNominalStanceDistance() / 2) - leftPose.getPosition().distanceXY(midStancePose.getPosition()));
            cost += Math.abs((stancePoseParameters.getNominalStanceDistance() / 2) - rightPose.getPosition().distanceXY(midStancePose.getPosition()));
            cost += Math.abs(leftPose.getPosition().getZ() - rightPose.getPosition().getZ());
            cost += stancePoseParameters.getContactCostWeight() * (contactCostLeft + contactCostRight);

            leftPose.setZ(heightLeft);
            rightPose.setZ(heightRight);

            // In order to not jump back and forth between two good poses, only accept a pose if its at least epsilon better for the cost
            double epsilon = 0.02;
            if (cost < minCost - epsilon)
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
         snapToEnvironment(environmentHandler, bestFramePoses.get(side), side);
      }
   }

   private void snapToEnvironment(FootstepPlannerEnvironmentHandler environmentHandler, FramePose3D poseToSnap, RobotSide side)
   {
      // Transform the polygon to be surrounding the pose we want to step on
      ConvexPolygon2D footPolygon = new ConvexPolygon2D(footPolygons.get(side));
      footPolygon.applyTransform(poseToSnap);

      RigidBodyTransform snapTransform = heightMapPolygonSnapper.snapPolygonToHeightMap(footPolygon, environmentHandler, 0.05, Math.toRadians(45.0));

      if (snapTransform != null)
      {
         snapTransform.getTranslation().setZ(0);
         snapTransform.getRotation().setYawPitchRoll(0, snapTransform.getRotation().getPitch(), snapTransform.getRotation().getRoll());
         poseToSnap.getRotation().applyTransform(snapTransform);
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
      UnitVector3DReadOnly normal = terrainMapData.computeSurfaceNormalInWorld((float) poseToSnap.getX(), (float) poseToSnap.getY(), 1);
      RigidBodyTransform snapTransform = PolygonSnapperTools.createTransformToMatchSurfaceNormalPreserveX(normal);
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

   public StancePoseParameters getStancePoseParameters()
   {
      return stancePoseParameters;
   }
}
