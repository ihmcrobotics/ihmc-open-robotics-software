package us.ihmc.behaviors.activeMapping;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerEnvironmentHandler;
import us.ihmc.footstepPlanning.polygonSnapping.HeightMapPolygonSnapper;
import us.ihmc.footstepPlanning.polygonSnapping.PolygonSnapperTools;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
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
      bestFramePoses.get(RobotSide.LEFT).setToNaN();
      bestFramePoses.get(RobotSide.RIGHT).setToNaN();
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
      return new SideDependentList<>(new FramePose3D(bestFramePoses.get(RobotSide.LEFT)), new FramePose3D(bestFramePoses.get(RobotSide.RIGHT)));
   }

   public void reset()
   {
      leftPoses.clear();
      rightPoses.clear();
//      bestFramePoses.get(RobotSide.LEFT).setToNaN();
//      bestFramePoses.get(RobotSide.RIGHT).setToNaN();
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

   private final PoseReferenceFrame midStanceFrame = new PoseReferenceFrame("MidStanceFrame", ReferenceFrame.getWorldFrame());

   public void searchForOptimalGoalStance(ArrayList<FramePose3D> leftFootPoses,
                                          ArrayList<FramePose3D> rightFootPoses,
                                          FramePose3DReadOnly midStancePose,
                                          TerrainMapData terrainMap)
   {
      midStanceFrame.setPoseAndUpdate(midStancePose);

      double previousBestCost = Double.POSITIVE_INFINITY;
      for (RobotSide robotSide : RobotSide.values)
      {
         bestFramePoses.get(robotSide).setZ(0.0);
         bestFramePoses.get(robotSide).getOrientation().set(midStancePose.getOrientation());
      }
      if (!bestFramePoses.get(RobotSide.LEFT).containsNaN() && !bestFramePoses.get(RobotSide.RIGHT).containsNaN())
         previousBestCost = evaluateCostOfPoses(bestFramePoses.get(RobotSide.LEFT), bestFramePoses.get(RobotSide.RIGHT), terrainMap);


      DMatrixRMaj costMatrix = new DMatrixRMaj(leftFootPoses.size(), rightFootPoses.size());
      for (int i = 0; i < leftFootPoses.size(); i++)
      {
         FramePose3D leftPose = leftFootPoses.get(i);
         for (int j = 0; j < rightFootPoses.size(); j++)
         {
            FramePose3D rightPose = rightFootPoses.get(j);
            double cost = evaluateCostOfPoses(leftPose, rightPose, terrainMap);
            costMatrix.set(i, j,  cost);
         }
      }

      double bestCost = CommonOps_DDRM.elementMin(costMatrix);

      // if this is false, then the solution that we found previously is better, and we should not change the poses.
      boolean hasBetterSolution = (1.0 - bestCost / previousBestCost) >= stancePoseParameters.getCostImprovementForSwitch();

      if (hasBetterSolution)
      {
         for (int i = 0; i < leftFootPoses.size(); i++)
         {
            boolean foundSolution = false;
            for (int j = 0; j < rightFootPoses.size(); j++)
            {
               if (MathTools.epsilonEquals(costMatrix.get(i, j), bestCost, 1e-4))
               {
                  bestFramePoses.get(RobotSide.LEFT).setMatchingFrame(leftFootPoses.get(i));
                  bestFramePoses.get(RobotSide.RIGHT).setMatchingFrame(rightFootPoses.get(j));
                  foundSolution = true;
                  break;
               }
            }

            if (foundSolution)
               break;
         }
      }
   }

   private double evaluateCostOfPoses(FramePose3D leftPose, FramePose3D rightPose, TerrainMapData terrainMap)
   {
      leftPose.changeFrame(ReferenceFrame.getWorldFrame());
      rightPose.changeFrame(ReferenceFrame.getWorldFrame());

      double heightLeft = terrainMap.getHeightInWorld(leftPose.getPosition().getX32(), leftPose.getPosition().getY32());
      double heightRight = terrainMap.getHeightInWorld(rightPose.getPosition().getX32(), rightPose.getPosition().getY32());
      leftPose.setZ(heightLeft);
      rightPose.setZ(heightRight);

      double contactCostLeft = Math.abs(stancePoseParameters.getMaxContactValue() - terrainMap.getContactScoreInWorld(leftPose.getPosition().getX32(),
                                                                                                                      leftPose.getPosition().getY32()));

      double contactCostRight = Math.abs(stancePoseParameters.getMaxContactValue() - terrainMap.getContactScoreInWorld(rightPose.getPosition().getX32(),
                                                                                                                       rightPose.getPosition().getY32()));

      leftPose.changeFrame(midStanceFrame);
      rightPose.changeFrame(midStanceFrame);

      // Encourage footholds that, combined, achieve the desired stance width.
      double cost = Math.abs(stancePoseParameters.getNominalStanceDistance() - Math.abs(leftPose.getY() - rightPose.getY())) / 3.0;
      // Encourage footholds that are evenly split in width.
      double halfStanceWidth = stancePoseParameters.getNominalStanceDistance() / 2.0;
      cost += Math.abs(Math.abs(leftPose.getY()) - halfStanceWidth) / 3.0;
      cost += Math.abs(Math.abs(rightPose.getY()) - halfStanceWidth) / 3.0;
      // Penalize the cost of the X coordinate not being aligned with the goal in the goal frame. TODO consider making this not a squared cost
      cost += (Math.abs(leftPose.getX()) + Math.abs(rightPose.getX())) * 2.0 / 3.0;
      cost += Math.abs(leftPose.getX() -  rightPose.getX()) / 3.0;
      // Encourage footholds at the same height.
      cost += stancePoseParameters.getHeightChangeCostWeight() * Math.abs(leftPose.getPosition().getZ() - rightPose.getPosition().getZ());
      // Discourage footholds that have bad contact scores.
      cost += stancePoseParameters.getContactCostWeight() * (contactCostLeft + contactCostRight);

      leftPose.changeFrame(ReferenceFrame.getWorldFrame());
      rightPose.changeFrame(ReferenceFrame.getWorldFrame());

      return cost;
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
