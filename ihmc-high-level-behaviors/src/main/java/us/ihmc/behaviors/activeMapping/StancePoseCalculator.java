package us.ihmc.behaviors.activeMapping;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;

public class StancePoseCalculator
{
   private int windowSize = 5;
   private float resolution = 0.1f;
   private float maxWidth = 0.5f;
   private float maxLength = 0.5f;
   private float maxYaw = 0.1f;

   private ArrayList<FramePose3D> leftPoses = new ArrayList<>();
   private ArrayList<FramePose3D> rightPoses = new ArrayList<>();

   public StancePoseCalculator(float maxWidth, float maxLength, float maxYaw)
   {
      this.maxWidth = maxWidth;
      this.maxLength = maxLength;
      this.maxYaw = maxYaw;
   }

   public SideDependentList<FramePose3D> getStancePoses(FramePose3D goalPose, TerrainMapData terrainMap)
   {
      insertCandidatePoses(leftPoses, goalPose, RobotSide.LEFT);
      insertCandidatePoses(rightPoses, goalPose, RobotSide.RIGHT);
      return searchForOptimalGoalStance(leftPoses, rightPoses, terrainMap);
   }

   public void insertCandidatePoses(ArrayList<FramePose3D> poses, FramePose3D goalPose, RobotSide side)
   {
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
         }
      }
   }

   public SideDependentList<FramePose3D> searchForOptimalGoalStance(ArrayList<FramePose3D> leftPoses,
                                                                    ArrayList<FramePose3D> rightPoses,
                                                                    TerrainMapData terrainMap)
   {
      double minCost = Double.POSITIVE_INFINITY;
      double cost = 0.0f;

      FramePose3D bestLeftPose = new FramePose3D();
      FramePose3D bestRightPose = new FramePose3D();

      for (FramePose3D leftPose : leftPoses)
      {
         for (FramePose3D rightPose : rightPoses)
         {
            cost = Math.abs(0.5f - leftPose.getPositionDistance(rightPose));

            // TODO: Compute elevation and feasibility costs using height and contact maps
            //float height = terrainMap.getHeightAt()

            if (cost < minCost)
            {
               minCost = cost;
               bestLeftPose.set(leftPose);
               bestRightPose.set(rightPose);
            }
         }
      }

      return new SideDependentList<>(bestLeftPose, bestRightPose);
   }
}
