package us.ihmc.footstepPlanning.tools;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.log.FootstepPlannerLog;
import us.ihmc.footstepPlanning.monteCarloPlanning.MonteCarloFootstepPlannerRequest;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

public class FootstepPlannerRequestFactory
{
   public static MonteCarloFootstepPlannerRequest createMCFPRequest(FootstepPlannerLog footstepPlannerLog,
                                                                    TerrainMapData loadedTerrainMapData,
                                                                    HeightMapData latestHeightMapData)
   {
      MonteCarloFootstepPlannerRequest request = new MonteCarloFootstepPlannerRequest();
      request.setTerrainMapData(loadedTerrainMapData);
      request.setHeightMapData(latestHeightMapData);
      request.setStartFootPose(RobotSide.LEFT, footstepPlannerLog.getRequestPacket().getStartLeftFootPose());
      request.setStartFootPose(RobotSide.RIGHT, footstepPlannerLog.getRequestPacket().getStartRightFootPose());
      request.setGoalFootPose(RobotSide.LEFT, footstepPlannerLog.getRequestPacket().getGoalLeftFootPose());
      request.setGoalFootPose(RobotSide.RIGHT, footstepPlannerLog.getRequestPacket().getGoalRightFootPose());
      request.setRequestedInitialStanceSide(RobotSide.fromByte(footstepPlannerLog.getRequestPacket().getRequestedInitialStanceSide()));
      request.setTimeout(0.05);

      LogTools.debug("Start: {}, Goal: {}, Origin: {}",
                     request.getStartFootPoses().get(RobotSide.LEFT),
                     request.getGoalFootPoses().get(RobotSide.LEFT),
                     request.getTerrainMapData().getSensorOrigin());

      return request;
   }

   public static FootstepPlannerRequest createASFPRequest(TerrainMapData terrainMapData,
                                                              HeightMapData heightMapData,
                                                              FootstepPlannerLog footstepPlannerLog,
                                                              FootstepPlan referencePlan)
   {
      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.setHeightMapData(heightMapData);
      request.setTerrainMapData(terrainMapData);
      request.setStartFootPose(RobotSide.LEFT, footstepPlannerLog.getRequestPacket().getStartLeftFootPose());
      request.setStartFootPose(RobotSide.RIGHT, footstepPlannerLog.getRequestPacket().getStartRightFootPose());
      request.setGoalFootPose(RobotSide.LEFT, footstepPlannerLog.getRequestPacket().getGoalLeftFootPose());
      request.setGoalFootPose(RobotSide.RIGHT, footstepPlannerLog.getRequestPacket().getGoalRightFootPose());
      request.setTimeout(2.5);
      request.setPlanBodyPath(false);
      request.setSnapGoalSteps(true);
      request.setPerformAStarSearch(true);
      request.setAssumeFlatGround(false);
      request.setSwingPlannerType(SwingPlannerType.NONE);
      request.setAbortIfGoalStepSnappingFails(true);
      request.setReferencePlan(referencePlan);
      request.setRequestedInitialStanceSide(RobotSide.fromByte(footstepPlannerLog.getRequestPacket().getRequestedInitialStanceSide())); // 0 -> left, 1 -> right

      return request;
   }

   public static MonteCarloFootstepPlannerRequest createMCFPRequest(DataSet dataset,
                                                                    TerrainMapData loadedTerrainMapData,
                                                                    HeightMapData latestHeightMapData)
   {
      MonteCarloFootstepPlannerRequest request = new MonteCarloFootstepPlannerRequest();
      request.setTerrainMapData(loadedTerrainMapData);
      request.setHeightMapData(latestHeightMapData);
      request.setStartFootPose(RobotSide.LEFT, new Pose3D(dataset.getPlannerInput().getStartPosition().getX(),
                                                          dataset.getPlannerInput().getStartPosition().getY(),
                                                          dataset.getPlannerInput().getStartPosition().getZ(),
                                                          dataset.getPlannerInput().getStartYaw(),
                                                          0.0, 0.0));

      request.setStartFootPose(RobotSide.RIGHT, new Pose3D(dataset.getPlannerInput().getStartPosition().getX(),
                                                           dataset.getPlannerInput().getStartPosition().getY(),
                                                           dataset.getPlannerInput().getStartPosition().getZ(),
                                                           dataset.getPlannerInput().getStartYaw(),
                                                           0.0, 0.0));

      request.setGoalFootPose(RobotSide.LEFT, new Pose3D(dataset.getPlannerInput().getGoalPosition().getX(),
                                                         dataset.getPlannerInput().getGoalPosition().getY(),
                                                         dataset.getPlannerInput().getGoalPosition().getZ(),
                                                         dataset.getPlannerInput().getGoalYaw(),
                                                         0.0, 0.0));

      request.setGoalFootPose(RobotSide.RIGHT, new Pose3D(dataset.getPlannerInput().getGoalPosition().getX(),
                                                          dataset.getPlannerInput().getGoalPosition().getY(),
                                                          dataset.getPlannerInput().getGoalPosition().getZ(),
                                                          dataset.getPlannerInput().getGoalYaw(),
                                                          0.0, 0.0));


      request.setRequestedInitialStanceSide(RobotSide.LEFT);
      request.setTimeout(0.05);

      LogTools.debug("Start: {}, Goal: {}, Origin: {}",
                     request.getStartFootPoses().get(RobotSide.LEFT),
                     request.getGoalFootPoses().get(RobotSide.LEFT),
                     request.getTerrainMapData().getSensorOrigin());

      return request;
   }

   public static FootstepPlannerRequest createASFPRequest(TerrainMapData terrainMapData,
                                                          HeightMapData heightMapData,
                                                          DataSet dataset,
                                                          FootstepPlan referencePlan)
   {
      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.setHeightMapData(heightMapData);
      request.setTerrainMapData(terrainMapData);
      request.setStartFootPose(RobotSide.LEFT, new Pose3D(dataset.getPlannerInput().getStartPosition().getX(),
                                                          dataset.getPlannerInput().getStartPosition().getY(),
                                                          dataset.getPlannerInput().getStartPosition().getZ(),
                                                          dataset.getPlannerInput().getStartYaw(),
                                                          0.0, 0.0));

      request.setStartFootPose(RobotSide.RIGHT, new Pose3D(dataset.getPlannerInput().getStartPosition().getX(),
                                                           dataset.getPlannerInput().getStartPosition().getY(),
                                                           dataset.getPlannerInput().getStartPosition().getZ(),
                                                           dataset.getPlannerInput().getStartYaw(),
                                                           0.0, 0.0));

      request.setGoalFootPose(RobotSide.LEFT, new Pose3D(dataset.getPlannerInput().getGoalPosition().getX(),
                                                         dataset.getPlannerInput().getGoalPosition().getY(),
                                                         dataset.getPlannerInput().getGoalPosition().getZ(),
                                                         dataset.getPlannerInput().getGoalYaw(),
                                                         0.0, 0.0));

      request.setGoalFootPose(RobotSide.RIGHT, new Pose3D(dataset.getPlannerInput().getGoalPosition().getX(),
                                                          dataset.getPlannerInput().getGoalPosition().getY(),
                                                          dataset.getPlannerInput().getGoalPosition().getZ(),
                                                          dataset.getPlannerInput().getGoalYaw(),
                                                          0.0, 0.0));

      request.setTimeout(2.5);
      request.setPlanBodyPath(false);
      request.setSnapGoalSteps(true);
      request.setPerformAStarSearch(true);
      request.setAssumeFlatGround(false);
      request.setSwingPlannerType(SwingPlannerType.NONE);
      request.setAbortIfGoalStepSnappingFails(true);
      request.setReferencePlan(referencePlan);
      request.setRequestedInitialStanceSide(RobotSide.LEFT); // 0 -> left, 1 -> right

      return request;
   }

}
