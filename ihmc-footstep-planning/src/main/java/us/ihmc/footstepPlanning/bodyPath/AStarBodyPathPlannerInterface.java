package us.ihmc.footstepPlanning.bodyPath;

import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.log.AStarBodyPathEdgeData;
import us.ihmc.footstepPlanning.log.AStarBodyPathIterationData;
import us.ihmc.pathPlanning.graph.structure.GraphEdge;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.HashMap;
import java.util.List;

public interface AStarBodyPathPlannerInterface
{
    void handleRequest(FootstepPlannerRequest request, FootstepPlannerOutput outputToPack);

    void clearLoggedData();

    BodyPathLatticePoint getNextNode();

    void halt();

    List<AStarBodyPathIterationData> getIterationData();

    HashMap<GraphEdge<BodyPathLatticePoint>, AStarBodyPathEdgeData> getEdgeDataMap();

    YoRegistry getRegistry();
}
