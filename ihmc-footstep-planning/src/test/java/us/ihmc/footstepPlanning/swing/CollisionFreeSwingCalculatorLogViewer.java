package us.ihmc.footstepPlanning.swing;

import perception_msgs.TerrainMapMessage;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.fastddsjava.cdr.idl.IDLFloatSequence;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.log.FootstepPlannerLog;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogLoader;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.perception.gpuMapping.HeightMapTools;
import us.ihmc.perception.gpuMapping.TerrainMapData;
import us.ihmc.perception.gpuMapping.TerrainMapMessageTools;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

import java.awt.*;

public class CollisionFreeSwingCalculatorLogViewer
{
   public CollisionFreeSwingCalculatorLogViewer()
   {
      FootstepPlannerLogLoader logLoader = new FootstepPlannerLogLoader();
      FootstepPlannerLogLoader.LoadResult loadResult = logLoader.load();

      if (loadResult != FootstepPlannerLogLoader.LoadResult.LOADED)
      {
         return;
      }

      FootstepPlannerLog log = logLoader.getLog();

      SwingPlannerParametersBasics swingPlannerParameters = new DefaultSwingPlannerParameters();
      swingPlannerParameters.set(log.getSwingPlannerParametersPacket());

      DefaultFootstepPlannerParametersBasics footstepPlannerParameters = new DefaultFootstepPlannerParameters();
      footstepPlannerParameters.set(log.getFootstepParametersPacket());

      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Dummy"));
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

      TerrainMapMessage terrainMapMessage = log.getRequestPacket().getTerrainMapMessage();
      TerrainMapData terrainMapData = TerrainMapMessageTools.unpackMessage(terrainMapMessage);

      Graphics3DObject terrainGraphics = new Graphics3DObject();

      IDLFloatSequence heights = terrainMapMessage.getHeightMap();
      double gridResolutionXY = terrainMapMessage.getCellSizeInMeters();
      int centerIndex = HeightMapTools.computeCenterIndex(terrainMapMessage.getWidthInMeters(), gridResolutionXY);

      for (int key = 0; key < heights.size(); key++)
      {
         int xIndex = HeightMapTools.keyToXIndex(key, centerIndex);
         int yIndex = HeightMapTools.keyToYIndex(key, centerIndex);
         double x = HeightMapTools.indexToCoordinate(xIndex, terrainMapMessage.getGridCenterX(), gridResolutionXY, centerIndex);
         double y = HeightMapTools.indexToCoordinate(yIndex, terrainMapMessage.getGridCenterY(), gridResolutionXY, centerIndex);
         double height = heights.get(key);

         terrainGraphics.translate(x, y, 0.5 * height);
         terrainGraphics.addCube(terrainMapData.getCellSize(), terrainMapData.getCellSize(), height, true, computeColorFromHeight(height));
         terrainGraphics.identity();
      }
      
      scs.addStaticLinkGraphics(terrainGraphics);

      CollisionFreeSwingCalculator swingCalculator = new CollisionFreeSwingCalculator(footstepPlannerParameters,
                                                                                      swingPlannerParameters,
                                                                                      new ProxyAtlasWalkingControllerParameters(),
                                                                                      new SideDependentList<>(ProxyAtlasWalkingControllerParameters::getProxyAtlasFootPolygon),
                                                                                      scs,
                                                                                      graphicsListRegistry,
                                                                                      scs.getRootRegistry());

      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      scs.setGroundVisible(false);
      scs.startOnAThread();

      FootstepPlan footstepPlan = FootstepDataMessageConverter.convertToFootstepPlan(log.getStatusPacket().getFootstepDataList());
      SideDependentList<Pose3D> initialFootPoses = new SideDependentList<>(log.getRequestPacket().getStartLeftFootPose().getPose(),
                                                                          log.getRequestPacket().getStartRightFootPose().getPose());

//      swingCalculator.setPlanarRegionsList(planarRegionsList);
      swingCalculator.setTerrainMapData(terrainMapData);
      swingCalculator.computeSwingTrajectories(initialFootPoses, footstepPlan);
      scs.cropBuffer();
   }

   public static AppearanceDefinition computeColorFromHeight(double height)
   {
      double[] redGreenBlue = HeightMapTools.getRedGreenBlue(height);
      return YoAppearance.Color(new Color((float) redGreenBlue[0], (float) redGreenBlue[1], (float) redGreenBlue[2], 1.0f));
   }

   public static void main(String[] args)
   {
      new CollisionFreeSwingCalculatorLogViewer();
   }
}
