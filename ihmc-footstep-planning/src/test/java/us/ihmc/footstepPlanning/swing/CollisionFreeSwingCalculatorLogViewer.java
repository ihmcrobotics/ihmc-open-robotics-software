package us.ihmc.footstepPlanning.swing;

import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.euclid.geometry.Pose3D;
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
import us.ihmc.idl.IDLSequence;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;
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

      HeightMapMessage heightMapMessage = log.getRequestPacket().getHeightMapMessage();
      HeightMapData heightMapData = HeightMapMessageTools.unpackMessage(heightMapMessage);

      Graphics3DObject terrainGraphics = new Graphics3DObject();

      IDLSequence.Float heights = heightMapMessage.getHeights();
      double gridResolutionXY = heightMapMessage.getXyResolution();
      int centerIndex = HeightMapTools.computeCenterIndex(heightMapMessage.getGridSizeXy(), gridResolutionXY);

      for (int i = 0; i < heights.size(); i++)
      {
         int xIndex = HeightMapTools.keyToXIndex(heightMapMessage.getKeys().get(i), centerIndex);
         int yIndex = HeightMapTools.keyToYIndex(heightMapMessage.getKeys().get(i), centerIndex);
         double x = HeightMapTools.indexToCoordinate(xIndex, heightMapMessage.getGridCenterX(), gridResolutionXY, centerIndex);
         double y = HeightMapTools.indexToCoordinate(yIndex, heightMapMessage.getGridCenterY(), gridResolutionXY, centerIndex);
         double height = heights.get(i);

         terrainGraphics.translate(x, y, 0.5 * height);
         terrainGraphics.addCube(heightMapData.getGridResolutionXY(), heightMapData.getGridResolutionXY(), height, true, computeColorFromHeight(height));
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
      SideDependentList<Pose3D> initialFootPoses = new SideDependentList<>(log.getRequestPacket().getStartLeftFootPose(), log.getRequestPacket().getStartRightFootPose());

//      swingCalculator.setPlanarRegionsList(planarRegionsList);
      swingCalculator.setHeightMapData(heightMapData);
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
