package us.ihmc.footstepPlanning.swing;

import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.log.FootstepPlannerLog;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogLoader;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.idl.IDLSequence;
import us.ihmc.javafx.IdMappedColorFunction;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.graphics.Graphics3DObjectTools;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

import java.awt.*;
import java.util.Random;

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

      FootstepPlannerParametersBasics footstepPlannerParameters = new DefaultFootstepPlannerParameters();
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
      // Using interpolation between key color points
      double r = 0, g = 0, b = 0;
      double redR = 1.0, redG = 0.0, redB = 0.0;
      double magentaR = 1.0, magentaG = 0.0, magentaB = 1.0;
      double orangeR = 1.0, orangeG = 200.0 / 255.0, orangeB = 0.0;
      double yellowR = 1.0, yellowG = 1.0, yellowB = 0.0;
      double blueR = 0.0, blueG = 0.0, blueB = 1.0;
      double greenR = 0.0, greenG = 1.0, greenB = 0.0;
      double gradientSize = 0.2;
      double gradientLength = 1.0;
      double alpha = height % gradientLength;
      if (alpha < 0)
         alpha = 1 + alpha;
      while (alpha > 5 * gradientSize)
         alpha -= 5 * gradientSize;

      if (alpha <= gradientSize * 1)
      {
         r = InterpolationTools.linearInterpolate(magentaR, blueR, (alpha) / gradientSize);
         g = InterpolationTools.linearInterpolate(magentaG, blueG, (alpha) / gradientSize);
         b = InterpolationTools.linearInterpolate(magentaB, blueB, (alpha) / gradientSize);
      }
      else if (alpha <= gradientSize * 2)
      {
         r = InterpolationTools.linearInterpolate(blueR, greenR, (alpha - gradientSize * 1) / gradientSize);
         g = InterpolationTools.linearInterpolate(blueG, greenG, (alpha - gradientSize * 1) / gradientSize);
         b = InterpolationTools.linearInterpolate(blueB, greenB, (alpha - gradientSize * 1) / gradientSize);
      }
      else if (alpha <= gradientSize * 3)
      {
         r = InterpolationTools.linearInterpolate(greenR, yellowR, (alpha - gradientSize * 2) / gradientSize);
         g = InterpolationTools.linearInterpolate(greenG, yellowG, (alpha - gradientSize * 2) / gradientSize);
         b = InterpolationTools.linearInterpolate(greenB, yellowB, (alpha - gradientSize * 2) / gradientSize);
      }
      else if (alpha <= gradientSize * 4)
      {
         r = InterpolationTools.linearInterpolate(yellowR, orangeR, (alpha - gradientSize * 3) / gradientSize);
         g = InterpolationTools.linearInterpolate(yellowG, orangeG, (alpha - gradientSize * 3) / gradientSize);
         b = InterpolationTools.linearInterpolate(yellowB, orangeB, (alpha - gradientSize * 3) / gradientSize);
      }
      else if (alpha <= gradientSize * 5)
      {
         r = InterpolationTools.linearInterpolate(orangeR, redR, (alpha - gradientSize * 4) / gradientSize);
         g = InterpolationTools.linearInterpolate(orangeG, redG, (alpha - gradientSize * 4) / gradientSize);
         b = InterpolationTools.linearInterpolate(orangeB, redB, (alpha - gradientSize * 4) / gradientSize);
      }
      else
      {
         throw new RuntimeException("no valid color");
      }

      if (r == 0.0 && g == 0.0 && b == 0.0)
         throw new RuntimeException("Shouldn't return black.)");
      return YoAppearance.Color(new Color((float) r, (float) g, (float) b, 1.0f));
   }

   public static void main(String[] args)
   {
      new CollisionFreeSwingCalculatorLogViewer();
   }
}
