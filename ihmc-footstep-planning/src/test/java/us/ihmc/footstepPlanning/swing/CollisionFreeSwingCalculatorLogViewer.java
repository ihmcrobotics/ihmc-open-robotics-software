package us.ihmc.footstepPlanning.swing;

import javafx.scene.paint.Color;
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
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.javafx.IdMappedColorFunction;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.graphics.Graphics3DObjectTools;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

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

      swingPlannerParameters.setPercentageLowMaxDisplacement(0.08);
      swingPlannerParameters.setMaxDisplacementLow(0.007);
      swingPlannerParameters.setExtraSizeHigh(Axis3D.X, 0.24);
      swingPlannerParameters.setExtraSizeHigh(Axis3D.Z, 0.18);
      swingPlannerParameters.setExtraSizePercentageLow(Axis3D.Z, 0.14);
      swingPlannerParameters.setExtraSizePercentageHigh(Axis3D.Z, 0.27);
      swingPlannerParameters.setMotionCorrelationAlpha(0.73);

      FootstepPlannerParametersBasics footstepPlannerParameters = new DefaultFootstepPlannerParameters();
      footstepPlannerParameters.set(log.getFootstepParametersPacket());

      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Dummy"));
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

      PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(log.getRequestPacket().getPlanarRegionsListMessage());

      Graphics3DObject regionsGraphic = new Graphics3DObject();
      IdMappedColorFunction colorMapper = IdMappedColorFunction.INSTANCE;
      Random random = new Random(0xC0FEFE);
      for (int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++)
      {
         Color color = colorMapper.apply(random.nextInt(200));
         Graphics3DObjectTools.addPlanarRegion(regionsGraphic, planarRegionsList.getPlanarRegion(i), 0.01, YoAppearance.RGBColor(color.getRed(), color.getGreen(), color.getBlue()));
      }
      scs.addStaticLinkGraphics(regionsGraphic);

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

      swingCalculator.setPlanarRegionsList(planarRegionsList);
      swingCalculator.computeSwingTrajectories(initialFootPoses, footstepPlan);
      scs.cropBuffer();
   }

   public static void main(String[] args)
   {
      new CollisionFreeSwingCalculatorLogViewer();
   }
}
