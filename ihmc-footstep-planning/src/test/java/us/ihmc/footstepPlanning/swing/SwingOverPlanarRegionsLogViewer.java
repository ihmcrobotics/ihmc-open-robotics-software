package us.ihmc.footstepPlanning.swing;

import java.awt.Color;
import java.io.File;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.trajectories.SwingOverPlanarRegionsTrajectoryExpander;
import us.ihmc.commonWalkingControlModules.trajectories.SwingOverPlanarRegionsVisualizer;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.AStarBodyPathPlannerParameters;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.log.FootstepPlannerLog;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogLoader;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SwingOverPlanarRegionsLogViewer
{
   public SwingOverPlanarRegionsLogViewer(String fileName)
   {
      FootstepPlannerLogLoader logLoader = new FootstepPlannerLogLoader();
      File file = new File(getClass().getClassLoader().getResource(fileName).getFile());

      if (logLoader.load(file) != FootstepPlannerLogLoader.LoadResult.LOADED)
      {
         return;
      }

      FootstepPlannerLog log = logLoader.getLog();

      FootstepDataListMessage footsteps = log.getStatusPacket().getFootstepDataList();
      FootstepPlan footstepPlan = new FootstepPlan();
      for (int i = 0; i < footsteps.getFootstepDataList().size(); i++)
      {
         FootstepDataMessage message = footsteps.getFootstepDataList().get(i);
         footstepPlan.addFootstep(RobotSide.fromByte(message.getRobotSide()),
                                  new FramePose3D(ReferenceFrame.getWorldFrame(), message.getLocation(), message.getOrientation()));
      }
      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.setFromPacket(log.getRequestPacket());

      SwingPlannerParametersBasics parameters = new DefaultSwingPlannerParameters();
      parameters.set(log.getSwingPlannerParametersPacket());

      PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(log.getStatusPacket().getPlanarRegionsList());

      WalkingControllerParameters walkingControllerParameters = new ProxyAtlasWalkingControllerParameters();
      ConvexPolygon2D foot = ProxyAtlasWalkingControllerParameters.getProxyAtlasFootPolygon();

      SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>(ProxyAtlasWalkingControllerParameters::getProxyAtlasFootPolygon);
      FootstepPlanningModule planningModule = new FootstepPlanningModule(getClass().getSimpleName(),
                                                                         new DefaultVisibilityGraphParameters(),
                                                                         new AStarBodyPathPlannerParameters(),
                                                                         new DefaultFootstepPlannerParameters(),
                                                                         parameters,
                                                                         walkingControllerParameters,
                                                                         footPolygons,
                                                                         null);

      Graphics3DObject startGraphics = new Graphics3DObject();
      Graphics3DObject endGraphics = new Graphics3DObject();
      startGraphics.addExtrudedPolygon(foot, 0.02, YoAppearance.Color(Color.blue));
      endGraphics.addExtrudedPolygon(foot, 0.02, YoAppearance.Color(Color.RED));

      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      YoGraphicsListRegistry yoGraphicsListRegistry = planningModule.getSwingOverPlanarRegionsTrajectoryExpander().getGraphicsListRegistry();

      YoFramePoint3D firstWaypoint = new YoFramePoint3D("firstWaypoint", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoint3D secondWaypoint = new YoFramePoint3D("secondWaypoint", ReferenceFrame.getWorldFrame(), registry);

      yoGraphicsListRegistry.registerYoGraphic("outputWaypoints", new YoGraphicPosition("firstWaypoint", firstWaypoint, 0.02, YoAppearance.White()));
      yoGraphicsListRegistry.registerYoGraphic("outputWaypoints", new YoGraphicPosition("secondWaypoint", secondWaypoint, 0.02, YoAppearance.White()));

      PlanarRegionsListDefinedEnvironment environment = new PlanarRegionsListDefinedEnvironment("environment", planarRegionsList, 1e-2, false);

      SwingOverPlanarRegionsTrajectoryExpander expander = planningModule.getSwingOverPlanarRegionsTrajectoryExpander();

      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Dummy"));

      SwingOverPlanarRegionsVisualizer visualizer = new SwingOverPlanarRegionsVisualizer(scs, registry, yoGraphicsListRegistry, foot, expander);
      expander.attachVisualizer(visualizer::update);

      scs.setDT(1.0, 1);
      scs.addYoRegistry(registry);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.setGroundVisible(false);
      scs.addStaticLinkGraphics(environment.getTerrainObject3D().getLinkGraphics());

      planningModule.getSwingPlanningModule().computeSwingWaypoints(request.getHeightMapData(),
                                                                    footstepPlan,
                                                                    request.getStartFootPoses(),
                                                                    SwingPlannerType.TWO_WAYPOINT_POSITION);

      scs.startOnAThread();
      scs.cropBuffer();
      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      String fileName = "20200717_143721207_FootstepPlannerLog";
      SwingOverPlanarRegionsLogViewer viewer = new SwingOverPlanarRegionsLogViewer(fileName);
   }
}
