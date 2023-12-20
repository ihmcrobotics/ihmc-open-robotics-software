package us.ihmc.footstepPlanning.graphSearch;

import javafx.scene.paint.Color;
import us.ihmc.commonWalkingControlModules.polygonWiggling.GradientDescentStepConstraintSolver;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.PlanarRegionFootstepSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.log.FootstepPlannerEdgeData;
import us.ihmc.footstepPlanning.log.FootstepPlannerIterationData;
import us.ihmc.footstepPlanning.log.FootstepPlannerLog;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogLoader;
import us.ihmc.footstepPlanning.swing.ProxyAtlasWalkingControllerParameters;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.javafx.IdMappedColorFunction;
import us.ihmc.pathPlanning.graph.structure.GraphEdge;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.graphics.Graphics3DObjectTools;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;

import java.util.List;
import java.util.Map;
import java.util.Random;

public class GradientDescentStepConstraintSolverLogViewer
{
   public GradientDescentStepConstraintSolverLogViewer()
   {
      FootstepPlannerLogLoader logLoader = new FootstepPlannerLogLoader();
      FootstepPlannerLogLoader.LoadResult loadResult = logLoader.load();

      if (loadResult != FootstepPlannerLogLoader.LoadResult.LOADED)
      {
         return;
      }

      FootstepPlannerLog log = logLoader.getLog();
      SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>(ProxyAtlasWalkingControllerParameters::getProxyAtlasFootPolygon);

      FootstepPlannerParametersBasics footstepPlannerParameters = new DefaultFootstepPlannerParameters();
      footstepPlannerParameters.set(log.getFootstepParametersPacket());
      footstepPlannerParameters.setMinClearanceFromStance(0.1);

      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Dummy"));
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

      PlanarRegionFootstepSnapAndWiggler snapAndWiggler = new PlanarRegionFootstepSnapAndWiggler(footPolygons,
                                                                                     footstepPlannerParameters,
                                                                                     scs,
                                                                                     graphicsListRegistry,
                                                                                     scs.getRootRegistry());
      PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(log.getRequestPacket().getPlanarRegionsListMessage());
      snapAndWiggler.setPlanarRegionsList(planarRegionsList);

      Graphics3DObject regionsGraphic = new Graphics3DObject();
      IdMappedColorFunction colorMapper = IdMappedColorFunction.INSTANCE;
      Random random = new Random(0xC0FEFE);
      for (int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++)
      {
         Color color = colorMapper.apply(random.nextInt(200));
         Graphics3DObjectTools.addPlanarRegion(regionsGraphic, planarRegionsList.getPlanarRegion(i), 0.01, YoAppearance.RGBColor(color.getRed(), color.getGreen(), color.getBlue()));
      }

      scs.addStaticLinkGraphics(regionsGraphic);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      scs.setGroundVisible(false);
      SimulationOverheadPlotterFactory plotterFactory = scs.createSimulationOverheadPlotterFactory();
      plotterFactory.addYoGraphicsListRegistries(graphicsListRegistry);
      plotterFactory.createOverheadPlotter();

      scs.startOnAThread();

      FootstepPlannerIterationData firstIterationData = log.getIterationData().get(0);
      FootstepGraphNode node = firstIterationData.getParentNode();
      Map<GraphEdge<FootstepGraphNode>, FootstepPlannerEdgeData> edgeDataMap = log.getEdgeDataMap();
      GraphEdge<FootstepGraphNode> solutionEdge = findSolutionEdge(node, firstIterationData, edgeDataMap);

      while (true)
      {
         /* perform snap, updates scs, etc. */
         snapAndWiggler.snapFootstep(solutionEdge.getEndNode().getSecondStep(), solutionEdge.getEndNode().getFirstStep(), true);

         /* parent node of the subsequent iteration */
         FootstepGraphNode newParentNode = solutionEdge.getEndNode();

         /* try to find iteration where this node was expanded */
         FootstepPlannerIterationData iterationData = findIterationData(newParentNode, log.getIterationData());

         /* if it's not there, this is the end of the plan */
         if (iterationData == null)
            break;

         solutionEdge = findSolutionEdge(newParentNode, iterationData, edgeDataMap);
      }

      scs.cropBuffer();
   }

   private static GraphEdge<FootstepGraphNode> findSolutionEdge(FootstepGraphNode node,
                                                                FootstepPlannerIterationData iterationData,
                                                                Map<GraphEdge<FootstepGraphNode>, FootstepPlannerEdgeData> edgeDataMap)
   {
      List<FootstepGraphNode> childNodes = iterationData.getChildNodes();
      for (int i = 0; i < childNodes.size(); i++)
      {
         GraphEdge<FootstepGraphNode> graphEdge = new GraphEdge<>(node, childNodes.get(i));
         if (edgeDataMap.get(graphEdge).isSolutionEdge())
            return graphEdge;
      }

      return null;
   }

   private static FootstepPlannerIterationData findIterationData(FootstepGraphNode parentNode, List<FootstepPlannerIterationData> iterationDataList)
   {
      for (int i = 0; i < iterationDataList.size(); i++)
      {
         if (iterationDataList.get(i).getParentNode().equals(parentNode))
            return iterationDataList.get(i);
      }

      return null;
   }

   public static void main(String[] args)
   {
      new GradientDescentStepConstraintSolverLogViewer();
   }
}
