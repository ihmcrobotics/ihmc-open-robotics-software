package us.ihmc.footstepPlanning.polygonWiggling;

import java.util.List;
import java.util.Random;

import controller_msgs.msg.dds.FootstepPlannerParametersPacket;
import javafx.scene.paint.Color;
import us.ihmc.commonWalkingControlModules.polygonWiggling.GradientDescentStepConstraintInput;
import us.ihmc.commonWalkingControlModules.polygonWiggling.GradientDescentStepConstraintSolver;
import us.ihmc.commonWalkingControlModules.polygonWiggling.WiggleParameters;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.shape.primitives.Cylinder3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnappingTools;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstepTools;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.log.FootstepPlannerEdgeData;
import us.ihmc.footstepPlanning.log.FootstepPlannerLog;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogLoader;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.javaFXVisualizers.IdMappedColorFunction;
import us.ihmc.pathPlanning.graph.structure.GraphEdge;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.RigidBodyTransformGenerator;
import us.ihmc.robotics.graphics.Graphics3DObjectTools;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoRegistry;

public class GradientDescentStepConstraintSolverVisualizer
{
   public GradientDescentStepConstraintSolverVisualizer()
   {
      FootstepPlannerLogLoader logLoader = new FootstepPlannerLogLoader();
      FootstepPlannerLogLoader.LoadResult loadResult = logLoader.load();

      if (loadResult != FootstepPlannerLogLoader.LoadResult.LOADED)
      {
         return;
      }

      FootstepPlannerLog log = logLoader.getLog();
      List<FootstepGraphNode> footstepPlan = log.getFootstepPlan();
      PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(log.getRequestPacket().getPlanarRegionsListMessage());

      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("testRobot"));
      scs.setGroundVisible(false);
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      GradientDescentStepConstraintSolver gradientDescentStepConstraintSolver = new GradientDescentStepConstraintSolver(scs, graphicsListRegistry, registry);

      Graphics3DObject regionsGraphic = new Graphics3DObject();
      IdMappedColorFunction colorMapper = IdMappedColorFunction.INSTANCE;
      Random random = new Random(0xDEADBEEF);
      for (int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++)
      {
         Color color = colorMapper.apply(random.nextInt(200));
         Graphics3DObjectTools.addPlanarRegion(regionsGraphic, planarRegionsList.getPlanarRegion(i), 0.01, YoAppearance.RGBColor(color.getRed(), color.getGreen(), color.getBlue()));
      }
      scs.addStaticLinkGraphics(regionsGraphic);

      scs.setDT(0.1, 1);

      graphicsListRegistry.addArtifactListsToPlotter(scs.createSimulationOverheadPlotterFactory().createOverheadPlotter().getPlotter());
      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      scs.getRootRegistry().addChild(registry);
      scs.startOnAThread();

      GradientDescentStepConstraintInput gradientDescentStepConstraintInput = new GradientDescentStepConstraintInput();
      SideDependentList<ConvexPolygon2D> footPolygons = getProxyAtlasFootPolygons();

      ConvexPolygon2D footPolygon = new ConvexPolygon2D();
      RigidBodyTransform tempTransform = new RigidBodyTransform();
      Cylinder3D legCollisionShape = new Cylinder3D();
      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      RigidBodyTransformGenerator transformGenerator = new RigidBodyTransformGenerator();
      RigidBodyTransform legCollisionShapeToSoleTransform = new RigidBodyTransform();
      parameters.set(log.getFootstepParametersPacket());

      int maxIndex = footstepPlan.size();
      for (int i = 1; i < maxIndex; i++)
      {
         FootstepGraphNode footstepNode = footstepPlan.get(i);
         FootstepPlannerEdgeData edgeData = log.getEdgeDataMap().get(new GraphEdge<>(footstepPlan.get(i - 1), footstepNode));
         FootstepSnapData snapData = edgeData.getEndStepSnapData();
         int regionIndex = snapData.getRegionIndex();
         if (regionIndex == -1)
         {
            // could happen because it's the goal step and the provided snap is trusted
            continue;
         }

         PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(regionIndex);

         // from FootstepNodeSnapAndWiggler#computeWiggleTransform
         DiscreteFootstepTools.getFootPolygon(footstepNode.getSecondStep(), footPolygons.get(footstepNode.getSecondStepSide()), footPolygon);
         tempTransform.set(snapData.getSnapTransform());
         tempTransform.preMultiply(planarRegion.getTransformToLocal());
         ConvexPolygon2D footPolygonInRegionFrame = FootstepSnappingTools.computeTransformedPolygon(footPolygon, tempTransform);

         gradientDescentStepConstraintInput.clear();
         gradientDescentStepConstraintInput.setInitialStepPolygon(footPolygonInRegionFrame);
         gradientDescentStepConstraintInput.setWiggleParameters(getWiggleParameters(log.getFootstepParametersPacket()));
         gradientDescentStepConstraintInput.setPlanarRegion(planarRegion);
         gradientDescentStepConstraintInput.setPlanarRegionsList(planarRegionsList);

         RigidBodyTransform snappedNodeTransform = snapData.getSnappedStepTransform(footstepNode.getSecondStep());
         tempTransform.set(snappedNodeTransform);
         tempTransform.preMultiply(planarRegion.getTransformToLocal());
         gradientDescentStepConstraintInput.setFootstepInRegionFrame(tempTransform);

         double forwardPoint = footPolygon.getMaxX() + parameters.getShinToeClearance();
         double backwardPoint = footPolygon.getMinX() - parameters.getShinHeelClearance();
         double shinRadius = 0.5 * (forwardPoint - backwardPoint);
         double shinXOffset = 0.5 * (forwardPoint + backwardPoint);

         legCollisionShape.setSize(parameters.getShinLength(), shinRadius);
         transformGenerator.identity();
         transformGenerator.translate(shinXOffset, 0.0, parameters.getShinHeightOffset());
         transformGenerator.translate(0.0, 0.0, 0.5 * parameters.getShinLength());
         transformGenerator.getRigidyBodyTransform(legCollisionShapeToSoleTransform);
         gradientDescentStepConstraintSolver.setLegCollisionShape(legCollisionShape, legCollisionShapeToSoleTransform);

         gradientDescentStepConstraintSolver.wigglePolygon(gradientDescentStepConstraintInput);
      }

      scs.cropBuffer();
   }

   private static WiggleParameters getWiggleParameters(FootstepPlannerParametersPacket parameters)
   {
      WiggleParameters wiggleParameters = new WiggleParameters();
      wiggleParameters.deltaInside = parameters.getWiggleInsideDeltaTarget();
      wiggleParameters.maxX = parameters.getMaximumXyWiggleDistance();
      wiggleParameters.minX = -parameters.getMaximumXyWiggleDistance();
      wiggleParameters.maxY = parameters.getMaximumXyWiggleDistance();
      wiggleParameters.minY = -parameters.getMaximumXyWiggleDistance();
      wiggleParameters.maxYaw = parameters.getMaximumYawWiggle();
      wiggleParameters.minYaw = -parameters.getMaximumYawWiggle();
      return wiggleParameters;
   }

   // TODO log foot polygons...
   private static SideDependentList<ConvexPolygon2D> getProxyAtlasFootPolygons()
   {
      ConvexPolygon2D footPolygon = new ConvexPolygon2D();
      footPolygon.addVertex(0.11, 0.043);
      footPolygon.addVertex(0.11, -0.043);
      footPolygon.addVertex(-0.11, 0.055);
      footPolygon.addVertex(-0.11, -0.055);
      footPolygon.update();

      return new SideDependentList<>(() -> new ConvexPolygon2D(footPolygon));
   }

   public static void main(String[] args)
   {
      new GradientDescentStepConstraintSolverVisualizer();
   }
}
