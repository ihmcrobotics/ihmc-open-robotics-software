package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerEnvironmentHandler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapDataReadOnly;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstepTools;
import us.ihmc.footstepPlanning.graphSearch.stepExpansion.IdealStepCalculatorInterface;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.function.ToDoubleFunction;

public class FootstepCostCalculator implements FootstepCostCalculatorInterface
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final FootstepPlannerParametersReadOnly parameters;
   private final FootstepSnapperReadOnly snapper;
   private final IdealStepCalculatorInterface idealStepCalculator;
   private final ToDoubleFunction<FootstepGraphNode> heuristics;
   private final SideDependentList<? extends ConvexPolygon2DReadOnly> footPolygons;
   private final SideDependentList<ConvexPolygon2DReadOnly> scaledFootPolygons = new SideDependentList<>();

   private final RigidBodyTransform stanceStepTransform = new RigidBodyTransform();
   private final RigidBodyTransform idealStepTransform = new RigidBodyTransform();
   private final RigidBodyTransform candidateStepTransform = new RigidBodyTransform();
   private final YoDouble edgeCost = new YoDouble("edgeCost", registry);
   private final YoDouble totalCost = new YoDouble("totalCost", registry);
   private final YoDouble heuristicCost = new YoDouble("heuristicCost", registry);
   private final YoDouble idealStepHeuristicCost = new YoDouble("idealStepHeuristicCost", registry);

   private HeightMapData heightMapData = null;
   private final YoBoolean cliffDetected = new YoBoolean("cliffDetected", registry);
   private final ConvexPolygon2D scaledFootPolygon = new ConvexPolygon2D();
   private final Plane3D bestFitPlane = new Plane3D();

   private final FootstepPlannerEnvironmentHandler environmentHandler;

   public FootstepCostCalculator(FootstepPlannerParametersReadOnly parameters,
                                 FootstepSnapperReadOnly snapper,
                                 IdealStepCalculatorInterface idealStepCalculator,
                                 ToDoubleFunction<FootstepGraphNode> heuristics,
                                 SideDependentList<? extends ConvexPolygon2DReadOnly> footPolygons,
                                 FootstepPlannerEnvironmentHandler environmentHandler,
                                 YoRegistry parentRegistry)
   {
      this.parameters = parameters;
      this.snapper = snapper;
      this.idealStepCalculator = idealStepCalculator;
      this.heuristics = heuristics;
      this.footPolygons = footPolygons;
      this.environmentHandler = environmentHandler;

      for (RobotSide robotSide : RobotSide.values())
      {
         ConvexPolygon2D scaledFootPolygon = new ConvexPolygon2D(footPolygons.get(robotSide));
         scaledFootPolygon.scale(1.0 + parameters.getFootPolygonScaleFactor());
         scaledFootPolygons.put(robotSide, scaledFootPolygon);
      }

      parentRegistry.addChild(registry);
   }

   @Override
   public double computeCost(DiscreteFootstep candidateStep, DiscreteFootstep stanceStep, DiscreteFootstep startOfSwing)
   {
      DiscreteFootstep idealStep = idealStepCalculator.computeIdealStep(stanceStep, startOfSwing);

      DiscreteFootstepTools.getSnappedStepTransform(stanceStep, snapper.snapFootstep(stanceStep).getSnapTransform(), stanceStepTransform);
      FootstepSnapDataReadOnly candidateSnapData = snapper.snapFootstep(candidateStep);
      DiscreteFootstepTools.getSnappedStepTransform(candidateStep, candidateSnapData.getSnapTransform(), candidateStepTransform);
      idealStepTransform.getTranslation().set(idealStep.getX(), idealStep.getY(), stanceStepTransform.getTranslationZ());
      idealStepTransform.getRotation().setToYawOrientation(idealStep.getYaw());

      // calculate offset from ideal in a z-up frame
      stanceStepTransform.getRotation().setToYawOrientation(stanceStepTransform.getRotation().getYaw());
      idealStepTransform.preMultiplyInvertOther(stanceStepTransform);
      candidateStepTransform.preMultiplyInvertOther(stanceStepTransform);

      double xOffset = candidateStepTransform.getTranslationX() - idealStepTransform.getTranslationX();
      double yOffset = candidateStepTransform.getTranslationY() - idealStepTransform.getTranslationY();
      double zOffset = candidateStepTransform.getTranslationZ() - idealStepTransform.getTranslationZ();
      double yawOffset = AngleTools.computeAngleDifferenceMinusPiToPi(candidateStepTransform.getRotation().getYaw(), idealStepTransform.getRotation().getYaw());
      double pitchOffset = AngleTools.computeAngleDifferenceMinusPiToPi(candidateStepTransform.getRotation().getPitch(), idealStepTransform.getRotation().getPitch());
      double rollOffset = AngleTools.computeAngleDifferenceMinusPiToPi(candidateStepTransform.getRotation().getRoll(), idealStepTransform.getRotation().getRoll());

      edgeCost.set(0.0);
      edgeCost.add(Math.abs(xOffset * parameters.getForwardWeight()));
      edgeCost.add(Math.abs(yOffset * parameters.getLateralWeight()));
      edgeCost.add(Math.abs(zOffset * (zOffset > 0.0 ? parameters.getStepUpWeight() : parameters.getStepDownWeight())));
      edgeCost.add(Math.abs(yawOffset * parameters.getYawWeight()));
      edgeCost.add(Math.abs(pitchOffset * parameters.getPitchWeight()));
      edgeCost.add(Math.abs(rollOffset * parameters.getRollWeight()));

      if (environmentHandler.hasFallbackHeightMap() && candidateSnapData.getSnappedToHeightMap())
      {
         double rmsError = candidateSnapData.getRMSErrorHeightMap();
         double rmsAlpha = EuclidCoreTools.clamp(
               (rmsError - parameters.getRMSMinErrorToPenalize()) / (parameters.getRMSErrorThreshold() - parameters.getRMSMinErrorToPenalize()),
               0.0,
               1.0);
         edgeCost.add(rmsAlpha * parameters.getRMSErrorCost());
      }

      if (heightMapData != null)
      {
         edgeCost.add(computeHeightMapCliffCost(candidateStep));
      }

      edgeCost.add(computeAreaCost(candidateStep));
      edgeCost.add(parameters.getCostPerStep());

      // subtract off heuristic cost difference - i.e. ignore difference in goal proximity due to step adjustment
      idealStepHeuristicCost.set(heuristics.applyAsDouble(new FootstepGraphNode(stanceStep, idealStep)));
      heuristicCost.set(heuristics.applyAsDouble(new FootstepGraphNode(stanceStep, candidateStep)));
      double deltaHeuristics = idealStepHeuristicCost.getDoubleValue() - heuristicCost.getDoubleValue();

      if(deltaHeuristics > 0.0)
      {
         edgeCost.add(deltaHeuristics);
      }
      else
      {
         // TODO
         edgeCost.set(Math.max(0.0, edgeCost.getValue() - deltaHeuristics));
      }

      totalCost.set(edgeCost.getDoubleValue() + heuristicCost.getDoubleValue());
      return edgeCost.getValue();
   }

   public void setHeightMapData(HeightMapData heightMapData)
   {
      this.heightMapData = heightMapData;
   }

   private double computeAreaCost(DiscreteFootstep footstep)
   {
      FootstepSnapDataReadOnly snapData = snapper.snapFootstep(footstep);
      if (snapData != null)
      {
         double area;
         if (!environmentHandler.hasFallbackHeightMap() || !snapData.getSnappedToHeightMap())
         {
            ConvexPolygon2DReadOnly footholdAfterSnap = snapData.getCroppedFoothold();
            if(footholdAfterSnap.isEmpty() || footholdAfterSnap.containsNaN())
            {
               return 0.0;
            }

            area = footholdAfterSnap.getArea();
         }
         else
         {
            area = snapData.getHeightMapArea();
         }

         double footArea = footPolygons.get(footstep.getRobotSide()).getArea();
         double percentAreaUnoccupied = Math.max(0.0, 1.0 - area / footArea);
         return percentAreaUnoccupied * parameters.getFootholdAreaWeight();
      }
      else
      {
         return 0.0;
      }
   }

   private double computeHeightMapCliffCost(DiscreteFootstep footstep)
   {
      /* Transform to step location */
      PerceptionDebugTools.printHeightMap("PlannerHeightMap:", heightMapData, 25);
      FootstepSnapDataReadOnly snapData = snapper.snapFootstep(footstep);
      DiscreteFootstepTools.getFootPolygon(footstep, scaledFootPolygons.get(footstep.getRobotSide()), scaledFootPolygon);
      RigidBodyTransformReadOnly snapTransform = snapData.getSnapTransform();
      scaledFootPolygon.applyTransform(snapTransform, false);

      /* Compute best-fit plane */
      RigidBodyTransformReadOnly snappedStepTransform = snapData.getSnappedStepTransform(footstep);
      bestFitPlane.getPoint().set(snappedStepTransform.getTranslation());
      bestFitPlane.getNormal().set(Axis3D.Z);
      snappedStepTransform.getRotation().transform(bestFitPlane.getNormal());

      for (int pointIdx = 0; pointIdx < scaledFootPolygon.getNumberOfVertices(); pointIdx++)
      {
         Point2DReadOnly polygonVertex = scaledFootPolygon.getVertex(pointIdx);
         double zBestFitPlane = bestFitPlane.getZOnPlane(polygonVertex.getX(), polygonVertex.getY());
         double zHeightMap = heightMapData.getHeightAt(polygonVertex.getX(), polygonVertex.getY());
         double distanceFromBestFitPlane = Math.abs(zBestFitPlane - zHeightMap);

         /* Compute cliff cost */
         if (distanceFromBestFitPlane > parameters.getCliffHeight())
         {
            cliffDetected.set(true);
            // Hack to prevent us from using this node
            return 24;
         }
      }

      cliffDetected.set(false);
      return 0.0;
   }

   public void resetLoggedVariables()
   {
      edgeCost.setToNaN();
      totalCost.setToNaN();
      heuristicCost.setToNaN();
   }
}
