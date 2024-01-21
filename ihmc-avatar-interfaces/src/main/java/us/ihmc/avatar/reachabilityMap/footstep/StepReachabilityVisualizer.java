package us.ihmc.avatar.reachabilityMap.footstep;

import us.ihmc.commonWalkingControlModules.staticReachability.StepReachabilityData;
import us.ihmc.commonWalkingControlModules.staticReachability.StepReachabilityLatticePoint;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerEnvironmentHandler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnappingTools;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.stepChecking.FootstepPoseHeuristicChecker;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.registry.YoRegistry;

public class StepReachabilityVisualizer
{
   private enum Mode
   {
      REACHABILITY_GRADIENT, REACHABILITY_MAP, HEURISTIC_REJECTION_REASON, CHECKER_COMPARISON
   }

   private static final Mode mode = Mode.CHECKER_COMPARISON;

   private final double reachabilityThreshold = 2.2;
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
   private final DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
   private final FootstepPlannerEnvironmentHandler environmentHandler = new FootstepPlannerEnvironmentHandler();
   private final FootstepSnapAndWiggler snapper = new FootstepSnapAndWiggler(footPolygons, parameters, environmentHandler);
   private final FootstepPoseHeuristicChecker checker = new FootstepPoseHeuristicChecker(parameters, snapper, registry);
   private final StepReachabilityData stepReachabilityData;

   public StepReachabilityVisualizer(StepReachabilityData stepReachabilityData)
   {
      this.stepReachabilityData = stepReachabilityData;

      // Set up SCS and coordinate object
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(true, 16000);
      SimulationConstructionSet scs = new SimulationConstructionSet(parameters);
      Graphics3DObject coordinate = new Graphics3DObject();
      coordinate.addCoordinateSystem(0.5);
      scs.addStaticLinkGraphics(coordinate);
      scs.setGroundVisible(false);
      scs.setCameraFix(0.0, 0.0, 1.0);
      scs.setCameraPosition(8.0, 0.0, 3.0);
      scs.startOnAThread();

      for (StepReachabilityLatticePoint latticePoint : stepReachabilityData.getLegReachabilityMap().keySet())
      {
         // Represent foot position as an arrow
         Graphics3DObject validStep = new Graphics3DObject();
         validStep.translate(latticePoint.getXIndex(), latticePoint.getYIndex(), latticePoint.getZIndex());
         validStep.rotate(Math.toRadians(90), new Vector3D(0,1.0,0));
         validStep.rotate(latticePoint.getYawIndex() * stepReachabilityData.getGridSizeYaw() / stepReachabilityData.getYawDivisions(), new Vector3D(1.0,0,0));

         BipedalFootstepPlannerNodeRejectionReason rejectionReason = checkStepHeuristicValidity(latticePoint);

         switch (mode)
         {
            case REACHABILITY_GRADIENT:
            {
               // Reachability for this foot position indicated by green-red gradient
               double reachabilityValue = stepReachabilityData.getLegReachabilityMap().get(latticePoint);
               if (reachabilityValue > 40)
                  reachabilityValue = 40;
               AppearanceDefinition appearance = YoAppearance.RGBColor(reachabilityValue / 40, (40 - reachabilityValue) / 40, 0);
               validStep.addArrow(0.4, appearance, appearance);
               scs.addStaticLinkGraphics(validStep);
               break;
            }
            case REACHABILITY_MAP:
            {
               // Reachability for this foot position indicated by green/red color
               AppearanceDefinition appearance;
               double reachabilityValue = stepReachabilityData.getLegReachabilityMap().get(latticePoint);
               if (reachabilityValue <= reachabilityThreshold)
                  appearance = YoAppearance.Green();
               else
                  appearance = YoAppearance.Red();
               validStep.addArrow(0.4, appearance, appearance);
               scs.addStaticLinkGraphics(validStep);
               break;
            }
            case HEURISTIC_REJECTION_REASON:
            {
               AppearanceDefinition appearance;
               if (rejectionReason == BipedalFootstepPlannerNodeRejectionReason.STEP_NOT_WIDE_ENOUGH)
                  appearance = YoAppearance.Pink();
               else if (rejectionReason == BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_FAR)
                  appearance = YoAppearance.Blue();
               else if (rejectionReason == BipedalFootstepPlannerNodeRejectionReason.STEP_YAWS_TOO_MUCH)
                  appearance = YoAppearance.Red();
               else if (rejectionReason == BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_WIDE)
                  appearance = YoAppearance.Purple();
               else if (rejectionReason == BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_HIGH_OR_LOW)
                  appearance = YoAppearance.YellowGreen();
               else if (rejectionReason == BipedalFootstepPlannerNodeRejectionReason.STEP_NOT_LONG_ENOUGH)
                  appearance = YoAppearance.HotPink();
               else
               {
                  appearance = YoAppearance.White();
                  LogTools.info(rejectionReason);
               }
               validStep.addArrow(0.4, appearance, appearance);
               scs.addStaticLinkGraphics(validStep);
               break;
            }
            case CHECKER_COMPARISON:
            {
               AppearanceDefinition appearance = null;
               boolean addArrow = false;

               if (stepReachabilityData.getLegReachabilityMap().get(latticePoint) < reachabilityThreshold)
               {
                  addArrow = true;
                  // Both reachability map and heuristic accept
                  if (rejectionReason == null)
                  {
                     appearance = YoAppearance.Green();
                  }

                  // Reachability map accepts and heuristic doesn't
                  else if (rejectionReason == BipedalFootstepPlannerNodeRejectionReason.STEP_NOT_WIDE_ENOUGH)
                  {
                     appearance = YoAppearance.Pink();
                  }
                  else if (rejectionReason == BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_FAR)
                  {
                     appearance = YoAppearance.Blue();
                  }
                  else if (rejectionReason == BipedalFootstepPlannerNodeRejectionReason.STEP_YAWS_TOO_MUCH)
                  {
                     appearance = YoAppearance.Red();
                  }
                  else if (rejectionReason == BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_WIDE)
                  {
                     appearance = YoAppearance.Purple();
                  }
                  else if (rejectionReason == BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_HIGH_OR_LOW)
                  {
                     appearance = YoAppearance.YellowGreen();
                  }
                  else if (rejectionReason == BipedalFootstepPlannerNodeRejectionReason.STEP_NOT_LONG_ENOUGH)
                  {
                     appearance = YoAppearance.White();
                  }
                  else
                  {
                     appearance = YoAppearance.HotPink();
                     LogTools.info(rejectionReason);
                  }
               }
               // Heuristic accepts and reachability map doesn't
               else if (rejectionReason == null)
               {
                  addArrow = true;
                  appearance = YoAppearance.Black();
               }
               // If both reject don't render an arrow

               if (addArrow)
               {
                  validStep.addArrow(0.4, appearance, appearance);
                  scs.addStaticLinkGraphics(validStep);
               }

               break;
            }
         }
      }
   }

   private BipedalFootstepPlannerNodeRejectionReason checkStepHeuristicValidity(StepReachabilityLatticePoint latticePoint)
   {
      double stepX = latticePoint.getXIndex() * stepReachabilityData.getXyzSpacing();
      double stepY = latticePoint.getYIndex() * stepReachabilityData.getXyzSpacing();
      double stepZ = latticePoint.getZIndex() * stepReachabilityData.getXyzSpacing();
      double stepYaw = latticePoint.getYawIndex() * (stepReachabilityData.getGridSizeYaw() / stepReachabilityData.getYawDivisions());

      DiscreteFootstep candidateStep = new DiscreteFootstep(stepX, stepY, stepYaw, RobotSide.LEFT);
      DiscreteFootstep stanceStep = new DiscreteFootstep(0, 0, 0, RobotSide.RIGHT);

      FootstepSnapData candidateSnapData = new FootstepSnapData();
      Pose3D snappedCandidateStep = new Pose3D();
      snappedCandidateStep.getPosition().set(stepX, stepY, stepZ);
      snappedCandidateStep.getOrientation().setToYawOrientation(stepYaw);
      candidateSnapData.getSnapTransform().set(FootstepSnappingTools.computeSnapTransform(candidateStep, snappedCandidateStep));

      FootstepSnapData stanceSnapData = new FootstepSnapData();
      stanceSnapData.getSnapTransform().set(new RigidBodyTransform());

      snapper.addSnapData(candidateStep, candidateSnapData);

      BipedalFootstepPlannerNodeRejectionReason rejectionReason = checker.snapAndCheckValidity(candidateStep, stanceStep, null);
      return rejectionReason;
   }
}
