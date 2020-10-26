package us.ihmc.footstepPlanning.graphSearch;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnappingTools;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstepTools;
import us.ihmc.footstepPlanning.graphSearch.stepExpansion.IdealStepCalculatorInterface;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.stepCost.FootstepCostCalculator;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.HashMap;
import java.util.Random;

public class FootstepCostCalculatorTest
{
   @Test
   public void testFootstepCostCalculation()
   {
      Random random = new Random(1776L);

      DefaultFootstepPlannerParameters footstepPlannerParameters = new DefaultFootstepPlannerParameters();
      SideDependentList<ConvexPolygon2D> defaultFootPolygons = PlannerTools.createDefaultFootPolygons();
      FootstepSnapAndWiggler snapper = new FootstepSnapAndWiggler(defaultFootPolygons, footstepPlannerParameters);

      HashMap<DiscreteFootstep, DiscreteFootstep> idealStepMap = new HashMap<>();
      IdealStepCalculatorInterface idealStepCalculator = (stance, startOfSwing) -> idealStepMap.computeIfAbsent(stance, n -> DiscreteFootstep.generateRandomFootstep(random, 2.0, n.getRobotSide().getOppositeSide()));

      YoRegistry registry = new YoRegistry("testRegistry");
      FootstepCostCalculator stepCostCalculator = new FootstepCostCalculator(footstepPlannerParameters, snapper, idealStepCalculator, node -> 10.0, defaultFootPolygons,
                                                                             registry);
      int numberOfTests = 1000;

      for (int i = 0; i < numberOfTests; i++)
      {
         DiscreteFootstep step1 = DiscreteFootstep.generateRandomFootstep(random, 10.0);
         DiscreteFootstep step0 = DiscreteFootstepTools.constructStepInPreviousStepFrame(0.0, 0.3, 0.0, step1);

         DiscreteFootstep idealStepNode = idealStepCalculator.computeIdealStep(step1, step0);

         FramePose3D stanceFoot = new FramePose3D();
         double stanceHeight = EuclidCoreRandomTools.nextDouble(random, -10.0, 10.0);
         double stancePitch = EuclidCoreRandomTools.nextDouble(random, -0.25 * Math.PI, 0.25 * Math.PI);
         double stanceRoll = EuclidCoreRandomTools.nextDouble(random, -0.25 * Math.PI, 0.25 * Math.PI);
         stanceFoot.getPosition().set(step1.getX(), step1.getY(), stanceHeight);
         stanceFoot.getOrientation().set(new Quaternion(step1.getYaw(), stancePitch, stanceRoll));

         FramePose3D idealStep = new FramePose3D();
         idealStep.getPosition().set(idealStepNode.getX(), idealStepNode.getY(), stanceFoot.getZ());
         idealStep.getOrientation().setYawPitchRoll(idealStepNode.getYaw(), 0.0, 0.0);

         // test ideal step cost equals base step cost
         snapper.reset();
         snapper.addSnapData(step1, new FootstepSnapData(FootstepSnappingTools.computeSnapTransform(step1, stanceFoot), new ConvexPolygon2D()));
         snapper.addSnapData(idealStepNode, new FootstepSnapData(FootstepSnappingTools.computeSnapTransform(idealStepNode, idealStep), new ConvexPolygon2D()));
         double stepCost = stepCostCalculator.computeCost(idealStepNode, step1, step0);
         Assertions.assertTrue(MathTools.epsilonEquals(stepCost, footstepPlannerParameters.getCostPerStep(), 1e-10), "Ideal step cost does not equal per step cost.");

         // test partial area cost for ideal step
         ConvexPolygon2D foothold = PlannerTools.createDefaultFootPolygon();
         double percentAreaCost = random.nextDouble();
         double percentFoothold = EuclidCoreTools.interpolate(1.0, footstepPlannerParameters.getMinimumFootholdPercent(), percentAreaCost);
         foothold.scale(Math.sqrt(percentFoothold));

         snapper.reset();
         snapper.addSnapData(step1, new FootstepSnapData(FootstepSnappingTools.computeSnapTransform(step1, stanceFoot), new ConvexPolygon2D()));
         snapper.addSnapData(idealStepNode, new FootstepSnapData(FootstepSnappingTools.computeSnapTransform(idealStepNode, idealStep), foothold));
         stepCost = stepCostCalculator.computeCost(idealStepNode, step1, step0);
         double expectedCost = footstepPlannerParameters.getCostPerStep() + footstepPlannerParameters.getFootholdAreaWeight() * percentAreaCost;
         Assertions.assertTrue(MathTools.epsilonEquals(stepCost, expectedCost, 1e-10), "Area based cost does not equal expected value.");

         // test random footstep cost
         DiscreteFootstep randomStep = new DiscreteFootstep(EuclidCoreRandomTools.nextDouble(random, 2.0),
                                                            EuclidCoreRandomTools.nextDouble(random, 2.0),
                                                            EuclidCoreRandomTools.nextDouble(random, Math.PI),
                                                            idealStepNode.getRobotSide());

         double randomStepHeight = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double randomStepPitch = EuclidCoreRandomTools.nextDouble(random, 0.25 * Math.PI);
         double randomStepRoll = EuclidCoreRandomTools.nextDouble(random, 0.25 * Math.PI);
         FramePose3D randomStepPose = new FramePose3D();
         randomStepPose.getPosition().set(randomStep.getX(), randomStep.getY(), randomStepHeight);
         randomStepPose.getOrientation().set(new Quaternion(randomStep.getYaw(), randomStepPitch, randomStepRoll));

         FramePose3D stanceFootZUpPose = new FramePose3D();
         stanceFootZUpPose.getPosition().set(stanceFoot.getPosition());
         stanceFootZUpPose.getOrientation().setYawPitchRoll(step1.getYaw(), 0.0, 0.0);
         PoseReferenceFrame stanceFootZUpFrame = new PoseReferenceFrame("stanceNodeFrame", stanceFootZUpPose);

         randomStepPose.changeFrame(stanceFootZUpFrame);
         idealStep.changeFrame(stanceFootZUpFrame);

         double deltaX = randomStepPose.getX() - idealStep.getX();
         double deltaY = randomStepPose.getY() - idealStep.getY();
         double deltaZ = randomStepPose.getZ() - idealStep.getZ();
         double deltaYaw = AngleTools.computeAngleDifferenceMinusPiToPi(randomStepPose.getYaw(), idealStep.getYaw());
         double deltaPitch = AngleTools.computeAngleDifferenceMinusPiToPi(randomStepPose.getPitch(), idealStep.getPitch());
         double deltaRoll = AngleTools.computeAngleDifferenceMinusPiToPi(randomStepPose.getRoll(), idealStep.getRoll());

         expectedCost = footstepPlannerParameters.getCostPerStep();
         expectedCost += Math.abs(deltaX * footstepPlannerParameters.getForwardWeight());
         expectedCost += Math.abs(deltaY * footstepPlannerParameters.getLateralWeight());
         expectedCost += Math.abs(deltaZ * (deltaZ > 0.0 ? footstepPlannerParameters.getStepUpWeight() : footstepPlannerParameters.getStepDownWeight()));
         expectedCost += Math.abs(deltaYaw * footstepPlannerParameters.getYawWeight());
         expectedCost += Math.abs(deltaPitch * footstepPlannerParameters.getPitchWeight());
         expectedCost += Math.abs(deltaRoll * footstepPlannerParameters.getRollWeight());

         randomStepPose.changeFrame(ReferenceFrame.getWorldFrame());

         snapper.reset();
         snapper.addSnapData(step1, new FootstepSnapData(FootstepSnappingTools.computeSnapTransform(step1, stanceFoot), new ConvexPolygon2D()));
         snapper.addSnapData(randomStep, new FootstepSnapData(FootstepSnappingTools.computeSnapTransform(randomStep, randomStepPose), new ConvexPolygon2D()));
         randomStepPose.changeFrame(stanceFootZUpFrame);

         stepCost = stepCostCalculator.computeCost(randomStep, step1, step0);
         Assertions.assertTrue(MathTools.epsilonEquals(stepCost, expectedCost, 1e-10), "Pose based cost does not equal expected value.");
      }
   }
}
