package us.ihmc.footstepPlanning.graphSearch;

import java.util.HashMap;
import java.util.Random;
import java.util.function.UnaryOperator;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnappingTools;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.SideDependentList;

public class FootstepCostCalculatorTest
{
   @Test
   public void testFootstepCostCalculation()
   {
      Random random = new Random(1776L);

      DefaultFootstepPlannerParameters footstepPlannerParameters = new DefaultFootstepPlannerParameters();
      SideDependentList<ConvexPolygon2D> defaultFootPolygons = PlannerTools.createDefaultFootPolygons();
      FootstepNodeSnapAndWiggler snapper = new FootstepNodeSnapAndWiggler(defaultFootPolygons, footstepPlannerParameters);

      HashMap<FootstepNode, FootstepNode> idealStepMap = new HashMap<>();
      UnaryOperator<FootstepNode> idealStepCalculator = node -> idealStepMap.computeIfAbsent(node, n -> FootstepNode.generateRandomFootstepNode(random, 2.0));

      FootstepCostCalculator stepCostCalculator = new FootstepCostCalculator(footstepPlannerParameters, snapper, idealStepCalculator, node -> 10.0, defaultFootPolygons, null);
      int numberOfTests = 1000;

      for (int i = 0; i < numberOfTests; i++)
      {
         FootstepNode stanceNode = FootstepNode.generateRandomFootstepNode(random, 10.0);
         FootstepNode idealStepNode = idealStepCalculator.apply(stanceNode);

         FramePose3D stanceFoot = new FramePose3D();
         double stanceHeight = EuclidCoreRandomTools.nextDouble(random, -10.0, 10.0);
         double stancePitch = EuclidCoreRandomTools.nextDouble(random, -0.25 * Math.PI, 0.25 * Math.PI);
         double stanceRoll = EuclidCoreRandomTools.nextDouble(random, -0.25 * Math.PI, 0.25 * Math.PI);
         stanceFoot.getPosition().set(stanceNode.getX(), stanceNode.getY(), stanceHeight);
         stanceFoot.getOrientation().set(new Quaternion(stanceNode.getYaw(), stancePitch, stanceRoll));

         FramePose3D idealStep = new FramePose3D();
         idealStep.getPosition().set(idealStepNode.getX(), idealStepNode.getY(), stanceFoot.getZ());
         idealStep.getOrientation().setYawPitchRoll(idealStepNode.getYaw(), 0.0, 0.0);

         // test ideal step cost equals base step cost
         snapper.reset();
         snapper.addSnapData(stanceNode, new FootstepNodeSnapData(FootstepNodeSnappingTools.computeSnapTransform(stanceNode, stanceFoot), new ConvexPolygon2D()));
         snapper.addSnapData(idealStepNode, new FootstepNodeSnapData(FootstepNodeSnappingTools.computeSnapTransform(idealStepNode, idealStep), new ConvexPolygon2D()));
         double stepCost = stepCostCalculator.computeCost(stanceNode, idealStepNode);
         Assertions.assertTrue(MathTools.epsilonEquals(stepCost, footstepPlannerParameters.getCostPerStep(), 1e-10), "Ideal step cost does not equal per step cost.");

         // test partial area cost for ideal step
         ConvexPolygon2D foothold = PlannerTools.createDefaultFootPolygon();
         double percentAreaCost = random.nextDouble();
         double percentFoothold = EuclidCoreTools.interpolate(1.0, footstepPlannerParameters.getMinimumFootholdPercent(), percentAreaCost);
         foothold.scale(Math.sqrt(percentFoothold));

         snapper.reset();
         snapper.addSnapData(stanceNode, new FootstepNodeSnapData(FootstepNodeSnappingTools.computeSnapTransform(stanceNode, stanceFoot), new ConvexPolygon2D()));
         snapper.addSnapData(idealStepNode, new FootstepNodeSnapData(FootstepNodeSnappingTools.computeSnapTransform(idealStepNode, idealStep), foothold));
         stepCost = stepCostCalculator.computeCost(stanceNode, idealStepNode);
         double expectedCost = footstepPlannerParameters.getCostPerStep() + footstepPlannerParameters.getFootholdAreaWeight() * percentAreaCost;
         Assertions.assertTrue(MathTools.epsilonEquals(stepCost, expectedCost, 1e-10), "Area based cost does not equal expected value.");

         // test random footstep cost
         FootstepNode randomNode = new FootstepNode(EuclidCoreRandomTools.nextDouble(random, 2.0),
                                                    EuclidCoreRandomTools.nextDouble(random, 2.0),
                                                    EuclidCoreRandomTools.nextDouble(random, Math.PI),
                                                    idealStepNode.getRobotSide());

         double randomStepHeight = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double randomStepPitch = EuclidCoreRandomTools.nextDouble(random, 0.25 * Math.PI);
         double randomStepRoll = EuclidCoreRandomTools.nextDouble(random, 0.25 * Math.PI);
         FramePose3D randomStepPose = new FramePose3D();
         randomStepPose.getPosition().set(randomNode.getX(), randomNode.getY(), randomStepHeight);
         randomStepPose.getOrientation().set(new Quaternion(randomNode.getYaw(), randomStepPitch, randomStepRoll));

         FramePose3D stanceFootZUpPose = new FramePose3D();
         stanceFootZUpPose.getPosition().set(stanceFoot.getPosition());
         stanceFootZUpPose.getOrientation().setYawPitchRoll(stanceNode.getYaw(), 0.0, 0.0);
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
         snapper.addSnapData(stanceNode, new FootstepNodeSnapData(FootstepNodeSnappingTools.computeSnapTransform(stanceNode, stanceFoot), new ConvexPolygon2D()));
         snapper.addSnapData(randomNode, new FootstepNodeSnapData(FootstepNodeSnappingTools.computeSnapTransform(randomNode, randomStepPose), new ConvexPolygon2D()));
         randomStepPose.changeFrame(stanceFootZUpFrame);

         stepCost = stepCostCalculator.computeCost(stanceNode, randomNode);
         Assertions.assertTrue(MathTools.epsilonEquals(stepCost, expectedCost, 1e-10), "Pose based cost does not equal expected value.");
      }
   }
}
