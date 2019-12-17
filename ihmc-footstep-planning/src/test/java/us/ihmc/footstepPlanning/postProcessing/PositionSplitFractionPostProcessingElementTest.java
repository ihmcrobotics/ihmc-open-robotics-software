package us.ihmc.footstepPlanning.postProcessing;

import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepPlanningRequestPacket;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.FootstepPostProcessingPacket;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.configurations.ICPPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.footstepPlanning.postProcessing.parameters.DefaultFootstepPostProcessingParameters;
import us.ihmc.footstepPlanning.postProcessing.parameters.FootstepPostProcessingParametersBasics;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.List;

import static us.ihmc.robotics.Assert.assertEquals;

public class PositionSplitFractionPostProcessingElementTest
{
   private static final double epsilon = 1e-7;

   @Test
   public void testStepDownWithClosingStep()
   {
      double width = 0.25;
      double stepLength = 0.3;
      RobotSide stanceSide = RobotSide.LEFT;
      ICPPlannerParameters icpPlannerParameters = getPlannerParameters();
      FootstepPostProcessingParametersBasics parameters = new DefaultFootstepPostProcessingParameters();

      double fullSplitFractionFromHeight = 0.895;
      double fullWeightDistributionFromHeight = 0.8;
      parameters.setTransferSplitFractionAtFullDepth(fullSplitFractionFromHeight);
      parameters.setTransferWeightDistributionAtFullDepth(fullWeightDistributionFromHeight);

      double minHeightToStartShifting = -parameters.getStepHeightForLargeStepDown();
      double heightForFullShift = -parameters.getLargestStepDownHeight();

      PositionSplitFractionPostProcessingElement postProcessingElement = new PositionSplitFractionPostProcessingElement(parameters, icpPlannerParameters);

      FootstepPostProcessingPacket output = new FootstepPostProcessingPacket();
      output.getLeftFootPositionInWorld().set(0.0, width / 2.0, 0.0);
      output.getRightFootPositionInWorld().set(0.0, -width / 2.0, 0.0);

      RecyclingArrayList<FootstepDataMessage> footstepList = output.getFootstepDataList().getFootstepDataList();
      FootstepDataMessage firstStep = footstepList.add();
      firstStep.setRobotSide(stanceSide.getOppositeSide().toByte());
      firstStep.getLocation().set(stepLength, stanceSide.negateIfLeftSide(width / 2.0), minHeightToStartShifting);

      FootstepDataMessage secondStep = footstepList.add();
      secondStep.setRobotSide(stanceSide.toByte());
      secondStep.getLocation().set(stepLength, stanceSide.negateIfRightSide(width / 2.0), minHeightToStartShifting);

      FootstepPostProcessingPacket processedOutput = postProcessingElement.postProcessFootstepPlan(output);
      assertEquals(output.getFootstepDataList().getFootstepDataList().size(), processedOutput.getFootstepDataList().getFootstepDataList().size());
      assertEquals(2, processedOutput.getFootstepDataList().getFootstepDataList().size());
      for (int i = 0; i < 2; i++)
      {
         FootstepDataMessage message = output.getFootstepDataList().getFootstepDataList().get(i);
         FootstepDataMessage processedMessage = processedOutput.getFootstepDataList().getFootstepDataList().get(i);

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(message.getLocation(), processedMessage.getLocation(), epsilon);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(message.getOrientation(), processedMessage.getOrientation(), epsilon);
      }
      assertEquals(-1.0, processedOutput.getFootstepDataList().getFinalTransferSplitFraction());
      assertEquals(-1.0, processedOutput.getFootstepDataList().getFinalTransferWeightDistribution());

      FootstepDataMessage firstProcessedStep = processedOutput.getFootstepDataList().getFootstepDataList().get(0);
      FootstepDataMessage secondProcessedStep = processedOutput.getFootstepDataList().getFootstepDataList().get(1);
      assertEquals(-1.0, firstProcessedStep.getTransferSplitFraction());
      assertEquals(-1.0, firstProcessedStep.getTransferWeightDistribution());
      assertEquals(-1.0, firstProcessedStep.getSwingDurationShiftFraction());
      assertEquals(-1.0, firstProcessedStep.getSwingSplitFraction());

      assertEquals(-1.0, secondProcessedStep.getTransferSplitFraction());
      assertEquals(-1.0, secondProcessedStep.getTransferWeightDistribution());
      assertEquals(-1.0, secondProcessedStep.getSwingDurationShiftFraction());
      assertEquals(-1.0, secondProcessedStep.getSwingSplitFraction());

      // check for full shift
      firstStep.getLocation().setZ(heightForFullShift - 0.05);
      secondStep.getLocation().setZ(heightForFullShift - 0.05);

      processedOutput = postProcessingElement.postProcessFootstepPlan(output);
      assertEquals(output.getFootstepDataList().getFootstepDataList().size(), processedOutput.getFootstepDataList().getFootstepDataList().size());
      assertEquals(2, processedOutput.getFootstepDataList().getFootstepDataList().size());
      for (int i = 0; i < 2; i++)
      {
         FootstepDataMessage message = output.getFootstepDataList().getFootstepDataList().get(i);
         FootstepDataMessage processedMessage = processedOutput.getFootstepDataList().getFootstepDataList().get(i);

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(message.getLocation(), processedMessage.getLocation(), epsilon);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(message.getOrientation(), processedMessage.getOrientation(), epsilon);
      }
      assertEquals(-1.0, processedOutput.getFootstepDataList().getFinalTransferSplitFraction());
      assertEquals(-1.0, processedOutput.getFootstepDataList().getFinalTransferWeightDistribution());

      firstProcessedStep = processedOutput.getFootstepDataList().getFootstepDataList().get(0);
      secondProcessedStep = processedOutput.getFootstepDataList().getFootstepDataList().get(1);
      assertEquals(-1.0, firstProcessedStep.getTransferSplitFraction());
      assertEquals(-1.0, firstProcessedStep.getTransferWeightDistribution());
      assertEquals(-1.0, firstProcessedStep.getSwingDurationShiftFraction());
      assertEquals(-1.0, firstProcessedStep.getSwingSplitFraction());

      assertEquals(fullSplitFractionFromHeight, secondProcessedStep.getTransferSplitFraction());
      assertEquals(fullWeightDistributionFromHeight, secondProcessedStep.getTransferWeightDistribution());
      assertEquals(-1.0, secondProcessedStep.getSwingDurationShiftFraction());
      assertEquals(-1.0, secondProcessedStep.getSwingSplitFraction());

      // check between starting to shift and full shift
      double incrementToCheck = 0.01;
      for (double height = minHeightToStartShifting - incrementToCheck; height >= heightForFullShift; height -= incrementToCheck)
      {
         firstStep.getLocation().setZ(height);
         secondStep.getLocation().setZ(height);

         processedOutput = postProcessingElement.postProcessFootstepPlan(output);
         assertEquals(output.getFootstepDataList().getFootstepDataList().size(), processedOutput.getFootstepDataList().getFootstepDataList().size());
         assertEquals(2, processedOutput.getFootstepDataList().getFootstepDataList().size());
         for (int i = 0; i < 2; i++)
         {
            FootstepDataMessage message = output.getFootstepDataList().getFootstepDataList().get(i);
            FootstepDataMessage processedMessage = processedOutput.getFootstepDataList().getFootstepDataList().get(i);

            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(message.getLocation(), processedMessage.getLocation(), epsilon);
            EuclidCoreTestTools.assertQuaternionGeometricallyEquals(message.getOrientation(), processedMessage.getOrientation(), epsilon);
         }
         assertEquals(-1.0, processedOutput.getFootstepDataList().getFinalTransferSplitFraction());
         assertEquals(-1.0, processedOutput.getFootstepDataList().getFinalTransferWeightDistribution());

         firstProcessedStep = processedOutput.getFootstepDataList().getFootstepDataList().get(0);
         secondProcessedStep = processedOutput.getFootstepDataList().getFootstepDataList().get(1);
         assertEquals(-1.0, firstProcessedStep.getTransferSplitFraction());
         assertEquals(-1.0, firstProcessedStep.getTransferWeightDistribution());
         assertEquals(-1.0, firstProcessedStep.getSwingDurationShiftFraction());
         assertEquals(-1.0, firstProcessedStep.getSwingSplitFraction());

         double alphaThrough = (minHeightToStartShifting - height) / (minHeightToStartShifting - heightForFullShift);
         double expectedSplitFraction = InterpolationTools.linearInterpolate(icpPlannerParameters.getTransferSplitFraction(), fullSplitFractionFromHeight, alphaThrough);
         double expectedWeightDistribution = InterpolationTools.linearInterpolate(0.5, fullWeightDistributionFromHeight, alphaThrough);

         assertEquals(expectedSplitFraction, secondProcessedStep.getTransferSplitFraction());
         assertEquals(expectedWeightDistribution, secondProcessedStep.getTransferWeightDistribution());
         assertEquals(-1.0, secondProcessedStep.getSwingDurationShiftFraction());
         assertEquals(-1.0, secondProcessedStep.getSwingSplitFraction());
      }
   }

   @Test
   public void testSingleLargeStepDown()
   {
      double width = 0.25;
      double stepLength = 0.3;
      RobotSide stanceSide = RobotSide.LEFT;
      ICPPlannerParameters icpPlannerParameters = getPlannerParameters();
      FootstepPostProcessingParametersBasics parameters = new DefaultFootstepPostProcessingParameters();

      double fullSplitFractionFromHeight = 0.895;
      double fullWeightDistributionFromHeight = 0.8;
      parameters.setTransferSplitFractionAtFullDepth(fullSplitFractionFromHeight);
      parameters.setTransferWeightDistributionAtFullDepth(fullWeightDistributionFromHeight);

      double minHeightToStartShifting = -parameters.getStepHeightForLargeStepDown();
      double heightForFullShift = -parameters.getLargestStepDownHeight();

      PositionSplitFractionPostProcessingElement postProcessingElement = new PositionSplitFractionPostProcessingElement(parameters, icpPlannerParameters);

      FootstepPostProcessingPacket output = new FootstepPostProcessingPacket();
      output.getLeftFootPositionInWorld().set(0.0, width / 2.0, 0.0);
      output.getRightFootPositionInWorld().set(0.0, -width / 2.0, 0.0);

      RecyclingArrayList<FootstepDataMessage> footstepList = output.getFootstepDataList().getFootstepDataList();
      FootstepDataMessage firstStep = footstepList.add();
      firstStep.setRobotSide(stanceSide.getOppositeSide().toByte());
      firstStep.getLocation().set(stepLength, stanceSide.negateIfLeftSide(width / 2.0), minHeightToStartShifting);

      FootstepPostProcessingPacket processedOutput = postProcessingElement.postProcessFootstepPlan(output);
      assertEquals(output.getFootstepDataList().getFootstepDataList().size(), processedOutput.getFootstepDataList().getFootstepDataList().size());
      assertEquals(1, processedOutput.getFootstepDataList().getFootstepDataList().size());
      FootstepDataMessage message = output.getFootstepDataList().getFootstepDataList().get(0);
      FootstepDataMessage processedMessage = processedOutput.getFootstepDataList().getFootstepDataList().get(0);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(message.getLocation(), processedMessage.getLocation(), epsilon);
      EuclidCoreTestTools.assertQuaternionGeometricallyEquals(message.getOrientation(), processedMessage.getOrientation(), epsilon);
      assertEquals(-1.0, processedOutput.getFootstepDataList().getFinalTransferSplitFraction());
      assertEquals(-1.0, processedOutput.getFootstepDataList().getFinalTransferWeightDistribution());

      FootstepDataMessage firstProcessedStep = processedOutput.getFootstepDataList().getFootstepDataList().get(0);
      assertEquals(-1.0, firstProcessedStep.getTransferSplitFraction());
      assertEquals(-1.0, firstProcessedStep.getTransferWeightDistribution());
      assertEquals(-1.0, firstProcessedStep.getSwingDurationShiftFraction());
      assertEquals(-1.0, firstProcessedStep.getSwingSplitFraction());

      // check for full shift
      firstStep.getLocation().setZ(heightForFullShift - 0.05);

      processedOutput = postProcessingElement.postProcessFootstepPlan(output);
      assertEquals(output.getFootstepDataList().getFootstepDataList().size(), processedOutput.getFootstepDataList().getFootstepDataList().size());
      assertEquals(1, processedOutput.getFootstepDataList().getFootstepDataList().size());
      message = output.getFootstepDataList().getFootstepDataList().get(0);
      processedMessage = processedOutput.getFootstepDataList().getFootstepDataList().get(0);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(message.getLocation(), processedMessage.getLocation(), epsilon);
      EuclidCoreTestTools.assertQuaternionGeometricallyEquals(message.getOrientation(), processedMessage.getOrientation(), epsilon);
      assertEquals(fullSplitFractionFromHeight, processedOutput.getFootstepDataList().getFinalTransferSplitFraction());
      assertEquals(fullWeightDistributionFromHeight, processedOutput.getFootstepDataList().getFinalTransferWeightDistribution());

      firstProcessedStep = processedOutput.getFootstepDataList().getFootstepDataList().get(0);
      assertEquals(-1.0, firstProcessedStep.getTransferSplitFraction());
      assertEquals(-1.0, firstProcessedStep.getTransferWeightDistribution());
      assertEquals(-1.0, firstProcessedStep.getSwingDurationShiftFraction());
      assertEquals(-1.0, firstProcessedStep.getSwingSplitFraction());

      // check between starting to shift and full shift
      double incrementToCheck = 0.01;
      for (double height = minHeightToStartShifting - incrementToCheck; height >= heightForFullShift; height -= incrementToCheck)
      {
         firstStep.getLocation().setZ(height);

         processedOutput = postProcessingElement.postProcessFootstepPlan(output);
         assertEquals(output.getFootstepDataList().getFootstepDataList().size(), processedOutput.getFootstepDataList().getFootstepDataList().size());
         assertEquals(1, processedOutput.getFootstepDataList().getFootstepDataList().size());

         message = output.getFootstepDataList().getFootstepDataList().get(0);
         processedMessage = processedOutput.getFootstepDataList().getFootstepDataList().get(0);

         double alphaThrough = (minHeightToStartShifting - height) / (minHeightToStartShifting - heightForFullShift);
         double expectedSplitFraction = InterpolationTools.linearInterpolate(icpPlannerParameters.getTransferSplitFraction(), fullSplitFractionFromHeight, alphaThrough);
         double expectedWeightDistribution = InterpolationTools.linearInterpolate(0.5, fullWeightDistributionFromHeight, alphaThrough);

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(message.getLocation(), processedMessage.getLocation(), epsilon);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(message.getOrientation(), processedMessage.getOrientation(), epsilon);
         assertEquals(expectedSplitFraction, processedOutput.getFootstepDataList().getFinalTransferSplitFraction());
         assertEquals(expectedWeightDistribution, processedOutput.getFootstepDataList().getFinalTransferWeightDistribution());

         firstProcessedStep = processedOutput.getFootstepDataList().getFootstepDataList().get(0);
         assertEquals(-1.0, firstProcessedStep.getTransferSplitFraction());
         assertEquals(-1.0, firstProcessedStep.getTransferWeightDistribution());
         assertEquals(-1.0, firstProcessedStep.getSwingDurationShiftFraction());
         assertEquals(-1.0, firstProcessedStep.getSwingSplitFraction());
      }
   }

   private ICPPlannerParameters getPlannerParameters()
   {
      ICPPlannerParameters parameters = new SmoothCMPPlannerParameters();
      return parameters;
   }
}
