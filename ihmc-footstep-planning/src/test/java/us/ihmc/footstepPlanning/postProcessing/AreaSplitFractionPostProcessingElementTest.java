package us.ihmc.footstepPlanning.postProcessing;

import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepPostProcessingPacket;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.configurations.ICPPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.postProcessing.parameters.DefaultFootstepPostProcessingParameters;
import us.ihmc.footstepPlanning.postProcessing.parameters.FootstepPostProcessingParametersBasics;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.robotics.Assert.assertEquals;

public class AreaSplitFractionPostProcessingElementTest
{
   private static final double epsilon = 1e-7;

   @Test
   public void testShiftingFromArea()
   {
      double width = 0.25;
      double stepLength = 0.3;
      RobotSide stanceSide = RobotSide.LEFT;
      ICPPlannerParameters icpPlannerParameters = getPlannerParameters();
      FootstepPostProcessingParametersBasics parameters = new DefaultFootstepPostProcessingParameters();

      double fractionLoadIfOtherFootHasNoWidth = 0.5;
      double splitFractionIfOtherFootHasNoWidth = 0.5;
      double fractionLoadIfFootHasFullSupport = 0.8;
      double splitFractionIfFootHasFullSupport = 0.9;
      parameters.setFractionLoadIfOtherFootHasNoWidth(fractionLoadIfOtherFootHasNoWidth);
      parameters.setFractionTimeOnFootIfOtherFootHasNoWidth(splitFractionIfOtherFootHasNoWidth);
      parameters.setFractionLoadIfFootHasFullSupport(fractionLoadIfFootHasFullSupport);
      parameters.setFractionTimeOnFootIfFootHasFullSupport(splitFractionIfFootHasFullSupport);

      AreaSplitFractionPostProcessingElement postProcessingElement = new AreaSplitFractionPostProcessingElement(parameters, icpPlannerParameters, null);

      FootstepPostProcessingPacket output = new FootstepPostProcessingPacket();
      output.getLeftFootPositionInWorld().set(0.0, width / 2.0, 0.0);
      output.getRightFootPositionInWorld().set(0.0, -width / 2.0, 0.0);
      getFullSupportFoot().forEach(point -> output.getLeftFootContactPoints2d().add().set(point));
      getFullSupportFoot().forEach(point -> output.getRightFootContactPoints2d().add().set(point));

      RecyclingArrayList<FootstepDataMessage> footstepList = output.getFootstepDataList().getFootstepDataList();
      FootstepDataMessage firstStep = footstepList.add();
      firstStep.setRobotSide(stanceSide.getOppositeSide().toByte());
      firstStep.getLocation().set(stepLength, stanceSide.negateIfLeftSide(width / 2.0), 0.0);
      getLineSupportFoot().forEach(point -> firstStep.getPredictedContactPoints2d().add().set(point));

      FootstepDataMessage secondStep = footstepList.add();
      secondStep.setRobotSide(stanceSide.toByte());
      secondStep.getLocation().set(2.0 * stepLength, stanceSide.negateIfRightSide(width / 2.0), 0.0);
      getLineSupportFoot().forEach(point -> secondStep.getPredictedContactPoints2d().add().set(point));

      FootstepDataMessage thirdStep = footstepList.add();
      thirdStep.setRobotSide(stanceSide.getOppositeSide().toByte());
      thirdStep.getLocation().set(2.0 * stepLength, stanceSide.negateIfLeftSide(width / 2.0), 0.0);
      getFullSupportFoot().forEach(point -> thirdStep.getPredictedContactPoints2d().add().set(point));



      double expectedWeightShiftFromUnequal = ((fractionLoadIfFootHasFullSupport - 0.5) + (fractionLoadIfOtherFootHasNoWidth - 0.5)) / 2.0;
      double expectedSplitFractionFromUnequal = ((splitFractionIfFootHasFullSupport - 0.5) + (splitFractionIfOtherFootHasNoWidth - 0.5)) / 2.0;




      FootstepPostProcessingPacket processedOutput = postProcessingElement.postProcessFootstepPlan(output);
      assertEquals(output.getFootstepDataList().getFootstepDataList().size(), processedOutput.getFootstepDataList().getFootstepDataList().size());
      assertEquals(3, processedOutput.getFootstepDataList().getFootstepDataList().size());
      for (int i = 0; i < 3; i++)
      {
         FootstepDataMessage message = output.getFootstepDataList().getFootstepDataList().get(i);
         FootstepDataMessage processedMessage = processedOutput.getFootstepDataList().getFootstepDataList().get(i);

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(message.getLocation(), processedMessage.getLocation(), epsilon);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(message.getOrientation(), processedMessage.getOrientation(), epsilon);
      }


      FootstepDataMessage firstProcessedStep = processedOutput.getFootstepDataList().getFootstepDataList().get(0);
      FootstepDataMessage secondProcessedStep = processedOutput.getFootstepDataList().getFootstepDataList().get(1);
      FootstepDataMessage thirdProcessedStep = processedOutput.getFootstepDataList().getFootstepDataList().get(2);

      assertEquals(-1.0, firstProcessedStep.getTransferSplitFraction());
      assertEquals(-1.0, firstProcessedStep.getTransferWeightDistribution());
      assertEquals(-1.0, firstProcessedStep.getSwingDurationShiftFraction());
      assertEquals(-1.0, firstProcessedStep.getSwingSplitFraction());

      assertEquals(0.5 - expectedWeightShiftFromUnequal, secondProcessedStep.getTransferWeightDistribution());
      assertEquals(0.5 + expectedSplitFractionFromUnequal, secondProcessedStep.getTransferSplitFraction());
      assertEquals(-1.0, secondProcessedStep.getSwingDurationShiftFraction());
      assertEquals(-1.0, secondProcessedStep.getSwingSplitFraction());

      assertEquals(-1.0, thirdProcessedStep.getTransferWeightDistribution());
      assertEquals(-1.0, thirdProcessedStep.getTransferSplitFraction());
      assertEquals(-1.0, thirdProcessedStep.getSwingDurationShiftFraction());
      assertEquals(-1.0, thirdProcessedStep.getSwingSplitFraction());

      assertEquals(0.5 + expectedWeightShiftFromUnequal, processedOutput.getFootstepDataList().getFinalTransferWeightDistribution());
      assertEquals(0.5 - expectedSplitFractionFromUnequal, processedOutput.getFootstepDataList().getFinalTransferSplitFraction());
   }

   @Test
   public void testShiftingFromWidth()
   {
      double width = 0.25;
      double stepLength = 0.3;
      RobotSide stanceSide = RobotSide.LEFT;
      ICPPlannerParameters icpPlannerParameters = getPlannerParameters();
      FootstepPostProcessingParametersBasics parameters = new DefaultFootstepPostProcessingParameters();

      double fractionLoadIfOtherFootHasNoWidth = 0.8;
      double splitFractionIfOtherFootHasNoWidth = 0.9;
      double fractionLoadIfFootHasFullSupport = 0.5;
      double splitFractionIfFootHasFullSupport = 0.5;
      parameters.setFractionLoadIfOtherFootHasNoWidth(fractionLoadIfOtherFootHasNoWidth);
      parameters.setFractionTimeOnFootIfOtherFootHasNoWidth(splitFractionIfOtherFootHasNoWidth);
      parameters.setFractionLoadIfFootHasFullSupport(fractionLoadIfFootHasFullSupport);
      parameters.setFractionTimeOnFootIfFootHasFullSupport(splitFractionIfFootHasFullSupport);

      AreaSplitFractionPostProcessingElement postProcessingElement = new AreaSplitFractionPostProcessingElement(parameters, icpPlannerParameters, null);

      FootstepPostProcessingPacket output = new FootstepPostProcessingPacket();
      output.getLeftFootPositionInWorld().set(0.0, width / 2.0, 0.0);
      output.getRightFootPositionInWorld().set(0.0, -width / 2.0, 0.0);
      getFullSupportFoot().forEach(point -> output.getLeftFootContactPoints2d().add().set(point));
      getFullSupportFoot().forEach(point -> output.getRightFootContactPoints2d().add().set(point));

      RecyclingArrayList<FootstepDataMessage> footstepList = output.getFootstepDataList().getFootstepDataList();
      FootstepDataMessage firstStep = footstepList.add();
      firstStep.setRobotSide(stanceSide.getOppositeSide().toByte());
      firstStep.getLocation().set(stepLength, stanceSide.negateIfLeftSide(width / 2.0), 0.0);
      getLineSupportFoot().forEach(point -> firstStep.getPredictedContactPoints2d().add().set(point));

      FootstepDataMessage secondStep = footstepList.add();
      secondStep.setRobotSide(stanceSide.toByte());
      secondStep.getLocation().set(2.0 * stepLength, stanceSide.negateIfRightSide(width / 2.0), 0.0);
      getLineSupportFoot().forEach(point -> secondStep.getPredictedContactPoints2d().add().set(point));

      FootstepDataMessage thirdStep = footstepList.add();
      thirdStep.setRobotSide(stanceSide.getOppositeSide().toByte());
      thirdStep.getLocation().set(2.0 * stepLength, stanceSide.negateIfLeftSide(width / 2.0), 0.0);
      getFullSupportFoot().forEach(point -> thirdStep.getPredictedContactPoints2d().add().set(point));



      double expectedWeightShiftFromUnequal = ((fractionLoadIfFootHasFullSupport - 0.5) + (fractionLoadIfOtherFootHasNoWidth - 0.5)) / 2.0;
      double expectedSplitFractionFromUnequal = ((splitFractionIfFootHasFullSupport - 0.5) + (splitFractionIfOtherFootHasNoWidth - 0.5)) / 2.0;




      FootstepPostProcessingPacket processedOutput = postProcessingElement.postProcessFootstepPlan(output);
      assertEquals(output.getFootstepDataList().getFootstepDataList().size(), processedOutput.getFootstepDataList().getFootstepDataList().size());
      assertEquals(3, processedOutput.getFootstepDataList().getFootstepDataList().size());
      for (int i = 0; i < 3; i++)
      {
         FootstepDataMessage message = output.getFootstepDataList().getFootstepDataList().get(i);
         FootstepDataMessage processedMessage = processedOutput.getFootstepDataList().getFootstepDataList().get(i);

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(message.getLocation(), processedMessage.getLocation(), epsilon);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(message.getOrientation(), processedMessage.getOrientation(), epsilon);
      }


      FootstepDataMessage firstProcessedStep = processedOutput.getFootstepDataList().getFootstepDataList().get(0);
      FootstepDataMessage secondProcessedStep = processedOutput.getFootstepDataList().getFootstepDataList().get(1);
      FootstepDataMessage thirdProcessedStep = processedOutput.getFootstepDataList().getFootstepDataList().get(2);

      assertEquals(-1.0, firstProcessedStep.getTransferSplitFraction());
      assertEquals(-1.0, firstProcessedStep.getTransferWeightDistribution());
      assertEquals(-1.0, firstProcessedStep.getSwingDurationShiftFraction());
      assertEquals(-1.0, firstProcessedStep.getSwingSplitFraction());

      assertEquals(0.5 - expectedWeightShiftFromUnequal, secondProcessedStep.getTransferWeightDistribution());
      assertEquals(0.5 + expectedSplitFractionFromUnequal, secondProcessedStep.getTransferSplitFraction());
      assertEquals(-1.0, secondProcessedStep.getSwingDurationShiftFraction());
      assertEquals(-1.0, secondProcessedStep.getSwingSplitFraction());

      assertEquals(-1.0, thirdProcessedStep.getTransferWeightDistribution());
      assertEquals(-1.0, thirdProcessedStep.getTransferSplitFraction());
      assertEquals(-1.0, thirdProcessedStep.getSwingDurationShiftFraction());
      assertEquals(-1.0, thirdProcessedStep.getSwingSplitFraction());

      assertEquals(0.5 + expectedWeightShiftFromUnequal, processedOutput.getFootstepDataList().getFinalTransferWeightDistribution());
      assertEquals(0.5 - expectedSplitFractionFromUnequal, processedOutput.getFootstepDataList().getFinalTransferSplitFraction());
   }

   @Test
   public void testShiftingFromWidthAndArea()
   {
      double width = 0.25;
      double stepLength = 0.3;
      RobotSide stanceSide = RobotSide.LEFT;
      ICPPlannerParameters icpPlannerParameters = getPlannerParameters();
      FootstepPostProcessingParametersBasics parameters = new DefaultFootstepPostProcessingParameters();

      double fractionLoadIfOtherFootHasNoWidth = 0.8;
      double splitFractionIfOtherFootHasNoWidth = 0.9;
      double fractionLoadIfFootHasFullSupport = 0.75;
      double splitFractionIfFootHasFullSupport = 0.85;
      parameters.setFractionLoadIfOtherFootHasNoWidth(fractionLoadIfOtherFootHasNoWidth);
      parameters.setFractionTimeOnFootIfOtherFootHasNoWidth(splitFractionIfOtherFootHasNoWidth);
      parameters.setFractionLoadIfFootHasFullSupport(fractionLoadIfFootHasFullSupport);
      parameters.setFractionTimeOnFootIfFootHasFullSupport(splitFractionIfFootHasFullSupport);

      AreaSplitFractionPostProcessingElement postProcessingElement = new AreaSplitFractionPostProcessingElement(parameters, icpPlannerParameters, null);

      FootstepPostProcessingPacket output = new FootstepPostProcessingPacket();
      output.getLeftFootPositionInWorld().set(0.0, width / 2.0, 0.0);
      output.getRightFootPositionInWorld().set(0.0, -width / 2.0, 0.0);
      getFullSupportFoot().forEach(point -> output.getLeftFootContactPoints2d().add().set(point));
      getFullSupportFoot().forEach(point -> output.getRightFootContactPoints2d().add().set(point));

      RecyclingArrayList<FootstepDataMessage> footstepList = output.getFootstepDataList().getFootstepDataList();
      FootstepDataMessage firstStep = footstepList.add();
      firstStep.setRobotSide(stanceSide.getOppositeSide().toByte());
      firstStep.getLocation().set(stepLength, stanceSide.negateIfLeftSide(width / 2.0), 0.0);
      getLineSupportFoot().forEach(point -> firstStep.getPredictedContactPoints2d().add().set(point));

      FootstepDataMessage secondStep = footstepList.add();
      secondStep.setRobotSide(stanceSide.toByte());
      secondStep.getLocation().set(2.0 * stepLength, stanceSide.negateIfRightSide(width / 2.0), 0.0);
      getLineSupportFoot().forEach(point -> secondStep.getPredictedContactPoints2d().add().set(point));

      FootstepDataMessage thirdStep = footstepList.add();
      thirdStep.setRobotSide(stanceSide.getOppositeSide().toByte());
      thirdStep.getLocation().set(2.0 * stepLength, stanceSide.negateIfLeftSide(width / 2.0), 0.0);
      getFullSupportFoot().forEach(point -> thirdStep.getPredictedContactPoints2d().add().set(point));



      double expectedWeightShiftFromUnequal = ((fractionLoadIfFootHasFullSupport - 0.5) + (fractionLoadIfOtherFootHasNoWidth - 0.5)) / 2.0;
      double expectedSplitFractionFromUnequal = ((splitFractionIfFootHasFullSupport - 0.5) + (splitFractionIfOtherFootHasNoWidth - 0.5)) / 2.0;




      FootstepPostProcessingPacket processedOutput = postProcessingElement.postProcessFootstepPlan(output);
      assertEquals(output.getFootstepDataList().getFootstepDataList().size(), processedOutput.getFootstepDataList().getFootstepDataList().size());
      assertEquals(3, processedOutput.getFootstepDataList().getFootstepDataList().size());
      for (int i = 0; i < 3; i++)
      {
         FootstepDataMessage message = output.getFootstepDataList().getFootstepDataList().get(i);
         FootstepDataMessage processedMessage = processedOutput.getFootstepDataList().getFootstepDataList().get(i);

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(message.getLocation(), processedMessage.getLocation(), epsilon);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(message.getOrientation(), processedMessage.getOrientation(), epsilon);
      }


      FootstepDataMessage firstProcessedStep = processedOutput.getFootstepDataList().getFootstepDataList().get(0);
      FootstepDataMessage secondProcessedStep = processedOutput.getFootstepDataList().getFootstepDataList().get(1);
      FootstepDataMessage thirdProcessedStep = processedOutput.getFootstepDataList().getFootstepDataList().get(2);

      assertEquals(-1.0, firstProcessedStep.getTransferSplitFraction());
      assertEquals(-1.0, firstProcessedStep.getTransferWeightDistribution());
      assertEquals(-1.0, firstProcessedStep.getSwingDurationShiftFraction());
      assertEquals(-1.0, firstProcessedStep.getSwingSplitFraction());

      assertEquals(0.5 - expectedWeightShiftFromUnequal, secondProcessedStep.getTransferWeightDistribution());
      assertEquals(0.5 + expectedSplitFractionFromUnequal, secondProcessedStep.getTransferSplitFraction());
      assertEquals(-1.0, secondProcessedStep.getSwingDurationShiftFraction());
      assertEquals(-1.0, secondProcessedStep.getSwingSplitFraction());

      assertEquals(-1.0, thirdProcessedStep.getTransferWeightDistribution());
      assertEquals(-1.0, thirdProcessedStep.getTransferSplitFraction());
      assertEquals(-1.0, thirdProcessedStep.getSwingDurationShiftFraction());
      assertEquals(-1.0, thirdProcessedStep.getSwingSplitFraction());

      assertEquals(0.5 + expectedWeightShiftFromUnequal, processedOutput.getFootstepDataList().getFinalTransferWeightDistribution());
      assertEquals(0.5 - expectedSplitFractionFromUnequal, processedOutput.getFootstepDataList().getFinalTransferSplitFraction());
   }


   private static List<Point3D> getFullSupportFoot()
   {
      double length = 0.3;
      double width = 0.15;

      return getRectangleSupportFoot(length, width);
   }

   private static List<Point3D> getLineSupportFoot()
   {
      double length = 0.3;
      double width = 0.0;

      return getRectangleSupportFoot(length, width);

   }

   private static List<Point3D> getRectangleSupportFoot(double length, double width)
   {
      List<Point3D> points = new ArrayList<>();
      points.add(new Point3D(length / 2.0, width / 2.0, 0.0));
      points.add(new Point3D(length / 2.0, -width / 2.0, 0.0));
      points.add(new Point3D(-length / 2.0, -width / 2.0, 0.0));
      points.add(new Point3D(-length / 2.0, width / 2.0, 0.0));

      return points;
   }

   private ICPPlannerParameters getPlannerParameters()
   {
      ICPPlannerParameters parameters = new SmoothCMPPlannerParameters();
      return parameters;
   }
}
