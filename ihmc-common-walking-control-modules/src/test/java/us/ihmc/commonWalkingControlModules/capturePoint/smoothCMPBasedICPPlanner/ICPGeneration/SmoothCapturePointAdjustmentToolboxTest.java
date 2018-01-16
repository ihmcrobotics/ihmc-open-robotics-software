package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.ICPGeneration;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameTuple3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class SmoothCapturePointAdjustmentToolboxTest
{
   private static final int nTests = 10000;
   private static final double omega0 = 3.4;
   private static final double EPSILON = 10e-5;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final Random random = new Random(4910204121L);

   YoVariableRegistry registry = new YoVariableRegistry("");
   String namePrefix = "SmoothCapturePointAdjustmentToolboxTest";

   private final SmoothCapturePointToolbox icpToolbox = new SmoothCapturePointToolbox();
   private final SmoothCapturePointAdjustmentToolbox icpAdjustmentToolbox = new SmoothCapturePointAdjustmentToolbox(icpToolbox);
   private final List<FrameTuple3D<?, ?>> icpQuantityInitialConditionList = new ArrayList<>();

   @Before
   public void setupTest()
   {
      icpQuantityInitialConditionList.clear();
      icpQuantityInitialConditionList.add(new FramePoint3D());
      while(icpQuantityInitialConditionList.size() < SmoothCapturePointAdjustmentToolbox.defaultSize)
         icpQuantityInitialConditionList.add(new FrameVector3D());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAdjustICPDuringInitialTransfer3DLinear()
   {
      // Linear polynomial: y(x) = a0 + a1*x
      int numberOfCoefficients = 2;
      int numberOfSegments = 3;
      int numberOfSwingSegments = 0;

      FrameTrajectory3D linear3DSegment1 = new FrameTrajectory3D(numberOfCoefficients, worldFrame);
      FrameTrajectory3D linear3DSegment2 = new FrameTrajectory3D(numberOfCoefficients, worldFrame);
      FrameTrajectory3D linear3DSegment3 = new FrameTrajectory3D(numberOfCoefficients, worldFrame);

      List<FrameTrajectory3D> cmpPolynomials3D = new ArrayList<FrameTrajectory3D>();
      List<FramePoint3D> entryCornerPoints = new ArrayList<FramePoint3D>();
      List<FramePoint3D> exitCornerPoints = new ArrayList<FramePoint3D>();

      // Boundary Conditions
      List<FrameTuple3D<?, ?>> icp0QuantitiesBefore = new ArrayList<>();
      List<FrameTuple3D<?, ?>> cmp0QuantitiesBefore = new ArrayList<>();
      List<FrameTuple3D<?, ?>> cmp2QuantitiesBefore = new ArrayList<>();

      for (int i = 0; i < nTests; i++)
      {
         cmpPolynomials3D.clear();
         entryCornerPoints.clear();
         exitCornerPoints.clear();
         icp0QuantitiesBefore.clear();
         cmp0QuantitiesBefore.clear();
         cmp2QuantitiesBefore.clear();

         double t0 = 0.0;
         FramePoint3D cmp0 = new FramePoint3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));

         //double scale1 = 1.0 / random.nextDouble();
         double t1 = t0 + 0.1 * (random.nextDouble() + 0.1);
         FrameVector3D dcmp1 = new FrameVector3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));
         FramePoint3D cmp1 = new FramePoint3D(worldFrame);
         cmp1.add(cmp0, dcmp1);

         //double scale2 = 1.0 / random.nextDouble();
         double t2 = t1 + 0.1 * (random.nextDouble() + 0.1);
         FrameVector3D dcmp2 = new FrameVector3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));
         FramePoint3D cmp2 = new FramePoint3D(worldFrame);
         cmp2.add(cmp1, dcmp2);

         //double scale3 = 1.0 / random.nextDouble();
         double t3 = t2 + 0.1 * (random.nextDouble() + 0.1);
         FrameVector3D dcmp3 = new FrameVector3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));
         FramePoint3D cmp3 = new FramePoint3D(worldFrame);
         cmp3.add(cmp2, dcmp3);

         linear3DSegment1.setLinear(t0, t1, cmp0, cmp1);
         linear3DSegment2.setLinear(t1, t2, cmp1, cmp2);
         linear3DSegment3.setLinear(t2, t3, cmp2, cmp3);

         cmpPolynomials3D.add(linear3DSegment1);
         cmpPolynomials3D.add(linear3DSegment2);
         cmpPolynomials3D.add(linear3DSegment3);

         for (int j = 0; j < numberOfSegments; j++)
         {
            entryCornerPoints.add(new FramePoint3D());
            exitCornerPoints.add(new FramePoint3D());
         }

         icpToolbox.computeDesiredCornerPoints3D(entryCornerPoints, exitCornerPoints, cmpPolynomials3D, omega0);

         for (int j = 0; j < numberOfCoefficients / 2; j++)
         {
            FramePoint3D icp0QuantityBC = new FramePoint3D();
            linear3DSegment1.getDerivative(j, linear3DSegment1.getInitialTime(), icp0QuantityBC);
            icp0QuantitiesBefore.add(icp0QuantityBC);

            FramePoint3D cmp0QuantityBC = new FramePoint3D();
            linear3DSegment1.getDerivative(j, linear3DSegment1.getInitialTime(), cmp0QuantityBC);
            cmp0QuantitiesBefore.add(cmp0QuantityBC);

            FramePoint3D cmp2QuantityBC = new FramePoint3D();
            linear3DSegment2.getDerivative(j, linear3DSegment2.getFinalTime(), cmp2QuantityBC);
            cmp2QuantitiesBefore.add(cmp2QuantityBC);
         }

         setICPInitialConditionsForAdjustment(cmpPolynomials3D.get(0).getInitialTime(), exitCornerPoints, cmpPolynomials3D,
                                                                   numberOfSwingSegments, omega0);
         icpAdjustmentToolbox.adjustDesiredTrajectoriesForInitialSmoothing3D(omega0, cmpPolynomials3D, icpQuantityInitialConditionList, entryCornerPoints, exitCornerPoints);

         for (int j = 0; j < numberOfCoefficients / 2; j++)
         {
            FramePoint3D icp0QuantityAfter = new FramePoint3D();
            linear3DSegment1.getDerivative(j, linear3DSegment1.getInitialTime(), icp0QuantityAfter);
            EuclidCoreTestTools.assertTuple3DEquals("", icp0QuantitiesBefore.get(j), icp0QuantityAfter, EPSILON);
            //
            FramePoint3D cmp0QuantityAfter = new FramePoint3D();
            linear3DSegment1.getDerivative(j, linear3DSegment1.getInitialTime(), cmp0QuantityAfter);
            EuclidCoreTestTools.assertTuple3DEquals("", cmp0QuantitiesBefore.get(j), cmp0QuantityAfter, EPSILON);

            FramePoint3D cmp2QuantityAfter = new FramePoint3D();
            linear3DSegment2.getDerivative(j, linear3DSegment2.getFinalTime(), cmp2QuantityAfter);
            EuclidCoreTestTools.assertTuple3DEquals("", cmp2QuantitiesBefore.get(j), cmp2QuantityAfter, EPSILON);

            FramePoint3D cmp1QuantitySegment1 = new FramePoint3D();
            linear3DSegment1.getDerivative(j, linear3DSegment1.getFinalTime(), cmp1QuantitySegment1);
            FramePoint3D cmp1QuantitySegment2 = new FramePoint3D();
            linear3DSegment2.getDerivative(j, linear3DSegment2.getInitialTime(), cmp1QuantitySegment2);
            EuclidCoreTestTools.assertTuple3DEquals("", cmp1QuantitySegment1, cmp1QuantitySegment2, EPSILON);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAdjustICPDuringRegularTransfer3DLinear()
   {
      // Linear polynomial: y(x) = a0 + a1*x
      int numberOfCoefficients = 2;
      int numberOfSegments = 4;
      int numberOfSwingSegments = 1;

      FrameTrajectory3D linear3DSegment1 = new FrameTrajectory3D(numberOfCoefficients, worldFrame);
      FrameTrajectory3D linear3DSegment2 = new FrameTrajectory3D(numberOfCoefficients, worldFrame);
      FrameTrajectory3D linear3DSegment3 = new FrameTrajectory3D(numberOfCoefficients, worldFrame);
      FrameTrajectory3D linear3DSegment4 = new FrameTrajectory3D(numberOfCoefficients, worldFrame);

      List<FrameTrajectory3D> cmpPolynomials3DSwing = new ArrayList<FrameTrajectory3D>();
      List<FrameTrajectory3D> cmpPolynomials3DTransfer = new ArrayList<FrameTrajectory3D>();
      List<FramePoint3D> entryCornerPointsSwing = new ArrayList<FramePoint3D>();
      List<FramePoint3D> exitCornerPointsSwing = new ArrayList<FramePoint3D>();
      List<FramePoint3D> entryCornerPointsTransfer = new ArrayList<FramePoint3D>();
      List<FramePoint3D> exitCornerPointsTransfer = new ArrayList<FramePoint3D>();

      // Boundary Conditions
      List<FrameTuple3D<?, ?>> icp1QuantitiesBefore = new ArrayList<>();
      List<FrameTuple3D<?, ?>> cmp1QuantitiesBefore = new ArrayList<>();
      List<FrameTuple3D<?, ?>> cmp3QuantitiesBefore = new ArrayList<>();

      for (int i = 0; i < nTests; i++)
      {
         cmpPolynomials3DSwing.clear();
         cmpPolynomials3DTransfer.clear();
         entryCornerPointsSwing.clear();
         exitCornerPointsSwing.clear();
         icp1QuantitiesBefore.clear();
         cmp1QuantitiesBefore.clear();
         cmp3QuantitiesBefore.clear();

         double t0 = 0.0;
         FramePoint3D cmp0 = new FramePoint3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));

         //double scale1 = 1.0 / random.nextDouble();
         double t1 = t0 + 0.1 * (random.nextDouble() + 0.1);
         FrameVector3D dcmp1 = new FrameVector3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));
         FramePoint3D cmp1 = new FramePoint3D(worldFrame);
         cmp1.add(cmp0, dcmp1);

         //double scale2 = 1.0 / random.nextDouble();
         double t2 = t1 + 0.1 * (random.nextDouble() + 0.1);
         FrameVector3D dcmp2 = new FrameVector3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));
         FramePoint3D cmp2 = new FramePoint3D(worldFrame);
         cmp2.add(cmp1, dcmp2);

         //double scale3 = 1.0 / random.nextDouble();
         double t3 = t2 + 0.1 * (random.nextDouble() + 0.1);
         FrameVector3D dcmp3 = new FrameVector3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));
         FramePoint3D cmp3 = new FramePoint3D(worldFrame);
         cmp3.add(cmp2, dcmp3);

         //double scale4 = 1.0 / random.nextDouble();
         double t4 = t3 + 0.1 * (random.nextDouble() + 0.1);
         FrameVector3D dcmp4 = new FrameVector3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));
         FramePoint3D cmp4 = new FramePoint3D(worldFrame);
         cmp4.add(cmp3, dcmp4);

         // Segment 1 = Swing; Segment 2, 3, 4 = Transfer
         linear3DSegment1.setLinear(t0, t1, cmp0, cmp1);
         linear3DSegment2.setLinear(t1, t2, cmp1, cmp2);
         linear3DSegment3.setLinear(t2, t3, cmp2, cmp3);
         linear3DSegment4.setLinear(t3, t4, cmp3, cmp4);

         cmpPolynomials3DSwing.add(linear3DSegment1);
         cmpPolynomials3DSwing.add(linear3DSegment2);
         cmpPolynomials3DSwing.add(linear3DSegment3);

         cmpPolynomials3DTransfer.add(linear3DSegment2);
         cmpPolynomials3DTransfer.add(linear3DSegment3);
         cmpPolynomials3DTransfer.add(linear3DSegment4);

         for (int j = 0; j < numberOfSegments - 1; j++)
         {
            entryCornerPointsSwing.add(new FramePoint3D());
            exitCornerPointsSwing.add(new FramePoint3D());

            entryCornerPointsTransfer.add(new FramePoint3D());
            exitCornerPointsTransfer.add(new FramePoint3D());
         }

         icpToolbox.computeDesiredCornerPoints3D(entryCornerPointsSwing, exitCornerPointsSwing, cmpPolynomials3DSwing, omega0);

         for (int j = 0; j < numberOfCoefficients / 2; j++)
         {
            FramePoint3D icp1QuantityBC = new FramePoint3D();
            icpToolbox.calculateICPQuantityFromCorrespondingCMPPolynomial3D(omega0, linear3DSegment1.getFinalTime(), j, linear3DSegment1,
                                                                            exitCornerPointsSwing.get(0), icp1QuantityBC); // TODO: Tranfer?!

            icp1QuantitiesBefore.add(icp1QuantityBC);

            FramePoint3D cmp1QuantityBC = new FramePoint3D();
            linear3DSegment2.getDerivative(j, linear3DSegment2.getInitialTime(), cmp1QuantityBC);
            cmp1QuantitiesBefore.add(cmp1QuantityBC);

            FramePoint3D cmp3QuantityBC = new FramePoint3D();
            linear3DSegment3.getDerivative(j, linear3DSegment3.getFinalTime(), cmp3QuantityBC);
            cmp3QuantitiesBefore.add(cmp3QuantityBC);
         }

         setICPInitialConditionsForAdjustment(cmpPolynomials3DSwing.get(0).getFinalTime(), exitCornerPointsSwing, cmpPolynomials3DSwing, numberOfSwingSegments,
                                              omega0);

         icpToolbox.computeDesiredCornerPoints3D(entryCornerPointsTransfer, exitCornerPointsTransfer, cmpPolynomials3DTransfer, omega0);
         icpAdjustmentToolbox.adjustDesiredTrajectoriesForInitialSmoothing3D(omega0, cmpPolynomials3DTransfer, icpQuantityInitialConditionList, entryCornerPointsTransfer, exitCornerPointsTransfer);

         for (int j = 0; j < numberOfCoefficients / 2; j++)
         {
            FramePoint3D icp1QuantityAfter = new FramePoint3D();
            icpToolbox.calculateICPQuantityFromCorrespondingCMPPolynomial3D(omega0, linear3DSegment2.getInitialTime(), j, linear3DSegment2,
                                                                            exitCornerPointsTransfer.get(0), icp1QuantityAfter);
            EuclidCoreTestTools.assertTuple3DEquals("", icp1QuantitiesBefore.get(j), icp1QuantityAfter, EPSILON);

            FramePoint3D cmp1QuantityAfter = new FramePoint3D();
            linear3DSegment2.getDerivative(j, linear3DSegment2.getInitialTime(), cmp1QuantityAfter);
            EuclidCoreTestTools.assertTuple3DEquals("", cmp1QuantitiesBefore.get(j), cmp1QuantityAfter, EPSILON);

            FramePoint3D cmp3QuantityAfter = new FramePoint3D();
            linear3DSegment3.getDerivative(j, linear3DSegment3.getFinalTime(), cmp3QuantityAfter);
            EuclidCoreTestTools.assertTuple3DEquals("", cmp3QuantitiesBefore.get(j), cmp3QuantityAfter, EPSILON);

            FramePoint3D cmp2QuantitySegment2 = new FramePoint3D();
            linear3DSegment2.getDerivative(j, linear3DSegment2.getFinalTime(), cmp2QuantitySegment2);
            FramePoint3D cmp2QuantitySegment3 = new FramePoint3D();
            linear3DSegment3.getDerivative(j, linear3DSegment3.getInitialTime(), cmp2QuantitySegment3);
            EuclidCoreTestTools.assertTuple3DEquals("", cmp2QuantitySegment2, cmp2QuantitySegment3, EPSILON);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAdjustICPDuringRegularTransferRecomputed3DLinear()
   {
      // Linear polynomial: y(x) = a0 + a1*x
      int numberOfCoefficients = 2;
      int numberOfSegments = 4;
      int numberOfSwingSegments = 1;

      FrameTrajectory3D linear3DSegment1 = new FrameTrajectory3D(numberOfCoefficients, worldFrame);
      FrameTrajectory3D linear3DSegment2 = new FrameTrajectory3D(numberOfCoefficients, worldFrame);
      FrameTrajectory3D linear3DSegment3 = new FrameTrajectory3D(numberOfCoefficients, worldFrame);
      FrameTrajectory3D linear3DSegment4 = new FrameTrajectory3D(numberOfCoefficients, worldFrame);

      FrameTrajectory3D linear3DSegment2Updated = new FrameTrajectory3D(numberOfCoefficients, worldFrame);
      FrameTrajectory3D linear3DSegment3Updated = new FrameTrajectory3D(numberOfCoefficients, worldFrame);
      FrameTrajectory3D linear3DSegment4Updated = new FrameTrajectory3D(numberOfCoefficients, worldFrame);

      List<FrameTrajectory3D> cmpPolynomials3DSwing = new ArrayList<FrameTrajectory3D>();
      List<FrameTrajectory3D> cmpPolynomials3DTransferUpdated = new ArrayList<FrameTrajectory3D>();

      List<FramePoint3D> entryCornerPointsSwing = new ArrayList<FramePoint3D>();
      List<FramePoint3D> exitCornerPointsSwing = new ArrayList<FramePoint3D>();

      List<FramePoint3D> entryCornerPointsTransferUpdated = new ArrayList<FramePoint3D>();
      List<FramePoint3D> exitCornerPointsTransferUpdated = new ArrayList<FramePoint3D>();

      // The way this test is implemented now, (tFinal - endTimeOffset) needs to be within the final swing segment
      // Otherwise (numberOfSwingSegments - 1) is not correct segment for setICPInitialConditions
      double endTimeOffset = 0.03;

      // Boundary Conditions
      List<FrameTuple3D<?, ?>> icp1QuantitiesBefore = new ArrayList<>();
      List<FrameTuple3D<?, ?>> cmp1QuantitiesBefore = new ArrayList<>();
      List<FrameTuple3D<?, ?>> cmp3QuantitiesBefore = new ArrayList<>();

      for (int i = 0; i < nTests; i++)
      {
         cmpPolynomials3DSwing.clear();
         cmpPolynomials3DTransferUpdated.clear();
         entryCornerPointsSwing.clear();
         exitCornerPointsSwing.clear();
         icp1QuantitiesBefore.clear();
         cmp1QuantitiesBefore.clear();
         cmp3QuantitiesBefore.clear();

         double t0 = 0.0;
         FramePoint3D cmp0 = new FramePoint3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));

         //double scale1 = 1.0 / random.nextDouble();
         double t1 = t0 + 0.1 * (random.nextDouble() + 0.1);
         FrameVector3D dcmp1 = new FrameVector3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));
         FramePoint3D cmp1 = new FramePoint3D(worldFrame);
         cmp1.add(cmp0, dcmp1);

         //double scale2 = 1.0 / random.nextDouble();
         double t2 = t1 + 0.1 * (random.nextDouble() + 0.1);
         FrameVector3D dcmp2 = new FrameVector3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));
         FramePoint3D cmp2 = new FramePoint3D(worldFrame);
         cmp2.add(cmp1, dcmp2);

         //double scale3 = 1.0 / random.nextDouble();
         double t3 = t2 + 0.1 * (random.nextDouble() + 0.1);
         FrameVector3D dcmp3 = new FrameVector3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));
         FramePoint3D cmp3 = new FramePoint3D(worldFrame);
         cmp3.add(cmp2, dcmp3);

         //double scale4 = 1.0 / random.nextDouble();
         double t4 = t3 + 0.1 * (random.nextDouble() + 0.1);
         FrameVector3D dcmp4 = new FrameVector3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));
         FramePoint3D cmp4 = new FramePoint3D(worldFrame);
         cmp4.add(cmp3, dcmp4);

         FrameVector3D random1 = new FrameVector3D(worldFrame, new Point3D(random.nextDouble() / 20.0, random.nextDouble() / 20.0, 0.0));
         FramePoint3D cmp1Updated = new FramePoint3D(worldFrame);
         cmp1Updated.add(cmp1, random1);

         FrameVector3D random2 = new FrameVector3D(worldFrame, new Point3D(random.nextDouble() / 20.0, random.nextDouble() / 20.0, 0.0));
         FramePoint3D cmp2Updated = new FramePoint3D(worldFrame);
         cmp2Updated.add(cmp2, random2);

         FrameVector3D random3 = new FrameVector3D(worldFrame, new Point3D(random.nextDouble() / 20.0, random.nextDouble() / 20.0, 0.0));
         FramePoint3D cmp3Updated = new FramePoint3D(worldFrame);
         cmp3Updated.add(cmp3, random3);

         FrameVector3D random4 = new FrameVector3D(worldFrame, new Point3D(random.nextDouble() / 20.0, random.nextDouble() / 20.0, 0.0));
         FramePoint3D cmp4Updated = new FramePoint3D(worldFrame);
         cmp4Updated.add(cmp4, random4);

         // Segment 1 = Swing; Segment 2, 3, 4 = Transfer
         linear3DSegment1.setLinear(t0, t1, cmp0, cmp1);
         linear3DSegment2.setLinear(t1, t2, cmp1, cmp2);
         linear3DSegment3.setLinear(t2, t3, cmp2, cmp3);
         linear3DSegment4.setLinear(t3, t4, cmp3, cmp4);

         linear3DSegment2Updated.setLinear(t1, t2, cmp1Updated, cmp2Updated);
         linear3DSegment3Updated.setLinear(t2, t3, cmp2Updated, cmp3Updated);
         linear3DSegment4Updated.setLinear(t3, t4, cmp3Updated, cmp4Updated);

         cmpPolynomials3DSwing.add(linear3DSegment1);
         cmpPolynomials3DSwing.add(linear3DSegment2);
         cmpPolynomials3DSwing.add(linear3DSegment3);

         cmpPolynomials3DTransferUpdated.add(linear3DSegment2Updated);
         cmpPolynomials3DTransferUpdated.add(linear3DSegment3Updated);
         cmpPolynomials3DTransferUpdated.add(linear3DSegment4Updated);

         for (int j = 0; j < numberOfSegments - 1; j++)
         {
            entryCornerPointsSwing.add(new FramePoint3D());
            exitCornerPointsSwing.add(new FramePoint3D());

            entryCornerPointsTransferUpdated.add(new FramePoint3D());
            exitCornerPointsTransferUpdated.add(new FramePoint3D());
         }

         icpToolbox.computeDesiredCornerPoints3D(entryCornerPointsSwing, exitCornerPointsSwing, cmpPolynomials3DSwing, omega0);

         for (int j = 0; j < numberOfCoefficients / 2; j++)
         {
            FramePoint3D icp1QuantityBC = new FramePoint3D();
            icpToolbox.calculateICPQuantityFromCorrespondingCMPPolynomial3D(omega0, linear3DSegment1.getFinalTime() - endTimeOffset, j, linear3DSegment1,
                                                                            exitCornerPointsSwing.get(0), icp1QuantityBC); // TODO: Transfer?!

            icp1QuantitiesBefore.add(icp1QuantityBC);

            FramePoint3D cmp1QuantityBC = new FramePoint3D();
            linear3DSegment2Updated.getDerivative(j, linear3DSegment2Updated.getInitialTime(), cmp1QuantityBC);
            cmp1QuantitiesBefore.add(cmp1QuantityBC);

            FramePoint3D cmp3QuantityBC = new FramePoint3D();
            linear3DSegment3Updated.getDerivative(j, linear3DSegment3Updated.getFinalTime(), cmp3QuantityBC);
            cmp3QuantitiesBefore.add(cmp3QuantityBC);
         }

         setICPInitialConditionsForAdjustment(cmpPolynomials3DSwing.get(0).getFinalTime() - endTimeOffset, exitCornerPointsSwing,
                                                                   cmpPolynomials3DSwing, numberOfSwingSegments - 1, omega0);

         icpToolbox.computeDesiredCornerPoints3D(entryCornerPointsTransferUpdated, exitCornerPointsTransferUpdated, cmpPolynomials3DTransferUpdated, omega0);
         icpAdjustmentToolbox.adjustDesiredTrajectoriesForInitialSmoothing3D(omega0, cmpPolynomials3DTransferUpdated, icpQuantityInitialConditionList, entryCornerPointsTransferUpdated, exitCornerPointsTransferUpdated);

         for (int j = 0; j < numberOfCoefficients / 2; j++)
         {
            FramePoint3D icp1QuantityAfter = new FramePoint3D();
            icpToolbox.calculateICPQuantityFromCorrespondingCMPPolynomial3D(omega0, linear3DSegment2Updated.getInitialTime(), j, linear3DSegment2Updated,
                                                                            exitCornerPointsTransferUpdated.get(0), icp1QuantityAfter);
            EuclidCoreTestTools.assertTuple3DEquals("", icp1QuantitiesBefore.get(j), icp1QuantityAfter, EPSILON);

            FramePoint3D cmp1QuantityAfter = new FramePoint3D();
            linear3DSegment2Updated.getDerivative(j, linear3DSegment2Updated.getInitialTime(), cmp1QuantityAfter);
            EuclidCoreTestTools.assertTuple3DEquals("", cmp1QuantitiesBefore.get(j), cmp1QuantityAfter, EPSILON);

            FramePoint3D cmp3QuantityAfter = new FramePoint3D();
            linear3DSegment3Updated.getDerivative(j, linear3DSegment3Updated.getFinalTime(), cmp3QuantityAfter);
            EuclidCoreTestTools.assertTuple3DEquals("", cmp3QuantitiesBefore.get(j), cmp3QuantityAfter, EPSILON);

            FramePoint3D cmp2QuantitySegment2 = new FramePoint3D();
            linear3DSegment2Updated.getDerivative(j, linear3DSegment2Updated.getFinalTime(), cmp2QuantitySegment2);
            FramePoint3D cmp2QuantitySegment3 = new FramePoint3D();
            linear3DSegment3Updated.getDerivative(j, linear3DSegment3Updated.getInitialTime(), cmp2QuantitySegment3);
            EuclidCoreTestTools.assertTuple3DEquals("", cmp2QuantitySegment2, cmp2QuantitySegment3, EPSILON);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAdjustICPDuringInitialTransfer3DCubic()
   {
      // Linear polynomial: y(x) = a0 + a1*x + a2*x + a3*x
      int numberOfCoefficients = 4;
      int numberOfSegments = 3;
      int numberOfSwingSegments = 0;

      FrameTrajectory3D cubic3DSegment1 = new FrameTrajectory3D(numberOfCoefficients, worldFrame);
      FrameTrajectory3D cubic3DSegment2 = new FrameTrajectory3D(numberOfCoefficients, worldFrame);
      FrameTrajectory3D cubic3DSegment3 = new FrameTrajectory3D(numberOfCoefficients, worldFrame);

      List<FrameTrajectory3D> cmpPolynomials3D = new ArrayList<FrameTrajectory3D>();
      List<FramePoint3D> entryCornerPoints = new ArrayList<FramePoint3D>();
      List<FramePoint3D> exitCornerPoints = new ArrayList<FramePoint3D>();

      // Boundary Conditions
      List<FrameTuple3D<?, ?>> icp0QuantitiesBefore = new ArrayList<>();
      List<FrameTuple3D<?, ?>> cmp0QuantitiesBefore = new ArrayList<>();
      List<FrameTuple3D<?, ?>> cmp2QuantitiesBefore = new ArrayList<>();

      for (int i = 0; i < nTests; i++)
      {
         cmpPolynomials3D.clear();
         entryCornerPoints.clear();
         exitCornerPoints.clear();
         icp0QuantitiesBefore.clear();
         cmp0QuantitiesBefore.clear();
         cmp2QuantitiesBefore.clear();

         double t0 = 0.0;
         FramePoint3D cmp0 = new FramePoint3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));

         //double scale1 = 1.0 / random.nextDouble();
         double t1 = t0 + 1.0 * (random.nextDouble() + 0.1);
         FrameVector3D dcmp1 = new FrameVector3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));
         FramePoint3D cmp1 = new FramePoint3D(worldFrame);
         cmp1.add(cmp0, dcmp1);

         //double scale2 = 1.0 / random.nextDouble();
         double t2 = t1 + 1.0 * (random.nextDouble() + 0.1);
         FrameVector3D dcmp2 = new FrameVector3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));
         FramePoint3D cmp2 = new FramePoint3D(worldFrame);
         cmp2.add(cmp1, dcmp2);

         //double scale3 = 1.0 / random.nextDouble();
         double t3 = t2 + 1.0 * (random.nextDouble() + 0.1);
         FrameVector3D dcmp3 = new FrameVector3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));
         FramePoint3D cmp3 = new FramePoint3D(worldFrame);
         cmp3.add(cmp2, dcmp3);

         cubic3DSegment1.setCubic(t0, t1, cmp0, cmp1);
         cubic3DSegment2.setCubic(t1, t2, cmp1, cmp2);
         cubic3DSegment3.setCubic(t2, t3, cmp2, cmp3);

         cmpPolynomials3D.add(cubic3DSegment1);
         cmpPolynomials3D.add(cubic3DSegment2);
         cmpPolynomials3D.add(cubic3DSegment3);

         for (int j = 0; j < numberOfSegments; j++)
         {
            entryCornerPoints.add(new FramePoint3D());
            exitCornerPoints.add(new FramePoint3D());
         }

         icpToolbox.computeDesiredCornerPoints3D(entryCornerPoints, exitCornerPoints, cmpPolynomials3D, omega0);

         for (int j = 0; j < numberOfCoefficients / 2; j++)
         {
            FramePoint3D icp0QuantityBC = new FramePoint3D();
            cubic3DSegment1.getDerivative(j, cubic3DSegment1.getInitialTime(), icp0QuantityBC);
            icp0QuantitiesBefore.add(icp0QuantityBC);

            FramePoint3D cmp0QuantityBC = new FramePoint3D();
            cubic3DSegment1.getDerivative(j, cubic3DSegment1.getInitialTime(), cmp0QuantityBC);
            cmp0QuantitiesBefore.add(cmp0QuantityBC);

            FramePoint3D cmp2QuantityBC = new FramePoint3D();
            cubic3DSegment2.getDerivative(j, cubic3DSegment2.getFinalTime(), cmp2QuantityBC);
            cmp2QuantitiesBefore.add(cmp2QuantityBC);
         }

         setICPInitialConditionsForAdjustment(cmpPolynomials3D.get(0).getInitialTime(), exitCornerPoints, cmpPolynomials3D,
                                                                   numberOfSwingSegments, omega0);
         icpAdjustmentToolbox.adjustDesiredTrajectoriesForInitialSmoothing3D(omega0, cmpPolynomials3D, icpQuantityInitialConditionList, entryCornerPoints, exitCornerPoints);

         for (int j = 0; j < numberOfCoefficients / 2; j++)
         {
            FramePoint3D icp0QuantityAfter = new FramePoint3D();
            cubic3DSegment1.getDerivative(j, cubic3DSegment1.getInitialTime(), icp0QuantityAfter);
            EuclidCoreTestTools.assertTuple3DEquals("", icp0QuantitiesBefore.get(j), icp0QuantityAfter, EPSILON);

            FramePoint3D cmp0QuantityAfter = new FramePoint3D();
            cubic3DSegment1.getDerivative(j, cubic3DSegment1.getInitialTime(), cmp0QuantityAfter);
            EuclidCoreTestTools.assertTuple3DEquals("", cmp0QuantitiesBefore.get(j), cmp0QuantityAfter, EPSILON);

            FramePoint3D cmp2QuantityAfter = new FramePoint3D();
            cubic3DSegment2.getDerivative(j, cubic3DSegment2.getFinalTime(), cmp2QuantityAfter);
            EuclidCoreTestTools.assertTuple3DEquals("" + j, cmp2QuantitiesBefore.get(j), cmp2QuantityAfter, EPSILON);

            FramePoint3D cmp1QuantitySegment1 = new FramePoint3D();
            cubic3DSegment1.getDerivative(j, cubic3DSegment1.getFinalTime(), cmp1QuantitySegment1);
            FramePoint3D cmp1QuantitySegment2 = new FramePoint3D();
            cubic3DSegment2.getDerivative(j, cubic3DSegment2.getInitialTime(), cmp1QuantitySegment2);
            EuclidCoreTestTools.assertTuple3DEquals("", cmp1QuantitySegment1, cmp1QuantitySegment2, EPSILON);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAdjustICPDuringRegularTransfer3DCubic()
   {
      // Linear polynomial: y(x) = a0 + a1*x + a2*x + a3*x
      int numberOfCoefficients = 4;
      int numberOfSegments = 4;
      int numberOfSwingSegments = 1;

      FrameTrajectory3D cubic3DSegment1 = new FrameTrajectory3D(numberOfCoefficients, worldFrame);
      FrameTrajectory3D cubic3DSegment2 = new FrameTrajectory3D(numberOfCoefficients, worldFrame);
      FrameTrajectory3D cubic3DSegment3 = new FrameTrajectory3D(numberOfCoefficients, worldFrame);
      FrameTrajectory3D cubic3DSegment4 = new FrameTrajectory3D(numberOfCoefficients, worldFrame);

      List<FrameTrajectory3D> cmpPolynomials3DSwing = new ArrayList<FrameTrajectory3D>();
      List<FrameTrajectory3D> cmpPolynomials3DTransfer = new ArrayList<FrameTrajectory3D>();
      List<FramePoint3D> entryCornerPointsSwing = new ArrayList<FramePoint3D>();
      List<FramePoint3D> exitCornerPointsSwing = new ArrayList<FramePoint3D>();
      List<FramePoint3D> entryCornerPointsTransfer = new ArrayList<FramePoint3D>();
      List<FramePoint3D> exitCornerPointsTransfer = new ArrayList<FramePoint3D>();

      // Boundary Conditions
      List<FrameTuple3D<?, ?>> icp1QuantitiesBefore = new ArrayList<>();
      List<FrameTuple3D<?, ?>> cmp1QuantitiesBefore = new ArrayList<>();
      List<FrameTuple3D<?, ?>> cmp3QuantitiesBefore = new ArrayList<>();

      for (int i = 0; i < nTests; i++)
      {
         cmpPolynomials3DSwing.clear();
         cmpPolynomials3DTransfer.clear();
         entryCornerPointsSwing.clear();
         exitCornerPointsSwing.clear();
         icp1QuantitiesBefore.clear();
         cmp1QuantitiesBefore.clear();
         cmp3QuantitiesBefore.clear();

         double t0 = 0.0;
         FramePoint3D cmp0 = new FramePoint3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));

         //double scale1 = 1.0 / random.nextDouble();
         double t1 = t0 + 1.0 * (random.nextDouble() + 0.1);
         FrameVector3D dcmp1 = new FrameVector3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));
         FramePoint3D cmp1 = new FramePoint3D(worldFrame);
         cmp1.add(cmp0, dcmp1);

         //double scale2 = 1.0 / random.nextDouble();
         double t2 = t1 + 1.0 * (random.nextDouble() + 0.1);
         FrameVector3D dcmp2 = new FrameVector3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));
         FramePoint3D cmp2 = new FramePoint3D(worldFrame);
         cmp2.add(cmp1, dcmp2);

         //double scale3 = 1.0 / random.nextDouble();
         double t3 = t2 + 1.0 * (random.nextDouble() + 0.1);
         FrameVector3D dcmp3 = new FrameVector3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));
         FramePoint3D cmp3 = new FramePoint3D(worldFrame);
         cmp3.add(cmp2, dcmp3);

         //double scale4 = 1.0 / random.nextDouble();
         double t4 = t3 + 1.0 * (random.nextDouble() + 0.1);
         FrameVector3D dcmp4 = new FrameVector3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));
         FramePoint3D cmp4 = new FramePoint3D(worldFrame);
         cmp4.add(cmp3, dcmp4);

         // Segment 1 = Swing; Segment 2, 3, 4 = Transfer
         cubic3DSegment1.setCubic(t0, t1, cmp0, cmp1);
         cubic3DSegment2.setCubic(t1, t2, cmp1, cmp2);
         cubic3DSegment3.setCubic(t2, t3, cmp2, cmp3);
         cubic3DSegment4.setCubic(t3, t4, cmp3, cmp4);

         cmpPolynomials3DSwing.add(cubic3DSegment1);
         cmpPolynomials3DSwing.add(cubic3DSegment2);
         cmpPolynomials3DSwing.add(cubic3DSegment3);

         cmpPolynomials3DTransfer.add(cubic3DSegment2);
         cmpPolynomials3DTransfer.add(cubic3DSegment3);
         cmpPolynomials3DTransfer.add(cubic3DSegment4);

         for (int j = 0; j < numberOfSegments - 1; j++)
         {
            entryCornerPointsSwing.add(new FramePoint3D());
            exitCornerPointsSwing.add(new FramePoint3D());

            entryCornerPointsTransfer.add(new FramePoint3D());
            exitCornerPointsTransfer.add(new FramePoint3D());
         }

         icpToolbox.computeDesiredCornerPoints3D(entryCornerPointsSwing, exitCornerPointsSwing, cmpPolynomials3DSwing, omega0);

         for (int j = 0; j < numberOfCoefficients / 2; j++)
         {
            FramePoint3D icp1QuantityBC = new FramePoint3D();
            icpToolbox.calculateICPQuantityFromCorrespondingCMPPolynomial3D(omega0, cubic3DSegment1.getFinalTime(), j, cubic3DSegment1,
                                                                            exitCornerPointsSwing.get(0), icp1QuantityBC);
            icp1QuantitiesBefore.add(icp1QuantityBC);

            FramePoint3D cmp1QuantityBC = new FramePoint3D();
            cubic3DSegment2.getDerivative(j, cubic3DSegment2.getInitialTime(), cmp1QuantityBC);
            cmp1QuantitiesBefore.add(cmp1QuantityBC);

            FramePoint3D cmp3QuantityBC = new FramePoint3D();
            cubic3DSegment3.getDerivative(j, cubic3DSegment3.getFinalTime(), cmp3QuantityBC);
            cmp3QuantitiesBefore.add(cmp3QuantityBC);
         }

         setICPInitialConditionsForAdjustment(cmpPolynomials3DSwing.get(0).getFinalTime(), exitCornerPointsSwing, cmpPolynomials3DSwing,
                                                                   numberOfSwingSegments, omega0);

         icpToolbox.computeDesiredCornerPoints3D(entryCornerPointsTransfer, exitCornerPointsTransfer, cmpPolynomials3DTransfer, omega0);
         icpAdjustmentToolbox.adjustDesiredTrajectoriesForInitialSmoothing3D(omega0, cmpPolynomials3DTransfer, icpQuantityInitialConditionList, entryCornerPointsTransfer, exitCornerPointsTransfer);

         for (int j = 0; j < numberOfCoefficients / 2; j++)
         {
            FramePoint3D icp1QuantityAfter = new FramePoint3D();
            icpToolbox.calculateICPQuantityFromCorrespondingCMPPolynomial3D(omega0, cubic3DSegment2.getInitialTime(), j, cubic3DSegment2,
                                                                            exitCornerPointsTransfer.get(0), icp1QuantityAfter);
            EuclidCoreTestTools.assertTuple3DEquals("" + j, icp1QuantitiesBefore.get(j), icp1QuantityAfter, EPSILON);

            FramePoint3D cmp1QuantityAfter = new FramePoint3D();
            cubic3DSegment2.getDerivative(j, cubic3DSegment2.getInitialTime(), cmp1QuantityAfter);
            EuclidCoreTestTools.assertTuple3DEquals("", cmp1QuantitiesBefore.get(j), cmp1QuantityAfter, EPSILON);

            FramePoint3D cmp3QuantityAfter = new FramePoint3D();
            cubic3DSegment3.getDerivative(j, cubic3DSegment3.getFinalTime(), cmp3QuantityAfter);
            EuclidCoreTestTools.assertTuple3DEquals("", cmp3QuantitiesBefore.get(j), cmp3QuantityAfter, EPSILON);

            FramePoint3D cmp2QuantitySegment2 = new FramePoint3D();
            cubic3DSegment2.getDerivative(j, cubic3DSegment2.getFinalTime(), cmp2QuantitySegment2);
            FramePoint3D cmp2QuantitySegment3 = new FramePoint3D();
            cubic3DSegment3.getDerivative(j, cubic3DSegment3.getInitialTime(), cmp2QuantitySegment3);
            EuclidCoreTestTools.assertTuple3DEquals("", cmp2QuantitySegment2, cmp2QuantitySegment3, EPSILON);
         }
      }
   }

   public void setICPInitialConditionsForAdjustment(double localTime, List<FramePoint3D> exitCornerPointsFromCoPs, List<FrameTrajectory3D> copPolynomials3D,
                                                    int currentSwingSegment, double omega0)
   {
      if (currentSwingSegment < 0)
      {
         FrameTrajectory3D copPolynomial3D = copPolynomials3D.get(0);
         for (int i = 0; i < copPolynomials3D.get(0).getNumberOfCoefficients() / 2; i++)
         {
            FrameTuple3D<?, ?> icpQuantityInitialCondition = icpQuantityInitialConditionList.get(i);

            copPolynomial3D.getDerivative(i, localTime, icpQuantityInitialCondition);
         }
      }
      else
      {
         FrameTrajectory3D copPolynomial3D = copPolynomials3D.get(currentSwingSegment);
         for (int i = 0; i < copPolynomials3D.get(0).getNumberOfCoefficients() / 2; i++)
         {
            FrameTuple3D<?, ?> icpQuantityInitialCondition = icpQuantityInitialConditionList.get(i);

            icpToolbox.calculateICPQuantityFromCorrespondingCMPPolynomial3D(omega0, localTime, i, copPolynomial3D,
                                                                            exitCornerPointsFromCoPs.get(currentSwingSegment), icpQuantityInitialCondition);
         }
      }
   }
}
