package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.Test;

import us.ihmc.commons.PrintTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameTuple3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.math.trajectories.YoFrameTrajectory3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SmoothCapturePointAdjustmentToolboxTest
{
   private static final int nTests = 20;
   private static final double omega0 = 3.4;
   private static final double EPSILON = 10e-6;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private Random random = new Random();
   
   YoVariableRegistry registry = new YoVariableRegistry("");
   String namePrefix = "SmoothCapturePointAdjustmentToolboxTest";
   
   private final SmoothCapturePointToolbox icpToolbox = new SmoothCapturePointToolbox();
   private final SmoothCapturePointAdjustmentToolbox icpAdjustmentToolbox = new SmoothCapturePointAdjustmentToolbox(icpToolbox);
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAdjustICPDuringInitialTransfer3DLinear()
   {
      // Linear polynomial: y(x) = a0 + a1*x
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix);
      
      boolean isInitialTransfer = true;
      int numberOfCoefficients = 2;
      int numberOfSegments = 3;
      int numberOfSwingSegments = 0;
      
      YoFrameTrajectory3D linear3DSegment1 = new YoFrameTrajectory3D(namePrefix + "LinearAdjustedSegment1", numberOfCoefficients, worldFrame, registry);
      YoFrameTrajectory3D linear3DSegment2 = new YoFrameTrajectory3D(namePrefix + "LinearAdjustedSegment2", numberOfCoefficients, worldFrame, registry);
      YoFrameTrajectory3D linear3DSegment3 = new YoFrameTrajectory3D(namePrefix + "LinearAdjustedSegment3", numberOfCoefficients, worldFrame, registry);

      List<YoFrameTrajectory3D> cmpPolynomials3D = new ArrayList<YoFrameTrajectory3D>();
      List<FramePoint3D> entryCornerPoints = new ArrayList<FramePoint3D>();
      List<FramePoint3D> exitCornerPoints = new ArrayList<FramePoint3D>();
      
      // Boundary Conditions
      List<FrameTuple3D<?, ?>> icp0QuantitiesBefore = new ArrayList<FrameTuple3D<?, ?>>();
      List<FrameTuple3D<?, ?>> cmp0QuantitiesBefore = new ArrayList<FrameTuple3D<?, ?>>();
      List<FrameTuple3D<?, ?>> cmp2QuantitiesBefore = new ArrayList<FrameTuple3D<?, ?>>();
      
      for(int i = 0; i < nTests; i++)
      {
         cmpPolynomials3D.clear();
         entryCornerPoints.clear();
         exitCornerPoints.clear();
         icp0QuantitiesBefore.clear();
         cmp0QuantitiesBefore.clear();
         cmp2QuantitiesBefore.clear();
                  
         double t0 = 0.0;
         FramePoint3D cmp0 = new FramePoint3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));
         
         double scale1 = 1.0 / Math.random();
         double t1 = t0 + scale1 * Math.random();
         FrameVector3D dcmp1 = new FrameVector3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));
         FramePoint3D cmp1 = new FramePoint3D(worldFrame);
         cmp1.add(cmp0, dcmp1);
         
         double scale2 = 1.0 / Math.random();
         double t2 = t1 + scale2 * Math.random();
         FrameVector3D dcmp2 = new FrameVector3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));
         FramePoint3D cmp2 = new FramePoint3D(worldFrame);
         cmp2.add(cmp1, dcmp2);
         
         double scale3 = 1.0 / Math.random();
         double t3 = t2 + scale3 * Math.random();
         FrameVector3D dcmp3 = new FrameVector3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));
         FramePoint3D cmp3 = new FramePoint3D(worldFrame);
         cmp3.add(cmp2, dcmp3);
         
         linear3DSegment1.setLinear(t0, t1, cmp0, cmp1);
         linear3DSegment2.setLinear(t1, t2, cmp1, cmp2);
         linear3DSegment3.setLinear(t2, t3, cmp2, cmp3);
         
         cmpPolynomials3D.add(linear3DSegment1);
         cmpPolynomials3D.add(linear3DSegment2);
         cmpPolynomials3D.add(linear3DSegment3);
                  
         for(int j = 0; j < numberOfSegments; j++)
         {
            entryCornerPoints.add(new FramePoint3D());
            exitCornerPoints.add(new FramePoint3D());
         }
         
         icpToolbox.computeDesiredCornerPoints(entryCornerPoints, exitCornerPoints, cmpPolynomials3D, omega0);

         for(int j = 0; j < numberOfCoefficients / 2; j++)
         {
            FrameTuple3D<?, ?> icp0QuantityBC = new FramePoint3D();
            linear3DSegment1.getDerivative(j, linear3DSegment1.getInitialTime(), icp0QuantityBC);
            icp0QuantitiesBefore.add(icp0QuantityBC); 
            
            FrameTuple3D<?, ?> cmp0QuantityBC = new FramePoint3D();
            linear3DSegment1.getDerivative(j, linear3DSegment1.getInitialTime(), cmp0QuantityBC);
            cmp0QuantitiesBefore.add(cmp0QuantityBC);
            
            FrameTuple3D<?, ?> cmp2QuantityBC = new FramePoint3D();
            linear3DSegment2.getDerivative(j, linear3DSegment2.getFinalTime(), cmp2QuantityBC);
            cmp2QuantitiesBefore.add(cmp2QuantityBC);
         }

         icpAdjustmentToolbox.setICPInitialConditions(exitCornerPoints, cmpPolynomials3D, numberOfSwingSegments, isInitialTransfer, omega0);
         icpAdjustmentToolbox.adjustDesiredTrajectoriesForInitialSmoothing(entryCornerPoints, exitCornerPoints, cmpPolynomials3D, omega0);

         for(int j = 0; j < numberOfCoefficients / 2; j++)
         {
            FrameTuple3D<?, ?> icp0QuantityAfter = new FramePoint3D();
            linear3DSegment1.getDerivative(j, linear3DSegment1.getInitialTime(), icp0QuantityAfter);
            EuclidCoreTestTools.assertTuple3DEquals("", icp0QuantitiesBefore.get(j), icp0QuantityAfter, EPSILON);
//            
            FrameTuple3D<?, ?> cmp0QuantityAfter = new FramePoint3D();
            linear3DSegment1.getDerivative(j, linear3DSegment1.getInitialTime(), cmp0QuantityAfter);
            EuclidCoreTestTools.assertTuple3DEquals("", cmp0QuantitiesBefore.get(j), cmp0QuantityAfter, EPSILON);
            
            FrameTuple3D<?, ?> cmp2QuantityAfter = new FramePoint3D();
            linear3DSegment2.getDerivative(j, linear3DSegment2.getFinalTime(), cmp2QuantityAfter);
            EuclidCoreTestTools.assertTuple3DEquals("", cmp2QuantitiesBefore.get(j), cmp2QuantityAfter, EPSILON);
            
            FrameTuple3D<?, ?> cmp1QuantitySegment1 = new FramePoint3D();
            linear3DSegment1.getDerivative(j, linear3DSegment1.getFinalTime(), cmp1QuantitySegment1);
            FrameTuple3D<?, ?> cmp1QuantitySegment2 = new FramePoint3D();
            linear3DSegment2.getDerivative(j, linear3DSegment2.getInitialTime(), cmp1QuantitySegment2);
            EuclidCoreTestTools.assertTuple3DEquals("", cmp1QuantitySegment1, cmp1QuantitySegment2, EPSILON);
         }
      }
   }
   
//   @ContinuousIntegrationTest(estimatedDuration = 0.0)
//   @Test(timeout = 30000)
   public void testAdjustICPDuringRegularTransfer3DLinear()
   {
      // Linear polynomial: y(x) = a0 + a1*x
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix);
      
      boolean isInitialTransfer = false;
      int numberOfCoefficients = 2;
      int numberOfSegments = 4;
      int numberOfSwingSegments = 1;
      
      YoFrameTrajectory3D linear3DSegment1 = new YoFrameTrajectory3D(namePrefix + "LinearAdjustedSegment1", numberOfCoefficients, worldFrame, registry);
      YoFrameTrajectory3D linear3DSegment2 = new YoFrameTrajectory3D(namePrefix + "LinearAdjustedSegment2", numberOfCoefficients, worldFrame, registry);
      YoFrameTrajectory3D linear3DSegment3 = new YoFrameTrajectory3D(namePrefix + "LinearAdjustedSegment3", numberOfCoefficients, worldFrame, registry);
      YoFrameTrajectory3D linear3DSegment4 = new YoFrameTrajectory3D(namePrefix + "LinearAdjustedSegment4", numberOfCoefficients, worldFrame, registry);

      List<YoFrameTrajectory3D> cmpPolynomials3D = new ArrayList<YoFrameTrajectory3D>();
      List<FramePoint3D> entryCornerPoints = new ArrayList<FramePoint3D>();
      List<FramePoint3D> exitCornerPoints = new ArrayList<FramePoint3D>();
      
      // Boundary Conditions
      List<FrameTuple3D<?, ?>> icp1QuantitiesBefore = new ArrayList<FrameTuple3D<?, ?>>();
      List<FrameTuple3D<?, ?>> cmp1QuantitiesBefore = new ArrayList<FrameTuple3D<?, ?>>();
      List<FrameTuple3D<?, ?>> cmp3QuantitiesBefore = new ArrayList<FrameTuple3D<?, ?>>();
      
      for(int i = 0; i < nTests; i++)
      {
         cmpPolynomials3D.clear();
         entryCornerPoints.clear();
         exitCornerPoints.clear();
         icp1QuantitiesBefore.clear();
         cmp1QuantitiesBefore.clear();
         cmp3QuantitiesBefore.clear();
                  
         double t0 = 0.0;
         FramePoint3D cmp0 = new FramePoint3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));
         
         double scale1 = 1.0 / Math.random();
         double t1 = t0 + scale1 * Math.random();
         FrameVector3D dcmp1 = new FrameVector3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));
         FramePoint3D cmp1 = new FramePoint3D(worldFrame);
         cmp1.add(cmp0, dcmp1);
         
         double scale2 = 1.0 / Math.random();
         double t2 = t1 + scale2 * Math.random();
         FrameVector3D dcmp2 = new FrameVector3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));
         FramePoint3D cmp2 = new FramePoint3D(worldFrame);
         cmp2.add(cmp1, dcmp2);
         
         double scale3 = 1.0 / Math.random();
         double t3 = t2 + scale3 * Math.random();
         FrameVector3D dcmp3 = new FrameVector3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));
         FramePoint3D cmp3 = new FramePoint3D(worldFrame);
         cmp3.add(cmp2, dcmp3);
         
         double scale4 = 1.0 / Math.random();
         double t4 = t3 + scale4 * Math.random();
         FrameVector3D dcmp4 = new FrameVector3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));
         FramePoint3D cmp4 = new FramePoint3D(worldFrame);
         cmp4.add(cmp3, dcmp4);
         
         // Segment 1 = Swing; Segment 2, 3, 4 = Transfer
         linear3DSegment1.setLinear(t0, t1, cmp0, cmp1);
         linear3DSegment2.setLinear(t1, t2, cmp1, cmp2);
         linear3DSegment3.setLinear(t2, t3, cmp2, cmp3);
         linear3DSegment4.setLinear(t3, t4, cmp3, cmp4);
         
         cmpPolynomials3D.add(linear3DSegment1);
         cmpPolynomials3D.add(linear3DSegment2);
         cmpPolynomials3D.add(linear3DSegment3);
         cmpPolynomials3D.add(linear3DSegment4);
                  
         for(int j = 0; j < numberOfSegments; j++)
         {
            entryCornerPoints.add(new FramePoint3D());
            exitCornerPoints.add(new FramePoint3D());
         }
         
         icpToolbox.computeDesiredCornerPoints(entryCornerPoints, exitCornerPoints, cmpPolynomials3D, omega0);

         for(int j = 0; j < numberOfCoefficients / 2; j++)
         {
            FrameTuple3D<?, ?> icp1QuantityBC = new FramePoint3D();
            icpToolbox.calculateICPQuantityFromCorrespondingCMPPolynomial3D(omega0, linear3DSegment1.getFinalTime(), j, linear3DSegment1, exitCornerPoints.get(0), icp1QuantityBC);
            icp1QuantitiesBefore.add(icp1QuantityBC);               
            
            FrameTuple3D<?, ?> cmp1QuantityBC = new FramePoint3D();
            linear3DSegment2.getDerivative(j, linear3DSegment2.getInitialTime(), cmp1QuantityBC);
            cmp1QuantitiesBefore.add(cmp1QuantityBC);
            
            FrameTuple3D<?, ?> cmp3QuantityBC = new FramePoint3D();
            linear3DSegment3.getDerivative(j, linear3DSegment3.getFinalTime(), cmp3QuantityBC);
            cmp3QuantitiesBefore.add(cmp3QuantityBC);
         }
         
         PrintTools.debug("icp1QuantitiesBefore = " + icp1QuantitiesBefore.toString());
         PrintTools.debug("cmp1QuantitiesBefore = " + cmp1QuantitiesBefore.toString());
         PrintTools.debug("cmp3QuantitiesBefore = " + cmp3QuantitiesBefore.toString());
         
         icpAdjustmentToolbox.setICPInitialConditions(exitCornerPoints, cmpPolynomials3D, numberOfSwingSegments, isInitialTransfer, omega0);
         icpAdjustmentToolbox.adjustDesiredTrajectoriesForInitialSmoothing(entryCornerPoints, exitCornerPoints, cmpPolynomials3D, omega0);

         for(int j = 0; j < numberOfCoefficients / 2; j++)
         {
            FrameTuple3D<?, ?> icp1QuantityAfter = new FramePoint3D();
            icpToolbox.calculateICPQuantityFromCorrespondingCMPPolynomial3D(omega0, linear3DSegment2.getInitialTime(), j, linear3DSegment2, exitCornerPoints.get(1), icp1QuantityAfter);
            EuclidCoreTestTools.assertTuple3DEquals("", icp1QuantitiesBefore.get(j), icp1QuantityAfter, EPSILON);
            
            FrameTuple3D<?, ?> cmp1QuantityAfter = new FramePoint3D();
            linear3DSegment2.getDerivative(j, linear3DSegment2.getInitialTime(), cmp1QuantityAfter);
            EuclidCoreTestTools.assertTuple3DEquals("", cmp1QuantitiesBefore.get(j), cmp1QuantityAfter, EPSILON);
            
            FrameTuple3D<?, ?> cmp3QuantityAfter = new FramePoint3D();
            linear3DSegment3.getDerivative(j, linear3DSegment3.getFinalTime(), cmp3QuantityAfter);
            EuclidCoreTestTools.assertTuple3DEquals("", cmp3QuantitiesBefore.get(j), cmp3QuantityAfter, EPSILON);
            
            FrameTuple3D<?, ?> cmp2QuantitySegment2 = new FramePoint3D();
            linear3DSegment2.getDerivative(j, linear3DSegment2.getFinalTime(), cmp2QuantitySegment2);
            FrameTuple3D<?, ?> cmp2QuantitySegment3 = new FramePoint3D();
            linear3DSegment3.getDerivative(j, linear3DSegment3.getInitialTime(), cmp2QuantitySegment3);
            EuclidCoreTestTools.assertTuple3DEquals("", cmp2QuantitySegment2, cmp2QuantitySegment3, EPSILON);
         }
         
//         EuclidCoreTestTools.assertTuple3DEquals("", icpVelocityDesiredCurrent, icpVelocityDesiredCurrentDynamics, EPSILON);
      }
   }
}
