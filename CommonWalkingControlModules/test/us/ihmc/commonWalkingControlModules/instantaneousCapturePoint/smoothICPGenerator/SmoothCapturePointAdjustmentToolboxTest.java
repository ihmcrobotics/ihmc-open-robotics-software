package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.Test;

import us.ihmc.commons.PrintTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameTuple;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.YoFrameTrajectory3D;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
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
      List<FramePoint> entryCornerPoints = new ArrayList<FramePoint>();
      List<FramePoint> exitCornerPoints = new ArrayList<FramePoint>();
      
      // Boundary Conditions
      List<FrameTuple<?, ?>> icp0QuantitiesBefore = new ArrayList<FrameTuple<?, ?>>();
      List<FrameTuple<?, ?>> cmp0QuantitiesBefore = new ArrayList<FrameTuple<?, ?>>();
      List<FrameTuple<?, ?>> cmp2QuantitiesBefore = new ArrayList<FrameTuple<?, ?>>();
      
      for(int i = 0; i < nTests; i++)
      {
         cmpPolynomials3D.clear();
         entryCornerPoints.clear();
         exitCornerPoints.clear();
         icp0QuantitiesBefore.clear();
         cmp0QuantitiesBefore.clear();
         cmp2QuantitiesBefore.clear();
                  
         double t0 = 0.0;
         FramePoint cmp0 = new FramePoint(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));
         
         double scale1 = 1.0 / Math.random();
         double t1 = t0 + scale1 * Math.random();
         FrameVector dcmp1 = new FrameVector(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));
         FramePoint cmp1 = new FramePoint(worldFrame);
         cmp1.add(cmp0, dcmp1);
         
         double scale2 = 1.0 / Math.random();
         double t2 = t1 + scale2 * Math.random();
         FrameVector dcmp2 = new FrameVector(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));
         FramePoint cmp2 = new FramePoint(worldFrame);
         cmp2.add(cmp1, dcmp2);
         
         double scale3 = 1.0 / Math.random();
         double t3 = t2 + scale3 * Math.random();
         FrameVector dcmp3 = new FrameVector(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));
         FramePoint cmp3 = new FramePoint(worldFrame);
         cmp3.add(cmp2, dcmp3);
         
         linear3DSegment1.setLinear(t0, t1, cmp0, cmp1);
         linear3DSegment2.setLinear(t1, t2, cmp1, cmp2);
         linear3DSegment3.setLinear(t2, t3, cmp2, cmp3);
         
         cmpPolynomials3D.add(linear3DSegment1);
         cmpPolynomials3D.add(linear3DSegment2);
         cmpPolynomials3D.add(linear3DSegment3);
                  
         for(int j = 0; j < numberOfSegments; j++)
         {
            entryCornerPoints.add(new FramePoint());
            exitCornerPoints.add(new FramePoint());
         }
         
         icpToolbox.computeDesiredCornerPoints(entryCornerPoints, exitCornerPoints, cmpPolynomials3D, omega0);

         for(int j = 0; j < numberOfCoefficients / 2; j++)
         {
            FrameTuple<?, ?> icp0QuantityBC = new FramePoint();
            linear3DSegment1.getDerivative(j, linear3DSegment1.getInitialTime(), icp0QuantityBC);
            icp0QuantitiesBefore.add(icp0QuantityBC); 
            
            FrameTuple<?, ?> cmp0QuantityBC = new FramePoint();
            linear3DSegment1.getDerivative(j, linear3DSegment1.getInitialTime(), cmp0QuantityBC);
            cmp0QuantitiesBefore.add(cmp0QuantityBC);
            
            FrameTuple<?, ?> cmp2QuantityBC = new FramePoint();
            linear3DSegment2.getDerivative(j, linear3DSegment2.getFinalTime(), cmp2QuantityBC);
            cmp2QuantitiesBefore.add(cmp2QuantityBC);
         }

         icpAdjustmentToolbox.setICPInitialConditions(exitCornerPoints, cmpPolynomials3D, numberOfSwingSegments, isInitialTransfer, omega0);
         icpAdjustmentToolbox.adjustDesiredTrajectoriesForInitialSmoothing(entryCornerPoints, exitCornerPoints, cmpPolynomials3D, omega0);

         for(int j = 0; j < numberOfCoefficients / 2; j++)
         {
            FrameTuple<?, ?> icp0QuantityAfter = new FramePoint();
            linear3DSegment1.getDerivative(j, linear3DSegment1.getInitialTime(), icp0QuantityAfter);
            EuclidCoreTestTools.assertTuple3DEquals("", icp0QuantitiesBefore.get(j).getVectorCopy(), icp0QuantityAfter.getVectorCopy(), EPSILON);
//            
            FrameTuple<?, ?> cmp0QuantityAfter = new FramePoint();
            linear3DSegment1.getDerivative(j, linear3DSegment1.getInitialTime(), cmp0QuantityAfter);
            EuclidCoreTestTools.assertTuple3DEquals("", cmp0QuantitiesBefore.get(j).getVectorCopy(), cmp0QuantityAfter.getVectorCopy(), EPSILON);
            
            FrameTuple<?, ?> cmp2QuantityAfter = new FramePoint();
            linear3DSegment2.getDerivative(j, linear3DSegment2.getFinalTime(), cmp2QuantityAfter);
            EuclidCoreTestTools.assertTuple3DEquals("", cmp2QuantitiesBefore.get(j).getVectorCopy(), cmp2QuantityAfter.getVectorCopy(), EPSILON);
            
            FrameTuple<?, ?> cmp1QuantitySegment1 = new FramePoint();
            linear3DSegment1.getDerivative(j, linear3DSegment1.getFinalTime(), cmp1QuantitySegment1);
            FrameTuple<?, ?> cmp1QuantitySegment2 = new FramePoint();
            linear3DSegment2.getDerivative(j, linear3DSegment2.getInitialTime(), cmp1QuantitySegment2);
            EuclidCoreTestTools.assertTuple3DEquals("", cmp1QuantitySegment1.getVectorCopy(), cmp1QuantitySegment2.getVectorCopy(), EPSILON);
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
      List<FramePoint> entryCornerPoints = new ArrayList<FramePoint>();
      List<FramePoint> exitCornerPoints = new ArrayList<FramePoint>();
      
      // Boundary Conditions
      List<FrameTuple<?, ?>> icp1QuantitiesBefore = new ArrayList<FrameTuple<?, ?>>();
      List<FrameTuple<?, ?>> cmp1QuantitiesBefore = new ArrayList<FrameTuple<?, ?>>();
      List<FrameTuple<?, ?>> cmp3QuantitiesBefore = new ArrayList<FrameTuple<?, ?>>();
      
      for(int i = 0; i < nTests; i++)
      {
         cmpPolynomials3D.clear();
         entryCornerPoints.clear();
         exitCornerPoints.clear();
         icp1QuantitiesBefore.clear();
         cmp1QuantitiesBefore.clear();
         cmp3QuantitiesBefore.clear();
                  
         double t0 = 0.0;
         FramePoint cmp0 = new FramePoint(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));
         
         double scale1 = 1.0 / Math.random();
         double t1 = t0 + scale1 * Math.random();
         FrameVector dcmp1 = new FrameVector(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));
         FramePoint cmp1 = new FramePoint(worldFrame);
         cmp1.add(cmp0, dcmp1);
         
         double scale2 = 1.0 / Math.random();
         double t2 = t1 + scale2 * Math.random();
         FrameVector dcmp2 = new FrameVector(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));
         FramePoint cmp2 = new FramePoint(worldFrame);
         cmp2.add(cmp1, dcmp2);
         
         double scale3 = 1.0 / Math.random();
         double t3 = t2 + scale3 * Math.random();
         FrameVector dcmp3 = new FrameVector(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));
         FramePoint cmp3 = new FramePoint(worldFrame);
         cmp3.add(cmp2, dcmp3);
         
         double scale4 = 1.0 / Math.random();
         double t4 = t3 + scale4 * Math.random();
         FrameVector dcmp4 = new FrameVector(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 0.0));
         FramePoint cmp4 = new FramePoint(worldFrame);
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
            entryCornerPoints.add(new FramePoint());
            exitCornerPoints.add(new FramePoint());
         }
         
         icpToolbox.computeDesiredCornerPoints(entryCornerPoints, exitCornerPoints, cmpPolynomials3D, omega0);

         for(int j = 0; j < numberOfCoefficients / 2; j++)
         {
            FrameTuple<?, ?> icp1QuantityBC = new FramePoint();
            icpToolbox.calculateICPQuantityFromCorrespondingCMPPolynomial3D(omega0, linear3DSegment1.getFinalTime(), j, linear3DSegment1, exitCornerPoints.get(0), icp1QuantityBC);
            icp1QuantitiesBefore.add(icp1QuantityBC);               
            
            FrameTuple<?, ?> cmp1QuantityBC = new FramePoint();
            linear3DSegment2.getDerivative(j, linear3DSegment2.getInitialTime(), cmp1QuantityBC);
            cmp1QuantitiesBefore.add(cmp1QuantityBC);
            
            FrameTuple<?, ?> cmp3QuantityBC = new FramePoint();
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
            FrameTuple<?, ?> icp1QuantityAfter = new FramePoint();
            icpToolbox.calculateICPQuantityFromCorrespondingCMPPolynomial3D(omega0, linear3DSegment2.getInitialTime(), j, linear3DSegment2, exitCornerPoints.get(1), icp1QuantityAfter);
            EuclidCoreTestTools.assertTuple3DEquals("", icp1QuantitiesBefore.get(j).getVectorCopy(), icp1QuantityAfter.getVectorCopy(), EPSILON);
            
            FrameTuple<?, ?> cmp1QuantityAfter = new FramePoint();
            linear3DSegment2.getDerivative(j, linear3DSegment2.getInitialTime(), cmp1QuantityAfter);
            EuclidCoreTestTools.assertTuple3DEquals("", cmp1QuantitiesBefore.get(j).getVectorCopy(), cmp1QuantityAfter.getVectorCopy(), EPSILON);
            
            FrameTuple<?, ?> cmp3QuantityAfter = new FramePoint();
            linear3DSegment3.getDerivative(j, linear3DSegment3.getFinalTime(), cmp3QuantityAfter);
            EuclidCoreTestTools.assertTuple3DEquals("", cmp3QuantitiesBefore.get(j).getVectorCopy(), cmp3QuantityAfter.getVectorCopy(), EPSILON);
            
            FrameTuple<?, ?> cmp2QuantitySegment2 = new FramePoint();
            linear3DSegment2.getDerivative(j, linear3DSegment2.getFinalTime(), cmp2QuantitySegment2);
            FrameTuple<?, ?> cmp2QuantitySegment3 = new FramePoint();
            linear3DSegment3.getDerivative(j, linear3DSegment3.getInitialTime(), cmp2QuantitySegment3);
            EuclidCoreTestTools.assertTuple3DEquals("", cmp2QuantitySegment2.getVectorCopy(), cmp2QuantitySegment3.getVectorCopy(), EPSILON);
         }
         
//         EuclidCoreTestTools.assertTuple3DEquals("", icpVelocityDesiredCurrent.getVectorCopy(), icpVelocityDesiredCurrentDynamics.getVectorCopy(), EPSILON);
      }
   }
}
