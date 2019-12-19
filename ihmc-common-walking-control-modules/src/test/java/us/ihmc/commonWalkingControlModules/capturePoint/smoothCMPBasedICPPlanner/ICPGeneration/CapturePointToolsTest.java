package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.ICPGeneration;

import static us.ihmc.robotics.Assert.*;

import java.util.ArrayList;
import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTrajectoryTools;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.lists.FrameTupleArrayList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector2D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class CapturePointToolsTest
{
   int nTests = 20;
   Random random = new Random();
   YoVariableRegistry registry = new YoVariableRegistry("");

   private static final double EPSILON = 10e-6;

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }


   @Test
   public void testComputeDesiredCapturePointVelocity()
   {
      YoFramePoint3D initialCapturePointPosition = new YoFramePoint3D("", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoint3D computedCapturePoint1 = new YoFramePoint3D("1", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoint3D computedCapturePoint2 = new YoFramePoint3D("2", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoint3D initialCenterOfPressure = new YoFramePoint3D("3", ReferenceFrame.getWorldFrame(), registry);
      YoFrameVector3D differentiatedCapturePointPosition = new YoFrameVector3D("4", ReferenceFrame.getWorldFrame(), registry);
      YoFrameVector3D computedCapturePointVelocity = new YoFrameVector3D("5", ReferenceFrame.getWorldFrame(), registry);

      for (int i = 0; i < nTests; i++)
      {
         initialCapturePointPosition.set(random.nextDouble(), random.nextDouble(), 0);
         initialCenterOfPressure.set(initialCapturePointPosition.getX() + 0.02, initialCapturePointPosition.getY() + 0.01, 0);

         double deltaT = 0.001;
         double time = random.nextDouble() * 0.1 + 0.05;
         double omega0 = 0.5;

         CapturePointTools.computeDesiredCapturePointPosition(omega0, time, initialCapturePointPosition, initialCenterOfPressure, computedCapturePoint1);
         CapturePointTools.computeDesiredCapturePointPosition(omega0, time + deltaT, initialCapturePointPosition, initialCenterOfPressure,
                                                              computedCapturePoint2);

         differentiatedCapturePointPosition.set(computedCapturePoint2);
         differentiatedCapturePointPosition.sub(computedCapturePoint1);
         differentiatedCapturePointPosition.scale(1 / deltaT);

         CapturePointTools.computeDesiredCapturePointVelocity(omega0, time + deltaT, initialCapturePointPosition, initialCenterOfPressure,
                                                              computedCapturePointVelocity);

         EuclidCoreTestTools.assertTuple3DEquals("", computedCapturePointVelocity, differentiatedCapturePointPosition, 1e-3);
      }
   }

   @Test
   public void testComputeDesiredCapturePointAcceleration()
   {
      YoFramePoint3D initialCapturePointPosition = new YoFramePoint3D("", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoint3D initialCenterOfPressure = new YoFramePoint3D("3", ReferenceFrame.getWorldFrame(), registry);
      YoFrameVector3D computedCapturePointVelocity = new YoFrameVector3D("5", ReferenceFrame.getWorldFrame(), registry);
      YoFrameVector3D desiredCapturePointAcceleration = new YoFrameVector3D("6", ReferenceFrame.getWorldFrame(), registry);

      for (int i = 0; i < nTests; i++)
      {
         initialCapturePointPosition.set(random.nextDouble(), random.nextDouble(), 0);
         initialCenterOfPressure.set(initialCapturePointPosition.getX() + 0.02, initialCapturePointPosition.getY() + 0.01, 0);

         double time = random.nextDouble() * 0.1 + 0.05;
         double omega0 = 0.5;

         CapturePointTools.computeDesiredCapturePointVelocity(omega0, time, initialCapturePointPosition, initialCenterOfPressure, computedCapturePointVelocity);
         CapturePointTools.computeDesiredCapturePointAcceleration(omega0, time, initialCapturePointPosition, initialCenterOfPressure,
               desiredCapturePointAcceleration);
         computedCapturePointVelocity.scale(omega0);

         EuclidCoreTestTools.assertTuple3DEquals("", computedCapturePointVelocity, desiredCapturePointAcceleration, 1e-10);

         computedCapturePointVelocity.scale(1 / omega0);
         CapturePointTools.computeDesiredCapturePointAcceleration(omega0, computedCapturePointVelocity, desiredCapturePointAcceleration);
         computedCapturePointVelocity.scale(omega0);

         EuclidCoreTestTools.assertTuple3DEquals("", computedCapturePointVelocity, desiredCapturePointAcceleration, 1e-10);
      }
   }

   @Test
   public void testComputeDesiredCentroidalMomentumPivot()
   {
      YoFramePoint3D capturePointPosition = new YoFramePoint3D("", ReferenceFrame.getWorldFrame(), registry);
      YoFrameVector3D capturePointVelocity = new YoFrameVector3D("2", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoint3D centroidalMomentumPivot = new YoFramePoint3D("3", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoint3D computedCentroidalMomentumPivot = new YoFramePoint3D("7", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoint2D capturePointPosition2D = new YoFramePoint2D("4", ReferenceFrame.getWorldFrame(), registry);
      YoFrameVector2D capturePointVelocity2D = new YoFrameVector2D("5", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoint2D centroidalMomentumPivot2D = new YoFramePoint2D("6", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoint2D computedCentroidalMomentumPivot2D = new YoFramePoint2D("8", ReferenceFrame.getWorldFrame(), registry);


      for (int i = 0; i < nTests; i++)
      {
         capturePointPosition.set(EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame(), 10.0));
         capturePointVelocity.set(EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame(), 10.0));
         capturePointPosition2D.set(EuclidFrameRandomTools.nextFramePoint2D(random, ReferenceFrame.getWorldFrame(), 10.0));
         capturePointVelocity2D.set(EuclidFrameRandomTools.nextFramePoint2D(random, ReferenceFrame.getWorldFrame(), 10.0));

         double omega0 = 0.5;

         CapturePointTools.computeCentroidalMomentumPivot(capturePointPosition, capturePointVelocity, omega0, centroidalMomentumPivot);
         CapturePointTools.computeCentroidalMomentumPivot(capturePointPosition2D, capturePointVelocity2D, omega0, centroidalMomentumPivot2D);

         computedCentroidalMomentumPivot.set(capturePointVelocity);
         computedCentroidalMomentumPivot2D.set(capturePointVelocity2D);
         computedCentroidalMomentumPivot.scale(-1.0 / omega0);
         computedCentroidalMomentumPivot2D.scale(-1.0 / omega0);
         computedCentroidalMomentumPivot.add(capturePointPosition);
         computedCentroidalMomentumPivot2D.add(capturePointPosition2D);

         EuclidCoreTestTools.assertTuple3DEquals("", computedCentroidalMomentumPivot, centroidalMomentumPivot, 1e-10);
         EuclidCoreTestTools.assertTuple2DEquals("", computedCentroidalMomentumPivot2D, centroidalMomentumPivot2D, 1e-10);
      }
   }

   @Test
   public void testComputeDesiredCapturePoint()
   {
      YoFramePoint3D centerOfMassPosition = new YoFramePoint3D("", ReferenceFrame.getWorldFrame(), registry);
      YoFrameVector3D centerOfMassVelocity = new YoFrameVector3D("2", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoint3D capturePointPosition = new YoFramePoint3D("3", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoint3D computedCapturePointPosition = new YoFramePoint3D("7", ReferenceFrame.getWorldFrame(), registry);


      for (int i = 0; i < nTests; i++)
      {
         centerOfMassPosition.set(EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame(), 10.0));
         centerOfMassVelocity.set(EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame(), 10.0));

         double omega0 = 0.5;

         CapturePointTools.computeDesiredCapturePointPosition(centerOfMassPosition, centerOfMassVelocity, omega0, capturePointPosition);

         computedCapturePointPosition.set(centerOfMassVelocity);
         computedCapturePointPosition.scale(1.0 / omega0);
         computedCapturePointPosition.add(centerOfMassPosition);

         EuclidCoreTestTools.assertTuple3DEquals("", computedCapturePointPosition, capturePointPosition, 1e-10);
      }
   }

   @Test
   public void testComputeDesiredCapturePointVelocity2()
   {
      YoFrameVector3D centerOfMassVelocity = new YoFrameVector3D("", ReferenceFrame.getWorldFrame(), registry);
      YoFrameVector3D centerOfMassAcceleration = new YoFrameVector3D("2", ReferenceFrame.getWorldFrame(), registry);
      YoFrameVector3D capturePointVelocity = new YoFrameVector3D("3", ReferenceFrame.getWorldFrame(), registry);
      YoFrameVector3D computedCapturePointVelocity= new YoFrameVector3D("7", ReferenceFrame.getWorldFrame(), registry);


      for (int i = 0; i < nTests; i++)
      {
         centerOfMassAcceleration.set(EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame(), 10.0));
         centerOfMassAcceleration.set(EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame(), 10.0));

         double omega0 = 0.5;

         CapturePointTools.computeDesiredCapturePointVelocity(centerOfMassVelocity, centerOfMassAcceleration, omega0, capturePointVelocity);

         computedCapturePointVelocity.set(centerOfMassAcceleration);
         computedCapturePointVelocity.scale(1.0 / omega0);
         computedCapturePointVelocity.add(centerOfMassVelocity);

         EuclidCoreTestTools.assertTuple3DEquals("", computedCapturePointVelocity, capturePointVelocity, 1e-10);
      }
   }

}
