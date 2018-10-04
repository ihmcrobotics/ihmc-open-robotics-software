package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoMGeneration;

import org.junit.After;
import org.junit.Assert;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPPlannerSegmentedTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPPlannerTrajectoryGenerator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class CoMIntegrationToolsTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double integrationDT = 0.0001;
   private static final double epsilon = 0.005;

   @After
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testTransferIntegration()
   {
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      YoDouble omega0 = new YoDouble("Omega0", registry);
      omega0.set(3.0);

      double duration = 2.0;
      YoFramePoint3D initialICP = new YoFramePoint3D("initialICP", worldFrame, registry);
      YoFrameVector3D initialICPVelocity = new YoFrameVector3D("initialICPVelocity", worldFrame, registry);
      YoFramePoint3D finalICP = new YoFramePoint3D("finalICP", worldFrame, registry);
      YoFrameVector3D finalICPVelocity = new YoFrameVector3D("finalICPVelocity", worldFrame, registry);
      YoFramePoint3D initialCoM = new YoFramePoint3D("initialCoM", worldFrame, registry);

      YoFramePoint3D desiredCapturePoint = new YoFramePoint3D("desiredCapturePoint", worldFrame, registry);
      YoFramePoint3D desiredCoMPosition = new YoFramePoint3D("desiredCoMPosition", worldFrame, registry);
      YoFrameVector3D desiredCoMVelocity = new YoFrameVector3D("desiredCoMVelocity", worldFrame, registry);
      YoFramePoint3D integratedCoMPosition = new YoFramePoint3D("integratedCoMPosition", worldFrame, registry);

      YoFramePoint3D cmp = new YoFramePoint3D("cmp", worldFrame, registry);
      cmp.set(0.1, 0.03, 0.0);
      YoFramePoint3D cornerPoint = new YoFramePoint3D("cornerPoint", worldFrame, registry);
      cornerPoint.set(0.08, 0.06, 0.0);

      CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), duration / 2.0, cornerPoint, cmp, finalICP);
      CapturePointTools.computeDesiredCapturePointVelocity(omega0.getDoubleValue(), duration / 2.0, cornerPoint, cmp, finalICPVelocity);

      ICPPlannerTrajectoryGenerator icpTrajectoryGenerator = new ICPPlannerTrajectoryGenerator("DoubleSupport", worldFrame, omega0, registry);

      icpTrajectoryGenerator.setTrajectoryTime(duration);
      icpTrajectoryGenerator.setInitialConditions(initialICP, initialICPVelocity, worldFrame);
      icpTrajectoryGenerator.setFinalConditions(finalICP, finalICPVelocity, worldFrame);
      icpTrajectoryGenerator.setInitialCoMPosition(initialCoM, worldFrame);
      icpTrajectoryGenerator.initialize();

      FramePoint3D finalCoM = new FramePoint3D();
      icpTrajectoryGenerator.computeFinalCoMPosition(finalCoM);

      FramePoint3D dummyPoint = new FramePoint3D();

      double time = 0.0;
      while (time < duration)
      {
         icpTrajectoryGenerator.compute(time);
         icpTrajectoryGenerator.get(desiredCapturePoint);
         icpTrajectoryGenerator.getCoMPosition(desiredCoMPosition);

         desiredCoMVelocity.set(desiredCapturePoint);
         desiredCoMVelocity.sub(integratedCoMPosition);
         desiredCoMVelocity.scale(omega0.getDoubleValue());

         dummyPoint.set(desiredCoMVelocity);
         dummyPoint.scale(integrationDT);
         integratedCoMPosition.add(dummyPoint);

         time += integrationDT;
      }

      // check that the reference trajectory is equal to that that we get from integration
      Assert.assertEquals(integratedCoMPosition.getX(), desiredCoMPosition.getX(), epsilon);
      Assert.assertEquals(integratedCoMPosition.getY(), desiredCoMPosition.getY(), epsilon);

      // check that the predicted final is equal to that that we get from integration
      Assert.assertEquals(integratedCoMPosition.getX(), finalCoM.getX(), epsilon);
      Assert.assertEquals(integratedCoMPosition.getY(), finalCoM.getY(), epsilon);

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testSwingIntegration()
   {
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      YoDouble omega0 = new YoDouble("Omega0", registry);
      omega0.set(3.0);

      double swingDuration = 2.0;
      double transferDuration = 1.0;

      double timeSpentOnExitCMP = 0.5 * (swingDuration + transferDuration);
      double timeSpentOnEntryCMP = 0.5 * (swingDuration + transferDuration);

      double transferInitialDuration = 0.5 * transferDuration;
      double transferFinalDuration = 0.5 * transferDuration;
      double swingInitialDuration = timeSpentOnEntryCMP - transferFinalDuration;
      double swingFinalDuration = timeSpentOnExitCMP - transferInitialDuration;

      YoFramePoint3D initialICP = new YoFramePoint3D("initialICP", worldFrame, registry);
      YoFramePoint3D finalICP = new YoFramePoint3D("finalICP", worldFrame, registry);
      YoFramePoint3D initialCoM = new YoFramePoint3D("initialCoM", worldFrame, registry);
      finalICP.set(-0.07, 0.4, 0.0);

      YoFramePoint3D desiredCapturePoint = new YoFramePoint3D("desiredCapturePoint", worldFrame, registry);
      YoFramePoint3D desiredCoMPosition = new YoFramePoint3D("desiredCoMPosition", worldFrame, registry);
      YoFrameVector3D desiredCoMVelocity = new YoFrameVector3D("desiredCoMVelocity", worldFrame, registry);
      YoFramePoint3D integratedCoMPosition = new YoFramePoint3D("integratedCoMPosition", worldFrame, registry);

      YoFramePoint3D entryCMP = new YoFramePoint3D("entryCMP", worldFrame, registry);
      YoFramePoint3D exitCMP = new YoFramePoint3D("exitCMP", worldFrame, registry);
      entryCMP.set(0.1, 0.0, 0.0);
      exitCMP.set(0.08, 0.08, 0.0);


      YoFramePoint3D entryCornerPoint = new YoFramePoint3D("entryCornerPoint", worldFrame, registry);
      YoFramePoint3D exitCornerPoint = new YoFramePoint3D("exitCornerPoint", worldFrame, registry);

      CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), -swingFinalDuration, finalICP, exitCMP, exitCornerPoint);
      CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), -timeSpentOnEntryCMP, exitCornerPoint, entryCMP, entryCornerPoint);
      CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), transferInitialDuration, entryCornerPoint, entryCMP, initialICP);

      ICPPlannerSegmentedTrajectoryGenerator icpTrajectoryGenerator = new ICPPlannerSegmentedTrajectoryGenerator("DoubleSupport", worldFrame, omega0, registry);

      icpTrajectoryGenerator.setBoundaryICP(initialICP, finalICP);
      icpTrajectoryGenerator.setCornerPoints(entryCornerPoint, exitCornerPoint);
      icpTrajectoryGenerator.setReferenceCMPs(entryCMP, exitCMP);
      icpTrajectoryGenerator.setReferenceFrames(worldFrame, worldFrame);
      icpTrajectoryGenerator.setInitialCoMPosition(initialCoM, worldFrame);
      icpTrajectoryGenerator.setTrajectoryTime(swingInitialDuration, swingFinalDuration);
      icpTrajectoryGenerator.initialize();

      FramePoint3D finalCoM = new FramePoint3D();
      icpTrajectoryGenerator.computeFinalCoMPosition(finalCoM);

      FramePoint3D dummyPoint = new FramePoint3D();

      double time = 0.0;
      while (time < swingDuration)
      {
         icpTrajectoryGenerator.compute(time);
         icpTrajectoryGenerator.getPosition(desiredCapturePoint);
         icpTrajectoryGenerator.getCoMPosition(desiredCoMPosition);

         desiredCoMVelocity.set(desiredCapturePoint);
         desiredCoMVelocity.sub(integratedCoMPosition);
         desiredCoMVelocity.scale(omega0.getDoubleValue());

         dummyPoint.set(desiredCoMVelocity);
         dummyPoint.scale(integrationDT);
         integratedCoMPosition.add(dummyPoint);

         time += integrationDT;
      }

      // check that the reference trajectory is equal to that that we get from integration
      Assert.assertEquals(integratedCoMPosition.getX(), desiredCoMPosition.getX(), epsilon);
      Assert.assertEquals(integratedCoMPosition.getY(), desiredCoMPosition.getY(), epsilon);

      // check that the predicted final is equal to that that we get from integration
      Assert.assertEquals(integratedCoMPosition.getX(), finalCoM.getX(), epsilon);
      Assert.assertEquals(integratedCoMPosition.getY(), finalCoM.getY(), epsilon);

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSwingIntegrationWithSpline()
   {
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      YoDouble omega0 = new YoDouble("Omega0", registry);
      omega0.set(3.0);

      double swingDuration = 0.6;
      double transferDuration = 1.0;

      double timeSpentOnExitCMP = 0.5 * (swingDuration + transferDuration);
      double timeSpentOnEntryCMP = 0.5 * (swingDuration + transferDuration);

      double transferInitialDuration = 0.5 * transferDuration;
      double transferFinalDuration = 0.5 * transferDuration;
      double swingInitialDuration = timeSpentOnEntryCMP - transferFinalDuration;
      double swingFinalDuration = timeSpentOnExitCMP - transferInitialDuration;

      YoFramePoint3D initialICP = new YoFramePoint3D("initialICP", worldFrame, registry);
      YoFramePoint3D finalICP = new YoFramePoint3D("finalICP", worldFrame, registry);
      YoFramePoint3D initialCoM = new YoFramePoint3D("initialCoM", worldFrame, registry);
      finalICP.set(-0.07, 0.4, 0.0);

      YoFramePoint3D desiredCapturePoint = new YoFramePoint3D("desiredCapturePoint", worldFrame, registry);
      YoFramePoint3D desiredCoMPosition = new YoFramePoint3D("desiredCoMPosition", worldFrame, registry);
      YoFrameVector3D desiredCoMVelocity = new YoFrameVector3D("desiredCoMVelocity", worldFrame, registry);
      YoFramePoint3D integratedCoMPosition = new YoFramePoint3D("integratedCoMPosition", worldFrame, registry);

      YoFramePoint3D entryCMP = new YoFramePoint3D("entryCMP", worldFrame, registry);
      YoFramePoint3D exitCMP = new YoFramePoint3D("exitCMP", worldFrame, registry);
      entryCMP.set(0.1, 0.0, 0.0);
      exitCMP.set(0.08, 0.08, 0.0);


      YoFramePoint3D entryCornerPoint = new YoFramePoint3D("entryCornerPoint", worldFrame, registry);
      YoFramePoint3D exitCornerPoint = new YoFramePoint3D("exitCornerPoint", worldFrame, registry);

      CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), -swingFinalDuration, finalICP, exitCMP, exitCornerPoint);
      CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), -timeSpentOnEntryCMP, exitCornerPoint, entryCMP, entryCornerPoint);
      CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), transferInitialDuration, entryCornerPoint, entryCMP, initialICP);

      ICPPlannerSegmentedTrajectoryGenerator icpTrajectoryGenerator = new ICPPlannerSegmentedTrajectoryGenerator("DoubleSupport", worldFrame, omega0, registry);
      icpTrajectoryGenerator.setMaximumSplineDuration(0.5);
      icpTrajectoryGenerator.setMinimumTimeToSpendOnFinalCMP(0.0);

      icpTrajectoryGenerator.setBoundaryICP(initialICP, finalICP);
      icpTrajectoryGenerator.setCornerPoints(entryCornerPoint, exitCornerPoint);
      icpTrajectoryGenerator.setReferenceCMPs(entryCMP, exitCMP);
      icpTrajectoryGenerator.setReferenceFrames(worldFrame, worldFrame);
      icpTrajectoryGenerator.setInitialCoMPosition(initialCoM, worldFrame);
      icpTrajectoryGenerator.setTrajectoryTime(swingInitialDuration, swingFinalDuration);
      icpTrajectoryGenerator.initialize();

      FramePoint3D finalCoM = new FramePoint3D();
      icpTrajectoryGenerator.computeFinalCoMPosition(finalCoM);

      FramePoint3D dummyPoint = new FramePoint3D();

      double time = 0.0;
      while (time < swingDuration)
      {
         icpTrajectoryGenerator.compute(time);
         icpTrajectoryGenerator.getPosition(desiredCapturePoint);
         icpTrajectoryGenerator.getCoMPosition(desiredCoMPosition);

         desiredCoMVelocity.set(desiredCapturePoint);
         desiredCoMVelocity.sub(integratedCoMPosition);
         desiredCoMVelocity.scale(omega0.getDoubleValue());

         dummyPoint.set(desiredCoMVelocity);
         dummyPoint.scale(integrationDT);
         integratedCoMPosition.add(dummyPoint);

         time += integrationDT;
      }

      // check that the reference trajectory is equal to that that we get from integration
      Assert.assertEquals(integratedCoMPosition.getX(), desiredCoMPosition.getX(), epsilon);
      Assert.assertEquals(integratedCoMPosition.getY(), desiredCoMPosition.getY(), epsilon);

      // check that the predicted final is equal to that that we get from integration
      Assert.assertEquals(integratedCoMPosition.getX(), finalCoM.getX(), epsilon);
      Assert.assertEquals(integratedCoMPosition.getY(), finalCoM.getY(), epsilon);

   }
}
