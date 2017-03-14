package us.ihmc.commonWalkingControlModules.dynamicReachability;

import junit.framework.Assert;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerSegmentedTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class CoMIntegrationToolsTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double integrationDT = 0.0001;
   private static final double epsilon = 0.005;

   @ContinuousIntegrationTest(estimatedDuration = 0.7)
   @Test(timeout = 30000)
   public void testTransferIntegration()
   {
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      DoubleYoVariable omega0 = new DoubleYoVariable("Omega0", registry);
      omega0.set(3.0);

      double duration = 2.0;
      YoFramePoint initialICP = new YoFramePoint("initialICP", worldFrame, registry);
      YoFrameVector initialICPVelocity = new YoFrameVector("initialICPVelocity", worldFrame, registry);
      YoFramePoint finalICP = new YoFramePoint("finalICP", worldFrame, registry);
      YoFrameVector finalICPVelocity = new YoFrameVector("finalICPVelocity", worldFrame, registry);
      YoFramePoint initialCoM = new YoFramePoint("initialCoM", worldFrame, registry);

      YoFramePoint desiredCapturePoint = new YoFramePoint("desiredCapturePoint", worldFrame, registry);
      YoFramePoint desiredCoMPosition = new YoFramePoint("desiredCoMPosition", worldFrame, registry);
      YoFrameVector desiredCoMVelocity = new YoFrameVector("desiredCoMVelocity", worldFrame, registry);
      YoFramePoint integratedCoMPosition = new YoFramePoint("integratedCoMPosition", worldFrame, registry);

      YoFramePoint cmp = new YoFramePoint("cmp", worldFrame, registry);
      cmp.set(0.1, 0.03, 0.0);
      YoFramePoint cornerPoint = new YoFramePoint("cornerPoint", worldFrame, registry);
      cornerPoint.set(0.08, 0.06, 0.0);

      CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), duration / 2.0, cornerPoint, cmp, finalICP);
      CapturePointTools.computeDesiredCapturePointVelocity(omega0.getDoubleValue(), duration / 2.0, cornerPoint, cmp, finalICPVelocity);

      ICPPlannerTrajectoryGenerator icpTrajectoryGenerator = new ICPPlannerTrajectoryGenerator("DoubleSupport", worldFrame, omega0, registry);

      icpTrajectoryGenerator.setTrajectoryTime(duration);
      icpTrajectoryGenerator.setInitialConditions(initialICP, initialICPVelocity, worldFrame);
      icpTrajectoryGenerator.setFinalConditions(finalICP, finalICPVelocity, worldFrame);
      icpTrajectoryGenerator.setInitialCoMPosition(initialCoM, worldFrame);
      icpTrajectoryGenerator.initialize();

      FramePoint finalCoM = new FramePoint(worldFrame);
      icpTrajectoryGenerator.computeFinalCoMPosition(finalCoM);

      FramePoint dummyPoint = new FramePoint();

      double time = 0.0;
      while (time < duration)
      {
         icpTrajectoryGenerator.compute(time);
         icpTrajectoryGenerator.get(desiredCapturePoint);
         icpTrajectoryGenerator.getCoMPosition(desiredCoMPosition);

         desiredCoMVelocity.set(desiredCapturePoint);
         desiredCoMVelocity.sub(integratedCoMPosition);
         desiredCoMVelocity.scale(omega0.getDoubleValue());

         desiredCoMVelocity.getFrameTuple(dummyPoint);
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

   @ContinuousIntegrationTest(estimatedDuration = 100000.0)
   @Test(timeout = 30000)
   public void testSwingIntegration()
   {
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      DoubleYoVariable omega0 = new DoubleYoVariable("Omega0", registry);
      omega0.set(3.0);

      double swingDuration = 2.0;
      double transferDuration = 1.0;

      double timeSpentOnExitCMP = 0.5 * (swingDuration + transferDuration);
      double timeSpentOnEntryCMP = 0.5 * (swingDuration + transferDuration);

      double transferInitialDuration = 0.5 * transferDuration;
      double transferFinalDuration = 0.5 * transferDuration;
      double swingInitialDuration = timeSpentOnEntryCMP - transferFinalDuration;
      double swingFinalDuration = timeSpentOnExitCMP - transferInitialDuration;

      YoFramePoint initialICP = new YoFramePoint("initialICP", worldFrame, registry);
      YoFramePoint finalICP = new YoFramePoint("finalICP", worldFrame, registry);
      YoFramePoint initialCoM = new YoFramePoint("initialCoM", worldFrame, registry);
      finalICP.set(-0.07, 0.4, 0.0);

      YoFramePoint desiredCapturePoint = new YoFramePoint("desiredCapturePoint", worldFrame, registry);
      YoFramePoint desiredCoMPosition = new YoFramePoint("desiredCoMPosition", worldFrame, registry);
      YoFrameVector desiredCoMVelocity = new YoFrameVector("desiredCoMVelocity", worldFrame, registry);
      YoFramePoint integratedCoMPosition = new YoFramePoint("integratedCoMPosition", worldFrame, registry);

      YoFramePoint entryCMP = new YoFramePoint("entryCMP", worldFrame, registry);
      YoFramePoint exitCMP = new YoFramePoint("exitCMP", worldFrame, registry);
      entryCMP.set(0.1, 0.0, 0.0);
      exitCMP.set(0.08, 0.08, 0.0);


      YoFramePoint entryCornerPoint = new YoFramePoint("entryCornerPoint", worldFrame, registry);
      YoFramePoint exitCornerPoint = new YoFramePoint("exitCornerPoint", worldFrame, registry);

      CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), -swingFinalDuration, finalICP, exitCMP, exitCornerPoint);
      CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), -timeSpentOnEntryCMP, exitCornerPoint, entryCMP, entryCornerPoint);
      CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), transferInitialDuration, entryCornerPoint, entryCMP, initialICP);

      ICPPlannerSegmentedTrajectoryGenerator icpTrajectoryGenerator = new ICPPlannerSegmentedTrajectoryGenerator("DoubleSupport", worldFrame, omega0, null, registry);

      icpTrajectoryGenerator.setBoundaryICP(initialICP, finalICP);
      icpTrajectoryGenerator.setCornerPoints(entryCornerPoint, exitCornerPoint);
      icpTrajectoryGenerator.setReferenceCMPs(entryCMP, exitCMP);
      icpTrajectoryGenerator.setReferenceFrames(worldFrame, worldFrame);
      icpTrajectoryGenerator.setInitialCoMPosition(initialCoM, worldFrame);
      icpTrajectoryGenerator.setTrajectoryTime(swingInitialDuration, swingFinalDuration);
      icpTrajectoryGenerator.initialize();

      FramePoint finalCoM = new FramePoint(worldFrame);
      icpTrajectoryGenerator.computeFinalCoMPosition(finalCoM);

      FramePoint dummyPoint = new FramePoint();

      double time = 0.0;
      while (time < swingDuration)
      {
         icpTrajectoryGenerator.compute(time);
         icpTrajectoryGenerator.get(desiredCapturePoint);
         icpTrajectoryGenerator.getCoMPosition(desiredCoMPosition);

         desiredCoMVelocity.set(desiredCapturePoint);
         desiredCoMVelocity.sub(integratedCoMPosition);
         desiredCoMVelocity.scale(omega0.getDoubleValue());

         desiredCoMVelocity.getFrameTuple(dummyPoint);
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
