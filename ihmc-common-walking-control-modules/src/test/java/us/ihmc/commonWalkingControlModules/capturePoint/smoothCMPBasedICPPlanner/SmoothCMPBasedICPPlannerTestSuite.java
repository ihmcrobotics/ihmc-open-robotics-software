package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.AMGeneration.FootstepAngularMomentumPredictorTest;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.AMGeneration.TorqueTrajectoryTest;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CMPGeneration.ReferenceCMPTrajectoryGeneratorTest;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoMGeneration.CoMIntegrationToolsTest;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.*;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.ICPGeneration.CapturePointToolsTest;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.ICPGeneration.ReferenceICPTrajectoryGeneratorTest;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.ICPGeneration.SmoothCapturePointAdjustmentToolboxTest;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.ICPGeneration.SmoothCapturePointToolboxTest;
import us.ihmc.commons.MutationTestFacilitator;

@RunWith(Suite.class)
@Suite.SuiteClasses(
        {
           FootstepAngularMomentumPredictorTest.class,
           TorqueTrajectoryTest.class,
           ReferenceCMPTrajectoryGeneratorTest.class,
           CoMIntegrationToolsTest.class,
           SmoothCapturePointAdjustmentToolboxTest.class,
           CoPPointsInFootTest.class,
           ReferenceCenterOfPressureWaypointCalculatorTest.class,
           CapturePointToolsTest.class,
           ReferenceICPTrajectoryGeneratorTest.class,
           SmoothCapturePointAdjustmentToolboxTest.class,
           SmoothCapturePointToolboxTest.class,
           CoPPlanningToolsTest.class,
           CoPTrajectoryTest.class,
           FootstepDataTest.class,
           SmoothCMPBasedICPPlannerTest.class
        }
)

public class SmoothCMPBasedICPPlannerTestSuite
{
   public static void main(String args[])
   {
      MutationTestFacilitator.facilitateMutationTestForPackage(SmoothCMPBasedICPPlannerTestSuite.class);
   }
}
