package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.AMGeneration.FootstepAngularMomentumPredictorTest;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.AMGeneration.TorqueTrajectoryTest;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.CMPGeneration.ReferenceCMPTrajectoryGeneratorTest;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.CoMGeneration.CoMIntegrationToolsTest;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.CoPGeneration.CoPPlanningToolsTest;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.CoPGeneration.CoPPointsInFootTest;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.CoPGeneration.CoPTrajectory;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.CoPGeneration.CoPTrajectoryTest;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.CoPGeneration.FootstepDataTest;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.CoPGeneration.ReferenceCenterOfPressureWaypointCalculatorTest;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.ICPGeneration.CapturePointToolsTest;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.ICPGeneration.ReferenceICPTrajectoryGeneratorTest;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.ICPGeneration.SmoothCapturePointAdjustmentToolboxTest;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.ICPGeneration.SmoothCapturePointToolboxTest;
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
