package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import static org.junit.Assert.*;

import java.util.List;

import org.junit.Test;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.DummyExtendedCapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ExtendedCapturePointPlannerParameters;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.robotSide.RobotSide;

public class ReferenceCenterOfPressureTrajectoryCalculatorTest
{

   @Test
   public void testCoPWayPointGeneration()
   {      
      //BipedSupportPolygons bipedSupportPolygons = new BipedSupportPolygons(ankleZUpFrames, midFeetZUpFrame, soleZUpFrames, null, null);
      ReferenceCenterOfPressureTrajectoryCalculator testCoPGenerator = new ReferenceCenterOfPressureTrajectoryCalculator("TestClass");
      assertTrue("Object not initialized", testCoPGenerator != null);
      DummyExtendedCapturePointPlannerParameters icpPlannerParameters = new DummyExtendedCapturePointPlannerParameters();      
      //testCoPGenerator.initializeParameters(icpPlannerParameters, bipedSupportPolygons, contactableFeet, parentRegistry);
   }

}
