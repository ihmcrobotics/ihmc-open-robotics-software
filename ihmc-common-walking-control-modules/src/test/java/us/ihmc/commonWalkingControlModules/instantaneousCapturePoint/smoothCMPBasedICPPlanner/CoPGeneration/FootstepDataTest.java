package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.CoPGeneration;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.commons.Epsilons;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootstepDataTest
{
   @Test
   public void testConstructor()
   {
      FootstepData testObject = new FootstepData();
      assertTrue(testObject != null);
      Footstep testFootstep = new Footstep(RobotSide.LEFT);
      FootstepTiming testFootstepTiming = new FootstepTiming(0.2, 0.1);
      testFootstepTiming.setAbsoluteTime(0.1, 0.5);
      testObject.set(testFootstep, testFootstepTiming);
      assertTrue(testObject.getFootstep() == testFootstep);
      assertTrue(testObject.getStepTime() == testFootstepTiming.getStepTime());
      assertTrue(testObject.getSwingSide() == testFootstep.getRobotSide());
      assertTrue(testObject.getSupportSide() == testFootstep.getRobotSide().getOppositeSide());
      assertTrue(testObject.getSwingTime() == testFootstepTiming.getSwingTime());
      assertTrue(testObject.getTransferTime() == testFootstepTiming.getTransferTime());
      assertTrue(MathTools.epsilonEquals(testObject.getTransferStartTime(), testFootstepTiming.getExecutionStartTime(), Epsilons.ONE_BILLIONTH));
      assertTrue(testObject.getSwingStartTime() == testFootstepTiming.getSwingStartTime());
      assertTrue(testObject.getFootstepPoseReferenceFrame() == testFootstep.getFootstepPose().getReferenceFrame());
      assertTrue(testObject.getFramePose() == testFootstep.getFootstepPose());
      testObject.setSwingTime(0.8);
      assertTrue(testObject.getSwingTime() == 0.8);
      Footstep newFootstep = new Footstep(RobotSide.LEFT);
      testObject.setFootstep(newFootstep);
      assertTrue(testObject.getFootstep() == newFootstep);
   }
   
   @Test 
   public void testCopy()
   {
      Footstep footstep = new Footstep(RobotSide.RIGHT);
      FootstepTiming timing = new FootstepTiming(0.1, 1.2);
      FootstepData obj1 = new FootstepData(footstep, timing);
      FootstepData obj2 = new FootstepData();
      assertTrue(obj2.getFootstep() == null);
      obj2.set(obj1);
      assertTrue(obj2.getFootstep() == footstep);
      assertTrue(obj2.getStepTime() == timing.getStepTime());
   }
}
