package us.ihmc.commonWalkingControlModules.controlModules;

import static org.junit.Assert.assertEquals;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.LegStrengthCalculator;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;


public class TeeterTotterLegStrengthCalculatorTest
{
   private LegStrengthCalculator legStrengthCalculator;
   private SideDependentList<Double> legStrengths;
   private SideDependentList<FramePoint2d> virtualToePoints;
   private FramePoint2d coPDesired;
   private final ReferenceFrame world = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame otherWorld = ReferenceFrame.constructARootFrame("otherWorld");

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }
   
   @After
   public void showMemoryUsageAfterTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
   @Before
   public void setUp()
   {
      YoVariableRegistry registry = new YoVariableRegistry("TeeterTotterTest");
      legStrengthCalculator = new TeeterTotterLegStrengthCalculator(registry);
      legStrengths = new SideDependentList<Double>();
      virtualToePoints = new SideDependentList<FramePoint2d>();
   }

	@DeployableTestMethod(duration = 0.4)
	@Test(timeout = 30000)
   public void testCenter()
   {
      virtualToePoints.put(RobotSide.LEFT, new FramePoint2d(world, 0.0, 0.0));
      virtualToePoints.put(RobotSide.RIGHT, new FramePoint2d(world, 1.0, 1.0));
      coPDesired = new FramePoint2d(world, 0.5, 0.5);

      legStrengthCalculator.packLegStrengths(legStrengths, virtualToePoints, coPDesired);

      for (RobotSide robotSide : RobotSide.values)
      {
         assertEquals(legStrengths.get(robotSide), 0.5, 1e-10);
      }
   }

	@DeployableTestMethod(duration = 0.3)
	@Test(timeout = 30000,expected = RuntimeException.class)
   public void testFrameMismatch1()
   {
      virtualToePoints.put(RobotSide.LEFT, new FramePoint2d(world, 0.0, 0.0));
      virtualToePoints.put(RobotSide.LEFT, new FramePoint2d(otherWorld, 1.0, 1.0));
      coPDesired = new FramePoint2d(world, 0.5, 0.5);
      legStrengthCalculator.packLegStrengths(legStrengths, virtualToePoints, coPDesired);
   }

	@DeployableTestMethod(duration = 0.4)
	@Test(timeout = 30000,expected = RuntimeException.class)
   public void testFrameMismatch2()
   {
      virtualToePoints.put(RobotSide.LEFT, new FramePoint2d(world, 0.0, 0.0));
      virtualToePoints.put(RobotSide.LEFT, new FramePoint2d(world, 1.0, 1.0));
      coPDesired = new FramePoint2d(otherWorld, 0.5, 0.5);
      legStrengthCalculator.packLegStrengths(legStrengths, virtualToePoints, coPDesired);
   }

	@DeployableTestMethod(duration = 0.4)
	@Test(timeout = 30000)
   public void testAllWeightOnOneLeg()
   {
      virtualToePoints.put(RobotSide.LEFT, new FramePoint2d(world, 0.0, 0.0));
      virtualToePoints.put(RobotSide.RIGHT, new FramePoint2d(world, 1.0, 1.0));

      for (RobotSide supportSide : RobotSide.values)
      {
         coPDesired = new FramePoint2d(virtualToePoints.get(supportSide));

         legStrengthCalculator.packLegStrengths(legStrengths, virtualToePoints, coPDesired);
         assertEquals(legStrengths.get(supportSide), 1.0, 1e-10);
         assertEquals(legStrengths.get(supportSide.getOppositeSide()), 0.0, 1e-10);
      }
   }

	@DeployableTestMethod(duration = 0.4)
	@Test(timeout = 30000)
   public void testCoPOffVTPLineSegment()
   {
      virtualToePoints.put(RobotSide.LEFT, new FramePoint2d(world, 0.0, 0.0));
      virtualToePoints.put(RobotSide.RIGHT, new FramePoint2d(world, 1.0, 1.0));
      coPDesired = new FramePoint2d(world, 2.0, 2.0);
      legStrengthCalculator.packLegStrengths(legStrengths, virtualToePoints, coPDesired);
      assertEquals(legStrengths.get(RobotSide.RIGHT), 1.0, 1e-10);
   }
}
