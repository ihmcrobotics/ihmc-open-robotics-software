package us.ihmc.robotics.kinematics.fourbar;

import org.junit.Before;

import us.ihmc.tools.MemoryTools;


/**
 * It makes sure that the fast runner four bar calculator and the calculator used in Beast, Steppr, and Wanderer give the same result
 */
public class FourBarCalculatorsComparisonTest
{
//   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   
   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }
   
//   @After
//   public void destroySimulationAndRecycleMemory()
//   {
//      if (simulationTestingParameters.getKeepSCSUp())
//      {
//         ThreadTools.sleepForever();
//      }
//
//      if (blockingSimulationRunner != null)
//      {
//         blockingSimulationRunner.destroySimulation();
//         blockingSimulationRunner = null;
//      }
//
//      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
//   }
//
//   private BlockingSimulationRunner blockingSimulationRunner;
  
}
