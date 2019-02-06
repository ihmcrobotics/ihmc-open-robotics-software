package us.ihmc.simulationConstructionSetTools.util.dataProcessors;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;

public class RobotAllJointsDataCheckerTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();


   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         //ThreadTools.sleepForever();
      }
   }

   @Test
   public void test()
   {
      TwoLinkRobotForTesting twoLinkRobotForTesting = new TwoLinkRobotForTesting();

      SimulationConstructionSet scs = new SimulationConstructionSet(twoLinkRobotForTesting, simulationTestingParameters);
      scs.setDT(0.00001, 100);
      scs.startOnAThread();

      twoLinkRobotForTesting.setElbowPosition(0.0);
      twoLinkRobotForTesting.setUpperPosition(3.0);
      
      twoLinkRobotForTesting.setElbowVelocity(-2.0);
      twoLinkRobotForTesting.setUpperVelocity(-3.0);
      
      scs.simulate(6.0);
      
      while(scs.isSimulating())
      {
         Thread.yield();
      }
      
      RobotAllJointsDataChecker robotAllJointsDataChecker = new RobotAllJointsDataChecker(scs, twoLinkRobotForTesting);
      robotAllJointsDataChecker.cropFirstPoint();
      
      scs.applyDataProcessingFunction(robotAllJointsDataChecker);
   }

}
