package us.ihmc.exampleSimulations.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

//import us.ihmc.utilities.code.unitTesting.runner.JUnitTestSuiteRunner;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
   us.ihmc.exampleSimulations.springflamingo.SpringFlamingoSimulationTest.class
})

public class ExampleSimulationsDockerTestSuite
{
   public static void main(String[] args)
   {
      //new JUnitTestSuiteRunner(ExampleSimulationsDockerTestSuite.class);
   }
}

