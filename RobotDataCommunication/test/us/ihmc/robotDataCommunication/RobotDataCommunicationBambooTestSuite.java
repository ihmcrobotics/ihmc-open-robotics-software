package us.ihmc.robotDataCommunication;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.utilities.code.unitTesting.generator.JUnitTestSuiteGenerator;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
//    us.ihmc.robotDataCommunication.YoVariableConnectionBurstTest.class
})

public class RobotDataCommunicationBambooTestSuite
{
   public static void main(String[] args)
   {
      JUnitTestSuiteGenerator.generateTestSuite(RobotDataCommunicationBambooTestSuite.class);
   }
}