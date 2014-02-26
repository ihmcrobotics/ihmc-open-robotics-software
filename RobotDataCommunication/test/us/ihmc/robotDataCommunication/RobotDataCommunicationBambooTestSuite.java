package us.ihmc.robotDataCommunication;

import org.junit.runner.*;
import org.junit.runners.*;

import us.ihmc.utilities.test.JUnitTestSuiteConstructor;

@RunWith(Suite.class)
@Suite.SuiteClasses(
{
	   us.ihmc.robotDataCommunication.YoVariableConnectionBurstTest.class
})


public class RobotDataCommunicationBambooTestSuite {

	public static void main(String[] args) {
	      String packageName = "us.ihmc.RobotDataCommunication";
	      System.out.println(JUnitTestSuiteConstructor.createTestSuite("RobotDataCommunicationBambooTestSuite", packageName));

	}

}