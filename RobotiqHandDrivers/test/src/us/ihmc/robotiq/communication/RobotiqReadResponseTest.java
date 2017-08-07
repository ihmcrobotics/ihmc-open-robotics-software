package us.ihmc.robotiq.communication;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class RobotiqReadResponseTest
{
   @Test(timeout = 30000)
	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   public void testSetAll()
   {
      final int ITERATIONS = 1000;
      
      Random random = new Random(1234L);
      
      RobotiqReadResponse responseToPack = new RobotiqReadResponse();
      RobotiqReadResponse randomResponse = new RobotiqReadResponse();
      byte[] registerValues = new byte[15];

      for(int i = 0; i < ITERATIONS; i++)
      {
         random.nextBytes(registerValues);
         randomResponse.getGripperStatus().setRegisterValue(registerValues[0]);
         randomResponse.getObjectDetection().setRegisterValue(registerValues[1]);
         randomResponse.getFaultStatus().setRegisterValue(registerValues[2]);
         randomResponse.getFingerAPositionEcho().setRegisterValue(registerValues[3]);
         randomResponse.getFingerAPosition().setRegisterValue(registerValues[4]);
         randomResponse.getFingerACurrent().setRegisterValue(registerValues[5]);
         randomResponse.getFingerBPositionEcho().setRegisterValue(registerValues[6]);
         randomResponse.getFingerBPosition().setRegisterValue(registerValues[7]);
         randomResponse.getFingerBCurrent().setRegisterValue(registerValues[8]);
         randomResponse.getFingerCPositionEcho().setRegisterValue(registerValues[9]);
         randomResponse.getFingerCPosition().setRegisterValue(registerValues[10]);
         randomResponse.getFingerCCurrent().setRegisterValue(registerValues[11]);
         randomResponse.getScissorPositionEcho().setRegisterValue(registerValues[12]);
         randomResponse.getScissorPosition().setRegisterValue(registerValues[13]);
         randomResponse.getScissorCurrent().setRegisterValue(registerValues[14]);
         
         responseToPack.setAll(randomResponse);
         
         assertTrue(responseToPack.equals(randomResponse));
      }
   }

}
