package us.ihmc.valkyrie.kinematics.transmissions;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.valkyrie.kinematics.LinearActuator;
import us.ihmc.valkyrie.kinematics.ValkyrieJointInterface;
import us.ihmc.valkyrie.kinematics.YoValkyrieJointWriter;

public class InefficientPushRodTransmissionTest
{
   private static final boolean DEBUG = false;

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout = 30000)
   public void testForwardBackward()
   {
      Random random = new Random(1234L);

      double reflectBottom = 1.0;
      double reflectTop = 1.0;
      boolean topJointFirst = true;
      
      PushRodTransmissionJoint pushRodTransmissionJoint = PushRodTransmissionJoint.ANKLE;

      InefficientPushRodTransmission inefficientPushrodTransmission = new InefficientPushRodTransmission(pushRodTransmissionJoint, 
            reflectTop, reflectBottom, topJointFirst, null, null);

      LinearActuator[] actuatorData = new LinearActuator[2];
      actuatorData[0] = new LinearActuator("dummy");
      actuatorData[1] = new LinearActuator("dummy");

      ValkyrieJointInterface[] jointData = new ValkyrieJointInterface[2];
      jointData[0] = new YoValkyrieJointWriter("joint0", null);
      jointData[1] = new YoValkyrieJointWriter("joint1", null);

      double increment = 0.05;
      for (double pitch = -Math.PI / 3.0; pitch < Math.PI / 3.0; pitch = pitch + increment)
      {
         for (double roll = -Math.PI / 3.0; roll < Math.PI / 3.0; roll = roll + increment)
         {
            double actuatorForce0 = RandomTools.generateRandomDouble(random, -100.0, 100.0);
            double actuatorForce1 = RandomTools.generateRandomDouble(random, -100.0, 100.0);

            actuatorData[0].setEffortCommand(actuatorForce0);
            actuatorData[1].setEffortCommand(actuatorForce1);

            jointData[0].setPosition(pitch);
            jointData[1].setPosition(roll);

            inefficientPushrodTransmission.actuatorToJointEffort(actuatorData, jointData);

            double pitchTorque = jointData[0].getEffort();
            double rollTorque = jointData[1].getEffort();

            printIfDebug("pitch = " + pitch + ", roll = " + roll);

            printIfDebug("actuatorForce0 = " + actuatorForce0 + ", actuatorForce1 = " + actuatorForce1 + ", pitchTorque = " + pitchTorque + ", rollTorque = " + rollTorque);
            actuatorData[0].setEffortCommand(Double.NaN);
            actuatorData[1].setEffortCommand(Double.NaN);

            jointData[0].setDesiredEffort(pitchTorque);
            jointData[1].setDesiredEffort(rollTorque);

            inefficientPushrodTransmission.jointToActuatorEffort(actuatorData, jointData);

            double actuatorForceReturn0 = actuatorData[0].getEffort();
            double actuatorForceReturn1 = actuatorData[1].getEffort();

            printIfDebug("actuatorForceReturn0 = " + actuatorForceReturn0 + ", actuatorForceReturn1 = " + actuatorForceReturn1);
            printIfDebug("");

            assertEquals(actuatorForce0, actuatorForceReturn0, 1e-7);
            assertEquals(actuatorForce1, actuatorForceReturn1, 1e-7);
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testRegression()
   {
      double reflectBottom = 1.0;
      double reflectTop = 1.0;
      boolean topJointFirst = true;

      PushRodTransmissionJoint pushRodTransmissionJoint = PushRodTransmissionJoint.ANKLE;

      InefficientPushRodTransmission inefficientPushrodTransmission = new InefficientPushRodTransmission(pushRodTransmissionJoint, 
            reflectTop, reflectBottom, topJointFirst, null, null);

      LinearActuator[] actuatorData = new LinearActuator[2];
      actuatorData[0] = new LinearActuator("dummy");
      actuatorData[1] = new LinearActuator("dummy");

      ValkyrieJointInterface[] jointData = new ValkyrieJointInterface[2];
      jointData[0] = new YoValkyrieJointWriter("joint0", null);
      jointData[1] = new YoValkyrieJointWriter("joint1", null);

      double pitch = 0.0;
      double roll = 0.0;

      double actuatorForce1 = 1.0;
      double actuatorForce0 = 0.0;

      double regressionPitchTorque = -0.0366712094326246;
      double regressionRollTorque = -0.034118686505983736;
      verifyARegressionTest(inefficientPushrodTransmission, pitch, roll, actuatorData, jointData, actuatorForce0, actuatorForce1, regressionPitchTorque,
                            regressionRollTorque);

      
      actuatorForce1 = 0.0;
      actuatorForce0 = 1.0;
      regressionPitchTorque = -0.0366712094326246;
      regressionRollTorque = 0.034118686505983736;

      verifyARegressionTest(inefficientPushrodTransmission, pitch, roll, actuatorData, jointData, actuatorForce0, actuatorForce1, regressionPitchTorque,
                            regressionRollTorque);


      pitch = -0.2;
      roll = 0.1;
      actuatorForce1 = 1.0;
      actuatorForce0 = 0.0;
      regressionPitchTorque = -0.03695254741929382;
      regressionRollTorque = -0.02988230913579041;

      verifyARegressionTest(inefficientPushrodTransmission, pitch, roll, actuatorData, jointData, actuatorForce0, actuatorForce1, regressionPitchTorque,
                            regressionRollTorque);

      actuatorForce1 = 0.0;
      actuatorForce0 = 1.0;
      regressionPitchTorque = -0.034740329545336665;
      regressionRollTorque = 0.03440182578269918;

      verifyARegressionTest(inefficientPushrodTransmission, pitch, roll, actuatorData, jointData, actuatorForce0, actuatorForce1, regressionPitchTorque,
                            regressionRollTorque);
//      System.out.println("made it this far");
   }

   private void verifyARegressionTest(InefficientPushRodTransmission inefficientPushrodTransmission, double pitch, double roll, LinearActuator[] actuatorData,
                                      ValkyrieJointInterface[] jointData, double actuatorForce0, double actuatorForce1, double pitchTorque, double rollTorque)
   {
      actuatorData[0].setEffortCommand(actuatorForce0);
      actuatorData[1].setEffortCommand(actuatorForce1);

      jointData[0].setPosition(pitch);
      jointData[1].setPosition(roll);

      inefficientPushrodTransmission.actuatorToJointEffort(actuatorData, jointData);

      double computedPitchTorque = jointData[0].getEffort();
      double computedRollTorque = jointData[1].getEffort();
      if(DEBUG){         
         System.out.println("pitch tau: " + computedPitchTorque);
         System.out.println("roll tau: " + computedRollTorque);
      }
      assertEquals(computedPitchTorque, pitchTorque, 1e-7);
      assertEquals(computedRollTorque, rollTorque, 1e-7);
   }

   private void printIfDebug(String string)
   {
      if (DEBUG)
         System.out.println(string);
   }

}
