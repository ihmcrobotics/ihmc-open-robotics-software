package us.ihmc.valkyrie.kinematics.transmissions;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.junit.Test;

import us.ihmc.utilities.RandomTools;
import us.ihmc.valkyrie.kinematics.ValkyrieJoint;
import us.ihmc.valkyrie.roboNet.DummyTurboDriver;
import us.ihmc.valkyrie.roboNet.TurboDriver;

public class InefficientPushRodTransmissionTest
{
   private static final boolean DEBUG = true;

   @Test
   public void testForwardBackward()
   {
      Random random = new Random(1234L);
      
      double reflect = 1.0;
      InefficientPushRodTransmission inefficientPushrodTransmission = new InefficientPushRodTransmission(reflect, null, null);

      TurboDriver[] actuatorData = new DummyTurboDriver[2];
      actuatorData[0] = new DummyTurboDriver();
      actuatorData[1] = new DummyTurboDriver();

      ValkyrieJoint[] jointData = new ValkyrieJoint[2];
      jointData[0] = new ValkyrieJoint("joint0", null);
      jointData[1] = new ValkyrieJoint("joint1", null);

      double increment = 0.02;
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

            printIfDebug("actuatorForce0 = " + actuatorForce0 + ", actuatorForce1 = " + actuatorForce1 + ", pitchTorque = " + pitchTorque + ", rollTorque = "
                         + rollTorque);
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
   
   
   @Test
   public void testRegression()
   {      
      double reflect = 1.0;

      InefficientPushRodTransmission inefficientPushrodTransmission = new InefficientPushRodTransmission(reflect, null, null);

      TurboDriver[] actuatorData = new DummyTurboDriver[2];
      actuatorData[0] = new DummyTurboDriver();
      actuatorData[1] = new DummyTurboDriver();

      ValkyrieJoint[] jointData = new ValkyrieJoint[2];
      jointData[0] = new ValkyrieJoint("joint0", null);
      jointData[1] = new ValkyrieJoint("joint1", null);

      double pitch = 0.0;
      double roll = 0.0;
     
      

            double actuatorForce0 = 1.0;
            double actuatorForce1 = 0.0;

            actuatorData[0].setEffortCommand(actuatorForce0);
            actuatorData[1].setEffortCommand(actuatorForce1);

            jointData[0].setPosition(pitch);
            jointData[1].setPosition(roll);

            inefficientPushrodTransmission.actuatorToJointEffort(actuatorData, jointData);

            double pitchTorque = jointData[0].getEffort();
            double rollTorque = jointData[1].getEffort();

            assertEquals(pitchTorque, -0.0366712094326246, 1e-7);
            assertEquals(pitchTorque, -0.0366712094326246, 1e-7);
            

//            computeAndPrint(inefficientPushrodTransmissionJacobian, pitch, roll, jacobian, scs);
//            assertJacobianEquals(jacobian, -0.0366712094326246, -0.0366712094326246, 0.034118686505983736, -0.034118686505983736);

            
            printIfDebug("pitch = " + pitch + ", roll = " + roll);

            printIfDebug("actuatorForce0 = " + actuatorForce0 + ", actuatorForce1 = " + actuatorForce1 + ", pitchTorque = " + pitchTorque + ", rollTorque = "
                         + rollTorque);
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

   private void printIfDebug(String string)
   {
      if (DEBUG)
         System.out.println(string);
   }

}
