package us.ihmc.valkyrie.kinematics.transmissions;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;

import java.util.Random;

import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.utilities.RandomTools;
import us.ihmc.valkyrie.kinematics.ValkyrieJoint;
import us.ihmc.valkyrie.roboNet.DummyTurboDriver;
import us.ihmc.valkyrie.roboNet.TurboDriver;

public class ComparePushRodTransmissionsTest
{
   private static final boolean DEBUG = false;

   @Test
   public void testCompareInefficientToEfficient()
   {
      Random random = new Random(1255L);
      double epsilon = 1e-7;

      double reflect = 1.0;
      PushRodTransmissionJoint pushRodTransmissionJoint = PushRodTransmissionJoint.ANKLE;
      
      InefficientPushRodTransmission inefficientPushrodTransmission = new InefficientPushRodTransmission(pushRodTransmissionJoint, reflect, null, null);
      EfficientPushRodTransmission efficientPushrodTransmission = new EfficientPushRodTransmission(pushRodTransmissionJoint, reflect);

      compareTwoPushRodTransmissionInterfaces(random, epsilon, inefficientPushrodTransmission, efficientPushrodTransmission);
   }
   
   // Seems that the interpolated should be same as the pushrod when use futeks is false. Should try to get this to work
   // Or figure out if the interpolated is just plain wrong.
   @Ignore 
   @Test
   public void testCompareInefficientToInterpolated()
   {
      Random random = new Random(1255L);

      String ankleNamespace = "v1_ankle";
      double compliance = 0.0;
      double reflect = 1.0;
      PushRodTransmissionJoint pushRodTransmissionJoint = PushRodTransmissionJoint.ANKLE;

      InterpolatedPushRodTransmission interpolatedPushRodTransmission = new InterpolatedPushRodTransmission(ankleNamespace, reflect, compliance);
      InefficientPushRodTransmission inefficientPushrodTransmission = new InefficientPushRodTransmission(pushRodTransmissionJoint, reflect, null, null);
      inefficientPushrodTransmission.setUseFuteks(false);

      double epsilon = 1e-7;

      compareTwoPushRodTransmissionInterfaces(random, epsilon, inefficientPushrodTransmission, interpolatedPushRodTransmission);
   }


   private void compareTwoPushRodTransmissionInterfaces(Random random, double epsilon, PushRodTransmissionInterface inefficientPushrodTransmission,
           PushRodTransmissionInterface efficientPushrodTransmission)
   {
      TurboDriver[] actuatorData = new DummyTurboDriver[2];
      actuatorData[0] = new DummyTurboDriver();
      actuatorData[1] = new DummyTurboDriver();

      ValkyrieJoint[] jointData = new ValkyrieJoint[2];
      jointData[0] = new ValkyrieJoint("joint0", null);
      jointData[1] = new ValkyrieJoint("joint1", null);

      double increment = 0.05;

      for (double pitch = -1.0; pitch < 1.0; pitch = pitch + increment)
      {
         for (double roll = -0.5; roll < 0.5; roll = roll + increment)
         {
            printIfDebug("pitch = " + pitch + ", roll = " + roll);

            jointData[0].setPosition(pitch);
            jointData[1].setPosition(roll);

            // Check the actuatorToJointEffort

            double force0 = RandomTools.generateRandomDouble(random, -100.0, 100.0);
            double force1 = RandomTools.generateRandomDouble(random, -100.0, 100.0);

            actuatorData[0].setEffortCommand(force0);
            actuatorData[1].setEffortCommand(force1);

            jointData[0].setEffort(Double.NaN);
            jointData[1].setEffort(Double.NaN);

            inefficientPushrodTransmission.jointToActuatorPosition(actuatorData, jointData);
            inefficientPushrodTransmission.actuatorToJointEffort(actuatorData, jointData);

            double inefficientPitchTorque = jointData[0].getEffort();
            double inefficientRollTorque = jointData[1].getEffort();

            jointData[0].setEffort(Double.NaN);
            jointData[1].setEffort(Double.NaN);

            efficientPushrodTransmission.jointToActuatorPosition(actuatorData, jointData);
            efficientPushrodTransmission.actuatorToJointEffort(actuatorData, jointData);

            double efficientPitchTorque = jointData[0].getEffort();
            double efficientRollTorque = jointData[1].getEffort();

            printIfDebug("f0 = " + force0 + ", f1 = " + force1 + ", ineffPitchTau = " + inefficientPitchTorque + ", ineffRollTau = " + inefficientRollTorque
                         + ", effPitchTau = " + efficientPitchTorque + ", effRollTau = " + efficientRollTorque);

            assertFalse(Double.isNaN(inefficientPitchTorque));
            assertFalse(Double.isNaN(inefficientRollTorque));

            assertEquals(inefficientPitchTorque, efficientPitchTorque, epsilon);
            assertEquals(inefficientRollTorque, efficientRollTorque, epsilon);

            // Check the jointToActuatorEffort
            double pitchTorque = RandomTools.generateRandomDouble(random, -40.0, 40.0);
            double rollTorque = RandomTools.generateRandomDouble(random, -40.0, 40.0);

            jointData[0].setDesiredEffort(pitchTorque);
            jointData[1].setDesiredEffort(rollTorque);

            actuatorData[0].setEffortCommand(Double.NaN);
            actuatorData[1].setEffortCommand(Double.NaN);

            inefficientPushrodTransmission.jointToActuatorEffort(actuatorData, jointData);

            double inefficientActuatorForce0 = actuatorData[0].getEffort();
            double inefficientActuatorForce1 = actuatorData[1].getEffort();

            actuatorData[0].setEffortCommand(Double.NaN);
            actuatorData[1].setEffortCommand(Double.NaN);

            efficientPushrodTransmission.jointToActuatorEffort(actuatorData, jointData);

            double efficientActuatorForce0 = actuatorData[0].getEffort();
            double efficientActuatorForce1 = actuatorData[1].getEffort();

            printIfDebug("pitchTorque = " + pitchTorque + ", rollTorque = " + rollTorque + ", inefficientActuatorForce0 = " + inefficientActuatorForce0
                         + ", inefficientActuatorForce1 = " + inefficientActuatorForce1 + ", efficientActuatorForce0 = " + efficientActuatorForce0
                         + ", efficientActuatorForce1 = " + efficientActuatorForce1);
            printIfDebug("");

            assertFalse(Double.isNaN(inefficientActuatorForce0));
            assertFalse(Double.isNaN(inefficientActuatorForce1));

            assertEquals(inefficientActuatorForce0, efficientActuatorForce0, epsilon);
            assertEquals(inefficientActuatorForce1, efficientActuatorForce1, epsilon);
         }
      }
   }

   private void printIfDebug(String string)
   {
      if (DEBUG)
         System.out.println(string);

   }
}
