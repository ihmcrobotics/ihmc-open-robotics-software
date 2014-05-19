package us.ihmc.valkyrie.kinematics.transmissions;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;

import java.util.Random;

import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.valkyrie.kinematics.ValkyrieJoint;
import us.ihmc.valkyrie.roboNet.DummyTurboDriver;
import us.ihmc.valkyrie.roboNet.TurboDriver;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class ComparePushRodTransmissionsTest
{
   private static final boolean DEBUG = false;
   private static final boolean VISUALIZE = false;
   
   @Test
   public void testCompareInefficientToEfficientAnkle()
   {
      Random random = new Random(1255L);
      double epsilon = 1e-7;

      double reflect = 1.0;
      PushRodTransmissionJoint pushRodTransmissionJoint = PushRodTransmissionJoint.ANKLE;
      
      YoVariableRegistry registry = new YoVariableRegistry("test");
      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();
      
      InefficientPushRodTransmission inefficientPushrodTransmission = new InefficientPushRodTransmission(pushRodTransmissionJoint, reflect, registry, dynamicGraphicObjectsListRegistry);
      EfficientPushRodTransmission efficientPushrodTransmission = new EfficientPushRodTransmission(pushRodTransmissionJoint, reflect);

      compareTwoPushRodTransmissionInterfaces(random, epsilon, inefficientPushrodTransmission, efficientPushrodTransmission, registry, dynamicGraphicObjectsListRegistry);
   }
   
   @Ignore
   @Test
   public void testCompareInefficientToEfficientWaist()
   {
      Random random = new Random(1255L);
      double epsilon = 1e-7;

      double reflect = 1.0;
      PushRodTransmissionJoint pushRodTransmissionJoint = PushRodTransmissionJoint.WAIST;
      
      YoVariableRegistry registry = new YoVariableRegistry("test");
      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();

      InefficientPushRodTransmission inefficientPushrodTransmission = new InefficientPushRodTransmission(pushRodTransmissionJoint, reflect, registry, dynamicGraphicObjectsListRegistry);
      EfficientPushRodTransmission efficientPushrodTransmission = new EfficientPushRodTransmission(pushRodTransmissionJoint, reflect);

      compareTwoPushRodTransmissionInterfaces(random, epsilon, inefficientPushrodTransmission, efficientPushrodTransmission, registry, dynamicGraphicObjectsListRegistry);
   }
   
   // Seems that the interpolated should be same as the pushrod when use futeks is false. Should try to get this to work
   // Or figure out if the interpolated is just plain wrong.
   @Ignore 
   @Test
   public void testCompareInefficientToInterpolatedAnkles()
   {
      Random random = new Random(1255L);

      String ankleNamespace = "v1_ankle";
      double compliance = 0.0;
      double reflect = 1.0;
      PushRodTransmissionJoint pushRodTransmissionJoint = PushRodTransmissionJoint.ANKLE;

      YoVariableRegistry registry = new YoVariableRegistry("test");
      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();

      InterpolatedPushRodTransmission interpolatedPushRodTransmission = new InterpolatedPushRodTransmission(ankleNamespace, reflect, compliance);
      InefficientPushRodTransmission inefficientPushrodTransmission = new InefficientPushRodTransmission(pushRodTransmissionJoint, reflect, registry, dynamicGraphicObjectsListRegistry);
      inefficientPushrodTransmission.setUseFuteks(false);

      double epsilon = 1e-7;

      compareTwoPushRodTransmissionInterfaces(random, epsilon, inefficientPushrodTransmission, interpolatedPushRodTransmission, registry, dynamicGraphicObjectsListRegistry);
   }
   
   @Ignore
   @Test
   public void testCompareInefficientToInterpolatedWaist()
   {
      Random random = new Random(1255L);

      String ankleNamespace = "v1_waist";
      double compliance = 0.0;
      double reflect = 1.0;
      PushRodTransmissionJoint pushRodTransmissionJoint = PushRodTransmissionJoint.WAIST;

      YoVariableRegistry registry = new YoVariableRegistry("test");
      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();

      InefficientPushRodTransmission inefficientPushrodTransmission = new InefficientPushRodTransmission(pushRodTransmissionJoint, reflect, registry, dynamicGraphicObjectsListRegistry);
      InterpolatedPushRodTransmission interpolatedPushRodTransmission = new InterpolatedPushRodTransmission(ankleNamespace, reflect, compliance);
      inefficientPushrodTransmission.setUseFuteks(false);

      double epsilon = 1e-7;

      compareTwoPushRodTransmissionInterfaces(random, epsilon, inefficientPushrodTransmission, interpolatedPushRodTransmission, registry, dynamicGraphicObjectsListRegistry);
   }

   
   private void compareTwoPushRodTransmissionInterfaces(Random random, double epsilon, PushRodTransmissionInterface pushrodTransmissionA,
           PushRodTransmissionInterface pushrodTransmissionB, YoVariableRegistry registry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      Robot robot = new Robot("comparePushrodTransmission");
      robot.getRobotsYoVariableRegistry().addChild(registry);

      DoubleYoVariable topJointAngle = new DoubleYoVariable("topJointAngle", registry);
      DoubleYoVariable bottomJointAngle = new DoubleYoVariable("bottomJointAngle", registry);
      
      DoubleYoVariable actuatorForceA0 = new DoubleYoVariable("actuatorForceA0", registry);
      DoubleYoVariable actuatorForceA1 = new DoubleYoVariable("actuatorForceA1", registry);
      
      DoubleYoVariable actuatorForceB0 = new DoubleYoVariable("actuatorForceB0", registry);
      DoubleYoVariable actuatorForceB1 = new DoubleYoVariable("actuatorForceB1", registry);
      
      DoubleYoVariable force0 = new DoubleYoVariable("force0", registry);
      DoubleYoVariable force1 = new DoubleYoVariable("force1", registry);
      
      DoubleYoVariable topJointTorqueA = new DoubleYoVariable("topJointTorqueA", registry);
      DoubleYoVariable bottomJointTorqueA = new DoubleYoVariable("bottomJointTorqueA", registry);
      DoubleYoVariable topJointTorqueB = new DoubleYoVariable("topJointTorqueB", registry);
      DoubleYoVariable bottomJointTorqueB = new DoubleYoVariable("bottomJointTorqueB", registry);
      
      DoubleYoVariable topJointTorque = new DoubleYoVariable("topJointTorque", registry);
      DoubleYoVariable bottomJointTorque = new DoubleYoVariable("bottomJointTorque", registry);

      SimulationConstructionSet scs;
      if (VISUALIZE)
      {
         scs = new SimulationConstructionSet(robot);
         dynamicGraphicObjectsListRegistry.addDynamicGraphicsObjectListsToSimulationConstructionSet(scs);
         scs.setCameraPosition(0.62, -0.4, 1.26);
         scs.setCameraFix(0.0, 0.0, 1.02);

         scs.startOnAThread();
      }
      else
      {
         scs = null;
      }
      
      
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

            topJointAngle.set(pitch);
            bottomJointAngle.set(roll);
            
            // Check the actuatorToJointEffort

            force0.set(RandomTools.generateRandomDouble(random, -100.0, 100.0));
            force1.set(RandomTools.generateRandomDouble(random, -100.0, 100.0));

            actuatorData[0].setEffortCommand(force0.getDoubleValue());
            actuatorData[1].setEffortCommand(force1.getDoubleValue());

            jointData[0].setEffort(Double.NaN);
            jointData[1].setEffort(Double.NaN);

            pushrodTransmissionA.jointToActuatorPosition(actuatorData, jointData);
            pushrodTransmissionA.actuatorToJointEffort(actuatorData, jointData);

            topJointTorqueA.set(jointData[0].getEffort());
            bottomJointTorqueA.set(jointData[1].getEffort());

            jointData[0].setEffort(Double.NaN);
            jointData[1].setEffort(Double.NaN);

            pushrodTransmissionB.jointToActuatorPosition(actuatorData, jointData);
            pushrodTransmissionB.actuatorToJointEffort(actuatorData, jointData);

            topJointTorqueB.set(jointData[0].getEffort());
            bottomJointTorqueB.set(jointData[1].getEffort());

            printIfDebug("f0 = " + force0 + ", f1 = " + force1 + ", topJointTorqueA = " + topJointTorqueA.getDoubleValue ()+ ", bottomJointTorqueA = " + bottomJointTorqueA.getDoubleValue()
                         + ", topJointTorqueB = " + topJointTorqueB.getDoubleValue() + ", effRollTau = " + bottomJointTorqueB.getDoubleValue());

            assertFalse(Double.isNaN(topJointTorqueA.getDoubleValue()));
            assertFalse(Double.isNaN(bottomJointTorqueA.getDoubleValue()));

            assertEquals(topJointTorqueA.getDoubleValue(), topJointTorqueB.getDoubleValue(), epsilon);
            assertEquals(bottomJointTorqueA.getDoubleValue(), bottomJointTorqueB.getDoubleValue(), epsilon);

            if (VISUALIZE)
            {
               scs.tickAndUpdate();
            }
         }
      }
      
      for (double pitch = -1.0; pitch < 1.0; pitch = pitch + increment)
      {
         for (double roll = -0.5; roll < 0.5; roll = roll + increment)
         {
            printIfDebug("pitch = " + pitch + ", roll = " + roll);

            jointData[0].setPosition(pitch);
            jointData[1].setPosition(roll);

            topJointAngle.set(pitch);
            bottomJointAngle.set(roll);

            // Check the jointToActuatorEffort
            topJointTorque.set(RandomTools.generateRandomDouble(random, -40.0, 40.0));
            bottomJointTorque.set(RandomTools.generateRandomDouble(random, -40.0, 40.0));

            jointData[0].setDesiredEffort(topJointTorque.getDoubleValue());
            jointData[1].setDesiredEffort(bottomJointTorque.getDoubleValue());

            actuatorData[0].setEffortCommand(Double.NaN);
            actuatorData[1].setEffortCommand(Double.NaN);

            pushrodTransmissionA.jointToActuatorEffort(actuatorData, jointData);

            actuatorForceA0.set(actuatorData[0].getEffort());
            actuatorForceA1.set(actuatorData[1].getEffort());

            actuatorData[0].setEffortCommand(Double.NaN);
            actuatorData[1].setEffortCommand(Double.NaN);

            pushrodTransmissionB.jointToActuatorEffort(actuatorData, jointData);

            actuatorForceB0.set(actuatorData[0].getEffort());
            actuatorForceB1.set(actuatorData[1].getEffort());

            printIfDebug("topJointTorque = " + topJointTorque.getDoubleValue() + ", bottomJointTorque = " + bottomJointTorque.getDoubleValue() + ", actuatorForceA0 = " + actuatorForceA0
                         + ", actuatorForceA1 = " + actuatorForceA1 + ", actuatorForceB0 = " + actuatorForceB0
                         + ", actuatorForceB1 = " + actuatorForceB1);
            printIfDebug("");

            assertFalse(Double.isNaN(actuatorForceA0.getDoubleValue()));
            assertFalse(Double.isNaN(actuatorForceA1.getDoubleValue()));

            assertEquals(actuatorForceA0.getDoubleValue(), actuatorForceB0.getDoubleValue(), epsilon);
            assertEquals(actuatorForceA1.getDoubleValue(), actuatorForceB1.getDoubleValue(), epsilon);
            
            if (VISUALIZE)
            {
               scs.tickAndUpdate();
            }
         }
      }
      
      if (VISUALIZE)
      {
         scs.gotoInPointNow();
         scs.setIndex(1);
         scs.setInPoint();
         scs.cropBuffer();

         ThreadTools.sleepForever();
      }
   }

   private void printIfDebug(String string)
   {
      if (DEBUG)
         System.out.println(string);

   }
}
