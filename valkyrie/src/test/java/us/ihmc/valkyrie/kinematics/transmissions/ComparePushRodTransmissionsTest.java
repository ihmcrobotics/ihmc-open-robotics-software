package us.ihmc.valkyrie.kinematics.transmissions;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;

import java.util.Random;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.valkyrie.kinematics.LinearActuator;
import us.ihmc.valkyrie.kinematics.ValkyrieJointInterface;
import us.ihmc.valkyrie.kinematics.YoValkyrieJointWriter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ComparePushRodTransmissionsTest
{
   private static final boolean DEBUG = true;
   private static final boolean VISUALIZE = false;

	@Test
   public void testCompareInefficientToEfficientAnkle()
   {
      Random random = new Random(1255L);
      double epsilon = 1e-7;

      double reflectTop = 1.0;
      double reflectBottom = 1.0;
      boolean topJointFirst = true;

      PushRodTransmissionJoint pushRodTransmissionJoint = PushRodTransmissionJoint.ANKLE;
      
      YoVariableRegistry registry = new YoVariableRegistry("test");
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      
      InefficientPushRodTransmission inefficientPushrodTransmission = new InefficientPushRodTransmission(pushRodTransmissionJoint, reflectTop, reflectBottom, topJointFirst, registry, yoGraphicsListRegistry);
      EfficientPushRodTransmission efficientPushrodTransmission = new EfficientPushRodTransmission(pushRodTransmissionJoint, reflectBottom, true);

      compareTwoPushRodTransmissionForce(random, epsilon, inefficientPushrodTransmission, efficientPushrodTransmission, registry, yoGraphicsListRegistry);
   }

	@Disabled
	@Test
   public void testTiming()
   {
      Random random = new Random(1255L);
      double epsilon = 1e-7;

      double reflectTop = 1.0;
      double reflectBottom = 1.0;
      boolean topJointFirst = true;

      PushRodTransmissionJoint pushRodTransmissionJoint = PushRodTransmissionJoint.ANKLE;
      
      YoVariableRegistry registry = new YoVariableRegistry("test");
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      
      InefficientPushRodTransmission inefficientPushrodTransmission = new InefficientPushRodTransmission(pushRodTransmissionJoint, reflectTop, reflectBottom, topJointFirst, registry, yoGraphicsListRegistry);
      EfficientPushRodTransmission efficientPushrodTransmission = new EfficientPushRodTransmission(pushRodTransmissionJoint, reflectBottom, true);

      testTimingTwoPushRodTransmissionInterfaces(random, epsilon, inefficientPushrodTransmission, efficientPushrodTransmission, registry, yoGraphicsListRegistry);
   }

   @Disabled
   @Test
   public void testCompareInefficientToEfficientWaist()
   {
      Random random = new Random(1255L);
      double epsilon = 1e-3;

      double reflectTop = 1.0;
      double reflectBottom = -1.0;
      boolean topJointFirst = false;

      PushRodTransmissionJoint pushRodTransmissionJoint = PushRodTransmissionJoint.WAIST;
      
      YoVariableRegistry registry = new YoVariableRegistry("test");
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      InefficientPushRodTransmission inefficientPushrodTransmission = new InefficientPushRodTransmission(pushRodTransmissionJoint, reflectTop, reflectBottom, topJointFirst, registry, yoGraphicsListRegistry);
      EfficientPushRodTransmission efficientPushrodTransmission = new EfficientPushRodTransmission(pushRodTransmissionJoint, reflectBottom, true);

      compareTwoPushRodTransmissionForce(random, epsilon, inefficientPushrodTransmission, efficientPushrodTransmission, registry, yoGraphicsListRegistry);
   }

   @Disabled
   @Test
   public void testCompareInefficientToJSCWaist()
   {
      Random random = new Random(1255L);
      double epsilon = 4e-2;
//      double epsilon = 1.0;

      double reflectTop = 1.0;
      double reflectBottom = -1.0;
      boolean topJointFirst = false;

      PushRodTransmissionJoint pushRodTransmissionJoint = PushRodTransmissionJoint.WAIST;
      
      YoVariableRegistry registry = new YoVariableRegistry("test");
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      InefficientPushRodTransmission inefficientPushrodTransmission = new InefficientPushRodTransmission(pushRodTransmissionJoint, reflectTop, reflectBottom, topJointFirst, registry, yoGraphicsListRegistry);
      JSCWaistPushRodTransmission nasaPushrodTransmission = new JSCWaistPushRodTransmission(pushRodTransmissionJoint);

      compareTwoPushRodTransmissionForce(random, epsilon, inefficientPushrodTransmission, nasaPushrodTransmission, registry, yoGraphicsListRegistry);
   }

   @Disabled
   @Test
   public void testCompareInefficientToEfficientWaistOverRenishaw()
   {
      Random random = new Random(1255L);
      double epsilon = 1e-7;
//      double epsilon = 1.0;

      double reflectTop = 1.0;
      double reflectBottom = -1.0;
      boolean topJointFirst = false;

      PushRodTransmissionJoint pushRodTransmissionJoint = PushRodTransmissionJoint.WAIST;
      
      YoVariableRegistry registry = new YoVariableRegistry("test");
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      InefficientPushRodTransmission inefficientPushrodTransmission = new InefficientPushRodTransmission(pushRodTransmissionJoint, reflectTop, reflectBottom, topJointFirst, registry, yoGraphicsListRegistry);
      inefficientPushrodTransmission.setUseFuteks(false);
      EfficientPushRodTransmission efficientPushrodTransmission = new EfficientPushRodTransmission(pushRodTransmissionJoint, reflectBottom, false);

      compareTwoPushRodTransmissionForce(random, epsilon, inefficientPushrodTransmission, efficientPushrodTransmission, registry, yoGraphicsListRegistry);
   }

   @Disabled
   @Test
   public void testCompareEfficientToJSCWaist()
   {
      Random random = new Random(1255L);
      double epsilon = 1.0e-2;
//      double epsilon = 1.0;

      double reflectBottom = -1.0;

      PushRodTransmissionJoint pushRodTransmissionJoint = PushRodTransmissionJoint.WAIST;
      
      YoVariableRegistry registry = new YoVariableRegistry("test");
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      EfficientPushRodTransmission efficientPushrodTransmission = new EfficientPushRodTransmission(pushRodTransmissionJoint, reflectBottom, true);
      JSCWaistPushRodTransmission nasaPushrodTransmission = new JSCWaistPushRodTransmission(pushRodTransmissionJoint);

      compareTwoPushRodTransmissionVelocity(random, epsilon, efficientPushrodTransmission, nasaPushrodTransmission, registry, yoGraphicsListRegistry);
   }
   
   private void compareTwoPushRodTransmissionForce(Random random, double epsilon, PushRodTransmissionInterface pushrodTransmissionA,
           PushRodTransmissionInterface pushrodTransmissionB, YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      Robot robot = new Robot("comparePushrodTransmission");
      robot.getRobotsYoVariableRegistry().addChild(registry);

      YoDouble topJointAngle = new YoDouble("topJointAngle", registry);
      YoDouble bottomJointAngle = new YoDouble("bottomJointAngle", registry);
      
      YoDouble actuatorForceA0 = new YoDouble("actuatorForceA0", registry);
      YoDouble actuatorForceA1 = new YoDouble("actuatorForceA1", registry);
      
      YoDouble actuatorForceB0 = new YoDouble("actuatorForceB0", registry);
      YoDouble actuatorForceB1 = new YoDouble("actuatorForceB1", registry);
      
      YoDouble force0 = new YoDouble("force0", registry);
      YoDouble force1 = new YoDouble("force1", registry);
      
      YoDouble topJointTorqueA = new YoDouble("topJointTorqueA", registry);
      YoDouble bottomJointTorqueA = new YoDouble("bottomJointTorqueA", registry);
      YoDouble topJointTorqueB = new YoDouble("topJointTorqueB", registry);
      YoDouble bottomJointTorqueB = new YoDouble("bottomJointTorqueB", registry);
      
      YoDouble topJointTorque = new YoDouble("topJointTorque", registry);
      YoDouble bottomJointTorque = new YoDouble("bottomJointTorque", registry);

      SimulationConstructionSet scs;
      if (VISUALIZE)
      {
         scs = new SimulationConstructionSet(robot);
         scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
         scs.setCameraPosition(0.62, -0.4, 1.26);
         scs.setCameraFix(0.0, 0.0, 1.02);

         scs.startOnAThread();
      }
      else
      {
         scs = null;
      }
      
      LinearActuator[] actuatorData = new LinearActuator[2];
      actuatorData[0] = new LinearActuator("dummy");
      actuatorData[1] = new LinearActuator("dummy");

      ValkyrieJointInterface[] jointData = new ValkyrieJointInterface[2];
      jointData[0] = new YoValkyrieJointWriter("joint0", registry);
      jointData[1] = new YoValkyrieJointWriter("joint1", registry);

      double increment = 0.05;

      compareActuatorToJointEffort(random, epsilon, pushrodTransmissionA, pushrodTransmissionB, topJointAngle, bottomJointAngle, force0, force1,
            topJointTorqueA, bottomJointTorqueA, topJointTorqueB, bottomJointTorqueB, scs, actuatorData, jointData, increment);
      
      compareJointToActuatorEffort(random, epsilon, pushrodTransmissionA, pushrodTransmissionB, topJointAngle, bottomJointAngle, actuatorForceA0,
            actuatorForceA1, actuatorForceB0, actuatorForceB1, topJointTorque, bottomJointTorque, scs, actuatorData, jointData, increment);
      
      if (VISUALIZE)
      {
         scs.gotoInPointNow();
         scs.setIndex(1);
         scs.setInPoint();
         scs.cropBuffer();

         ThreadTools.sleepForever();
      }
   }

   private void compareJointToActuatorEffort(Random random, double epsilon, PushRodTransmissionInterface pushrodTransmissionA,
         PushRodTransmissionInterface pushrodTransmissionB, YoDouble topJointAngle, YoDouble bottomJointAngle, YoDouble actuatorForceA0,
         YoDouble actuatorForceA1, YoDouble actuatorForceB0, YoDouble actuatorForceB1, YoDouble topJointTorque,
         YoDouble bottomJointTorque, SimulationConstructionSet scs, LinearActuator[] actuatorData, ValkyrieJointInterface[] jointData, double increment)
   {
      for (double topJoint = -1.0; topJoint < 1.0; topJoint = topJoint + increment)
      {
         for (double bottomJoint = -0.5; bottomJoint < 0.5; bottomJoint = bottomJoint + increment)
         {
            printIfDebug("topJoint = " + topJoint + ", bottomJoint = " + bottomJoint);

            jointData[0].setPosition(topJoint);
            jointData[1].setPosition(bottomJoint);

            topJointAngle.set(topJoint);
            bottomJointAngle.set(bottomJoint);

            // Check the jointToActuatorEffort
            topJointTorque.set(RandomNumbers.nextDouble(random, -40.0, 40.0));
            bottomJointTorque.set(RandomNumbers.nextDouble(random, -40.0, 40.0));

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
   }

   private void compareActuatorToJointEffort(Random random, double epsilon, PushRodTransmissionInterface pushrodTransmissionA,
         PushRodTransmissionInterface pushrodTransmissionB, YoDouble topJointAngle, YoDouble bottomJointAngle, YoDouble force0,
         YoDouble force1, YoDouble topJointTorqueA, YoDouble bottomJointTorqueA, YoDouble topJointTorqueB,
         YoDouble bottomJointTorqueB, SimulationConstructionSet scs, LinearActuator[] actuatorData, ValkyrieJointInterface[] jointData,
         double increment)
   {
      for (double topJoint = -1.0; topJoint < 1.0; topJoint = topJoint + increment)
      {
         for (double bottomJoint = -0.5; bottomJoint < 0.5; bottomJoint = bottomJoint + increment)
         {
            printIfDebug("topJoint = " + topJoint + ", bottomJoint = " + bottomJoint);

            jointData[0].setPosition(topJoint);
            jointData[1].setPosition(bottomJoint);

            topJointAngle.set(topJoint);
            bottomJointAngle.set(bottomJoint);
            
            // Check the actuatorToJointEffort

            force0.set(RandomNumbers.nextDouble(random, -100.0, 100.0));
            force1.set(RandomNumbers.nextDouble(random, -100.0, 100.0));
//            force0.set(1.0);
//            force1.set(1.0);

            actuatorData[0].setEffortCommand(force0.getDoubleValue());
            actuatorData[1].setEffortCommand(force1.getDoubleValue());

            jointData[0].setEffort(Double.NaN);
            jointData[1].setEffort(Double.NaN);

            pushrodTransmissionA.actuatorToJointEffort(actuatorData, jointData);

            topJointTorqueA.set(jointData[0].getEffort());
            bottomJointTorqueA.set(jointData[1].getEffort());

            jointData[0].setEffort(Double.NaN);
            jointData[1].setEffort(Double.NaN);

            pushrodTransmissionB.actuatorToJointEffort(actuatorData, jointData);

            topJointTorqueB.set(jointData[0].getEffort());
            bottomJointTorqueB.set(jointData[1].getEffort());

            printIfDebug( force0 + ", " + force1 + ", topJointTorqueA = " + topJointTorqueA.getDoubleValue () + ", bottomJointTorqueA = " + bottomJointTorqueA.getDoubleValue()
                         + ", topJointTorqueB = " + topJointTorqueB.getDoubleValue() + ", tauRollEfficient = " + bottomJointTorqueB.getDoubleValue());

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
   }

   private void compareTwoPushRodTransmissionVelocity(Random random, double epsilon, PushRodTransmissionInterface pushrodTransmissionA,
           PushRodTransmissionInterface pushrodTransmissionB, YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      Robot robot = new Robot("comparePushrodTransmission");
      robot.getRobotsYoVariableRegistry().addChild(registry);

      YoDouble topJointAngle = new YoDouble("topJointAngle", registry);
      YoDouble bottomJointAngle = new YoDouble("bottomJointAngle", registry);
      
      YoDouble velocity0 = new YoDouble("velocity0", registry);
      YoDouble velocity1 = new YoDouble("velocity1", registry);
      
      YoDouble topJointVelocityA = new YoDouble("topJointVelocityA", registry);
      YoDouble bottomJointVelocityA = new YoDouble("bottomJointVelocityA", registry);
      YoDouble topJointVelocityB = new YoDouble("topJointVelocityB", registry);
      YoDouble bottomJointVelocityB = new YoDouble("bottomJointVelocityB", registry);
      
      SimulationConstructionSet scs;
      if (VISUALIZE)
      {
         scs = new SimulationConstructionSet(robot);
         scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
         scs.setCameraPosition(0.62, -0.4, 1.26);
         scs.setCameraFix(0.0, 0.0, 1.02);

         scs.startOnAThread();
      }
      else
      {
         scs = null;
      }
      
      LinearActuator[] actuatorData = new LinearActuator[2];
      actuatorData[0] = new LinearActuator("dummy");
      actuatorData[1] = new LinearActuator("dummy");

      ValkyrieJointInterface[] jointData = new ValkyrieJointInterface[2];
      jointData[0] = new YoValkyrieJointWriter("joint0", registry);
      jointData[1] = new YoValkyrieJointWriter("joint1", registry);

      double increment = 0.05;

      compareActuatorToJointVelocity(random, epsilon, pushrodTransmissionA, pushrodTransmissionB, topJointAngle, bottomJointAngle, velocity0, velocity1,
            topJointVelocityA, bottomJointVelocityA, topJointVelocityB, bottomJointVelocityB, scs, actuatorData, jointData, increment);
      
      if (VISUALIZE)
      {
         scs.gotoInPointNow();
         scs.setIndex(1);
         scs.setInPoint();
         scs.cropBuffer();

         ThreadTools.sleepForever();
      }
   }

   private void compareActuatorToJointVelocity(Random random, double epsilon, PushRodTransmissionInterface pushrodTransmissionA,
         PushRodTransmissionInterface pushrodTransmissionB, YoDouble topJointAngle, YoDouble bottomJointAngle, YoDouble actuatorVelocity0,
         YoDouble actuatorVelocity1, YoDouble topJointVelocityA, YoDouble bottomJointVelocityA, YoDouble topJointVelocityB,
         YoDouble bottomJointVelocityB, SimulationConstructionSet scs, LinearActuator[] actuatorData, ValkyrieJointInterface[] jointData,
         double increment)
   {
      for (double topJoint = -1.0; topJoint < 1.0; topJoint = topJoint + increment)
      {
         for (double bottomJoint = -0.5; bottomJoint < 0.5; bottomJoint = bottomJoint + increment)
         {
            printIfDebug("topJoint = " + topJoint + ", bottomJoint = " + bottomJoint);

            jointData[0].setPosition(topJoint);
            jointData[1].setPosition(bottomJoint);

            topJointAngle.set(topJoint);
            bottomJointAngle.set(bottomJoint);
            
            // Check the actuatorToJointEffort

            actuatorVelocity0.set(RandomNumbers.nextDouble(random, -10.0, 10.0));
            actuatorVelocity1.set(RandomNumbers.nextDouble(random, -10.0, 10.0));

            actuatorData[0].setVelocityCommand(actuatorVelocity0.getDoubleValue());
            actuatorData[1].setVelocityCommand(actuatorVelocity1.getDoubleValue());

            jointData[0].setVelocity(Double.NaN);
            jointData[1].setVelocity(Double.NaN);

            pushrodTransmissionA.actuatorToJointVelocity(actuatorData, jointData);

            topJointVelocityA.set(jointData[0].getVelocity());
            bottomJointVelocityA.set(jointData[1].getVelocity());

            jointData[0].setVelocity(Double.NaN);
            jointData[1].setVelocity(Double.NaN);

            pushrodTransmissionB.actuatorToJointVelocity(actuatorData, jointData);

            topJointVelocityB.set(jointData[0].getVelocity());
            bottomJointVelocityB.set(jointData[1].getVelocity());

            printIfDebug( actuatorVelocity0 + ", " + actuatorVelocity1 + ", topJointVelocityA = " + topJointVelocityA.getDoubleValue () + ", bottomJointVelocityA = " + bottomJointVelocityA.getDoubleValue()
                         + ", topJointVelocityB = " + topJointVelocityB.getDoubleValue() + ", bottomJointVelocityB = " + bottomJointVelocityB.getDoubleValue());

            assertFalse(Double.isNaN(topJointVelocityA.getDoubleValue()));
            assertFalse(Double.isNaN(bottomJointVelocityA.getDoubleValue()));

            assertEquals(topJointVelocityA.getDoubleValue(), topJointVelocityB.getDoubleValue(), epsilon);
            assertEquals(bottomJointVelocityA.getDoubleValue(), bottomJointVelocityB.getDoubleValue(), epsilon);

            if (VISUALIZE)
            {
               scs.tickAndUpdate();
            }
         }
      }
   }
   

   @SuppressWarnings("unused")
   private void testTimingTwoPushRodTransmissionInterfaces(Random random, double epsilon, PushRodTransmissionInterface pushrodTransmissionA,
         PushRodTransmissionInterface pushrodTransmissionB, YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      Robot robot = new Robot("testTimingPushrodTransmission");
      robot.getRobotsYoVariableRegistry().addChild(registry);

      YoDouble topJointAngle = new YoDouble("topJointAngle", registry);
      YoDouble bottomJointAngle = new YoDouble("bottomJointAngle", registry);

      YoDouble actuatorForceA0 = new YoDouble("actuatorForceA0", registry);
      YoDouble actuatorForceA1 = new YoDouble("actuatorForceA1", registry);

      YoDouble actuatorForceB0 = new YoDouble("actuatorForceB0", registry);
      YoDouble actuatorForceB1 = new YoDouble("actuatorForceB1", registry);

      YoDouble force0 = new YoDouble("force0", registry);
      YoDouble force1 = new YoDouble("force1", registry);

      YoDouble topJointTorqueA = new YoDouble("topJointTorqueA", registry);
      YoDouble bottomJointTorqueA = new YoDouble("bottomJointTorqueA", registry);
      YoDouble topJointTorqueB = new YoDouble("topJointTorqueB", registry);
      YoDouble bottomJointTorqueB = new YoDouble("bottomJointTorqueB", registry);

      YoDouble topJointTorque = new YoDouble("topJointTorque", registry);
      YoDouble bottomJointTorque = new YoDouble("bottomJointTorque", registry);


      LinearActuator[] actuatorData = new LinearActuator[2];
      actuatorData[0] = new LinearActuator("dummy");
      actuatorData[1] = new LinearActuator("dummy");

      ValkyrieJointInterface[] jointData = new ValkyrieJointInterface[2];
      jointData[0] = new YoValkyrieJointWriter("joint0", registry);
      jointData[1] = new YoValkyrieJointWriter("joint1", registry);

      double topJoint = 0.5;
      double bottomJoint = -0.25;

      jointData[0].setPosition(topJoint);
      jointData[1].setPosition(bottomJoint);

      topJointAngle.set(topJoint);
      bottomJointAngle.set(bottomJoint);

      force0.set(1.0);
      force1.set(1.0);

      actuatorData[0].setEffortCommand(force0.getDoubleValue());
      actuatorData[1].setEffortCommand(force1.getDoubleValue());

      jointData[0].setEffort(Double.NaN);
      jointData[1].setEffort(Double.NaN);

      int numberOfCalls = 10000000;
      
      long startTimeA = System.currentTimeMillis();
      for (int i=0; i<numberOfCalls; i++)
      {
         pushrodTransmissionA.jointToActuatorPosition(actuatorData, jointData);
         pushrodTransmissionA.actuatorToJointEffort(actuatorData, jointData);
      }
      long endTimeA = System.currentTimeMillis();

      topJointTorqueA.set(jointData[0].getEffort());
      bottomJointTorqueA.set(jointData[1].getEffort());

      jointData[0].setEffort(Double.NaN);
      jointData[1].setEffort(Double.NaN);
      
      long startTimeB = System.currentTimeMillis();
      for (int i=0; i<numberOfCalls; i++)
      {
         pushrodTransmissionB.jointToActuatorPosition(actuatorData, jointData);
         pushrodTransmissionB.actuatorToJointEffort(actuatorData, jointData);
      }
      long endTimeB = System.currentTimeMillis();

      double totalTimeA = (endTimeA - startTimeA) * 0.001;
      double totalTimeB = (endTimeB - startTimeB) * 0.001;
      
      double timePerA = totalTimeA / (double) numberOfCalls;
      double timePerB = totalTimeB / (double) numberOfCalls;
      
      System.out.println("totalTimeA = " + totalTimeA);
      System.out.println("totalTimeB = " + totalTimeB);
      
      System.out.println("timePerA = " + timePerA * 1000.0 + " milliseconds.");
      System.out.println("timePerB = " + timePerB * 1000.0 + " milliseconds.");
      
      topJointTorqueB.set(jointData[0].getEffort());
      bottomJointTorqueB.set(jointData[1].getEffort());

      assertFalse(Double.isNaN(topJointTorqueA.getDoubleValue()));
      assertFalse(Double.isNaN(bottomJointTorqueA.getDoubleValue()));

      assertEquals(topJointTorqueA.getDoubleValue(), topJointTorqueB.getDoubleValue(), epsilon);
      assertEquals(bottomJointTorqueA.getDoubleValue(), bottomJointTorqueB.getDoubleValue(), epsilon);

   }

   private void printIfDebug(String string)
   {
      if (DEBUG)
         System.out.println(string);
   }
}
