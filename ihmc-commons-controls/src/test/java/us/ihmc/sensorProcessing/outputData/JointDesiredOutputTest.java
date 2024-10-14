package us.ihmc.sensorProcessing.outputData;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Method;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.apache.commons.lang3.StringUtils;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;

public class JointDesiredOutputTest
{
   private static final int ITERATIONS = 1000;
   private static List<String> doubleFieldNames = getDoubleFieldNames(true);

   private static List<String> getDoubleFieldNames(boolean capitalize)
   {
      Stream<String> nameStream = Stream.of(JointDesiredOutputReadOnly.class.getDeclaredMethods())
                                        .filter(m -> m.getReturnType() == double.class && m.getParameterCount() == 0 && m.getName().startsWith("get"))
                                        .map(m -> m.getName().substring("get".length()));
      if (capitalize)
      {
         nameStream = nameStream.map(n -> StringUtils.capitalize(n));
      }
      else
      {
         nameStream = nameStream.map(n -> StringUtils.uncapitalize(n));
      }

      return nameStream.collect(Collectors.toList());
   }

   @Test
   public void testConstructorsAndBasicAPI() throws Exception
   {
      JointDesiredOutput jointDesiredOutput = new JointDesiredOutput();
      assertFalse(jointDesiredOutput.hasControlMode());
      assertFalse(jointDesiredOutput.peekResetIntegratorsRequest());
      assertFalse(jointDesiredOutput.pollResetIntegratorsRequest());
      for (String doubleFieldName : doubleFieldNames)
      {
         Method has = JointDesiredOutput.class.getMethod("has" + doubleFieldName);
         Method get = JointDesiredOutput.class.getMethod("get" + doubleFieldName);
         assertFalse((boolean) has.invoke(jointDesiredOutput));
         assertEquals(Double.NaN, (double) get.invoke(jointDesiredOutput));
      }

      Random random = new Random(7853);
      JointDesiredControlMode controlMode = EuclidCoreRandomTools.nextElementIn(random, JointDesiredControlMode.values);
      jointDesiredOutput.setControlMode(controlMode);
      assertTrue(jointDesiredOutput.hasControlMode());
      assertEquals(controlMode, jointDesiredOutput.getControlMode());

      jointDesiredOutput.setResetIntegrators(true);
      assertTrue(jointDesiredOutput.peekResetIntegratorsRequest());
      assertTrue(jointDesiredOutput.pollResetIntegratorsRequest());
      assertFalse(jointDesiredOutput.pollResetIntegratorsRequest());
      jointDesiredOutput.setResetIntegrators(true);

      for (String doubleFieldName : doubleFieldNames)
      {
         double input = EuclidCoreRandomTools.nextDouble(random, 10.0);
         Method has = JointDesiredOutput.class.getMethod("has" + doubleFieldName);
         Method get = JointDesiredOutput.class.getMethod("get" + doubleFieldName);
         Method set = JointDesiredOutput.class.getMethod("set" + doubleFieldName, double.class);

         assertFalse((boolean) has.invoke(jointDesiredOutput));
         set.invoke(jointDesiredOutput, input);
         assertTrue((boolean) has.invoke(jointDesiredOutput));
         assertEquals(input, (double) get.invoke(jointDesiredOutput));
      }

      jointDesiredOutput.clear();
      assertFalse(jointDesiredOutput.hasControlMode());
      assertFalse(jointDesiredOutput.peekResetIntegratorsRequest());
      assertFalse(jointDesiredOutput.pollResetIntegratorsRequest());
      for (String doubleFieldName : doubleFieldNames)
      {
         Method has = JointDesiredOutput.class.getMethod("has" + doubleFieldName);
         Method get = JointDesiredOutput.class.getMethod("get" + doubleFieldName);
         assertFalse((boolean) has.invoke(jointDesiredOutput));
         assertEquals(Double.NaN, (double) get.invoke(jointDesiredOutput));
      }
   }

   @Test
   public void testSet() throws Exception
   {
      Random random = new Random(3564564);

      for (int i = 0; i < ITERATIONS; i++)
      {
         JointDesiredOutput output1 = nextJointDesiredOutput(random);
         JointDesiredOutput output2 = nextJointDesiredOutput(random);

         assertNotEquals(output1, output2);
         output1.set(output2);
         assertEquals(output1, output2);

         assertEquals(output1.getControlMode(), output2.getControlMode());
         assertEquals(output1.peekResetIntegratorsRequest(), output2.peekResetIntegratorsRequest());

         for (String doubleFieldName : doubleFieldNames)
         {
            Method get = JointDesiredOutput.class.getMethod("get" + doubleFieldName);
            assertEquals(get.invoke(output1), get.invoke(output2));
         }
      }
   }

   @Test
   public void testEquals() throws Exception
   {
      Random random = new Random(4534657);

      for (int i = 0; i < ITERATIONS; i++)
      {
         JointDesiredOutput output1 = nextJointDesiredOutput(random);
         JointDesiredOutput output2 = new JointDesiredOutput();

         output2.set(output1);
         assertEquals(output1, output2);
         while (output2.getControlMode() == output1.getControlMode())
            output2.setControlMode(EuclidCoreRandomTools.nextElementIn(random, JointDesiredControlMode.values));
         assertNotEquals(output1, output2);

         output2.set(output1);
         assertEquals(output1, output2);
         output2.setResetIntegrators(!output2.peekResetIntegratorsRequest());
         assertNotEquals(output1, output2);

         for (String doubleFieldName : doubleFieldNames)
         {
            Method set = JointDesiredOutput.class.getMethod("set" + doubleFieldName, double.class);

            output2.set(output1);
            assertEquals(output1, output2);
            set.invoke(output2, EuclidCoreRandomTools.nextDouble(random, 10.0));
            assertNotEquals(output1, output2);
         }
      }
   }

   @Test
   public void testCompleteWith() throws Exception
   {
      Random random = new Random(3242362);

      JointDesiredOutput destination = new JointDesiredOutput();
      JointDesiredOutput source = nextJointDesiredOutput(random);

      destination.completeWith(source);
      assertEquals(source, destination);

      for (int i = 0; i < ITERATIONS; i++)
      {
         { // If destination is fully defined it does not change
            destination.clear();
            JointDesiredOutput destinationOriginal = nextJointDesiredOutput(random);
            destination.set(destinationOriginal);
            destination.completeWith(source);
            assertNotEquals(source, destination);
            assertEquals(destinationOriginal, destination);
         }

         // Testing the fields do not get overridden one by one
         {
            destination.clear();
            JointDesiredControlMode controlMode = EuclidCoreRandomTools.nextElementIn(random, JointDesiredControlMode.values);
            destination.setControlMode(controlMode);
            destination.completeWith(source);
            assertEquals(controlMode, destination.getControlMode());
            destination.setControlMode(source.getControlMode());
            assertEquals(source, destination);
         }

         {
            destination.clear();
            destination.setResetIntegrators(true);
            destination.completeWith(source);
            assertTrue(destination.peekResetIntegratorsRequest());
            destination.setResetIntegrators(source.peekResetIntegratorsRequest());
            assertEquals(source, destination);
         }

         for (String doubleFieldName : doubleFieldNames)
         {
            Method get = JointDesiredOutput.class.getMethod("get" + doubleFieldName);
            Method set = JointDesiredOutput.class.getMethod("set" + doubleFieldName, double.class);

            destination.clear();
            double input = EuclidCoreRandomTools.nextDouble(random, 10.0);
            set.invoke(destination, input);
            destination.completeWith(source);
            assertEquals(input, get.invoke(destination));
            set.invoke(destination, get.invoke(source));
            assertEquals(source, destination);
         }
      }
   }

   @Test
   public void testClampedDesiredPosition()
   {
      Random random = new Random(456474);
      JointDesiredOutput jointDesiredOutput = new JointDesiredOutput();

      for (int i = 0; i < ITERATIONS; i++)
      {
         double q = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double errMax = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         double q_d_min = q - errMax;
         double q_d_max = q + errMax;
         jointDesiredOutput.setPositionFeedbackMaxError(errMax);

         double q_d = EuclidCoreRandomTools.nextDouble(random, q_d_min, q_d_max);
         jointDesiredOutput.setDesiredPosition(q_d);
         assertEquals(q_d, jointDesiredOutput.getClampedDesiredPosition(q));

         q_d = q_d_max + EuclidCoreRandomTools.nextDouble(random, 0, 10);
         jointDesiredOutput.setDesiredPosition(q_d);
         assertEquals(q_d_max, jointDesiredOutput.getClampedDesiredPosition(q));

         q_d = q_d_min - EuclidCoreRandomTools.nextDouble(random, 0, 10);
         jointDesiredOutput.setDesiredPosition(q_d);
         assertEquals(q_d_min, jointDesiredOutput.getClampedDesiredPosition(q));
      }
   }

   @Test
   public void testClampedDesiredVelocity()
   {
      Random random = new Random(456474);
      JointDesiredOutput jointDesiredOutput = new JointDesiredOutput();

      for (int i = 0; i < ITERATIONS; i++)
      {
         double qd = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double errMax = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         double qd_d_min = qd - errMax;
         double qd_d_max = qd + errMax;
         jointDesiredOutput.setVelocityFeedbackMaxError(errMax);

         double qd_d = EuclidCoreRandomTools.nextDouble(random, qd_d_min, qd_d_max);
         jointDesiredOutput.setDesiredVelocity(qd_d);
         assertEquals(qd_d, jointDesiredOutput.getClampedDesiredVelocity(qd));

         qd_d = qd_d_max + EuclidCoreRandomTools.nextDouble(random, 0, 10);
         jointDesiredOutput.setDesiredVelocity(qd_d);
         assertEquals(qd_d_max, jointDesiredOutput.getClampedDesiredVelocity(qd));

         qd_d = qd_d_min - EuclidCoreRandomTools.nextDouble(random, 0, 10);
         jointDesiredOutput.setDesiredVelocity(qd_d);
         assertEquals(qd_d_min, jointDesiredOutput.getClampedDesiredVelocity(qd));
      }
   }

   private static JointDesiredOutput nextJointDesiredOutput(Random random) throws Exception
   {
      JointDesiredOutput next = new JointDesiredOutput();
      next.setControlMode(EuclidCoreRandomTools.nextElementIn(random, JointDesiredControlMode.values));
      next.setResetIntegrators(random.nextBoolean());

      for (String doubleFieldName : doubleFieldNames)
      {
         double input = EuclidCoreRandomTools.nextDouble(random, 10.0);
         Method set = JointDesiredOutput.class.getMethod("set" + doubleFieldName, double.class);
         set.invoke(next, input);
      }

      return next;
   }
}
