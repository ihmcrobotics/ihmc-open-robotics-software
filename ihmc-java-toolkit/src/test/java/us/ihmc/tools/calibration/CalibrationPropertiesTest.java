package us.ihmc.tools.calibration;

import static us.ihmc.robotics.Assert.*;

import java.io.File;
import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;


import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
public class CalibrationPropertiesTest
{
   private String fileName = "testCalFile";
   private File currentPropertiesFile;
   private CalibrationProperties calibrationProperties;
   
   @BeforeEach
   public void setUp()
   {
      // create a calibration file with the specifies path.

      currentPropertiesFile = new File(fileName);
      calibrationProperties = new CalibrationProperties(".", fileName);
   }
   
   @AfterEach
   public void tearDown()
   {
      currentPropertiesFile.delete();
   }

	@Test
   public void testSetIntegerProperty()
   {
      Random random = new Random(1984L);
      int ITERATIONS = 100;

      String key = "abcd";

      for(int i = 0; i < ITERATIONS; i++)
      {
         int value = random.nextInt();
         calibrationProperties.setProperty(key, value);
         assertEquals(value, calibrationProperties.getIntegerProperty(key));
      }

      assertEquals(0, calibrationProperties.getIntegerProperty("notAKey"));
   }

   @Test
   public void testSetDoubleProperty()
   {
      final Random random = new Random(1984L);
      final int ITERATIONS = 100;
      final double EPSILON = 0.000001;

      String key = "abcd";

      for(int i = 0; i < ITERATIONS; i++)
      {
         double value = random.nextDouble();
         calibrationProperties.setProperty(key, value);
         assertEquals(value, calibrationProperties.getDoubleProperty(key), EPSILON);
      }

      assertEquals(0.0, calibrationProperties.getDoubleProperty("notAKey"), EPSILON);
   }

	@Test
   public void testSaveAndLoad()
   {
      String key = "abcd";
      int value = 23;
      calibrationProperties.setProperty(key, value);
      calibrationProperties.save();

      CalibrationProperties calibrationProperties2 = new CalibrationProperties(".", fileName);
      assertEquals(value, calibrationProperties2.getIntegerProperty(key));
   }

	@Test
   public void testArithmetic1()
   {
      String key = "abcd";
      int value1 = 23;
      int value2 = 56;
      String value = value1 + "+" + value2;
      calibrationProperties.setProperty(key, value);
      assertEquals(value1 + value2, calibrationProperties.getIntegerProperty(key));
   }

	@Test
   public void testArithmetic2()
   {
      String key = "abcd";
      String value = "-23 + 0-4 +8 +-5";
      calibrationProperties.setProperty(key, value);
      assertEquals(-24, calibrationProperties.getIntegerProperty(key));
   }
}
