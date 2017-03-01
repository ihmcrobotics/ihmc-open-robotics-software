package us.ihmc.robotics.linearDynamicSystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class BodeUnitsConverterTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testMagnitudeToDecibels()
   {
      double epsilon = 1e-10;
      assertEquals(20.0, BodeUnitsConverter.convertMagnitudeToDecibels(10.0), epsilon);
      assertEquals(40.0, BodeUnitsConverter.convertMagnitudeToDecibels(100.0), epsilon);
      assertEquals(28.691378080683975, BodeUnitsConverter.convertMagnitudeToDecibels(27.2), epsilon);
      
      double[] magnitudes = new double[]{10.0, 100.0, 27.2};
      double[] decibels = BodeUnitsConverter.convertMagnitudeToDecibels(magnitudes);
      
      assertEquals(20.0, decibels[0], epsilon);
      assertEquals(40.0, decibels[1], epsilon);
      assertEquals(28.691378080683975, decibels[2], epsilon);
      
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testNaN()
   {
      double magnitude = -1.0;
      double decibels = BodeUnitsConverter.convertMagnitudeToDecibels(magnitude);
      assertTrue(Double.isNaN(decibels)); 
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testNegativeInfinity()
   {
      double magnitude = 0.0;
      double decibels = BodeUnitsConverter.convertMagnitudeToDecibels(magnitude);
      assertTrue(Double.isInfinite(decibels));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testRadiansToDegrees()
   {      
      double[] phaseInRadians = new double[]{0.0, Math.PI/4.0, Math.PI, Math.PI*2.0, Math.PI*4.0};
      double[] phaseInDegrees = BodeUnitsConverter.convertRadianToDegrees(phaseInRadians);
      
      double epsilon = 1e-10;
      assertEquals(0.0, phaseInDegrees[0], epsilon);
      assertEquals(45.0, phaseInDegrees[1], epsilon);
      assertEquals(180.0, phaseInDegrees[2], epsilon);
      assertEquals(360.0, phaseInDegrees[3], epsilon);
      assertEquals(720.0, phaseInDegrees[4], epsilon);
      
      phaseInRadians = new double[]{-0.0, -Math.PI/4.0, -Math.PI, -Math.PI*2.0, -Math.PI*4.0};
      phaseInDegrees = BodeUnitsConverter.convertRadianToDegrees(phaseInRadians);
      
      assertEquals(-0.0, phaseInDegrees[0], epsilon);
      assertEquals(-45.0, phaseInDegrees[1], epsilon);
      assertEquals(-180.0, phaseInDegrees[2], epsilon);
      assertEquals(-360.0, phaseInDegrees[3], epsilon);
      assertEquals(-720.0, phaseInDegrees[4], epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
  public void testRadiansPerSecondToHz()
  {
     double[] freqInRadPerSecond = new double[]{0.0, Math.PI/4.0, Math.PI, Math.PI*2.0, Math.PI*4.0};
     double[] freqInHz = BodeUnitsConverter.convertRadPerSecondToHz(freqInRadPerSecond);
     
     double epsilon = 1e-10;
     assertEquals(0.0, freqInHz[0], epsilon);
     assertEquals(0.125, freqInHz[1], epsilon);
     assertEquals(0.5, freqInHz[2], epsilon);
     assertEquals(1.0, freqInHz[3], epsilon);
     assertEquals(2.0, freqInHz[4], epsilon);
     
     freqInRadPerSecond = new double[]{-0.0, -Math.PI/4.0, -Math.PI, -Math.PI*2.0, -Math.PI*4.0};
     freqInHz = BodeUnitsConverter.convertRadPerSecondToHz(freqInRadPerSecond);
     
     assertEquals(-0.0, freqInHz[0], epsilon);
     assertEquals(-0.125, freqInHz[1], epsilon);
     assertEquals(-0.5, freqInHz[2], epsilon);
     assertEquals(-1.0, freqInHz[3], epsilon);
     assertEquals(-2.0, freqInHz[4], epsilon);
  }

}
