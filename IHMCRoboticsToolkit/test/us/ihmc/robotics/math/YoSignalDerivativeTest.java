package us.ihmc.robotics.math;

import static org.junit.Assert.assertEquals;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.math.YoSignalDerivative.DifferentiationMode;


public class YoSignalDerivativeTest
{
   private static double epsilon = 1e-10;
   private YoSignalDerivative yoSignalDerivative;

   @Before
   public void setUp() throws Exception
   {
      YoVariableRegistry registry = new YoVariableRegistry("testRegistry");
      yoSignalDerivative = new YoSignalDerivative("test", registry);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void checkConstructor()
   {
      String name = yoSignalDerivative.getName();
      assertEquals("test", name);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void checkSetDifferentiationMode()
   {
      yoSignalDerivative.setDifferentiationMode(DifferentiationMode.ON_SIGNAL_CHANGE);
      DifferentiationMode differentiationMode = yoSignalDerivative.getDifferentiationMode();
      assertEquals(DifferentiationMode.ON_SIGNAL_CHANGE, differentiationMode);
      
      yoSignalDerivative.setDifferentiationMode(DifferentiationMode.USING_DT);
      DifferentiationMode differentiationMode2 = yoSignalDerivative.getDifferentiationMode();
      assertEquals(DifferentiationMode.USING_DT, differentiationMode2);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void checkDTMode()
   {
      yoSignalDerivative.initialize(DifferentiationMode.USING_DT, 0.5, 0.0, 0.0);
      DifferentiationMode differentiationMode = yoSignalDerivative.getDifferentiationMode();
      assertEquals(DifferentiationMode.USING_DT, differentiationMode);
      
      double derivative = yoSignalDerivative.getDerivative(1, 0.5);
      assertEquals(1, derivative, epsilon);
      
      double derivative2 = yoSignalDerivative.getDerivative(0.5, 1);
      assertEquals(-1, derivative2, epsilon);
      
      double derivative3 = yoSignalDerivative.getDerivative(4, 2);
      assertEquals(3.5, derivative3, epsilon);
      
      double derivative4 = yoSignalDerivative.getDerivative(4, 3);
      assertEquals(0.0, derivative4, epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void checkSignalChangeModeWithDefaultTolerance()
   {
      yoSignalDerivative.initialize(DifferentiationMode.ON_SIGNAL_CHANGE, 0.0, 0.1, 4.0);
      DifferentiationMode differentiationMode = yoSignalDerivative.getDifferentiationMode();
      assertEquals(DifferentiationMode.ON_SIGNAL_CHANGE, differentiationMode);
      
      double derivative = yoSignalDerivative.getDerivative(0.0, 0.25);
      assertEquals(4.0, derivative, epsilon);
      
      double derivative2 = yoSignalDerivative.getDerivative(1, 0.5);
      assertEquals(2.5, derivative2, epsilon);
      
      double derivative3 = yoSignalDerivative.getDerivative(0.5, 1);
      assertEquals(-1, derivative3, epsilon);
      
      double derivative4 = yoSignalDerivative.getDerivative(4, 2);
      assertEquals(3.5, derivative4, epsilon);
      
      double derivative5 = yoSignalDerivative.getDerivative(4, 3);
      assertEquals(3.5, derivative5, epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void checkSignalChangeModeWithUserTolerance()
   {
      yoSignalDerivative.initialize(DifferentiationMode.ON_SIGNAL_CHANGE, 0.1, 0.0, 0.0, 0.0);
      DifferentiationMode differentiationMode = yoSignalDerivative.getDifferentiationMode();
      assertEquals(DifferentiationMode.ON_SIGNAL_CHANGE, differentiationMode);
      
      double derivative = yoSignalDerivative.getDerivative(1, 0.5);
      assertEquals(2, derivative, epsilon);
      
      double derivative2 = yoSignalDerivative.getDerivative(0.5, 1);
      assertEquals(-1, derivative2, epsilon);
      
      double derivative3 = yoSignalDerivative.getDerivative(4, 2);
      assertEquals(3.5, derivative3, epsilon);
      
      double derivative4 = yoSignalDerivative.getDerivative(4, 3);
      assertEquals(3.5, derivative4, epsilon);
      
      double derivative5 = yoSignalDerivative.getDerivative(4.09, 4);
      assertEquals(3.5, derivative5, epsilon);
      
      double derivative6 = yoSignalDerivative.getDerivative(4.108, 5);
      assertEquals(0.036, derivative6, epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void checkResetToZero()
   {
      yoSignalDerivative.initialize(DifferentiationMode.ON_SIGNAL_CHANGE, 1.0, 0.5, 3.0);
      yoSignalDerivative.resetToZero();
      
      double derivative = yoSignalDerivative.getDerivative(1.5, 1);
      assertEquals(1.5, derivative, epsilon);
   }
}
