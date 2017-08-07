package us.ihmc.robotics.functionApproximation;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class LinearMappingTest
{
   private static final boolean VERBOSE = false;

   private LinearMapping linearMappingOneD = null;
   private LinearMapping linearMappingTwoD = null;

   @Before
   public void setUp() throws Exception
   {
      ArrayList<double[]> inputDimensions = new ArrayList<double[]>();
      ArrayList<double[]> outputDimensions = new ArrayList<double[]>();

      inputDimensions.add(new double[] { 0.0, 1.0 });
      outputDimensions.add(new double[] { 1.0, 10.0 });

      linearMappingOneD = new LinearMapping(inputDimensions, outputDimensions);

      inputDimensions.add(new double[] { -1.0, 1.0 });
      outputDimensions.add(new double[] { 10.0, -5.0 });

      linearMappingTwoD = new LinearMapping(inputDimensions, outputDimensions);
   }

   @After
   public void tearDown() throws Exception
   {
      linearMappingOneD = null;
      linearMappingTwoD = null;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMapFromInputSpaceToOutputSpace()
   {
      double[] input = new double[] { 1.0 / 9.0 };
      double[] expectedReturn = new double[] { 2.0 };
      double[] actualReturn = linearMappingOneD.mapFromInputSpaceToOutputSpace(input);
      assertEquals("oneD return value", expectedReturn[0], actualReturn[0], 1e-10);

      input = new double[] { 1.0 / 9.0, 0.0 };
      expectedReturn = new double[] { 2.0, 2.5 };
      actualReturn = linearMappingTwoD.mapFromInputSpaceToOutputSpace(input);
      assertEquals("twoD return value", expectedReturn[0], actualReturn[0], 1e-10);
      assertEquals("twoD return value", expectedReturn[1], actualReturn[1], 1e-10);

      // test out of bounds... should be fine
      input = new double[] { -1.0, 10.0 };
      expectedReturn = new double[] { -8.0, (2.5 - 75.0) };
      actualReturn = linearMappingTwoD.mapFromInputSpaceToOutputSpace(input);
      assertEquals("twoD return value", expectedReturn[0], actualReturn[0], 1e-10);
      assertEquals("twoD return value", expectedReturn[1], actualReturn[1], 1e-10);

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMapFromOutputSpaceToInputSpace()
   {
      double[] input = new double[] { 1.0 / 9.0 };
      double[] expectedReturn = new double[] { 2.0 };
      double[] actualReturn = linearMappingOneD.mapFromOutputSpaceToInputSpace(expectedReturn);

      if (VERBOSE)
      {
         System.err.println("TestLinearMapping::testMapFromOutputSpaceToInputSpace: actualReturn[0] : " + actualReturn[0]);
      }

      assertEquals("oneD return value", input[0], actualReturn[0], 1e-10);

      input = new double[] { 1.0 / 9.0, 0.0 };
      expectedReturn = new double[] { 2.0, 2.5 };
      actualReturn = linearMappingTwoD.mapFromOutputSpaceToInputSpace(expectedReturn);
      assertEquals("twoD return value", input[0], actualReturn[0], 1e-10);
      assertEquals("twoD return value", input[1], actualReturn[1], 1e-10);

      // test out of bounds... should be fine
      input = new double[] { -1.0, 10.0 };
      expectedReturn = new double[] { -8.0, (2.5 - 75.0) };
      actualReturn = linearMappingTwoD.mapFromOutputSpaceToInputSpace(expectedReturn);
      assertEquals("twoD return value", input[0], actualReturn[0], 1e-10);
      assertEquals("twoD return value", input[1], actualReturn[1], 1e-10);
   }
}
