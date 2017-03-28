package us.ihmc.simulationConstructionSetTools.util.globalParameters;

import static org.junit.Assert.*;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class MultiplicativeDoubleGlobalParameterTest
{
   private static final boolean VERBOSE = false;
   private final double eps = 1e-10;

   @Before
   public void setUp()
   {
      GlobalParameter.clearGlobalRegistry();
   }

   @After
   public void tearDown()
   {
      GlobalParameter.clearGlobalRegistry();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSetThrowsException()
   {
      SystemOutGlobalParameterChangedListener systemOutGlobalParameterChangedListener = null;
      if (VERBOSE) systemOutGlobalParameterChangedListener = new SystemOutGlobalParameterChangedListener();

      double valueA = 4.67;
      double valueB = -765.7654;

      DoubleGlobalParameter doubleGlobalParameterA = new DoubleGlobalParameter("testParameterA", "test description", valueA,
                                                        systemOutGlobalParameterChangedListener);
      DoubleGlobalParameter doubleGlobalParameterB = new DoubleGlobalParameter("testParameterB", "test description", valueB,
                                                        systemOutGlobalParameterChangedListener);

      MultiplicativeDoubleGlobalParameter multiplicativeDoubleGlobalParameter = new MultiplicativeDoubleGlobalParameter("testMulti",
                                                                                   "multiplicative parameter",
                                                                                   new DoubleGlobalParameter[] {doubleGlobalParameterA,
              doubleGlobalParameterB}, systemOutGlobalParameterChangedListener);

      try
      {
         multiplicativeDoubleGlobalParameter.set(10.0);
         fail();
      }
      catch (Exception e)
      {
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testMultiplicativeDoubleGlobalParameter()
   {
      SystemOutGlobalParameterChangedListener systemOutGlobalParameterChangedListener = null;
      if (VERBOSE) systemOutGlobalParameterChangedListener = new SystemOutGlobalParameterChangedListener();

      double valueA = 4.67;
      double valueB = -765.7654;

      DoubleGlobalParameter doubleGlobalParameterA = new DoubleGlobalParameter("testParameterA", "test description", valueA,
                                                        systemOutGlobalParameterChangedListener);
      DoubleGlobalParameter doubleGlobalParameterB = new DoubleGlobalParameter("testParameterB", "test description", valueB,
                                                        systemOutGlobalParameterChangedListener);

      MultiplicativeDoubleGlobalParameter multiplicativeDoubleGlobalParameter = new MultiplicativeDoubleGlobalParameter("testMulti",
                                                                                   "multiplicative parameter",
                                                                                   new DoubleGlobalParameter[] {doubleGlobalParameterA,
              doubleGlobalParameterB}, systemOutGlobalParameterChangedListener);


      assertEquals(valueA * valueB, multiplicativeDoubleGlobalParameter.getValue(), eps);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testMultiplicativeDoubleGlobalParameterUpdate()
   {
      SystemOutGlobalParameterChangedListener systemOutGlobalParameterChangedListener = null;
      if (VERBOSE) systemOutGlobalParameterChangedListener = new SystemOutGlobalParameterChangedListener();


      double valueA = 4.67;
      double valueB = -765.7654;

      DoubleGlobalParameter doubleGlobalParameterA = new DoubleGlobalParameter("testParameterA", "test description", valueA,
                                                        systemOutGlobalParameterChangedListener);
      DoubleGlobalParameter doubleGlobalParameterB = new DoubleGlobalParameter("testParameterB", "test description", valueB,
                                                        systemOutGlobalParameterChangedListener);

      MultiplicativeDoubleGlobalParameter multiplicativeDoubleGlobalParameter = new MultiplicativeDoubleGlobalParameter("testMulti",
                                                                                   "multiplicative parameter",
                                                                                   new DoubleGlobalParameter[] {doubleGlobalParameterA,
              doubleGlobalParameterB}, systemOutGlobalParameterChangedListener);


      valueA = -795.09;
      doubleGlobalParameterA.set(valueA);
      assertEquals(valueA * valueB, multiplicativeDoubleGlobalParameter.getValue(), eps);

      valueB = 0.58674;
      doubleGlobalParameterB.set(valueB);
      assertEquals(valueA * valueB, multiplicativeDoubleGlobalParameter.getValue(), eps);

      valueA = 0.0;
      valueB = 345675.866;
      doubleGlobalParameterA.set(valueA);
      doubleGlobalParameterB.set(valueB);
      assertEquals(valueA * valueB, multiplicativeDoubleGlobalParameter.getValue(), eps);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testFamilyTree()
   {
//    SystemOutGlobalParameterChangedListener systemOutGlobalParameterChangedListener = new SystemOutGlobalParameterChangedListener();
      SystemOutGlobalParameterChangedListener systemOutGlobalParameterChangedListener = null;    // new SystemOutGlobalParameterChangedListener();


      double valueA = 4.67;
      double valueB = -765.7654;

      DoubleGlobalParameter grandParentA = new DoubleGlobalParameter("grandParentA", "test descriptionA", valueA, systemOutGlobalParameterChangedListener);
      DoubleGlobalParameter grandParentB = new DoubleGlobalParameter("grandParentB", "test descriptionB", valueB, systemOutGlobalParameterChangedListener);

      MultiplicativeDoubleGlobalParameter parentA = new MultiplicativeDoubleGlobalParameter("parentA", "multiplicative parameter",
                                                       new DoubleGlobalParameter[] {grandParentA}, systemOutGlobalParameterChangedListener);

      MultiplicativeDoubleGlobalParameter parentB = new MultiplicativeDoubleGlobalParameter("parentB", "multiplicative parameter",
                                                       new DoubleGlobalParameter[] {grandParentB}, systemOutGlobalParameterChangedListener);

      MultiplicativeDoubleGlobalParameter childA = new MultiplicativeDoubleGlobalParameter("childA", "multiplicative parameter",
                                                      new DoubleGlobalParameter[] {grandParentA,
              parentA}, systemOutGlobalParameterChangedListener);

      MultiplicativeDoubleGlobalParameter childB = new MultiplicativeDoubleGlobalParameter("childB", "multiplicative parameter",
                                                      new DoubleGlobalParameter[] {grandParentA,
              grandParentB, parentA, parentB}, systemOutGlobalParameterChangedListener);


      double expectedParentA = valueA;
      assertEquals(expectedParentA, parentA.getValue(), eps);

      double expectedParentB = valueB;
      assertEquals(expectedParentB, parentB.getValue(), eps);

      double expectedChildA = valueA * valueA;
      assertEquals(expectedChildA, childA.getValue(), eps);

      double expectedChildB = valueA * valueB * valueA * valueB;
      assertEquals(expectedChildB, childB.getValue(), eps);

      valueA = -67.835;
      grandParentA.set(valueA);
      expectedParentA = valueA;
      assertEquals(expectedParentA, parentA.getValue(), eps);

      expectedParentB = valueB;
      assertEquals(expectedParentB, parentB.getValue(), eps);

      expectedChildA = valueA * valueA;
      assertEquals(expectedChildA, childA.getValue(), eps);

      expectedChildB = valueA * valueB * valueA * valueB;
      assertEquals(expectedChildB, childB.getValue(), eps);

      valueA = -67.835;
      valueB = 96485.835;
      grandParentA.set(valueA);
      grandParentB.set(valueB);

      expectedParentA = valueA;
      assertEquals(expectedParentA, parentA.getValue(), eps);

      expectedParentB = valueB;
      assertEquals(expectedParentB, parentB.getValue(), eps);

      expectedChildA = valueA * valueA;
      assertEquals(expectedChildA, childA.getValue(), eps);

      expectedChildB = valueA * valueB * valueA * valueB;
      assertEquals(expectedChildB, childB.getValue(), eps);
   }
}
