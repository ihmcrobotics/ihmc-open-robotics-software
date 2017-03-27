package us.ihmc.simulationConstructionSetTools.util.globalParameters;


import static org.junit.Assert.*;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class AndBooleanGlobalParameterTest
{
   private static final boolean VERBOSE = false;
   
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

      boolean valueA = true;
      boolean valueB = false;

      BooleanGlobalParameter booleanGlobalParameterA = new BooleanGlobalParameter("testParameterA", "test description", valueA,
                                                          systemOutGlobalParameterChangedListener);
      BooleanGlobalParameter booleanGlobalParameterB = new BooleanGlobalParameter("testParameterB", "test description", valueB,
                                                          systemOutGlobalParameterChangedListener);

      AndBooleanGlobalParameter multiplicativeDoubleGlobalParameter = new AndBooleanGlobalParameter("testMulti", "multiplicative parameter",
                                                                         new BooleanGlobalParameter[] {booleanGlobalParameterA,
              booleanGlobalParameterB}, systemOutGlobalParameterChangedListener);

      try
      {
         multiplicativeDoubleGlobalParameter.set(false);
         fail();
      }
      catch (Exception e)
      {
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testAndBooleanGlobalParameter()
   {
      SystemOutGlobalParameterChangedListener systemOutGlobalParameterChangedListener = null;
      if (VERBOSE) systemOutGlobalParameterChangedListener = new SystemOutGlobalParameterChangedListener();

      boolean valueA = true;
      boolean valueB = false;

      BooleanGlobalParameter booleanGlobalParameterA = new BooleanGlobalParameter("testParameterA", "test description", valueA,
                                                          systemOutGlobalParameterChangedListener);
      BooleanGlobalParameter booleanGlobalParameterB = new BooleanGlobalParameter("testParameterB", "test description", valueB,
                                                          systemOutGlobalParameterChangedListener);

      AndBooleanGlobalParameter multiplicativeDoubleGlobalParameter = new AndBooleanGlobalParameter("testMulti", "multiplicative parameter",
                                                                         new BooleanGlobalParameter[] {booleanGlobalParameterA,
              booleanGlobalParameterB}, systemOutGlobalParameterChangedListener);


      assertEquals(valueA && valueB, multiplicativeDoubleGlobalParameter.getValue());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testAndBooleanGlobalParameterUpdate()
   {
      SystemOutGlobalParameterChangedListener systemOutGlobalParameterChangedListener = null;
      if (VERBOSE) systemOutGlobalParameterChangedListener = new SystemOutGlobalParameterChangedListener();


      boolean valueA = true;
      boolean valueB = false;

      BooleanGlobalParameter booleanGlobalParameterA = new BooleanGlobalParameter("testParameterA", "test description", valueA,
                                                          systemOutGlobalParameterChangedListener);
      BooleanGlobalParameter booleanGlobalParameterB = new BooleanGlobalParameter("testParameterB", "test description", valueB,
                                                          systemOutGlobalParameterChangedListener);

      AndBooleanGlobalParameter multiplicativeDoubleGlobalParameter = new AndBooleanGlobalParameter("testMulti", "multiplicative parameter",
                                                                         new BooleanGlobalParameter[] {booleanGlobalParameterA,
              booleanGlobalParameterB}, systemOutGlobalParameterChangedListener);


      valueA = false;
      booleanGlobalParameterA.set(valueA);
      assertEquals(valueA && valueB, multiplicativeDoubleGlobalParameter.getValue());

      valueB = true;
      booleanGlobalParameterB.set(valueB);
      assertEquals(valueA && valueB, multiplicativeDoubleGlobalParameter.getValue());

      valueA = true;
      valueB = true;
      booleanGlobalParameterA.set(valueA);
      booleanGlobalParameterB.set(valueB);
      assertEquals(valueA && valueB, multiplicativeDoubleGlobalParameter.getValue());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testFamilyTree()
   {
//    SystemOutGlobalParameterChangedListener systemOutGlobalParameterChangedListener = new SystemOutGlobalParameterChangedListener();
      SystemOutGlobalParameterChangedListener systemOutGlobalParameterChangedListener = null;    // new SystemOutGlobalParameterChangedListener();


      boolean valueA = false;
      boolean valueB = true;

      BooleanGlobalParameter grandParentA = new BooleanGlobalParameter("grandParentA", "test descriptionA", valueA, systemOutGlobalParameterChangedListener);
      BooleanGlobalParameter grandParentB = new BooleanGlobalParameter("grandParentB", "test descriptionB", valueB, systemOutGlobalParameterChangedListener);

      AndBooleanGlobalParameter parentA = new AndBooleanGlobalParameter("parentA", "multiplicative parameter", new BooleanGlobalParameter[] {grandParentA},
                                             systemOutGlobalParameterChangedListener);

      AndBooleanGlobalParameter parentB = new AndBooleanGlobalParameter("parentB", "multiplicative parameter", new BooleanGlobalParameter[] {grandParentB},
                                             systemOutGlobalParameterChangedListener);

      AndBooleanGlobalParameter childA = new AndBooleanGlobalParameter("childA", "multiplicative parameter", new BooleanGlobalParameter[] {grandParentA,
              parentA}, systemOutGlobalParameterChangedListener);

      AndBooleanGlobalParameter childB = new AndBooleanGlobalParameter("childB", "multiplicative parameter", new BooleanGlobalParameter[] {grandParentA,
              grandParentB, parentA, parentB}, systemOutGlobalParameterChangedListener);


      boolean expectedParentA = valueA;
      assertEquals(expectedParentA, parentA.getValue());

      boolean expectedParentB = valueB;
      assertEquals(expectedParentB, parentB.getValue());

      boolean expectedChildA = false;
      assertEquals(expectedChildA, childA.getValue());

      boolean expectedChildB = valueA && valueB && valueA && valueB;
      assertEquals(expectedChildB, childB.getValue());

      valueA = !valueA;
      grandParentA.set(valueA);
      expectedParentA = valueA;
      assertEquals(expectedParentA, parentA.getValue());

      expectedParentB = valueB;
      assertEquals(expectedParentB, parentB.getValue());

      expectedChildA = true;
      assertEquals(expectedChildA, childA.getValue());

      expectedChildB = valueA && valueB && valueA && valueB;
      assertEquals(expectedChildB, childB.getValue());

      valueA = !valueA;
      valueB = !valueB;
      grandParentA.set(valueA);
      grandParentB.set(valueB);

      expectedParentA = valueA;
      assertEquals(expectedParentA, parentA.getValue());

      expectedParentB = valueB;
      assertEquals(expectedParentB, parentB.getValue());

      expectedChildA = false;
      assertEquals(expectedChildA, childA.getValue());

      expectedChildB = valueA && valueB && valueA && valueB;
      assertEquals(expectedChildB, childB.getValue());
   }


}
