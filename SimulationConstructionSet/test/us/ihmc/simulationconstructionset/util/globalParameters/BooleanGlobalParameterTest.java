package us.ihmc.simulationconstructionset.util.globalParameters;


import static org.junit.Assert.*;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;



public class BooleanGlobalParameterTest
{
   private static final boolean VERBOSE = false;
   private final boolean DEFAULT_VALUE = true;

   @Before
   public void setUp() throws Exception
   {
      GlobalParameter.clearGlobalRegistry();
   }

   @After
   public void tearDown() throws Exception
   {
      GlobalParameter.clearGlobalRegistry();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetValue()
   {
      SystemOutGlobalParameterChangedListener systemOutGlobalParameterChangedListener = null;
      if (VERBOSE) systemOutGlobalParameterChangedListener = new SystemOutGlobalParameterChangedListener();

      BooleanGlobalParameter booleanGlobalParameter = new BooleanGlobalParameter("testParameter", "test description", DEFAULT_VALUE,
                                                         systemOutGlobalParameterChangedListener);
      assertEquals(DEFAULT_VALUE, booleanGlobalParameter.getValue());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSetValue()
   {
      SystemOutGlobalParameterChangedListener systemOutGlobalParameterChangedListener = null;
      if (VERBOSE) systemOutGlobalParameterChangedListener = new SystemOutGlobalParameterChangedListener();


      BooleanGlobalParameter booleanGlobalParameter = new BooleanGlobalParameter("testParameter", "test description", DEFAULT_VALUE,
                                                         systemOutGlobalParameterChangedListener);

      boolean newValue = false;
      booleanGlobalParameter.set(newValue);
      assertEquals(newValue, booleanGlobalParameter.getValue());

      newValue = false;
      booleanGlobalParameter.set(newValue, "setting");
      assertEquals(newValue, booleanGlobalParameter.getValue());

      newValue = true;
      booleanGlobalParameter.setOnlyIfChange(newValue, "setting");
      assertEquals(newValue, booleanGlobalParameter.getValue());

      newValue = false;
      booleanGlobalParameter.setOnlyIfChange(newValue, "setting");
      assertEquals(newValue, booleanGlobalParameter.getValue());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000,expected = RuntimeException.class)
   public void testThatCantHaveParentsUnlessOverwriteUpdateMethodOne()
   {
      BooleanGlobalParameter parent = new BooleanGlobalParameter("parent", "parent", DEFAULT_VALUE, null);
      @SuppressWarnings("unused")
      BooleanGlobalParameter invalidChild = new BooleanGlobalParameter("invalidChild", "test description", new GlobalParameter[] {parent}, null);

      parent.set(false);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000,expected = RuntimeException.class)
   public void testCantSetChild()
   {
      BooleanGlobalParameter parent = new BooleanGlobalParameter("parent", "", true, null);
      BooleanGlobalParameter child = new BooleanGlobalParameter("child", "", new GlobalParameter[] {parent}, null);

      child.set(false, "Shouldn't be able to change this!");
   }



}
