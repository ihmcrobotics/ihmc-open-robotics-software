package us.ihmc.simulationConstructionSetTools.util.globalParameters;


import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import static us.ihmc.robotics.Assert.*;

public class BooleanGlobalParameterTest
{
   private static final boolean VERBOSE = false;
   private final boolean DEFAULT_VALUE = true;

   @BeforeEach
   public void setUp() throws Exception
   {
      GlobalParameter.clearGlobalRegistry();
   }

   @AfterEach
   public void tearDown() throws Exception
   {
      GlobalParameter.clearGlobalRegistry();
   }

	@Test
   public void testGetValue()
   {
      SystemOutGlobalParameterChangedListener systemOutGlobalParameterChangedListener = null;
      if (VERBOSE) systemOutGlobalParameterChangedListener = new SystemOutGlobalParameterChangedListener();

      BooleanGlobalParameter booleanGlobalParameter = new BooleanGlobalParameter("testParameter", "test description", DEFAULT_VALUE,
                                                         systemOutGlobalParameterChangedListener);
      assertEquals(DEFAULT_VALUE, booleanGlobalParameter.getValue());
   }

	@Test
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

	@Test
   public void testThatCantHaveParentsUnlessOverwriteUpdateMethodOne()
   {
      BooleanGlobalParameter parent = new BooleanGlobalParameter("parent", "parent", DEFAULT_VALUE, null);
      @SuppressWarnings("unused")
      BooleanGlobalParameter invalidChild = new BooleanGlobalParameter("invalidChild", "test description", new GlobalParameter[] {parent}, null);

      parent.set(false);
   }

	@Test
   public void testCantSetChild()
   {
      BooleanGlobalParameter parent = new BooleanGlobalParameter("parent", "", true, null);
      BooleanGlobalParameter child = new BooleanGlobalParameter("child", "", new GlobalParameter[] {parent}, null);

      child.set(false, "Shouldn't be able to change this!");
   }



}
