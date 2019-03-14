package us.ihmc.simulationConstructionSetTools.util.globalParameters;

import org.junit.jupiter.api.*;

import static us.ihmc.robotics.Assert.*;

public class DoubleGlobalParameterTest
{
   private static final boolean VERBOSE = false;
   private final double DEFAULT_VALUE = 11.99;
   private final double eps = 1e-10;

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

      DoubleGlobalParameter doubleGlobalParameter = new DoubleGlobalParameter("testParameter", "test description", DEFAULT_VALUE,
                                                       systemOutGlobalParameterChangedListener);
      assertEquals(DEFAULT_VALUE, doubleGlobalParameter.getValue(), eps);
   }

	@Test
   public void testSetValue()
   {
      SystemOutGlobalParameterChangedListener systemOutGlobalParameterChangedListener = null;
      if (VERBOSE) systemOutGlobalParameterChangedListener = new SystemOutGlobalParameterChangedListener();


      DoubleGlobalParameter doubleGlobalParameter = new DoubleGlobalParameter("testParameter", "test description", DEFAULT_VALUE,
                                                       systemOutGlobalParameterChangedListener);

      double newValue = -0.045;
      doubleGlobalParameter.set(newValue);
      assertEquals(newValue, doubleGlobalParameter.getValue(), eps);

      newValue = 1100.345;
      doubleGlobalParameter.set(newValue, "setting");
      assertEquals(newValue, doubleGlobalParameter.getValue(), eps);

      newValue = 1100.345;
      doubleGlobalParameter.setOnlyIfChange(newValue, "setting");
      assertEquals(newValue, doubleGlobalParameter.getValue(), eps);

      newValue = -906.345;
      doubleGlobalParameter.setOnlyIfChange(newValue, "setting");
      assertEquals(newValue, doubleGlobalParameter.getValue(), eps);
   }

	@Test
   public void testThatCantHaveParentsUnlessOverwriteUpdateMethodOne()
   {
      Assertions.assertThrows(RuntimeException.class, () -> {
      DoubleGlobalParameter parent = new DoubleGlobalParameter("parent", "parent", DEFAULT_VALUE, null);
      new DoubleGlobalParameter("invalidChild", "test description", new GlobalParameter[] {parent}, null); // invalid

      parent.set(1.0);
      });
   }

	@Test
   public void testCantSetChild()
   {
      Assertions.assertThrows(RuntimeException.class, () -> {
      DoubleGlobalParameter parent = new DoubleGlobalParameter("parent", "", 0.7, null);
      DoubleGlobalParameter child = new DoubleGlobalParameter("child", "", new GlobalParameter[] {parent}, null);

      child.set(0.99, "Shouldn't be able to change this!");
      });
   }
}
