package us.ihmc.simulationconstructionset.dataBuffer;

import org.junit.Before;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.yoVariables.dataBuffer.DataBuffer;
import us.ihmc.yoVariables.dataBuffer.DataBufferEntry;
import us.ihmc.simulationconstructionset.gui.config.VarGroup;
import us.ihmc.simulationconstructionset.gui.config.VarGroupList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class DataBufferToolsTest
{
   private final int testBufferSize = 100;

   private YoVariableRegistry registry;
   private DataBuffer dataBuffer = new DataBuffer(testBufferSize);

   private YoDouble a, b, c;
   private DataBufferEntry aBuffer, bBuffer, cBuffer;

   @Before
   public void setUp()
   {
      registry = new YoVariableRegistry("testRegistry");

      a = new YoDouble("a_arm", registry);
      b = new YoDouble("b_arm", registry);
      c = new YoDouble("c_arm", registry);

      aBuffer = new DataBufferEntry(a, testBufferSize);
      bBuffer = new DataBufferEntry(b, testBufferSize);
      cBuffer = new DataBufferEntry(c, testBufferSize);
   }


   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout=300000)
   public void testGetVarsFromGroup()
   {
      dataBuffer.addEntry(aBuffer);
      dataBuffer.addEntry(bBuffer);
      dataBuffer.addEntry(cBuffer);

      VarGroup varGroupOne = new VarGroup("varGroupOne");
      VarGroup varGroupTwo = new VarGroup("varGroupTwo");
      VarGroup varGroupThree = new VarGroup("varGroupThree");

      varGroupOne.addVar("a_arm");
      varGroupOne.addVar("c_arm");

      String[] allRegularExpressions = { ".*" };
      String[] cRegularExpressions = { "c.*" };

      varGroupTwo.addRegularExpressions(allRegularExpressions);
      varGroupThree.addRegularExpressions(cRegularExpressions);

      VarGroupList varGroupList = new VarGroupList();
      varGroupList.addVarGroup(varGroupOne);
      varGroupList.addVarGroup(varGroupTwo);
      varGroupList.addVarGroup(varGroupThree);

      ArrayList<YoVariable<?>> allVarsFromGroup = DataBufferTools.getVarsFromGroup(dataBuffer, "all", varGroupList);

      assertTrue(allVarsFromGroup.contains(a));
      assertTrue(allVarsFromGroup.contains(b));
      assertTrue(allVarsFromGroup.contains(c));

      ArrayList<YoVariable<?>> aVarsFromGroup = DataBufferTools.getVarsFromGroup(dataBuffer,"varGroupOne", varGroupList);

      assertTrue(aVarsFromGroup.contains(a));
      assertFalse(aVarsFromGroup.contains(b));
      assertTrue(aVarsFromGroup.contains(c));

      ArrayList<YoVariable<?>> regExpVarsFromGroup = DataBufferTools.getVarsFromGroup(dataBuffer,"varGroupTwo", varGroupList);

      assertTrue(regExpVarsFromGroup.contains(a));
      assertTrue(regExpVarsFromGroup.contains(b));
      assertTrue(regExpVarsFromGroup.contains(c));

      ArrayList<YoVariable<?>> cRegExpVarsFromGroup = DataBufferTools.getVarsFromGroup(dataBuffer, "varGroupThree", varGroupList);

      assertFalse(cRegExpVarsFromGroup.contains(a));
      assertFalse(cRegExpVarsFromGroup.contains(b));
      assertTrue(cRegExpVarsFromGroup.contains(c));
   }
}
