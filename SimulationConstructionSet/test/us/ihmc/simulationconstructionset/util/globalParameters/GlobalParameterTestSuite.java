package us.ihmc.simulationconstructionset.util.globalParameters;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
   us.ihmc.simulationconstructionset.util.globalParameters.AndBooleanGlobalParameterTest.class,
   us.ihmc.simulationconstructionset.util.globalParameters.BooleanGlobalParameterTest.class,
   us.ihmc.simulationconstructionset.util.globalParameters.DoubleGlobalParameterTest.class,
   us.ihmc.simulationconstructionset.util.globalParameters.IntGlobalParameterTest.class,
   us.ihmc.simulationconstructionset.util.globalParameters.MultiplicativeDoubleGlobalParameterTest.class,
   us.ihmc.simulationconstructionset.util.globalParameters.OrBooleanGlobalParameterTest.class
})

public class GlobalParameterTestSuite
{
   public static void main(String[] args)
   {
   }
}
