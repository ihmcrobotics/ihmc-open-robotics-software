package us.ihmc.simulationConstructionSetTools.util.globalParameters;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
   AndBooleanGlobalParameterTest.class,
   BooleanGlobalParameterTest.class,
   DoubleGlobalParameterTest.class,
   IntGlobalParameterTest.class,
   MultiplicativeDoubleGlobalParameterTest.class,
   OrBooleanGlobalParameterTest.class
})

public class GlobalParameterTestSuite
{
   public static void main(String[] args)
   {
   }
}
