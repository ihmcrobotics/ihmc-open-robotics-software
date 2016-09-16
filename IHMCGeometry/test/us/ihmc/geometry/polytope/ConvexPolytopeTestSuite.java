package us.ihmc.geometry.polytope;

import org.junit.runner.RunWith;
import org.junit.runners.Suite.SuiteClasses;

import us.ihmc.tools.testing.MutationTestingTools;
import us.ihmc.tools.testing.TestPlanSuite;
import us.ihmc.tools.testing.TestPlanSuite.TestSuiteTarget;
import us.ihmc.tools.testing.TestPlanTarget;

@RunWith(TestPlanSuite.class)
@TestSuiteTarget(TestPlanTarget.Fast)
@SuiteClasses({ ConvexPolytopeTest.class, GilbertJohnsonKeerthiCollisionDetectorTest.class })

public class ConvexPolytopeTestSuite
{
   public static void main(String[] args)
   {
      String targetTests = ConvexPolytopeTestSuite.class.getName();
      String targetClassesInSamePackage = MutationTestingTools.createClassSelectorStringFromTargetString(targetTests);
      MutationTestingTools.doPITMutationTestAndOpenResult(targetTests, targetClassesInSamePackage);
   }
}
