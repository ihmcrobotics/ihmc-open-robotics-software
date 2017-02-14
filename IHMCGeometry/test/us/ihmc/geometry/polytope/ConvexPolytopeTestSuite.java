package us.ihmc.geometry.polytope;

import org.junit.runner.RunWith;
import org.junit.runners.Suite.SuiteClasses;

import us.ihmc.continuousIntegration.ContinuousIntegrationSuite;
import us.ihmc.continuousIntegration.ContinuousIntegrationSuite.ContinuousIntegrationSuiteCategory;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.tools.testing.MutationTestingTools;

@RunWith(ContinuousIntegrationSuite.class)
@ContinuousIntegrationSuiteCategory(IntegrationCategory.FAST)
@SuiteClasses({ ExpandingPolytopeAlgorithmTest.class, IcoSphereCreatorTest.class, SimplexPolytopeTest.class, ExpandingPolytopeEntryTest.class, ExpandingPolytopeEntryFromSimpleMeshGeneratorTest.class,
      ConvexPolytopeTest.class, GilbertJohnsonKeerthiCollisionDetectorTest.class, ConvexPolytopeFromExpandingPolytopeEntryGeneratorTest.class, ExpandingPolytopeSilhouetteConstructorTest.class })

public class ConvexPolytopeTestSuite
{
   public static void main(String[] args)
   {
      String targetTests = ConvexPolytopeTestSuite.class.getName();
      String targetClassesInSamePackage = MutationTestingTools.createClassSelectorStringFromTargetString(targetTests);
      MutationTestingTools.doPITMutationTestAndOpenResult(targetTests, targetClassesInSamePackage);
   }
}
