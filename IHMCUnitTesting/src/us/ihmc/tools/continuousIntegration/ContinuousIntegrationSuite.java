package us.ihmc.tools.continuousIntegration;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.util.Arrays;

import org.junit.runner.Description;
import org.junit.runner.manipulation.Filter;
import org.junit.runner.manipulation.NoTestsRemainException;
import org.junit.runners.Suite;
import org.junit.runners.model.InitializationError;
import org.junit.runners.model.RunnerBuilder;

import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class ContinuousIntegrationSuite extends Suite
{
   @Retention(RetentionPolicy.RUNTIME)
   @Target(ElementType.TYPE)
   public @interface ContinuousIntegrationSuiteCategory
   {
       public IntegrationCategory value();
   }
   
   public ContinuousIntegrationSuite(Class<?> clazz, RunnerBuilder builder) throws InitializationError
   {
      super(clazz, builder);
      
      try
      {
         filter(new TestPlanFilter(getTestSuiteTarget(clazz)));
      }
      catch (NoTestsRemainException e)
      {
         throw new InitializationError(ContinuousIntegrationSuite.class.getSimpleName() + ": No unfiltered tests in suite.");
      }
   }
   
   private static class TestPlanFilter extends Filter
   {
      private final IntegrationCategory target;
      
      public TestPlanFilter(IntegrationCategory target)
      {
         this.target = target;
      }
      
      @Override
      public boolean shouldRun(Description description)
      {
         IntegrationCategory[] directTargets = getDirectTargets(description);
         if (directTargets != null)
         {
            boolean containsTarget = Arrays.asList(directTargets).contains(target);
            
            if (description.isTest())
            {
               return containsTarget;
            }
            else if (containsTarget)
            {
               return true;
            }
         }
         if (description.isTest())
         {
            IntegrationCategory[] parentTargets = getParentTargets(description);
            if (parentTargets != null)
            {
               return Arrays.asList(parentTargets).contains(target);
            }
         }
         
         if (target.equals(IntegrationCategory.defaultCategory))
         {
            return true;
         }
         
         for (Description child : description.getChildren())
         {
            if (shouldRun(child))
            {
               return true;
            }
         }
         return false;
      }

      @Override
      public String describe()
      {
         return "Filter for tests in " + target.getName() + ".";
      }
      
      private IntegrationCategory[] getParentTargets(Description description)
      {
         Class<?> testClass = description.getTestClass();
         if (testClass != null)
         {
            Description parentDescription = Description.createSuiteDescription(testClass);
            return getDirectTargets(parentDescription);
         }
         
         return null;
      }
      
      private IntegrationCategory[] getDirectTargets(Description description)
      {
         ContinuousIntegrationTest deployableTestMethod = description.getAnnotation(ContinuousIntegrationTest.class);
         if (deployableTestMethod != null && deployableTestMethod.categoriesOverride().length > 0)
         {
            return deployableTestMethod.categoriesOverride();
         }
         ContinuousIntegrationPlan deployableTestClass = description.getAnnotation(ContinuousIntegrationPlan.class);
         if (deployableTestClass != null && deployableTestClass.categories().length > 0)
         {
            return deployableTestClass.categories();
         }
         
         return null;
      }
   }
   
   private IntegrationCategory getTestSuiteTarget(Class<?> clazz)
   {
      ContinuousIntegrationSuiteCategory annotation = clazz.getAnnotation(ContinuousIntegrationSuiteCategory.class);
      return annotation == null ? null : annotation.value();
   }
}
