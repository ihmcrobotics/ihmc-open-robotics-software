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
         filter(new CategoryFilter(getSuiteCategory(clazz)));
      }
      catch (NoTestsRemainException e)
      {
         throw new InitializationError(ContinuousIntegrationSuite.class.getSimpleName() + ": No unfiltered tests in suite.");
      }
   }
   
   private static class CategoryFilter extends Filter
   {
      private final IntegrationCategory integrationCategory;
      
      public CategoryFilter(IntegrationCategory integrationCategory)
      {
         this.integrationCategory = integrationCategory;
      }
      
      @Override
      public boolean shouldRun(Description description)
      {
         IntegrationCategory[] directCategories = getDirectCategories(description);
         if (directCategories != null)
         {
            boolean containsCategory = Arrays.asList(directCategories).contains(integrationCategory);
            
            if (description.isTest())
            {
               return containsCategory;
            }
            else if (containsCategory)
            {
               return true;
            }
         }
         if (description.isTest())
         {
            IntegrationCategory[] parentTargets = getParentTargets(description);
            if (parentTargets != null)
            {
               return Arrays.asList(parentTargets).contains(integrationCategory);
            }
         }
         
         if (integrationCategory.equals(IntegrationCategory.defaultCategory))
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
         return "Filter for tests in " + integrationCategory.getName() + ".";
      }
      
      private IntegrationCategory[] getParentTargets(Description description)
      {
         Class<?> testClass = description.getTestClass();
         if (testClass != null)
         {
            Description parentDescription = Description.createSuiteDescription(testClass);
            return getDirectCategories(parentDescription);
         }
         
         return null;
      }
      
      private IntegrationCategory[] getDirectCategories(Description description)
      {
         ContinuousIntegrationTest testMethodAnnotation = description.getAnnotation(ContinuousIntegrationTest.class);
         if (testMethodAnnotation != null && testMethodAnnotation.categoriesOverride().length > 0)
         {
            return testMethodAnnotation.categoriesOverride();
         }
         ContinuousIntegrationPlan testClassAnnotation = description.getAnnotation(ContinuousIntegrationPlan.class);
         if (testClassAnnotation != null && testClassAnnotation.categories().length > 0)
         {
            return testClassAnnotation.categories();
         }
         
         return null;
      }
   }
   
   private IntegrationCategory getSuiteCategory(Class<?> clazz)
   {
      ContinuousIntegrationSuiteCategory annotation = clazz.getAnnotation(ContinuousIntegrationSuiteCategory.class);
      return annotation == null ? null : annotation.value();
   }
}
