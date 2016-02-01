package us.ihmc.tools.testing;

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

import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class TestPlanSuite extends Suite
{
   @Retention(RetentionPolicy.RUNTIME)
   @Target(ElementType.TYPE)
   public @interface TestSuiteTarget
   {
       public TestPlanTarget value();
   }
   
   public TestPlanSuite(Class<?> clazz, RunnerBuilder builder) throws InitializationError
   {
      super(clazz, builder);
      
      try
      {
         filter(new TestPlanFilter(getTestSuiteTarget(clazz)));
      }
      catch (NoTestsRemainException e)
      {
         throw new InitializationError(TestPlanSuite.class.getSimpleName() + ": No unfiltered tests in suite.");
      }
   }
   
   private static class TestPlanFilter extends Filter
   {
      private final TestPlanTarget target;
      
      public TestPlanFilter(TestPlanTarget target)
      {
         this.target = target;
      }
      
      @Override
      public boolean shouldRun(Description description)
      {
         TestPlanTarget[] directTargets = getDirectTargets(description);
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
            TestPlanTarget[] parentTargets = getParentTargets(description);
            if (parentTargets != null)
            {
               return Arrays.asList(parentTargets).contains(target);
            }
         }
         
         if (target.equals(TestPlanTarget.defaultTarget))
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
         return "Filter for tests in " + target.name() + ".";
      }
      
      private TestPlanTarget[] getParentTargets(Description description)
      {
         Class<?> testClass = description.getTestClass();
         if (testClass != null)
         {
            Description parentDescription = Description.createSuiteDescription(testClass);
            return getDirectTargets(parentDescription);
         }
         
         return null;
      }
      
      private TestPlanTarget[] getDirectTargets(Description description)
      {
         DeployableTestMethod deployableTestMethod = description.getAnnotation(DeployableTestMethod.class);
         if (deployableTestMethod != null && deployableTestMethod.targets().length > 0)
         {
            return deployableTestMethod.targets();
         }
         DeployableTestClass deployableTestClass = description.getAnnotation(DeployableTestClass.class);
         if (deployableTestClass != null && deployableTestClass.targets().length > 0)
         {
            return deployableTestClass.targets();
         }
         
         return null;
      }
   }
   
   private TestPlanTarget getTestSuiteTarget(Class<?> clazz)
   {
      TestSuiteTarget annotation = clazz.getAnnotation(TestSuiteTarget.class);
      return annotation == null ? null : annotation.value();
   }
}
