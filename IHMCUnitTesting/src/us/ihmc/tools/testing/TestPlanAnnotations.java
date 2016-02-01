package us.ihmc.tools.testing;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

public class TestPlanAnnotations
{
   // TODO Rename to TestPlanClass?
   @Retention(RetentionPolicy.RUNTIME)
   @Target(ElementType.TYPE)
   public @interface DeployableTestClass
   {
      TestPlanTarget[] targets() default {TestPlanTarget.Fast};
   }
   
   // TODO Rename to TestPlanMethod?
   @Retention(RetentionPolicy.RUNTIME)
   @Target(ElementType.METHOD)
   public @interface DeployableTestMethod
   {
      double estimatedDuration() default 1.0;
      TestPlanTarget[] targets() default {};
   }
}
