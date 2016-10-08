package us.ihmc.tools.testing;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

public class TestPlanAnnotations
{
   public static final Double defaultEstimatedDuration = 0.1;
   
   @Retention(RetentionPolicy.RUNTIME)
   @Target(ElementType.TYPE)
   public @interface ContinuousIntegrationPlan
   {
      TestPlanTarget[] targets() default {TestPlanTarget.Fast};
   }
   
   @Retention(RetentionPolicy.RUNTIME)
   @Target(ElementType.METHOD)
   public @interface ContinuousIntegrationTest
   {
      double estimatedDuration();
      TestPlanTarget[] targetsOverride() default {};
   }
}
