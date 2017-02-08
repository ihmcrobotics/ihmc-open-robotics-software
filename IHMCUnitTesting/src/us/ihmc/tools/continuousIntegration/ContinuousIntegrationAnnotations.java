package us.ihmc.tools.continuousIntegration;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

public class ContinuousIntegrationAnnotations
{
   @Retention(RetentionPolicy.RUNTIME)
   @Target(ElementType.TYPE)
   public @interface ContinuousIntegrationPlan
   {
      IntegrationCategory[] categories() default {IntegrationCategory.FAST};
   }
   
   @Retention(RetentionPolicy.RUNTIME)
   @Target(ElementType.METHOD)
   public @interface ContinuousIntegrationTest
   {
      double estimatedDuration();
      IntegrationCategory[] categoriesOverride() default {};
   }
}
