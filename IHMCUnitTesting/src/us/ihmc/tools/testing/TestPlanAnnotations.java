package us.ihmc.tools.testing;

import java.lang.annotation.ElementType;
import java.lang.annotation.Target;

public class TestPlanAnnotations
{
   @Target(ElementType.TYPE)
   public @interface DeployableTestClass
   {
      TestPlanTarget[] targets() default {TestPlanTarget.Fast};
   }
   
   @Target(ElementType.METHOD)
   public @interface DeployableTestMethod
   {
      double estimatedDuration() default 1.0;
   }
}
