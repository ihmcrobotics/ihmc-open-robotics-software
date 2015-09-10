package us.ihmc.tools.testing;

import static org.junit.Assume.assumeTrue;

public enum TestPlanTarget
{
   Exclude,
   Manual,
   Fast,
   Slow,
   VideoA,
   VideoB,
   UI,
   CodeQuality,
   Flaky,
   InDevelopment;
   
   public static final TestPlanTarget[] values = values();
   private static final String bambooJobName = System.getenv("BAMBOO_JOB_NAME");
   
   public static boolean isRunningOnBamboo()
   {
      return bambooJobName != null;
   }
   
   public static boolean isRunningOnPlan(TestPlanTarget bambooPlanType)
   {
      return bambooPlanType != null && bambooJobName.endsWith(bambooPlanType.name());
   }
   
   public static void assumeRunningLocally()
   {
      assumeTrue("Test only set to run locally.", !isRunningOnBamboo());
   }
   
   public static void assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget... bambooPlanType)
   {
      if (!isRunningOnBamboo())
         return;
      
      boolean isRunningOnPlan = false;
      
      for (TestPlanTarget planType : bambooPlanType)
         isRunningOnPlan |= isRunningOnPlan(planType);
         
      assumeTrue("Test only set to run on " + bambooPlanType, isRunningOnPlan);
   }
}
