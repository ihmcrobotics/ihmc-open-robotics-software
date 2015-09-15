package us.ihmc.tools.testing;

import static org.junit.Assume.assumeTrue;

import java.util.ArrayList;

public enum TestPlanTarget
{
   Exclude(false, true),
   Manual(false, false),
   Fast(true, false),
   Slow(true, false),
   VideoA(true, false),
   VideoB(true, false),
   UI(true, false),
   CodeQuality(false, false),
   Flaky(true, false),
   InDevelopment(true, false);
   
   public static final TestPlanTarget[] values = values();
   public static final TestPlanTarget[] includedTargets;
   public static final TestPlanTarget[] loadBalancedTargets;
   static
   {
      includedTargets = new TestPlanTarget[values.length - 1];
      int i = 0;
      for (TestPlanTarget target : values)
      {
         if (target.isExcluded())
            continue;

         includedTargets[i++] = target;
      }
      
      ArrayList<TestPlanTarget> balancedTargets = new ArrayList<>();
      for (TestPlanTarget target : values)
      {
         if (target.isLoadBalanced())
            balancedTargets.add(target);
      }
      loadBalancedTargets = balancedTargets.toArray(new TestPlanTarget[0]);
   }
   
   private static final String bambooJobName = System.getenv("BAMBOO_JOB_NAME");
   
   private final boolean loadBalanced;
   private final boolean excluded;
   
   private TestPlanTarget(boolean loadBalanced, boolean excluded)
   {
      this.loadBalanced = loadBalanced;
      this.excluded = excluded;
   }
   
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
   
   public static TestPlanTarget fromString(String name)
   {
      for (TestPlanTarget testPlanTarget : values)
      {
         if (testPlanTarget.name().equals(name))
         {
            return testPlanTarget;
         }
      }
      
      return null;
   }
   
   public boolean isIncludedAndNotLoadBalanced()
   {
      return !isExcluded() && !isLoadBalanced();
   }

   public boolean isLoadBalanced()
   {
      return loadBalanced;
   }

   public boolean isExcluded()
   {
      return excluded;
   }
}
