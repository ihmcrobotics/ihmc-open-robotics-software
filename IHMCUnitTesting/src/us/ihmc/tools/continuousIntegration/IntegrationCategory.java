package us.ihmc.tools.continuousIntegration;

import java.util.ArrayList;

public enum IntegrationCategory
{
   // No suites for these
   EXCLUDE("Exclude", false, true),
   MANUAL("Manual", false, true),
   
   // Active, not load balanced
   COMPILE("Compile", false, false),
   CODE_QUALITY("CodeQuality", false, false),
   HEALTH("RepoHealth", false, false),
   
   // Load balanced (i.e A, B, C suites)
   FAST("Fast", true, false),
   SLOW("Slow", true, false),
   VIDEO("Video", true, false),
   UI("UI", true, false),
   FLAKY("Flaky", true, false),
   IN_DEVELOPMENT("InDevelopment", true, false),
   ;
   
   public static final IntegrationCategory[] values = values();
   public static final IntegrationCategory defaultCategory = FAST;
   public static final IntegrationCategory[] includedCategories;
   public static final IntegrationCategory[] loadBalancedCategories;
   public static final IntegrationCategory[] unbalancedCategories;
   
   static
   {
      ArrayList<IntegrationCategory> notExcludedCategories = new ArrayList<>();
      ArrayList<IntegrationCategory> balancedCategories = new ArrayList<>();
      ArrayList<IntegrationCategory> notBalancedCategories = new ArrayList<>();
      for (IntegrationCategory category : values)
      {
         if (!category.isExcluded())
         {
            notExcludedCategories.add(category);
         }
         if (category.isLoadBalanced())
         {
            balancedCategories.add(category);
         }
         if (category.isIncludedAndNotLoadBalanced())
         {
            notBalancedCategories.add(category);
         }
      }
      loadBalancedCategories = balancedCategories.toArray(new IntegrationCategory[0]);
      unbalancedCategories = notBalancedCategories.toArray(new IntegrationCategory[0]);
      includedCategories = notExcludedCategories.toArray(new IntegrationCategory[0]);
   }
   
   private final String name;
   private final boolean loadBalanced;
   private final boolean excluded;
   
   private IntegrationCategory(String name, boolean loadBalanced, boolean excluded)
   {
      this.name = name;
      this.loadBalanced = loadBalanced;
      this.excluded = excluded;
   }
   
   public String getName()
   {
      return name;
   }
   
   public static IntegrationCategory fromString(String name)
   {
      for (IntegrationCategory category : values)
      {
         if (category.name().equals(name))
         {
            return category;
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
