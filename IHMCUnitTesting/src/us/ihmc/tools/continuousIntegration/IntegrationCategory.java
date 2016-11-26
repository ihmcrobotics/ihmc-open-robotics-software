package us.ihmc.tools.continuousIntegration;

import java.util.ArrayList;

public enum IntegrationCategory
{
   EXCLUDE("Exclude", false, true),
   MANUAL("Manual", false, false),
   FAST("Fast", true, false),
   SLOW("Slow", true, false),
   VIDEO("Video", true, false),
   UI("UI", true, false),
   CODE_QUALITY("CodeQuality", false, false),
   COMPILE("Compile", false, false),
   FLAKY("Flaky", true, false),
   IN_DEVELOPMENT("InDevelopment", true, false),
   HEALTH("RepoHealth", false, false);
   
   public static final IntegrationCategory[] values = values();
   public static final IntegrationCategory defaultCategory = FAST;
   public static final IntegrationCategory[] includedCategories;
   public static final IntegrationCategory[] loadBalancedCategories;
   static
   {
      includedCategories = new IntegrationCategory[values.length - 1];
      int i = 0;
      for (IntegrationCategory category : values)
      {
         if (category.isExcluded())
            continue;

         includedCategories[i++] = category;
      }
      
      ArrayList<IntegrationCategory> balancedCategories = new ArrayList<>();
      for (IntegrationCategory category : values)
      {
         if (category.isLoadBalanced())
            balancedCategories.add(category);
      }
      loadBalancedCategories = balancedCategories.toArray(new IntegrationCategory[0]);
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
