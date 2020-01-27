package us.ihmc.humanoidBehaviors.navigation;

import us.ihmc.humanoidBehaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.BehaviorDefinition;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.messager.MessagerAPIFactory;

public class NavigationBehavior implements BehaviorInterface
{
   public static final BehaviorDefinition DEFINITION = new BehaviorDefinition("Navigation", NavigationBehavior::new, NavigationBehaviorAPI.create());

   private final BehaviorHelper helper;

   public NavigationBehavior(BehaviorHelper helper)
   {
      this.helper = helper;


   }

   @Override
   public void setEnabled(boolean enabled)
   {

   }

   public static class NavigationBehaviorAPI
   {
      private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
      private static final MessagerAPIFactory.Category RootCategory = apiFactory.createRootCategory("NavigationBehavior");
      private static final MessagerAPIFactory.CategoryTheme NavigationTheme = apiFactory.createCategoryTheme("Navigation");

      public static final MessagerAPIFactory.Topic<Object> Step = topic("Step");

      private static final <T> MessagerAPIFactory.Topic<T> topic(String name)
      {
         return RootCategory.child(NavigationTheme).topic(apiFactory.createTypedTopicTheme(name));
      }

      public static final MessagerAPIFactory.MessagerAPI create()
      {
         return apiFactory.getAPIAndCloseFactory();
      }
   }
}
