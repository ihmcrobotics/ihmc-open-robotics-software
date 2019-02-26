package us.ihmc.humanoidBehaviors.ui.model;

import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;

public class FXUIMessagerAPIFactory
{
   private final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private final Category rootCategory;

   public FXUIMessagerAPIFactory(Class<?> applicationClazz)
   {
      this.rootCategory = apiFactory.createRootCategory(apiFactory.createCategoryTheme(applicationClazz.getSimpleName()));
   }

   public <T> Topic<T> createTopic(String topicName, Class<T> topicType)
   {
      CategoryTheme theme = apiFactory.createCategoryTheme(topicName + "Theme");
      return rootCategory.child(theme).topic(apiFactory.createTypedTopicTheme(topicType.getSimpleName()));
   }

   public final MessagerAPI getAPIAndCloseFactory()
   {
      return apiFactory.getAPIAndCloseFactory();
   }
}
