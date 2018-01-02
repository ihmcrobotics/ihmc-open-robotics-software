package us.ihmc.robotEnvironmentAwareness.communication;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

public class APIFactory
{
   private boolean isFactoryClosed = false;
   private Map<Integer, CategoryTheme> categoryThemeIdSet = new HashMap<>();
   private Map<Integer, TopicTheme> topicThemeIdSet = new HashMap<>();

   private API api;

   public APIFactory()
   {
   }

   public Category getRootCategory(CategoryTheme rootTheme)
   {
      assertFactoryIsOpen();
      Category root = new Category(null, rootTheme);
      api = new API(root);
      return root;
   }

   public API getAPIAndCloseFactory()
   {
      isFactoryClosed = true;
      categoryThemeIdSet = null;
      topicThemeIdSet = null;
      return api;
   }

   public CategoryTheme createCategoryTheme(String name)
   {
      assertFactoryIsOpen();
      CategoryTheme newTheme = new CategoryTheme(name);
      if (categoryThemeIdSet.put(newTheme.getId(), newTheme) != null)
         throw new RuntimeException("Duplicate category theme id.");
      return newTheme;
   }

   public TopicTheme createTopicTheme(String name)
   {
      assertFactoryIsOpen();
      TopicTheme newTheme = new TopicTheme(name);
      TopicTheme oldTheme = topicThemeIdSet.put(newTheme.getId(), newTheme);
      if (oldTheme != null && !oldTheme.equals(newTheme))
         throw new RuntimeException("Duplicate topic theme id.");
      return newTheme;
   }

   public <T> TypedTopicTheme<T> createTypedTopicTheme(String name)
   {
      TypedTopicTheme<T> newTheme = new TypedTopicTheme<T>(name);
      TopicTheme oldTheme = topicThemeIdSet.put(newTheme.getId(), newTheme);
      if (oldTheme != null && !oldTheme.equals(newTheme))
         throw new RuntimeException("Duplicate topic theme id.");
      return newTheme;
   }

   public static class API
   {
      private final Category root;

      private API(Category root)
      {
         this.root = root;
      }

      public <T> Topic<T> findTopic(APIElementId topicId)
      {
         if (topicId.getShortIdAtDepth(0) != root.getShortId())
            throw new RuntimeException("The topic id does not belong to this API.");
         else
            return root.findTopic(topicId);
      }

      public <T> boolean containsTopic(Topic<T> topic)
      {
         return containsTopic(topic.getUniqueId());
      }

      public boolean containsTopic(APIElementId uniqueId)
      {
         if (uniqueId.getShortIdAtDepth(0) != root.getShortId())
            return false;
         else
            return root.findTopic(uniqueId) != null;
      }

      public Category getRoot()
      {
         return root;
      }

      @Override
      public String toString()
      {
         return "API of " + root.getName();
      }
   }

   public class CategoryTheme
   {
      private final int id;
      private final String name;

      private CategoryTheme(String name)
      {
         this.name = name;
         id = name.hashCode();
         assertFactoryIsOpen();
      }

      public int getId()
      {
         return id;
      }

      public String getName()
      {
         return name;
      }

      @Override
      public boolean equals(Object obj)
      {
         if (obj instanceof CategoryTheme)
            return equals((CategoryTheme) obj);
         else
            return false;
      }

      public boolean equals(CategoryTheme other)
      {
         return id == other.id && other.name.equals(other.name);
      }

      @Override
      public String toString()
      {
         return getName() + ", id = " + id;
      }
   }

   public class TopicTheme
   {
      private final int id;
      private final String name;

      TopicTheme(String name)
      {
         this.name = name;
         id = name.hashCode();
         assertFactoryIsOpen();
      }

      public int getId()
      {
         return id;
      }

      public String getName()
      {
         return name;
      }

      @Override
      public boolean equals(Object obj)
      {
         if (obj instanceof TopicTheme)
            return equals((TopicTheme) obj);
         else
            return false;
      }

      public boolean equals(TopicTheme other)
      {
         return id == other.id && other.name.equals(other.name);
      }

      @Override
      public String toString()
      {
         return getName() + ", id = " + id;
      }
   }

   public class TypedTopicTheme<T> extends TopicTheme
   {
      TypedTopicTheme(String name)
      {
         super(name);
      }

      @Override
      public boolean equals(Object obj)
      {
         if (obj instanceof TypedTopicTheme)
            return super.equals((TopicTheme) obj);
         else
            return false;
      }
   }

   public class Category
   {
      private final Category parent;
      private final CategoryTheme theme;
      private final Map<Integer, Category> childrenCategories = new HashMap<>();
      private final Map<Integer, Topic<?>> childrenTopics = new HashMap<>();

      private Category(Category parentCategory, CategoryTheme categoryTheme)
      {
         this.parent = parentCategory;
         this.theme = categoryTheme;
         assertFactoryIsOpen();
      }

      public Category child(CategoryTheme subCategoryTheme)
      {
         assertFactoryIsOpen();
         Category childCategory = childrenCategories.get(subCategoryTheme.getId());

         if (childCategory == null)
         {
            Category newChild = new Category(this, subCategoryTheme);
            childrenCategories.put(subCategoryTheme.getId(), newChild);
            return newChild;
         }
         else if (childCategory.theme.equals(subCategoryTheme))
         {
            return childCategory;
         }
         else
         {
            throw new RuntimeException("Not expecting a CategoryTheme with same id as one of this children but different name.");
         }
      }

      public <T> Topic<T> topic(TypedTopicTheme<T> topicTheme)
      {
         return topic((TopicTheme) topicTheme);
      }

      public <T> Topic<T> topic(TopicTheme topicTheme)
      {
         assertFactoryIsOpen();
         @SuppressWarnings("unchecked")
         Topic<T> childTopic = (Topic<T>) childrenTopics.get(topicTheme.getId());

         if (childTopic == null)
         {
            Topic<T> newChild = new Topic<T>(this, topicTheme);
            childrenTopics.put(topicTheme.getId(), newChild);
            return newChild;
         }
         else if (childTopic.theme.equals(topicTheme))
         {
            return childTopic;
         }
         else
         {
            throw new RuntimeException("Not expecting a TopicTheme with same id as one of this children but different name.");
         }
      }

      public Category[] getChildrenCategories()
      {
         return childrenCategories.values().toArray(new Category[0]);
      }
      
      public Topic<?>[] getChildrenTopics()
      {
         return childrenTopics.values().toArray(new Topic[0]);
      }

      @SuppressWarnings("unchecked")
      public <T> Topic<T> findTopic(APIElementId topicId)
      {
         int childrenDepth = getDepth() + 1;
         int shortIdAtDepth = topicId.getShortIdAtDepth(childrenDepth);

         if (childrenDepth == topicId.getElementDepth())
            return (Topic<T>) childrenTopics.get(shortIdAtDepth);
         else if (childrenCategories.containsKey(shortIdAtDepth))
         {
            Category child = childrenCategories.get(shortIdAtDepth);
            return child.findTopic(topicId);
         }
         else
            return null;
      }

      private int getDepth()
      {
         if (parent == null)
            return 0;
         else
            return parent.getDepth() + 1;
      }

      private void fillChildUniqueId(int[] topicUniqueIdToFill)
      {
         topicUniqueIdToFill[getDepth()] = theme.getId();
         if (parent != null)
            parent.fillChildUniqueId(topicUniqueIdToFill);
      }

      private int getShortId()
      {
         return theme.getId();
      }

      public APIElementId getUniqueId()
      {
         int idLength = getDepth() + 1;
         int[] uniqueId = new int[idLength];
         uniqueId[idLength - 1] = theme.getId();
         if (parent != null)
            parent.fillChildUniqueId(uniqueId);
         return new APIElementId(uniqueId);
      }

      public String getName()
      {
         if (parent == null)
            return getSimpleName();
         else
            return parent.getName() + "/" + getSimpleName();
      }

      public String getSimpleName()
      {
         return theme.getName();
      }

      @Override
      public boolean equals(Object obj)
      {
         if (obj instanceof Category)
            return equals((Category) obj);
         else
            return false;
      }

      public boolean equals(Category other)
      {
         if (!theme.equals(other.theme))
            return false;
         return parent == null ? other.parent == null : parent.equals(other.parent);
      }

      @Override
      public String toString()
      {
         return getName() + ", id = " + getUniqueId();
      }
   }

   public static class Topic<T>
   {
      private final TopicTheme theme;
      private final Category category;

      private Topic(Category topicCategory, TopicTheme topicTheme)
      {
         if (topicCategory == null)
            throw new RuntimeException("A topic has to belong to a category.");

         this.category = topicCategory;
         this.theme = topicTheme;
      }

      private int getDepth()
      {
         return category.getDepth() + 1;
      }

      public APIElementId getUniqueId()
      {
         int idLength = getDepth() + 1;
         int[] uniqueId = new int[idLength];
         uniqueId[idLength - 1] = theme.getId();
         category.fillChildUniqueId(uniqueId);
         return new APIElementId(uniqueId);
      }

      public String getName()
      {
         return category.getName() + "/" + getSimpleName();
      }

      public String getSimpleName()
      {
         return theme.getName();
      }

      @SuppressWarnings("unchecked")
      @Override
      public boolean equals(Object obj)
      {
         if (obj instanceof Topic)
            return equals((Topic<T>) obj);
         else
            return false;
      }

      public boolean equals(Topic<T> other)
      {
         if (!theme.equals(other.theme))
            return false;
         return category.equals(other.category);
      }

      @Override
      public String toString()
      {
         return getName() + ", id = " + getUniqueId();
      }
   }

   public static class APIElementId
   {
      public int[] id;

      public APIElementId()
      {
      }

      public APIElementId(int[] id)
      {
         this.id = id;
      }

      private int getElementDepth()
      {
         return id.length - 1;
      }

      private int getShortIdAtDepth(int depth)
      {
         return id[depth];
      }

      @Override
      public boolean equals(Object obj)
      {
         if (obj instanceof APIElementId)
            return equals((APIElementId) obj);
         else
            return false;
      }

      public boolean equals(APIElementId other)
      {
         return Arrays.equals(id, other.id);
      }

      @Override
      public String toString()
      {
         return Arrays.toString(id);
      }
   }

   private void assertFactoryIsOpen()
   {
      if (isFactoryClosed)
         throw new RuntimeException("This API factory is closed, the API cannot be changed.");
   }
}
