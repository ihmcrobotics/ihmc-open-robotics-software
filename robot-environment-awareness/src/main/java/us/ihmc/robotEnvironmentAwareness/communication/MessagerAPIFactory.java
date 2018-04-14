package us.ihmc.robotEnvironmentAwareness.communication;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import gnu.trove.map.TIntObjectMap;
import gnu.trove.map.hash.TIntObjectHashMap;

/**
 * This class can be used to generate the API for a set of messagers that are to communicate
 * messages between each other.
 * 
 * @author Sylvain Bertrand
 */
public class MessagerAPIFactory
{
   private boolean isFactoryClosed = false;
   private Map<Integer, CategoryTheme> categoryThemeIdSet = new HashMap<>();
   private Map<Integer, TopicTheme> topicThemeIdSet = new HashMap<>();

   private MessagerAPI api;

   public MessagerAPIFactory()
   {
   }

   /**
    * Creates the root category for this API from which sub-categories and topics can be created.
    * 
    * @param rootCategoryName the name of the root category.
    * @return the root category of this API.
    */
   public Category createRootCategory(String rootCategoryName)
   {
      return createRootCategory(createCategoryTheme(rootCategoryName));
   }

   /**
    * Creates the root category for this API from which sub-categories and topics can be created.
    * 
    * @param rootTheme the theme for the root category.
    * @return the root category of this API.
    */
   public Category createRootCategory(CategoryTheme rootTheme)
   {
      assertFactoryIsOpen();
      Category root = new Category(null, rootTheme);
      api = new MessagerAPI(root);
      return root;
   }

   /**
    * Gets the generated API and closes this factory, i.e. no additional topic can be created after
    * calling this method.
    * 
    * @return the generated API.
    */
   public MessagerAPI getAPIAndCloseFactory()
   {
      isFactoryClosed = true;
      categoryThemeIdSet = null;
      topicThemeIdSet = null;
      return api;
   }

   /**
    * Creates a theme for creating categories.
    * 
    * @param name the name of the category theme.
    * @return the category theme.
    * @see Category#child(CategoryTheme)
    */
   public CategoryTheme createCategoryTheme(String name)
   {
      assertFactoryIsOpen();
      CategoryTheme newTheme = new CategoryTheme(name);
      if (categoryThemeIdSet.put(newTheme.getId(), newTheme) != null)
         throw new RuntimeException("Duplicate category theme id.");
      return newTheme;
   }

   /**
    * Creates a topic theme with no type associated.
    * <p>
    * Prefer creating {@link TypedTopicTheme} which allows to associate a type to a theme providing
    * extra information at compilation time.
    * </p>
    * 
    * @param name the name of the topic theme.
    * @return the topic theme.
    * @see #createTypedTopicTheme(String)
    * @see Category#topic(TopicTheme)
    */
   public TopicTheme createTopicTheme(String name)
   {
      assertFactoryIsOpen();
      TopicTheme newTheme = new TopicTheme(name);
      TopicTheme oldTheme = topicThemeIdSet.put(newTheme.getId(), newTheme);
      if (oldTheme != null && !oldTheme.equals(newTheme))
         throw new RuntimeException("Duplicate topic theme id.");
      return newTheme;
   }

   /**
    * Creates a topic theme with a type associated with it.
    * 
    * @param name the name of the topic theme.
    * @return the topic theme.
    * @see Category#topic(TypedTopicTheme)
    */
   public <T> TypedTopicTheme<T> createTypedTopicTheme(String name)
   {
      TypedTopicTheme<T> newTheme = new TypedTopicTheme<T>(name);
      TopicTheme oldTheme = topicThemeIdSet.put(newTheme.getId(), newTheme);
      if (oldTheme != null && !oldTheme.equals(newTheme))
         throw new RuntimeException("Duplicate topic theme id.");
      return newTheme;
   }

   /**
    * This represent an API that can be used to create messager.
    * 
    * @author Sylvain Bertrand
    *
    */
   public static class MessagerAPI
   {
      /** The root category of this API to which all sub-categories and topics are attached. */
      private final Category root;

      private MessagerAPI(Category root)
      {
         this.root = root;
      }

      /**
       * Retrieves the corresponding topic to the given ID.
       * 
       * @param topicId the ID of the topic to retrieve.
       * @return the topic.
       */
      public <T> Topic<T> findTopic(TopicID topicId)
      {
         if (topicId.getShortIdAtDepth(0) != root.getShortId())
            throw new RuntimeException("The topic id does not belong to this API.");
         else
            return root.findTopic(topicId);
      }

      /**
       * Tests whether this API declares the given topic.
       * 
       * @param topic the query.
       * @return {@code true} if this API declares the given topic, {@code false} otherwise.
       */
      public <T> boolean containsTopic(Topic<T> topic)
      {
         return containsTopic(topic.getUniqueId());
      }

      /**
       * Tests whether this API declares a topic with an ID equal to the given message ID.
       * 
       * @param uniqueId the query.
       * @return {@code true} if this API declares a topic with the given ID, {@code false}
       *         otherwise.
       */
      public boolean containsTopic(TopicID uniqueId)
      {
         if (uniqueId.getShortIdAtDepth(0) != root.getShortId())
            return false;
         else
            return root.findTopic(uniqueId) != null;
      }

      /**
       * Gets the root category of this API.
       * 
       * @return the root category.
       */
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

   /**
    * A category theme is used to create a category via {@link Category#child(CategoryTheme)} which
    * in turn can be used to create a topic.
    * 
    * @author Sylvain Bertrand
    */
   public class CategoryTheme
   {
      private final int id;
      private final String name;

      /**
       * Creates a new category theme with the given name.
       * 
       * @param name the name of the new category theme.
       */
      private CategoryTheme(String name)
      {
         this.name = name;
         id = name.hashCode();
         assertFactoryIsOpen();
      }

      /**
       * Gets the ID of this category theme.
       * 
       * @return this category theme ID.
       */
      public int getId()
      {
         return id;
      }

      /**
       * Gets the name of this category theme.
       * 
       * @return this category theme name.
       */
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

   /**
    * A topic theme is used to create a topic via {@link Category#topic(TopicTheme)}.
    * 
    * @author Sylvain Bertrand
    */
   public class TopicTheme
   {
      private final int id;
      private final String name;

      /**
       * Creates a new topic theme with the given name.
       * 
       * @param name the name of the new topic theme.
       */
      private TopicTheme(String name)
      {
         this.name = name;
         id = name.hashCode();
         assertFactoryIsOpen();
      }

      /**
       * Gets the ID of this topic theme.
       * 
       * @return this topic theme ID.
       */
      public int getId()
      {
         return id;
      }

      /**
       * Gets the name of this category theme.
       * 
       * @return this category theme name.
       */
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

   /**
    * A typed topic theme is a topic theme to which a type can be associated.
    * 
    * @author Sylvain Bertrand
    *
    * @param <T> the type to associate with this topic theme.
    */
   public class TypedTopicTheme<T> extends TopicTheme
   {
      /**
       * Creates a new typed topic theme with the given name.
       * 
       * @param name the name of the new topic theme.
       */
      private TypedTopicTheme(String name)
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

   /**
    * A category can be used to create sub-categories and topics.
    * 
    * @author Sylvain Bertrand
    */
   public class Category
   {
      /** The parent category of this category. */
      private final Category parent;
      /** This category theme. */
      private final CategoryTheme theme;
      /** The map from ID to each child category. */
      private final TIntObjectMap<Category> childrenCategories = new TIntObjectHashMap<>();
      /** The map from ID to each child topic. */
      private final TIntObjectMap<Topic<?>> childrenTopics = new TIntObjectHashMap<>();

      private Category(Category parentCategory, CategoryTheme categoryTheme)
      {
         this.parent = parentCategory;
         this.theme = categoryTheme;
         assertFactoryIsOpen();
      }

      /**
       * Creates and returns a new category registered as child of this category.
       * 
       * @param subCategoryTheme the theme of the new child category.
       * @return the new child category.
       */
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

      /**
       * Creates and returns a new topic registered as child of this category.
       * 
       * @param topicTheme the theme of the new child topic.
       * @return the new child topic.
       */
      public <T> Topic<T> topic(TypedTopicTheme<T> topicTheme)
      {
         return topic((TopicTheme) topicTheme);
      }

      /**
       * Creates and return a new topic registered as child of this category.
       * 
       * @param topicTheme the theme of the new child topic.
       * @return the new child topic.
       */
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

      /**
       * Gets the children categories of this category as an array.
       * 
       * @return the array of this category's child categories.
       */
      public Category[] getChildrenCategories()
      {
         return childrenCategories.values(new Category[childrenCategories.size()]);
      }

      /**
       * Gets the children categories of this category as an array.
       * 
       * @return the array of this category's child topics.
       */
      public Topic<?>[] getChildrenTopics()
      {
         return childrenTopics.values(new Topic[childrenTopics.size()]);
      }

      /**
       * Search for the topic corresponding to the given ID.
       * 
       * @param topicId the query.
       * @return the corresponding topic if found, {@code null} otherwise.
       */
      @SuppressWarnings("unchecked")
      public <T> Topic<T> findTopic(TopicID topicId)
      {
         int childrenDepth = getDepth() + 1;
         int shortIdAtDepth = topicId.getShortIdAtDepth(childrenDepth);

         if (childrenDepth == topicId.getTopicDepth())
            return (Topic<T>) childrenTopics.get(shortIdAtDepth);
         else if (childrenCategories.containsKey(shortIdAtDepth))
         {
            Category child = childrenCategories.get(shortIdAtDepth);
            return child.findTopic(topicId);
         }
         else
            return null;
      }

      /**
       * Gets the distance from this category to the root.
       * 
       * @return this category's depth.
       */
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

      /**
       * Creates and returns the ID corresponding to this category.
       * 
       * @return this category's unique ID.
       */
      private TopicID getUniqueId()
      {
         int idLength = getDepth() + 1;
         int[] uniqueId = new int[idLength];
         uniqueId[idLength - 1] = theme.getId();
         if (parent != null)
            parent.fillChildUniqueId(uniqueId);
         return new TopicID(uniqueId);
      }

      /**
       * Gets the full name, i.e. including this category's ancestors, of this category.
       * 
       * @return this category's name.
       */
      public String getName()
      {
         if (parent == null)
            return getSimpleName();
         else
            return parent.getName() + "/" + getSimpleName();
      }

      /**
       * Gets the name that was used to create this category.
       * 
       * @return this category's simple name.
       */
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

   /**
    * A topic can be used to provide additional information to data when sending a
    * {@link REAMessage}.
    * 
    * @author Sylvain Bertrand
    *
    * @param <T> the type associated to this topic.
    */
   public static class Topic<T>
   {
      /** This topic theme. */
      private final TopicTheme theme;
      /** The category to which this category belongs. */
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

      /**
       * Creates and returns the ID corresponding to this topic.
       * 
       * @return this topic's unique ID.
       */
      public TopicID getUniqueId()
      {
         int idLength = getDepth() + 1;
         int[] uniqueId = new int[idLength];
         uniqueId[idLength - 1] = theme.getId();
         category.fillChildUniqueId(uniqueId);
         return new TopicID(uniqueId);
      }

      /**
       * Gets the full name, i.e. including this topic's category and ancestors, of this topic.
       * 
       * @return this category's name.
       */
      public String getName()
      {
         return category.getName() + "/" + getSimpleName();
      }

      /**
       * Gets the name that was used to create this topic.
       * 
       * @return this topic's simple name.
       */
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

   /**
    * When using the messager over network, it is preferable to send {@link Topic} information using
    * simple data types such as {@link TopicID} instead of directly sending the topic itself.
    * <p>
    * The ID of a topic can be via {@link Topic#getUniqueId()} and the topic can be retrieved using
    * its ID via {@link MessagerAPI#findTopic(TopicID)}.
    * </p>
    * 
    * @author Sylvain Bertrand
    */
   public static class TopicID
   {
      /**
       * The ID of a topic.
       * <p>
       * This field is public and non-final only for serialization purposes, it is not meant to be
       * accessed directly.
       * </p>
       */
      public int[] id;

      /** Empty constructor only used for serialization purposes. */
      public TopicID()
      {
      }

      /**
       * Creates a new topic ID.
       * 
       * @param id the id of a topic.
       */
      public TopicID(int[] id)
      {
         this.id = id;
      }

      private int getTopicDepth()
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
         if (obj instanceof TopicID)
            return equals((TopicID) obj);
         else
            return false;
      }

      public boolean equals(TopicID other)
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
