package us.ihmc.javaFXToolkit.messager;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
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
   private Map<Integer, CategoryTheme> categoryThemeIDSet = new HashMap<>();
   private Map<Integer, TopicTheme> topicThemeIDSet = new HashMap<>();

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
    * Allows to combine multiple APIs together.
    * <p>
    * This effectively adds the root categories of the given APIs and adds them to the API being
    * built.
    * </p>
    * <p>
    * The root categories of each API must have unique names.
    * </p>
    * 
    * @param apis the other APIs to be included to the API constructed by this factory.
    */
   public void includeMessagerAPIs(MessagerAPI... apis)
   {
      if (api == null)
         throw new RuntimeException("Call MessageAPIFactory.createRootCategory(...) before calling this method.");

      for (MessagerAPI messagerAPI : apis)
      {
         for (Category root : messagerAPI.roots)
         {
            api.addRoot(root);
         }
      }
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
      categoryThemeIDSet = null;
      topicThemeIDSet = null;
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
      if (categoryThemeIDSet.put(newTheme.getID(), newTheme) != null)
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
      TopicTheme oldTheme = topicThemeIDSet.put(newTheme.getID(), newTheme);
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
      TopicTheme oldTheme = topicThemeIDSet.put(newTheme.getID(), newTheme);
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
      private final List<Category> roots = new ArrayList<>();

      private MessagerAPI(Category root)
      {
         this.roots.add(root);
      }

      private void addRoot(Category newRoot)
      {
         for (Category root : roots)
         {
            if (root.getName() == newRoot.getName() || root.getShortID() == newRoot.getShortID())
               throw new RuntimeException("Roots must have unique name and ID.");
         }
         roots.add(newRoot);
      }

      /**
       * Retrieves the corresponding topic to the given ID.
       * 
       * @param topicID the ID of the topic to retrieve.
       * @return the topic.
       */
      public <T> Topic<T> findTopic(TopicID topicID)
      {
         for (Category root : roots)
         {
            if (topicID.getShortIDAtDepth(0) == root.getShortID())
               return root.findTopic(topicID);
         }
         throw new RuntimeException("The topic id does not belong to this API.");
      }

      /**
       * Tests whether this API declares the given topic.
       * 
       * @param topic the query.
       * @return {@code true} if this API declares the given topic, {@code false} otherwise.
       */
      public <T> boolean containsTopic(Topic<T> topic)
      {
         return containsTopic(topic.getUniqueID());
      }

      /**
       * Tests whether this API declares a topic with an ID equal to the given message ID.
       * 
       * @param topicID the query.
       * @return {@code true} if this API declares a topic with the given ID, {@code false}
       *         otherwise.
       */
      public boolean containsTopic(TopicID topicID)
      {
         for (Category root : roots)
         {
            if (topicID.getShortIDAtDepth(0) == root.getShortID())
               return root.findTopic(topicID) != null;
         }
         return false;
      }

      @Override
      public String toString()
      {
         return "API of " + roots.get(0).getName();
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
      private int getID()
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
      int getID()
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
         Category childCategory = childrenCategories.get(subCategoryTheme.getID());

         if (childCategory == null)
         {
            Category newChild = new Category(this, subCategoryTheme);
            childrenCategories.put(subCategoryTheme.getID(), newChild);
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
         Topic<T> childTopic = (Topic<T>) childrenTopics.get(topicTheme.getID());

         if (childTopic == null)
         {
            Topic<T> newChild = new Topic<T>(this, topicTheme);
            childrenTopics.put(topicTheme.getID(), newChild);
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
       * @param topicID the query.
       * @return the corresponding topic if found, {@code null} otherwise.
       */
      @SuppressWarnings("unchecked")
      public <T> Topic<T> findTopic(TopicID topicID)
      {
         int childrenDepth = getDepth() + 1;
         int shortIDAtDepth = topicID.getShortIDAtDepth(childrenDepth);

         if (childrenDepth == topicID.getTopicDepth())
            return (Topic<T>) childrenTopics.get(shortIDAtDepth);
         else if (childrenCategories.containsKey(shortIDAtDepth))
         {
            Category child = childrenCategories.get(shortIDAtDepth);
            return child.findTopic(topicID);
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

      private void fillChildUniqueID(int[] topicUniqueIDToFill)
      {
         topicUniqueIDToFill[getDepth()] = theme.getID();
         if (parent != null)
            parent.fillChildUniqueID(topicUniqueIDToFill);
      }

      private int getShortID()
      {
         return theme.getID();
      }

      /**
       * Creates and returns the ID corresponding to this category.
       * 
       * @return this category's unique ID.
       */
      private TopicID getUniqueID()
      {
         int idLength = getDepth() + 1;
         int[] uniqueID = new int[idLength];
         uniqueID[idLength - 1] = theme.getID();
         if (parent != null)
            parent.fillChildUniqueID(uniqueID);
         return new TopicID(uniqueID);
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
         return getName() + ", id = " + getUniqueID();
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
      public TopicID getUniqueID()
      {
         int idLength = getDepth() + 1;
         int[] uniqueID = new int[idLength];
         uniqueID[idLength - 1] = theme.getID();
         category.fillChildUniqueID(uniqueID);
         return new TopicID(uniqueID);
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
         return getName() + ", id = " + getUniqueID();
      }
   }

   /**
    * When using the messager over network, it is preferable to send {@link Topic} information using
    * simple data types such as {@link TopicID} instead of directly sending the topic itself.
    * <p>
    * The ID of a topic can be via {@link Topic#getUniqueID()} and the topic can be retrieved using
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

      private int getShortIDAtDepth(int depth)
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
