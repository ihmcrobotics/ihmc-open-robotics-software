package us.ihmc.tools.reflect;

import java.lang.reflect.AccessibleObject;
import java.lang.reflect.Array;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;

public final class ReflectionTools
{
   private ReflectionTools()
   {
      // disallow construction
   }

   /**
    * Prints the declared fields of the class.
    * This includes public, protected, default (package) access, and private fields, but excludes inherited fields.
    * @param cls class to print the fields for
    */
   public static void printDeclaredFields(Class<?> cls)
   {
      Field[] fields = cls.getDeclaredFields();
      for (Field field : fields)
      {
         System.out.println(field.getName());
      }
   }

   /**
    * Prints the contents of the declared fields of the instance.
    * This includes public, protected, default (package) access, and private fields, but excludes inherited fields.
    * @param <T> type of the instance
    * @param cls type to print the declared fields for
    * @param instance instance to print the declared field contents for
    * @throws IllegalAccessException
    * @throws IllegalArgumentException
    */
   public static <T> void printDeclaredFieldContents(Class<? extends T> cls, T instance) throws IllegalArgumentException, IllegalAccessException
   {
      Field[] fields = cls.getDeclaredFields();
      for (Field field : fields)
      {
         printFieldContents(instance, field);
      }
   }

   /**
    * Prints the contents of all fields of the instance
    * @param instance instance instance to print the field contents for
    * @throws IllegalAccessException
    * @throws IllegalArgumentException
    */
   public static void printAllFieldContents(Object instance) throws IllegalArgumentException, IllegalAccessException
   {
      ArrayList<Field> allFields = getAllFields(instance.getClass());

      for (Field field : allFields)
      {
         printFieldContents(instance, field);
      }
   }

   /**
    * Compares the declared primitives of two instances of a class
    * @param <T> type of the instances
    * @param cls type to compare the declared fields for
    * @param instance1 first instance to test
    * @param instance2 second instance to test
    * @return a list of differing primitives
    * @throws IllegalAccessException
    * @throws IllegalArgumentException
    */
   public static <T> ArrayList<Field> compareDeclaredPrimitives(Class<? extends T> cls, T instance1, T instance2)
           throws IllegalArgumentException, IllegalAccessException
   {
      ArrayList<Field> differingFields = new ArrayList<Field>();
      Field[] fields = cls.getDeclaredFields();
      AccessibleObject.setAccessible(fields, true);

      for (Field field : fields)
      {
         if (field.getType().isPrimitive() &&!isPrimitiveFieldContentTheSame(instance1, instance2, field))
         {
            differingFields.add(field);
         }
      }

      return differingFields;
   }

   /**
    * Compares all primitives of two instances of a class
    * @param <T> type of the instances
    * @param instance1 first instance to test
    * @param instance2 second instance to test
    * @return a list of differing primitives
    * @throws IllegalAccessException
    * @throws IllegalArgumentException
    */
   public static <T> ArrayList<Field> compareAllPrimitives(T instance1, T instance2) throws IllegalArgumentException, IllegalAccessException
   {
      ArrayList<Field> allFields = getAllFields(instance1.getClass());

      ArrayList<Field> differingFields = new ArrayList<Field>();

      for (Field field : allFields)
      {
         field.setAccessible(true);

         if (field.getType().isPrimitive() &&!isPrimitiveFieldContentTheSame(instance1, instance2, field))
         {
            differingFields.add(field);
         }
      }

      return differingFields;
   }

   /**
    * Compares the values of a field for two instances of a class.
    * @param <T> type of the instances. Type must contain the specified field.
    * @param instance1 first instance to test
    * @param instance2 second instance to test
    * @param field field to test
    * @return true iff the primitive fields are equal
    * @throws IllegalAccessException
    * @throws IllegalArgumentException
    */
   public static <T> boolean isPrimitiveFieldContentTheSame(T instance1, T instance2, Field field) throws IllegalArgumentException, IllegalAccessException
   {
      Class<?> type = field.getType();

      if (type == byte.class)
         return (field.getByte(instance1) == field.getByte(instance2));
      else if (type == short.class)
         return (field.getShort(instance1) == field.getShort(instance2));
      else if (type == int.class)
         return (field.getInt(instance1) == field.getInt(instance2));
      else if (type == long.class)
         return (field.getLong(instance1) == field.getLong(instance2));
      else if (type == float.class)
      {
         float float1 = field.getFloat(instance1);
         float float2 = field.getFloat(instance2);

         return ((float1 == float2) || (Double.isNaN(float1) && Double.isNaN(float2)));
      }
      else if (type == double.class)
      {
         double double1 = field.getDouble(instance1);
         double double2 = field.getDouble(instance2);

         return ((double1 == double2) || (Double.isNaN(double1) && Double.isNaN(double2)));
      }
      else if (type == boolean.class)
         return (field.getBoolean(instance1) == field.getBoolean(instance2));
      else if (type == char.class)
         return (field.getChar(instance1) == field.getChar(instance2));
      else
         throw new RuntimeException("Field content is not primitive");
   }

   /**
    * Compares the values of a field for two instances of a class.
    * @param <T> type of the instances. Type must contain the specified field.
    * @param instance1 first instance to test
    * @param instance2 second instance to test
    * @param field field to test
    * @return true iff the primitive fields are equal
    */
   public static <T> boolean isPrimitiveArrayEntryTheSame(T instance1, T instance2, int index)
   {
      Class<?> type = instance1.getClass().getComponentType();

      if (type == byte.class)
         return (Array.getByte(instance1, index) == Array.getByte(instance2, index));
      else if (type == short.class)
         return (Array.getShort(instance1, index) == Array.getShort(instance2, index));
      else if (type == int.class)
         return (Array.getInt(instance1, index) == Array.getInt(instance2, index));
      else if (type == long.class)
         return (Array.getLong(instance1, index) == Array.getLong(instance2, index));
      else if (type == float.class)
      {
         float float1 = Array.getFloat(instance1, index);
         float float2 = Array.getFloat(instance2, index);

         return ((float1 == float2) || (Double.isNaN(float1) && Double.isNaN(float2)));
      }
      else if (type == double.class)
      {
         double double1 = Array.getDouble(instance1, index);
         double double2 = Array.getDouble(instance2, index);

         return ((double1 == double2) || (Double.isNaN(double1) && Double.isNaN(double2)));
      }
      else if (type == boolean.class)
         return (Array.getBoolean(instance1, index) == Array.getBoolean(instance2, index));
      else if (type == char.class)
         return (Array.getChar(instance1, index) == Array.getChar(instance2, index));
      else
         throw new RuntimeException("Array component type is not primitive");
   }

   public static boolean isBooleanFieldSet(Class<?> clazz, String fieldName)
   {
      try
      {
         Field field = clazz.getField(fieldName);
         
         return field.getBoolean(null);
      }
      catch (NoSuchFieldException | SecurityException | IllegalArgumentException | IllegalAccessException e)
      {
         return false;
      }
   }
   
   public static boolean isBooleanFieldExists(Class<?> clazz, String fieldName)
   {
      try
      {
         clazz.getField(fieldName);
         
         return true;
      }
      catch (NoSuchFieldException | SecurityException | IllegalArgumentException e)
      {
         return false;
      }
   }

   public static String getStringRepresentationOfFieldContent(Object object, Field field) throws IllegalArgumentException, IllegalAccessException
   {
      String ret = field.get(object).toString();
      return ret;
   }

   public static String getStringRepresentationOfArrayEntry(Object array, int index)
   {
      return Array.get(array, index).toString();
   }



   public static ArrayList<Field> getAllFields(Class<?> cls)
   {
      ArrayList<Field> allFields = new ArrayList<Field>();
      while (cls != null)
      {
         Field[] fields = cls.getDeclaredFields();
         for (Field field : fields)
         {
            allFields.add(field);
         }

         cls = cls.getSuperclass();
      }

      return allFields;
   }

   public static void printFieldContents(Object instance, Field field) throws IllegalArgumentException, IllegalAccessException
   {
      field.setAccessible(true);
      Class<?> type = field.getType();

      System.out.println(type + " " + field.getName() + " = " + field.get(instance));
   }

   public static <T> List<T> getFieldContentsAssignableFromClass(Object object, Class<T> clazz)
   {
      ArrayList<Field> fields = ReflectionTools.getAllFields(object.getClass());

      List<T> fieldContentList = new ArrayList<T>();
      try
      {
         for (Field field : fields)
         {
            if (clazz.isAssignableFrom(field.getType()))
            {
               boolean fieldAccessible = field.isAccessible();
               if (!fieldAccessible)
                  field.setAccessible(true);
               fieldContentList.add(clazz.cast(field.get(object)));
               field.setAccessible(fieldAccessible);
            }
         }
      }
      catch (IllegalArgumentException e)
      {
         e.printStackTrace();
      }
      catch (IllegalAccessException e)
      {
         e.printStackTrace();
      }

      return fieldContentList;
   }
}
