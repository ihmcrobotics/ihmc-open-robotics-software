package us.ihmc.tools.property;

import org.apache.commons.lang3.StringUtils;

import java.util.ArrayList;
import java.util.List;

public class StoredPropertyKeyList
{
   private int indexCount = 0;

   private final List<StoredPropertyKey<?>> keys = new ArrayList<>();

   private String saveFileName;

   public StoredPropertyKeyList(Class<?> classToNameAfter)
   {
      this(StringUtils.uncapitalize(classToNameAfter.getSimpleName()));
   }

   public StoredPropertyKeyList(String saveFileName)
   {
      this.saveFileName = saveFileName;
   }

   public DoubleStoredPropertyKey addDoubleKey(String titleCasedName)
   {
      DoubleStoredPropertyKey key = new DoubleStoredPropertyKey(indexCount++, titleCasedName);
      keys.add(key);
      return key;
   }

   public DoubleStoredPropertyKey addDoubleKey(String titleCasedName, double defaultValue)
   {
      DoubleStoredPropertyKey key = new DoubleStoredPropertyKey(indexCount++, titleCasedName, defaultValue);
      keys.add(key);
      return key;
   }

   public IntegerStoredPropertyKey addIntegerKey(String titleCasedName)
   {
      IntegerStoredPropertyKey key = new IntegerStoredPropertyKey(indexCount++, titleCasedName);
      keys.add(key);
      return key;
   }

   public IntegerStoredPropertyKey addIntegerKey(String titleCasedName, int defaultValue)
   {
      IntegerStoredPropertyKey key = new IntegerStoredPropertyKey(indexCount++, titleCasedName, defaultValue);
      keys.add(key);
      return key;
   }

   public BooleanStoredPropertyKey addBooleanKey(String titleCasedName)
   {
      BooleanStoredPropertyKey key = new BooleanStoredPropertyKey(indexCount++, titleCasedName);
      keys.add(key);
      return key;
   }

   public BooleanStoredPropertyKey addBooleanKey(String titleCasedName, boolean defaultValue)
   {
      BooleanStoredPropertyKey key = new BooleanStoredPropertyKey(indexCount++, titleCasedName, defaultValue);
      keys.add(key);
      return key;
   }

   public List<StoredPropertyKey<?>> keys()
   {
      return keys;
   }

   public String getSaveFileName()
   {
      return saveFileName;
   }
}
