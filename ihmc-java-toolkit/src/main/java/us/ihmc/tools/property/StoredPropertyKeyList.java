package us.ihmc.tools.property;

import java.util.ArrayList;
import java.util.List;

public class StoredPropertyKeyList implements StoredPropertyKeyListBasics
{
   private int indexCount = 0;

   private final List<StoredPropertyKey<?>> keys = new ArrayList<>();

   @Override
   public DoubleStoredPropertyKey addDoubleKey(String titleCasedName)
   {
      DoubleStoredPropertyKey key = new DoubleStoredPropertyKey(indexCount++, titleCasedName);
      keys.add(key);
      return key;
   }

   @Override
   public DoubleStoredPropertyKey addDoubleKey(String titleCasedName, double defaultValue)
   {
      DoubleStoredPropertyKey key = new DoubleStoredPropertyKey(indexCount++, titleCasedName, defaultValue);
      keys.add(key);
      return key;
   }

   @Override
   public IntegerStoredPropertyKey addIntegerKey(String titleCasedName)
   {
      IntegerStoredPropertyKey key = new IntegerStoredPropertyKey(indexCount++, titleCasedName);
      keys.add(key);
      return key;
   }

   @Override
   public IntegerStoredPropertyKey addIntegerKey(String titleCasedName, int defaultValue)
   {
      IntegerStoredPropertyKey key = new IntegerStoredPropertyKey(indexCount++, titleCasedName, defaultValue);
      keys.add(key);
      return key;
   }

   @Override
   public BooleanStoredPropertyKey addBooleanKey(String titleCasedName)
   {
      BooleanStoredPropertyKey key = new BooleanStoredPropertyKey(indexCount++, titleCasedName);
      keys.add(key);
      return key;
   }

   @Override
   public BooleanStoredPropertyKey addBooleanKey(String titleCasedName, boolean defaultValue)
   {
      BooleanStoredPropertyKey key = new BooleanStoredPropertyKey(indexCount++, titleCasedName, defaultValue);
      keys.add(key);
      return key;
   }

   @Override
   public List<StoredPropertyKey<?>> keys()
   {
      return keys;
   }
}
