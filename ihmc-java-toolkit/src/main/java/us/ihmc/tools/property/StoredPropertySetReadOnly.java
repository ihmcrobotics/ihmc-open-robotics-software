package us.ihmc.tools.property;

import java.util.List;

public interface StoredPropertySetReadOnly
{
   double get(DoubleStoredPropertyKey key);

   int get(IntegerStoredPropertyKey key);

   boolean get(BooleanStoredPropertyKey key);

   <T> T get(StoredPropertyKey<T> key);

   <T> StoredPropertyReadOnly<T> getProperty(StoredPropertyKey<T> key);

   List<Object> getAll();
}