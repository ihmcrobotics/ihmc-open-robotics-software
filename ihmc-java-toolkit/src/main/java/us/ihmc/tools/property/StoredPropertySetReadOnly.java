package us.ihmc.tools.property;

import java.util.List;

public interface StoredPropertySetReadOnly
{
   double get(DoubleStoredPropertyKey key);

   int get(IntegerStoredPropertyKey key);

   boolean get(BooleanStoredPropertyKey key);

   List<Object> getAll();
}