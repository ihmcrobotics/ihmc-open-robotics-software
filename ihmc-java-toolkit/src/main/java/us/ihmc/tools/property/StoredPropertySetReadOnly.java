package us.ihmc.tools.property;

import java.util.List;

public interface StoredPropertySetReadOnly
{
   double getValue(DoubleStoredPropertyKey key);

   int getValue(IntegerStoredPropertyKey key);

   boolean getValue(BooleanStoredPropertyKey key);

   List<Object> getAllValues();
}