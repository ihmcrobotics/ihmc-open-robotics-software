package us.ihmc.tools.property;

public interface StoredPropertyKeyListBasics extends StoredPropertyKeyListReadOnly
{
   DoubleStoredPropertyKey addDoubleKey(String titleCasedName);

   DoubleStoredPropertyKey addDoubleKey(String titleCasedName, double defaultValue);

   IntegerStoredPropertyKey addIntegerKey(String titleCasedName);

   IntegerStoredPropertyKey addIntegerKey(String titleCasedName, int defaultValue);

   BooleanStoredPropertyKey addBooleanKey(String titleCasedName);

   BooleanStoredPropertyKey addBooleanKey(String titleCasedName, boolean defaultValue);
}
