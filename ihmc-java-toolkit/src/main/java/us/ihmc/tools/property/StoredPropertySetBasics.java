package us.ihmc.tools.property;

import java.nio.file.Path;
import java.util.List;

public interface StoredPropertySetBasics extends StoredPropertySetReadOnly
{
   void set(DoubleStoredPropertyKey key, double value);

   void set(IntegerStoredPropertyKey key, int value);

   void set(BooleanStoredPropertyKey key, boolean value);

   <T> void set(StoredPropertyKey<T> key, T value);

   @Override
   <T> StoredPropertyBasics<T> getProperty(StoredPropertyKey<T> key);

   void setAll(List<Object> newValues);

   void setAllFromStrings(List<String> stringValues);

   void addPropertyChangedListener(StoredPropertyKey key, Runnable onPropertyChanged);

   void removePropertyChangedListener(StoredPropertyKey key, Runnable onPropertyChanged);

   void updateBackingSaveFile(String versionSuffix);

   void load();

   void load(String file);

   void load(String file, boolean crashIfMissingKey);

   void save();

   StoredPropertyKeyListReadOnly getKeyList();

   Path findSaveFileDirectory();

   String getCurrentVersionSuffix();

   String getCapitalizedClassName();
}