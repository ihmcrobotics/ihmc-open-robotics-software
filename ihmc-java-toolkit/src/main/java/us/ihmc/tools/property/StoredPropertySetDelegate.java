package us.ihmc.tools.property;

import us.ihmc.commons.thread.Notification;

import java.nio.file.Path;
import java.util.List;

/**
 * Used to swap out the underlying properties for StoredPropertySets that
 * get passed around.
 */
public class StoredPropertySetDelegate implements StoredPropertySetBasics
{
   private StoredPropertySet storedPropertySet;

   public void setStoredPropertySet(StoredPropertySet storedPropertySet)
   {
      this.storedPropertySet = storedPropertySet;
   }

   @Override
   public double get(DoubleStoredPropertyKey key)
   {
      return storedPropertySet.get(key);
   }

   @Override
   public int get(IntegerStoredPropertyKey key)
   {
      return storedPropertySet.get(key);
   }

   @Override
   public boolean get(BooleanStoredPropertyKey key)
   {
      return storedPropertySet.get(key);
   }

   @Override
   public <T> T get(StoredPropertyKey<T> key)
   {
      return storedPropertySet.get(key);
   }

   @Override
   public void set(DoubleStoredPropertyKey key, double value)
   {
      storedPropertySet.set(key, value);
   }

   @Override
   public void set(IntegerStoredPropertyKey key, int value)
   {
      storedPropertySet.set(key, value);
   }

   @Override
   public void set(BooleanStoredPropertyKey key, boolean value)
   {
      storedPropertySet.set(key, value);
   }

   @Override
   public <T> void set(StoredPropertyKey<T> key, T value)
   {
      storedPropertySet.set(key, value);
   }

   @Override
   public <T> StoredPropertyBasics<T> getProperty(StoredPropertyKey<T> key)
   {
      return storedPropertySet.getProperty(key);
   }

   @Override
   public void set(StoredPropertySetReadOnly other)
   {
      storedPropertySet.set(other);
   }

   @Override
   public void setAll(List<Object> newValues)
   {
      storedPropertySet.setAll(newValues);
   }

   @Override
   public void setAllFromStrings(List<String> stringValues)
   {
      storedPropertySet.setAllFromStrings(stringValues);
   }

   @Override
   public void setFromColonCommaString(String colonCommaString)
   {
      storedPropertySet.setFromColonCommaString(colonCommaString);
   }

   @Override
   public void addPropertyChangedListener(StoredPropertyKey key, Runnable onPropertyChanged)
   {
      storedPropertySet.addPropertyChangedListener(key, onPropertyChanged);
   }

   @Override
   public void removePropertyChangedListener(StoredPropertyKey key, Runnable onPropertyChanged)
   {
      storedPropertySet.removePropertyChangedListener(key, onPropertyChanged);
   }

   @Override
   public void addAnyPropertyChangedListener(Notification anyPropertyChangedNotification)
   {
      storedPropertySet.addAnyPropertyChangedListener(anyPropertyChangedNotification);
   }

   @Override
   public void removeAnyPropertyChangedListener(Notification anyPropertyChangedNotification)
   {
      storedPropertySet.removeAnyPropertyChangedListener(anyPropertyChangedNotification);
   }

   @Override
   public void updateBackingSaveFile(String versionSuffix)
   {
      storedPropertySet.updateBackingSaveFile(versionSuffix);
   }

   @Override
   public void load()
   {
      storedPropertySet.load();
   }

   @Override
   public void load(String file)
   {
      storedPropertySet.load(file);
   }

   @Override
   public void load(String file, boolean crashIfMissingKey)
   {
      storedPropertySet.load(file, crashIfMissingKey);
   }

   @Override
   public void save()
   {
      storedPropertySet.save();
   }

   @Override
   public StoredPropertyKeyListReadOnly getKeyList()
   {
      return storedPropertySet.getKeyList();
   }

   @Override
   public Path findSaveFileDirectory()
   {
      return storedPropertySet.findSaveFileDirectory();
   }

   @Override
   public void setTitle(String title)
   {
      storedPropertySet.setTitle(title);
   }

   @Override
   public List<Object> getAll()
   {
      return storedPropertySet.getAll();
   }

   @Override
   public List<String> getAllAsStrings()
   {
      return storedPropertySet.getAllAsStrings();
   }

   @Override
   public String getTitle()
   {
      return storedPropertySet.getTitle();
   }

   @Override
   public String getCurrentVersionSuffix()
   {
      return storedPropertySet.getCurrentVersionSuffix();
   }

   @Override
   public String getCapitalizedClassName()
   {
      return storedPropertySet.getCapitalizedClassName();
   }
}
