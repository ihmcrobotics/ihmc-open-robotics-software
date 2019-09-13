package us.ihmc.tools.property;

public class StoredProperty<T> implements StoredPropertyBasics<T>
{
   private final StoredPropertyKey<T> key;
   private final StoredPropertySetBasics set;

   public StoredProperty(StoredPropertyKey<T> key, StoredPropertySetBasics set)
   {
      this.key = key;
      this.set = set;
   }

   @Override
   public T get()
   {
      return set.get(key);
   }

   @Override
   public void set(T value)
   {
      set.set(key, value);
   }

   @Override
   public void load()
   {
      set.load();
   }

   @Override
   public void save()
   {
      set.save();
   }
}
