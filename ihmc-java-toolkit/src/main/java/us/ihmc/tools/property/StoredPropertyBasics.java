package us.ihmc.tools.property;

public interface StoredPropertyBasics<T> extends StoredPropertyReadOnly<T>
{
   void set(T value);

   void load();

   void save();
}
