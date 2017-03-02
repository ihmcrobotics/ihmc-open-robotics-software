package us.ihmc.tools.factories;

public abstract class FactoryField<T>
{
   protected boolean hasBeenSet = false;
   protected boolean disposed = false;
   protected String fieldName;
   protected T fieldValue = null;
   
   public FactoryField(String fieldName)
   {
      this.fieldName = fieldName;
   }
   
   public void set(T fieldValue)
   {
      checkNotDisposed();
      
      hasBeenSet = true;
      this.fieldValue = fieldValue;
   }
   
   public T get()
   {
      checkNotDisposed();
      
      if (!hasBeenSet)
      {
         throw new FactoryFieldNotSetException(fieldName);
      }
      else
      {
         return fieldValue;
      }
   }
   
   public void dispose()
   {
      disposed = true;
   }
   
   protected void checkNotDisposed()
   {
      if (disposed)
      {
         throw new FactoryDisposedException();
      }
   }
}
