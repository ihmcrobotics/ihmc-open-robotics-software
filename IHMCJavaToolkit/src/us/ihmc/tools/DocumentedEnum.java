package us.ihmc.tools;

public interface DocumentedEnum<T>
{
   public String getDocumentation(T var);
   
   public T[] getDocumentedValues();
}
