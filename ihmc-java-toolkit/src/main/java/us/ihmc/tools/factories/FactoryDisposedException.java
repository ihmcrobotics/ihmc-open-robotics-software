package us.ihmc.tools.factories;

@SuppressWarnings("serial")
public class FactoryDisposedException extends RuntimeException
{
   public FactoryDisposedException()
   {
      super("Factory is disposed. Factories can only be used once!");
   }
}
