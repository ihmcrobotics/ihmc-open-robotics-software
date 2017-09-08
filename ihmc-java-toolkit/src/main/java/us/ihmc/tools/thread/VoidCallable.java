package us.ihmc.tools.thread;

import java.util.concurrent.Callable;

public abstract class VoidCallable implements Callable<Object>
{
   @Override
   public Object call() throws Exception
   {
      onCalled();
      return null;
   }
   
   public abstract void onCalled();
}
