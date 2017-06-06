package us.ihmc.communication.remote;

import java.io.Serializable;
import java.util.Vector;

public class RemoteRequest implements Serializable
{
   private static final long serialVersionUID = -3338612762215398251L;
   public String methodName;
   public Vector<Serializable> parameters;

   public RemoteRequest(String methodname, Vector<Serializable> parameters)
   {
      this.methodName = methodname;
      this.parameters = parameters;
   }

   @Override
   public String toString()
   {
      return methodName + ": " + parameters;
   }
}
