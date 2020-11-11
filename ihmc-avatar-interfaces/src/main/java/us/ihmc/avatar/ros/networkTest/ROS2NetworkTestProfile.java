package us.ihmc.avatar.ros.networkTest;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;

import java.net.InetAddress;
import java.util.ArrayList;
import java.util.List;

public abstract class ROS2NetworkTestProfile
{
   protected String localHostName = ExceptionTools.handle(() -> InetAddress.getLocalHost().getHostName(), DefaultExceptionHandler.RUNTIME_EXCEPTION);

   public abstract List<String> getHostnames();

   public abstract void runExperiment();

   public List<String> getRemoteHostnames()
   {
      ArrayList<String> remoteHostnames = new ArrayList<>();
      for (String hostname : getHostnames())
      {
         if (!hostname.equals(localHostName))
         {
            remoteHostnames.add(hostname);
         }
      }
      return remoteHostnames;
   }
}
