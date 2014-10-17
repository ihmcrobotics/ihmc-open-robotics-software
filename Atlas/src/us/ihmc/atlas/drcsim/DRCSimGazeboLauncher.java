package us.ihmc.atlas.drcsim;

import java.io.File;
import java.io.IOException;
import java.util.Map;

public class DRCSimGazeboLauncher
{
   public DRCSimGazeboLauncher()
   {      
      runProcess("ihmc_gazebo_catkin_ws", new String[] {"catkin_make"});
   }
   
   private void runProcess(String directory, String[] args)
   {
      try
      {
         ProcessBuilder builder = new ProcessBuilder(args);
         
         builder.directory(new File(directory));
         
         builder.redirectErrorStream(true);
         
//         builder.environment().
         
         Map<String, String> environment = builder.environment();
         
         environment.put("PATH", environment.get("PATH") + ":/opt/ros/indigo/bin");
         
         for (String key : environment.keySet())
         {
            System.out.println(key + "=" + environment.get(key));
         }
         
         builder.redirectOutput(ProcessBuilder.Redirect.INHERIT);
         
         Process p = builder.start();
         
         p.waitFor();
      }
      catch (IOException | InterruptedException e)
      {
         e.printStackTrace();
      }
   }
   
   public static void main(String[] args)
   {
      new DRCSimGazeboLauncher();
   }
}
