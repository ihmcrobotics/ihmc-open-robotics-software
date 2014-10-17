package us.ihmc.atlas.drcsim;

import org.apache.commons.io.IOUtils;

import java.io.IOException;

public class DRCSimGazeboLauncher
{
   public static void main(String[] args)
   {
      new DRCSimGazeboLauncher();
   }

   public DRCSimGazeboLauncher()
   {      
      runProcess();
   }
   
   private void runProcess()
   {
      //Build plugin
      String commandline1 = "source ~/.bashrc;" +
            "source /opt/ros/indigo/setup.bash;" +
            "cd ihmc_gazebo_catkin_ws;"+
            "catkin_make";
      runCommandLine(commandline1);

      //Launch gazebo with ihmc_plugin
      String commandline2 = "source ~/.bashrc;" +
            "source /opt/ros/indigo/setup.bash;" +
            "source /usr/share/drcsim/setup.sh;" +
            "source $PWD/ihmc_gazebo_catkin_ws/devel/setup.bash;" +
            "roslaunch ihmc_gazebo ihmc_atlas.launch";
      runCommandLine(commandline2);
   }

   public static void runCommandLine(String commandline){
      String[] commands = new String[]{"/bin/bash", "-l","-c", commandline};
      ProcessBuilder processBuilder = new ProcessBuilder(commands);
      processBuilder.inheritIO();
      processBuilder.redirectErrorStream(true);

      try {
         Process p = processBuilder.start();
         p.waitFor();
         IOUtils.copy(p.getErrorStream(), System.out);
      } catch (IOException e) {
         e.printStackTrace();
      } catch (InterruptedException e) {
         e.printStackTrace();
      }
   }
}
