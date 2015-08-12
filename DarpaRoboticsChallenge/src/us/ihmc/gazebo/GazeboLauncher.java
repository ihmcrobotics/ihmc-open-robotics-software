package us.ihmc.gazebo;

import us.ihmc.tools.thread.ThreadTools;

public class GazeboLauncher
{
   public static final boolean RUN_CATKIN_MAKE = true;
   public enum InitialPose {STANDING, CAR_FRONT, CAR_LATER};
   public static final InitialPose INITIAL_POSE = InitialPose.CAR_FRONT;
   
   public static void main(String[] args)
   {
      new GazeboLauncher();
   }

   public GazeboLauncher()
   {      
      runProcess();
   }
   
   private void runProcess()
   {
      //Build plugin
      if (RUN_CATKIN_MAKE)
      {
         String commandline1 = "source ~/.bashrc;" +
               "source /opt/ros/indigo/setup.bash;" +
               "cd ihmc_gazebo_catkin_ws;"+
               "catkin_make";
         ThreadTools.runCommandLine(commandline1);
      }

      //Launch gazebo with ihmc_plugin
      String commandline2 = "source ~/.bashrc;" +
            "source /opt/ros/indigo/setup.bash;" +
            "source /usr/share/drcsim/setup.sh;" +
            "source $PWD/ihmc_gazebo_catkin_ws/devel/setup.bash;" +
            "roslaunch ihmc_gazebo ihmc_atlas_" + INITIAL_POSE.name().toLowerCase() + ".launch";
      ThreadTools.runCommandLine(commandline2);
   }
}
