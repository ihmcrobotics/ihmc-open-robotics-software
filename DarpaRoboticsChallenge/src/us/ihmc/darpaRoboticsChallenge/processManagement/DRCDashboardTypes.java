package us.ihmc.darpaRoboticsChallenge.processManagement;

import java.util.LinkedHashMap;

public class DRCDashboardTypes
{
   public enum DRCROSTasks
   {
      QUAL1, QUAL2, QUAL3, QUAL4, VRC1, VRC2, VRC3/*, DRIVING_TASK1, DRIVING_TASK1_WITH_EXTERNAL_CAMS, DRIVING_TEST_TRACK_WITH_EXTERNAL_CAMS*/;
   }
   
   public enum DRCPluginTasks
   {
      QUAL1, QUAL2, QUAL3, QUAL4, VRC1, VRC2, VRC3, ATLAS, ATLAS_VEHICLE, PARKING_LOT, HAND
   }

   private static final LinkedHashMap<DRCPluginTasks, String> pluginTaskPath = new LinkedHashMap<DRCPluginTasks, String>()
   {
      private static final long serialVersionUID = 7295926598139266176L;

      {
         put(DRCPluginTasks.QUAL1, "cd ~/workspace/GazeboStateCommunicator/launch/; ./qual1.sh &\n");
         put(DRCPluginTasks.QUAL2, "cd ~/workspace/GazeboStateCommunicator/launch/; ./qual2.sh &\n");
         put(DRCPluginTasks.QUAL3, "cd ~/workspace/GazeboStateCommunicator/launch/; ./qual3.sh &\n");
         put(DRCPluginTasks.QUAL4, "cd ~/workspace/GazeboStateCommunicator/launch/; ./qual4.sh &\n");
         put(DRCPluginTasks.VRC1, "cd ~/workspace/GazeboStateCommunicator/launch/; ./task1.sh &\n");
         put(DRCPluginTasks.VRC2, "cd ~/workspace/GazeboStateCommunicator/launch/; ./task2.sh &\n");
         put(DRCPluginTasks.VRC3, "cd ~/workspace/GazeboStateCommunicator/launch/; ./task3.sh &\n");
         put(DRCPluginTasks.ATLAS, "cd ~/workspace/GazeboStateCommunicator/launch/; ./atlas.sh &\n");
         put(DRCPluginTasks.ATLAS_VEHICLE, "cd ~/workspace/GazeboStateCommunicator/launch/; ./atlasVehicle.sh &\n");
         put(DRCPluginTasks.PARKING_LOT, "cd ~/workspace/GazeboStateCommunicator/launch/; ./parkingLot.sh &\n");
         put(DRCPluginTasks.HAND, "cd ~/workspace/GazeboStateCommunicator/launch/; ./hand.sh &\n");
      }
   };

   public static String getPluginCommand(DRCPluginTasks drcTask)
   {
      return pluginTaskPath.get(drcTask);
   }
   
   private static final LinkedHashMap<DRCROSTasks, String> defaultTaskPath = new LinkedHashMap<DRCROSTasks, String>()
   {
      private static final long serialVersionUID = 2304570433813141139L;

      {
         put(DRCROSTasks.QUAL2, "roslaunch atlas_utils qual_task_2.launch &\n");
         put(DRCROSTasks.QUAL1, "roslaunch atlas_utils qual_task_1.launch &\n");
         put(DRCROSTasks.QUAL3, "roslaunch atlas_utils qual_task_3.launch &\n");
         put(DRCROSTasks.QUAL4, "roslaunch atlas_utils qual_task_4.launch &\n");
         put(DRCROSTasks.VRC1, "roslaunch atlas_utils vrc_task_1.launch &\n");
         put(DRCROSTasks.VRC2, "roslaunch atlas_utils vrc_task_2.launch &\n");
         put(DRCROSTasks.VRC3, "roslaunch atlas_utils vrc_task_3.launch &\n");
      }
   };
   
   public static String getDefaultCommand(DRCROSTasks drcTask)
   {
      return defaultTaskPath.get(drcTask);
   }
}
