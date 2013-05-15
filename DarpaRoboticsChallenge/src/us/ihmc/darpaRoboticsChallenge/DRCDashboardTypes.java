package us.ihmc.darpaRoboticsChallenge;

import java.util.LinkedHashMap;

public class DRCDashboardTypes
{

   public enum DRCTask
   {
      QUAL1, QUAL2, QUAL3, QUAL4, VRC1, VRC2, VRC3/*, DRIVING_TASK1, DRIVING_TASK1_WITH_EXTERNAL_CAMS, DRIVING_TEST_TRACK_WITH_EXTERNAL_CAMS*/;
   }

   private static final LinkedHashMap<DRCTask, String> pluginTaskPath = new LinkedHashMap<DRCTask, String>()
   {
      private static final long serialVersionUID = 7295926598139266176L;

      {
         put(DRCTask.QUAL1, "cd ~/workspace/GazeboStateCommunicator/launch/; ./qual1.sh\n");
         put(DRCTask.QUAL2, "cd ~/workspace/GazeboStateCommunicator/launch/; ./qual2.sh\n");
         put(DRCTask.QUAL3, "cd ~/workspace/GazeboStateCommunicator/launch/; ./qual3.sh\n");
         put(DRCTask.QUAL4, "cd ~/workspace/GazeboStateCommunicator/launch/; ./qual4.sh\n");
         put(DRCTask.VRC1, "cd ~/workspace/GazeboStateCommunicator/launch/; ./task1.sh\n");
         put(DRCTask.VRC2, "cd ~/workspace/GazeboStateCommunicator/launch/; ./task2.sh\n");
         put(DRCTask.VRC3, "cd ~/workspace/GazeboStateCommunicator/launch/; ./task3.sh\n");
//         put(DRCTask.DRIVING_TASK1, "roslaunch atlas_utils vrc_task_1.launch\n");
//         put(DRCTask.DRIVING_TASK1_WITH_EXTERNAL_CAMS, "cd ~/workspace/GazeboStateCommunicator/launch/; ./drivingTask1.sh\n");
//         put(DRCTask.DRIVING_TEST_TRACK_WITH_EXTERNAL_CAMS, "cd ~/workspace/GazeboStateCommunicator/launch/; ./drivingTestTrack.sh\n");
      }
   };

   public static String getPluginCommand(DRCTask drcTask)
   {
      return pluginTaskPath.get(drcTask);
   }
   
   private static final LinkedHashMap<DRCTask, String> defaultTaskPath = new LinkedHashMap<DRCTask, String>()
   {
      private static final long serialVersionUID = 2304570433813141139L;

      {
         put(DRCTask.QUAL1, "roslaunch atlas_utils qual_task_1.launch\n");
         put(DRCTask.QUAL2, "roslaunch atlas_utils qual_task_2.launch\n");
         put(DRCTask.QUAL3, "roslaunch atlas_utils qual_task_3.launch\n");
         put(DRCTask.QUAL4, "roslaunch atlas_utils qual_task_4.launch\n");
         put(DRCTask.VRC1, "roslaunch atlas_utils vrc_task_1.launch\n");
         put(DRCTask.VRC2, "roslaunch atlas_utils vrc_task_2.launch\n");
         put(DRCTask.VRC3, "roslaunch atlas_utils vrc_task_3.launch\n");
      }
   };
   
   public static String getDefaultCommand(DRCTask drcTask)
   {
      return defaultTaskPath.get(drcTask);
   }
}
