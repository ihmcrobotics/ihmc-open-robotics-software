package us.ihmc.darpaRoboticsChallenge.processManagement;

public class GazeboProcessManagementCommandStrings
{
   public static final String GET_ROSLAUNCH_PID="ps aux | grep roslaunch | grep -v grep | awk '{ print $2 }'";
   public static final String GET_ROSLAUNCH_TASK="ps aux | grep roslaunch | grep -v grep | awk ' { print $(NF-0) } ' | cut -d '/' -f2 | cut -d '.' -f1";
   public static final String KILL_RUNNING_ROS_SIM="kill -2 `ps aux | grep roslaunch | grep -v grep | awk '{ print $2 }'`";
   
   public static final String GET_SCS_PID="ps aux | grep java | grep ThirdParty | grep -v grep | awk '{ print $2 }'";
   public static final String FORCE_KILL_REMOTE_SCS="kill -9 `ps aux | grep java | grep ThirdParty | grep -v grep | awk '{ print $2 }'`";
}
