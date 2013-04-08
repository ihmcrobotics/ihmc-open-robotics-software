package us.ihmc.darpaRoboticsChallenge;

import java.util.HashMap;

public class DRCDashboardTypes {
	
	public enum DRCTask
	{
		QUAL1, QUAL2, QUAL3, QUAL4, VRC1, VRC2, VRC3, DRIVING_TASK1, DRIVING_TEST_TRACK;
	}
	
	private static final HashMap<DRCTask, String> taskPath = new HashMap<DRCTask, String>()
	{
		private static final long serialVersionUID = 7295926598139266176L;

		{
			put(DRCTask.QUAL1, "cd ~/workspace/GazeboStateCommunicator/launch/\n./qual1.sh\n");
			put(DRCTask.QUAL2, "cd ~/workspace/GazeboStateCommunicator/launch/\n./qual2.sh\n");
			put(DRCTask.QUAL3, "cd ~/workspace/GazeboStateCommunicator/launch/\n./qual3.sh\n");
			put(DRCTask.QUAL4, "cd ~/workspace/GazeboStateCommunicator/launch/\n./qual4.sh\n");
			put(DRCTask.VRC1, "cd ~/workspace/GazeboStateCommunicator/launch/\n./task1.sh\n");
			put(DRCTask.VRC2, "cd ~/workspace/GazeboStateCommunicator/launch/\n./task2.sh\n");
			put(DRCTask.VRC3, "cd ~/workspace/GazeboStateCommunicator/launch/\n./task3.sh\n");
			put(DRCTask.DRIVING_TASK1, "cd ~/workspace/GazeboStateCommunicator/launch/\n./drivingTask1.sh\n");
			put(DRCTask.DRIVING_TEST_TRACK, "cd ~/workspace/GazeboStateCommunicator/launch/\n./drivingTestTrack.sh\n");
		}
	};
	
	public static String getDRCTaskLaunchScriptPath(DRCTask drcTask)
	{
		return taskPath.get(drcTask);
	}
	
	public enum RobotModel
	{
		ATLAS_NO_HANDS, ATLAS_SANDIA_HANDS;
	}
}
