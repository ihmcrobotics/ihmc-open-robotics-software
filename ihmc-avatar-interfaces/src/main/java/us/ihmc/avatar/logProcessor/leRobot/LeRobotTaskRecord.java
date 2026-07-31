package us.ihmc.avatar.logProcessor.leRobot;

/**
 * Task record written to meta/tasks.parquet in the LeRobot v3.0 dataset format.
 * <p>
 * Part of the {@link LeRobotDataset} generation system from IHMC logs.
 */
public record LeRobotTaskRecord(long taskIndex, String task) {}
