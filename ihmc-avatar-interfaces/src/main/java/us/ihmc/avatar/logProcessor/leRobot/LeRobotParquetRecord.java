package us.ihmc.avatar.logProcessor.leRobot;

import com.jerolba.carpet.annotation.Alias;

import java.util.List;

/**
 * Parquet-on-disk record for LeRobot v3.0 dataset files.
 * Contains only the standard lerobot columns; IHMC-specific fields
 * (log_position, log_name) are kept in-memory via {@link LeRobotEpisodeRecord}
 * but not persisted to parquet so that the schema matches what lerobot expects.
 */
public record LeRobotParquetRecord(
      @Alias("observation.state") List<Float> state,
      List<Float> action,
      long episodeIndex,
      long frameIndex,
      float timestamp,
      @Alias("next.done") boolean nextDone,
      long index,
      long taskIndex)
{}
