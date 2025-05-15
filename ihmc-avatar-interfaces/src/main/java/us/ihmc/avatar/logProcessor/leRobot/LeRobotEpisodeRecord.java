package us.ihmc.avatar.logProcessor.leRobot;

import com.jerolba.carpet.annotation.Alias;

import java.util.List;

/**
 * Represents a record structure for writing and reading data from the LeRobot parquet files
 * using {@link com.jerolba.carpet.CarpetWriter} and {@link com.jerolba.carpet.CarpetReader}.
 * The fields match the columns in the *.parquet file.
 */
public record LeRobotEpisodeRecord
(
    List<Float> state,
    List<Float> action, // goal position of arm joints
    long episodeIndex, // index of the episode for this sample
    long frameIndex, // index of the frame for this sample in the episode; starts at 0 for each episode
    float timestamp, // in the episode
    int ihmcLogPosition, // in the ihmc log
    @Alias("next.done") boolean nextDone, // true for the last frame of an episode
    long index, // general index in the whole dataset
    long taskIndex // probably 0 mostly, since we only train one task at a time
)
{

}
