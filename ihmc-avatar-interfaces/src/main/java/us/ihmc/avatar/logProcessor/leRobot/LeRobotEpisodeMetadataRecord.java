package us.ihmc.avatar.logProcessor.leRobot;

import com.jerolba.carpet.annotation.Alias;

import java.util.List;

/**
 * Per-episode record written to meta/episodes/chunk-000/file-000.parquet in v3.0 format.
 *
 * lerobot v3 requires per-episode chunk/file indices (always 0 for single-shard datasets)
 * and from/to timestamps that tell the video decoder which portion of the concatenated
 * file-000.mp4 corresponds to each episode.
 *
 * Column names that contain slashes are mapped via {@link Alias}.
 */
public record LeRobotEpisodeMetadataRecord(
      long episodeIndex,
      List<String> tasks,
      long length,
      long datasetFromIndex,
      long datasetToIndex,
      @Alias("data/chunk_index") long dataChunkIndex,
      @Alias("data/file_index") long dataFileIndex,
      @Alias("videos/observation.images.cam_zed_left/chunk_index") long camZedLeftChunkIndex,
      @Alias("videos/observation.images.cam_zed_left/file_index") long camZedLeftFileIndex,
      @Alias("videos/observation.images.cam_zed_left/from_timestamp") double camZedLeftFromTimestamp,
      @Alias("videos/observation.images.cam_zed_left/to_timestamp") double camZedLeftToTimestamp,
      @Alias("videos/observation.images.cam_zed_right/chunk_index") long camZedRightChunkIndex,
      @Alias("videos/observation.images.cam_zed_right/file_index") long camZedRightFileIndex,
      @Alias("videos/observation.images.cam_zed_right/from_timestamp") double camZedRightFromTimestamp,
      @Alias("videos/observation.images.cam_zed_right/to_timestamp") double camZedRightToTimestamp)
{}
