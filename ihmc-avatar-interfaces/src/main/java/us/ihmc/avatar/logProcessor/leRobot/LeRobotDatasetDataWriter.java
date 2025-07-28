package us.ihmc.avatar.logProcessor.leRobot;

import com.jerolba.carpet.CarpetWriter;
import com.jerolba.carpet.ColumnNamingStrategy;
import us.ihmc.avatar.scs2.SCS2LogSessionWithVideo;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.euclid.YoPoint3D;
import us.ihmc.yoVariables.euclid.YoPose3D;
import us.ihmc.yoVariables.euclid.YoQuaternion;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.io.OutputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/**
 * Handles writing the non-visual robot data from YoVariables into Apache Parquet files.
 * <p>
 * Part of the {@link LeRobotDataset} generation system from IHMC logs.
 * <p>
 * TODO: Transform the end effector poses to walking frame or something
 */
public class LeRobotDatasetDataWriter
{
   private final List<LeRobotEpisodeRecord> records = new ArrayList<>();

   record EndEffectorVariables(YoPose3D current,YoPose3D desired) { }
   private final SideDependentList<EndEffectorVariables> endEffectorVariables = new SideDependentList<>();
   private final long episodeIndex;
   private final long datasetLengthSoFar;
   private final SCS2LogSessionWithVideo session;

   public LeRobotDatasetDataWriter(long episodeIndex, long datasetLengthSoFar, SCS2LogSessionWithVideo session)
   {
      this.episodeIndex = episodeIndex;
      this.datasetLengthSoFar = datasetLengthSoFar;
      this.session = session;

      YoRegistry rootRegistry = session.getRootRegistry();
      SideDependentList<String> robotHandNames = LeRobotDatasetTools.getRobotHandNames(session.getRobotDefinitions().get(0));
      for (RobotSide side : robotHandNames.sides())
         endEffectorVariables.put(side, new EndEffectorVariables(findYoPose(robotHandNames.get(side), "Current", rootRegistry),
                                                                 findYoPose(robotHandNames.get(side), "Desired", rootRegistry)));
   }

   public static YoPose3D findYoPose(String handName, String qualifier, YoRegistry rootRegistry)
   {
      String kstModule = LeRobotDatasetTools.findRegistry(rootRegistry, "root.main", "KinematicsStreamingToolboxModule");
      String kstController = kstModule + "KinematicsStreamingToolboxController.HumanoidKinematicsToolboxController.";
      if (rootRegistry.findVariable("%s%s%sX".formatted(kstController, handName, qualifier)) instanceof YoDouble xVariable
       && rootRegistry.findVariable("%s%s%sY".formatted(kstController, handName, qualifier)) instanceof YoDouble yVariable
       && rootRegistry.findVariable("%s%s%sZ".formatted(kstController, handName, qualifier)) instanceof YoDouble zVariable
       && rootRegistry.findVariable("%s%s%sQx".formatted(kstController, handName, qualifier)) instanceof YoDouble qxVariable
       && rootRegistry.findVariable("%s%s%sQy".formatted(kstController, handName, qualifier)) instanceof YoDouble qyVariable
       && rootRegistry.findVariable("%s%s%sQz".formatted(kstController, handName, qualifier)) instanceof YoDouble qzVariable
       && rootRegistry.findVariable("%s%s%sQs".formatted(kstController, handName, qualifier)) instanceof YoDouble qsVariable)
         return new YoPose3D(new YoPoint3D(xVariable, yVariable, zVariable), new YoQuaternion(qxVariable, qyVariable, qzVariable, qsVariable));
      else
      {
         LogTools.error("Could not find YoPose3D");
         return null;
      }
   }

   public LeRobotEpisodeRecord addFrame(long timestampMicros, long frameIndex, LeRobotDatasetEpisodeStatistics statistics, int ihmcLogPosition)
   {
      List<Float> state = new ArrayList<>();
      List<Float> action = new ArrayList<>();
      for (RobotSide side : RobotSide.values)
      {
         state.add(endEffectorVariables.get(side).current().getPosition().getX32());
         state.add(endEffectorVariables.get(side).current().getPosition().getY32());
         state.add(endEffectorVariables.get(side).current().getPosition().getZ32());
         state.add(endEffectorVariables.get(side).current().getOrientation().getX32());
         state.add(endEffectorVariables.get(side).current().getOrientation().getY32());
         state.add(endEffectorVariables.get(side).current().getOrientation().getZ32());
         state.add(endEffectorVariables.get(side).current().getOrientation().getS32());
         action.add(endEffectorVariables.get(side).desired().getPosition().getX32());
         action.add(endEffectorVariables.get(side).desired().getPosition().getY32());
         action.add(endEffectorVariables.get(side).desired().getPosition().getZ32());
         action.add(endEffectorVariables.get(side).desired().getOrientation().getX32());
         action.add(endEffectorVariables.get(side).desired().getOrientation().getY32());
         action.add(endEffectorVariables.get(side).desired().getOrientation().getZ32());
         action.add(endEffectorVariables.get(side).desired().getOrientation().getS32());
      }

      float timestamp = timestampMicros / 1e6f; // in seconds, beginning of episode is 0.0 s
      int taskIndex = 0; // We're only training one task at a time for now
      boolean isLastFrame = false;
      LeRobotEpisodeRecord record = new LeRobotEpisodeRecord(state,
                                                             action,
                                                             episodeIndex,
                                                             frameIndex,
                                                             timestamp,
                                                             ihmcLogPosition,
                                                             isLastFrame,
                                                             datasetLengthSoFar + frameIndex,
                                                             taskIndex);
      statistics.processParquetRecord(record);
      records.add(record);
      return record;
   }

   public void writeFile(Path parquetPath)
   {
      // Mark the last frame
      LeRobotEpisodeRecord last = records.get(records.size() - 1);
      records.set(records.size() - 1, new LeRobotEpisodeRecord(last.state(),
                                                               last.action(),
                                                               last.episodeIndex(),
                                                               last.frameIndex(),
                                                               last.timestamp(),
                                                               last.ihmcLogPosition(),
                                                               true, // <-- Main thing we're doing here
                                                               last.index(),
                                                               last.taskIndex()));

      writeParquetFile(parquetPath, records);
   }

   public static void writeParquetFile(Path parquetPath, List<LeRobotEpisodeRecord> records)
   {
      try
      {
         OutputStream outputStream = Files.newOutputStream(parquetPath);
         CarpetWriter<LeRobotEpisodeRecord> writer = new CarpetWriter.Builder<>(outputStream, LeRobotEpisodeRecord.class)
               .withColumnNamingStrategy(ColumnNamingStrategy.SNAKE_CASE).build();

         for (LeRobotEpisodeRecord record : records)
            writer.write(record);

         writer.close();
         outputStream.close();
      }
      catch (Exception e)
      {
         LogTools.error(e.getMessage());
         e.printStackTrace();
      }
   }
}
