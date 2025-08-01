package us.ihmc.avatar.logProcessor.leRobot;

import com.jerolba.carpet.CarpetWriter;
import com.jerolba.carpet.ColumnNamingStrategy;
import org.apache.commons.lang.StringUtils;
import us.ihmc.avatar.scs2.SCS2LogSessionWithVideo;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.robot.RobotDefinition;
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
 * TODO: If using hand poses, transform them to walking frame or something
 */
public class LeRobotDatasetDataWriter
{
   public static final boolean USE_HAND_POSES = false;

   private final List<LeRobotEpisodeRecord> records = new ArrayList<>();
   private record StateVariables(YoDouble[] current, YoDouble[] desired) { }
   private final SideDependentList<StateVariables> stateVariables = new SideDependentList<>();
   private final long episodeIndex;
   private final YoRegistry rootRegistry;
   private final long datasetLengthSoFar;

   public LeRobotDatasetDataWriter(long episodeIndex, long datasetLengthSoFar, SCS2LogSessionWithVideo session)
   {
      this.episodeIndex = episodeIndex;
      this.datasetLengthSoFar = datasetLengthSoFar;

      rootRegistry = session.getRootRegistry();

      RobotDefinition robotDefinition = session.getRobotDefinitions().get(0);

      if (USE_HAND_POSES)
      {
         SideDependentList<String> robotHandNames = LeRobotDatasetTools.getRobotHandNames(robotDefinition);
         for (RobotSide side : robotHandNames.sides())
         {
            YoPose3D current = findYoPose(robotHandNames.get(side), "Current", rootRegistry);
            YoPose3D desired = findYoPose(robotHandNames.get(side), "Desired", rootRegistry);
            YoDouble[] currentState = new YoDouble[] {
                  current.getYoX(), current.getYoY(), current.getYoZ(),
                  current.getYoQx(), current.getYoQy(), current.getYoQz(), current.getYoQs()
            };
            YoDouble[] desiredState = new YoDouble[] {
                  desired.getYoX(), desired.getYoY(), desired.getYoZ(),
                  desired.getYoQx(), desired.getYoQy(), desired.getYoQz(), desired.getYoQs()
            };
            stateVariables.put(side, new StateVariables(currentState, desiredState));
         }
      }
      else
      {
         SideDependentList<List<String>> robotArmJointNames = LeRobotDatasetTools.getRobotArmJointNames(robotDefinition);
         for (RobotSide side : robotArmJointNames.sides())
         {
            String kstModule = LeRobotDatasetTools.findRegistry(rootRegistry, "root.main", "KinematicsStreamingToolboxModule");
            String kstController = kstModule + "KinematicsStreamingToolboxController.HumanoidKinematicsToolboxController.";
            String capitalizedRobotName = StringUtils.capitalize(session.getLogProperties().getModel().getNameAsString());
            String hwPosition = kstModule + "%sROS2HardwareCommunication.".formatted(capitalizedRobotName);
            String ikSolver = kstController + "WholeBodyControllerCore.WholeBodyInverseKinematicsSolver.";

            List<String> armJointNames = robotArmJointNames.get(side);
            YoDouble[] currentState = new YoDouble[armJointNames.size()];
            YoDouble[] desiredState = new YoDouble[armJointNames.size()];

            for (int i = 0; i < armJointNames.size(); i++)
               if (rootRegistry.findVariable("%sMotorState_Position_%s_%s".formatted(hwPosition,
                                                                                     armJointNames.get(i),
                                                                                     capitalizedRobotName)) instanceof YoDouble variable)
                  currentState[i] = variable;
            for (int i = 0; i < armJointNames.size(); i++)
               if (rootRegistry.findVariable("%sq_qp_%s".formatted(ikSolver, armJointNames.get(i))) instanceof YoDouble variable)
                  desiredState[i] = variable;

            stateVariables.put(side, new StateVariables(currentState, desiredState));
         }
      }
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
         for (int i = 0; i < stateVariables.get(side).current().length; i++)
            state.add((float) stateVariables.get(side).current()[i].getDoubleValue());
         for (int i = 0; i < stateVariables.get(side).desired().length; i++)
            state.add((float) stateVariables.get(side).desired()[i].getDoubleValue());
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
