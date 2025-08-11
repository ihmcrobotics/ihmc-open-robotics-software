package us.ihmc.avatar.logProcessor.leRobot;

import com.jerolba.carpet.CarpetWriter;
import com.jerolba.carpet.ColumnNamingStrategy;
import org.apache.commons.lang.StringUtils;
import us.ihmc.avatar.scs2.SCS2LogSessionWithVideo;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
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
   public static final boolean USE_HAND_POSES = true;

   private final List<LeRobotEpisodeRecord> records = new ArrayList<>();
   private YoPose3D pelvisPoseCurrent;
   private YoPose3D pelvisPoseDesired;
   private final SideDependentList<YoPose3D> handPosesCurrent = new SideDependentList<>();
   private final SideDependentList<YoPose3D> handPosesDesired = new SideDependentList<>();
   private final SideDependentList<YoDouble[]> jointAnglesCurrent = new SideDependentList<>();
   private final SideDependentList<YoDouble[]> jointAnglesDesired = new SideDependentList<>();
   private final MutableReferenceFrame pelvisFrame = new MutableReferenceFrame("pelvisFrame");
   private final FramePose3D handFramePose = new FramePose3D();
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
         pelvisPoseCurrent = findYoPose("pelvis", "Current", rootRegistry);
         pelvisPoseDesired = findYoPose("pelvis", "Desired", rootRegistry);

         SideDependentList<String> robotHandNames = LeRobotDatasetTools.getRobotHandNames(robotDefinition);
         for (RobotSide side : robotHandNames.sides())
         {
            handPosesCurrent.put(side, findYoPose(robotHandNames.get(side), "Current", rootRegistry));
            handPosesDesired.put(side, findYoPose(robotHandNames.get(side), "Desired", rootRegistry));
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

            jointAnglesCurrent.put(side, currentState);
            jointAnglesDesired.put(side, desiredState);
         }
      }
   }

   public static YoPose3D findYoPose(String linkName, String qualifier, YoRegistry rootRegistry)
   {
      String kstModule = LeRobotDatasetTools.findRegistry(rootRegistry, "root.main", "KinematicsStreamingToolboxModule");
      String kstController = kstModule + "KinematicsStreamingToolboxController.HumanoidKinematicsToolboxController.";
      String feedbackController = kstController + "WholeBodyControllerCore.WholeBodyFeedbackController.FeedbackControllerToolbox.";
      if (rootRegistry.findVariable("%s%s%sPositionX".formatted(feedbackController, linkName, qualifier)) instanceof YoDouble xVariable && rootRegistry.findVariable("%s%s%sPositionY".formatted(feedbackController, linkName, qualifier)) instanceof YoDouble yVariable
          && rootRegistry.findVariable("%s%s%sPositionZ".formatted(feedbackController, linkName, qualifier)) instanceof YoDouble zVariable && rootRegistry.findVariable("%s%s%sOrientationQx".formatted(feedbackController, linkName, qualifier)) instanceof YoDouble qxVariable
          && rootRegistry.findVariable("%s%s%sOrientationQy".formatted(feedbackController, linkName, qualifier)) instanceof YoDouble qyVariable && rootRegistry.findVariable("%s%s%sOrientationQz".formatted(feedbackController, linkName, qualifier)) instanceof YoDouble qzVariable
          && rootRegistry.findVariable("%s%s%sOrientationQs".formatted(feedbackController, linkName, qualifier)) instanceof YoDouble qsVariable)
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

      if (USE_HAND_POSES)
      {
         for (RobotSide side : handPosesCurrent.sides())
         {
            pelvisFrame.update(pelvisPoseCurrent::get);
            handFramePose.setIncludingFrame(ReferenceFrame.getWorldFrame(), handPosesCurrent.get(side));
            handFramePose.changeFrame(pelvisFrame.getReferenceFrame());
            state.add((float) handFramePose.getX());
            state.add((float) handFramePose.getY());
            state.add((float) handFramePose.getZ());
            state.add((float) handFramePose.getOrientation().getX());
            state.add((float) handFramePose.getOrientation().getY());
            state.add((float) handFramePose.getOrientation().getZ());
            state.add((float) handFramePose.getOrientation().getS());

            pelvisFrame.update(pelvisPoseDesired::get);
            handFramePose.setIncludingFrame(ReferenceFrame.getWorldFrame(), handPosesDesired.get(side));
            handFramePose.changeFrame(pelvisFrame.getReferenceFrame());
            action.add((float) handFramePose.getX());
            action.add((float) handFramePose.getY());
            action.add((float) handFramePose.getZ());
            action.add((float) handFramePose.getOrientation().getX());
            action.add((float) handFramePose.getOrientation().getY());
            action.add((float) handFramePose.getOrientation().getZ());
            action.add((float) handFramePose.getOrientation().getS());
         }
      }
      else
      {
         for (RobotSide side : jointAnglesCurrent.sides())
         {
            for (int i = 0; i < jointAnglesCurrent.get(side).length; i++)
               state.add((float) jointAnglesCurrent.get(side)[i].getDoubleValue());
            for (int i = 0; i < jointAnglesDesired.get(side).length; i++)
               action.add((float) jointAnglesDesired.get(side)[i].getDoubleValue());
         }
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
