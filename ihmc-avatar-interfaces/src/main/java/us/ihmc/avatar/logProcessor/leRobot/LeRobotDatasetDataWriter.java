package us.ihmc.avatar.logProcessor.leRobot;

import com.jerolba.carpet.CarpetWriter;
import com.jerolba.carpet.ColumnNamingStrategy;
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

public class LeRobotDatasetDataWriter
{
   private final List<LeRobotEpisodeRecord> records = new ArrayList<>();

   private final YoRegistry yoRegistry;
   private final String feedbackController;
   record EndEffectorVariables(YoPose3D current,YoPose3D desired) { }
   private final SideDependentList<EndEffectorVariables> endEffectorVariables = new SideDependentList<>();
   private final long episodeIndex;
   private final long datasetLengthSoFar;

   public LeRobotDatasetDataWriter(long episodeIndex, long datasetLengthSoFar, YoRegistry yoRegistry)
   {
      this.episodeIndex = episodeIndex;
      this.datasetLengthSoFar = datasetLengthSoFar;
      this.yoRegistry = yoRegistry;

      String highLevelController = "root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.";
      String wbcc = highLevelController + "HighLevelHumanoidControllerFactory.WholeBodyControllerCoreFactory.WholeBodyControllerCore.";
      feedbackController = wbcc + "WholeBodyFeedbackController.FeedbackControllerToolbox.";

      for (RobotSide side : RobotSide.values)
         endEffectorVariables.put(side, new EndEffectorVariables(findYoPose(side, "Current"), findYoPose(side, "Desired")));
   }

   private YoPose3D findYoPose(RobotSide side, String qualifier)
   {
      if (yoRegistry.findVariable("%s%s_GRIPPER_YAW_LINK%sPositionX".formatted(feedbackController, side.name(), qualifier)) instanceof YoDouble xVariable
       && yoRegistry.findVariable("%s%s_GRIPPER_YAW_LINK%sPositionY".formatted(feedbackController, side.name(), qualifier)) instanceof YoDouble yVariable
       && yoRegistry.findVariable("%s%s_GRIPPER_YAW_LINK%sPositionZ".formatted(feedbackController, side.name(), qualifier)) instanceof YoDouble zVariable
       && yoRegistry.findVariable("%s%s_GRIPPER_YAW_LINK%sOrientationQx".formatted(feedbackController, side.name(), qualifier)) instanceof YoDouble qxVariable
       && yoRegistry.findVariable("%s%s_GRIPPER_YAW_LINK%sOrientationQy".formatted(feedbackController, side.name(), qualifier)) instanceof YoDouble qyVariable
       && yoRegistry.findVariable("%s%s_GRIPPER_YAW_LINK%sOrientationQz".formatted(feedbackController, side.name(), qualifier)) instanceof YoDouble qzVariable
       && yoRegistry.findVariable("%s%s_GRIPPER_YAW_LINK%sOrientationQs".formatted(feedbackController, side.name(), qualifier)) instanceof YoDouble qsVariable)
         return new YoPose3D(new YoPoint3D(xVariable, yVariable, zVariable), new YoQuaternion(qxVariable, qyVariable, qzVariable, qsVariable));
      else
      {
         LogTools.error("Could not find YoPose3D");
         return null;
      }
   }

   public void addFrame(long timestampMicros, long frameIndex)
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
      records.add(new LeRobotEpisodeRecord(state,
                                           action,
                                           episodeIndex,
                                           frameIndex,
                                           timestamp, isLastFrame,
                                           datasetLengthSoFar + frameIndex, taskIndex));
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
                                                               true, // <-- Main thing we're doing here
                                                               last.index(),
                                                               last.taskIndex()));
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
