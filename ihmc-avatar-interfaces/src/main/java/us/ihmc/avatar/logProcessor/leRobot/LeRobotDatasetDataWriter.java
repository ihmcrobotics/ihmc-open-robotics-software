package us.ihmc.avatar.logProcessor.leRobot;

import com.jerolba.carpet.CarpetWriter;
import com.jerolba.carpet.ColumnNamingStrategy;
import com.jerolba.carpet.annotation.Alias;
import us.ihmc.commons.exception.ExceptionTools;
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
import java.util.Random;

import static us.ihmc.commons.exception.DefaultExceptionHandler.*;

public class LeRobotDatasetDataWriter
{
   record EpisodeRecord(List<Float> state,
                        List<Float> action, // goal position of arm joints
                        long episodeIndex, // index of the episode for this sample
                        long frameIndex, // index of the frame for this sample in the episode; starts at 0 for each episode
                        float timestamp, // in the episode
                        @Alias("next.done") boolean nextDone,
                        long index, // general index in the whole dataset
                        long taskIndex
   ) { }

   private final OutputStream outputStream;
   private final CarpetWriter<EpisodeRecord> carpetWriter;
   private Random random = new Random();

   private final YoRegistry yoRegistry;
   private final String feedbackController;
   record EndEffectorVariables(YoPose3D current,YoPose3D desired) { }
   private final SideDependentList<EndEffectorVariables> endEffectorVariables = new SideDependentList<>();

   public LeRobotDatasetDataWriter(Path parquetPath, YoRegistry yoRegistry)
   {
      this.yoRegistry = yoRegistry;

      outputStream = ExceptionTools.handle(() -> Files.newOutputStream(parquetPath), MESSAGE_AND_STACKTRACE);
      carpetWriter = ExceptionTools.handle(() ->
         new CarpetWriter.Builder<>(outputStream, EpisodeRecord.class).withColumnNamingStrategy(ColumnNamingStrategy.SNAKE_CASE).build()
      , MESSAGE_AND_STACKTRACE);

      String highLevelController = "root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.";
      String wbcc = highLevelController + "HighLevelHumanoidControllerFactory.WholeBodyControllerCoreFactory.WholeBodyControllerCore.";
      feedbackController = wbcc + "WholeBodyFeedbackController.FeedbackControllerToolbox.";

      for (RobotSide side : RobotSide.values)
      {
         endEffectorVariables.put(side, new EndEffectorVariables(findYoPose(side, "Current"), findYoPose(side, "Desired")));
      }
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

   public void write()
   {
      List<Float> observationState = new ArrayList<>();
      List<Float> action = new ArrayList<>();
      for (RobotSide side : RobotSide.values)
      {
         observationState.add(endEffectorVariables.get(side).current().getPosition().getX32());
         observationState.add(endEffectorVariables.get(side).current().getPosition().getY32());
         observationState.add(endEffectorVariables.get(side).current().getPosition().getZ32());
         observationState.add(endEffectorVariables.get(side).current().getOrientation().getX32());
         observationState.add(endEffectorVariables.get(side).current().getOrientation().getY32());
         observationState.add(endEffectorVariables.get(side).current().getOrientation().getZ32());
         observationState.add(endEffectorVariables.get(side).current().getOrientation().getS32());
         action.add(endEffectorVariables.get(side).desired().getPosition().getX32());
         action.add(endEffectorVariables.get(side).desired().getPosition().getY32());
         action.add(endEffectorVariables.get(side).desired().getPosition().getZ32());
         action.add(endEffectorVariables.get(side).desired().getOrientation().getX32());
         action.add(endEffectorVariables.get(side).desired().getOrientation().getY32());
         action.add(endEffectorVariables.get(side).desired().getOrientation().getZ32());
         action.add(endEffectorVariables.get(side).desired().getOrientation().getS32());
      }

      EpisodeRecord value = new EpisodeRecord(observationState,
                                              action,
                                              random.nextLong(),
                                              random.nextLong(),
                                              random.nextFloat(),
                                              random.nextBoolean(),
                                              random.nextLong(),
                                              random.nextLong());

      ExceptionTools.handle(() -> carpetWriter.write(value), MESSAGE_AND_STACKTRACE);
   }

   public void close()
   {
      if (carpetWriter != null)
         ExceptionTools.handle(carpetWriter::close, MESSAGE_AND_STACKTRACE);
      if (outputStream != null)
         ExceptionTools.handle(outputStream::close, MESSAGE_AND_STACKTRACE);
   }
}
