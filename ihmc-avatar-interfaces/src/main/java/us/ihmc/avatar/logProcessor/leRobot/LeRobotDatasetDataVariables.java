package us.ihmc.avatar.logProcessor.leRobot;

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

import java.util.ArrayList;
import java.util.List;

/**
 * Handles writing the non-visual robot data from YoVariables into Apache Parquet files.
 * <p>
 * Part of the {@link LeRobotDataset} generation system from IHMC logs.
 * <p>
 * TODO: If using hand poses, transform them to walking frame or something
 */
public class LeRobotDatasetDataVariables
{
   private final LeRobotDatasetEpisode episode;

   private final YoPose3D pelvisPoseCurrent;
   private final YoPose3D pelvisPoseDesired;
   private final SideDependentList<YoPose3D> handPosesCurrent = new SideDependentList<>();
   private final SideDependentList<YoPose3D> handPosesDesired = new SideDependentList<>();
   private final SideDependentList<YoPose3D> forearmPosesCurrent = new SideDependentList<>();
   private final SideDependentList<YoPose3D> forearmPosesDesired = new SideDependentList<>();
   private final MutableReferenceFrame pelvisFrame = new MutableReferenceFrame("pelvisFrame");
   private final FramePose3D framePose = new FramePose3D();
   private final YoRegistry rootRegistry;

   public LeRobotDatasetDataVariables(LeRobotDatasetEpisode episode, SCS2LogSessionWithVideo session)
   {
      this.episode = episode;

      rootRegistry = session.getRootRegistry();

      RobotDefinition robotDefinition = session.getRobotDefinitions().get(0);

      pelvisPoseCurrent = findYoPose("pelvis", "Current");
      pelvisPoseDesired = findYoPose("pelvis", "Desired");

      SideDependentList<String> robotHandNames = LeRobotDatasetTools.getRobotLinkNames(robotDefinition, "wrist_yaw");
      for (RobotSide side : robotHandNames.sides())
      {
         handPosesCurrent.put(side, findYoPose(robotHandNames.get(side), "Current"));
         handPosesDesired.put(side, findYoPose(robotHandNames.get(side), "Desired"));
      }
      SideDependentList<String> robotForearmNames = LeRobotDatasetTools.getRobotLinkNames(robotDefinition, "wrist_roll");
      for (RobotSide side : robotHandNames.sides())
      {
         forearmPosesCurrent.put(side, findYoPose(robotForearmNames.get(side), "Current"));
         forearmPosesDesired.put(side, findYoPose(robotForearmNames.get(side), "Desired"));
      }
   }

   public YoPose3D findYoPose(String linkName, String qualifier)
   {
      String kstModule = LeRobotDatasetTools.findRegistry(rootRegistry, "root.main", "IKStreamingRTThread");
      String kstController = kstModule + "KinematicsStreamingToolboxController.HumanoidKinematicsToolboxController.";
      String feedbackController = kstController + "WholeBodyControllerCore.WholeBodyFeedbackController.FeedbackControllerToolbox.";
      if (rootRegistry.findVariable("%s%s%sPositionX".formatted(feedbackController, linkName, qualifier)) instanceof YoDouble xVariable
       && rootRegistry.findVariable("%s%s%sPositionY".formatted(feedbackController, linkName, qualifier)) instanceof YoDouble yVariable
       && rootRegistry.findVariable("%s%s%sPositionZ".formatted(feedbackController, linkName, qualifier)) instanceof YoDouble zVariable
       && rootRegistry.findVariable("%s%s%sOrientationQx".formatted(feedbackController, linkName, qualifier)) instanceof YoDouble qxVariable
       && rootRegistry.findVariable("%s%s%sOrientationQy".formatted(feedbackController, linkName, qualifier)) instanceof YoDouble qyVariable
       && rootRegistry.findVariable("%s%s%sOrientationQz".formatted(feedbackController, linkName, qualifier)) instanceof YoDouble qzVariable
       && rootRegistry.findVariable("%s%s%sOrientationQs".formatted(feedbackController, linkName, qualifier)) instanceof YoDouble qsVariable)
         return new YoPose3D(new YoPoint3D(xVariable, yVariable, zVariable), new YoQuaternion(qxVariable, qyVariable, qzVariable, qsVariable));
      else
      {
         LogTools.error("Could not find YoPose3D");
         return null;
      }
   }

   public void addFrame(long timestampMicros, LeRobotDatasetEpisodeStatistics statistics, int logPosition, String logName)
   {
      List<Float> state = new ArrayList<>();
      List<Float> action = new ArrayList<>();

      for (RobotSide side : handPosesCurrent.sides())
      {
         pelvisFrame.update(pelvisPoseCurrent::get);
         framePose.setIncludingFrame(ReferenceFrame.getWorldFrame(), handPosesCurrent.get(side));
         framePose.changeFrame(pelvisFrame.getReferenceFrame());
         state.add((float) framePose.getX());
         state.add((float) framePose.getY());
         state.add((float) framePose.getZ());
         state.add((float) framePose.getOrientation().getX());
         state.add((float) framePose.getOrientation().getY());
         state.add((float) framePose.getOrientation().getZ());
         state.add((float) framePose.getOrientation().getS());
         framePose.setIncludingFrame(ReferenceFrame.getWorldFrame(), forearmPosesCurrent.get(side));
         framePose.changeFrame(pelvisFrame.getReferenceFrame());
         state.add((float) framePose.getX());
         state.add((float) framePose.getY());
         state.add((float) framePose.getZ());
         state.add((float) framePose.getOrientation().getX());
         state.add((float) framePose.getOrientation().getY());
         state.add((float) framePose.getOrientation().getZ());
         state.add((float) framePose.getOrientation().getS());

         pelvisFrame.update(pelvisPoseDesired::get);
         framePose.setIncludingFrame(ReferenceFrame.getWorldFrame(), handPosesDesired.get(side));
         framePose.changeFrame(pelvisFrame.getReferenceFrame());
         action.add((float) framePose.getX());
         action.add((float) framePose.getY());
         action.add((float) framePose.getZ());
         action.add((float) framePose.getOrientation().getX());
         action.add((float) framePose.getOrientation().getY());
         action.add((float) framePose.getOrientation().getZ());
         action.add((float) framePose.getOrientation().getS());
         framePose.setIncludingFrame(ReferenceFrame.getWorldFrame(), forearmPosesDesired.get(side));
         framePose.changeFrame(pelvisFrame.getReferenceFrame());
         action.add((float) framePose.getX());
         action.add((float) framePose.getY());
         action.add((float) framePose.getZ());
         action.add((float) framePose.getOrientation().getX());
         action.add((float) framePose.getOrientation().getY());
         action.add((float) framePose.getOrientation().getZ());
         action.add((float) framePose.getOrientation().getS());
      }

      float timestamp = timestampMicros / 1e6f; // in seconds, beginning of episode is 0.0 s
      int taskIndex = 0; // We're only training one task at a time for now
      boolean isLastFrame = false;
      LeRobotEpisodeRecord record = new LeRobotEpisodeRecord(state,
                                                             action,
                                                             episode.getEpisodeIndex(),
                                                             episode.getRecords().size(),
                                                             timestamp,
                                                             logPosition,
                                                             logName,
                                                             isLastFrame,
                                                             episode.getDataset().getTotalEpisodeFrames() + episode.getRecords().size(),
                                                             taskIndex);
      statistics.processParquetRecord(record);
      episode.getRecords().add(record);
   }
}
