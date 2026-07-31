package us.ihmc.avatar.logProcessor.leRobot;

import us.ihmc.avatar.scs2.SCS2LogSessionWithVideo;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
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
 * State and action are: the wrist/gripper pose of each hand (expressed in the camera frame, read
 * from the KST FeedbackControllerToolbox when available), the two neck joint angles, and the
 * Ability Hand finger joints. Only {@code q1} is recorded for the index/middle/ring/pinky
 * fingers — their {@code q2} is a fixed mechanical mimic of {@code q1} (see the Ability Hand
 * URDF's {@code <mimic>} tags), so it carries no information the dataset needs. The thumb has two
 * independently-actuated joints ({@code q1} rotation, {@code q2} flexion) and both are recorded.
 * Neck action values come from {@code qDesired_NECK_Y/Z}, a genuine commanded-position channel in
 * the log's {@code AlexROS2HardwareCommunication} registry (neck goes through the standard
 * whole-body-controller-core desired-output pipeline). Finger action values are a copy of the
 * measured {@code q_} position, <b>not</b> {@code qDesired_}: the Ability Hand driver
 * ({@code EtherSnacksAbilityHand.write(...)}) never populates the generic desired-output map that
 * feeds {@code qDesired_}, so for Ability Hand joints that variable is a dead placeholder stuck at
 * 0.0 for the whole log (verified against real Lever_VLA log data — every {@code qDesired_*_ability_hand_*}
 * value was exactly 0.0 across 5364 frames / 19 episodes). The Ability Hand's real command values
 * live in a separate, actuator-indexed namespace ({@code <SIDE>_AbilityHand_PositionCommand0-5} /
 * {@code VelocityCommand0-5}, selected by {@code <SIDE>_AbilityHand_ControlMode}); wiring that up
 * properly is future work, not done here.
 * Forearm orientation and spine are intentionally not recorded: forearm orientation is largely
 * redundant with the gripper pose's own orientation, and spine wasn't requested.
 * <p>
 * The camera frame hangs off the torso, whose position is always 0 0 0 — only its orientation
 * is tracked, read from the walking controller's chest taskspace control module (the walking
 * controller doesn't position-control the chest, so no position variables exist for it).
 * <p>
 * Part of the {@link LeRobotDataset} generation system from IHMC logs.
 */
public class LeRobotDatasetDataVariables
{
   /** Neck joints, measured (state) and commanded (action). */
   public static final String[] NECK_JOINT_NAMES = {"NECK_Y", "NECK_Z"};

   /** Ability Hand joints, measured (state) and commanded (action). Mimic {@code q2}s (all but the thumb's) are omitted. */
   public static final String[] FINGER_JOINT_NAMES = {
         "left_ability_hand_index_q1",
         "left_ability_hand_middle_q1",
         "left_ability_hand_ring_q1",
         "left_ability_hand_pinky_q1",
         "left_ability_hand_thumb_q1",
         "left_ability_hand_thumb_q2",
         "right_ability_hand_index_q1",
         "right_ability_hand_middle_q1",
         "right_ability_hand_ring_q1",
         "right_ability_hand_pinky_q1",
         "right_ability_hand_thumb_q1",
         "right_ability_hand_thumb_q2",
   };

   private static final String[] POSE_SUFFIXES = {"x", "y", "z", "qx", "qy", "qz", "qs"};

   public static final int ACTION_SIZE = 2 * POSE_SUFFIXES.length + NECK_JOINT_NAMES.length + FINGER_JOINT_NAMES.length;
   public static final int STATE_SIZE = ACTION_SIZE;

   private final LeRobotDatasetEpisode episode;

   private final YoRegistry rootRegistry;
   private final List<YoRegistry> feedbackToolboxes;
   private final String kstFeedbackController;
   private final YoQuaternion torsoOrientationCurrent;
   private final SideDependentList<YoPose3D> handPosesCurrent = new SideDependentList<>();
   private final SideDependentList<YoPose3D> handPosesDesired = new SideDependentList<>();
   private final List<YoDouble> neckMeasured = new ArrayList<>();
   private final List<YoDouble> neckDesired = new ArrayList<>();
   private final List<YoDouble> fingerPositions = new ArrayList<>();
   private final MutableReferenceFrame torsoFrame = new MutableReferenceFrame("torsoFrame");
   private final ReferenceFrame cameraFrame;
   private final FramePose3D framePose = new FramePose3D();

   public LeRobotDatasetDataVariables(LeRobotDatasetEpisode episode,
                                      SCS2LogSessionWithVideo session,
                                      HumanoidJointNameMap jointMap,
                                      HumanoidRobotSensorInformation sensorInformation)
   {
      this.episode = episode;
      rootRegistry = session.getRootRegistry();

      String kstModule = LeRobotDatasetTools.findRegistry(rootRegistry, "root.main", "IKStreamingRTThread");
      if (kstModule == null)
      {
         LogTools.warn("Could not find IKStreamingRTThread registry in the log; using walking-controller and hardware fallbacks");
         kstFeedbackController = null;
      }
      else
      {
         String kstController = kstModule + "KinematicsStreamingToolboxController.HumanoidKinematicsToolboxController.";
         kstFeedbackController = kstController + "WholeBodyControllerCore.WholeBodyFeedbackController.FeedbackControllerToolbox.";
      }

      feedbackToolboxes = rootRegistry.collectSubtreeRegistries().stream()
                                      .filter(r -> r.getName().equals("FeedbackControllerToolbox"))
                                      .toList();
      if (feedbackToolboxes.isEmpty())
         LogTools.error("Could not find any FeedbackControllerToolbox registry in the log");

      torsoOrientationCurrent = findTorsoOrientation(rootRegistry, jointMap.getChestName());
      cameraFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("cameraFrame",
                                                                                      torsoFrame.getReferenceFrame(),
                                                                                      sensorInformation.getExperimentalCameraTransform());

      for (RobotSide side : RobotSide.values)
      {
         handPosesCurrent.put(side, findYoPose(jointMap.getHandName(side), "Current"));
         handPosesDesired.put(side, findYoPose(jointMap.getHandName(side), "Desired"));
      }

      YoRegistry hardwareCommunication = rootRegistry.collectSubtreeRegistries().stream()
                                             .filter(r -> r.getName().equals("AlexROS2HardwareCommunication"))
                                             .findFirst()
                                             .orElse(null);
      if (hardwareCommunication == null)
         LogTools.warn("Could not find AlexROS2HardwareCommunication registry");

      for (String joint : NECK_JOINT_NAMES)
      {
         YoDouble measured = findHardwareVariable(hardwareCommunication, "q_" + joint);
         if (measured == null)
            LogTools.warn("Could not find q_{} in AlexROS2HardwareCommunication", joint);
         neckMeasured.add(measured);

         YoDouble desired = findHardwareVariable(hardwareCommunication, "qDesired_" + joint);
         if (desired == null)
            LogTools.warn("Could not find qDesired_{} in AlexROS2HardwareCommunication", joint);
         neckDesired.add(desired);
      }

      for (String joint : FINGER_JOINT_NAMES)
      {
         YoDouble q = findHardwareVariable(hardwareCommunication, "q_" + joint);
         if (q == null)
            LogTools.warn("Could not find q_{} in AlexROS2HardwareCommunication", joint);
         fingerPositions.add(q);
      }
   }

   private YoDouble findHardwareVariable(YoRegistry hardwareCommunication, String variableName)
   {
      return hardwareCommunication != null && hardwareCommunication.getVariable(variableName) instanceof YoDouble d ? d : null;
   }

   private YoPose3D findYoPose(String linkName, String qualifier)
   {
      YoPose3D pose = findYoPoseQuiet(linkName, qualifier);
      if (pose == null)
         LogTools.error("Could not find {}{} pose in KST or any FeedbackControllerToolbox", linkName, qualifier);
      return pose;
   }

   private YoPose3D findYoPoseQuiet(String linkName, String qualifier)
   {
      if (linkName == null)
         return null;

      YoPose3D kstPose = findYoPose(rootRegistry, kstFeedbackController, linkName, qualifier);
      if (kstPose != null)
         return kstPose;

      for (YoRegistry toolbox : feedbackToolboxes)
      {
         YoPose3D pose = findYoPose(toolbox, "", linkName, qualifier);
         if (pose != null)
            return pose;
      }

      return null;
   }

   private YoPose3D findYoPose(YoRegistry registry, String prefix, String linkName, String qualifier)
   {
      if (registry == null || prefix == null)
         return null;

      if (registry.findVariable("%s%s%sPositionX".formatted(prefix, linkName, qualifier)) instanceof YoDouble xVariable
       && registry.findVariable("%s%s%sPositionY".formatted(prefix, linkName, qualifier)) instanceof YoDouble yVariable
       && registry.findVariable("%s%s%sPositionZ".formatted(prefix, linkName, qualifier)) instanceof YoDouble zVariable
       && registry.findVariable("%s%s%sOrientationQx".formatted(prefix, linkName, qualifier)) instanceof YoDouble qxVariable
       && registry.findVariable("%s%s%sOrientationQy".formatted(prefix, linkName, qualifier)) instanceof YoDouble qyVariable
       && registry.findVariable("%s%s%sOrientationQz".formatted(prefix, linkName, qualifier)) instanceof YoDouble qzVariable
       && registry.findVariable("%s%s%sOrientationQs".formatted(prefix, linkName, qualifier)) instanceof YoDouble qsVariable)
         return new YoPose3D(new YoPoint3D(xVariable, yVariable, zVariable), new YoQuaternion(qxVariable, qyVariable, qzVariable, qsVariable));

      return null;
   }

   /**
    * The walking controller's chest taskspace control module has the measured chest orientation as
    * &lt;chest&gt;TaskspaceOrientationCurrentQ*; the FeedbackControllerToolbox's &lt;chest&gt;DesiredOrientationQ*
    * is the fallback (the chest tracks its desired closely).
    */
   private YoQuaternion findTorsoOrientation(YoRegistry root, String chestName)
   {
      YoRegistry taskspaceModule = root.collectSubtreeRegistries().stream()
                                       .filter(r -> r.getName().equals(chestName + "TaskspaceControlModule"))
                                       .findFirst()
                                       .orElse(null);
      if (taskspaceModule != null
       && taskspaceModule.getVariable("%sTaskspaceOrientationCurrentQx".formatted(chestName)) instanceof YoDouble qxVariable
       && taskspaceModule.getVariable("%sTaskspaceOrientationCurrentQy".formatted(chestName)) instanceof YoDouble qyVariable
       && taskspaceModule.getVariable("%sTaskspaceOrientationCurrentQz".formatted(chestName)) instanceof YoDouble qzVariable
       && taskspaceModule.getVariable("%sTaskspaceOrientationCurrentQs".formatted(chestName)) instanceof YoDouble qsVariable)
         return new YoQuaternion(qxVariable, qyVariable, qzVariable, qsVariable);

      for (YoRegistry toolbox : feedbackToolboxes)
      {
         if (toolbox.getVariable("%sDesiredOrientationQx".formatted(chestName)) instanceof YoDouble qxVariable
          && toolbox.getVariable("%sDesiredOrientationQy".formatted(chestName)) instanceof YoDouble qyVariable
          && toolbox.getVariable("%sDesiredOrientationQz".formatted(chestName)) instanceof YoDouble qzVariable
          && toolbox.getVariable("%sDesiredOrientationQs".formatted(chestName)) instanceof YoDouble qsVariable)
            return new YoQuaternion(qxVariable, qyVariable, qzVariable, qsVariable);
      }

      LogTools.error("Could not find {} orientation in the taskspace control module or any FeedbackControllerToolbox", chestName);
      return null;
   }

   public static List<String> getActionFeatureNames()
   {
      List<String> names = new ArrayList<>();
      for (RobotSide side : RobotSide.values)
         for (String suffix : POSE_SUFFIXES)
            names.add("%s_gripper_%s".formatted(side.getLowerCaseName(), suffix));
      for (String joint : NECK_JOINT_NAMES)
         names.add(joint.toLowerCase());
      for (String joint : FINGER_JOINT_NAMES)
         names.add(joint);
      return names;
   }

   public static List<String> getStateFeatureNames()
   {
      return getActionFeatureNames();
   }

   public void addFrame(double timestamp, LeRobotDatasetEpisodeStatistics statistics, int logPosition, String logName)
   {
      if (torsoOrientationCurrent == null)
         return;

      List<Float> state  = new ArrayList<>();
      List<Float> action = new ArrayList<>();

      torsoFrame.update(transform ->
      {
         transform.getTranslation().setToZero(); // The torso is always at 0 0 0; only its orientation is tracked
         transform.getRotation().set(torsoOrientationCurrent);
      });

      for (RobotSide side : RobotSide.values)
      {
         YoPose3D current = handPosesCurrent.get(side);
         YoPose3D desired = handPosesDesired.get(side);
         if (current == null || desired == null)
            return;

         framePose.setIncludingFrame(ReferenceFrame.getWorldFrame(), current);
         framePose.changeFrame(cameraFrame);
         addPose(state, framePose);

         framePose.setIncludingFrame(ReferenceFrame.getWorldFrame(), desired);
         framePose.changeFrame(cameraFrame);
         addPose(action, framePose);
      }

      for (int i = 0; i < NECK_JOINT_NAMES.length; i++)
      {
         YoDouble measured = neckMeasured.get(i);
         YoDouble desired = neckDesired.get(i);
         if (measured == null || desired == null)
            return;
         state.add((float) measured.getValue());
         action.add((float) desired.getValue());
      }

      for (YoDouble q : fingerPositions)
      {
         if (q == null)
            return;
         state.add((float) q.getValue());
         action.add((float) q.getValue());
      }

      int taskIndex = 0;
      boolean isLastFrame = false;
      LeRobotEpisodeRecord record = new LeRobotEpisodeRecord(state,
                                                             action,
                                                             episode.getEpisodeIndex(),
                                                             episode.getRecords().size(),
                                                             (float) timestamp,
                                                             logPosition,
                                                             logName,
                                                             isLastFrame,
                                                             episode.getDataset().getTotalEpisodeFrames(),
                                                             taskIndex);
      statistics.processParquetRecord(record);
      episode.getRecords().add(record);
   }

   private void addPose(List<Float> values, FramePose3D pose)
   {
      values.add((float) pose.getX());
      values.add((float) pose.getY());
      values.add((float) pose.getZ());
      values.add((float) pose.getOrientation().getX());
      values.add((float) pose.getOrientation().getY());
      values.add((float) pose.getOrientation().getZ());
      values.add((float) pose.getOrientation().getS());
   }
}
