package us.ihmc.rdx.ui.interactable;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.ChestTrajectoryMessage;
import controller_msgs.GoHomeMessage;
import controller_msgs.PelvisHeightTrajectoryMessage;
import controller_msgs.RobotConfigurationData;
import imgui.flag.ImGuiCol;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.tools.WholeBodyCommandLimits;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.UnitConversions;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.algorithms.GeometricJacobianCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.teleoperation.RDXDesiredRobot;
import us.ihmc.rdx.ui.teleoperation.RDXTeleoperationParameters;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.MultiBodySystemMissingTools;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.Set;

/**
 * Sliders for the pelvis height and the torso (chest) yaw, pitch and roll, with one "Show actual" and one "Preview"
 * for all four.
 * <ul>
 * <li>The pelvis height is the pelvis z in the walking frame, the "Walking" frame of the behavior tree, which is on
 * the ground under the mid-feet.</li>
 * <li>The torso angles are in the walking frame, the "Walking" frame of the behavior tree.</li>
 * <li>The slider ranges are the {@link WholeBodyCommandLimits}, and every command is clamped to them, so the robot
 * is never asked for what its policy was not trained for.</li>
 * <li>Show actual draws the robot's values as notches.</li>
 * <li>Preview poses one ghost robot with all four sliders, the feet kept in place, and sends nothing until Send is
 * pressed. Send only sends what was changed, so previewing the height does not also command the torso.</li>
 * <li>Without preview, the pelvis height and the torso are each sent as their sliders are dragged.</li>
 * <li>The sliders keep the values they are set to. They start at the robot's values, and a home command, from
 * this UI or anywhere else, sets them to the home values.</li>
 * </ul>
 */
public class RDXPelvisAndTorsoSliders
{
   private static final int HEIGHT = 0;
   private static final int YAW = 1;
   private static final int PITCH = 2;
   private static final int ROLL = 3;
   private static final String[] SLIDER_LABELS = {"Pelvis Height:", "Torso Yaw:", "Torso Pitch:", "Torso Roll:"};
   /** The pelvis height is one group of sliders, the torso angles the other. Each group is commanded on its own. */
   private static final int PELVIS_GROUP = 0;
   private static final int TORSO_GROUP = 1;
   private static final int[] GROUP_OF_SLIDER = {PELVIS_GROUP, TORSO_GROUP, TORSO_GROUP, TORSO_GROUP};
   private static final int NUMBER_OF_GROUPS = 2;

   private static final double MINIMUM_TORSO_TRAJECTORY_TIME = 0.5;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final RDXTeleoperationParameters teleoperationParameters;
   private final WholeBodyCommandLimits limits;

   /** Desired values: the height in meters, the angles in degrees. Kept separate from the robot's, which can be drawn as notches. */
   private final float[][] sliderValues = {{Float.NaN}, {Float.NaN}, {Float.NaN}, {Float.NaN}};
   /** The robot's values: the height in meters, the angles in radians */
   private final double[] valuesFromRobot = {Double.NaN, Double.NaN, Double.NaN, Double.NaN};
   private final double robotDataExpirationDuration = 1.0;
   private final Throttler updateThrottler = new Throttler();
   private final double updatePeriod = UnitConversions.hertzToSeconds(10.0);
   private final Throttler[] sendThrottlers = {new Throttler(), new Throttler()};
   private final double sendPeriod = UnitConversions.hertzToSeconds(5.0);

   private final ImBoolean showActual = new ImBoolean(false);
   private final ImBoolean preview = new ImBoolean(false);
   /** Per group, whether the preview holds a value that was not sent yet */
   private final boolean[] previewEdited = new boolean[NUMBER_OF_GROUPS];
   /** Per group, set when a home command was seen, so the sliders go to the home values on the next render */
   private final boolean[] homeRequested = new boolean[NUMBER_OF_GROUPS];
   private final FrameQuaternion chestOrientation = new FrameQuaternion();
   private final FrameQuaternion desiredChestInWorld = new FrameQuaternion();
   private final FrameQuaternion previewChestInWorld = new FrameQuaternion();
   private final Quaternion remainingChestRotation = new Quaternion();

   private final RDXDesiredRobot previewRobot;
   // Leg IK for the preview: keep each sole where it is on the robot while the pelvis moves
   private static final int LEG_IK_MAX_ITERATIONS = 50;
   private static final double LEG_IK_CONVERGENCE_TOLERANCE = 1.0e-5;
   private static final double LEG_IK_DAMPING = 0.05;
   private final SideDependentList<GeometricJacobianCalculator> legJacobianCalculators = new SideDependentList<>();
   private final FramePose3D targetSolePose = new FramePose3D();
   private final Vector3D rotationError = new Vector3D();
   private final DMatrixRMaj legError = new DMatrixRMaj(6, 1);
   private final DMatrixRMaj dampedJJT = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj dampedJJTInverseError = new DMatrixRMaj(6, 1);
   private final DMatrixRMaj legJointDeltas = new DMatrixRMaj(6, 1);

   public RDXPelvisAndTorsoSliders(ROS2SyncedRobotModel syncedRobot,
                                   ROS2ControllerHelper ros2ControllerHelper,
                                   RDXTeleoperationParameters teleoperationParameters,
                                   WholeBodyCommandLimits limits)
   {
      this.syncedRobot = syncedRobot;
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.teleoperationParameters = teleoperationParameters;
      this.limits = limits;

      previewRobot = new RDXDesiredRobot(syncedRobot.getRobotModel());
      previewRobot.setSceneLevels(RDXSceneLevel.VIRTUAL);

      syncedRobot.addRobotConfigurationDataReceivedCallback(this::receiveRobotConfigurationData);
      // Home commands from anywhere, like the behavior tree or another UI, set the desired values too
      ros2ControllerHelper.subscribeViaCallback(HumanoidControllerAPI.getTopic(GoHomeMessage.class, syncedRobot.getRobotModel().getSimpleRobotName()),
                                                this::receiveGoHomeMessage);
   }

   private void receiveGoHomeMessage(GoHomeMessage message)
   {
      if (message.getHumanoidBodyPart() == GoHomeMessage.HUMANOID_BODY_PART_PELVIS)
         homePelvisHeight();
      else if (message.getHumanoidBodyPart() == GoHomeMessage.HUMANOID_BODY_PART_CHEST)
         homeTorso();
   }

   /** Sets the desired pelvis height to the controller's home height, as a home command does. */
   public void homePelvisHeight()
   {
      synchronized (homeRequested)
      {
         homeRequested[PELVIS_GROUP] = true;
      }
   }

   /** Sets the desired torso orientation to the controller's home, upright and facing forward, as a home command does. */
   public void homeTorso()
   {
      synchronized (homeRequested)
      {
         homeRequested[TORSO_GROUP] = true;
      }
   }

   public void create()
   {
      previewRobot.create();

      FullHumanoidRobotModel previewFullRobotModel = previewRobot.getDesiredFullRobotModel();
      for (RobotSide side : RobotSide.values)
      {
         GeometricJacobianCalculator legJacobianCalculator = new GeometricJacobianCalculator();
         legJacobianCalculator.setKinematicChain(previewFullRobotModel.getPelvis(), previewFullRobotModel.getFoot(side));
         legJacobianCalculator.setJacobianFrame(previewFullRobotModel.getSoleFrame(side));
         legJacobianCalculators.put(side, legJacobianCalculator);
      }
   }

   private void receiveRobotConfigurationData(RobotConfigurationData robotConfigurationData)
   {
      if (updateThrottler.run(updatePeriod))
      {
         double pelvisZ = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getPelvisZUpFrame).getZ();
         double walkingFrameZ = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getMidFeetUnderPelvisFrame).getZ();
         valuesFromRobot[HEIGHT] = pelvisZ - walkingFrameZ;

         FrameQuaternion chestInWalkingFrame = new FrameQuaternion(syncedRobot.getReferenceFrames().getChestFrame());
         chestInWalkingFrame.changeFrame(syncedRobot.getReferenceFrames().getMidFeetUnderPelvisFrame());
         valuesFromRobot[YAW] = chestInWalkingFrame.getYaw();
         valuesFromRobot[PITCH] = chestInWalkingFrame.getPitch();
         valuesFromRobot[ROLL] = chestInWalkingFrame.getRoll();
      }
   }

   public void update()
   {
      // Update first: the ghost sets up its bodies on its first update after loading, and coloring and showing need them
      previewRobot.update();

      boolean showPreviewRobot = previewRobot.isRobotLoaded() && preview.get() && hasDesiredValues(PELVIS_GROUP) && hasDesiredValues(TORSO_GROUP);
      if (showPreviewRobot)
      {
         // Start from the robot's configuration, the arms included
         FullHumanoidRobotModel previewFullRobotModel = previewRobot.getDesiredFullRobotModel();
         FullHumanoidRobotModel actualFullRobotModel = syncedRobot.getFullRobotModel();
         MultiBodySystemMissingTools.copyOneDoFJointsConfiguration(actualFullRobotModel.getOneDoFJoints(), previewFullRobotModel.getOneDoFJoints());
         previewFullRobotModel.getRootJoint().setJointConfiguration(actualFullRobotModel.getRootJoint().getJointPose());

         // Set the spine joints for as much of the desired chest orientation as they can make. Robots like Alex
         // only have a spine yaw, and pitch and roll the torso with the pelvis, at the hips.
         computeDesiredChestOrientation();
         desiredChestInWorld.setIncludingFrame(chestOrientation);
         chestOrientation.changeFrame(syncedRobot.getReferenceFrames().getPelvisFrame());
         for (SpineJointName spineJointName : syncedRobot.getRobotModel().getJointMap().getSpineJointNames())
         {
            OneDoFJointBasics spineJoint = previewFullRobotModel.getSpineJoint(spineJointName);
            if (spineJoint == null)
               continue;
            double q = switch (spineJointName)
            {
               case SPINE_YAW -> chestOrientation.getYaw();
               case SPINE_PITCH -> chestOrientation.getPitch();
               case SPINE_ROLL -> chestOrientation.getRoll();
               default -> spineJoint.getQ();
            };
            spineJoint.setQ(MathTools.clamp(q, spineJoint.getJointLimitLower(), spineJoint.getJointLimitUpper()));
         }

         previewFullRobotModel.updateFrames();

         // Rotate the pelvis about its origin by what the spine could not do, so the chest gets the desired orientation
         previewChestInWorld.setToZero(previewFullRobotModel.getChest().getParentJoint().getFrameAfterJoint());
         previewChestInWorld.changeFrame(ReferenceFrame.getWorldFrame());
         remainingChestRotation.set(desiredChestInWorld);
         remainingChestRotation.multiplyConjugateOther(previewChestInWorld);
         previewFullRobotModel.getRootJoint().getJointPose().getOrientation().preMultiply(remainingChestRotation);

         // Move the pelvis by the height change, then solve the legs so the feet stay where they are
         previewFullRobotModel.getRootJoint().getJointPose().getPosition().addZ(getDesiredHeight() - valuesFromRobot[HEIGHT]);
         previewFullRobotModel.updateFrames();
         for (RobotSide side : RobotSide.values)
            solveLegIK(side, actualFullRobotModel, previewFullRobotModel);

         previewRobot.setActive(true);
         previewRobot.setWholeBodyColor(Color.ORANGE);
         previewRobot.setPelvisShowing(true);
         previewRobot.setChestShowing(true);
         previewRobot.setHeadShowing(syncedRobot.getRobotModel().getRobotVersion().hasHead());
         for (RobotSide side : RobotSide.values)
         {
            previewRobot.setArmShowing(side, syncedRobot.getRobotModel().getRobotVersion().hasArm(side));
            previewRobot.setLegShowing(side, true);
         }
      }
      else
      {
         previewRobot.setActive(false);
      }
   }

   /**
    * Damped least squares IK from the pelvis to the sole, with the Jacobian and the error both expressed in the
    * preview sole frame. The pelvis is fixed, so the target is the robot's current sole pose in world.
    */
   private void solveLegIK(RobotSide side, FullHumanoidRobotModel actualFullRobotModel, FullHumanoidRobotModel previewFullRobotModel)
   {
      GeometricJacobianCalculator legJacobianCalculator = legJacobianCalculators.get(side);
      ReferenceFrame previewSoleFrame = previewFullRobotModel.getSoleFrame(side);
      int numberOfDoFs = legJacobianCalculator.getNumberOfDegreesOfFreedom();
      legJointDeltas.reshape(numberOfDoFs, 1);

      for (int iteration = 0; iteration < LEG_IK_MAX_ITERATIONS; iteration++)
      {
         targetSolePose.setToZero(actualFullRobotModel.getSoleFrame(side));
         targetSolePose.changeFrame(previewSoleFrame);
         targetSolePose.getOrientation().getRotationVector(rotationError);

         // Mecano Jacobians have the angular part first, then the linear part
         rotationError.get(0, legError);
         targetSolePose.getPosition().get(3, legError);
         if (CommonOps_DDRM.elementMaxAbs(legError) < LEG_IK_CONVERGENCE_TOLERANCE)
            break;

         legJacobianCalculator.reset();
         DMatrixRMaj jacobian = legJacobianCalculator.getJacobianMatrix();
         CommonOps_DDRM.multTransB(jacobian, jacobian, dampedJJT);
         for (int i = 0; i < 6; i++)
            dampedJJT.add(i, i, LEG_IK_DAMPING * LEG_IK_DAMPING);
         if (!CommonOps_DDRM.solve(dampedJJT, legError, dampedJJTInverseError))
            break;
         CommonOps_DDRM.multTransA(jacobian, dampedJJTInverseError, legJointDeltas);

         int column = 0;
         for (JointReadOnly joint : legJacobianCalculator.getJointsFromBaseToEndEffector())
         {
            OneDoFJointBasics oneDoFJoint = (OneDoFJointBasics) joint;
            double q = oneDoFJoint.getQ() + legJointDeltas.get(column++, 0);
            oneDoFJoint.setQ(MathTools.clamp(q, oneDoFJoint.getJointLimitLower(), oneDoFJoint.getJointLimitUpper()));
         }
         previewFullRobotModel.updateFrames();
      }
   }

   /** Renders the four labelled sliders, each starting at {@code widgetStartX}, then the shared options. */
   public void renderImGuiWidgets(float widgetStartX)
   {
      // Start the desired values at the robot's, then leave them where the operator puts them
      for (int slider = 0; slider < SLIDER_LABELS.length; slider++)
      {
         if (Float.isNaN(sliderValues[slider][0]) && !Double.isNaN(valuesFromRobot[slider]))
            sliderValues[slider][0] = (float) toSliderUnits(slider, valuesFromRobot[slider]);
      }
      // A home command sets the desired values to the home values, and drops what the preview held
      synchronized (homeRequested)
      {
         if (homeRequested[PELVIS_GROUP])
         {
            sliderValues[HEIGHT][0] = (float) limits.clampPelvisHeight(limits.getPelvisHomeHeight());
            previewEdited[PELVIS_GROUP] = false;
         }
         if (homeRequested[TORSO_GROUP])
         {
            for (int slider = YAW; slider <= ROLL; slider++)
               sliderValues[slider][0] = 0.0f;
            previewEdited[TORSO_GROUP] = false;
         }
         homeRequested[PELVIS_GROUP] = false;
         homeRequested[TORSO_GROUP] = false;
      }

      boolean[] changed = new boolean[NUMBER_OF_GROUPS];
      boolean[] released = new boolean[NUMBER_OF_GROUPS];
      for (int slider = 0; slider < SLIDER_LABELS.length; slider++)
      {
         int group = GROUP_OF_SLIDER[slider];
         ImGui.text(SLIDER_LABELS[slider]);
         ImGui.sameLine();
         ImGui.setCursorPosX(widgetStartX);

         float minimum = (float) toSliderUnits(slider, getMinimum(slider));
         float maximum = (float) toSliderUnits(slider, getMaximum(slider));
         double robotValue = toSliderUnits(slider, valuesFromRobot[slider]);
         String valueFormat = slider == HEIGHT ? "%.3f m" : "%.1f" + EuclidCoreMissingTools.DEGREE_SYMBOL;
         String format = valueFormat;
         if (showActual.get())
         {
            float sliderWidth = ImGui.getContentRegionAvailX();
            if (!Double.isNaN(robotValue))
            {
               float notch = (float) MathTools.clamp((robotValue - minimum) / (maximum - minimum), 0.0, 1.0);
               ImGuiTools.renderSliderOrProgressNotch(notch * sliderWidth, ImGui.getColorU32(ImGuiCol.Text));
            }
            // The desired value, then the robot's in brackets
            format += " (" + valueFormat.formatted(robotValue) + ")";
         }

         ImGui.setNextItemWidth(-1);
         float previousValue = sliderValues[slider][0];
         ImGui.sliderFloat(labels.getHidden(SLIDER_LABELS[slider]), sliderValues[slider], minimum, maximum, format);
         changed[group] |= !Float.isNaN(sliderValues[slider][0]) && sliderValues[slider][0] != previousValue;
         released[group] |= ImGui.isItemDeactivatedAfterEdit();
      }

      ImGui.setCursorPosX(widgetStartX);
      ImGui.checkbox(labels.get("Show actual"), showActual);
      ImGui.sameLine();
      // Unchecking drops the unsent previewed values: the sliders go back to the robot's values
      if (ImGui.checkbox(labels.get("Preview"), preview) && !preview.get())
         dropPreviewEdits();
      if (preview.get())
      {
         for (int group = 0; group < NUMBER_OF_GROUPS; group++)
            previewEdited[group] |= changed[group];
         ImGui.sameLine();
         ImGui.beginDisabled(!robotDataIsFresh() || !(previewEdited[PELVIS_GROUP] || previewEdited[TORSO_GROUP]));
         if (ImGui.button(labels.get("Send")))
         {
            // Only send what was changed
            for (int group = 0; group < NUMBER_OF_GROUPS; group++)
               if (previewEdited[group])
                  send(group);
            clearPreviewEdits();
         }
         ImGui.endDisabled();
      }
      else
      {
         for (int group = 0; group < NUMBER_OF_GROUPS; group++)
         {
            // Always send on release so the robot gets the final values the sliders are left at
            if (((changed[group] && sendThrottlers[group].run(sendPeriod)) || released[group]) && robotDataIsFresh())
               send(group);
         }
      }
   }

   /** Puts the sliders of the groups with unsent previewed values back at the robot's values */
   private void dropPreviewEdits()
   {
      for (int slider = 0; slider < SLIDER_LABELS.length; slider++)
      {
         if (previewEdited[GROUP_OF_SLIDER[slider]])
            sliderValues[slider][0] = (float) toSliderUnits(slider, valuesFromRobot[slider]);
      }
      clearPreviewEdits();
   }

   private boolean hasDesiredValues(int group)
   {
      for (int slider = 0; slider < SLIDER_LABELS.length; slider++)
         if (GROUP_OF_SLIDER[slider] == group && Float.isNaN(sliderValues[slider][0]))
            return false;
      return true;
   }

   private void clearPreviewEdits()
   {
      for (int group = 0; group < NUMBER_OF_GROUPS; group++)
         previewEdited[group] = false;
   }

   private boolean robotDataIsFresh()
   {
      return syncedRobot.getDataReceptionTimerSnapshot().isRunning(robotDataExpirationDuration);
   }

   /** The height stays in meters; the angles go from radians to degrees */
   private static double toSliderUnits(int slider, double value)
   {
      return slider == HEIGHT ? value : Math.toDegrees(value);
   }

   /** The slider value in meters or radians */
   private double getSliderValue(int slider)
   {
      return slider == HEIGHT ? sliderValues[slider][0] : Math.toRadians(sliderValues[slider][0]);
   }

   private double getMinimum(int slider)
   {
      return switch (slider)
      {
         case HEIGHT -> limits.getPelvisMinimumHeight();
         case YAW -> limits.getTorsoMinimumYaw();
         case PITCH -> limits.getTorsoMinimumPitch();
         default -> limits.getTorsoMinimumRoll();
      };
   }

   private double getMaximum(int slider)
   {
      return switch (slider)
      {
         case HEIGHT -> limits.getPelvisMaximumHeight();
         case YAW -> limits.getTorsoMaximumYaw();
         case PITCH -> limits.getTorsoMaximumPitch();
         default -> limits.getTorsoMaximumRoll();
      };
   }

   /** The desired height in the walking frame, clamped, since the slider starts at the robot's, which can be outside the limits */
   private double getDesiredHeight()
   {
      return limits.clampPelvisHeight(getSliderValue(HEIGHT));
   }

   /** Packs {@link #chestOrientation} with the sliders' orientation, clamped to the limits, in world */
   private void computeDesiredChestOrientation()
   {
      chestOrientation.setToZero(syncedRobot.getReferenceFrames().getMidFeetUnderPelvisFrame());
      chestOrientation.setYawPitchRoll(MathTools.clamp(getSliderValue(YAW), getMinimum(YAW), getMaximum(YAW)),
                                       MathTools.clamp(getSliderValue(PITCH), getMinimum(PITCH), getMaximum(PITCH)),
                                       MathTools.clamp(getSliderValue(ROLL), getMinimum(ROLL), getMaximum(ROLL)));
      limits.clampTorsoOrientation(chestOrientation, syncedRobot.getReferenceFrames().getMidFeetUnderPelvisFrame());
      chestOrientation.changeFrame(ReferenceFrame.getWorldFrame());
   }

   private void send(int group)
   {
      if (!hasDesiredValues(group))
         return;

      if (group == PELVIS_GROUP)
         sendDesiredHeight();
      else
         sendDesiredTorsoOrientation();
   }

   /** @return the trajectory time */
   private double sendDesiredHeight()
   {
      double desiredHeight = getDesiredHeight();
      // The walking frame is level, so its z plus the height is the pelvis z in world
      double walkingFrameZ = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getMidFeetUnderPelvisFrame).getZ();
      double desiredHeightInWorld = desiredHeight + walkingFrameZ;

      double minTrajectoryTime = Math.abs(desiredHeight - valuesFromRobot[HEIGHT]) / teleoperationParameters.getPelvisHeightChangeVelocity();
      double trajectoryTime = MathTools.clamp(teleoperationParameters.getTrajectoryTime(), minTrajectoryTime, 1000.0); // Safety

      LogTools.info("Commanding pelvis height %.3f m in the walking frame (now %.3f m) over %.2f s".formatted(desiredHeight,
                                                                                                          valuesFromRobot[HEIGHT],
                                                                                                          trajectoryTime));

      PelvisHeightTrajectoryMessage message = new PelvisHeightTrajectoryMessage();
      message.getEuclideanTrajectory()
             .set(HumanoidMessageTools.createEuclideanTrajectoryMessage(trajectoryTime,
                                                                        new Point3D(0.0, 0.0, desiredHeightInWorld),
                                                                        ReferenceFrame.getWorldFrame()));
      message.getEuclideanTrajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));
      message.getEuclideanTrajectory().getSelectionMatrix().setXSelected(false);
      message.getEuclideanTrajectory().getSelectionMatrix().setYSelected(false);
      message.getEuclideanTrajectory().getSelectionMatrix().setZSelected(true);
      ros2ControllerHelper.publishToController(message);
      return trajectoryTime;
   }

   /** @return the trajectory time */
   private double sendDesiredTorsoOrientation()
   {
      double largestChange = 0.0;
      for (int slider = YAW; slider <= ROLL; slider++)
         largestChange = Math.max(largestChange, Math.abs(getSliderValue(slider) - valuesFromRobot[slider]));
      computeDesiredChestOrientation();

      double trajectoryTime = Double.isNaN(largestChange) ? teleoperationParameters.getTrajectoryTime()
                                                          : largestChange / teleoperationParameters.getChestOrientationVelocity();
      trajectoryTime = MathTools.clamp(trajectoryTime, MINIMUM_TORSO_TRAJECTORY_TIME, 1000.0); // Safety

      LogTools.info("Commanding torso yaw %.1f pitch %.1f roll %.1f deg in the walking frame over %.2f s".formatted(sliderValues[YAW][0],
                                                                                                                  sliderValues[PITCH][0],
                                                                                                                  sliderValues[ROLL][0],
                                                                                                                  trajectoryTime));

      ChestTrajectoryMessage message = new ChestTrajectoryMessage();
      message.getSo3Trajectory()
             .set(HumanoidMessageTools.createSO3TrajectoryMessage(trajectoryTime,
                                                                  chestOrientation,
                                                                  EuclidCoreTools.zeroVector3D,
                                                                  syncedRobot.getReferenceFrames().getPelvisZUpFrame()));
      message.getSo3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));
      ros2ControllerHelper.publishToController(message);
      return trajectoryTime;
   }

   /** Turns the preview off and drops the unsent previewed values, putting those sliders back at the robot's values. */
   public void disablePreview()
   {
      preview.set(false);
      dropPreviewEdits();
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      previewRobot.getRenderables(renderables, pool, sceneLevels);
   }

   public void destroy()
   {
      previewRobot.destroy();
   }
}
