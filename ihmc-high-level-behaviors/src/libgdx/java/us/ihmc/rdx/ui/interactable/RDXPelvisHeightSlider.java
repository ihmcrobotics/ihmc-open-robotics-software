package us.ihmc.rdx.ui.interactable;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.PelvisHeightTrajectoryMessage;
import controller_msgs.RobotConfigurationData;
import imgui.flag.ImGuiCol;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.UnitConversions;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.Point3D;
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
import us.ihmc.robotics.MultiBodySystemMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.string.StringTools;

import java.util.Set;

public class RDXPelvisHeightSlider
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final String sliderName;
   private final RDXTeleoperationParameters teleoperationParameters;
   /** The desired height. Kept separate from the robot's height, which can be drawn as a notch, like the Ability Hand finger sliders. */
   private final float[] sliderValue = {Float.NaN};
   private double robotDataExpirationDuration = 1.0;
   private double minHeight = -Math.toRadians(5.0);
   private double maxHeight = Math.toRadians(5.0);
   private volatile double valueFromRobot = Double.NaN;
   private final Throttler updateThrottler = new Throttler();
   private double updatePeriod = UnitConversions.hertzToSeconds(10.0);
   private final Throttler sendThrottler = new Throttler();
   private double sendPeriod = UnitConversions.hertzToSeconds(5.0);
   private boolean released = false;

   private final ImBoolean showActual = new ImBoolean(false);
   /** When previewing, the slider moves a ghost robot and nothing is sent until Send is pressed. */
   private final ImBoolean preview = new ImBoolean(false);
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

   public RDXPelvisHeightSlider(ROS2SyncedRobotModel syncedRobot,
                                ROS2ControllerHelper ros2ControllerHelper,
                                RDXTeleoperationParameters teleoperationParameters)
   {
      this.syncedRobot = syncedRobot;
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.teleoperationParameters = teleoperationParameters;
      sliderName = "##Pelvis height";

      previewRobot = new RDXDesiredRobot(syncedRobot.getRobotModel());
      previewRobot.setSceneLevels(RDXSceneLevel.VIRTUAL);

      syncedRobot.addRobotConfigurationDataReceivedCallback(this::receiveRobotConfigurationData);
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
         double midFeetZ = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getMidFeetZUpFrame).getZ();
         double midFeetToPelvis = pelvisZ - midFeetZ;
         valueFromRobot = midFeetToPelvis;
      }
   }

   public void update()
   {
      // Update first: the ghost sets up its bodies on its first update after loading, and coloring and showing need them
      previewRobot.update();

      // The ghost loads in the background, so it can only be posed and shown once it is loaded
      boolean showPreviewRobot = previewRobot.isRobotLoaded() && preview.get() && !Float.isNaN(sliderValue[0]) && !Double.isNaN(valueFromRobot);
      if (showPreviewRobot)
      {
         // Start from the robot's current configuration, move the pelvis by the height change,
         // then solve the legs so the feet stay where they are. The arms keep the robot's values.
         FullHumanoidRobotModel previewFullRobotModel = previewRobot.getDesiredFullRobotModel();
         FullHumanoidRobotModel actualFullRobotModel = syncedRobot.getFullRobotModel();
         MultiBodySystemMissingTools.copyOneDoFJointsConfiguration(actualFullRobotModel.getOneDoFJoints(), previewFullRobotModel.getOneDoFJoints());
         previewFullRobotModel.getRootJoint().setJointConfiguration(actualFullRobotModel.getRootJoint().getJointPose());
         previewFullRobotModel.getRootJoint().getJointPose().getPosition().addZ(sliderValue[0] - valueFromRobot);
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

   public void renderImGuiWidgets()
   {
      float startX = ImGui.getCursorPosX();
      boolean changed = renderImGuiSliderAndReturnChanged();

      ImGui.setCursorPosX(startX);
      ImGui.checkbox(labels.get("Show actual"), showActual);
      ImGui.sameLine();
      ImGui.checkbox(labels.get("Preview"), preview);
      if (preview.get())
      {
         ImGui.sameLine();
         ImGui.beginDisabled(!robotDataIsFresh() || Float.isNaN(sliderValue[0]));
         if (ImGui.button(labels.get("Send")))
            sendDesiredHeight();
         ImGui.endDisabled();
      }
      else if (changed || released)
      {
         // Always send on release so the robot gets the final desired height the slider is left at
         if ((sendThrottler.run(sendPeriod) || released) && robotDataIsFresh())
            sendDesiredHeight();
      }
   }

   private boolean robotDataIsFresh()
   {
      return syncedRobot.getDataReceptionTimerSnapshot().isRunning(robotDataExpirationDuration);
   }

   private void sendDesiredHeight()
   {
      double desiredHeight = sliderValue[0];
      double pelvisZ = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getPelvisZUpFrame).getZ();
      double midFeetZ = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getMidFeetZUpFrame).getZ();
      double desiredHeightInWorld = desiredHeight + midFeetZ;

      double change = Math.abs(desiredHeight - valueFromRobot);
      double trajectoryTime = change * teleoperationParameters.getPelvisHeightChangeVelocity() * sendPeriod;
      trajectoryTime = MathTools.clamp(trajectoryTime, 0.5, 1000.0); // Safety

      LogTools.info(StringTools.format3D("Commanding height trajectory. traj. time: {} desired: {} (pelvis - midFeetZ): {} in world: {}",
                                         trajectoryTime,
                                         desiredHeight,
                                         pelvisZ - midFeetZ,
                                         desiredHeightInWorld));

      PelvisHeightTrajectoryMessage message = new PelvisHeightTrajectoryMessage();
      message.getEuclideanTrajectory()
             .set(HumanoidMessageTools.createEuclideanTrajectoryMessage(trajectoryTime,
                                                                        new Point3D(0.0, 0.0, desiredHeightInWorld),
                                                                        ReferenceFrame.getWorldFrame()));
      long frameId = MessageTools.toFrameId(ReferenceFrame.getWorldFrame());
      message.getEuclideanTrajectory().getFrameInformation().setDataReferenceFrameId(frameId);
      message.getEuclideanTrajectory().getSelectionMatrix().setXSelected(false);
      message.getEuclideanTrajectory().getSelectionMatrix().setYSelected(false);
      message.getEuclideanTrajectory().getSelectionMatrix().setZSelected(true);
      ros2ControllerHelper.publishToController(message);
   }

   private boolean renderImGuiSliderAndReturnChanged()
   {
      minHeight = teleoperationParameters.getPelvisMinimumHeight();
      maxHeight = teleoperationParameters.getPelvisMaximumHeight();
      // Start the desired height at the robot's height, then leave it where the operator puts it
      if (Float.isNaN(sliderValue[0]) && !Double.isNaN(valueFromRobot))
         sliderValue[0] = (float) valueFromRobot;

      String format = "%.3f m";
      if (showActual.get())
      {
         float sliderWidth = ImGui.getContentRegionAvailX();
         float currentNotch = (float) ((valueFromRobot - minHeight) / (maxHeight - minHeight));
         if (!Double.isNaN(valueFromRobot))
            ImGuiTools.renderSliderOrProgressNotch((float) MathTools.clamp(currentNotch, 0.0, 1.0) * sliderWidth, ImGui.getColorU32(ImGuiCol.Text));
         format = "%%.3f m (%.3f m)".formatted(valueFromRobot);
      }

      float previousValue = sliderValue[0];
      ImGui.sliderFloat(labels.get(sliderName), sliderValue, (float) minHeight, (float) maxHeight, format);
      released = ImGui.isItemDeactivatedAfterEdit() && !Double.isNaN(sliderValue[0]);
      float currentValue = sliderValue[0];
      return !Double.isNaN(sliderValue[0]) && currentValue != previousValue;
   }

   /** Turns the preview off and drops the unsent previewed height, putting the slider back at the robot's height. */
   public void disablePreview()
   {
      if (preview.get())
      {
         preview.set(false);
         sliderValue[0] = (float) valueFromRobot;
      }
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
