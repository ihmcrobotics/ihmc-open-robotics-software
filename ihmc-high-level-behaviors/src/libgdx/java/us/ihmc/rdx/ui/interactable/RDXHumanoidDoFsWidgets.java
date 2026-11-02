package us.ihmc.rdx.ui.interactable;

import com.badlogic.gdx.graphics.Color;
import controller_msgs.*;
import ihmc_common_msgs.QueueableMessage;
import ihmc_common_msgs.TrajectoryPoint1DMessage;
import imgui.ImGui;
import imgui.ImVec4;
import imgui.flag.ImGuiCol;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import org.apache.commons.lang3.mutable.MutableObject;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FrameYawPitchRoll;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiInputDouble;
import us.ihmc.rdx.imgui.ImGuiSliderDouble;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.teleoperation.RDXDesiredRobot;
import us.ihmc.rdx.ui.teleoperation.RDXTeleoperationParameters;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class RDXHumanoidDoFsWidgets
{
   private enum InteractableDoFs
   {
      NONE("None"), NECK("Neck"), CHEST("Chest"), LEFT_ARM("Left Arm"), RIGHT_ARM("Right Arm"), LEFT_LEG("Left Leg"), RIGHT_LEG("Right Leg");

      private final String displayName;

      InteractableDoFs(String displayName)
      {
         this.displayName = displayName;
      }
   }
   /** Radio button grid for picking the body part, so it is one click instead of opening a combo. */
   private static final InteractableDoFs[][] BODY_PART_ROWS = {{InteractableDoFs.NONE},
                                                              {InteractableDoFs.NECK, InteractableDoFs.CHEST},
                                                              {InteractableDoFs.LEFT_ARM, InteractableDoFs.RIGHT_ARM},
                                                              {InteractableDoFs.LEFT_LEG, InteractableDoFs.RIGHT_LEG}};
   private final int NUMBER_OF_DOFS_SPHERICAL_JOINT = 3;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final RDXTeleoperationParameters teleoperationParameters;
   private final RDXDesiredRobot desiredRobot;

   private final ImInt currentInteractablePart = new ImInt(InteractableDoFs.NONE.ordinal());
   private final ImInt previousInteractablePart = new ImInt(InteractableDoFs.NONE.ordinal());
   /** Draws each joint's actual angle as a notch on its slider, like the pelvis height slider. */
   private final ImBoolean showActual = new ImBoolean(false);
   /** Screen X where the joint slider bars start, recorded while rendering them, so Show actual can line up with them. */
   private float sliderStartScreenX = Float.NaN;

   private final SideDependentList<ImDoubleWrapper[]> armJointAngleWidgets = new SideDependentList<>();
   private final SideDependentList<double[]> armJointWidgetValues = new SideDependentList<>();
   private final SideDependentList<double[]> armValuesFromRobot = new SideDependentList<>();
   private final SideDependentList<ArmJointName[]> armJointNamesArray = new SideDependentList<>();

   private final ImDoubleWrapper[] spineJointAngleWidgets = new ImDoubleWrapper[NUMBER_OF_DOFS_SPHERICAL_JOINT];
   private final double[] spineJointWidgetValues = new double[NUMBER_OF_DOFS_SPHERICAL_JOINT];
   private final double[] spineValuesFromRobot = new double[NUMBER_OF_DOFS_SPHERICAL_JOINT];
   private final SpineJointName[] spineJointNamesArray;

   private final ImDoubleWrapper[] neckJointAngleWidgets = new ImDoubleWrapper[NUMBER_OF_DOFS_SPHERICAL_JOINT*2];
   private double[] neckJointWidgetValues;
   private double[] neckValuesFromRobot;
   private NeckJointName[] neckJointNamesArray;

   private final SideDependentList<ImDoubleWrapper[]> legJointAngleWidgets = new SideDependentList<>();
   private final SideDependentList<double[]> legJointWidgetValues = new SideDependentList<>();
   private final SideDependentList<double[]> legValuesFromRobot = new SideDependentList<>();
   private final SideDependentList<LegJointName[]> legJointNamesArray = new SideDependentList<>();

   public RDXHumanoidDoFsWidgets(ROS2SyncedRobotModel syncedRobot,
                                 ROS2ControllerHelper ros2ControllerHelper,
                                 RDXTeleoperationParameters teleoperationParameters)
   {
      this.syncedRobot = syncedRobot;
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.teleoperationParameters = teleoperationParameters;

      desiredRobot = new RDXDesiredRobot(syncedRobot.getRobotModel());
      desiredRobot.setSceneLevels(RDXSceneLevel.VIRTUAL);

      SideDependentList<String[]> armJointNames = new SideDependentList<>();
      SideDependentList<double[]> armJointLowerLimits = new SideDependentList<>();
      SideDependentList<double[]> armJointUpperLimits = new SideDependentList<>();
      SideDependentList<String[]> legJointNames = new SideDependentList<>();
      SideDependentList<double[]> legJointLowerLimits = new SideDependentList<>();
      SideDependentList<double[]> legJointUpperLimits = new SideDependentList<>();
      double[] spineLowerLimits = new double[NUMBER_OF_DOFS_SPHERICAL_JOINT];
      double[] spineUpperLimits = new double[NUMBER_OF_DOFS_SPHERICAL_JOINT];
      double[] neckLowerLimits = new double[NUMBER_OF_DOFS_SPHERICAL_JOINT];
      double[] neckUpperLimits = new double[NUMBER_OF_DOFS_SPHERICAL_JOINT];
      for (RobotSide side : RobotSide.values)
      {
         if (syncedRobot.getRobotModel().getRobotVersion().hasArm(side))
         {
            ArmJointName[] singleArmJointNamesArray = syncedRobot.getRobotModel().getJointMap().getArmJointNames(side);
            String[] singleArmJointNames = new String[singleArmJointNamesArray.length];
            for (int i = 0; i < singleArmJointNames.length; i++)
               singleArmJointNames[i] = singleArmJointNamesArray[i].toString();

            armJointNames.put(side, singleArmJointNames);
            armJointNamesArray.put(side, singleArmJointNamesArray);
            armJointAngleWidgets.put(side, new ImDoubleWrapper[singleArmJointNamesArray.length]);
            armJointLowerLimits.put(side, new double[singleArmJointNamesArray.length]);
            armJointUpperLimits.put(side, new double[singleArmJointNamesArray.length]);
            armJointWidgetValues.put(side, new double[singleArmJointNamesArray.length]);
            armValuesFromRobot.put(side, new double[singleArmJointNamesArray.length]);

            int jointIndex = 0;
            for (ArmJointName armJointName : singleArmJointNamesArray)
            {
               OneDoFJointBasics armJoint = syncedRobot.getFullRobotModel().getArmJoint(side, armJointName);
               armJointLowerLimits.get(side)[jointIndex] = armJoint.getJointLimitLower();
               armJointUpperLimits.get(side)[jointIndex] = armJoint.getJointLimitUpper();
               ++jointIndex;
            }

            for (int i = 0; i < armJointAngleWidgets.get(side).length; i++)
            {
               armJointAngleWidgets.get(side)[i] = createImDoubleWrapper(i, armJointWidgetValues.get(side), armValuesFromRobot.get(side), armJointLowerLimits.get(side), armJointUpperLimits.get(side), armJointNames.get(side));
            }
         }

         LegJointName[] singleLegJointNamesArray = syncedRobot.getRobotModel().getJointMap().getLegJointNames();
         String[] singleLegJointNames = new String[singleLegJointNamesArray.length];
         for (int i = 0; i < singleLegJointNames.length; i++)
            singleLegJointNames[i] = singleLegJointNamesArray[i].toString();

         legJointNames.put(side, singleLegJointNames);
         legJointNamesArray.put(side, singleLegJointNamesArray);
         legJointAngleWidgets.put(side, new ImDoubleWrapper[singleLegJointNamesArray.length]);
         legJointLowerLimits.put(side, new double[singleLegJointNamesArray.length]);
         legJointUpperLimits.put(side, new double[singleLegJointNamesArray.length]);
         legJointWidgetValues.put(side, new double[singleLegJointNamesArray.length]);
         legValuesFromRobot.put(side, new double[singleLegJointNamesArray.length]);

         int jointIndex = 0;
         for (LegJointName legJointName : singleLegJointNamesArray)
         {
            OneDoFJointBasics legJoint = syncedRobot.getFullRobotModel().getLegJoint(side, legJointName);
            legJointLowerLimits.get(side)[jointIndex] = legJoint.getJointLimitLower();
            legJointUpperLimits.get(side)[jointIndex] = legJoint.getJointLimitUpper();
            ++jointIndex;
         }

         for (int i = 0; i < legJointAngleWidgets.get(side).length; i++)
         {
            legJointAngleWidgets.get(side)[i] = createImDoubleWrapper(i, legJointWidgetValues.get(side), legValuesFromRobot.get(side), legJointLowerLimits.get(side), legJointUpperLimits.get(side), legJointNames.get(side));
         }
      }

      this.spineJointNamesArray = syncedRobot.getRobotModel().getJointMap().getSpineJointNames();
      String[] spineJointNames = new String[spineJointNamesArray.length];
      for (int i=0; i < spineJointNames.length; i++)
         spineJointNames[i] = spineJointNamesArray[i].toString();
      int jointIndex = 0;
      for (SpineJointName spineJointName : spineJointNamesArray)
      {
         OneDoFJointBasics spineJoint = syncedRobot.getFullRobotModel().getSpineJoint(spineJointName);
         spineLowerLimits[jointIndex] = spineJoint.getJointLimitLower();
         spineUpperLimits[jointIndex] = spineJoint.getJointLimitUpper();
         ++jointIndex;
      }
      for (int i = 0; i < spineJointAngleWidgets.length; i++)
      {
         spineJointAngleWidgets[i] = createImDoubleWrapper(i, spineJointWidgetValues, spineValuesFromRobot, spineLowerLimits, spineUpperLimits, spineJointNames);
      }

      if (syncedRobot.getRobotModel().getRobotVersion().hasHead())
      {
         this.neckJointNamesArray = syncedRobot.getRobotModel().getJointMap().getNeckJointNames();
         String[] neckJointNames = new String[neckJointNamesArray.length];
         for (int i = 0; i < neckJointNames.length; i++)
            neckJointNames[i] = neckJointNamesArray[i].toString();
         jointIndex = 0;
         for (NeckJointName neckJointName : neckJointNamesArray)
         {
            OneDoFJointBasics neckJoint = syncedRobot.getFullRobotModel().getNeckJoint(neckJointName);
            neckLowerLimits[jointIndex] = neckJoint.getJointLimitLower();
            neckUpperLimits[jointIndex] = neckJoint.getJointLimitUpper();
            ++jointIndex;
         }

         neckJointWidgetValues = new double[neckJointNamesArray.length];
         neckValuesFromRobot = new double[neckJointNamesArray.length];
         for (int i = 0; i < neckJointAngleWidgets.length; i++)
         {
            neckJointAngleWidgets[i] = createImDoubleWrapper(i, neckJointWidgetValues, neckValuesFromRobot, neckLowerLimits, neckUpperLimits, neckJointNames);
         }
      }
   }

   private ImDoubleWrapper createImDoubleWrapper(int jointIndex,
                                                 double[] jointWidgetValues,
                                                 double[] valuesFromRobot,
                                                 double[] lowerLimits,
                                                 double[] upperLimits,
                                                 String[] jointNames)
   {
      final MutableObject<ImGuiInputDouble> fancyInput = new MutableObject<>();
      final MutableObject<ImGuiSliderDouble> fancySlider = new MutableObject<>();
      return new ImDoubleWrapper(() -> jointWidgetValues[jointIndex],
                                 jointAngle -> jointWidgetValues[jointIndex] = jointAngle,
                                 imDouble ->
                                 {
                                    if (fancyInput.getValue() == null)
                                    {
                                       fancyInput.setValue(new ImGuiInputDouble("j" + jointIndex, "%.3f", imDouble));
                                       fancyInput.getValue().setWidgetWidth(119.0f);

                                       fancySlider.setValue(new ImGuiSliderDouble("", "", imDouble));
                                    }

                                    fancyInput.getValue().render(0.01, 0.1);

                                    ImGui.sameLine();
                                    String sliderText = "%s %.1f%s".formatted(jointNames[jointIndex],
                                                                              Math.toDegrees(imDouble.get()),
                                                                              EuclidCoreMissingTools.DEGREE_SYMBOL);
                                    if (showActual.get())
                                       sliderText += " (%.1f%s)".formatted(Math.toDegrees(valuesFromRobot[jointIndex]), EuclidCoreMissingTools.DEGREE_SYMBOL);
                                    fancySlider.getValue().setWidgetText(sliderText);
                                    fancySlider.getValue().render(lowerLimits[jointIndex], upperLimits[jointIndex]);
                                    sliderStartScreenX = ImGui.getItemRectMinX();

                                    if (showActual.get())
                                       renderActualNotch(valuesFromRobot[jointIndex], lowerLimits[jointIndex], upperLimits[jointIndex]);
                                 });
   }

   /** Draws a notch over the slider that was just rendered, at the joint's actual angle. */
   private void renderActualNotch(double actualValue, double lowerLimit, double upperLimit)
   {
      float sliderMinX = ImGui.getItemRectMinX();
      float sliderMaxX = ImGui.getItemRectMaxX();
      float fraction = (float) MathTools.clamp((actualValue - lowerLimit) / (upperLimit - lowerLimit), 0.0, 1.0);
      float notchX = sliderMinX + fraction * (sliderMaxX - sliderMinX);
      float verticalExtents = 3.0f;
      float notchWidth = 2.0f;
      ImGui.getWindowDrawList().addRectFilled(notchX - notchWidth / 2.0f,
                                              ImGui.getItemRectMinY() - verticalExtents,
                                              notchX + notchWidth / 2.0f,
                                              ImGui.getItemRectMaxY() + verticalExtents,
                                              ImGui.getColorU32(ImGuiCol.Text));
   }

   private void receiveRobotConfigurationData()
   {
      for (RobotSide side : RobotSide.values)
      {
         if (syncedRobot.getRobotModel().getRobotVersion().hasArm(side))
         {
            for (int i = 0; i < armJointNamesArray.get(side).length; i++)
            {
               OneDoFJointBasics armJoint = syncedRobot.getFullRobotModel().getArmJoint(side, armJointNamesArray.get(side)[i]);
               armValuesFromRobot.get(side)[i] = armJoint.getQ();
            }
         }
         for (int i = 0; i < legJointNamesArray.get(side).length; i++)
         {
            OneDoFJointBasics legJoint = syncedRobot.getFullRobotModel().getLegJoint(side, legJointNamesArray.get(side)[i]);
            legValuesFromRobot.get(side)[i] = legJoint.getQ();
         }
      }
      for (int i = 0; i < spineJointNamesArray.length; i++)
      {
         OneDoFJointBasics spineJoint = syncedRobot.getFullRobotModel().getSpineJoint(spineJointNamesArray[i]);
         spineValuesFromRobot[i] = spineJoint.getQ();
      }
      if (syncedRobot.getRobotModel().getRobotVersion().hasHead())
      {
         for (int i = 0; i < neckJointNamesArray.length; i++)
         {
            OneDoFJointBasics neckJoint = syncedRobot.getFullRobotModel().getNeckJoint(neckJointNamesArray[i]);
            neckValuesFromRobot[i] = neckJoint.getQ();
         }
      }
   }

   public void resetToRobotValues()
   {
      for (RobotSide side : RobotSide.values)
      {
         if (syncedRobot.getRobotModel().getRobotVersion().hasArm(side))
         {
            for (int i = 0; i < armJointNamesArray.get(side).length; i++)
            {
               armJointWidgetValues.get(side)[i] = armValuesFromRobot.get(side)[i];
               desiredRobot.getDesiredFullRobotModel().getArmJoint(side, armJointNamesArray.get(side)[i]).setQ(armJointWidgetValues.get(side)[i]);
            }
         }
         for (int i = 0; i < legJointNamesArray.get(side).length; i++)
         {
            legJointWidgetValues.get(side)[i] = legValuesFromRobot.get(side)[i];
            desiredRobot.getDesiredFullRobotModel().getLegJoint(side, legJointNamesArray.get(side)[i]).setQ(legJointWidgetValues.get(side)[i]);
         }
      }
      for (int i = 0; i < spineJointNamesArray.length; i++)
      {
         spineJointWidgetValues[i] = spineValuesFromRobot[i];
         desiredRobot.getDesiredFullRobotModel().getSpineJoint(spineJointNamesArray[i]).setQ(spineJointWidgetValues[i]);
      }
      if (syncedRobot.getRobotModel().getRobotVersion().hasHead())
      {
         for (int i = 0; i < neckJointNamesArray.length; i++)
         {
            neckJointWidgetValues[i] = neckValuesFromRobot[i];
            desiredRobot.getDesiredFullRobotModel().getNeckJoint(neckJointNamesArray[i]).setQ(neckJointWidgetValues[i]);
         }
      }

      desiredRobot.getDesiredFullRobotModel().getRootJoint().setJointConfiguration(syncedRobot.getFullRobotModel().getRootJoint().getJointPose());
      desiredRobot.getDesiredFullRobotModel().updateFrames();
   }

   public void renderImGuiWidgets()
   {
      // The second column starts past the widest first column radio button so the columns line up
      float rowStartX = ImGui.getCursorPosX();
      float widestFirstColumnText = 0.0f;
      for (InteractableDoFs[] row : BODY_PART_ROWS)
         widestFirstColumnText = Math.max(widestFirstColumnText, ImGuiTools.calcTextSizeX(row[0].displayName));
      float secondColumnX = rowStartX + ImGui.getFrameHeight() + ImGui.getStyle().getItemInnerSpacingX() + widestFirstColumnText
                            + 2.0f * ImGui.getStyle().getItemSpacingX();
      for (int row = 0; row < BODY_PART_ROWS.length; row++)
      {
         if (row > 0)
            ImGui.setCursorPosX(rowStartX);
         for (int column = 0; column < BODY_PART_ROWS[row].length; column++)
         {
            InteractableDoFs part = BODY_PART_ROWS[row][column];
            if (column > 0)
            {
               ImGui.sameLine();
               ImGui.setCursorPosX(secondColumnX);
            }

            boolean isDisabled = shouldDisableOption(part.ordinal());
            if (isDisabled)
               ImGui.beginDisabled();
            ImGui.radioButton(labels.get(part.displayName), currentInteractablePart, part.ordinal());
            if (isDisabled)
               ImGui.endDisabled();
         }
      }

      if (currentInteractablePart.get() != previousInteractablePart.get())
      {
         receiveRobotConfigurationData();
         resetToRobotValues();
         desiredRobot.setActive(false);
      }

      InteractableDoFs selectedPart = InteractableDoFs.values()[currentInteractablePart.get()];
      if (selectedPart != InteractableDoFs.NONE)
      {
         // Slightly lighter than the regular buttons so the send action stands out from the sliders below
         pushLightenedStyleColor(ImGuiCol.Button);
         pushLightenedStyleColor(ImGuiCol.ButtonHovered);
         pushLightenedStyleColor(ImGuiCol.ButtonActive);
         if (ImGui.button(labels.get("Send to Robot")))
         {
            switch (selectedPart)
            {
               case NECK -> sendNeck();
               case CHEST -> sendChest();
               case LEFT_ARM -> sendArm(RobotSide.LEFT);
               case RIGHT_ARM -> sendArm(RobotSide.RIGHT);
               case LEFT_LEG -> sendLeg(RobotSide.LEFT);
               case RIGHT_LEG -> sendLeg(RobotSide.RIGHT);
               default -> throw new IllegalStateException("Unexpected value: " + selectedPart);
            }
         }
         ImGui.popStyleColor(3);

         // The actual angles are otherwise only refreshed on send and reset
         if (showActual.get())
            receiveRobotConfigurationData();
      }

      switch (selectedPart)
      {
         case NONE ->
         {
         }
         case CHEST -> renderChestWidgets();
         case NECK -> renderNeckWidgets();
         case LEFT_ARM -> renderArmWidgets(RobotSide.LEFT);
         case RIGHT_ARM -> renderArmWidgets(RobotSide.RIGHT);
         case LEFT_LEG -> renderLegWidgets(RobotSide.LEFT);
         case RIGHT_LEG -> renderLegWidgets(RobotSide.RIGHT);
         default -> throw new IllegalStateException("Unexpected value: " + selectedPart);
      }

      if (selectedPart != InteractableDoFs.NONE)
      {
         if (!Float.isNaN(sliderStartScreenX))
            ImGui.setCursorScreenPos(sliderStartScreenX, ImGui.getCursorScreenPosY());
         ImGui.checkbox(labels.get("Show actual"), showActual);
      }

      previousInteractablePart.set(currentInteractablePart);
   }

   private void pushLightenedStyleColor(int imGuiCol)
   {
      ImVec4 color = ImGui.getStyle().getColor(imGuiCol);
      float lighten = 0.25f; // Fraction of the way toward white
      ImGui.pushStyleColor(imGuiCol,
                           color.x + (1.0f - color.x) * lighten,
                           color.y + (1.0f - color.y) * lighten,
                           color.z + (1.0f - color.z) * lighten,
                           color.w);
   }

   private void sendNeck()
   {
      ros2ControllerHelper.publishToController(HumanoidMessageTools.createHeadHybridTrajectoryMessage(desiredRobot.getDesiredFullRobotModel().getHead().getBodyFixedFrame(),
                                                                                                      desiredRobot.getDesiredFullRobotModel().getChest().getBodyFixedFrame(),
                                                                                                      neckJointWidgetValues,
                                                                                                      teleoperationParameters.getTrajectoryTime()));
   }

   private void sendChest()
   {
      receiveRobotConfigurationData();
      buildAndPublishChestTrajectoryMessage();
   }

   private void sendArm(RobotSide side)
   {
      receiveRobotConfigurationData();
      double[] desiredValues = new double[armJointNamesArray.get(side).length];
      double maxChange = 0;
      for (int i =0; i < desiredValues.length; i++)
      {
         desiredValues[i] = armJointWidgetValues.get(side)[i];
         double change = Math.abs(desiredValues[i] - armValuesFromRobot.get(side)[i]);
         if (change > maxChange)
            maxChange = change;
      }
      double trajectoryTime = maxChange/2 * teleoperationParameters.getTrajectoryTime();
      trajectoryTime = MathTools.clamp(trajectoryTime, 0.5, 20.0); // Safety

      JointspaceTrajectoryMessage jointspaceTrajectoryMessage = buildArmJointspaceTrajectoryMessage(trajectoryTime, side);
      publishArmJointspaceCommand(jointspaceTrajectoryMessage, side);
   }

   private void sendLeg(RobotSide side)
   {
      receiveRobotConfigurationData();
      double[] desiredValues = new double[legJointNamesArray.get(side).length];
      double maxChange = 0;
      for (int i =0; i < desiredValues.length; i++)
      {
         desiredValues[i] = legJointWidgetValues.get(side)[i];
         double change = Math.abs(desiredValues[i] - legValuesFromRobot.get(side)[i]);
         if (change > maxChange)
            maxChange = change;
      }
      double trajectoryTime = maxChange * teleoperationParameters.getTrajectoryTime();
      trajectoryTime = MathTools.clamp(trajectoryTime, 0.5, 20.0); // Safety

      JointspaceTrajectoryMessage jointspaceTrajectoryMessage = buildLegJointspaceTrajectoryMessage(trajectoryTime, side);
      publishLegJointspaceCommand(jointspaceTrajectoryMessage, side);
   }

   private boolean shouldDisableOption(int partId)
   {
      if (partId == InteractableDoFs.LEFT_ARM.ordinal() && !syncedRobot.getRobotModel().getRobotVersion().hasArm(RobotSide.LEFT) ||
          partId == InteractableDoFs.RIGHT_ARM.ordinal() && !syncedRobot.getRobotModel().getRobotVersion().hasArm(RobotSide.RIGHT) ||
          partId == InteractableDoFs.NECK.ordinal() && !syncedRobot.getRobotModel().getRobotVersion().hasHead())
      {
         return true;
      }
      return false;
   }

   private void renderNeckWidgets()
   {
      ImGui.pushItemWidth(80.0f);
      for (int i = 0; i < neckJointNamesArray.length; i++)
      {
         neckJointAngleWidgets[i].renderImGuiWidget();
      }
      ImGui.popItemWidth();
      receiveRobotConfigurationData();
      updateDesiredRobotNeck();
   }

   private void renderChestWidgets()
   {
      ImGui.pushItemWidth(80.0f);
      for (int i = 0; i < spineJointNamesArray.length; i++)
      {
         spineJointAngleWidgets[i].renderImGuiWidget();
      }
      ImGui.popItemWidth();
      updateDesiredRobotChest();
   }

   private void renderArmWidgets(RobotSide side)
   {
      ImGui.pushItemWidth(80.0f);
      for (int i = 0; i < armJointNamesArray.get(side).length; i++)
      {
         armJointAngleWidgets.get(side)[i].renderImGuiWidget();
      }
      ImGui.popItemWidth();
      updateDesiredRobotArm(side);
   }

   private void updateDesiredRobotArm(RobotSide side)
   {
      desiredRobot.setActive(true);
      desiredRobot.setWholeBodyColor(Color.ORANGE);
      desiredRobot.setArmShowing(side, true);
      for (int i = 0; i < armJointNamesArray.get(side).length; i++)
      {
         desiredRobot.getDesiredFullRobotModel().getArmJoint(side, armJointNamesArray.get(side)[i]).setQ(armJointWidgetValues.get(side)[i]);
      }
      desiredRobot.getDesiredFullRobotModel().updateFrames();
   }

   private void renderLegWidgets(RobotSide side)
   {
      ImGui.pushItemWidth(80.0f);
      for (int i = 0; i < legJointNamesArray.get(side).length; i++)
      {
         legJointAngleWidgets.get(side)[i].renderImGuiWidget();
      }
      ImGui.popItemWidth();
      updateDesiredRobotLeg(side);
   }

   private void updateDesiredRobotLeg(RobotSide side)
   {
      desiredRobot.setActive(true);
      desiredRobot.setWholeBodyColor(Color.ORANGE);
      desiredRobot.setLegShowing(side, true);
      for (int i = 0; i < legJointNamesArray.get(side).length; i++)
      {
         desiredRobot.getDesiredFullRobotModel().getLegJoint(side, legJointNamesArray.get(side)[i]).setQ(legJointWidgetValues.get(side)[i]);
      }
      desiredRobot.getDesiredFullRobotModel().updateFrames();
   }

   private void updateDesiredRobotChest()
   {
      desiredRobot.setActive(true);
      desiredRobot.setWholeBodyColor(Color.ORANGE);
      desiredRobot.setChestShowing(true);
      for (int i = 0; i < spineJointNamesArray.length; i++)
      {
         desiredRobot.getDesiredFullRobotModel().getSpineJoint(spineJointNamesArray[i]).setQ(spineJointWidgetValues[i]);
      }
      desiredRobot.getDesiredFullRobotModel().updateFrames();
   }

   private void updateDesiredRobotNeck()
   {
      desiredRobot.setActive(true);
      desiredRobot.setWholeBodyColor(Color.ORANGE);
      desiredRobot.setHeadShowing(true);
      for (int i = 0; i < neckJointNamesArray.length; i++)
      {
         desiredRobot.getDesiredFullRobotModel().getNeckJoint(neckJointNamesArray[i]).setQ(neckJointWidgetValues[i]);
      }
      desiredRobot.getDesiredFullRobotModel().updateFrames();
   }

   private JointspaceTrajectoryMessage buildArmJointspaceTrajectoryMessage(double trajectoryTime, RobotSide side)
   {
      JointspaceTrajectoryMessage jointspaceTrajectoryMessage = new JointspaceTrajectoryMessage();
      jointspaceTrajectoryMessage.getQueueingProperties().setExecutionMode(QueueableMessage.EXECUTION_MODE_OVERRIDE);

      double[] jointAngles = new double[syncedRobot.getRobotModel().getJointMap().getArmJointNamesAsStrings(side).size()];

      for (int i = 0; i < jointAngles.length; i++)
         jointAngles[i] = armJointWidgetValues.get(side)[i];

      for (double q : jointAngles)
      {
         OneDoFJointTrajectoryMessage oneDoFJointTrajectoryMessage = jointspaceTrajectoryMessage.getJointTrajectoryMessages().add();
         TrajectoryPoint1DMessage trajectoryPoint1DMessage = oneDoFJointTrajectoryMessage.getTrajectoryPoints().add();
         trajectoryPoint1DMessage.setTime(trajectoryTime);
         trajectoryPoint1DMessage.setPosition(q);
         trajectoryPoint1DMessage.setVelocity(0.0);
      }

      return jointspaceTrajectoryMessage;
   }

   private void publishArmJointspaceCommand(JointspaceTrajectoryMessage jointspaceTrajectoryMessage, RobotSide side)
   {
      ArmTrajectoryMessage armTrajectoryMessage = new ArmTrajectoryMessage();
      armTrajectoryMessage.setRobotSide(side.toByte());
      armTrajectoryMessage.getJointspaceTrajectory().set(jointspaceTrajectoryMessage);
      LogTools.info("Publishing arm jointspace trajectory");
      ros2ControllerHelper.publishToController(armTrajectoryMessage);
   }

   private JointspaceTrajectoryMessage buildLegJointspaceTrajectoryMessage(double trajectoryTime, RobotSide side)
   {
      JointspaceTrajectoryMessage jointspaceTrajectoryMessage = new JointspaceTrajectoryMessage();
      jointspaceTrajectoryMessage.getQueueingProperties().setExecutionMode(QueueableMessage.EXECUTION_MODE_OVERRIDE);

      double[] jointAngles = new double[syncedRobot.getRobotModel().getJointMap().getLegJointNamesAsStrings(side).size()];

      for (int i = 0; i < jointAngles.length; i++)
         jointAngles[i] = legJointWidgetValues.get(side)[i];

      for (double q : jointAngles)
      {
         OneDoFJointTrajectoryMessage oneDoFJointTrajectoryMessage = jointspaceTrajectoryMessage.getJointTrajectoryMessages().add();
         TrajectoryPoint1DMessage trajectoryPoint1DMessage = oneDoFJointTrajectoryMessage.getTrajectoryPoints().add();
         trajectoryPoint1DMessage.setTime(trajectoryTime);
         trajectoryPoint1DMessage.setPosition(q);
         trajectoryPoint1DMessage.setVelocity(0.0);
      }

      return jointspaceTrajectoryMessage;
   }

   private void publishLegJointspaceCommand(JointspaceTrajectoryMessage jointspaceTrajectoryMessage, RobotSide side)
   {
      LegTrajectoryMessage legTrajectoryMessage = new LegTrajectoryMessage();
      legTrajectoryMessage.setRobotSide(side.toByte());
      legTrajectoryMessage.getJointspaceTrajectory().set(jointspaceTrajectoryMessage);
      LogTools.info("Publishing leg jointspace trajectory");
      ros2ControllerHelper.publishToController(legTrajectoryMessage);
   }

   private void buildAndPublishChestTrajectoryMessage()
   {
      FrameYawPitchRoll frameChestYawPitchRoll = new FrameYawPitchRoll(syncedRobot.getReferenceFrames().getChestFrame());
      frameChestYawPitchRoll.changeFrame(syncedRobot.getReferenceFrames().getPelvisZUpFrame());
      for (int i = 0; i < spineJointNamesArray.length; i++)
      {
         switch (spineJointNamesArray[i])
         {
            case SPINE_PITCH -> frameChestYawPitchRoll.setPitch(spineJointWidgetValues[i]);
            case SPINE_YAW -> frameChestYawPitchRoll.setYaw(spineJointWidgetValues[i]);
            case SPINE_ROLL -> frameChestYawPitchRoll.setRoll(spineJointWidgetValues[i]);
         }
      }

      ChestTrajectoryMessage message = new ChestTrajectoryMessage();
      message.getSo3Trajectory()
             .set(HumanoidMessageTools.createSO3TrajectoryMessage(teleoperationParameters.getTrajectoryTime(),
                                                                  frameChestYawPitchRoll,
                                                                  EuclidCoreTools.zeroVector3D,
                                                                  syncedRobot.getReferenceFrames().getPelvisZUpFrame()));
      long frameId = MessageTools.toFrameId(syncedRobot.getReferenceFrames().getPelvisZUpFrame());
      message.getSo3Trajectory().getFrameInformation().setDataReferenceFrameId(frameId);
      ros2ControllerHelper.publishToController(message);
   }

   public RDXDesiredRobot getDesiredRobot()
   {
      return desiredRobot;
   }

   public void reset()
   {
      currentInteractablePart.set(InteractableDoFs.NONE.ordinal());
      desiredRobot.setActive(false);
   }

   public void destroy()
   {
      desiredRobot.destroy();
   }
}