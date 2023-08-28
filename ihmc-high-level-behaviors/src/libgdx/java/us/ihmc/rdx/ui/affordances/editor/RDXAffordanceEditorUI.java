package us.ihmc.rdx.ui.affordances.editor;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiCol;

import imgui.type.ImBoolean;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.perception.sceneGraph.PredefinedSceneNodeLibrary;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiInputText;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.interactable.RDXInteractableSakeGripper;
import us.ihmc.rdx.ui.interactable.RDXInteractableObjectBuilder;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.tools.io.*;

import java.util.*;

public class RDXAffordanceEditorUI
{
   private static final double DEFAULT_DURATION = 1.0;
   private static final double LINEAR_VELOCITY = 0.1;
   private static final double ANGULAR_VELOCITY = 1.0; // [rad/s] for the sake gripper this is ~= to 0.1 m/s for a point on the edge of the gripper
   private static final SideDependentList<ColorDefinition> HAND_COLORS;

   static
   {
      HAND_COLORS = new SideDependentList<>();
      HAND_COLORS.put(RobotSide.LEFT, ColorDefinitions.SandyBrown());
      HAND_COLORS.put(RobotSide.RIGHT, ColorDefinitions.SlateBlue());
   }

   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private RobotSide[] activeSide;

   private final SideDependentList<RDXInteractableSakeGripper> interactableHands = new SideDependentList<>();
   private final SideDependentList<RigidBodyTransform> handTransformsToWorld = new SideDependentList<>();
   private final SideDependentList<FramePose3D> handPoses = new SideDependentList<>();
   private RDXInteractableObjectBuilder objectBuilder;
   private final PoseReferenceFrame objectFrame = new PoseReferenceFrame("objectFrame", ReferenceFrame.getWorldFrame());
   private ReferenceFrame initialObjectFrame;
   private ModifiableReferenceFrame affordanceFrame = new ModifiableReferenceFrame("affordanceFrame", ReferenceFrame.getWorldFrame());
   private final float[] gripperClosure = new float[1];

   // affordance poses
   private RDXAffordanceFrame graspFrame;
   private RDXAffordanceFrames preGraspFrames;
   private RDXAffordanceFrames postGraspFrames;

   // mirroring
   private final ImBoolean mirrorActive = new ImBoolean(false);
   private final Map<String, Boolean> activeTransformAxisMirror = new LinkedHashMap<>();
   private boolean negatedAxis = false;
   private final RigidBodyTransform frameActiveSideTransform =  new RigidBodyTransform();
   private final ReferenceFrame frameActiveSide = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                                                         frameActiveSideTransform);
   private final RigidBodyTransform frameNonActiveSideTransform =  new RigidBodyTransform();
   private final ReferenceFrame frameNonActiveSide = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                                                            frameNonActiveSideTransform);
   private FramePose3DReadOnly lastActiveHandPose = new FramePose3D(frameActiveSide);

   // locking to object
   private SideDependentList<Boolean> affordancePoseLocked = new SideDependentList<>();
   private SideDependentList<Boolean> handsLocked = new SideDependentList<>();
   private final SideDependentList<PoseReferenceFrame> handLockedFrames = new SideDependentList<>();

   private RDXActiveAffordanceMenu[] activeMenu;
   private boolean playing = false;

   private final ImGuiInputText textInput = new ImGuiInputText("Enter file name to save/load");
   private String fileName = "";
   private final WorkspaceResourceDirectory configurationsDirectory = new WorkspaceResourceDirectory(getClass(), "/affordances");
   private final ArrayList<double[]> csvDataMatrix = new ArrayList<>();

   public RDXAffordanceEditorUI()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getPrimaryScene().addModelInstance(new ModelInstance(RDXModelBuilder.createCoordinateFrame(0.3)));
            objectBuilder = new RDXInteractableObjectBuilder(baseUI, PredefinedSceneNodeLibrary.defaultObjects());
            baseUI.getImGuiPanelManager().addPanel(objectBuilder.getWindowName(), objectBuilder::renderImGuiWidgets);

            for (RobotSide side : RobotSide.values)
            {
               handTransformsToWorld.put(side, new RigidBodyTransform());
               handTransformsToWorld.get(side).getRotation().setYawPitchRoll(0.0, Math.toRadians(-90.0), 0.0);
               handTransformsToWorld.get(side).getTranslation().set(-0.5, side.negateIfRightSide(0.2), 0);
               interactableHands.put(side,
                                     new RDXInteractableSakeGripper(baseUI.getPrimary3DPanel(),
                                                                    handTransformsToWorld.get(side),
                                                                    new ColorDefinition(HAND_COLORS.get(side).getRed(),
                                                                                        HAND_COLORS.get(side).getGreen(),
                                                                                        HAND_COLORS.get(side).getBlue(),
                                                                                        0.8)));
               handPoses.put(side, new FramePose3D(ReferenceFrame.getWorldFrame(), handTransformsToWorld.get(side)));

               handsLocked.put(side, false);
               affordancePoseLocked.put(side, false);
            }

            activeSide = new RobotSide[1];
            activeSide[0] = RobotSide.RIGHT;
            activeMenu = new RDXActiveAffordanceMenu[1];
            activeMenu[0] = RDXActiveAffordanceMenu.PRE_GRASP;
            preGraspFrames = new RDXAffordanceFrames(interactableHands,
                                                     handTransformsToWorld,
                                                     handPoses,
                                                     objectBuilder.getSelectedObject().getTransformToWorld(),
                                                     activeSide,
                                                     activeMenu,
                                                     new ArrayList<>(Arrays.asList(new Color(0xFFE4B5FF),
                                                                                   new Color(0xFF8C00FF),
                                                                                   new Color(0xFFDAB9FF),
                                                                                   new Color(0xFF6600FF),
                                                                                   new Color(0xFFA07AFF))));
            activeMenu[0] = RDXActiveAffordanceMenu.GRASP;
            graspFrame = new RDXAffordanceFrame(interactableHands,
                                                handTransformsToWorld,
                                                handPoses,
                                                objectBuilder.getSelectedObject().getTransformToWorld(),
                                                activeSide,
                                                activeMenu,
                                                Color.BLACK);
            activeMenu[0] = RDXActiveAffordanceMenu.POST_GRASP;
            postGraspFrames = new RDXAffordanceFrames(interactableHands,
                                                      handTransformsToWorld,
                                                      handPoses,
                                                      objectBuilder.getSelectedObject().getTransformToWorld(),
                                                      activeSide,
                                                      activeMenu,
                                                      new ArrayList<>(Arrays.asList(new Color(0xD8BFD8FF),
                                                                                    new Color(0xBA55D3FF),
                                                                                    new Color(0x9932CCFF),
                                                                                    new Color(0x8A2BE2FF),
                                                                                    new Color(0x4B0082FF))));
            activeMenu[0] = RDXActiveAffordanceMenu.NONE;
            for (RobotSide side : RobotSide.values)
            {
               baseUI.getPrimaryScene().addRenderableProvider(interactableHands.get(side));
               baseUI.getImGuiPanelManager().addPanel(interactableHands.get(side).getPose3DGizmo().createTunerPanel(side.getCamelCaseName() + " Hand"));
            }
            baseUI.getPrimaryScene().addRenderableProvider(objectBuilder.getSelectedObject());
            baseUI.getPrimaryScene().addRenderableProvider(RDXAffordanceEditorUI.this::getRenderables);
            baseUI.getImGuiPanelManager().addPanel("Affordance Panel", RDXAffordanceEditorUI.this::renderImGuiWidgets);

            activeTransformAxisMirror.put("X", false);
            activeTransformAxisMirror.put("Y", false);
            activeTransformAxisMirror.put("Z", false);
            activeTransformAxisMirror.put("Yaw", false);
            activeTransformAxisMirror.put("Pitch", false);
            activeTransformAxisMirror.put("Roll", false);
         }

         @Override
         public void render()
         {
            update();
            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
         }
      });
   }

   public void update()
   {
      if (objectBuilder.isAnyObjectSelected())
      {
         FramePose3D objectPose = new FramePose3D(ReferenceFrame.getWorldFrame(), objectBuilder.getSelectedObject().getTransformToWorld());
         // when editing post grasp poses, we want to move the object frame and the hand together
         for (RobotSide side : handPoses.keySet())
         {
            if (activeMenu[0] == RDXActiveAffordanceMenu.POST_GRASP && handsLocked.get(side))
            {
               // used to update the hand pose according to object pose in post-grasping once fixed contact with object
               if (!affordancePoseLocked.get(side))
               {
                  objectFrame.setPoseAndUpdate(objectPose);
                  handLockedFrames.put(side, new PoseReferenceFrame(side.getLowerCaseName() + "HandFrame", objectFrame));
                  handPoses.get(side).changeFrame(objectFrame);
                  handLockedFrames.get(side).setPoseAndUpdate(handPoses.get(side));
                  handPoses.get(side).changeFrame(ReferenceFrame.getWorldFrame());
                  affordancePoseLocked.replace(side, true);
               }
               objectFrame.setPoseAndUpdate(objectPose);
               FramePose3D pose = new FramePose3D(handLockedFrames.get(side));
               pose.changeFrame(ReferenceFrame.getWorldFrame());
               handTransformsToWorld.get(side).set(pose.getOrientation(), pose.getTranslation());
            }
            else
            {
               handsLocked.replace(side, false);
               affordancePoseLocked.replace(side, false);
            }
         }
      }

      for (RobotSide side : handPoses.keySet())
      {
         // update hand poses according to where the hand is
         handPoses.get(side).changeFrame(ReferenceFrame.getWorldFrame());
         handPoses.get(side).set(handTransformsToWorld.get(side));

         // selected hand determines active side
         if (interactableHands.get(side).isSelected())
         {
            activeSide[0] = side;
         }
      }

      // mirroring
      if (mirrorActive.get())
      {
         RobotSide nonActiveSide = (activeSide[0] == RobotSide.RIGHT ? RobotSide.LEFT : RobotSide.RIGHT);
         FramePose3D activeHandPose = new FramePose3D(handPoses.get(activeSide[0]));
         activeHandPose.changeFrame(frameActiveSide);

         FramePose3D nonActiveHandPose = new FramePose3D(handPoses.get(nonActiveSide));
         nonActiveHandPose.changeFrame(frameNonActiveSide);

         Point3DBasics activeSideTranslationIncrement = new FramePoint3D();
         YawPitchRoll activeSideYawPitchRollIncrement = new YawPitchRoll();
         for (var axis : activeTransformAxisMirror.entrySet())
         {
            if (axis.getValue())
            {
               switch (axis.getKey())
               {
                  case "X" ->
                  {
                     if (negatedAxis)
                        activeSideTranslationIncrement.setX(-activeHandPose.getTranslation().getX() + lastActiveHandPose.getTranslation().getX());
                     else
                        activeSideTranslationIncrement.setX(activeHandPose.getTranslation().getX() - lastActiveHandPose.getTranslation().getX());
                  }
                  case "Y" ->
                  {
                     if (negatedAxis)
                        activeSideTranslationIncrement.setY(-activeHandPose.getTranslation().getY() + lastActiveHandPose.getTranslation().getY());
                     else
                        activeSideTranslationIncrement.setY(activeHandPose.getTranslation().getY() - lastActiveHandPose.getTranslation().getY());
                  }
                  case "Z" ->
                  {
                     if (negatedAxis)
                        activeSideTranslationIncrement.setZ(-activeHandPose.getTranslation().getZ() + lastActiveHandPose.getTranslation().getZ());
                     else
                        activeSideTranslationIncrement.setZ(activeHandPose.getTranslation().getZ() - lastActiveHandPose.getTranslation().getZ());
                  }
                  case "Yaw" ->
                  {
                     if (negatedAxis)
                        activeSideYawPitchRollIncrement.setYaw(-activeHandPose.getYaw() + lastActiveHandPose.getYaw());
                     else
                        activeSideYawPitchRollIncrement.setYaw(activeHandPose.getYaw() - lastActiveHandPose.getYaw());
                  }
                  case "Pitch" ->
                  {
                     if (negatedAxis)
                        activeSideYawPitchRollIncrement.setPitch(-activeHandPose.getPitch() + lastActiveHandPose.getPitch());
                     else
                        activeSideYawPitchRollIncrement.setPitch(activeHandPose.getPitch() - lastActiveHandPose.getPitch());
                  }
                  case "Roll" ->
                  {
                     if (negatedAxis)
                        activeSideYawPitchRollIncrement.setRoll(-activeHandPose.getRoll() + lastActiveHandPose.getRoll());
                     else
                        activeSideYawPitchRollIncrement.setRoll(activeHandPose.getRoll() - lastActiveHandPose.getRoll());
                  }
               }
            }
         }
         RigidBodyTransform nonActiveSideTransform = new RigidBodyTransform(activeSideYawPitchRollIncrement, activeSideTranslationIncrement);
         if (!nonActiveSideTransform.geometricallyEquals(new RigidBodyTransform(), 0.001))
         {
            nonActiveHandPose.applyTransform(nonActiveSideTransform);
            nonActiveHandPose.changeFrame(ReferenceFrame.getWorldFrame());
            handTransformsToWorld.get(nonActiveSide).set(nonActiveHandPose);
            lastActiveHandPose = new FramePose3D(frameActiveSide, activeHandPose);
         }
      }

      graspFrame.update();
      preGraspFrames.update();
      postGraspFrames.update();

      if (handPoses.containsKey(activeSide[0]))
         gripperClosure[0] = interactableHands.get(activeSide[0]).getGripperClosure();
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("HANDS MENU");
      ColorDefinition handColor = HAND_COLORS.get(RobotSide.LEFT);
      ImGui.pushStyleColor(ImGuiCol.CheckMark,
                           (float) handColor.getRed(),
                           (float) handColor.getGreen(),
                           (float) handColor.getBlue(),
                           (float) handColor.getAlpha());
      if (ImGui.radioButton(labels.get("LEFT"), activeSide[0] == RobotSide.LEFT))
      {
         activeSide[0] = RobotSide.LEFT;
         interactableHands.get(RobotSide.LEFT).setSelected(true);
         if (handPoses.containsKey(RobotSide.RIGHT))
            interactableHands.get(RobotSide.RIGHT).setSelected(false);
      }
      ImGui.popStyleColor();
      ImGui.sameLine();
      handColor = HAND_COLORS.get(RobotSide.RIGHT);
      ImGui.pushStyleColor(ImGuiCol.CheckMark,
                           (float) handColor.getRed(),
                           (float) handColor.getGreen(),
                           (float) handColor.getBlue(),
                           (float) handColor.getAlpha());
      if (ImGui.radioButton(labels.get("RIGHT"), activeSide[0] == RobotSide.RIGHT))
      {
         activeSide[0] = RobotSide.RIGHT;
         interactableHands.get(RobotSide.RIGHT).setSelected(true);
         if (handPoses.containsKey(RobotSide.LEFT))
            interactableHands.get(RobotSide.LEFT).setSelected(false);
      }
      ImGui.popStyleColor();
      ImGui.sameLine();
      for (RobotSide side : RobotSide.values)
      {
         if (activeSide[0] == side && !handPoses.containsKey(side))
         {
            if (ImGui.button(labels.get("ADD") + "##side"))
            {
               interactableHands.get(side).setShowing(true);
               handTransformsToWorld.get(side).getRotation().setYawPitchRoll(0.0, Math.toRadians(-90.0), 0.0);
               handTransformsToWorld.get(side).getTranslation().set(-0.5, side.negateIfRightSide(0.2), 0);
               handPoses.put(side, new FramePose3D(ReferenceFrame.getWorldFrame(), handTransformsToWorld.get(side)));
            }
         }
         else if (activeSide[0] == side && handPoses.containsKey(side))
         {
            if (ImGui.button(labels.get("REMOVE") + "##side"))
            {
               interactableHands.get(side).setShowing(false);
               handPoses.remove(side);
            }
         }
      }

      if (handPoses.containsKey(activeSide[0]))
      {
         ImGui.text("Hand configuration: ");
         if (ImGui.button(labels.get("OPEN")))
            interactableHands.get(activeSide[0]).openGripper();
         ImGui.sameLine();
         if (ImGui.button(labels.get("HALF_CLOSE")))
            interactableHands.get(activeSide[0]).setGripperToHalfClose();
         ImGui.sameLine();
         if (ImGui.button(labels.get("CLOSE")))
            interactableHands.get(activeSide[0]).closeGripper();
         ImGui.sameLine();
         if (ImGui.button(labels.get("CRUSH")))
            interactableHands.get(activeSide[0]).crushGripper();
         if (ImGui.sliderFloat("SET CLOSURE",
                               gripperClosure,
                               interactableHands.get(activeSide[0]).getMinGripperClosure(),
                               interactableHands.get(activeSide[0]).getMaxGripperClosure()))
            interactableHands.get(activeSide[0]).setGripperClosure(gripperClosure[0]);
         ImGui.separator();
      }

      if (handPoses.containsKey(RobotSide.RIGHT) && handPoses.containsKey(RobotSide.LEFT))
      {
         if (ImGui.checkbox("Mirror Other Hand", mirrorActive))
            resetMirror();
         if (ImGui.button("Set Reference Frame Axis"))
         {
            setReferenceFrameMirror();
         }
         ImGui.text("Transform: ");
         Map<String, Boolean> changedColorTranslationAxisButton = new HashMap<>();
         for (var axisMirror : activeTransformAxisMirror.entrySet())
         {
            changedColorTranslationAxisButton.put(axisMirror.getKey(), false);
            ImGui.sameLine();
            if (axisMirror.getValue())
            {
               ImGui.pushStyleColor(ImGuiCol.Button, 0.0f, 0.0f, 1.0f, 0.5f);
               changedColorTranslationAxisButton.replace(axisMirror.getKey(), true);
            }
            if (ImGui.button(labels.get(axisMirror.getKey()) + "##" + "Transform"))
            {
               axisMirror.setValue(!axisMirror.getValue());
               setReferenceFrameMirror();
            }
            if (changedColorTranslationAxisButton.get(axisMirror.getKey()))
               ImGui.popStyleColor();
         }
         if (ImGui.button(labels.get("Switch Direction")))
         {
            negatedAxis = !negatedAxis;
         }

         ImGui.separator();
      }
      else
      {
         mirrorActive.set(false);
         resetMirror();
      }

      ImGui.text("PRE-GRASP MENU");
      ImGui.text("Pre-grasp Frames: ");
      ImGui.sameLine();
      preGraspFrames.renderImGuiWidgets(labels, "pregrasp");
      ImGui.separator();

      ImGui.text("GRASP MENU");
      ImGui.text("Grasp Frame: ");
      ImGui.sameLine();
      graspFrame.renderImGuiWidgets(labels, "grasp");
      ImGui.separator();

      ImGui.text("POST-GRASP MENU");
      ImGui.text("Post-grasp Frames: ");
      ImGui.sameLine();
      postGraspFrames.renderImGuiWidgets(labels, "postgrasp");
      boolean changedColorLockOneHand = false;
      if (handPoses.containsKey(activeSide[0]))
      {
         if (handsLocked.get(activeSide[0]))
         {
            ImGui.pushStyleColor(ImGuiCol.Button, 0.0f, 0.0f, 1.0f, 0.5f);
            changedColorLockOneHand = true;
         }
         if (!(handsLocked.get(RobotSide.LEFT) && handsLocked.get(RobotSide.RIGHT)))
         {
            if (ImGui.button(labels.get("LOCK HAND TO OBJECT")) && activeMenu[0] == RDXActiveAffordanceMenu.POST_GRASP)
               handsLocked.replace(activeSide[0], !handsLocked.get(activeSide[0]));
         }
         if (changedColorLockOneHand)
            ImGui.popStyleColor();

         boolean changedColorLockBothHands = false;
         if ((handsLocked.get(RobotSide.LEFT) && handsLocked.get(RobotSide.RIGHT)))
         {
            ImGui.pushStyleColor(ImGuiCol.Button, 0.0f, 0.0f, 1.0f, 0.5f);
            changedColorLockBothHands = true;
         }
         if (!((handsLocked.get(RobotSide.LEFT) && !handsLocked.get(RobotSide.RIGHT)) || (!handsLocked.get(RobotSide.LEFT)
                                                                                          && handsLocked.get(RobotSide.RIGHT))))
         { // not in alternate state, this means single hand lock is not activate
            if (ImGui.button(labels.get("LOCK BOTH HANDS TO OBJECT")) && activeMenu[0] == RDXActiveAffordanceMenu.POST_GRASP)
            {
               for (RobotSide side : RobotSide.values)
                  handsLocked.replace(side, !handsLocked.get(side));
            }
            if (changedColorLockBothHands)
               ImGui.popStyleColor();
         }
      }
      ImGui.separator();

      ImGui.text("PREVIOUS/NEXT FRAME: ");
      ImGui.sameLine();
      if (ImGui.button(labels.get("<"), 20, 25))
      {
         switch (activeMenu[0])
         {
            case PRE_GRASP ->
            {
               if (!preGraspFrames.isFirst())
                  preGraspFrames.selectPrevious();
            }
            case GRASP ->
            {
               if (preGraspFrames.getNumberOfFrames() > 0)
               {
                  activeMenu[0] = RDXActiveAffordanceMenu.PRE_GRASP;
                  preGraspFrames.setSelectedIndexToSize();
                  preGraspFrames.selectPrevious();
               }
            }
            case POST_GRASP ->
            {
               if (!postGraspFrames.isFirst())
                  postGraspFrames.selectPrevious();
               else
               {
                  if (graspFrame.isSet(activeSide[0]))
                  {
                     activeMenu[0] = RDXActiveAffordanceMenu.GRASP;
                     graspFrame.selectFrame();
                  }
                  else if (preGraspFrames.getNumberOfFrames() > 0)
                  {
                     activeMenu[0] = RDXActiveAffordanceMenu.PRE_GRASP;
                     preGraspFrames.setSelectedIndexToSize();
                     preGraspFrames.selectPrevious();
                  }
               }
            }
         }
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get(">"), 20, 25) || playing)
      {
         if (playing)
         {
            try
            {
               Thread.sleep(112); // about 9Hz
            }
            catch (InterruptedException e)
            {
            }
         }
         switch (activeMenu[0])
         {
            case PRE_GRASP ->
            {
               if (!preGraspFrames.isLast())
                  preGraspFrames.selectNext();
               else
               {
                  if (graspFrame.isSet(activeSide[0]))
                  {
                     activeMenu[0] = RDXActiveAffordanceMenu.GRASP;
                     graspFrame.selectFrame();
                  }
                  else if (postGraspFrames.getNumberOfFrames() > 0)
                  {
                     activeMenu[0] = RDXActiveAffordanceMenu.POST_GRASP;
                     postGraspFrames.resetSelectedIndex();
                     postGraspFrames.selectNext();
                  }
               }
            }
            case GRASP ->
            {
               if (postGraspFrames.getNumberOfFrames() > 0)
               {
                  activeMenu[0] = RDXActiveAffordanceMenu.POST_GRASP;
                  postGraspFrames.resetSelectedIndex();
                  postGraspFrames.selectNext();
               }
            }
            case POST_GRASP ->
            {
               if (!postGraspFrames.isLast())
                  postGraspFrames.selectNext();
               else
                  playing = false;
            }
         }
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("PLAY")))
      {
         playing = true;
      }
      ImGui.separator();

      if (ImGui.button("RESET"))
      {
         objectBuilder.getSelectedObject().resetToInitialPose();
         reset();
      }
      if (textInput.render())
      {
         fileName = textInput.getString();
      }
      //      if (ImGui.button("SAVE"))
      //      {
      //         saveToFile(fileName);
      //      }
      //      ImGui.sameLine();
      //      if (ImGui.button(labels.get("LOAD")))
      //      {
      //         objectBuilder.reset();
      //         reset();
      //         loadFromFile(fileName);
      //      }
   }

   private void reset()
   {
      for (RobotSide side : handPoses.keySet())
      {
         handTransformsToWorld.get(side).setToZero();
         handTransformsToWorld.get(side).getTranslation().set(-0.5, side.negateIfRightSide(0.2), 0);
         handTransformsToWorld.get(side).getRotation().setYawPitchRoll(0.0, Math.toRadians(-90.0), 0.0);
         interactableHands.get(side).closeGripper();
      }
      mirrorActive.set(false);
      resetMirror();

      graspFrame.reset();
      preGraspFrames.reset();
      postGraspFrames.reset();
      activeMenu[0] = RDXActiveAffordanceMenu.NONE;
   }

   private void resetMirror()
   {
      if (!mirrorActive.get())
      {
         lastActiveHandPose = new FramePose3D(frameActiveSide, new RigidBodyTransform());
         for (var axisMirror : activeTransformAxisMirror.entrySet())
            axisMirror.setValue(false);
      }
   }

   private void setReferenceFrameMirror()
   {
      RobotSide nonActiveSide = (activeSide[0] == RobotSide.RIGHT ? RobotSide.LEFT : RobotSide.RIGHT);
      frameActiveSideTransform.set((interactableHands.get(activeSide[0]).getReferenceFrameHand().getTransformToWorldFrame()));
      frameActiveSide.update();
      frameNonActiveSideTransform.set((interactableHands.get(nonActiveSide).getReferenceFrameHand().getTransformToWorldFrame()));
      frameNonActiveSide.update();

      lastActiveHandPose = new FramePose3D(frameActiveSide, new RigidBodyTransform());
   }

   private ArrayList<Double> computeTrajectoryDurations(ArrayList<FramePose3D> preGraspPoses, FramePose3D graspPose, ArrayList<FramePose3D> postGraspPoses)
   {
      ArrayList<Double> trajectoryDurations = new ArrayList<>();
      int sizePreGrasp = preGraspPoses.size();
      // compute trajectory duration as the necessary duration required to reach two consecutive frames at the desired velocity
      for (int i = 0; i < sizePreGrasp; i++)
      {
         if (i != 0)
         {
            double positionDistance = preGraspPoses.get(i).getPositionDistance(preGraspPoses.get(i - 1));
            double angularDistance = preGraspPoses.get(i).getOrientationDistance(preGraspPoses.get(i - 1));
            // take max duration required to achieve the desired linear velocity or angular velocity
            trajectoryDurations.add(Math.max(positionDistance / LINEAR_VELOCITY, angularDistance / ANGULAR_VELOCITY));
         }
         else
            trajectoryDurations.add(DEFAULT_DURATION);
      }

      if (sizePreGrasp > 0)
      {
         double positionDistance = graspPose.getPositionDistance(preGraspPoses.get(sizePreGrasp - 1));
         double angularDistance = graspPose.getOrientationDistance(preGraspPoses.get(sizePreGrasp - 1));
         // take max duration required to achieve the desired linear velocity or angular velocity
         trajectoryDurations.add(Math.max(positionDistance / LINEAR_VELOCITY, angularDistance / ANGULAR_VELOCITY));
      }
      else
         trajectoryDurations.add(DEFAULT_DURATION);

      for (int i = 0; i < postGraspPoses.size(); i++)
      {
         if (i != 0)
         {
            double positionDistance = postGraspPoses.get(i).getPositionDistance(postGraspPoses.get(i - 1));
            double angularDistance = postGraspPoses.get(i).getOrientationDistance(postGraspPoses.get(i - 1));
            // take max duration required to achieve the desired linear velocity or angular velocity
            trajectoryDurations.add(Math.max(positionDistance / LINEAR_VELOCITY, angularDistance / ANGULAR_VELOCITY));
         }
         else if (!graspFrame.isSet(activeSide[0]))
         {
            double positionDistance = postGraspPoses.get(i).getPositionDistance(graspPose);
            double angularDistance = postGraspPoses.get(i).getOrientationDistance(graspPose);
            // take max duration required to achieve the desired linear velocity or angular velocity
            trajectoryDurations.add(Math.max(positionDistance / LINEAR_VELOCITY, angularDistance / ANGULAR_VELOCITY));
         }
         else
            trajectoryDurations.add(DEFAULT_DURATION);
      }

      return trajectoryDurations;
   }

   //   public void saveToFile(String fileName)
   //   {
   //      // change affordance reference from world to initial object frame
   //      RigidBodyTransform initialObjectTransform = new RigidBodyTransform(objectBuilder.getSelectedObject().getInitialTransformToWorld());
   //      initialObjectFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(ReferenceFrame.getWorldFrame(), initialObjectTransform);
   //      affordanceFrame.changeParentFrame(initialObjectFrame);
   //
   //      WorkspaceResourceFile file = new WorkspaceResourceFile(configurationsDirectory, fileName + ".json");
   //      if (file.isFileAccessAvailable())
   //      {
   //         LogTools.info("Saving to file ...");
   //         JSONFileTools.save(file, jsonNode ->
   //         {
   //            jsonNode.put("name", fileName);
   //            ArrayNode actionsArrayNode = jsonNode.putArray("actions");
   //            var preGraspPoses = preGraspFrames.getPoses();
   //            var graspPose = graspFrame.getPose();
   //            var postGraspPoses = postGraspFrames.getPoses();
   //            ArrayList<Double> trajectoryDurations = computeTrajectoryDurations(preGraspPoses, graspPose, postGraspPoses);
   //            var preGraspHandConfigurations = preGraspFrames.getHandConfigurations();
   //            var postGraspHandConfigurations = postGraspFrames.getHandConfigurations();
   //            for (int i = 0; i < preGraspPoses.size(); i++)
   //            {
   //               ObjectNode actionNode = actionsArrayNode.addObject();
   //               actionNode.put("type", "RDXHandPoseAction");
   //               actionNode.put("parentFrame", objectBuilder.getSelectedObjectName());
   //               actionNode.put("side", side.getLowerCaseName());
   //               actionNode.put("trajectoryDuration", trajectoryDurations.get(i));
   //               preGraspPoses.get(i).changeFrame(affordanceFrame.getReferenceFrame());
   //               RigidBodyTransform transformToParent = new RigidBodyTransform(preGraspPoses.get(i));
   //               JSONTools.toJSON(actionNode, transformToParent);
   //
   //               double[] dataTrajectories = new double[16];
   //               transformToParent.get(dataTrajectories);
   //               csvDataMatrix.add(dataTrajectories);
   //
   //               if (preGraspHandConfigurations.get(i) != null)
   //               {
   //                  dataTrajectories = new double[16];
   //                  for (int data = 0; data < dataTrajectories.length; data++)
   //                     dataTrajectories[data] = 0.0;
   //                  dataTrajectories[0] = HandConfiguration.valueOf(preGraspHandConfigurations.get(i).toString()).ordinal();
   //                  csvDataMatrix.add(dataTrajectories);
   //
   //                  ObjectNode extraActionNode = actionsArrayNode.addObject();
   //                  extraActionNode.put("type", "RDXHandConfigurationAction");
   //                  extraActionNode.put("side", side.getLowerCaseName());
   //                  extraActionNode.put("grip", preGraspHandConfigurations.get(i).toString());
   //               }
   //            }
   //            if (graspFrame.isSet())
   //            {
   //               ObjectNode actionNode = actionsArrayNode.addObject();
   //               actionNode.put("type", "RDXHandPoseAction");
   //               actionNode.put("parentFrame", objectBuilder.getSelectedObjectName());
   //               actionNode.put("side", side.getLowerCaseName());
   //               actionNode.put("trajectoryDuration", trajectoryDurations.get(preGraspPoses.size()));
   //               graspPose.changeFrame(affordanceFrame.getReferenceFrame());
   //               RigidBodyTransform transformToParent = new RigidBodyTransform(graspPose);
   //               JSONTools.toJSON(actionNode, transformToParent);
   //
   //               double[] dataTrajectories = new double[16];
   //               transformToParent.get(dataTrajectories);
   //               csvDataMatrix.add(dataTrajectories);
   //
   //               if (graspFrame.getHandConfiguration() != null)
   //               {
   //                  ObjectNode extraActionNode = actionsArrayNode.addObject();
   //                  extraActionNode.put("type", "RDXHandConfigurationAction");
   //                  extraActionNode.put("side", side.getLowerCaseName());
   //                  extraActionNode.put("grip", graspFrame.getHandConfiguration().toString());
   //
   //                  dataTrajectories = new double[16];
   //                  for (int data = 0; data < dataTrajectories.length; data++)
   //                     dataTrajectories[data] = 0.0;
   //                  dataTrajectories[0] = HandConfiguration.valueOf(graspFrame.getHandConfiguration().toString()).ordinal();
   //                  csvDataMatrix.add(dataTrajectories);
   //               }
   //            }
   //            for (int i = 0; i < postGraspPoses.size(); i++)
   //            {
   //               ObjectNode actionNode = actionsArrayNode.addObject();
   //               actionNode.put("type", "RDXHandPoseAction");
   //               actionNode.put("parentFrame", objectBuilder.getSelectedObjectName());
   //               actionNode.put("side", side.getLowerCaseName());
   //               actionNode.put("trajectoryDuration", trajectoryDurations.get(preGraspPoses.size() + 1 + i));
   //               postGraspPoses.get(i).changeFrame(affordanceFrame.getReferenceFrame());
   //               RigidBodyTransform transformToParent = new RigidBodyTransform(postGraspPoses.get(i));
   //               JSONTools.toJSON(actionNode, transformToParent);
   //
   //               double[] dataTrajectories = new double[16];
   //               transformToParent.get(dataTrajectories);
   //               csvDataMatrix.add(dataTrajectories);
   //
   //               if (postGraspHandConfigurations.get(i) != null)
   //               {
   //                  dataTrajectories = new double[16];
   //                  for (int data = 0; data < dataTrajectories.length; data++)
   //                     dataTrajectories[data] = 0.0;
   //                  dataTrajectories[0] = HandConfiguration.valueOf(postGraspHandConfigurations.get(i).toString()).ordinal();
   //                  csvDataMatrix.add(dataTrajectories);
   //
   //                  ObjectNode extraActionNode = actionsArrayNode.addObject();
   //                  extraActionNode.put("type", "RDXHandConfigurationAction");
   //                  extraActionNode.put("side", side.getLowerCaseName());
   //                  extraActionNode.put("grip", postGraspHandConfigurations.get(i).toString());
   //               }
   //            }
   //         });
   //         LogTools.info("SAVED to file {}", file.getFileName());
   //      }
   //      else
   //      {
   //         LogTools.warn("Could not save to {}", file.getFileName());
   //      }
   //
   //      WorkspaceResourceFile extraFile = new WorkspaceResourceFile(configurationsDirectory, fileName + "Extra.json");
   //      if (extraFile.isFileAccessAvailable())
   //      {
   //         JSONFileTools.save(extraFile, jsonNode ->
   //         {
   //            jsonNode.put("name", fileName);
   //            jsonNode.put("object", objectBuilder.getSelectedObjectName());
   //            JSONTools.toJSON(jsonNode, new RigidBodyTransform(objectBuilder.getSelectedObject().getInitialPose()));
   //            ArrayNode framesArrayNode = jsonNode.putArray("frames");
   //            var preGraspObjectTransforms = preGraspFrames.getObjectTransforms();
   //            var preGraspHandConfigurations = preGraspFrames.getHandConfigurations();
   //            jsonNode.put("numberPreGraspFrames", preGraspObjectTransforms.size());
   //            for (int i = 0; i < preGraspObjectTransforms.size(); i++)
   //            {
   //               ObjectNode frameArray = framesArrayNode.addObject();
   //               JSONTools.toJSON(frameArray, preGraspObjectTransforms.get(i));
   //               frameArray.put("grip", preGraspHandConfigurations.get(i) == null ? "" : preGraspHandConfigurations.get(i).toString());
   //            }
   //            if (graspFrame.isSet())
   //            {
   //               ObjectNode frameArray = framesArrayNode.addObject();
   //               JSONTools.toJSON(frameArray, graspFrame.getObjectTransform());
   //               frameArray.put("grip", graspFrame.getHandConfiguration() == null ? "" : graspFrame.getHandConfiguration().toString());
   //            }
   //            var postGraspObjectTransforms = postGraspFrames.getObjectTransforms();
   //            var postGraspHandConfigurations = postGraspFrames.getHandConfigurations();
   //            jsonNode.put("numberPostGraspFrames", postGraspObjectTransforms.size());
   //            for (int i = 0; i < postGraspObjectTransforms.size(); i++)
   //            {
   //               ObjectNode frameArray = framesArrayNode.addObject();
   //               JSONTools.toJSON(frameArray, postGraspObjectTransforms.get(i));
   //               frameArray.put("grip", postGraspHandConfigurations.get(i) == null ? "" : postGraspHandConfigurations.get(i).toString());
   //            }
   //         });
   //         LogTools.info("SAVED to file {}", extraFile.getFileName());
   //      }
   //      else
   //      {
   //         LogTools.warn("Could not save extra info to {}", extraFile.getFileName());
   //      }
   //
   //      generateCSVFiles(fileName);
   //   }
   //
   //   public void generateCSVFiles(String fileName)
   //   {
   //      Path filePath = Paths.get(configurationsDirectory.getFilesystemDirectory().toString(), fileName + ".csv");
   //      File csvFile = new File(filePath.toString());
   //      try (PrintWriter writer = new PrintWriter(csvFile))
   //      {
   //         for (int row = 0; row < csvDataMatrix.size(); row++)
   //         {
   //            double[] dataLine = csvDataMatrix.get(row);
   //            for (int col = 0; col < dataLine.length; col++)
   //            {
   //               writer.print(dataLine[col]);
   //               if (col < dataLine.length - 1)
   //                  writer.append(",");
   //            }
   //            if (row < csvDataMatrix.size() - 1)
   //            {
   //               writer.println();
   //            }
   //         }
   //         LogTools.info("SAVED to file {}", csvFile.getName());
   //         csvDataMatrix.clear();
   //      }
   //      catch (IOException e)
   //      {
   //         e.printStackTrace();
   //      }
   //   }
   //
   //   public void loadFromFile(String fileName)
   //   {
   //      Path filePath = Paths.get(configurationsDirectory.getFilesystemDirectory().toString(), fileName + "Extra.json");
   //      final int[] preGraspFramesSize = new int[1];
   //      final int[] postGraspFramesSize = new int[1];
   //      if (Files.exists(filePath))
   //      {
   //         JSONFileTools.load(filePath, jsonNode ->
   //         {
   //            String objectName = jsonNode.get("object").asText();
   //            if (!objectName.isEmpty())
   //            {
   //               objectBuilder.loadObject(jsonNode.get("object").asText());
   //               RigidBodyTransform initialTransform = new RigidBodyTransform();
   //               JSONTools.toEuclid(jsonNode, initialTransform);
   //               objectBuilder.getSelectedObject().setPose(initialTransform);
   //            }
   //            preGraspFramesSize[0] = jsonNode.get("numberPreGraspFrames").asInt();
   //            postGraspFramesSize[0] = jsonNode.get("numberPostGraspFrames").asInt();
   //            JsonNode framesArrayNode = jsonNode.get("frames");
   //            for (int i = 0; i < preGraspFramesSize[0]; i++)
   //            {
   //               RigidBodyTransform preGraspObjectTransform = new RigidBodyTransform();
   //               JSONTools.toEuclid(framesArrayNode.get(i), preGraspObjectTransform);
   //               preGraspFrames.addObjectTransform(preGraspObjectTransform);
   //               String configuration = framesArrayNode.get(i).get("grip").asText();
   //               preGraspFrames.addHandConfiguration(configuration.isEmpty() ? null : HandConfiguration.valueOf(configuration));
   //            }
   //            RigidBodyTransform graspObjectTransform = new RigidBodyTransform();
   //            JSONTools.toEuclid(framesArrayNode.get(preGraspFramesSize[0]), graspObjectTransform);
   //            graspFrame.setObjectTransform(graspObjectTransform);
   //            String configuration = framesArrayNode.get(preGraspFramesSize[0]).get("grip").asText();
   //            graspFrame.setHandConfiguration(configuration.isEmpty() ? null : HandConfiguration.valueOf(configuration));
   //            for (int i = 0; i < postGraspFramesSize[0]; i++)
   //            {
   //               RigidBodyTransform postGraspObjectTransform = new RigidBodyTransform();
   //               JSONTools.toEuclid(framesArrayNode.get(i + preGraspFramesSize[0] + 1), postGraspObjectTransform);
   //               postGraspFrames.addObjectTransform(postGraspObjectTransform);
   //               configuration = framesArrayNode.get(i + preGraspFramesSize[0] + 1).get("grip").asText();
   //               postGraspFrames.addHandConfiguration(configuration.isEmpty() ? null : HandConfiguration.valueOf(configuration));
   //            }
   //         });
   //         LogTools.info("LOADED file {}", filePath);
   //      }
   //      else
   //      {
   //         LogTools.warn("Could not load file {}", filePath);
   //      }
   //
   //      // change affordance reference from whatever it is now to loaded initial object pose
   //      RigidBodyTransform initialObjectTransform = new RigidBodyTransform(objectBuilder.getSelectedObject().getInitialTransformToWorld());
   //      initialObjectFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(ReferenceFrame.getWorldFrame(), initialObjectTransform);
   //      affordanceFrame.changeParentFrame(initialObjectFrame);
   //
   //      filePath = Paths.get(configurationsDirectory.getFilesystemDirectory().toString(), fileName + ".json");
   //      if (Files.exists(filePath))
   //      {
   //         JSONFileTools.load(filePath, jsonNode ->
   //         {
   //            JSONTools.forEachArrayElement(jsonNode, "actions", actionNode ->
   //            {
   //               String actionType = actionNode.get("type").asText();
   //               if (actionType.equals("RDXHandPoseAction"))
   //               {
   //                  //                  side = RobotSide.getSideFromString(jsonNode.get("side").asText());
   //                  RigidBodyTransform frameTransform = new RigidBodyTransform();
   //                  JSONTools.toEuclid(actionNode, frameTransform);
   //                  if (preGraspFrames.getNumberOfFrames() < preGraspFramesSize[0])
   //                     preGraspFrames.addFrame(new FramePose3D(affordanceFrame.getReferenceFrame(), frameTransform));
   //                  else if (!graspFrame.isSet())
   //                     graspFrame.setFrame(new FramePose3D(affordanceFrame.getReferenceFrame(), frameTransform));
   //                  else
   //                     postGraspFrames.addFrame(new FramePose3D(affordanceFrame.getReferenceFrame(), frameTransform));
   //               }
   //            });
   //         });
   //         LogTools.info("LOADED file {}", filePath);
   //      }
   //      else
   //      {
   //         LogTools.warn("Could not load file {}", filePath);
   //      }
   //   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      graspFrame.getRenderables(renderables, pool);
      preGraspFrames.getRenderables(renderables, pool);
      postGraspFrames.getRenderables(renderables, pool);
   }

   public static void main(String[] args)
   {
      new RDXAffordanceEditorUI();
   }
}