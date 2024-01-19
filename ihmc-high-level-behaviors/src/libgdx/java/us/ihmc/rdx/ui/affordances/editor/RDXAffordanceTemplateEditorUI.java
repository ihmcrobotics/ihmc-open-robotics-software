package us.ihmc.rdx.ui.affordances.editor;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import us.ihmc.commons.nio.BasicPathVisitor;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.multiBodies.door.DoorSceneNodeDefinitions;
import us.ihmc.perception.sceneGraph.rigidBody.RigidBodySceneObjectDefinitions;
import us.ihmc.rdx.imgui.ImGuiDirectory;
import us.ihmc.rdx.imgui.ImGuiInputText;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.interactable.RDXInteractableAffordanceTemplateHand;
import us.ihmc.rdx.ui.interactable.RDXInteractableNub;
import us.ihmc.rdx.ui.interactable.RDXInteractableObjectBuilder;
import us.ihmc.rdx.ui.interactable.RDXInteractableSakeGripper;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;

import java.util.ArrayList;
import java.util.Arrays;

public class RDXAffordanceTemplateEditorUI
{
   private static final int REPLAY_FREQUENCY = 5;
   private static final SideDependentList<ColorDefinition> HAND_COLORS;
   static
   {
      HAND_COLORS = new SideDependentList<>();
      HAND_COLORS.put(RobotSide.LEFT, ColorDefinitions.SandyBrown());
      HAND_COLORS.put(RobotSide.RIGHT, ColorDefinitions.SlateBlue());
   }

   private final RDX3DPanel panel3D;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final RDXAffordanceTemplateEditorStatus status;

   private final SideDependentList<RDXInteractableAffordanceTemplateHand> interactableHands = new SideDependentList<>();
   private final SideDependentList<RigidBodyTransform> handTransformsToWorld = new SideDependentList<>();
   private final SideDependentList<FramePose3D> handPoses = new SideDependentList<>();
   private final RDXInteractableObjectBuilder objectBuilder;
   private String currentObjectName = "";
   private final float[] gripperClosure = new float[1];

   // affordance poses
   private final RDXAffordanceTemplateFrame graspFrame;
   private final RDXAffordanceTemplateFrames preGraspFrames;
   private final RDXAffordanceTemplateFrames postGraspFrames;

   private final RDXAffordanceTemplateMirror mirror;
   private final RDXAffordanceTemplateLocker locker;
   private final RDXAffordanceTemplateFileManager fileManager;

   private boolean playing = false;
   private long lastPlayTime = 0;

   private final ImGuiInputText textInput = new ImGuiInputText("(optional) Enter additional description");
   private final ImGuiDirectory fileManagerDirectory;

   public RDXAffordanceTemplateEditorUI(RDXBaseUI baseUI)
   {
      panel3D = baseUI.getPrimary3DPanel();

      ReferenceFrameLibrary referenceFrameLibrary = new ReferenceFrameLibrary();

      SceneGraph sceneGraph = new SceneGraph(referenceFrameLibrary);
      sceneGraph.modifyTree(modificationQueue ->
                            {
                               DoorSceneNodeDefinitions.ensureRightPushDoorNodesAdded(sceneGraph, modificationQueue, sceneGraph.getRootNode());
                               DoorSceneNodeDefinitions.ensureLeftPushDoorNodesAdded(sceneGraph, modificationQueue, sceneGraph.getRootNode());
                               RigidBodySceneObjectDefinitions.ensureBoxNodeAdded(sceneGraph, modificationQueue, sceneGraph.getRootNode());
                               RigidBodySceneObjectDefinitions.ensureCanOfSoupNodeAdded(sceneGraph, modificationQueue, sceneGraph.getRootNode());
                               RigidBodySceneObjectDefinitions.ensureDebrisNodeAdded(sceneGraph, modificationQueue, sceneGraph.getRootNode());
                               RigidBodySceneObjectDefinitions.ensureShoeNodeAdded(sceneGraph, modificationQueue, sceneGraph.getRootNode());
                               RigidBodySceneObjectDefinitions.ensureLaptopNodeAdded(sceneGraph, modificationQueue, sceneGraph.getRootNode());
                               RigidBodySceneObjectDefinitions.ensureBookNodeAdded(sceneGraph, modificationQueue, sceneGraph.getRootNode());
                               RigidBodySceneObjectDefinitions.ensureCerealNodeAdded(sceneGraph, modificationQueue, sceneGraph.getRootNode());
                               RigidBodySceneObjectDefinitions.ensureMugNodeAdded(sceneGraph, modificationQueue, sceneGraph.getRootNode());
                               RigidBodySceneObjectDefinitions.ensureBikeNodeAdded(sceneGraph, modificationQueue, sceneGraph.getRootNode());
                            });

      objectBuilder = new RDXInteractableObjectBuilder(baseUI, sceneGraph);
      baseUI.getImGuiPanelManager().addPanel(objectBuilder.getWindowName(), objectBuilder::renderImGuiWidgets);

      for (RobotSide side : RobotSide.values)
      {
         handTransformsToWorld.put(side, new RigidBodyTransform());
         handTransformsToWorld.get(side).getRotation().setYawPitchRoll(0.0, Math.toRadians(-90.0), 0.0);
         handTransformsToWorld.get(side).getTranslation().set(-0.5, side.negateIfRightSide(0.2), 0);
         interactableHands.put(side,
                               new RDXInteractableSakeGripper(panel3D,
                                                              handTransformsToWorld.get(side),
                                                              new ColorDefinition(HAND_COLORS.get(side).getRed(),
                                                                                  HAND_COLORS.get(side).getGreen(),
                                                                                  HAND_COLORS.get(side).getBlue(),
                                                                                  0.8)));
         handPoses.put(side, new FramePose3D(ReferenceFrame.getWorldFrame(), handTransformsToWorld.get(side)));
      }

      status = new RDXAffordanceTemplateEditorStatus(RobotSide.RIGHT, RDXActiveAffordanceMenu.NONE, RDXInteractableSakeGripper.class);
      status.setActiveMenu(RDXActiveAffordanceMenu.PRE_GRASP);
      preGraspFrames = new RDXAffordanceTemplateFrames(interactableHands,
                                                       handTransformsToWorld,
                                                       handPoses,
                                                       objectBuilder.getSelectedObject().getTransformToWorld(),
                                                       status,
                                                       new ArrayList<>(Arrays.asList(new Color(0xFFE4B5FF),
                                                                                     new Color(0xFF8C00FF),
                                                                                     new Color(0xFFDAB9FF),
                                                                                     new Color(0xFF6600FF),
                                                                                     new Color(0xFFA07AFF))));
      status.setActiveMenu(RDXActiveAffordanceMenu.GRASP);
      graspFrame = new RDXAffordanceTemplateFrame(interactableHands,
                                                  handTransformsToWorld,
                                                  handPoses,
                                                  objectBuilder.getSelectedObject().getTransformToWorld(),
                                                  status,
                                                  Color.BLACK);
      status.setActiveMenu(RDXActiveAffordanceMenu.POST_GRASP);
      postGraspFrames = new RDXAffordanceTemplateFrames(interactableHands,
                                                        handTransformsToWorld,
                                                        handPoses,
                                                        objectBuilder.getSelectedObject().getTransformToWorld(),
                                                        status,
                                                        new ArrayList<>(Arrays.asList(new Color(0xD8BFD8FF),
                                                                                      new Color(0xBA55D3FF),
                                                                                      new Color(0x9932CCFF),
                                                                                      new Color(0x8A2BE2FF),
                                                                                      new Color(0x4B0082FF))));
      status.setActiveMenu(RDXActiveAffordanceMenu.NONE);
      for (RobotSide side : RobotSide.values)
         baseUI.getImGuiPanelManager().addPanel(interactableHands.get(side).getPose3DGizmo().createTunerPanel(side.getCamelCaseName() + " Hand"));
      baseUI.getPrimaryScene().addRenderableProvider(objectBuilder.getSelectedObject());
      baseUI.getPrimaryScene().addRenderableProvider(RDXAffordanceTemplateEditorUI.this::getRenderables);
      baseUI.getImGuiPanelManager().addPanel("Affordance Template Panel", RDXAffordanceTemplateEditorUI.this::renderImGuiWidgets);

      mirror = new RDXAffordanceTemplateMirror(interactableHands, handTransformsToWorld, handPoses, status);
      locker = new RDXAffordanceTemplateLocker(handTransformsToWorld, handPoses, status);

      fileManager = new RDXAffordanceTemplateFileManager(handPoses.keySet(), preGraspFrames, graspFrame, postGraspFrames, objectBuilder);
      fileManagerDirectory = new ImGuiDirectory(fileManager.getConfigurationDirectory(),
                                                fileName -> !currentObjectName.isEmpty() && fileName.contains(currentObjectName) && fileName.equals(fileManager.getLoadingFile()),
                                                pathEntry -> pathEntry.type() == BasicPathVisitor.PathType.FILE
                                                             && pathEntry.path().getFileName().toString().contains(currentObjectName)
                                                             && pathEntry.path().getFileName().toString().endsWith(".json")
                                                             && !pathEntry.path().getFileName().toString().contains("Frames")
                                                             && !pathEntry.path().getFileName().toString().contains("Extra"),
                                                fileManager::setLoadingFile);
   }

   public void update()
   {
      if (objectBuilder.isAnyObjectSelected())
      {
         FramePose3D objectPose = new FramePose3D(ReferenceFrame.getWorldFrame(), objectBuilder.getSelectedObject().getTransformToWorld());
         // when editing post grasp poses, we want to move the object frame and the hand together
         locker.update(objectPose);
      }
      if (objectBuilder.getSelectedObjectNotification().poll())
      {
         currentObjectName = objectBuilder.getSelectedObjectNotification().read();
         reset();
         objectBuilder.resetPose();
         fileManagerDirectory.reindexDirectory();
         fileManager.setLoadingFile("");
      }

      for (RobotSide side : handPoses.keySet())
      {
         // update hand poses according to where the hand is
         handPoses.get(side).changeFrame(ReferenceFrame.getWorldFrame());
         handPoses.get(side).set(handTransformsToWorld.get(side));

         // selected hand determines active side
         if (interactableHands.get(side).isSelected())
         {
            if (status.getActiveSide() != side)
            {
               mirror.reset();
               status.setActiveSide(side);
            }
         }
      }
      if (handPoses.containsKey(status.getActiveSide()))
      {
         if (interactableHands.get(status.getActiveSide()).hasGripper())
         {
            // update closure of gripper
            if (handPoses.containsKey(status.getActiveSide()))
               gripperClosure[0] = interactableHands.get(status.getActiveSide()).getGripperClosure();
         }
      }

      mirror.update();

      graspFrame.update();
      preGraspFrames.update();
      postGraspFrames.update();
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Hands Menu");
      if (ImGui.radioButton(labels.get("Sake Hand"), status.getActiveHandModel().equals(RDXInteractableSakeGripper.class)))
      {
         status.setActiveHandModel(RDXInteractableSakeGripper.class);
         for (RobotSide side : handPoses.keySet())
         {
            interactableHands.get(side).removeRenderables(panel3D);
            interactableHands.replace(side,
                                      new RDXInteractableSakeGripper(panel3D,
                                                                     handTransformsToWorld.get(side),
                                                                     new ColorDefinition(HAND_COLORS.get(side).getRed(),
                                                                                         HAND_COLORS.get(side).getGreen(),
                                                                                         HAND_COLORS.get(side).getBlue(),
                                                                                         0.8)));
         }
      }
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Nub"), status.getActiveHandModel().equals(RDXInteractableNub.class)))
      {
         status.setActiveHandModel(RDXInteractableNub.class);
         for (RobotSide side : handPoses.keySet())
         {
            interactableHands.get(side).removeRenderables(panel3D);
            interactableHands.replace(side,
                                      new RDXInteractableNub(panel3D,
                                                             handTransformsToWorld.get(side),
                                                             new ColorDefinition(HAND_COLORS.get(side).getRed(),
                                                                                 HAND_COLORS.get(side).getGreen(),
                                                                                 HAND_COLORS.get(side).getBlue(),
                                                                                 0.8)));
         }
      }

      ColorDefinition handColor = HAND_COLORS.get(RobotSide.LEFT);
      ImGui.pushStyleColor(ImGuiCol.CheckMark,
                           (float) handColor.getRed(),
                           (float) handColor.getGreen(),
                           (float) handColor.getBlue(),
                           (float) handColor.getAlpha());
      if (ImGui.radioButton(labels.get("Left"), status.getActiveSide() == RobotSide.LEFT))
      {
         status.setActiveSide(RobotSide.LEFT);
         if (handPoses.containsKey(RobotSide.LEFT))
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
      if (ImGui.radioButton(labels.get("Right"), status.getActiveSide() == RobotSide.RIGHT))
      {
         status.setActiveSide(RobotSide.RIGHT);
         if (handPoses.containsKey(RobotSide.RIGHT))
            interactableHands.get(RobotSide.RIGHT).setSelected(true);
         if (handPoses.containsKey(RobotSide.LEFT))
            interactableHands.get(RobotSide.LEFT).setSelected(false);
      }
      ImGui.popStyleColor();
      ImGui.sameLine();
      for (RobotSide side : RobotSide.values)
      {
         if (status.getActiveSide() == side && !handPoses.containsKey(side))
         {
            if (ImGui.button(labels.get("Add") + "##side"))
            {
               if (status.getActiveHandModel().equals(RDXInteractableSakeGripper.class))
               {
                  interactableHands.put(side,
                                        new RDXInteractableSakeGripper(panel3D,
                                                                       handTransformsToWorld.get(side),
                                                                       new ColorDefinition(HAND_COLORS.get(side).getRed(),
                                                                                           HAND_COLORS.get(side).getGreen(),
                                                                                           HAND_COLORS.get(side).getBlue(),
                                                                                           0.8)));
               }
               else if (status.getActiveHandModel().equals(RDXInteractableNub.class))
               {
                  interactableHands.put(side,
                                        new RDXInteractableNub(panel3D,
                                                               handTransformsToWorld.get(side),
                                                               new ColorDefinition(HAND_COLORS.get(side).getRed(),
                                                                                   HAND_COLORS.get(side).getGreen(),
                                                                                   HAND_COLORS.get(side).getBlue(),
                                                                                   0.8)));
               }
               handTransformsToWorld.get(side).getRotation().setYawPitchRoll(0.0, Math.toRadians(-90.0), 0.0);
               handTransformsToWorld.get(side).getTranslation().set(-0.5, side.negateIfRightSide(0.2), 0);
               handPoses.put(side, new FramePose3D(ReferenceFrame.getWorldFrame(), handTransformsToWorld.get(side)));
            }
         }
         else if (status.getActiveSide() == side && handPoses.containsKey(side))
         {
            if (ImGui.button(labels.get("Remove") + "##side"))
            {
               interactableHands.get(side).removeRenderables(panel3D);
               interactableHands.remove(side);
               handPoses.remove(side);
            }
         }
      }

      RobotSide activeSide = status.getActiveSide();
      if (handPoses.containsKey(activeSide))
      {
         ImGui.text("Hand configuration: ");
         for (String configuration : interactableHands.get(activeSide).getAvailableConfigurations())
         {
            if (ImGui.button(labels.get(configuration)))
            {
               interactableHands.get(activeSide).setToConfiguration(configuration);
            }
         }
         if (interactableHands.get(activeSide).hasGripper())
         {
            if (ImGui.sliderFloat("Set Closure",
                                  gripperClosure,
                                  interactableHands.get(activeSide).getMinGripperClosure(),
                                  interactableHands.get(activeSide).getMaxGripperClosure()))
               interactableHands.get(activeSide).setGripperClosure(gripperClosure[0]);
         }
         ImGui.separator();
      }

      if (objectBuilder.isAnyObjectSelected())
      {
         mirror.renderImGuiWidgets(labels);

         ImGui.text("Pre-Grasp Menu");
         ImGui.text("Pre-grasp Frames: ");
         ImGui.sameLine();
         preGraspFrames.renderImGuiWidgets(labels, "pregrasp", mirror.isActive());
         ImGui.separator();

         ImGui.text("Grasp Menu");
         ImGui.text("Grasp Frame: ");
         ImGui.sameLine();
         graspFrame.renderImGuiWidgets(labels, "grasp", mirror.isActive());
         ImGui.separator();

         ImGui.text("Post-Grasp Menu");
         ImGui.text("Post-grasp Frames: ");
         ImGui.sameLine();
         postGraspFrames.renderImGuiWidgets(labels, "postgrasp", mirror.isActive() && locker.areBothHandsLocked());
         locker.renderImGuiWidgets(labels);
         ImGui.separator();

         ImGui.text("Previous/Next Frame: ");
         ImGui.sameLine();
         if (ImGui.button(labels.get("<"), 20, 25))
         {
            switch (status.getActiveMenu())
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
                     status.setActiveMenu(RDXActiveAffordanceMenu.PRE_GRASP);
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
                     if (graspFrame.isSet(status.getActiveSide()))
                     {
                        status.setActiveMenu(RDXActiveAffordanceMenu.GRASP);
                        graspFrame.selectFrame();
                     }
                     else if (preGraspFrames.getNumberOfFrames() > 0)
                     {
                        status.setActiveMenu(RDXActiveAffordanceMenu.PRE_GRASP);
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
            long currentTime = System.currentTimeMillis();
            if ((playing && currentTime - lastPlayTime >= 1000/REPLAY_FREQUENCY) || !playing)
            {
               lastPlayTime = currentTime;
               switch (status.getActiveMenu())
               {
                  case PRE_GRASP ->
                  {
                     if (!preGraspFrames.isLast())
                        preGraspFrames.selectNext();
                     else
                     {
                        if (graspFrame.isSet(status.getActiveSide()))
                        {
                           status.setActiveMenu(RDXActiveAffordanceMenu.GRASP);
                           graspFrame.selectFrame();
                        }
                        else if (postGraspFrames.getNumberOfFrames() > 0)
                        {
                           status.setActiveMenu(RDXActiveAffordanceMenu.POST_GRASP);
                           postGraspFrames.resetSelectedIndex();
                           postGraspFrames.selectNext();
                        }
                     }
                  }
                  case GRASP ->
                  {
                     if (postGraspFrames.getNumberOfFrames() > 0)
                     {
                        status.setActiveMenu(RDXActiveAffordanceMenu.POST_GRASP);
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
         }
         ImGui.sameLine();
         if (ImGui.button(labels.get("Play")))
         {
            playing = true;
            lastPlayTime = System.currentTimeMillis();;
         }

         if (ImGui.button("Reset"))
         {
            objectBuilder.getSelectedObject().resetToInitialPose();
            reset();
         }
         ImGui.separator();

         ImGui.text("Name of selected object automatically used for file saving");
         textInput.render();
         if (ImGui.button("Save"))
            fileManager.saveToFile(textInput.getString() + currentObjectName);
         ImGui.separator();

         fileManagerDirectory.renderImGuiWidgets();
         ImGui.text("Click on radio button and then on Load");
         if (ImGui.button(labels.get("Load")))
         {
            reset();
            fileManager.load();
         }
      }
      else
      {
         ImGui.pushStyleColor(ImGuiCol.Text, 1.0f, 0.0f, 0.0f, 1.0f);
         ImGui.text("Select an object first from the Object Panel");
         ImGui.popStyleColor();
      }

   }

   private void reset()
   {
      for (RobotSide side : handPoses.keySet())
      {
         handTransformsToWorld.get(side).setToZero();
         handTransformsToWorld.get(side).getTranslation().set(-0.5, side.negateIfRightSide(0.2), 0);
         handTransformsToWorld.get(side).getRotation().setYawPitchRoll(0.0, Math.toRadians(-90.0), 0.0);
         interactableHands.get(side).setToDefaultConfiguration();
      }
      mirror.reset();

      graspFrame.reset();
      preGraspFrames.reset();
      postGraspFrames.reset();
      status.setActiveMenu(RDXActiveAffordanceMenu.NONE);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      graspFrame.getRenderables(renderables, pool);
      preGraspFrames.getRenderables(renderables, pool);
      postGraspFrames.getRenderables(renderables, pool);
   }
}