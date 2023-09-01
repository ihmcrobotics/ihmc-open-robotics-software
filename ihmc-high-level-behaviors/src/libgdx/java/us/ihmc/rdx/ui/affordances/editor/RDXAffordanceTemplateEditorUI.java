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
import us.ihmc.perception.sceneGraph.PredefinedSceneNodeLibrary;
import us.ihmc.rdx.imgui.ImGuiInputText;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.interactable.RDXInteractableObjectBuilder;
import us.ihmc.rdx.ui.interactable.RDXInteractableSakeGripper;
import us.ihmc.rdx.ui.tools.ImGuiDirectory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;

import java.util.ArrayList;
import java.util.Arrays;

public class RDXAffordanceTemplateEditorUI
{
   private static final SideDependentList<ColorDefinition> HAND_COLORS;
   static
   {
      HAND_COLORS = new SideDependentList<>();
      HAND_COLORS.put(RobotSide.LEFT, ColorDefinitions.SandyBrown());
      HAND_COLORS.put(RobotSide.RIGHT, ColorDefinitions.SlateBlue());
   }

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private RobotSide[] activeSide;

   private final SideDependentList<RDXInteractableSakeGripper> interactableHands = new SideDependentList<>();
   private final SideDependentList<RigidBodyTransform> handTransformsToWorld = new SideDependentList<>();
   private final SideDependentList<FramePose3D> handPoses = new SideDependentList<>();
   private RDXInteractableObjectBuilder objectBuilder;
   private String currentObjectName = "";
   private String previousObjectName = "";
   private final float[] gripperClosure = new float[1];

   // affordance poses
   private RDXAffordanceTemplateFrame graspFrame;
   private RDXAffordanceTemplateFrames preGraspFrames;
   private RDXAffordanceTemplateFrames postGraspFrames;

   private RDXAffordanceTemplateMirror mirror;
   private RDXAffordanceTemplateLocker locker;
   private RDXAffordanceTemplateFileManager fileManager;

   private RDXActiveAffordanceMenu[] activeMenu;
   private boolean playing = false;

   private final ImGuiInputText textInput = new ImGuiInputText("(optional) Enter additional description");
   private final ImGuiDirectory fileManagerDirectory;

   public RDXAffordanceTemplateEditorUI(RDXBaseUI baseUI)
   {
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
      }

      activeSide = new RobotSide[1];
      activeSide[0] = RobotSide.RIGHT;
      activeMenu = new RDXActiveAffordanceMenu[1];
      activeMenu[0] = RDXActiveAffordanceMenu.PRE_GRASP;
      preGraspFrames = new RDXAffordanceTemplateFrames(interactableHands,
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
      graspFrame = new RDXAffordanceTemplateFrame(interactableHands,
                                                  handTransformsToWorld,
                                                  handPoses,
                                                  objectBuilder.getSelectedObject().getTransformToWorld(),
                                                  activeSide,
                                                  activeMenu,
                                                  Color.BLACK);
      activeMenu[0] = RDXActiveAffordanceMenu.POST_GRASP;
      postGraspFrames = new RDXAffordanceTemplateFrames(interactableHands,
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
      baseUI.getPrimaryScene().addRenderableProvider(RDXAffordanceTemplateEditorUI.this::getRenderables);
      baseUI.getImGuiPanelManager().addPanel("Affordance Template Panel", RDXAffordanceTemplateEditorUI.this::renderImGuiWidgets);

      mirror = new RDXAffordanceTemplateMirror(interactableHands, handTransformsToWorld, handPoses, activeSide);
      locker = new RDXAffordanceTemplateLocker(handTransformsToWorld, handPoses, activeSide, activeMenu);
      fileManager = new RDXAffordanceTemplateFileManager(handPoses.keySet(), preGraspFrames, graspFrame, postGraspFrames, objectBuilder);

      fileManagerDirectory = new ImGuiDirectory(fileManager.getConfigurationDirectory(),
                                                fileName -> !currentObjectName.isEmpty() && fileName.contains(currentObjectName),
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
      currentObjectName = objectBuilder.getSelectedObjectName();
      if (!currentObjectName.equals(previousObjectName))
      {
         currentObjectName = objectBuilder.getSelectedObjectName();
         if (!currentObjectName.equals(previousObjectName))
         {
            reset();
            objectBuilder.resetPose();
            fileManagerDirectory.reindexDirectory();
            fileManager.setLoadingFile("");
         }
         previousObjectName = currentObjectName;
      }

      for (RobotSide side : handPoses.keySet())
      {
         // update hand poses according to where the hand is
         handPoses.get(side).changeFrame(ReferenceFrame.getWorldFrame());
         handPoses.get(side).set(handTransformsToWorld.get(side));

         // selected hand determines active side
         if (interactableHands.get(side).isSelected())
         {
            if (activeSide[0] != side)
            {
               mirror.reset();
               activeSide[0] = side;
            }
         }
      }
      // update hand configuration
      if (handPoses.containsKey(activeSide[0]))
         gripperClosure[0] = interactableHands.get(activeSide[0]).getGripperClosure();

      mirror.update();

      graspFrame.update();
      preGraspFrames.update();
      postGraspFrames.update();
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Hands Menu");
      ColorDefinition handColor = HAND_COLORS.get(RobotSide.LEFT);
      ImGui.pushStyleColor(ImGuiCol.CheckMark,
                           (float) handColor.getRed(),
                           (float) handColor.getGreen(),
                           (float) handColor.getBlue(),
                           (float) handColor.getAlpha());
      if (ImGui.radioButton(labels.get("Left"), activeSide[0] == RobotSide.LEFT))
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
      if (ImGui.radioButton(labels.get("Right"), activeSide[0] == RobotSide.RIGHT))
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
            if (ImGui.button(labels.get("Add") + "##side"))
            {
               interactableHands.get(side).setShowing(true);
               handTransformsToWorld.get(side).getRotation().setYawPitchRoll(0.0, Math.toRadians(-90.0), 0.0);
               handTransformsToWorld.get(side).getTranslation().set(-0.5, side.negateIfRightSide(0.2), 0);
               handPoses.put(side, new FramePose3D(ReferenceFrame.getWorldFrame(), handTransformsToWorld.get(side)));
            }
         }
         else if (activeSide[0] == side && handPoses.containsKey(side))
         {
            if (ImGui.button(labels.get("Remove") + "##side"))
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
         if (ImGui.sliderFloat("Set Closure",
                               gripperClosure,
                               interactableHands.get(activeSide[0]).getMinGripperClosure(),
                               interactableHands.get(activeSide[0]).getMaxGripperClosure()))
            interactableHands.get(activeSide[0]).setGripperClosure(gripperClosure[0]);
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
         if (ImGui.button(labels.get("Play")))
            playing = true;

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
         interactableHands.get(side).closeGripper();
      }
      mirror.reset();

      graspFrame.reset();
      preGraspFrames.reset();
      postGraspFrames.reset();
      activeMenu[0] = RDXActiveAffordanceMenu.NONE;
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      graspFrame.getRenderables(renderables, pool);
      preGraspFrames.getRenderables(renderables, pool);
      postGraspFrames.getRenderables(renderables, pool);
   }
}