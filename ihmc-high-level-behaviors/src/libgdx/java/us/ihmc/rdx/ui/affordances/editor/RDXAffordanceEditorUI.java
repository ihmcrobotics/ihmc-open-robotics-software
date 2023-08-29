package us.ihmc.rdx.ui.affordances.editor;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiCol;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.PredefinedSceneNodeLibrary;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiInputText;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.interactable.RDXInteractableSakeGripper;
import us.ihmc.rdx.ui.interactable.RDXInteractableObjectBuilder;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;

import java.util.*;

public class RDXAffordanceEditorUI
{
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
   private final float[] gripperClosure = new float[1];

   // affordance poses
   private RDXAffordanceFrame graspFrame;
   private RDXAffordanceFrames preGraspFrames;
   private RDXAffordanceFrames postGraspFrames;

   private RDXAffordanceMirror mirror;
   private RDXAffordanceLocker locker;
   private AffordanceExporter exporter;

   private RDXActiveAffordanceMenu[] activeMenu;
   private boolean playing = false;

   private final ImGuiInputText textInput = new ImGuiInputText("Enter file name to save/load");
   private String fileName = "";

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

            mirror = new RDXAffordanceMirror(interactableHands, handTransformsToWorld, handPoses, activeSide);
            locker = new RDXAffordanceLocker(handTransformsToWorld, handPoses, activeSide, activeMenu);
            exporter = new AffordanceExporter(handPoses.keySet(), preGraspFrames, graspFrame, postGraspFrames, objectBuilder);
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
         locker.update(objectPose);
      }

      for (RobotSide side : handPoses.keySet())
      {
         // update hand poses according to where the hand is
         handPoses.get(side).changeFrame(ReferenceFrame.getWorldFrame());
         handPoses.get(side).set(handTransformsToWorld.get(side));

         // selected hand determines active side
         if (interactableHands.get(side).isSelected())
            activeSide[0] = side;
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

      mirror.renderImGuiWidgets(labels);

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
      locker.renderImGuiWidgets(labels);
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
      if (ImGui.button("SAVE"))
      {
         exporter.saveToFile(fileName);
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("LOAD")))
      {
         objectBuilder.reset();
         reset();
         exporter.loadFromFile(fileName);
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

   public static void main(String[] args)
   {
      new RDXAffordanceEditorUI();
   }
}