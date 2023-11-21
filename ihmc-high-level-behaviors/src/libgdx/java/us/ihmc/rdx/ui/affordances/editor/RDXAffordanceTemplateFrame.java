package us.ihmc.rdx.ui.affordances.editor;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.ui.interactable.RDXInteractableAffordanceTemplateHand;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * This class deals with an AT frame.
 * This includes a pose of the hand, a frame graphics representing that pose, a hand configuration, and an associated object pose
 */
public class RDXAffordanceTemplateFrame
{
   private SideDependentList<FramePose3D> poses = new SideDependentList<>();
   private final SideDependentList<Boolean> isPoseSet = new SideDependentList<>();
   private final SideDependentList<PoseReferenceFrame> poseFrames = new SideDependentList<>();
   private final SideDependentList<RDXReferenceFrameGraphic> frameGraphics = new SideDependentList<>();
   private final SideDependentList<String> handConfigurations = new SideDependentList<>();
   private final SideDependentList<RDXInteractableAffordanceTemplateHand> interactableHands;
   private final SideDependentList<FramePose3D> handPoses;
   private final SideDependentList<RigidBodyTransform> handTransformsToWorld;
   private final RigidBodyTransform objectTransformToWorld;
   private final RigidBodyTransform objectTransformOfFrame = new RigidBodyTransform();
   private final RDXAffordanceTemplateEditorStatus editorStatus;
   private final RDXActiveAffordanceMenu menu;
   private boolean changedColor = false;

   public RDXAffordanceTemplateFrame(SideDependentList<RDXInteractableAffordanceTemplateHand> interactableHands,
                                     SideDependentList<RigidBodyTransform> handTransformsToWorld,
                                     SideDependentList<FramePose3D> handPoses,
                                     RigidBodyTransform objectTransformToWorld,
                                     RDXAffordanceTemplateEditorStatus editorStatus,
                                     Color color)
   {
      this.interactableHands = interactableHands;
      this.handPoses = handPoses;
      this.handTransformsToWorld = handTransformsToWorld;
      this.objectTransformToWorld = objectTransformToWorld;
      this.editorStatus = editorStatus;
      this.menu = editorStatus.getActiveMenu();

      for (RobotSide side : handPoses.keySet())
      {
         poseFrames.put(side, new PoseReferenceFrame("handFrame", ReferenceFrame.getWorldFrame()));
         frameGraphics.put(side, new RDXReferenceFrameGraphic(0.1, color));
         isPoseSet.put(side, false);
         poses.put(side, null);
         handConfigurations.put(side, null);
      }
   }

   public void update()
   {
      for (RobotSide side : handPoses.keySet())
      {
         poses.replace(side, new FramePose3D(poseFrames.get(side)));
         poses.get(side).changeFrame(ReferenceFrame.getWorldFrame());
         frameGraphics.get(side).updateFromFramePose(poses.get(side));
      }
   }

   public void renderImGuiWidgets(ImGuiUniqueLabelMap labels, String labelId, boolean editingBothHands)
   {
      RobotSide activeSide = editorStatus.getActiveSide();
      if (ImGui.button(labels.get("Set") + "##" + labelId) && handPoses.containsKey(activeSide))
      {
         if (editingBothHands)
         {
            for (RobotSide side : RobotSide.values)
            {
               isPoseSet.replace(side, true);
               setFrame(handPoses.get(side), side);
            }
         }
         else {
            isPoseSet.replace(activeSide, true);
            setFrame(handPoses.get(activeSide));
         }
         objectTransformOfFrame.set(objectTransformToWorld);
         editorStatus.setActiveMenu(this.menu);
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("Clear All") + "##" + labelId))
      {
         reset();
         editorStatus.setActiveMenu(RDXActiveAffordanceMenu.NONE);
      }

      for (RobotSide side : handPoses.keySet())
      {
         if (isPoseSet.get(side))
         {
            if (side == RobotSide.RIGHT && isPoseSet.get(RobotSide.LEFT))
            {
               ImGui.text("- - - - - - - - - - -");
            }

            if(editorStatus.getActiveMenu().equals(this.menu) && activeSide == side)
            {
               changedColor = true;
               ImGui.pushStyleColor(ImGuiCol.Button, 0.0f, 1.0f, 0.0f, 1.0f);
            }
            if (ImGui.button(labels.get((side == RobotSide.LEFT ? "L" : "R") + " Grasp Frame") + "##" + labelId))
            {
               editorStatus.setActiveSide(side);
               editorStatus.setActiveMenu(this.menu);
               selectFrame();
            }
            if (changedColor)
            {
               ImGui.popStyleColor();
               changedColor = false;
            }
            ImGui.sameLine();
            // handle the delete button click event here...
            ImGui.pushStyleColor(ImGuiCol.Button, 1.0f, 1.0f, 1.0f, 1.0f);
            if (ImGui.button(labels.get("X") + "##" + (side  == RobotSide.RIGHT ? "R" : "L") + labelId, 15, 15))
            {
               isPoseSet.replace(side, false);
               handConfigurations.replace(side, null);
               editorStatus.setActiveMenu(RDXActiveAffordanceMenu.NONE);
            }
            ImGui.popStyleColor();
         }
      }

      ImGui.text("Hand Configuration: " + (handConfigurations.get(activeSide) == null ? "" : handConfigurations.get(activeSide).toString()));
      ImGui.sameLine();
      if (ImGui.button(labels.get("Set") + "##hand" + labelId) && editorStatus.getActiveMenu().equals(this.menu))
      {
         handConfigurations.replace(activeSide, interactableHands.get(activeSide).getConfiguration());
      }
   }

   public void reset()
   {
      for (RobotSide side : handPoses.keySet())
      {
         isPoseSet.replace(side, false);
         handConfigurations.replace(side, null);
      }
      objectTransformOfFrame.setToZero();
   }

   public boolean isSet(RobotSide side)
   {
      return isPoseSet.get(side);
   }

   public void setFrame(FramePose3D poseReference)
   {
      setFrame(poseReference, editorStatus.getActiveSide());
   }

   public void setFrame(FramePose3D poseReference, RobotSide side)
   {
//      poses.replace(side, new FramePose3D(poseReference.getReferenceFrame(), poseReference));
      poses.replace(side, poseReference);
      updateInternal(side);
   }

   public void updateInternal(RobotSide side)
   {
      poses.get(side).changeFrame(ReferenceFrame.getWorldFrame());
      poseFrames.get(side).setPoseAndUpdate(poses.get(side));
      isPoseSet.replace(side, true);
      frameGraphics.get(side).updateFromFramePose(poses.get(side));
   }

   public void selectFrame()
   {
      for (RobotSide side : handPoses.keySet())
      {
         if (isPoseSet.get(side))
         {
            handTransformsToWorld.get(side).set(poses.get(side));  // move hand to grasp point
            objectTransformToWorld.set(objectTransformOfFrame);
         }
         if (handConfigurations.get(side) != null)
            interactableHands.get(side).setToConfiguration(handConfigurations.get(side));

         if (side == editorStatus.getActiveSide())
            interactableHands.get(side).setSelected(true);
         else
            interactableHands.get(side).setSelected(false);
      }
      editorStatus.disableMirror();
   }

   public void setHandConfiguration(String configuration, RobotSide side)
   {
      handConfigurations.replace(side, configuration);
   }

   public void setObjectTransform(RigidBodyTransform transform)
   {
      objectTransformOfFrame.set(transform);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (RobotSide side : handPoses.keySet())
         if (isPoseSet.get(side))
            frameGraphics.get(side).getRenderables(renderables, pool);
   }

   public SideDependentList<FramePose3D> getPoses()
   {
      return poses;
   }

   public void setPoses(SideDependentList<FramePose3D> poses)
   {
      this.poses = poses;
   }

   public String getHandConfiguration(RobotSide side)
   {
      return handConfigurations.get(side);
   }

   public RigidBodyTransform getObjectTransform()
   {
      return objectTransformOfFrame;
   }
}