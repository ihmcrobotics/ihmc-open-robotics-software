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
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.ui.interactable.RDXInteractableSakeGripper;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

public class RDXAffordanceFrame
{
   private FramePose3D pose = new FramePose3D();
   private boolean isPoseSet = false;
   private final RDXReferenceFrameGraphic frameGraphic;
   private final PoseReferenceFrame poseFrame;
   private HandConfiguration handConfiguration;
   public final RDXInteractableSakeGripper interactableHand;
   private final FramePose3D handPose;
   private final RigidBodyTransform handTransformToWorld;
   private final RigidBodyTransform objectTransformToWorld;
   private final RigidBodyTransform objectTransformOfFrame = new RigidBodyTransform();
   private RDXActiveAffordanceMenu[] activeMenu;
   private final RDXActiveAffordanceMenu menu;
   public boolean changedColor = false;

   public RDXAffordanceFrame(RDXInteractableSakeGripper interactableHand,
                             RigidBodyTransform handTransformToWorld,
                             FramePose3D handPose,
                             RigidBodyTransform objectTransformToWorld,
                             RDXActiveAffordanceMenu[] activeMenu,
                             Color color)
   {
      this.interactableHand = interactableHand;
      this.handPose = handPose;
      this.handTransformToWorld = handTransformToWorld;
      this.objectTransformToWorld = objectTransformToWorld;
      this.activeMenu = activeMenu;
      this.menu = activeMenu[0];
      poseFrame = new PoseReferenceFrame("handFrame", ReferenceFrame.getWorldFrame());
      frameGraphic = new RDXReferenceFrameGraphic(0.1, color);
   }

   public void update()
   {
      pose = new FramePose3D(poseFrame);
      pose.changeFrame(ReferenceFrame.getWorldFrame());
      frameGraphic.updateFromFramePose(pose);
   }

   public void renderImGuiWidgets(ImGuiUniqueLabelMap labels, String labelId)
   {
      if (ImGui.button(labels.get("SET") + "##" + labelId))
      {
         isPoseSet = true;
         setFrame(handPose);
         objectTransformOfFrame.set(objectTransformToWorld);
         activeMenu[0] = this.menu;
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("CLEAR") + "##" + labelId))
      {
         reset();
         activeMenu[0] = RDXActiveAffordanceMenu.NONE;
      }

      if (isPoseSet)
      {
         if(activeMenu[0].equals(this.menu))
         {
            changedColor = true;
            ImGui.pushStyleColor(ImGuiCol.Button, 0.0f, 1.0f, 0.0f, 1.0f);
         }
         if (ImGui.button(labels.get("Grasp Frame") + "##" + labelId))
         {
            activeMenu[0] = this.menu;
            selectFrame();
         }
         if (changedColor)
         {
            ImGui.popStyleColor();
            changedColor = false;
         }
      }

      ImGui.text("Hand Configuration: " + (handConfiguration == null ? "" : handConfiguration.toString()));
      ImGui.sameLine();
      if (ImGui.button(labels.get("SET") + "##hand" + labelId) && activeMenu[0].equals(this.menu))
      {
         handConfiguration = interactableHand.getConfiguration();
      }
   }

   public void reset()
   {
      isPoseSet = false;
      handConfiguration = null;
      objectTransformOfFrame.setToZero();
   }

   public boolean isSet()
   {
      return isPoseSet;
   }

   public void setFrame(FramePose3D poseReference)
   {
      pose = new FramePose3D(poseReference.getReferenceFrame(), poseReference);
      pose.changeFrame(ReferenceFrame.getWorldFrame());
      poseFrame.setPoseAndUpdate(pose);
      isPoseSet = true;
      frameGraphic.updateFromFramePose(pose);
   }

   public void selectFrame()
   {
      if (isPoseSet)
      {
         handTransformToWorld.set(pose);  // move hand to grasp point
         objectTransformToWorld.set(objectTransformOfFrame);
      }
      if (handConfiguration != null)
         interactableHand.setGripperToConfiguration(handConfiguration);
   }

   public void setHandConfiguration(HandConfiguration configuration)
   {
      handConfiguration = configuration;
   }

   public void setObjectTransform(RigidBodyTransform transform)
   {
      objectTransformOfFrame.set(transform);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (isPoseSet)
         frameGraphic.getRenderables(renderables, pool);
   }

   public FramePose3D getPose()
   {
      return pose;
   }

   public HandConfiguration getHandConfiguration()
   {
      return handConfiguration;
   }

   public RigidBodyTransform getObjectTransform()
   {
      return objectTransformOfFrame;
   }
}