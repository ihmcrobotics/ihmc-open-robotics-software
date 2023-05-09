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
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

import java.util.ArrayList;

public class RDXAffordanceFrames
{
   private final ArrayList<FramePose3D> poses = new ArrayList<>();
   private final ArrayList<PoseReferenceFrame> poseFrames = new ArrayList<>();
   private final ArrayList<RDXReferenceFrameGraphic> frameGraphics = new ArrayList<>();
   private final ArrayList<Color> colors;
   private final ArrayList<Integer> poseIndices = new ArrayList<>();
   private int index = 0;
   private int colorIndex = 0;
   public final PoseReferenceFrame affordanceFrame;
   public final RDXInteractableSakeGripper interactableHand;
   private final FramePose3D handPose;
   private final ArrayList<HandConfiguration> handConfigurations = new ArrayList<>();
   private HandConfiguration selectedFrameConfiguration;
   private int selectedIndex = -1;
   private final RigidBodyTransform handTransformToWorld;
   private final RigidBodyTransform objectTransformToWorld;
   private final ArrayList<RigidBodyTransform> objectTransforms = new ArrayList<>();
   boolean changedColor = false;
   private RDXActiveAffordanceMenu[] activeMenu;
   private final RDXActiveAffordanceMenu menu;

   public RDXAffordanceFrames(RDXInteractableSakeGripper interactableHand,
                              RigidBodyTransform handTransformToWorld,
                              FramePose3D handPose,
                              RigidBodyTransform objectTransformToWorld,
                              PoseReferenceFrame affordanceFrame,
                              RDXActiveAffordanceMenu[] activeMenu,
                              ArrayList<Color> colors)
   {
      this.interactableHand = interactableHand;
      this.handPose = handPose;
      this.handTransformToWorld = handTransformToWorld;
      this.objectTransformToWorld = objectTransformToWorld;
      this.affordanceFrame = affordanceFrame;
      this.activeMenu = activeMenu;
      this.menu = activeMenu[0];
      this.colors = colors;
   }

   public void update()
   {
      for (int i = 0; i < poses.size(); ++i)
      {
         poses.set(i, new FramePose3D(poseFrames.get(i)));
         poses.get(i).changeFrame(ReferenceFrame.getWorldFrame());
         frameGraphics.get(i).updateFromFramePose(poses.get(i));
      }
   }

   public void renderImGuiWidgets(ImGuiUniqueLabelMap labels, String lableId)
   {
      if (ImGui.button(labels.get("ADD") + "##" + lableId))
      {
         activeMenu[0] = this.menu;
         index++;
         PoseReferenceFrame frame = new PoseReferenceFrame(index + "Frame", affordanceFrame);
         frame.setPoseAndUpdate(handPose);

         poseIndices.add(index);
         poses.add(handPose);
         poseFrames.add(frame);
         frameGraphics.add(new RDXReferenceFrameGraphic(0.1, colors.get(colorIndex % colors.size())));
         colorIndex++;
         // no hand configuration is set right when you add a new frame
         selectedFrameConfiguration = null;
         // add an empty spot for the hand configuration
         handConfigurations.add(null);
         // add a spot for the object transform associated with this frame
         objectTransforms.add(new RigidBodyTransform(objectTransformToWorld));
         // select the frame you've just added
         selectedIndex = poseIndices.size() - 1;
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("SET") + "##" + lableId) && activeMenu[0].equals(this.menu))
      {
         poseFrames.get(selectedIndex).setPoseAndUpdate(handPose);
         poses.set(selectedIndex, handPose);
         objectTransforms.set(selectedIndex, new RigidBodyTransform(objectTransformToWorld));
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("CLEAR ALL") + "##" + lableId))
      {
         reset();
      }
      if (poseIndices.size() > 0)
      {
         for (int i = 0; i < poseIndices.size(); ++i)
         {
            if (i % 5 != 0)
               ImGui.sameLine();
            // display selected frame in green
            if (selectedIndex == i && activeMenu[0].equals(this.menu))
            {
               ImGui.pushStyleColor(ImGuiCol.Button, 0.0f, 1.0f, 0.0f, 1.0f);
               changedColor = true;
            }
            if (ImGui.button(labels.get(poseIndices.get(i).toString()) + "##" + lableId))
            {
               activeMenu[0] = this.menu;
               // move hand to selected frame
               handTransformToWorld.set(poses.get(i));
               selectedIndex = i;
               // if hand configuration has been assigned to this frame
               if (handConfigurations.get(i) != null)
               {
                  interactableHand.setGripperToConfiguration(handConfigurations.get(i)); // update hand configuration when teleporting
                  selectedFrameConfiguration = handConfigurations.get(i);
               }
               else
                  selectedFrameConfiguration = null;
               // update pose of the object
               objectTransformToWorld.set(objectTransforms.get(selectedIndex));
            }
            if (changedColor)
            {
               ImGui.popStyleColor();
               changedColor = false;
            }
            ImGui.sameLine();
            // handle the delete button click event here...
            ImGui.pushStyleColor(ImGuiCol.Button, 1.0f, 1.0f, 1.0f, 1.0f);
            if (ImGui.button(labels.get("X") + "##" + lableId + i, 15, 15))
            {
               poseFrames.remove(i);
               poses.remove(i);
               frameGraphics.remove(i);
               poseIndices.remove(i);
               handConfigurations.remove(i);
               selectedFrameConfiguration = null;
               selectedIndex = -1;
            }
            ImGui.popStyleColor();
         }
      }
      else
      {
         colorIndex = 0;
         index = 0;
      }

      ImGui.text("Hand Configuration: " + (selectedFrameConfiguration == null ? "" : selectedFrameConfiguration.toString()));
      ImGui.sameLine();
      if (ImGui.button(labels.get("SET") + "##hand" + lableId) && activeMenu[0].equals(this.menu))
      {
         if (selectedIndex >= 0)
         {
            handConfigurations.set(selectedIndex, interactableHand.getConfiguration());
            selectedFrameConfiguration = handConfigurations.get(selectedIndex);
         }
      }
   }

   public void reset()
   {
      poseIndices.clear();
      poseFrames.clear();
      poses.clear();
      frameGraphics.clear();
      colorIndex = 0;
      handConfigurations.clear();
      objectTransforms.clear();
      selectedFrameConfiguration = null;
      selectedIndex = -1;
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (RDXReferenceFrameGraphic frameGraphic : frameGraphics)
      {
         frameGraphic.getRenderables(renderables, pool);
      }
   }
}
