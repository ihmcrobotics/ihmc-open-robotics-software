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

import java.awt.*;
import java.util.ArrayList;

public class RDXAffordancePoses
{
   private final ArrayList<FramePose3D> poses = new ArrayList<>();
   private final ArrayList<PoseReferenceFrame> frames = new ArrayList<>();
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
   boolean changedColor = false;

   public RDXAffordancePoses(RDXInteractableSakeGripper interactableHand,
                             RigidBodyTransform handTransformToWorld,
                             FramePose3D handPose,
                             PoseReferenceFrame affordanceFrame,
                             ArrayList<Color> colors)
   {
      this.interactableHand = interactableHand;
      this.handPose = handPose;
      this.handTransformToWorld = handTransformToWorld;
      this.affordanceFrame = affordanceFrame;
      this.colors = colors;
   }

   public void update()
   {
      for (int i = 0; i < poses.size(); ++i)
      {
         poses.set(i, new FramePose3D(frames.get(i)));
         poses.get(i).changeFrame(ReferenceFrame.getWorldFrame());
         frameGraphics.get(i).updateFromFramePose(poses.get(i));
      }
   }

   public void renderImGuiWidgets(ImGuiUniqueLabelMap labels, String id)
   {
      if (ImGui.button(labels.get("ADD") + "##" + id))
      {
         index++;
         PoseReferenceFrame frame = new PoseReferenceFrame(index + "Frame", affordanceFrame);
         frame.setPoseAndUpdate(handPose);

         poseIndices.add(index);
         poses.add(handPose);
         frames.add(frame);
         frameGraphics.add(new RDXReferenceFrameGraphic(0.1, colors.get(colorIndex % colors.size())));
         handConfigurations.add(null);
         colorIndex++;
         selectedFrameConfiguration = null;
         selectedIndex = poseIndices.size() - 1;
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("SET") + "##" + id))
      {
         frames.get(selectedIndex).setPoseAndUpdate(handPose);
         poses.set(selectedIndex, handPose);
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("CLEAR ALL") + "##" + id))
      {
         poseIndices.clear();
         frames.clear();
         poses.clear();
         frameGraphics.clear();
         colorIndex = 0;
         handConfigurations.clear();
         selectedFrameConfiguration = null;
         selectedIndex = -1;
      }
      if (poseIndices.size() > 0)
      {
         for (int i = 0; i < poseIndices.size(); ++i)
         {
            if (i % 5 != 0)
               ImGui.sameLine();
            if (selectedIndex == i)
            {
               ImGui.pushStyleColor(ImGuiCol.Button, 0.0f, 1.0f, 0.0f, 1.0f);
               changedColor = true;
            }
            if (ImGui.button(labels.get(poseIndices.get(i).toString()) + "##" + id))
            {
               // move hand to selected frame
               handTransformToWorld.set(poses.get(i));
               selectedIndex = i;
               if (handConfigurations.get(i) != null) // if hand configuration has been assigned to this frame
               {
                  interactableHand.setGripperToConfiguration(handConfigurations.get(i)); // update hand configuration when teleporting
                  selectedFrameConfiguration = handConfigurations.get(i);
               }
               else
                  selectedFrameConfiguration = null;
            }
            if (changedColor)
            {
               ImGui.popStyleColor();
               changedColor = false;
            }
            ImGui.sameLine();
            // handle the delete button click event here...
            if (ImGui.button(labels.get("X") + "##" + id + i, 15, 15))
            {
               frames.remove(i);
               poses.remove(i);
               frameGraphics.remove(i);
               handConfigurations.remove(i);
               poseIndices.remove(i);
               selectedFrameConfiguration = null;
               selectedIndex = -1;
            }
         }
      }
      else
      {
         colorIndex = 0;
         index = 0;
      }

      ImGui.text("Hand Configuration: " + (selectedFrameConfiguration == null ? "" : selectedFrameConfiguration.toString()));
      ImGui.sameLine();
      if (ImGui.button(labels.get("SET") + "##hand" + id))
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
      frames.clear();
      poses.clear();
      frameGraphics.clear();
      handConfigurations.clear();
      colorIndex = 0;
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
