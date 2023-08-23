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
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;

public class RDXAffordanceFrames
{
   private final SideDependentList<ArrayList<FramePose3D>> poses = new SideDependentList<>();
   private final SideDependentList<ArrayList<PoseReferenceFrame>> poseFrames = new SideDependentList<>();
   private final SideDependentList<ArrayList<RDXReferenceFrameGraphic>> frameGraphics = new SideDependentList<>();
   private final ArrayList<Integer> poseIndices = new ArrayList<>();
   private int index = 0;
   private final ArrayList<Color> colors;
   private SideDependentList<Integer> colorIndex = new SideDependentList<Integer>();
   public final SideDependentList<RDXInteractableSakeGripper> interactableHands;
   private final SideDependentList<FramePose3D> handPoses;
   private final SideDependentList<ArrayList<HandConfiguration>> handConfigurations = new SideDependentList<>();
   private HandConfiguration selectedFrameConfiguration;
   private int selectedIndex = -1;
   private final SideDependentList<RigidBodyTransform> handTransformsToWorld;
   private final RigidBodyTransform objectTransformToWorld;
   private final ArrayList<RigidBodyTransform> objectTransforms = new ArrayList<>();
   boolean changedColor = false;
   private RobotSide activeSide;
   private RDXActiveAffordanceMenu[] activeMenu;
   private final RDXActiveAffordanceMenu menu;

   public RDXAffordanceFrames(SideDependentList<RDXInteractableSakeGripper> interactableHands,
                              SideDependentList<RigidBodyTransform> handTransformsToWorld,
                              SideDependentList<FramePose3D> handPoses,
                              RigidBodyTransform objectTransformToWorld,
                              RobotSide activeSide,
                              RDXActiveAffordanceMenu[] activeMenu,
                              ArrayList<Color> colors)
   {
      this.interactableHands = interactableHands;
      this.handPoses = handPoses;
      this.handTransformsToWorld = handTransformsToWorld;
      this.objectTransformToWorld = objectTransformToWorld;
      this.activeSide = activeSide;
      this.activeMenu = activeMenu;
      this.menu = activeMenu[0];
      this.colors = colors;

      for (RobotSide side : handPoses.keySet())
      {
         handConfigurations.put(side,  new ArrayList<>());
         poses.put(side,  new ArrayList<>());
         poseFrames.put(side,  new ArrayList<>());
         frameGraphics.put(side,  new ArrayList<>());
         colorIndex.put(side, 0);
      }
   }

   public void update()
   {
      for (RobotSide side : handPoses.keySet())
      {
         for (int i = 0; i < poses.size(); ++i)
         {
            poses.get(side).set(i, new FramePose3D(poseFrames.get(side).get(i)));
            poses.get(side).get(i).changeFrame(ReferenceFrame.getWorldFrame());
            frameGraphics.get(side).get(i).updateFromFramePose(poses.get(side).get(i));
         }
      }
   }

   public void renderImGuiWidgets(ImGuiUniqueLabelMap labels, String lableId)
   {
      if (ImGui.button(labels.get("ADD") + "##" + lableId))
      {
         activeMenu[0] = this.menu;
         addFrame(handPoses.get(activeSide));
         // add a spot for the object transform associated with this frame
         objectTransforms.add(new RigidBodyTransform(objectTransformToWorld));
         // add an empty spot for the hand configuration
         handConfigurations.get(activeSide).add(null);
         // select the frame you've just added
         selectedIndex = poseIndices.size() - 1;
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("SET") + "##" + lableId) && activeMenu[0].equals(this.menu))
      {
         poseFrames.get(activeSide).get(selectedIndex).setPoseAndUpdate(handPoses.get(activeSide));
         poses.get(activeSide).set(selectedIndex, handPoses.get(activeSide));
         objectTransforms.set(selectedIndex, new RigidBodyTransform(objectTransformToWorld));
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("CLEAR ALL") + "##" + lableId))
      {
         reset();
         activeMenu[0] = RDXActiveAffordanceMenu.NONE;
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
            if (ImGui.button(labels.get("R" + poseIndices.get(i).toString()) + "##" + lableId))
            {
               activeMenu[0] = this.menu;
               activeSide = RobotSide.RIGHT;
               selectFrame(i);
            }

            ImGui.sameLine();
            // Add a dummy spacing for a specific distance (change the 'x' value as needed)
            float distanceBeforeSeparator = 20.0f; // Set the desired distance
            ImGui.dummy(distanceBeforeSeparator, 1);
            // Draw the vertical separator
            ImGui.separator();
            ImGui.sameLine();
            // Add a dummy spacing for a specific distance after the separator if needed (change the 'x' value as needed)
            float distanceAfterSeparator = 20.0f; // Set the desired distance
            ImGui.dummy(distanceAfterSeparator, 1);

            if (ImGui.button(labels.get("L" + poseIndices.get(i).toString()) + "##" + lableId))
            {
               activeMenu[0] = this.menu;
               activeSide = RobotSide.LEFT;
               selectFrame(i);
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
               poseFrames.get(activeSide).remove(i);
               poses.get(activeSide).remove(i);
               frameGraphics.get(activeSide).remove(i);
               handConfigurations.get(activeSide).remove(i);
               poseIndices.remove(i);
               objectTransforms.remove(i);
               selectedFrameConfiguration = null;
               selectedIndex = -1;
               activeMenu[0] = RDXActiveAffordanceMenu.NONE;
            }
            ImGui.popStyleColor();
         }
      }
      else
      {
         colorIndex.replace(activeSide, 0);
         index = 0;
      }

      ImGui.text("Hand Configuration: " + (selectedFrameConfiguration == null ? "" : selectedFrameConfiguration.toString()));
      ImGui.sameLine();
      if (ImGui.button(labels.get("SET") + "##hand" + lableId) && activeMenu[0].equals(this.menu))
      {
         if (selectedIndex >= 0)
         {
            handConfigurations.get(activeSide).set(selectedIndex, interactableHands.get(activeSide).getConfiguration());
            selectedFrameConfiguration = handConfigurations.get(activeSide).get(selectedIndex);
         }
      }
   }

   public void addFrame(FramePose3D poseReference)
   {
      index++;
      poseReference.changeFrame(ReferenceFrame.getWorldFrame());
      PoseReferenceFrame frame = new PoseReferenceFrame(activeSide.getLowerCaseName() + index + "Frame", poseReference.getReferenceFrame());
      frame.setPoseAndUpdate(poseReference);

      poseIndices.add(index);
      poses.get(activeSide).add(poseReference);

      poseFrames.get(activeSide).add(frame);
      frameGraphics.get(activeSide).add(new RDXReferenceFrameGraphic(0.1, colors.get(colorIndex.get(activeSide) % colors.size())));
      colorIndex.replace(activeSide, colorIndex.get(activeSide) + 1);

      // no hand configuration is set right when you add a new frame
      selectedFrameConfiguration = null;
   }

   public void selectFrame(int index)
   {
      // move hand to selected frame
      handTransformsToWorld.get(activeSide).set(poses.get(activeSide).get(index));
      selectedIndex = index;
      // if hand configuration has been assigned to this frame
      if (handConfigurations.get(activeSide).get(index) != null)
      {
         interactableHands.get(activeSide).setGripperToConfiguration(handConfigurations.get(activeSide).get(index)); // update hand configuration when teleporting
         selectedFrameConfiguration = handConfigurations.get(activeSide).get(index);
      }
      else
         selectedFrameConfiguration = null;
      // update pose of the object
      objectTransformToWorld.set(objectTransforms.get(selectedIndex));
   }

   public void addObjectTransform(RigidBodyTransform transform)
   {
      objectTransforms.add(transform);
   }

   public void addHandConfiguration(HandConfiguration configuration, RobotSide side)
   {
      handConfigurations.get(side).add(configuration);
   }

   public void reset()
   {
      for (RobotSide side : handPoses.keySet())
      {
         poseFrames.get(side).clear();
         poses.get(side).clear();
         frameGraphics.get(side).clear();
         colorIndex.replace(side, 0);
         handConfigurations.get(side).clear();
      }
      poseIndices.clear();
      objectTransforms.clear();
      selectedFrameConfiguration = null;
      selectedIndex = -1;
      index = 0;
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (RobotSide side : RobotSide.values)
         for (RDXReferenceFrameGraphic frameGraphic : frameGraphics.get(side))
         {
            frameGraphic.getRenderables(renderables, pool);
         }
   }

   public int getNumberOfFrames()
   {
      return poseFrames.get(RobotSide.RIGHT).size();
   }

   public ArrayList<FramePose3D> getPoses(RobotSide side)
   {
      return poses.get(side);
   }

   public ArrayList<HandConfiguration> getHandConfigurations(RobotSide side)
   {
      return handConfigurations.get(side);
   }

   public ArrayList<RigidBodyTransform> getObjectTransforms()
   {
      return objectTransforms;
   }

   public void selectNext()
   {
      selectedIndex++;
      selectFrame(selectedIndex);
   }

   public void selectPrevious()
   {
      selectedIndex--;
      selectFrame(selectedIndex);
   }

   public void resetSelectedIndex()
   {
      selectedIndex = -1;
   }

   public void setSelectedIndexToSize()
   {
      selectedIndex = poseIndices.size();
   }

   public boolean isFirst()
   {
      return selectedIndex == 0;
   }

   public boolean isLast()
   {
      return selectedIndex == poseIndices.size() - 1;
   }
}