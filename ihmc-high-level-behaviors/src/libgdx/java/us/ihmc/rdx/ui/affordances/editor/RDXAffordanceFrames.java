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
import java.util.List;

public class RDXAffordanceFrames
{
   private final SideDependentList<List<FramePose3D>> poses = new SideDependentList<>();
   private final SideDependentList<List<PoseReferenceFrame>> poseFrames = new SideDependentList<>();
   private final SideDependentList<List<RDXReferenceFrameGraphic>> frameGraphics = new SideDependentList<>();
   private final List<Integer> poseIndices = new ArrayList<>();
   private int index = 0;
   private final List<Color> colors;
   private SideDependentList<Integer> colorIndex = new SideDependentList<Integer>();
   public final SideDependentList<RDXInteractableSakeGripper> interactableHands;
   private final SideDependentList<FramePose3D> handPoses;
   private final SideDependentList<List<HandConfiguration>> handConfigurations = new SideDependentList<>();
   private HandConfiguration selectedFrameConfiguration;
   private int selectedIndex = -1;
   private final SideDependentList<RigidBodyTransform> handTransformsToWorld;
   private final RigidBodyTransform objectTransformToWorld;
   private final List<RigidBodyTransform> objectTransforms = new ArrayList<>();
   boolean changedColor = false;
   private RobotSide[] activeSide;
   private RDXActiveAffordanceMenu[] activeMenu;
   private final RDXActiveAffordanceMenu menu;

   public RDXAffordanceFrames(SideDependentList<RDXInteractableSakeGripper> interactableHands,
                              SideDependentList<RigidBodyTransform> handTransformsToWorld,
                              SideDependentList<FramePose3D> handPoses,
                              RigidBodyTransform objectTransformToWorld,
                              RobotSide[] activeSide,
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
         for (int i = 0; i < poses.get(side).size(); ++i)
         {
            poses.get(side).set(i, new FramePose3D(poseFrames.get(side).get(i)));
            poses.get(side).get(i).changeFrame(ReferenceFrame.getWorldFrame());
            if (frameGraphics.get(side).get(i) != null)
               frameGraphics.get(side).get(i).updateFromFramePose(poses.get(side).get(i));
         }
      }
   }

   public void renderImGuiWidgets(ImGuiUniqueLabelMap labels, String lableId)
   {
      if (ImGui.button(labels.get("ADD") + "##" + lableId))
      {
         activeMenu[0] = this.menu;
         addFrame(handPoses.get(activeSide[0]));
         // add a spot for the object transform associated with this frame
         objectTransforms.add(new RigidBodyTransform(objectTransformToWorld));
         // select the frame you've just added
         selectedIndex = poseIndices.size() - 1;
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("SET") + "##" + lableId) && activeMenu[0].equals(this.menu))
      {
         poseFrames.get(activeSide[0]).get(selectedIndex).setPoseAndUpdate(handPoses.get(activeSide[0]));
         poses.get(activeSide[0]).set(selectedIndex, handPoses.get(activeSide[0]));
         objectTransforms.set(selectedIndex, new RigidBodyTransform(objectTransformToWorld));
         if (frameGraphics.get(activeSide[0]).get(selectedIndex) == null)
         {
            frameGraphics.get(activeSide[0]).set(selectedIndex, new RDXReferenceFrameGraphic(0.1, colors.get(colorIndex.get(activeSide[0]) % colors.size())));
         }
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("CLEAR ALL") + "##" + lableId))
      {
         reset();
         activeMenu[0] = RDXActiveAffordanceMenu.NONE;
      }
      for (RobotSide side : RobotSide.values)
      {
         if (poseIndices.size() > 0)
         {
            for (int i = 0; i < poseIndices.size(); ++i)
            {
               if (side == RobotSide.RIGHT && i == 0)
               {
                  ImGui.text("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
               }

               if (i % 5 != 0)
                  ImGui.sameLine();
               // display selected frame in green
               if (selectedIndex == i && activeMenu[0].equals(this.menu) && activeSide[0] == side)
               {
                  ImGui.pushStyleColor(ImGuiCol.Button, 0.0f, 1.0f, 0.0f, 1.0f);
                  changedColor = true;
               }
               if (ImGui.button(labels.get((side  == RobotSide.RIGHT ? "R" : "L") + poseIndices.get(i).toString()) + "##" + lableId))
               {
                  activeMenu[0] = this.menu;
                  activeSide[0] = side;
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
               if (ImGui.button(labels.get("X") + "##" + lableId + (side  == RobotSide.RIGHT ? "R" : "L") + i, 15, 15))
               {
                  for (RobotSide eachSide : RobotSide.values)
                  {
                     poseFrames.get(eachSide).remove(i);
                     poses.get(eachSide).remove(i);
                     frameGraphics.get(eachSide).remove(i);
                     handConfigurations.get(eachSide).remove(i);
                  }
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
            colorIndex.replace(activeSide[0], 0);
            index = 0;
         }
      }
      ImGui.text("Hand Configuration: " + (selectedFrameConfiguration == null ? "" : selectedFrameConfiguration.toString()));
      ImGui.sameLine();
      if (ImGui.button(labels.get("SET") + "##hand" + lableId) && activeMenu[0].equals(this.menu))
      {
         if (selectedIndex >= 0)
         {
            handConfigurations.get(activeSide[0]).set(selectedIndex, interactableHands.get(activeSide[0]).getConfiguration());
            selectedFrameConfiguration = handConfigurations.get(activeSide[0]).get(selectedIndex);
         }
      }
   }

   public void addFrame(FramePose3D poseReference)
   {
      index++;
      poseReference.changeFrame(ReferenceFrame.getWorldFrame());
      PoseReferenceFrame frame = new PoseReferenceFrame(activeSide[0].getLowerCaseName() + index + "Frame", poseReference.getReferenceFrame());
      frame.setPoseAndUpdate(poseReference);

      poseIndices.add(index);
      poses.get(activeSide[0]).add(poseReference);
      poseFrames.get(activeSide[0]).add(frame);
      frameGraphics.get(activeSide[0]).add(new RDXReferenceFrameGraphic(0.1, colors.get(colorIndex.get(activeSide[0]) % colors.size())));
      colorIndex.replace(activeSide[0], colorIndex.get(activeSide[0]) + 1);

      //add an empty graphics frame for the other hand, with same pose from previous frame associated with that side
      RobotSide nonActiveSide = activeSide[0] == RobotSide.RIGHT ? RobotSide.LEFT : RobotSide.RIGHT;
      FramePose3D otherSidePoseReference;
      int nonActiveSideSize = poses.get(nonActiveSide).size();
      if (nonActiveSideSize > 1) // if there is a previous frame
         otherSidePoseReference = new FramePose3D(poses.get(nonActiveSide).get(nonActiveSideSize - 1));
      else // use current pose
         otherSidePoseReference = new FramePose3D(handPoses.get(nonActiveSide));
      otherSidePoseReference.changeFrame(ReferenceFrame.getWorldFrame());
      frame = new PoseReferenceFrame(nonActiveSide.getLowerCaseName() + index + "Frame", otherSidePoseReference.getReferenceFrame());
      frame.setPoseAndUpdate(otherSidePoseReference);

      poses.get(nonActiveSide).add(otherSidePoseReference);
      poseFrames.get(nonActiveSide).add(frame);
      frameGraphics.get(nonActiveSide).add(null);

      // no hand configuration is set right when you add a new frame
      selectedFrameConfiguration = null;
      // add an empty spot for the hand configurations
      handConfigurations.get(activeSide[0]).add(null);
      handConfigurations.get(nonActiveSide).add(null);
   }

   public void selectFrame(int index)
   {
      for (RobotSide side : RobotSide.values)
      {
         // move hand to selected frame
         handTransformsToWorld.get(side).set(poses.get(side).get(index));
         // if hand configuration has been assigned to this frame
         if (handConfigurations.get(side).get(index) != null)
         {
            interactableHands.get(side).setGripperToConfiguration(handConfigurations.get(side).get(index)); // update hand configuration when teleporting
            selectedFrameConfiguration = handConfigurations.get(side).get(index);
         }
         else
            selectedFrameConfiguration = null;

         if (side == activeSide[0])
            interactableHands.get(side).setSelected(true);
         else
            interactableHands.get(side).setSelected(false);
      }
      selectedIndex = index;
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
            if (frameGraphic != null)
               frameGraphic.getRenderables(renderables, pool);
         }
   }

   public int getNumberOfFrames()
   {
      return poseFrames.get(RobotSide.RIGHT).size();
   }

   public List<FramePose3D> getPoses(RobotSide side)
   {
      return poses.get(side);
   }

   public List<HandConfiguration> getHandConfigurations(RobotSide side)
   {
      return handConfigurations.get(side);
   }

   public List<RigidBodyTransform> getObjectTransforms()
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