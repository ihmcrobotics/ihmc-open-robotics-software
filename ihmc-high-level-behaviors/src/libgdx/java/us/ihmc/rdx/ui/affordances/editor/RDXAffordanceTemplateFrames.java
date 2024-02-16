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

import java.util.ArrayList;
import java.util.List;

public class RDXAffordanceTemplateFrames
{
   private final SideDependentList<List<FramePose3D>> poses = new SideDependentList<>();
   private final SideDependentList<List<PoseReferenceFrame>> poseFrames = new SideDependentList<>();
   private final SideDependentList<List<RDXReferenceFrameGraphic>> frameGraphics = new SideDependentList<>();
   private final List<Integer> poseIndices = new ArrayList<>();
   private int index = 0;
   private final List<Color> colors;
   private int colorIndex = 0;
   private boolean changedColor = false;
   private final SideDependentList<RDXInteractableAffordanceTemplateHand> interactableHands;
   private final SideDependentList<FramePose3D> handPoses;
   private final SideDependentList<List<String>> handConfigurations = new SideDependentList<>();
   private final SideDependentList<List<Boolean>> arePosesSet = new SideDependentList<>();
   private String selectedFrameConfiguration;
   private int selectedIndex = -1;
   private final SideDependentList<RigidBodyTransform> handTransformsToWorld;
   private final RigidBodyTransform objectTransformToWorld;
   private final List<RigidBodyTransform> objectTransforms = new ArrayList<>();
   private final RDXAffordanceTemplateEditorStatus editorStatus;
   private final RDXActiveAffordanceMenu menu;

   public RDXAffordanceTemplateFrames(SideDependentList<RDXInteractableAffordanceTemplateHand> interactableHands,
                                      SideDependentList<RigidBodyTransform> handTransformsToWorld,
                                      SideDependentList<FramePose3D> handPoses,
                                      RigidBodyTransform objectTransformToWorld,
                                      RDXAffordanceTemplateEditorStatus editorStatus,
                                      ArrayList<Color> colors)
   {
      this.interactableHands = interactableHands;
      this.handPoses = handPoses;
      this.handTransformsToWorld = handTransformsToWorld;
      this.objectTransformToWorld = objectTransformToWorld;
      this.editorStatus = editorStatus;
      this.menu = editorStatus.getActiveMenu();
      this.colors = colors;

      for (RobotSide side : handPoses.keySet())
      {
         handConfigurations.put(side,  new ArrayList<>());
         poses.put(side,  new ArrayList<>());
         poseFrames.put(side,  new ArrayList<>());
         arePosesSet.put(side, new ArrayList<>());
         frameGraphics.put(side,  new ArrayList<>());
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

   public void renderImGuiWidgets(ImGuiUniqueLabelMap labels, String labelId, boolean editingBothHands)
   {
      RobotSide activeSide = editorStatus.getActiveSide();
      if (ImGui.button(labels.get("Add") + "##" + labelId) && handPoses.containsKey(activeSide))
      {
         editorStatus.setActiveMenu(this.menu);
         addFrame(handPoses.get(activeSide));
         // add a spot for the object transform associated with this frame
         objectTransforms.add(new RigidBodyTransform(objectTransformToWorld));
         // select the frame you've just added
         selectedIndex = poseIndices.size() - 1;
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("Set") + "##" + labelId) && editorStatus.getActiveMenu().equals(this.menu))
      {
         if (editingBothHands)
         {
            for (RobotSide side : handPoses.keySet())
            {
               poseFrames.get(side).get(selectedIndex).setPoseAndUpdate(handPoses.get(side));
               poses.get(side).set(selectedIndex, handPoses.get(side));
               objectTransforms.set(selectedIndex, new RigidBodyTransform(objectTransformToWorld));
               if (frameGraphics.get(side).get(selectedIndex) == null || !arePosesSet.get(side).get(selectedIndex))
               {
                  frameGraphics.get(side).set(selectedIndex, new RDXReferenceFrameGraphic(0.1, colors.get(colorIndex % colors.size())));
                  colorIndex++;
               }
               arePosesSet.get(side).set(selectedIndex, true);
            }
         }
         else
         {
            poseFrames.get(activeSide).get(selectedIndex).setPoseAndUpdate(handPoses.get(activeSide));
            poses.get(activeSide).set(selectedIndex, handPoses.get(activeSide));
            objectTransforms.set(selectedIndex, new RigidBodyTransform(objectTransformToWorld));
            if (frameGraphics.get(activeSide).get(selectedIndex) == null || !arePosesSet.get(activeSide).get(selectedIndex))
            {
               frameGraphics.get(activeSide).set(selectedIndex, new RDXReferenceFrameGraphic(0.1, colors.get(colorIndex % colors.size())));
               colorIndex++;
            }
            arePosesSet.get(activeSide).set(selectedIndex, true);
         }
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("Clear All") + "##" + labelId))
      {
         reset();
         editorStatus.setActiveMenu(RDXActiveAffordanceMenu.NONE);
      }
      for (RobotSide side : handPoses.keySet())
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
               if (selectedIndex == i && editorStatus.getActiveMenu().equals(this.menu) && activeSide == side)
               {
                  ImGui.pushStyleColor(ImGuiCol.Button, 0.0f, 1.0f, 0.0f, 1.0f);
                  changedColor = true;
               }
               if (ImGui.button(labels.get((side  == RobotSide.RIGHT ? "R" : "L") + poseIndices.get(i).toString()) + "##" + labelId))
               {
                  editorStatus.setActiveMenu(this.menu);
                  editorStatus.setActiveSide(side);
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
               if (ImGui.button(labels.get("X") + "##" + labelId + (side  == RobotSide.RIGHT ? "R" : "L") + i, 15, 15))
               {
                  for (RobotSide eachSide : handPoses.keySet())
                  {
                     poseFrames.get(eachSide).remove(i);
                     poses.get(eachSide).remove(i);
                     arePosesSet.get(eachSide).remove(i);
                     frameGraphics.get(eachSide).remove(i);
                     handConfigurations.get(eachSide).remove(i);
                  }
                  poseIndices.remove(i);
                  objectTransforms.remove(i);
                  selectedFrameConfiguration = null;
                  selectedIndex = -1;
                  editorStatus.setActiveMenu(RDXActiveAffordanceMenu.NONE);
               }
               ImGui.popStyleColor();
            }
         }

         else
         {
            colorIndex = 0;
            index = 0;
         }
      }
      activeSide = editorStatus.getActiveSide();
      ImGui.text("Hand Configuration: " + (selectedFrameConfiguration == null ? "" : selectedFrameConfiguration.toString()));
      ImGui.sameLine();
      if (ImGui.button(labels.get("Set") + "##hand" + labelId) && editorStatus.getActiveMenu().equals(this.menu))
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
      RobotSide activeSide = editorStatus.getActiveSide();
      poseReference.changeFrame(ReferenceFrame.getWorldFrame());
      PoseReferenceFrame frame = new PoseReferenceFrame(activeSide.getLowerCaseName() + index + "Frame", poseReference.getReferenceFrame());
      frame.setPoseAndUpdate(poseReference);

      poseIndices.add(index);
      poses.get(activeSide).add(poseReference);
      poseFrames.get(activeSide).add(frame);
      arePosesSet.get(activeSide).add(true);
      frameGraphics.get(activeSide).add(new RDXReferenceFrameGraphic(0.1, colors.get(colorIndex % colors.size())));
      colorIndex++;

      // no hand configuration is set right when you add a new frame
      selectedFrameConfiguration = null;
      // add an empty spot for the hand configurations
      handConfigurations.get(activeSide).add(null);

      //add frame for the other hand, with same pose from previous frame associated with that side
      RobotSide nonActiveSide = activeSide == RobotSide.RIGHT ? RobotSide.LEFT : RobotSide.RIGHT;
      if (handPoses.containsKey(nonActiveSide))
      {
         FramePose3D otherSidePoseReference = new FramePose3D(handPoses.get(nonActiveSide));
         otherSidePoseReference.changeFrame(ReferenceFrame.getWorldFrame());
         frame = new PoseReferenceFrame(nonActiveSide.getLowerCaseName() + index + "Frame", otherSidePoseReference.getReferenceFrame());
         frame.setPoseAndUpdate(otherSidePoseReference);

         poses.get(nonActiveSide).add(otherSidePoseReference);
         poseFrames.get(nonActiveSide).add(frame);
         arePosesSet.get(nonActiveSide).add(false);
         // add empty graphics if initial pose is not set
         boolean hasFrameBeenSetOnce = false;
         for (boolean isInitialPoseSet : arePosesSet.get(nonActiveSide))
         {
            if (isInitialPoseSet)
            {
               hasFrameBeenSetOnce = true;
               break;
            }
         }
         if (!hasFrameBeenSetOnce)
            frameGraphics.get(nonActiveSide).add(null);
         else
         {
            frameGraphics.get(nonActiveSide).add(new RDXReferenceFrameGraphic(0.1, Color.RED));
         }
         // add an empty spot for the hand configurations
         handConfigurations.get(nonActiveSide).add(null);
      }
   }

   public void loadFrame(FramePose3D poseReference, RobotSide side, int index)
   {
      poseReference.changeFrame(ReferenceFrame.getWorldFrame());
      PoseReferenceFrame frame = new PoseReferenceFrame(side.getLowerCaseName() + index + "Frame", poseReference.getReferenceFrame());
      frame.setPoseAndUpdate(poseReference);

      poses.get(side).add(poseReference);
      poseFrames.get(side).add(frame);
      boolean hasFrameBeenSetOnce = false;
      for (boolean isInitialPoseSet : arePosesSet.get(side))
      {
         if (isInitialPoseSet)
         {
            hasFrameBeenSetOnce = true;
            break;
         }
      }
      if (!hasFrameBeenSetOnce)
         frameGraphics.get(side).add(null);
      else if (!arePosesSet.get(side).get(index-1))
      {
         frameGraphics.get(side).add(new RDXReferenceFrameGraphic(0.1, Color.RED));
      }
      else
         frameGraphics.get(side).add(new RDXReferenceFrameGraphic(0.1, colors.get(colorIndex % colors.size())));
      colorIndex++;

      // no hand configuration is set right when you add a new frame
      selectedFrameConfiguration = null;
      // add an empty spot for the hand configurations
      handConfigurations.get(side).add(null);
   }

   public void selectFrame(int index)
   {
      for (RobotSide side : handPoses.keySet())
      {
         // move hand to selected frame
         handTransformsToWorld.get(side).set(poses.get(side).get(index));
         // if hand configuration has been assigned to this frame
         if (handConfigurations.get(side).get(index) != null)
         {
            interactableHands.get(side).setToConfiguration(handConfigurations.get(side).get(index)); // update hand configuration when teleporting
            if (side == editorStatus.getActiveSide())
               selectedFrameConfiguration = handConfigurations.get(side).get(index);
         }
         else if (side == editorStatus.getActiveSide())
         {
            selectedFrameConfiguration = null;
         }

         interactableHands.get(side).setSelected(side == editorStatus.getActiveSide());
      }
      selectedIndex = index;
      // update pose of the object
      objectTransformToWorld.set(objectTransforms.get(selectedIndex));
      editorStatus.disableMirror();
   }

   public void addObjectTransform(RigidBodyTransform transform)
   {
      objectTransforms.add(transform);
   }

   public void addHandConfiguration(String configuration, RobotSide side)
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
         handConfigurations.get(side).clear();
         arePosesSet.get(side).clear();
      }
      poseIndices.clear();
      objectTransforms.clear();
      selectedFrameConfiguration = null;
      selectedIndex = -1;
      index = 0;
      colorIndex = 0;
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (RobotSide side : handPoses.keySet())
         for (RDXReferenceFrameGraphic frameGraphic : frameGraphics.get(side))
         {
            if (frameGraphic != null)
               frameGraphic.getRenderables(renderables, pool);
         }
   }

   public int getNumberOfFrames(RobotSide side)
   {
      return poseFrames.get(side).size();
   }

   public SideDependentList<List<FramePose3D>> getPoses()
   {
      return poses;
   }

   public SideDependentList<List<Boolean>> getArePosesSet()
   {
      return arePosesSet;
   }

   public SideDependentList<List<String>> getHandConfigurations()
   {
      return handConfigurations;
   }

   public List<RigidBodyTransform> getObjectTransforms()
   {
      return objectTransforms;
   }

   public void addIndexPose(int index)
   {
      poseIndices.add(index);
      selectedIndex = index;
      this.index = index;
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