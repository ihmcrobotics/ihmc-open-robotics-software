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

public class RDXAffordanceFrame
{
   private SideDependentList<FramePose3D> poses = new SideDependentList<>();
   private SideDependentList<Boolean> isPoseSet = new SideDependentList<>();
   private final SideDependentList<PoseReferenceFrame> poseFrames = new SideDependentList<>();
   private final SideDependentList<RDXReferenceFrameGraphic> frameGraphics = new SideDependentList<>();
   private SideDependentList<HandConfiguration> handConfigurations = new SideDependentList<>();
   private final SideDependentList<RDXInteractableSakeGripper> interactableHands;
   private final SideDependentList<FramePose3D> handPoses;
   private final SideDependentList<RigidBodyTransform> handTransformsToWorld;
   private final RigidBodyTransform objectTransformToWorld;
   private final RigidBodyTransform objectTransformOfFrame = new RigidBodyTransform();
   private final RobotSide[] activeSide;
   private final RDXActiveAffordanceMenu[] activeMenu;
   private RDXActiveAffordanceMenu menu;
   private boolean changedColor = false;

   public RDXAffordanceFrame(SideDependentList<RDXInteractableSakeGripper> interactableHands,
                             SideDependentList<RigidBodyTransform> handTransformsToWorld,
                             SideDependentList<FramePose3D> handPoses,
                             RigidBodyTransform objectTransformToWorld,
                             RobotSide[] activeSide,
                             RDXActiveAffordanceMenu[] activeMenu,
                             Color color)
   {
      this.interactableHands = interactableHands;
      this.handPoses = handPoses;
      this.handTransformsToWorld = handTransformsToWorld;
      this.objectTransformToWorld = objectTransformToWorld;
      this.activeSide = activeSide;
      this.activeMenu = activeMenu;
      this.menu = activeMenu[0];

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

   public void renderImGuiWidgets(ImGuiUniqueLabelMap labels, String labelId)
   {
      if (ImGui.button(labels.get("SET") + "##" + labelId) && handPoses.containsKey(activeSide[0]))
      {
         isPoseSet.replace(activeSide[0], true);
         setFrame(handPoses.get(activeSide[0]));
         objectTransformOfFrame.set(objectTransformToWorld);
         activeMenu[0] = this.menu;
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("CLEAR ALL") + "##" + labelId))
      {
         reset();
         activeMenu[0] = RDXActiveAffordanceMenu.NONE;
      }

      for (RobotSide side : RobotSide.values)
      {
         if (isPoseSet.get(side))
         {
            if (side == RobotSide.RIGHT && isPoseSet.get(RobotSide.LEFT))
            {
               ImGui.text("- - - - - - - - - - -");
            }

            if(activeMenu[0].equals(this.menu) && activeSide[0] == side)
            {
               changedColor = true;
               ImGui.pushStyleColor(ImGuiCol.Button, 0.0f, 1.0f, 0.0f, 1.0f);
            }
            if (ImGui.button(labels.get((side == RobotSide.LEFT ? "L" : "R") + " Grasp Frame") + "##" + labelId))
            {
               activeSide[0] = side;
               activeMenu[0] = this.menu;
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
               activeMenu[0] = RDXActiveAffordanceMenu.NONE;
            }
            ImGui.popStyleColor();
         }
      }

      ImGui.text("Hand Configuration: " + (handConfigurations.get(activeSide[0]) == null ? "" : handConfigurations.get(activeSide[0]).toString()));
      ImGui.sameLine();
      if (ImGui.button(labels.get("SET") + "##hand" + labelId) && activeMenu[0].equals(this.menu))
      {
         handConfigurations.replace(activeSide[0], interactableHands.get(activeSide[0]).getConfiguration());
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
      poses.replace(activeSide[0], new FramePose3D(poseReference.getReferenceFrame(), poseReference));
      poses.get(activeSide[0]).changeFrame(ReferenceFrame.getWorldFrame());
      poseFrames.get(activeSide[0]).setPoseAndUpdate(poses.get(activeSide[0]));
      isPoseSet.replace(activeSide[0], true);
      frameGraphics.get(activeSide[0]).updateFromFramePose(poses.get(activeSide[0]));
   }

   public void selectFrame()
   {
      if (isPoseSet.get(activeSide[0]))
      {
         handTransformsToWorld.get(activeSide[0]).set(poses.get(activeSide[0]));  // move hand to grasp point
         objectTransformToWorld.set(objectTransformOfFrame);
      }
      if (handConfigurations.get(activeSide[0]) != null)
         interactableHands.get(activeSide[0]).setGripperToConfiguration(handConfigurations.get(activeSide[0]));

      for (RobotSide side : RobotSide.values)
      {
         if (side == activeSide[0])
            interactableHands.get(side).setSelected(true);
         else
            interactableHands.get(side).setSelected(false);
      }
   }

   public void setHandConfiguration(HandConfiguration configuration)
   {
      handConfigurations.replace(activeSide[0], configuration);
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

   public HandConfiguration getHandConfiguration(RobotSide side)
   {
      return handConfigurations.get(side);
   }

   public RigidBodyTransform getObjectTransform()
   {
      return objectTransformOfFrame;
   }
}