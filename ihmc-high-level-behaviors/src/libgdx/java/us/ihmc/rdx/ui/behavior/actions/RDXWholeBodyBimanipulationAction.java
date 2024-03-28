package us.ihmc.rdx.ui.behavior.actions;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.sequence.actions.WholeBodyBimanipulationActionDefinition;
import us.ihmc.behaviors.sequence.actions.WholeBodyBimanipulationActionState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.rdx.imgui.ImGuiSliderDoubleWrapper;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.input.ImGui3DViewPickResult;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDX3DPanelTooltip;
import us.ihmc.rdx.ui.affordances.RDXInteractableHand;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionNode;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.rdx.ui.teleoperation.RDXDesiredRobot;
import us.ihmc.robotics.interaction.MouseCollidable;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Set;

public class RDXWholeBodyBimanipulationAction extends RDXActionNode<WholeBodyBimanipulationActionState, WholeBodyBimanipulationActionDefinition>
{
   private final WholeBodyBimanipulationActionState state;
   private final WholeBodyBimanipulationActionDefinition definition;
   private final ImGui3DViewPickResult pickResult = new ImGui3DViewPickResult();
   private final RDX3DPanelTooltip tooltip;
   private final SideDependentList<RDXInteractableHand> interactableHands = new SideDependentList<>();
   private final RDXDesiredRobot desiredRobot;

   public RDXWholeBodyBimanipulationAction(long id,
                                           CRDTInfo crdtInfo,
                                           WorkspaceResourceDirectory saveFileDirectory,
                                           RDX3DPanel panel3D,
                                           ReferenceFrameLibrary referenceFrameLibrary,
                                           DRCRobotModel robotModel,
                                           ROS2SyncedRobotModel syncedRobot)
   {
      super(new WholeBodyBimanipulationActionState(id, crdtInfo, saveFileDirectory, referenceFrameLibrary));

      state = getState();
      definition = getDefinition();

      definition.setName("Wholebody Bimanipulation");

      desiredRobot = new RDXDesiredRobot(robotModel);
      desiredRobot.setSceneLevels(RDXSceneLevel.VIRTUAL);
      desiredRobot.create();
      desiredRobot.setActive(true);

      tooltip = new RDX3DPanelTooltip(panel3D);
      panel3D.addImGuiOverlayAddition(this::render3DPanelImGuiOverlays);
   }

   @Override
   public void update()
   {
      super.update();
      desiredRobot.update();
   }

//   @Override
//   protected void renderImGuiWidgetsInternal()
//   {
//   }

   public void render3DPanelImGuiOverlays()
   {
//      if (isMouseHovering)
//      {
//         tooltip.render("%s Action\nIndex: %d\nName: %s".formatted(getActionTypeTitle(), state.getActionIndex(), definition.getName()));
//      }
   }

   @Override
   public void calculate3DViewPick(ImGui3DViewInput input)
   {
//      if (state.getPalmFrame().isChildOfWorld())
//      {
//         poseGizmo.calculate3DViewPick(input);
//
//         pickResult.reset();
//         for (MouseCollidable mouseCollidable : mouseCollidables)
//         {
//            double collision = mouseCollidable.collide(input.getPickRayInWorld(), collisionShapeFrame.getReferenceFrame());
//            if (!Double.isNaN(collision))
//               pickResult.addPickCollision(collision);
//         }
//         if (pickResult.getPickCollisionWasAddedSinceReset())
//            input.addPickResult(pickResult);
//      }
   }

   @Override
   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (state.getHandFrame(RobotSide.LEFT).isChildOfWorld())
      {
         for (RobotSide side : interactableHands.sides())
         {
            interactableHands.get(side).process3DViewInput(input);
         }

         tooltip.setInput(input);
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      desiredRobot.getRenderables(renderables, pool, Collections.singleton(RDXSceneLevel.VIRTUAL));
   }

   @Override
   public String getActionTypeTitle()
   {
      return "Whole Body Bimanipulation";
   }
}
