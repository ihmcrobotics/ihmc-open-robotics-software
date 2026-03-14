package us.ihmc.rdx.behaviorTree.actions;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.behaviors.behaviorTree.action.actions.LegActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.LegActionState;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;
import us.ihmc.rdx.behaviorTree.RDXCRDTTools;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiReferenceFrameLibraryCombo;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.input.ImGui3DViewPickResult;
import us.ihmc.rdx.ui.RDX3DPanelTooltip;
import us.ihmc.rdx.ui.affordances.RDXInteractableHighlightModel;
import us.ihmc.rdx.ui.affordances.RDXInteractableTools;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.robotics.MultiBodySystemMissingTools;
import us.ihmc.robotics.interaction.MouseCollidable;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.simulation.collision.Collidable;

import java.util.ArrayList;
import java.util.List;

public class RDXLegAction extends RDXActionNode<LegActionState, LegActionDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiReferenceFrameLibraryCombo parentFrameComboBox;
   private final ImDoubleWrapper trajectoryDurationWidget;
   /** Gizmo is control frame */
   private final RDXSelectablePose3DGizmo poseGizmo;
   private final MutableReferenceFrame graphicFrame = new MutableReferenceFrame();
   private final MutableReferenceFrame collisionShapeFrame = new MutableReferenceFrame();
   private boolean isMouseHovering = false;
   private final ImGui3DViewPickResult pickResult = new ImGui3DViewPickResult();
   private final ArrayList<MouseCollidable> mouseCollidables = new ArrayList<>();
   private final SideDependentList<RDXInteractableHighlightModel> highlightModels = new SideDependentList<>();
   private final RDX3DPanelTooltip tooltip;

   public RDXLegAction(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(new LegActionState(id, rootNode.getState()), rootNode);

      poseGizmo = new RDXSelectablePose3DGizmo();
      poseGizmo.create(panel3D);

      parentFrameComboBox = new ImGuiReferenceFrameLibraryCombo("Parent frame",
                                                                scene::getAllFrameNames,
                                                                definition::getParentFrameName,
                                                                state.getFootFrame()::changeFrame);
      trajectoryDurationWidget = new ImDoubleWrapper(definition::getTrajectoryDuration,
                                                     definition::setTrajectoryDuration,
                                                     imDouble -> ImGuiTools.volatileInputDouble(labels.get("Trajectory duration"), imDouble));

      for (RobotSide side : RobotSide.values)
      {
         String footBodyName = syncedRobot.getFullRobotModel().getFoot(side).getName();
         String modelFileName = RDXInteractableTools.getModelFileName(robotModel.getRobotDefinition().getRigidBodyDefinition(footBodyName));
         highlightModels.put(side, new RDXInteractableHighlightModel(modelFileName));

         MultiBodySystemBasics footOnlySystem = MultiBodySystemMissingTools.createSingleBodySystem(syncedRobot.getFullRobotModel().getFoot(side));
         List<Collidable> footCollidables = selectionCollisionModel.getRobotCollidables(footOnlySystem);

         for (Collidable footCollidable : footCollidables)
         {
            mouseCollidables.add(new MouseCollidable(footCollidable));
         }
      }

      tooltip = new RDX3DPanelTooltip(panel3D);
      panel3D.addImGuiOverlayAddition(this::render3DPanelImGuiOverlays);
   }

   @Override
   public void update()
   {
      super.update();

      if (state.getFootFrame().isChildOfWorld())
      {
         if (poseGizmo.getPoseGizmo().getGizmoFrame() != state.getFootFrame().getReferenceFrame())
         {
            poseGizmo.getPoseGizmo().setGizmoFrame(state.getFootFrame().getReferenceFrame());
            graphicFrame.setParentFrame(state.getFootFrame().getReferenceFrame());
            collisionShapeFrame.setParentFrame(state.getFootFrame().getReferenceFrame());
         }

         RDXCRDTTools.syncGizmoWithBidirectionalField(poseGizmo.getPoseGizmo(), definition.getFootToParentTransform(), definition);

         highlightModels.get(definition.getSide()).setPose(graphicFrame.getReferenceFrame());
         if (poseGizmo.isSelected() || isMouseHovering)
         {
            highlightModels.get(definition.getSide()).setTransparency(0.7);
         }
         else
         {
            highlightModels.get(definition.getSide()).setTransparency(0.5);
         }
      }
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      ImGui.checkbox(labels.get("Adjust Goal Pose"), poseGizmo.getSelected());
      parentFrameComboBox.render();
      ImGui.pushItemWidth(80.0f);
      trajectoryDurationWidget.renderImGuiWidget();
      ImGui.popItemWidth();
   }

   public void render3DPanelImGuiOverlays()
   {
      if (isMouseHovering)
      {
         tooltip.render("%s Action\nIndex: %d\nName: %s".formatted(getLeafTypeTitle(), state.getLeafIndex(), definition.getName()));
      }
   }

   @Override
   public void deselectGizmos()
   {
      poseGizmo.setSelected(false);
   }

   @Override
   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (getSelected() && state.getFootFrame().isChildOfWorld())
      {
         poseGizmo.calculate3DViewPick(input);

         pickResult.reset();
         for (MouseCollidable mouseCollidable : mouseCollidables)
         {
            double collision = mouseCollidable.collide(input.getPickRayInWorld(), collisionShapeFrame.getReferenceFrame());
            if (!Double.isNaN(collision))
               pickResult.addPickCollision(collision);
         }
         if (pickResult.getPickCollisionWasAddedSinceReset())
            input.addPickResult(pickResult);
      }
   }

   @Override
   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (getSelected() && state.getFootFrame().isChildOfWorld())
      {
         isMouseHovering = input.getClosestPick() == pickResult;

         boolean isClickedOn = isMouseHovering && input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left);
         if (isClickedOn)
            poseGizmo.setSelected(true);

         poseGizmo.process3DViewInput(input, isMouseHovering);

         tooltip.setInput(input);
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if ((state.getIsNextForExecution() || getSelected() || state.getIsExecuting()) && state.getFootFrame().isChildOfWorld())
      {
         highlightModels.get(definition.getSide()).getRenderables(renderables, pool);
         poseGizmo.getVirtualRenderables(renderables, pool);
      }
   }

   public ReferenceFrame getReferenceFrame()
   {
      return poseGizmo.getPoseGizmo().getGizmoFrame();
   }

   @Override
   public String getLeafTypeTitle()
   {
      return definition.getSide().getPascalCaseName() + " Leg";
   }
}
