package us.ihmc.rdx.ui.behavior.actions;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.sequence.actions.FootPoseActionDefinition;
import us.ihmc.behaviors.sequence.actions.FootPoseActionState;
import us.ihmc.behaviors.sequence.actions.HandPoseActionDefinition;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.rdx.imgui.*;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.input.ImGui3DViewPickResult;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDX3DPanelTooltip;
import us.ihmc.rdx.ui.affordances.RDXInteractableHighlightModel;
import us.ihmc.rdx.ui.affordances.RDXInteractableTools;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionNode;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.MultiBodySystemMissingTools;
import us.ihmc.robotics.interaction.MouseCollidable;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.util.ArrayList;
import java.util.List;

public class RDXFootPoseAction extends RDXActionNode<FootPoseActionState, FootPoseActionDefinition>
{
   private final FootPoseActionState state;
   private final FootPoseActionDefinition definition;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean adjustGoalPose = new ImBoolean();
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

   public RDXFootPoseAction(long id,
                            CRDTInfo crdtInfo,
                            WorkspaceResourceDirectory saveFileDirectory,
                            RDX3DPanel panel3D,
                            DRCRobotModel robotModel,
                            FullHumanoidRobotModel syncedFullRobotModel,
                            RobotCollisionModel selectionCollisionModel,
                            ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(new FootPoseActionState(id, crdtInfo,saveFileDirectory, referenceFrameLibrary));

      state = getState();
      definition = getDefinition();

      getDefinition().setName("Foot pose");

      poseGizmo = new RDXSelectablePose3DGizmo(ReferenceFrame.getWorldFrame(), getDefinition().getFootToParentTransform().getValue(), adjustGoalPose);
      poseGizmo.create(panel3D);

      parentFrameComboBox = new ImGuiReferenceFrameLibraryCombo("Parent frame",
                                                                referenceFrameLibrary,
                                                                getDefinition()::getParentFrameName,
                                                                getState().getFootFrame()::changeFrame);
      trajectoryDurationWidget = new ImDoubleWrapper(getDefinition()::getTrajectoryDuration,
                                                     getDefinition()::setTrajectoryDuration,
                                                     imDouble -> ImGuiTools.volatileInputDouble(labels.get("Trajectory duration"), imDouble));

      for (RobotSide side : RobotSide.values)
      {
         String footBodyName = syncedFullRobotModel.getFoot(side).getName();
         String modelFileName = RDXInteractableTools.getModelFileName(robotModel.getRobotDefinition().getRigidBodyDefinition(footBodyName));
         highlightModels.put(side, new RDXInteractableHighlightModel(modelFileName));

         MultiBodySystemBasics footOnlySystem = MultiBodySystemMissingTools.createSingleBodySystem(syncedFullRobotModel.getFoot(side));
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

         poseGizmo.getPoseGizmo().update();
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
      ImGui.checkbox(labels.get("Adjust Goal Pose"), adjustGoalPose);
      parentFrameComboBox.render();
      ImGui.pushItemWidth(80.0f);
      trajectoryDurationWidget.renderImGuiWidget();
      ImGui.popItemWidth();
   }

   public void render3DPanelImGuiOverlays()
   {
      if (isMouseHovering)
      {
         tooltip.render("%s Action\nIndex: %d\nName: %s".formatted(getActionTypeTitle(), state.getActionIndex(), getDefinition().getName()));
      }
   }

   @Override
   public void calculate3DViewPick(ImGui3DViewInput input)
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

   @Override
   public void process3DViewInput(ImGui3DViewInput input)
   {
      isMouseHovering = input.getClosestPick() == pickResult;

      boolean isClickedOn = isMouseHovering && input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left);
      if (isClickedOn)
      {
         adjustGoalPose.set(true);
      }

      poseGizmo.process3DViewInput(input, isMouseHovering);

      tooltip.setInput(input);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (state.getFootFrame().isChildOfWorld())
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
   public String getActionTypeTitle()
   {
      return definition.getSide().getPascalCaseName() + " Foot Pose";
   }
}
