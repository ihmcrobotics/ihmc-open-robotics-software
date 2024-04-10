package us.ihmc.rdx.ui.behavior.actions;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.sequence.actions.*;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiReferenceFrameLibraryCombo;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.mesh.RDXMutableArrowModel;
import us.ihmc.rdx.ui.RDX3DPanelTooltip;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionNode;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXFootstepGraphic;
import us.ihmc.rdx.ui.widgets.ImGuiFootstepsWidget;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.lists.RecyclingArrayListTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RDXKickDoorApproachPlanAction extends RDXActionNode<KickDoorApproachPlanActionState, KickDoorApproachPlanActionDefinition>
{
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final KickDoorApproachPlanActionState state;
   private final KickDoorApproachPlanActionDefinition definition;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean editManuallyPlacedSteps = new ImBoolean();
   private final ImDoubleWrapper swingDurationWidget;
   private final ImDoubleWrapper transferDurationWidget;
   private int numberOfAllocatedFootsteps = 0;
   private final SideDependentList<RDXFootstepGraphic> goalFeetGraphics = new SideDependentList<>();
   private final RDX3DPanelTooltip tooltip;
   private final ImGuiFootstepsWidget footstepsWidget = new ImGuiFootstepsWidget();

   public RDXKickDoorApproachPlanAction(long id,
                                        CRDTInfo crdtInfo,
                                        WorkspaceResourceDirectory saveFileDirectory,
                                        RDXBaseUI baseUI,
                                        DRCRobotModel robotModel,
                                        ROS2SyncedRobotModel syncedRobot,
                                        ReferenceFrameLibrary referenceFrameLibrary,
                                        FootstepPlannerParametersBasics footstepPlannerParameters)
   {
      super(new KickDoorApproachPlanActionState(id, crdtInfo, saveFileDirectory, referenceFrameLibrary));

      state = getState();
      definition = getDefinition();

      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;

      definition.setName("Footstep plan");


      swingDurationWidget = new ImDoubleWrapper(definition::getSwingDuration,
                                                definition::setSwingDuration,
                                                imDouble -> ImGui.inputDouble(labels.get("Swing duration"), imDouble));
      transferDurationWidget = new ImDoubleWrapper(definition::getTransferDuration,
                                                   definition::setTransferDuration,
                                                   imDouble -> ImGui.inputDouble(labels.get("Transfer duration"), imDouble));


      for (RobotSide side : RobotSide.values)
      {
         RDXFootstepGraphic goalFootGraphic = new RDXFootstepGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints(), side);
         goalFootGraphic.create();
         goalFeetGraphics.put(side, goalFootGraphic);
      }

      tooltip = new RDX3DPanelTooltip(baseUI.getPrimary3DPanel());
      baseUI.getPrimary3DPanel().addImGuiOverlayAddition(this::render3DPanelImGuiOverlays);
   }

   @Override
   public void update()
   {
      super.update();


      if (state.areFramesInWorld())
      {
         // Add a footstep to the action data only


         // Update arrow graphic geometry

         goalFeetGraphics.get(RobotSide.LEFT).setPose(state.getLeftFootGoalPose().getValueReadOnly());
         goalFeetGraphics.get(RobotSide.RIGHT).setPose(state.getRightFootGoalPose().getValueReadOnly());

      }
   }

   @Override
   public void renderTreeViewIconArea()
   {
      super.renderTreeViewIconArea();

      footstepsWidget.render(ImGui.getFrameHeight());
      ImGui.sameLine();
   }

   @Override
   public void calculateVRPick(RDXVRContext vrContext)
   {
   }

   @Override
   public void processVRInput(RDXVRContext vrContext)
   {
   }

   @Override
   public void calculate3DViewPick(ImGui3DViewInput input)
   {
   }

   @Override
   public void process3DViewInput(ImGui3DViewInput input)
   {

   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      ImGui.pushItemWidth(80.0f);
      swingDurationWidget.renderImGuiWidget();
      transferDurationWidget.renderImGuiWidget();
      ImGui.popItemWidth();
      ImGui.text("Execution mode:");
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Override"), definition.getExecutionMode().getValue() == ExecutionMode.OVERRIDE))
         definition.getExecutionMode().setValue(ExecutionMode.OVERRIDE);
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Queue"), definition.getExecutionMode().getValue() == ExecutionMode.QUEUE))
         definition.getExecutionMode().setValue(ExecutionMode.QUEUE);

   }

   @Override
   public void deselectGizmos()
   {
      editManuallyPlacedSteps.set(false);
   }

   public void render3DPanelImGuiOverlays()
   {

   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (state.areFramesInWorld())
      {


            for (RobotSide side : RobotSide.values)
            {
               goalFeetGraphics.get(side).setHighlighted(footstepsWidget.getIsHovered().get(side));
               goalFeetGraphics.get(side).getRenderables(renderables, pool);
            }
      }
   }

   @Override
   public String getActionTypeTitle()
   {
      return "Kick Door Approach Plan";
   }

   public ImBoolean getEditManuallyPlacedSteps()
   {
      return editManuallyPlacedSteps;
   }
}
