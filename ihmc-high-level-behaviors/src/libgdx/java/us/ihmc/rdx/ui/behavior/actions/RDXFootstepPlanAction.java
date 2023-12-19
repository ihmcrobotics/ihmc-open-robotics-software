package us.ihmc.rdx.ui.behavior.actions;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.sequence.actions.FootstepPlanActionDefinition;
import us.ihmc.behaviors.sequence.actions.FootstepPlanActionFootstepState;
import us.ihmc.behaviors.sequence.actions.FootstepPlanActionState;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiReferenceFrameLibraryCombo;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionNode;
import us.ihmc.robotics.lists.RecyclingArrayListTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RDXFootstepPlanAction extends RDXActionNode<FootstepPlanActionState, FootstepPlanActionDefinition>
{
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private final FootstepPlanActionState state;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiReferenceFrameLibraryCombo parentFrameComboBox;
   private final ImDoubleWrapper swingDurationWidget;
   private final ImDoubleWrapper transferDurationWidget;
   private int numberOfAllocatedFootsteps = 0;
   private final RecyclingArrayList<RDXFootstepPlanActionFootstep> footsteps;
   private final TypedNotification<RobotSide> userAddedFootstep = new TypedNotification<>();
   private final Notification userRemovedFootstep = new Notification();
   private boolean frameIsChildOfWorld = false;

   public RDXFootstepPlanAction(long id,
                                CRDTInfo crdtInfo,
                                WorkspaceResourceDirectory saveFileDirectory,
                                RDXBaseUI baseUI,
                                DRCRobotModel robotModel,
                                ROS2SyncedRobotModel syncedRobot,
                                ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(new FootstepPlanActionState(id, crdtInfo, saveFileDirectory, referenceFrameLibrary));

      state = getState();

      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;
      this.referenceFrameLibrary = referenceFrameLibrary;

      getDefinition().setDescription("Footstep plan");

      parentFrameComboBox = new ImGuiReferenceFrameLibraryCombo("Parent frame",
                                                                referenceFrameLibrary,
                                                                getDefinition()::getParentFrameName,
                                                                this::changeParentFrame);
      swingDurationWidget = new ImDoubleWrapper(getDefinition()::getSwingDuration,
                                                getDefinition()::setSwingDuration,
                                                imDouble -> ImGui.inputDouble(labels.get("Swing duration"), imDouble));
      transferDurationWidget = new ImDoubleWrapper(getDefinition()::getTransferDuration,
                                                   getDefinition()::setTransferDuration,
                                                   imDouble -> ImGui.inputDouble(labels.get("Transfer duration"), imDouble));

      footsteps = new RecyclingArrayList<>(() ->
         new RDXFootstepPlanActionFootstep(baseUI,
                                           robotModel,
                                           this,
                                           RecyclingArrayListTools.getUnsafe(state.getFootsteps(), numberOfAllocatedFootsteps++)));
   }

   @Override
   public void update()
   {
      super.update();

      RecyclingArrayListTools.synchronizeSize(footsteps, state.getFootsteps());

      frameIsChildOfWorld = referenceFrameLibrary.containsFrame(getDefinition().getParentFrameName());
      if (frameIsChildOfWorld)
      {
         // Add a footstep to the action data only
         if (userAddedFootstep.poll())
         {
            RobotSide newSide = userAddedFootstep.read();
            RecyclingArrayListTools.addToAll(getDefinition().getFootsteps().getValue(), state.getFootsteps());
            RDXFootstepPlanActionFootstep addedFootstep = footsteps.add();
            addedFootstep.getDefinition().setSide(newSide);
            addedFootstep.getState().update();
            FramePose3D newFootstepPose = new FramePose3D();
            if (footsteps.size() > 1)
            {
               RDXFootstepPlanActionFootstep previousFootstep = footsteps.get(footsteps.size() - 2);
               newFootstepPose.setToZero(previousFootstep.getState().getSoleFrame().getReferenceFrame());

               if (previousFootstep.getDefinition().getSide() != newSide)
               {
                  double minStepWidth = robotModel.getWalkingControllerParameters().getSteppingParameters().getInPlaceWidth();
                  newFootstepPose.getPosition().addY(newSide.negateIfRightSide(minStepWidth));
               }
            }
            else
            {
               newFootstepPose.setToZero(syncedRobot.getReferenceFrames().getSoleFrame(newSide));
            }

            double aLittleInFront = 0.15;
            newFootstepPose.getPosition().addX(aLittleInFront);

            newFootstepPose.changeFrame(addedFootstep.getState().getSoleFrame().getReferenceFrame().getParent());
            addedFootstep.getDefinition().getSoleToPlanFrameTransform().getValue().set(newFootstepPose);
         }

         if (userRemovedFootstep.poll())
         {
            RecyclingArrayListTools.removeLast(footsteps);
            RecyclingArrayListTools.removeLast(state.getFootsteps());
            RecyclingArrayListTools.removeLast(getDefinition().getFootsteps().getValue());
         }

         for (RDXFootstepPlanActionFootstep footstep : footsteps)
         {
            footstep.update();
         }
      }
   }

   @Override
   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (frameIsChildOfWorld)
      {
         for (RDXFootstepPlanActionFootstep footstep : footsteps)
         {
            footstep.calculate3DViewPick(input);
         }
      }
   }

   @Override
   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (frameIsChildOfWorld)
      {
         for (RDXFootstepPlanActionFootstep footstep : footsteps)
         {
            footstep.process3DViewInput(input);
         }
      }
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      parentFrameComboBox.render();

      ImGui.pushItemWidth(80.0f);
      swingDurationWidget.renderImGuiWidget();
      transferDurationWidget.renderImGuiWidget();
      ImGui.popItemWidth();

      ImGui.text("Number of footsteps: %d".formatted(footsteps.size()));

      if (frameIsChildOfWorld) // Not allowing modification if not renderable
      {
         ImGui.text("Add:");
         for (RobotSide side : RobotSide.values)
         {
            ImGui.sameLine();
            if (ImGui.button(labels.get(side.getPascalCaseName())))
               userAddedFootstep.set(side);
         }
         if (!getState().getFootsteps().isEmpty())
         {
            ImGui.sameLine();
            ImGui.text("Remove:");
            ImGui.sameLine();
            if (ImGui.button(labels.get("Last")))
            {
               userRemovedFootstep.set();
            }
         }
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (frameIsChildOfWorld)
      {
         for (RDXFootstepPlanActionFootstep footstep : footsteps)
         {
            footstep.getVirtualRenderables(renderables, pool);
         }
      }
   }

   @Override
   public String getActionTypeTitle()
   {
      return "Footstep Plan";
   }

   public void changeParentFrame(String newParentFrameName)
   {
      getDefinition().setParentFrameName(newParentFrameName);
      for (FootstepPlanActionFootstepState footstepState : getState().getFootsteps())
      {
         footstepState.getSoleFrame().changeFrame(newParentFrameName);
      }
   }
}
