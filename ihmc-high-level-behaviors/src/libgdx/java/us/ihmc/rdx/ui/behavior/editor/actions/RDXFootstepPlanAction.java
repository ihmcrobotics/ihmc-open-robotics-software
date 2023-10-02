package us.ihmc.rdx.ui.behavior.editor.actions;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImString;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.sequence.actions.FootstepActionDefinition;
import us.ihmc.behaviors.sequence.actions.FootstepPlanActionDefinition;
import us.ihmc.behaviors.sequence.actions.FootstepPlanActionState;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiReferenceFrameLibraryCombo;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.behavior.editor.RDXBehaviorAction;
import us.ihmc.rdx.ui.behavior.editor.RDXBehaviorActionBasics;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXFootstepPlanAction implements RDXBehaviorAction
{
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private final FootstepPlanActionState state = new FootstepPlanActionState();
   private final FootstepPlanActionDefinition definition = state.getDefinition();
   private final RDXBehaviorActionBasics rdxActionBasics = new RDXBehaviorActionBasics(this);
   private final ImGuiReferenceFrameLibraryCombo referenceFrameLibraryCombo;
   private final RecyclingArrayList<RDXFootstepAction> footsteps;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final TypedNotification<RobotSide> userAddedFootstep = new TypedNotification<>();
   private final Notification userRemovedFootstep = new Notification();
   private final ImDoubleWrapper swingDurationWidget = new ImDoubleWrapper(definition::getSwingDuration,
                                                                           definition::setSwingDuration,
                                                                           imDouble -> ImGui.inputDouble(labels.get("Swing duration"), imDouble));
   private final ImDoubleWrapper transferDurationWidget = new ImDoubleWrapper(definition::getTransferDuration,
                                                                              definition::setTransferDuration,
                                                                              imDouble -> ImGui.inputDouble(labels.get("Transfer duration"), imDouble));

   public RDXFootstepPlanAction(RDXBaseUI baseUI,
                                DRCRobotModel robotModel,
                                ROS2SyncedRobotModel syncedRobot,
                                ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;
      this.referenceFrameLibrary = referenceFrameLibrary;

      referenceFrameLibraryCombo = new ImGuiReferenceFrameLibraryCombo(referenceFrameLibrary);

      footsteps = new RecyclingArrayList<>(() -> new RDXFootstepAction(definition, baseUI, robotModel, getSelected()::get));
   }

   @Override
   public void updateAfterLoading()
   {
      referenceFrameLibraryCombo.setSelectedReferenceFrame(definition.getConditionalReferenceFrame());
   }

   public void setToReferenceFrame(ReferenceFrame referenceFrame)
   {
      definition.getConditionalReferenceFrame().setParentFrameName(ReferenceFrame.getWorldFrame().getName());
      definition.setTransformToParent(referenceFrame.getTransformToWorldFrame());
      update();
   }

   @Override
   public void update(boolean concurrentActionIsNextForExecution)
   {
      definition.update(referenceFrameLibrary);

      // Add a footstep to the action data only
      if (userAddedFootstep.poll())
      {
         RobotSide newSide = userAddedFootstep.read();
         FootstepActionDefinition addedFootstep = definition.getFootsteps().add();
         addedFootstep.setSide(newSide);
         FramePose3D newFootstepPose = new FramePose3D();
         if (!footsteps.isEmpty())
         {
            RDXFootstepAction previousFootstep = footsteps.get(footsteps.size() - 1);
            newFootstepPose.setToZero(previousFootstep.getFootstepFrame());

            if (previousFootstep.getSide() != newSide)
            {
               double minStepWidth = robotModel.getWalkingControllerParameters().getSteppingParameters().getMinStepWidth();
               newFootstepPose.getPosition().addY(newSide.negateIfRightSide(minStepWidth));
            }
         }
         else
         {
            newFootstepPose.setToZero(syncedRobot.getReferenceFrames().getSoleFrame(newSide));
         }

         double aLittleInFront = 0.15;
         newFootstepPose.getPosition().addX(aLittleInFront);

         newFootstepPose.changeFrame(definition.getConditionalReferenceFrame().get());
         addedFootstep.getSolePose().set(newFootstepPose);
      }

      if (userRemovedFootstep.poll())
         definition.getFootsteps().remove(definition.getFootsteps().size() - 1);

      // Synchronizes the RDX footsteps to the action data
      footsteps.clear();
      while (footsteps.size() < definition.getFootsteps().size())
         footsteps.add();
      for (int i = 0; i < definition.getFootsteps().size(); i++)
      {
         footsteps.get(i).update(definition.getFootsteps().get(i), i);
      }
   }

   @Override
   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      for (RDXFootstepAction footstep : footsteps)
      {
         footstep.calculate3DViewPick(input);
      }
   }

   @Override
   public void process3DViewInput(ImGui3DViewInput input)
   {
      for (RDXFootstepAction footstep : footsteps)
      {
         footstep.process3DViewInput(input);
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      rdxActionBasics.renderImGuiWidgets();
   }

   @Override
   public void renderImGuiSettingWidgets()
   {
      if (referenceFrameLibraryCombo.render())
      {
         definition.getConditionalReferenceFrame().setParentFrameName(referenceFrameLibraryCombo.getSelectedReferenceFrame().getParent().getName());
      }

      ImGui.pushItemWidth(80.0f);
      swingDurationWidget.renderImGuiWidget();
      transferDurationWidget.renderImGuiWidget();
      ImGui.popItemWidth();

      ImGui.text("Number of footsteps: %d".formatted(footsteps.size()));
      ImGui.text("Add:");
      for (RobotSide side : RobotSide.values)
      {
         ImGui.sameLine();
         if (ImGui.button(labels.get(side.getPascalCaseName())))
            userAddedFootstep.set(side);
      }
      ImGui.sameLine();
      ImGui.text("Remove:");
      ImGui.sameLine();
      if (ImGui.button(labels.get("Last")))
      {
         userRemovedFootstep.set();
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (RDXFootstepAction footstep : footsteps)
      {
         footstep.getVirtualRenderables(renderables, pool);
      }
   }

   @Override
   public String getActionTypeTitle()
   {
      return "Footstep Plan";
   }

   @Override
   public ImBooleanWrapper getSelected()
   {
      return rdxActionBasics.getSelected();
   }

   @Override
   public ImBoolean getExpanded()
   {
      return rdxActionBasics.getExpanded();
   }

   @Override
   public ImString getImDescription()
   {
      return rdxActionBasics.getDescription();
   }

   @Override
   public ImString getRejectionTooltip()
   {
      return rdxActionBasics.getRejectionTooltip();
   }

   @Override
   public int getActionIndex()
   {
      return rdxActionBasics.getActionIndex();
   }

   @Override
   public void setActionIndex(int actionIndex)
   {
      rdxActionBasics.setActionIndex(actionIndex);
   }

   @Override
   public int getActionNextExecutionIndex()
   {
      return rdxActionBasics.getActionNextExecutionIndex();
   }

   @Override
   public void setActionNextExecutionIndex(int actionNextExecutionIndex)
   {
      rdxActionBasics.setActionNextExecutionIndex(actionNextExecutionIndex);
   }

   @Override
   public FootstepPlanActionState getState()
   {
      return state;
   }

   public FootstepPlanActionDefinition getDefinition()
   {
      return definition;
   }
}
