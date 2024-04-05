package us.ihmc.rdx.ui.behavior.actions;

import imgui.ImGui;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.sequence.actions.KickDoorActionDefinition;
import us.ihmc.behaviors.sequence.actions.KickDoorActionState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.ImStringWrapper;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionNode;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RDXKickDoorAction extends RDXActionNode<KickDoorActionState, KickDoorActionDefinition>
{
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final KickDoorActionState state;
   private final KickDoorActionDefinition definition;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDoubleWrapper kickHeight;
   private final ImDoubleWrapper kickImpulse;
   private final ImDoubleWrapper kickDistance;
   private final ImDoubleWrapper preKickWeightDistribution;

   public RDXKickDoorAction(long id,
                            CRDTInfo crdtInfo,
                            WorkspaceResourceDirectory saveFileDirectory,
                            RDXBaseUI baseUI,
                            DRCRobotModel robotModel,
                            ROS2SyncedRobotModel syncedRobot,
                            ReferenceFrameLibrary referenceFrameLibrary,
                            FootstepPlannerParametersBasics footstepPlannerParameters)
   {
      super(new KickDoorActionState(id, crdtInfo, saveFileDirectory, referenceFrameLibrary));

      state = getState();
      definition = getDefinition();

      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;

      definition.setName("Kick Door");

      //TODO: add robot side to the gui
      kickHeight = new ImDoubleWrapper(definition::getKickHeight,
                                       definition::setKickHeight,
                                       imDouble -> ImGui.inputDouble(labels.get("Kick height"), imDouble));
      kickImpulse = new ImDoubleWrapper(definition::getKickImpulse,
                                        definition::setKickImpulse,
                                        imDouble -> ImGui.inputDouble(labels.get("Kick impulse"), imDouble));
      kickDistance = new ImDoubleWrapper(definition::getKickTargetDistance,
                                         definition::setKickTargetDistance,
                                         imDouble -> ImGui.inputDouble(labels.get("Kick distance"), imDouble));
      preKickWeightDistribution = new ImDoubleWrapper(definition::getPrekickWeightDistribution,
                                                      definition::setPrekickWeightDistribution,
                                                      imDouble -> ImGui.inputDouble(labels.get("Weight distribution"), imDouble));
   }

   @Override
   public void update()
   {
      super.update();
   }

   @Override
   public void renderTreeViewIconArea()
   {
      super.renderTreeViewIconArea();

      ImGui.sameLine();
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      ImGui.pushItemWidth(80.0f);
      kickHeight.renderImGuiWidget();
      kickImpulse.renderImGuiWidget();
      kickDistance.renderImGuiWidget();
      preKickWeightDistribution.renderImGuiWidget();
      ImGui.popItemWidth();
   }

   @Override
   public String getActionTypeTitle()
   {
      return "Kick Door";
   }
}
