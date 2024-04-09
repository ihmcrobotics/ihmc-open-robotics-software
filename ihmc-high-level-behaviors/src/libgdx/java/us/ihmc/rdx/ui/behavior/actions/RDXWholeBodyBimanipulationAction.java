package us.ihmc.rdx.ui.behavior.actions;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.sequence.actions.WholeBodyBimanipulationActionDefinition;
import us.ihmc.behaviors.sequence.actions.WholeBodyBimanipulationActionState;
import us.ihmc.communication.crdt.CRDTDetachableReferenceFrame;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalRigidBodyTransform;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiReferenceFrameLibraryCombo;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.input.ImGui3DViewPickResult;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDX3DPanelTooltip;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionNode;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.rdx.ui.teleoperation.RDXDesiredRobot;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.MultiBodySystemMissingTools;
import us.ihmc.robotics.interaction.MouseCollidable;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.wholeBodyController.HandTransformTools;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class RDXWholeBodyBimanipulationAction extends RDXActionNode<WholeBodyBimanipulationActionState, WholeBodyBimanipulationActionDefinition>
{
   private final WholeBodyBimanipulationActionState state;
   private final WholeBodyBimanipulationActionDefinition definition;
   private final SideDependentList<RDXSelectablePose3DGizmo> poseGizmos = new SideDependentList<>();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ROS2SyncedRobotModel syncedRobot;

   private final MutableReferenceFrame collisionShapeFrame = new MutableReferenceFrame();
   private final ArrayList<MouseCollidable> mouseCollidables = new ArrayList<>();
   private final RDXDesiredRobot desiredRobot;
   private final OneDoFJointBasics[] desiredOneDoFJointsExcludingHands;
   private final ImGuiReferenceFrameLibraryCombo parentFrameComboBox;
   private final ImDoubleWrapper trajectoryDurationWidget;
   private final ReferenceFrameLibrary referenceFrameLibrary;

   public RDXWholeBodyBimanipulationAction(long id,
                                           CRDTInfo crdtInfo,
                                           WorkspaceResourceDirectory saveFileDirectory,
                                           RDX3DPanel panel3D,
                                           ReferenceFrameLibrary referenceFrameLibrary,
                                           DRCRobotModel robotModel,
                                           RobotCollisionModel selectionCollisionModel,
                                           ROS2SyncedRobotModel syncedRobot)
   {
      super(new WholeBodyBimanipulationActionState(id, crdtInfo, saveFileDirectory, referenceFrameLibrary));

      state = getState();
      definition = getDefinition();
      this.referenceFrameLibrary = referenceFrameLibrary;

      definition.setName("Wholebody Bimanipulation");

      trajectoryDurationWidget = new ImDoubleWrapper(definition::getTrajectoryDuration,
                                                     definition::setTrajectoryDuration,
                                                     imDouble -> ImGui.inputDouble(labels.get("Trajectory duration"), imDouble));

      desiredRobot = new RDXDesiredRobot(robotModel);
      desiredRobot.setSceneLevels(RDXSceneLevel.VIRTUAL);
      desiredRobot.create();
      desiredRobot.setActive(true);
      desiredOneDoFJointsExcludingHands = FullRobotModelUtils.getAllJointsExcludingHands(desiredRobot.getDesiredFullRobotModel());

      for (RobotSide side : RobotSide.values)
      {
         poseGizmos.put(side, new RDXSelectablePose3DGizmo(ReferenceFrame.getWorldFrame(), definition.getHandToParentTransform(side).getValue()));
         poseGizmos.get(side).create(panel3D);
      }

      this.syncedRobot = syncedRobot;
      FullHumanoidRobotModel syncedFullRobotModel = syncedRobot.getFullRobotModel();
      for (RobotSide side : RobotSide.values)
      {
         MultiBodySystemBasics handOnlySystem = MultiBodySystemMissingTools.createSingleBodySystem(syncedFullRobotModel.getHand(side));
         List<Collidable> handCollidables = selectionCollisionModel.getRobotCollidables(handOnlySystem);

         RigidBodyTransformReadOnly linkToControlFrameTransform = HandTransformTools.getHandLinkToControlFrameTransform(syncedFullRobotModel, side);
         collisionShapeFrame.update(transformToParent -> transformToParent.set(linkToControlFrameTransform));

         for (Collidable handCollidable : handCollidables)
         {
            mouseCollidables.add(new MouseCollidable(handCollidable));
         }
      }

      parentFrameComboBox = new ImGuiReferenceFrameLibraryCombo("Parent frame",
                                                                referenceFrameLibrary,
                                                                definition::getParentFrameName, state::changeParentFrame);

      definition.getHandToParentTransform(RobotSide.LEFT).getValue().set(new RigidBodyTransform(new Quaternion(Math.toRadians(-30.0), Math.toRadians(-25.0), 0.0), new Point3D(0.0, 0.127, 0.0)));
      definition.getHandToParentTransform(RobotSide.RIGHT).getValue().set(new RigidBodyTransform(new Quaternion(Math.toRadians(30.0), Math.toRadians(-25.0), 0.0), new Point3D(0.0, -0.127, 0.0)));
      definition.setTrajectoryDuration(4.0);

      tooltip = new RDX3DPanelTooltip(panel3D);
   }

   @Override
   public void update()
   {
      super.update();

      for (int i = 0; i < desiredOneDoFJointsExcludingHands.length; i++)
      {
         desiredOneDoFJointsExcludingHands[i].setQ(state.getJointAngle(i));
      }
      desiredRobot.update();

      if (state.getHandFrame(RobotSide.LEFT).isChildOfWorld())
      {
         for (RobotSide side : RobotSide.values)
         {
            if (poseGizmos.get(side).getPoseGizmo().getGizmoFrame() != state.getHandFrame(side).getReferenceFrame())
            {
               poseGizmos.get(side).getPoseGizmo().setGizmoFrame(state.getHandFrame(side).getReferenceFrame());
            }

            poseGizmos.get(side).getPoseGizmo().update();
         }
      }
   }

   @Override
   public void renderTreeViewIconArea()
   {
      super.renderTreeViewIconArea();
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      ImGui.checkbox(labels.get("Adjust " + RobotSide.LEFT.getPascalCaseName() + " Goal Pose"), poseGizmos.get(RobotSide.LEFT).getSelected());
      ImGui.sameLine();
      ImGui.checkbox(labels.get("Adjust " + RobotSide.RIGHT.getPascalCaseName() + " Goal Pose"), poseGizmos.get(RobotSide.RIGHT).getSelected());

      parentFrameComboBox.render();
      ImGui.pushItemWidth(80.0f);
      trajectoryDurationWidget.renderImGuiWidget();
      for (RobotSide side : RobotSide.values)
      {
         if (ImGui.button(labels.get("Set Pose to " + side.getPascalCaseName() + " Synced Hand")))
         {
            CRDTDetachableReferenceFrame actionPalmFrame = getState().getHandFrame(side);
            CRDTUnidirectionalRigidBodyTransform palmTransformToParent = definition.getHandToParentTransform(side);
            MovingReferenceFrame syncedPalmFrame = syncedRobot.getReferenceFrames().getHandFrame(side);
            FramePose3D syncedPalmPose = new FramePose3D();
            syncedPalmPose.setToZero(syncedPalmFrame);
            syncedPalmPose.changeFrame(actionPalmFrame.getReferenceFrame().getParent());
            palmTransformToParent.getValue().set(syncedPalmPose);
            actionPalmFrame.update();
         }
      }

      if (ImGui.button(labels.get("Force update Latest Standing Configuration")))
      {
         state.setForceLatestStandingRobotConfigurationUpdate(true);
      }

      for (RobotSide side : RobotSide.values)
      {
         if (ImGui.button(labels.get("Zero " + side.getPascalCaseName() + " Hand pose in Object frame")))
         {
            CRDTDetachableReferenceFrame actionPalmFrame = getState().getHandFrame(side);
            CRDTUnidirectionalRigidBodyTransform palmTransformToParent = definition.getHandToParentTransform(side);
            FramePose3D syncedPalmPose = new FramePose3D();
            syncedPalmPose.setToZero(actionPalmFrame.getReferenceFrame().getParent());
            palmTransformToParent.getValue().set(syncedPalmPose);
            actionPalmFrame.update();
         }
      }
   }

   @Override
   public void deselectGizmos()
   {
      super.deselectGizmos();
      for (RobotSide side : RobotSide.values)
      {
         poseGizmos.get(side).setSelected(false);
      }
   }

   @Override
   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      for (RobotSide side : RobotSide.values)
      {
         poseGizmos.get(side).calculate3DViewPick(input);
      }
   }

   private boolean isMouseHovering = false;
   private final ImGui3DViewPickResult pickResult = new ImGui3DViewPickResult();
   private final RDX3DPanelTooltip tooltip;

   @Override
   public void process3DViewInput(ImGui3DViewInput input)
   {
      for (RobotSide side : RobotSide.values)
      {
         if (state.getHandFrame(side).isChildOfWorld())
         {
            isMouseHovering = input.getClosestPick() == pickResult;

            boolean isClickedOn = isMouseHovering && input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left);
            if (isClickedOn)
            {
               poseGizmos.get(side).setSelected(true);
            }

            poseGizmos.get(side).process3DViewInput(input);

            tooltip.setInput(input);
         }
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (state.getHandFrame(RobotSide.LEFT).isChildOfWorld())
      {
         for (RobotSide side : RobotSide.values)
         {
            poseGizmos.get(side).getVirtualRenderables(renderables, pool);
         }

         if (state.getIsNextForExecution())
         {
            desiredRobot.setWholeBodyColor(Color.WHITE);
            for (RobotSide side : RobotSide.values)
               desiredRobot.setArmShowing(side, true);
            for (RobotSide side : RobotSide.values)
               desiredRobot.setLegShowing(side, true);
            desiredRobot.setChestShowing(true);
            desiredRobot.setPelvisShowing(true);

            desiredRobot.getRenderables(renderables, pool, Collections.singleton(RDXSceneLevel.VIRTUAL));
         }
      }
   }

   public ReferenceFrame getReferenceFrame(RobotSide side)
   {
      return poseGizmos.get(side).getPoseGizmo().getGizmoFrame();
   }

   @Override
   public String getActionTypeTitle()
   {
      return "Whole Body Bimanipulation";
   }
}
