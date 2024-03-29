package us.ihmc.rdx.ui.behavior.actions;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.sequence.actions.WholeBodyBimanipulationActionDefinition;
import us.ihmc.behaviors.sequence.actions.WholeBodyBimanipulationActionState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.rdx.imgui.ImGuiReferenceFrameLibraryCombo;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDX3DPanel;
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

   private final MutableReferenceFrame collisionShapeFrame = new MutableReferenceFrame();
   private final ArrayList<MouseCollidable> mouseCollidables = new ArrayList<>();
   private final RDXDesiredRobot desiredRobot;
   private final OneDoFJointBasics[] desiredOneDoFJointsExcludingHands;
   private final ImGuiReferenceFrameLibraryCombo parentFrameComboBox;

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

      definition.setName("Wholebody Bimanipulation");

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
                                                                definition::getParentFrameName,
                                                                getState().getHandFrame(RobotSide.LEFT)::changeFrame);

      definition.getHandToParentTransform(RobotSide.LEFT).getValue().set(new RigidBodyTransform(new Quaternion(Math.toRadians(-30.0), Math.toRadians(-25.0), 0.0), new Point3D(0.0, 0.127, 0.0)));
      definition.getHandToParentTransform(RobotSide.RIGHT).getValue().set(new RigidBodyTransform(new Quaternion(Math.toRadians(30.0), Math.toRadians(-25.0), 0.0), new Point3D(0.0, -0.127, 0.0)));
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
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (state.getHandFrame(RobotSide.LEFT).isChildOfWorld())
      {
         for (RobotSide side : RobotSide.values)
         {
            poseGizmos.get(side).getVirtualRenderables(renderables, pool);
         }

         if (state.getIsNextForExecution())
            desiredRobot.getRenderables(renderables, pool, Collections.singleton(RDXSceneLevel.VIRTUAL));
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
