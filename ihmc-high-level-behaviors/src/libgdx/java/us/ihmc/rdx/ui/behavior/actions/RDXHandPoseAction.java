package us.ihmc.rdx.ui.behavior.actions;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.sequence.actions.HandPoseActionDefinition;
import us.ihmc.behaviors.sequence.actions.HandPoseActionState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiReferenceFrameLibraryCombo;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.input.ImGui3DViewPickResult;
import us.ihmc.rdx.simulation.scs2.RDXFrameNodePart;
import us.ihmc.rdx.simulation.scs2.RDXMultiBodySystemFactories;
import us.ihmc.rdx.simulation.scs2.RDXRigidBody;
import us.ihmc.rdx.simulation.scs2.RDXVisualTools;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDX3DPanelTooltip;
import us.ihmc.rdx.ui.affordances.RDXInteractableHighlightModel;
import us.ihmc.rdx.ui.affordances.RDXInteractableTools;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionNode;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.rdx.ui.teleoperation.RDXIKSolverColors;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.MultiBodySystemMissingTools;
import us.ihmc.robotics.interaction.MouseCollidable;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.simulationToolkit.RobotDefinitionTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.wholeBodyController.HandTransformTools;

import java.util.ArrayList;
import java.util.List;

public class RDXHandPoseAction extends RDXActionNode<HandPoseActionState, HandPoseActionDefinition>
{
   private final HandPoseActionState state;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   /** Gizmo is control frame */
   private final RDXSelectablePose3DGizmo poseGizmo;
   private final SideDependentList<String> handNames = new SideDependentList<>();
   private final MutableReferenceFrame graphicFrame = new MutableReferenceFrame();
   private final MutableReferenceFrame collisionShapeFrame = new MutableReferenceFrame();
   private boolean isMouseHovering = false;
   private final ImGui3DViewPickResult pickResult = new ImGui3DViewPickResult();
   private final ArrayList<MouseCollidable> mouseCollidables = new ArrayList<>();
   private final SideDependentList<RDXInteractableHighlightModel> highlightModels = new SideDependentList<>();
   private final ImGuiReferenceFrameLibraryCombo parentFrameComboBox;
   private final ImDoubleWrapper trajectoryDurationWidget;
   private final ImBooleanWrapper executeWithNextActionWrapper;
   private final ImBooleanWrapper holdPoseInWorldLaterWrapper;
   private final ImBooleanWrapper jointSpaceControlWrapper;
   private final SideDependentList<RDXRigidBody> armMultiBodyGraphics = new SideDependentList<>();
   private final SideDependentList<OneDoFJointBasics[]> armGraphicOneDoFJoints = new SideDependentList<>();
   private final SideDependentList<Color> currentColor = new SideDependentList<>();
   private final RDX3DPanelTooltip tooltip;

   public RDXHandPoseAction(long id,
                            CRDTInfo crdtInfo,
                            WorkspaceResourceDirectory saveFileDirectory,
                            RDX3DPanel panel3D,
                            DRCRobotModel robotModel,
                            FullHumanoidRobotModel syncedFullRobotModel,
                            RobotCollisionModel selectionCollisionModel,
                            ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(new HandPoseActionState(id, crdtInfo, saveFileDirectory, referenceFrameLibrary));

      state = getState();

      getDefinition().setDescription("Hand pose");

      poseGizmo = new RDXSelectablePose3DGizmo(ReferenceFrame.getWorldFrame(), getDefinition().getPalmTransformToParent().getValue(), getSelected());
      poseGizmo.create(panel3D);

      trajectoryDurationWidget = new ImDoubleWrapper(getDefinition()::getTrajectoryDuration,
                                                     getDefinition()::setTrajectoryDuration,
                                                     imDouble -> ImGui.inputDouble(labels.get("Trajectory duration"), imDouble));
      executeWithNextActionWrapper = new ImBooleanWrapper(getDefinition()::getExecuteWithNextAction,
                                                          getDefinition()::setExecuteWithNextAction,
                                                          imBoolean -> ImGui.checkbox(labels.get("Execute with next action"), imBoolean));
      holdPoseInWorldLaterWrapper = new ImBooleanWrapper(getDefinition()::getHoldPoseInWorldLater,
                                                         getDefinition()::setHoldPoseInWorldLater,
                                                         imBoolean -> ImGui.checkbox(labels.get("Hold pose in world later"), imBoolean));
      jointSpaceControlWrapper = new ImBooleanWrapper(getDefinition()::getJointSpaceControl,
                                                      getDefinition()::setJointSpaceControl,
                                                      imBoolean -> {
                                                         if (ImGui.radioButton(labels.get("Joint space"), imBoolean.get()))
                                                            imBoolean.set(true);
                                                         ImGui.sameLine();
                                                         if (ImGui.radioButton(labels.get("Task space"), !imBoolean.get()))
                                                            imBoolean.set(false);
                                                      });

      for (RobotSide side : RobotSide.values)
      {
         handNames.put(side, syncedFullRobotModel.getHand(side).getName());

         RigidBodyTransformReadOnly graphicToControlFrameTransform = HandTransformTools.getHandGraphicToControlFrameTransform(syncedFullRobotModel,
                                                                                                                              robotModel.getUIParameters(),
                                                                                                                              side);
         graphicFrame.update(transformToParent -> transformToParent.set(graphicToControlFrameTransform));

         String handBodyName = handNames.get(side);
         String modelFileName = RDXInteractableTools.getModelFileName(robotModel.getRobotDefinition().getRigidBodyDefinition(handBodyName));
         highlightModels.put(side, new RDXInteractableHighlightModel(modelFileName));

         MultiBodySystemBasics handOnlySystem = MultiBodySystemMissingTools.createSingleBodySystem(syncedFullRobotModel.getHand(side));
         List<Collidable> handCollidables = selectionCollisionModel.getRobotCollidables(handOnlySystem);

         RigidBodyTransformReadOnly linkToControlFrameTransform = HandTransformTools.getHandLinkToControlFrameTransform(syncedFullRobotModel, side);
         collisionShapeFrame.update(transformToParent -> transformToParent.set(linkToControlFrameTransform));

         for (Collidable handCollidable : handCollidables)
         {
            mouseCollidables.add(new MouseCollidable(handCollidable));
         }

         HumanoidJointNameMap jointMap = robotModel.getJointMap();
         ArmJointName firstArmJointName = jointMap.getArmJointNames()[0];
         RobotDefinition armDefinition = RobotDefinitionTools.cloneLimbOnlyDefinitionWithElevator(robotModel.getRobotDefinition(),
                                                                                                  jointMap.getChestName(),
                                                                                                  jointMap.getArmJointName(side, firstArmJointName));
         MaterialDefinition material = new MaterialDefinition(RDXIKSolverColors.GOOD_QUALITY_COLOR_DEFINITION);
         currentColor.put(side, RDXIKSolverColors.GOOD_QUALITY_COLOR);
         RobotDefinition.forEachRigidBodyDefinition(armDefinition.getRootBodyDefinition(), body ->
         {
            body.getVisualDefinitions().forEach(visual -> visual.setMaterialDefinition(material));
         });
         RigidBodyBasics armOnlyMultiBody
               = MultiBodySystemMissingTools.getDetachedCopyOfSubtreeWithElevator(syncedFullRobotModel.getChest(),
                                                                                  syncedFullRobotModel.getArmJoint(side, firstArmJointName),
                                                                                  syncedFullRobotModel.getHand(side).getName());
         armMultiBodyGraphics.put(side,
                                  RDXMultiBodySystemFactories.toRDXMultiBodySystem(armOnlyMultiBody, armDefinition, RDXVisualTools.DESIRED_ROBOT_SCALING));
         armMultiBodyGraphics.get(side).getRigidBodiesToHide().add("elevator");
         armMultiBodyGraphics.get(side).getRigidBodiesToHide().add(jointMap.getChestName());
         armGraphicOneDoFJoints.put(side, MultiBodySystemMissingTools.getSubtreeJointArray(OneDoFJointBasics.class, armMultiBodyGraphics.get(side)));
      }

      parentFrameComboBox = new ImGuiReferenceFrameLibraryCombo("Parent frame",
                                                                referenceFrameLibrary,
                                                                getDefinition()::getPalmParentFrameName,
                                                                getState().getPalmFrame()::changeFrame);

      tooltip = new RDX3DPanelTooltip(panel3D);
      panel3D.addImGuiOverlayAddition(this::render3DPanelImGuiOverlays);
   }

   @Override
   public void update()
   {
      super.update();

      if (state.getPalmFrame().isChildOfWorld())
      {
         if (poseGizmo.getPoseGizmo().getGizmoFrame() != state.getPalmFrame().getReferenceFrame())
         {
            poseGizmo.getPoseGizmo().setGizmoFrame(state.getPalmFrame().getReferenceFrame());
            graphicFrame.setParentFrame(state.getPalmFrame().getReferenceFrame());
            collisionShapeFrame.setParentFrame(state.getPalmFrame().getReferenceFrame());
         }

         poseGizmo.getPoseGizmo().update();
         highlightModels.get(getDefinition().getSide()).setPose(graphicFrame.getReferenceFrame());

         if (poseGizmo.isSelected() || isMouseHovering)
         {
            highlightModels.get(getDefinition().getSide()).setTransparency(0.7);
         }
         else
         {
            highlightModels.get(getDefinition().getSide()).setTransparency(0.5);
         }

         // IK solution visualization via ghost arms
         if (state.getIsNextForExecution())
            visualizeIK();
      }
   }

   private void visualizeIK()
   {
      SixDoFJoint floatingJoint = (SixDoFJoint) armMultiBodyGraphics.get(getDefinition().getSide()).getRigidBody().getChildrenJoints().get(0);
      floatingJoint.getJointPose().set(state.getGoalChestToWorldTransform().getValueReadOnly());

      for (int i = 0; i < armGraphicOneDoFJoints.get(getDefinition().getSide()).length; i++)
      {
         armGraphicOneDoFJoints.get(getDefinition().getSide())[i].setQ(state.getJointAngles().getValueReadOnly(i));
      }
      armMultiBodyGraphics.get(getDefinition().getSide()).updateFramesRecursively();
      armMultiBodyGraphics.get(getDefinition().getSide()).updateSubtreeGraphics();

      // We probably don't want to recolor the mesh every tick.
      Color color = RDXIKSolverColors.getColor(state.getSolutionQuality());
      if (color != currentColor.get(getDefinition().getSide()))
      {
         currentColor.put(getDefinition().getSide(), color);
         for (RigidBodyBasics body : armMultiBodyGraphics.get(getDefinition().getSide()).subtreeIterable())
         {
            if (body instanceof RDXRigidBody rdxRigidBody)
            {
               if (rdxRigidBody.getVisualGraphicsNode() != null)
               {
                  for (RDXFrameNodePart part : rdxRigidBody.getVisualGraphicsNode().getParts())
                  {
                     part.getModelInstance().setDiffuseColor(color);
                  }
               }
            }
         }
      }
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      ImGui.sameLine();
      executeWithNextActionWrapper.renderImGuiWidget();
      jointSpaceControlWrapper.renderImGuiWidget();
      if (!getDefinition().getJointSpaceControl())
      {
         holdPoseInWorldLaterWrapper.renderImGuiWidget();
      }
      parentFrameComboBox.render();
      ImGui.pushItemWidth(80.0f);
      trajectoryDurationWidget.renderImGuiWidget();
      ImGui.popItemWidth();
   }

   public void render3DPanelImGuiOverlays()
   {
      if (isMouseHovering)
      {
         tooltip.render("%s Action\nIndex: %d\nDescription: %s".formatted(getActionTypeTitle(),
                                                                          state.getActionIndex(),
                                                                          getDefinition().getDescription()));
      }
   }

   @Override
   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (state.getPalmFrame().isChildOfWorld())
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
      if (state.getPalmFrame().isChildOfWorld())
      {
         isMouseHovering = input.getClosestPick() == pickResult;

         boolean isClickedOn = isMouseHovering && input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left);
         if (isClickedOn)
         {
            getSelected().set(true);
         }

         poseGizmo.process3DViewInput(input, isMouseHovering);

         tooltip.setInput(input);
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (state.getPalmFrame().isChildOfWorld())
      {
         highlightModels.get(getDefinition().getSide()).getRenderables(renderables, pool);
         poseGizmo.getVirtualRenderables(renderables, pool);

         if (state.getIsNextForExecution())
            armMultiBodyGraphics.get(getDefinition().getSide()).getVisualRenderables(renderables, pool);
      }
   }

   @Override
   public String getActionTypeTitle()
   {
      return getDefinition().getSide().getPascalCaseName() + " Hand Pose";
   }

   public ReferenceFrame getReferenceFrame()
   {
      return poseGizmo.getPoseGizmo().getGizmoFrame();
   }
}
