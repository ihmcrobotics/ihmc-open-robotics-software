package us.ihmc.rdx.ui.behavior.editor.actions;

import behavior_msgs.msg.dds.HandPoseJointAnglesStatusMessage;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.sequence.BehaviorActionSequence;
import us.ihmc.behaviors.sequence.actions.HandPoseActionData;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ros2.ROS2ControllerPublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiReferenceFrameLibraryCombo;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.input.ImGui3DViewPickResult;
import us.ihmc.rdx.simulation.scs2.RDXMultiBodySystemFactories;
import us.ihmc.rdx.simulation.scs2.RDXRigidBody;
import us.ihmc.rdx.simulation.scs2.RDXVisualTools;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDX3DPanelTooltip;
import us.ihmc.rdx.ui.affordances.*;
import us.ihmc.rdx.ui.behavior.editor.RDXBehaviorAction;
import us.ihmc.rdx.ui.collidables.RDXRobotCollisionModel;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.MultiBodySystemMissingTools;
import us.ihmc.robotics.interaction.MouseCollidable;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.simulationToolkit.RobotDefinitionTools;
import us.ihmc.wholeBodyController.HandTransformTools;

import java.util.ArrayList;
import java.util.List;

public class RDXHandPoseAction extends RDXBehaviorAction
{
   public static final String GOOD_QUALITY_COLOR = "0x4B61D1";
   public static final String BAD_QUALITY_COLOR = "0xD14B4B";
   private final HandPoseActionData actionData = new HandPoseActionData();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   /** Gizmo is control frame */
   private final RDXSelectablePose3DGizmo poseGizmo = new RDXSelectablePose3DGizmo(actionData.getPalmFrame(), actionData.getTransformToParent());
   private final ImBooleanWrapper selectedWrapper = new ImBooleanWrapper(() -> poseGizmo.getSelected().get(),
                                                                         value -> poseGizmo.getSelected().set(value),
                                                                         imBoolean -> ImGui.checkbox(labels.get("Selected"), imBoolean));
   private final SideDependentList<String> handNames = new SideDependentList<>();
   private final ModifiableReferenceFrame graphicFrame = new ModifiableReferenceFrame(actionData.getPalmFrame());
   private final ModifiableReferenceFrame collisionShapeFrame = new ModifiableReferenceFrame(actionData.getPalmFrame());
   private final RigidBodyBasics syncedChest;
   private final Color goodQualityColor;
   private final Color badQualityColor;
   private boolean isMouseHovering = false;
   private final ImGui3DViewPickResult pickResult = new ImGui3DViewPickResult();
   private final ArrayList<MouseCollidable> mouseCollidables = new ArrayList<>();
   private final SideDependentList<RDXInteractableHighlightModel> highlightModels = new SideDependentList<>();
   private final ImGuiReferenceFrameLibraryCombo referenceFrameLibraryCombo;
   private final ImDoubleWrapper trajectoryDurationWidget = new ImDoubleWrapper(actionData::getTrajectoryDuration,
                                                                                actionData::setTrajectoryDuration,
                                                                                imDouble -> ImGui.inputDouble(labels.get("Trajectory duration"), imDouble));
   private final ImBooleanWrapper executeWithNextActionWrapper = new ImBooleanWrapper(actionData::getExecuteWithNextAction,
                                                                                      actionData::setExecuteWithNextAction,
                                                                         imBoolean -> ImGui.checkbox(labels.get("Execute With Next Action"), imBoolean));
   private final ImBooleanWrapper holdPoseInWorldLaterWrapper = new ImBooleanWrapper(actionData::getHoldPoseInWorldLater,
                                                                                     actionData::setHoldPoseInWorldLater,
                                                                                         imBoolean -> ImGui.checkbox(labels.get("Hold Pose In World Later"), imBoolean));
   private final SideDependentList<RDXRigidBody> armMultiBodyGraphics = new SideDependentList<>();
   private final SideDependentList<OneDoFJointBasics[]> armGraphicOneDoFJoints = new SideDependentList<>();
   private final SideDependentList<Color> currentColor = new SideDependentList<>();
   private boolean displayIKSolution = false;
   private final IHMCROS2Input<HandPoseJointAnglesStatusMessage> handJointAnglesStatusSubscription;
   private final RDX3DPanelTooltip tooltip;

   public RDXHandPoseAction(RDX3DPanel panel3D,
                            DRCRobotModel robotModel,
                            FullHumanoidRobotModel syncedFullRobotModel,
                            RobotCollisionModel selectionCollisionModel,
                            ReferenceFrameLibrary referenceFrameLibrary,
                            ROS2ControllerPublishSubscribeAPI ros2)
   {
      actionData.setReferenceFrameLibrary(referenceFrameLibrary);

      ColorDefinition goodQualityColorDefinition = ColorDefinitions.parse(GOOD_QUALITY_COLOR).derive(0.0, 1.0, 1.0, 0.5);
      goodQualityColor = RDXVisualTools.toColor(goodQualityColorDefinition);
      badQualityColor = RDXVisualTools.toColor(ColorDefinitions.parse(BAD_QUALITY_COLOR).derive(0.0, 1.0, 1.0, 0.5));

      RDXRobotCollisionModel robotCollisionModel = new RDXRobotCollisionModel(selectionCollisionModel);
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
         MaterialDefinition material = new MaterialDefinition(goodQualityColorDefinition);
         currentColor.put(side, goodQualityColor);
         RobotDefinition.forEachRigidBodyDefinition(armDefinition.getRootBodyDefinition(), body ->
         {
            body.getVisualDefinitions().forEach(visual -> visual.setMaterialDefinition(material));
         });
         RigidBodyBasics armOnlyMultiBody
               = MultiBodySystemMissingTools.getDetachedCopyOfSubtreeWithElevator(syncedFullRobotModel.getChest(),
                                                                                  syncedFullRobotModel.getArmJoint(side, firstArmJointName));
         armMultiBodyGraphics.put(side,
                                  RDXMultiBodySystemFactories.toRDXMultiBodySystem(armOnlyMultiBody, armDefinition, RDXVisualTools.DESIRED_ROBOT_SCALING));
         armMultiBodyGraphics.get(side).getRigidBodiesToHide().add("elevator");
         armMultiBodyGraphics.get(side).getRigidBodiesToHide().add(jointMap.getChestName());
         armGraphicOneDoFJoints.put(side, MultiBodySystemMissingTools.getSubtreeJointArray(OneDoFJointBasics.class, armMultiBodyGraphics.get(side)));
      }

      referenceFrameLibraryCombo = new ImGuiReferenceFrameLibraryCombo(referenceFrameLibrary);
      poseGizmo.create(panel3D);

      tooltip = new RDX3DPanelTooltip(panel3D);
      panel3D.addImGuiOverlayAddition(this::render3DPanelImGuiOverlays);

      handJointAnglesStatusSubscription = ros2.subscribe(BehaviorActionSequence.HAND_POSE_JOINT_ANGLES_STATUS);
      syncedChest = syncedFullRobotModel.getChest();
   }

   @Override
   public void updateAfterLoading()
   {
      referenceFrameLibraryCombo.setSelectedReferenceFrame(actionData.getParentFrame().getName());
   }

   public void setSide(RobotSide side)
   {
      actionData.setSide(side);
   }

   public void setIncludingFrame(ReferenceFrame parentFrame, RigidBodyTransform transformToParent)
   {
      actionData.changeParentFrame(parentFrame);
      actionData.setTransformToParent(transformToParentToPack -> transformToParentToPack.set(transformToParent));
      update();
   }

   public void setToReferenceFrame(ReferenceFrame referenceFrame)
   {
      actionData.changeParentFrame(ReferenceFrame.getWorldFrame());
      actionData.setTransformToParent(transformToParentToPack -> transformToParentToPack.set(referenceFrame.getTransformToWorldFrame()));
      update();
   }

   @Override
   public void update()
   {
      actionData.update();

      if (poseGizmo.getPoseGizmo().getGizmoFrame() != actionData.getPalmFrame())
      {
         poseGizmo.getPoseGizmo().setGizmoFrame(actionData.getPalmFrame());
         graphicFrame.changeParentFrame(actionData.getPalmFrame());
         collisionShapeFrame.changeParentFrame(actionData.getPalmFrame());
      }

      poseGizmo.getPoseGizmo().update();
      highlightModels.get(actionData.getSide()).setPose(graphicFrame.getReferenceFrame());

      if (poseGizmo.isSelected() || isMouseHovering)
      {
         highlightModels.get(actionData.getSide()).setTransparency(0.7);
      }
      else
      {
         highlightModels.get(actionData.getSide()).setTransparency(0.5);
      }

//      displayIKSolution = getActionIndex() == getActionNextExecutionIndex();
//      if (displayIKSolution && handJointAnglesStatusSubscription.hasReceivedFirstMessage())
//      {
//         HandPoseJointAnglesStatusMessage handPoseJointAnglesStatusMessage = handJointAnglesStatusSubscription.getLatest();
//         if (handPoseJointAnglesStatusMessage.getActionInformation().getActionIndex() == getActionIndex())
//         {
//            SixDoFJoint floatingJoint = (SixDoFJoint) armMultiBodyGraphics.get(getActionData().getSide()).getRigidBody().getChildrenJoints().get(0);
//            floatingJoint.getJointPose().set(syncedChest.getParentJoint().getFrameAfterJoint().getTransformToRoot());
//            for (int i = 0; i < handPoseJointAnglesStatusMessage.getJointAngles().length; i++)
//            {
//               armGraphicOneDoFJoints.get(getActionData().getSide())[i].setQ(handPoseJointAnglesStatusMessage.getJointAngles()[i]);
//            }
//            armMultiBodyGraphics.get(getActionData().getSide()).updateFramesRecursively();
//            armMultiBodyGraphics.get(getActionData().getSide()).updateSubtreeGraphics();
//         }
//
//         // We probably don't want to recolor the mesh every tick.
//         Color color = handPoseJointAnglesStatusMessage.getSolutionQuality() > 1.0 ? badQualityColor : goodQualityColor;
//         if (color != currentColor.get(getActionData().getSide()))
//         {
//            currentColor.put(getActionData().getSide(), color);
//            for (RigidBodyBasics body : armMultiBodyGraphics.get(getActionData().getSide()).subtreeIterable())
//            {
//               if (body instanceof RDXRigidBody rdxRigidBody)
//               {
//                  if (rdxRigidBody.getVisualGraphicsNode() != null)
//                  {
//                     for (RDXFrameNodePart part : rdxRigidBody.getVisualGraphicsNode().getParts())
//                     {
//                        part.getModelInstance().setDiffuseColor(color);
//                     }
//                  }
//               }
//            }
//         }
//      }
   }

   @Override
   public void renderImGuiSettingWidgets()
   {
      ImGui.sameLine();
      executeWithNextActionWrapper.renderImGuiWidget();
      holdPoseInWorldLaterWrapper.renderImGuiWidget();
      if (referenceFrameLibraryCombo.render())
      {
         actionData.changeParentFrameWithoutMoving(referenceFrameLibraryCombo.getSelectedReferenceFrame().get());
         update();
      }
      ImGui.pushItemWidth(80.0f);
      trajectoryDurationWidget.renderImGuiWidget();
      ImGui.popItemWidth();
   }

   public void render3DPanelImGuiOverlays()
   {
      if (isMouseHovering)
      {
         tooltip.render("%s Action\nIndex: %d\nDescription: %s".formatted(getActionTypeTitle(),
                                                                          getActionIndex(),
                                                                          actionData.getDescription()));
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
         selectedWrapper.set(true);
      }

      poseGizmo.process3DViewInput(input, isMouseHovering);

      tooltip.setInput(input);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      highlightModels.get(actionData.getSide()).getRenderables(renderables, pool);
      poseGizmo.getVirtualRenderables(renderables, pool);

//      if (displayIKSolution)
//         armMultiBodyGraphics.get(getActionData().getSide()).getVisualRenderables(renderables, pool);
   }

   @Override
   public String getActionTypeTitle()
   {
      return actionData.getSide().getPascalCaseName() + " Hand Pose";
   }

   public ReferenceFrame getReferenceFrame()
   {
      return poseGizmo.getPoseGizmo().getGizmoFrame();
   }

   @Override
   public ImBooleanWrapper getSelected()
   {
      return selectedWrapper;
   }

   @Override
   public HandPoseActionData getActionData()
   {
      return actionData;
   }
}
