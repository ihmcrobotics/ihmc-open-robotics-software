package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.Color;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.rdx.simulation.scs2.RDXFrameNodePart;
import us.ihmc.rdx.simulation.scs2.RDXMultiBodySystemFactories;
import us.ihmc.rdx.simulation.scs2.RDXRigidBody;
import us.ihmc.rdx.simulation.scs2.RDXVisualTools;
import us.ihmc.rdx.ui.teleoperation.RDXIKSolverColors;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.MultiBodySystemMissingTools;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.simulationToolkit.RobotDefinitionTools;

/**
 * A 3D arm graphic that can be posed and colored for visualizing arm configurations.
 */
public class RDXArmMultiBodyGraphic
{
   private final RDXRigidBody rootBody;
   private final OneDoFJointBasics[] joints;
   private final RigidBodyBasics hand;
   private final ReferenceFrame handControlFrame;
   private Color color;

   public RDXArmMultiBodyGraphic(DRCRobotModel robotModel, FullHumanoidRobotModel syncedFullRobotModel, RobotSide side)
   {
      HumanoidJointNameMap jointMap = robotModel.getJointMap();
      ArmJointName firstArmJointName = jointMap.getArmJointNames()[0];
      RobotDefinition armDefinition = RobotDefinitionTools.cloneLimbOnlyDefinitionWithElevator(robotModel.getRobotDefinition(),
                                                                                               jointMap.getChestName(),
                                                                                               jointMap.getArmJointName(side, firstArmJointName));
      MaterialDefinition material = new MaterialDefinition(RDXIKSolverColors.GOOD_QUALITY_COLOR_DEFINITION);
      color = RDXIKSolverColors.GOOD_QUALITY_COLOR;
      RobotDefinition.forEachRigidBodyDefinition(armDefinition.getRootBodyDefinition(), body ->
      {
         body.getVisualDefinitions().forEach(visual -> visual.setMaterialDefinition(material));
      });
      RigidBodyBasics armOnlyMultiBody
            = MultiBodySystemMissingTools.getDetachedCopyOfSubtreeWithElevator(syncedFullRobotModel.getChest(),
                                                                               syncedFullRobotModel.getArmJoint(side, firstArmJointName),
                                                                               syncedFullRobotModel.getHand(side).getName());
      rootBody = RDXMultiBodySystemFactories.toRDXMultiBodySystem(armOnlyMultiBody, armDefinition, RDXVisualTools.DESIRED_ROBOT_SCALING);
      rootBody.getRigidBodiesToHide().add("elevator");
      rootBody.getRigidBodiesToHide().add(jointMap.getChestName());
      joints = MultiBodySystemMissingTools.getSubtreeJointArray(OneDoFJointBasics.class, rootBody);

      hand = joints[joints.length - 1].getSuccessor();
      RigidBodyTransform controlToCoMTransform = new RigidBodyTransform();
      syncedFullRobotModel.getHandControlFrame(side).getTransformToDesiredFrame(controlToCoMTransform, syncedFullRobotModel.getHand(side).getBodyFixedFrame());
      handControlFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(hand.getBodyFixedFrame(), controlToCoMTransform);
   }

   public void updateAfterModifyingConfiguration()
   {
      rootBody.updateFramesRecursively();
      rootBody.updateSubtreeGraphics();
   }

   public void setColor(Color newColor)
   {
      if (newColor != color)
      {
         color = newColor;
         for (RigidBodyBasics body : rootBody.subtreeIterable())
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

   public RDXRigidBody getRootBody()
   {
      return rootBody;
   }

   public SixDoFJoint getFloatingJoint()
   {
      return (SixDoFJoint) rootBody.getChildrenJoints().get(0);
   }

   public OneDoFJointBasics[] getJoints()
   {
      return joints;
   }

   public RigidBodyBasics getHand()
   {
      return hand;
   }

   public ReferenceFrame getHandControlFrame()
   {
      return handControlFrame;
   }

   public Color getColor()
   {
      return color;
   }
}
