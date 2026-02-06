package us.ihmc.rdx.ui.affordances;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.rdx.simulation.scs2.RDXMultiBodySystemFactories;
import us.ihmc.rdx.simulation.scs2.RDXRigidBody;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

public class RDXInteractableTools
{
   public static String getModelFileName(RigidBodyDefinition rigidBodyDefinition)
   {
      ModelFileGeometryDefinition modelFileGeometryDefinition = null;
      for (VisualDefinition visualDefinition : rigidBodyDefinition.getVisualDefinitions())
      {
         if (visualDefinition.getGeometryDefinition() instanceof ModelFileGeometryDefinition)
         {
            modelFileGeometryDefinition = (ModelFileGeometryDefinition) visualDefinition.getGeometryDefinition();
            break;
         }
      }
      if (modelFileGeometryDefinition == null || modelFileGeometryDefinition.getFileName() == null)
      {
         LogTools.error("Interactables for {} need a model file or implementation of shape visuals", rigidBodyDefinition.getName());
         return null;
      }
      return modelFileGeometryDefinition.getFileName();
   }

   public static RDXRigidBody loadAbilityHand(RobotDefinition robotDefinition, RobotSide side)
   {
      RigidBodyDefinition wristLink = robotDefinition.getRigidBodyDefinition(side.getLowerCaseName() + "_wrist_yaw_link"); // H1
      if (wristLink == null)
         wristLink = robotDefinition.getRigidBodyDefinition(side.getSideNameInAllCaps() + "_GRIPPER_Z_LINK"); // Alex

      RDXRigidBody body = null;
      if (wristLink != null)
      {
         RobotDefinition handRobot = new RobotDefinition(robotDefinition.getName() + "_left_hand");
         RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");
         handRobot.setRootBodyDefinition(elevator);
         SixDoFJointDefinition floatingRoot = new SixDoFJointDefinition("floating_base");
         elevator.addChildJoint(floatingRoot);
         RigidBodyDefinition copiedWristLink = wristLink.copyRecursive();
         floatingRoot.setSuccessor(copiedWristLink);
         copiedWristLink.getVisualDefinitions().removeIf(definition -> definition.getName() != null // Keep only hand mesh
                                                                       && !definition.getName().toLowerCase().contains("palm_mesh"));

         RigidBodyBasics handMultiBody = handRobot.newInstance(ReferenceFrame.getWorldFrame());
         body = RDXMultiBodySystemFactories.toRDXMultiBodySystem(handMultiBody, handRobot);
         body.updateSubtreeGraphics();
      }

      return body;
   }
}
