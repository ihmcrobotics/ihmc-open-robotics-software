package us.ihmc.avatar.inverseKinematics;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxOptimizationSettings;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.MultiBodySystemMissingTools;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

public class ArmIKSolver
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   // TODO: Mess with these settings
   private final KinematicsToolboxOptimizationSettings optimizationSettings = new KinematicsToolboxOptimizationSettings();

   public ArmIKSolver(RobotSide side,
                      DRCRobotModel robotModel,
                      FullHumanoidRobotModel actualRobot,
                      FullHumanoidRobotModel desiredRobot,
                      ReferenceFrame handControlDesiredFrame)
   {


      ArmJointName[] armJointNames = robotModel.getJointMap().getArmJointNames();
      OneDoFJointBasics[] armOneDoFJoints = new OneDoFJointBasics[armJointNames.length];
      for (int i = 0; i < armJointNames.length; i++)
      {
         armOneDoFJoints[i] = actualRobot.getArmJoint(side, armJointNames[i]);
      }

      OneDoFJointBasics rootJoint = armOneDoFJoints[0];
      RigidBodyBasics actualChest = rootJoint.getPredecessor(); // chest

      RigidBody detachedArmOnlyRobot = MultiBodySystemMissingTools.getDetachedCopyOfSubtree(actualChest, armOneDoFJoints[0]);

      // Remove fingers
      RigidBodyBasics hand = MultiBodySystemTools.findRigidBody(detachedArmOnlyRobot, robotModel.getJointMap().getHandName(side));
      hand.getChildrenJoints().clear();


      RigidBodyBasics chest = MultiBodySystemTools.findRigidBody(detachedArmOnlyRobot, robotModel.getJointMap().getChestName());

      List<RigidBodyBasics> controllableRigidBodies = new ArrayList<>();
      controllableRigidBodies.add(chest);
      controllableRigidBodies.add(hand);


//      new WholeBodyControllerCore(registry);
   }
}
