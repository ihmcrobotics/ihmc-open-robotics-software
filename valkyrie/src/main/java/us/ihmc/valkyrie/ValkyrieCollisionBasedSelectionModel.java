package us.ihmc.valkyrie;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.FrameCapsule3D;
import us.ihmc.euclid.referenceFrame.FrameSphere3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;

/**
 * Implementation of a collision model that can be used to select rigid bodies via a GUI.
 * <p>
 * This implementation should not be used as a representative collision model for a physics engine
 * nor for doing collision avoidance.
 * </p>
 */
public class ValkyrieCollisionBasedSelectionModel implements RobotCollisionModel
{
   private final HumanoidJointNameMap jointMap;

   public ValkyrieCollisionBasedSelectionModel(HumanoidJointNameMap jointMap)
   {
      this.jointMap = jointMap;
   }

   @Override
   public List<Collidable> getRobotCollidables(MultiBodySystemBasics multiBodySystem)
   {
      List<Collidable> collidables = new ArrayList<>();
      long collisionMask = -1;
      long collisionGroup = -1;

      { // Head
         RigidBodyBasics head = RobotCollisionModel.findRigidBody(jointMap.getHeadName(), multiBodySystem);
         MovingReferenceFrame headFrame = head.getParentJoint().getFrameAfterJoint();
         FrameSphere3D headShape = new FrameSphere3D(headFrame, 0.15);
         headShape.getPosition().set(0.077, 0.0, 0.001);
         collidables.add(new Collidable(head, collisionMask, collisionGroup, headShape));
      }

      for (RobotSide robotSide : RobotSide.values)
      { // Arms
         { // Shoulder pitch link
            JointBasics shoulderPitch = RobotCollisionModel.findJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH), multiBodySystem);
            MovingReferenceFrame shoudlerPitchFrame = shoulderPitch.getFrameAfterJoint();
            FrameCapsule3D shoulderPitchShape = new FrameCapsule3D(shoudlerPitchFrame, 0.10, 0.075);
            shoulderPitchShape.getPosition().set(-0.0025, robotSide.negateIfRightSide(0.25), 0.0);
            shoulderPitchShape.getAxis().set(Axis3D.X);
            collidables.add(new Collidable(shoulderPitch.getSuccessor(), collisionMask, collisionGroup, shoulderPitchShape));
         }

         { // Shoulder roll link
            JointBasics shoulderRoll = RobotCollisionModel.findJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL), multiBodySystem);
            MovingReferenceFrame shoulderRollFrame = shoulderRoll.getFrameAfterJoint();
            FrameCapsule3D shouldRollShape = new FrameCapsule3D(shoulderRollFrame, 0.08, 0.075);
            shouldRollShape.getPosition().set(0.0, robotSide.negateIfRightSide(0.05), 0.0);
            shouldRollShape.getAxis().set(Axis3D.Y);
            collidables.add(new Collidable(shoulderRoll.getSuccessor(), collisionMask, collisionGroup, shouldRollShape));
         }

         { // Shoulder yaw link
            JointBasics shoulderYaw = RobotCollisionModel.findJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW), multiBodySystem);
            MovingReferenceFrame shoulderYawFrame = shoulderYaw.getFrameAfterJoint();
            FrameCapsule3D shoulderYawShape = new FrameCapsule3D(shoulderYawFrame, 0.08, 0.075);
            shoulderYawShape.getPosition().set(0.0, robotSide.negateIfRightSide(0.21), 0.0);
            shoulderYawShape.getAxis().set(Axis3D.Y);
            collidables.add(new Collidable(shoulderYaw.getSuccessor(), collisionMask, collisionGroup, shoulderYawShape));
         }

         { // Elbow
            JointBasics elbowPitch = RobotCollisionModel.findJoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH), multiBodySystem);
            MovingReferenceFrame elbowPitchFrame = elbowPitch.getFrameAfterJoint();
            FrameCapsule3D elbowShape = new FrameCapsule3D(elbowPitchFrame, 0.08, 0.05);
            elbowShape.getPosition().set(0.0, 0.0, 0.0);
            elbowShape.getAxis().set(Axis3D.Z);
            collidables.add(new Collidable(elbowPitch.getSuccessor(), collisionMask, collisionGroup, elbowShape));
         }

         { // Forearm
            JointBasics elbowRoll = RobotCollisionModel.findJoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_ROLL), multiBodySystem);
            MovingReferenceFrame elbowRollFrame = elbowRoll.getFrameAfterJoint();
            FrameCapsule3D forearmShape = new FrameCapsule3D(elbowRollFrame, 0.15, 0.075);
            forearmShape.getPosition().set(0.0, robotSide.negateIfRightSide(0.14), 0.0);
            forearmShape.getAxis().set(Axis3D.Y);
            collidables.add(new Collidable(elbowRoll.getSuccessor(), collisionMask, collisionGroup, forearmShape));
         }

         { // Hand
            RigidBodyBasics hand = RobotCollisionModel.findRigidBody(jointMap.getHandName(robotSide), multiBodySystem);
            MovingReferenceFrame handFrame = hand.getParentJoint().getFrameAfterJoint();
            FrameCapsule3D handShape = new FrameCapsule3D(handFrame, 0.02, 0.055);
            handShape.getPosition().set(-0.007, robotSide.negateIfRightSide(0.062), -0.01);
            handShape.getAxis().set(Axis3D.Y);
            collidables.add(new Collidable(hand, collisionMask, collisionGroup, handShape));
         }
      }

      { // Chest
         RigidBodyBasics chest = RobotCollisionModel.findRigidBody(jointMap.getChestName(), multiBodySystem);
         MovingReferenceFrame chestFrame = chest.getParentJoint().getFrameAfterJoint();
         FrameBox3D chestShape = new FrameBox3D(chestFrame, 0.45, 0.35, 0.4);
         chestShape.getPosition().set(-0.08, 0.0, 0.21);
         collidables.add(new Collidable(chest, collisionMask, collisionGroup, chestShape));
      }

      { // Pelvis
         RigidBodyBasics pelvis = RobotCollisionModel.findRigidBody(jointMap.getPelvisName(), multiBodySystem);
         MovingReferenceFrame pelvisFrame = pelvis.getParentJoint().getFrameAfterJoint();
         FrameBox3D pelvisShape = new FrameBox3D(pelvisFrame, 0.25, 0.35, 0.1);
         pelvisShape.getPosition().set(0.0, 0.0, -0.06);
         collidables.add(new Collidable(pelvis, collisionMask, collisionGroup, pelvisShape));

         FrameBox3D pelvisInnerShape = new FrameBox3D(pelvisFrame, 0.25, 0.075, 0.23);
         pelvisInnerShape.getPosition().set(0.0, 0.0, -0.225);
         collidables.add(new Collidable(pelvis, collisionMask, collisionGroup, pelvisInnerShape));
      }

      for (RobotSide robotSide : RobotSide.values)
      { // Legs
         { // Hip yaw link
            JointBasics hipYaw = RobotCollisionModel.findJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_ROLL), multiBodySystem);
            MovingReferenceFrame hipYawFrame = hipYaw.getFrameAfterJoint();
            FrameSphere3D hipYawShape = new FrameSphere3D(hipYawFrame, 0.13);
            hipYawShape.getPosition().set(0.0, 0.0, -0.03);
            collidables.add(new Collidable(hipYaw.getSuccessor(), collisionMask, collisionGroup, hipYawShape));
         }

         { // Thigh
            RigidBodyBasics thigh = RobotCollisionModel.findJoint(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH), multiBodySystem)
                                                       .getPredecessor();
            MovingReferenceFrame thighFrame = thigh.getParentJoint().getFrameAfterJoint();
            FrameCapsule3D thighShape = new FrameCapsule3D(thighFrame, 0.22, 0.12);
            thighShape.getPosition().set(0.02, robotSide.negateIfRightSide(0.09), -0.22);
            thighShape.getAxis().set(new Vector3D(0.0, 0.0, 1.0));
            collidables.add(new Collidable(thigh, collisionMask, collisionGroup, thighShape));
         }

         { // Shin
            RigidBodyBasics shin = RobotCollisionModel.findJoint(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH), multiBodySystem).getSuccessor();
            MovingReferenceFrame shinFrame = shin.getParentJoint().getFrameAfterJoint();
            FrameCapsule3D shinShape = new FrameCapsule3D(shinFrame, 0.22, 0.12);
            shinShape.getPosition().set(-0.01, 0.0, -0.18);
            shinShape.getAxis().set(new Vector3D(0.0, 0.0, 1.0));
            collidables.add(new Collidable(shin, collisionMask, collisionGroup, shinShape));
         }

         { // Foot
            JointBasics ankleRoll = RobotCollisionModel.findJoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_ROLL), multiBodySystem);
            MovingReferenceFrame ankleRollFrame = ankleRoll.getFrameAfterJoint();
            FrameBox3D footShape = new FrameBox3D(ankleRollFrame, 0.275, 0.16, 0.095);
            footShape.getPosition().set(0.044, 0.0, -0.042);
            collidables.add(new Collidable(ankleRoll.getSuccessor(), collisionMask, collisionGroup, footShape));
         }
      }

      return collidables;
   }
}
