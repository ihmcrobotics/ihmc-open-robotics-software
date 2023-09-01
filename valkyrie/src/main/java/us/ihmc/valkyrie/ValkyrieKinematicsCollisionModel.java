package us.ihmc.valkyrie;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FrameCapsule3D;
import us.ihmc.euclid.referenceFrame.FrameSphere3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.CollidableHelper;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ValkyrieKinematicsCollisionModel implements RobotCollisionModel
{
   private final HumanoidJointNameMap jointMap;

   /**
    * When {@code true}, the collisions are tweaked to be more conservative. These collision shapes
    * were used during IHMC Open House 2023, where kids were able to take control of the robot with the
    * VR and IK streaming. The collision shapes may be too conservative for an experienced user trying
    * to achieve a task with the robot.
    */
   private boolean enableConservativeCollisions = false;

   public ValkyrieKinematicsCollisionModel(HumanoidJointNameMap jointMap)
   {
      this.jointMap = jointMap;
   }

   public void setEnableConservativeCollisions(boolean enableConservativeCollisions)
   {
      this.enableConservativeCollisions = enableConservativeCollisions;
   }

   @Override
   public List<Collidable> getRobotCollidables(MultiBodySystemBasics multiBodySystem)
   {
      CollidableHelper helper = new CollidableHelper();
      List<Collidable> allCollidables = new ArrayList<>();
      allCollidables.addAll(setupArmCollisions(multiBodySystem, helper));
      allCollidables.addAll(setupNeckCollisions(multiBodySystem, helper));
      return allCollidables;
   }

   private List<Collidable> setupArmCollisions(MultiBodySystemBasics multiBodySystem, CollidableHelper helper)
   {
      List<Collidable> collidables = new ArrayList<>();

      String bodyName = "Body";
      SideDependentList<String> armNames = new SideDependentList<>("LeftArm", "RightArm");
      SideDependentList<String> legNames = new SideDependentList<>("LeftLeg", "RightLeg");

      { // Body
         long collisionMask = helper.getCollisionMask(bodyName);
         long collisionGroup = helper.createCollisionGroup(armNames.get(RobotSide.LEFT), armNames.get(RobotSide.RIGHT));

         if (enableConservativeCollisions)
         { // 
            RigidBodyBasics head = multiBodySystem.findRigidBody(jointMap.getHeadName());
            MovingReferenceFrame headFrame = head.getParentJoint().getFrameAfterJoint();
            FrameSphere3D headShape = new FrameSphere3D(headFrame, 0.09, 0, 0, 0.18);
            collidables.add(new Collidable(head, collisionMask, collisionGroup, headShape));
         }

         RigidBodyBasics torso = multiBodySystem.findRigidBody(jointMap.getChestName());
         MovingReferenceFrame torsoFrame = torso.getParentJoint().getFrameAfterJoint();
         FrameCapsule3D torsoShapeTop = new FrameCapsule3D(torsoFrame, 0.08, 0.15);
         if (enableConservativeCollisions)
            torsoShapeTop.getPosition().set(0.04, 0.0, 0.253);
         else
            torsoShapeTop.getPosition().set(0.004, 0.0, 0.253);
         torsoShapeTop.getAxis().set(Axis3D.Y);
         collidables.add(new Collidable(torso, collisionMask, collisionGroup, torsoShapeTop));

         FrameCapsule3D torsoShapeBottom = new FrameCapsule3D(torsoFrame, 0.1, 0.12);
         torsoShapeBottom.getPosition().set(-0.006, 0.0, 0.103);
         if (enableConservativeCollisions)
         {
            torsoShapeBottom.setRadius(0.14);
            torsoShapeBottom.getPosition().set(0.0, 0.0, 0.103);
         }
         torsoShapeBottom.getAxis().set(Axis3D.Y);
         collidables.add(new Collidable(torso, collisionMask, collisionGroup, torsoShapeBottom));

         if (enableConservativeCollisions)
         {
            FrameSphere3D torsoShapeBack = new FrameSphere3D(torsoFrame, -0.175, 0, 0.22, 0.25);
            collidables.add(new Collidable(torso, collisionMask, collisionGroup, torsoShapeBack));
         }

         RigidBodyBasics pelvis = multiBodySystem.findRigidBody(jointMap.getPelvisName());
         MovingReferenceFrame pelvisFrame = pelvis.getParentJoint().getFrameAfterJoint();
         FrameCapsule3D pelvisShapeTop = new FrameCapsule3D(pelvisFrame, 0.15, 0.1);
         if (enableConservativeCollisions)
            pelvisShapeTop.setSize(0.22, 0.14);
         pelvisShapeTop.getAxis().set(Axis3D.Y);
         pelvisShapeTop.getPosition().set(0.0, 0.0, -0.08);
         collidables.add(new Collidable(pelvis, collisionMask, collisionGroup, pelvisShapeTop));
         FrameCapsule3D pelvisShapeBottom = new FrameCapsule3D(pelvisFrame, 0.125, 0.14);
         pelvisShapeBottom.getPosition().set(0.0, 0.0, -0.155);
         if (enableConservativeCollisions)
         {
            pelvisShapeBottom.setSize(0.125, 0.15);
            pelvisShapeBottom.getPosition().set(0.01, 0.0, -0.155);
         }

         collidables.add(new Collidable(pelvis, collisionMask, collisionGroup, pelvisShapeBottom));

         for (RobotSide robotSide : RobotSide.values)
         {
            RigidBodyBasics hip = multiBodySystem.findJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH)).getSuccessor();
            MovingReferenceFrame hipFrame = hip.getParentJoint().getFrameAfterJoint();
            FrameSphere3D hipShape = new FrameSphere3D(hipFrame, 0.125);
            hipShape.getPosition().set(0.007, robotSide.negateIfRightSide(0.081), -0.278);
            collidables.add(new Collidable(hip, collisionMask, collisionGroup, hipShape));
         }
      }

      for (RobotSide robotSide : RobotSide.values)
      { // Arms
         long collisionMask = helper.getCollisionMask(armNames.get(robotSide));
         long collisionGroup = helper.createCollisionGroup(bodyName,
                                                           armNames.get(robotSide.getOppositeSide()),
                                                           legNames.get(RobotSide.LEFT),
                                                           legNames.get(RobotSide.RIGHT));

         RigidBodyBasics hand = multiBodySystem.findRigidBody(jointMap.getHandName(robotSide));
         JointBasics elbowRollJoint = multiBodySystem.findJoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_ROLL));

         if (elbowRollJoint != null)
         {
            MovingReferenceFrame handFrame = hand.getParentJoint().getFrameAfterJoint();
            FrameCapsule3D handShapeFist = new FrameCapsule3D(handFrame, 0.04, 0.035);
            handShapeFist.getPosition().set(-0.007, robotSide.negateIfRightSide(0.085), -0.012);
            handShapeFist.getAxis().set(Axis3D.Z);
            collidables.add(new Collidable(hand, collisionMask, collisionGroup, handShapeFist));
            FrameCapsule3D handShapePalm = new FrameCapsule3D(handFrame, 0.035, 0.04);
            handShapePalm.getPosition().set(-0.012, robotSide.negateIfRightSide(0.042), -0.01);
            handShapePalm.getAxis().set(Axis3D.Z);
            collidables.add(new Collidable(hand, collisionMask, collisionGroup, handShapePalm));

            RigidBodyBasics forearm = elbowRollJoint.getSuccessor();
            MovingReferenceFrame forearmFrame = forearm.getParentJoint().getFrameAfterJoint();
            FrameCapsule3D forearmShape = new FrameCapsule3D(forearmFrame, 0.21, 0.08);
            forearmShape.getPosition().set(0.005, robotSide.negateIfRightSide(0.11), 0.0);
            forearmShape.getAxis().set(Axis3D.Y);
            collidables.add(new Collidable(forearm, collisionMask, collisionGroup, forearmShape));
         }
         else
         { // Assuming this val with arm mass sim
            RigidBodyBasics massSim = multiBodySystem.findJoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH)).getSuccessor();
            MovingReferenceFrame massSimFrame = massSim.getParentJoint().getFrameAfterJoint();
            FrameCapsule3D massSimShape = new FrameCapsule3D(massSimFrame, 0.36, 0.065);
            if (enableConservativeCollisions)
               massSimShape.setRadius(0.09);
            massSimShape.getPosition().set(-0.025, robotSide.negateIfRightSide(0.17), 0.0);
            massSimShape.getAxis().set(Axis3D.Y);
            collidables.add(new Collidable(massSim, collisionMask, collisionGroup, massSimShape));
         }
      }

      for (RobotSide robotSide : RobotSide.values)
      { // Legs
         long collisionMask = helper.getCollisionMask(legNames.get(robotSide));
         long collisionGroup = helper.createCollisionGroup(armNames.get(RobotSide.LEFT), armNames.get(RobotSide.RIGHT));

         RigidBodyBasics upperLeg = multiBodySystem.findJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH)).getSuccessor();
         MovingReferenceFrame upperLegFrame = upperLeg.getParentJoint().getFrameAfterJoint();
         FrameCapsule3D upperLegShapeTop = new FrameCapsule3D(upperLegFrame, 0.25, 0.1);
         if (enableConservativeCollisions)
            upperLegShapeTop.setRadius(0.13);
         upperLegShapeTop.getPosition().set(0.032, robotSide.negateIfRightSide(0.091), -0.108);
         upperLegShapeTop.getAxis().set(new Vector3D(-0.139, robotSide.negateIfLeftSide(0.1), 0.99));
         collidables.add(new Collidable(upperLeg, collisionMask, collisionGroup, upperLegShapeTop));
         FrameCapsule3D upperLegShapeBottom = new FrameCapsule3D(upperLegFrame, 0.20, 0.105);
         upperLegShapeBottom.getPosition().set(0.027, robotSide.negateIfRightSide(0.081), -0.308);
         upperLegShapeBottom.getAxis().set(new Vector3D(0.208, robotSide.negateIfRightSide(0.1), 0.978));
         collidables.add(new Collidable(upperLeg, collisionMask, collisionGroup, upperLegShapeBottom));

         RigidBodyBasics lowerLeg = multiBodySystem.findJoint(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH)).getSuccessor();
         MovingReferenceFrame lowerLegFrame = lowerLeg.getParentJoint().getFrameAfterJoint();
         FrameCapsule3D lowerLegShape = new FrameCapsule3D(lowerLegFrame, 0.23, 0.11);
         lowerLegShape.getPosition().set(-0.012, 0.0, -0.189);
         lowerLegShape.getAxis().set(new Vector3D(0.08, 0.0, 1.0));
         collidables.add(new Collidable(lowerLeg, collisionMask, collisionGroup, lowerLegShape));
      }

      return collidables;
   }

   private List<Collidable> setupNeckCollisions(MultiBodySystemBasics multiBodySystem, CollidableHelper helper)
   {
      List<Collidable> collidables = new ArrayList<>();

      String chinName = "Chin";
      String bodyName = "TorsoChin";

      { // Body
         long collisionMask = helper.getCollisionMask(bodyName);
         long collisionGroup = helper.createCollisionGroup(chinName);

         RigidBodyBasics torso = multiBodySystem.findRigidBody(jointMap.getChestName());
         MovingReferenceFrame torsoFrame = torso.getParentJoint().getFrameAfterJoint();
         FrameCapsule3D torsoShapeTop = new FrameCapsule3D(torsoFrame, 0.15, 0.15);
         torsoShapeTop.getPosition().set(0.014, 0.0, 0.243);
         torsoShapeTop.getAxis().set(Axis3D.Y);
         collidables.add(new Collidable(torso, collisionMask, collisionGroup, torsoShapeTop));
      }

      { // Chin
         long collisionMask = helper.getCollisionMask(chinName);
         long collisionGroup = helper.createCollisionGroup(bodyName);

         RigidBodyBasics head = multiBodySystem.findRigidBody(jointMap.getHeadName());
         MovingReferenceFrame headFrame = head.getParentJoint().getFrameAfterJoint();
         FrameCapsule3D chinShape = new FrameCapsule3D(headFrame, 0.05, 0.02);
         chinShape.getPosition().set(0.172, 0.0, -0.129);
         chinShape.getAxis().set(Axis3D.Y);
         collidables.add(new Collidable(head, collisionMask, collisionGroup, chinShape));
      }

      return collidables;
   }
}
