package us.ihmc.atlas.contactEstimation;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.FrameCapsule3D;
import us.ihmc.euclid.referenceFrame.FrameEllipsoid3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
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

import java.util.ArrayList;
import java.util.List;

public class AtlasMultiContactCollisionModel implements RobotCollisionModel
{
   private final HumanoidJointNameMap jointMap;

   public AtlasMultiContactCollisionModel(HumanoidJointNameMap jointMap)
   {
      this.jointMap = jointMap;
   }

   // TODO Need to implement the RobotiQ hands, this implementation only cover the knobs.
   @Override
   public List<Collidable> getRobotCollidables(MultiBodySystemBasics multiBodySystem)
   {
      CollidableHelper helper = new CollidableHelper();
      List<Collidable> collidables = new ArrayList<>();

      String bodyName = "Body"; // head + chest + pelvis + legs
      SideDependentList<String> armNames = new SideDependentList<>("LeftArm", "RightArm");

      { // Body
         long collisionMask = helper.getCollisionMask(bodyName);
         long collisionGroup = helper.createCollisionGroup(armNames.get(RobotSide.LEFT), armNames.get(RobotSide.RIGHT));

         RigidBodyBasics head = RobotCollisionModel.findRigidBody(jointMap.getHeadName(), multiBodySystem);
         RigidBodyBasics torso = RobotCollisionModel.findRigidBody(jointMap.getChestName(), multiBodySystem);
         RigidBodyBasics pelvis = RobotCollisionModel.findRigidBody(jointMap.getPelvisName(), multiBodySystem);

         // Torso ---------------------------------------------------------------------
         MovingReferenceFrame torsoFrame = torso.getParentJoint().getFrameAfterJoint();

         FrameBox3D backpack = new FrameBox3D(torsoFrame, 0.44, 0.38, 0.6);
         backpack.getPosition().set(-0.1, 0.0, 0.3);
         collidables.add(new Collidable(torso, collisionMask, collisionGroup, backpack));

         // Pelvis  ---------------------------------------------------------------------
         MovingReferenceFrame pelvisFrame = pelvis.getParentJoint().getFrameAfterJoint();
         FrameEllipsoid3D pelvisShape = new FrameEllipsoid3D(pelvisFrame, 0.18, 0.18, 0.23);
         pelvisShape.getPosition().set(0.0, 0.0, 0.0);
         collidables.add(new Collidable(pelvis, collisionMask, collisionGroup, pelvisShape));

         // Legs ---------------------------------------------------------------------
         for (RobotSide robotSide : RobotSide.values)
         {
            JointBasics hipPitchJoint = RobotCollisionModel.findJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH), multiBodySystem);
            RigidBodyBasics thigh = hipPitchJoint.getSuccessor();
            ReferenceFrame thighFrame = hipPitchJoint.getFrameAfterJoint();

            FrameCapsule3D thighShapeTop = new FrameCapsule3D(thighFrame, 0.1, 0.09);
            thighShapeTop.getPosition().set(0.0, 0.0, -0.1);
            thighShapeTop.getAxis().set(Axis3D.Z);
            collidables.add(new Collidable(thigh, collisionMask, collisionGroup, thighShapeTop));

            FrameCapsule3D thighShapeBottom = new FrameCapsule3D(thighFrame, 0.15, 0.085);
            thighShapeBottom.getPosition().set(-0.018, 0.0, -0.25);
            thighShapeBottom.getAxis().set(0.22, 0.0, 1.0);
            collidables.add(new Collidable(thigh, collisionMask, collisionGroup, thighShapeBottom));

            FrameCapsule3D thighShapeBack = new FrameCapsule3D(thighFrame, 0.15, 0.085);
            thighShapeBack.getPosition().set(-0.05, 0.0, -0.15);
            thighShapeBack.getAxis().set(Axis3D.Z);
            collidables.add(new Collidable(thigh, collisionMask, collisionGroup, thighShapeBack));

            JointBasics shinPitchJoint = RobotCollisionModel.findJoint(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH), multiBodySystem);
            RigidBodyBasics shin = shinPitchJoint.getSuccessor();
            ReferenceFrame shinFrame = shinPitchJoint.getFrameAfterJoint();

            FrameCapsule3D shinShape = new FrameCapsule3D(shinFrame, 0.3, 0.08);
            shinShape.getPosition().set(0.015, 0.0, -0.2);
            shinShape.getAxis().set(0.1, 0.0, 1.0);
            collidables.add(new Collidable(shin, collisionMask, collisionGroup, shinShape));
         }
      }

      collidables.addAll(setupLegCollisions(multiBodySystem, helper));

      return collidables;
   }

   private List<Collidable> setupLegCollisions(MultiBodySystemBasics multiBodySystem, CollidableHelper helper)
   {
      List<Collidable> collidables = new ArrayList<>();

      String leftFootName = "LeftFoot";
      String rightFootName = "RightFoot";

      // Collision between feet
      { // Left foot
         long collisionMask = helper.getCollisionMask(leftFootName);
         long collisionGroup = helper.createCollisionGroup(rightFootName);

         JointBasics ankleRoll = RobotCollisionModel.findJoint(jointMap.getLegJointName(RobotSide.LEFT, LegJointName.ANKLE_ROLL), multiBodySystem);
         MovingReferenceFrame ankleRollFrame = ankleRoll.getFrameAfterJoint();
         RigidBodyBasics foot = ankleRoll.getSuccessor();

         FrameBox3D footShape = new FrameBox3D(ankleRollFrame, 0.26, 0.14, 0.055);
         footShape.getPosition().set(0.045, 0.0, -0.05);
         collidables.add(new Collidable(foot, collisionMask, collisionGroup, footShape));
      }

      { // Right foot
         long collisionMask = helper.getCollisionMask(rightFootName);
         long collisionGroup = helper.createCollisionGroup(leftFootName);

         JointBasics ankleRoll = RobotCollisionModel.findJoint(jointMap.getLegJointName(RobotSide.RIGHT, LegJointName.ANKLE_ROLL), multiBodySystem);
         MovingReferenceFrame ankleRollFrame = ankleRoll.getFrameAfterJoint();
         RigidBodyBasics foot = ankleRoll.getSuccessor();

         FrameBox3D footShape = new FrameBox3D(ankleRollFrame, 0.26, 0.14, 0.055);
         footShape.getPosition().set(0.045, 0.0, -0.05);
         collidables.add(new Collidable(foot, collisionMask, collisionGroup, footShape));
      }

      // Collision between thighs
      String leftThighName = "LeftThigh";
      String rightThighName = "RightThigh";

      { // Left thigh
         long collisionMask = helper.getCollisionMask(leftThighName);
         long collisionGroup = helper.createCollisionGroup(rightThighName);

         JointBasics hipPitch = RobotCollisionModel.findJoint(jointMap.getLegJointName(RobotSide.LEFT, LegJointName.HIP_PITCH), multiBodySystem);
         MovingReferenceFrame hipPitchFrame = hipPitch.getFrameAfterJoint();
         RigidBodyBasics thigh = hipPitch.getSuccessor();

         FrameCapsule3D thighTopShape = new FrameCapsule3D(hipPitchFrame, 0.1, 0.09);
         thighTopShape.getPosition().set(0.0, 0.0, -0.1);
         thighTopShape.getAxis().set(Axis3D.Z);
         collidables.add(new Collidable(thigh, collisionMask, collisionGroup, thighTopShape));

         FrameCapsule3D thighFrontShape = new FrameCapsule3D(hipPitchFrame, 0.15, 0.085);
         thighFrontShape.getPosition().set(-0.018, 0.0, -0.25);
         thighFrontShape.getAxis().set(new Vector3D(0.22, 0.0, 1.0));
         collidables.add(new Collidable(thigh, collisionMask, collisionGroup, thighFrontShape));

         FrameCapsule3D thighBackShape = new FrameCapsule3D(hipPitchFrame, 0.15, 0.085);
         thighBackShape.getPosition().set(-0.05, 0.0, -0.15);
         thighBackShape.getAxis().set(Axis3D.Z);
         collidables.add(new Collidable(thigh, collisionMask, collisionGroup, thighBackShape));
      }

      { // Right thigh
         long collisionMask = helper.getCollisionMask(rightThighName);
         long collisionGroup = helper.createCollisionGroup(leftThighName);

         JointBasics hipPitch = RobotCollisionModel.findJoint(jointMap.getLegJointName(RobotSide.RIGHT, LegJointName.HIP_PITCH), multiBodySystem);
         MovingReferenceFrame hipPitchFrame = hipPitch.getFrameAfterJoint();
         RigidBodyBasics thigh = hipPitch.getSuccessor();

         FrameCapsule3D thighTopShape = new FrameCapsule3D(hipPitchFrame, 0.1, 0.09);
         thighTopShape.getPosition().set(0.0, 0.0, -0.1);
         thighTopShape.getAxis().set(Axis3D.Z);
         collidables.add(new Collidable(thigh, collisionMask, collisionGroup, thighTopShape));

         FrameCapsule3D thighFrontShape = new FrameCapsule3D(hipPitchFrame, 0.15, 0.085);
         thighFrontShape.getPosition().set(-0.018, 0.0, -0.25);
         thighFrontShape.getAxis().set(new Vector3D(0.22, 0.0, 1.0));
         collidables.add(new Collidable(thigh, collisionMask, collisionGroup, thighFrontShape));

         FrameCapsule3D thighBackShape = new FrameCapsule3D(hipPitchFrame, 0.15, 0.085);
         thighBackShape.getPosition().set(-0.05, 0.0, -0.15);
         thighBackShape.getAxis().set(Axis3D.Z);
         collidables.add(new Collidable(thigh, collisionMask, collisionGroup, thighBackShape));
      }

      // Collision between shins
      String leftShinName = "LeftShin";
      String rightShinName = "RightShin";

      { // Left Shin
         long collisionMask = helper.getCollisionMask(leftShinName);
         long collisionGroup = helper.createCollisionGroup(rightShinName);

         JointBasics kneePitch = RobotCollisionModel.findJoint(jointMap.getLegJointName(RobotSide.LEFT, LegJointName.KNEE_PITCH), multiBodySystem);
         MovingReferenceFrame kneePitchFrame = kneePitch.getFrameAfterJoint();
         RigidBodyBasics shin = kneePitch.getSuccessor();

         FrameCapsule3D shinShape = new FrameCapsule3D(kneePitchFrame, 0.3, 0.08);
         shinShape.getPosition().set(0.015, 0.0, -0.2);
         shinShape.getAxis().set(new Vector3D(0.1, 0.0, 1.0));
         collidables.add(new Collidable(shin, collisionMask, collisionGroup, shinShape));
      }

      { // Right Shin
         long collisionMask = helper.getCollisionMask(rightThighName);
         long collisionGroup = helper.createCollisionGroup(leftThighName);
         RobotSide robotSide = RobotSide.RIGHT;

         JointBasics kneePitch = RobotCollisionModel.findJoint(jointMap.getLegJointName(RobotSide.RIGHT, LegJointName.KNEE_PITCH), multiBodySystem);
         MovingReferenceFrame kneePitchFrame = kneePitch.getFrameAfterJoint();
         RigidBodyBasics shin = kneePitch.getSuccessor();

         FrameCapsule3D shinShape = new FrameCapsule3D(kneePitchFrame, 0.3, 0.08);
         shinShape.getPosition().set(0.015, 0.0, -0.2);
         shinShape.getAxis().set(new Vector3D(0.1, 0.0, 1.0));
         collidables.add(new Collidable(shin, collisionMask, collisionGroup, shinShape));
      }

      return collidables;
   }
}
