package us.ihmc.valkyrie;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.Capsule3D;
import us.ihmc.euclid.shape.primitives.PointShape3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.CollidableHelper;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationToolkit.physicsEngine.ExperimentalSimulation;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

/**
 * Collision model for Valkyrie used for simulating shape-to-shape collisions. It is used only with
 * {@link ExperimentalSimulation} and not with the default SCS physics engine.
 * <p>
 * {@link ExperimentalSimulation} can be used instead of the default SCS physics engine using
 * {@link DRCSCSInitialSetup#setUseExperimentalPhysicsEngine(boolean)}.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class ValkyrieSimulationCollisionModel implements RobotCollisionModel
{
   private CollidableHelper helper;
   private String[] otherCollisionMasks;
   private final DRCRobotJointMap jointMap;
   private String robotCollisionMask;

   public ValkyrieSimulationCollisionModel(DRCRobotJointMap jointMap)
   {
      this.jointMap = jointMap;
      // Setting default info.
      setCollidableHelper(new CollidableHelper(), jointMap.getModelName(), "other");
   }

   public void setCollidableHelper(CollidableHelper helper, String robotCollisionMask, String... otherCollisionMasks)
   {
      this.helper = helper;
      this.robotCollisionMask = robotCollisionMask;
      this.otherCollisionMasks = otherCollisionMasks;
   }

   @Override
   public List<Collidable> getRobotCollidables(MultiBodySystemBasics multiBodySystem)
   {
      List<Collidable> collidables = new ArrayList<>();
      long collisionMask = helper.getCollisionMask(robotCollisionMask);
      long collisionGroup = helper.createCollisionGroup(otherCollisionMasks);

      { // Head
         RigidBodyBasics head = RobotCollisionModel.findRigidBody(jointMap.getHeadName(), multiBodySystem);
         MovingReferenceFrame headFrame = head.getParentJoint().getFrameAfterJoint();
         Sphere3D headShape = new Sphere3D(0.15);
         headShape.getPosition().set(0.077, 0.0, 0.001);
         collidables.add(new Collidable(head, collisionMask, collisionGroup, headShape, headFrame));
      }

      for (RobotSide robotSide : RobotSide.values)
      { // Arms
         { // Shoulder
            JointBasics shoulderRoll = RobotCollisionModel.findJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL), multiBodySystem);
            MovingReferenceFrame shoudlerRollFrame = shoulderRoll.getFrameAfterJoint();
            Capsule3D shoulderShape = new Capsule3D(0.10, 0.075);
            shoulderShape.getPosition().set(-0.0025, 0.0, 0.0);
            shoulderShape.getAxis().set(Axis3D.X);
            collidables.add(new Collidable(shoulderRoll.getPredecessor(), collisionMask, collisionGroup, shoulderShape, shoudlerRollFrame));
         }

         { // Upper-arm
            JointBasics shoulderYaw = RobotCollisionModel.findJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW), multiBodySystem);
            MovingReferenceFrame shoulderYawFrame = shoulderYaw.getFrameAfterJoint();
            Capsule3D upperArmShape = new Capsule3D(0.25, 0.075);
            upperArmShape.getPosition().set(0.0, robotSide.negateIfRightSide(0.14), 0.0);
            upperArmShape.getAxis().set(Axis3D.Y);
            collidables.add(new Collidable(shoulderYaw.getPredecessor(), collisionMask, collisionGroup, upperArmShape, shoulderYawFrame));
         }

         { // Elbow
            JointBasics elbow = RobotCollisionModel.findJoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH), multiBodySystem);
            MovingReferenceFrame elbowFrame = elbow.getFrameAfterJoint();
            Capsule3D elbowShape = new Capsule3D(0.08, 0.05);
            elbowShape.getPosition().set(0.0, 0.0, 0.0);
            elbowShape.getAxis().set(Axis3D.Z);
            collidables.add(new Collidable(elbow.getPredecessor(), collisionMask, collisionGroup, elbowShape, elbowFrame));
         }

         { // Forearm
            JointBasics elbow = RobotCollisionModel.findJoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH), multiBodySystem);
            MovingReferenceFrame elbowFrame = elbow.getFrameAfterJoint();
            Capsule3D forearmShape = new Capsule3D(0.15, 0.075);
            forearmShape.getPosition().set(-0.03, robotSide.negateIfRightSide(0.14), 0.0);
            forearmShape.getAxis().set(Axis3D.Y);
            collidables.add(new Collidable(elbow.getSuccessor(), collisionMask, collisionGroup, forearmShape, elbowFrame));
         }

         { // Hand
            RigidBodyBasics hand = RobotCollisionModel.findRigidBody(jointMap.getHandName(robotSide), multiBodySystem);
            MovingReferenceFrame handFrame = hand.getParentJoint().getFrameAfterJoint();
            Capsule3D handShape = new Capsule3D(0.02, 0.055);
            handShape.getPosition().set(-0.007, robotSide.negateIfRightSide(0.062), -0.01);
            handShape.getAxis().set(Axis3D.Y);
            collidables.add(new Collidable(hand, collisionMask, collisionGroup, handShape, handFrame));
         }
      }

      { // Torso
         RigidBodyBasics torso = RobotCollisionModel.findRigidBody(jointMap.getChestName(), multiBodySystem);
         MovingReferenceFrame torsoFrame = torso.getParentJoint().getFrameAfterJoint();
         Capsule3D chestFrontShape = new Capsule3D(0.13, 0.12);
         chestFrontShape.getPosition().set(0.034, 0.0, 0.253);
         chestFrontShape.getAxis().set(Axis3D.Y);
         collidables.add(new Collidable(torso, collisionMask, collisionGroup, chestFrontShape, torsoFrame));

         Box3D chestBackShape = new Box3D(0.35, 0.35, 0.4);
         chestBackShape.getPosition().set(-0.111, 0.0, 0.208);
         collidables.add(new Collidable(torso, collisionMask, collisionGroup, chestBackShape, torsoFrame));
      }

      { // Pelvis
         RigidBodyBasics pelvis = RobotCollisionModel.findRigidBody(jointMap.getPelvisName(), multiBodySystem);
         MovingReferenceFrame pelvisFrame = pelvis.getParentJoint().getFrameAfterJoint();
         Capsule3D pelvisTopShape = new Capsule3D(0.2, 0.1);
         pelvisTopShape.getPosition().set(-0.005, 0.0, -0.076);
         pelvisTopShape.getAxis().set(Axis3D.Y);
         collidables.add(new Collidable(pelvis, collisionMask, collisionGroup, pelvisTopShape, pelvisFrame));

         Capsule3D pelvisFrontShape = new Capsule3D(0.19, 0.1);
         pelvisFrontShape.getPosition().set(0.025, 0.0, -0.154);
         pelvisFrontShape.getAxis().set(Axis3D.Z);
         collidables.add(new Collidable(pelvis, collisionMask, collisionGroup, pelvisFrontShape, pelvisFrame));

         Capsule3D pelvisBackShape = new Capsule3D(0.19, 0.1);
         pelvisBackShape.getPosition().set(-0.015, 0.0, -0.154);
         pelvisBackShape.getAxis().set(Axis3D.Z);
         collidables.add(new Collidable(pelvis, collisionMask, collisionGroup, pelvisBackShape, pelvisFrame));
      }

      for (RobotSide robotSide : RobotSide.values)
      { // Legs
         { // Hip
            JointBasics hipYaw = RobotCollisionModel.findJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW), multiBodySystem);
            MovingReferenceFrame hipYawFrame = hipYaw.getFrameAfterJoint();
            Sphere3D hipShape = new Sphere3D(0.13);
            hipShape.getPosition().set(0.0, 0.0, -0.03);
            collidables.add(new Collidable(hipYaw.getPredecessor(), collisionMask, collisionGroup, hipShape, hipYawFrame));
         }

         { // Thigh
            RigidBodyBasics thigh = RobotCollisionModel.findJoint(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH), multiBodySystem)
                                                       .getPredecessor();
            MovingReferenceFrame thighFrame = thigh.getParentJoint().getFrameAfterJoint();
            Capsule3D thighUpperShape = new Capsule3D(0.2, 0.1);
            thighUpperShape.getPosition().set(0.0195, robotSide.negateIfRightSide(0.086), -0.093);
            thighUpperShape.getAxis().set(new Vector3D(-0.15, robotSide.negateIfRightSide(-0.05), 1.0));
            collidables.add(new Collidable(thigh, collisionMask, collisionGroup, thighUpperShape, thighFrame));

            Capsule3D thighFrontShape = new Capsule3D(0.15, 0.095);
            thighFrontShape.getPosition().set(0.0424, robotSide.negateIfRightSide(0.081), -0.258);
            thighFrontShape.getAxis().set(new Vector3D(0.1, 0.0, 1.0));
            collidables.add(new Collidable(thigh, collisionMask, collisionGroup, thighFrontShape, thighFrame));

            Capsule3D thighLowerShape = new Capsule3D(0.25, 0.09);
            thighLowerShape.getPosition().set(0.017, robotSide.negateIfRightSide(0.091), -0.288);
            thighLowerShape.getAxis().set(new Vector3D(0.1, robotSide.negateIfRightSide(0.05), 1.0));
            collidables.add(new Collidable(thigh, collisionMask, collisionGroup, thighLowerShape, thighFrame));
         }

         { // Shin
            RigidBodyBasics shin = RobotCollisionModel.findJoint(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH), multiBodySystem).getSuccessor();
            MovingReferenceFrame shinFrame = shin.getParentJoint().getFrameAfterJoint();
            Capsule3D shinShape = new Capsule3D(0.3, 0.08);
            shinShape.getPosition().set(0.008, 0.0, -0.189);
            shinShape.getAxis().set(new Vector3D(0.1, 0.0, 1.0));
            collidables.add(new Collidable(shin, collisionMask, collisionGroup, shinShape, shinFrame));
         }

         { // Foot
            JointBasics ankleRoll = RobotCollisionModel.findJoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_ROLL), multiBodySystem);
            MovingReferenceFrame ankleRollFrame = ankleRoll.getFrameAfterJoint();
            Box3D footShape = new Box3D(0.275, 0.16, 0.095);
            footShape.getPosition().set(0.044, 0.0, -0.042);
            collidables.add(new Collidable(ankleRoll.getSuccessor(), collisionMask, collisionGroup, footShape, ankleRollFrame));

            for (RobotSide footSide : RobotSide.values)
            {
               PointShape3D footFrontCorner = new PointShape3D(new Point3D(0.17, footSide.negateIfRightSide(0.07), -0.09));
               collidables.add(new Collidable(ankleRoll.getSuccessor(), collisionMask, collisionGroup, footFrontCorner, ankleRollFrame));

               PointShape3D footBackCorner = new PointShape3D(new Point3D(-0.08, footSide.negateIfRightSide(0.07), -0.09));
               collidables.add(new Collidable(ankleRoll.getSuccessor(), collisionMask, collisionGroup, footBackCorner, ankleRollFrame));
            }
         }
      }

      return collidables;
   }
}
