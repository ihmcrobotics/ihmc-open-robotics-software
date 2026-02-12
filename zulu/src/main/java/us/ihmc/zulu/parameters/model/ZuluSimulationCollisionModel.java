package us.ihmc.zulu.parameters.model;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.FrameCapsule3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBox3DBasics;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.geometry.shapes.FrameSTPBox3D;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.simulation.collision.Collidable;
import us.ihmc.scs2.simulation.collision.CollidableHelper;

import java.util.ArrayList;
import java.util.List;

/**
 * Collision model for Zulu used for simulating shape-to-shape collisions.
 */
public class ZuluSimulationCollisionModel implements RobotCollisionModel
{
   private final HumanoidJointNameMap jointMap;
   private final boolean useSTPShapesForSmoothContact;
   private final double stpMinimumMargin = 1.0e-5;
   private final double stpMaximumMargin = 4.0e-4;

   private CollidableHelper helper;
   private String[] otherCollisionMasks;
   private String robotCollisionMask;
   private long collisionMask;
   private long collisionGroup;

   public ZuluSimulationCollisionModel(HumanoidJointNameMap jointMap)
   {
      this(jointMap, true);
   }

   public ZuluSimulationCollisionModel(HumanoidJointNameMap jointMap, boolean useSTPShapesForSmoothContact)
   {
      this.jointMap = jointMap;
      this.useSTPShapesForSmoothContact = useSTPShapesForSmoothContact;
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

      RigidBodyBasics head = RobotCollisionModel.findRigidBody(jointMap.getHeadName(), multiBodySystem);
      RigidBodyBasics torso = RobotCollisionModel.findRigidBody(jointMap.getChestName(), multiBodySystem);
      RigidBodyBasics pelvis = RobotCollisionModel.findRigidBody(jointMap.getPelvisName(), multiBodySystem);
      // Head ---------------------------------------------------------------------
      if (head != null)
      {
         MovingReferenceFrame headFrame = head.getBodyFixedFrame();
         FrameCapsule3D headShapeMultisense = new FrameCapsule3D(headFrame, 0.08, 0.15);
         headShapeMultisense.getPosition().set(0.03, 0.0, 0.03);
         headShapeMultisense.getAxis().set(Axis3D.Z);
         collidables.add(new Collidable(head, collisionMask, collisionGroup, headShapeMultisense));
      }
      // Torso ---------------------------------------------------------------------
      if (torso != null)
      {
         MovingReferenceFrame torsoFrame = torso.getParentJoint().getFrameAfterJoint();
         // Capsule along the forward axis covering the bottom part of the "abdomen" and of the chest.
         FrameCapsule3D torsoShapeBottomCenter = new FrameCapsule3D(torsoFrame, 0.1, 0.10);
         torsoShapeBottomCenter.getPosition().set(-0.01, 0.0, 0.22);
         torsoShapeBottomCenter.getAxis().set(Axis3D.Z);
         collidables.add(new Collidable(torso, collisionMask, collisionGroup, torsoShapeBottomCenter));
      }
      // Pelvis ---------------------------------------------------------------------
      if (pelvis != null)
      {
         MovingReferenceFrame pelvisFrame = pelvis.getParentJoint().getFrameAfterJoint();
         FrameCapsule3D pelvisShape = new FrameCapsule3D(pelvisFrame, 0.05, 0.135);
         pelvisShape.getAxis().set(Axis3D.Y);
         pelvisShape.getPosition().set(-0.06, 0.0, -0.02);
         collidables.add(new Collidable(pelvis, collisionMask, collisionGroup, pelvisShape));
      }
      for (RobotSide robotSide : RobotSide.values)
      {
         { // Hand
            RigidBodyBasics hand = RobotCollisionModel.findRigidBody(jointMap.getHandName(robotSide), multiBodySystem);
            if (hand != null)
            {
               ReferenceFrame handFrame = hand.getParentJoint().getFrameAfterJoint();
               FrameCapsule3D handShapeKnob = new FrameCapsule3D(handFrame, 0.06, 0.06);
               handShapeKnob.getPosition().set(0.0, 0.0, 0.0);
               handShapeKnob.getAxis().set(Axis3D.Z);
               collidables.add(new Collidable(hand, collisionMask, collisionGroup, handShapeKnob));
            }
         }
         { // Foot
            JointBasics ankleRoll = RobotCollisionModel.findJoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_ROLL), multiBodySystem);
            if (ankleRoll != null)
            {
               MovingReferenceFrame ankleRollFrame = ankleRoll.getFrameAfterJoint();
               RigidBodyBasics foot = ankleRoll.getSuccessor();
               // Using a STP box so the sole is slightly rounded allowing for continuous and smooth contact with the ground.
               FrameBox3DBasics footShape = newBoxWithSTP(ankleRollFrame, new Vector3D(0.26, 0.14, 0.055));
               footShape.getPosition().set(0.045, 0.0, -0.05);
               collidables.add(new Collidable(foot, collisionMask, collisionGroup, footShape));
            }
         }
      }

      return collidables;
   }

   private FrameBox3DBasics newBoxWithSTP(ReferenceFrame referenceFrame, Vector3DReadOnly size)
   {
      if (useSTPShapesForSmoothContact)
      {
         FrameSTPBox3D stpShape = new FrameSTPBox3D(referenceFrame, size);
         stpShape.setMargins(stpMinimumMargin, stpMaximumMargin);
         return stpShape;
      }
      else
      {
         return new FrameBox3D(referenceFrame, size);
      }
   }
}
