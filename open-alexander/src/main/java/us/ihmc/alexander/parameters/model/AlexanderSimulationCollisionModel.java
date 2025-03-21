package us.ihmc.alexander.parameters.model;

import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBox3DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
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
 * Collision model for Alexander used for simulating shape-to-shape collisions.
 * {@link DRCSCSInitialSetup#setUseExperimentalPhysicsEngine(boolean)}.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class AlexanderSimulationCollisionModel implements RobotCollisionModel
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

   public AlexanderSimulationCollisionModel(HumanoidJointNameMap jointMap)
   {
      this(jointMap, true);
   }

   public AlexanderSimulationCollisionModel(HumanoidJointNameMap jointMap, boolean useSTPShapesForSmoothContact)
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
      collisionMask = helper.getCollisionMask(robotCollisionMask);
      collisionGroup = helper.createCollisionGroup(otherCollisionMasks);

      for (RobotSide robotSide : RobotSide.values)
      {
         { // Foot
            JointBasics ankleRoll = RobotCollisionModel.findJoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_ROLL), multiBodySystem);
            MovingReferenceFrame ankleRollFrame = ankleRoll.getFrameAfterJoint();
            RigidBodyBasics foot = ankleRoll.getSuccessor();

            Vector3D footSize = new Vector3D(0.253, 0.125, 0.025);
            Point3D footPosition = new Point3D(0.053, 0.0, -0.075);
            // Using an STP box so the sole is slightly rounded allowing for continuous and smooth contact with the ground.
            FrameBox3DBasics footShape = newBoxWithSTP(ankleRollFrame, footSize);
            footShape.getPosition().set(footPosition);
            collidables.add(new Collidable(foot, collisionMask, collisionGroup, footShape));
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
