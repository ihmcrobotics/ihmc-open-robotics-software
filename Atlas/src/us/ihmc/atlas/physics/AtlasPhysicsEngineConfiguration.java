package us.ihmc.atlas.physics;

import javax.vecmath.Vector3d;

import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.physics.CollisionHandler;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;
import us.ihmc.simulationconstructionset.physics.CollisionShapeFactory;
import us.ihmc.simulationconstructionset.physics.ScsCollisionConfigure;
import us.ihmc.simulationconstructionset.physics.ScsCollisionDetector;

/**
 * @author Peter Abeles
 */
// TODO create ground plane
// TODO get link for left and right foot
// TODO create box for each foot
public class AtlasPhysicsEngineConfiguration implements ScsCollisionConfigure
{
   private static final int GROUP_GROUND = 0xFFFFFFFF;
   private static final int GROUP_FEET = 0xFFFFFFFF;

   private AtlasJointMap jointMap;
   private FloatingRootJointRobot sdfRobot;

   public AtlasPhysicsEngineConfiguration(AtlasJointMap jointMap, FloatingRootJointRobot sdfRobot)
   {
      this.jointMap = jointMap;
      this.sdfRobot = sdfRobot;
   }

   @Override
   public void setup(Robot robot, ScsCollisionDetector collisionDetector, CollisionHandler collisionHandler)
   {
      String leftFootJointName = jointMap.getJointBeforeFootName(RobotSide.LEFT);
      String rightFootJointName = jointMap.getJointBeforeFootName(RobotSide.RIGHT);

      Joint leftFootJoint = sdfRobot.getJoint(leftFootJointName);
      Joint rightFootJoint = sdfRobot.getJoint(rightFootJointName);

      Link leftLink = leftFootJoint.getLink();
      Link rightLink = rightFootJoint.getLink();

      CollisionShapeFactory factoryShape = collisionDetector.getShapeFactory();
      CollisionShapeDescription collisionFoot = factoryShape.createBox(jointMap.getPhysicalProperties().getActualFootLength() / 2, jointMap.getPhysicalProperties().getActualFootWidth() / 2, 0.05);

      //      public static final double ankleHeight = 0.084;
      RigidBodyTransform ankleToSole = TransformTools.createTranslationTransform(new Vector3d(0.0, 0.0, 0.084));//jointMap.getPhysicalProperties().getAnkle_to_sole_frame_tranform();
      RigidBodyTransform soleToAnkle = new RigidBodyTransform();
      ankleToSole.invert(soleToAnkle);

      factoryShape.addShape(leftLink, soleToAnkle, collisionFoot, false, GROUP_FEET, GROUP_GROUND);
      factoryShape.addShape(rightLink, soleToAnkle, collisionFoot, false, GROUP_FEET, GROUP_GROUND);

      // HACK.  Add the ground plane
      CollisionShapeDescription collisionGround = factoryShape.createBox(20, 20, 0.1);

      FloatingJoint groundJoint = new FloatingJoint("ground", "ground", new Vector3d(), robot);
      Link linkGround = new Link("Ground Plane Hack");
      linkGround.setMass(1000);
      linkGround.setMomentOfInertia(0.1 * 100, 0.1 * 100, 0.1 * 100);

      factoryShape.addShape(linkGround, null, collisionGround, true, GROUP_GROUND, 0xFFFFFFFF);

      groundJoint.setLink(linkGround);
      groundJoint.setPositionAndVelocity(0.0, 0.0, -0.1, 0.0, 0.0, 0.0);
      groundJoint.setDynamic(false);
      robot.addRootJoint(groundJoint);
   }
}
