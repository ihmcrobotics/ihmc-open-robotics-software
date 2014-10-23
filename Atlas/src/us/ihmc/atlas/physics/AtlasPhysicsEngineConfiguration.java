package us.ihmc.atlas.physics;

import us.ihmc.utilities.math.geometry.RigidBodyTransform;

import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.atlas.parameters.AtlasPhysicalProperties;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.utilities.math.geometry.TransformTools;
import us.ihmc.utilities.robotSide.RobotSide;

import com.yobotics.simulationconstructionset.FloatingJoint;
import com.yobotics.simulationconstructionset.Joint;
import com.yobotics.simulationconstructionset.Link;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.physics.CollisionShapeDescription;
import com.yobotics.simulationconstructionset.physics.CollisionShapeFactory;
import com.yobotics.simulationconstructionset.physics.ScsCollisionConfigure;
import com.yobotics.simulationconstructionset.physics.ScsCollisionDetector;

/**
 * @author Peter Abeles
 */
// TODO create ground plane
// TODO get link for left and right foot
// TODO create box for each foot
public class AtlasPhysicsEngineConfiguration implements ScsCollisionConfigure
{
   public static int GROUP_GROUND = 0xFFFFFFFF;
   public static int GROUP_FEET = 0xFFFFFFFF;

   private DRCRobotJointMap jointMap;
   private SDFRobot sdfRobot;

   public AtlasPhysicsEngineConfiguration(DRCRobotJointMap jointMap, SDFRobot sdfRobot)
   {
      this.jointMap = jointMap;
      this.sdfRobot = sdfRobot;
   }

   @Override
   public void setup(Robot robot, ScsCollisionDetector collisionDetector)
   {

      String leftFootJointName = jointMap.getJointBeforeFootName(RobotSide.LEFT);
      String rightFootJointName = jointMap.getJointBeforeFootName(RobotSide.RIGHT);

      Joint leftFootJoint = sdfRobot.getJoint(leftFootJointName);
      Joint rightFootJoint = sdfRobot.getJoint(rightFootJointName);

      Link leftLink = leftFootJoint.getLink();
      Link rightLink = rightFootJoint.getLink();

      CollisionShapeFactory factoryShape = collisionDetector.getShapeFactory();
      CollisionShapeDescription collisionFoot = factoryShape.createBox(AtlasPhysicalProperties.footLength / 2, AtlasPhysicalProperties.footWidth / 2, 0.05);

      //      public static final double ankleHeight = 0.084;
      RigidBodyTransform ankleToSole = TransformTools.createTranslationTransform(new Vector3d(0.0, 0.0, 0.084));//AtlasPhysicalProperties.ankle_to_sole_frame_tranform;
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
