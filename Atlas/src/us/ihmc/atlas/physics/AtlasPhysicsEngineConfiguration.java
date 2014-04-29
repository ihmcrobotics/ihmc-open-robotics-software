package us.ihmc.atlas.physics;

import com.yobotics.simulationconstructionset.FloatingJoint;
import com.yobotics.simulationconstructionset.Joint;
import com.yobotics.simulationconstructionset.Link;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.physics.CollisionShapeDescription;
import com.yobotics.simulationconstructionset.physics.FactoryCollisionShape;
import com.yobotics.simulationconstructionset.physics.ScsCollisionConfigure;
import com.yobotics.simulationconstructionset.physics.ScsCollisionDetector;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.atlas.parameters.AtlasPhysicalProperties;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.TransformTools;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

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
   private SDFRobot robotModel;
   private AtlasPhysicalProperties properties = new AtlasPhysicalProperties();

   public AtlasPhysicsEngineConfiguration(DRCRobotJointMap jointMap , SDFRobot robotModel )
   {
      this.jointMap = jointMap;
      this.robotModel = robotModel;
   }

   @Override
   public void setup( Robot robot ,  ScsCollisionDetector collisionDetector) {


      String leftFootJointName = jointMap.getJointBeforeFootName(RobotSide.LEFT);
      String rightFootJointName = jointMap.getJointBeforeFootName(RobotSide.RIGHT);

      Joint leftFootJoint = robotModel.getJoint(leftFootJointName);
      Joint rightFootJoint = robotModel.getJoint(rightFootJointName);

      Link leftLink = leftFootJoint.getLink();
      Link rightLink = rightFootJoint.getLink();

      FactoryCollisionShape factoryShape = collisionDetector.getShapeFactory();
      CollisionShapeDescription collisionFoot = factoryShape.createBox(properties.foot_length/2,properties.foot_width/2,0.05);

//      public static final double ankleHeight = 0.084;
      Transform3D ankleToSole =  TransformTools.createTranslationTransform(new Vector3d(0.0, 0.0, 0.084));//AtlasPhysicalProperties.ankle_to_sole_frame_tranform;
      Transform3D soleToAnkle = new Transform3D();
      ankleToSole.invert(soleToAnkle);

      factoryShape.addShape(leftLink,soleToAnkle,collisionFoot,GROUP_FEET,GROUP_GROUND);
      factoryShape.addShape(rightLink,soleToAnkle,collisionFoot,GROUP_FEET,GROUP_GROUND);

      // HACK.  Add the ground plane
      CollisionShapeDescription collisionGround = factoryShape.createBox(20,20,0.1);

      FloatingJoint groundJoint = new FloatingJoint("ground", "ground", new Vector3d(), robot);
      Link linkGround = new Link("Ground Plane Hack");
      linkGround.setMass(1000);
      linkGround.physics.setMomentOfInertia(0.1 * 100, 0.1 * 100, 0.1 * 100);

      factoryShape.addShape(linkGround,null,collisionGround,GROUP_GROUND,0xFFFFFFFF);

      groundJoint.setLink(linkGround);
      groundJoint.setPositionAndVelocity(0.0, 0.0, -0.1, 0.0, 0.0, 0.0);
      groundJoint.setDynamic(false);
      robot.addRootJoint(groundJoint);
   }
}
