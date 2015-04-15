package us.ihmc.atlas.sensors;

import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.collisions.SDFCollisionBoxProvider;
import us.ihmc.ihmcPerception.depthData.collisionShapes.CollisionCylinder;
import us.ihmc.ihmcPerception.depthData.collisionShapes.CollisionSphere;
import us.ihmc.utilities.humanoidRobot.partNames.LegJointName;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;

public class AtlasCollisionBoxProvider extends SDFCollisionBoxProvider
{

   public AtlasCollisionBoxProvider(JaxbSDFLoader loader, AtlasJointMap jointMap)
   {
      super(loader, jointMap.getModelName());

      for (RobotSide robotSide : RobotSide.values)
      {
         
         // Add mittens around the hands. This is about the size of the robotiq hands
         String joint = jointMap.getJointBeforeHandName(robotSide);
         RigidBodyTransform mittenPose = new RigidBodyTransform();
         mittenPose.setTranslation(0, robotSide.negateIfRightSide(0.27), -0.005);
         CollisionSphere hand = new CollisionSphere(mittenPose, 0.12 + extent);
         addCollisionShape(joint, hand);
         
         // Add extra collision sphere around the knee, to avoid hitting points there on the real robot
         String kneeJoint = jointMap.getLegJointName(robotSide, LegJointName.KNEE);
         RigidBodyTransform kneePose = new RigidBodyTransform();
         kneePose.rotX(Math.PI / 2.0);
         kneePose.setTranslation(0.05, 0, 0.03);
         CollisionCylinder knee = new CollisionCylinder(kneePose, 0.11, 0.15 + 2.0 * extent);
         addCollisionShape(kneeJoint, knee);
      }

   }

}
