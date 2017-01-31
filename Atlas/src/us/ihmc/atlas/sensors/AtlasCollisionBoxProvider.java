package us.ihmc.atlas.sensors;

import us.ihmc.modelFileLoaders.SdfLoader.JaxbSDFLoader;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.avatar.drcRobot.collisions.SDFCollisionBoxProvider;
import us.ihmc.ihmcPerception.depthData.collisionShapes.CollisionBox;
import us.ihmc.ihmcPerception.depthData.collisionShapes.CollisionCylinder;
import us.ihmc.ihmcPerception.depthData.collisionShapes.CollisionSphere;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;

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
         String kneeJoint = jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH);
         RigidBodyTransform kneePose = new RigidBodyTransform();
         kneePose.setRotationRollAndZeroTranslation(Math.PI / 2.0);
         kneePose.setTranslation(0.05, 0, 0.03);
         CollisionCylinder knee = new CollisionCylinder(kneePose, 0.11, 0.15 + 2.0 * extent);
         addCollisionShape(kneeJoint, knee);
         
         // Add boxes for the stupid hose
         
         String elbowJoint = jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH);
         RigidBodyTransform elbowBoxTransform = new RigidBodyTransform();
         elbowBoxTransform.setTranslation(0, robotSide.negateIfLeftSide(0.1), -0.2);
         CollisionBox stupidHoseBox = new CollisionBox(elbowBoxTransform, 0.15, 0.3, 0.15);
         addCollisionShape(elbowJoint, stupidHoseBox);
      }
      
      // Add wide backpack for hose
      
      String backRollJoint = jointMap.getSpineJointName(SpineJointName.SPINE_ROLL);
      RigidBodyTransform backPackTransform = new RigidBodyTransform();
      backPackTransform.setTranslation(-0.2, 0, 0.2);
      CollisionBox backPackHoseBox = new CollisionBox(backPackTransform, 0.2, 0.5, 0.3);
      addCollisionShape(backRollJoint, backPackHoseBox);
      
      RigidBodyTransform haloTransform = new RigidBodyTransform();
      haloTransform.setTranslation(0, 0, 1);
      CollisionCylinder halo = new CollisionCylinder(haloTransform, 0.35, 1.0);
      addCollisionShape(backRollJoint, halo);

   }

}
