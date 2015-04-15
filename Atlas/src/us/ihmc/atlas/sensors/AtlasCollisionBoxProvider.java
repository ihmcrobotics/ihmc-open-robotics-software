package us.ihmc.atlas.sensors;

import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.collisions.SDFCollisionBoxProvider;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;

import com.jme3.scene.Mesh;
import com.jme3.scene.shape.Sphere;

public class AtlasCollisionBoxProvider extends SDFCollisionBoxProvider
{

   public AtlasCollisionBoxProvider(JaxbSDFLoader loader, AtlasJointMap jointMap)
   {
      super(loader, jointMap.getModelName());

      for (RobotSide robotSide : RobotSide.values)
      {
         Mesh hand = new Sphere(18, 18, 0.12f + extent);
         String joint = jointMap.getJointBeforeHandName(robotSide);
         RigidBodyTransform pose = new RigidBodyTransform();
         pose.setTranslation(0, robotSide.negateIfRightSide(0.27), -0.005);
         addMesh(joint, hand, pose);
      }

   }

}
