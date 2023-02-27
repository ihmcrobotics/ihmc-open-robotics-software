package us.ihmc.atlas;

import java.util.function.Consumer;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;

public class AtlasRobotDefinitionHandMutator implements Consumer<RobotDefinition>
{
   public AtlasRobotDefinitionHandMutator()
   {
   }

   @Override
   public void accept(RobotDefinition robotDefinition)
   {
      addCollision(robotDefinition.getRigidBodyDefinition("l_finger_1_link_0"),
                   new Quaternion(0, 0, 0, 1),
                   new Vector3D(0.0, 0.0, -0.012),
                   new Sphere3DDefinition(0.019));
      
      addCollision(robotDefinition.getRigidBodyDefinition("l_finger_1_link_1"),
                   new Quaternion(-0.268, 0, 0, 1),
                   new Vector3D(0.0, 0.0269, -0.0276),
                   new Box3DDefinition(0.0307, 0.05, 0.035));
      
      addCollision(robotDefinition.getRigidBodyDefinition("l_finger_1_link_2"),
                   new Quaternion(-0.28, 0, 0, 1),
                   new Vector3D(-0.0003, 0.02, -0.0175),
                   new Box3DDefinition(0.03, 0.038, 0.025));
      
      addCollision(robotDefinition.getRigidBodyDefinition("l_finger_1_link_3"),
                   new Quaternion(0, 0, 0, 1),
                   new Vector3D(0, 0.0214, 0.00392),
                   new Box3DDefinition(0.028, 0.0329, 0.008));
      
      addCollision(robotDefinition.getRigidBodyDefinition("l_finger_2_link_0"),
                   new Quaternion(0, 0, 0, 1),
                   new Vector3D(0.0, 0.0, -0.012),
                   new Sphere3DDefinition(0.019));
      
      addCollision(robotDefinition.getRigidBodyDefinition("l_finger_2_link_1"),
                   new Quaternion(-0.268, 0, 0, 1),
                   new Vector3D(0.0, 0.0269, -0.0276),
                   new Box3DDefinition(0.0307, 0.05, 0.035));
      
      addCollision(robotDefinition.getRigidBodyDefinition("l_finger_2_link_2"),
                   new Quaternion(-0.28, 0, 0, 1),
                   new Vector3D(-0.0003, 0.02, -0.0175),
                   new Box3DDefinition(0.03, 0.038, 0.025));
      
      addCollision(robotDefinition.getRigidBodyDefinition("l_finger_2_link_3"),
                   new Quaternion(0, 0, 0, 1),
                   new Vector3D(0, 0.0214, 0.00392),
                   new Box3DDefinition(0.028, 0.0329, 0.008));

      addCollision(robotDefinition.getRigidBodyDefinition("l_finger_middle_link_0"),
                   new Quaternion(0, 0, 0, 1),
                   new Vector3D(0.0, 0.0, 0.012),
                   new Sphere3DDefinition(0.019));
      
      addCollision(robotDefinition.getRigidBodyDefinition("l_finger_middle_link_1"),
                   new Quaternion(0.268, 0.0, 0.0, 1),
                   new Vector3D(0, 0.0269, 0.02764),
                   new Box3DDefinition(0.0307, 0.05, 0.035));

      addCollision(robotDefinition.getRigidBodyDefinition("l_finger_middle_link_2"),
                   new Quaternion(0.265, 0.0, 0.0, 1),
                   new Vector3D(9.0E-4, 0.019, 0.0167),
                   new Box3DDefinition(0.03, 0.038, 0.025));
      
      addCollision(robotDefinition.getRigidBodyDefinition("l_finger_middle_link_3"),
                   new Quaternion(0.0, 0.0, 0.0, 1),
                   new Vector3D(0.0, 0.0214, -0.00392),
                   new Box3DDefinition(0.028, 0.0329, 0.008));
      
      addCollision(robotDefinition.getRigidBodyDefinition("r_finger_1_link_0"),
                   new Quaternion(0, 0, 0, 1),
                   new Vector3D(0.0, 0.0, -0.012),
                   new Sphere3DDefinition(0.019));
      
      addCollision(robotDefinition.getRigidBodyDefinition("r_finger_1_link_1"),
                   new Quaternion(0.268, 0, 0, 1),
                   new Vector3D(0.0, -0.0269, -0.0276),
                   new Box3DDefinition(0.0307, 0.05, 0.035));
      
      addCollision(robotDefinition.getRigidBodyDefinition("r_finger_1_link_2"),
                   new Quaternion(0.28, 0, 0, 1),
                   new Vector3D(0.0003, -0.02, -0.0175),
                   new Box3DDefinition(0.03, 0.038, 0.025));
      
      addCollision(robotDefinition.getRigidBodyDefinition("r_finger_1_link_3"),
                   new Quaternion(0, 0, 0, 1),
                   new Vector3D(0, -0.0214, 0.00392),
                   new Box3DDefinition(0.028, 0.0329, 0.008));
      
      addCollision(robotDefinition.getRigidBodyDefinition("r_finger_2_link_0"),
                   new Quaternion(0, 0, 0, 1),
                   new Vector3D(0.0, 0.0, -0.012),
                   new Sphere3DDefinition(0.019));
      
      addCollision(robotDefinition.getRigidBodyDefinition("r_finger_2_link_1"),
                   new Quaternion(0.268, 0, 0, 1),
                   new Vector3D(0.0, -0.0269, -0.0276),
                   new Box3DDefinition(0.0307, 0.05, 0.035));
      
      addCollision(robotDefinition.getRigidBodyDefinition("r_finger_2_link_2"),
                   new Quaternion(0.28, 0, 0, 1),
                   new Vector3D(0.0003, -0.02, -0.0175),
                   new Box3DDefinition(0.03, 0.038, 0.025));
      
      addCollision(robotDefinition.getRigidBodyDefinition("r_finger_2_link_3"),
                   new Quaternion(0, 0, 0, 1),
                   new Vector3D(0, -0.0214, 0.00392),
                   new Box3DDefinition(0.028, 0.0329, 0.008));
      
      addCollision(robotDefinition.getRigidBodyDefinition("r_finger_middle_link_0"),
                   new Quaternion(0, 0, 0, 1),
                   new Vector3D(0.0, 0.0, 0.012),
                   new Sphere3DDefinition(0.019));

      addCollision(robotDefinition.getRigidBodyDefinition("r_finger_middle_link_1"),
                   new Quaternion(-0.268, 0.0, 0.0, 1),
                   new Vector3D(0, -0.0269, 0.02764),
                   new Box3DDefinition(0.0307, 0.05, 0.035));

      addCollision(robotDefinition.getRigidBodyDefinition("r_finger_middle_link_2"),
                   new Quaternion(-0.265, 0.0, 0.0, 1),
                   new Vector3D(-9.0E-4, -0.019, 0.0167),
                   new Box3DDefinition(0.03, 0.038, 0.025));
      
      addCollision(robotDefinition.getRigidBodyDefinition("r_finger_middle_link_3"),
                   new Quaternion(0.0, 0.0, 0.0, 1),
                   new Vector3D(0.0, -0.0214, -0.00392),
                   new Box3DDefinition(0.028, 0.0329, 0.008));
   }
   
   private void addCollision(RigidBodyDefinition rigidBodyDefinition, Quaternion quaternion, Vector3D translation, GeometryDefinition geometryShape)
   {
      if (rigidBodyDefinition == null)
         return;
      
      rigidBodyDefinition.getCollisionShapeDefinitions().clear();
      CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition(new RigidBodyTransform(quaternion, translation), geometryShape);
      collisionShapeDefinition.setName(rigidBodyDefinition.getName() + "_collision");
      rigidBodyDefinition.getCollisionShapeDefinitions().add(collisionShapeDefinition);
   }
}
