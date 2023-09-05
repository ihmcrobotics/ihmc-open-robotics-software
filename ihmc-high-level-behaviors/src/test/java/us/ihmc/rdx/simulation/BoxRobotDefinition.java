package us.ihmc.rdx.simulation;

import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

public class BoxRobotDefinition extends RobotDefinition
{
   public BoxRobotDefinition()
   {
      super("Box");

      RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");
      SixDoFJointDefinition floatingJoint = new SixDoFJointDefinition(getRootJointName());
      RigidBodyDefinition box = createBoxRigidBody();
      box.setMass(10.0);
      box.getMomentOfInertia().setToDiagonal(0.1, 0.1, 0.1);

      setRootBodyDefinition(elevator);
      elevator.addChildJoint(floatingJoint);
      floatingJoint.setSuccessor(box);
   }

   public String getRootJointName()
   {
      return "rootJoint";
   }

   private final RigidBodyDefinition createBoxRigidBody()
   {
      RigidBodyDefinition ball = new RigidBodyDefinition("Box");
      GeometryDefinition geometryDefinition = new Box3DDefinition(0.3, 0.3, 0.3);
      MaterialDefinition materialDefinition = new MaterialDefinition(ColorDefinitions.Red());
      ball.addVisualDefinition(new VisualDefinition(geometryDefinition, materialDefinition));
      ball.addCollisionShapeDefinition(new CollisionShapeDefinition(geometryDefinition));
      return ball;
   }
}
