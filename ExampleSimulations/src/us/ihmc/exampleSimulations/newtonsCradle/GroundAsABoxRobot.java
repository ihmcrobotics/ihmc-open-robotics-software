package us.ihmc.exampleSimulations.newtonsCradle;

import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.NullJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.physics.CollisionShape;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;
import us.ihmc.simulationconstructionset.physics.CollisionShapeFactory;
import us.ihmc.simulationconstructionset.physics.ScsCollisionDetector;

public class GroundAsABoxRobot extends Robot
{
   public GroundAsABoxRobot(ScsCollisionDetector collisionDetector)
   {
      super("GroundAsABoxRobot");
      NullJoint baseJoint = new NullJoint("base", new Vector3d(), this);

      //    FloatingJoint baseJoint = new FloatingJoint("base", new Vector3d(), this);
      Link baseLink = new Link("base");
      baseLink.setMassAndRadiiOfGyration(1000000000.0, 100.0, 100.0, 100.0);
      Graphics3DObject baseLinkGraphics = new Graphics3DObject();
      baseLinkGraphics.translate(0.0, 0.0, -0.05 / 2.0);
      baseLinkGraphics.addCube(4.0, 4.0, 0.05, YoAppearance.Green());
      baseLink.setLinkGraphics(baseLinkGraphics);
      baseLink.enableCollisions(100.0, this.getRobotsYoVariableRegistry());

      CollisionShapeFactory collisionShapeFactory = collisionDetector.getShapeFactory();
      CollisionShapeDescription<?> shapeDesc = collisionShapeFactory.createBox(4.0 / 2.0, 4.0 / 2.0, 0.05 / 2.0);

      RigidBodyTransform shapeToLinkTransform = new RigidBodyTransform();
      shapeToLinkTransform.setTranslation(new Vector3d(-0.0, 0.0, 0.0));
      CollisionShape groundShape = collisionShapeFactory.addShape(baseLink, shapeToLinkTransform, shapeDesc, true, 0xFFFFFFFF, 0xFFFFFFFF);
      groundShape.setIsGround(true);

      //    baseJoint.setVelocity(0.0, 0.0, 1.0);

      baseJoint.setLink(baseLink);
      this.addRootJoint(baseJoint);
      this.addStaticLink(baseLink);
   }
}
