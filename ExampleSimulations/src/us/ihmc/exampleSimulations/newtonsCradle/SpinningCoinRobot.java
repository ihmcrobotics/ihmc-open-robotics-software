package us.ihmc.exampleSimulations.newtonsCradle;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;
import us.ihmc.simulationconstructionset.physics.CollisionShapeFactory;
import us.ihmc.simulationconstructionset.physics.ScsCollisionDetector;
import us.ihmc.simulationconstructionset.physics.collision.gdx.GdxCollisionDetector;

public class SpinningCoinRobot
{
   private final ScsCollisionDetector collisionDetector;

   private final ArrayList<Robot> robots = new ArrayList<Robot>();

   public SpinningCoinRobot()
   {
      YoVariableRegistry registry = new YoVariableRegistry("Collision");
      collisionDetector = new GdxCollisionDetector(registry, 100.0);
      CollisionShapeFactory collisionShapeFactory = collisionDetector.getShapeFactory();
      collisionShapeFactory.setMargin(0.002);

      Robot robot = new Robot("SpinningCoin");

      Vector3d offset = new Vector3d(0.0, 0.0, 0.0);
      FloatingJoint floatingJoint = new FloatingJoint("root", offset, robot);

      Link link = createCylinderCoin(collisionShapeFactory, robot);

      floatingJoint.setLink(link);
      robot.addRootJoint(floatingJoint);

      double x = 0.1;
      double y = 0.1;
      double z = 0.1;

      double yaw = 0.0;
      double pitch = 0.0;
      double roll = 0.1;

      floatingJoint.setPosition(x, y, z);
      floatingJoint.setYawPitchRoll(yaw, pitch, roll);

      robot.addYoVariableRegistry(registry);
      this.robots.add(robot);
   }

   private Link createCylinderCoin(CollisionShapeFactory collisionShapeFactory, Robot robot)
   {
      double objectHeight = 0.005;
      double objectRadius = 0.2;
      double objectMass = 0.1;

      Link link = new Link("coin");
      link.setMassAndRadiiOfGyration(objectMass, objectRadius / 2.0, objectRadius / 2.0, objectRadius / 2.0);
      link.setComOffset(0.0, 0.0, 0.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0.0, 0.0, -objectHeight / 2.0);

      AppearanceDefinition color = YoAppearance.Purple();
      linkGraphics.addCylinder(objectHeight, objectRadius, color);
      link.setLinkGraphics(linkGraphics);

      CollisionShapeDescription shapeDesc = collisionShapeFactory.createCylinder(objectRadius, objectHeight);

      RigidBodyTransform shapeToLinkTransform = new RigidBodyTransform();
      shapeToLinkTransform.setTranslation(new Vector3d(0.0, 0.0, 0.0));
      collisionShapeFactory.addShape(link, shapeToLinkTransform, shapeDesc, false, 0xFFFFFFFF, 0xFFFFFFFF);
      link.enableCollisions(2.0, robot.getRobotsYoVariableRegistry());
      return link;
   }

   public ArrayList<Robot> getRobots()
   {
      return robots;
   }

   public ScsCollisionDetector getCollisionDetector()
   {
      return collisionDetector;
   }

}
