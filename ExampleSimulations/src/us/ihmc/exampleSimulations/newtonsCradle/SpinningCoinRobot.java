package us.ihmc.exampleSimulations.newtonsCradle;

import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.NullJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;
import us.ihmc.simulationconstructionset.physics.CollisionShapeFactory;
import us.ihmc.simulationconstructionset.physics.ScsCollisionDetector;
import us.ihmc.simulationconstructionset.physics.collision.gdx.GdxCollisionDetector;

public class SpinningCoinRobot extends Robot
{
   private final ScsCollisionDetector collisionDetector;
   private final double coinWidth = 0.00175; //quarter
   private final double coinRadius = 0.01213;
   private final double coinMass = 0.00567;
   private double spinningAngularVelocity = 30.0 * 2.0 * Math.PI;

   private final double margin = 0.0002;

   public SpinningCoinRobot()
   {
      super("SpinningCoin");

      collisionDetector = new GdxCollisionDetector(100.0);
      CollisionShapeFactory collisionShapeFactory = collisionDetector.getShapeFactory();
      collisionShapeFactory.setMargin(margin);

      Vector3d offset = new Vector3d(0.0, 0.0, 0.0);
      FloatingJoint floatingJoint = new FloatingJoint("root", offset, this);

      Link link = createCylinderCoin(collisionShapeFactory, this);

      floatingJoint.setLink(link);
      this.addRootJoint(floatingJoint);

      double x = 0.1;
      double y = 0.1;
      double z = 0.04;

      double yaw = 0.0;
      double pitch = 0.0;
      double roll = Math.PI/2.0; //1.2;

      floatingJoint.setPosition(x, y, z);
      floatingJoint.setYawPitchRoll(yaw, pitch, roll);

//      floatingJoint.setVelocity(new Vector3d(1.0, 0.0, 0.0));
      floatingJoint.setAngularVelocityInBody(new Vector3d(0.0, spinningAngularVelocity, 0.0));
      createGroundAsPartOfRobot(collisionShapeFactory);
   }

   private void createGroundAsPartOfRobot(CollisionShapeFactory collisionShapeFactory)
   {
      NullJoint baseJoint = new NullJoint("base", new Vector3d(), this);

//    FloatingJoint baseJoint = new FloatingJoint("base", new Vector3d(), this);
      Link baseLink = new Link("base");
      baseLink.setMassAndRadiiOfGyration(1000000000.0, 100.0, 100.0, 100.0);
      Graphics3DObject baseLinkGraphics = new Graphics3DObject();
      baseLinkGraphics.addCube(100.0, 100.0, 0.01);
      baseLink.setLinkGraphics(baseLinkGraphics);
      baseLink.enableCollisions(100.0, this.getRobotsYoVariableRegistry());

      CollisionShapeDescription shapeDesc = collisionShapeFactory.createBox(100.0, 100.0, 0.01);

      RigidBodyTransform shapeToLinkTransform = new RigidBodyTransform();
      shapeToLinkTransform.setTranslation(new Vector3d(0.0, 0.0, 0.0));
      collisionShapeFactory.addShape(baseLink, shapeToLinkTransform, shapeDesc, false, 0xFFFFFFFF, 0xFFFFFFFF);

      baseJoint.setLink(baseLink);
      this.addRootJoint(baseJoint);
   }

   private Link createCylinderCoin(CollisionShapeFactory collisionShapeFactory, Robot robot)
   {
      Link link = new Link("coin");
      link.setMassAndRadiiOfGyration(coinMass, coinRadius / 2.0, coinRadius / 2.0, coinWidth / 2.0);
      link.setComOffset(0.0, 0.0, 0.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0.0, 0.0, -coinWidth / 2.0);

      AppearanceDefinition color = YoAppearance.Purple();
      linkGraphics.addCylinder(coinWidth, coinRadius, color);
      link.setLinkGraphics(linkGraphics);

      linkGraphics.identity();
      linkGraphics.translate(0.0, 0.0, coinWidth / 2.0 );
      linkGraphics.addCube(coinRadius/3.0, coinRadius/3.0, coinWidth/4.0, YoAppearance.AliceBlue());
      linkGraphics.translate(0.0, 0.0, -coinWidth - coinWidth/4.0 );
      linkGraphics.addCube(coinRadius/3.0, coinRadius/3.0, coinWidth/4.0, YoAppearance.Gold());

      link.addEllipsoidFromMassProperties(YoAppearance.DarkGreen());
      CollisionShapeDescription shapeDesc = collisionShapeFactory.createCylinder(coinRadius, coinWidth);

      RigidBodyTransform shapeToLinkTransform = new RigidBodyTransform();
      shapeToLinkTransform.setTranslation(new Vector3d(0.0, 0.0, 0.0));
      collisionShapeFactory.addShape(link, shapeToLinkTransform, shapeDesc, false, 0xFFFFFFFF, 0xFFFFFFFF);
      link.enableCollisions(2.0, robot.getRobotsYoVariableRegistry());
      return link;
   }

   public ScsCollisionDetector getCollisionDetector()
   {
      return collisionDetector;
   }

}
