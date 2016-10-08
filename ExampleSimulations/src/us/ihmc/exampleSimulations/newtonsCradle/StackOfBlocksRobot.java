package us.ihmc.exampleSimulations.newtonsCradle;

import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.NullJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;
import us.ihmc.simulationconstructionset.physics.CollisionShapeFactory;
import us.ihmc.simulationconstructionset.physics.ScsCollisionDetector;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionDetector;

public class StackOfBlocksRobot
{
   private final ScsCollisionDetector collisionDetector;

   private final ArrayList<Robot> robots = new ArrayList<Robot>();

   public StackOfBlocksRobot()
   {
      int numberOfObjects = 10;

//      collisionDetector = new GdxCollisionDetector(100.0);
      collisionDetector = new SimpleCollisionDetector();

      CollisionShapeFactory collisionShapeFactory = collisionDetector.getShapeFactory();
      collisionShapeFactory.setMargin(0.002);

      Random random = new Random(1886L);

      createFallingObjects(numberOfObjects, collisionShapeFactory, random);
      createGroundAsABox(collisionShapeFactory);
   }

   private void createFallingObjects(int numberOfObjects, CollisionShapeFactory collisionShapeFactory, Random random)
   {
      double objectHeight = 0.2;
      
      for (int i = 0; i < numberOfObjects; i++)
      {
         Robot robot = new Robot("StackOfBlocksRobot" + i);

         Vector3d offset = new Vector3d(0.0, 0.0, 0.0);
         FloatingJoint floatingJoint = new FloatingJoint("object" + i, "object" + i, offset, robot);

//         Link link;
//         
//         if ((i ==1 ) || (i ==2))
//         {
//         link = createSphere(objectHeight, collisionShapeFactory, random, i, robot);
//            
//         }
//         else
//         {
//            link = createCylinder(objectHeight, collisionShapeFactory, random, i, robot);
//            
//         }
         Link link = createBox(objectHeight, collisionShapeFactory, random, i, robot);

         floatingJoint.setLink(link);
         robot.addRootJoint(floatingJoint);

         double x = 0.0;
         double y = 0.0;
         double z = (objectHeight * 1.1) * (i + 1.0);

         double yaw = 0.0;
         double pitch = RandomTools.generateRandomDouble(random, -Math.PI/30.0, Math.PI/30.0);
         double roll = RandomTools.generateRandomDouble(random, -Math.PI/30.0, Math.PI/30.0);

         floatingJoint.setPosition(x, y, z);
         floatingJoint.setYawPitchRoll(yaw, pitch, roll);

         this.robots.add(robot);
      }
   }

   private void createGroundAsABox(CollisionShapeFactory collisionShapeFactory)
   {
      Robot baseRobot = new Robot("BaseRobot");
      NullJoint baseJoint = new NullJoint("base", new Vector3d(), baseRobot);

//    FloatingJoint baseJoint = new FloatingJoint("base", new Vector3d(), this);
      Link baseLink = new Link("base");
      baseLink.setMassAndRadiiOfGyration(1000000000.0, 100.0, 100.0, 100.0);
      Graphics3DObject baseLinkGraphics = new Graphics3DObject();
      baseLinkGraphics.translate(0.0, 0.0, -0.01 / 2.0);
      baseLinkGraphics.addCube(4.0, 4.0, 0.01, YoAppearance.Green());
      baseLink.setLinkGraphics(baseLinkGraphics);
      baseLink.enableCollisions(100.0, baseRobot.getRobotsYoVariableRegistry());

      CollisionShapeDescription<?> shapeDesc = collisionShapeFactory.createBox(4.0 / 2.0, 4.0 / 2.0, 0.01 / 2.0);

      RigidBodyTransform shapeToLinkTransform = new RigidBodyTransform();
      shapeToLinkTransform.setTranslation(new Vector3d(-0.005, 0.0, 0.0));
      collisionShapeFactory.addShape(baseLink, shapeToLinkTransform, shapeDesc, true, 0xFFFFFFFF, 0xFFFFFFFF);

//    baseJoint.setVelocity(0.0, 0.0, 1.0);

      baseJoint.setLink(baseLink);
      baseRobot.addRootJoint(baseJoint);

      baseRobot.addStaticLink(baseLink);
      this.robots.add(baseRobot);
   }

 
   private Link createBox(double objectHeight, CollisionShapeFactory collisionShapeFactory, Random random, int i, Robot robot)
   {
      double objectLength = 0.1;
      double objectWidth = 0.05;
      double objectMass = 0.2;

      Link link = new Link("object" + i);
      link.setMassAndRadiiOfGyration(objectMass, objectLength / 2.0, objectWidth / 2.0, objectHeight / 2.0);
      link.setComOffset(0.0, 0.0, 0.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0.0, 0.0, -objectHeight / 2.0);
      AppearanceDefinition randomColor = YoAppearance.randomColor(random);
      linkGraphics.addCube(objectLength, objectWidth, objectHeight, randomColor);
      link.setLinkGraphics(linkGraphics);

      CollisionShapeDescription<?> shapeDesc = collisionShapeFactory.createBox(objectLength / 2.0, objectWidth / 2.0, objectHeight / 2.0);

      RigidBodyTransform shapeToLinkTransform = new RigidBodyTransform();
      shapeToLinkTransform.setTranslation(new Vector3d(0.0, 0.0, 0.0));
      collisionShapeFactory.addShape(link, shapeToLinkTransform, shapeDesc, false, 0xFFFFFFFF, 0xFFFFFFFF);
      link.enableCollisions(2.0, robot.getRobotsYoVariableRegistry());
      return link;
   }
   
   private Link createSphere(double objectHeight, CollisionShapeFactory collisionShapeFactory, Random random, int i, Robot robot)
   {
      double objectRadius = objectHeight/2.0;
      double objectMass = 0.2;

      Link link = new Link("object" + i);
      link.setMassAndRadiiOfGyration(objectMass, objectRadius / 2.0, objectRadius / 2.0, objectRadius / 2.0);
      link.setComOffset(0.0, 0.0, 0.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      AppearanceDefinition randomColor = YoAppearance.randomColor(random);
      linkGraphics.addSphere(objectRadius, randomColor);
      link.setLinkGraphics(linkGraphics);

      CollisionShapeDescription<?> shapeDesc = collisionShapeFactory.createSphere(objectRadius);

      RigidBodyTransform shapeToLinkTransform = new RigidBodyTransform();
      shapeToLinkTransform.setTranslation(new Vector3d(0.0, 0.0, 0.0));
      collisionShapeFactory.addShape(link, shapeToLinkTransform, shapeDesc, false, 0xFFFFFFFF, 0xFFFFFFFF);
      link.enableCollisions(2.0, robot.getRobotsYoVariableRegistry());
      return link;
   }

   private Link createCylinder(double objectHeight, CollisionShapeFactory collisionShapeFactory, Random random, int i, Robot robot)
   {
      double objectRadius = 0.2;
      double objectMass = 0.2;

      Link link = new Link("object" + i);
      link.setMassAndRadiiOfGyration(objectMass, objectRadius / 2.0, objectRadius / 2.0, objectHeight / 2.0);
      link.setComOffset(0.0, 0.0, 0.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0.0, 0.0, -objectHeight / 2.0);

      AppearanceDefinition randomColor = YoAppearance.randomColor(random);
      linkGraphics.addCylinder(objectHeight, objectRadius, randomColor);
      link.setLinkGraphics(linkGraphics);

      CollisionShapeDescription<?> shapeDesc = collisionShapeFactory.createCylinder(objectRadius, objectHeight);

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
