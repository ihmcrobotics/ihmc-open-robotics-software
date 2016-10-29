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
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;
import us.ihmc.simulationconstructionset.physics.CollisionShapeFactory;
import us.ihmc.simulationconstructionset.physics.ScsCollisionDetector;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionDetector;

public class PileOfRandomObjectsRobot
{
   private final ScsCollisionDetector collisionDetector;

   private final ArrayList<Robot> robots = new ArrayList<Robot>();

   public PileOfRandomObjectsRobot()
   {
      int numberOfObjects = 200;

      //      collisionDetector = new GdxCollisionDetector(100.0);
      collisionDetector = new SimpleCollisionDetector();
      ((SimpleCollisionDetector) collisionDetector).setUseSimpleSpeedupMethod();

      CollisionShapeFactory collisionShapeFactory = collisionDetector.getShapeFactory();
      collisionShapeFactory.setMargin(0.002);

      Random random = new Random(1886L);

      createFallingObjects(numberOfObjects, collisionShapeFactory, random);
      //      createBoardFrame(collisionShapeFactory, random);
   }

   private void createFallingObjects(int numberOfObjects, CollisionShapeFactory collisionShapeFactory, Random random)
   {
      for (int i = 0; i < numberOfObjects; i++)
      {
         Robot robot = new Robot("RandomRobot" + i);

         Vector3d offset = new Vector3d(0.0, 0.0, 0.0);
         FloatingJoint floatingJoint = new FloatingJoint("object" + i, "object" + i, offset, robot);

         Link link;

         int shape = random.nextInt(4);
         if (shape == 0)
         {
            link = createRandomBox(collisionShapeFactory, random, i, robot);
         }
         else if (shape == 1)
         {
            link = createRandomSphere(collisionShapeFactory, random, i, robot);
         }
         else if (shape == 2)
         {
            link = createRandomCapsule(collisionShapeFactory, random, i, robot);
         }
         else
         {
            link = createRandomCylinder(collisionShapeFactory, random, i, robot);
         }

         floatingJoint.setLink(link);
         robot.addRootJoint(floatingJoint);

         double xyExtents = 0.25;
         double x = RandomTools.generateRandomDouble(random, -xyExtents, xyExtents);
         double y = RandomTools.generateRandomDouble(random, -xyExtents, xyExtents);
         double z = RandomTools.generateRandomDouble(random, 0.2, 6.0);

         double angleExtents = Math.PI / 2.0;
         double yaw = RandomTools.generateRandomDouble(random, -angleExtents, angleExtents);
         double pitch = RandomTools.generateRandomDouble(random, -angleExtents, angleExtents);
         double roll = RandomTools.generateRandomDouble(random, -angleExtents, angleExtents);

         floatingJoint.setPosition(x, y, z);
         floatingJoint.setYawPitchRoll(yaw, pitch, roll);

         this.robots.add(robot);
      }
   }

   private void createBoardFrame(CollisionShapeFactory collisionShapeFactory, Random random)
   {
      double boardZ = 0.14;

      FloatingJoint board0 = createContainerBoard("board0", collisionShapeFactory, random);
      board0.setPosition(0.51, 0.0, boardZ);
      board0.setYawPitchRoll(Math.PI / 2.0, 0.0, 0.0);

      FloatingJoint board1 = createContainerBoard("board1", collisionShapeFactory, random);
      board1.setPosition(0.0, 0.35, boardZ);
      board1.setYawPitchRoll(0.0, 0.0, 0.0);

      FloatingJoint board2 = createContainerBoard("board2", collisionShapeFactory, random);
      board2.setPosition(0.0, -0.35, boardZ);
      board2.setYawPitchRoll(0.0, 0.0, 0.0);

      FloatingJoint board3 = createContainerBoard("board3", collisionShapeFactory, random);
      board3.setPosition(-0.51, 0.0, boardZ);
      board3.setYawPitchRoll(Math.PI / 2.0, 0.0, 0.0);
   }

   private FloatingJoint createContainerBoard(String name, CollisionShapeFactory collisionShapeFactory, Random random)
   {
      Robot robot = new Robot(name);

      Vector3d offset = new Vector3d(0.0, 0.0, 0.0);
      FloatingJoint floatingJoint = new FloatingJoint(name, offset, robot);

      Link link = createContainerBoardLink(name, collisionShapeFactory, random, robot);
      floatingJoint.setLink(link);
      robot.addRootJoint(floatingJoint);
      this.robots.add(robot);

      return floatingJoint;
   }

   private Link createContainerBoardLink(String name, CollisionShapeFactory collisionShapeFactory, Random random, Robot robot)
   {
      double objectWidth = 0.2;
      double objectLength = 0.8;
      double objectHeight = 0.2;
      double objectMass = 10.0;

      Link link = new Link(name);
      link.setMassAndRadiiOfGyration(objectMass, objectLength / 2.0, objectWidth / 2.0, objectHeight / 2.0);
      link.setComOffset(0.0, 0.0, 0.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0.0, 0.0, -objectHeight / 2.0);
      AppearanceDefinition randomColor = YoAppearance.Aquamarine();

      linkGraphics.addCube(objectLength, objectWidth, objectHeight, randomColor);
      link.setLinkGraphics(linkGraphics);

      CollisionShapeDescription<?> shapeDesc = collisionShapeFactory.createBox(objectLength / 2.0, objectWidth / 2.0, objectHeight / 2.0);

      RigidBodyTransform shapeToLinkTransform = new RigidBodyTransform();
      shapeToLinkTransform.setTranslation(new Vector3d(0.0, 0.0, 0.0));
      collisionShapeFactory.addShape(link, shapeToLinkTransform, shapeDesc, false, 0xFFFFFFFF, 0xFFFFFFFF);
      link.enableCollisions(2.0, robot.getRobotsYoVariableRegistry());
      return link;
   }

   private Link createRandomBox(CollisionShapeFactory collisionShapeFactory, Random random, int i, Robot robot)
   {
      double objectLength = RandomTools.generateRandomDouble(random, 0.04, 0.1);
      double objectWidth = RandomTools.generateRandomDouble(random, 0.04, 0.2);
      double objectHeight = RandomTools.generateRandomDouble(random, 0.04, 0.1);
      double objectMass = RandomTools.generateRandomDouble(random, 0.2, 1.0);

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

   private Link createRandomSphere(CollisionShapeFactory collisionShapeFactory, Random random, int i, Robot robot)
   {
      double objectRadius = RandomTools.generateRandomDouble(random, 0.01, 0.05);
      double objectMass = RandomTools.generateRandomDouble(random, 0.2, 1.0);

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

   private Link createRandomCapsule(CollisionShapeFactory collisionShapeFactory, Random random, int i, Robot robot)
   {
      double objectRadius = RandomTools.generateRandomDouble(random, 0.01, 0.05);
      double objectHeight = 2.0 * objectRadius + RandomTools.generateRandomDouble(random, 0.02, 0.05);
      double objectMass = RandomTools.generateRandomDouble(random, 0.2, 1.0);

      Link link = new Link("object" + i);
      link.setMassAndRadiiOfGyration(objectMass, objectRadius / 2.0, objectRadius / 2.0, objectHeight / 2.0);
      link.setComOffset(0.0, 0.0, 0.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      AppearanceDefinition randomColor = YoAppearance.randomColor(random);
      linkGraphics.addCapsule(objectRadius, objectHeight, randomColor);
      link.setLinkGraphics(linkGraphics);

      CollisionShapeDescription<?> shapeDesc = collisionShapeFactory.createCapsule(objectRadius, objectHeight);

      RigidBodyTransform shapeToLinkTransform = new RigidBodyTransform();
      shapeToLinkTransform.setTranslation(new Vector3d(0.0, 0.0, 0.0));
      collisionShapeFactory.addShape(link, shapeToLinkTransform, shapeDesc, false, 0xFFFFFFFF, 0xFFFFFFFF);
      link.enableCollisions(2.0, robot.getRobotsYoVariableRegistry());
      return link;
   }

   private Link createRandomCylinder(CollisionShapeFactory collisionShapeFactory, Random random, int i, Robot robot)
   {
      double objectHeight = RandomTools.generateRandomDouble(random, 0.05, 0.15);
      double objectRadius = RandomTools.generateRandomDouble(random, 0.01, 0.10);
      double objectMass = RandomTools.generateRandomDouble(random, 0.2, 1.0);

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
