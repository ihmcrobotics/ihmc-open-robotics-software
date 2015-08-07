package us.ihmc.exampleSimulations.newtonsCradle;

import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.NullJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;
import us.ihmc.simulationconstructionset.physics.CollisionShapeFactory;
import us.ihmc.simulationconstructionset.physics.ScsCollisionDetector;
import us.ihmc.simulationconstructionset.physics.collision.gdx.GdxCollisionDetector;
import us.ihmc.tools.random.RandomTools;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class PileOfRandomObjectsRobot
{
   private final ScsCollisionDetector collisionDetector;

   private final ArrayList<Robot> robots = new ArrayList<Robot>();
   
   public PileOfRandomObjectsRobot()
   {
      int numberOfObjects = 200;

      YoVariableRegistry registry = new YoVariableRegistry("Collision");
      collisionDetector = new GdxCollisionDetector(registry, 100.0);
      CollisionShapeFactory collisionShapeFactory = collisionDetector.getShapeFactory();
      collisionShapeFactory.setMargin(0.002);

      Random random = new Random(1886L);

      for (int i = 0; i < numberOfObjects; i++)
      {
         Robot robot = new Robot("RandomRobot" + i);
         
         double objectWidth = RandomTools.generateRandomDouble(random, 0.01, 0.1);
         double objectLength = RandomTools.generateRandomDouble(random, 0.01, 0.1);
         double objectHeight = RandomTools.generateRandomDouble(random, 0.01, 0.1);
         double objectMass = RandomTools.generateRandomDouble(random, 0.2, 1.0);

         Vector3d offset = new Vector3d(0.0, 0.0, 0.0);
         FloatingJoint floatingJoint = new FloatingJoint("object" + i, "object" + i, offset, robot);

         Link link = new Link("object" + i);
         link.setMassAndRadiiOfGyration(objectMass, objectLength / 2.0, objectWidth / 2.0, objectHeight / 2.0);
         link.setComOffset(0.0, 0.0, 0.0);

         Graphics3DObject linkGraphics = new Graphics3DObject();
         linkGraphics.translate(0.0, 0.0, -objectHeight / 2.0);
         AppearanceDefinition randomColor = YoAppearance.randomColor(random);
         linkGraphics.addCube(objectLength, objectWidth, objectHeight, randomColor);
         link.setLinkGraphics(linkGraphics);

         CollisionShapeDescription shapeDesc = collisionShapeFactory.createBox(objectLength / 2.0, objectWidth / 2.0, objectHeight / 2.0);
         RigidBodyTransform shapeToLinkTransform = new RigidBodyTransform();
         shapeToLinkTransform.setTranslation(new Vector3d(0.0, 0.0, 0.0));
         collisionShapeFactory.addShape(link, shapeToLinkTransform, shapeDesc, false, 0xFFFFFFFF, 0xFFFFFFFF);
         link.enableCollisions(2.0, robot.getRobotsYoVariableRegistry());

         floatingJoint.setLink(link);
         robot.addRootJoint(floatingJoint);

         double x = RandomTools.generateRandomDouble(random, -0.1, 0.1);
         double y = RandomTools.generateRandomDouble(random, -0.1, 0.1);
         double z = RandomTools.generateRandomDouble(random, 0.2, 3.0);

         double yaw = RandomTools.generateRandomDouble(random, -Math.PI/2.0, Math.PI/2.0);
         double pitch = RandomTools.generateRandomDouble(random, -Math.PI/2.0, Math.PI/2.0);
         double roll = RandomTools.generateRandomDouble(random, -Math.PI/2.0, Math.PI/2.0);

         floatingJoint.setPosition(x, y, z);
         floatingJoint.setYawPitchRoll(yaw, pitch, roll);
        
         this.robots.add(robot);
      }


      Robot baseRobot = new Robot("BaseRobot");
      NullJoint baseJoint = new NullJoint("base", new Vector3d(), baseRobot);

//    FloatingJoint baseJoint = new FloatingJoint("base", new Vector3d(), this);
      Link baseLink = new Link("base");
      baseLink.setMassAndRadiiOfGyration(1000000000.0, 100.0, 100.0, 100.0);
      Graphics3DObject baseLinkGraphics = new Graphics3DObject();
      baseLinkGraphics.translate(0.0, 0.0, -0.01);
      baseLinkGraphics.addCube(100.0, 100.0, 0.01, YoAppearance.Green());
      baseLink.setLinkGraphics(baseLinkGraphics);
      baseLink.enableCollisions(100.0, baseRobot.getRobotsYoVariableRegistry());

      CollisionShapeDescription shapeDesc = collisionShapeFactory.createBox(100.0, 100.0, 0.01 / 2.0);

      RigidBodyTransform shapeToLinkTransform = new RigidBodyTransform();
      shapeToLinkTransform.setTranslation(new Vector3d(-0.005, 0.0, 0.0));
      collisionShapeFactory.addShape(baseLink, shapeToLinkTransform, shapeDesc, true, 0xFFFFFFFF, 0xFFFFFFFF);

//    baseJoint.setVelocity(0.0, 0.0, 1.0);

      baseJoint.setLink(baseLink);
      baseRobot.addRootJoint(baseJoint);

      baseRobot.addStaticLink(baseLink);
      this.robots.add(baseRobot);
      
      baseRobot.getRobotsYoVariableRegistry().addChild(registry);
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
