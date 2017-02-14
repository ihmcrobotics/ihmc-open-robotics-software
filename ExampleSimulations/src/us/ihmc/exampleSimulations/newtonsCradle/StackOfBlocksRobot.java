package us.ihmc.exampleSimulations.newtonsCradle;

import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;
import us.ihmc.simulationconstructionset.physics.CollisionShapeFactory;
import us.ihmc.simulationconstructionset.physics.ScsCollisionDetector;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionDetector;

public class StackOfBlocksRobot
{
   private final ArrayList<Robot> robots = new ArrayList<Robot>();

   public StackOfBlocksRobot(int numberOfBlocks)
   {
      Random random = new Random(1886L);
      createFallingObjects(numberOfBlocks, random);
   }

   private void createFallingObjects(int numberOfObjects, Random random)
   {
      double objectHeight = 0.1;

      for (int i = 0; i < numberOfObjects; i++)
      {
         Robot robot = new Robot("StackOfBlocksRobot" + i);

         Vector3d offset = new Vector3d(0.0, 0.0, 0.0);
         FloatingJoint floatingJoint = new FloatingJoint("object" + i, "object" + i, offset, robot);
         Link link = createBox(objectHeight, random, i, robot);

         floatingJoint.setLink(link);
         robot.addRootJoint(floatingJoint);

         double x = 0.0;
         double y = 0.0;
         double z = (objectHeight * 1.05) * (i + 1.0);

         double yaw = 0.0;
         double pitch = RandomTools.generateRandomDouble(random, -Math.PI/90.0, Math.PI/90.0);
         double roll = RandomTools.generateRandomDouble(random, -Math.PI/90.0, Math.PI/90.0);

         floatingJoint.setPosition(x, y, z);
         floatingJoint.setYawPitchRoll(yaw, pitch, roll);

         this.robots.add(robot);
      }
   }

   private Link createBox(double objectHeight, Random random, int i, Robot robot)
   {
      double objectLength = 0.1;
      double objectWidth = 0.08;
      double objectMass = 0.2;

      Link link = new Link("object" + i);
      link.setMassAndRadiiOfGyration(objectMass, objectLength / 2.0, objectWidth / 2.0, objectHeight / 2.0);
      link.setComOffset(0.0, 0.0, 0.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0.0, 0.0, -objectHeight / 2.0);
      AppearanceDefinition randomColor = YoAppearance.randomColor(random);
      linkGraphics.addCube(objectLength, objectWidth, objectHeight, randomColor);
      link.setLinkGraphics(linkGraphics);


      CollisionMeshDescription collisionMeshDescription = new CollisionMeshDescription();
      collisionMeshDescription.addCubeReferencedAtCenter(objectLength, objectWidth, objectHeight);
      collisionMeshDescription.setCollisionGroup(0xff);
      collisionMeshDescription.setCollisionMask(0xff);
      link.addCollisionMesh(collisionMeshDescription);
      return link;
   }

   public ArrayList<Robot> getRobots()
   {
      return robots;
   }

}
