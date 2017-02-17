package us.ihmc.simulationconstructionset.physics.collision;

import java.util.ArrayList;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.physics.CollisionHandler;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;
import us.ihmc.simulationconstructionset.physics.CollisionShapeFactory;
import us.ihmc.simulationconstructionset.physics.Contacts;
import us.ihmc.simulationconstructionset.physics.ScsCollisionDetector;
import us.ihmc.simulationconstructionset.physics.collision.gdx.GdxCollisionDetector;

public class SCSCollisionDetectorEvaluator
{

   public SCSCollisionDetectorEvaluator()
   {
      double lengthX = 1.0;
      double widthY = 1.0;
      double heightZ = 1.0;

      double halfX = lengthX/2.0;
      double halfY = widthY/2.0;
      double halfZ = heightZ/2.0;

      Robot robot = new Robot("robot");
      IntegerYoVariable numberOfCollisions = new IntegerYoVariable("numberOfCollisions", robot.getRobotsYoVariableRegistry());

      ArrayList<YoGraphicPosition> pointsOnA = new ArrayList<>();
      ArrayList<YoGraphicPosition> pointsOnB = new ArrayList<>();

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      for (int i=0; i<4; i++)
      {
         YoGraphicPosition pointOnAViz = new YoGraphicPosition("pointOnA", "_" + i, robot.getRobotsYoVariableRegistry(), 0.03, YoAppearance.Purple());
         YoGraphicPosition pointOnBViz = new YoGraphicPosition("pointOnB", "_" + i, robot.getRobotsYoVariableRegistry(), 0.03, YoAppearance.Gold());

         yoGraphicsListRegistry.registerYoGraphic("Collision", pointOnAViz);
         yoGraphicsListRegistry.registerYoGraphic("Collision", pointOnBViz);

         pointsOnA.add(pointOnAViz);
         pointsOnB.add(pointOnBViz);
      }

      robot.addDynamicGraphicObjectsListRegistry(yoGraphicsListRegistry );
      // Create the joints:
      FloatingJoint jointOne = new FloatingJoint("one", "cubeOne", new Vector3D(), robot);
      Link linkOne = new Link("CubeOne");

      FloatingJoint jointTwo = new FloatingJoint("two", "cubeTwo", new Vector3D(), robot);
      Link linkTwo = new Link("CubeTwo");

      // Set mass parameters
      double mass = 1.0;

      double epsilon = 0.3;
      double mu = 0.7;
      CollisionHandler collisionHandler = new DefaultCollisionHandler(epsilon, mu);
 
      linkOne.setMass(mass);
      linkOne.setMomentOfInertia(0.1 * mass, 0.1 * mass, 0.1 * mass);
      linkOne.enableCollisions(4, collisionHandler, robot.getRobotsYoVariableRegistry());

      linkTwo.setMass(mass);
      linkTwo.setMomentOfInertia(0.1 * mass, 0.1 * mass, 0.1 * mass);
      linkTwo.enableCollisions(4, collisionHandler, robot.getRobotsYoVariableRegistry());

      // Graphics
      Graphics3DObject linkOneGraphics = new Graphics3DObject();
      linkOneGraphics.translate(0.0, 0.0, -halfZ);
      linkOneGraphics.addCube(lengthX, widthY, heightZ, YoAppearance.Red());
      linkOne.setLinkGraphics(linkOneGraphics);

      Graphics3DObject linkTwoGraphics = new Graphics3DObject();
      linkTwoGraphics.translate(0.0, 0.0, -halfZ);
      linkTwoGraphics.addCube(lengthX, widthY, heightZ, YoAppearance.Green());
      linkTwo.setLinkGraphics(linkTwoGraphics);

      // Collison Detector
      double worldRadius = 100.0;
      ScsCollisionDetector collisionDetector = new GdxCollisionDetector(worldRadius);

      CollisionShapeFactory factory = collisionDetector.getShapeFactory();
      factory.setMargin(0.002);
      CollisionShapeDescription shapeDescriptionOne = factory.createBox(halfX, halfY, halfZ);

      int collisionGroup = 0xFFFFFFFF;
      int collisionMask = 0xFFFFFFFF;
      RigidBodyTransform shapeToLinkOne = new RigidBodyTransform();

      factory.addShape(linkOne, shapeToLinkOne, shapeDescriptionOne, false, collisionGroup, collisionMask);

      CollisionShapeDescription shapeDescriptionTwo = factory.createBox(halfX, halfY, halfZ);
//      CollisionShapeDescription shapeDescriptionTwo = factory.createSphere(halfX);
      RigidBodyTransform shapeToLinkTwo = new RigidBodyTransform();
      factory.addShape(linkTwo, shapeToLinkTwo, shapeDescriptionTwo, false, collisionGroup, collisionMask);

      // Assemble
      jointOne.setLink(linkOne);
      jointTwo.setLink(linkTwo);

      robot.addRootJoint(jointOne);
      robot.addRootJoint(jointTwo);

      robot.setGravity(0.0);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      scs.setGroundVisible(false);
      scs.startOnAThread();


      double x = -1.2;
      double y = 0.0;
      double z = 0.0;

      CollisionDetectionResult result = new CollisionDetectionResult();

      for (int i=0; i<4000; i++)
      {
         jointOne.setPosition(x, y, z);
         robot.update();

         x = x + 0.001;
         robot.setTime(robot.getTime() + 0.001);

         result.clear();
         collisionDetector.performCollisionDetection(result);

         numberOfCollisions.set(result.getNumberOfCollisions());

         for (int j=0; j<pointsOnA.size(); j++)
         {
            pointsOnA.get(j).setPosition(Double.NaN, Double.NaN, Double.NaN);
            pointsOnB.get(j).setPosition(Double.NaN, Double.NaN, Double.NaN);
         }

         int vizIndex = 0;
         for (int j=0; j<numberOfCollisions.getIntegerValue(); j++)
         {
            Contacts collision = result.getCollision(j);

            int numberOfContacts = collision.getNumberOfContacts();

            for (int k=0; k<numberOfContacts; k++)
            {
               Point3D pointOnA = new Point3D();
               collision.getWorldA(k, pointOnA);
               pointsOnA.get(vizIndex).setPosition(pointOnA);

               Point3D pointOnB = new Point3D();
               collision.getWorldB(k, pointOnB);
               pointsOnB.get(vizIndex).setPosition(pointOnB);

               vizIndex++;
            }
         }

         scs.tickAndUpdate();
      }

   }

   public static void main(String[] args)
   {
      new SCSCollisionDetectorEvaluator();
   }
}
