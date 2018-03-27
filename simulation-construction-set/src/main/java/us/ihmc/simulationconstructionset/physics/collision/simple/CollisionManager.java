package us.ihmc.simulationconstructionset.physics.collision.simple;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.Shape3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.RigidJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.physics.CollisionHandler;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;
import us.ihmc.simulationconstructionset.physics.CollisionShapeFactory;
import us.ihmc.simulationconstructionset.physics.ScsCollisionDetector;
import us.ihmc.simulationconstructionset.physics.collision.DefaultCollisionVisualizer;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class CollisionManager
{
   private ScsCollisionDetector collisionDetector;
   private final static double collisionDetectorMargin = 0.002;

   private CollisionHandler collisionHandler;
   private DefaultCollisionVisualizer collisionVisualizer = null;

   private final static double impulseScale = 100.0;
   private final static double forceScale = 100.0;
   private final static double collisionBallRadius = 0.01;
   private final int numberOfVectorsToCreate = 1000;

   private final TerrainObject3D environmentObject;
   
   public CollisionManager(CollisionHandler collisionHandler)
   {
      this(null, collisionHandler);
   }

   public CollisionManager(TerrainObject3D environmentObject, CollisionHandler collisionHandler)
   {
      this.environmentObject = environmentObject;
      this.collisionHandler = collisionHandler;
   }

   public void setUpCollisionVisualizer(SimulationConstructionSet scs)
   {
      this.collisionVisualizer = new DefaultCollisionVisualizer(forceScale, impulseScale, collisionBallRadius, scs, numberOfVectorsToCreate);

      if (collisionHandler != null)
         addListener();
      else
         throw new NullPointerException(getClass().getName() + "collision handler should be defined");
   }

   private void addListener()
   {
      if (collisionVisualizer != null)
         collisionHandler.addListener(collisionVisualizer);
   }

   public void setUpCollisionDetector(ScsCollisionDetector collisionDetector)
   {
      this.collisionDetector = collisionDetector;
      this.collisionDetector.initialize();
   }

   public void setUpEnvironment()
   {
      Robot environmentRobot;
      Joint environmentRobotRootJoint;
      Link environmentStaticLink;
      environmentRobot = new Robot("environmentRobot");
      environmentRobotRootJoint = new RigidJoint("envRootJoint", new Vector3D(), environmentRobot);
      environmentStaticLink = new Link("environmentLink");
      environmentRobotRootJoint.setLink(environmentStaticLink);

      if (environmentObject != null)
      {
         List<? extends Shape3D<?>> simpleShapes = environmentObject.getTerrainCollisionShapes();
         CollisionShapeFactory shapeFactory = getCollisionDetector().getShapeFactory();

         for (int i = 0; i < simpleShapes.size(); i++)
         {
            CollisionShapeDescription<?> collisionShapeDescription = shapeFactory.createSimpleCollisionShape(simpleShapes.get(i));

            RigidBodyTransform transform = new RigidBodyTransform();
            simpleShapes.get(i).getPose(transform);
            shapeFactory.addShape(environmentStaticLink, transform, collisionShapeDescription, true, 0xFFFF, 0xFFFF);
         }
      }

      environmentStaticLink.enableCollisions(10, getCollisionHandler(), environmentRobot.getRobotsYoVariableRegistry());
   }

   public void createCollisionShapesFromRobots(Robot[] robots)
   {
      ScsCollisionDetector collisionDetector;

      collisionDetector = new SimpleCollisionDetector();

      CollisionShapeFactory collisionShapeFactory = collisionDetector.getShapeFactory();
      collisionShapeFactory.setMargin(collisionDetectorMargin);

      for (int i = 0; i < robots.length; i++)
      {
         Robot robot = robots[i];
         createCollisionShapesFromLinks(robot, collisionShapeFactory, collisionHandler, robot.getRobotsYoVariableRegistry());
      }

      this.collisionDetector = collisionDetector;
   }

   public ScsCollisionDetector getCollisionDetector()
   {
      return collisionDetector;
   }

   public CollisionHandler getCollisionHandler()
   {
      return collisionHandler;
   }

   public DefaultCollisionVisualizer getCollisionVisualizer()
   {
      return collisionVisualizer;
   }

   private static void createCollisionShapesFromLinks(Robot robot, CollisionShapeFactory collisionShapeFactory, CollisionHandler collisionHandler,
                                                      YoVariableRegistry registry)
   {
      ArrayList<Joint> rootJoints = robot.getRootJoints();
      for (int i = 0; i < rootJoints.size(); i++)
      {
         Joint rootJoint = rootJoints.get(i);
         createCollisionShapesFromLinksRecursively(rootJoint, collisionShapeFactory, collisionHandler, registry);
      }
   }

   private static void createCollisionShapesFromLinksRecursively(Joint joint, CollisionShapeFactory collisionShapeFactory, CollisionHandler collisionHandler,
                                                                 YoVariableRegistry registry)
   {
      Link link = joint.getLink();
      ArrayList<CollisionMeshDescription> collisionMeshDescriptions = link.getCollisionMeshDescriptions();

      if (collisionMeshDescriptions != null)
      {
         for (int i = 0; i < collisionMeshDescriptions.size(); i++)
         {
            CollisionMeshDescription collisionMesh = collisionMeshDescriptions.get(i);
            collisionShapeFactory.addCollisionMeshDescription(link, collisionMesh);
         }

         link.enableCollisions(collisionHandler);
      }

      ArrayList<Joint> childrenJoints = joint.getChildrenJoints();
      for (int i = 0; i < childrenJoints.size(); i++)
      {
         Joint childJoint = childrenJoints.get(i);
         createCollisionShapesFromLinksRecursively(childJoint, collisionShapeFactory, collisionHandler, registry);
      }
   }

}
