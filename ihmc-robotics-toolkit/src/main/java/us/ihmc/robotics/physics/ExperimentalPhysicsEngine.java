package us.ihmc.robotics.physics;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * Physics engine that simulates the dynamic behavior of multiple robots and their contact
 * interactions.
 * <p>
 * Its uses Featherstone forward dynamics algorithm to account for controller outputs, i.e. joint
 * efforts. The interactions are simulated using an impulse-based framework.
 * </p>
 * <p>
 * References that this physics engine is based on:
 * <ul>
 * <li>Multi-body forward dynamics algorithm: Featherstone, Roy. <i>Rigid Body Dynamics
 * Algorithms</i> Springer, 2008.
 * <li>Multi-body impulse response algorithm: Mirtich, Brian Vincent. <i>Impulse-based dynamic
 * simulation of rigid body systems</i>. University of California, Berkeley, 1996.
 * <li>Impulse-based contact resolution algorithm: Hwangbo, Jemin, Joonho Lee, and Marco Hutter.
 * <i>Per-contact iteration method for solving contact dynamics.</i> IEEE Robotics and Automation
 * Letters 3.2 (2018): 895-902.
 * </ul>
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class ExperimentalPhysicsEngine
{
   private static final String collidableVisualizerGroupName = "Physics Engine - Active Collidable";
   private final ReferenceFrame rootFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry physicsEngineRegistry = new YoVariableRegistry("PhysicsPlugins");
   private final YoGraphicsListRegistry physicsEngineGraphicsRegistry = new YoGraphicsListRegistry();
   private final List<PhysicsEngineRobotData> robotList = new ArrayList<>();
   private final List<MultiBodySystemStateReader> physicsOutputReaders = new ArrayList<>();

   private final List<Collidable> environmentCollidables = new ArrayList<>();

   private final SimpleCollisionDetection collisionDetectionPlugin;
   private final MultiRobotForwardDynamicsPlugin multiRobotPhysicsEnginePlugin;
   private final MultiRobotFirstOrderIntegrator integrationMethod = new MultiRobotFirstOrderIntegrator();

   private final CollidableListVisualizer environmentCollidableVisualizers;
   private final List<CollidableListVisualizer> robotCollidableVisualizers = new ArrayList<>();

   private boolean initialize = true;

   public ExperimentalPhysicsEngine()
   {
      collisionDetectionPlugin = new SimpleCollisionDetection(rootFrame);
      multiRobotPhysicsEnginePlugin = new MultiRobotForwardDynamicsPlugin(rootFrame, physicsEngineRegistry);
      AppearanceDefinition environmentCollidableAppearance = YoAppearance.AluminumMaterial();
      environmentCollidableAppearance.setTransparency(0.5);
      environmentCollidableVisualizers = new CollidableListVisualizer(collidableVisualizerGroupName,
                                                                      environmentCollidableAppearance,
                                                                      physicsEngineRegistry,
                                                                      physicsEngineGraphicsRegistry);
   }

   public void addRobot(String robotName, RigidBodyBasics rootBody, MultiBodySystemStateWriter controllerOutputWriter,
                        MultiBodySystemStateWriter robotInitialStateWriter, RobotCollisionModel robotCollisionModel,
                        MultiBodySystemStateReader physicsOutputReader)
   {
      PhysicsEngineRobotData robot = new PhysicsEngineRobotData(robotName,
                                                                rootBody,
                                                                robotInitialStateWriter,
                                                                controllerOutputWriter,
                                                                robotCollisionModel,
                                                                physicsEngineGraphicsRegistry);
      multiRobotPhysicsEnginePlugin.addRobot(robot);
      integrationMethod.addMultiBodySystem(robot.getMultiBodySystem());
      physicsOutputReader.setMultiBodySystem(robot.getMultiBodySystem());
      physicsOutputReaders.add(physicsOutputReader);
      AppearanceDefinition robotCollidableAppearance = YoAppearance.DarkGreen();
      robotCollidableAppearance.setTransparency(0.5);
      CollidableListVisualizer collidableVisualizers = new CollidableListVisualizer(collidableVisualizerGroupName,
                                                                                    robotCollidableAppearance,
                                                                                    robot.getRobotRegistry(),
                                                                                    physicsEngineGraphicsRegistry);
      robot.getCollidables().forEach(collidableVisualizers::addCollidable);
      robotCollidableVisualizers.add(collidableVisualizers);
      physicsEngineRegistry.addChild(robot.getRobotRegistry());
      robotList.add(robot);
   }

   public void addExternalWrenchReader(ExternalWrenchReader externalWrenchReader)
   {
      multiRobotPhysicsEnginePlugin.addExternalWrenchReader(externalWrenchReader);
   }

   public boolean initialize()
   {
      if (!initialize)
         return false;

      for (int i = 0; i < robotList.size(); i++)
      {
         PhysicsEngineRobotData robot = robotList.get(i);
         robot.initialize();
         physicsOutputReaders.get(i).read();
      }
      initialize = false;
      return true;
   }

   public void simulate(double dt, Vector3DReadOnly gravity)
   {
      if (initialize())
         return;

      for (PhysicsEngineRobotData robot : robotList)
      {
         robot.resetCalculators();
         robot.updateCollidableBoundingBoxes();
      }

      environmentCollidables.forEach(Collidable::updateBoundingBox);
      collisionDetectionPlugin.evaluationCollisions(robotList, () -> environmentCollidables);
      multiRobotPhysicsEnginePlugin.submitCollisions(collisionDetectionPlugin);
      multiRobotPhysicsEnginePlugin.doScience(dt, gravity);
      integrationMethod.integrate(dt);

      environmentCollidableVisualizers.update(collisionDetectionPlugin.getAllCollisions());
      robotCollidableVisualizers.forEach(visualizer -> visualizer.update(collisionDetectionPlugin.getAllCollisions()));

      for (int i = 0; i < robotList.size(); i++)
      {
         PhysicsEngineRobotData robot = robotList.get(i);
         robot.updateFrames();
         physicsOutputReaders.get(i).read();
      }
   }

   public void addEnvironmentCollidables(Collection<? extends Collidable> collidables)
   {
      environmentCollidables.addAll(collidables);
      collidables.forEach(environmentCollidableVisualizers::addCollidable);
   }

   public YoVariableRegistry getPhysicsEngineRegistry()
   {
      return physicsEngineRegistry;
   }

   public YoGraphicsListRegistry getPhysicsEngineGraphicsRegistry()
   {
      return physicsEngineGraphicsRegistry;
   }
}
