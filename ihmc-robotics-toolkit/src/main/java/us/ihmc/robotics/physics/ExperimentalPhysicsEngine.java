package us.ihmc.robotics.physics;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;

import gnu.trove.list.linked.TDoubleLinkedList;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

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

   private final List<Collidable> environmentCollidables = new ArrayList<>();

   private final SimpleCollisionDetection collisionDetectionPlugin = new SimpleCollisionDetection(rootFrame);
   private final CollisionSlipEstimatorMap collisionSlipEstimator = new CollisionSlipEstimatorMap(rootFrame);
   private final MultiRobotForwardDynamicsPlugin multiRobotPhysicsEnginePlugin = new MultiRobotForwardDynamicsPlugin(rootFrame, physicsEngineRegistry);

   private final CollidableListVisualizer environmentCollidableVisualizers;
   private final List<CollidableListVisualizer> robotCollidableVisualizers = new ArrayList<>();

   private final YoDouble time = new YoDouble("physicsTime", physicsEngineRegistry);
   private final YoDouble rawTickDurationMilliseconds = new YoDouble("rawTickDurationMilliseconds", physicsEngineRegistry);
   private final YoDouble averageTickDurationMilliseconds = new YoDouble("averageTickDurationMilliseconds", physicsEngineRegistry);
   private final YoDouble rawRealTimeRate = new YoDouble("rawRealTimeRate", physicsEngineRegistry);
   private final YoDouble averageRealTimeRate = new YoDouble("averageRealTimeRate", physicsEngineRegistry);
   private final int averageWindow = 100;
   private final TDoubleLinkedList rawTickDurationBuffer = new TDoubleLinkedList();

   private boolean initialize = true;

   public ExperimentalPhysicsEngine()
   {
      AppearanceDefinition environmentCollidableAppearance = YoAppearance.AluminumMaterial();
      environmentCollidableAppearance.setTransparency(0.5);
      environmentCollidableVisualizers = new CollidableListVisualizer(collidableVisualizerGroupName,
                                                                      environmentCollidableAppearance,
                                                                      physicsEngineRegistry,
                                                                      physicsEngineGraphicsRegistry);
   }

   public void addEnvironmentCollidable(Collidable collidable)
   {
      environmentCollidables.add(collidable);
      environmentCollidableVisualizers.addCollidable(collidable);
   }

   public void addEnvironmentCollidables(Collection<? extends Collidable> collidables)
   {
      collidables.forEach(this::addEnvironmentCollidable);
   }

   public void addRobot(String robotName, RigidBodyBasics rootBody, MultiBodySystemStateWriter controllerOutputWriter,
                        MultiBodySystemStateWriter robotInitialStateWriter, RobotCollisionModel robotCollisionModel,
                        MultiBodySystemStateReader physicsOutputReader)
   {
      PhysicsEngineRobotData robot = new PhysicsEngineRobotData(robotName, rootBody, robotCollisionModel, physicsEngineGraphicsRegistry);
      YoVariableRegistry robotRegistry = robot.getRobotRegistry();
      robot.setRobotInitialStateWriter(robotInitialStateWriter);
      robot.setControllerOutputWriter(controllerOutputWriter);
      robot.addPhysicsOutputReader(physicsOutputReader);
      multiRobotPhysicsEnginePlugin.addRobot(robot);
      AppearanceDefinition robotCollidableAppearance = YoAppearance.DarkGreen();
      robotCollidableAppearance.setTransparency(0.5);
      CollidableListVisualizer collidableVisualizers = new CollidableListVisualizer(collidableVisualizerGroupName,
                                                                                    robotCollidableAppearance,
                                                                                    robotRegistry,
                                                                                    physicsEngineGraphicsRegistry);
      robot.getCollidables().forEach(collidableVisualizers::addCollidable);
      robotCollidableVisualizers.add(collidableVisualizers);
      physicsEngineRegistry.addChild(robotRegistry);
      robotList.add(robot);
   }

   public void addRobotPhysicsOutputReader(String robotName, MultiBodySystemStateReader physicsOutputReader)
   {
      PhysicsEngineRobotData physicsEngineRobotData = robotList.stream().filter(robot -> robot.getRobotName().equals(robotName)).findFirst().get();
      physicsEngineRobotData.addPhysicsOutputReader(physicsOutputReader);
   }

   public void addExternalWrenchReader(ExternalWrenchReader externalWrenchReader)
   {
      multiRobotPhysicsEnginePlugin.addExternalWrenchReader(externalWrenchReader);
   }

   public void setGlobalConstraintParameters(ConstraintParametersReadOnly parameters)
   {
      multiRobotPhysicsEnginePlugin.setGlobalConstraintParameters(parameters);
   }

   public void setGlobalContactParameters(ContactParametersReadOnly parameters)
   {
      multiRobotPhysicsEnginePlugin.setGlobalContactParameters(parameters);
   }

   public boolean initialize()
   {
      if (!initialize)
         return false;

      for (int i = 0; i < robotList.size(); i++)
      {
         PhysicsEngineRobotData robot = robotList.get(i);
         robot.initialize();
         robot.notifyPhysicsOutputReaders();
      }
      initialize = false;
      return true;
   }

   public void simulate(double dt, Vector3DReadOnly gravity)
   {
      if (initialize())
         return;

      long startTick = System.nanoTime();

      for (PhysicsEngineRobotData robot : robotList)
      {
         robot.resetCalculators();
         robot.updateCollidableBoundingBoxes();
      }

      environmentCollidables.forEach(collidable -> collidable.updateBoundingBox(rootFrame));

      collisionSlipEstimator.processPreCollisionDetection();
      if (multiRobotPhysicsEnginePlugin.hasGlobalContactParameters())
         collisionDetectionPlugin.setMinimumPenetration(multiRobotPhysicsEnginePlugin.getGlobalContactParameters().getMinimumPenetration());
      collisionDetectionPlugin.evaluationCollisions(robotList, () -> environmentCollidables);
      collisionSlipEstimator.processPostCollisionDetection(collisionDetectionPlugin.getAllCollisions());

      multiRobotPhysicsEnginePlugin.submitCollisions(collisionDetectionPlugin.getAllCollisions());
      multiRobotPhysicsEnginePlugin.doScience(time.getValue(), dt, gravity);

      environmentCollidableVisualizers.update(collisionDetectionPlugin.getAllCollisions());
      robotCollidableVisualizers.forEach(visualizer -> visualizer.update(collisionDetectionPlugin.getAllCollisions()));

      for (int i = 0; i < robotList.size(); i++)
      {
         PhysicsEngineRobotData robot = robotList.get(i);
         robot.updateFrames();
         robot.notifyPhysicsOutputReaders();
      }

      time.add(dt);

      long endTick = System.nanoTime();

      double dtMilliseconds = dt * 1.0e3;
      double tickDuration = (endTick - startTick) / 1.0e6;
      rawTickDurationMilliseconds.set(tickDuration);
      rawRealTimeRate.set(dtMilliseconds / tickDuration);
      rawTickDurationBuffer.add(tickDuration);

      if (rawTickDurationBuffer.size() >= averageWindow)
      {
         averageTickDurationMilliseconds.set(rawTickDurationBuffer.sum() / averageWindow);
         averageRealTimeRate.set(dtMilliseconds / averageTickDurationMilliseconds.getValue());
         rawTickDurationBuffer.removeAt(0);
      }
   }

   public List<String> getRobotNames()
   {
      return robotList.stream().map(PhysicsEngineRobotData::getRobotName).collect(Collectors.toList());
   }

   public double getTime()
   {
      return time.getValue();
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
