package us.ihmc.robotics.physics;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;

import gnu.trove.list.linked.TDoubleLinkedList;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
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

   private final YoRegistry registry = new YoRegistry("PhysicsPlugins");
   private final YoGraphicsListRegistry physicsEngineGraphicsRegistry = new YoGraphicsListRegistry();
   private final List<PhysicsEngineRobotData> robotList = new ArrayList<>();
   private final Map<RigidBodyBasics, PhysicsEngineRobotData> robotMap = new HashMap<>();
   private final YoMultiContactImpulseCalculatorPool multiContactImpulseCalculatorPool;
   private final List<ExternalWrenchReader> externalWrenchReaders = new ArrayList<>();
   private final List<ExternalWrenchProvider> externalWrenchProviders = new ArrayList<>();
   private final List<InertialMeasurementReader> inertialMeasurementReaders = new ArrayList<>();

   private List<MultiRobotCollisionGroup> collisionGroups;

   private final List<Collidable> environmentCollidables = new ArrayList<>();

   private final SimpleCollisionDetection collisionDetectionPlugin = new SimpleCollisionDetection(rootFrame);

   private final CollidableListVisualizer environmentCollidableVisualizers;
   private final List<CollidableListVisualizer> robotCollidableVisualizers = new ArrayList<>();

   private final YoBoolean hasGlobalContactParameters;
   private final YoContactParameters globalContactParameters;
   private final YoBoolean hasGlobalConstraintParameters;
   private final YoConstraintParameters globalConstraintParameters;

   private final YoDouble time = new YoDouble("physicsTime", registry);
   private final YoDouble rawTickDurationMilliseconds = new YoDouble("rawTickDurationMilliseconds", registry);
   private final YoDouble averageTickDurationMilliseconds = new YoDouble("averageTickDurationMilliseconds", registry);
   private final YoDouble rawRealTimeRate = new YoDouble("rawRealTimeRate", registry);
   private final YoDouble averageRealTimeRate = new YoDouble("averageRealTimeRate", registry);
   private final int averageWindow = 100;
   private final TDoubleLinkedList rawTickDurationBuffer = new TDoubleLinkedList();

   private boolean initialize = true;

   public ExperimentalPhysicsEngine()
   {
      YoRegistry multiContactCalculatorRegistry = new YoRegistry(MultiContactImpulseCalculator.class.getSimpleName());
      registry.addChild(multiContactCalculatorRegistry);

      hasGlobalContactParameters = new YoBoolean("hasGlobalContactParameters", registry);
      globalContactParameters = new YoContactParameters("globalContact", registry);
      hasGlobalConstraintParameters = new YoBoolean("hasGlobalConstraintParameters", registry);
      globalConstraintParameters = new YoConstraintParameters("globalConstraint", registry);
      multiContactImpulseCalculatorPool = new YoMultiContactImpulseCalculatorPool(1, rootFrame, multiContactCalculatorRegistry);

      AppearanceDefinition environmentCollidableAppearance = YoAppearance.AluminumMaterial();
      environmentCollidableAppearance.setTransparency(0.5);
      environmentCollidableVisualizers = new CollidableListVisualizer(collidableVisualizerGroupName,
                                                                      environmentCollidableAppearance,
                                                                      registry,
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
                        MultiBodySystemStateReader physicsOutputStateReader)
   {
      addRobot(robotName, rootBody, controllerOutputWriter, robotInitialStateWriter, robotCollisionModel, null, physicsOutputStateReader);
   }

   public void addRobot(String robotName, RigidBodyBasics rootBody, MultiBodySystemStateWriter controllerOutputWriter,
                        MultiBodySystemStateWriter robotInitialStateWriter, RobotCollisionModel robotCollisionModel,
                        MultiBodySystemStateWriter physicsInputStateWriter, MultiBodySystemStateReader physicsOutputStateReader)
   {
      PhysicsEngineRobotData robot = new PhysicsEngineRobotData(robotName, rootBody, robotCollisionModel, physicsEngineGraphicsRegistry);
      YoRegistry robotRegistry = robot.getRobotRegistry();
      robot.setRobotInitialStateWriter(robotInitialStateWriter);
      robot.setControllerOutputWriter(controllerOutputWriter);
      robot.addPhysicsInputStateWriter(physicsInputStateWriter);
      robot.addPhysicsOutputStateReader(physicsOutputStateReader);
      robotMap.put(robot.getRootBody(), robot);
      AppearanceDefinition robotCollidableAppearance = YoAppearance.DarkGreen();
      robotCollidableAppearance.setTransparency(0.5);
      CollidableListVisualizer collidableVisualizers = new CollidableListVisualizer(collidableVisualizerGroupName,
                                                                                    robotCollidableAppearance,
                                                                                    robotRegistry,
                                                                                    physicsEngineGraphicsRegistry);
      robot.getCollidables().forEach(collidableVisualizers::addCollidable);
      robotCollidableVisualizers.add(collidableVisualizers);
      registry.addChild(robotRegistry);
      robotList.add(robot);
   }

   public void addRobotPhysicsOutputStateReader(String robotName, MultiBodySystemStateReader physicsOutputReader)
   {
      PhysicsEngineRobotData physicsEngineRobotData = robotList.stream().filter(robot -> robot.getRobotName().equals(robotName)).findFirst().get();
      physicsEngineRobotData.addPhysicsOutputStateReader(physicsOutputReader);
   }

   public void addExternalWrenchReader(ExternalWrenchReader externalWrenchReader)
   {
      externalWrenchReaders.add(externalWrenchReader);
   }

   public void addExternalWrenchProvider(ExternalWrenchProvider externalWrenchProvider)
   {
      externalWrenchProviders.add(externalWrenchProvider);
   }

   public void addInertialMeasurementReader(InertialMeasurementReader reader)
   {
      inertialMeasurementReaders.add(reader);
   }

   public void setGlobalConstraintParameters(ConstraintParametersReadOnly parameters)
   {
      globalConstraintParameters.set(parameters);
      hasGlobalConstraintParameters.set(true);
   }

   public void setGlobalContactParameters(ContactParametersReadOnly parameters)
   {
      globalContactParameters.set(parameters);
      hasGlobalContactParameters.set(true);
   }

   public boolean initialize()
   {
      if (!initialize)
         return false;

      for (PhysicsEngineRobotData robot : robotList)
      {
         robot.initialize();
         robot.notifyPhysicsOutputStateReaders();

         for (InertialMeasurementReader reader : inertialMeasurementReaders)
         {
            reader.initialize(robot.getMultiBodySystem(),
                              robot.getForwardDynamicsPlugin().getForwardDynamicsCalculator().getAccelerationProvider(),
                              robot.getIntegrator().getRigidBodyTwistChangeProvider());
         }
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

      for (PhysicsEngineRobotData robotPlugin : robotMap.values())
      {
         SingleRobotForwardDynamicsPlugin forwardDynamicsPlugin = robotPlugin.getForwardDynamicsPlugin();
         forwardDynamicsPlugin.resetExternalWrenches();
         forwardDynamicsPlugin.applyExternalWrenches(externalWrenchProviders);
         forwardDynamicsPlugin.applyControllerOutput();
         if (robotPlugin.notifyPhysicsInputStateWriters())
            robotPlugin.updateFrames();
         forwardDynamicsPlugin.doScience(time.getValue(), dt, gravity);
         forwardDynamicsPlugin.readJointVelocities();
      }

      environmentCollidables.forEach(collidable -> collidable.updateBoundingBox(rootFrame));
      if (hasGlobalContactParameters.getValue())
         collisionDetectionPlugin.setMinimumPenetration(globalContactParameters.getMinimumPenetration());
      collisionDetectionPlugin.evaluationCollisions(robotList, () -> environmentCollidables, dt);

      collisionGroups = MultiRobotCollisionGroup.toCollisionGroups(collisionDetectionPlugin.getAllCollisions());

      Set<RigidBodyBasics> uncoveredRobotsRootBody = new HashSet<>(robotMap.keySet());
      List<MultiContactImpulseCalculator> impulseCalculators = new ArrayList<>();

      multiContactImpulseCalculatorPool.clear();

      for (MultiRobotCollisionGroup collisionGroup : collisionGroups)
      {
         MultiContactImpulseCalculator calculator = multiContactImpulseCalculatorPool.nextAvailable();

         calculator.configure(robotMap, collisionGroup);

         if (hasGlobalConstraintParameters.getValue())
            calculator.setConstraintParameters(globalConstraintParameters);
         if (hasGlobalContactParameters.getValue())
            calculator.setContactParameters(globalContactParameters);

         impulseCalculators.add(calculator);
         uncoveredRobotsRootBody.removeAll(collisionGroup.getRootBodies());
      }

      for (RigidBodyBasics rootBody : uncoveredRobotsRootBody)
      {
         PhysicsEngineRobotData robot = robotMap.get(rootBody);
         RobotJointLimitImpulseBasedCalculator jointLimitConstraintCalculator = robot.getJointLimitConstraintCalculator();
         jointLimitConstraintCalculator.initialize(dt);
         jointLimitConstraintCalculator.updateInertia(null, null);
         jointLimitConstraintCalculator.computeImpulse(dt);
         robot.getIntegrator().addJointVelocityChange(jointLimitConstraintCalculator.getJointVelocityChange(0));
      }

      for (MultiContactImpulseCalculator impulseCalculator : impulseCalculators)
      {
         impulseCalculator.computeImpulses(time.getValue(), dt, false);
         impulseCalculator.applyJointVelocityChanges();
         impulseCalculator.readExternalWrenches(dt, externalWrenchReaders);
      }

      for (PhysicsEngineRobotData robotPlugin : robotMap.values())
      {
         SingleRobotForwardDynamicsPlugin forwardDynamicsPlugin = robotPlugin.getForwardDynamicsPlugin();
         forwardDynamicsPlugin.writeJointAccelerations();
         robotPlugin.getIntegrator().integrate(dt);
      }

      environmentCollidableVisualizers.update(collisionDetectionPlugin.getAllCollisions());
      robotCollidableVisualizers.forEach(visualizer -> visualizer.update(collisionDetectionPlugin.getAllCollisions()));

      for (int i = 0; i < robotList.size(); i++)
      {
         PhysicsEngineRobotData robot = robotList.get(i);
         robot.updateFrames();
         robot.notifyPhysicsOutputStateReaders();
      }

      for (InertialMeasurementReader reader : inertialMeasurementReaders)
      {
         reader.read(dt);
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

   public YoRegistry getPhysicsEngineRegistry()
   {
      return registry;
   }

   public YoGraphicsListRegistry getPhysicsEngineGraphicsRegistry()
   {
      return physicsEngineGraphicsRegistry;
   }
}
