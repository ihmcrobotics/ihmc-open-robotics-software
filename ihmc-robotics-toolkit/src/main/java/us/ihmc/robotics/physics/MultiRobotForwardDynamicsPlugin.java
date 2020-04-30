package us.ihmc.robotics.physics;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class MultiRobotForwardDynamicsPlugin
{
   private final Map<RigidBodyBasics, PhysicsEngineRobotData> robots = new HashMap<>();
   private final YoMultiContactImpulseCalculatorPool multiContactImpulseCalculatorPool;
   private final List<ExternalWrenchReader> externalWrenchReaders = new ArrayList<>();

   private List<MultiRobotCollisionGroup> collisionGroups;

   private final YoBoolean hasGlobalContactParameters;
   private final YoContactParameters globalContactParameters;
   private final YoBoolean hasGlobalConstraintParameters;
   private final YoConstraintParameters globalConstraintParameters;

   public MultiRobotForwardDynamicsPlugin(ReferenceFrame rootFrame, YoVariableRegistry registry)
   {
      YoVariableRegistry multiContactCalculatorRegistry = new YoVariableRegistry(MultiContactImpulseCalculator.class.getSimpleName());
      registry.addChild(multiContactCalculatorRegistry);

      hasGlobalContactParameters = new YoBoolean("hasGlobalContactParameters", registry);
      globalContactParameters = new YoContactParameters("globalContact", registry);
      hasGlobalConstraintParameters = new YoBoolean("hasGlobalConstraintParameters", registry);
      globalConstraintParameters = new YoConstraintParameters("globalConstraint", registry);
      multiContactImpulseCalculatorPool = new YoMultiContactImpulseCalculatorPool(1, rootFrame, multiContactCalculatorRegistry);
   }

   public void addRobot(PhysicsEngineRobotData robot)
   {
      robots.put(robot.getRootBody(), robot);
   }

   public void removeRobot(PhysicsEngineRobotData robot)
   {
      robots.remove(robot.getRootBody());
   }

   public void addExternalWrenchReader(ExternalWrenchReader externalWrenchReader)
   {
      externalWrenchReaders.add(externalWrenchReader);
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

   public void submitCollisions(CollisionListResult allCollisions)
   {
      collisionGroups = MultiRobotCollisionGroup.toCollisionGroups(allCollisions);
   }

   public void doScience(double time, double dt, Vector3DReadOnly gravity)
   {
      for (PhysicsEngineRobotData robotPlugin : robots.values())
      { // First pass without the external forces
         SingleRobotForwardDynamicsPlugin forwardDynamicsPlugin = robotPlugin.getForwardDynamicsPlugin();
         forwardDynamicsPlugin.resetExternalWrenches();
         forwardDynamicsPlugin.applyControllerOutput();
         forwardDynamicsPlugin.doScience(time, dt, gravity);
         forwardDynamicsPlugin.readJointVelocities();
      }

      Set<RigidBodyBasics> uncoveredRobotsRootBody = new HashSet<>(robots.keySet());
      List<MultiContactImpulseCalculator> impulseCalculators = new ArrayList<>();

      multiContactImpulseCalculatorPool.clear();

      for (MultiRobotCollisionGroup collisionGroup : collisionGroups)
      {
         MultiContactImpulseCalculator calculator = multiContactImpulseCalculatorPool.nextAvailable();

         calculator.configure(robots, collisionGroup);

         if (hasGlobalConstraintParameters.getValue())
            calculator.setConstraintParameters(globalConstraintParameters);
         if (hasGlobalContactParameters.getValue())
            calculator.setContactParameters(globalContactParameters);

         impulseCalculators.add(calculator);
         uncoveredRobotsRootBody.removeAll(collisionGroup.getRootBodies());
      }

      for (RigidBodyBasics rootBody : uncoveredRobotsRootBody)
      {
         PhysicsEngineRobotData robot = robots.get(rootBody);
         RobotJointLimitImpulseBasedCalculator jointLimitConstraintCalculator = robot.getJointLimitConstraintCalculator();
         jointLimitConstraintCalculator.initialize(dt);
         jointLimitConstraintCalculator.updateInertia(null, null);
         jointLimitConstraintCalculator.computeImpulse(dt);
         robot.getIntegrator().addJointVelocityChange(jointLimitConstraintCalculator.getJointVelocityChange(0));
      }

      for (MultiContactImpulseCalculator impulseCalculator : impulseCalculators)
      {
         impulseCalculator.computeImpulses(time, dt, false);
         impulseCalculator.applyJointVelocityChanges();
         impulseCalculator.readExternalWrenches(dt, externalWrenchReaders);
      }

      for (PhysicsEngineRobotData robotPlugin : robots.values())
      {
         SingleRobotForwardDynamicsPlugin forwardDynamicsPlugin = robotPlugin.getForwardDynamicsPlugin();
         forwardDynamicsPlugin.writeJointAccelerations();
         robotPlugin.getIntegrator().integrate(dt);
      }
   }

   public boolean hasGlobalContactParameters()
   {
      return hasGlobalContactParameters.getValue();
   }

   public ContactParametersReadOnly getGlobalContactParameters()
   {
      return globalContactParameters;
   }
}
