package us.ihmc.robotics.physics;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class PhysicsEngineRobotData implements CollidableHolder
{
   private final RigidBodyBasics rootBody;
   private final MultiBodySystemStateWriter robotInitialStateWriter;

   private final YoVariableRegistry robotRegistry;
   private final MultiBodySystemBasics multiBodySystem;
   private final List<Collidable> collidables;

   // TODO Following fields are specific to the type of engine used, they need interfacing.
   private final SingleRobotForwardDynamicsPlugin forwardDynamicsPlugin;
   private final RobotJointLimitImpulseBasedCalculator jointLimitConstraintCalculator;
   private final RecyclingArrayList<SingleContactImpulseCalculator> environmentContactConstraintCalculatorPool;
   private final Map<RigidBodyBasics, RecyclingArrayList<SingleContactImpulseCalculator>> interRobotContactConstraintCalculatorPools = new HashMap<>();

   public PhysicsEngineRobotData(String robotName, RigidBodyBasics rootBody, MultiBodySystemStateWriter robotInitialStateWriter,
                                 MultiBodySystemStateWriter controllerOutputWriter, RobotCollisionModel robotCollisionModel)
   {
      this.rootBody = rootBody;
      this.robotInitialStateWriter = robotInitialStateWriter;

      robotRegistry = new YoVariableRegistry(robotName);
      multiBodySystem = MultiBodySystemBasics.toMultiBodySystemBasics(rootBody);
      if (robotInitialStateWriter != null)
         robotInitialStateWriter.setMultiBodySystem(multiBodySystem);

      collidables = robotCollisionModel.getRobotCollidables(multiBodySystem);

      forwardDynamicsPlugin = new SingleRobotForwardDynamicsPlugin(multiBodySystem, controllerOutputWriter);
      YoVariableRegistry jointLimitConstraintCalculatorRegistry = new YoVariableRegistry("jointLimitConstraints");
      robotRegistry.addChild(jointLimitConstraintCalculatorRegistry);
      jointLimitConstraintCalculator = new YoRobotJointLimitImpulseBasedCalculator(rootBody,
                                                                                   forwardDynamicsPlugin.getForwardDynamicsCalculator(),
                                                                                   jointLimitConstraintCalculatorRegistry);

      environmentContactConstraintCalculatorPool = new RecyclingArrayList<>(() ->
      {
         return new SingleContactImpulseCalculator(multiBodySystem.getInertialFrame(),
                                                   rootBody,
                                                   forwardDynamicsPlugin.getForwardDynamicsCalculator(),
                                                   null,
                                                   null);
      });
   }

   public void initialize()
   {
      robotInitialStateWriter.write();
      updateFrames();
   }

   public void updateCollidableBoundingBoxes()
   {
      collidables.forEach(Collidable::updateBoundingBox);
   }

   public void updateFrames()
   {
      rootBody.updateFramesRecursively();
   }

   @Override
   public List<Collidable> getCollidables()
   {
      return collidables;
   }

   public RigidBodyBasics getRootBody()
   {
      return rootBody;
   }

   public MultiBodySystemBasics getMultiBodySystem()
   {
      return multiBodySystem;
   }

   public SingleRobotForwardDynamicsPlugin getForwardDynamicsPlugin()
   {
      return forwardDynamicsPlugin;
   }

   public void resetCalculators()
   {
      environmentContactConstraintCalculatorPool.clear();
      interRobotContactConstraintCalculatorPools.forEach((rigidBodyBasics, calculators) -> calculators.clear());
   }

   public RobotJointLimitImpulseBasedCalculator getJointLimitConstraintCalculator()
   {
      return jointLimitConstraintCalculator;
   }

   public SingleContactImpulseCalculator getOrCreateEnvironmentContactConstraintCalculator()
   {
      return environmentContactConstraintCalculatorPool.add();
   }

   public SingleContactImpulseCalculator getOrCreateInterRobotContactConstraintCalculator(PhysicsEngineRobotData otherRobot)
   {
      if (otherRobot == null)
         return getOrCreateEnvironmentContactConstraintCalculator();

      RecyclingArrayList<SingleContactImpulseCalculator> calculators = interRobotContactConstraintCalculatorPools.get(otherRobot.getRootBody());

      if (calculators == null)
      {
         calculators = new RecyclingArrayList<>(() ->
         {
            return new SingleContactImpulseCalculator(multiBodySystem.getInertialFrame(),
                                                      rootBody,
                                                      forwardDynamicsPlugin.getForwardDynamicsCalculator(),
                                                      otherRobot.getRootBody(),
                                                      otherRobot.getForwardDynamicsPlugin().getForwardDynamicsCalculator());
         });
      }

      return calculators.add();
   }

   public YoVariableRegistry getRobotRegistry()
   {
      return robotRegistry;
   }
}