package us.ihmc.robotics.physics;

import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.lists.SupplierBuilder;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class PhysicsEngineRobotData implements CollidableHolder
{
   private final RigidBodyBasics rootBody;
   private final MultiBodySystemStateWriter robotInitialStateWriter;

   private final YoVariableRegistry robotRegistry;
   private final YoVariableRegistry contactConstraintCalculatorRegistry = new YoVariableRegistry(SingleContactImpulseCalculator.class.getSimpleName());
   private final MultiBodySystemBasics multiBodySystem;
   private final List<Collidable> collidables;

   // TODO Following fields are specific to the type of engine used, they need interfacing.
   private final SingleRobotForwardDynamicsPlugin forwardDynamicsPlugin;
   private final RobotJointLimitImpulseBasedCalculator jointLimitConstraintCalculator;
   private final RecyclingArrayList<YoSingleContactImpulseCalculator> environmentContactConstraintCalculatorPool;
   private final Map<RigidBodyBasics, RecyclingArrayList<YoSingleContactImpulseCalculator>> interRobotContactConstraintCalculatorPools = new HashMap<>();

   public PhysicsEngineRobotData(String robotName, RigidBodyBasics rootBody, MultiBodySystemStateWriter robotInitialStateWriter,
                                 MultiBodySystemStateWriter controllerOutputWriter, RobotCollisionModel robotCollisionModel,
                                 YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.rootBody = rootBody;
      this.robotInitialStateWriter = robotInitialStateWriter;
      robotRegistry = new YoVariableRegistry(robotName);
      multiBodySystem = MultiBodySystemBasics.toMultiBodySystemBasics(rootBody);
      if (robotInitialStateWriter != null)
         robotInitialStateWriter.setMultiBodySystem(multiBodySystem);
      if (controllerOutputWriter != null)
         controllerOutputWriter.setMultiBodySystem(multiBodySystem);

      collidables = robotCollisionModel != null ? robotCollisionModel.getRobotCollidables(multiBodySystem) : Collections.emptyList();

      forwardDynamicsPlugin = new SingleRobotForwardDynamicsPlugin(multiBodySystem, controllerOutputWriter);

      YoVariableRegistry jointLimitConstraintCalculatorRegistry = new YoVariableRegistry(RobotJointLimitImpulseBasedCalculator.class.getSimpleName());
      robotRegistry.addChild(jointLimitConstraintCalculatorRegistry);

      jointLimitConstraintCalculator = new YoRobotJointLimitImpulseBasedCalculator(rootBody,
                                                                                   forwardDynamicsPlugin.getForwardDynamicsCalculator(),
                                                                                   jointLimitConstraintCalculatorRegistry);

      robotRegistry.addChild(contactConstraintCalculatorRegistry);

      environmentContactConstraintCalculatorPool = new RecyclingArrayList<>(20, SupplierBuilder.indexedSupplier(identifier ->
      {
         YoSingleContactImpulseCalculator calculator = new YoSingleContactImpulseCalculator("Single",
                                                                                            identifier,
                                                                                            multiBodySystem.getInertialFrame(),
                                                                                            rootBody,
                                                                                            forwardDynamicsPlugin.getForwardDynamicsCalculator(),
                                                                                            null,
                                                                                            null,
                                                                                            contactConstraintCalculatorRegistry);
         if (yoGraphicsListRegistry != null)
            calculator.setupGraphics(yoGraphicsListRegistry);
         calculator.clear();
         return calculator;
      }));
      resetCalculators();
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
      environmentContactConstraintCalculatorPool.forEach(calculator -> calculator.clear());
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

      RecyclingArrayList<YoSingleContactImpulseCalculator> calculators = interRobotContactConstraintCalculatorPools.get(otherRobot.getRootBody());

      if (calculators == null)
      {
         calculators = new RecyclingArrayList<>(SupplierBuilder.indexedSupplier(identifier ->
         {
            return new YoSingleContactImpulseCalculator("Dual",
                                                        identifier,
                                                        multiBodySystem.getInertialFrame(),
                                                        rootBody,
                                                        forwardDynamicsPlugin.getForwardDynamicsCalculator(),
                                                        otherRobot.getRootBody(),
                                                        otherRobot.getForwardDynamicsPlugin().getForwardDynamicsCalculator(),
                                                        contactConstraintCalculatorRegistry);
         }));
         interRobotContactConstraintCalculatorPools.put(otherRobot.getRootBody(), calculators);
      }

      return calculators.add();
   }

   public YoVariableRegistry getRobotRegistry()
   {
      return robotRegistry;
   }
}