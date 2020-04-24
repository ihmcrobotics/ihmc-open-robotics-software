package us.ihmc.robotics.physics;

import java.util.ArrayList;
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
   private static final String ContactCalculatorNameSuffix = SingleContactImpulseCalculator.class.getSimpleName();

   private final String robotName;
   private final RigidBodyBasics rootBody;
   private MultiBodySystemStateWriter robotInitialStateWriter;
   private final List<MultiBodySystemStateReader> physicsOutputReaders = new ArrayList<>();

   private final YoVariableRegistry robotRegistry;
   private final YoVariableRegistry environmentContactCalculatorRegistry = new YoVariableRegistry("Environment" + ContactCalculatorNameSuffix);
   private final YoVariableRegistry interRobotContactCalculatorRegistry = new YoVariableRegistry("InterRobot" + ContactCalculatorNameSuffix);
   private final YoVariableRegistry selfContactCalculatorRegistry = new YoVariableRegistry("Self" + ContactCalculatorNameSuffix);
   private final MultiBodySystemBasics multiBodySystem;
   private final List<Collidable> collidables;

   // TODO Following fields are specific to the type of engine used, they need interfacing.
   private final SingleRobotForwardDynamicsPlugin forwardDynamicsPlugin;
   private final RobotJointLimitImpulseBasedCalculator jointLimitConstraintCalculator;
   private final RecyclingArrayList<YoSingleContactImpulseCalculator> environmentContactConstraintCalculatorPool;
   private final RecyclingArrayList<YoSingleContactImpulseCalculator> selfContactConstraintCalculatorPool;
   private final Map<RigidBodyBasics, RecyclingArrayList<YoSingleContactImpulseCalculator>> interRobotContactConstraintCalculatorPools = new HashMap<>();

   public PhysicsEngineRobotData(String robotName, RigidBodyBasics rootBody, RobotCollisionModel robotCollisionModel,
                                 YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.robotName = robotName;
      this.rootBody = rootBody;
      robotRegistry = new YoVariableRegistry(robotName);
      multiBodySystem = MultiBodySystemBasics.toMultiBodySystemBasics(rootBody);

      collidables = robotCollisionModel != null ? robotCollisionModel.getRobotCollidables(multiBodySystem) : Collections.emptyList();

      forwardDynamicsPlugin = new SingleRobotForwardDynamicsPlugin(multiBodySystem);

      YoVariableRegistry jointLimitConstraintCalculatorRegistry = new YoVariableRegistry(RobotJointLimitImpulseBasedCalculator.class.getSimpleName());
      robotRegistry.addChild(jointLimitConstraintCalculatorRegistry);

      jointLimitConstraintCalculator = new YoRobotJointLimitImpulseBasedCalculator(rootBody,
                                                                                   forwardDynamicsPlugin.getForwardDynamicsCalculator(),
                                                                                   jointLimitConstraintCalculatorRegistry);

      robotRegistry.addChild(environmentContactCalculatorRegistry);
      robotRegistry.addChild(interRobotContactCalculatorRegistry);
      robotRegistry.addChild(selfContactCalculatorRegistry);

      environmentContactConstraintCalculatorPool = new RecyclingArrayList<>(20, SupplierBuilder.indexedSupplier(identifier ->
      {
         YoSingleContactImpulseCalculator calculator = new YoSingleContactImpulseCalculator(robotName + "Single",
                                                                                            identifier,
                                                                                            multiBodySystem.getInertialFrame(),
                                                                                            rootBody,
                                                                                            forwardDynamicsPlugin.getForwardDynamicsCalculator(),
                                                                                            null,
                                                                                            null,
                                                                                            environmentContactCalculatorRegistry);
         if (yoGraphicsListRegistry != null)
            calculator.setupGraphics(yoGraphicsListRegistry);
         return calculator;
      }));

      selfContactConstraintCalculatorPool = new RecyclingArrayList<>(8, SupplierBuilder.indexedSupplier(identifier ->
      {
         YoSingleContactImpulseCalculator calculator = new YoSingleContactImpulseCalculator(robotName + "Self",
                                                                                            identifier,
                                                                                            multiBodySystem.getInertialFrame(),
                                                                                            rootBody,
                                                                                            forwardDynamicsPlugin.getForwardDynamicsCalculator(),
                                                                                            rootBody,
                                                                                            forwardDynamicsPlugin.getForwardDynamicsCalculator(),
                                                                                            selfContactCalculatorRegistry);
         if (yoGraphicsListRegistry != null)
            calculator.setupGraphics(yoGraphicsListRegistry);
         return calculator;
      }));

      resetCalculators();
   }

   public void setControllerOutputWriter(MultiBodySystemStateWriter controllerOutputWriter)
   {
      if (controllerOutputWriter != null)
         controllerOutputWriter.setMultiBodySystem(multiBodySystem);
      forwardDynamicsPlugin.setControllerOutputWriter(controllerOutputWriter);
   }

   public void setRobotInitialStateWriter(MultiBodySystemStateWriter robotInitialStateWriter)
   {
      this.robotInitialStateWriter = robotInitialStateWriter;
      setControllerOutputWriter(robotInitialStateWriter);
   }

   public void addPhysicsOutputReader(MultiBodySystemStateReader physicsOutputReader)
   {
      if (physicsOutputReader == null)
         return;
      physicsOutputReader.setMultiBodySystem(multiBodySystem);
      physicsOutputReaders.add(physicsOutputReader);
   }

   public void initialize()
   {
      robotInitialStateWriter.write();
      updateFrames();
   }

   public void notifyPhysicsOutputReaders()
   {
      physicsOutputReaders.forEach(MultiBodySystemStateReader::read);
   }

   public void updateCollidableBoundingBoxes()
   {
      collidables.forEach(collidable -> collidable.updateBoundingBox(multiBodySystem.getInertialFrame()));
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

   public String getRobotName()
   {
      return robotName;
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
      selfContactConstraintCalculatorPool.forEach(calculator -> calculator.clear());
      selfContactConstraintCalculatorPool.clear();
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

   public SingleContactImpulseCalculator getOrCreateSelfContactConstraintCalculator()
   {
      return selfContactConstraintCalculatorPool.add();
   }

   public SingleContactImpulseCalculator getOrCreateInterRobotContactConstraintCalculator(PhysicsEngineRobotData otherRobot)
   {
      if (otherRobot == null)
         return getOrCreateEnvironmentContactConstraintCalculator();
      if (otherRobot == this)
         return getOrCreateSelfContactConstraintCalculator();

      RecyclingArrayList<YoSingleContactImpulseCalculator> calculators = interRobotContactConstraintCalculatorPools.get(otherRobot.getRootBody());

      if (calculators == null)
      {
         calculators = new RecyclingArrayList<>(8, SupplierBuilder.indexedSupplier(identifier ->
         {
            return new YoSingleContactImpulseCalculator(robotName + "Dual",
                                                        identifier,
                                                        multiBodySystem.getInertialFrame(),
                                                        rootBody,
                                                        forwardDynamicsPlugin.getForwardDynamicsCalculator(),
                                                        otherRobot.getRootBody(),
                                                        otherRobot.getForwardDynamicsPlugin().getForwardDynamicsCalculator(),
                                                        interRobotContactCalculatorRegistry);
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