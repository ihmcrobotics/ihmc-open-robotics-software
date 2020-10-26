package us.ihmc.robotics.physics;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

public class PhysicsEngineRobotData implements CollidableHolder
{
   private static final String ContactCalculatorNameSuffix = SingleContactImpulseCalculator.class.getSimpleName();

   private final String robotName;
   private final RigidBodyBasics rootBody;
   private MultiBodySystemStateWriter robotInitialStateWriter;
   private final List<MultiBodySystemStateWriter> physicsInputStateWriters = new ArrayList<>();
   private final List<MultiBodySystemStateReader> physicsOutputStateReaders = new ArrayList<>();

   private final YoRegistry robotRegistry;
   private final YoRegistry environmentContactCalculatorRegistry = new YoRegistry("Environment" + ContactCalculatorNameSuffix);
   private final YoRegistry interRobotContactCalculatorRegistry = new YoRegistry("InterRobot" + ContactCalculatorNameSuffix);
   private final YoRegistry selfContactCalculatorRegistry = new YoRegistry("Self" + ContactCalculatorNameSuffix);
   private final MultiBodySystemBasics multiBodySystem;
   private final List<Collidable> collidables;

   // TODO Following fields are specific to the type of engine used, they need interfacing.
   private final SingleRobotForwardDynamicsPlugin forwardDynamicsPlugin;
   private final RobotJointLimitImpulseBasedCalculator jointLimitConstraintCalculator;
   private final YoSingleContactImpulseCalculatorPool environmentContactConstraintCalculatorPool;
   private final YoSingleContactImpulseCalculatorPool selfContactConstraintCalculatorPool;
   private final Map<RigidBodyBasics, YoSingleContactImpulseCalculatorPool> interRobotContactConstraintCalculatorPools = new HashMap<>();

   private final SingleRobotFirstOrderIntegrator integrator;

   public PhysicsEngineRobotData(String robotName, RigidBodyBasics rootBody, RobotCollisionModel robotCollisionModel,
                                 YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.robotName = robotName;
      this.rootBody = rootBody;
      robotRegistry = new YoRegistry(robotName);
      multiBodySystem = MultiBodySystemBasics.toMultiBodySystemBasics(rootBody);

      collidables = robotCollisionModel != null ? robotCollisionModel.getRobotCollidables(multiBodySystem) : Collections.emptyList();

      forwardDynamicsPlugin = new SingleRobotForwardDynamicsPlugin(multiBodySystem);
      FrameShapePosePredictor frameShapePosePredictor = new FrameShapePosePredictor(forwardDynamicsPlugin.getForwardDynamicsCalculator());
      collidables.forEach(collidable -> collidable.setFrameShapePosePredictor(frameShapePosePredictor));

      YoRegistry jointLimitConstraintCalculatorRegistry = new YoRegistry(RobotJointLimitImpulseBasedCalculator.class.getSimpleName());
      robotRegistry.addChild(jointLimitConstraintCalculatorRegistry);

      jointLimitConstraintCalculator = new YoRobotJointLimitImpulseBasedCalculator(rootBody,
                                                                                   forwardDynamicsPlugin.getForwardDynamicsCalculator(),
                                                                                   jointLimitConstraintCalculatorRegistry);

      robotRegistry.addChild(environmentContactCalculatorRegistry);
      robotRegistry.addChild(interRobotContactCalculatorRegistry);
      robotRegistry.addChild(selfContactCalculatorRegistry);

      environmentContactConstraintCalculatorPool = new YoSingleContactImpulseCalculatorPool(20,
                                                                                            robotName + "Single",
                                                                                            multiBodySystem.getInertialFrame(),
                                                                                            rootBody,
                                                                                            forwardDynamicsPlugin.getForwardDynamicsCalculator(),
                                                                                            null,
                                                                                            null,
                                                                                            yoGraphicsListRegistry,
                                                                                            environmentContactCalculatorRegistry);

      selfContactConstraintCalculatorPool = new YoSingleContactImpulseCalculatorPool(8,
                                                                                     robotName + "Self",
                                                                                     multiBodySystem.getInertialFrame(),
                                                                                     rootBody,
                                                                                     forwardDynamicsPlugin.getForwardDynamicsCalculator(),
                                                                                     rootBody,
                                                                                     forwardDynamicsPlugin.getForwardDynamicsCalculator(),
                                                                                     yoGraphicsListRegistry,
                                                                                     selfContactCalculatorRegistry);

      integrator = new SingleRobotFirstOrderIntegrator(multiBodySystem);
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

   public void addPhysicsInputStateWriter(MultiBodySystemStateWriter physicsInputStateWriter)
   {
      if (physicsInputStateWriter == null)
         return;
      physicsInputStateWriter.setMultiBodySystem(multiBodySystem);
      physicsInputStateWriters.add(physicsInputStateWriter);
   }

   public void addPhysicsOutputStateReader(MultiBodySystemStateReader physicsOutputStateReader)
   {
      if (physicsOutputStateReader == null)
         return;
      physicsOutputStateReader.setMultiBodySystem(multiBodySystem);
      physicsOutputStateReaders.add(physicsOutputStateReader);
   }

   public void initialize()
   {
      robotInitialStateWriter.write();
      updateFrames();
   }

   public boolean notifyPhysicsInputStateWriters()
   {
      boolean stateChanged = false;

      for (MultiBodySystemStateWriter stateWriter : physicsInputStateWriters)
      {
         stateChanged |= stateWriter.write();
      }

      return stateChanged;
   }

   public void notifyPhysicsOutputStateReaders()
   {
      physicsOutputStateReaders.forEach(MultiBodySystemStateReader::read);
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

   public SingleRobotFirstOrderIntegrator getIntegrator()
   {
      return integrator;
   }

   public void resetCalculators()
   {
      environmentContactConstraintCalculatorPool.clear();
      selfContactConstraintCalculatorPool.clear();
      interRobotContactConstraintCalculatorPools.forEach((rigidBodyBasics, calculators) -> calculators.clear());
      integrator.reset();
   }

   public RobotJointLimitImpulseBasedCalculator getJointLimitConstraintCalculator()
   {
      return jointLimitConstraintCalculator;
   }

   public SingleContactImpulseCalculator getOrCreateEnvironmentContactConstraintCalculator()
   {
      return environmentContactConstraintCalculatorPool.nextAvailable();
   }

   public SingleContactImpulseCalculator getOrCreateSelfContactConstraintCalculator()
   {
      return selfContactConstraintCalculatorPool.nextAvailable();
   }

   public SingleContactImpulseCalculator getOrCreateInterRobotContactConstraintCalculator(PhysicsEngineRobotData otherRobot)
   {
      if (otherRobot == null)
         return getOrCreateEnvironmentContactConstraintCalculator();
      if (otherRobot == this)
         return getOrCreateSelfContactConstraintCalculator();

      YoSingleContactImpulseCalculatorPool calculators = interRobotContactConstraintCalculatorPools.get(otherRobot.getRootBody());

      if (calculators == null)
      {
         calculators = new YoSingleContactImpulseCalculatorPool(8,
                                                                robotName + otherRobot.getRobotName() + "Dual",
                                                                multiBodySystem.getInertialFrame(),
                                                                rootBody,
                                                                forwardDynamicsPlugin.getForwardDynamicsCalculator(),
                                                                otherRobot.getRootBody(),
                                                                otherRobot.getForwardDynamicsPlugin().getForwardDynamicsCalculator(),
                                                                null,
                                                                interRobotContactCalculatorRegistry);
         interRobotContactConstraintCalculatorPools.put(otherRobot.getRootBody(), calculators);
      }

      return calculators.nextAvailable();
   }

   public YoRegistry getRobotRegistry()
   {
      return robotRegistry;
   }
}