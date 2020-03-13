package us.ihmc.robotics.physics;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.algorithms.ForwardDynamicsCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

public class MultiRobotForwardDynamicsPlugin
{
   private final ReferenceFrame rootFrame;
   private final Map<RigidBodyBasics, SingleRobotForwardDynamicsPlugin> singleRobotPluginMap = new HashMap<>();
   private final Map<RigidBodyBasics, ForwardDynamicsCalculator> robotForwardDynamicsCalculatorMap = new HashMap<>();
   private SimpleCollisionDetection collisionDetection;

   public MultiRobotForwardDynamicsPlugin(ReferenceFrame rootFrame)
   {
      this.rootFrame = rootFrame;
   }

   public SingleRobotForwardDynamicsPlugin addMultiBodySystem(MultiBodySystemBasics multiBodySystem, MultiBodySystemStateWriter controllerOutputWriter)
   {
      SingleRobotForwardDynamicsPlugin robotForwardDynamicsPlugin = new SingleRobotForwardDynamicsPlugin(multiBodySystem);
      robotForwardDynamicsPlugin.submitControllerOutput(controllerOutputWriter);
      singleRobotPluginMap.put(multiBodySystem.getRootBody(), robotForwardDynamicsPlugin);
      robotForwardDynamicsCalculatorMap.put(multiBodySystem.getRootBody(), robotForwardDynamicsPlugin.getForwardDynamicsCalculator());
      return robotForwardDynamicsPlugin;
   }

   public void removeMultiBodySystem(MultiBodySystemBasics multiBodySystem)
   {
      singleRobotPluginMap.remove(multiBodySystem.getRootBody());
      robotForwardDynamicsCalculatorMap.remove(multiBodySystem.getRootBody());
   }

   public void submitCollisions(SimpleCollisionDetection collisionDetectionPlugin)
   {
      this.collisionDetection = collisionDetectionPlugin;
   }

   public void doScience(double dt, Vector3DReadOnly gravity)
   {
      for (SingleRobotForwardDynamicsPlugin robotPlugin : singleRobotPluginMap.values())
      { // First pass without the external forces
         robotPlugin.resetExternalWrenches();
         robotPlugin.applyControllerOutput();
         robotPlugin.doScience(dt, gravity);
         robotPlugin.readJointVelocities();
      }

      List<MultiRobotCollisionGroup> collisionGroups = MultiRobotCollisionGroup.toCollisionGroups(collisionDetection.getAllCollisions());
      List<MultiContactImpulseCalculator> impulseCalculators = new ArrayList<>();

      for (MultiRobotCollisionGroup collisionGroup : collisionGroups)
      {
         impulseCalculators.add(new MultiContactImpulseCalculator(rootFrame, dt, robotForwardDynamicsCalculatorMap, collisionGroup));
      }

      for (MultiContactImpulseCalculator impulseCalculator : impulseCalculators)
      {
         impulseCalculator.computeImpulses(false);
         impulseCalculator.applyJointVelocityChange(singleRobotPluginMap);
      }

      for (SingleRobotForwardDynamicsPlugin robotPlugin : singleRobotPluginMap.values())
      { // Second pass with the external forces.
         robotPlugin.writeJointVelocities();
         robotPlugin.writeJointAccelerations();
      }
   }
}
