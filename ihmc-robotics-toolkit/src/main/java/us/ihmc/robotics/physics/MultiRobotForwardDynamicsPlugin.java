package us.ihmc.robotics.physics;

import java.util.*;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

public class MultiRobotForwardDynamicsPlugin
{
   private final ReferenceFrame rootFrame;
   private final Map<RigidBodyBasics, PhysicsEngineRobotData> robots = new HashMap<>();

   private List<MultiRobotCollisionGroup> collisionGroups;

   public MultiRobotForwardDynamicsPlugin(ReferenceFrame rootFrame)
   {
      this.rootFrame = rootFrame;
   }

   public void addRobot(PhysicsEngineRobotData robot)
   {
      robots.put(robot.getRootBody(), robot);
   }

   public void removeRobot(PhysicsEngineRobotData robot)
   {
      robots.remove(robot.getRootBody());
   }

   public void submitCollisions(SimpleCollisionDetection collisionDetectionPlugin)
   {
      collisionGroups = MultiRobotCollisionGroup.toCollisionGroups(collisionDetectionPlugin.getAllCollisions());
   }

   public void doScience(double dt, Vector3DReadOnly gravity)
   {
      for (PhysicsEngineRobotData robotPlugin : robots.values())
      { // First pass without the external forces
         SingleRobotForwardDynamicsPlugin forwardDynamicsPlugin = robotPlugin.getForwardDynamicsPlugin();
         forwardDynamicsPlugin.resetExternalWrenches();
         forwardDynamicsPlugin.applyControllerOutput();
         forwardDynamicsPlugin.doScience(dt, gravity);
         forwardDynamicsPlugin.readJointVelocities();
      }

      Set<RigidBodyBasics> uncoveredRobotsRootBody = new HashSet<>(robots.keySet());
      List<MultiContactImpulseCalculator> impulseCalculators = new ArrayList<>();

      for (MultiRobotCollisionGroup collisionGroup : collisionGroups)
      {
         MultiContactImpulseCalculator calculator = new MultiContactImpulseCalculator(rootFrame);
         calculator.configure(robots, collisionGroup);
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
         robot.getForwardDynamicsPlugin().addJointVelocities(jointLimitConstraintCalculator.getJointVelocityChange(0));
      }

      for (MultiContactImpulseCalculator impulseCalculator : impulseCalculators)
      {
         impulseCalculator.computeImpulses(dt, false);
         robots.forEach((rootBody, robot) -> impulseCalculator.applyJointVelocityChange(rootBody, robot.getForwardDynamicsPlugin()::addJointVelocities));
      }

      for (PhysicsEngineRobotData robotPlugin : robots.values())
      {
         SingleRobotForwardDynamicsPlugin forwardDynamicsPlugin = robotPlugin.getForwardDynamicsPlugin();
         forwardDynamicsPlugin.writeJointVelocities();
         forwardDynamicsPlugin.writeJointAccelerations();
      }
   }
}
