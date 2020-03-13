package us.ihmc.robotics.physics;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.algorithms.ForwardDynamicsCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

/**
 * Inspired from: <i>Per-Contact Iteration Method for Solving Contact Dynamics</i>
 *
 * @author Sylvain Bertrand
 */
public class MultiContactImpulseCalculator
{
   private final List<SingleContactImpulseCalculator> impulseCalculators = new ArrayList<>();

   private double alpha_min = 0.3;
   private double gamma = 0.99;
   private double tolerance = 1.0e-6;

   private int maxNumberOfIterations = 500;
   private int iterationCounter = 0;

   public MultiContactImpulseCalculator(ReferenceFrame rootFrame, double dt, Map<RigidBodyBasics, ForwardDynamicsCalculator> robotForwardDynamicsCalculatorMap,
                                        MultiRobotCollisionGroup collisionGroup)
   {
      CombinedRigidBodyTwistProviders combinedRigidBodyTwistProviders = new CombinedRigidBodyTwistProviders(rootFrame);

      for (CollisionResult collisionResult : collisionGroup.getGroupCollisions())
      {
         RigidBodyBasics rootA = collisionResult.getCollidableA().getRootBody();
         RigidBodyBasics rootB = collisionResult.getCollidableB().getRootBody();
         ForwardDynamicsCalculator robotA = robotForwardDynamicsCalculatorMap.get(rootA);
         ForwardDynamicsCalculator robotB = rootB != null ? robotForwardDynamicsCalculatorMap.get(rootB) : null;

         SingleContactImpulseCalculator impulseCalculator = new SingleContactImpulseCalculator(collisionResult,
                                                                                               rootFrame,
                                                                                               dt,
                                                                                               robotA,
                                                                                               robotB);
         impulseCalculators.add(impulseCalculator);

         combinedRigidBodyTwistProviders.addRigidBodyTwistProvider(impulseCalculator.getTwistChangeProviderA());
         combinedRigidBodyTwistProviders.addRigidBodyTwistProvider(impulseCalculator.getTwistChangeProviderB());
      }

      for (SingleContactImpulseCalculator impulseCalculator : impulseCalculators)
      {
         CombinedRigidBodyTwistProviders externalTwistModifier = new CombinedRigidBodyTwistProviders(combinedRigidBodyTwistProviders);
         externalTwistModifier.removeRigidBodyTwistProvider(impulseCalculator.getTwistChangeProviderA());
         externalTwistModifier.removeRigidBodyTwistProvider(impulseCalculator.getTwistChangeProviderB());
         impulseCalculator.setExternalTwistModifier(externalTwistModifier);
      }
   }

   public double computeImpulses(boolean verbose)
   {
      if (impulseCalculators.size() == 1)
      {
         impulseCalculators.get(0).computeImpulse();
         return 0.0;
      }
      else
      {
         double alpha = 1.0;
         double maxUpdateMagnitude = Double.POSITIVE_INFINITY;

         iterationCounter = 0;

         while (maxUpdateMagnitude > tolerance)
         {
            maxUpdateMagnitude = Double.NEGATIVE_INFINITY;
            int numberOfClosingContacts = 0;

            for (int i = 0; i < impulseCalculators.size(); i++)
            {
               SingleContactImpulseCalculator impulseCalculator = impulseCalculators.get(i);
               impulseCalculator.updateImpulse(alpha);
               double updateMagnitude = impulseCalculator.getVelocityUpdate();
               if (verbose)
                  System.out.println("Calc index: " + i + ", closing contact: " + impulseCalculator.isContactClosing() + ", impulse update: "
                        + impulseCalculator.getImpulseUpdate() + ", velocity update: " + impulseCalculator.getVelocityUpdate());
               maxUpdateMagnitude = Math.max(maxUpdateMagnitude, updateMagnitude);
               impulseCalculator.applyImpulseLazy();

               if (impulseCalculator.isContactClosing())
                  numberOfClosingContacts++;
            }

            iterationCounter++;

            if (iterationCounter == 1 && numberOfClosingContacts <= 1)
               break;

            alpha = alpha_min + gamma * (alpha - alpha_min);

            if (iterationCounter > maxNumberOfIterations)
               throw new IllegalStateException("Unable to converge during Successive Over-Relaxation method");
         }

         return maxUpdateMagnitude;
      }
   }

   public void setTolerance(double tolerance)
   {
      this.tolerance = tolerance;
   }

   public void setSingleContactTolerance(double gamma)
   {
      impulseCalculators.forEach(calculator -> calculator.setTolerance(gamma));
   }

   public void applyJointVelocityChange(Map<RigidBodyBasics, SingleRobotForwardDynamicsPlugin> singleRobotPluginMap)
   {
      for (SingleContactImpulseCalculator impulseCalculator : impulseCalculators)
      {
         if (!impulseCalculator.isContactClosing())
            continue;

         CollisionResult collision = impulseCalculator.getCollisionResult();
         RigidBodyBasics rootBodyA = collision.getCollidableA().getRootBody();
         RigidBodyBasics rootBodyB = collision.getCollidableB().getRootBody();

         SingleRobotForwardDynamicsPlugin robotPluginA = singleRobotPluginMap.get(rootBodyA);
         robotPluginA.addJointVelocities(impulseCalculator.computeJointVelocityChangeA());

         if (rootBodyB != null)
         {
            SingleRobotForwardDynamicsPlugin robotPluginB = singleRobotPluginMap.get(rootBodyB);
            robotPluginB.addJointVelocities(impulseCalculator.computeJointVelocityChangeB());
         }
      }
   }

   public int getNumberOfIterations()
   {
      return iterationCounter;
   }

   public List<SingleContactImpulseCalculator> getImpulseCalculators()
   {
      return impulseCalculators;
   }

   public static List<MultiContactImpulseCalculator> createMultiRobotCollisionGroups(ReferenceFrame rootFrame, double dt,
                                                                                     Map<RigidBodyBasics, ForwardDynamicsCalculator> robotForwardDynamicsCalculatorMap,
                                                                                     List<MultiRobotCollisionGroup> collisionGroups)
   {
      List<MultiContactImpulseCalculator> calculators = new ArrayList<>();

      for (MultiRobotCollisionGroup collisionGroup : collisionGroups)
      {
         calculators.add(new MultiContactImpulseCalculator(rootFrame, dt, robotForwardDynamicsCalculatorMap, collisionGroup));
      }

      return calculators;
   }
}
