package us.ihmc.robotics.physics;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.algorithms.ForwardDynamicsCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;

/**
 * Inspired from: <i>Per-Contact Iteration Method for Solving Contact Dynamics</i>
 *
 * @author Sylvain Bertrand
 */
public class MultiContactImpulseCalculator
{
   private final List<SingleContactImpulseCalculator> contactCalculators = new ArrayList<>();
   private final List<RobotJointLimitImpulseBasedCalculator> jointLimitCalculators = new ArrayList<>();
   private final List<ImpulseBasedConstraintCalculator> calculators = new ArrayList<>();

   private double alpha_min = 0.3;
   private double gamma = 0.99;
   private double tolerance = 1.0e-6;

   private int maxNumberOfIterations = 500;
   private int iterationCounter = 0;

   public MultiContactImpulseCalculator(ReferenceFrame rootFrame, double dt, Map<RigidBodyBasics, ForwardDynamicsCalculator> robotForwardDynamicsCalculatorMap,
                                        MultiRobotCollisionGroup collisionGroup)
   {
      for (RigidBodyBasics rootBody : collisionGroup.getRootBodies())
      {
         ForwardDynamicsCalculator robot = robotForwardDynamicsCalculatorMap.get(rootBody);
         jointLimitCalculators.add(new RobotJointLimitImpulseBasedCalculator(rootBody, dt, robot));
      }

      for (CollisionResult collisionResult : collisionGroup.getGroupCollisions())
      {
         RigidBodyBasics rootA = collisionResult.getCollidableA().getRootBody();
         RigidBodyBasics rootB = collisionResult.getCollidableB().getRootBody();
         ForwardDynamicsCalculator robotA = robotForwardDynamicsCalculatorMap.get(rootA);
         ForwardDynamicsCalculator robotB = rootB != null ? robotForwardDynamicsCalculatorMap.get(rootB) : null;

         contactCalculators.add(new SingleContactImpulseCalculator(collisionResult, rootFrame, dt, robotA, robotB));
      }

      calculators.addAll(contactCalculators);
      calculators.addAll(jointLimitCalculators);

      for (ImpulseBasedConstraintCalculator calculator : calculators)
      {
         CombinedRigidBodyTwistProviders externalRigidBodyTwistModifier = calculators.stream().filter(other -> other != calculator)
                                                                                     .collect(CombinedRigidBodyTwistProviders.collectFromCalculator(rootFrame));
         CombinedJointStateProviders externalJointTwistModifier = calculators.stream().filter(other -> other != calculator)
                                                                             .collect(CombinedJointStateProviders.collectFromCalculator(JointStateType.VELOCITY));
         calculator.setExternalTwistModifiers(externalRigidBodyTwistModifier, externalJointTwistModifier);
      }
   }

   public double computeImpulses(boolean verbose)
   {
      if (calculators.size() == 1)
      {
         calculators.get(0).computeImpulse();
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

            for (int i = 0; i < calculators.size(); i++)
            {
               ImpulseBasedConstraintCalculator calculator = calculators.get(i);
               calculator.updateImpulse(alpha);
               double updateMagnitude = calculator.getVelocityUpdate();
               if (verbose)
               {
                  if (calculator instanceof SingleContactImpulseCalculator)
                  {
                     SingleContactImpulseCalculator contactCalculator = (SingleContactImpulseCalculator) calculator;
                     System.out.println("Calc index: " + i + ", active: " + contactCalculator.isConstraintActive() + ", closing: "
                           + contactCalculator.isContactClosing() + ", impulse update: " + contactCalculator.getImpulseUpdate() + ", velocity update: "
                           + contactCalculator.getVelocityUpdate());
                  }
                  else if (calculator instanceof OneDoFJointLimitImpulseBasedCalculator)
                  {
                     OneDoFJointLimitImpulseBasedCalculator jointCalculator = (OneDoFJointLimitImpulseBasedCalculator) calculator;
                     System.out.println("Calc index: " + i + ", active: " + jointCalculator.isConstraintActive() + ", impulse update: "
                           + jointCalculator.getImpulseUpdate() + ", velocity update: " + jointCalculator.getVelocityUpdate());
                  }
               }
               maxUpdateMagnitude = Math.max(maxUpdateMagnitude, updateMagnitude);

               if (calculator.isConstraintActive())
                  numberOfClosingContacts++;
            }

            iterationCounter++;

            if (iterationCounter == 1 && numberOfClosingContacts <= 1)
               break;

            alpha = alpha_min + gamma * (alpha - alpha_min);

            if (iterationCounter > maxNumberOfIterations)
            {
               System.err.println("Unable to converge during Successive Over-Relaxation method");
               break;
            }
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
      contactCalculators.forEach(calculator -> calculator.setTolerance(gamma));
   }

   public void setSpringConstant(double springConstant)
   {
      contactCalculators.forEach(calculator -> calculator.setSpringConstant(springConstant));
   }

   public void applyJointVelocityChange(Map<RigidBodyBasics, SingleRobotForwardDynamicsPlugin> singleRobotPluginMap)
   {
      for (ImpulseBasedConstraintCalculator calculator : calculators)
      {
         if (!calculator.isConstraintActive())
            continue;

         for (int i = 0; i < calculator.getNumberOfRobotsInvolved(); i++)
         {
            RigidBodyBasics rootBody = calculator.getRootBody(i);
            SingleRobotForwardDynamicsPlugin robotPlugin = singleRobotPluginMap.get(rootBody);
            robotPlugin.addJointVelocities(calculator.getJointVelocityChange(i));
         }
      }
   }

   public int getNumberOfIterations()
   {
      return iterationCounter;
   }

   public List<SingleContactImpulseCalculator> getImpulseCalculators()
   {
      return contactCalculators;
   }
}
