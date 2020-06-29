package us.ihmc.robotics.physics;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;

/**
 * Inspired from: <i>Per-Contact Iteration Method for Solving Contact Dynamics</i>
 *
 * @author Sylvain Bertrand
 */
public class MultiContactImpulseCalculator
{
   private final ReferenceFrame rootFrame;

   private final List<SingleContactImpulseCalculator> contactCalculators = new ArrayList<>();
   private final List<RobotJointLimitImpulseBasedCalculator> jointLimitCalculators = new ArrayList<>();
   private final List<ImpulseBasedConstraintCalculator> allCalculators = new ArrayList<>();
   private final Map<RigidBodyBasics, List<Supplier<DMatrixRMaj>>> robotToCalculatorsOutputMap = new HashMap<>();

   private double alphaMin = 0.7;
   private double gamma = 0.99;
   private double tolerance = 1.0e-6;
   private boolean solveContactsIndependentlyOnFailure = false;

   private int maxNumberOfIterations = 100;
   private int iterationCounter = 0;

   private static boolean hasCalculatorFailedOnce = false;

   private Map<RigidBodyBasics, PhysicsEngineRobotData> robots;

   public MultiContactImpulseCalculator(ReferenceFrame rootFrame)
   {
      this.rootFrame = rootFrame;
   }

   public void configure(Map<RigidBodyBasics, PhysicsEngineRobotData> robots, MultiRobotCollisionGroup collisionGroup)
   {
      this.robots = robots;

      contactCalculators.clear();
      jointLimitCalculators.clear();
      allCalculators.clear();
      robotToCalculatorsOutputMap.clear();

      for (RigidBodyBasics rootBody : collisionGroup.getRootBodies())
      {
         PhysicsEngineRobotData robot = robots.get(rootBody);
         jointLimitCalculators.add(robot.getJointLimitConstraintCalculator());
      }

      for (int i = 0; i < collisionGroup.getNumberOfCollisions(); i++)
      {
         CollisionResult collisionResult = collisionGroup.getGroupCollisions().get(i);

         RigidBodyBasics rootA = collisionResult.getCollidableA().getRootBody();
         RigidBodyBasics rootB = collisionResult.getCollidableB().getRootBody();
         SingleContactImpulseCalculator calculator;

         if (rootB == null)
         {
            calculator = robots.get(rootA).getOrCreateEnvironmentContactConstraintCalculator();
         }
         else
         {
            calculator = robots.get(rootA).getOrCreateInterRobotContactConstraintCalculator(robots.get(rootB));
         }

         calculator.setCollision(collisionResult);
         contactCalculators.add(calculator);
      }

      allCalculators.addAll(contactCalculators);
      allCalculators.addAll(jointLimitCalculators);

      for (ImpulseBasedConstraintCalculator calculator : allCalculators)
      {
         for (int i = 0; i < calculator.getNumberOfRobotsInvolved(); i++)
         {
            final int robotIndex = i;
            RigidBodyBasics roobtBody = calculator.getRootBody(i);
            List<Supplier<DMatrixRMaj>> robotCalculatorsOutput = robotToCalculatorsOutputMap.get(roobtBody);
            if (robotCalculatorsOutput == null)
            {
               robotCalculatorsOutput = new ArrayList<>();
               robotToCalculatorsOutputMap.put(roobtBody, robotCalculatorsOutput);
            }
            robotCalculatorsOutput.add(() -> calculator.getJointVelocityChange(robotIndex));
         }

         CombinedRigidBodyTwistProviders externalRigidBodyTwistModifier = assembleExternalRigidBodyTwistModifierForCalculator(calculator);
         CombinedJointStateProviders externalJointTwistModifier = assembleExternalJointTwistModifierForCalculator(calculator);
         calculator.setExternalTwistModifiers(externalRigidBodyTwistModifier, externalJointTwistModifier);
      }
   }

   public double computeImpulses(double time, double dt, boolean verbose)
   {
      for (ImpulseBasedConstraintCalculator calculator : allCalculators)
      { // Request the calculators to predict velocity without applying impulse.
         calculator.initialize(dt);
      }

      for (Iterator<RobotJointLimitImpulseBasedCalculator> iterator = jointLimitCalculators.iterator(); iterator.hasNext();)
      { // Once initialized, if the a joint limit calculator does not detect joints about to violate their limits, we skip it for this iteration.
         RobotJointLimitImpulseBasedCalculator calculator = iterator.next();
         if (calculator.getActiveLimits().isEmpty())
         {
            allCalculators.remove(calculator);
            iterator.remove();
         }
      }

      for (ImpulseBasedConstraintCalculator calculator : allCalculators)
      {
         /*
          * Request each calculator to compute the apparent inertia coefficients needed for the calculator
          * itself and also for the other calculators to propagate the twist change from one calculator to
          * the other.
          */
         List<? extends RigidBodyBasics> rigidBodyTargets = collectRigidBodyTargetsForCalculator(calculator);
         List<? extends JointBasics> jointTargets = collectJointTargetsForCalculator(calculator);
         calculator.updateInertia(rigidBodyTargets, jointTargets);
      }

      if (allCalculators.size() == 1)
      {
         /*
          * Single calculator => no need to use the successive over-relaxation method, a single update is
          * enough to evaluate the solution.
          */
         ImpulseBasedConstraintCalculator calculator = allCalculators.get(0);
         calculator.computeImpulse(dt);
         calculator.finalizeImpulse();
         return 0.0;
      }
      else
      { // Successive over-relaxation method to evaluate multiple inter-dependent constraints.
         double alpha = 1.0;
         double maxUpdateMagnitude = Double.POSITIVE_INFINITY;

         iterationCounter = 0;

         while (maxUpdateMagnitude > tolerance)
         {
            maxUpdateMagnitude = Double.NEGATIVE_INFINITY;
            int numberOfClosingContacts = 0;

            for (int i = 0; i < allCalculators.size(); i++)
            { // Request every calculator to update.
               ImpulseBasedConstraintCalculator calculator = allCalculators.get(i);
               calculator.updateImpulse(dt, alpha, false);
               calculator.updateTwistModifiers();
               double updateMagnitude = calculator.getVelocityUpdate();
               if (verbose)
               {
                  if (calculator instanceof SingleContactImpulseCalculator)
                  {
                     SingleContactImpulseCalculator contactCalculator = (SingleContactImpulseCalculator) calculator;
                     System.out.println("Iteration " + iterationCounter + ", calc index: " + i + ", active: " + contactCalculator.isConstraintActive()
                           + ", closing: " + contactCalculator.isContactClosing() + ", impulse update: " + contactCalculator.getImpulseUpdate()
                           + ", velocity update: " + contactCalculator.getVelocityUpdate());
                  }
                  else
                  {
                     System.out.println("Iteration " + iterationCounter + ", alc index: " + i + ", active: " + calculator.isConstraintActive()
                           + ", impulse update: " + calculator.getImpulseUpdate() + ", velocity update: " + calculator.getVelocityUpdate());
                  }
               }
               maxUpdateMagnitude = Math.max(maxUpdateMagnitude, updateMagnitude);

               if (calculator.isConstraintActive())
                  numberOfClosingContacts++;
            }

            iterationCounter++;

            if (iterationCounter == 1 && numberOfClosingContacts <= 1)
               break;

            alpha = alphaMin + gamma * (alpha - alphaMin);

            if (iterationCounter > maxNumberOfIterations)
            {
               if (!hasCalculatorFailedOnce)
               {
                  LogTools.error("Unable to converge during Successive Over-Relaxation method. Only reporting the first failure.");
                  hasCalculatorFailedOnce = true;
               }
               break;
            }
         }

         if (solveContactsIndependentlyOnFailure && iterationCounter > maxNumberOfIterations)
         { // The solver failed.
            for (ImpulseBasedConstraintCalculator calculator : allCalculators)
            { // Solving the contacts independently.
               calculator.computeImpulse(dt);
               calculator.finalizeImpulse();
            }
         }
         else
         {
            for (ImpulseBasedConstraintCalculator calculator : allCalculators)
            {
               calculator.finalizeImpulse();
            }
         }

         return maxUpdateMagnitude;
      }
   }

   public void setAlphaMin(double alphaMin)
   {
      this.alphaMin = alphaMin;
   }

   public void setGamma(double gamma)
   {
      this.gamma = gamma;
   }

   public void setTolerance(double tolerance)
   {
      this.tolerance = tolerance;
   }

   public void setMaxNumberOfIterations(int maxNumberOfIterations)
   {
      this.maxNumberOfIterations = maxNumberOfIterations;
   }

   public void setSolveContactsIndependentlyOnFailure(boolean solveContactsIndependentlyOnFailure)
   {
      this.solveContactsIndependentlyOnFailure = solveContactsIndependentlyOnFailure;
   }

   public void setSingleContactTolerance(double gamma)
   {
      contactCalculators.forEach(calculator -> calculator.setTolerance(gamma));
   }

   public void setConstraintParameters(ConstraintParametersReadOnly constraintParameters)
   {
      jointLimitCalculators.forEach(calculator -> calculator.setConstraintParameters(constraintParameters));
   }

   public void setContactParameters(ContactParametersReadOnly contactParameters)
   {
      contactCalculators.forEach(calculator -> calculator.setContactParameters(contactParameters));
   }

   public void applyJointVelocityChange(RigidBodyBasics rootBody, Consumer<DMatrixRMaj> jointVelocityChangeConsumer)
   {
      List<Supplier<DMatrixRMaj>> robotCalculatorsOutput = robotToCalculatorsOutputMap.get(rootBody);

      if (robotCalculatorsOutput == null)
         return;

      robotCalculatorsOutput.forEach(output -> jointVelocityChangeConsumer.accept(output.get()));
   }

   public void applyJointVelocityChanges()
   {
      for (ImpulseBasedConstraintCalculator calculator : allCalculators)
      {
         if (!calculator.isConstraintActive())
            continue;

         for (int i = 0; i < calculator.getNumberOfRobotsInvolved(); i++)
         {
            RigidBodyBasics rootBody = calculator.getRootBody(i);
            SingleRobotFirstOrderIntegrator integrator = robots.get(rootBody).getIntegrator();
            DMatrixRMaj jointVelocityChange = calculator.getJointVelocityChange(i);
            integrator.addJointVelocityChange(jointVelocityChange);
         }
      }
   }

   public void readExternalWrenches(double dt, List<ExternalWrenchReader> externalWrenchReaders)
   {
      contactCalculators.forEach(calculator -> calculator.readExternalWrench(dt, externalWrenchReaders));
   }

   public double getAlphaMin()
   {
      return alphaMin;
   }

   public double getGamma()
   {
      return gamma;
   }

   public double getTolerance()
   {
      return tolerance;
   }

   public int getMaxNumberOfIterations()
   {
      return maxNumberOfIterations;
   }

   public int getNumberOfIterations()
   {
      return iterationCounter;
   }

   public List<SingleContactImpulseCalculator> getImpulseCalculators()
   {
      return contactCalculators;
   }

   public boolean hasConverged()
   {
      return iterationCounter <= maxNumberOfIterations;
   }

   private CombinedRigidBodyTwistProviders assembleExternalRigidBodyTwistModifierForCalculator(ImpulseBasedConstraintCalculator calculator)
   {
      CombinedRigidBodyTwistProviders rigidBodyTwistProviders = new CombinedRigidBodyTwistProviders(rootFrame);

      for (ImpulseBasedConstraintCalculator otherCalculator : allCalculators)
      {
         if (otherCalculator != calculator)
         {
            for (int i = 0; i < otherCalculator.getNumberOfRobotsInvolved(); i++)
            {
               rigidBodyTwistProviders.add(otherCalculator.getRigidBodyTwistChangeProvider(i));
            }
         }
      }

      return rigidBodyTwistProviders;
   }

   private CombinedJointStateProviders assembleExternalJointTwistModifierForCalculator(ImpulseBasedConstraintCalculator calculator)
   {
      CombinedJointStateProviders jointTwistProviders = new CombinedJointStateProviders(JointStateType.VELOCITY);

      for (ImpulseBasedConstraintCalculator otherCalculator : allCalculators)
      {
         if (otherCalculator != calculator)
         {
            for (int i = 0; i < otherCalculator.getNumberOfRobotsInvolved(); i++)
            {
               jointTwistProviders.add(otherCalculator.getJointTwistChangeProvider(i));
            }
         }
      }

      return jointTwistProviders;
   }

   private List<RigidBodyBasics> collectRigidBodyTargetsForCalculator(ImpulseBasedConstraintCalculator calculator)
   {
      List<RigidBodyBasics> rigidBodyTargets = new ArrayList<>();

      for (ImpulseBasedConstraintCalculator otherCalculator : allCalculators)
      {
         if (otherCalculator != calculator)
            rigidBodyTargets.addAll(otherCalculator.getRigidBodyTargets());
      }

      return rigidBodyTargets;
   }

   private List<JointBasics> collectJointTargetsForCalculator(ImpulseBasedConstraintCalculator calculator)
   {
      List<JointBasics> jointTargets = new ArrayList<>();

      for (ImpulseBasedConstraintCalculator otherCalculator : allCalculators)
      {
         if (otherCalculator != calculator)
            jointTargets.addAll(otherCalculator.getJointTargets());
      }

      return jointTargets;
   }
}
