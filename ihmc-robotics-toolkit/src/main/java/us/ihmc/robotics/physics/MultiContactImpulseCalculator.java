package us.ihmc.robotics.physics;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
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
   private final Map<RigidBodyBasics, List<Supplier<DenseMatrix64F>>> robotToCalculatorsOutputMap = new HashMap<>();

   private double alpha_min = 0.3;
   private double gamma = 0.99;
   private double tolerance = 1.0e-6;

   private int maxNumberOfIterations = 100;
   private int iterationCounter = 0;

   public MultiContactImpulseCalculator(ReferenceFrame rootFrame)
   {
      this.rootFrame = rootFrame;
   }

   public void configure(Map<RigidBodyBasics, PhysicsEngineRobotData> robots, MultiRobotCollisionGroup collisionGroup)
   {
      contactCalculators.clear();
      jointLimitCalculators.clear();
      allCalculators.clear();
      robotToCalculatorsOutputMap.clear();

      for (RigidBodyBasics rootBody : collisionGroup.getRootBodies())
      {
         PhysicsEngineRobotData robot = robots.get(rootBody);
         jointLimitCalculators.add(robot.getJointLimitConstraintCalculator());
      }

      for (CollisionResult collisionResult : collisionGroup.getGroupCollisions())
      {
         RigidBodyBasics rootA = collisionResult.getCollidableA().getRootBody();
         RigidBodyBasics rootB = collisionResult.getCollidableB().getRootBody();
         PhysicsEngineRobotData robotA = robots.get(rootA);
         PhysicsEngineRobotData robotB = rootB != null ? robots.get(rootB) : null;

         contactCalculators.add(new SingleContactImpulseCalculator(collisionResult, rootFrame, robotA, robotB));
      }

      allCalculators.addAll(contactCalculators);
      allCalculators.addAll(jointLimitCalculators);

      for (ImpulseBasedConstraintCalculator calculator : allCalculators)
      {
         for (int i = 0; i < calculator.getNumberOfRobotsInvolved(); i++)
         {
            final int robotIndex = i;
            RigidBodyBasics roobtBody = calculator.getRootBody(i);
            List<Supplier<DenseMatrix64F>> robotCalculatorsOutput = robotToCalculatorsOutputMap.get(roobtBody);
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

   public double computeImpulses(double dt, boolean verbose)
   {
      for (ImpulseBasedConstraintCalculator calculator : allCalculators)
      {
         calculator.initialize(dt);
      }

      for (ImpulseBasedConstraintCalculator calculator : allCalculators)
      {
         List<? extends RigidBodyBasics> rigidBodyTargets = collectRigidBodyTargetsForCalculator(calculator);
         List<? extends JointBasics> jointTargets = collectJointTargetsForCalculator(calculator);
         calculator.updateInertia(rigidBodyTargets, jointTargets);
      }

      if (allCalculators.size() == 1)
      {
         allCalculators.get(0).computeImpulse(dt);
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

            for (int i = 0; i < allCalculators.size(); i++)
            {
               ImpulseBasedConstraintCalculator calculator = allCalculators.get(i);
               calculator.updateImpulse(dt, alpha);
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
                  else
                  {
                     System.out.println("Calc index: " + i + ", active: " + calculator.isConstraintActive() + ", impulse update: "
                           + calculator.getImpulseUpdate() + ", velocity update: " + calculator.getVelocityUpdate());
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

   public void applyJointVelocityChange(RigidBodyBasics rootBody, Consumer<DenseMatrix64F> jointVelocityChangeConsumer)
   {
      List<Supplier<DenseMatrix64F>> robotCalculatorsOutput = robotToCalculatorsOutputMap.get(rootBody);

      if (robotCalculatorsOutput == null)
         return;

      robotCalculatorsOutput.forEach(output ->
      {
         DenseMatrix64F jointVelocityChange = output.get();
         if (jointVelocityChange != null)
            jointVelocityChangeConsumer.accept(jointVelocityChange);
      });
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
