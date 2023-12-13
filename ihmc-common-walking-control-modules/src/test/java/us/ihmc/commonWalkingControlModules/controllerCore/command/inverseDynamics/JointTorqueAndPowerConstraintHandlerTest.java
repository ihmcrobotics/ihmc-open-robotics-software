package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import java.util.Random;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;

public class JointTorqueAndPowerConstraintHandlerTest
{
   private final JointTorqueAndPowerConstraintHandler handler = new JointTorqueAndPowerConstraintHandler();
   
   @Test
   public void testJointTorqueAndPowerConstraintHandlerOnFixedBaseMechanism()
   {
      RigidBody elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
      RevoluteJoint joint1 = new RevoluteJoint("joint", elevator, Axis3D.X);
      OneDoFJointBasics joint = joint1;

      joint.setEffortLimits(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
      joint.setQd(0.0);

      /* Test with random desired torques and random torque and power limits */
      int numberOfTests = 1000;
      Random random = new Random(23890);
      for (int i = 0; i < numberOfTests; i++)
      {
         //reset effort limits
         joint.setEffortLimits(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

         // random power limits and random joint velocity
         double powerLimitLower = -EuclidCoreRandomTools.nextDouble(random, 1.0, 100.0);
         double powerLimitUpper = EuclidCoreRandomTools.nextDouble(random, 1.0, 100.0);
         joint.setQd(EuclidCoreRandomTools.nextDouble(random, 2.0));

         // randomly decide whether to consider joint effort limits
         boolean hasTorqueConstraints = random.nextBoolean();
         if (hasTorqueConstraints)
         {
            joint.setEffortLimits(-EuclidCoreRandomTools.nextDouble(random, 1.0, 50.0), EuclidCoreRandomTools.nextDouble(random, 1.0, 50.0));
         }

         // execute method we are testing
         handler.computeTorqueConstraints(joint, powerLimitLower, powerLimitUpper, hasTorqueConstraints);
         double torqueLimitLower = handler.getTorqueLimitLower();
         double torqueLimitUpper = handler.getTorqueLimitUpper();
         double torqueLimitFromPowerLower = handler.getTorqueLimitsFromPowerLower();
         double torqueLimitFromPowerUpper = handler.getTorqueLimitsFromPowerUpper();

         //         System.out.println("Iter " + i + ", Pl " + torqueLimitFromPowerLower + ", Tl " + joint.getEffortLimitLower() + ", Lb " + torqueLimitLower + ",\tPu "
         //                            + torqueLimitFromPowerUpper + ", Tu " + joint.getEffortLimitUpper() + ", Ub " + torqueLimitUpper);

         double epsilon = 1e-8; // epsilon 1e-12 is too restrictive, epsilon 1e-8 is really close, depends on jointTorqueWeight

         if (hasTorqueConstraints)
         {
            if (joint.getEffortLimitUpper() <= torqueLimitFromPowerUpper + epsilon)
            {
               // Assert torqueLimit equals joint effort limit if it is more restrictive
               Assertions.assertTrue(EuclidCoreTools.epsilonEquals(torqueLimitUpper, joint.getEffortLimitUpper(), epsilon));
            }
            else
            {
               // Assert torqueLimit equals power-based limit if it is more restrictive
               Assertions.assertTrue(EuclidCoreTools.epsilonEquals(torqueLimitUpper, torqueLimitFromPowerUpper, epsilon));
            }

            if (joint.getEffortLimitLower() >= torqueLimitFromPowerLower - epsilon)
            {
               // Assert torqueLimit equals joint effort limit if it is more restrictive
               Assertions.assertTrue(EuclidCoreTools.epsilonEquals(torqueLimitLower, joint.getEffortLimitLower(), epsilon));
            }
            else
            {
               // Assert torqueLimit equals power-based limit if it is more restrictive
               Assertions.assertTrue(EuclidCoreTools.epsilonEquals(torqueLimitLower, torqueLimitFromPowerLower, epsilon));
            }
         }
         else
         {
            // Assert torqueLimit equals power-based limit since there are no joint effort limits
            Assertions.assertTrue(EuclidCoreTools.epsilonEquals(torqueLimitUpper, torqueLimitFromPowerUpper, epsilon));
            Assertions.assertTrue(EuclidCoreTools.epsilonEquals(torqueLimitLower, torqueLimitFromPowerLower, epsilon));
         }
      }
   }
}
