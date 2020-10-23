package us.ihmc.robotics.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Arrays;
import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.RandomMatrices_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools.RandomFloatingRevoluteJointChain;
import us.ihmc.mecano.tools.MultiBodySystemStateIntegrator;
import us.ihmc.mecano.tools.MultiBodySystemTools;

public class SingleRobotFirstOrderIntegratorTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final int ITERATIONS = 1000;
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testAgainstMecanoIntegrator()
   {
      Random random = new Random(3465467567L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double dt = random.nextDouble();
         RandomFloatingRevoluteJointChain robotOriginal = new RandomFloatingRevoluteJointChain(random, 20);
         MultiBodySystemBasics robotClone = MultiBodySystemBasics.clone(MultiBodySystemReadOnly.toMultiBodySystemInput(robotOriginal.getElevator()),
                                                                        worldFrame);

         MultiBodySystemStateIntegrator mecanoIntegrator = new MultiBodySystemStateIntegrator(dt);
         SingleRobotFirstOrderIntegrator integrator = new SingleRobotFirstOrderIntegrator(robotClone);

         for (JointStateType stateSelection : Arrays.asList(JointStateType.CONFIGURATION, JointStateType.VELOCITY, JointStateType.ACCELERATION))
         {
            robotOriginal.nextState(random, stateSelection);
            MultiBodySystemTools.copyJointsState(robotOriginal.getJoints(), robotClone.getAllJoints(), stateSelection);
         }

         mecanoIntegrator.doubleIntegrateFromAccelerationSubtree(robotOriginal.getElevator());
         integrator.integrate(dt);

         for (int jointIndex = 0; jointIndex < robotOriginal.getJoints().size(); jointIndex++)
         {
            JointBasics originalJoint = robotOriginal.getJoints().get(jointIndex);
            JointBasics cloneJoint = robotClone.getAllJoints().get(jointIndex);

            if (originalJoint instanceof FloatingJointBasics)
            {
               FloatingJointBasics originalFloatingJoint = (FloatingJointBasics) originalJoint;
               FloatingJointBasics cloneFloatingJoint = (FloatingJointBasics) cloneJoint;
               EuclidGeometryTestTools.assertPose3DEquals(originalFloatingJoint.getJointPose(), cloneFloatingJoint.getJointPose(), EPSILON);
               EuclidCoreTestTools.assertTuple3DEquals(originalFloatingJoint.getJointTwist().getAngularPart(),
                                                       cloneFloatingJoint.getJointTwist().getAngularPart(),
                                                       EPSILON);
               EuclidCoreTestTools.assertTuple3DEquals(originalFloatingJoint.getJointTwist().getLinearPart(),
                                                       cloneFloatingJoint.getJointTwist().getLinearPart(),
                                                       EPSILON);
               EuclidCoreTestTools.assertTuple3DEquals(originalFloatingJoint.getJointAcceleration().getAngularPart(),
                                                       cloneFloatingJoint.getJointAcceleration().getAngularPart(),
                                                       EPSILON);
               EuclidCoreTestTools.assertTuple3DEquals(originalFloatingJoint.getJointAcceleration().getLinearPart(),
                                                       cloneFloatingJoint.getJointAcceleration().getLinearPart(),
                                                       EPSILON);
            }
            else
            {
               OneDoFJointBasics originalOneDoFJoint = (OneDoFJointBasics) originalJoint;
               OneDoFJointBasics cloneOneDoFJoint = (OneDoFJointBasics) cloneJoint;
               assertEquals(originalOneDoFJoint.getQ(), cloneOneDoFJoint.getQ(), EPSILON);
               assertEquals(originalOneDoFJoint.getQd(), cloneOneDoFJoint.getQd(), EPSILON);
               assertEquals(originalOneDoFJoint.getQdd(), cloneOneDoFJoint.getQdd(), EPSILON);
            }
         }
      }
   }

   @Test
   public void testVelocityChangeIntegration()
   {
      Random random = new Random(3465467567L);

      for (int i = 0; i < ITERATIONS; i++)
      { // For the clone robot we add velocity change, that velocity change is transformed into an acceleration change that is added to the robot original
         double dt = random.nextDouble();
         RandomFloatingRevoluteJointChain randomRobot = new RandomFloatingRevoluteJointChain(random, 20);
         MultiBodySystemBasics robotOriginal = MultiBodySystemBasics.toMultiBodySystemBasics(randomRobot.getElevator());
         MultiBodySystemBasics robotClone = MultiBodySystemBasics.clone(robotOriginal, worldFrame);

         SingleRobotFirstOrderIntegrator integrator = new SingleRobotFirstOrderIntegrator(robotOriginal);
         SingleRobotFirstOrderIntegrator integratorWithVelocity = new SingleRobotFirstOrderIntegrator(robotClone);

         for (JointStateType stateSelection : Arrays.asList(JointStateType.CONFIGURATION, JointStateType.VELOCITY, JointStateType.ACCELERATION))
         {
            MultiBodySystemRandomTools.nextState(random, stateSelection, robotOriginal.getAllJoints());
            MultiBodySystemTools.copyJointsState(robotOriginal.getAllJoints(), robotClone.getAllJoints(), stateSelection);
         }

         int numberOfDoFs = MultiBodySystemTools.computeDegreesOfFreedom(robotOriginal.getAllJoints());
         DMatrixRMaj velocityChange = RandomMatrices_DDRM.rectangle(numberOfDoFs, 1, -10.0, 10.0, random);
         DMatrixRMaj originalJointAcceleration = new DMatrixRMaj(numberOfDoFs, 1);
         MultiBodySystemTools.extractJointsState(robotOriginal.getAllJoints(), JointStateType.ACCELERATION, originalJointAcceleration);
         DMatrixRMaj accelerationChange = new DMatrixRMaj(velocityChange);
         CommonOps_DDRM.scale(1.0 / dt, accelerationChange);
         CommonOps_DDRM.addEquals(originalJointAcceleration, accelerationChange);
         MultiBodySystemTools.insertJointsState(robotOriginal.getAllJoints(), JointStateType.ACCELERATION, originalJointAcceleration);

         integrator.integrate(dt);
         integratorWithVelocity.addJointVelocityChange(velocityChange);
         integratorWithVelocity.integrate(dt);

         for (int jointIndex = 0; jointIndex < robotOriginal.getAllJoints().size(); jointIndex++)
         {
            JointBasics originalJoint = robotOriginal.getAllJoints().get(jointIndex);
            JointBasics cloneJoint = robotClone.getAllJoints().get(jointIndex);

            if (originalJoint instanceof FloatingJointBasics)
            {
               FloatingJointBasics originalFloatingJoint = (FloatingJointBasics) originalJoint;
               FloatingJointBasics cloneFloatingJoint = (FloatingJointBasics) cloneJoint;
               EuclidGeometryTestTools.assertPose3DEquals(originalFloatingJoint.getJointPose(), cloneFloatingJoint.getJointPose(), EPSILON);
               EuclidCoreTestTools.assertTuple3DEquals(originalFloatingJoint.getJointTwist().getAngularPart(),
                                                       cloneFloatingJoint.getJointTwist().getAngularPart(),
                                                       EPSILON);
               EuclidCoreTestTools.assertTuple3DEquals(originalFloatingJoint.getJointTwist().getLinearPart(),
                                                       cloneFloatingJoint.getJointTwist().getLinearPart(),
                                                       EPSILON);
               // Acceleration will be different as the velocity change does not impact acceleration
            }
            else
            {
               OneDoFJointBasics originalOneDoFJoint = (OneDoFJointBasics) originalJoint;
               OneDoFJointBasics cloneOneDoFJoint = (OneDoFJointBasics) cloneJoint;
               assertEquals(originalOneDoFJoint.getQ(), cloneOneDoFJoint.getQ(), EPSILON);
               assertEquals(originalOneDoFJoint.getQd(), cloneOneDoFJoint.getQd(), EPSILON);
               // Acceleration will be different as the velocity change does not impact acceleration
            }
         }
      }
   }
}
