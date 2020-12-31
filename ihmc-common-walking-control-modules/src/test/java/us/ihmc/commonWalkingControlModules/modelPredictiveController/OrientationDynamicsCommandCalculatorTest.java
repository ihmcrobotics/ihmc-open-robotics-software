package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.OrientationDynamicsCommand;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ZeroConeRotationCalculator;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.sql.Ref;
import java.util.Random;

import static us.ihmc.robotics.Assert.fail;

public class OrientationDynamicsCommandCalculatorTest
{
   @Test
   public void testCompute()
   {
      double gravityZ = -9.81;
      double omega = 3.0;
      double mu = 0.8;
      double dt = 1e-3;
      double mass = 1.5;

      FrameVector3D gravityVector = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);

      ContactStateMagnitudeToForceMatrixHelper rhoHelper = new ContactStateMagnitudeToForceMatrixHelper(4, 4, new ZeroConeRotationCalculator());
      CoefficientJacobianMatrixHelper helper = new CoefficientJacobianMatrixHelper(4, 4);
      ContactPlaneHelper contactPlaneHelper = new ContactPlaneHelper(4, 4, new ZeroConeRotationCalculator());

      MPCIndexHandler indexHandler = new MPCIndexHandler(4);
      SpatialInertia inertia = new SpatialInertia(ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
      inertia.setMomentOfInertia(10.0, 10.0, 5.0);
      inertia.setMass(mass);

      indexHandler.initialize((i) -> 4, 1);

      Random random = new Random(1738L);
      for (int i = 0; i < 1000; i++)
      {
         Orientation3DReadOnly orientationEstimate = EuclidCoreRandomTools.nextOrientation3D(random);
         Vector3DReadOnly angularVelocityEstimate = EuclidCoreRandomTools.nextVector3D(random);
         Point3DReadOnly comPositionEstimate = EuclidCoreRandomTools.nextPoint3D(random);

         double time = RandomNumbers.nextDouble(random, 0.0, 3.0);

         OrientationDynamicsCommand command = new OrientationDynamicsCommand();
         command.setOrientationEstimate(orientationEstimate);
         command.setAngularVelocityEstimate(angularVelocityEstimate);
         command.setComPositionEstimate(comPositionEstimate);
         command.setTimeOfObjective(time);
         command.setOmega(omega);
         command.setWeight(10);
         command.setSegmentNumber(0);
         command.setBodyInertia(inertia);

         OrientationDynamicsCommandCalculator calculator = new OrientationDynamicsCommandCalculator(indexHandler, mass);
         calculator.compute(command);

         fail();

      }
   }
}
