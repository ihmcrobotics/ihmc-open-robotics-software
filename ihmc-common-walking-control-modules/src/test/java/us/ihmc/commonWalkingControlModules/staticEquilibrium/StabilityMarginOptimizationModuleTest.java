package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.spatial.SpatialForce;

public class StabilityMarginOptimizationModuleTest
{
   @Test
   public void testCenterOfMassStabilityMargin_SimpleFlatGround()
   {
      double epsilon = 1.0e-10;
      double gravity = 9.81;
      double robotMass = 10.0;

      MutableWholeBodyContactState triangleFlatGround = ContactStateExamples.createTriangleFlatGround();
      CenterOfMassStabilityMarginOptimizationModule optimizationModule = new CenterOfMassStabilityMarginOptimizationModule(robotMass);
      optimizationModule.updateContactState(triangleFlatGround);

      optimizationModule.setEqualityConstraintEpsilon(0.0);

      boolean success = optimizationModule.solve(0.5, 0.5);
      Assertions.assertTrue(success);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      ///////////////////  Test static equilibrium constraint is met

      Point2DReadOnly optimizedCoM = optimizationModule.getOptimizedStabilityPoint();
      SpatialForce netWrenchComputed = new SpatialForce();

      for (int i = 0; i < triangleFlatGround.getNumberOfContactPoints(); i++)
      {
         Vector3D resolvedForce = new Vector3D();
         optimizationModule.getResolvedForce(i, resolvedForce);

         FramePoint3D contactPoint = new FramePoint3D(triangleFlatGround.getContactFrame(i));
         contactPoint.changeFrame(ReferenceFrame.getWorldFrame());

         Vector3D resolvedTorque = new Vector3D();
         resolvedTorque.cross(contactPoint, resolvedForce);

         netWrenchComputed.add(resolvedTorque, resolvedForce);
      }

      SpatialForce netWrenchExpected = new SpatialForce();
      netWrenchExpected.setLinearPartZ(robotMass * gravity);
      netWrenchExpected.setAngularPartX(optimizedCoM.getY() * robotMass * gravity);
      netWrenchExpected.setAngularPartY(-optimizedCoM.getX() * robotMass * gravity);

      Assertions.assertTrue(netWrenchExpected.epsilonEquals(netWrenchComputed, epsilon), "Static equilibrium condition not met");

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      ///////////////////  Test basis vectors

      for (int i = 0; i < triangleFlatGround.getNumberOfContactPoints(); i++)
      {
         double coefficientOfFriction = triangleFlatGround.getCoefficientOfFriction(i);
         double angleExpected = Math.atan(coefficientOfFriction);

         ReferenceFrame contactFrame = triangleFlatGround.getContactFrame(i);
         FrameVector3D contactNormal = new FrameVector3D(contactFrame, Axis3D.Z);
         contactNormal.changeFrame(ReferenceFrame.getWorldFrame());

         for (int j = 0; j < StabilityMarginOptimizationModule.NUM_BASIS_VECTORS; j++)
         {
            FrameVector3D basisVector = new FrameVector3D(optimizationModule.getBasisVector(i, j));
            Assertions.assertTrue(Math.abs(basisVector.norm() - 1.0) < epsilon, "Basis vector is not unit length");

            basisVector.changeFrame(ReferenceFrame.getWorldFrame());
            double angleComputed = Math.acos(basisVector.dot(contactNormal));
            Assertions.assertTrue(Math.abs(angleComputed - angleExpected) < epsilon, "Basis vector is invalid");
         }
      }
   }
}
