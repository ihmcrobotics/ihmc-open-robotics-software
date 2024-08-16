package us.ihmc.parameterEstimation.inertial;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.parameterEstimation.inertial.RigidBodyInertialParameters;
import us.ihmc.parameterEstimation.inertial.RigidBodyInertialParametersTools;

import static org.junit.jupiter.api.Assertions.*;

public class RigidBodyInertialParametersToolsTest
{
   ReferenceFrame bodyFrame = ReferenceFrame.getWorldFrame();
   ReferenceFrame expressedInFrame = ReferenceFrame.getWorldFrame();

   double mass = 1.0;
   Vector3D centerOfMassOffset = new Vector3D(1.0, 1.0, 1.0);
   Matrix3D momentOfInertia = new Matrix3D(new double[] {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
   SpatialInertia spatialInertia = new SpatialInertia(bodyFrame, expressedInFrame, momentOfInertia, mass, centerOfMassOffset);

   @Test
   public void testIsPhysicallyConsistent()
   {
      // Known passing example: unit mass, unit center of mass offset, identity moment of inertia matrix
      RigidBodyInertialParameters parameterVector = new RigidBodyInertialParameters(spatialInertia);
      assertTrue(RigidBodyInertialParametersTools.isPhysicallyConsistent(parameterVector));

      // Known failing example: zero mass, unit center of mass offset, identity moment of inertia matrix
      mass = 0.0;
      spatialInertia.setMass(mass);
      parameterVector.setParameterVectorPiBasis(spatialInertia);
      assertFalse(RigidBodyInertialParametersTools.isPhysicallyConsistent(parameterVector));

      // Known failing example: negative unit mass, unit center of mass offset, identity moment of inertia matrix
      mass = -1.0;
      spatialInertia.setMass(mass);
      parameterVector.setParameterVectorPiBasis(spatialInertia);
      assertFalse(RigidBodyInertialParametersTools.isPhysicallyConsistent(parameterVector));

      // Known failing example: unit mass, unit center of mass offset, zero inertia matrix
      spatialInertia.setMomentOfInertia(0.0, 0.0, 0.0);
      parameterVector.setParameterVectorPiBasis(spatialInertia);
      assertFalse(RigidBodyInertialParametersTools.isPhysicallyConsistent(parameterVector));

      // Known failing example: unit mass, unit center of mass offset, identity moment of inertia matrix with y-diagonal negated
      mass = 1.0;
      spatialInertia.setMass(mass);
      spatialInertia.setMomentOfInertia(1.0, -1.0, 1.0);
      parameterVector.setParameterVectorPiBasis(spatialInertia);
      assertFalse(RigidBodyInertialParametersTools.isPhysicallyConsistent(parameterVector));
   }

   @Test
   public void testIsFullyPhysicallyConsistent()
   {
      // Known passing example: unit mass, unit center of mass offset, identity moment of inertia matrix
      RigidBodyInertialParameters parameterVector = new RigidBodyInertialParameters(spatialInertia);
      assertTrue(RigidBodyInertialParametersTools.isFullyPhysicallyConsistent(parameterVector));

      // Known failing example: unit mass, unit center of mass offset, diagonal moment of inertia matrix with entries: 3, 1, 1 (violates triangle inequality)
      spatialInertia.setMomentOfInertia(3.0, 1.0, 1.0);
      parameterVector.setParameterVectorPiBasis(spatialInertia);
      assertFalse(RigidBodyInertialParametersTools.isFullyPhysicallyConsistent(parameterVector));
   }
}