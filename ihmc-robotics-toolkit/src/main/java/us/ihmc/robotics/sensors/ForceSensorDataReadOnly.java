package us.ihmc.robotics.sensors;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;

public interface ForceSensorDataReadOnly
{
   public String getSensorName();

   public ReferenceFrame getMeasurementFrame();

   public RigidBodyBasics getMeasurementLink();

   /**
    * Get force sensor wrench data. The returned type is expected to be read-only.
    */
   void getWrenchMatrix(DMatrixRMaj wrenchMatrixToPack);

   /**
    * Get force sensor wrench data.
    */
   WrenchReadOnly getWrench();
}