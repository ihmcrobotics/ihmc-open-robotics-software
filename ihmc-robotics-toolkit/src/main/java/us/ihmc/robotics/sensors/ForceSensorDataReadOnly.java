package us.ihmc.robotics.sensors;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;

public interface ForceSensorDataReadOnly
{
   public String getSensorName();

   public ReferenceFrame getMeasurementFrame();

   public RigidBodyBasics getMeasurementLink();

   /*
    * Get force sensor wrench data and write to Matrix argument.
    */
   public void getWrench(DMatrixRMaj wrenchToPack);

   /*
    * Get force sensor wrench data and write to Wrench argument.
    */
   public void getWrench(Wrench wrenchToPack);

   /*
    * Get force sensor wrench data and write to float array argument.
    */
   public void getWrench(float[] wrenchToPack);
   
   /*
    * Get force sensor wrench data and write to vector arguments for moment and force.
    */
   public void getWrench(Vector3DBasics momentToPack, Vector3DBasics forceToPack);
}