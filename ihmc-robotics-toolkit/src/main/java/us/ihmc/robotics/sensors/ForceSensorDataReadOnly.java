package us.ihmc.robotics.sensors;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Wrench;

public interface ForceSensorDataReadOnly
{
   public ReferenceFrame getMeasurementFrame();

   public RigidBody getMeasurementLink();

   public void getWrench(DenseMatrix64F wrenchToPack);

   public void getWrench(Wrench wrenchToPack);

   public void getWrench(float[] wrenchToPack);
   
   public void getWrench(Vector3DBasics momentToPack, Vector3DBasics forceToPack);
}