package us.ihmc.robotics.sensors;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Wrench;

public interface ForceSensorDataReadOnly
{
   public ReferenceFrame getMeasurementFrame();

   public RigidBody getMeasurementLink();

   public void packWrench(DenseMatrix64F wrenchToPack);

   public void packWrench(Wrench wrenchToPack);

   public void packWrench(float[] wrenchToPack);
}