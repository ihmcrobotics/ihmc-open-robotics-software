package us.ihmc.robotics.sensors;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.screwTheory.GenericCRC32;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Wrench;

public class ForceSensorData implements ForceSensorDataReadOnly
{
   private final DenseMatrix64F wrench = new DenseMatrix64F(Wrench.SIZE, 1);

   private final ReferenceFrame measurementFrame;
   private final RigidBody measurementLink;

   public ForceSensorData(ForceSensorDefinition forceSensorDefinition)
   {
      measurementFrame = forceSensorDefinition.getSensorFrame();
      measurementLink = forceSensorDefinition.getRigidBody();
   }

   public void setWrench(Vector3DReadOnly moment, Vector3DReadOnly force)
   {
      moment.get(0, wrench);
      force.get(3, wrench);
   }

   public void setWrench(DenseMatrix64F newWrench)
   {
      wrench.set(newWrench);
   }

   public void setWrench(float[] newWrench)
   {
      for (int i = 0; i < Wrench.SIZE; i++)
      {
         wrench.set(i, 0, newWrench[i]);
      }
   }

   public void setWrench(Wrench newWrench)
   {
      measurementFrame.checkReferenceFrameMatch(newWrench.getExpressedInFrame());
      measurementFrame.checkReferenceFrameMatch(newWrench.getBodyFrame());
      newWrench.getMatrix(wrench);
   }

   @Override
   public ReferenceFrame getMeasurementFrame()
   {
      return measurementFrame;
   }

   @Override
   public RigidBody getMeasurementLink()
   {
      return measurementLink;
   }

   @Override
   public void getWrench(DenseMatrix64F wrenchToPack)
   {
      wrenchToPack.set(wrench);
   }

   @Override
   public void getWrench(Wrench wrenchToPack)
   {
      wrenchToPack.changeBodyFrameAttachedToSameBody(measurementFrame);
      wrenchToPack.set(measurementFrame, wrench);
   }

   @Override
   public void getWrench(Vector3DBasics momentToPack, Vector3DBasics forceToPack)
   {
      momentToPack.set(0, wrench);
      forceToPack.set(3, wrench);
   }

   @Override
   public void getWrench(float[] wrenchToPack)
   {
      for (int i = 0; i < Wrench.SIZE; i++)
      {
         wrenchToPack[i] = (float) wrench.get(i, 0);
      }
   }

   public void set(ForceSensorData forceSensorData)
   {
      this.wrench.set(forceSensorData.wrench);
   }

   public void calculateChecksum(GenericCRC32 checksum)
   {
      checksum.update(wrench);
   }
}
