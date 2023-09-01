package us.ihmc.robotics.sensors;

import java.util.Arrays;
import java.util.Objects;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.robotics.MecanoMissingTools;
import us.ihmc.robotics.screwTheory.GenericCRC32;

public class ForceSensorData implements ForceSensorDataReadOnly, Settable<ForceSensorData>
{
   private final DMatrixRMaj wrenchMatrix = new DMatrixRMaj(Wrench.SIZE, 1);

   private String sensorName;
   private ReferenceFrame measurementFrame;
   private RigidBodyBasics measurementLink;

   private transient final WrenchReadOnly wrench = MecanoMissingTools.newLinkedWrenchReadOnly(this::getMeasurementFrame, this::getMeasurementFrame, wrenchMatrix);

   public ForceSensorData()
   {
   }

   public ForceSensorData(ForceSensorDefinition forceSensorDefinition)
   {
      setDefinition(forceSensorDefinition);
   }

   public void setDefinition(ForceSensorDefinition forceSensorDefinition)
   {
      setDefinition(forceSensorDefinition.getSensorName(), forceSensorDefinition.getSensorFrame(), forceSensorDefinition.getRigidBody());
   }

   public void setDefinition(String sensorName, ReferenceFrame measurementFrame, RigidBodyBasics measurementLink)
   {
      this.sensorName = sensorName;
      this.measurementFrame = measurementFrame;
      this.measurementLink = measurementLink;
   }

   public void setWrench(Vector3DReadOnly moment, Vector3DReadOnly force)
   {
      moment.get(0, wrenchMatrix);
      force.get(3, wrenchMatrix);
   }

   public void setWrench(DMatrixRMaj newWrench)
   {
      wrenchMatrix.set(newWrench);
   }

   public void setWrench(float[] newWrench)
   {
      for (int i = 0; i < SpatialVectorReadOnly.SIZE; i++)
      {
         wrenchMatrix.set(i, 0, newWrench[i]);
      }
   }

   public void setWrench(WrenchReadOnly newWrench)
   {
      measurementFrame.checkReferenceFrameMatch(newWrench.getReferenceFrame());
      measurementFrame.checkReferenceFrameMatch(newWrench.getBodyFrame());
      newWrench.get(wrenchMatrix);
   }

   @Override
   public String getSensorName()
   {
      return sensorName;
   }

   @Override
   public ReferenceFrame getMeasurementFrame()
   {
      return measurementFrame;
   }

   @Override
   public RigidBodyBasics getMeasurementLink()
   {
      return measurementLink;
   }

   @Override
   public void getWrenchMatrix(DMatrixRMaj wrenchMatrixToPack)
   {
      wrenchMatrixToPack.set(wrenchMatrix);
   }

   public DMatrixRMaj getWrenchMatrix()
   {
      return wrenchMatrix;
   }

   @Override
   public WrenchReadOnly getWrench()
   {
      return wrench;
   }

   @Override
   public void set(ForceSensorData other)
   {
      set((ForceSensorDataReadOnly) other);
   }

   public void set(ForceSensorDataReadOnly other)
   {
      sensorName = other.getSensorName();
      measurementFrame = other.getMeasurementFrame();
      measurementLink = other.getMeasurementLink();
      other.getWrenchMatrix(wrenchMatrix);
   }

   @Override
   public boolean equals(Object obj)
   {
      if (obj == this)
      {
         return true;
      }
      else if (obj instanceof ForceSensorData)
      {
         ForceSensorData other = (ForceSensorData) obj;
         if (!Objects.equals(sensorName, other.sensorName))
            return false;
         if (measurementFrame != other.measurementFrame)
            return false;
         if (measurementLink != other.measurementLink)
            return false;
         if (!MatrixTools.equals(wrenchMatrix, other.wrenchMatrix))
            return false;
         return true;
      }
      else
      {
         return false;
      }
   }

   public void calculateChecksum(GenericCRC32 checksum)
   {
      checksum.update(wrenchMatrix);
   }

   @Override
   public String toString()
   {
      return "[sensorName=" + sensorName + ", wrenchMatrix=" + Arrays.toString(wrenchMatrix.data) + ", measurementFrame=" + measurementFrame
             + ", measurementLink=" + measurementLink.getName() + "]";
   }
}
