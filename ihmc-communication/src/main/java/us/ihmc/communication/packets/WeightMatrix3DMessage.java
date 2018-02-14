package us.ihmc.communication.packets;

import java.util.Random;

import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.utils.NameBasedHashCodeTools;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;

@RosMessagePacket(documentation = "", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE, topic = "/control/weight_matrix")
public class WeightMatrix3DMessage extends Packet<WeightMatrix3DMessage>
{
   @RosExportedField(documentation = "The Id of the reference frame defining the weight frame."
         + " This reference frame defines the x axis,y axis, z axis for the weights."
         + " This frame is optional. It is preferable to provide it when possible, but when it is absent, i.e. equal to {@code 0L},"
         + " the weight matrix will then be generated regardless to what frame is it used in.")
   public long weightFrameId = NameBasedHashCodeTools.NULL_HASHCODE;
   @RosExportedField(documentation = "Specifies the qp weight for the x-axis, If set to NaN the controller will use the default weight for this axis. The weight is NaN by default.")
   public double xWeight = Double.NaN;
   @RosExportedField(documentation = "Specifies the qp weight for the y-axis, If set to NaN the controller will use the default weight for this axis. The weight is NaN by default.")
   public double yWeight = Double.NaN;
   @RosExportedField(documentation = "Specifies the qp weight for the z-axis, If set to NaN the controller will use the default weight for this axis. The weight is NaN by default.")
   public double zWeight = Double.NaN;

   /**
    * Creates a new weight matrix message. It is initialized with all the weights set to NaN. Until
    * the weights are changed, this weight matrix is independent from its frame.
    */
   public WeightMatrix3DMessage()
   {
   }

   public WeightMatrix3DMessage(Random random)
   {
      weightFrameId = random.nextLong();
      xWeight = random.nextDouble();
      yWeight = random.nextDouble();
      zWeight = random.nextDouble();
   }

   /**
    * Copy constructor.
    */
   public WeightMatrix3DMessage(WeightMatrix3DMessage weightMatrix)
   {
      set(weightMatrix);
   }
   
   public WeightMatrix3DMessage(WeightMatrix3D weightMatrix)
   {
      set(weightMatrix);
   }

   /**
    * Sets this weight matrix message to {@code qpWeightMatrix}.
    * 
    * @param weightMatrix3D the weight matrix to copy the data of. parameter will not be modified.
    */
   public void set(WeightMatrix3DMessage weightMatrix)
   {
      weightFrameId = weightMatrix.getWeightFrameId();
      this.xWeight = weightMatrix.getXWeight();
      this.yWeight = weightMatrix.getYWeight();
      this.zWeight = weightMatrix.getZWeight();
   }
   
   /**
    * Sets this weight matrix message to {@code weightMatrix}.
    * 
    * @param weightMatrix the weight matrix to copy the data of. parameter will not be modified.
    */
   public void set(WeightMatrix3D weightMatrix)
   {
      setWeightFrame(weightMatrix.getWeightFrame());
      this.xWeight = weightMatrix.getXAxisWeight();
      this.yWeight = weightMatrix.getYAxisWeight();
      this.zWeight = weightMatrix.getZAxisWeight();
   }

   /**
    * Sets the weights
    * <p>
    * Note that it is preferable to also set weight frame to which these weights are expressed in
    * to.
    * </p>
    * 
    * @param xWeight sets the weight of the x-axis motion
    * @param yWeight sets the weight of the y-axis motion
    * @param zWeight sets the weight of the z-axis motion
    */
   public void setWeights(double xWeight, double yWeight, double zWeight)
   {
      this.xWeight = xWeight;
      this.yWeight = yWeight;
      this.zWeight = zWeight;
   }

   /**
    * sets the frame the weights are expressed in
    * 
    * @param weightFrame the new frame to which the weights are expressed in 
    */
   public void setWeightFrame(ReferenceFrame weightFrame)
   {
      if (weightFrame == null)
         weightFrameId = NameBasedHashCodeTools.NULL_HASHCODE;
      else
         weightFrameId = weightFrame.getNameBasedHashCode();
   }

   /**
    * Sets the ID of the weight frame to use. The weights refer to the
    * axes of the weight frame.
    * 
    * @param weightFrameID the ID of the new frame to which the weights are referring to.
    */
   public void setWeightFrameId(long weightFrameId)
   {
      this.weightFrameId = weightFrameId;
   }

   /**
    * Unpacks this message into the given {@code weightMatrix3D}.
    * <p>
    * Note that the weight frame can not be retrieved here, it has to be set afterwards.
    * </p>
    * 
    * @param weightMatrix3D the weight matrix into which this message is being unpacked.
    *           Modified.
    */
   public void getWeightMatrix(WeightMatrix3D weightMatrix3D)
   {
      weightMatrix3D.clearWeightFrame();
      weightMatrix3D.setWeights(xWeight, yWeight, zWeight);
   }

   /**
    * Returns the unique ID referring to the weight frame to use with this weight matrix.
    * 
    * @return the weight frame ID.
    */
   public long getWeightFrameId()
   {
      return weightFrameId;
   }

   /**
    * gets the x weight
    * @return the x weight
    */
   public double getXWeight()
   {
      return xWeight;
   }

   /**
    * sets the x weight. If set to Double.NaN the controller will use the default weight for that axis
    * @return the new x weight
    */
   public void setXWeight(double xWeight)
   {
      this.xWeight = xWeight;
   }

   /**
    * gets the y weight
    * @return the y weight
    */
   public double getYWeight()
   {
      return yWeight;
   }

   /**
    * sets the y weight. If set to Double.NaN the controller will use the default weight for that axis
    * @return the new y weight
    */
   public void setYWeight(double yWeight)
   {
      this.yWeight = yWeight;
   }

   /**
    * gets the z weight
    * @return the z weight
    */
   public double getZWeight()
   {
      return zWeight;
   }

   /**
    * sets the z weight. If set to Double.NaN the controller will use the default weight for that axis
    * @return the new z weight
    */
   public void setZWeight(double zWeight)
   {
      this.zWeight = zWeight;
   }
   
   
   @Override
   public int hashCode()
   {
      final int prime = 31;
      int result = 1;
      result = prime * result + (int) (weightFrameId ^ (weightFrameId >>> 32));
      long temp;
      temp = Double.doubleToLongBits(xWeight);
      result = prime * result + (int) (temp ^ (temp >>> 32));
      temp = Double.doubleToLongBits(yWeight);
      result = prime * result + (int) (temp ^ (temp >>> 32));
      temp = Double.doubleToLongBits(zWeight);
      result = prime * result + (int) (temp ^ (temp >>> 32));
      return result;
   }

   @Override
   public boolean equals(Object obj)
   {
      if (this == obj)
         return true;
      if (obj == null)
         return false;
      if (getClass() != obj.getClass())
         return false;
      WeightMatrix3DMessage other = (WeightMatrix3DMessage) obj;
      return epsilonEquals(other, 1e-4);
   }

   @Override
   public boolean epsilonEquals(WeightMatrix3DMessage other, double epsilon)
   {
      if (weightFrameId != other.weightFrameId)
         return false;
      
      if(Double.isNaN(xWeight) ^ Double.isNaN(other.xWeight)) // xor is correct
      {
         return false;
      }

      if(Double.isNaN(yWeight) ^ Double.isNaN(other.yWeight)) // xor is correct
      {
         return false;
      }
      
      if(Double.isNaN(zWeight) ^ Double.isNaN(other.zWeight)) // xor is correct
      {
         return false;
      }
      
      if (!Double.isNaN(xWeight) && !Double.isNaN(other.xWeight) && xWeight != other.xWeight)
      {
         return false;
      }
      if (!Double.isNaN(yWeight) && !Double.isNaN(other.yWeight) && yWeight != other.yWeight)
      {
         return false;
      }
      if (!Double.isNaN(zWeight) && !Double.isNaN(other.zWeight) && zWeight != other.zWeight)
      {
         return false;
      }

      return true;
   }
}
