package us.ihmc.robotDataLogger.jointState;

import java.nio.LongBuffer;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotDataLogger.generated.YoProtoHandshakeProto.YoProtoHandshake.JointDefinition.JointType;
import us.ihmc.robotics.screwTheory.Twist;

public class SixDoFState extends JointState
{
   public static final int numberOfStateVariables = 13;
   
   private final Quat4d rotation = new Quat4d();
   
   private final Vector3d translation = new Vector3d();
   private final Twist twist = new Twist();
   

   public SixDoFState(String name)
   {
      super(name, JointType.SiXDoFJoint);
   }


   public void get(double[] array)
   {
      array[0] = rotation.getW();
      array[1] = rotation.getX();
      array[2] = rotation.getY();
      array[3] = rotation.getZ();

      array[4] = translation.getX();
      array[5] = translation.getY();
      array[6] = translation.getZ();

      twist.getArray(array, 7);
   }

   public void update(LongBuffer buffer)
   {
      
      rotation.setW(Double.longBitsToDouble(buffer.get()));
      rotation.setX(Double.longBitsToDouble(buffer.get()));
      rotation.setY(Double.longBitsToDouble(buffer.get()));
      rotation.setZ(Double.longBitsToDouble(buffer.get()));
      translation.setX(Double.longBitsToDouble(buffer.get()));
      translation.setY(Double.longBitsToDouble(buffer.get()));
      translation.setZ(Double.longBitsToDouble(buffer.get()));
      
      twist.setAngularPartX(Double.longBitsToDouble(buffer.get()));
      twist.setAngularPartY(Double.longBitsToDouble(buffer.get()));
      twist.setAngularPartZ(Double.longBitsToDouble(buffer.get()));
      
      twist.setLinearPartX(Double.longBitsToDouble(buffer.get()));
      twist.setLinearPartY(Double.longBitsToDouble(buffer.get()));
      twist.setLinearPartZ(Double.longBitsToDouble(buffer.get()));
   }

   @Override
   public int getNumberOfStateVariables()
   {
      return numberOfStateVariables;
   }


   public void getRotation(Matrix3d rotationMatrix)
   {
      rotationMatrix.set(rotation);
   }


   public void getTranslation(Vector3d tempVector)
   {
      tempVector.set(translation);
   }


   public void getTwistAngularPart(Vector3d tempVector)
   {
      twist.getAngularPart(tempVector);
   }
   
   public void getTwistLinearPart(Vector3d tempVector)
   {
      twist.getLinearPart(tempVector);
   }
}
