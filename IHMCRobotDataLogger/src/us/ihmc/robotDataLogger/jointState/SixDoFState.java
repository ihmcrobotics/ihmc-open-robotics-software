package us.ihmc.robotDataLogger.jointState;

import java.nio.LongBuffer;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotDataLogger.JointType;
import us.ihmc.robotics.screwTheory.Twist;

public class SixDoFState extends JointState
{
   public static final int numberOfStateVariables = 13;
   
   private final Quaternion rotation = new Quaternion();
   
   private final Vector3D translation = new Vector3D();
   private final Twist twist = new Twist();
   

   public SixDoFState(String name)
   {
      super(name, JointType.SiXDoFJoint);
   }


   public void get(double[] array)
   {
      array[0] = rotation.getS();
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
      
      double qs = Double.longBitsToDouble(buffer.get());
      double qx = Double.longBitsToDouble(buffer.get());
      double qy = Double.longBitsToDouble(buffer.get());
      double qz = Double.longBitsToDouble(buffer.get());
      rotation.set(qx, qy, qz, qs);
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


   public void getRotation(RotationMatrix rotationMatrix)
   {
      rotationMatrix.set(rotation);
   }


   public void getTranslation(Vector3D tempVector)
   {
      tempVector.set(translation);
   }


   public void getTwistAngularPart(Vector3D tempVector)
   {
      twist.getAngularPart(tempVector);
   }
   
   public void getTwistLinearPart(Vector3D tempVector)
   {
      twist.getLinearPart(tempVector);
   }
}
