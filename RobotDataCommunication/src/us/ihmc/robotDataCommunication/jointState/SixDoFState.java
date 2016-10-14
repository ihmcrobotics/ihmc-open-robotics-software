package us.ihmc.robotDataCommunication.jointState;

import java.nio.LongBuffer;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.simulationconstructionset.FloatingJoint;

public class SixDoFState extends JointState<FloatingJoint>
{
   public static final int numberOfStateVariables = 13;
   
   private final Quat4d rotation = new Quat4d();
   private final Matrix3d rotationMatrix = new Matrix3d();
   
   private final Vector3d translation = new Vector3d();
   private final Twist twist = new Twist();
   
   private final Vector3d tempVector = new Vector3d();

   public SixDoFState(String name)
   {
      super(name);
   }

   public void get(FloatingJoint joint)
   {
      rotationMatrix.set(rotation);
      joint.setRotation(rotationMatrix);
      joint.setPosition(translation);

      twist.getAngularPart(tempVector);
      joint.setAngularVelocityInBody(tempVector);
      
      twist.getLinearPart(tempVector);
      joint.setVelocity(tempVector);
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
}
