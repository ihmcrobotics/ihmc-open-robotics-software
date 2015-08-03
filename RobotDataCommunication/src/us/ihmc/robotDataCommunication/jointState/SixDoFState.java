package us.ihmc.robotDataCommunication.jointState;

import java.nio.LongBuffer;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.robotics.screwTheory.Twist;

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

      twist.packAngularPart(tempVector);
      joint.setAngularVelocityInBody(tempVector);
      
      twist.packLinearPart(tempVector);
      joint.setVelocity(tempVector);
   }

   public void get(double[] array)
   {
      array[0] = rotation.w;
      array[1] = rotation.x;
      array[2] = rotation.y;
      array[3] = rotation.z;

      array[4] = translation.x;
      array[5] = translation.y;
      array[6] = translation.z;

      twist.packArray(array, 7);
   }

   public void update(LongBuffer buffer)
   {
      
      rotation.w    = Double.longBitsToDouble(buffer.get());
      rotation.x    = Double.longBitsToDouble(buffer.get());
      rotation.y    = Double.longBitsToDouble(buffer.get());
      rotation.z    = Double.longBitsToDouble(buffer.get());
      translation.x = Double.longBitsToDouble(buffer.get());
      translation.y = Double.longBitsToDouble(buffer.get());
      translation.z = Double.longBitsToDouble(buffer.get());
      
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
