package us.ihmc.humanoidRobotics.communication.packets.sensing;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.random.RandomTools;

public class HeadPosePacket extends Packet<HeadPosePacket>
{
	public double yaw, pitch, roll ;
	public Vector3D measuredGravityInWorld;
	public enum MeasurementStatus {UNSTABLE_WAIT, STABLE};
	public MeasurementStatus status;
	
	
	public HeadPosePacket(Random random)
	{
	   yaw = random.nextDouble();
	   pitch = random.nextDouble();
	   roll = random.nextDouble();
	   measuredGravityInWorld = RandomTools.generateRandomVector(random);
	   
	   status = random.nextBoolean() ? MeasurementStatus.STABLE : MeasurementStatus.STABLE;
	}
	
	public HeadPosePacket()
	{
		measuredGravityInWorld = new Vector3D();
		reset();
	}
	
	public void setEulerAngles(double[] yawPitchRoll)
	{
        yaw = yawPitchRoll[0];
        pitch = yawPitchRoll[1];
        roll = yawPitchRoll[2];
	}
	public void reset()
	{
		measuredGravityInWorld.set(Double.NaN, Double.NaN, Double.NaN);
		yaw = Double.NaN;
		pitch = Double.NaN;
		roll = Double.NaN;
	}


   @Override
   public boolean epsilonEquals(HeadPosePacket other, double epsilon)
   {
      boolean ret = true;
      
      ret &= MathTools.epsilonEquals(yaw, other.yaw, epsilon);
      ret &= MathTools.epsilonEquals(pitch, other.pitch, epsilon);
      ret &= MathTools.epsilonEquals(roll, other.roll, epsilon);
      ret &= measuredGravityInWorld.epsilonEquals(other.measuredGravityInWorld, epsilon);
      ret &= status.equals(other.status);
      
      return ret;
   }
}

