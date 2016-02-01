package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.MathTools;

public class WristSensorNoisePacket extends Packet<WristSensorNoisePacket>
{
  public float leftWristNoise;
  public float rightWristNoise;
  
  public WristSensorNoisePacket()
  {
     // Empty constructor for deserialization
  }
  
  public WristSensorNoisePacket(float _leftWristNoise, float _rightWristNoise)
  {
     this.leftWristNoise  = _leftWristNoise;
     this.rightWristNoise = _rightWristNoise;
  }
  
  public float getLeftWristNoise()
  {
     return leftWristNoise;
  }

  public float getRightWristNoise()
  {
     return rightWristNoise;
  }
  
  public boolean equals(Object obj)
  {
     return ((obj instanceof WristSensorNoisePacket) && this.epsilonEquals((WristSensorNoisePacket) obj, 0));
  }

  public String toString()
  {
     return "Left wrist noise: " + leftWristNoise + "  Right wrist noise: " + rightWristNoise;
  }

  @Override
  public boolean epsilonEquals(WristSensorNoisePacket other, double epsilon)
  {
     return
           MathTools.epsilonEquals(leftWristNoise,  other.leftWristNoise, epsilon ) &&
           MathTools.epsilonEquals(rightWristNoise, other.rightWristNoise, epsilon) ;
  }

  public WristSensorNoisePacket(Random random)
  {
     leftWristNoise  = random.nextFloat();
     rightWristNoise = random.nextFloat();
  }
  
  
}
