package us.ihmc.humanoidRobotics.communication.packets.sensing;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;

public class MultisenseMocapExperimentPacket extends Packet<MultisenseMocapExperimentPacket>
{
   public float[] flatPoints;
   public MultisenseTest testInfo;
   
   public MultisenseMocapExperimentPacket(Random random)
   {
      
      int size = Math.abs(random.nextInt(100000));
      flatPoints = new float[size];
      for (int i = 0; i < flatPoints.length; i++)
      {
         flatPoints[i] = random.nextFloat();
      }
      MultisenseTest[] enumValues = MultisenseTest.values();
      testInfo = enumValues[random.nextInt(enumValues.length)];
   }
   
   public MultisenseMocapExperimentPacket()
   {
      setDestination(PacketDestination.BROADCAST);
   }
   
   public void setPointCloud(Point3D[] pointCloud, MultisenseTest testInfo)
   {
      this.testInfo = testInfo;
      flatPoints = new float[pointCloud.length*3];
      for (int i = 0; i < pointCloud.length; i++)
      {
         Point3D point = pointCloud[i];
         flatPoints[3 * i] = (float) point.getX();
         flatPoints[3 * i + 1] = (float) point.getY();
         flatPoints[3 * i + 2] = (float) point.getZ();
      }
   }

   public Point3D32[] getPointCloud()
   {
      
      int numberOfPoints = flatPoints.length/3;
      
      Point3D32[] points = new Point3D32[numberOfPoints];
      for(int i = 0; i < numberOfPoints; i++)
      {
         Point3D32 point = new Point3D32();
         point.setX(flatPoints[3 * i]);
         point.setY(flatPoints[3 * i + 1]);
         point.setZ(flatPoints[3 * i + 2]);
         points[i] = point;
      }
      
      return points;
   }
   
   @Override
   public boolean epsilonEquals(MultisenseMocapExperimentPacket other, double epsilon)
   {
      boolean ret = flatPoints.length == other.flatPoints.length;
      for (int i = 0; i < flatPoints.length; i++)
      {
         ret &= flatPoints[i] == other.flatPoints[i];
      }
      ret = testInfo.equals(other.testInfo);
      
      return ret;
   }
}
