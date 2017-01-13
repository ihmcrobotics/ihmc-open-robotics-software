package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.random.RandomTools;

public class FootstepDataListCorruptor
{
   private final Random random = new Random(1778L);
   private final Vector3d minLocationCorruption = new Vector3d();
   private final Vector3d maxLocationCorruption = new Vector3d();
   private final double maxRotationCorruption;
   
   public FootstepDataListCorruptor(Vector3d minLocationCorruption, Vector3d maxLocationCorruption, double maxRotationCorruption)
   {
      this.minLocationCorruption.set(minLocationCorruption);
      this.maxLocationCorruption.set(maxLocationCorruption);
      this.maxRotationCorruption = maxRotationCorruption;
   }
   
   public FootstepDataListMessage corruptDataList(FootstepDataListMessage footstepDataList)
   {
      FootstepDataListMessage ret = new FootstepDataListMessage(footstepDataList.defaultSwingTime, footstepDataList.defaultTransferTime);
      
      for (FootstepDataMessage footstepData : footstepDataList)
      {
         ret.add(corruptFootstepData(footstepData));
      }
      
      return ret;
   }
   
   public FootstepDataMessage corruptFootstepData(FootstepDataMessage footstepData)
   {
      FootstepDataMessage ret = footstepData.clone();
      
      Point3d location = new Point3d();
      Quat4d orientation = new Quat4d();
      
      ret.getOrientation(orientation);
      corruptOrientation(orientation);
      ret.setOrientation(orientation);
      
      ret.getLocation(location);
      corruptLocationVector(location);
      ret.setLocation(location);
      return ret;
   }
   
   private void corruptOrientation(Quat4d orientation)
   {
      Vector3d axis = RandomTools.generateRandomVector(random);
      double angle = RandomTools.generateRandomDouble(random, -maxRotationCorruption, maxRotationCorruption);
      
      AxisAngle4d axisAngle4d = new AxisAngle4d();
      axisAngle4d.set(axis, angle);
      
      Quat4d corruption = new Quat4d();
      corruption.set(axisAngle4d);
            
      orientation.mul(corruption);
   }

   private void corruptLocationVector(Point3d location)
   {
      Vector3d randomVector = RandomTools.generateRandomVector(random, minLocationCorruption, maxLocationCorruption);
      location.add(randomVector);
   }
   
}
