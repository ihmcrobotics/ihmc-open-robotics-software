package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.random.RandomTools;

public class FootstepDataListCorruptor
{
   private final Random random = new Random(1778L);
   private final Vector3D minLocationCorruption = new Vector3D();
   private final Vector3D maxLocationCorruption = new Vector3D();
   private final double maxRotationCorruption;
   
   public FootstepDataListCorruptor(Vector3D minLocationCorruption, Vector3D maxLocationCorruption, double maxRotationCorruption)
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
      
      Point3D location = new Point3D();
      Quaternion orientation = new Quaternion();
      
      ret.getOrientation(orientation);
      corruptOrientation(orientation);
      ret.setOrientation(orientation);
      
      ret.getLocation(location);
      corruptLocationVector(location);
      ret.setLocation(location);
      return ret;
   }
   
   private void corruptOrientation(Quaternion orientation)
   {
      Vector3D axis = RandomTools.generateRandomVector(random);
      double angle = RandomNumbers.nextDouble(random, -maxRotationCorruption, maxRotationCorruption);
      
      AxisAngle axisAngle4d = new AxisAngle();
      axisAngle4d.set(axis, angle);
      
      Quaternion corruption = new Quaternion();
      corruption.set(axisAngle4d);
            
      orientation.multiply(corruption);
   }

   private void corruptLocationVector(Point3D location)
   {
      Vector3D randomVector = RandomTools.generateRandomVector(random, minLocationCorruption, maxLocationCorruption);
      location.add(randomVector);
   }
   
}
