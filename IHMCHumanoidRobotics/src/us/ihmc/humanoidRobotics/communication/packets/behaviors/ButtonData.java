package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.random.RandomGeometry;

public class ButtonData extends Packet<ButtonData>
{
   public Vector3D pushDirection;
   public Point3D pushPoint;
   
   public ButtonData(Random random)
   {
      double max = Double.MAX_VALUE / 2;
      pushPoint = RandomGeometry.nextPoint3D(random, max, max, max);
      pushDirection = RandomGeometry.nextVector3D(random, max, max, max, max, max, max);
   }
   
   public ButtonData()
   {
      this.pushDirection = new Vector3D();
      this.pushPoint = new Point3D();
   }

   
   public ButtonData(Vector3D pushVector, Point3D attackPoint)
   {
    
      this.pushDirection = pushVector;
      this.pushPoint = attackPoint;
   }
   
   public ButtonData(ButtonData buttonData)
   {
      this.pushDirection= buttonData.getPushDirection();
      this.pushPoint = buttonData.getPushPosition();
   }
      
   public Vector3D getPushDirection()
   {
      return pushDirection;
   }
   
   public Point3D getPushPosition()
   {
      return pushPoint;
   }
   
   public boolean epsilonEquals(ButtonData buttonData, double epsilon)
   {
      boolean vectorEquals = pushDirection.epsilonEquals(buttonData.getPushDirection(), epsilon);
      boolean positionEquals = pushPoint.epsilonEquals(buttonData.getPushPosition(), epsilon);
      
      return vectorEquals && positionEquals;
   }

}
