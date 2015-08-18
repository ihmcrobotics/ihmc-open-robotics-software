package us.ihmc.communication.packets.behaviors;

import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.Packet;
import us.ihmc.tools.random.RandomTools;

public class ButtonData extends Packet<ButtonData>
{
   public Vector3d pushDirection;
   public Point3d pushPoint;
   
   public ButtonData(Random random)
   {
      double max = Double.MAX_VALUE / 2;
      pushPoint = RandomTools.generateRandomPoint(random, max, max, max);
      pushDirection = RandomTools.generateRandomVector(random, max, max, max, max, max, max);
   }
   
   public ButtonData()
   {
      this.pushDirection = new Vector3d();
      this.pushPoint = new Point3d();
   }

   
   public ButtonData(Vector3d pushVector, Point3d attackPoint)
   {
    
      this.pushDirection = pushVector;
      this.pushPoint = attackPoint;
   }
   
   public ButtonData(ButtonData buttonData)
   {
      this.pushDirection= buttonData.getPushDirection();
      this.pushPoint = buttonData.getPushPosition();
   }
      
   public Vector3d getPushDirection()
   {
      return pushDirection;
   }
   
   public Point3d getPushPosition()
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
