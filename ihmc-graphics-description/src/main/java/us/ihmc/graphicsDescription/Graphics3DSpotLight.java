package us.ihmc.graphicsDescription;

import java.awt.Color;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class Graphics3DSpotLight
{
   private final Point3D position = new Point3D();
   private final Vector3D direction = new Vector3D(0, -1, 0);
   private Color color = new Color(0, 0, 0, 0);
   private double spotInnerAngle = Math.PI / (4.0 * 8.0);
   private double spotOuterAngle = Math.PI / (4.0 * 6.0);
   private double spotRange = 100;

   public Point3D getPosition()
   {
      return position;
   }

   public void setPosition(Point3D position)
   {
      this.position.set(position);
   }

   public Vector3D getDirection()
   {
      return direction;
   }

   public void setDirection(Vector3D direction)
   {
      this.direction.set(direction);
   }

   public Color getColor()
   {
      return color;
   }

   public void setColor(Color color)
   {
      this.color = color;
   }

   public double getSpotInnerAngle()
   {
      return spotInnerAngle;
   }

   public void setSpotInnerAngle(double spotInnerAngle)
   {
      this.spotInnerAngle = spotInnerAngle;
   }

   public double getSpotOuterAngle()
   {
      return spotOuterAngle;
   }

   public void setSpotOuterAngle(double spotOuterAngle)
   {
      this.spotOuterAngle = spotOuterAngle;
   }

   public double getSpotRange()
   {
      return spotRange;
   }

   public void setSpotRange(double spotRange)
   {
      this.spotRange = spotRange;
   }

}
