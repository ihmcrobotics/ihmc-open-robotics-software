package us.ihmc.robotics.quadTree;

public class Box
{
   public final double minX;
   public final double minY;
   public final double maxX;
   public final double maxY;
   public final double centreX;
   public final double centreY;

   public Box(double minX, double minY, double maxX, double maxY)
   {
      this.minX = Math.min(minX, maxX);
      this.minY = Math.min(minY, maxY);
      this.maxX = Math.max(minX, maxX);
      this.maxY = Math.max(minY, maxY);
      this.centreX = (minX + maxX) / 2;
      this.centreY = (minY + maxY) / 2;
   }
   
   

   public boolean containsOrEquals(double x, double y)
   {
      return (x >= this.minX &&
              y >= this.minY &&
              x <= this.maxX &&
              y <= this.maxY);
   }

   public boolean containsOrEquals(Box box)
   {
      return (box.minX >= this.minX &&
              box.minY >= this.minY &&
              box.maxX <= this.maxX &&
              box.maxY <= this.maxY);
   }

   public Box intersection(Box r)
   {
      double tempX1 = this.minX;
      double tempY1 = this.minY;
      double tempX2 = this.maxX;
      double tempY2 = this.maxY;
      if (this.minX < r.minX)
      {
         tempX1 = r.minX;
      }
      if (this.minY < r.minY)
      {
         tempY1 = r.minY;
      }
      if (tempX2 > r.maxX)
      {
         tempX2 = r.maxX;
      }
      if (tempY2 > r.maxY)
      {
         tempY2 = r.maxY;
      }
      if (tempX2 - tempX1 <= 0.f || tempY2 - tempY1 <= 0.f)
      {
         return null;
      }

      return new Box(tempX1, tempY1, tempX2, tempY2);
   }

   public boolean intersects(Box other)
   {
      if ((this.maxX - this.minX) <= 0 || (this.maxY - this.minY) <= 0)
      {
         return false;
      }
      return (other.maxX > this.minX &&
              other.maxY > this.minY &&
              other.minX < this.maxX &&
              other.minY < this.maxY);
   }

   public Box union(Box b)
   {
      return new Box(Math.min(this.minX, b.minX), Math.min(this.minY, b.minY), Math.max(this.maxX, b.maxX), Math.max(this.maxY, b.maxY));
   }

   public double calcDist(double x, double y)
   {
      double distanceX;
      double distanceY;

      if (this.minX <= x && x <= this.maxX)
      {
         distanceX = 0;
      }
      else
      {
         distanceX = Math.min(Math.abs(this.minX - x), Math.abs(this.maxX - x));
      }
      if (this.minY <= y && y <= this.maxY)
      {
         distanceY = 0;
      }
      else
      {
         distanceY = Math.min(Math.abs(this.minY - y), Math.abs(this.maxY - y));
      }

      return Math.sqrt(distanceX * distanceX + distanceY * distanceY);
   }

   public Box scale(double scaleX, double scaleY)
   {
      scaleY *= this.centreY - this.minY;
      scaleX *= this.centreX - this.minX;
      return new Box(this.minX - scaleX, this.minY - scaleY, this.maxX + scaleX, this.maxY + scaleY);
   }

   @Override
   public String toString()
   {
      return "lowerLeft: (" + minX + ", " + minY + ") upperRight: (" + maxX + ", " + maxY + ")";
   }

}