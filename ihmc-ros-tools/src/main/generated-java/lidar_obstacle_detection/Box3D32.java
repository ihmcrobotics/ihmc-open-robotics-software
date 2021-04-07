package lidar_obstacle_detection;

import org.ros.internal.message.RawMessage;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class Box3D32 implements GDXBoxMessage
{
   /**
    * The x-coordinate.
    */
   private double xMin;

   private double xMax;

   /**
    * The y-coordinate.
    */
   private double yMin;

   private double yMax;

   /**
    * The z-coordinate.
    */
   private double zMin;

   private double zMax;

   RigidBodyTransform Transform;



   /**
    * Creates a new point and initializes it coordinates to zero.
    */
   public Box3D32()
   {
      this.xMin = 0;
      this.xMax = 0;
      this.yMin = 0;
      this.yMax = 0;
      this.zMin = 0;
      this.zMax = 0;
      this.Transform.set(1,0,0,0,0,1,0,0,0,0,1,0);
   }

   /**
    * Creates a new point and initializes it with the given coordinates.
    *
    * @param x the x-coordinate.
    * @param y the y-coordinate.
    * @param z the z-coordinate.
    */
   public Box3D32(double xMin, double xMax, double yMin, double yMax, double zMin, double zMax)
   {
      this.xMin = xMin;
      this.xMax = xMax;
      this.yMin = yMin;
      this.yMax = yMax;
      this.zMin = zMin;
      this.zMax = zMax;
   }

   /**
    * Creates a new point and initializes its component {@code x}, {@code y}, {@code z} in order from
    * the given array.
    *
    * @param pointArray the array containing this point's coordinates. Not modified.
    */
   public Box3D32(double[] pointArray)
   {
      this.xMin = pointArray[0];
      this.xMax = pointArray[1];
      this.yMin = pointArray[2];
      this.yMax = pointArray[3];
      this.zMin = pointArray[4];
      this.zMax = pointArray[5];
      this.Transform.set(pointArray[6],pointArray[7],pointArray[8],pointArray[9],pointArray[10]
                        ,pointArray[11],pointArray[12],pointArray[13],pointArray[14],pointArray[15]
                        ,pointArray[16],pointArray[17]);
   }

   @Override
   public double getXMin()
   {
      return this.xMin;
   }

   @Override
   public void setXMin(double xMin)
   {
      this.xMin = xMin;
   }

   @Override
   public double getYMin()
   {
      return this.yMin;
   }

   @Override
   public void setYMin(double yMin)
   {
      this.yMin = yMin;
   }

   @Override
   public double getZMin()
   {
      return this.zMin;
   }

   @Override
   public void setZMin(double zMin)
   {
      this.zMin = zMin;
   }

   @Override
   public double getXMax()
   {
      return xMax;
   }

   @Override
   public void setXMax(double xMax)
   {
      this.xMax = xMax;
   }

   @Override
   public double getYMax()
   {
      return yMax;
   }

   @Override
   public void setYMax(double yMax)
   {
      this.yMax = yMax;
   }

   @Override
   public double getZMax()
   {
      return zMax;
   }

   @Override
   public void setZMax(double zMax)
   {
      this.zMax = zMax;
   }

   public RigidBodyTransform getTransform()
   {
      return this.Transform;
   }

   public void setTrasform(RigidBodyTransform transform)
   {
      this.Transform = transform;
   }

   @Override
   public RawMessage toRawMessage()
   {
      return this.toRawMessage();
   }

   public void set(GDXBoxMessage gdxBoxMessage, RigidBodyTransform Transform)
   {
      this.xMin = gdxBoxMessage.getXMin();
      this.xMax = gdxBoxMessage.getXMax();
      this.yMin = gdxBoxMessage.getYMin();
      this.yMax = gdxBoxMessage.getYMax();
      this.zMin = gdxBoxMessage.getZMin();
      this.zMax = gdxBoxMessage.getZMax();
      this.Transform = Transform;
   }
   public void set(Box3D32 box){
      this.xMin = box.xMin;
      this.xMax = box.xMax;
      this.yMin = box.yMin;
      this.yMax = box.yMax;
      this.zMin = box.zMin;
      this.zMax = box.zMax;
      this.Transform = box.Transform;
   }
}
