package lidar_obstacle_detection;

import org.ros.internal.message.RawMessage;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;

public class Box3DO32 implements GDXBoxMessage
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

   private us.ihmc.euclid.tuple3D.Point3D point1;
   private us.ihmc.euclid.tuple3D.Point3D point2;
   private us.ihmc.euclid.tuple3D.Point3D point3;
   private us.ihmc.euclid.tuple3D.Point3D point4;
   private us.ihmc.euclid.tuple3D.Point3D point5;
   private us.ihmc.euclid.tuple3D.Point3D point6;
   private us.ihmc.euclid.tuple3D.Point3D point7;
   private us.ihmc.euclid.tuple3D.Point3D point8;


   RigidBodyTransform Transform;



   /**
    * Creates a new point and initializes it coordinates to zero.
    */
   public Box3DO32()
   {
      this.xMin = 0;
      this.xMax = 0;
      this.yMin = 0;
      this.yMax = 0;
      this.zMin = 0;
      this.zMax = 0;
      this.Transform = new RigidBodyTransform(1,0,0,0,0,1,0,0,0,0,1,0);
      this.point1 = new Point3D(0,0,0);
      this.point2 = new Point3D(0,0,0);
      this.point3 = new Point3D(0,0,0);
      this.point4 = new Point3D(0,0,0);
      this.point5 = new Point3D(0,0,0);
      this.point6 = new Point3D(0,0,0);
      this.point7 = new Point3D(0,0,0);
      this.point8 = new Point3D(0,0,0);

   }

   /**
    * Creates a new point and initializes it with the given coordinates.
    *
    * @param x the x-coordinate.
    * @param y the y-coordinate.
    * @param z the z-coordinate.
    */
   public Box3DO32(double xMin, double xMax, double yMin, double yMax, double zMin, double zMax)
   {
      this.xMin = xMin;
      this.xMax = xMax;
      this.yMin = yMin;
      this.yMax = yMax;
      this.zMin = zMin;
      this.zMax = zMax;
      this.point1 = new Point3D(xMin,yMin,zMin);
      this.point2 = new Point3D(xMin,yMin,zMax);
      this.point3 = new Point3D(xMin,yMax,zMin);
      this.point4 = new Point3D(xMin,yMax,zMax);
      this.point5 = new Point3D(xMax,yMin,zMin);
      this.point6 = new Point3D(xMax,yMin,zMax);
      this.point7 = new Point3D(xMax,yMax,zMin);
      this.point8 = new Point3D(xMax,yMax,zMax);

   }

   public Box3DO32(Point3D point1, Point3D point2, Point3D point3, Point3D point4, Point3D point5, Point3D point6, Point3D point7, Point3D point8)
   {
      this.xMin = 0;
      this.xMax = 0;
      this.yMin = 0;
      this.yMax = 0;
      this.zMin = 0;
      this.zMax = 0;

      this.point1 = point1;
      this.point2 = point2;
      this.point3 = point3;
      this.point4 = point4;
      this.point5 = point5;
      this.point6 = point6;
      this.point7 = point7;
      this.point8 = point8;

   }


   /**
    * Creates a new point and initializes its component {@code x}, {@code y}, {@code z} in order from
    * the given array.
    *
    * @param pointArray the array containing this point's coordinates. Not modified.
    */
   public Box3DO32(double[] pointArray)
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
      this.point1 = new Point3D(xMin,yMin,zMin);
      this.point2 = new Point3D(xMin,yMin,zMax);
      this.point3 = new Point3D(xMin,yMax,zMin);
      this.point4 = new Point3D(xMin,yMax,zMax);
      this.point5 = new Point3D(xMax,yMin,zMin);
      this.point6 = new Point3D(xMax,yMin,zMax);
      this.point7 = new Point3D(xMax,yMax,zMin);
      this.point8 = new Point3D(xMax,yMax,zMax);
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

   public Point3D getPoint1(){
      return this.point1;
   }
   public void setPoint1(Point3D point1){
      this.point1 = point1;
   }

   public Point3D getPoint2(){
      return this.point2;
   }
   public void setPoint2(Point3D point2){
      this.point2 = point2;
   }


   public Point3D getPoint3(){
      return this.point3;
   }
   public void setPoint3(Point3D point3){
      this.point3 = point3;
   }

   public Point3D getPoint4(){
      return this.point4;
   }
   public void setPoint4(Point3D point4){
      this.point4 = point4;
   }

   public Point3D getPoint5(){
      return this.point5;
   }
   public void setPoint5(Point3D point5){
      this.point5 = point5;
   }

   public Point3D getPoint6(){
      return this.point6;
   }
   public void setPoint6(Point3D point6){
      this.point6 = point6;
   }

   public Point3D getPoint7(){
      return this.point7;
   }
   public void setPoint7(Point3D point7){
      this.point7 = point7;
   }

   public Point3D getPoint8(){
      return this.point8;
   }
   public void setPoint8(Point3D point8){
      this.point8 = point8;
   }

   public void set8points(Point3D point1, Point3D point2, Point3D point3, Point3D point4, Point3D point5, Point3D point6, Point3D point7, Point3D point8){
      this.point1 = point1;
      this.point2 = point2;
      this.point3 = point3;
      this.point4 = point4;
      this.point5 = point5;
      this.point6 = point6;
      this.point7 = point7;
      this.point8 = point8;
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
   public void set(Box3DO32 box){
      this.xMin = box.xMin;
      this.xMax = box.xMax;
      this.yMin = box.yMin;
      this.yMax = box.yMax;
      this.zMin = box.zMin;
      this.zMax = box.zMax;
      this.Transform = box.Transform;
      this.point1 = box.getPoint1();
      this.point2 = box.getPoint2();
      this.point3 = box.getPoint3();
      this.point4 = box.getPoint4();
      this.point5 = box.getPoint5();
      this.point6 = box.getPoint6();
      this.point7 = box.getPoint7();
      this.point8 = box.getPoint8();


   }
}
