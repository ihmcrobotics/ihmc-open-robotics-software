package lidar_obstacle_detection;

public interface GDXBoxMessage extends org.ros.internal.message.Message {
   static final java.lang.String _TYPE = "lidar_obstacle_detection/TunningParam";
   static final java.lang.String _DEFINITION =
          "float64 xMin\n"
         +"float64 yMin\n"
         +"float64 zMin\n"
         +"float64 xMax\n"
         +"float64 yMax\n"
         +"float64 zMax\n";
   double getXMin();
   void setXMin(double xMin);
   double  getYMin();
   void setYMin(double yMin);
   double  getZMin();
   void setZMin(double zMin);
   double  getXMax();
   void setXMax(double xMax);
   double  getYMax();
   void setYMax(double yMax);
   double  getZMax();
   void setZMax(double zMax);

}
