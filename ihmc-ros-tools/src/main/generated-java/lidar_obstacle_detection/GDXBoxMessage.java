package lidar_obstacle_detection;

public interface GDXBoxMessage extends org.ros.internal.message.Message {
   static final java.lang.String _TYPE = "lidar_obstacle_detection/GDXBoxMessage";
   static final java.lang.String _DEFINITION =
          "float64 x_min\n"
         +"float64 y_min\n"
         +"float64 z_min\n"
         +"float64 x_max\n"
         +"float64 y_max\n"
         +"float64 z_max\n";
   double getx_min();
   void setx_min(double value);
   double  gety_min();
   void sety_min(double value);
   double  getz_min();
   void setz_min(double value);
   double  getx_max();
   void setx_max(double value);
   double  gety_max();
   void sety_max(double value);
   double  getz_max();
   void setz_max(double value);

}
