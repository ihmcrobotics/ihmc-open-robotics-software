package ihmc_msgs;

public interface GDXBoxMessage extends org.ros.internal.message.Message {
   static final java.lang.String _TYPE = "ihmc_msgs/GDXBoxesMessage";
   static final java.lang.String _DEFINITION = "## GDXBoxesMessage\n# This message is used to visualize the bounding box of the pointcloud\n\n\n";
   double getminx();
   void setminx(double value);
   double  getminy();
   void setminy(double value);
   double  getminz();
   void setminz(double value);
   double  getmaxx();
   void setmaxx(double value);
   double  getmaxy();
   void setmaxy(double value);
   double  getmaxz();
   void setmaxz(double value);

}
