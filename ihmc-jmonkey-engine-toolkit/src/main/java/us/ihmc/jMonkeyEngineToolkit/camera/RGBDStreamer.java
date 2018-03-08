package us.ihmc.jMonkeyEngineToolkit.camera;

import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public interface RGBDStreamer
{
   public long getTimeStamp();

   public Point3DReadOnly getCameraPosition();

   public QuaternionReadOnly getCameraOrientation();

   public double getFieldOfView();

   /**
    * Called every time a new RGBD image is available
    * 
    * @param coordinates Point coordinates. This array is re-used after this call
    * @param colors A-RGB values of the coordinates.  This array is re-used after this call
    * @param timeStamp
    * @param cameraPosition
    * @param cameraOrientation
    */
   public void updateRBGD(Point3D32[] coordinates, int[] colors, long timeStamp, Point3DReadOnly cameraPosition, QuaternionReadOnly cameraOrientation);

   public boolean isReadyForNewData();

}
