package us.ihmc.simulationconstructionset.gui.camera;

import us.ihmc.jMonkeyEngineToolkit.camera.CameraPropertiesHolder;

public class AbstractCameraPropertiesHolder implements CameraPropertiesHolder
{
   public boolean isTracking()
   {
      return false;
   }


   public boolean isTrackingX()
   {
      return false;
   }


   public boolean isTrackingY()
   {
      return false;
   }


   public boolean isTrackingZ()
   {
      return false;
   }


   public boolean isDolly()
   {
      return false;
   }


   public boolean isDollyX()
   {
      return false;
   }


   public boolean isDollyY()
   {
      return false;
   }


   public boolean isDollyZ()
   {
      return false;
   }


   public void setTracking(boolean track)
   {
   }


   public void setTrackingX(boolean trackX)
   {
   }


   public void setTrackingY(boolean trackY)
   {
   }


   public void setTrackingZ(boolean trackZ)
   {
   }


   public void setDolly(boolean dolly)
   {
   }


   public void setDollyX(boolean dollyX)
   {
   }


   public void setDollyY(boolean dollyY)
   {
   }


   public void setDollyZ(boolean dollyZ)
   {
   }


   public double getTrackingXOffset()
   {
      return 0;
   }


   public double getTrackingYOffset()
   {
      return 0;
   }


   public double getTrackingZOffset()
   {
      return 0;
   }


   public double getDollyXOffset()
   {
      return 0;
   }


   public double getDollyYOffset()
   {
      return 0;
   }


   public double getDollyZOffset()
   {
      return 0;
   }


   public void setTrackingXOffset(double dx)
   {
   }


   public void setTrackingYOffset(double dy)
   {
   }


   public void setTrackingZOffset(double dz)
   {
   }


   public void setDollyXOffset(double dx)
   {
   }


   public void setDollyYOffset(double dy)
   {
   }


   public void setDollyZOffset(double dz)
   {
   }


   public double getFixX()
   {
      return 0;
   }


   public double getFixY()
   {
      return 0;
   }


   public double getFixZ()
   {
      return 0;
   }


   public double getCamX()
   {
      return 0;
   }


   public double getCamY()
   {
      return 0;
   }


   public double getCamZ()
   {
      return 0;
   }


   public void setFixX(double fx)
   {
   }


   public void setFixY(double fy)
   {
   }


   public void setFixZ(double fz)
   {
   }


   public void setCamX(double cx)
   {
   }


   public void setCamY(double cy)
   {
   }


   public void setCamZ(double cz)
   {
   }


   public double getTrackXVar()
   {
      return 0;
   }


   public double getTrackYVar()
   {
      return 0;
   }


   public double getTrackZVar()
   {
      return 0;
   }


   public double getDollyXVar()
   {
      return 0;
   }


   public double getDollyYVar()
   {
      return 0;
   }


   public double getDollyZVar()
   {
      return 0;
   }


   public void update()
   {
   }


   public void setFieldOfView(double fieldOfView)
   {
      
   }


   public void setClipDistanceNear(double near)
   {
      
   }


   public void setClipDistanceFar(double far)
   {
      
   }

}
