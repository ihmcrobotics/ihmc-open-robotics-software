package us.ihmc.simulationconstructionset.gui.camera;

import us.ihmc.jMonkeyEngineToolkit.camera.CameraPropertiesHolder;

public class AbstractCameraPropertiesHolder implements CameraPropertiesHolder
{
   @Override
   public boolean isTracking()
   {
      return false;
   }


   @Override
   public boolean isTrackingX()
   {
      return false;
   }


   @Override
   public boolean isTrackingY()
   {
      return false;
   }


   @Override
   public boolean isTrackingZ()
   {
      return false;
   }


   @Override
   public boolean isDolly()
   {
      return false;
   }


   @Override
   public boolean isDollyX()
   {
      return false;
   }


   @Override
   public boolean isDollyY()
   {
      return false;
   }


   @Override
   public boolean isDollyZ()
   {
      return false;
   }


   @Override
   public void setTracking(boolean track)
   {
   }


   @Override
   public void setTrackingX(boolean trackX)
   {
   }


   @Override
   public void setTrackingY(boolean trackY)
   {
   }


   @Override
   public void setTrackingZ(boolean trackZ)
   {
   }


   @Override
   public void setDolly(boolean dolly)
   {
   }


   @Override
   public void setDollyX(boolean dollyX)
   {
   }


   @Override
   public void setDollyY(boolean dollyY)
   {
   }


   @Override
   public void setDollyZ(boolean dollyZ)
   {
   }


   @Override
   public double getTrackingXOffset()
   {
      return 0;
   }


   @Override
   public double getTrackingYOffset()
   {
      return 0;
   }


   @Override
   public double getTrackingZOffset()
   {
      return 0;
   }


   @Override
   public double getDollyXOffset()
   {
      return 0;
   }


   @Override
   public double getDollyYOffset()
   {
      return 0;
   }


   @Override
   public double getDollyZOffset()
   {
      return 0;
   }


   @Override
   public void setTrackingXOffset(double dx)
   {
   }


   @Override
   public void setTrackingYOffset(double dy)
   {
   }


   @Override
   public void setTrackingZOffset(double dz)
   {
   }


   @Override
   public void setDollyXOffset(double dx)
   {
   }


   @Override
   public void setDollyYOffset(double dy)
   {
   }


   @Override
   public void setDollyZOffset(double dz)
   {
   }


   @Override
   public double getFixX()
   {
      return 0;
   }


   @Override
   public double getFixY()
   {
      return 0;
   }


   @Override
   public double getFixZ()
   {
      return 0;
   }


   @Override
   public double getCamX()
   {
      return 0;
   }


   @Override
   public double getCamY()
   {
      return 0;
   }


   @Override
   public double getCamZ()
   {
      return 0;
   }


   @Override
   public void setFixX(double fx)
   {
   }


   @Override
   public void setFixY(double fy)
   {
   }


   @Override
   public void setFixZ(double fz)
   {
   }


   @Override
   public void setCamX(double cx)
   {
   }


   @Override
   public void setCamY(double cy)
   {
   }


   @Override
   public void setCamZ(double cz)
   {
   }


   @Override
   public double getTrackXVar()
   {
      return 0;
   }


   @Override
   public double getTrackYVar()
   {
      return 0;
   }


   @Override
   public double getTrackZVar()
   {
      return 0;
   }


   @Override
   public double getDollyXVar()
   {
      return 0;
   }


   @Override
   public double getDollyYVar()
   {
      return 0;
   }


   @Override
   public double getDollyZVar()
   {
      return 0;
   }


   @Override
   public void update()
   {
   }


   @Override
   public void setFieldOfView(double fieldOfView)
   {
      
   }


   @Override
   public void setClipDistanceNear(double near)
   {
      
   }


   @Override
   public void setClipDistanceFar(double far)
   {
      
   }

}
