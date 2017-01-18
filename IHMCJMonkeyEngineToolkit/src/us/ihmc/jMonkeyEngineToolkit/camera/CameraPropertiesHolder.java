package us.ihmc.jMonkeyEngineToolkit.camera;

public interface CameraPropertiesHolder
{
   public boolean isTracking();

   public boolean isTrackingX();

   public boolean isTrackingY();

   public boolean isTrackingZ();

   public boolean isDolly();

   public boolean isDollyX();

   public boolean isDollyY();

   public boolean isDollyZ();

   public void setTracking(boolean track);

   public void setTrackingX(boolean trackX);

   public void setTrackingY(boolean trackY);

   public void setTrackingZ(boolean trackZ);

   public void setDolly(boolean dolly);

   public void setDollyX(boolean dollyX);

   public void setDollyY(boolean dollyY);

   public void setDollyZ(boolean dollyZ);

   public double getTrackingXOffset();

   public double getTrackingYOffset();

   public double getTrackingZOffset();

   public double getDollyXOffset();

   public double getDollyYOffset();

   public double getDollyZOffset();

   public void setTrackingXOffset(double dx);

   public void setTrackingYOffset(double dy);

   public void setTrackingZOffset(double dz);

   public void setDollyXOffset(double dx);

   public void setDollyYOffset(double dy);

   public void setDollyZOffset(double dz);
   
   public void setFieldOfView(double fieldOfView);
   public void setClipDistanceNear(double near);
   public void setClipDistanceFar(double far);

   public double getFixX();

   public double getFixY();

   public double getFixZ();

   public double getCamX();

   public double getCamY();

   public double getCamZ();

   public void setFixX(double fx);

   public void setFixY(double fy);

   public void setFixZ(double fz);

   public void setCamX(double cx);

   public void setCamY(double cy);

   public void setCamZ(double cz);

   public double getTrackXVar();

   public double getTrackYVar();

   public double getTrackZVar();

   public double getDollyXVar();

   public double getDollyYVar();

   public double getDollyZVar();

   public void update();
}
