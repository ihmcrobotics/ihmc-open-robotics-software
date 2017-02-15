package us.ihmc.jMonkeyEngineToolkit;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

public interface CameraAdapter
{

   public abstract Point3d getCameraPosition();

   public abstract Quat4d getCameraRotation();

   public abstract float getHorizontalFovInRadians();

}
