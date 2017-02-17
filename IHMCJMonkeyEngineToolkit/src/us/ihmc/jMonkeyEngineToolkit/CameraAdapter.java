package us.ihmc.jMonkeyEngineToolkit;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public interface CameraAdapter
{

   public abstract Point3DReadOnly getCameraPosition();

   public abstract QuaternionReadOnly getCameraRotation();

   public abstract float getHorizontalFovInRadians();

}
