package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface GenericSphereController
{
   void doControl();
   Vector3DReadOnly getForces();
}
