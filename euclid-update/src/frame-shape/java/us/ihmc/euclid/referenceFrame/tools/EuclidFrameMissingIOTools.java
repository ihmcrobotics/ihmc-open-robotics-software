package us.ihmc.euclid.referenceFrame.tools;

import us.ihmc.euclid.referenceFrame.interfaces.FramePlane3DReadOnly;

import static us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools.getLine3DString;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools.getPlane3DString;

public class EuclidFrameMissingIOTools
{
   public static String getFramePlane3DString(String format, FramePlane3DReadOnly plane3D)
   {
      if (plane3D == null)
         return "null";
      else
         return getPlane3DString(format, plane3D) + ", " + plane3D.getReferenceFrame();
   }
}
