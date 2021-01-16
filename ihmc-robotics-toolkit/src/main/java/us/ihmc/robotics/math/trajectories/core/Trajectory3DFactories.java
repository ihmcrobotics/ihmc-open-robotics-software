package us.ihmc.robotics.math.trajectories.core;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;

import static us.ihmc.euclid.tools.EuclidHashCodeTools.toIntHashCode;

public class Trajectory3DFactories
{
   public static Point3DReadOnly newLinkedPoint3DReadOnly(DoubleProvider xProvider,
                                                          DoubleProvider yProvider,
                                                          DoubleProvider zProvider)
   {
      return new Point3DReadOnly()
      {
         @Override
         public double getX()
         {
            return xProvider.getValue();
         }

         @Override
         public double getY()
         {
            return yProvider.getValue();
         }

         @Override
         public double getZ()
         {
            return zProvider.getValue();
         }


         @Override
         public boolean equals(Object object)
         {
            if (object == this)
               return true;
            else if (object instanceof Point3DReadOnly)
               return equals((Point3DReadOnly) object);
            else
               return false;
         }

         @Override
         public int hashCode()
         {
            return toIntHashCode(toIntHashCode(getX(), getY(), getZ()));
         }

         @Override
         public String toString()
         {
            return EuclidCoreIOTools.getTuple3DString(this);
         }
      };
   }

   public static Vector3DReadOnly newLinkedVector3DReadOnly(DoubleProvider xProvider,
                                                            DoubleProvider yProvider,
                                                            DoubleProvider zProvider)
   {
      return new Vector3DReadOnly()
      {
         @Override
         public double getX()
         {
            return xProvider.getValue();
         }

         @Override
         public double getY()
         {
            return yProvider.getValue();
         }

         @Override
         public double getZ()
         {
            return zProvider.getValue();
         }


         @Override
         public boolean equals(Object object)
         {
            if (object == this)
               return true;
            else if (object instanceof Vector3DReadOnly)
               return equals((Vector3DReadOnly) object);
            else
               return false;
         }

         @Override
         public int hashCode()
         {
            return toIntHashCode(toIntHashCode(getX(), getY(), getZ()));
         }

         @Override
         public String toString()
         {
            return EuclidCoreIOTools.getTuple3DString(this);
         }
      };
   }
}
