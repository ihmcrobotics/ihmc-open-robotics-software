package us.ihmc.robotics.math.trajectories.core;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;

import java.util.function.DoubleConsumer;

import static us.ihmc.euclid.tools.EuclidHashCodeTools.toIntHashCode;

public class Trajectory3DFactories
{
   public static Point3DReadOnly newLinkedPoint3DReadOnly(DoubleProvider xProvider, DoubleProvider yProvider, DoubleProvider zProvider)
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

   public static Vector3DReadOnly newLinkedVector3DReadOnly(DoubleProvider xProvider, DoubleProvider yProvider, DoubleProvider zProvider)
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

   public static Point3DBasics newLinkedPoint3DBasics(DoubleProvider xProvider,
                                                      DoubleConsumer xConsumer,
                                                      DoubleProvider yProvider,
                                                      DoubleConsumer yConsumer,
                                                      DoubleProvider zProvider,
                                                      DoubleConsumer zConsumer)
   {
      return new Point3DBasics()
      {
         @Override
         public void setX(double x)
         {
            xConsumer.accept(x);
         }

         @Override
         public void setY(double y)
         {
            yConsumer.accept(y);
         }

         @Override
         public void setZ(double z)
         {
            zConsumer.accept(z);
         }

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

   public static Vector3DBasics newLinkedVector3DBasics(DoubleProvider xProvider,
                                                        DoubleConsumer xConsumer,
                                                        DoubleProvider yProvider,
                                                        DoubleConsumer yConsumer,
                                                        DoubleProvider zProvider,
                                                        DoubleConsumer zConsumer)
   {
      return new Vector3DBasics()
      {
         @Override
         public void setX(double x)
         {
            xConsumer.accept(x);
         }

         @Override
         public void setY(double y)
         {
            yConsumer.accept(y);
         }

         @Override
         public void setZ(double z)
         {
            zConsumer.accept(z);
         }

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
}
