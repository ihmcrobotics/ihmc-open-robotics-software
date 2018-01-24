package us.ihmc.robotics.math.frames;

import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameLine2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector2DBasics;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

// Note: You should only make these once at the initialization of a controller. You shouldn't make
// any on the fly
// since they contain YoVariables.
public class YoFrameLine2d implements FixedFrameLine2DBasics
{
   private final YoDouble pointX, pointY, vectorX, vectorY; // This is where the data is stored. All operations must act on these numbers.
   private final ReferenceFrame referenceFrame;
   protected FrameLine2D frameLine; // This is only for assistance. The data is stored in the YoVariables, not in here!

   private final FixedFramePoint2DBasics point = new FixedFramePoint2DBasics()
   {
      @Override
      public void setX(double x)
      {
         pointX.set(x);
      }

      @Override
      public void setY(double y)
      {
         pointY.set(y);
      }

      @Override
      public ReferenceFrame getReferenceFrame()
      {
         return referenceFrame;
      }

      @Override
      public double getX()
      {
         return pointX.getDoubleValue();
      }

      @Override
      public double getY()
      {
         return pointY.getDoubleValue();
      }
   };

   private final FixedFrameVector2DBasics direction = new FixedFrameVector2DBasics()
   {
      @Override
      public void setX(double x)
      {
         vectorX.set(x);
      }

      @Override
      public void setY(double y)
      {
         vectorY.set(y);
      }

      @Override
      public ReferenceFrame getReferenceFrame()
      {
         return referenceFrame;
      }

      @Override
      public double getX()
      {
         return vectorX.getDoubleValue();
      }

      @Override
      public double getY()
      {
         return vectorY.getDoubleValue();
      }
   };

   public YoFrameLine2d(String namePrefix, String nameSuffix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      pointX = new YoDouble(namePrefix + "PointX" + nameSuffix, registry);
      pointY = new YoDouble(namePrefix + "PointY" + nameSuffix, registry);
      vectorX = new YoDouble(namePrefix + "VectorX" + nameSuffix, registry);
      vectorY = new YoDouble(namePrefix + "VectorY" + nameSuffix, registry);

      this.referenceFrame = frame;
   }

   public YoFrameLine2d(YoDouble pointX, YoDouble pointY, YoDouble vectorX, YoDouble vectorY, ReferenceFrame frame)
   {
      this.pointX = pointX;
      this.pointY = pointY;
      this.vectorX = vectorX;
      this.vectorY = vectorY;

      this.referenceFrame = frame;
   }

   @Override
   public FixedFramePoint2DBasics getPoint()
   {
      return point;
   }

   @Override
   public FixedFrameVector2DBasics getDirection()
   {
      return direction;
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public YoDouble getYoPointX()
   {
      return pointX;
   }

   public YoDouble getYoPointY()
   {
      return pointY;
   }

   public YoDouble getYoVectorX()
   {
      return vectorX;
   }

   public YoDouble getYoVectorY()
   {
      return vectorY;
   }

   @Override
   public String toString()
   {
      return "(" + pointX.getDoubleValue() + ", " + pointY.getDoubleValue() + "), (" + vectorX.getDoubleValue() + ", " + vectorY.getDoubleValue() + ")-"
            + referenceFrame;
   }
}
