package us.ihmc.robotics.math.frames;

import us.ihmc.euclid.referenceFrame.FrameLineSegment2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameLineSegment2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Note: You should only make these once at the initialization of a controller. You shouldn't make
 * any on the fly since they contain YoVariables.
 */
public class YoFrameLineSegment2d implements FixedFrameLineSegment2DBasics
{
   /** This is where the data is stored. All operations must act on these numbers. */
   private final YoDouble firstEndpointX, firstEndpointY, secondEndpointX, secondEndpointY;
   private final ReferenceFrame referenceFrame;
   /** This is only for assistance. The data is stored in the YoVariables, not in here! */
   protected FrameLineSegment2D frameLineSegment;

   private final FixedFramePoint2DBasics firstEndpoint = new FixedFramePoint2DBasics()
   {
      @Override
      public void setX(double x)
      {
         firstEndpointX.set(x);
      }

      @Override
      public void setY(double y)
      {
         firstEndpointY.set(y);
      }

      @Override
      public ReferenceFrame getReferenceFrame()
      {
         return referenceFrame;
      }

      @Override
      public double getX()
      {
         return firstEndpointX.getDoubleValue();
      }

      @Override
      public double getY()
      {
         return firstEndpointY.getDoubleValue();
      }
   };

   private final FixedFramePoint2DBasics secondEndpoint = new FixedFramePoint2DBasics()
   {
      @Override
      public void setX(double x)
      {
         secondEndpointX.set(x);
      }

      @Override
      public void setY(double y)
      {
         secondEndpointY.set(y);
      }

      @Override
      public ReferenceFrame getReferenceFrame()
      {
         return referenceFrame;
      }

      @Override
      public double getX()
      {
         return secondEndpointX.getDoubleValue();
      }

      @Override
      public double getY()
      {
         return secondEndpointY.getDoubleValue();
      }
   };

   public YoFrameLineSegment2d(String namePrefix, String nameSuffix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      this(namePrefix, nameSuffix, "", frame, registry);
   }

   public YoFrameLineSegment2d(String namePrefix, String nameSuffix, String description, ReferenceFrame frame, YoVariableRegistry registry)
   {
      firstEndpointX = new YoDouble(namePrefix + "FirstEndpointX" + nameSuffix, description, registry);
      firstEndpointY = new YoDouble(namePrefix + "FirstEndpointY" + nameSuffix, description, registry);
      secondEndpointX = new YoDouble(namePrefix + "SecondEndpointX" + nameSuffix, description, registry);
      secondEndpointY = new YoDouble(namePrefix + "SecondEndpointY" + nameSuffix, description, registry);

      this.referenceFrame = frame;
      frameLineSegment = new FrameLineSegment2D(referenceFrame);
   }

   public YoFrameLineSegment2d(YoDouble firstEndpointX, YoDouble firstEndpointY, YoDouble secondEndpointX, YoDouble secondEndpointY, ReferenceFrame frame)
   {
      this.firstEndpointX = firstEndpointX;
      this.firstEndpointY = firstEndpointY;
      this.secondEndpointX = secondEndpointX;
      this.secondEndpointY = secondEndpointY;

      this.referenceFrame = frame;
      frameLineSegment = new FrameLineSegment2D(referenceFrame);
   }

   @Override
   public FixedFramePoint2DBasics getFirstEndpoint()
   {
      return firstEndpoint;
   }

   @Override
   public FixedFramePoint2DBasics getSecondEndpoint()
   {
      return secondEndpoint;
   }

   public YoDouble getYoFirstEndpointX()
   {
      return firstEndpointX;
   }

   public YoDouble getYoFirstEndpointY()
   {
      return firstEndpointY;
   }

   public YoDouble getYoSecondEndpointX()
   {
      return secondEndpointX;
   }

   public YoDouble getYoSecondEndpointY()
   {
      return secondEndpointY;
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public boolean areEndpointsTheSame()
   {
      return firstEndpoint.equals(secondEndpoint);
   }

   public YoDouble[] getDoubleYoVariables()
   {
      return new YoDouble[] {firstEndpointX, firstEndpointY, secondEndpointX, secondEndpointY};
   }

   @Override
   public String toString()
   {
      return "(" + firstEndpointX.getDoubleValue() + ", " + firstEndpointY.getDoubleValue() + "), (" + secondEndpointX.getDoubleValue() + ", "
            + secondEndpointY.getDoubleValue() + ")-" + referenceFrame;
   }
}
