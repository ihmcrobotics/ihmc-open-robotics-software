package us.ihmc.robotics.math.frames;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.interfaces.PointInterface;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

//Note: You should only make these once at the initialization of a controller. You shouldn't make any on the fly
//since they contain YoVariables.
public class YoFramePoint extends YoFrameTuple<YoFramePoint, FramePoint> implements PointInterface
{
   public YoFramePoint(String namePrefix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      this(namePrefix, "", frame, registry);
   }
   
   public YoFramePoint(String namePrefix, String nameSuffix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      super(namePrefix, nameSuffix, frame, registry);
   }

   public YoFramePoint(DoubleYoVariable xVariable, DoubleYoVariable yVariable, DoubleYoVariable zVariable, ReferenceFrame frame)
   {
      super(xVariable, yVariable, zVariable, frame);
   }

   protected FramePoint createEmptyFrameTuple()
   {
      return new FramePoint();
   }

   public double distance(FramePoint framePoint)
   {
      return getFrameTuple().distance(framePoint);
   }

   public double distance(YoFramePoint yoFramePoint)
   {
      return distance(yoFramePoint.getFrameTuple());
   }

   public double getXYPlaneDistance(FramePoint framePoint)
   {
      return getFrameTuple().getXYPlaneDistance(framePoint);
   }

   public double getXYPlaneDistance(FramePoint2d framePoint2d)
   {
      return getFrameTuple().getXYPlaneDistance(framePoint2d);
   }

   public double getXYPlaneDistance(YoFramePoint yoFramePoint)
   {
      return getXYPlaneDistance(yoFramePoint.getFrameTuple());
   }

   public double getXYPlaneDistance(YoFramePoint2d yoFramePoint2d)
   {
      return getXYPlaneDistance(yoFramePoint2d.getFrameTuple2d());
   }

   @Override
   public void getPoint(Point3D pointToPack)
   {
      this.get(pointToPack);
   }

   private final Point3D tempPoint = new Point3D();
   
   @Override
   public void setPoint(PointInterface pointInterface)
   {
      pointInterface.getPoint(tempPoint);
      this.set(tempPoint);
   }

   @Override
   public void setPoint(Point3D point)
   {
      this.set(point);
   }

   /**
    * Sets this point to the location of the origin of passed in referenceFrame.
    */
   @Override
   public void setFromReferenceFrame(ReferenceFrame referenceFrame)
   {
      super.setFromReferenceFrame(referenceFrame);
   }
}
