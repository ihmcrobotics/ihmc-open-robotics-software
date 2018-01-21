package us.ihmc.robotics.math.frames;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoFramePoseUsingQuaternions implements FixedFramePose3DBasics
{
   private final ReferenceFrame referenceFrame;
   private final YoFramePoint position;
   private final YoFrameQuaternion orientation;

   public YoFramePoseUsingQuaternions(YoFramePoint position, YoFrameQuaternion orientation)
   {
      position.checkReferenceFrameMatch(orientation);
      referenceFrame = position.getReferenceFrame();
      this.position = position;
      this.orientation = orientation;
   }

   public YoFramePoseUsingQuaternions(String prefix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      this(prefix, "", frame, registry);
   }

   public YoFramePoseUsingQuaternions(String prefix, String suffix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      referenceFrame = frame;
      position = new YoFramePoint(prefix, suffix, frame, registry);
      orientation = new YoFrameQuaternion(prefix, suffix, frame, registry);
   }

   public void set(YoFramePose yoFramePose)
   {
      set(yoFramePose.getPosition(), yoFramePose.getOrientation().getFrameOrientation());
   }

   public void setAndMatchFrame(FramePose3D framePose)
   {
      position.setAndMatchFrame(framePose.getPosition());
      orientation.setAndMatchFrame(framePose.getOrientation());
   }

   public void setAndMatchFrame(FramePoint3DReadOnly framePoint, FrameQuaternionReadOnly frameOrientation)
   {
      position.setAndMatchFrame(framePoint);
      orientation.setAndMatchFrame(frameOrientation);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public YoFramePoint getPosition()
   {
      return position;
   }

   @Override
   public YoFrameQuaternion getOrientation()
   {
      return orientation;
   }

   public YoDouble getYoX()
   {
      return position.getYoX();
   }

   public YoDouble getYoY()
   {
      return position.getYoY();
   }

   public YoDouble getYoZ()
   {
      return position.getYoZ();
   }

   public YoDouble getYoQs()
   {
      return orientation.getYoQs();
   }

   public YoDouble getYoQx()
   {
      return orientation.getYoQx();
   }

   public YoDouble getYoQy()
   {
      return orientation.getYoQy();
   }

   public YoDouble getYoQz()
   {
      return orientation.getYoQz();
   }

   public void attachVariableChangedListener(VariableChangedListener variableChangedListener)
   {
      position.attachVariableChangedListener(variableChangedListener);
      orientation.attachVariableChangedListener(variableChangedListener);
   }

   @Override
   public String toString()
   {
      return EuclidGeometryIOTools.getPose3DString(new Pose3D(this)) + "-" + referenceFrame;
   }
}
