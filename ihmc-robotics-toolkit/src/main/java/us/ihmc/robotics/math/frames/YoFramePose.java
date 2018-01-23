package us.ihmc.robotics.math.frames;

import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoFramePose implements ReferenceFrameHolder, Clearable
{
   private final YoFramePoint position;
   private final YoFrameOrientation orientation;

   private final FrameQuaternion tempFrameOrientation = new FrameQuaternion();

   public YoFramePose(YoFramePoint position, YoFrameOrientation orientation)
   {
      position.checkReferenceFrameMatch(orientation);
      this.position = position;
      this.orientation = orientation;
   }

   public YoFramePose(String prefix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      this(prefix, "", frame, registry);
   }

   public YoFramePose(String prefix, String suffix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      position = new YoFramePoint(prefix, suffix, frame, registry);
      orientation = new YoFrameOrientation(prefix, suffix, frame, registry);
   }

   public YoFramePoint getPosition()
   {
      return position;
   }

   public YoFrameOrientation getOrientation()
   {
      return orientation;
   }

   public void getFramePose(FramePose3D framePoseToPack)
   {
      orientation.getFrameOrientationIncludingFrame(tempFrameOrientation);

      framePoseToPack.setPosition(position);
      framePoseToPack.setOrientation(tempFrameOrientation);
   }

   public void getFramePoseIncludingFrame(FramePose3D framePoseToPack)
   {
      framePoseToPack.setToZero(getReferenceFrame());
      getFramePose(framePoseToPack);
   }

   public void getPose(RigidBodyTransform rigidBodyTransformToPack)
   {
      orientation.getFrameOrientationIncludingFrame(tempFrameOrientation);
      rigidBodyTransformToPack.setRotation(tempFrameOrientation);
      rigidBodyTransformToPack.setTranslation(position);
   }

   public void set(FramePose3D framePose)
   {
      framePose.checkReferenceFrameMatch(getReferenceFrame());

      position.set(framePose.getPosition());
      orientation.set(framePose.getOrientation());
   }

   public void setAndMatchFrame(FramePose3D framePose)
   {
      position.setAndMatchFrame(framePose.getPosition());
      orientation.setAndMatchFrame(framePose.getOrientation());
   }

   /**
    * Sets this frame pose to the origin of the passed in reference frame.
    * 
    * @param referenceFrame
    */
   public void setFromReferenceFrame(ReferenceFrame referenceFrame)
   {
      position.setFromReferenceFrame(referenceFrame);
      orientation.setFromReferenceFrame(referenceFrame);
   }

   public void setPosition(FramePoint3D framePoint)
   {
      position.set(framePoint);
   }

   public void setPosition(Tuple3DReadOnly position)
   {
      this.position.set(position);
   }

   public void setOrientation(FrameQuaternion frameOrientation)
   {
      orientation.set(frameOrientation);
   }

   public void setOrientation(QuaternionReadOnly quaternion)
   {
      orientation.set(quaternion);
   }

   public void set(FramePoint3DReadOnly framePoint, FrameQuaternionReadOnly frameOrientation)
   {
      position.set(framePoint);
      orientation.set(frameOrientation);
   }

   public void set(YoFramePose yoFramePose)
   {
      set(yoFramePose.getPosition(), yoFramePose.getOrientation().getFrameOrientation());
   }

   public void setAndMatchFrame(FramePoint3DReadOnly framePoint, FrameQuaternionReadOnly frameOrientation)
   {
      position.setAndMatchFrame(framePoint);
      orientation.setAndMatchFrame(frameOrientation);
   }

   public void setPosition(double x, double y, double z)
   {
      position.set(x, y, z);
   }

   public void setXYZ(double x, double y, double z)
   {
      position.set(x, y, z);
   }

   public void setXYZ(double[] pos)
   {
      setXYZ(pos[0], pos[1], pos[2]);
   }

   public void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      orientation.setYawPitchRoll(yaw, pitch, roll);
   }

   public void setYaw(double yaw)
   {
      orientation.setYaw(yaw);
   }

   public void setPitch(double pitch)
   {
      orientation.setPitch(pitch);
   }

   public void setRoll(double roll)
   {
      orientation.setRoll(roll);
   }

   public void setYawPitchRoll(double[] yawPitchRoll)
   {
      orientation.setYawPitchRoll(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);
   }

   public void setXYZYawPitchRoll(double[] pose)
   {
      setXYZ(pose[0], pose[1], pose[2]);
      setYawPitchRoll(pose[3], pose[4], pose[5]);
   }

   @Override
   public void setToNaN()
   {
      position.setToNaN();
      orientation.setToNaN();
   }

   @Override
   public void setToZero()
   {
      position.setToZero();
      orientation.setToZero();
   }

   @Override
   public boolean containsNaN()
   {
      return position.containsNaN() || orientation.containsNaN();
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return position.getReferenceFrame();
   }

   public void attachVariableChangedListener(VariableChangedListener variableChangedListener)
   {
      position.attachVariableChangedListener(variableChangedListener);
      orientation.attachVariableChangedListener(variableChangedListener);
   }

   public double getDistance(YoFramePose goalYoPose)
   {
      return position.distance(goalYoPose.getPosition());
   }

   public void setX(double x)
   {
      position.setX(x);
   }

   public void setY(double y)
   {
      position.setY(y);
   }

   public void setZ(double z)
   {
      position.setZ(z);
   }

   public double getX()
   {
      return getPosition().getX();
   }

   public double getY()
   {
      return getPosition().getY();
   }

   public double getZ()
   {
      return getPosition().getZ();
   }

   public double getRoll()
   {
      return getOrientation().getRoll().getDoubleValue();
   }

   public double getPitch()
   {
      return getOrientation().getPitch().getDoubleValue();
   }

   public double getYaw()
   {
      return getOrientation().getYaw().getDoubleValue();
   }

   public void add(YoFramePose yoFramePose)
   {
      getPosition().add(yoFramePose.getPosition());
      getOrientation().add(yoFramePose.getOrientation());
   }

   public YoDouble getYoX()
   {
      return getPosition().getYoX();
   }

   public YoDouble getYoY()
   {
      return getPosition().getYoY();
   }

   public YoDouble getYoZ()
   {
      return getPosition().getYoZ();
   }

   public YoDouble getYoPitch()
   {
      return getOrientation().getPitch();
   }

   public YoDouble getYoRoll()
   {
      return getOrientation().getRoll();
   }

   public YoDouble getYoYaw()
   {
      return getOrientation().getYaw();
   }
}
