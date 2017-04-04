package us.ihmc.robotics.math.frames;

import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.AbstractReferenceFrameHolder;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoFramePose extends AbstractReferenceFrameHolder implements Clearable
{
   private final YoFramePoint position;
   private final YoFrameOrientation orientation;

   private final FramePoint tempFramePoint = new FramePoint();
   private final FrameOrientation tempFrameOrientation = new FrameOrientation();

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

   public void getFramePose(FramePose framePoseToPack)
   {
      position.getFrameTupleIncludingFrame(tempFramePoint);
      orientation.getFrameOrientationIncludingFrame(tempFrameOrientation);

      framePoseToPack.setPosition(tempFramePoint);
      framePoseToPack.setOrientation(tempFrameOrientation);
   }

   public void getFramePoseIncludingFrame(FramePose framePoseToPack)
   {
      framePoseToPack.setToZero(getReferenceFrame());
      getFramePose(framePoseToPack);
   }

   public void getPose(RigidBodyTransform rigidBodyTransformToPack)
   {
      position.getFrameTupleIncludingFrame(tempFramePoint);
      orientation.getFrameOrientationIncludingFrame(tempFrameOrientation);
      tempFrameOrientation.getTransform3D(rigidBodyTransformToPack);
      rigidBodyTransformToPack.setTranslation(tempFramePoint.getX(), tempFramePoint.getY(), tempFramePoint.getZ());
   }

   public void set(FramePose framePose)
   {
      set(framePose, true);
   }

   public void set(FramePose framePose, boolean notifyListeners)
   {
      framePose.checkReferenceFrameMatch(getReferenceFrame());

      framePose.getPositionIncludingFrame(tempFramePoint);
      framePose.getOrientationIncludingFrame(tempFrameOrientation);
      position.set(tempFramePoint, notifyListeners);
      orientation.set(tempFrameOrientation, notifyListeners);
   }

   public void setAndMatchFrame(FramePose framePose)
   {
      setAndMatchFrame(framePose, true);
   }

   public void setAndMatchFrame(FramePose framePose, boolean notifyListeners)
   {
      framePose.getPositionIncludingFrame(tempFramePoint);
      framePose.getOrientationIncludingFrame(tempFrameOrientation);
      tempFramePoint.changeFrame(getReferenceFrame());
      tempFrameOrientation.changeFrame(getReferenceFrame());
      position.set(tempFramePoint, notifyListeners);
      orientation.set(tempFrameOrientation, notifyListeners);
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

   public void setPosition(FramePoint framePoint)
   {
      boolean notifyListeners = true;
      position.set(framePoint, notifyListeners);
   }

   public void setPosition(Tuple3DReadOnly position)
   {
      this.position.set(position);
   }

   public void setOrientation(FrameOrientation frameOrientation)
   {
      boolean notifyListeners = true;
      orientation.set(frameOrientation, notifyListeners);
   }

   public void setOrientation(QuaternionReadOnly quaternion)
   {
      orientation.set(quaternion);
   }

   public void set(FramePoint framePoint, FrameOrientation frameOrientation)
   {
      boolean notifyListeners = true;
      position.set(framePoint, notifyListeners);
      orientation.set(frameOrientation, notifyListeners);
   }

   public void set(YoFramePose yoFramePose)
   {
      set(yoFramePose.getPosition().getFrameTuple(), yoFramePose.getOrientation().getFrameOrientation());
   }

   public void setAndMatchFrame(FramePoint framePoint, FrameOrientation frameOrientation)
   {
      boolean notifyListeners = true;
      position.setAndMatchFrame(framePoint, notifyListeners);
      orientation.setAndMatchFrame(frameOrientation, notifyListeners);
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

   public DoubleYoVariable getYoX()
   {
      return getPosition().getYoX();
   }

   public DoubleYoVariable getYoY()
   {
      return getPosition().getYoY();
   }

   public DoubleYoVariable getYoZ()
   {
      return getPosition().getYoZ();
   }

   public DoubleYoVariable getYoPitch()
   {
      return getOrientation().getPitch();
   }

   public DoubleYoVariable getYoRoll()
   {
      return getOrientation().getRoll();
   }

   public DoubleYoVariable getYoYaw()
   {
      return getOrientation().getYaw();
   }
}
