package us.ihmc.robotics.math.frames;

import javax.vecmath.Quat4d;
import javax.vecmath.Tuple3d;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.ReferenceFrameHolder;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoFramePose extends ReferenceFrameHolder
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

   public void setPosition(FramePoint framePoint)
   {
      boolean notifyListeners = true;
      position.set(framePoint, notifyListeners);
   }
   
   public void setPosition(Tuple3d position)
   {
      this.position.set(position);      
   }
   
   public void setOrientation(FrameOrientation frameOrientation)
   {
      boolean notifyListeners = true;
      orientation.set(frameOrientation, notifyListeners);
   }
   
   public void setOrientation(Quat4d quaternion)
   {
      orientation.set(quaternion);
   }
   
   public void set(FramePoint framePoint, FrameOrientation frameOrientation)
   {
      boolean notifyListeners = true;
      position.set(framePoint, notifyListeners);
      orientation.set(frameOrientation, notifyListeners);
   }

   public void setAndMatchFrame(FramePoint framePoint, FrameOrientation frameOrientation)
   {
      boolean notifyListeners = true;
      position.setAndMatchFrame(framePoint, notifyListeners);
      orientation.setAndMatchFrame(frameOrientation, notifyListeners);
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

   public void setYawPitchRoll(double[] yawPitchRoll)
   {
      orientation.setYawPitchRoll(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);
   }

   public void setXYZYawPitchRoll(double[] pose)
   {
      setXYZ(pose[0], pose[1], pose[2]);
      setYawPitchRoll(pose[3], pose[4], pose[5]);
   }

   public void setToNaN()
   {
      position.setToNaN();
      orientation.setToNaN();
   }

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
}



















