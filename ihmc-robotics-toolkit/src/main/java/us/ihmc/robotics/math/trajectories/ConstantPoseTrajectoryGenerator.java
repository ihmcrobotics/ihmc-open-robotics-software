package us.ihmc.robotics.math.trajectories;

import java.util.ArrayList;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePointInMultipleFrames;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameQuaternionInMultipleFrames;
import us.ihmc.robotics.math.frames.YoMultipleFramesHolder;


public class ConstantPoseTrajectoryGenerator implements PoseTrajectoryGenerator
{
   private final boolean allowMultipleFrames;

   private final YoFramePoint position;
   private final YoFrameQuaternion orientation;

   private final ArrayList<YoMultipleFramesHolder> multipleFramesHolders;

   public ConstantPoseTrajectoryGenerator(YoFramePoint position, YoFrameQuaternion orientation)
   {
      position.checkReferenceFrameMatch(orientation);

      allowMultipleFrames = false;

      this.position = position;
      this.orientation = orientation;

      multipleFramesHolders = null;
   }

   public ConstantPoseTrajectoryGenerator(YoFramePointInMultipleFrames position, YoFrameQuaternionInMultipleFrames orientation)
   {
      position.checkReferenceFrameMatch(orientation);

      allowMultipleFrames = true;

      this.position = position;
      this.orientation = orientation;

      multipleFramesHolders = new ArrayList<YoMultipleFramesHolder>();
      multipleFramesHolders.add(position);
      multipleFramesHolders.add(orientation);
   }

   public ConstantPoseTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      this(namePrefix, false, referenceFrame, parentRegistry);
   }

   public ConstantPoseTrajectoryGenerator(String namePrefix, boolean allowMultipleFrames, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      this.allowMultipleFrames = allowMultipleFrames;

      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());

      YoFramePointInMultipleFrames yoFramePointInMultipleFrames = new YoFramePointInMultipleFrames(namePrefix + "ConstantPosition", registry, referenceFrame);
      position = yoFramePointInMultipleFrames;
      YoFrameQuaternionInMultipleFrames yoFrameQuaternionInMultiplesFrames = new YoFrameQuaternionInMultipleFrames(namePrefix + "ConstantOrientation",
            registry, referenceFrame);
      orientation = yoFrameQuaternionInMultiplesFrames;

      multipleFramesHolders = new ArrayList<YoMultipleFramesHolder>();
      multipleFramesHolders.add(yoFramePointInMultipleFrames);
      multipleFramesHolders.add(yoFrameQuaternionInMultiplesFrames);
   }

   public void registerAndSwitchFrame(ReferenceFrame desiredFrame)
   {
      registerNewTrajectoryFrame(desiredFrame);
      switchTrajectoryFrame(desiredFrame);
   }

   public void registerNewTrajectoryFrame(ReferenceFrame newReferenceFrame)
   {
      checkIfMultipleFramesAllowed();

      for (int i = 0; i < multipleFramesHolders.size(); i++)
         multipleFramesHolders.get(i).registerReferenceFrame(newReferenceFrame);
   }

   public void changeFrame(ReferenceFrame referenceFrame)
   {
      checkIfMultipleFramesAllowed();

      for (int i = 0; i < multipleFramesHolders.size(); i++)
         multipleFramesHolders.get(i).changeFrame(referenceFrame);
   }

   public void switchTrajectoryFrame(ReferenceFrame referenceFrame)
   {
      checkIfMultipleFramesAllowed();

      for (int i = 0; i < multipleFramesHolders.size(); i++)
         multipleFramesHolders.get(i).switchCurrentReferenceFrame(referenceFrame);
   }

   private void checkIfMultipleFramesAllowed()
   {
      if (!allowMultipleFrames)
         throw new RuntimeException("Must set allowMultipleFrames to true in the constructor if you ever want to register a new frame.");
   }

   public void setConstantPose(FramePose constantPose)
   {
      position.checkReferenceFrameMatch(constantPose);
      position.set(constantPose.getX(), constantPose.getY(), constantPose.getZ());
      orientation.set(constantPose.getYaw(), constantPose.getPitch(), constantPose.getRoll());
   }

   public void setConstantPose(FramePoint3D constantPosition, FrameQuaternion constantOrientation)
   {
      this.position.set(position);
      this.orientation.set(orientation);
   }

   public void initialize()
   {
      // Do nothing
   }

   public void compute(double time)
   {
      // Do nothing
   }

   public boolean isDone()
   {
      return true;
   }

   public void getPosition(FramePoint3D positionToPack)
   {
      position.getFrameTupleIncludingFrame(positionToPack);
   }

   public void getVelocity(FrameVector3D velocityToPack)
   {
      velocityToPack.setToZero(position.getReferenceFrame());
   }

   public void getAcceleration(FrameVector3D accelerationToPack)
   {
      accelerationToPack.setToZero(position.getReferenceFrame());
   }

   public void getLinearData(FramePoint3D positionToPack, FrameVector3D velocityToPack, FrameVector3D accelerationToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
   }

   public void getOrientation(FrameQuaternion orientationToPack)
   {
      orientation.getFrameOrientationIncludingFrame(orientationToPack);
   }

   public void getAngularVelocity(FrameVector3D angularVelocityToPack)
   {
      angularVelocityToPack.setToZero(orientation.getReferenceFrame());
   }

   public void getAngularAcceleration(FrameVector3D angularAccelerationToPack)
   {
      angularAccelerationToPack.setToZero(orientation.getReferenceFrame());
   }

   public void getAngularData(FrameQuaternion orientationToPack, FrameVector3D angularVelocityToPack, FrameVector3D angularAccelerationToPack)
   {
      getOrientation(orientationToPack);
      getAngularVelocity(angularVelocityToPack);
      getAngularAcceleration(angularAccelerationToPack);
   }

   private final Quaternion temp = new Quaternion();
   
   public void getPose(FramePose framePoseToPack)
   {
      framePoseToPack.changeFrame(position.getReferenceFrame());
      framePoseToPack.setPosition(position.getFrameTuple());
      
      orientation.get(temp);
      framePoseToPack.setOrientation(temp);
   }
   
   public String toString()
   {
      String ret = "";
      ret += "Current position: " + position.toString();
      ret += "\nCurrent orientation: " + orientation.toString();
      return ret;
   }

   public void showVisualization()
   {
   }

   public void hideVisualization()
   {
   }
}