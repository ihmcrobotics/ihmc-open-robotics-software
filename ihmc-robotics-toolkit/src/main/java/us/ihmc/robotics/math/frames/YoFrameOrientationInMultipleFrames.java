package us.ihmc.robotics.math.frames;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameYawPitchRoll;

public class YoFrameOrientationInMultipleFrames extends YoFrameYawPitchRoll implements YoMultipleFramesHolder
{
   private final YoMultipleFramesHelper multipleFramesHelper;

   private final FrameQuaternion frameOrientation = new FrameQuaternion();
   private final Quaternion quaternion = new Quaternion();

   public YoFrameOrientationInMultipleFrames(String namePrefix, YoVariableRegistry registry, ReferenceFrame... referenceFrames)
   {
      super(namePrefix, null, registry);

      multipleFramesHelper = new YoMultipleFramesHelper(namePrefix, registry, referenceFrames);
   }

   @Override
   public void registerReferenceFrame(ReferenceFrame newReferenceFrame)
   {
      multipleFramesHelper.registerReferenceFrame(newReferenceFrame);
   }

   @Override
   public void changeFrame(ReferenceFrame desiredFrame)
   {
      getQuaternion(quaternion);
      ReferenceFrame currentReferenceFrame = multipleFramesHelper.switchCurrentReferenceFrame(desiredFrame);
      frameOrientation.setIncludingFrame(currentReferenceFrame, quaternion);
      frameOrientation.changeFrame(desiredFrame);
      quaternion.set(frameOrientation);
      set(quaternion);
   }

   @Override
   public ReferenceFrame switchCurrentReferenceFrame(ReferenceFrame newCurrentReferenceFrame)
   {
      ReferenceFrame previousReferenceFrame = multipleFramesHelper.switchCurrentReferenceFrame(newCurrentReferenceFrame);
      setYawPitchRoll(0.0, 0.0, 0.0);
      return previousReferenceFrame;
   }
   
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return multipleFramesHelper.getCurrentReferenceFrame();
   }
   
   @Override
   public boolean isReferenceFrameRegistered(ReferenceFrame referenceFrame)
   {
      return multipleFramesHelper.isReferenceFrameRegistered(referenceFrame);
   }

   @Override
   public int getNumberOfReferenceFramesRegistered()
   {
      return multipleFramesHelper.getNumberOfReferenceFramesRegistered();
   }

   @Override
   public void getRegisteredReferenceFrames(List<ReferenceFrame> referenceFramesToPack)
   {
      multipleFramesHelper.getRegisteredReferenceFrames(referenceFramesToPack);
   }

   @Override
   public String toString()
   {
      String ret = "";

      List<ReferenceFrame> referenceFrames = new ArrayList<ReferenceFrame>();
      multipleFramesHelper.getRegisteredReferenceFrames(referenceFrames);

      for (int i = 0; i < referenceFrames.size(); i++)
      {
         if (i > 0)
            ret += "\n";
         ret += toStringForASingleReferenceFrame(referenceFrames.get(i));
      }

      return ret;
   }
   
   public String toStringForASingleReferenceFrame(ReferenceFrame referenceFrame)
   {
      getFrameOrientationIncludingFrame(frameOrientation);
      frameOrientation.changeFrame(referenceFrame);
      return frameOrientation.toString();
   }

   @Override
   public void setToNaN(ReferenceFrame desiredReferenceFrame)
   {
      setToNaN();
      multipleFramesHelper.switchCurrentReferenceFrame(desiredReferenceFrame);
   }
}
