package us.ihmc.robotics.math.frames;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoFramePointInMultipleFrames extends YoFramePoint implements YoMultipleFramesHolder
{
   private final YoMultipleFramesHelper multipleFramesHelper;

   private final FramePoint3D framePoint = new FramePoint3D();
   private final Point3D point = new Point3D();

   private final String namePrefix;
   private final YoVariableRegistry registry;

   public YoFramePointInMultipleFrames(String namePrefix, YoVariableRegistry registry, ReferenceFrame... referenceFrames)
   {
      super(namePrefix, null, registry);

      this.namePrefix = namePrefix;
      this.registry = registry;

      multipleFramesHelper = new YoMultipleFramesHelper(namePrefix, registry, referenceFrames);
   }

   @Override
   public boolean containsNaN()
   {
      return super.containsNaN();
   }

   @Override
   public void registerReferenceFrame(ReferenceFrame newReferenceFrame)
   {
      multipleFramesHelper.registerReferenceFrame(newReferenceFrame);
   }

   @Override
   public void changeFrame(ReferenceFrame desiredFrame)
   {
      point.set(this);
      ReferenceFrame currentReferenceFrame = multipleFramesHelper.switchCurrentReferenceFrame(desiredFrame);
      framePoint.setIncludingFrame(currentReferenceFrame, point);
      framePoint.changeFrame(desiredFrame);
      point.set(framePoint);
      set(point);
   }

   public void setIncludingFrame(FrameTuple3DReadOnly frameTuple)
   {
      multipleFramesHelper.switchCurrentReferenceFrame(frameTuple.getReferenceFrame());
      set(frameTuple);
   }

   public void setIncludingFrame(FrameTuple2DReadOnly frameTuple2d, double z)
   {
      multipleFramesHelper.switchCurrentReferenceFrame(frameTuple2d.getReferenceFrame());
      set(frameTuple2d, z);
   }

   /**
    * Change the current reference frame and set to zero the coordinates (different from changeFrame() ).
    * @return the previous current reference frame.
    */
   @Override
   public ReferenceFrame switchCurrentReferenceFrame(ReferenceFrame referenceFrame)
   {
      ReferenceFrame previousReferenceFrame = multipleFramesHelper.switchCurrentReferenceFrame(referenceFrame);
      setToZero();
      return previousReferenceFrame;
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
   public ReferenceFrame getReferenceFrame()
   {
      return multipleFramesHelper.getCurrentReferenceFrame();
   }

   @Override
   public boolean isReferenceFrameRegistered(ReferenceFrame referenceFrame)
   {
      return multipleFramesHelper.isReferenceFrameRegistered(referenceFrame);
   }

   private YoFramePoint yoFramePointInWorld;
   public YoFramePoint buildUpdatedYoFramePointForVisualizationOnly()
   {
      if (yoFramePointInWorld == null)
      {
         final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
         if (!isReferenceFrameRegistered(worldFrame))
            registerReferenceFrame(worldFrame);

         yoFramePointInWorld = new YoFramePoint(namePrefix, worldFrame.getName(), worldFrame, registry);

         attachVariableChangedListener(new VariableChangedListener()
         {
            private final FramePoint3D localFramePoint = new FramePoint3D();
            private final YoFramePoint point = yoFramePointInWorld;

            @Override
            public void notifyOfVariableChange(YoVariable<?> v)
            {
               localFramePoint.setIncludingFrame(YoFramePointInMultipleFrames.this);
               point.setAndMatchFrame(localFramePoint);
            }
         });
      }
      return yoFramePointInWorld;
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
      framePoint.setIncludingFrame(this);
      framePoint.changeFrame(referenceFrame);
      return framePoint.toString();
   }

   @Override
   public void setToNaN(ReferenceFrame desiredReferenceFrame)
   {
      setToNaN();
      multipleFramesHelper.switchCurrentReferenceFrame(desiredReferenceFrame);
   }
}
