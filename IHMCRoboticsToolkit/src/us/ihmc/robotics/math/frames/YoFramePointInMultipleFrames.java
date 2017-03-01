package us.ihmc.robotics.math.frames;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameTuple;
import us.ihmc.robotics.geometry.FrameTuple2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoFramePointInMultipleFrames extends YoFramePoint implements YoMultipleFramesHolder
{
   private final YoMultipleFramesHelper multipleFramesHelper;

   private final FramePoint framePoint = new FramePoint();
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
   public void registerReferenceFrame(ReferenceFrame newReferenceFrame)
   {
      multipleFramesHelper.registerReferenceFrame(newReferenceFrame);
   }

   @Override
   public void changeFrame(ReferenceFrame desiredFrame)
   {
      get(point);
      ReferenceFrame currentReferenceFrame = multipleFramesHelper.switchCurrentReferenceFrame(desiredFrame);
      framePoint.setIncludingFrame(currentReferenceFrame, point);
      framePoint.changeFrame(desiredFrame);
      framePoint.get(point);
      set(point);
   }

   public void setIncludingFrame(FrameTuple<?, ?> frameTuple)
   {
      multipleFramesHelper.switchCurrentReferenceFrame(frameTuple.getReferenceFrame());
      set(frameTuple);
   }

   public void setIncludingFrame(YoFrameTuple<?, ?> yoFrameTuple)
   {
      multipleFramesHelper.switchCurrentReferenceFrame(yoFrameTuple.getReferenceFrame());
      set(yoFrameTuple);
   }

   public void setXYIncludingFrame(FrameTuple2d<?, ?> frameTuple2d)
   {
      multipleFramesHelper.switchCurrentReferenceFrame(frameTuple2d.getReferenceFrame());
      setXY(frameTuple2d);
   }

   public void setXYIncludingFrame(YoFrameTuple2d<?, ?> yoFrameTuple2d)
   {
      multipleFramesHelper.switchCurrentReferenceFrame(yoFrameTuple2d.getReferenceFrame());
      setXY(yoFrameTuple2d);
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
            private final FramePoint localFramePoint = new FramePoint();
            private final YoFramePoint point = yoFramePointInWorld;

            @Override
            public void variableChanged(YoVariable<?> v)
            {
               getFrameTupleIncludingFrame(localFramePoint);
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
      getFrameTupleIncludingFrame(framePoint);
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
