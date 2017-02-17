package us.ihmc.robotics.math.frames;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameTuple2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoFramePoint2dInMultipleFrames extends YoFramePoint2d implements YoMultipleFramesHolder
{
   private final YoMultipleFramesHelper multipleFramesHelper;

   private final FramePoint2d framePoint2d = new FramePoint2d();
   private final Point2D point2d = new Point2D();

   private final String namePrefix;
   private final YoVariableRegistry registry;

   public YoFramePoint2dInMultipleFrames(String namePrefix, YoVariableRegistry registry, ReferenceFrame... referenceFrames)
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
      get(point2d);
      ReferenceFrame currentReferenceFrame = multipleFramesHelper.switchCurrentReferenceFrame(desiredFrame);
      framePoint2d.setIncludingFrame(currentReferenceFrame, point2d);
      framePoint2d.changeFrame(desiredFrame);
      framePoint2d.get(point2d);
      set(point2d);
   }

   public void setIncludingFrame(FrameTuple2d<?, ?> frameTuple2d)
   {
      multipleFramesHelper.switchCurrentReferenceFrame(frameTuple2d.getReferenceFrame());
      set(frameTuple2d);
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

   private YoFramePoint2d yoFramePointInWorld;
   public YoFramePoint2d buildUpdatedYoFramePointForVisualizationOnly()
   {
      if (yoFramePointInWorld == null)
      {
         final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
         if (!isReferenceFrameRegistered(worldFrame))
            registerReferenceFrame(worldFrame);

         yoFramePointInWorld = new YoFramePoint2d(namePrefix, worldFrame.getName(), worldFrame, registry);

         attachVariableChangedListener(new VariableChangedListener()
         {
            private final FramePoint2d localFramePoint = new FramePoint2d();
            private final YoFramePoint2d point = yoFramePointInWorld;

            @Override
            public void variableChanged(YoVariable<?> v)
            {
               getFrameTuple2dIncludingFrame(localFramePoint);
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
         getFrameTuple2dIncludingFrame(framePoint2d);
         framePoint2d.changeFrame(referenceFrames.get(i));
         if (i > 0)
            ret += "\n";
         ret += framePoint2d.toString();
      }

      return ret;
   }

   @Override
   public void setToNaN(ReferenceFrame desiredReferenceFrame)
   {
      setToNaN();
      multipleFramesHelper.switchCurrentReferenceFrame(desiredReferenceFrame);
   }
}
