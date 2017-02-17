package us.ihmc.robotics.math.frames;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FrameTuple;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoFrameVectorInMultipleFrames extends YoFrameVector implements YoMultipleFramesHolder
{
   private final YoMultipleFramesHelper multipleFramesHelper;

   private final FrameVector frameVector = new FrameVector();
   private final Vector3D vector = new Vector3D();

   private final String namePrefix;
   private final YoVariableRegistry registry;

   public YoFrameVectorInMultipleFrames(String namePrefix, YoVariableRegistry registry, ReferenceFrame... referenceFrames)
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
      get(vector);
      ReferenceFrame currentReferenceFrame = multipleFramesHelper.switchCurrentReferenceFrame(desiredFrame);
      frameVector.setIncludingFrame(currentReferenceFrame, vector);
      frameVector.changeFrame(desiredFrame);
      frameVector.get(vector);
      set(vector);
   }

   public void setIncludingFrame(YoFrameTuple<?, ?> yoFrameTuple)
   {
      setIncludingFrame(yoFrameTuple.getFrameTuple());
   }

   public void setIncludingFrame(FrameTuple<?, ?> frameTuple)
   {
      multipleFramesHelper.switchCurrentReferenceFrame(frameTuple.getReferenceFrame());
      set(frameTuple);
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

   public final void subIncludingFrame(YoFrameTuple<?, ?> yoFrameTuple1, YoFrameTuple<?, ?> yoFrameTuple2)
   {
      yoFrameTuple1.checkReferenceFrameMatch(yoFrameTuple2);
      multipleFramesHelper.switchCurrentReferenceFrame(yoFrameTuple1.getReferenceFrame());
      sub(yoFrameTuple1, yoFrameTuple2);
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

   private YoFrameVector yoFrameVectorInWorld;
   public YoFrameVector buildUpdatedYoFrameVectorForVisualizationOnly()
   {
      if (yoFrameVectorInWorld == null)
      {
         final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
         if (!isReferenceFrameRegistered(worldFrame))
            registerReferenceFrame(worldFrame);

         yoFrameVectorInWorld = new YoFrameVector(namePrefix, worldFrame.getName(), worldFrame, registry);

         attachVariableChangedListener(new VariableChangedListener()
         {
            private final FrameVector localFrameVector = new FrameVector();
            private final YoFrameVector vector = yoFrameVectorInWorld;

            @Override
            public void variableChanged(YoVariable<?> v)
            {
               getFrameTupleIncludingFrame(localFrameVector);
               vector.setAndMatchFrame(localFrameVector);
            }
         });
      }
      return yoFrameVectorInWorld;
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
      getFrameTupleIncludingFrame(frameVector);
      frameVector.changeFrame(referenceFrame);
      return frameVector.toString();
   }

   @Override
   public void setToNaN(ReferenceFrame desiredReferenceFrame)
   {
      setToNaN();
      multipleFramesHelper.switchCurrentReferenceFrame(desiredReferenceFrame);
   }
}
