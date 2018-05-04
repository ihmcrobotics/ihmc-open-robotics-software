package us.ihmc.robotics.math.frames;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoFrameVectorInMultipleFrames extends YoFrameVector3D implements YoMultipleFramesHolder
{
   private final YoMultipleFramesHelper multipleFramesHelper;

   private final FrameVector3D frameVector = new FrameVector3D();
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
      vector.set(this);
      ReferenceFrame currentReferenceFrame = multipleFramesHelper.switchCurrentReferenceFrame(desiredFrame);
      frameVector.setIncludingFrame(currentReferenceFrame, vector);
      frameVector.changeFrame(desiredFrame);
      vector.set(frameVector);
      set(vector);
   }

   public void setIncludingFrame(FrameTuple3DReadOnly frameTuple)
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

   private YoFrameVector3D yoFrameVectorInWorld;
   public YoFrameVector3D buildUpdatedYoFrameVectorForVisualizationOnly()
   {
      if (yoFrameVectorInWorld == null)
      {
         final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
         if (!isReferenceFrameRegistered(worldFrame))
            registerReferenceFrame(worldFrame);

         yoFrameVectorInWorld = new YoFrameVector3D(namePrefix, worldFrame.getName(), worldFrame, registry);

         attachVariableChangedListener(new VariableChangedListener()
         {
            private final FrameVector3D localFrameVector = new FrameVector3D();
            private final YoFrameVector3D vector = yoFrameVectorInWorld;

            @Override
            public void notifyOfVariableChange(YoVariable<?> v)
            {
               localFrameVector.setIncludingFrame(YoFrameVectorInMultipleFrames.this);
               vector.setMatchingFrame(localFrameVector);
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
      frameVector.setIncludingFrame(this);
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
