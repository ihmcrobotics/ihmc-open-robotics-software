package us.ihmc.robotics.geometry.yoFrameObjects;

import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createName;

import java.util.List;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.AbstractFrameObject;
import us.ihmc.robotics.geometry.AbstractReferenceFrameHolder;
import us.ihmc.robotics.math.frames.YoMultipleFramesHelper;
import us.ihmc.robotics.math.frames.YoMultipleFramesHolder;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public abstract class YoFrameWaypoint<Y extends YoFrameWaypoint<Y, F, S>, F extends AbstractFrameObject<F, S>, S extends GeometryObject<S>>
      extends AbstractReferenceFrameHolder implements YoMultipleFramesHolder, GeometryObject<Y>
{
   private final String namePrefix;
   private final String nameSuffix;

   private final YoMultipleFramesHelper multipleFramesHelper;
   protected final F frameWaypoint;

   protected YoFrameWaypoint(F frameWaypoint, String namePrefix, String nameSuffix, YoVariableRegistry registry, ReferenceFrame... referenceFrames)
   {
      this.namePrefix = namePrefix;
      this.nameSuffix = nameSuffix;
      multipleFramesHelper = new YoMultipleFramesHelper(createName(namePrefix, nameSuffix, ""), registry, referenceFrames);
      this.frameWaypoint = frameWaypoint;
   }

   public void set(S simpleWaypoint)
   {
      frameWaypoint.setIncludingFrame(getReferenceFrame(), simpleWaypoint);
      getYoValuesFromFrameWaypoint();
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, S simpleWaypoint)
   {
      multipleFramesHelper.switchCurrentReferenceFrame(referenceFrame);
      frameWaypoint.setIncludingFrame(getReferenceFrame(), simpleWaypoint);
      getYoValuesFromFrameWaypoint();
   }

   public void set(F frameWaypoint)
   {
      checkReferenceFrameMatch(frameWaypoint);
      this.frameWaypoint.setIncludingFrame(frameWaypoint);
      getYoValuesFromFrameWaypoint();
   }

   public void setIncludingFrame(F frameWaypoint)
   {
      multipleFramesHelper.switchCurrentReferenceFrame(frameWaypoint.getReferenceFrame());
      this.frameWaypoint.setIncludingFrame(frameWaypoint);
      getYoValuesFromFrameWaypoint();
   }

   @Override
   public void set(Y other)
   {
      checkReferenceFrameMatch(other);
      other.getIncludingFrame(frameWaypoint);
      getYoValuesFromFrameWaypoint();
   }

   public void setIncludingFrame(Y other)
   {
      multipleFramesHelper.switchCurrentReferenceFrame(other.getReferenceFrame());
      other.getIncludingFrame(frameWaypoint);
      getYoValuesFromFrameWaypoint();
   }

   @Override
   public final void registerReferenceFrame(ReferenceFrame newReferenceFrame)
   {
      multipleFramesHelper.registerReferenceFrame(newReferenceFrame);
   }

   @Override
   public final void changeFrame(ReferenceFrame referenceFrame)
   {
      putYoValuesIntoFrameWaypoint();
      // this is in the correct frame already
      if (referenceFrame == getReferenceFrame())
      {
         return;
      }

      getReferenceFrame().verifySameRoots(referenceFrame);
      RigidBodyTransform referenceTf, desiredTf;

      if ((referenceTf = getReferenceFrame().getTransformToRoot()) != null)
      {
         frameWaypoint.applyTransform(referenceTf);
      }

      if ((desiredTf = referenceFrame.getInverseTransformToRoot()) != null)
      {
         frameWaypoint.applyTransform(desiredTf);
      }

      getYoValuesFromFrameWaypoint();
      multipleFramesHelper.switchCurrentReferenceFrame(referenceFrame);
   }

   @Override
   public final void applyTransform(Transform transform)
   {
      putYoValuesIntoFrameWaypoint();
      frameWaypoint.applyTransform(transform);
      getYoValuesFromFrameWaypoint();
   }

   @Override
   public void setToZero()
   {
      frameWaypoint.setToZero(getReferenceFrame());
      getYoValuesFromFrameWaypoint();
   }

   public void setToZero(ReferenceFrame referenceFrame)
   {
      multipleFramesHelper.switchCurrentReferenceFrame(referenceFrame);
      frameWaypoint.setToZero(referenceFrame);
      getYoValuesFromFrameWaypoint();
   }

   @Override
   public void setToNaN()
   {
      frameWaypoint.setToNaN(getReferenceFrame());
      getYoValuesFromFrameWaypoint();
   }

   @Override
   public void setToNaN(ReferenceFrame referenceFrame)
   {
      multipleFramesHelper.switchCurrentReferenceFrame(referenceFrame);
      frameWaypoint.setToNaN(referenceFrame);
      getYoValuesFromFrameWaypoint();
   }

   @Override
   public boolean containsNaN()
   {
      putYoValuesIntoFrameWaypoint();
      return frameWaypoint.containsNaN();
   }

   @Override
   public final int getNumberOfReferenceFramesRegistered()
   {
      return multipleFramesHelper.getNumberOfReferenceFramesRegistered();
   }

   @Override
   public final void getRegisteredReferenceFrames(List<ReferenceFrame> referenceFramesToPack)
   {
      multipleFramesHelper.getRegisteredReferenceFrames(referenceFramesToPack);
   }

   @Override
   public final boolean isReferenceFrameRegistered(ReferenceFrame referenceFrame)
   {
      return multipleFramesHelper.isReferenceFrameRegistered(referenceFrame);
   }

   @Override
   public final ReferenceFrame switchCurrentReferenceFrame(ReferenceFrame newCurrentReferenceFrame)
   {
      ReferenceFrame previousReferenceFrame = multipleFramesHelper.switchCurrentReferenceFrame(newCurrentReferenceFrame);
      setToZero();
      return previousReferenceFrame;
   }

   public void get(S simpleWaypoint)
   {
      putYoValuesIntoFrameWaypoint();
      frameWaypoint.get(simpleWaypoint);
   }

   public void get(F frameWaypoint)
   {
      checkReferenceFrameMatch(frameWaypoint);
      putYoValuesIntoFrameWaypoint();
      frameWaypoint.set(this.frameWaypoint);
   }

   public void getIncludingFrame(F frameWaypoint)
   {
      putYoValuesIntoFrameWaypoint();
      frameWaypoint.setIncludingFrame(this.frameWaypoint);
   }

   protected abstract void putYoValuesIntoFrameWaypoint();

   protected abstract void getYoValuesFromFrameWaypoint();

   @Override
   public final ReferenceFrame getReferenceFrame()
   {
      return multipleFramesHelper.getCurrentReferenceFrame();
   }

   public final String getNamePrefix()
   {
      return namePrefix;
   }

   public final String getNameSuffix()
   {
      return nameSuffix;
   }

   @Override
   public boolean epsilonEquals(Y other, double epsilon)
   {
      putYoValuesIntoFrameWaypoint();
      other.putYoValuesIntoFrameWaypoint();
      return frameWaypoint.epsilonEquals(other.frameWaypoint, epsilon);
   }
}
