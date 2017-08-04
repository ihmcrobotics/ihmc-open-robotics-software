package us.ihmc.robotics.math.trajectories;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.robotics.math.trajectories.YoFrameTrajectory3D;
import us.ihmc.commons.Epsilons;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * <p>Provides a basic frame work to create and access a list of {@link YoFrameTrajectory3D}. 
 * No methods are provided to set the trajectory list and must be done by extending the class as required</p>
 * The following members are created to ease said task 
 * <list> 
 * <li> segments: List object containing trajectory segments 
 * <li> maxNumberOfSegments: Indicates the maximum number of segments that have been initialized. Further segments can be added but must be initialized first
 * <li> maxNumberOfCoefficients: Maximum number of coefficients that the segments can hold
 * <li> numberOfSegments: YoInteger containing the number of segments used. Is set to zero by default
 * <li> currentSegmentIndex: YoInteger indicating the segment index that is used for computation 
 * <li> currentSegment: Reference to the current segment used for computation
 */
public abstract class YoSegmentedFrameTrajectory3D implements SegmentedFrameTrajectory3DInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   protected final String name;

   protected final int maxNumberOfSegments;
   protected final int maxNumberOfCoefficients;
   protected final List<YoFrameTrajectory3D> segments = new ArrayList<>();

   protected final YoInteger numberOfSegments;
   protected final YoInteger currentSegmentIndex;

   protected YoFrameTrajectory3D currentSegment;
   protected double[] nodeTime;

   public YoSegmentedFrameTrajectory3D(String name, int maxNumberOfSegments, int maxNumberOfCoefficients, YoVariableRegistry registry)
   {
      this.name = name;
      this.maxNumberOfSegments = maxNumberOfSegments;
      this.maxNumberOfCoefficients = maxNumberOfCoefficients;
      numberOfSegments = new YoInteger(name + "NumberOfSegments", registry);
      currentSegmentIndex = new YoInteger(name + "CurrentSegmentIndex", registry);
      currentSegmentIndex.set(-1);
      for (int i = 0; i < maxNumberOfSegments; i++)
      {
         YoFrameTrajectory3D segmentTrajectory = new YoFrameTrajectory3D(name + "Segment" + i, maxNumberOfCoefficients, worldFrame, registry);
         segmentTrajectory.reset();
         segments.add(segmentTrajectory);
      }
      nodeTime = new double[maxNumberOfSegments + 1];
   }

   public void reset()
   {
      for (int i = 0; i < segments.size(); i++)
         segments.get(i).reset();
      currentSegmentIndex.set(-1);
      currentSegment = null;
      numberOfSegments.set(0);
   }

   public void update(double timeInState)
   {
      setCurrentSegmentIndexFromStateTime(timeInState);
      currentSegment.compute(timeInState);
   }

   public void update(double timeInState, FramePoint desiredPositionToPack)
   {
      update(timeInState);
      currentSegment.getFramePosition(desiredPositionToPack);
   }

   public void update(double timeInState, FramePoint desiredPositionToPack, FrameVector desiredVelocityToPack)
   {
      update(timeInState, desiredPositionToPack);
      currentSegment.getFrameVelocity(desiredVelocityToPack);
   }

   public void update(double timeInState, FramePoint desiredPositonToPack, FrameVector desiredVelocityToPack, FrameVector desiredAccelerationToPack)
   {
      update(timeInState, desiredPositonToPack, desiredVelocityToPack);
      currentSegment.getFrameAcceleration(desiredAccelerationToPack);
   }

   private void setCurrentSegmentIndexFromStateTime(double timeInState)
   {
      int segmentIndex = 0;
      for (; segmentIndex < segments.size(); segmentIndex++)
         if (segments.get(segmentIndex).timeIntervalContains(timeInState, Epsilons.ONE_THOUSANDTH))
            break;
      if (segmentIndex == segments.size())
         throw new RuntimeException(name + ": Unable to locate suitable segment for given time:" + timeInState);
      currentSegment = segments.get(segmentIndex);
      currentSegmentIndex.set(segmentIndex);
   }

   public List<YoFrameTrajectory3D> getSegments()
   {
      return segments;
   }

   public int getNumberOfSegments()
   {
      return numberOfSegments.getIntegerValue();
   }

   protected int getCurrentSegmentIndex()
   {
      return currentSegmentIndex.getIntegerValue();
   }

   public int getCurrentSegmentIndex(double timeInState)
   {
      setCurrentSegmentIndexFromStateTime(timeInState);
      return currentSegmentIndex.getIntegerValue();
   }

   public YoFrameTrajectory3D getCurrentSegment(double timeInState)
   {
      setCurrentSegmentIndexFromStateTime(timeInState);
      return currentSegment;
   }

   public YoFrameTrajectory3D getSegment(int segmentIndex)
   {
      return segments.get(segmentIndex);
   }

   public double[] getNodeTimes()
   {
      nodeTime[0] = segments.get(0).getInitialTime();
      int i;
      for (i = 0; i < getNumberOfSegments(); i++)
         nodeTime[i + 1] = segments.get(i).getFinalTime();
      for (; i < maxNumberOfSegments + 1; i++)
         nodeTime[i + 1] = Double.NaN;
      return nodeTime;
   }
   
   public int getMaxNumberOfSegments()
   {
      return maxNumberOfSegments;
   }
   
   public void setNumberOfSegments(int numberOfSegments)
   {
      this.numberOfSegments.set(numberOfSegments);
   }
   
   @Override
   public String toString()
   {
      String ret = "";
      ret += name;
      for(int i = 0 ; i < numberOfSegments.getIntegerValue(); i++)
         ret += "\nSegment " + i + ":\n" + segments.get(i).toString();
      return ret;
   }
}
