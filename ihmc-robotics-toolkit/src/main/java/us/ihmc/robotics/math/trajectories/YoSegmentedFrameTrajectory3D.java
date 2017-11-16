package us.ihmc.robotics.math.trajectories;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
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
public abstract class YoSegmentedFrameTrajectory3D //implements YoSegmentedFrameTrajectory3DInterface
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

   public void set(SegmentedFrameTrajectory3D trajToCopy)
   {
      if(getMaxNumberOfSegments() < trajToCopy.getNumberOfSegments())
         throw new RuntimeException("Insufficient segments to copy trajectory, needed: " + trajToCopy.getNumberOfSegments() + " available: " + getMaxNumberOfSegments());
      for(int i = 0; i < trajToCopy.getNumberOfSegments(); i++)
      {
         segments.get(i).set(trajToCopy.getSegment(i));
         numberOfSegments.increment();
      }
   }
   
   public void update(double timeInState)
   {
      setCurrentSegmentIndexFromStateTime(timeInState);
      timeInState = Math.min(timeInState, currentSegment.getFinalTime());
      currentSegment.compute(timeInState);
   }

   public void update(double timeInState, FramePoint3D desiredPositionToPack)
   {
      update(timeInState);
      currentSegment.getFramePosition(desiredPositionToPack);
   }

   public void update(double timeInState, FramePoint3D desiredPositionToPack, FrameVector3D desiredVelocityToPack)
   {
      update(timeInState, desiredPositionToPack);
      currentSegment.getFrameVelocity(desiredVelocityToPack);
   }

   public void update(double timeInState, FramePoint3D desiredPositonToPack, FrameVector3D desiredVelocityToPack, FrameVector3D desiredAccelerationToPack)
   {
      update(timeInState, desiredPositonToPack, desiredVelocityToPack);
      currentSegment.getFrameAcceleration(desiredAccelerationToPack);
   }

   public void getFramePosition(FramePoint3D desiredPositionToPack)
   {
      currentSegment.getFramePosition(desiredPositionToPack);
   }

   public void getFrameVelocity(FrameVector3D desiredVelocityToPack)
   {
      currentSegment.getFrameVelocity(desiredVelocityToPack);
   }

   public void getFrameAcceleration(FrameVector3D desiredAccelerationToPack)
   {
      currentSegment.getFrameAcceleration(desiredAccelerationToPack);
   }

   private void setCurrentSegmentIndexFromStateTime(double timeInState)
   {
      int segmentIndex = 0;
      if (MathTools.isGreaterThanOrEqualToWithPrecision(timeInState, segments.get(0).getInitialTime(), Epsilons.ONE_TEN_THOUSANDTH))
      {
         for (; segmentIndex < getNumberOfSegments() - 1; segmentIndex++)
            if (segments.get(segmentIndex).timeIntervalContains(timeInState, Epsilons.ONE_TEN_THOUSANDTH))
               break;
      }
      else
         throw new RuntimeException("Unable to find suitable segment at time " + timeInState + ", InitialTime: " + segments.get(0).getInitialTime());
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

   public int getCurrentSegmentIndex()
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
      for (; i < maxNumberOfSegments; i++)
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

   
   /**
    * Returns the coefficients for all the set trajectories
    */
   @Override
   public String toString()
   {
      String ret = "";
      ret += name;
      for (int i = 0; i < numberOfSegments.getIntegerValue(); i++)
         ret += "\nSegment " + i + ":\n" + segments.get(i).toString();
      return ret;
   }

   /**
    * Returns the coefficients for all the trajectories that can be set
    * @return
    */
   public String toString2()
   {
      String ret = "";
      ret += name;
      for (int i = 0; i < segments.size(); i++)
         ret += "\nSegment " + i + ":\n" + segments.get(i).toString();
      return ret;
   }
   
   /**
    * Returns the start and end points of each segment that has been set. Creates garbage. Use only for debugging 
    * @return
    */
   public String toString3()
   {
      String ret = "";
      ret += name;
      FramePoint3D tempFramePointForPrinting = new FramePoint3D();
      for (int i = 0; i < numberOfSegments.getIntegerValue(); i++)
      {
         ret += "\nSegment " + i + ":\n"; 
         segments.get(i).getStartPoint(tempFramePointForPrinting);
         ret += "Start Point: t = " + segments.get(i).getInitialTime() + ", " + tempFramePointForPrinting.toString();
         segments.get(i).getEndPoint(tempFramePointForPrinting);
         ret += "End Point: t = " + segments.get(i).getFinalTime() + ", " + tempFramePointForPrinting.toString();
      }
      return ret;
   }
   
}
