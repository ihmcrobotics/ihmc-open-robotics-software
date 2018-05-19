package us.ihmc.robotics.math.trajectories;

import java.util.List;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.lists.GenericTypeBuilder;
import us.ihmc.robotics.lists.RecyclingArrayList;

/**
 * <p>Provides a basic frame work to create and access a list of {@link FrameTrajectory3D}. 
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
public class SegmentedFrameTrajectory3D implements SegmentedFrameTrajectory3DInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   protected final int maxNumberOfSegments;
   protected final int maxNumberOfCoefficients;

   protected final RecyclingArrayList<FrameTrajectory3D> segments;

   protected int currentSegmentIndex;

   protected FrameTrajectory3D currentSegment;
   protected double[] nodeTime;

   public SegmentedFrameTrajectory3D(int maxNumberOfSegments, int maxNumberOfCoefficients)
   {
      this.maxNumberOfSegments = maxNumberOfSegments;
      this.maxNumberOfCoefficients = maxNumberOfCoefficients;
      currentSegmentIndex = -1;
      segments = new RecyclingArrayList<>(maxNumberOfSegments, new FrameTrajectory3DBuilder());
      nodeTime = new double[maxNumberOfSegments + 1];
   }

   public void reset()
   {
      segments.clear();
      currentSegmentIndex = -1;
      currentSegment = null;
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

   private void setCurrentSegmentIndexFromStateTime(double timeInState)
   {
      int segmentIndex = 0;
      if (getNumberOfSegments() == 0) // standing
         return;

      if (MathTools.isGreaterThanOrEqualToWithPrecision(timeInState, segments.get(0).getInitialTime(), Epsilons.ONE_TEN_THOUSANDTH))
      {
         for (; segmentIndex < getNumberOfSegments() - 1; segmentIndex++)
            if (segments.get(segmentIndex).timeIntervalContains(timeInState, Epsilons.ONE_TEN_THOUSANDTH))
               break;
      }
      else
         throw new RuntimeException("Unable to find suitable segment at time " + timeInState + ", InitialTime: " + segments.get(0).getInitialTime());
      currentSegment = segments.get(segmentIndex);
      currentSegmentIndex = segmentIndex;
   }

   public List<FrameTrajectory3D> getSegments()
   {
      return segments;
   }

   public int getNumberOfSegments()
   {
      return segments.size();
   }

   public int getCurrentSegmentIndex()
   {
      return currentSegmentIndex;
   }

   public int getCurrentSegmentIndex(double timeInState)
   {
      setCurrentSegmentIndexFromStateTime(timeInState);
      return currentSegmentIndex;
   }

   public FrameTrajectory3D getCurrentSegment(double timeInState)
   {
      setCurrentSegmentIndexFromStateTime(timeInState);
      return currentSegment;
   }

   public void setNext(FrameTrajectory3D other)
   {
      FrameTrajectory3D segment = segments.add();
      segment.set(other);
   }

   public void setAll(SegmentedFrameTrajectory3D other)
   {
      for (int segmentIndex = 0; segmentIndex < other.getNumberOfSegments(); segmentIndex++)
      {
         FrameTrajectory3D segment = segments.add();
         segment.set(other.getSegment(segmentIndex));
      }
   }

   public double getFinalTime()
   {
      return segments.getLast().getFinalTime();
   }

   public FrameTrajectory3D add()
   {
      return segments.add();
   }

   public FrameTrajectory3D getSegment(int segmentIndex)
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

   /*
   public void setNumberOfSegments(int numberOfSegments)
   {
      this.numberOfSegments = numberOfSegments;
   }

*/
   
   /**
    * Returns the coefficients for all the set trajectories
    */
   @Override
   public String toString()
   {
      String ret = "";
      for (int i = 0; i < segments.size(); i++)
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
      FramePoint3D tempFramePointForPrinting = new FramePoint3D();
      for (int i = 0; i < segments.size(); i++)
      {
         ret += "\nSegment " + i + ":\n"; 
         segments.get(i).getStartPoint(tempFramePointForPrinting);
         ret += "Start Point: t = " + segments.get(i).getInitialTime() + ", " + tempFramePointForPrinting.toString();
         segments.get(i).getEndPoint(tempFramePointForPrinting);
         ret += "End Point: t = " + segments.get(i).getFinalTime() + ", " + tempFramePointForPrinting.toString();
      }
      return ret;
   }

   private class FrameTrajectory3DBuilder extends GenericTypeBuilder<FrameTrajectory3D>
   {
      @Override
      public FrameTrajectory3D newInstance()
      {
         FrameTrajectory3D frameTrajectory3D = new FrameTrajectory3D(maxNumberOfCoefficients, worldFrame);
         frameTrajectory3D.reset();
         return frameTrajectory3D;
      }
   }
}
