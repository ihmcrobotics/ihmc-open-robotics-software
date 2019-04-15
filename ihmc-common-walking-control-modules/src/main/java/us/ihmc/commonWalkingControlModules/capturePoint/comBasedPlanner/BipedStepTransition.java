package us.ihmc.commonWalkingControlModules.capturePoint.comBasedPlanner;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.robotics.time.TimeInterval;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.List;

/**
 * This is used to define contact transitions for bipeds. This essentially converts {@link TimeInterval} to step transition events, including
 * touchdowns and lift offs. This is used then to convert to contact sequences.
 */
public class BipedStepTransition
{
   public static final double sameTimeEpsilon = 1e-3;

   private final List<RobotSide> transitionSides = new ArrayList<>();
   private final List<BipedStepTransitionType> transitionTypes = new ArrayList<>();
   private final SideDependentList<FramePose3D> transitionPoses = new SideDependentList<>();
   private double transitionTime = Double.MAX_VALUE;

   public BipedStepTransition()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         FramePose3D transitionPose = new FramePose3D();
         transitionPose.setToNaN();
         transitionPoses.put(robotSide, transitionPose);
      }
   }

   public void reset()
   {
      transitionTime = Double.MAX_VALUE;
      transitionSides.clear();
      transitionTypes.clear();
      for (RobotSide robotSide : RobotSide.values)
      {
         transitionPoses.get(robotSide).setToNaN();
      }
   }

   public void setTransitionTime(double time)
   {
      this.transitionTime = time;
   }

   public void addTransition(BipedStepTransitionType transitionType, RobotSide transitionSide, FramePose3DReadOnly transitionPose)
   {
      transitionTypes.add(transitionType);
      transitionSides.add(transitionSide);
      transitionPoses.get(transitionSide).set(transitionPose);
   }

   public void addTransition(BipedStepTransition other)
   {
      if (!MathTools.epsilonEquals(transitionTime, other.transitionTime, sameTimeEpsilon))
         throw new IllegalArgumentException("These transitions occur at different times!");

      for (int i = 0; i < other.transitionSides.size(); i++)
      {
         RobotSide addingSide = other.transitionSides.get(i);
         transitionSides.add(addingSide);
         transitionTypes.add(other.transitionTypes.get(i));
         transitionPoses.get(addingSide).set(other.transitionPoses.get(addingSide));
      }
   }

   public double getTransitionTime()
   {
      return transitionTime;
   }

   public int getNumberOfFeetInTransition()
   {
      return transitionSides.size();
   }

   public BipedStepTransitionType getTransitionType(int transitionNumber)
   {
      return transitionTypes.get(transitionNumber);
   }

   public RobotSide getTransitionSide(int transitionNumber)
   {
      return transitionSides.get(transitionNumber);
   }

   public FramePose3DReadOnly transitionPose(RobotSide transitionSide)
   {
      return transitionPoses.get(transitionSide);
   }
}
