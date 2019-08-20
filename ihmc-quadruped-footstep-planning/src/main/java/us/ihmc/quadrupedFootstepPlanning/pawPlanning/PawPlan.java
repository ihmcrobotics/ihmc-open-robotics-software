package us.ihmc.quadrupedFootstepPlanning.pawPlanning;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedOrientedStep;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeInterval;

import java.util.ArrayList;
import java.util.Collections;

public class PawPlan
{
   private final ArrayList<QuadrupedTimedOrientedStep> pawSteps = new ArrayList<>();
   private final FramePose3D lowLevelPlanGoal = new FramePose3D();

   public PawPlan()
   {
   }

   public void setLowLevelPlanGoal(FramePose3DReadOnly lowLevelPlanGoal)
   {
      this.lowLevelPlanGoal.setIncludingFrame(lowLevelPlanGoal);
   }

   public int getNumberOfSteps()
   {
      return pawSteps.size();
   }

   public QuadrupedTimedStep getPawStep(int pawIndex)
   {
      return pawSteps.get(pawIndex);
   }

   public void addPawStep(QuadrupedTimedOrientedStep pawStep)
   {
      pawSteps.add(pawStep);
   }

   public void addPawPlan(PawPlan other)
   {
      pawSteps.addAll(other.pawSteps);
   }

   public QuadrupedTimedOrientedStep addPawStep(RobotQuadrant robotQuadrant, FramePoint3D soleFramePoint, double groundClearance, TimeInterval timeInterval)
   {
      return addPawStep(robotQuadrant, soleFramePoint, timeInterval.getStartTime(), groundClearance, timeInterval.getEndTime());
   }

   public QuadrupedTimedOrientedStep addPawStep(RobotQuadrant robotQuadrant, FramePoint3D soleFramePoint, double groundClearance, double startTime, double endTime)
   {
      QuadrupedTimedOrientedStep paw = new QuadrupedTimedOrientedStep();
      paw.setRobotQuadrant(robotQuadrant);
      paw.setGoalPosition(soleFramePoint);
      paw.getTimeInterval().setInterval(startTime, endTime);
      paw.setGroundClearance(groundClearance);
      pawSteps.add(paw);
      return paw;
   }

   public void reverse()
   {
      Collections.reverse(pawSteps);
   }

   public void clear()
   {
      lowLevelPlanGoal.setToNaN();
      pawSteps.clear();
   }

   public void remove(int pawIndex)
   {
      pawSteps.remove(pawIndex);
   }

   public FramePose3DReadOnly getLowLevelPlanGoal()
   {
      lowLevelPlanGoal.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      return lowLevelPlanGoal;
   }
}
