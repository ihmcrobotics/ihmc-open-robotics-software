package us.ihmc.aware.planning;

import us.ihmc.aware.util.ContactState;
import us.ihmc.aware.util.PreallocatedQueue;
import us.ihmc.aware.util.QuadrupedTimedStep;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;

import java.util.List;

public class QuadrupedTimedStepDcmPlanner
{
   private final ThreeDoFMinimumJerkTrajectory transientDcmTrajectory;
   private final PiecewiseReverseDcmTrajectory timedStepDcmTrajectory;
   private final QuadrupedTimedStepCopPlanner copPlanner;
   private final FramePoint finalDcmPosition;
   private final FramePoint currentDcmPosition;
   private final FrameVector currentDcmVelocity;
   private final QuadrupedTimedStep[] stepArray;
   private double transferTime;

   public QuadrupedTimedStepDcmPlanner(int maxSteps, double gravity, double comHeight)
   {
      this.transientDcmTrajectory = new ThreeDoFMinimumJerkTrajectory();
      this.timedStepDcmTrajectory = new PiecewiseReverseDcmTrajectory(maxSteps, gravity, comHeight);
      this.copPlanner = new QuadrupedTimedStepCopPlanner(2 * maxSteps);
      this.finalDcmPosition = new FramePoint();
      this.currentDcmPosition = new FramePoint();
      this.currentDcmVelocity = new FrameVector();
      this.stepArray = new QuadrupedTimedStep[maxSteps];
      this.transferTime = 0.5;
   }

   public void setTransferTime(double transferTime)
   {
      this.transferTime = transferTime;
   }

   public void initializeTrajectory(PreallocatedQueue<QuadrupedTimedStep> stepQueue, QuadrantDependentList<FramePoint> initialSolePosition, QuadrantDependentList<ContactState> initialContactState, double initialComHeight,
         FramePoint initialDcmPosition)
   {
      int numberOfTransitions = copPlanner.compute(stepQueue, initialSolePosition, initialContactState);
      initializeTrajectory(numberOfTransitions, copPlanner.getTimeAtTransitions(), copPlanner.getCopAtTransitions(), initialComHeight, initialDcmPosition);
   }

   public void initializeTrajectory(List<QuadrupedTimedStep> stepQueue, QuadrantDependentList<FramePoint> initialSolePosition, QuadrantDependentList<ContactState> initialContactState, double initialComHeight,
         FramePoint initialDcmPosition)
   {
      int numberOfTransitions = copPlanner.compute(stepQueue, initialSolePosition, initialContactState);
      initializeTrajectory(numberOfTransitions, copPlanner.getTimeAtTransitions(), copPlanner.getCopAtTransitions(), initialComHeight, initialDcmPosition);
   }

   public void initializeTrajectory(int numberOfSteps, QuadrupedTimedStep[] stepArray, QuadrantDependentList<FramePoint> initialSolePosition, QuadrantDependentList<ContactState> initialContactState, double initialComHeight,
         FramePoint initialDcmPosition)
   {
      int numberOfTransitions = copPlanner.compute(numberOfSteps, stepArray, initialSolePosition, initialContactState);
      initializeTrajectory(numberOfTransitions, copPlanner.getTimeAtTransitions(), copPlanner.getCopAtTransitions(), initialComHeight, initialDcmPosition);
   }

   public void initializeTrajectory(int numberOfTransitions, double[] timeAtTransitions, FramePoint[] copAtTransitions, double initialComHeight, FramePoint initialDcmPosition)
   {
      finalDcmPosition.setIncludingFrame(copAtTransitions[numberOfTransitions - 1]);
      finalDcmPosition.changeFrame(ReferenceFrame.getWorldFrame());
      finalDcmPosition.add(0, 0, initialComHeight);
      timedStepDcmTrajectory.setComHeight(initialComHeight);
      timedStepDcmTrajectory.initializeTrajectory(numberOfTransitions, timeAtTransitions, copAtTransitions, timeAtTransitions[numberOfTransitions - 1], finalDcmPosition);
      timedStepDcmTrajectory.computeTrajectory(timeAtTransitions[0]);
      timedStepDcmTrajectory.getPosition(currentDcmPosition);
      transientDcmTrajectory.initializeTrajectory(initialDcmPosition, currentDcmPosition, timeAtTransitions[0] - transferTime, timeAtTransitions[0]);
   }

   public void initializeTrajectory(int numberOfTransitions, double[] timeAtTransitions, FramePoint[] copAtTransitions, double initialComHeight, FramePoint initialDcmPosition, FramePoint finalDcmPosition)
   {
      timedStepDcmTrajectory.setComHeight(initialComHeight);
      timedStepDcmTrajectory.initializeTrajectory(numberOfTransitions, timeAtTransitions, copAtTransitions, timeAtTransitions[numberOfTransitions - 1], finalDcmPosition);
      timedStepDcmTrajectory.computeTrajectory(timeAtTransitions[0]);
      timedStepDcmTrajectory.getPosition(currentDcmPosition);
      transientDcmTrajectory.initializeTrajectory(initialDcmPosition, currentDcmPosition, timeAtTransitions[0] - transferTime, timeAtTransitions[0]);
   }

   public void computeTrajectory(double currentTime)
   {
      double timedStepStartTime = copPlanner.getTimeAtTransitions()[0];
      if (currentTime < timedStepStartTime)
      {
         transientDcmTrajectory.computeTrajectory(currentTime);
         transientDcmTrajectory.getPosition(currentDcmPosition);
         transientDcmTrajectory.getVelocity(currentDcmVelocity);
      }
      else
      {
         timedStepDcmTrajectory.computeTrajectory(currentTime);
         timedStepDcmTrajectory.getPosition(currentDcmPosition);
         timedStepDcmTrajectory.getVelocity(currentDcmVelocity);
      }
   }

   public void getPosition(FramePoint position)
   {
      position.setIncludingFrame(currentDcmPosition);
   }

   public void getVelocity(FrameVector velocity)
   {
      velocity.setIncludingFrame(currentDcmVelocity);
   }
}
