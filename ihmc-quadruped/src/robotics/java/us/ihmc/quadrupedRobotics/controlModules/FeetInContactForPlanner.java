package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.log.LogTools;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeIntervalReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.ArrayList;
import java.util.List;

public class FeetInContactForPlanner
{
   private static final double timeEpsilonForSameCompletion = 1.0e-3;

   private final QuadrupedControllerToolbox controllerToolbox;

   private final QuadrantDependentList<YoBoolean> footContactStateForPlanner = new QuadrantDependentList<>();
   private final List<RobotQuadrant> feetInContact = new ArrayList<>();

   private final QuadrantDependentList<TimeIntervalReadOnly> stepsInProgress = new QuadrantDependentList<>();

   private final RecyclingArrayList<LinkedStep> stepsWithLinkedCompletions = new RecyclingArrayList<>(LinkedStep::new);

   private final DoubleProvider time;
   private final DoubleProvider durationToAllowEarlyTouchdown;

   public FeetInContactForPlanner(QuadrupedControllerToolbox controllerToolbox, DoubleProvider durationToAllowEarlyTouchdown, YoRegistry registry)
   {
      this.controllerToolbox = controllerToolbox;
      this.durationToAllowEarlyTouchdown = durationToAllowEarlyTouchdown;

      time = controllerToolbox.getRuntimeEnvironment().getRobotTimestamp();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         footContactStateForPlanner.put(robotQuadrant, new YoBoolean(robotQuadrant.getShortName() + "ContactStateForPlanner", registry));
      }
   }

   public void update()
   {
      feetInContact.clear();
      List<RobotQuadrant> actualFeetInContact = controllerToolbox.getFeetInContact();
      for (int i = 0; i < actualFeetInContact.size(); i++)
         feetInContact.add(actualFeetInContact.get(i));

      int stepIndex = 0;
      while (stepIndex < stepsWithLinkedCompletions.size())
      {
         LinkedStep linkedStep = stepsWithLinkedCompletions.get(stepIndex);
         RobotQuadrant robotQuadrant = linkedStep.getRobotQuadrant();
         if (linkedStep.isStepDone(time.getValue()))
         {
            stepsInProgress.remove(robotQuadrant);
            stepsWithLinkedCompletions.remove(stepIndex);
         }
         else
         {
            stepIndex++;
         }
      }

      /*
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         TimeIntervalReadOnly stepInProgress = stepsInProgress.get(robotQuadrant);

         // remove from the current feet in contact if it was scheduled to end later (meaning it touched down early)
         if (stepInProgress != null)
            feetInContact.remove(robotQuadrant);
      }
      */

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         footContactStateForPlanner.get(robotQuadrant).set(feetInContact.contains(robotQuadrant));
   }

   public List<RobotQuadrant> getFeetInContactForPlanner()
   {
      return feetInContact;
   }

   public void beganStep(RobotQuadrant robotQuadrant, TimeIntervalReadOnly timeInterval)
   {
      LinkedStep completionLinkedStep = stepsWithLinkedCompletions.add();
      completionLinkedStep.reset();
      completionLinkedStep.set(robotQuadrant, timeInterval);

      for (int i = 0; i < stepsWithLinkedCompletions.size(); i++)
      {
         LinkedStep otherStep = stepsWithLinkedCompletions.get(i);
         if (completionLinkedStep == otherStep)
            continue;

         TimeIntervalReadOnly timeIntervalInProgress = otherStep.getTimeInterval();

         if (MathTools.epsilonEquals(timeInterval.getEndTime(), timeIntervalInProgress.getEndTime(), timeEpsilonForSameCompletion))
         {
            completionLinkedStep.bindStepForCompletion(otherStep);
         }
      }

      stepsInProgress.put(robotQuadrant, timeInterval);
   }

   public void completedStep(RobotQuadrant robotQuadrant)
   {
      for (int i = 0; i < stepsWithLinkedCompletions.size(); i++)
      {
         LinkedStep step = stepsWithLinkedCompletions.get(i);
         if (robotQuadrant == step.getRobotQuadrant())
            step.setIsFootInContact(true);
      }
   }

   public class LinkedStep
   {
      private boolean isFootInContact = false;
      private RobotQuadrant robotQuadrant;
      private TimeIntervalReadOnly timeInterval;

      private List<LinkedStep> boundStepsForCompletion = new ArrayList<>();

      public void setIsFootInContact(boolean isStepDone)
      {
         this.isFootInContact = isStepDone;
      }

      public void set(RobotQuadrant robotQuadrant, TimeIntervalReadOnly timeInterval)
      {
         this.robotQuadrant = robotQuadrant;
         this.timeInterval = timeInterval;
      }

      public RobotQuadrant getRobotQuadrant()
      {
         return robotQuadrant;
      }

      public TimeIntervalReadOnly getTimeInterval()
      {
         return timeInterval;
      }

      public void reset()
      {
         isFootInContact = false;
         robotQuadrant = null;
         timeInterval = null;
         boundStepsForCompletion.clear();
      }

      public void bindStepForCompletion(LinkedStep stepToBind)
      {
         boundStepsForCompletion.add(stepToBind);
         List<LinkedStep> otherBoundStepsForCompletion = stepToBind.getBoundStepsForCompletion();
         for (int i = 0; i < otherBoundStepsForCompletion.size(); i++)
            boundStepsForCompletion.add(otherBoundStepsForCompletion.get(i));

         otherBoundStepsForCompletion.add(this);
      }

      public List<LinkedStep> getBoundStepsForCompletion()
      {
         return boundStepsForCompletion;
      }

      public boolean isStepInContact(double time)
      {
         if (!isFootInContact)
            return false;

         return time > timeInterval.getEndTime() - durationToAllowEarlyTouchdown.getValue();
      }

      public boolean isStepDone(double time)
      {
         if (!isStepInContact(time))
            return false;

         for (int i = 0; i < boundStepsForCompletion.size(); i++)
         {
            if (!boundStepsForCompletion.get(i).isStepInContact(time))
               return false;
         }

         return true;
      }
   }
}
