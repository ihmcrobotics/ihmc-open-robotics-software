package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.log.LogTools;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeIntervalReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;

import java.util.ArrayList;
import java.util.List;

public class FeetInContactForPlanner
{
   private final QuadrupedControllerToolbox controllerToolbox;

   private final List<RobotQuadrant> feetInContact = new ArrayList<>();

   private final QuadrantDependentList<TimeIntervalReadOnly> stepsInProgress = new QuadrantDependentList<>();
   private final List<RobotQuadrant> stepsThatShouldBeRemoved = new ArrayList<>();

   private final DoubleProvider time;

   public FeetInContactForPlanner(QuadrupedControllerToolbox controllerToolbox)
   {
      this.controllerToolbox = controllerToolbox;
      time = controllerToolbox.getRuntimeEnvironment().getRobotTimestamp();
   }

   public void update()
   {
      feetInContact.clear();
      List<RobotQuadrant> actualFeetInContact = controllerToolbox.getFeetInContact();
      for (int i = 0; i < actualFeetInContact.size(); i++)
         feetInContact.add(actualFeetInContact.get(i));

      int stepIndex = 0;
      while (stepIndex < stepsThatShouldBeRemoved.size())
      {
         RobotQuadrant robotQuadrant = stepsThatShouldBeRemoved.get(stepIndex);
         TimeIntervalReadOnly step = stepsInProgress.get(robotQuadrant);
         if (time.getValue() >= step.getEndTime())
         {
            stepsInProgress.remove(robotQuadrant);
            stepsThatShouldBeRemoved.remove(stepIndex);
         }
         else
         {
            stepIndex++;
         }
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         TimeIntervalReadOnly stepInProgress = stepsInProgress.get(robotQuadrant);

         // remove from the current feet in progress if it was scheduled to end later (meaning it touched down early)
         if (stepInProgress != null)
            feetInContact.remove(robotQuadrant);
      }
   }

   public List<RobotQuadrant> getFeetInContactForPlanner()
   {
      return feetInContact;
   }

   public void beganStep(RobotQuadrant robotQuadrant, TimeIntervalReadOnly timeInterval)
   {
      stepsInProgress.put(robotQuadrant, timeInterval);
   }

   public void completedStep(RobotQuadrant robotQuadrant)
   {
      stepsThatShouldBeRemoved.add(robotQuadrant);
   }
}
