package us.ihmc.simulationConstructionSetTools.util.simulationrunner;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import junit.framework.AssertionFailedError;
import us.ihmc.commons.PrintTools;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationDoneListener;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.commons.thread.ThreadTools;

public class GoalOrientedTestConductor implements VariableChangedListener, SimulationDoneListener
{
   private final SimulationConstructionSet scs;
   private final SimulationTestingParameters simulationTestingParameters;
   
   private boolean yoTimeChangedListenerActive = false;
   
   private List<YoVariableTestGoal> sustainGoals = new ArrayList<>();
   private List<YoVariableTestGoal> waypointGoals = new ArrayList<>();
   private List<YoVariableTestGoal> terminalGoals = new ArrayList<>();
   
   // Temp lists for reporting
   private List<YoVariableTestGoal> sustainGoalsNotMeeting = new ArrayList<>();
   private List<YoVariableTestGoal> waypointGoalsNotMet = new ArrayList<>();
   private List<YoVariableTestGoal> terminalGoalsNotMeeting = new ArrayList<>();

   private final AtomicBoolean createAssertionFailedException = new AtomicBoolean();
   private final AtomicBoolean printSuccessMessage = new AtomicBoolean();

   private String assertionFailedMessage = null;
   
   public GoalOrientedTestConductor(SimulationConstructionSet scs, SimulationTestingParameters simulationTestingParameters)
   {
      this.scs = scs;
      this.simulationTestingParameters = simulationTestingParameters;
      
      YoDouble yoTime = (YoDouble) scs.getVariable("t");
      yoTime.addVariableChangedListener(this);
      scs.startOnAThread();
      scs.addSimulateDoneListener(this);
   }
   
   @Override
   public void notifyOfVariableChange(YoVariable<?> v)
   {
      if (yoTimeChangedListenerActive)
      {
         sustainGoalsNotMeeting.clear();
         waypointGoalsNotMet.clear();
         terminalGoalsNotMeeting.clear();
         
         for (int i = 0; i < sustainGoals.size(); i++)
         {
            if (!sustainGoals.get(i).currentlyMeetsGoal())
            {
               sustainGoalsNotMeeting.add(sustainGoals.get(i));
            }
         }
         
         for (int i = 0; i < waypointGoals.size(); i++)
         {
            if (!waypointGoals.get(i).hasMetGoal())
            {
               waypointGoalsNotMet.add(waypointGoals.get(i));
            }
         }
   
         for (int i = 0; i < terminalGoals.size(); i++)
         {
            if (!terminalGoals.get(i).currentlyMeetsGoal())
            {
               terminalGoalsNotMeeting.add(terminalGoals.get(i));
            }
         }
         
         if (sustainGoalsNotMeeting.size() > 0)
         {
            createAssertionFailedException.set(true);
         }
         else if (terminalGoalsNotMeeting.isEmpty())
         {
            if(waypointGoalsNotMet.size() > 0)
            {
               createAssertionFailedException.set(true);
            }
            else
            {
               printSuccessMessage.set(true);
            }
         }
      }
   }
   
   private void printSuccessMessage()
   {
      StringBuffer message = new StringBuffer();
      message.append("Success:");
      for (YoVariableTestGoal goal : sustainGoals)
      {
         message.append("\nGoal sustained: ");
         message.append(goal.toString());
      }
      for (YoVariableTestGoal goal : waypointGoals)
      {
         message.append("\nWaypoint met: ");
         message.append(goal.toString());
      }
      for (YoVariableTestGoal goal : terminalGoals)
      {
         message.append("\nTerminal goal met: ");
         message.append(goal.toString());
      }
      PrintTools.info(this, message.toString());
   }
   
   private void printSimulatingMessage()
   {
      StringBuffer message = new StringBuffer();
      message.append("Simulating with goals:");
      for (YoVariableTestGoal goal : sustainGoals)
      {
         message.append("\nSustain goal: ");
         message.append(goal.toString());
      }
      for (YoVariableTestGoal goal : waypointGoals)
      {
         message.append("\nWaypoint goal: ");
         message.append(goal.toString());
      }
      for (YoVariableTestGoal goal : terminalGoals)
      {
         message.append("\nTerminal goal: ");
         message.append(goal.toString());
      }
      PrintTools.info(this, message.toString());
   }

   private void createAssertionFailedException()
   {
      StringBuffer message = new StringBuffer();
      message.append("Assertion failed:");
      for (YoVariableTestGoal goal : sustainGoalsNotMeeting)
      {
         message.append("\nGoal not sustained: ");
         message.append(goal.toString());
      }
      for (YoVariableTestGoal goal : waypointGoalsNotMet)
      {
         message.append("\nWaypoint not met: ");
         message.append(goal.toString());
      }
      for (YoVariableTestGoal goal : terminalGoalsNotMeeting)
      {
         message.append("\nTerminal goal not met: ");
         message.append(goal.toString());
      }
      
      assertionFailedMessage = message.toString();
   }
   
   private void stop()
   {
      yoTimeChangedListenerActive = false;
      
      sustainGoals.clear();
      waypointGoals.clear();
      terminalGoals.clear();
      scs.stop();
   }
   
   public void simulate() throws AssertionFailedError
   {
      assertionFailedMessage = null;
      yoTimeChangedListenerActive = true;

      createAssertionFailedException.set(false);
      printSuccessMessage.set(false);
      
      printSimulatingMessage();
      
      scs.simulate();

      while (!createAssertionFailedException.get() && !printSuccessMessage.get())
      {
         Thread.yield();
      }

      if(createAssertionFailedException.get())
      {
         createAssertionFailedException();
         stop();
      }
      else if(printSuccessMessage.get())
      {
         printSuccessMessage();
         stop();
      }
      
      //wait to see if scs threw any exceptions
      ThreadTools.sleep(10);
      
      if (assertionFailedMessage != null)
      {
         PrintTools.error(this, assertionFailedMessage);
         
         throw new AssertionFailedError(assertionFailedMessage);
      }
   }

   public void concludeTesting(int additionalStackDepthForRelevantCallingMethod)
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }
      
      if (simulationTestingParameters.getCreateSCSVideos())
      {
         BambooTools.createVideoWithDateTimeClassMethodAndShareOnSharedDriveIfAvailable(scs.getRobots()[0].getName(), scs, additionalStackDepthForRelevantCallingMethod + 1);
      }
      
      ThreadTools.sleep(200);
      scs.closeAndDispose();
   }

   public void concludeTesting(String videoName)
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }
      
      if (simulationTestingParameters.getCreateSCSVideos())
      {
         BambooTools.createVideoWithDateTimeAndStoreInDefaultDirectory(scs, videoName);
      }
      
      ThreadTools.sleep(200);
      scs.closeAndDispose();
   }
   
   public void concludeTesting()
   {
      concludeTesting(2);
   }
   
   public void addTimeLimit(YoDouble timeYoVariable, double timeLimit)
   {
      sustainGoals.add(YoVariableTestGoal.doubleLessThan(timeYoVariable, timeYoVariable.getDoubleValue() + timeLimit));
   }
   
   public void addDurationGoal(YoDouble timeYoVariable, double durationFromNow)
   {
      terminalGoals.add(YoVariableTestGoal.timeInFuture(timeYoVariable, durationFromNow));
   }
   
   public void addSustainGoal(YoVariableTestGoal yoVariableTestGoal)
   {
      sustainGoals.add(yoVariableTestGoal);
   }

   public void addWaypointGoal(YoVariableTestGoal yoVariableTestGoal)
   {
      waypointGoals.add(yoVariableTestGoal);
   }

   public void addTerminalGoal(YoVariableTestGoal yoVariableTestGoal)
   {
      terminalGoals.add(yoVariableTestGoal);
   }
   
   public SimulationConstructionSet getScs()
   {
      return scs;
   }

   public void setKeepSCSUp(boolean keepSCSUp)
   {
      simulationTestingParameters.setKeepSCSUp(keepSCSUp);
   }

   @Override
   public void simulationDone()
   {
      
   }

   @Override
   public void simulationDoneWithException(Throwable throwable)
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         PrintTools.error(throwable.getMessage());
      }
      assertionFailedMessage = throwable.getMessage();
   }
}
