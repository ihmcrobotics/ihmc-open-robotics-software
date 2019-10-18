package us.ihmc.simulationConstructionSetTools.util.simulationrunner;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import org.opentest4j.AssertionFailedError;

import us.ihmc.log.LogTools;
import us.ihmc.robotics.testing.GoalOrientedTestGoal;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationDoneListener;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.commons.thread.ThreadTools;

import static us.ihmc.robotics.Assert.*;

public class GoalOrientedTestConductor implements SimulationDoneListener
{
   private final SimulationConstructionSet scs;
   private final SimulationTestingParameters simulationTestingParameters;
   
   private boolean yoTimeChangedListenerActive = false;
   
   private List<GoalOrientedTestGoal> sustainGoals = new ArrayList<>();
   private List<GoalOrientedTestGoal> waypointGoals = new ArrayList<>();
   private List<GoalOrientedTestGoal> terminalGoals = new ArrayList<>();
   
   // Temp lists for reporting
   private List<GoalOrientedTestGoal> sustainGoalsNotMeeting = new ArrayList<>();
   private List<GoalOrientedTestGoal> waypointGoalsNotMet = new ArrayList<>();
   private List<GoalOrientedTestGoal> terminalGoalsNotMeeting = new ArrayList<>();

   private final AtomicBoolean createAssertionFailedException = new AtomicBoolean();
   private final AtomicBoolean printSuccessMessage = new AtomicBoolean();
   private final AtomicBoolean scsHasCrashed = new AtomicBoolean();

   private String assertionFailedMessage = null;
   private String scsCrashedException = null;
   
   public GoalOrientedTestConductor(SimulationConstructionSet scs, SimulationTestingParameters simulationTestingParameters)
   {
      this.scs = scs;
      this.simulationTestingParameters = simulationTestingParameters;
      
      YoDouble yoTime = (YoDouble) scs.getVariable("t");
      yoTime.addVariableChangedListener(this::notifyOfVariableChange);
      scs.startOnAThread();
      scs.addSimulateDoneListener(this);
   }
   
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
      for (GoalOrientedTestGoal goal : sustainGoals)
      {
         message.append("\nGoal sustained: ");
         message.append(goal.toString());
      }
      for (GoalOrientedTestGoal goal : waypointGoals)
      {
         message.append("\nWaypoint met: ");
         message.append(goal.toString());
      }
      for (GoalOrientedTestGoal goal : terminalGoals)
      {
         message.append("\nTerminal goal met: ");
         message.append(goal.toString());
      }
      LogTools.info(message.toString());
   }
   
   private void printSimulatingMessage()
   {
      StringBuffer message = new StringBuffer();
      message.append("Simulating with goals:");
      for (GoalOrientedTestGoal goal : sustainGoals)
      {
         message.append("\nSustain goal: ");
         message.append(goal.toString());
      }
      for (GoalOrientedTestGoal goal : waypointGoals)
      {
         message.append("\nWaypoint goal: ");
         message.append(goal.toString());
      }
      for (GoalOrientedTestGoal goal : terminalGoals)
      {
         message.append("\nTerminal goal: ");
         message.append(goal.toString());
      }
      LogTools.info(message.toString());
   }

   private void createAssertionFailedException()
   {
      StringBuffer message = new StringBuffer();
      message.append("Assertion failed:");
      for (int i = 0; i < sustainGoalsNotMeeting.size(); i++)
      {
         message.append("\nGoal not sustained: ");
         message.append(sustainGoalsNotMeeting.get(i).toString());
      }
      for (int i = 0; i < waypointGoalsNotMet.size(); i++)
      {
         message.append("\nWaypoint not met: ");
         message.append(waypointGoalsNotMet.get(i).toString());
      }
      for (int i = 0; i < terminalGoalsNotMeeting.size(); i++)
      {
         message.append("\nTerminal goal not met: ");
         message.append(terminalGoalsNotMeeting.get(i).toString());
      }

      assertionFailedMessage = message.toString();
   }

   public void setTerrainObject3D(TerrainObject3D terrainObject3D)
   {}
   
   private void stop()
   {
      yoTimeChangedListenerActive = false;
      
      sustainGoals.clear();
      waypointGoals.clear();
      terminalGoals.clear();
      scs.stop();
   }
   
   public void simulate()
   {
      assertionFailedMessage = null;
      yoTimeChangedListenerActive = true;

      createAssertionFailedException.set(false);
      printSuccessMessage.set(false);
      scsHasCrashed.set(false);
      
      printSimulatingMessage();

      scs.simulate();

      while (!createAssertionFailedException.get() && !printSuccessMessage.get() && !scsHasCrashed.get())
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
      else if(scsHasCrashed.get())
      {
         stop();
         LogTools.error(scsCrashedException);
         fail();
      }
      
      //wait to see if scs threw any exceptions
      ThreadTools.sleep(10);
      
      if (assertionFailedMessage != null)
      {
         LogTools.error(assertionFailedMessage);
         
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
   
   public void addSustainGoal(GoalOrientedTestGoal yoVariableTestGoal)
   {
      sustainGoals.add(yoVariableTestGoal);
   }

   public void addWaypointGoal(GoalOrientedTestGoal yoVariableTestGoal)
   {
      waypointGoals.add(yoVariableTestGoal);
   }

   public void addTerminalGoal(GoalOrientedTestGoal yoVariableTestGoal)
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
         LogTools.error(throwable.getMessage());
      }
      scsCrashedException = throwable.getMessage();
      scsHasCrashed.set(true);
   }
}
