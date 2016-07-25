package us.ihmc.simulationconstructionset.util.simulationRunner;

import java.util.ArrayList;
import java.util.List;

import junit.framework.AssertionFailedError;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;

public class GoalOrientedTestConductor implements VariableChangedListener
{
   private final SimulationConstructionSet scs;
   private final SimulationTestingParameters simulationTestingParameters;
   
   // Yo variables
   private final DoubleYoVariable yoTime;
   
   private List<YoVariableTestGoal> sustainGoals = new ArrayList<>();
   private List<YoVariableTestGoal> waypointGoals = new ArrayList<>();
   private List<YoVariableTestGoal> terminalGoals = new ArrayList<>();
   
   // Temp lists for reporting
   private List<YoVariableTestGoal> sustainGoalsNotMeeting = new ArrayList<>();
   private List<YoVariableTestGoal> waypointGoalsNotMet = new ArrayList<>();
   private List<YoVariableTestGoal> terminalGoalsNotMeeting = new ArrayList<>();
   
   private String assertionFailedMessage = null;
   
   public GoalOrientedTestConductor(SimulationConstructionSet scs, SimulationTestingParameters simulationTestingParameters)
   {
      this.scs = scs;
      this.simulationTestingParameters = simulationTestingParameters;
      
      yoTime = (DoubleYoVariable) scs.getVariable("t");
      
      scs.startOnAThread();
   }
   
   @Override
   public void variableChanged(YoVariable<?> v)
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
         createAssertionFailedException();
         stop();
      }
      else if (terminalGoalsNotMeeting.isEmpty() && waypointGoalsNotMet.size() > 0)
      {
         createAssertionFailedException();
         stop();
      }
      else if (terminalGoalsNotMeeting.isEmpty() && waypointGoalsNotMet.isEmpty())
      {
         createSuccessMessage();
         stop();
      }
   }
   
   private void createSuccessMessage()
   {
      StringBuffer message = new StringBuffer();
      for (YoVariableTestGoal goal : sustainGoals)
      {
         message.append("\nGoal sustained: ");
         for (YoVariable<?> yoVariable : goal.getYoVariables())
         {
            yoVariable.getNameAndValueString(message);
            message.append("  ");
         }
      }
      for (YoVariableTestGoal goal : waypointGoals)
      {
         message.append("\nWaypoint met: ");
         for (YoVariable<?> yoVariable : goal.getYoVariables())
         {
            yoVariable.getNameAndValueString(message);
            message.append("  ");
         }
      }
      for (YoVariableTestGoal goal : terminalGoals)
      {
         message.append("\nTerminal goal met: ");
         for (YoVariable<?> yoVariable : goal.getYoVariables())
         {
            yoVariable.getNameAndValueString(message);
            message.append("  ");
         }
      }
      PrintTools.info(this, message.toString());
   }

   private void createAssertionFailedException()
   {
      StringBuffer message = new StringBuffer();
      
      for (YoVariableTestGoal goal : sustainGoalsNotMeeting)
      {
         message.append("\nGoal not sustained: ");
         for (YoVariable<?> yoVariable : goal.getYoVariables())
         {
            yoVariable.getNameAndValueString(message);
            message.append("  ");
         }
      }
      for (YoVariableTestGoal goal : waypointGoalsNotMet)
      {
         message.append("\nWaypoint not met: ");
         for (YoVariable<?> yoVariable : goal.getYoVariables())
         {
            yoVariable.getNameAndValueString(message);
            message.append("  ");
         }
      }
      for (YoVariableTestGoal goal : terminalGoalsNotMeeting)
      {
         message.append("\nTerminal goal not met: ");
         for (YoVariable<?> yoVariable : goal.getYoVariables())
         {
            yoVariable.getNameAndValueString(message);
            message.append("  ");
         }
      }
      
      assertionFailedMessage = message.toString();
   }
   
   private void stop()
   {
      yoTime.removeVariableChangedListener(this);
      
      sustainGoals.clear();
      waypointGoals.clear();
      terminalGoals.clear();
      scs.stop();
   }
   
   public void simulate() throws AssertionFailedError
   {
      assertionFailedMessage = null;
      yoTime.addVariableChangedListener(this);
      
      scs.simulate();
      
      while (scs.isSimulating())
      {
         ThreadTools.sleep(100);
      }
      
      if (assertionFailedMessage != null)
      {
         throw new AssertionFailedError(assertionFailedMessage);
      }
   }

   public void concludeTesting()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }
      
      if (simulationTestingParameters.getCreateSCSVideos())
      {
         BambooTools.createVideoAndDataWithDateTimeClassMethodAndShareOnSharedDriveIfAvailable(scs.getRobots()[0].getName(), scs, 1);
      }
      
      ThreadTools.sleep(200);
      scs.closeAndDispose();
   }
   
   public void addTimeLimit(DoubleYoVariable timeYoVariable, double timeLimit)
   {
      sustainGoals.add(YoVariableTestGoal.doubleLessThan(timeYoVariable, timeYoVariable.getDoubleValue() + timeLimit));
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
}
