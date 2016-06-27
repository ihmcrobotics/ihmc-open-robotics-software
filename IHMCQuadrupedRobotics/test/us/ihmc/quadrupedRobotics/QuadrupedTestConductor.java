package us.ihmc.quadrupedRobotics;

import java.util.ArrayList;
import java.util.List;

import junit.framework.AssertionFailedError;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;

public class QuadrupedTestConductor implements VariableChangedListener
{
   private final SimulationConstructionSet scs;
   
   // Yo variables
   private final DoubleYoVariable yoTime;
   
   private List<YoVariableTestGoal> sustainGoals = new ArrayList<>();
   private List<YoVariableTestGoal> waypointGoals = new ArrayList<>();
   private List<YoVariableTestGoal> terminalGoals = new ArrayList<>();
   
   // Temp lists for reporting
   private List<YoVariableTestGoal> sustainGoalsNotMeeting = new ArrayList<>();
   private List<YoVariableTestGoal> waypointGoalsNotMet = new ArrayList<>();
   private List<YoVariableTestGoal> terminalGoalsNotMeeting = new ArrayList<>();
   
   private AssertionFailedError assertionFailedError = null;
   
   public QuadrupedTestConductor(SimulationConstructionSet scs)
   {
      this.scs = scs;
      
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
      for (YoVariableTestGoal goal : terminalGoals)
      {
         message.append("Terminal goal met: ");
         goal.getYoVariable().getNameAndValueString(message);
         message.append("\n");
      }
      PrintTools.info(this, message.toString());
   }

   private void createAssertionFailedException()
   {
      StringBuffer message = new StringBuffer();
      
      for (YoVariableTestGoal goal : sustainGoalsNotMeeting)
      {
         message.append("Goal not sustained: ");
         goal.getYoVariable().getNameAndValueString(message);
         message.append("\n");
      }
      for (YoVariableTestGoal goal : waypointGoalsNotMet)
      {
         message.append("Waypoint not met: ");
         goal.getYoVariable().getNameAndValueString(message);
         message.append("\n");
      }
      for (YoVariableTestGoal goal : terminalGoalsNotMeeting)
      {
         message.append("Terminal goal not met: ");
         goal.getYoVariable().getNameAndValueString(message);
         message.append("\n");
      }
      
      assertionFailedError = new AssertionFailedError(message.toString());
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
      assertionFailedError = null;
      yoTime.addVariableChangedListener(this);
      
      scs.simulate();
      
      while (scs.isSimulating())
      {
         ThreadTools.sleep(100);
      }
      
      if (assertionFailedError != null)
      {
         throw assertionFailedError;
      }
   }

   public void destroy()
   {
      scs.closeAndDispose();
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
