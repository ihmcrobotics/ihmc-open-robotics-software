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
   
   private List<YoVariableTestGoal> finalGoals = new ArrayList<>();
   private List<YoVariableTestGoal> waypointGoals = new ArrayList<>();
   private List<YoVariableTestGoal> sustainGoals = new ArrayList<>();
   
   // Temp lists for reporting
   private List<YoVariableTestGoal> finalGoalsNotMeeting = new ArrayList<>();
   private List<YoVariableTestGoal> waypointGoalsNotMet = new ArrayList<>();
   private List<YoVariableTestGoal> sustainGoalsNotMeeting = new ArrayList<>();
   
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
      finalGoalsNotMeeting.clear();
      
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

      for (int i = 0; i < finalGoals.size(); i++)
      {
         if (!finalGoals.get(i).currentlyMeetsGoal())
         {
            finalGoalsNotMeeting.add(finalGoals.get(i));
         }
      }
      
      if (sustainGoalsNotMeeting.size() > 0)
      {
         createAssertionFailedException();
         stop();
      }
      else if (finalGoalsNotMeeting.isEmpty() && waypointGoalsNotMet.size() > 0)
      {
         createAssertionFailedException();
         stop();
      }
      else if (finalGoalsNotMeeting.isEmpty() && waypointGoalsNotMet.isEmpty())
      {
         createSuccessMessage();
         stop();
      }
   }
   
   private void createSuccessMessage()
   {
      StringBuffer message = new StringBuffer();
      for (YoVariableTestGoal goal : finalGoals)
      {
         message.append("Final goal met: ");
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
      for (YoVariableTestGoal goal : finalGoalsNotMeeting)
      {
         message.append("Final goal not met: ");
         goal.getYoVariable().getNameAndValueString(message);
         message.append("\n");
      }
      
      assertionFailedError = new AssertionFailedError(message.toString());
   }
   
   private void stop()
   {
      yoTime.removeVariableChangedListener(this);
      
      finalGoals.clear();
      waypointGoals.clear();
      sustainGoals.clear();
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
   
   public void addFinalGoal(YoVariableTestGoal yoVariableTestGoal)
   {
      finalGoals.add(yoVariableTestGoal);
   }
   
   public void addWaypointGoal(YoVariableTestGoal yoVariableTestGoal)
   {
      waypointGoals.add(yoVariableTestGoal);
   }
   
   public void addSustainGoal(YoVariableTestGoal yoVariableTestGoal)
   {
      sustainGoals.add(yoVariableTestGoal);
   }

   public SimulationConstructionSet getScs()
   {
      return scs;
   }
}
