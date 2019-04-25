package us.ihmc.humanoidBehaviors.behaviors.diagnostic;

import java.time.LocalDate;
import java.time.LocalTime;

public class Run
{
   public Run(int operatorID, int TaskID)
   {
      this.operatorID = operatorID;

      this.taskID = TaskID;
      date = LocalDate.now();
      time = LocalTime.now();
   }

   public int runID;
   public int operatorID;
   public String operatorName;
   public int taskID;
   public String taskName;
   public boolean successful;
   public String notes;
   public String logFile;
   public LocalDate date;
   public LocalTime time;

   public int getRunID()
   {
      return runID;
   }

   public void setRunID(int runID)
   {
      this.runID = runID;
   }

   public int getOperatorID()
   {
      return operatorID;
   }

   public void setOperatorID(int operatorID)
   {
      this.operatorID = operatorID;
   }

   public String getOperatorName()
   {
      return operatorName;
   }

   public void setOperatorName(String operatorName)
   {
      this.operatorName = operatorName;
   }

   public int getTaskID()
   {
      return taskID;
   }

   public void setTaskID(int taskID)
   {
      this.taskID = taskID;
   }

   public String getTaskName()
   {
      return taskName;
   }

   public void setTaskName(String taskName)
   {
      this.taskName = taskName;
   }

   public boolean isSuccessful()
   {
      return successful;
   }

   public void setSuccessful(boolean successful)
   {
      this.successful = successful;
   }

   public String getNotes()
   {
      return notes;
   }

   public void setNotes(String notes)
   {
      this.notes = notes;
   }

   public String getLogFile()
   {
      return logFile;
   }

   public void setLogFile(String logFile)
   {
      this.logFile = logFile;
   }

   public LocalDate getDate()
   {
      return date;
   }

   public void setDate(LocalDate date)
   {
      this.date = date;
   }

   public LocalTime getTime()
   {
      return time;
   }

   public void setTime(LocalTime time)
   {
      this.time = time;
   }

   @Override
   public String toString()
   {
      return "Run " + runID + ": (" + operatorID + ") " + taskID + ", successful=" + successful + ", notes='" + notes + '\'' + ", date=" + date + ", time="
            + time + '}';
   }
}
