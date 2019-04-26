package us.ihmc.humanoidBehaviors.behaviors.diagnostic;

import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.sql.Statement;
import java.time.LocalDate;
import java.time.LocalTime;
import java.util.ArrayList;
import java.util.concurrent.ConcurrentLinkedQueue;

public class SQLBehaviorDatabaseManager
{
   private final boolean DEBUG = false;
   private Connection databaseConnection;

   private boolean connected = false;
   private boolean shutdownThread = false;

   private final String databaseURL;

   private final String databaseUsername;
   private final String databasePassword;
   private final String database_host;
   private final String database_port;
   private final String database_name;

   ConcurrentLinkedQueue<PreparedStatement> statements = new ConcurrentLinkedQueue<PreparedStatement>();

   public SQLBehaviorDatabaseManager()
   {
      this.databaseUsername = "behaviors";
      this.databasePassword = "behaviors";
      this.database_host = "10.7.4.48";
      this.database_port = "32769";
      this.database_name = "shadylady";

      databaseURL = "jdbc:postgresql://" + database_host + ":" + database_port + "/" + database_name;

      Connection connection;
      boolean connectionSuccessful = true;

      try
      {
         connection = DriverManager.getConnection(databaseURL, databaseUsername, databasePassword);
      }
      catch (SQLException e)
      {
         connection = null;
         connectionSuccessful = false;
         //LogTools.error("Could not connect to database! DB thread will not run");
         e.printStackTrace();
      }

      if (connectionSuccessful)
      {
         this.databaseConnection = connection;

         Thread thread = new Thread(this::update, "Database_Connection");
         thread.setDaemon(true);
         thread.start();
         connected = true;
         Runtime.getRuntime().addShutdownHook(new Thread(SQLBehaviorDatabaseManager.this::shutdown));
      }
   }

   public boolean isConnected()
   {
      return connected;
   }

   private void update()
   {
      System.out.println("starting thread");
      while (!shutdownThread)
      {
         PreparedStatement stmt;
         while ((stmt = statements.poll()) != null)
         {
            try
            {
               System.out.println("processing thread statements " + shutdownThread);
               stmt.executeUpdate();
               stmt.close();
            }
            catch (SQLException e)
            {
               e.printStackTrace();
            }
         }
      }

      System.out.println("stopping thread");
   }

   private boolean sqlUpdate(String statement)
   {
      Statement stmt = null;
      try
      {

         if (DEBUG)
            System.out.println("Insert into database " + statement);

         stmt = databaseConnection.createStatement();
         //String sql = "INSERT INTO COMPANY (ID,NAME,AGE,ADDRESS,SALARY) "
         //   + "VALUES (1, 'Paul', 32, 'California', 20000.00 );";
         stmt.executeUpdate(statement);
         stmt.close();
         return true;
      }
      catch (Exception e)
      {
         System.err.println(e.getClass().getName() + ": ");
         e.printStackTrace();
      }
      return false;
   }

   private void shutdown()
   {
      try
      {
         //LogTools.info("Beginning database thread shutdown...");
         Thread.sleep(1000);
         shutdownThread = true;
         // LogTools.info("Closing database connection...");
         databaseConnection.close();
      }
      catch (Exception e)
      {
         //LogTools.error("Error closing database connections");
         e.printStackTrace();
         System.exit(1);
      }
   }

   public Operator saveOperator(String operatorName)
   {

      //first check to make sure operator does not exist
      Operator returnOperator = getOperator(operatorName);

      if (returnOperator != null)
      {
         return returnOperator;
      }

      //if it does not exist, create it and then return it. 

      sqlUpdate("INSERT INTO operators (name) VALUES ('" + operatorName + "');");
      returnOperator = getOperator(operatorName);

      if (returnOperator != null)
         return returnOperator;
      return null;
   }

   public Task saveTask(String taskName)
   {

      //first check to make sure operator does not exist
      Task returnTask = getTask(taskName);

      if (returnTask != null)
      {
         return returnTask;
      }

      //if it does not exist, create it and then return it. 

      sqlUpdate("INSERT INTO tasks (name) VALUES ('" + taskName + "');");
      returnTask = getTask(taskName);

      if (returnTask != null)
         return returnTask;
      return null;
   }

   public Run saveRun(Run run)
   {

      try
      {

         PreparedStatement st = databaseConnection
               .prepareStatement("INSERT INTO runs (operator,task,is_successful,notes,log_file,date,time) VALUES (?,?,?,?,?,?,?) RETURNING id;");
         st.setInt(1, run.getOperatorID());
         st.setInt(2, run.getTaskID());
         st.setBoolean(3, run.isSuccessful());
         st.setString(4, run.getNotes());
         st.setString(5, run.getLogFile());
         st.setObject(6, run.getDate());
         st.setObject(7, run.getTime());

         ResultSet lastUpdate = st.executeQuery();

         lastUpdate.next();
         int lastUpdateID = lastUpdate.getInt(1);

         st.close();
         return getRun(lastUpdateID);
      }
      catch (Exception e)
      {
         System.err.println(e.getClass().getName() + ": ");
         e.printStackTrace();
      }
      return null;
   }

   public boolean saveRunEvent(RunEvent run)
   {
      try
      {

         PreparedStatement st = databaseConnection
               .prepareStatement("INSERT INTO run_events (run_id,event_name,event_time_in_seconds,is_successful) VALUES (?,?,?,?)");
         st.setInt(1, run.getRunID());
         st.setString(2, run.getEventName());
         st.setFloat(3, run.getEventTimeInSeconds());
         st.setBoolean(4, run.isSuccessful());
         statements.add(st);
         return true;
      }
      catch (Exception e)
      {
         System.err.println(e.getClass().getName() + ": ");
         e.printStackTrace();
      }

      return false;
   }

   public Operator getOperator(String name)
   {
      Statement stmt = null;
      Operator returnedOperator = null;
      try
      {
         stmt = databaseConnection.createStatement();

         ResultSet rs = stmt.executeQuery("SELECT * FROM operators WHERE operator_name = '" + name + "';");
         while (rs.next())
         {
            returnedOperator = new Operator(rs.getString("operator_name"));
            returnedOperator.operatorID = rs.getInt("operator_id");
         }
         rs.close();
         stmt.close();
         return returnedOperator;
      }
      catch (Exception e)
      {
         System.err.println(e.getClass().getName() + ": ");
         e.printStackTrace();
      }
      return null;
   }

   public int getNumberOfRunEventsAddedForRun(int runID)
   {
      Statement stmt = null;
      int returnValue = 0;
      try
      {
         stmt = databaseConnection.createStatement();

         ResultSet rs = stmt.executeQuery("SELECT * FROM run_events WHERE id = " + runID + ";");
         while (rs.next())
         {
            returnValue++;
         }
         rs.close();
         stmt.close();
      }
      catch (Exception e)
      {
         System.err.println(e.getClass().getName() + ": ");
         e.printStackTrace();
      }
      return returnValue;
   }

   public Task getTask(String name)
   {
      Statement stmt = null;
      Task returnedTask = null;
      try
      {
         stmt = databaseConnection.createStatement();

         ResultSet rs = stmt.executeQuery("SELECT * FROM tasks WHERE task_name = '" + name + "';");
         while (rs.next())
         {
            returnedTask = new Task(rs.getString("task_name"));
            returnedTask.taskID = rs.getInt("task_id");
         }
         rs.close();
         stmt.close();
         return returnedTask;
      }
      catch (Exception e)
      {
         System.err.println(e.getClass().getName() + ": ");
         e.printStackTrace();
      }
      return null;
   }

   public Run getRun(int runId)
   {
      Statement stmt = null;
      Run returnedRun = null;
      try
      {
         stmt = databaseConnection.createStatement();

         ResultSet rs = stmt.executeQuery("SELECT * FROM runs WHERE run_id = " + runId + ";");
         while (rs.next())
         {
            returnedRun = new Run(rs.getInt("operator_id"), rs.getInt("task_id"));
            returnedRun.setRunID(rs.getInt("run_id"));
            returnedRun.setSuccessful(rs.getBoolean("is_successful"));
            returnedRun.setNotes(rs.getString("notes"));
            returnedRun.setLogFile(rs.getString("log_file"));
            returnedRun.setDate(rs.getObject("date", LocalDate.class));
            returnedRun.setTime(rs.getObject("time", LocalTime.class));
         }
         rs.close();
         stmt.close();
         return returnedRun;
      }
      catch (Exception e)
      {
         System.err.println(e.getClass().getName() + ": ");
         e.printStackTrace();
      }
      return null;
   }

   public ArrayList<Run> getAllRuns()
   {
      Statement statement = null;
      ArrayList<Run> runs = new ArrayList<>();
      try
      {
         statement = databaseConnection.createStatement();

         ResultSet rs = statement.executeQuery("SELECT * FROM runs LEFT JOIN operators ON runs.operator_id=operators.operator_id LEFT JOIN tasks ON runs.task_id=tasks.task_id;");
         while (rs.next())
         {
            Run run = new Run(rs.getInt("operator_id"), rs.getInt("task_id"));
            run.setRunID(rs.getInt("run_id"));
            run.setSuccessful(rs.getBoolean("is_successful"));
            run.setNotes(rs.getString("notes"));
            run.setLogFile(rs.getString("log_file"));
            run.setDate(rs.getObject("date", LocalDate.class));
            run.setTime(rs.getObject("time", LocalTime.class));
            run.setOperatorName(rs.getString("operator_name"));
            run.setTaskName(rs.getString("task_name"));

            runs.add(run);
         }
         rs.close();
         statement.close();
         return runs;
      }
      catch (Exception e)
      {
         System.err.println(e.getClass().getName() + ": ");
         e.printStackTrace();
      }
      return null;
   }

   public boolean updateRun(Run run)
   {
      PreparedStatement st = null;
      try
      {
         st = databaseConnection.prepareStatement(
               "UPDATE runs set " + "operator_id = ?, " + "task_id = ?, " + "is_successful = ?, " + "notes = ?, " + "log_file = ?, " + "date = ?," + "time = ? "
                     + " WHERE run_id = ?;");
         st.setInt(1, run.getOperatorID());
         st.setInt(2, run.getTaskID());
         st.setBoolean(3, run.isSuccessful());
         st.setString(4, run.getNotes());
         st.setString(5, run.getLogFile());
         st.setObject(6, run.getDate());
         st.setObject(7, run.getTime());
         st.setObject(8, run.getRunID());

         st.executeUpdate();

         st.close();
         return true;
      }
      catch (Exception e)
      {
         System.err.println(e.getClass().getName());
         e.printStackTrace();
      }
      return false;
   }

   public ArrayList<RunEvent> getEventsForRun(int runID)
   {
      Statement statement = null;
      ArrayList<RunEvent> runEventsList = new ArrayList<RunEvent>();
      try
      {
         statement = databaseConnection.createStatement();

         ResultSet rs = statement.executeQuery("SELECT * FROM run_events WHERE run_id = " + runID + ";");
         while (rs.next())
         {
            String eventName = rs.getString("event_name");
            Float eventTimeInSeconds = rs.getFloat("event_time_in_seconds");
            Boolean isSuccessful = rs.getBoolean("is_successful");
            Integer eventSequence = rs.getInt("event_sequence");
            RunEvent runEvent = new RunEvent(runID, eventName, eventTimeInSeconds, isSuccessful, eventSequence);
            runEventsList.add(runEvent);
         }
         rs.close();
         statement.close();
         return runEventsList;
      }
      catch (Exception e)
      {
         System.err.println(e.getClass().getName() + ": ");
         e.printStackTrace();
      }
      return null;
   }

   public class Operator
   {
      public Operator(String name)
      {
         this.name = name;
      }

      public int operatorID;
      public String name;
   }

   public class Task
   {
      public Task(String taskName)
      {
         this.name = taskName;
      }

      public int taskID;
      public String name;
   }



   public static void main(String[] args)
   {
      SQLBehaviorDatabaseManager test = new SQLBehaviorDatabaseManager();
      String userName = "matt";
      String taskName = "Walk Through Door";

      // System.out.println("adding in user " + userName);
      Operator returnedOperator = test.saveOperator(userName);
      //System.out.println("MY ID NUMBER IS " + returnedOperator.operatorID);

      //make a task
      System.out.println("adding in task " + taskName);
      Task returnedTask = test.saveTask(taskName);
      System.out.println("MY Task ID NUMBER IS " + returnedTask.taskID);

      //make a task
      System.out.println("adding in Run 1");
      Run newRun = new Run(returnedOperator.operatorID, returnedTask.taskID);
      newRun.setNotes("test run");
      newRun.setLogFile("test run log number");

      newRun.setSuccessful(true);
      Run lastRun = test.saveRun(newRun);
      System.out.println("MY Run ID NUMBER IS " + lastRun.getRunID());

      Run returnedRun = test.getRun(lastRun.getRunID());
      System.out.println(returnedRun.getRunID() + " " + returnedRun.getNotes());
      returnedRun.setNotes("new notes");

      test.updateRun(returnedRun);

      Run returnedRun2 = test.getRun(lastRun.getRunID());
      System.out.println(returnedRun2.getNotes());

      RunEvent event = new RunEvent(lastRun.getRunID(), "walk to door", 10, true);

      test.saveRunEvent(event);

      RunEvent event2 = new RunEvent(lastRun.getRunID(), "plan to door", 9, true);

      test.saveRunEvent(event2);
   }
}
