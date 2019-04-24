package us.ihmc.humanoidBehaviors.behaviors.diagnostic;

import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.sql.Statement;
import java.time.LocalDate;
import java.time.LocalTime;
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
   private int foo = 0;

   ConcurrentLinkedQueue<PreparedStatement> statements = new ConcurrentLinkedQueue<PreparedStatement>();

   public SQLBehaviorDatabaseManager()
   {
      this.databaseUsername = "database";
      this.databasePassword = "database";
      this.database_host = "10.7.4.48";
      this.database_port = "32769";
      this.database_name = "database";

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

         PreparedStatement st = databaseConnection.prepareStatement("INSERT INTO runs (operator,task,is_successful,notes,log_file,date,time) VALUES (?,?,?,?,?,?,?) RETURNING id;");
         st.setInt(1, run.operatorID);
         st.setInt(2, run.taskID);
         st.setBoolean(3, run.successful);
         st.setString(4, run.notes);
         st.setString(5, run.logFile);
         st.setObject(6, run.date);
         st.setObject(7, run.time);

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

         PreparedStatement st = databaseConnection.prepareStatement("INSERT INTO run_events (run_id,event_name,event_time_in_seconds,is_successful) VALUES (?,?,?,?)");
         st.setInt(1, run.runID);
         st.setString(2, run.eventName);
         st.setFloat(3, run.runTime);
         st.setBoolean(4, run.successful);
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

         ResultSet rs = stmt.executeQuery("SELECT * FROM operators WHERE name = '" + name + "';");
         while (rs.next())
         {
            returnedOperator = new Operator(rs.getString("name"));
            returnedOperator.operatorID = rs.getInt("id");
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

         ResultSet rs = stmt.executeQuery("SELECT * FROM tasks WHERE name = '" + name + "';");
         while (rs.next())
         {
            returnedTask = new Task(rs.getString("name"));
            returnedTask.taskID = rs.getInt("id");
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

         ResultSet rs = stmt.executeQuery("SELECT * FROM runs WHERE id = " + runId + ";");
         while (rs.next())
         {
            returnedRun = new Run(rs.getInt("operator"), rs.getInt("task"));
            returnedRun.runID = rs.getInt("id");
            returnedRun.successful = rs.getBoolean("is_successful");
            returnedRun.notes = rs.getString("notes");
            returnedRun.logFile = rs.getString("log_file");
            returnedRun.date = rs.getObject("date", LocalDate.class);
            returnedRun.time = rs.getObject("time", LocalTime.class);
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

   public boolean updateRun(Run run)
   {
      PreparedStatement st = null;
      try
      {
         st = databaseConnection.prepareStatement("UPDATE runs set " + "operator = ?, " + "task = ?, " + "is_successful = ?, " + "notes = ?, "
               + "log_file = ?, " + "date = ?," + "time = ? " + " WHERE id = ?;");
         st.setInt(1, run.operatorID);
         st.setInt(2, run.taskID);
         st.setBoolean(3, run.successful);
         st.setString(4, run.notes);
         st.setString(5, run.logFile);
         st.setObject(6, run.date);
         st.setObject(7, run.time);
         st.setObject(8, run.runID);

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

   public class Operator
   {
      public Operator(String name)
      {
         this.name = name;
      }

      public int operatorID;
      public String name;
   }

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
      public int taskID;
      public boolean successful;
      public String notes;
      public String logFile;
      public LocalDate date;
      public LocalTime time;
   }

   public class RunEvent
   {
      public RunEvent(int runID, String eventName, float runTime, boolean successfull)
      {
         this.runID = runID;
         this.eventName = eventName;
         this.runTime = runTime;
         this.successful = successfull;
      }

      public int runID;
      public String eventName;
      public float runTime;
      public boolean successful;
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
      Run newRun = test.new Run(returnedOperator.operatorID, returnedTask.taskID);
      newRun.notes = "test run";
      newRun.logFile = "test run log number";

      newRun.successful = true;
      Run lastRun = test.saveRun(newRun);
      System.out.println("MY Run ID NUMBER IS " + lastRun.runID);

      Run returnedRun = test.getRun(lastRun.runID);
      System.out.println(returnedRun.runID + " " + returnedRun.notes);
      returnedRun.notes = "new notes";

      test.updateRun(returnedRun);

      Run returnedRun2 = test.getRun(lastRun.runID);
      System.out.println(returnedRun2.notes);

      RunEvent event = test.new RunEvent(lastRun.runID, "walk to door", 10, true);

      test.saveRunEvent(event);

      RunEvent event2 = test.new RunEvent(lastRun.runID, "plan to door", 9, true);

      test.saveRunEvent(event2);

   }

}
