package us.ihmc.humanoidBehaviors.behaviors.diagnostic;

import java.sql.Connection;
import java.sql.Date;
import java.sql.DriverManager;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.sql.Statement;
import java.sql.Time;
import java.util.concurrent.ConcurrentLinkedQueue;


public class SQLDatabaseManager
{
   private final boolean DEBUG = true;
   private Connection databaseConnection;

   private boolean connected = false;
   private boolean shutdown = false;

   private final String databaseURL;

   private final String databaseUsername;
   private final String databasePassword;
   private final String database_host;
   private final String database_port;
   private final String database_name;

   ConcurrentLinkedQueue<String> statements = new ConcurrentLinkedQueue<String>();

   public SQLDatabaseManager()
   {
      this.databaseUsername = "shadylady";
      this.databasePassword = "ShadyLady";
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
         Runtime.getRuntime().addShutdownHook(new Thread(SQLDatabaseManager.this::shutdown));
      }

   }

   private void update()
   {
      /*
       * if(connected && !shutdown) { if() try { isNewDataAvailable = true;
       * while (isNewDataAvailable) { updateInsertPreparedStatement();
       * timeSeriesInsertPreparedStatement.addBatch(); batchCount++; if
       * (batchCount >= BATCHES_BEFORE_EXECUTE) {
       * executionTimer.startMeasurement();
       * timeSeriesInsertPreparedStatement.executeBatch();
       * executionTimer.stopMeasurement(); batchCount = 0; } isNewDataAvailable
       * = true; } } catch (SQLException e) {
       * LogTools.error("Error executing database update!");
       * e.printStackTrace(); } }
       */

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
         System.err.println(e.getClass().getName() + ": " + e.getMessage());
      }
      return false;
   }

   private void shutdown()
   {
      try
      {
         //LogTools.info("Beginning database thread shutdown...");
         //   finalizeRecording();

        // LogTools.info("Closing database connection...");
         databaseConnection.close();
      }
      catch (SQLException e)
      {
         //LogTools.error("Error closing database connections");
         e.printStackTrace();
         System.exit(1);
      }
   }

   /*
    * private void finalizeRecording() throws SQLException { shutdown = true;
    * LogTools.info("Processing remaining buffered data..."); synchronized
    * (timeSeriesInsertPreparedStatement) { while (isNewDataAvailable) {
    * updateInsertPreparedStatement(); isNewDataAvailable = true; }
    * timeSeriesInsertPreparedStatement.executeBatch(); }
    * LogTools.info("All data processed, safe to finish shut down."); }
    */

   public Operator saveOperator(String operatorName)
   {
      sqlUpdate("INSERT INTO operators (name) VALUES (" + operatorName + ");");
      //Operator returnOperator = getOperator(operatorName);

     // if (returnOperator != null)
      //   return returnOperator;
      return null;
   }

   public boolean saveRun(Run run)
   {
      return sqlUpdate("INSERT INTO runs (id,operator,task,is_successful,notes,log_file,date,time) VALUES (" + run.runID + "," + run.operatorID + ","
            + run.taskID + "," + run.successful + "," + run.notes + "," + run.logFile + "," + run.date + "," + run.time + ");");
   }

   public boolean saveRunEvent(RunEvent runEvent)
   {
      return sqlUpdate("INSERT INTO run_events (run_id,event_name,event_time_in_seconds,is_successful) VALUES (" + runEvent.runID + "," + runEvent.eventName
            + "," + runEvent.runTime + "," + runEvent.successful + ");");
   }

   public boolean saveTask(Task run)
   {
      return sqlUpdate("INSERT INTO tasks (id,name) VALUES (" + run.taskID + "," + run.name + ");");
   }

   public Operator getOperator(String name)
   {
      Statement stmt = null;
      Operator returnedOperator = new Operator();
      try
      {
         stmt = databaseConnection.createStatement();

         ResultSet rs = stmt.executeQuery("SELECT * FROM operators WHERE name = " + name + ";");
         while (rs.next())
         {
            returnedOperator.operatorID = rs.getInt("id");
            returnedOperator.name = rs.getString("name");
         }
         rs.close();
         stmt.close();
         return returnedOperator;
      }
      catch (Exception e)
      {
         System.err.println(e.getClass().getName() + ": " + e.getMessage());
      }
      return null;
   }

   public class Operator
   {
      public int operatorID;
      public String name;
   }

   public class Run
   {
      public int runID;
      public int operatorID;
      public int taskID;
      public boolean successful;
      public String notes;
      public String logFile;
      public Date date;
      public Time time;
   }

   public class RunEvent
   {
      public int runID;
      public String eventName;
      public float runTime;
      public boolean successful;
   }

   public class Task
   {
      public int taskID;
      public String name;
   }

   public static void main(String[] args)
   {
      SQLDatabaseManager test = new SQLDatabaseManager();
      System.out.println("MY ID NUMBER IS "+test.saveOperator("john").operatorID);
   }

}
