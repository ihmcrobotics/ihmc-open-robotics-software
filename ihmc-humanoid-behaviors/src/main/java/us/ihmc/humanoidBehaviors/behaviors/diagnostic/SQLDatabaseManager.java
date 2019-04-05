package us.ihmc.humanoidBehaviors.behaviors.diagnostic;

import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.sql.Statement;
import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.log.LogTools;

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

   public SQLDatabaseManager(String username, String password, String host, String port, String databaseName)
   {
      this.databaseUsername = username;
      this.databasePassword = password;
      this.database_host = host;
      this.database_port = port;
      this.database_name = databaseName;

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
         LogTools.error("Could not connect to database! DB thread will not run");
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
      if(connected && !shutdown)
      {
         if()
         try
         {
            isNewDataAvailable = true;

            while (isNewDataAvailable)
            {
               updateInsertPreparedStatement();

               timeSeriesInsertPreparedStatement.addBatch();

               batchCount++;

               if (batchCount >= BATCHES_BEFORE_EXECUTE)
               {
                  executionTimer.startMeasurement();

                  timeSeriesInsertPreparedStatement.executeBatch();

                  executionTimer.stopMeasurement();

                  batchCount = 0;

               }

               isNewDataAvailable = true;
            }
         }
         catch (SQLException e)
         {
            LogTools.error("Error executing database update!");
            e.printStackTrace();
         }
      }
      


   }

   public boolean insert(String statement)
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
      }
      catch (Exception e)
      {
         System.err.println(e.getClass().getName() + ": " + e.getMessage());
      }
   }

   public void sendSQLStatement(String statement)
   {
      insertStatement = connection.prepareStatement(statement);
      exerciseStatement = connection.prepareStatement(getExerciseSQLString);
      connectionSuccessful = true;
   }

   public boolean getExerciseFromDatabase(long exerciseId)
   {
      try
      {
         getExerciseInnerJoinPreparedStatement.clearParameters();
         getExerciseInnerJoinPreparedStatement.setLong(1, exerciseId);

         ResultSet resultSet = getExerciseInnerJoinPreparedStatement.executeQuery();
         resultSet.next();

         String exerciseName = resultSet.getString(1);
         boolean isLoaded = resultSet.getBoolean(2);

         // exerciseToPack.setName(exerciseName);
         // exerciseToPack.setLoaded(isLoaded);

         return true;
      }
      catch (SQLException e)
      {
         LogTools.error("Could not execute prepared statement!");
         e.printStackTrace();

         return false;
      }
   }

   private void shutdown()
   {
      try
      {
         LogTools.info("Beginning database thread shutdown...");
         finalizeRecording();

         LogTools.info("Closing database connection...");
         databaseConnection.close();
      }
      catch (SQLException e)
      {
         LogTools.error("Error closing database connections");
         e.printStackTrace();
         System.exit(1);
      }
   }

   private void finalizeRecording() throws SQLException
   {
      shutdown = true;
      LogTools.info("Processing remaining buffered data...");
      synchronized (timeSeriesInsertPreparedStatement)
      {
         while (isNewDataAvailable)
         {
            updateInsertPreparedStatement();
            isNewDataAvailable = true;
         }

         timeSeriesInsertPreparedStatement.executeBatch();
      }

      LogTools.info("All data processed, safe to finish shut down.");
   }

}
