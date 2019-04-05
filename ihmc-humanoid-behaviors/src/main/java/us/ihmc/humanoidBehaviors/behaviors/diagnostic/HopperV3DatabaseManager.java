package us.ihmc.humanoidBehaviors.behaviors.diagnostic;

import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.sql.SQLException;

import us.ihmc.affinity.CPUTopology;
import us.ihmc.commons.Conversions;
import us.ihmc.log.LogTools;
import us.ihmc.realtime.PeriodicParameters;
import us.ihmc.realtime.PeriodicRealtimeThread;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.realtime.RealtimeThread;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 */
public class HopperV3DatabaseManager
{

   private final String databaseURL;
   private final String postgresUser;
   private final String postgresPassword;

   private final String insertSQLString;
   private final String getExerciseSQLString;

   private final Connection databaseConnection;
   private final PreparedStatement timeSeriesInsertPreparedStatement;
   private final PreparedStatement getExerciseInnerJoinPreparedStatement;

   private final boolean databaseConnectionReady;

   private boolean connected = false;

   private volatile boolean isNewDataAvailable = false;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ExecutionTimer executionTimer;

   private static final int BATCHES_BEFORE_EXECUTE = 1000;
   private int batchCount = 0;

   private volatile boolean shutdown = false;
   private final boolean usePinnedRealtimeThreadForDatabase;

   public HopperV3DatabaseManager( YoVariableRegistry parentRegistry)
   {

      if (parentRegistry != null)
      {
         executionTimer = new ExecutionTimer("databaseTransactionTimer", registry);
      }
      else
      {
         executionTimer = null;
      }

      String use_high_priority_db_thread = System.getenv("PIN_DATABASE_THREAD");

      usePinnedRealtimeThreadForDatabase = use_high_priority_db_thread != null && Boolean.parseBoolean(use_high_priority_db_thread);

      String cobalt_database_username = System.getenv("COBALT_DATABASE_USERNAME");
      String cobalt_database_password = System.getenv("COBALT_DATABASE_PASSWORD");
      String cobalt_database_host = System.getenv("COBALT_DATABASE_HOST");
      String cobalt_database_port = System.getenv("COBALT_DATABASE_PORT");
      String cobalt_timeseries_database_name = System.getenv("COBALT_TIMESERIES_DATABASE_NAME");
      String cobalt_hypertable_name = System.getenv("COBALT_HYPERTABLE_NAME");

      if (cobalt_database_username == null || cobalt_database_password == null || cobalt_database_host == null || cobalt_database_port == null
            || cobalt_timeseries_database_name == null || cobalt_hypertable_name == null)
      {
         databaseConnectionReady = false;
         databaseURL = null;
         postgresUser = null;
         postgresPassword = null;
         insertSQLString = null;

         databaseConnection = null;
         timeSeriesInsertPreparedStatement = null;
         getExerciseInnerJoinPreparedStatement = null;
         getExerciseSQLString = null;

         LogTools.error("Database connection information not properly set, database thread will not run.");
      }
      else
      {
         databaseURL = "jdbc:postgresql://" + cobalt_database_host + ":" + cobalt_database_port + "/" + cobalt_timeseries_database_name;
         postgresUser = cobalt_database_username;
         postgresPassword = cobalt_database_password;

         insertSQLString = "INSERT INTO " + cobalt_hypertable_name + "(event_id, time, left_distance_pivots, right_distance_pivots, "
               + "left_velocity_pivots, right_velocity_pivots, left_bar_pivot_angle, "
               + "right_bar_pivot_angle, left_lower_joint_pitch, right_lower_joint_pitch, "
               + "left_lower_joint_roll, right_lower_joint_roll, left_output_joint_angle, " + "right_output_joint_angle, "
               + "left_joint_torque, right_joint_torque, left_output_force, right_output_force, "
               + "left_force_plate_cop, right_force_plate_cop, left_force_plate_load, right_force_plate_load) "
               + "VALUES(?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, point(?, ?), point(?, ?), ?, ?)";

         getExerciseSQLString = "SELECT exercises.name, exercises.is_loaded FROM exercises, user_exercise_event_history WHERE user_exercise_event_history.id = ? AND exercises.id = user_exercise_event_history.exercise_id";

         Connection connection;
         PreparedStatement insertStatement;
         PreparedStatement exerciseStatement;
         boolean connectionSuccessful;

         try
         {
            connection = DriverManager.getConnection(databaseURL, postgresUser, postgresPassword);
            insertStatement = connection.prepareStatement(insertSQLString);
            exerciseStatement = connection.prepareStatement(getExerciseSQLString);
            connectionSuccessful = true;
         }
         catch (SQLException e)
         {
            connection = null;
            insertStatement = null;
            exerciseStatement = null;
            connectionSuccessful = false;
            LogTools.error("Could not connect to database! DB thread will not run");
            e.printStackTrace();
         }

         this.databaseConnection = connection;
         this.timeSeriesInsertPreparedStatement = insertStatement;
         this.getExerciseInnerJoinPreparedStatement = exerciseStatement;
         this.databaseConnectionReady = connectionSuccessful;
      }

      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }
   }

   public void start()
   {
      if (!databaseConnectionReady)
      {
         return;
      }

      if(usePinnedRealtimeThreadForDatabase)
      {
         PriorityParameters dbPriorityParameters = new PriorityParameters(50);
         PeriodicParameters periodicParameters = new PeriodicParameters(Conversions.secondsToNanoseconds(1.0));
         RealtimeThread thread = new PeriodicRealtimeThread(dbPriorityParameters, periodicParameters, this::update, "hopper-v3-database");
         thread.setAffinity(new CPUTopology().getPackage(0).getCore(2).getDefaultProcessor());
         thread.start();
      }
      else
      {
         Thread thread = new Thread(this::update, "hopper-v3-database");
         thread.setDaemon(true);
         thread.start();
      }

      connected = true;
      Runtime.getRuntime().addShutdownHook(new Thread(HopperV3DatabaseManager.this::shutdown));
   }

   public boolean isConnected()
   {
      return connected;
   }

   private void update()
   {
      if(usePinnedRealtimeThreadForDatabase && !shutdown)
      {
         doUpdate();
      }
      else
      {
         while (!shutdown)
         {
            doUpdate();
            Thread.yield();
         }
      }


   }

   private void doUpdate()
   {
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

   private void shutdown()
   {
      try
      {
         LogTools.info("Beginning database thread shutdown...");
         finalizeRecording();

         LogTools.info("Closing prepared statements...");
         timeSeriesInsertPreparedStatement.close();
         getExerciseInnerJoinPreparedStatement.close();

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

   private void updateInsertPreparedStatement()
   {
      /*try
      {
         timeSeriesInsertPreparedStatement.setLong(1, configurationData.eventID);
         timeSeriesInsertPreparedStatement.setLong(2, configurationData.timestamp);

         timeSeriesInsertPreparedStatement.setDouble(3, configurationData.legConfigurationData.get(RobotSide.LEFT).distanceBetweenPivots);
         timeSeriesInsertPreparedStatement.setDouble(4, configurationData.legConfigurationData.get(RobotSide.RIGHT).distanceBetweenPivots);
      }
      catch (SQLException e)
      {
         LogTools.error("Could not execute prepared statement!");
         e.printStackTrace();
      }*/
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

   public static void main(String[] args)
   {
     // HopperV3DatabaseManager hopperV3DatabaseManager = new HopperV3DatabaseManager(null, null);
      //HopperV3Exercise hopperV3Exercise = new HopperV3Exercise();
    //  hopperV3DatabaseManager.getExerciseFromDatabase(79, hopperV3Exercise);

    //  System.out.println(hopperV3Exercise);
   }
}
