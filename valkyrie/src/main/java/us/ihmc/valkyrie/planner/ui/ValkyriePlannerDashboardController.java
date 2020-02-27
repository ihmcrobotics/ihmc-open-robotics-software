package us.ihmc.valkyrie.planner.ui;

import controller_msgs.msg.dds.ValkyrieFootstepPlanningStatus;
import javafx.animation.AnimationTimer;
import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.control.CheckBox;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import javafx.scene.control.SpinnerValueFactory.IntegerSpinnerValueFactory;
import javafx.scene.control.TextField;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.messager.Messager;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.valkyrie.planner.ValkyrieAStarFootstepPlanner;
import us.ihmc.valkyrie.planner.ValkyrieAStarFootstepPlanner.Status;
import us.ihmc.valkyrie.planner.ValkyrieAStarFootstepPlannerParameters;
import us.ihmc.valkyrie.planner.log.ValkyriePlannerLogLoader;
import us.ihmc.valkyrie.planner.log.ValkyriePlannerLogger;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class ValkyriePlannerDashboardController
{
   private Messager messager = null;

   private TimeElapsedManager timeElapsedManager = new TimeElapsedManager();
   private ValkyrieAStarFootstepPlanner planner = null;

   @FXML
   private TextField planningStatus;
   @FXML
   private TextField timeElapsed;
   @FXML
   private ComboBox<DataSetName> dataSetSelector;
   @FXML
   private Spinner<Double> timeout;
   @FXML
   private Spinner<Double> goalX;
   @FXML
   private Spinner<Double> goalY;
   @FXML
   private Spinner<Double> goalZ;
   @FXML
   private Spinner<Double> goalYaw;
   @FXML
   private Spinner<Double> waypointX;
   @FXML
   private Spinner<Double> waypointY;
   @FXML
   private Spinner<Double> waypointZ;
   @FXML
   private Spinner<Double> waypointYaw;

   @FXML
   private TextField logLoadStatus;
   @FXML
   private TextField logGenerationStatus;

   @FXML
   public void doPlanning()
   {
      messager.submitMessage(ValkyriePlannerMessagerAPI.doPlanning, true);
   }

   @FXML
   public void haltPlanning()
   {
      messager.submitMessage(ValkyriePlannerMessagerAPI.haltPlanning, true);
   }

   @FXML
   public void sendPlanningResult()
   {
      messager.submitMessage(ValkyriePlannerMessagerAPI.sendPlanningResult, true);
   }

   @FXML
   public void stopWalking()
   {
      messager.submitMessage(ValkyriePlannerMessagerAPI.stopWalking, true);
   }

   @FXML
   public void placeGoal()
   {
      messager.submitMessage(ValkyriePlannerMessagerAPI.placeGoal, true);
   }

   @FXML
   public void addWaypoint()
   {
      messager.submitMessage(ValkyriePlannerMessagerAPI.addWaypoint, true);
   }

   @FXML
   public void clearWaypoints()
   {
      messager.submitMessage(ValkyriePlannerMessagerAPI.clearWaypoints, true);
   }

   void setMessager(Messager messager)
   {
      this.messager = messager;
   }

   void setPlanner(ValkyrieAStarFootstepPlanner planner)
   {
      this.planner = planner;
   }

   public void updatePlanningStatus(ValkyrieFootstepPlanningStatus planningStatus)
   {
      Status status = Status.fromByte(planningStatus.getPlannerStatus());
      Platform.runLater(() -> this.planningStatus.setText(status.toString()));
   }

   public void setTimerEnabled(boolean enabled)
   {
      if(enabled)
         timeElapsedManager.start();
      else
         timeElapsedManager.stop();
   }

   private final AtomicBoolean generatingLog = new AtomicBoolean();
   private final AtomicBoolean loadingLog = new AtomicBoolean();

   public void generateLog()
   {
      if(planner != null && !generatingLog.get())
      {
         generatingLog.set(true);
         ValkyriePlannerLogger logger = new ValkyriePlannerLogger(planner);
         boolean success = logger.logSession();

         if(success)
            updateTextField(logGenerationStatus, logger.getLatestLogDirectory());
         else
            updateTextField(logGenerationStatus, "Error writing log");
         generatingLog.set(false);
      }
   }

   public void loadLog()
   {
      if(loadingLog.get())
         return;

      loadingLog.set(true);
      ValkyriePlannerLogLoader logLoader = new ValkyriePlannerLogLoader();
      if(logLoader.load())
      {
         messager.submitMessage(ValkyriePlannerMessagerAPI.logToLoad, logLoader.getLog());
         logLoadStatus.setText(logLoader.getLog().getLogName());
      }
      else
      {
         logLoadStatus.setText("Error loading log");
      }
      loadingLog.set(false);
   }

   private static void updateTextField(TextField textField, String text)
   {
      Platform.runLater(() -> textField.setText(text));
   }

   public void bindParameters()
   {
      Supplier<DoubleSpinnerValueFactory> doubleSpinnerValueFactory = () -> new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1);
      timeout.setValueFactory(new DoubleSpinnerValueFactory(0.0, Double.MAX_VALUE, 15.0, 1.0));
      goalX.setValueFactory(doubleSpinnerValueFactory.get());
      goalY.setValueFactory(doubleSpinnerValueFactory.get());
      goalZ.setValueFactory(doubleSpinnerValueFactory.get());
      goalYaw.setValueFactory(doubleSpinnerValueFactory.get());
      waypointX.setValueFactory(doubleSpinnerValueFactory.get());
      waypointY.setValueFactory(doubleSpinnerValueFactory.get());
      waypointZ.setValueFactory(doubleSpinnerValueFactory.get());
      waypointYaw.setValueFactory(doubleSpinnerValueFactory.get());

      dataSetSelector.getItems().setAll(DataSetName.values());
      dataSetSelector.getSelectionModel().selectedItemProperty().addListener((options, oldValue, newValue) -> messager.submitMessage(ValkyriePlannerMessagerAPI.dataSetSelected, dataSetSelector.getValue()));
   }

   private class TimeElapsedManager extends AnimationTimer
   {
      private boolean active = false;
      private final Stopwatch stopwatch = new Stopwatch();

      @Override
      public void start()
      {
         if(!active)
         {
            active = true;
            stopwatch.start();
            super.start();
         }
      }

      @Override
      public void stop()
      {
         active = false;
         super.stop();
      }

      @Override
      public void handle(long now)
      {
         timeElapsed.setText(String.format("%.2f", stopwatch.totalElapsed()));
      }
   }

   public Pose3D getGoalPose()
   {
      return new Pose3D(goalX.getValue(), goalY.getValue(), goalZ.getValue(), goalYaw.getValue(), 0.0, 0.0);
   }

   public Spinner<Double> getGoalX()
   {
      return goalX;
   }

   public Spinner<Double> getGoalY()
   {
      return goalY;
   }

   public Spinner<Double> getGoalZ()
   {
      return goalZ;
   }

   public Spinner<Double> getGoalYaw()
   {
      return goalYaw;
   }

   public Spinner<Double> getWaypointX()
   {
      return waypointX;
   }

   public Spinner<Double> getWaypointY()
   {
      return waypointY;
   }

   public Spinner<Double> getWaypointZ()
   {
      return waypointZ;
   }

   public Spinner<Double> getWaypointYaw()
   {
      return waypointYaw;
   }

   public void setGoalPose(Tuple3DReadOnly startPosition, double startYaw)
   {
      goalX.getValueFactory().setValue(startPosition.getX());
      goalY.getValueFactory().setValue(startPosition.getY());
      goalZ.getValueFactory().setValue(startPosition.getZ());
      goalYaw.getValueFactory().setValue(startYaw);
   }

   public double getTimeout()
   {
      return timeout.getValue();
   }
}
