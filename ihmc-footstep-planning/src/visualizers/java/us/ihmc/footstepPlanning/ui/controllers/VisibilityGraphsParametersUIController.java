package us.ihmc.footstepPlanning.ui.controllers;

import javafx.fxml.FXML;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import javafx.scene.control.SpinnerValueFactory.IntegerSpinnerValueFactory;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphParametersKeys;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.robotEnvironmentAwareness.ui.properties.JavaFXStoredPropertyMap;

public class VisibilityGraphsParametersUIController
{
   private JavaFXMessager messager;
   private VisibilityGraphsParametersBasics planningParameters;

   @FXML
   private CheckBox performPostProcessingNodeShifting;
   @FXML
   private CheckBox introduceMidpointInPostProcessing;
   @FXML
   private CheckBox computeOrientationsToAvoidObstacles;
   @FXML
   private CheckBox returnBestEffortSolution;
   @FXML
   private CheckBox includePreferredExtrusions;

   @FXML
   private Spinner<Double> clusterResolution;
   @FXML
   private Spinner<Double> maxInterRegionConnectionLength;
   @FXML
   private Spinner<Double> lengthForLongInterRegionEdge;
   @FXML
   private Spinner<Double> explorationDistanceFromStartGoal;

   @FXML
   private Spinner<Double> navigableExtrusionDistance;
   @FXML
   private Spinner<Double> preferredNavigableExtrusionDistance;
   @FXML
   private Spinner<Double> obstacleExtrusionDistance;
   @FXML
   private Spinner<Double> preferredObstacleExtrusionDistance;
   @FXML
   private Spinner<Double> obstacleExtrusionDistanceIfNotTooHighToStep;
   @FXML
   private Spinner<Double> tooHighToStepDistance;
//   @FXML
//   private Spinner<Double> canEasilyStepOver;
//   @FXML
//   private Spinner<Double> canDuckUnderHeight;


   @FXML
   private Spinner<Double> planarRegionMinArea;
   @FXML
   private Spinner<Integer> planarRegionMinSize;
   @FXML
   private Spinner<Double> regionOrthogonalAngle;
   @FXML
   private Spinner<Double> searchHostRegionEpsilon;
   @FXML
   private Spinner<Double> normalZThresholdForAccessibleRegions;

   @FXML
   private Spinner<Double> heuristicWeight;
   @FXML
   private Spinner<Double> distanceWeight;
   @FXML
   private Spinner<Double> elevationWeight;
   @FXML
   private Spinner<Double> occludedGoalEdgeWeight;
   @FXML
   private Spinner<Double> weightForInterRegionEdge;
   @FXML
   private Spinner<Double> weightForNonPreferredEdge;

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
   }

   public void setVisbilityGraphsParameters(VisibilityGraphsParametersBasics parameters)
   {
      this.planningParameters = parameters;
   }

   private void setupControls()
   {
      clusterResolution.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.0, 0.2, 0.02));
      maxInterRegionConnectionLength.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.5, 0.45, 0.05));
      lengthForLongInterRegionEdge.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.5, 0.3, 0.05));

      navigableExtrusionDistance.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.5, 0.02, 0.01));
      preferredNavigableExtrusionDistance.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.5, 0.2, 0.02));
      obstacleExtrusionDistance.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.5, 0.5, 0.05));
      preferredObstacleExtrusionDistance.setValueFactory(new DoubleSpinnerValueFactory(0.0, 2.5, 1.0, 0.5));
      obstacleExtrusionDistanceIfNotTooHighToStep.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.5, 0.05, 0.02));
      tooHighToStepDistance.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.6, 0.28, 0.02));
//      canEasilyStepOver.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.25, 0.0, 0.03));
//      canDuckUnderHeight.setValueFactory(new DoubleSpinnerValueFactory(1.0, 10.0, 0.0, 2.0));

      planarRegionMinArea.setValueFactory(new DoubleSpinnerValueFactory(0, 10.0, 0.0, 0.1));
      planarRegionMinSize.setValueFactory(new IntegerSpinnerValueFactory(0, 100, 0, 1));
      regionOrthogonalAngle.setValueFactory(new DoubleSpinnerValueFactory(0.0, Math.PI / 2.0, Math.toRadians(75.0), Math.toRadians(5.0)));
      searchHostRegionEpsilon.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.0, 0.03, 0.05));
      normalZThresholdForAccessibleRegions.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.0,  Math.cos(Math.toRadians(30.0)), Math.sin(Math.toRadians(5.0))));
      explorationDistanceFromStartGoal.setValueFactory(new DoubleSpinnerValueFactory(0, Double.POSITIVE_INFINITY, 1.0, 0.5));

      heuristicWeight.setValueFactory(new DoubleSpinnerValueFactory(0.0, 20.0, 3.0, 0.25));
      distanceWeight.setValueFactory(new DoubleSpinnerValueFactory(0.0, 10.0, 1.5, 1.1));
      elevationWeight.setValueFactory(new DoubleSpinnerValueFactory(0.0, 5.0, 0.0, 0.1));
      occludedGoalEdgeWeight.setValueFactory(new DoubleSpinnerValueFactory(0.0, 100.0, 50.0, 1.0));
      weightForInterRegionEdge.setValueFactory(new DoubleSpinnerValueFactory(0.0, 10.0, 1.5, 0.1));
      weightForNonPreferredEdge.setValueFactory(new DoubleSpinnerValueFactory(0.0, 10.0, 1.5, 0.1));
   }

   public void bindControls()
   {
      setupControls();

      JavaFXStoredPropertyMap javaFXStoredPropertyMap = new JavaFXStoredPropertyMap(planningParameters);

      javaFXStoredPropertyMap.put(performPostProcessingNodeShifting, VisibilityGraphParametersKeys.performPostProcessingNodeShifting);
      javaFXStoredPropertyMap.put(introduceMidpointInPostProcessing, VisibilityGraphParametersKeys.introduceMidpointsInPostProcessing);
      javaFXStoredPropertyMap.put(computeOrientationsToAvoidObstacles, VisibilityGraphParametersKeys.computeOrientationsToAvoidObstacles);
      javaFXStoredPropertyMap.put(returnBestEffortSolution, VisibilityGraphParametersKeys.returnBestEffortSolution);
      javaFXStoredPropertyMap.put(includePreferredExtrusions, VisibilityGraphParametersKeys.includePreferredExtrusions);

      javaFXStoredPropertyMap.put(maxInterRegionConnectionLength, VisibilityGraphParametersKeys.maxInterRegionConnectionLength);
      javaFXStoredPropertyMap.put(lengthForLongInterRegionEdge, VisibilityGraphParametersKeys.lengthForLongInterRegionEdge);
      javaFXStoredPropertyMap.put(normalZThresholdForAccessibleRegions, VisibilityGraphParametersKeys.normalZThresholdForAccessibleRegions);
      javaFXStoredPropertyMap.put(navigableExtrusionDistance, VisibilityGraphParametersKeys.navigableExtrusionDistance);
      javaFXStoredPropertyMap.put(preferredNavigableExtrusionDistance, VisibilityGraphParametersKeys.preferredNavigableExtrusionDistance);
      javaFXStoredPropertyMap.put(obstacleExtrusionDistance, VisibilityGraphParametersKeys.obstacleExtrusionDistance);
      javaFXStoredPropertyMap.put(preferredObstacleExtrusionDistance, VisibilityGraphParametersKeys.preferredObstacleExtrusionDistance);
      javaFXStoredPropertyMap.put(obstacleExtrusionDistanceIfNotTooHighToStep, VisibilityGraphParametersKeys.obstacleExtrusionDistanceIfNotTooHighToStep);
      javaFXStoredPropertyMap.put(tooHighToStepDistance, VisibilityGraphParametersKeys.tooHighToStepDistance);
//      javaFXStoredPropertyMap.put(canEasilyStepOver, VisibilityGraphParametersKeys.canEasilyStepOverHeight);
//      javaFXStoredPropertyMap.put(canDuckUnderHeight, VisibilityGraphParametersKeys.canDuckUnderHeight);
      javaFXStoredPropertyMap.put(clusterResolution, VisibilityGraphParametersKeys.clusterResolution);
      javaFXStoredPropertyMap.put(explorationDistanceFromStartGoal, VisibilityGraphParametersKeys.explorationDistanceFromStartGoal);
      javaFXStoredPropertyMap.put(planarRegionMinArea, VisibilityGraphParametersKeys.planarRegionMinArea);
      javaFXStoredPropertyMap.put(planarRegionMinSize, VisibilityGraphParametersKeys.planarRegionMinSize);
      javaFXStoredPropertyMap.put(regionOrthogonalAngle, VisibilityGraphParametersKeys.regionOrthogonalAngle);
      javaFXStoredPropertyMap.put(searchHostRegionEpsilon, VisibilityGraphParametersKeys.searchHostRegionEpsilon);

      javaFXStoredPropertyMap.put(heuristicWeight, VisibilityGraphParametersKeys.heuristicWeight);
      javaFXStoredPropertyMap.put(distanceWeight, VisibilityGraphParametersKeys.distanceWeight);
      javaFXStoredPropertyMap.put(elevationWeight, VisibilityGraphParametersKeys.elevationWeight);
      javaFXStoredPropertyMap.put(occludedGoalEdgeWeight, VisibilityGraphParametersKeys.occludedGoalEdgeWeight);
      javaFXStoredPropertyMap.put(weightForInterRegionEdge, VisibilityGraphParametersKeys.weightForInterRegionEdge);
      javaFXStoredPropertyMap.put(weightForNonPreferredEdge, VisibilityGraphParametersKeys.weightForNonPreferredEdge);

      // set messager updates to update all stored properties and select JavaFX properties
      messager.registerTopicListener(FootstepPlannerMessagerAPI.VisibilityGraphsParameters, parameters ->
      {
         planningParameters.set(parameters);

         javaFXStoredPropertyMap.copyStoredToJavaFX();
      });

      // set JavaFX user input to update stored properties and publish messager message
      javaFXStoredPropertyMap.bindStoredToJavaFXUserInput();
      javaFXStoredPropertyMap.bindToJavaFXUserInput(() -> publishParameters());
   }

   private void publishParameters()
   {
      messager.submitMessage(FootstepPlannerMessagerAPI.VisibilityGraphsParameters, planningParameters);
   }


}
