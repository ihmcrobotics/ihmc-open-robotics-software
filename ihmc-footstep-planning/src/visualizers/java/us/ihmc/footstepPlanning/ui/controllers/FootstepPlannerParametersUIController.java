package us.ihmc.footstepPlanning.ui.controllers;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ComputePath;

import javafx.fxml.FXML;
import javafx.scene.control.CheckBox;
import javafx.scene.control.TableView;
import javafx.scene.shape.Rectangle;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.javafx.parameter.JavaFXStoredPropertyMap;
import us.ihmc.javafx.parameter.StoredPropertyTableViewWrapper;
import us.ihmc.javafx.parameter.StoredPropertyTableViewWrapper.ParametersTableRow;
import us.ihmc.log.LogTools;
import us.ihmc.messager.javafx.JavaFXMessager;

public class FootstepPlannerParametersUIController
{
   private JavaFXMessager messager;
   private FootstepPlannerParametersBasics planningParameters;
   private JavaFXStoredPropertyMap javaFXStoredPropertyMap;
   private final StepShapeManager stepShapeManager = new StepShapeManager();
   private StoredPropertyTableViewWrapper tableViewWrapper;

   @FXML
   private Rectangle stepShape;
   @FXML
   private Rectangle stanceFootShape;
   @FXML
   private Rectangle swingFootShape;
   @FXML
   private Rectangle clearanceBox;
   @FXML
   private CheckBox autoReplan;
   private static final double footWidth = 0.15;
   private static final double footLength = 0.25;
   private static final double leftFootOriginX = 30;
   private static final double leftFootOriginY = 100;
   private static final double metersToPixel = 200;

   @FXML
   private TableView<ParametersTableRow> parameterTable;

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
   }

   public void setPlannerParameters(FootstepPlannerParametersBasics parameters)
   {
      this.planningParameters = parameters;
      javaFXStoredPropertyMap = new JavaFXStoredPropertyMap(planningParameters);
      stepShapeManager.update();
   }

   public void bindControls()
   {
      tableViewWrapper = new StoredPropertyTableViewWrapper(380.0, 260.0, 4, parameterTable, javaFXStoredPropertyMap, 5);
      tableViewWrapper.setTableUpdatedCallback(() ->
      {
         if (autoReplan.isSelected())
         {
            LogTools.info("Auto replan active. Replanning...");
            messager.submitMessage(ComputePath, true);
         }

         messager.submitMessage(FootstepPlannerMessagerAPI.PlannerParameters, planningParameters);
      });

      // set messager updates to update all stored properties and select JavaFX properties
      messager.addTopicListener(FootstepPlannerMessagerAPI.PlannerParameters, parameters ->
      {
         if (!parameters.equals(planningParameters)) // stop feedback loop
         {
            planningParameters.set(parameters);
            javaFXStoredPropertyMap.copyStoredToJavaFX();
            stepShapeManager.update();
         }
      });

      // these dimensions work best for valkyrie
      stanceFootShape.setHeight(footLength * metersToPixel);
      stanceFootShape.setWidth(footWidth * metersToPixel);
      stanceFootShape.setLayoutX(leftFootOriginX);
      stanceFootShape.setLayoutY(leftFootOriginY);

      swingFootShape.setHeight(footLength * metersToPixel);
      swingFootShape.setWidth(footWidth * metersToPixel);
      swingFootShape.setLayoutY(leftFootOriginY);
   }

   public void onPrimaryStageLoaded()
   {
      tableViewWrapper.removeHeader();
   }

   @FXML
   public void saveToFile()
   {
      planningParameters.save();
   }

   @FXML
   public void loadFile()
   {
      tableViewWrapper.loadNewFile();
      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerParameters, planningParameters);
   }

   private class StepShapeManager
   {
      double minStepYaw, maxStepYaw, minStepWidth, maxStepWidth, minStepLength, maxStepLength, worstYaw;

      void update()
      {
         double minStepYaw = planningParameters.getMinimumStepYaw();
         double maxStepYaw = planningParameters.getMaximumStepYaw();
         double minStepWidth = planningParameters.getMinimumStepWidth();
         double maxStepWidth = planningParameters.getMaximumStepWidth();
         double minStepLength = planningParameters.getMinimumStepLength();
         double maxStepLength = planningParameters.getMaximumStepReach();
         double worstYaw = maxStepYaw > Math.abs(minStepYaw) ? maxStepYaw : minStepYaw;

         if (EuclidCoreTools.epsilonEquals(minStepYaw, this.minStepYaw, 1e-3) &&
             EuclidCoreTools.epsilonEquals(maxStepYaw, this.maxStepYaw, 1e-3) &&
             EuclidCoreTools.epsilonEquals(minStepWidth, this.minStepWidth, 1e-3) &&
             EuclidCoreTools.epsilonEquals(maxStepWidth, this.maxStepWidth, 1e-3) &&
             EuclidCoreTools.epsilonEquals(minStepLength, this.minStepLength, 1e-3) &&
             EuclidCoreTools.epsilonEquals(maxStepLength, this.maxStepLength, 1e-3) &&
             EuclidCoreTools.epsilonEquals(worstYaw, this.worstYaw, 1e-3))
            return;

         this.minStepYaw = minStepYaw;
         this.maxStepYaw = maxStepYaw;
         this.minStepWidth = minStepWidth;
         this.maxStepWidth = maxStepWidth;
         this.minStepLength = minStepLength;
         this.maxStepLength = maxStepLength;
         this.worstYaw = worstYaw;
         setStepShape();
      }

      void setStepShape()
      {
         // TODO update for new clearance param

//         double minClearance = planningParameters.getMinClearanceFromStance();
//
//         double footCenterX = leftFootOriginX + 0.5 * footWidth * metersToPixel;
//         double footCenterY = leftFootOriginY + 0.5 * footLength * metersToPixel;
//
//         double furthestIn = Math.max(minStepWidth, minYClearance) * metersToPixel;
//
//         double width = maxStepWidth - minStepWidth;
//         double height = maxStepLength - minStepLength;
//         double xCenterInPanel = footCenterX + metersToPixel * minStepWidth;
//         double yCenterInPanel = footCenterY - metersToPixel * maxStepLength;
//
//         stepShape.setLayoutX(xCenterInPanel);
//         stepShape.setWidth(metersToPixel * width);
//         stepShape.setLayoutY(yCenterInPanel);
//         stepShape.setHeight(metersToPixel * height);
//
//         swingFootShape.setLayoutX(footCenterX + furthestIn - 0.5 * footWidth * metersToPixel);
//         swingFootShape.setRotate(Math.toDegrees(worstYaw));
//
//         clearanceBox.setLayoutX(leftFootOriginX + (0.5 * footWidth - minYClearance) * metersToPixel);
//         clearanceBox.setLayoutY(leftFootOriginY + (0.5 * footLength - minXClearance) * metersToPixel);
//         clearanceBox.setWidth(metersToPixel * (minYClearance * 2.0));
//         clearanceBox.setHeight(metersToPixel * (minXClearance * 2.0));
      }
   }
}
