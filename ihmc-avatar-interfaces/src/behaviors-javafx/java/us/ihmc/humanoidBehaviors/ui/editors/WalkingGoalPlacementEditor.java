package us.ihmc.humanoidBehaviors.ui.editors;

import javafx.scene.Group;
import javafx.scene.SubScene;
import javafx.scene.control.Button;
import javafx.scene.paint.Color;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.humanoidBehaviors.ui.behaviors.PoseGraphic;
import us.ihmc.humanoidBehaviors.ui.model.FXUIActionMap;
import us.ihmc.humanoidBehaviors.ui.model.FXUITrigger;

import java.util.function.Consumer;

public class WalkingGoalPlacementEditor extends Group
{
   private PoseGraphic goalGraphic;

   private SnappedPositionEditor snappedPositionEditor;
   private OrientationYawEditor orientationYawEditor;

   private FXUIActionMap placeGoalActionMap;

   public void init(SubScene sceneNode, Button placeGoalButton, Consumer<Pose3D> placedGoalConsumer)
   {
      goalGraphic = new PoseGraphic("Goal", Color.CADETBLUE, 0.03);
      getChildren().add(goalGraphic);

      snappedPositionEditor = new SnappedPositionEditor(sceneNode);
      orientationYawEditor = new OrientationYawEditor(sceneNode);

      placeGoalActionMap = new FXUIActionMap(startAction ->
      {
         placeGoalButton.setDisable(true);
         snappedPositionEditor.edit(SnappedPositionEditor.EditMode.BOTH, goalGraphic, exitType ->
         {
            placeGoalActionMap.triggerAction(exitType);
         });
      });
      placeGoalActionMap.mapAction(FXUITrigger.POSITION_LEFT_CLICK, trigger ->
      {
         orientationYawEditor.edit(goalGraphic, exitType -> placeGoalActionMap.triggerAction(exitType));
      });
      placeGoalActionMap.mapAction(FXUITrigger.ORIENTATION_LEFT_CLICK, trigger ->
      {
         placedGoalConsumer.accept(new Pose3D(goalGraphic.getPose()));

         placeGoalButton.setDisable(false);
      });
      placeGoalActionMap.mapAction(FXUITrigger.RIGHT_CLICK, trigger ->
      {
         placeGoalButton.setDisable(false);
      });
   }

   public void startGoalPlacement()
   {
      placeGoalActionMap.start();
   }
}
