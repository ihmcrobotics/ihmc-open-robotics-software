package us.ihmc.humanoidBehaviors.ui.behaviors;


import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.Group;
import javafx.scene.SubScene;
import javafx.scene.control.CheckBox;
import javafx.scene.paint.Color;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.exploreArea.ExploreAreaBehavior;
import us.ihmc.humanoidBehaviors.ui.graphics.PositionGraphic;
import us.ihmc.messager.Messager;

public class ExploreAreaBehaviorUIController extends Group
{
   @FXML
   private CheckBox exploreAreaCheckBox;

//   @FXML
//   private Button singleSupportButton;

   private Messager behaviorMessager;

   public void init(SubScene sceneNode, Messager behaviorMessager, DRCRobotModel robotModel)
   {
      this.behaviorMessager = behaviorMessager;
      behaviorMessager.registerTopicListener(ExploreAreaBehavior.ExploreAreaBehaviorAPI.ObservationPosition, result -> Platform.runLater(() -> displayObservationPosition(result)));
   }

   @FXML
   public void exploreArea()
   {
      behaviorMessager.submitMessage(ExploreAreaBehavior.ExploreAreaBehaviorAPI.ExploreArea, exploreAreaCheckBox.isSelected());
   }

//   @FXML
//   public void requestSingleSupport()
//   {
//      behaviorMessager.submitMessage(FancyPosesBehavior.API.GoToSingleSupport, true);
//   }

  public void displayObservationPosition(Point3D observationPosition)
  {
     PositionGraphic observationPositionGraphic = new PositionGraphic(Color.AZURE, 0.04);
     observationPositionGraphic.setPosition(observationPosition);
     getChildren().add(observationPositionGraphic.getNode());
  }
}
