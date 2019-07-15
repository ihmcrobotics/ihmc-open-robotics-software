package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.fxml.FXML;
import javafx.scene.control.CheckBox;
import us.ihmc.humanoidBehaviors.exploreArea.ExploreAreaBehavior;
import us.ihmc.messager.Messager;

public class ExploreAreaBehaviorUIController
{
   @FXML
   private CheckBox exploreAreaCheckBox;

//   @FXML
//   private Button singleSupportButton;

   private Messager behaviorMessager;

   public void init(Messager behaviorMessager)
   {
      this.behaviorMessager = behaviorMessager;
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

  
}
