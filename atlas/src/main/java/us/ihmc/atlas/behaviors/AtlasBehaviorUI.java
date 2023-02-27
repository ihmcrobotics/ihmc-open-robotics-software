package us.ihmc.atlas.behaviors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.behaviors.javafx.JavaFXBehaviorUI;
import us.ihmc.behaviors.javafx.JavaFXBehaviorUIRegistry;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;

public class AtlasBehaviorUI
{
   public AtlasBehaviorUI()
   {
      DRCRobotModel drcRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.REAL_ROBOT, false);

      JavaFXApplicationCreator.createAJavaFXApplication();

      JavaFXBehaviorUI.createInterprocess(JavaFXBehaviorUIRegistry.DEFAULT_BEHAVIORS,
                                          drcRobotModel,
                                          NetworkParameters.getHost(NetworkParameterKeys.networkManager));
   }

   public static void main(String[] args)
   {
      new AtlasBehaviorUI();
   }
}