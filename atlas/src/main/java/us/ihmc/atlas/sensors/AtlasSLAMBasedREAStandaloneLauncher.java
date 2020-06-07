package us.ihmc.atlas.sensors;

import java.util.List;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotEnvironmentAwareness.ui.SLAMBasedEnvironmentAwarenessUI;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

public class AtlasSLAMBasedREAStandaloneLauncher extends Application
{
   private SLAMBasedEnvironmentAwarenessUI ui;
   private AtlasSLAMModule module;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      DRCRobotModel drcRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.REAL_ROBOT, false);

      //TODO: run atlas and check the footstep coming well.
      RobotContactPointParameters<RobotSide> contactPointParameters = drcRobotModel.getContactPointParameters();
      SideDependentList<List<Point2D>> defaultContactPoints = new SideDependentList<>();
      for (RobotSide side : RobotSide.values)
      {
         defaultContactPoints.put(side, contactPointParameters.getControllerFootGroundContactPoints().get(side));
      }
      ui = SLAMBasedEnvironmentAwarenessUI.creatIntraprocessUI(primaryStage, defaultContactPoints);
      module = AtlasSLAMModule.createIntraprocessModule(drcRobotModel);

      ui.show();
      module.start();
   }

   @Override
   public void stop() throws Exception
   {
      ui.stop();
      module.stop();

      Platform.exit();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
