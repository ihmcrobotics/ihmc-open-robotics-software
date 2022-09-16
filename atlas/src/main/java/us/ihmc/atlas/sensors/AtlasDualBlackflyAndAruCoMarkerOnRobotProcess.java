package us.ihmc.atlas.sensors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.colorVision.DualBlackflyAndAruCoMarkerOnRobotProcess;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.perception.OpenCVArUcoMarker;

import java.util.ArrayList;

public class AtlasDualBlackflyAndAruCoMarkerOnRobotProcess
{
   public AtlasDualBlackflyAndAruCoMarkerOnRobotProcess()
   {
      AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.REAL_ROBOT);
      ArrayList<OpenCVArUcoMarker> arUcoMarkersToTrack = new ArrayList<>();
      arUcoMarkersToTrack.add(new OpenCVArUcoMarker(0, 0.1680));
      new DualBlackflyAndAruCoMarkerOnRobotProcess(atlasRobotModel, arUcoMarkersToTrack);
   }

   public static void main(String[] args)
   {
      new AtlasDualBlackflyAndAruCoMarkerOnRobotProcess();
   }
}
