package us.ihmc.robotDataLogger.handshake;

import us.ihmc.graphicsDescription.yoGraphics.RemoteYoGraphicFactory;
import us.ihmc.robotics.graphics.YoGraphicPlanarRegionsList;
import us.ihmc.robotics.graphics.YoGraphicPolynomial3D;

public class RoboticsRemoteYoGraphicFactory extends RemoteYoGraphicFactory
{
   public RoboticsRemoteYoGraphicFactory()
   {
      registerBuilder(YoGraphicPlanarRegionsList.class,
                      (name, vars, consts, appearance) -> YoGraphicPlanarRegionsList.createAsRemoteYoGraphic(name, vars, consts));
      registerBuilder(YoGraphicPolynomial3D.class, (name, vars, consts, appearance) -> YoGraphicPolynomial3D.createAsRemoteYoGraphic(name, vars, consts));
   }
}
