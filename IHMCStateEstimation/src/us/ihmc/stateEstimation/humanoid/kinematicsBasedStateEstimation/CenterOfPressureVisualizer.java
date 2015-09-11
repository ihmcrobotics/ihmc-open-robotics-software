package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.plotting.YoArtifactPosition;


public class CenterOfPressureVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final SideDependentList<YoFramePoint> footRawCoPPositionsInWorld = new SideDependentList<>();
   private final YoFramePoint overallRawCoPPositionInWorld;
   private final FramePoint2d tempRawCoP2d = new FramePoint2d();
   private final FramePoint tempRawCoP = new FramePoint();
   private final Wrench tempWrench = new Wrench();
   private final SideDependentList<FootSwitchInterface> footSwitches;

   public CenterOfPressureVisualizer(SideDependentList<FootSwitchInterface> footSwitches,
         YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.footSwitches = footSwitches;

      for (RobotSide robotSide : RobotSide.values)
      {
         String side = robotSide.getCamelCaseNameForMiddleOfExpression();
         YoFramePoint rawCoPPositionInWorld = new YoFramePoint("raw" + side + "CoPPositionsInWorld", worldFrame, registry);
         footRawCoPPositionsInWorld.put(robotSide, rawCoPPositionInWorld);

         YoGraphicPosition copDynamicGraphic = new YoGraphicPosition("Meas " + side + "CoP", rawCoPPositionInWorld, 0.008, YoAppearance.DarkRed(), GraphicType.DIAMOND);
         YoArtifactPosition copArtifact = copDynamicGraphic.createArtifact();
         yoGraphicsListRegistry.registerArtifact("StateEstimator", copArtifact);
      }

      overallRawCoPPositionInWorld = new YoFramePoint("overallRawCoPPositionInWorld", worldFrame, registry);
      YoGraphicPosition overallRawCoPDynamicGraphic = new YoGraphicPosition("Meas CoP", overallRawCoPPositionInWorld, 0.015, YoAppearance.DarkRed(), GraphicType.DIAMOND);
      YoArtifactPosition overallRawCoPArtifact = overallRawCoPDynamicGraphic.createArtifact();
      yoGraphicsListRegistry.registerArtifact("StateEstimator", overallRawCoPArtifact);

      parentRegistry.addChild(registry);
   }

   public void update()
   {
      if (footRawCoPPositionsInWorld != null)
      {
         overallRawCoPPositionInWorld.setToZero();
         double totalFootForce = 0.0;

         for (RobotSide robotSide : RobotSide.values)
         {
            footSwitches.get(robotSide).computeAndPackCoP(tempRawCoP2d);
            tempRawCoP.setIncludingFrame(tempRawCoP2d.getReferenceFrame(), tempRawCoP2d.getX(), tempRawCoP2d.getY(), 0.0);
            tempRawCoP.changeFrame(worldFrame);
            footRawCoPPositionsInWorld.get(robotSide).set(tempRawCoP);

            footSwitches.get(robotSide).computeAndPackFootWrench(tempWrench);
            double singleFootForce = tempWrench.getLinearPartZ();
            totalFootForce += singleFootForce;
            tempRawCoP.scale(singleFootForce);
            overallRawCoPPositionInWorld.add(tempRawCoP);
         }

         overallRawCoPPositionInWorld.scale(1.0 / totalFootForce);
      }
   }

   public void hide()
   {
      for (RobotSide robotSide : RobotSide.values)
         footRawCoPPositionsInWorld.get(robotSide).setToNaN();
      overallRawCoPPositionInWorld.setToNaN();
   }
}
