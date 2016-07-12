package us.ihmc.quadrupedRobotics.estimator;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.TouchdownDetectorBasedFootswitch;
import us.ihmc.commonWalkingControlModules.touchdownDetector.ActuatorForceBasedTouchdownDetector;
import us.ihmc.commonWalkingControlModules.touchdownDetector.JointVelocityBasedTouchdownDetector;
import us.ihmc.commonWalkingControlModules.touchdownDetector.TouchdownDetector;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;

import java.util.ArrayList;

public class QuadrupedTouchdownDetectorBasedFootSwitch extends TouchdownDetectorBasedFootswitch
{
   private final RobotQuadrant robotQuadrant;
   private final ContactablePlaneBody foot;
   private final SDFFullRobotModel fullRobotModel;
   private final double totalRobotWeight;
   private final YoFramePoint2d yoResolvedCoP;
   private final SensorOutputMapReadOnly sensorMap;

   public QuadrupedTouchdownDetectorBasedFootSwitch(RobotQuadrant robotQuadrant, ContactablePlaneBody foot, SDFFullRobotModel fullRobotModel, double totalRobotWeight,
         SensorOutputMapReadOnly sensorMap, YoVariableRegistry parentRegistry)
   {
      this.robotQuadrant = robotQuadrant;
      this.foot = foot;
      this.fullRobotModel = fullRobotModel;
      this.totalRobotWeight = totalRobotWeight;
      this.sensorMap = sensorMap;
      yoResolvedCoP = new YoFramePoint2d(foot.getName() + "ResolvedCoP", "", foot.getSoleFrame(), registry);
   }

   @Override
   protected void setupTouchdownDetectors(ArrayList<TouchdownDetector> touchdownDetectors)
   {
      ForceSensorDataHolderReadOnly forceSensorProcessedOutputs = sensorMap.getForceSensorProcessedOutputs();
      ActuatorForceBasedTouchdownDetector actuatorForceBasedTouchdownDetector = new ActuatorForceBasedTouchdownDetector(robotQuadrant.toString().toLowerCase() + "_knee_pitch",
            forceSensorProcessedOutputs.getByName(robotQuadrant.toString().toLowerCase() + "_knee_pitch"), 100.0, registry);

      touchdownDetectors.add(actuatorForceBasedTouchdownDetector);
   }

   @Override
   public boolean hasFootHitGround()
   {
      //TODO possibly something a little fancier with the TouchdownDetectors
      return false;
   }

   @Override
   public double computeFootLoadPercentage()
   {
      return Double.NaN;
   }

   @Override
   public void computeAndPackCoP(FramePoint2d copToPack)
   {
      copToPack.setToNaN(getMeasurementFrame());
   }

   @Override
   public void updateCoP()
   {
      yoResolvedCoP.setToZero();
   }

   @Override
   public void computeAndPackFootWrench(Wrench footWrenchToPack)
   {
      footWrenchToPack.setToZero();
      if (hasFootHitGround())
         footWrenchToPack.setLinearPartZ(totalRobotWeight / 4.0);
   }

   @Override
   public ReferenceFrame getMeasurementFrame()
   {
      return foot.getSoleFrame();
   }
}
