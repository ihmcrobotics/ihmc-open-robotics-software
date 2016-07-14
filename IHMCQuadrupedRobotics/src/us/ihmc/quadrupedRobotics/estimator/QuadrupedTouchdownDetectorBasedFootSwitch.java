package us.ihmc.quadrupedRobotics.estimator;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.TouchdownDetectorBasedFootswitch;
import us.ihmc.commonWalkingControlModules.touchdownDetector.ActuatorForceBasedTouchdownDetector;
import us.ihmc.commonWalkingControlModules.touchdownDetector.JointTorqueBasedTouchdownDetector;
import us.ihmc.commonWalkingControlModules.touchdownDetector.JointVelocityFiniteDifferenceBasedTouchdownDetector;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;

public class QuadrupedTouchdownDetectorBasedFootSwitch extends TouchdownDetectorBasedFootswitch
{
   private final RobotQuadrant robotQuadrant;
   private final ContactablePlaneBody foot;
   private final SDFFullRobotModel fullRobotModel;
   private final double totalRobotWeight;
   private final YoFramePoint2d yoResolvedCoP;
   private final BooleanYoVariable touchdownDetected;
   private final BooleanYoVariable trustTouchdownDetectors;

   public QuadrupedTouchdownDetectorBasedFootSwitch(RobotQuadrant robotQuadrant, ContactablePlaneBody foot, SDFFullRobotModel fullRobotModel, double totalRobotWeight,
         YoVariableRegistry parentRegistry)
   {
      super(robotQuadrant.getCamelCaseName() + "QuadrupedTouchdownFootSwitch", parentRegistry);

      this.robotQuadrant = robotQuadrant;
      this.foot = foot;
      this.fullRobotModel = fullRobotModel;
      this.totalRobotWeight = totalRobotWeight;
      yoResolvedCoP = new YoFramePoint2d(foot.getName() + "ResolvedCoP", "", foot.getSoleFrame(), registry);
      touchdownDetected = new BooleanYoVariable(robotQuadrant.getCamelCaseName() + "TouchdownDetected", registry);
      trustTouchdownDetectors = new BooleanYoVariable(robotQuadrant.getCamelCaseName() + "TouchdownDetectorsTrusted", registry);

      setupTouchdownDetectors();
   }

   protected void setupTouchdownDetectors()
   {
      JointTorqueBasedTouchdownDetector jointTorqueBasedTouchdownDetector = new JointTorqueBasedTouchdownDetector(fullRobotModel.getOneDoFJointByName(robotQuadrant.toString().toLowerCase() + "_knee_pitch"),
            registry);
      jointTorqueBasedTouchdownDetector.setTorqueThreshold(3.0);
      touchdownDetectors.add(jointTorqueBasedTouchdownDetector);

      JointVelocityFiniteDifferenceBasedTouchdownDetector jointVelocityFiniteDifferenceBasedTouchdownDetector = new JointVelocityFiniteDifferenceBasedTouchdownDetector(
            fullRobotModel.getOneDoFJointByName(robotQuadrant.toString().toLowerCase() + "_knee_pitch"), controllerThinksHasTouchedDown, registry);
      jointVelocityFiniteDifferenceBasedTouchdownDetector.setFootInSwingThreshold(0.05);
      jointVelocityFiniteDifferenceBasedTouchdownDetector.setTouchdownThreshold(0.1);

      touchdownDetectors.add(jointVelocityFiniteDifferenceBasedTouchdownDetector);
   }

   @Override
   public boolean hasFootHitGround()
   {
      boolean touchdown = true;
      for (int i = 0; i < touchdownDetectors.size(); i++)
      {
         touchdown &= touchdownDetectors.get(i).hasTouchedDown();
      }

      touchdownDetected.set(touchdown);

      if(trustTouchdownDetectors.getBooleanValue())
         return touchdownDetected.getBooleanValue();
      else
         return controllerThinksHasTouchedDown.getBooleanValue();
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

   @Override
   public void trustFootSwitch(boolean trustFootSwitch)
   {
      this.trustTouchdownDetectors.set(trustFootSwitch);
   }
}
