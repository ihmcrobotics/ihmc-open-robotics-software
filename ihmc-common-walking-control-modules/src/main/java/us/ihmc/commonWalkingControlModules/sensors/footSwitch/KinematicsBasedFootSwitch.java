package us.ihmc.commonWalkingControlModules.sensors.footSwitch;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.FootSwitchInterface;

public class KinematicsBasedFootSwitch implements FootSwitchInterface
{
   private final YoVariableRegistry registry;
   private final YoBoolean hitGround, fixedOnGround;
   private final YoDouble switchZThreshold;
   private final YoDouble soleZ, ankleZ;
   private final double totalRobotWeight;
   private final ContactablePlaneBody foot;
   private final ContactablePlaneBody[] otherFeet;

   private final YoFramePoint2d yoResolvedCoP;

   public KinematicsBasedFootSwitch(String footName, SideDependentList<? extends ContactablePlaneBody> bipedFeet, double switchZThreshold, double totalRobotWeight, RobotSide side, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(footName + getClass().getSimpleName());
      foot = bipedFeet.get(side);
      ContactablePlaneBody oppositeFoot = bipedFeet.get(side.getOppositeSide());
      otherFeet = new ContactablePlaneBody[] {oppositeFoot};
      this.totalRobotWeight = totalRobotWeight;
      hitGround = new YoBoolean(footName + "hitGround", registry);
      fixedOnGround = new YoBoolean(footName + "fixedOnGround", registry);
      soleZ = new YoDouble(footName + "soleZ", registry);
      ankleZ = new YoDouble(footName + "ankleZ", registry);
      this.switchZThreshold = new YoDouble(footName + "footSwitchZThreshold", registry);
      this.switchZThreshold.set(switchZThreshold);

      yoResolvedCoP = new YoFramePoint2d(footName + "ResolvedCoP", "", foot.getSoleFrame(), registry);

      parentRegistry.addChild(registry);
   }

   /**
    * Quadruped version, assumes flat ground
    * @param footName name for yovariable registry and yovariable names
    * @param quadrupedFeet all the feet
    * @param switchZThreshold difference in z between the lowest foot and the foot inquestion before assuming foot is in contact
    * @param totalRobotWeight
    * @param quadrant the foot in question
    * @param parentRegistry
    */
   public KinematicsBasedFootSwitch(String footName, QuadrantDependentList<? extends ContactablePlaneBody> quadrupedFeet, double switchZThreshold, double totalRobotWeight, RobotQuadrant quadrant, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(footName + getClass().getSimpleName());
      foot = quadrupedFeet.get(quadrant);
      
      ContactablePlaneBody acrossBodyFrontFoot = quadrupedFeet.get(quadrant.getAcrossBodyFrontQuadrant());
      ContactablePlaneBody acrossBodyHindFoot = quadrupedFeet.get(quadrant.getAcrossBodyHindQuadrant());
      ContactablePlaneBody sameSideFoot = quadrupedFeet.get(quadrant.getSameSideQuadrant());
      otherFeet = new ContactablePlaneBody[] {acrossBodyFrontFoot, acrossBodyHindFoot, sameSideFoot};
      
      this.totalRobotWeight = totalRobotWeight;
      hitGround = new YoBoolean(footName + "hitGround", registry);
      fixedOnGround = new YoBoolean(footName + "fixedOnGround", registry);
      soleZ = new YoDouble(footName + "soleZ", registry);
      ankleZ = new YoDouble(footName + "ankleZ", registry);
      this.switchZThreshold = new YoDouble(footName + "footSwitchZThreshold", registry);
      this.switchZThreshold.set(switchZThreshold);

      yoResolvedCoP = new YoFramePoint2d(footName + "ResolvedCoP", "", foot.getSoleFrame(), registry);

      parentRegistry.addChild(registry);
   }

   FramePoint3D tmpFramePoint = new FramePoint3D();

   private Point3DReadOnly getPointInWorld(ReferenceFrame frame)
   {
      tmpFramePoint.setToZero(frame);
      tmpFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
      return tmpFramePoint;
   }

   /**
    * is the foot in question within the switchZThreshold of the lowest foot
    */
   @Override
   public boolean hasFootHitGround()
   {
      double thisFootZ = getPointInWorld(foot.getSoleFrame()).getZ();
      double lowestFootZ = getLowestFootZInWorld();
      
      hitGround.set((thisFootZ - lowestFootZ) < switchZThreshold.getDoubleValue());
      soleZ.set(thisFootZ);
      ankleZ.set(getPointInWorld(foot.getFrameAfterParentJoint()).getZ());
      return hitGround.getBooleanValue();
   }

   private double getLowestFootZInWorld()
   {
      double lowestZ = Double.MAX_VALUE;
      for(int i = 0; i < otherFeet.length; i++)
      {
         double z = getPointInWorld(otherFeet[i].getSoleFrame()).getZ();
         if(z < lowestZ)
         {
            lowestZ = z;
         }
      }
      return lowestZ;
   }

   @Override
   public double computeFootLoadPercentage()
   {
      return Double.NaN;
   }

   @Override
   public void computeAndPackCoP(FramePoint2D copToPack)
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
         footWrenchToPack.setLinearPartZ(totalRobotWeight);
   }

   @Override
   public ReferenceFrame getMeasurementFrame()
   {
      return foot.getSoleFrame();
   }

   @Override
   public void reset()
   {
   }

   @Override
   public boolean getForceMagnitudePastThreshhold()
   {
      //a more liberal version of hasFootHitGround
      double thisFootZ = getPointInWorld(foot.getSoleFrame()).getZ();
      double lowestFootZ = getLowestFootZInWorld();
      fixedOnGround.set((thisFootZ - lowestFootZ) < switchZThreshold.getDoubleValue() * 2);
      return fixedOnGround.getBooleanValue();
   }

   @Override
   @Deprecated
   public void setFootContactState(boolean hasFootHitGround)
   {
   }

   @Override
   public void trustFootSwitch(boolean trustFootSwitch)
   {

   }
}
