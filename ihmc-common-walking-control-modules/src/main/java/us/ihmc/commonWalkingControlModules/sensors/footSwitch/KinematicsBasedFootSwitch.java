package us.ihmc.commonWalkingControlModules.sensors.footSwitch;

import java.util.Collection;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint2D;

public class KinematicsBasedFootSwitch implements FootSwitchInterface
{
   private final YoVariableRegistry registry;
   private final YoBoolean hitGround, fixedOnGround;
   private final DoubleProvider switchZThreshold;
   private final YoDouble soleZ, ankleZ;
   private final double totalRobotWeight;
   private final ContactablePlaneBody foot;
   private final ContactablePlaneBody[] otherFeet;

   private final YoFramePoint2D yoResolvedCoP;

   public KinematicsBasedFootSwitch(String footName, SegmentDependentList<RobotSide, ? extends ContactablePlaneBody> bipedFeet, DoubleProvider switchZThreshold,
                                    double totalRobotWeight, RobotSide side, YoVariableRegistry parentRegistry)
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
      this.switchZThreshold = switchZThreshold;

      yoResolvedCoP = new YoFramePoint2D(footName + "ResolvedCoP", "", foot.getSoleFrame(), registry);

      parentRegistry.addChild(registry);
   }

   public KinematicsBasedFootSwitch(String footName, ContactablePlaneBody foot, Collection<? extends ContactablePlaneBody> otherFeet, DoubleProvider switchZThreshold,
                                    double totalRobotWeight, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(footName + getClass().getSimpleName());
      this.foot = foot;
      this.otherFeet = otherFeet.toArray(new ContactablePlaneBody[otherFeet.size()]);
      this.totalRobotWeight = totalRobotWeight;
      hitGround = new YoBoolean(footName + "hitGround", registry);
      fixedOnGround = new YoBoolean(footName + "fixedOnGround", registry);
      soleZ = new YoDouble(footName + "soleZ", registry);
      ankleZ = new YoDouble(footName + "ankleZ", registry);
      this.switchZThreshold = switchZThreshold;

      yoResolvedCoP = new YoFramePoint2D(footName + "ResolvedCoP", "", foot.getSoleFrame(), registry);

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
   public KinematicsBasedFootSwitch(String footName, QuadrantDependentList<? extends ContactablePlaneBody> quadrupedFeet, DoubleProvider switchZThreshold,
                                    double totalRobotWeight, RobotQuadrant quadrant, YoVariableRegistry parentRegistry)
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
      this.switchZThreshold = switchZThreshold;

      yoResolvedCoP = new YoFramePoint2D(footName + "ResolvedCoP", "", foot.getSoleFrame(), registry);

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

      hitGround.set((thisFootZ - lowestFootZ) < switchZThreshold.getValue());
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
      fixedOnGround.set((thisFootZ - lowestFootZ) < switchZThreshold.getValue() * 2);
      return fixedOnGround.getBooleanValue();
   }

   @Override
   @Deprecated
   public void setFootContactState(boolean hasFootHitGround)
   {
   }

   @Override
   public void trustFootSwitchInSwing(boolean trustFootSwitch)
   {

   }

   @Override
   public void trustFootSwitchInSupport(boolean trustFootSwitch)
   {

   }
}
