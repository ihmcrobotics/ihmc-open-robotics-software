package us.ihmc.commonWalkingControlModules.sensors.footSwitch;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.FootSwitchInterface;

public class KinematicsBasedFootSwitch implements FootSwitchInterface
{
   private final YoVariableRegistry registry;
   private final BooleanYoVariable hitGround, fixedOnGround;
   private final DoubleYoVariable switchZThreshold;
   private final DoubleYoVariable soleZ, ankleZ;
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
      hitGround = new BooleanYoVariable(footName + "hitGround", registry);
      fixedOnGround = new BooleanYoVariable(footName + "fixedOnGround", registry);
      soleZ = new DoubleYoVariable(footName + "soleZ", registry);
      ankleZ = new DoubleYoVariable(footName + "ankleZ", registry);
      this.switchZThreshold = new DoubleYoVariable(footName + "footSwitchZThreshold", registry);
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
      hitGround = new BooleanYoVariable(footName + "hitGround", registry);
      fixedOnGround = new BooleanYoVariable(footName + "fixedOnGround", registry);
      soleZ = new DoubleYoVariable(footName + "soleZ", registry);
      ankleZ = new DoubleYoVariable(footName + "ankleZ", registry);
      this.switchZThreshold = new DoubleYoVariable(footName + "footSwitchZThreshold", registry);
      this.switchZThreshold.set(switchZThreshold);

      yoResolvedCoP = new YoFramePoint2d(footName + "ResolvedCoP", "", foot.getSoleFrame(), registry);

      parentRegistry.addChild(registry);
   }

   FramePoint tmpFramePoint = new FramePoint();

   private Point3D getPointInWorld(ReferenceFrame frame)
   {
      tmpFramePoint.setToZero(frame);
      tmpFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
      return tmpFramePoint.getPoint();
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
