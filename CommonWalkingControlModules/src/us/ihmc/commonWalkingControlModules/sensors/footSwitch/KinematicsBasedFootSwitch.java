package us.ihmc.commonWalkingControlModules.sensors.footSwitch;

import javax.vecmath.Point3d;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.robotics.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;

public class KinematicsBasedFootSwitch implements FootSwitchInterface
{
   DoubleYoVariable distanceFromGround;
   YoVariableRegistry registry;
   BooleanYoVariable hitGround, fixedOnGround;
   DoubleYoVariable switchZThreshold;
   SideDependentList<ContactablePlaneBody> bipedFeet;
   DoubleYoVariable soleZ,ankleZ;
   RobotSide side;
   double totalRobotWeight;

   public KinematicsBasedFootSwitch(String footName, SideDependentList<ContactablePlaneBody> bipedFeet, double switchZThreshold, double totalRobotWeight, RobotSide side, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(footName + getClass().getSimpleName());
      this.bipedFeet = bipedFeet;
      this.side = side;
      this.totalRobotWeight = totalRobotWeight;
      hitGround = new BooleanYoVariable(footName + "hitGround", registry);
      fixedOnGround= new BooleanYoVariable(footName+"fixedOnGround", registry);
      soleZ = new DoubleYoVariable(footName +"soleZ", registry);
      ankleZ = new DoubleYoVariable(footName + "ankleZ", registry);
      this.switchZThreshold = new DoubleYoVariable(footName + "footSwitchZThreshold", registry);
      this.switchZThreshold.set(switchZThreshold);
      parentRegistry.addChild(registry);

   }

   FramePoint tmpFramePoint = new FramePoint();

   private Point3d getPointInWorld(ReferenceFrame frame)
   {
      tmpFramePoint.setToZero(frame);
      tmpFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
      return tmpFramePoint.getPoint();
   }

   @Override
   public boolean hasFootHitGround()
   {
      double thisFootZ = getPointInWorld(bipedFeet.get(side).getSoleFrame()).getZ();
      double otherFootZ = getPointInWorld(bipedFeet.get(side.getOppositeSide()).getSoleFrame()).getZ();
      hitGround.set((thisFootZ-otherFootZ) < switchZThreshold.getDoubleValue());
      soleZ.set(thisFootZ);
      ankleZ.set(getPointInWorld(bipedFeet.get(side).getFrameAfterParentJoint()).getZ());
      return hitGround.getBooleanValue();
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
   public void computeAndPackFootWrench(Wrench footWrenchToPack)
   {
      footWrenchToPack.setToZero();
      if(hasFootHitGround())
         footWrenchToPack.setLinearPartZ(totalRobotWeight);
   }

   @Override
   public ReferenceFrame getMeasurementFrame()
   {
      return bipedFeet.get(side).getSoleFrame();
   }

   @Override
   public void reset()
   {
   }

   @Override
   public boolean getForceMagnitudePastThreshhold()
   {
      //a more liberal version of hasFootHitGround
      double thisFootZ = getPointInWorld(bipedFeet.get(side).getSoleFrame()).getZ();
      double otherFootZ = getPointInWorld(bipedFeet.get(side.getOppositeSide()).getSoleFrame()).getZ();
      fixedOnGround.set((thisFootZ-otherFootZ) < switchZThreshold.getDoubleValue()*2);  
      return fixedOnGround.getBooleanValue();
   }
}
