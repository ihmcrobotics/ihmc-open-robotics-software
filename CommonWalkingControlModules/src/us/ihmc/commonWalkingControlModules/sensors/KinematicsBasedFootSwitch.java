package us.ihmc.commonWalkingControlModules.sensors;

import javax.vecmath.Point3d;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.Wrench;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;

public class KinematicsBasedFootSwitch implements FootSwitchInterface
{
   DoubleYoVariable distanceFromGround;
   YoVariableRegistry registry;
   BooleanYoVariable hitGround;
   DoubleYoVariable switchZThreshold;
   SideDependentList<ContactablePlaneBody> bipedFeet;
   RobotSide side;

   public KinematicsBasedFootSwitch(String footName, SideDependentList<ContactablePlaneBody> bipedFeet, RobotSide side, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(footName + getClass().getSimpleName());
      this.bipedFeet = bipedFeet;
      this.side = side;
      hitGround = new BooleanYoVariable(footName + "hitGround", registry);
      switchZThreshold = new DoubleYoVariable(footName + "footSwitchZThreshold", registry);
      switchZThreshold.set(0.03);
      distanceFromGround = new DoubleYoVariable(footName + "Height", registry);
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
      copToPack.set(Double.NaN, Double.NaN);
   }

   @Override
   public void computeAndPackFootWrench(Wrench footWrenchToPack)
   {
      throw new RuntimeException("no such thing");
   }

   @Override
   public ReferenceFrame getMeasurementFrame()
   {
      return bipedFeet.get(side).getSoleFrame();
   }

   @Override
   public void reset()
   {
      hitGround.set(false);
   }

}
