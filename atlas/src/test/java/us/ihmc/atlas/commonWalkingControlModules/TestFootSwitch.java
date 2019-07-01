package us.ihmc.atlas.commonWalkingControlModules;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchInterface;

public class TestFootSwitch implements FootSwitchInterface
{
   private final ContactableFoot foot;
   private final ReferenceFrame opposideSoleZUp;
   private final double totalRobotWeight;

   private boolean hasFootHitGround = false;
   private final FramePoint3D solePoint = new FramePoint3D();

   public static SideDependentList<TestFootSwitch> createFootSwitches(SideDependentList<ContactableFoot> feet, double totalRobotWeight,
                                                                      SideDependentList<? extends ReferenceFrame> soleZupFrames)
   {
      SideDependentList<TestFootSwitch> ret = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         ReferenceFrame opposideSole = soleZupFrames.get(robotSide.getOppositeSide());
         ret.put(robotSide, new TestFootSwitch(feet.get(robotSide), totalRobotWeight, opposideSole));
      }
      return ret;
   }

   public TestFootSwitch(ContactableFoot foot, double totalRobotWeight, ReferenceFrame opposideSole)
   {
      this.foot = foot;
      this.opposideSoleZUp = opposideSole;
      this.totalRobotWeight = totalRobotWeight;
   }

   public void update()
   {
      solePoint.setToZero(foot.getSoleFrame());
      solePoint.changeFrame(opposideSoleZUp);
      hasFootHitGround = solePoint.getZ() < 0.01;
   }

   @Override
   public boolean hasFootHitGround()
   {
      return hasFootHitGround;
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
   }

   @Override
   public void computeAndPackFootWrench(Wrench footWrenchToPack)
   {
      footWrenchToPack.setToZero();
      if (hasFootHitGround())
         footWrenchToPack.setLinearPartZ(totalRobotWeight / 2.0);
   }

   @Override
   public ReferenceFrame getMeasurementFrame()
   {
      return foot.getSoleFrame();
   }

   @Override
   public void reset()
   {
      hasFootHitGround = false;
   }

   @Override
   public boolean getForceMagnitudePastThreshhold()
   {
      return false;
   }

   @Override
   public void setFootContactState(boolean hasFootHitGround)
   {
      this.hasFootHitGround = hasFootHitGround;
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
