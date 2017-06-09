package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ThreeMassAngularMomentumCalculator
{
   private final ReferenceFrame referenceFrame;
   private final double footMass;
   private final double pendulumMass;
   private final FramePoint comPosition;
   private final FrameVector comVelocity;
   private final FrameVector comAcceleration;
   private final FramePoint bodyPosition;
   private final FrameVector bodyVelocity;
   private final FrameVector bodyAcceleration;
   private final FrameVector bodyAngularMomentum;
   private final FrameVector bodyAngularMomentumRate;
   private final SideDependentList<FramePoint> footPosition = new SideDependentList<>();
   private final SideDependentList<FrameVector> footVelocity = new SideDependentList<>();
   private final SideDependentList<FrameVector> footAcceleration = new SideDependentList<>();
   private final SideDependentList<FrameVector> footAngularMomentum = new SideDependentList<>();
   private final SideDependentList<FrameVector> footAngularMomentumRate = new SideDependentList<>();
   private final FrameVector angularMomentum;
   private final FrameVector angularMomentumRate;

   public ThreeMassAngularMomentumCalculator(ReferenceFrame referenceFrame, double footMass, double totalMass)
   {
      this.referenceFrame = referenceFrame;
      this.footMass = footMass;
      this.pendulumMass = totalMass - 2.0 * footMass;
      this.comPosition = new FramePoint(referenceFrame);
      this.comVelocity = new FrameVector(referenceFrame);
      this.comAcceleration = new FrameVector(referenceFrame);
      this.bodyPosition = new FramePoint(referenceFrame);
      this.bodyVelocity = new FrameVector(referenceFrame);
      this.bodyAcceleration = new FrameVector(referenceFrame);
      this.bodyAngularMomentum = new FrameVector(referenceFrame);
      this.bodyAngularMomentumRate = new FrameVector(referenceFrame);
      for (RobotSide robotSide : RobotSide.values())
      {
         this.footPosition.put(robotSide, new FramePoint(referenceFrame));
         this.footVelocity.put(robotSide,  new FrameVector(referenceFrame));
         this.footAcceleration.put(robotSide,  new FrameVector(referenceFrame));
         this.footAngularMomentum.put(robotSide,  new FrameVector(referenceFrame));
         this.footAngularMomentumRate.put(robotSide,  new FrameVector(referenceFrame));
      }
      this.angularMomentum = new FrameVector(referenceFrame);
      this.angularMomentumRate = new FrameVector(referenceFrame);
   }

   public void compute(FramePoint comPosition, FrameVector comVelocity, FrameVector comAcceleration, SideDependentList<FramePoint> footPosition,
         SideDependentList<FrameVector> footVelocity, SideDependentList<FrameVector> footAcceleration)
   {
      this.comPosition.setIncludingFrame(comPosition);
      this.comPosition.changeFrame(referenceFrame);
      this.comVelocity.setIncludingFrame(comVelocity);
      this.comVelocity.changeFrame(referenceFrame);
      this.comAcceleration.setIncludingFrame(comAcceleration);
      this.comAcceleration.changeFrame(referenceFrame);
      for (RobotSide robotSide : RobotSide.values)
      {
         this.footPosition.get(robotSide).setIncludingFrame(footPosition.get(robotSide));
         this.footPosition.get(robotSide).changeFrame(referenceFrame);
         this.footVelocity.get(robotSide).setIncludingFrame(footVelocity.get(robotSide));
         this.footVelocity.get(robotSide).changeFrame(referenceFrame);
         this.footAcceleration.get(robotSide).setIncludingFrame(footAcceleration.get(robotSide));
         this.footAcceleration.get(robotSide).changeFrame(referenceFrame);
      }

      // Compute pendulum position, velocity, and acceleration in center of mass frame.
      bodyPosition.set(this.comPosition);
      bodyPosition.scale(2.0);
      bodyPosition.sub(this.footPosition.get(RobotSide.LEFT));
      bodyPosition.sub(this.footPosition.get(RobotSide.RIGHT));
      bodyPosition.scale(footMass / pendulumMass);
      bodyPosition.add(this.comPosition);
      bodyVelocity.set(this.comVelocity);
      bodyVelocity.scale(2.0);
      bodyVelocity.sub(this.footVelocity.get(RobotSide.LEFT));
      bodyVelocity.sub(this.footVelocity.get(RobotSide.RIGHT));
      bodyVelocity.scale(footMass / pendulumMass);
      bodyVelocity.add(this.comVelocity);
      bodyAcceleration.set(this.comAcceleration);
      bodyAcceleration.scale(2.0);
      bodyAcceleration.sub(this.footAcceleration.get(RobotSide.LEFT));
      bodyAcceleration.sub(this.footAcceleration.get(RobotSide.RIGHT));
      bodyAcceleration.scale(footMass / pendulumMass);
      bodyAcceleration.add(this.comAcceleration);

      // Compute centroidal angular momentum and angular momentum rate due to each mass.
      bodyAngularMomentum.set(bodyPosition);
      bodyAngularMomentum.sub(this.comPosition);
      bodyAngularMomentum.cross(bodyVelocity);
      bodyAngularMomentum.scale(pendulumMass);
      bodyAngularMomentumRate.set(bodyPosition);
      bodyAngularMomentumRate.sub(this.comPosition);
      bodyAngularMomentumRate.cross(bodyAcceleration);
      bodyAngularMomentumRate.scale(pendulumMass);
      for (RobotSide robotSide : RobotSide.values)
      {
         footAngularMomentum.get(robotSide).set(this.footPosition.get(robotSide));
         footAngularMomentum.get(robotSide).sub(this.comPosition);
         footAngularMomentum.get(robotSide).cross(this.footVelocity.get(robotSide));
         footAngularMomentum.get(robotSide).scale(footMass);
         footAngularMomentumRate.get(robotSide).set(this.footPosition.get(robotSide));
         footAngularMomentumRate.get(robotSide).sub(this.comPosition);
         footAngularMomentumRate.get(robotSide).cross(this.footAcceleration.get(robotSide));
         footAngularMomentumRate.get(robotSide).scale(footMass);
      }

      // Compute total centroidal angular momentum and angular momentum rate in trajectory frame.
      angularMomentum.set(bodyAngularMomentum);
      angularMomentumRate.set(bodyAngularMomentumRate);
      for (RobotSide robotSide : RobotSide.values)
      {
         angularMomentum.add(footAngularMomentum.get(robotSide));
         angularMomentumRate.add(footAngularMomentumRate.get(robotSide));
      }
   }

   public void getAngularMomentum(FrameVector angularMomentumToPack)
   {
      angularMomentumToPack.setIncludingFrame(angularMomentum);
   }

   public void getAngularMomentumRate(FrameVector angularMomentumRateToPack)
   {
      angularMomentumRateToPack.setIncludingFrame(angularMomentumRate);
   }
}
