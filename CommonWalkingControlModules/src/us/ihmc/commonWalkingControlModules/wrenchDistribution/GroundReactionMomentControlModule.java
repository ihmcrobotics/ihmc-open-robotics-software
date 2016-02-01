package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import javax.vecmath.Matrix3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.screwTheory.Momentum;


public class GroundReactionMomentControlModule
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame pelvisFrame;
   private final DoubleYoVariable kAngularMomentumZ = new DoubleYoVariable("kAngularMomentumZ", registry);
   private final DoubleYoVariable kPelvisYaw = new DoubleYoVariable("kPelvisYaw", registry);

   public GroundReactionMomentControlModule(ReferenceFrame pelvisFrame, YoVariableRegistry parentRegistry)
   {
      this.pelvisFrame = pelvisFrame;
      parentRegistry.addChild(registry);
   }

   public FrameVector determineGroundReactionMoment(Momentum momentum, double desiredPelvisYaw)
   {
      FrameVector ret = new FrameVector(worldFrame);
      FrameVector angularMomentum = new FrameVector(momentum.getExpressedInFrame(), momentum.getAngularPartCopy());
      angularMomentum.changeFrame(worldFrame);

      Matrix3d pelvisToWorld = new Matrix3d();
      pelvisFrame.getTransformToDesiredFrame(worldFrame).get(pelvisToWorld);
      double pelvisYaw = RotationTools.computeYaw(pelvisToWorld);

      double error = AngleTools.computeAngleDifferenceMinusPiToPi(desiredPelvisYaw, pelvisYaw);
      ret.setZ(-kAngularMomentumZ.getDoubleValue() * angularMomentum.getZ() + kPelvisYaw.getDoubleValue() * error);

      return ret;
   }

   public void setGains(double kAngularMomentumZ, double kPelvisYaw)
   {
      this.kAngularMomentumZ.set(kAngularMomentumZ);
      this.kPelvisYaw.set(kPelvisYaw);
   }
}
