package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.centerOfMassEstimator;

import java.util.List;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialForce;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.centerOfMassEstimator.WrenchBasedMomentumStateUpdater.WrenchSensor;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class WrenchBasedMomentumRateCalculator
{
   private final double mass;
   private final double gravity;
   private final int nWrenchSensors;

   private final YoFixedFrameSpatialForce totalSpatialForceAtCoM;
   private final YoVector3D linearMomentumRateState;
   private final YoVector3D angularMomentumRateState;
   private final List<WrenchSensor> wrenchSensors;

   private final Wrench wrench = new Wrench();
   private final ReferenceFrame centerOfMassFrame;

   public WrenchBasedMomentumRateCalculator(ReferenceFrame centerOfMassFrame,
                                            List<WrenchSensor> wrenchSensors,
                                            double mass,
                                            double gravity,
                                            YoRegistry registry)
   {
      this.centerOfMassFrame = centerOfMassFrame;
      this.wrenchSensors = wrenchSensors;
      this.mass = mass;
      this.gravity = gravity;

      nWrenchSensors = wrenchSensors.size();

      totalSpatialForceAtCoM = new YoFixedFrameSpatialForce("totalSpatialForceAtCoM", centerOfMassFrame, registry);
      linearMomentumRateState = new YoVector3D("linearMomentumRateState", registry);
      angularMomentumRateState = new YoVector3D("angularMomentumRateState", registry);
   }

   public void update()
   {
      totalSpatialForceAtCoM.setToZero();

      for (int i = 0; i < nWrenchSensors; i++)
      {
         WrenchSensor wrenchSensor = wrenchSensors.get(i);
         wrenchSensor.getMeasuredWrench(wrench);
         wrench.changeFrame(centerOfMassFrame);
         totalSpatialForceAtCoM.add(wrench);
      }

      linearMomentumRateState.set(totalSpatialForceAtCoM.getLinearPart());
      linearMomentumRateState.subZ(mass * gravity);
      angularMomentumRateState.set(totalSpatialForceAtCoM.getAngularPart());
   }

   public YoFixedFrameSpatialForce getTotalSpatialForceAtCoM()
   {
      return totalSpatialForceAtCoM;
   }

   public YoVector3D getLinearMomentumRateState()
   {
      return linearMomentumRateState;
   }

   public YoVector3D getAngularMomentumRateState()
   {
      return angularMomentumRateState;
   }
}
