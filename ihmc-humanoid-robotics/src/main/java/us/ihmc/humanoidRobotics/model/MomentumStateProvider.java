package us.ihmc.humanoidRobotics.model;

import org.ejml.simple.SimpleMatrix;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.mecano.algorithms.CenterOfMassJacobian;
import us.ihmc.mecano.algorithms.CentroidalMomentumCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.interfaces.MomentumReadOnly;
import us.ihmc.robotics.controllers.ControllerStateChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class MomentumStateProvider implements ControllerStateChangedListener
{
   private final CenterOfMassJacobian jacobian;
   private final CentroidalMomentumCalculator centroidalMomentumCalculator;
   private final ReferenceFrame worldFrame;
   private final double desiredHeight;
   private final double totalMass;

   private AlipKalmanFilter alipFilter;
   private FramePoint2D CMP;
   private FramePoint2D prevCMP;

   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);

   private final YoDouble yoComVelX = new YoDouble("comVelX", registry);
   private final YoDouble yoComVelY = new YoDouble("comVelY", registry);
   private final YoDouble yoModifiedComVelX = new YoDouble("modifiedComVelX", registry);
   private final YoDouble yoModifiedComVelY = new YoDouble("modifiedComVelY", registry);
   private final YoDouble yoModifiedComVelWithoutFilteringX = new YoDouble("modifiedComVelWithoutFilteringX", registry);
   private final YoDouble yoModifiedComVelWithoutFilteringY = new YoDouble("modifiedComVelWithoutFilteringY", registry);
   
   private final YoEnum<?> walkingCurrentState = new YoEnum("walkingControllerState", registry, null); //WalkingStateEnum.class

   public MomentumStateProvider(RigidBodyReadOnly elevator,
                                ReferenceFrame worldFrame,
                                ReferenceFrame centerOfMassFrame,
                                AlipKalmanFilter filter,
                                double desiredHeight,
                                double totalMass)
   {
      jacobian = new CenterOfMassJacobian(elevator, worldFrame);
      centroidalMomentumCalculator = new CentroidalMomentumCalculator(elevator, centerOfMassFrame);
      this.worldFrame = worldFrame;
      this.desiredHeight = desiredHeight;
      this.totalMass = totalMass;

      this.alipFilter = filter;
      CMP = new FramePoint2D();
      prevCMP = new FramePoint2D();

   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

   public void updateState(FramePoint2D CMP)
   {
      jacobian.reset();
      centroidalMomentumCalculator.reset();
      prevCMP = CMP;
      this.CMP = CMP;
   }

   public FramePoint3DReadOnly getCenterOfMassPosition()
   {
      return jacobian.getCenterOfMass();
   }

   public FrameVector3DReadOnly getModifiedCenterOfMassVelocity()
   {
      MomentumReadOnly centroidalMomentum = centroidalMomentumCalculator.getMomentum();

      // Filter centroidal angular momentum
      if (!alipFilter.hasBeenInitialized())
      {
         SimpleMatrix x0 = new SimpleMatrix(4, 1); // automatically initialized to zeros
         alipFilter.initialize(x0);
      }
      else
      {
         SimpleMatrix u = new SimpleMatrix(2, 1);
         Boolean modeSwitch = true;
         if (modeSwitch)
         {
            u.set(0, 0, CMP.getX() - prevCMP.getX());
            u.set(1, 0, CMP.getY() - prevCMP.getY());
         }

         SimpleMatrix y = new SimpleMatrix(4, 1);
         y.set(0, 0, jacobian.getCenterOfMass().getX() - CMP.getX());
         y.set(1, 0, jacobian.getCenterOfMass().getY() - CMP.getY());
         y.set(2, 0, -desiredHeight * centroidalMomentum.getLinearPart().getY() + centroidalMomentum.getAngularPart().getX());
         y.set(3, 0, desiredHeight * centroidalMomentum.getLinearPart().getX() + centroidalMomentum.getAngularPart().getY());
         alipFilter.Update(u, y);
      }
      SimpleMatrix estimatedAlip = alipFilter.getEstimatedState();

      double modifiedComVelX = estimatedAlip.get(3) / totalMass / desiredHeight;
      double modifiedComVelY = -estimatedAlip.get(2) / totalMass / desiredHeight;
      
      // For testing ///////////
      yoModifiedComVelX.set(modifiedComVelX);
      yoModifiedComVelY.set(modifiedComVelY);

      double modifiedComVelWithoutFilteringX = (centroidalMomentum.getLinearPart().getX() + centroidalMomentum.getAngularPart().getY() / desiredHeight)
            / centroidalMomentumCalculator.getTotalMass();
      double modifiedComVelWithoutFilteringY = (centroidalMomentum.getLinearPart().getY() - centroidalMomentum.getAngularPart().getX() / desiredHeight)
            / centroidalMomentumCalculator.getTotalMass();
      yoModifiedComVelWithoutFilteringX.set(modifiedComVelWithoutFilteringX);
      yoModifiedComVelWithoutFilteringY.set(modifiedComVelWithoutFilteringY);

      yoComVelX.set(jacobian.getCenterOfMassVelocity().getX());
      yoComVelY.set(jacobian.getCenterOfMassVelocity().getY());
      //////////////////////////
      
      return new FrameVector3D(worldFrame, modifiedComVelX, modifiedComVelY, jacobian.getCenterOfMassVelocity().getZ());
   }

   @Override
   public void controllerStateHasChanged(Enum<?> oldState, Enum<?> newState)
   {
      // TODO Auto-generated method stub
//      walkingCurrentState.setEnum(newState);
      walkingCurrentState.set(newState.ordinal());
   }

}
