package us.ihmc.darpaRoboticsChallenge.stateEstimation.kinematicsBasedStateEstimator;

import java.util.Collection;

import javax.vecmath.Vector3d;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.sensorProcessing.stateEstimation.JointAndIMUSensorDataSource;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoVariable;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public class PelvisIMUBasedLinearStateCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   
   private final DoubleYoVariable alphaGravityEstimation = new DoubleYoVariable("alphaGravityEstimation", registry);
   private final AlphaFilteredYoVariable gravityEstimation = new AlphaFilteredYoVariable("gravityEstimation", registry, alphaGravityEstimation);

   private final YoFrameVector pelvisLineaAcceleration = new YoFrameVector("imuAccelerationInWorld", worldFrame, registry);
   private final YoFrameVector pelvisLinearVelocityFromIMUOnly = new YoFrameVector("imuPelvisLinearVelocityIMUOnly", worldFrame, registry);
   private final YoFrameVector pelvisLinearVelocity = new YoFrameVector("imuPelvisLinearVelocity", worldFrame, registry);
   private final YoFrameVector pelvisPosition = new YoFrameVector("imuPelvisPosition", worldFrame, registry);
   
   private final BooleanYoVariable pelvisIMUBasedEstimationEnabled = new BooleanYoVariable("pelvisIMUBasedEstimationEnabled", registry);

   private ControlFlowOutputPort<Vector3d> linearAccelerationPort;

   private final ReferenceFrame pelvisFrame;
   
   private final double estimatorDT;

   // Temporary variables
   private final FrameVector tempIMUAcceleration = new FrameVector();
   private final FrameVector tempPelvisVelocityIntegrated = new FrameVector();

   public PelvisIMUBasedLinearStateCalculator(ReferenceFrame pelvisFrame, double estimatorDT, double gravitationalAcceleration, YoVariableRegistry parentRegistry)
   {
      this.pelvisFrame = pelvisFrame;
      this.estimatorDT = estimatorDT;
      
      gravityEstimation.reset();
      gravityEstimation.update(Math.abs(gravitationalAcceleration));
      
      pelvisIMUBasedEstimationEnabled.set(false);
      
      parentRegistry.addChild(registry);
   }
   
   public boolean isEstimationEnabled()
   {
      return pelvisIMUBasedEstimationEnabled.getBooleanValue();
   }
   
   public void setAlhaGravityEstimation(double alphaFilter)
   {
      alphaGravityEstimation.set(alphaFilter);
   }
   
   public void updatePelvisLinearAcceleration()
   {
      if (!isEstimationEnabled())
         return;

      tempIMUAcceleration.set(pelvisFrame, linearAccelerationPort.getData());
      gravityEstimation.update(tempIMUAcceleration.length());

      tempIMUAcceleration.changeFrame(worldFrame);
      tempIMUAcceleration.setZ(tempIMUAcceleration.getZ() - gravityEstimation.getDoubleValue());
      pelvisLineaAcceleration.set(tempIMUAcceleration);
   }
   
   public void updatePelvisLinearVelocity(FrameVector pelvisLinearVelocityPrevValue, FrameVector pelvisLinearVelocityToPack)
   {
      pelvisLineaAcceleration.getFrameVector(tempIMUAcceleration);
      tempIMUAcceleration.scale(estimatorDT);
      pelvisLinearVelocityFromIMUOnly.add(tempIMUAcceleration);
      
      pelvisLinearVelocity.set(pelvisLinearVelocityPrevValue);
      pelvisLinearVelocity.add(tempIMUAcceleration);
      pelvisLinearVelocity.getFrameVectorAndChangeFrameOfPackedVector(pelvisLinearVelocityToPack);
   }
   
   public void updatePelvisPosition(FramePoint pelvisPositionPrevValue, FramePoint pelvisPositionToPack)
   {
      pelvisLinearVelocity.getFrameVectorAndChangeFrameOfPackedVector(tempPelvisVelocityIntegrated);
      tempPelvisVelocityIntegrated.scale(estimatorDT);
      
      pelvisPosition.set(pelvisPositionPrevValue);
      pelvisPosition.add(tempPelvisVelocityIntegrated);
      pelvisPosition.getFramePointAndChangeFrameOfPackedPoint(pelvisPositionToPack);
   }

   public void getPelvisLinearAcceleration(FrameVector linearAccelerationToPack)
   {
      pelvisLineaAcceleration.getFrameVectorAndChangeFrameOfPackedVector(linearAccelerationToPack);
   }
   
   public void setJointAndIMUSensorDataSource(JointAndIMUSensorDataSource jointAndIMUSensorDataSource)
   {
      Collection<ControlFlowOutputPort<Vector3d>> linearAccelerationOutputPorts = jointAndIMUSensorDataSource.getSensorMap().getLinearAccelerationOutputPorts();

      for (ControlFlowOutputPort<Vector3d> controlFlowOutputPort : linearAccelerationOutputPorts)
      {
         linearAccelerationPort = controlFlowOutputPort;
      }
      
      if (linearAccelerationPort != null)
         pelvisIMUBasedEstimationEnabled.set(true);
   }
}
