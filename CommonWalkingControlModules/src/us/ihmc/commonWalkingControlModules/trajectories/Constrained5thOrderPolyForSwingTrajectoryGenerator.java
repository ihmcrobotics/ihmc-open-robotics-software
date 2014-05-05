package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleProvider;
import com.yobotics.simulationconstructionset.util.trajectory.PositionProvider;
import com.yobotics.simulationconstructionset.util.trajectory.VectorProvider;

public class Constrained5thOrderPolyForSwingTrajectoryGenerator
{
   private final PositionProvider initialPositionProvider;
   private final PositionProvider finalDesiredPositionProvider;
   private final VectorProvider finalDesiredVelocityProvider;
   
   private final YoFramePoint initialPosition3D;
   private final YoFramePoint finalDesiredPosition3D;
   private final YoFrameVector finalDesiredVelocity3D;
   
   private final DoubleYoVariable initialPosition1D;
   private final DoubleYoVariable finalDesiredPosition1D;
   private final DoubleYoVariable finalDesiredVelocity1D;
   
   private final DoubleYoVariable desiredPosition1D;
   private final DoubleYoVariable desiredVelocity1D;
   private final DoubleYoVariable desiredAcceleration1D;
   
   private boolean moveWithConstantVelocityAfterFinalTime;
   
   private final double stepTime;
   private double initialTime;
   private double previousTime;
   
   double maxHeightOfSwingFoot;
   
   private final WalkingControllerParameters walkingControllerParameters; 
   
   private final Constrained5thOrderPolyForSwingFootTrajectory swingFootTrajectory;
   
   private final ReferenceFrame referenceFrame;
   
   private final YoVariableRegistry registry;
   
   public Constrained5thOrderPolyForSwingTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, double initialTime,
         DoubleProvider stepTimeProvider,PositionProvider initialPositionProvider, PositionProvider finalDesiredPositionProvider, 
         VectorProvider finalDesiredVelocityProvider, YoVariableRegistry parentRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry,
         WalkingControllerParameters walkingControllerParameters, boolean moveWithConstantVelocityAfterFinalTime)
   {
      this.registry = new YoVariableRegistry(getClass().getSimpleName());
      parentRegistry.addChild(this.registry);
      
      this.referenceFrame = referenceFrame;
      
      this.moveWithConstantVelocityAfterFinalTime = moveWithConstantVelocityAfterFinalTime;
      
      this.initialPositionProvider = initialPositionProvider;
      this.finalDesiredPositionProvider = finalDesiredPositionProvider;
      this.finalDesiredVelocityProvider = finalDesiredVelocityProvider;
      
      this.initialPosition3D = new YoFramePoint(namePrefix + "InitialPosition", this.referenceFrame, this.registry);
      this.finalDesiredPosition3D = new YoFramePoint(namePrefix + "FinalDesiredPosition", this.referenceFrame, this.registry);
      this.finalDesiredVelocity3D = new YoFrameVector(namePrefix + "FinalDesiredVelocity", this.referenceFrame, this.registry);
      
      this.initialPosition1D = new DoubleYoVariable(namePrefix + "InitialPosition", this.registry);
      this.finalDesiredPosition1D = new DoubleYoVariable(namePrefix + "FinalDesiredPosition", this.registry);
      this.finalDesiredVelocity1D = new DoubleYoVariable(namePrefix + "FinalDesiredVelocity", this.registry);
      
      this.desiredPosition1D = new DoubleYoVariable(namePrefix + "DesiredPositionZ", this.registry);
      this.desiredVelocity1D = new DoubleYoVariable(namePrefix + "DesiredVelocityZ", this.registry);
      this.desiredAcceleration1D = new DoubleYoVariable(namePrefix + "DesiredAccelerationZ", this.registry);
      
      this.stepTime = stepTimeProvider.getValue();
      this.initialTime = initialTime;
      this.previousTime = initialTime;
      
      this.walkingControllerParameters = walkingControllerParameters;
      
      swingFootTrajectory = new Constrained5thOrderPolyForSwingFootTrajectory(namePrefix + "swingFootTrajectoryZ", this.registry);
   }
   
   public void initialize()
   {
      setInitialAndFinalPositionsAndVelocities(); 
      
      if(walkingControllerParameters != null)
      {
    	  maxHeightOfSwingFoot = walkingControllerParameters.getSwingHeightMaxForPushRecoveryTrajectory();  
      }
      else
      {
    	  maxHeightOfSwingFoot = 0.15; //Default value, needed for generic test without being 
    	  							   //robot specific(i.e. needing a robot models walkingControllerParameters.
      }
      swingFootTrajectory.setParams(initialPosition1D.getDoubleValue(), initialPosition1D.getDoubleValue()+maxHeightOfSwingFoot, finalDesiredPosition1D.getDoubleValue(), 
                                    finalDesiredVelocity1D.getDoubleValue(), initialTime, stepTime);
   }

   public double getDesiredPosition()
   {
      return desiredPosition1D.getDoubleValue();
   }
   
   public double getDesiredVelocity()
   {
      return desiredVelocity1D.getDoubleValue();
   }
   
   public double getDesiredAcceleration()
   {
      return desiredAcceleration1D.getDoubleValue();
   }
   
   private void setInitialAndFinalPositionsAndVelocities()
   {
      // Is this all necessary? Seems like I am doing too much here....
      FramePoint tempInitialPosition = new FramePoint(referenceFrame);
      FramePoint tempFinalPosition = new FramePoint(referenceFrame);
      FrameVector tempFinalVelocity = new FrameVector(referenceFrame);

      initialPositionProvider.get(tempInitialPosition);
      finalDesiredPositionProvider.get(tempFinalPosition);
      finalDesiredVelocityProvider.get(tempFinalVelocity);

      tempInitialPosition.changeFrame(referenceFrame);
      tempFinalPosition.changeFrame(referenceFrame);
      tempFinalVelocity.changeFrame(referenceFrame);
      
      initialPosition3D.set(tempInitialPosition);
      finalDesiredPosition3D.set(tempFinalPosition);
      finalDesiredVelocity3D.set(tempFinalVelocity);
      
      initialPosition3D.setX(initialPosition3D.getX());
      initialPosition3D.setY(initialPosition3D.getY());
      initialPosition3D.setY(initialPosition3D.getZ());
      
      finalDesiredPosition3D.setX(finalDesiredPosition3D.getX());
      finalDesiredPosition3D.setY(finalDesiredPosition3D.getY());
      finalDesiredPosition3D.setY(finalDesiredPosition3D.getZ());
      
      finalDesiredVelocity3D.setX(finalDesiredVelocity3D.getX());
      finalDesiredVelocity3D.setY(finalDesiredVelocity3D.getY());
      finalDesiredVelocity3D.setZ(finalDesiredVelocity3D.getZ());
      
      initialPosition1D.set(initialPosition3D.getZ());
      finalDesiredPosition1D.set(finalDesiredPosition3D.getZ());
      finalDesiredVelocity1D.set(finalDesiredVelocity3D.getZ());
   }
   
   public void compute(double time)
   {
	   swingFootTrajectory.computeTrajectory(time);
		   
	   desiredPosition1D.set(swingFootTrajectory.getPosition());
	   desiredVelocity1D.set(swingFootTrajectory.getVelocity());
	   desiredAcceleration1D.set(swingFootTrajectory.getAcceleration());
	   
	   previousTime = time;
      
   }
}
