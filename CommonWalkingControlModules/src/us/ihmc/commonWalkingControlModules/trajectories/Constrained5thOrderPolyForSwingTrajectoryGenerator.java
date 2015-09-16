package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.robotics.trajectories.providers.PositionProvider;
import us.ihmc.robotics.trajectories.providers.VectorProvider;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;


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
   
   private final double stepTime;
   private double initialTime;
   
   double maxHeightOfSwingFoot;
   
   private final WalkingControllerParameters walkingControllerParameters; 
   
   private final Constrained5thOrderPolyForSwingFootTrajectory swingFootTrajectory;
   
   private final ReferenceFrame referenceFrame;
   
   private final YoVariableRegistry registry;
   private boolean isInitialized;
   
   public Constrained5thOrderPolyForSwingTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, double initialTime,
         DoubleProvider stepTimeProvider,PositionProvider initialPositionProvider, PositionProvider finalDesiredPositionProvider, 
         VectorProvider finalDesiredVelocityProvider, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry,
         WalkingControllerParameters walkingControllerParameters)
   {
      this.registry = new YoVariableRegistry(getClass().getSimpleName());
      parentRegistry.addChild(this.registry);
      
      this.referenceFrame = referenceFrame;
      
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
      
      this.walkingControllerParameters = walkingControllerParameters;
      
      swingFootTrajectory = new Constrained5thOrderPolyForSwingFootTrajectory(namePrefix + "swingFootTrajectoryZ", this.registry);
   }
   
   public void initialize()
   {
	   if(!isInitialized)
	  {
		   setInitialAndFinalPositionsAndVelocities();
	  }
      double nominalMaxTrajectoryHeight;
      boolean steppingDown = false;
      boolean steppingUp = false;
      
      double stepUpFraction = 0.2;
      double stepDownFraction = 0.15;
      
      if(walkingControllerParameters != null)
      {
    	  nominalMaxTrajectoryHeight = walkingControllerParameters.getSwingHeightMaxForPushRecoveryTrajectory();
    	  if(initialPosition1D.getDoubleValue() - finalDesiredPosition1D.getDoubleValue()>0.03)
    	  {
    		  steppingDown = true;
    	  }
    	  else if(finalDesiredPosition1D.getDoubleValue() - initialPosition1D.getDoubleValue() > 0.03)
    	  {
    		  steppingUp = true;
    	  }
    	  
    	  double stepHeightDifference = Math.abs(Math.max(initialPosition1D.getDoubleValue(), finalDesiredPosition1D.getDoubleValue()) - 
	    		  Math.min(initialPosition1D.getDoubleValue(), finalDesiredPosition1D.getDoubleValue()));
    	  
	      if(steppingUp)
	      {
	    	  maxHeightOfSwingFoot = initialPosition1D.getDoubleValue() + nominalMaxTrajectoryHeight + stepHeightDifference + stepUpFraction * stepHeightDifference;
	      }
	      else if(steppingDown)
	      {
	    	  maxHeightOfSwingFoot = initialPosition1D.getDoubleValue() + nominalMaxTrajectoryHeight + stepDownFraction * stepHeightDifference;
	      }
	      else
	      {
			   
	    	  maxHeightOfSwingFoot = initialPosition1D.getDoubleValue() + nominalMaxTrajectoryHeight + stepHeightDifference;
	      }
      }
      else
      {
    	  maxHeightOfSwingFoot = 0.15;
      }
      
      swingFootTrajectory.setParams(initialPosition1D.getDoubleValue(), maxHeightOfSwingFoot, finalDesiredPosition1D.getDoubleValue(), 
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
   
   public void setIsInitialized(boolean value)
   {
	   isInitialized = value;
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
      initialPosition3D.setZ(initialPosition3D.getZ());
      
      finalDesiredPosition3D.setX(finalDesiredPosition3D.getX());
      finalDesiredPosition3D.setY(finalDesiredPosition3D.getY());
      finalDesiredPosition3D.setZ(finalDesiredPosition3D.getZ());
      
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
      
   }
}
