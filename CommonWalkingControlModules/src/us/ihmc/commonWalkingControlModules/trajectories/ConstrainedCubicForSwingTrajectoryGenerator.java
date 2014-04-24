package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredVelocityControlModule;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleProvider;
import com.yobotics.simulationconstructionset.util.trajectory.PositionProvider;
import com.yobotics.simulationconstructionset.util.trajectory.VectorProvider;

public class ConstrainedCubicForSwingTrajectoryGenerator
{
   private final PositionProvider initialPositionProvider;
   private final PositionProvider finalDesiredPositionProvider;
   
   private final YoFramePoint initialPosition3D;
   private final YoFramePoint finalDesiredPosition3D;
   
   private final DoubleYoVariable initialPosition1D;
   private final DoubleYoVariable finalDesiredPosition1D;
   
   private final DoubleYoVariable desiredPosition1D;
   private final DoubleYoVariable desiredVelocity1D;
   private final DoubleYoVariable desiredAcceleration1D;
   
   
   private final double stepTime;
   private final double initialTime;
   
   private final WalkingControllerParameters walkingControllerParameters; 
   
   private boolean isInitialized = false;
   
   private final ConstrainedCubicForSwingFootTrajectory swingFootTrajectory;
   
   private final ReferenceFrame referenceFrame;
   
   private final YoVariableRegistry registry;
   
   public ConstrainedCubicForSwingTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, DoubleYoVariable initialTime,
         DoubleProvider stepTimeProvider,PositionProvider initialPositionProvider, PositionProvider finalDesiredPositionProvider, 
         YoVariableRegistry parentRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry,
         WalkingControllerParameters walkingControllerParameters, boolean visualize)
   {
      this.registry = new YoVariableRegistry(getClass().getSimpleName());
      parentRegistry.addChild(this.registry);
      
      this.referenceFrame = referenceFrame;
      
      this.initialPositionProvider = initialPositionProvider;
      this.finalDesiredPositionProvider = finalDesiredPositionProvider;
      //@TODO make this less hacky. Will need to create a PositionProvider2D and a VectorProvider2D I think.
      this.initialPosition3D = new YoFramePoint(namePrefix + "InitialPosition", this.referenceFrame, this.registry);
      this.finalDesiredPosition3D = new YoFramePoint(namePrefix + "FinalDesiredPosition", this.referenceFrame, this.registry);
      
      this.initialPosition1D = new DoubleYoVariable(namePrefix + "InitialPosition", this.registry);
      this.finalDesiredPosition1D = new DoubleYoVariable(namePrefix + "FinalDesiredPosition", this.registry);
      
      this.desiredPosition1D = new DoubleYoVariable(namePrefix + "DesiredPositionZ", this.registry);
      this.desiredVelocity1D = new DoubleYoVariable(namePrefix + "DesiredVelocityZ", this.registry);
      this.desiredAcceleration1D = new DoubleYoVariable(namePrefix + "DesiredAccelerationZ", this.registry);
      
      this.stepTime = stepTimeProvider.getValue();
      this.initialTime = initialTime.getDoubleValue();
      
      this.walkingControllerParameters = walkingControllerParameters;
      
      swingFootTrajectory = new ConstrainedCubicForSwingFootTrajectory(namePrefix + "swingFootTrajectoryZ", this.registry);
   }
   
   public void initialize()
   {
      setInitialAndFinalPositionsAndVelocities();
      
      swingFootTrajectory.setParams(initialPosition1D.getDoubleValue(), 0.1, finalDesiredPosition1D.getDoubleValue(), initialTime, stepTime);
      
      isInitialized = true;
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
      FramePoint tempInitialPosition = new FramePoint(referenceFrame);
      FramePoint tempFinalPosition = new FramePoint(referenceFrame);

      initialPositionProvider.get(tempInitialPosition);
      finalDesiredPositionProvider.get(tempFinalPosition);

      tempInitialPosition.changeFrame(referenceFrame);
      tempFinalPosition.changeFrame(referenceFrame);
      
      initialPosition3D.set(tempInitialPosition);
      finalDesiredPosition3D.set(tempFinalPosition);
      
      initialPosition3D.setX(initialPosition3D.getX());
      initialPosition3D.setY(initialPosition3D.getY());
      initialPosition3D.setY(initialPosition3D.getZ());
      
      finalDesiredPosition3D.setX(finalDesiredPosition3D.getX());
      finalDesiredPosition3D.setY(finalDesiredPosition3D.getY());
      finalDesiredPosition3D.setY(finalDesiredPosition3D.getZ());
      
      initialPosition1D.set(initialPosition3D.getZ());
      finalDesiredPosition1D.set(finalDesiredPosition3D.getZ());
   }
   
   public void compute(double time)
   {
      if(!isInitialized)
      {
         initialize();
      }
      
      swingFootTrajectory.computeTrajectory(time);
      
      desiredPosition1D.set(swingFootTrajectory.getPosition());
      desiredVelocity1D.set(swingFootTrajectory.getVelocity());
      desiredAcceleration1D.set(swingFootTrajectory.getAcceleration());
   }
}
