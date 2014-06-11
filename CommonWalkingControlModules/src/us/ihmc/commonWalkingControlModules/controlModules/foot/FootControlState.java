package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import com.yobotics.simulationconstructionset.util.statemachines.State;

public abstract class FootControlState extends State<ConstraintType>
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   
   protected final FramePoint desiredPosition = new FramePoint(worldFrame);
   protected final FrameVector desiredLinearVelocity = new FrameVector(worldFrame);
   protected final FrameVector desiredLinearAcceleration = new FrameVector(worldFrame);
   
   private final YoFramePoint yoDesiredPosition;
   private final YoFrameVector yoDesiredLinearVelocity;
   private final YoFrameVector yoDesiredLinearAcceleration;
   
   protected final RigidBodySpatialAccelerationControlModule accelerationControlModule;
   
   public FootControlState(ConstraintType stateEnum, YoFramePoint yoDesiredPosition,
         YoFrameVector yoDesiredLinearVelocity, YoFrameVector yoDesiredLinearAcceleration,
         RigidBodySpatialAccelerationControlModule accelerationControlModule)
   {
      super(stateEnum);
      
      this.yoDesiredPosition = yoDesiredPosition;
      this.yoDesiredLinearVelocity = yoDesiredLinearVelocity;
      this.yoDesiredLinearAcceleration = yoDesiredLinearAcceleration;
      
      this.accelerationControlModule = accelerationControlModule;
   }
   
   public abstract void doSpecificAction();
   
   public void doAction()
   {
      doSpecificAction();
      
      desiredLinearVelocity.changeFrame(worldFrame);
      yoDesiredLinearVelocity.set(desiredLinearVelocity);
      desiredLinearAcceleration.changeFrame(worldFrame);
      yoDesiredLinearAcceleration.set(desiredLinearAcceleration);
   }
}
