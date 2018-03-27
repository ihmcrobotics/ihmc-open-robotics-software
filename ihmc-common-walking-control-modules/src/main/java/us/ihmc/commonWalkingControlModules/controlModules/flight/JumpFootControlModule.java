package us.ihmc.commonWalkingControlModules.controlModules.flight;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.JumpControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class JumpFootControlModule
{
   private final RobotSide robotSide;
   private final YoPlaneContactState contactState;
   private final FootSwitchInterface footSwitch;
   private final ContactableFoot contactableFoot;

   private final PoseReferenceFrame controlFrame;
   private final FramePose3D controlFramePose;
   private final SpatialAccelerationCommand spatialAccelerationCommand;
   
   
   public JumpFootControlModule(RobotSide robotSide, YoPlaneContactState footContactState, FootSwitchInterface footSwitch, ContactableFoot contactableFoot,
                                JumpControllerParameters jumpControlParameters, YoVariableRegistry registry)
   {
      this.robotSide = robotSide;
      this.contactState = footContactState;
      this.contactableFoot = contactableFoot;
      this.footSwitch = footSwitch;

      this.controlFramePose = new FramePose3D(contactableFoot.getSoleFrame());
      this.controlFrame = new PoseReferenceFrame(robotSide.getCamelCaseNameForStartOfExpression() + "ControlFrame", controlFramePose);
      this.spatialAccelerationCommand = new SpatialAccelerationCommand();
   }
   
   public void compute()
   {
      
   }
   
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return spatialAccelerationCommand;
   }
}
