package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ExternalWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandBuffer;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.interfaces.WrenchBasics;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialForce;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGenerator;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGeneratorMode;
import us.ihmc.commons.robotics.robotSide.RobotSide;
import us.ihmc.commons.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class LegElasticityDebuggator
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final CommonHumanoidReferenceFrames referenceFrames;
   private final SideDependentList<RigidBodyBasics> feet;

   private final YoFixedFrameSpatialForce footExternalWrenchOffset;
   private final SideDependentList<YoFramePose3D> footPosesMidZUp;
   private final SideDependentList<YoFramePose3D> footPosesOppFrame;
   private final YoFrameVector3D footSpacingMidZUp;
   private final YoFunctionGenerator footExternalForceOffsetX, footExternalForceOffsetY;

   public LegElasticityDebuggator(CommonHumanoidReferenceFrames referenceFrames, SideDependentList<RigidBodyBasics> feet, DoubleProvider time, YoRegistry parentRegistry)
   {
      this.referenceFrames = referenceFrames;
      this.feet = feet;
      MovingReferenceFrame midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
      footExternalWrenchOffset = new YoFixedFrameSpatialForce("FootExternalWrenchOffset", midFeetZUpFrame, registry);
      footExternalForceOffsetX = new YoFunctionGenerator("FootExternalForceX", time, registry);
      footExternalForceOffsetY = new YoFunctionGenerator("FootExternalForceY", time, registry);
      footPosesMidZUp = new SideDependentList<>(side -> new YoFramePose3D(side.getCamelCaseName() + "FootPoseMidZUp", midFeetZUpFrame, registry));
      footPosesOppFrame = new SideDependentList<>(side -> new YoFramePose3D(side.getCamelCaseName() + "FootPoseOppFrame",
                                                                            referenceFrames.getSoleFrame(side),
                                                                            registry));
      footSpacingMidZUp = new YoFrameVector3D("FootSpacingMidZUp", midFeetZUpFrame, registry);
      parentRegistry.addChild(registry);
   }

   public void update()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         footPosesMidZUp.get(robotSide).setFromReferenceFrame(referenceFrames.getSoleFrame(robotSide));
         footPosesOppFrame.get(robotSide).setFromReferenceFrame(referenceFrames.getSoleFrame(robotSide.getOppositeSide()));
      }

      footSpacingMidZUp.sub(footPosesMidZUp.get(RobotSide.LEFT).getPosition(), footPosesMidZUp.get(RobotSide.RIGHT).getPosition());
   }

   private final InverseDynamicsCommandBuffer output = new InverseDynamicsCommandBuffer();

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      if (footExternalForceOffsetX.getMode() != YoFunctionGeneratorMode.OFF)
         footExternalWrenchOffset.setLinearPartX(footExternalForceOffsetX.getValue());
      if (footExternalForceOffsetY.getMode() != YoFunctionGeneratorMode.OFF)
         footExternalWrenchOffset.setLinearPartY(footExternalForceOffsetY.getValue());

      output.clear();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics foot = feet.get(robotSide);
         ExternalWrenchCommand command = output.addExternalWrenchCommand();
         command.setRigidBody(foot);
         WrenchBasics footWrench = command.getExternalWrench();
         footWrench.setIncludingFrame(footExternalWrenchOffset);
         footWrench.setBodyFrame(foot.getBodyFixedFrame());
         footWrench.changeFrame(foot.getBodyFixedFrame());
         if (robotSide == RobotSide.RIGHT)
            footWrench.negate();
      }

      return output;
   }
}
