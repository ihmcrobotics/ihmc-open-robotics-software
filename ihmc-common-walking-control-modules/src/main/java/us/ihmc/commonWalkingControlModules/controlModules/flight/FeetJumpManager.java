package us.ihmc.commonWalkingControlModules.controlModules.flight;

import java.util.Collection;
import java.util.Map;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commonWalkingControlModules.configurations.JumpControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class FeetJumpManager implements JumpControlManagerInterface
{
   private final InverseDynamicsCommandList inverseDynamicsCommandList;
   private final FeedbackControlCommandList feedbackControlCommandList;
   private final SideDependentList<FootSwitchInterface> footSwitches;
   private final SideDependentList<ContactableFoot> contactableFeet;
   private final SideDependentList<JumpFootControlModule> footControlModules;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   public FeetJumpManager(HighLevelHumanoidControllerToolbox controllerToolbox, JumpControllerParameters jumpControlParameters,
                          Collection<ReferenceFrame> trajectoryFrames, TObjectDoubleHashMap<String> homeConfiguration,
                          YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry registry)
   {
      this.controllerToolbox = controllerToolbox;
      this.footSwitches = controllerToolbox.getFootSwitches();
      this.contactableFeet = controllerToolbox.getContactableFeet();
      this.inverseDynamicsCommandList = new InverseDynamicsCommandList();
      this.feedbackControlCommandList = new FeedbackControlCommandList();
      this.footControlModules = new SideDependentList<>();
      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      RigidBody elevator = fullRobotModel.getElevator();
      RigidBody pelvis = fullRobotModel.getPelvis();

      for (RobotSide robotSide : RobotSide.values)
      {
         FootSwitchInterface footSwitch = footSwitches.get(robotSide);
         ContactableFoot contactableFoot = contactableFeet.get(robotSide);
         JumpFootControlModule footControlModule = new JumpFootControlModule(robotSide, footSwitch, contactableFoot, elevator, pelvis, trajectoryFrames,
                                                                             homeConfiguration, jumpControlParameters, controllerToolbox.getYoTime(),
                                                                             graphicsListRegistry, registry);
         footControlModules.put(robotSide, footControlModule);
      }
   }

   public void setGains(RobotSide robotSide, Map<String, PIDGainsReadOnly> jointGainMap, PID3DGainsReadOnly taskspaceOrientationGains,
                        PID3DGainsReadOnly taskspacePositionGains, PID3DGainsReadOnly inverseDynamicsOrientationGain,
                        PID3DGainsReadOnly inverseDynamicsPositionGain)
   {
      footControlModules.get(robotSide).setGains(jointGainMap, taskspaceOrientationGains, taskspacePositionGains, inverseDynamicsOrientationGain,
                                                 inverseDynamicsPositionGain);
   }

   public void setWeights(RobotSide robotSide, Map<String, DoubleProvider> jointspaceWeights, Vector3DReadOnly taskspceAngularWeight,
                          Vector3DReadOnly taskSpaceLinearWeight, Vector3DReadOnly inverseDynamicsAngularWeight, Vector3DReadOnly inverseDynamicsLinearWeight)
   {
      footControlModules.get(robotSide).setWeights(jointspaceWeights, taskspceAngularWeight, taskSpaceLinearWeight, inverseDynamicsAngularWeight,
                                                   inverseDynamicsLinearWeight);
   }

   public void compute()
   {
      for (RobotSide robotSide : RobotSide.values)
         footControlModules.get(robotSide).compute();
   }

   public void complyAndDamp(RobotSide robotSide)
   {
      footControlModules.get(robotSide).complyAndDamp();
   }

   public void holdInJointspace(RobotSide robotSide)
   {
      footControlModules.get(robotSide).holdInJointSpace();
   }

   public void holdInTaskspace(RobotSide robotSide)
   {
      footControlModules.get(robotSide).holdInTaskspace();
   }

   public void makeFeetFullyConstrained(RobotSide robotSide)
   {
      controllerToolbox.setFootContactStateFullyConstrained(robotSide);
   }

   public void makeFeetFullyUnconstrained(RobotSide robotSide)
   {
      controllerToolbox.setFootContactStateFree(robotSide);
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      inverseDynamicsCommandList.clear();
      for (RobotSide robotSide : RobotSide.values)
      {
         JumpFootControlModule footControlModule = footControlModules.get(robotSide);
         inverseDynamicsCommandList.addCommand(footControlModule.getInverseDynamicsCommand());
      }
      return inverseDynamicsCommandList;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      feedbackControlCommandList.clear();
      for (RobotSide robotSide : RobotSide.values)
      {
         JumpFootControlModule footControlModule = footControlModules.get(robotSide);
         feedbackControlCommandList.addCommand(footControlModule.getFeedbackControlCommand());
      }
      return feedbackControlCommandList;
   }

   @Override
   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      FeedbackControlCommandList templateCommandList = new FeedbackControlCommandList();
      for (RobotSide robotSide : RobotSide.values)
      {
         JumpFootControlModule footControlModule = footControlModules.get(robotSide);
         templateCommandList.addCommand(footControlModule.createFeedbackControlTemplate());
      }
      return templateCommandList;
   }

   public double getGroundReactionForceZ()
   {
      double totalForceZ = 0.0;
      for (RobotSide robotSide : RobotSide.values)
         totalForceZ += footControlModules.get(robotSide).getGroundReactionForceZ();
      return totalForceZ;
   }

   public double getFootPosition(RobotSide robotSide, Axis axis)
   {
      return footControlModules.get(robotSide).getFootPosition(axis);
   }
}
