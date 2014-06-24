package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ManipulationControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.SE3PDGains;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.IndividualHandControlModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.states.direct.DirectControlManipulationTaskDispatcher;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.taskExecutor.PipeLine;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.taskExecutor.Task;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredHandLoadBearingProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.HandPoseProvider;
import us.ihmc.commonWalkingControlModules.packetProviders.ControlStatusProducer;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.VariableChangedListener;
import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.GainCalculator;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicReferenceFrame;

/**
 * @author twan
 *         Date: 5/13/13
 */
public class ManipulationControlModule
{
   public static final boolean HOLD_POSE_IN_JOINT_SPACE_WHEN_PREPARE_FOR_LOCOMOTION = true;
   
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final List<DynamicGraphicReferenceFrame> dynamicGraphicReferenceFrames = new ArrayList<DynamicGraphicReferenceFrame>();

   private final DoubleYoVariable kpArmTaskspace, kdArmTaskspace, kiArmTaskspace, zetaArmTaskspace, maxIntegralErrorArmTaskspace;
   
   private DirectControlManipulationTaskDispatcher directControlManipulationTaskDispatcher;

   private final BooleanYoVariable hasBeenInitialized = new BooleanYoVariable("hasBeenInitialized", registry);
   private SideDependentList<IndividualHandControlModule> individualHandControlModules;

   private final PipeLine<RobotSide> pipeline = new PipeLine<RobotSide>();
   private final SideDependentList<ReferenceFrame> midHandPositionControlFrames = new SideDependentList<ReferenceFrame>();
   private final SE3PDGains taskspaceControlGains = new SE3PDGains();

   public ManipulationControlModule(FullRobotModel fullRobotModel, TwistCalculator twistCalculator, final VariousWalkingProviders variousWalkingProviders,
                                    ArmControllerParameters armControlParameters, ManipulationControllerParameters parameters,
                                    DoubleYoVariable yoTime, MomentumBasedController momentumBasedController,
                                    DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, YoVariableRegistry parentRegistry)
   {
      SideDependentList<Integer> jacobianIds = new SideDependentList<Integer>();
      SideDependentList<RigidBody> endEffectors = new SideDependentList<RigidBody>();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody endEffector = fullRobotModel.getHand(robotSide);
         endEffectors.put(robotSide, endEffector);

         int jacobianId = momentumBasedController.getOrCreateGeometricJacobian(fullRobotModel.getChest(), endEffector, endEffector.getBodyFixedFrame());
         jacobianIds.put(robotSide, jacobianId);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         String frameName = endEffectors.get(robotSide).getName() + "PositionControlFrame";
         final ReferenceFrame frameAfterJoint = endEffectors.get(robotSide).getParentJoint().getFrameAfterJoint();
         ReferenceFrame handPositionControlFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(frameName, frameAfterJoint,
               parameters.getHandControlFramesWithRespectToFrameAfterWrist().get(robotSide));
         midHandPositionControlFrames.put(robotSide, handPositionControlFrame);
      }

      createFrameVisualizers(dynamicGraphicObjectsListRegistry, midHandPositionControlFrames, "midHandPositionControlFrames", false);

      setUpStateMachine(yoTime, fullRobotModel, twistCalculator, parameters, variousWalkingProviders, dynamicGraphicObjectsListRegistry,
                        momentumBasedController, midHandPositionControlFrames, jacobianIds, armControlParameters, variousWalkingProviders.getControlStatusProducer());

      kpArmTaskspace = new DoubleYoVariable("kpArmTaskspace", registry);
      kpArmTaskspace.set(armControlParameters.getArmTaskspaceKp());

      zetaArmTaskspace = new DoubleYoVariable("zetaArmTaskspace", registry);
      zetaArmTaskspace.set(armControlParameters.getArmTaskspaceZeta());

      kdArmTaskspace = new DoubleYoVariable("kdArmTaskspace", registry);

      kiArmTaskspace = new DoubleYoVariable("kiArmTaskspace", registry);
      kiArmTaskspace.set(armControlParameters.getArmTaskspaceKi());

      maxIntegralErrorArmTaskspace = new DoubleYoVariable("maxIntegralErrorArmTaskspace", registry);
      maxIntegralErrorArmTaskspace.set(armControlParameters.getArmTaskspaceMaxIntegralError());

      setupVariableListener();

      parentRegistry.addChild(registry);
   }

   private void setupVariableListener()
   {
      VariableChangedListener listener = new VariableChangedListener()
      {
         public void variableChanged(YoVariable<?> v)
         {
            kdArmTaskspace.set(GainCalculator.computeDerivativeGain(kpArmTaskspace.getDoubleValue(), zetaArmTaskspace.getDoubleValue()));

            taskspaceControlGains.set(kpArmTaskspace.getDoubleValue(), zetaArmTaskspace.getDoubleValue(), kiArmTaskspace.getDoubleValue(),
                  maxIntegralErrorArmTaskspace.getDoubleValue(), kpArmTaskspace.getDoubleValue(), zetaArmTaskspace.getDoubleValue(), kiArmTaskspace.getDoubleValue(),
                  maxIntegralErrorArmTaskspace.getDoubleValue());
         }
      };
      
      kpArmTaskspace.addVariableChangedListener(listener);
      zetaArmTaskspace.addVariableChangedListener(listener);
      kdArmTaskspace.addVariableChangedListener(listener);
      kiArmTaskspace.addVariableChangedListener(listener);
      maxIntegralErrorArmTaskspace.addVariableChangedListener(listener);
      
      listener.variableChanged(null);
   }

   public void setHasBeenInitialized(boolean hasBeenInitialized)
   {
      this.hasBeenInitialized.set(hasBeenInitialized);
   }

   private void createFrameVisualizers(DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry,
                                       SideDependentList<ReferenceFrame> framesToVisualize, String listName, boolean enable)
   {
      DynamicGraphicObjectsList list = new DynamicGraphicObjectsList(listName);
      if (dynamicGraphicObjectsListRegistry != null)
      {
         for (ReferenceFrame handPositionControlFrame : framesToVisualize)
         {
            if (handPositionControlFrame != null)
            {               DynamicGraphicReferenceFrame dynamicGraphicReferenceFrame = new DynamicGraphicReferenceFrame(handPositionControlFrame, registry, 0.1);
               dynamicGraphicReferenceFrames.add(dynamicGraphicReferenceFrame);
               list.add(dynamicGraphicReferenceFrame);
            }
         }
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjectsList(list);

         if (!enable)
            list.hideDynamicGraphicObjects();
      }
   }

   private void setUpStateMachine(DoubleYoVariable yoTime, FullRobotModel fullRobotModel, TwistCalculator twistCalculator,
                                  ManipulationControllerParameters parameters, final VariousWalkingProviders variousWalkingProviders,
                                  DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry,
                                  MomentumBasedController momentumBasedController,
                                  SideDependentList<ReferenceFrame> handPositionControlFrames, SideDependentList<Integer> jacobianIds, 
                                  ArmControllerParameters armControlParameters, ControlStatusProducer controlStatusProducer)
   {
      HandPoseProvider handPoseProvider = variousWalkingProviders.getDesiredHandPoseProvider();
      DesiredHandLoadBearingProvider handLoadBearingProvider = variousWalkingProviders.getDesiredHandLoadBearingProvider();

      individualHandControlModules = createIndividualHandControlModules(yoTime, fullRobotModel, twistCalculator, dynamicGraphicObjectsListRegistry,
              momentumBasedController, jacobianIds, armControlParameters, controlStatusProducer);

      
      directControlManipulationTaskDispatcher = new DirectControlManipulationTaskDispatcher(fullRobotModel, parameters, armControlParameters, handPoseProvider,
              handLoadBearingProvider, handPositionControlFrames, individualHandControlModules, pipeline, taskspaceControlGains, momentumBasedController,
              registry);
   }

   private SideDependentList<IndividualHandControlModule> createIndividualHandControlModules(DoubleYoVariable yoTime, FullRobotModel fullRobotModel,
           TwistCalculator twistCalculator, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry,
           MomentumBasedController momentumBasedController,
           SideDependentList<Integer> jacobianIds, ArmControllerParameters armControlParameters, ControlStatusProducer controlStatusProducer)
   {
      SideDependentList<IndividualHandControlModule> individualHandControlModules = new SideDependentList<IndividualHandControlModule>();

      for (RobotSide robotSide : RobotSide.values)
      {

         int jacobianId = jacobianIds.get(robotSide);

         IndividualHandControlModule individualHandControlModule = new IndividualHandControlModule(yoTime, robotSide, fullRobotModel, twistCalculator,
                                                                      dynamicGraphicObjectsListRegistry, midHandPositionControlFrames.get(robotSide),
                                                                      taskspaceControlGains, momentumBasedController, jacobianId, armControlParameters, controlStatusProducer, registry);
         individualHandControlModules.put(robotSide, individualHandControlModule);
      }

      return individualHandControlModules;
   }

   public void goToDefaultState()
   {
      directControlManipulationTaskDispatcher.goToDefaultState();
   }

   public void initialize()
   {
      doControl();
   }

   public void doControl()
   {
      // Important especially when switching between high level states. In such case, we don't want the arm to go to home position
      if (!hasBeenInitialized.getBooleanValue())
      {
         goToDefaultState();
         hasBeenInitialized.set(true);
      }

      updateGraphics();

      directControlManipulationTaskDispatcher.doAction();
      pipeline.doControl();

      for (RobotSide robotSide : RobotSide.values)
      {
         individualHandControlModules.get(robotSide).doControl();
      }

   }

   private void updateGraphics()
   {
      for (int i = 0; i < dynamicGraphicReferenceFrames.size(); i++)
      {
         dynamicGraphicReferenceFrames.get(i).update();
      }
   }

   public void prepareForLocomotion()
   {
      directControlManipulationTaskDispatcher.prepareForLocomotion();
   }

   public void submitTask(RobotSide robotSide, Task task)
   {
      pipeline.submit(robotSide, task);
   }

   public void submitTask(Task task)
   {
      pipeline.submit(task);
   }

   public void submitAll(List<Task> tasks)
   {
      pipeline.submitAll(tasks);
   }

   public void clear()
   {
      pipeline.clear();
   }

   public IndividualHandControlModule getIndividualHandControlModule(RobotSide robotSide)
   {
      return individualHandControlModules.get(robotSide);
   }

   public SideDependentList<IndividualHandControlModule> getIndividualHandControlModules()
   {
      return individualHandControlModules;
   }

   public SideDependentList<ReferenceFrame> getMidHandPositionControlFrames()
   {
      return midHandPositionControlFrames;
   }

   public void submitAll(RobotSide robotSide, List<Task> tasks)
   {
      pipeline.submitAll(robotSide, tasks);
   }
}
