package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation;

import java.util.ArrayList;
import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ManipulationControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.HandControllerInterface;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.IndividualHandControlModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.states.direct.DirectControlManipulationTaskDispatcher;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.states.fingerToroidManipulation.HighLevelFingerValveManipulationState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.taskExecutor.PipeLine;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.taskExecutor.Task;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredArmJointAngleProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredHandLoadBearingProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredHandPoseProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.FingerStateProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.TorusManipulationProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.TorusPoseProvider;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicReferenceFrame;

/**
 * @author twan
 *         Date: 5/13/13
 */
public class ManipulationControlModule
{
   private static final boolean DEBUG_TOROID = false;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final List<DynamicGraphicReferenceFrame> dynamicGraphicReferenceFrames = new ArrayList<DynamicGraphicReferenceFrame>();

   private DirectControlManipulationTaskDispatcher directControlManipulationTaskDispatcher;

   private final BooleanYoVariable hasBeenInitialized = new BooleanYoVariable("onFirstTick", registry);
   private SideDependentList<IndividualHandControlModule> individualHandControlModules;

   private final PipeLine<RobotSide> pipeline = new PipeLine<RobotSide>();
   private final SideDependentList<ReferenceFrame> midHandPositionControlFrames = new SideDependentList<ReferenceFrame>();
   private final SideDependentList<ReferenceFrame> fingerPositionControlFrames;
   private final SideDependentList<ReferenceFrame> barPositionControlFrames;
   private final SideDependentList<ReferenceFrame> creepyPositionControlFrames;
   private final SideDependentList<ReferenceFrame> knucklePositionControlFrames;
   private final SideDependentList<ReferenceFrame> fingersBentBackFrames;

   private final SideDependentList<HandControllerInterface> handControllers;

   public ManipulationControlModule(FullRobotModel fullRobotModel, TwistCalculator twistCalculator, final VariousWalkingProviders variousWalkingProviders,
                                    ArmControllerParameters armControlParameters, ManipulationControllerParameters parameters,
                                    SideDependentList<HandControllerInterface> handControllers, DoubleYoVariable yoTime,
                                    MomentumBasedController momentumBasedController, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry,
                                    YoVariableRegistry parentRegistry)
   {
      this.handControllers = handControllers;

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

      // hack because GFE robot specific:
      fingerPositionControlFrames = createFingerPositionControlFramesHack(endEffectors);
      barPositionControlFrames = createBarPositionControlFramesHack(midHandPositionControlFrames);
      creepyPositionControlFrames = createCreepyPositionControlFramesHack(midHandPositionControlFrames);
      knucklePositionControlFrames = createKnucklePositionControlFramesHack(midHandPositionControlFrames);

      // TODO: remove
      fingersBentBackFrames = new SideDependentList<ReferenceFrame>();
      for (RobotSide robotSide : RobotSide.values)
      {
         SideDependentList<ContactablePlaneBody> contactablePlaneHandsWithFingersBentBack = momentumBasedController
               .getContactablePlaneHandsWithFingersBentBack();
         if (contactablePlaneHandsWithFingersBentBack != null)
         {
            ContactablePlaneBody contactablePlaneHandWithFingersBentBack = contactablePlaneHandsWithFingersBentBack.get(robotSide);
            fingersBentBackFrames.put(robotSide, contactablePlaneHandWithFingersBentBack.getPlaneFrame());
         }
      }

      createFrameVisualizers(dynamicGraphicObjectsListRegistry, midHandPositionControlFrames, "midHandPositionControlFrames", false);
      createFrameVisualizers(dynamicGraphicObjectsListRegistry, fingerPositionControlFrames, "fingerPositionControlFrames", false);
      createFrameVisualizers(dynamicGraphicObjectsListRegistry, barPositionControlFrames, "barPositionControlFrames", false);
      createFrameVisualizers(dynamicGraphicObjectsListRegistry, creepyPositionControlFrames, "creepyPositionControlFrames", false);
      createFrameVisualizers(dynamicGraphicObjectsListRegistry, knucklePositionControlFrames, "knucklePositionControlFrames", false);
      createFrameVisualizers(dynamicGraphicObjectsListRegistry, fingersBentBackFrames, "fingersBentBackFrames", true);

      setUpStateMachine(yoTime, fullRobotModel, twistCalculator, parameters, variousWalkingProviders, dynamicGraphicObjectsListRegistry, handControllers,
                        momentumBasedController, midHandPositionControlFrames, jacobianIds, armControlParameters);

      parentRegistry.addChild(registry);
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
                                  SideDependentList<HandControllerInterface> handControllers, MomentumBasedController momentumBasedController,
                                  SideDependentList<ReferenceFrame> handPositionControlFrames, SideDependentList<Integer> jacobianIds, 
                                  ArmControllerParameters armControlParameters)
   {
      DesiredHandPoseProvider handPoseProvider = variousWalkingProviders.getDesiredHandPoseProvider();
      DesiredHandLoadBearingProvider handLoadBearingProvider = variousWalkingProviders.getDesiredHandLoadBearingProvider();
      TorusPoseProvider torusPoseProvider = variousWalkingProviders.getTorusPoseProvider();
      TorusManipulationProvider torusManipulationProvider = variousWalkingProviders.getTorusManipulationProvider();
      FingerStateProvider fingerStateProvider = variousWalkingProviders.getFingerStateProvider();
      DesiredArmJointAngleProvider armJointAngleProvider = variousWalkingProviders.getDesiredArmJointAngleProvider();


      individualHandControlModules = createIndividualHandControlModules(yoTime, fullRobotModel, twistCalculator, dynamicGraphicObjectsListRegistry,
              handControllers, momentumBasedController, jacobianIds, armControlParameters);

      final HighLevelFingerValveManipulationState fingerToroidManipulationState = new HighLevelFingerValveManipulationState(momentumBasedController,
                                                                                     fullRobotModel.getElevator(), torusPoseProvider,
                                                                                     torusManipulationProvider, handControllers, individualHandControlModules,
                                                                                     fingerPositionControlFrames, registry, dynamicGraphicObjectsListRegistry);

      directControlManipulationTaskDispatcher = new DirectControlManipulationTaskDispatcher(fullRobotModel, parameters, handPoseProvider,
              handLoadBearingProvider, fingerStateProvider, armJointAngleProvider, handControllers, handPositionControlFrames, individualHandControlModules,
              pipeline, fingerToroidManipulationState, torusManipulationProvider, momentumBasedController, registry);

//    HighLevelToroidManipulationState toroidManipulationState = new HighLevelToroidManipulationState(yoTime, fullRobotModel, twistCalculator,
//                                                                  handPositionControlFrames, handControllers, jacobians, torusPoseProvider,
//                                                                  momentumBasedController, dynamicGraphicObjectsListRegistry, parentRegistry);



      if (DEBUG_TOROID)
         fingerToroidManipulationState.setUpForDebugging();
   }

   private SideDependentList<IndividualHandControlModule> createIndividualHandControlModules(DoubleYoVariable yoTime, FullRobotModel fullRobotModel,
           TwistCalculator twistCalculator, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry,
           SideDependentList<HandControllerInterface> handControllers, MomentumBasedController momentumBasedController,
           SideDependentList<Integer> jacobianIds, ArmControllerParameters armControlParameters)
   {
      SideDependentList<IndividualHandControlModule> individualHandControlModules = new SideDependentList<IndividualHandControlModule>();

      for (RobotSide robotSide : RobotSide.values)
      {
         HandControllerInterface handControllerInterface = null;
         if (handControllers != null)
         {
            handControllerInterface = handControllers.get(robotSide);
         }


         int jacobianId = jacobianIds.get(robotSide);
         double gravityZ = momentumBasedController.getGravityZ();
         double controlDT = momentumBasedController.getControlDT();

         IndividualHandControlModule individualHandControlModule = new IndividualHandControlModule(yoTime, robotSide, fullRobotModel, twistCalculator,
                                                                      dynamicGraphicObjectsListRegistry, handControllerInterface, gravityZ, controlDT,
                                                                      momentumBasedController, jacobianId, armControlParameters, registry);
         individualHandControlModules.put(robotSide, individualHandControlModule);
      }

      return individualHandControlModules;
   }

   public void goToDefaultState()
   {
      directControlManipulationTaskDispatcher.goToDefaultState();
   }

   public void doControl()
   {
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

   public SideDependentList<ReferenceFrame> getFingerPositionControlFrames()
   {
      return fingerPositionControlFrames;
   }

   public SideDependentList<ReferenceFrame> getBarPositionControlFrames()
   {
      return barPositionControlFrames;
   }

   public SideDependentList<ReferenceFrame> getCreepyPositionControlFrames()
   {
      return creepyPositionControlFrames;
   }

   public SideDependentList<ReferenceFrame> getKnucklePositionControlFrames()
   {
      return knucklePositionControlFrames;
   }

   private SideDependentList<ReferenceFrame> createFingerPositionControlFramesHack(SideDependentList<RigidBody> endEffectors)
   {
      SideDependentList<ReferenceFrame> fingerPositionControlFrames = new SideDependentList<ReferenceFrame>();
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody indexFingerTip = getIndexFingerTipHack(endEffectors.get(robotSide));    // FIXME: total hack!

         // FIXME: reference frame is currently not updated based on finger angle. Angle is zero at all times:
         ReferenceFrame fingerTipFrameAfterJoint = indexFingerTip.getParentJoint().getFrameAfterJoint();
         Transform3D transform = new Transform3D();
         if (robotSide == RobotSide.LEFT)
         {
            transform.setEuler(new Vector3d(Math.PI, 0.0, 0.0));
         }

         transform.setTranslation(new Vector3d(0.057, 0.0, 0.0));    // TODO: hack: specific to DRC Sandia hands
         ReferenceFrame fingerPositionControlFrame =
            ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(robotSide.getCamelCaseNameForStartOfExpression() + "FingerPositionControlFrame",
               fingerTipFrameAfterJoint, transform);
         fingerPositionControlFrames.put(robotSide, fingerPositionControlFrame);
      }

      return fingerPositionControlFrames;
   }

   private RigidBody getIndexFingerTipHack(RigidBody endEffector)
   {
      RigidBody indexFingerTip = endEffector;

      while (indexFingerTip.hasChildrenJoints())
      {
         indexFingerTip = indexFingerTip.getChildrenJoints().get(0).getSuccessor();
      }

      return indexFingerTip;
   }

   private SideDependentList<ReferenceFrame> createBarPositionControlFramesHack(SideDependentList<ReferenceFrame> midHandPositionControlFrames)
   {
      SideDependentList<ReferenceFrame> ret = new SideDependentList<ReferenceFrame>();

      for (RobotSide robotSide : RobotSide.values)
      {
         ReferenceFrame midHandPositionControlFrame = midHandPositionControlFrames.get(robotSide);

         Transform3D preRotation = new Transform3D();
         preRotation.setEuler(new Vector3d(0.0, 0.670796, 0.0));

         Transform3D transform = new Transform3D();
         transform.setTranslation(new Vector3d(0.09, robotSide.negateIfRightSide(-0.03), 0.04));

         transform.mul(preRotation, transform);

         ReferenceFrame referenceFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(robotSide.getCamelCaseNameForStartOfExpression() +
               "Bar", midHandPositionControlFrame, transform);
         ret.put(robotSide, referenceFrame);
      }
      return ret;
   }


   private SideDependentList<ReferenceFrame> createCreepyPositionControlFramesHack(SideDependentList<ReferenceFrame> midHandPositionControlFrames)
   {
      SideDependentList<ReferenceFrame> ret = new SideDependentList<ReferenceFrame>();

      for (RobotSide robotSide : RobotSide.values)
      {
         ReferenceFrame midHandPositionControlFrame = midHandPositionControlFrames.get(robotSide);

         Transform3D preRotation = new Transform3D();
         preRotation.setEuler(new Vector3d(0.0, 0.670796, 0.0));

         Transform3D transform = new Transform3D();
         transform.setTranslation(new Vector3d(0.17, 0.0, 0.07));

         Transform3D postRotation = new Transform3D();
         postRotation.setEuler(new Vector3d(0.0, -0.3, 0.0));

         transform.mul(preRotation, transform);
         transform.mul(postRotation);

         ReferenceFrame referenceFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(robotSide.getCamelCaseNameForStartOfExpression() +
               "Creepy", midHandPositionControlFrame, transform);
         ret.put(robotSide, referenceFrame);
      }
      return ret;
   }


   private SideDependentList<ReferenceFrame> createKnucklePositionControlFramesHack(SideDependentList<ReferenceFrame> midHandPositionControlFrames)
   {
      SideDependentList<ReferenceFrame> ret = new SideDependentList<ReferenceFrame>();

      for (RobotSide robotSide : RobotSide.values)
      {
         ReferenceFrame midHandPositionControlFrame = midHandPositionControlFrames.get(robotSide);

         Transform3D preRotation = new Transform3D();
         preRotation.setEuler(new Vector3d(0.0, 0.670796, 0.0));

         Transform3D transform = new Transform3D();
         transform.setTranslation(new Vector3d(0.145, 0.0, 0.07));

//         Transform3D postRotation = new Transform3D();
//         postRotation.setEuler(new Vector3d(0.0, -0.3, 0.0));

         transform.mul(preRotation, transform);
//         transform.mul(postRotation);

         ReferenceFrame referenceFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(robotSide.getCamelCaseNameForStartOfExpression() +
               "Knuckle", midHandPositionControlFrame, transform);
         ret.put(robotSide, referenceFrame);
      }
      return ret;
   }


   public SideDependentList<HandControllerInterface> getHandControllers()
   {
      return handControllers;
   }

   public void submitAll(RobotSide robotSide, List<Task> tasks)
   {
      pipeline.submitAll(robotSide, tasks);
   }
}
