package us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.manipulation.states.toroidManipulation.states;

import us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.manipulation.ManipulableToroid;
import us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.manipulation.states.toroidManipulation.ToroidControlModule;
import us.ihmc.robotics.FormattingTools;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;


public class RotateToroidState<T extends Enum<T>> extends ToroidManipulationStateInterface<T>
{

   private final YoVariableRegistry registry;

   private final SideDependentList<SpatialAccelerationVector> handAccelerations = new SideDependentList<SpatialAccelerationVector>();
   private final SideDependentList<Wrench> handWrenches = new SideDependentList<Wrench>();
   private final SideDependentList<RigidBody> hands;

   private final ToroidControlModule toroidControlModule;
   private final SpatialAccelerationVector toroidSpatialAcceleration = new SpatialAccelerationVector();


   private final SpatialForceVector toroidWrench = new SpatialForceVector();
   private final DoubleYoVariable desiredToroidAngle;


   public RotateToroidState(T stateEnum, ManipulableToroid toroidUpdater, SideDependentList<RigidBody> hands,
                            double gravityZ, YoVariableRegistry parentRegistry)
   {
      super(stateEnum);

      this.toroidControlModule = new ToroidControlModule(toroidUpdater, parentRegistry);
      toroidControlModule.setProportionalGain(100.0);
      toroidControlModule.setDerivativeGain(20.0);

      String stateName = FormattingTools.underscoredToCamelCase(stateEnum.toString(), false) + "State";
      registry = new YoVariableRegistry(stateName);
      this.hands = hands;
      MathTools.checkIfInRange(gravityZ, 0.0, Double.POSITIVE_INFINITY);


      for (RobotSide robotSide : RobotSide.values)
      {
         ReferenceFrame handBodyFixedFrame = hands.get(robotSide).getBodyFixedFrame();

         handAccelerations.put(robotSide, new SpatialAccelerationVector());
         handWrenches.put(robotSide, new Wrench(handBodyFixedFrame, handBodyFixedFrame));
      }

      desiredToroidAngle = new DoubleYoVariable("desiredToroidAngle", registry);

      parentRegistry.addChild(registry);
   }

   @Override
   public SpatialAccelerationVector getDesiredHandAcceleration(RobotSide robotSide)
   {
      return handAccelerations.get(robotSide);
   }

   @Override
   public Wrench getHandExternalWrench(RobotSide robotSide)
   {
      return handWrenches.get(robotSide);
   }

   @Override
   public boolean isDone()
   {
      return true;
   }

   @Override
   public void doAction()
   {
      toroidControlModule.doControl(desiredToroidAngle.getDoubleValue(), 0.0, 0.0);

      toroidControlModule.get(toroidSpatialAcceleration);
      toroidControlModule.get(toroidWrench);

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody hand = hands.get(robotSide);
         RigidBody rootBody = ScrewTools.getRootBody(hand);
         SpatialAccelerationVector handSpatialAcceleration = handAccelerations.get(robotSide);
         packHandAccelerationGivenToolAcceleration(handSpatialAcceleration, toroidSpatialAcceleration, robotSide);
         handSpatialAcceleration.changeBaseFrameNoRelativeAcceleration(rootBody.getBodyFixedFrame());

         Wrench wrench = handWrenches.get(robotSide);
         wrench.set(toroidWrench);
         wrench.scale(0.5);
         wrench.changeFrame(hand.getBodyFixedFrame());
      }
   }

   private void packHandAccelerationGivenToolAcceleration(SpatialAccelerationVector handAccelerationToPack, SpatialAccelerationVector toolAcceleration,
         RobotSide robotSide)
   {
      ReferenceFrame endEffectorFrame = hands.get(robotSide).getBodyFixedFrame();//handPositionControlFrames.get(robotSide);
      handAccelerationToPack.set(toolAcceleration);
      handAccelerationToPack.changeBodyFrameNoRelativeAcceleration(endEffectorFrame);
      handAccelerationToPack.changeFrameNoRelativeMotion(endEffectorFrame);
   }

   @Override
   public void doTransitionIntoAction()
   {
   }

   @Override
   public void doTransitionOutOfAction()
   {
   }

}
