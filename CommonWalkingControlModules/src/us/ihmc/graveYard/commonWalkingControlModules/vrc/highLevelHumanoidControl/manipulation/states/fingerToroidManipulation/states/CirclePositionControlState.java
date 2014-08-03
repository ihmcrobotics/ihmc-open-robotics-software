package us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.manipulation.states.fingerToroidManipulation.states;

import us.ihmc.commonWalkingControlModules.controlModules.SE3PDGains;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlModule;
import us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.manipulation.states.fingerToroidManipulation.FingerToroidManipulationState;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.FormattingTools;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.trajectories.providers.ConstantDoubleProvider;
import us.ihmc.utilities.math.trajectories.providers.DoubleProvider;
import us.ihmc.utilities.math.trajectories.providers.OrientationProvider;
import us.ihmc.utilities.math.trajectories.providers.SE3ConfigurationProvider;
import us.ihmc.utilities.screwTheory.RigidBody;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.statemachines.State;
import com.yobotics.simulationconstructionset.util.trajectory.CirclePositionTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.OrientationInterpolationTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.OrientationTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.PositionTrajectoryGenerator;


public class CirclePositionControlState extends State<FingerToroidManipulationState>
{
   private final YoVariableRegistry registry;
   private final SideDependentList<PositionTrajectoryGenerator> positionTrajectoryGenerators = new SideDependentList<PositionTrajectoryGenerator>();
   private final SideDependentList<OrientationTrajectoryGenerator> orientationTrajectoryGenerators = new SideDependentList<OrientationTrajectoryGenerator>();
   private final SideDependentList<HandControlModule> individualHandControlModules;
   private final SideDependentList<ReferenceFrame> fingerPositionControlFrames;
   private final RigidBody rootBody;
   private final SE3PDGains gains;

   public CirclePositionControlState(FingerToroidManipulationState stateEnum, RigidBody rootBody, ReferenceFrame staticTorusFrame,
                                     SideDependentList<ReferenceFrame> handPositionControlFrames,
                                     SideDependentList<SE3ConfigurationProvider> initialConfigurationProviders,
                                     SideDependentList<? extends OrientationProvider> finalOrientationProviders, DoubleProvider desiredRotationAngleProvider,
                                     double trajectoryTime, SideDependentList<HandControlModule> individualHandControlModules,
                                     SideDependentList<ReferenceFrame> fingerPositionControlFrames, SE3PDGains gains, YoVariableRegistry parentRegistry)
   {
      super(stateEnum);
      registry = new YoVariableRegistry(FormattingTools.lowerCaseFirstLetter(stateEnum.toString()));

      this.rootBody = rootBody;
      this.individualHandControlModules = individualHandControlModules;
      this.fingerPositionControlFrames = fingerPositionControlFrames;

      String stateName = FormattingTools.underscoredToCamelCase(getStateEnum().toString(), true);
      for (final RobotSide robotSide : RobotSide.values)
      {
         String sideName = robotSide.getCamelCaseNameForStartOfExpression();

         CirclePositionTrajectoryGenerator circleTrajectoryGenerator = new CirclePositionTrajectoryGenerator(sideName + stateName + "CircleTrajectory",
                                                                          staticTorusFrame, new ConstantDoubleProvider(trajectoryTime),
                                                                          initialConfigurationProviders.get(robotSide), registry, desiredRotationAngleProvider);
         positionTrajectoryGenerators.put(robotSide, circleTrajectoryGenerator);

         DoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(trajectoryTime);

         ReferenceFrame handPositionControlFrame = handPositionControlFrames.get(robotSide);
         OrientationInterpolationTrajectoryGenerator orientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator(sideName + stateName
                                                                                         + "OrientationTrajectory", handPositionControlFrame,
                                                                                            trajectoryTimeProvider,
                                                                                            initialConfigurationProviders.get(robotSide),
                                                                                            finalOrientationProviders.get(robotSide), registry);
         orientationTrajectoryGenerator.setContinuouslyUpdateFinalOrientation(true);
         orientationTrajectoryGenerators.put(robotSide, orientationTrajectoryGenerator);
      }

      this.gains = gains;
      parentRegistry.addChild(registry);
   }

   @Override
   public void doAction()
   {
   }

   @Override
   public void doTransitionIntoAction()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         individualHandControlModules.get(robotSide).executeTaskSpaceTrajectory(positionTrajectoryGenerators.get(robotSide),
                                          orientationTrajectoryGenerators.get(robotSide), fingerPositionControlFrames.get(robotSide), rootBody, gains);
      }
   }

   @Override
   public void doTransitionOutOfAction()
   {
   }

   @Override
   public boolean isDone()
   {
      for (HandControlModule individualHandControlModule : individualHandControlModules)
      {
         if (!individualHandControlModule.isDone())
            return false;
      }

      return true;
   }

   public PositionTrajectoryGenerator getPositionTrajectoryGenerator(RobotSide robotSide)
   {
      return positionTrajectoryGenerators.get(robotSide);
   }

   public OrientationTrajectoryGenerator getOrientationTrajectoryGenerator(RobotSide robotSide)
   {
      return orientationTrajectoryGenerators.get(robotSide);
   }
}
