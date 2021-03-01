package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.DiscreteAngularVelocityOrientationCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.MPCCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.MPCCommandList;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.SE3MPCIndexHandler;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.AngularVelocityOrientationMPCTrajectoryHandler;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class AngularVelocitySE3ModelPredictiveController extends SE3ModelPredictiveController
{
   protected final YoVector3D currentBodyAngularVelocityError = new YoVector3D("currentBodyAngularVelocityError", registry);

   public AngularVelocitySE3ModelPredictiveController(Matrix3DReadOnly momentOfInertia,
                                                      double gravityZ,
                                                      double nominalCoMHeight,
                                                      double mass,
                                                      double dt,
                                                      YoRegistry parentRegistry)
   {
      this(new SE3MPCIndexHandler(numberOfBasisVectorsPerContactPoint),
           momentOfInertia,
           gravityZ,
           nominalCoMHeight,
           mass,
           dt,
           parentRegistry);

      qpSolver.setFirstOrientationVariableRegularization(1e1);
      qpSolver.setSecondOrientationVariableRegularization(1e0);
   }

   private AngularVelocitySE3ModelPredictiveController(SE3MPCIndexHandler indexHandler,
                                                       Matrix3DReadOnly momentOfInertia,
                                                       double gravityZ, double nominalCoMHeight, double mass, double dt,
                                                       YoRegistry parentRegistry)
   {
      super(indexHandler,
            new AngularVelocityOrientationMPCTrajectoryHandler(indexHandler),
            momentOfInertia, gravityZ, nominalCoMHeight, mass, dt,
            parentRegistry);
   }


   @Override
   protected void computeOrientationObjectives()
   {
      mpcCommands.addCommand(computeInitialDiscreteOrientationObjective(commandProvider.getNextDiscreteAngularVelocityOrientationCommand()));
      mpcCommands.addCommandList(computeDiscreteOrientationObjectives());
   }

   private final Matrix3D desiredRotation = new Matrix3D();
   private final Matrix3D transformedDesiredRotation = new Matrix3D();
   private final Matrix3D currentRotation = new Matrix3D();
   private final Matrix3D transformedCurrentRotation = new Matrix3D();
   private final FrameVector3D tempVector = new FrameVector3D();

   private MPCCommand<?> computeInitialDiscreteOrientationObjective(DiscreteAngularVelocityOrientationCommand objectiveToPack)
   {
      objectiveToPack.clear();
      objectiveToPack.setOmega(omega.getValue());
      objectiveToPack.setSegmentNumber(0);
      objectiveToPack.setDurationOfHold(indexHandler.getOrientationTickDuration(0));
      objectiveToPack.setTimeOfConstraint(0.0);
      objectiveToPack.setEndingDiscreteTickId(0);

      orientationTrajectoryHandler.compute(currentTimeInState.getDoubleValue());

      FrameOrientation3DReadOnly desiredOrientation = orientationTrajectoryHandler.getDesiredBodyOrientation();
      objectiveToPack.setDesiredBodyOrientation(desiredOrientation);
      // angular velocity in body frame
      tempVector.set(orientationTrajectoryHandler.getDesiredAngularVelocity());
      desiredOrientation.transform(tempVector);
      objectiveToPack.setDesiredBodyAngularVelocityInBodyFrame(tempVector);

      tempVector.setToZero();
      objectiveToPack.setDesiredNetAngularMomentumRate(tempVector);
      tempVector.set(orientationTrajectoryHandler.getDesiredInternalAngularMomentum());
      objectiveToPack.setDesiredInternalAngularMomentumRate(orientationTrajectoryHandler.getDesiredInternalAngularMomentumRate());

      objectiveToPack.setMomentOfInertiaInBodyFrame(momentOfInertia);

      objectiveToPack.setDesiredCoMPosition(currentCoMPosition);
      objectiveToPack.setDesiredCoMAcceleration(tempVector); // FIXME

      desiredOrientation.get(desiredRotation);
      currentBodyOrientation.get(currentRotation);

      desiredRotation.inverseTransform(currentRotation, transformedCurrentRotation);
      currentRotation.inverseTransform(desiredRotation, transformedDesiredRotation);
      transformedCurrentRotation.sub(transformedDesiredRotation);
      transformedCurrentRotation.scale(0.5);
      currentBodyAxisAngleError.setX(transformedCurrentRotation.getM21());
      currentBodyAxisAngleError.setY(transformedCurrentRotation.getM02());
      currentBodyAxisAngleError.setZ(transformedCurrentRotation.getM10());

      objectiveToPack.setCurrentAxisAngleError(currentBodyAxisAngleError);

      currentBodyAngularVelocityError.sub(currentBodyAngularVelocity, orientationTrajectoryHandler.getDesiredAngularVelocity());
      desiredOrientation.transform(currentBodyAngularVelocityError);
      objectiveToPack.setCurrentBodyAngularVelocityErrorInBodyFrame(currentBodyAngularVelocityError);

      for (int i = 0; i < contactPlaneHelperPool.get(0).size(); i++)
      {
         objectiveToPack.addContactPlaneHelper(contactPlaneHelperPool.get(0).get(i));
      }

      return objectiveToPack;
   }

   private final MPCCommandList commandList = new MPCCommandList();

   private MPCCommandList computeDiscreteOrientationObjectives()
   {
      commandList.clear();

      int tick = 1;
      int endTick = 0;
      double globalTime = currentTimeInState.getDoubleValue();
      for (int segment = 0; segment < indexHandler.getNumberOfSegments(); segment++)
      {
         endTick += indexHandler.getOrientationTicksInSegment(segment);
         double localTime = 0.0;
         for (; tick < endTick; tick++)
         {
            DiscreteAngularVelocityOrientationCommand objective = commandProvider.getNextDiscreteAngularVelocityOrientationCommand();

            localTime += indexHandler.getOrientationTickDuration(segment);
            globalTime += indexHandler.getOrientationTickDuration(segment);

            objective.clear();
            objective.setOmega(omega.getValue());
            objective.setSegmentNumber(segment);
            objective.setDurationOfHold(indexHandler.getOrientationTickDuration(segment));
            objective.setTimeOfConstraint(localTime);
            objective.setEndingDiscreteTickId(tick);

            linearTrajectoryHandler.compute(globalTime);
            orientationTrajectoryHandler.computeOutsidePreview(globalTime);


            FrameOrientation3DReadOnly desiredOrientation = orientationTrajectoryHandler.getDesiredBodyOrientationOutsidePreview();
            objective.setDesiredBodyOrientation(desiredOrientation);
            // angular velocity in body frame
            tempVector.set(orientationTrajectoryHandler.getDesiredBodyVelocityOutsidePreview());
            desiredOrientation.transform(tempVector);
            objective.setDesiredBodyAngularVelocityInBodyFrame(tempVector);

            tempVector.setToZero();
            objective.setDesiredNetAngularMomentumRate(tempVector);
            objective.setDesiredInternalAngularMomentumRate(orientationTrajectoryHandler.getDesiredInternalAngularMomentumRate());

            objective.setMomentOfInertiaInBodyFrame(momentOfInertia);

            objective.setDesiredCoMPosition(linearTrajectoryHandler.getDesiredCoMPosition());
            objective.setDesiredCoMAcceleration(linearTrajectoryHandler.getDesiredCoMAcceleration());

            for (int i = 0; i < contactPlaneHelperPool.get(0).size(); i++)
            {
               objective.addContactPlaneHelper(contactPlaneHelperPool.get(0).get(i));
            }

            commandList.addCommand(objective);
         }
      }

      return commandList;
   }
}
