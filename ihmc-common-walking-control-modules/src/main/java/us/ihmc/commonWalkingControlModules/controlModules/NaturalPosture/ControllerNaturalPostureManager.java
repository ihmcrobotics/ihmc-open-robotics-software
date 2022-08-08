package us.ihmc.commonWalkingControlModules.controlModules.NaturalPosture;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.commonWalkingControlModules.configurations.HumanoidRobotNaturalPosture;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.QPObjectiveCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.math.filters.FilteredVelocityYoVariable;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ControllerNaturalPostureManager 
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final double controlDT;

   private final QPObjectiveCommand naturalPostureControlCommand = new QPObjectiveCommand();

   HumanoidRobotNaturalPosture robotNaturalPosture;
   private final DMatrixRMaj npQPobjective = new DMatrixRMaj(1,1);
   private final DMatrixRMaj npQPjacobian = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj npQPweightMatrix = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj npQPselectionMatrix = new DMatrixRMaj(1, 1);
   private final Quaternion currentNPQuat = new Quaternion(0,0,0,1);
   private final YawPitchRoll npYPR = new YawPitchRoll();
   private final DMatrixRMaj yprDDot = new DMatrixRMaj(3,1);
   private final DMatrixRMaj Dnp = new DMatrixRMaj(3,3);

   private final YoDouble npYaw = new YoDouble("npYaw", registry);
   private final YoDouble npPitch = new YoDouble("npPitch", registry);
   private final YoDouble npRoll = new YoDouble("npRoll", registry);

   private final YoDouble npVelocityAlpha = new YoDouble("npVelocityAlpha", registry);

   private final FilteredVelocityYoVariable npYawVelocity, npPitchVelocity, npRollVelocity;

   private final YoDouble npKpYaw = new YoDouble("npKpYaw", registry);
   private final YoDouble npKpPitch = new YoDouble("npKpPitch", registry);
   private final YoDouble npKpRoll = new YoDouble("npKpRoll", registry);

   private final YoDouble npKdYaw = new YoDouble("npKdYaw", registry);
   private final YoDouble npKdPitch = new YoDouble("npKdPitch", registry);
   private final YoDouble npKdRoll = new YoDouble("npKdRoll", registry);

   private final YoDouble npQPWeightX = new YoDouble("npQPWeightX", registry);
   private final YoDouble npQPWeightY = new YoDouble("npQPWeightY", registry);
   private final YoDouble npQPWeightZ = new YoDouble("npQPWeightZ", registry);

   private final YoDouble npYawDesired = new YoDouble("npYawDesired", registry);
   private final YoDouble npPitchDesired = new YoDouble("npPitchDesired", registry);
   private final YoDouble npRollDesired = new YoDouble("npRollDesired", registry);

   private final YoDouble npYawAcceleration = new YoDouble("npYawAcceleration", registry);
   private final YoDouble npPitchAcceleration = new YoDouble("npPitchAcceleration", registry);
   private final YoDouble npRollAcceleration = new YoDouble("npRollAcceleration", registry);

   
   private final QPObjectiveCommand pelvisQPObjectiveCommand = new QPObjectiveCommand();

   private final YoDouble pPosePelvisYaw = new YoDouble("pPosePelvisYaw", registry);
   private final YoDouble pPosePelvisPitch = new YoDouble("pPosePelvisPitch", registry);
   private final YoDouble pPosePelvisRoll = new YoDouble("pPosePelvisRoll", registry);
   private final YoDouble pPosePelvisYawKp = new YoDouble("pPosePelvisYawKp", registry);
   private final YoDouble pPosePelvisPitchKp = new YoDouble("pPosePelvisPitchKp", registry);
   private final YoDouble pPosePelvisRollKp = new YoDouble("pPosePelvisRollKp", registry);
   private final YoDouble pPosePelvisYawKdFactor = new YoDouble("pPosePelvisYawKdFactor", registry);
   private final YoDouble pPosePelvisPitchKdFactor = new YoDouble("pPosePelvisPitchKdFactor", registry);
   private final YoDouble pPosePelvisRollKdFactor = new YoDouble("pPosePelvisRollKdFactor", registry);
   private final DMatrixRMaj pelvisQPobjective = new DMatrixRMaj(1,1);
   private final DMatrixRMaj pelvisQPjacobian = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj pelvisQPweightMatrix = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj pelvisQPselectionMatrix = new DMatrixRMaj(1, 1);
   private final YawPitchRoll pelvisYPR = new YawPitchRoll();
   private final DMatrixRMaj pelvisYPRdot = new DMatrixRMaj(3,1);
   private final FrameVector3D pelvisOmegaVec = new FrameVector3D();
   private final DMatrixRMaj pelvisOmega = new DMatrixRMaj(3,1);
//   private final DMatrixRMaj pelvisAlpha = new DMatrixRMaj(3,1);
   private final DMatrixRMaj Dpelvis = new DMatrixRMaj(3,3);
   private final DMatrixRMaj invDpelvis = new DMatrixRMaj(3,3);
   
   private final YoDouble pelvisYawAcceleration = new YoDouble("pelvisYawAcceleration", registry);
   private final YoDouble pelvisPitchAcceleration = new YoDouble("pelvisPitchAcceleration", registry);
   private final YoDouble pelvisRollAcceleration = new YoDouble("pelvisRollAcceleration", registry);

   
   
   private final FullHumanoidRobotModel fullRobotModel;

   private YoDouble yoTime;

   public ControllerNaturalPostureManager(HumanoidRobotNaturalPosture robotNaturalPosture,
                                          PID3DGainsReadOnly gains,
                                          HighLevelHumanoidControllerToolbox controllerToolbox,
                                          YoRegistry parentRegistry)
   {
      yoTime = controllerToolbox.getYoTime();
      controlDT = controllerToolbox.getControlDT();

      npYawVelocity = new FilteredVelocityYoVariable("npYawVelocity", "", npVelocityAlpha, npYaw, controlDT, registry);
      npPitchVelocity = new FilteredVelocityYoVariable("npPitchVelocity", "", npVelocityAlpha, npPitch, controlDT, registry);
      npRollVelocity = new FilteredVelocityYoVariable("npRollVelocity", "", npVelocityAlpha, npRoll, controlDT, registry);

//      this.gains = gains;
      fullRobotModel = controllerToolbox.getFullRobotModel();

      this.robotNaturalPosture = robotNaturalPosture;
      npQPobjective.reshape(3, 1);
      npQPjacobian.reshape(3,6+fullRobotModel.getOneDoFJoints().length);
      npQPweightMatrix.reshape(3, 3);
      npQPselectionMatrix.reshape(3, 3);
      CommonOps_DDRM.setIdentity(npQPselectionMatrix);
      
      // Desired NP values (wrt world)
      npPitchDesired.set(-0.03);
      
      double qpWeight = 5.0;  //5.0;
      npQPWeightX.set(qpWeight);
      npQPWeightY.set(qpWeight);
      npQPWeightZ.set(qpWeight);

      npKpYaw.set(150.0);
      npKpPitch.set(50.0);
      npKpRoll.set(150.0);

      npKdYaw.set(40.0);
      npKdPitch.set(10.0);
      npKdRoll.set(40.0);

      npVelocityAlpha.set(0.01);


      pelvisQPobjective.reshape(3, 1);
      pelvisQPjacobian.reshape(3,6+fullRobotModel.getOneDoFJoints().length);
      pelvisQPweightMatrix.reshape(3, 3);
      pelvisQPselectionMatrix.reshape(3, 3);
      CommonOps_DDRM.setIdentity(pelvisQPselectionMatrix);
      pPosePelvisYaw.set(0.0);
      pPosePelvisPitch.set(0.02);
      pPosePelvisRoll.set(0.0);
      pPosePelvisYawKp.set(1000.0);
      pPosePelvisPitchKp.set(3000);
      pPosePelvisRollKp.set(1500.0);
      pPosePelvisYawKdFactor.set(0.15);
      pPosePelvisPitchKdFactor.set(0.15);
      pPosePelvisRollKdFactor.set(0.15);      
      
      parentRegistry.addChild(registry);
   }
   
//   public void setWeights(Vector3DReadOnly naturalPostureAngularWeight)
//   {
//      this.naturalPostureAngularWeight = naturalPostureAngularWeight;
//   }

//   public void setSelectionMatrix(SelectionMatrix3D selectionMatrix)
//   {
//      this.selectionMatrix.set(selectionMatrix);
//   }

//   @Override
//   public void doAction(double timeInState)
   public void compute()
   {
      // Set QP objective for pelvis privileged pose:
      pelvisPrivilegedPoseQPObjectiveCommand();
      
      // POPULATE QP MATRICES HERE:
      
      npQPweightMatrix.set(0, 0, npQPWeightX.getValue());
      npQPweightMatrix.set(1, 1, npQPWeightY.getValue());
      npQPweightMatrix.set(2, 2, npQPWeightZ.getValue());

      // Get current NP:   GMN - we're assuming NP compute() is getting called somewhere else?
      currentNPQuat.set(robotNaturalPosture.getNaturalPostureQuaternion());
      
      // The NP servo:
      npYPR.setQuaternion(currentNPQuat.getX(),
                          currentNPQuat.getY(),
                          currentNPQuat.getZ(),
                          currentNPQuat.getS());
      
      npYaw.set(npYPR.getYaw());
      npPitch.set(npYPR.getPitch());
      npRoll.set(npYPR.getRoll());
      
      npYawVelocity.update();
      npPitchVelocity.update();
      npRollVelocity.update();

      double sbe = Math.sin(npPitch.getValue());
      double cbe = Math.cos(npPitch.getValue());
      double sal = Math.sin(npRoll.getValue());
      double cal = Math.cos(npRoll.getValue());
      
      // Dnp:
      //   + is *not* a 3x3 orthogonal transformation
      //   + is a velocity transformation: omega = Dnp * (Euler_rates)
      //   + the order of the Euler angles is required to derive the correct transformation
      //   + (i.e. we cannot find this transform from the quaternion only, since you also need the choice of Euler set)
      //   + the transformations for RPY & YPR are different, as the RPY Euler set is a different orientation representation
      //   + to avoid confusion, keep variable name & math consistent: keep the Y,P,R = 0,1,2 order when using YPR Euler sets
      //   + BUT the result: omega will be {x,y,z}
      Dnp.set(0,0,-sbe);     Dnp.set(0,1,0.0);   Dnp.set(0,2,1.0);
      Dnp.set(1,0, cbe*sal); Dnp.set(1,1, cal);  Dnp.set(1,2,0.0);
      Dnp.set(2,0, cbe*cal); Dnp.set(2,1,-sal);  Dnp.set(2,2,0.0);

//      double[] kp = gains.getProportionalGains();
//      double[] kp = new double[] {100.0, 0.0, 0.0};

      npYawAcceleration.set(npKpYaw.getValue()*(npYawDesired.getValue() - npYaw.getValue()) - npKdYaw.getValue() * npYawVelocity.getValue());
      npPitchAcceleration.set(npKpPitch.getValue()*(npPitchDesired.getValue() - npPitch.getValue()) - npKdPitch.getValue() * npPitchVelocity.getValue());
      npRollAcceleration.set(npKpRoll.getValue()*(npRollDesired.getValue() - npRoll.getValue()) - npKdRoll.getValue() * npRollVelocity.getValue());

      yprDDot.set(0, 0, npYawAcceleration.getValue());
      yprDDot.set(1, 0, npPitchAcceleration.getValue());
      yprDDot.set(2, 0, npRollAcceleration.getValue());
 
      // GMN: derivative terms???

      CommonOps_DDRM.mult(Dnp, yprDDot, npQPobjective);  // GMN: missing D-dot term (since InvDyn takes accels)
      
      npQPjacobian.set(robotNaturalPosture.getNaturalPostureJacobian());
      
      // Populate the QPObjectiveCommand:
      naturalPostureControlCommand.getObjective().set(npQPobjective);
      naturalPostureControlCommand.getJacobian().set(npQPjacobian);
      naturalPostureControlCommand.getSelectionMatrix().set(npQPselectionMatrix);
      naturalPostureControlCommand.getWeightMatrix().set(npQPweightMatrix);
   }

//   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return naturalPostureControlCommand;
   }
   
//   @Override
//   public QPObjectiveCommand getQPObjectiveCommand()
//   {
//      return naturalPostureControlCommand;
//   }
   
   // Implements a YPR servo on the pelvis, which is then used for the privileged
   // pose of the pelvis (via task null-space projection)
   private void pelvisPrivilegedPoseQPObjectiveCommand()
   {
//      pelvisQPweightMatrix.set(0, 0, pelvisQPWeightX.getValue());
//      pelvisQPweightMatrix.set(1, 1, pelvisQPWeightY.getValue());
//      pelvisQPweightMatrix.set(2, 2, pelvisQPWeightZ.getValue());
      pelvisQPweightMatrix.set(0, 0, 1.0);
      pelvisQPweightMatrix.set(1, 1, 1.0);
      pelvisQPweightMatrix.set(2, 2, 1.0);

      // Get current pelvis YPR and omega:
      pelvisYPR.set(fullRobotModel.getPelvis().getBodyFixedFrame().getTransformToWorldFrame().getRotation());
      pelvisOmegaVec.setIncludingFrame(fullRobotModel.getPelvis().getBodyFixedFrame().getTwistOfFrame().getAngularPart()); 
      // Ugh...
      pelvisOmega.set(0,0,pelvisOmegaVec.getX());
      pelvisOmega.set(1,0,pelvisOmegaVec.getY());
      pelvisOmega.set(2,0,pelvisOmegaVec.getZ());
      
      double sbe = Math.sin(pelvisYPR.getPitch());
      double cbe = Math.cos(pelvisYPR.getPitch());
      double sal = Math.sin(pelvisYPR.getRoll());
      double cal = Math.cos(pelvisYPR.getRoll());
      Dpelvis.set(0,0,-sbe);     Dpelvis.set(0,1,0.0);   Dpelvis.set(0,2,1.0);
      Dpelvis.set(1,0, cbe*sal); Dpelvis.set(1,1, cal);  Dpelvis.set(1,2,0.0);
      Dpelvis.set(2,0, cbe*cal); Dpelvis.set(2,1,-sal);  Dpelvis.set(2,2,0.0);
      
      CommonOps_DDRM.invert(Dpelvis, invDpelvis);
      CommonOps_DDRM.mult(invDpelvis, pelvisOmega, pelvisYPRdot); // pelvis YPR rates

      // The pelvis equilibrium pose servo:
      pelvisYawAcceleration.set(  pPosePelvisYawKp.getValue()  *(pPosePelvisYaw.getValue() - pelvisYPR.getYaw())   - 
                                  pPosePelvisYawKdFactor.getValue()*pPosePelvisYawKp.getValue()  * pelvisYPRdot.get(0,0));
      pelvisPitchAcceleration.set(pPosePelvisPitchKp.getValue()*(pPosePelvisPitch.getValue() - pelvisYPR.getPitch()) - 
                                  pPosePelvisPitchKdFactor.getValue()*pPosePelvisPitchKp.getValue()* pelvisYPRdot.get(1,0));
      pelvisRollAcceleration.set( pPosePelvisRollKp.getValue() *(pPosePelvisRoll.getValue() - pelvisYPR.getRoll())  - 
                                  pPosePelvisRollKdFactor.getValue()*pPosePelvisRollKp.getValue() * pelvisYPRdot.get(2,0));
      
      yprDDot.set(0, 0, pelvisYawAcceleration.getValue());
      yprDDot.set(1, 0, pelvisPitchAcceleration.getValue());
      yprDDot.set(2, 0, pelvisRollAcceleration.getValue());
 
      CommonOps_DDRM.mult(Dpelvis, yprDDot, pelvisQPobjective);  // GMN: missing D-dot*yprDot term
      
      pelvisQPjacobian.zero(); // GMN: necessary??
      pelvisQPjacobian.set(0,0,1.0);
      pelvisQPjacobian.set(1,1,1.0);
      pelvisQPjacobian.set(2,2,1.0);
      
      // Populate the QPObjectiveCommand:
      pelvisQPObjectiveCommand.setDoNullSpaceProjection(true);
      pelvisQPObjectiveCommand.getObjective().set(pelvisQPobjective);
      pelvisQPObjectiveCommand.getJacobian().set(pelvisQPjacobian);
      pelvisQPObjectiveCommand.getSelectionMatrix().set(pelvisQPselectionMatrix);
      pelvisQPObjectiveCommand.getWeightMatrix().set(pelvisQPweightMatrix);
   }
   
   public InverseDynamicsCommand<?> getPelvisPrivilegedPoseCommand()
   {
      return pelvisQPObjectiveCommand;
   }
}