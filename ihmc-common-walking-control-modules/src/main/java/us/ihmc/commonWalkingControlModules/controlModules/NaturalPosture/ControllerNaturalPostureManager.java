package us.ihmc.commonWalkingControlModules.controlModules.NaturalPosture;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.commonWalkingControlModules.configurations.HumanoidRobotNaturalPosture;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.QPObjectiveCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.math.filters.FilteredVelocityYoVariable;
//import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ControllerNaturalPostureManager 
//public class ControllerNaturalPostureManager implements NaturalPostureControlState
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

//   private final FrameQuaternion desiredNaturalPosture = new FrameQuaternion();
//   private final FrameVector3D desiredNaturalPostureAngularVelocity = new FrameVector3D();

   private final YoDouble yoTime;
   private final double controlDT;

   private final QPObjectiveCommand naturalPostureControlCommand = new QPObjectiveCommand();
   private Vector3DReadOnly naturalPostureAngularWeight;
//   private final Vector3D tempWeight = new Vector3D();
//   private final SelectionMatrix3D selectionMatrix = new SelectionMatrix3D();

   private final PID3DGainsReadOnly gains;
   
   HumanoidRobotNaturalPosture robotNaturalPosture;
   private final DMatrixRMaj npQPobjective = new DMatrixRMaj(1,1);
   private final DMatrixRMaj npQPjacobian = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj npQPweightMatrix = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj npQPselectionMatrix = new DMatrixRMaj(1, 1);
   private final Quaternion currentNPQuat = new Quaternion(0,0,0,1);
   private final YawPitchRoll npYPR = new YawPitchRoll();
   private final DMatrixRMaj tau = new DMatrixRMaj(3,1);
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

      this.gains = gains;
      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();

      this.robotNaturalPosture = robotNaturalPosture;
      npQPobjective.reshape(3, 1);
      npQPjacobian.reshape(3,6+fullRobotModel.getOneDoFJoints().length);
      npQPweightMatrix.reshape(3, 3);
      npQPselectionMatrix.reshape(3, 3);
      CommonOps_DDRM.setIdentity(npQPselectionMatrix);

      npQPWeightX.set(5.0);
      npQPWeightY.set(5.0);
      npQPWeightZ.set(5.0);

      npKpYaw.set(50.0);
      npKpPitch.set(50.0);
      npKpRoll.set(50.0);

      npKdYaw.set(10.0);
      npKdPitch.set(10.0);
      npKdRoll.set(10.0);

      npVelocityAlpha.set(0.01);

      parentRegistry.addChild(registry);
   }
   
   public void setWeights(Vector3DReadOnly naturalPostureAngularWeight)
   {
      this.naturalPostureAngularWeight = naturalPostureAngularWeight;
   }

//   public void setSelectionMatrix(SelectionMatrix3D selectionMatrix)
//   {
//      this.selectionMatrix.set(selectionMatrix);
//   }

//   @Override
//   public void doAction(double timeInState)
   public void compute()
   {
      // POPULATE QP MATRICES HERE:
      
      npQPweightMatrix.set(0, 0, npQPWeightX.getValue());
      npQPweightMatrix.set(1, 1, npQPWeightY.getValue());
      npQPweightMatrix.set(2, 2, npQPWeightZ.getValue());

//      for (int i=0; i<3; i++)
//      {
////         npQPweightMatrix.set(i,i,naturalPostureAngularWeight.getElement(i));
//         npQPweightMatrix.set(i,i,5.0);
//      }
      
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
      
      //TODO: A quaternion can be used to transform a matrix. Should be able to use currentNPQuat to transform tau, instead of needing to make this matrix. 
      //TODO: This seems to be swapping yaw(z) and roll (x). Shouldn't need to do that if you do things in x-y-z order (roll-pitch-yaw).
      Dnp.set(0,0,-sbe);     Dnp.set(0,1,0.0);   Dnp.set(0,2,1.0);
      Dnp.set(1,0, cbe*sal); Dnp.set(1,1, cal);  Dnp.set(1,2,0.0);
      Dnp.set(2,0, cbe*cal); Dnp.set(2,1,-sal);  Dnp.set(2,2,0.0);

//      double[] kp = gains.getProportionalGains();
//      double[] kp = new double[] {100.0, 0.0, 0.0};

      npYawAcceleration.set(npKpYaw.getValue()*(npYawDesired.getValue() - npYaw.getValue()) - npKdYaw.getValue() * npYawVelocity.getValue());
      npPitchAcceleration.set(npKpPitch.getValue()*(npPitchDesired.getValue() - npPitch.getValue()) - npKdPitch.getValue() * npPitchVelocity.getValue());
      npRollAcceleration.set(npKpRoll.getValue()*(npRollDesired.getValue() - npRoll.getValue()) - npKdRoll.getValue() * npRollVelocity.getValue());

      tau.set(0, 0, npYawAcceleration.getValue());
      tau.set(1, 0, npPitchAcceleration.getValue());
      tau.set(2, 0, npRollAcceleration.getValue());
 
      // GMN: derivative terms???

   // GMN: WE SHOULD INTERPRET THIS AS AN OMEGA COMMAND....?      
//      CommonOps_DDRM.invert(Dnp);       
//      CommonOps_DDRM.transpose(Dnp);    
      CommonOps_DDRM.mult(Dnp, tau, npQPobjective);
      
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
}