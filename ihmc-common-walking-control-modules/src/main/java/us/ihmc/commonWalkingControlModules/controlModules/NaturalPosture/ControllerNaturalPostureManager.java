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

   public ControllerNaturalPostureManager(HumanoidRobotNaturalPosture robotNaturalPosture,
                                          PID3DGainsReadOnly gains,
                                          HighLevelHumanoidControllerToolbox controllerToolbox,
                                          YoRegistry registry)
   {
      yoTime = controllerToolbox.getYoTime();
      
      this.gains = gains;
      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      
      this.robotNaturalPosture = robotNaturalPosture;
      npQPobjective.reshape(3, 1);
      npQPjacobian.reshape(3,6+fullRobotModel.getOneDoFJoints().length);
      npQPweightMatrix.reshape(3, 3);
      npQPselectionMatrix.reshape(3, 3);
      CommonOps_DDRM.setIdentity(npQPselectionMatrix);
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
      
      for (int i=0; i<3; i++)
      {
//         npQPweightMatrix.set(i,i,naturalPostureAngularWeight.getElement(i));
         npQPweightMatrix.set(i,i,5.0);
      }
      
      // Get current NP:   GMN - we're assuming NP compute() is getting called somewhere else?
      currentNPQuat.set(robotNaturalPosture.getNaturalPostureQuaternion());
      
      // The NP servo:
      npYPR.setQuaternion(currentNPQuat.getX(),
                          currentNPQuat.getY(),
                          currentNPQuat.getZ(),
                          currentNPQuat.getS());
      
      double npYaw = npYPR.getYaw();
      double npPitch = npYPR.getPitch();
      double npRoll = npYPR.getRoll();
      
      double sbe = Math.sin(npPitch);
      double cbe = Math.cos(npPitch);
      double sal = Math.sin(npRoll);
      double cal = Math.cos(npRoll);
      Dnp.set(0,0,-sbe);     Dnp.set(0,1,0.0);   Dnp.set(0,2,1.0);
      Dnp.set(1,0, cbe*sal); Dnp.set(1,1, cal);  Dnp.set(1,2,0.0);
      Dnp.set(2,0, cbe*cal); Dnp.set(2,1,-sal);  Dnp.set(2,2,0.0);
      
//      double[] kp = gains.getProportionalGains();
      double[] kp = new double[] {10,10,10};
      tau.set(0,0,kp[0]*(0 - npYaw));
      tau.set(1,0,kp[1]*(0 - npPitch));
      tau.set(2,0,kp[2]*(0 - npRoll));
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