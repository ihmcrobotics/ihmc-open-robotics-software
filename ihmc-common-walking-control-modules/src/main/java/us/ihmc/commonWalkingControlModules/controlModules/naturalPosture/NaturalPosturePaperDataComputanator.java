package us.ihmc.commonWalkingControlModules.controlModules.naturalPosture;

import org.ejml.data.DMatrixRBlock;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.block.linsol.qr.QrHouseHolderSolver_DDRB;
import org.ejml.ops.ConvertDMatrixStruct;
import us.ihmc.commonWalkingControlModules.configurations.HumanoidRobotNaturalPosture;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.algorithms.CentroidalMomentumCalculator;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class NaturalPosturePaperDataComputanator
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final HumanoidRobotNaturalPosture robotNaturalPosture;
   private final FullHumanoidRobotModel fullRobotModel;

   private final YoVector3D relativeAngularVel = new YoVector3D("relativeAngularVelZ", registry);
   private final YoVector3D omega_bc = new YoVector3D("omega_bc", registry);
   private final YoVector3D centroidalAngularMomentumApproxByACOM = new YoVector3D("centroidalAngularMomentumApproxByACOM", registry);

   /**
    * This class contains the code which was used to generate data for the Angular Center of Mass paper [REF: Y.-M. Chen, G. Nelson, R. Griffin, M. Posa, and
    * J. Pratt, "Angular Center of Mass for Humanoid Robots." 2022. http://arxiv.org/abs/2210.08111]. It should not be used for anything else, and should
    * probably be moved to a test.
    * TODO move this to a test.
    */
   public NaturalPosturePaperDataComputanator(HumanoidRobotNaturalPosture robotNaturalPosture,
                                              HighLevelHumanoidControllerToolbox controllerToolbox,
                                              YoRegistry parentRegistry)
   {
      this.robotNaturalPosture = robotNaturalPosture;
      fullRobotModel = controllerToolbox.getFullRobotModel();
      parentRegistry.addChild(registry);
   }

   public void computeDataForPaper()
   {
      // We would like to compare the relative angular velocity to the angular velocity of the ACOM frame
      // We also want to compare the CAM to the one approximated by ACOM

      fullRobotModel.getElevator().updateFramesRecursively();
      MomentumData momentumData = computeMomentum(fullRobotModel);

      DMatrixRMaj relativeVel = MatrixTools.mult(momentumData.connectionMatrix, momentumData.jointVelocity);

      relativeAngularVel.setX(relativeVel.get(0));
      relativeAngularVel.setY(relativeVel.get(1));
      relativeAngularVel.setZ(relativeVel.get(2));

      DMatrixRMaj b_omega_bc = MatrixTools.mult(robotNaturalPosture.getCenterOfMassOrientationJacobianRelativeToBase(), momentumData.jointVelocity);
      omega_bc.setX(b_omega_bc.get(0));
      omega_bc.setY(b_omega_bc.get(1));
      omega_bc.setZ(b_omega_bc.get(2));

      DMatrixRMaj b_omega_wb = new DMatrixRMaj(3, 1);
      b_omega_wb.set(0, momentumData.jointVelocityWithFloatingBase.get(0));
      b_omega_wb.set(1, momentumData.jointVelocityWithFloatingBase.get(1));
      b_omega_wb.set(2, momentumData.jointVelocityWithFloatingBase.get(2));
      DMatrixRMaj b_omega_wc = new DMatrixRMaj(3, 1);
      b_omega_wc.set(0, b_omega_wb.get(0) + b_omega_bc.get(0));
      b_omega_wc.set(1, b_omega_wb.get(1) + b_omega_bc.get(1));
      b_omega_wc.set(2, b_omega_wb.get(2) + b_omega_bc.get(2));

      DMatrixRMaj centroidalMomentumApproxByACOM = new DMatrixRMaj(6, 1);
      DMatrixRMaj Mbase = new DMatrixRMaj(6, 3);
      int[] srcColumnsBase = {0, 1, 2};
      MatrixTools.extractColumns(momentumData.momentumMatrix, srcColumnsBase, Mbase, 0);
      centroidalMomentumApproxByACOM = MatrixTools.mult(Mbase, b_omega_wc);

      centroidalAngularMomentumApproxByACOM.setX(centroidalMomentumApproxByACOM.get(0));
      centroidalAngularMomentumApproxByACOM.setY(centroidalMomentumApproxByACOM.get(1));
      centroidalAngularMomentumApproxByACOM.setZ(centroidalMomentumApproxByACOM.get(2));
   }

   private static class MomentumData
   {
      public DMatrixRMaj jointPositionWithFloatingBase;
      public DMatrixRMaj jointVelocityWithFloatingBase;
      public DMatrixRMaj jointPosition;
      public DMatrixRMaj jointVelocity;
      public DMatrixRMaj momentumMatrix;
      public DMatrixRMaj momentumVector;
      public DMatrixRMaj connectionMatrix;

      public MomentumData(DMatrixRMaj jointPositionWithFloatingBase,
                          DMatrixRMaj jointVelocityWithFloatingBase,
                          DMatrixRMaj jointPosition,
                          DMatrixRMaj jointVelocity,
                          DMatrixRMaj momentumMatrix,
                          DMatrixRMaj momentumVector,
                          DMatrixRMaj connectionMatrix)
      {
         this.jointPositionWithFloatingBase = jointPositionWithFloatingBase;
         this.jointVelocityWithFloatingBase = jointVelocityWithFloatingBase;
         this.jointPosition = jointPosition;
         this.jointVelocity = jointVelocity;
         this.momentumMatrix = momentumMatrix;
         this.momentumVector = momentumVector;
         this.connectionMatrix = connectionMatrix;
      }
   }

   private MomentumData computeMomentum(FullHumanoidRobotModel fullRobotModel)
   {
      // TODO: double check if the joint ordering is the same as centroidal momentum matirx's.
      // TODO: also double check if you should use pelvis as root or elevator
      //       Ans: I believe if we want to get the momentum rt world frame, then we should use elevator, because the floating base vel also contribute to the centroidal angular momentum

      JointBasics[] jointListWithFloatingBase = MultiBodySystemTools.collectSubtreeJoints(fullRobotModel.getElevator());
      DMatrixRMaj jointPositionWithFloatingBase = new DMatrixRMaj(MultiBodySystemTools.computeDegreesOfFreedom(jointListWithFloatingBase) + 1, 1);
      MultiBodySystemTools.extractJointsState(jointListWithFloatingBase, JointStateType.CONFIGURATION, jointPositionWithFloatingBase);
      DMatrixRMaj jointVelocityWithFloatingBase = new DMatrixRMaj(MultiBodySystemTools.computeDegreesOfFreedom(jointListWithFloatingBase), 1);
      MultiBodySystemTools.extractJointsState(jointListWithFloatingBase, JointStateType.VELOCITY, jointVelocityWithFloatingBase);
      System.out.println(jointPositionWithFloatingBase);

      JointBasics[] jointList = MultiBodySystemTools.collectSubtreeJoints(fullRobotModel.getPelvis());
      DMatrixRMaj jointPosition = new DMatrixRMaj(MultiBodySystemTools.computeDegreesOfFreedom(jointList), 1);
      MultiBodySystemTools.extractJointsState(jointList, JointStateType.CONFIGURATION, jointPosition);
      DMatrixRMaj jointVelocity = new DMatrixRMaj(MultiBodySystemTools.computeDegreesOfFreedom(jointList), 1);
      MultiBodySystemTools.extractJointsState(jointList, JointStateType.VELOCITY, jointVelocity);

      //TODO we can get this from the HLHControllerToolbox via the contained internal objects.
      DMatrixRMaj momentumMatrix = computeMomentumMatrix(fullRobotModel);
      DMatrixRMaj comMomentum = MatrixTools.mult(momentumMatrix, jointVelocityWithFloatingBase);

      // Get connection matrix
      DMatrixRMaj Mbase = new DMatrixRMaj(6, 6);
      DMatrixRMaj Mq = new DMatrixRMaj(6, jointPosition.getNumRows());
      DMatrixRMaj Mconnection = new DMatrixRMaj(6, jointPosition.getNumRows());
      DMatrixRBlock MbaseBlock = new DMatrixRBlock(6, 6);
      DMatrixRBlock MqBlock = new DMatrixRBlock(6, jointPosition.getNumRows());
      DMatrixRBlock MconnectionBlock = new DMatrixRBlock(6, jointPosition.getNumRows());

      int[] srcColumnsBase = {0, 1, 2, 3, 4, 5};
      int[] srcColumnsQ = {6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34};
      MatrixTools.extractColumns(momentumMatrix, srcColumnsBase, Mbase, 0);
      MatrixTools.extractColumns(momentumMatrix, srcColumnsQ, Mq, 0);

      // Ref:
      // https://www.tabnine.com/code/java/methods/org.ejml.ops.ConvertDMatrixStruct/convert
      ConvertDMatrixStruct.convert(Mbase, MbaseBlock);
      ConvertDMatrixStruct.convert(Mq, MqBlock);

      // Ref:
      // https://ejml.org/javadoc/org/ejml/dense/block/linsol/qr/QrHouseHolderSolver_DDRB.html
      QrHouseHolderSolver_DDRB solver = new QrHouseHolderSolver_DDRB();
      solver.setA(MbaseBlock);
      solver.solve(MqBlock, MconnectionBlock);
      ConvertDMatrixStruct.convert(MconnectionBlock, Mconnection);

      //		System.out.println("----------");
      //		System.out.println(Mq);
      //		System.out.println(MatrixTools.mult(Mbase, Mconnection));

      //		System.out.println("----------");
      //		System.out.println(Mbase);

      //		System.out.println("jointListWithFloatingBase");
      //		for (int i = 0; i < jointListWithFloatingBase.length; i++) {
      //			System.out.print(jointListWithFloatingBase[i]);
      //			System.out.println("");
      //		}
      //		System.out.println("jointList");
      //		for (int i = 0; i < jointList.length; i++) {
      //			System.out.print(jointList[i]);
      //			System.out.println("");
      //		}

      //		System.out.println(jointVelocities);
      //		System.out.println(Mbase);
      //		System.out.println(Mq);
      //    System.out.println(comMomentum);

      System.out.println("momentumMatrix = ");
      System.out.println(momentumMatrix);
      System.out.println("Mconnection = ");
      System.out.println(Mconnection);

      //		System.out.println("");
      //		System.out.println(jointPositionWithFloatingBase.getNumRows());
      //		System.out.println(jointPositionWithFloatingBase.getNumCols());
      //		System.out.println(jointVelocityWithFloatingBase.getNumRows());
      //		System.out.println(jointVelocityWithFloatingBase.getNumCols());
      //		System.out.println(jointPosition.getNumRows());
      //		System.out.println(jointPosition.getNumCols());
      //		System.out.println(jointVelocity.getNumRows());
      //		System.out.println(jointVelocity.getNumCols());
      //		System.out.println(momentumMatrix.getNumRows());
      //		System.out.println(momentumMatrix.getNumCols());
      //		System.out.println(comMomentum.getNumRows());
      //		System.out.println(comMomentum.getNumCols());

      return new MomentumData(jointPositionWithFloatingBase,
                              jointVelocityWithFloatingBase,
                              jointPosition,
                              jointVelocity,
                              momentumMatrix,
                              comMomentum,
                              Mconnection);
   }

   private DMatrixRMaj computeMomentumMatrix(FullHumanoidRobotModel fullRobotModel)
   {
      CenterOfMassReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", ReferenceFrame.getWorldFrame(), fullRobotModel.getElevator());
      centerOfMassFrame.update();
      CentroidalMomentumCalculator centroidalMomentumMatrix = new CentroidalMomentumCalculator(fullRobotModel.getElevator(), centerOfMassFrame);
      //      CentroidalMomentumCalculator centroidalMomentumMatrix = new CentroidalMomentumCalculator(fullRobotModel.getPelvis(), centerOfMassFrame);
      fullRobotModel.getElevator().updateFramesRecursively();

      // System.out.println(centroidalMomentumMatrix.getCentroidalMomentumMatrix());

      return centroidalMomentumMatrix.getCentroidalMomentumMatrix();
   }
}