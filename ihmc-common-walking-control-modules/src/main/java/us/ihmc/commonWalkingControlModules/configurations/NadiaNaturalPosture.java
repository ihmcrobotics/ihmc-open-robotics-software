package us.ihmc.commonWalkingControlModules.configurations;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.Arrays;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import com.esotericsoftware.kryo.util.IntMap;

import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.JointMatrixIndexProvider;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.linearAlgebra.NullspaceCalculator;
import us.ihmc.robotics.linearAlgebra.SVDNullspaceCalculator;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.registry.YoRegistry;

//Typical usage steps (work-in-progress):
//
//(0) Search below for all "URDF"/"urdf" occurrences to see which ROBOT's NP function is in use here. 
//Initialization:
//(1) Instantiate a 'new NadiaNaturalPosture(...)' for that ROBOT.
//(2) Call 'computeNaturalPosture(...)' with a nominal whole-ROBOT world pose (i.e. base included).
//  The pose should basically be within the joint limits as specified in the URDF.
//(3) Call (typically) 'getNaturalPostureQuaternionrtBase()' and see what quaternion is returned for 
//  NP relative to the Base (pelvis).
//(4) If desired, the NP quaternion can be arbitrarily offset (e.g. set to identity) using,
//  for instance, the initial NP quaternion returned in step (3).
//  Ex: call 'setNaturalPostureOffset(...)' with the .conjugate() of the NP quaternion from step (3).
//Every control spoke:    
//(5) Call 'computeNaturalPosture(...)' with a whole-ROBOT world pose (i.e. base included).
//  The pose should basically be within the joint limits as specified in the URDF.
//(6) Use the main getters:
//  'getNaturalPostureQuaternion()'
//  'getNaturalPostureJacobian()'
//(6a) If you are doing something special, you can get ROBOT NP relative to the pelvis also:
//  'getNaturalPostureQuaternionrtBase()'
//(7) Do control using NP as a whole-body representation of angular CoM.

public class NadiaNaturalPosture implements HumanoidRobotNaturalPosture
{
   private int NumDoFs; // excluding floating base joint

   // Joint ordering as of 20220810 (just for reference; the order of joint list from MultiBodySystemTools.collectSubtreeJoints(fullRobotModel.getPelvis()) ):
   // LEFT_HIP_Z
   // RIGHT_HIP_Z
   // SPINE_Z
   // LEFT_HIP_X
   // RIGHT_HIP_X
   // SPINE_X
   // LEFT_HIP_Y
   // RIGHT_HIP_Y
   // SPINE_Y
   // LEFT_KNEE_Y
   // RIGHT_KNEE_Y
   // LEFT_SHOULDER_Y
   // RIGHT_SHOULDER_Y
   // LEFT_ANKLE_Y
   // RIGHT_ANKLE_Y
   // LEFT_SHOULDER_X
   // RIGHT_SHOULDER_X
   // LEFT_ANKLE_X
   // RIGHT_ANKLE_X
   // LEFT_SHOULDER_Z
   // RIGHT_SHOULDER_Z
   // LEFT_ELBOW_Y
   // RIGHT_ELBOW_Y
   // LEFT_WRIST_Z
   // RIGHT_WRIST_Z
   // LEFT_WRIST_X
   // RIGHT_WRIST_X
   // LEFT_WRIST_Y
   // RIGHT_WRIST_Y

   // Joint indices 
   private int i0;
   private int i1;
   private int i2;
   private int i3;
   private int i4;
   private int i5;
   private int i6;
   private int i7;
   private int i8;
   private int i9;
   private int i10;
   private int i11;
   private int i12;
   private int i13;
   private int i14;
   private int i15;
   private int i16;
   private int i17;
   private int i18;
   private int i19;
   private int i20;
   private int i21;
   private int i22;
   private int i23;
   private int i24;
   private int i25;
   private int i26;
   private int i27;
   private int i28;

   private final Quaternion npQoffset = new Quaternion(0, 0, 0, 1);

   // Various Natural Posture results available via getters:
   private final Quaternion Q_Base_NP = new Quaternion(0, 0, 0, 1); // Natural Posture rt the pelvis
   private final Quaternion Q_World_NP = new Quaternion(0, 0, 0, 1); // Natural Posture rt the world
   private DMatrixRMaj jacobianNP = null;

   // For internal use:
   private DMatrixRMaj jacobianQuaternionNPrtBase = null;
   private DMatrixRMaj jacobianOmegaNPrtBase = null; // GMN: Would need 2.0* if used externally.

   private final YoFrameQuaternion yoQuaternionNPrtWorld; //, yoQuaternionNPrtBase;
   private final YoFrameQuaternion yoQuaternionIdent;
   private final YoFramePoint3D originNPpelvis;
   private final YoFramePoint3D originPelvis;
   //   private final YoFramePoint3D originWorld;
   private final YoGraphicCoordinateSystem naturalPostureVizPelvis;
   private final YoGraphicCoordinateSystem vizPelvis;
   //   private final YoGraphicCoordinateSystem naturalPostureVizWorld;
   private boolean doGraphics = false;

   private final JointMatrixIndexProvider jointMatrixIndexProvider;
   private final IntMap<OneDoFJointReadOnly> jointMap = new IntMap<>();

   // A nominal standing pose; stored in this class for convenience; upon calling 'initialize()' will create a nominal Qoffset, which has a getter:
   // TODO pull from the "initialConfiguration" class
   // This should include all actuated joints
   double[] qNomStanding = new double[] {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
   private final Quaternion QbaseNomStanding = new Quaternion(0, 0, 0, 1);
   private final Quaternion npNomQoffset = new Quaternion(0, 0, 0, 1);

   private FullHumanoidRobotModel fullRobotModel = null;

   private int[] jointIndexArray = null;
   private double[] jointPositionArray = null;

   // Testing
   private Integer[] legJointIndicesInVelVectorIncludingFloatingBaseJoint = null;

   public NadiaNaturalPosture(FullHumanoidRobotModel robotModel,
                              boolean useURDFJointNumbering,
                              YoRegistry registry,
                              YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.fullRobotModel = robotModel;

      yoQuaternionNPrtWorld = new YoFrameQuaternion("NadiaNaturalPostureInWorld", ReferenceFrame.getWorldFrame(), registry);
      yoQuaternionIdent = new YoFrameQuaternion("NadiaPelvisFrameViz", ReferenceFrame.getWorldFrame(), registry);
      //      yoQuaternionNPrtBase = new YoFrameQuaternion("naturalPostureInBase", robotModel.getRootBody().getBodyFixedFrame(), registry);

      if (yoGraphicsListRegistry != null)
      {
         doGraphics = true;

         originNPpelvis = new YoFramePoint3D("NadiaOriginNPpelvis", ReferenceFrame.getWorldFrame(), registry);
         naturalPostureVizPelvis = new YoGraphicCoordinateSystem("NadiaNaturalPostureP", originNPpelvis, yoQuaternionNPrtWorld, 0.5);
         yoGraphicsListRegistry.registerYoGraphic("NadiaNaturalPostureP", naturalPostureVizPelvis);

         originPelvis = new YoFramePoint3D("NadiaOriginPelvis", ReferenceFrame.getWorldFrame(), registry);
         vizPelvis = new YoGraphicCoordinateSystem("NadiaPelvisframe", originPelvis, yoQuaternionIdent, 0.4);
         yoGraphicsListRegistry.registerYoGraphic("NadiaPelvisframe", vizPelvis);
      }
      else
      {
         originNPpelvis = null;
         originPelvis = null;
         //         originWorld = null;
         naturalPostureVizPelvis = null;
         vizPelvis = null;
         //         naturalPostureVizWorld = null;
      }

      if (fullRobotModel != null)
      {
         jointMatrixIndexProvider = JointMatrixIndexProvider.toIndexProvider(HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(fullRobotModel));
      }
      else
      {
         jointMatrixIndexProvider = null;
      }

      // Initialize the size of a few matrices
      //      String[] jointNameInOrder = read1DString(folderPath + "all_joint_names_in_order.csv");
      NumDoFs = jointNameInOrder.length;
      jacobianNP = new DMatrixRMaj(3, 6 + NumDoFs);
      jacobianQuaternionNPrtBase = new DMatrixRMaj(4, NumDoFs);
      jacobianOmegaNPrtBase = new DMatrixRMaj(4, NumDoFs);
      jointPositionArray = new double[NumDoFs];

      // Set joint indices for auto-generated NP function:
      int iBase = 6; // GMN: offset for pelvis DoF

      i0 = getJointIndices(jointNameInOrder[0])[0] - iBase;
      i1 = getJointIndices(jointNameInOrder[1])[0] - iBase;
      i2 = getJointIndices(jointNameInOrder[2])[0] - iBase;
      i3 = getJointIndices(jointNameInOrder[3])[0] - iBase;
      i4 = getJointIndices(jointNameInOrder[4])[0] - iBase;
      i5 = getJointIndices(jointNameInOrder[5])[0] - iBase;
      i6 = getJointIndices(jointNameInOrder[6])[0] - iBase;
      i7 = getJointIndices(jointNameInOrder[7])[0] - iBase;
      i8 = getJointIndices(jointNameInOrder[8])[0] - iBase;
      i9 = getJointIndices(jointNameInOrder[9])[0] - iBase;
      i10 = getJointIndices(jointNameInOrder[10])[0] - iBase;
      i11 = getJointIndices(jointNameInOrder[11])[0] - iBase;
      i12 = getJointIndices(jointNameInOrder[12])[0] - iBase;
      i13 = getJointIndices(jointNameInOrder[13])[0] - iBase;
      i14 = getJointIndices(jointNameInOrder[14])[0] - iBase;
      i15 = getJointIndices(jointNameInOrder[15])[0] - iBase;
      i16 = getJointIndices(jointNameInOrder[16])[0] - iBase;
      i17 = getJointIndices(jointNameInOrder[17])[0] - iBase;
      i18 = getJointIndices(jointNameInOrder[18])[0] - iBase;
      i19 = getJointIndices(jointNameInOrder[19])[0] - iBase;
      i20 = getJointIndices(jointNameInOrder[20])[0] - iBase;
      i21 = getJointIndices(jointNameInOrder[21])[0] - iBase;
      i22 = getJointIndices(jointNameInOrder[22])[0] - iBase;
      i23 = getJointIndices(jointNameInOrder[23])[0] - iBase;
      i24 = getJointIndices(jointNameInOrder[24])[0] - iBase;
      i25 = getJointIndices(jointNameInOrder[25])[0] - iBase;
      i26 = getJointIndices(jointNameInOrder[26])[0] - iBase;
      i27 = getJointIndices(jointNameInOrder[27])[0] - iBase;
      i28 = getJointIndices(jointNameInOrder[28])[0] - iBase;

      jointMap.put(i0, fullRobotModel.getOneDoFJointByName(jointNameInOrder[0]));
      jointMap.put(i1, fullRobotModel.getOneDoFJointByName(jointNameInOrder[1]));
      jointMap.put(i2, fullRobotModel.getOneDoFJointByName(jointNameInOrder[2]));
      jointMap.put(i3, fullRobotModel.getOneDoFJointByName(jointNameInOrder[3]));
      jointMap.put(i4, fullRobotModel.getOneDoFJointByName(jointNameInOrder[4]));
      jointMap.put(i5, fullRobotModel.getOneDoFJointByName(jointNameInOrder[5]));
      jointMap.put(i6, fullRobotModel.getOneDoFJointByName(jointNameInOrder[6]));
      jointMap.put(i7, fullRobotModel.getOneDoFJointByName(jointNameInOrder[7]));
      jointMap.put(i8, fullRobotModel.getOneDoFJointByName(jointNameInOrder[8]));
      jointMap.put(i9, fullRobotModel.getOneDoFJointByName(jointNameInOrder[9]));
      jointMap.put(i10, fullRobotModel.getOneDoFJointByName(jointNameInOrder[10]));
      jointMap.put(i11, fullRobotModel.getOneDoFJointByName(jointNameInOrder[11]));
      jointMap.put(i12, fullRobotModel.getOneDoFJointByName(jointNameInOrder[12]));
      jointMap.put(i13, fullRobotModel.getOneDoFJointByName(jointNameInOrder[13]));
      jointMap.put(i14, fullRobotModel.getOneDoFJointByName(jointNameInOrder[14]));
      jointMap.put(i15, fullRobotModel.getOneDoFJointByName(jointNameInOrder[15]));
      jointMap.put(i16, fullRobotModel.getOneDoFJointByName(jointNameInOrder[16]));
      jointMap.put(i17, fullRobotModel.getOneDoFJointByName(jointNameInOrder[17]));
      jointMap.put(i18, fullRobotModel.getOneDoFJointByName(jointNameInOrder[18]));
      jointMap.put(i19, fullRobotModel.getOneDoFJointByName(jointNameInOrder[19]));
      jointMap.put(i20, fullRobotModel.getOneDoFJointByName(jointNameInOrder[20]));
      jointMap.put(i21, fullRobotModel.getOneDoFJointByName(jointNameInOrder[21]));
      jointMap.put(i22, fullRobotModel.getOneDoFJointByName(jointNameInOrder[22]));
      jointMap.put(i23, fullRobotModel.getOneDoFJointByName(jointNameInOrder[23]));
      jointMap.put(i24, fullRobotModel.getOneDoFJointByName(jointNameInOrder[24]));
      jointMap.put(i25, fullRobotModel.getOneDoFJointByName(jointNameInOrder[25]));
      jointMap.put(i26, fullRobotModel.getOneDoFJointByName(jointNameInOrder[26]));
      jointMap.put(i27, fullRobotModel.getOneDoFJointByName(jointNameInOrder[27]));
      jointMap.put(i28, fullRobotModel.getOneDoFJointByName(jointNameInOrder[28]));

      //      LogTools.info("-------------------------NP------------------------------------");
      //      LogTools.info("-------------------------NP------------------------------------");
      //      LogTools.info(EuclidCoreIOTools.getCollectionString(", ", indexProvider.getIndexedJointsInOrder(), j -> j.getName()));
      //      LogTools.info("-------------------------NP------------------------------------");
      //      LogTools.info("-------------------------NP------------------------------------");
      //      LogTools.info("-------------------------NP------------------------------------");

      //      Integer[] jointsToFit = read1DIntCsvToIntArray(folderPath + "joints_to_fit.csv");
      jointIndexArray = new int[jointsToFit.length];
      for (int i = 0; i < jointsToFit.length; i++)
      {
         jointIndexArray[i] = jointsToFit[i];
      }

      // Testing
      legJointIndicesInVelVectorIncludingFloatingBaseJoint = new Integer[12];
      legJointIndicesInVelVectorIncludingFloatingBaseJoint[0] = getJointIndices("LEFT_HIP_Z")[0];
      legJointIndicesInVelVectorIncludingFloatingBaseJoint[1] = getJointIndices("RIGHT_HIP_Z")[0];
      legJointIndicesInVelVectorIncludingFloatingBaseJoint[2] = getJointIndices("LEFT_HIP_X")[0];
      legJointIndicesInVelVectorIncludingFloatingBaseJoint[3] = getJointIndices("RIGHT_HIP_X")[0];
      legJointIndicesInVelVectorIncludingFloatingBaseJoint[4] = getJointIndices("LEFT_HIP_Y")[0];
      legJointIndicesInVelVectorIncludingFloatingBaseJoint[5] = getJointIndices("RIGHT_HIP_Y")[0];
      legJointIndicesInVelVectorIncludingFloatingBaseJoint[6] = getJointIndices("LEFT_KNEE_Y")[0];
      legJointIndicesInVelVectorIncludingFloatingBaseJoint[7] = getJointIndices("RIGHT_KNEE_Y")[0];
      legJointIndicesInVelVectorIncludingFloatingBaseJoint[8] = getJointIndices("LEFT_ANKLE_Y")[0];
      legJointIndicesInVelVectorIncludingFloatingBaseJoint[9] = getJointIndices("RIGHT_ANKLE_Y")[0];
      legJointIndicesInVelVectorIncludingFloatingBaseJoint[10] = getJointIndices("LEFT_ANKLE_X")[0];
      legJointIndicesInVelVectorIncludingFloatingBaseJoint[11] = getJointIndices("RIGHT_ANKLE_X")[0];
   }

   int[] getJointIndices(String jointName)
   {
      return getJointIndices(fullRobotModel.getOneDoFJointByName(jointName));
   }

   int[] getJointIndices(JointReadOnly joint)
   {
      return jointMatrixIndexProvider.getJointDoFIndices(joint);
   }

   public double[] getJointPositionArray()
   {
      Arrays.fill(jointPositionArray, 0.0);

      for (int jointIndex : jointIndexArray)
      {
         jointPositionArray[jointIndex] = jointMap.get(jointIndex).getQ();
      }

      return jointPositionArray;
   }

   @Override
   public void initialize()
   {
      computeNaturalPosture(this.qNomStanding, this.QbaseNomStanding);
      this.npNomQoffset.set(getNaturalPostureQuaternionrtBase());
      this.npNomQoffset.conjugate();
      this.npQoffset.set(this.npNomQoffset);
   }

   @Override
   public Quaternion getNominalStandingPoseQoffset()
   {
      return this.npNomQoffset;
   }

   public FullHumanoidRobotModel getFullRobotModel()
   {
      return this.fullRobotModel;
   }

   // Set a user-chosen orientation offset to apply to NP:
   @Override
   public void setNaturalPostureOffset(QuaternionReadOnly Qoffset)
   {
      // System.out.println(Qoffset);
      this.npQoffset.set(Qoffset);
   }

   @Override
   public Quaternion getNaturalPostureQuaternion()
   {
      return this.Q_World_NP;
   }

   @Override
   public Quaternion getNaturalPostureQuaternionrtBase()
   {
      return this.Q_Base_NP;
   }

   @Override
   public DMatrixRMaj getNaturalPostureJacobian()
   {
      return this.jacobianNP;
   }

   @Override
   public void compute(double[] q, Orientation3DReadOnly Q_world_base)
   {
      computeNaturalPosture(q, Q_world_base);
   }

   public void computeNaturalPosture(double[] q, Orientation3DReadOnly Q_world_base)
   {
      // Get the NP quaternion r.t. the base(pelvis) frame:
      computeQuaternionNPrtBase(q, this.Q_Base_NP);

      // Express the NP quaternion in the world-frame:
      this.Q_World_NP.set(Q_world_base);
      this.Q_World_NP.multiply(this.Q_Base_NP);

      // Get the NP quaternion jacobian r.t. the base(pelvis) frame:
      // + We need 'q' because, like quaternionNPrtBase, this jacobian is also an explicit function of q.
      // + We need 'quaternionNPrtBase` to deduce the 1st row of this jacobian from the unit-sphere constraint. 
      computeJacobianQuaternionNPrtBase(q, this.Q_Base_NP, this.jacobianQuaternionNPrtBase);
      // Convert the NP jacobian to map to NP omega r.t. world ewrt NP-frame:
      // + We need 'quaternionNPrtBase' to get two transforms: E (bring Qdot -> omega) & C_NP_Base (bring omega_ewrt_Base -> omega_ewrt_NP)
      // + We need 'jacobiandQuaternionNPrtBase' as once transformed, it forms all the joint-based portion of the jacobianNP.
      computeJacobianNP(this.Q_Base_NP, this.jacobianQuaternionNPrtBase, this.jacobianNP);

      //      SVDNullspaceCalculator nullspacecalculator = new SVDNullspaceCalculator(this.jacobianNP.numCols, true /*makeLargestComponentPositive*/);
      //      DMatrixRMaj nullspaceProjectorToPack = new DMatrixRMaj(this.jacobianNP.numCols, this.jacobianNP.numCols);
      //      nullspacecalculator.computeNullspaceProjector(this.jacobianNP, nullspaceProjectorToPack);
      //      System.out.println(nullspaceProjectorToPack);

      if (doGraphics == true)
      {
         FramePoint3D originPose = new FramePoint3D(this.fullRobotModel.getRootBody().getBodyFixedFrame());
         originPose.changeFrame(ReferenceFrame.getWorldFrame());
         yoQuaternionNPrtWorld.set(Q_World_NP);
         originNPpelvis.set(originPose);
         yoQuaternionIdent.set(Q_world_base);
         originPelvis.set(originPose);
      }
   }

   // Given full set of joint angles, returns NP quaternion rt the Base(i.e. pelvis)-frame (with user defined NP offset):
   private void computeQuaternionNPrtBase(double[] q, Quaternion quaternionToPack)
   {
      double[] Q = new double[4];
      Q[1] = getQx(q);
      Q[2] = getQy(q);
      Q[3] = getQz(q);
      Q[0] = Math.sqrt(1 - (Q[1] * Q[1] + Q[2] * Q[2] + Q[3] * Q[3])); // Should return NaN if argument is negative

      quaternionToPack.set(Q[1], Q[2], Q[3], Q[0]); // IHMC "stores" quaternions in the order (Qx,Qy,Qz,Qs)
      quaternionToPack.multiply(this.npQoffset);
   }

   // NOTE ***: For internal use only.
   // Given full set of joint angles and NP quaternion rt Base, returns this [Jacobian] -> Qdot_NP_rt_Base = [Jacobian] * q_dot: 
   private void computeJacobianQuaternionNPrtBase(double[] q, QuaternionReadOnly quaternionNPrtBase, DMatrixRMaj jacobianToPack)
   {
      double[][] JQ = new double[4][NumDoFs]; // GMN: These do initialize to all zero, right??  That's what I read online...
      getJQx(q, JQ); // replaces row index 1 of JQ
      getJQy(q, JQ); // replaces row index 2 of JQ
      getJQz(q, JQ); // replaces row index 3 of JQ

      // GMN: I'll have to do further investigation to see if we have methods that can work on submatrices...
      //      For now, I will do this manually:
      for (int i = 0; i < NumDoFs; i++)
      {
         JQ[0][i] = quaternionNPrtBase.getX() * JQ[1][i] + quaternionNPrtBase.getY() * JQ[2][i] + quaternionNPrtBase.getZ() * JQ[3][i];
         JQ[0][i] /= (-quaternionNPrtBase.getS());
      }

      jacobianToPack.set(JQ);
   }

   // NOTE ***: For internal use only.
   // Given NP quaternion rt Base & Jacobian of the same, returns this [Jacobian] -> 
   //         omega_NP_rt_world_ewrt_NP = [Jacobian] * [omega_Base_rt_world_ewrt_Base; q_dot]
   private void computeJacobianNP(QuaternionReadOnly quaternionNPrtBase, DMatrixRMaj jacobianQuaternionNPrtBase, DMatrixRMaj jacobianToPack)
   {
      // Need 3x3 tranform from Base-frame back to NP-frame:
      RotationMatrix CnpBase = new RotationMatrix(quaternionNPrtBase);
      CnpBase.transpose();

      // Insert transformation to bring omega_Base_rt_world_ewrt_Base back to NP-frame:
      for (int i = 0; i < 3; i++)
         for (int j = 0; j < 3; j++)
         {
            jacobianToPack.set(i, j, CnpBase.getElement(i, j)); // GMN: Need to see if we have submatrix operators...
            jacobianToPack.set(i, j + 3, 0.0);
         }

      // Need transformation of Qdot -> omega  (note the required '2.0*' comes in below):
      DMatrixRMaj E = EuclidCoreMissingTools.quaternionDotToOmegaTransform(quaternionNPrtBase);

      // Transform the quaternion jacobian to omega jacobian, and insert it: 
      CommonOps_DDRM.mult(E, jacobianQuaternionNPrtBase, this.jacobianOmegaNPrtBase);
      for (int i = 0; i < 3; i++)
         for (int j = 0; j < NumDoFs; j++)
            jacobianToPack.set(i, j + 6, 2.0 * this.jacobianOmegaNPrtBase.get(i + 1, j)); // for omega NP rt & ewrt NP-frame

      //      // [Testing] not use the x y of NP.
      //      for (int i = 0; i < 2; i++)
      //         for (int j = 0; j < 6 + NumDoFs; j++)
      //         {
      //            jacobianToPack.set(i, j, 0.0);
      //         }

      // [Testing] not use the leg joints
      //    for (int i = 0; i < 3; i++)
      //       for (int j = 0; j < legJointIndicesInVelVectorIncludingFloatingBaseJoint.length; j++)
      //       {
      //          jacobianToPack.set(i, legJointIndicesInVelVectorIncludingFloatingBaseJoint[j], 0.0);
      //       }

   }

   //==== CSV utils ===========================================================================================================================

   private static String[] read1DString(String filePath)
   {
      try
      {
         BufferedReader csvReader = new BufferedReader(new FileReader(filePath));
         String row;
         try
         {
            while ((row = csvReader.readLine()) != null)
            {
               String[] data = row.split(",");
               return data;
               // do something with the data
            }
            csvReader.close();
         }
         catch (IOException e)
         {
            // TODO Auto-generated catch block
            e.printStackTrace();
         }
      }
      catch (FileNotFoundException e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
      return new String[0];
   }

   private static int getNumRowsOfCsvFile(String filePath)
   {
      // Ref: https://stackoverflow.com/questions/18009416/how-to-count-total-rows-in-csv-using-java
      int count = 0;
      try
      {
         BufferedReader bufferedReader;
         bufferedReader = new BufferedReader(new FileReader(filePath));
         String input;
         try
         {
            while ((input = bufferedReader.readLine()) != null)
            {
               count++;
            }
         }
         catch (IOException e)
         {
            // TODO Auto-generated catch block
            e.printStackTrace();
         }

         System.out.println("Number of lines: " + count);

      }
      catch (FileNotFoundException e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
      return count;
   }

   private static int getNumColsOfCsvFile(String filePath)
   {
      String[] split = null;
      try (BufferedReader br = new BufferedReader(new FileReader(filePath)))
      {
         String line;
         while ((line = br.readLine()) != null)
         {
            split = line.split(",");
            //use the data here
         }
      }
      catch (FileNotFoundException e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
      catch (IOException e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }

      System.out.println("Number of columns: " + split.length);

      //      for (int i = 0; i < split.length ; i++) {
      //         System.out.println(split[i]);
      //      }

      return split.length;
   }

   private static Integer[] read1DIntCsvToIntArray(String filePath)
   {
      int numRows = getNumRowsOfCsvFile(filePath);
      int numCols = getNumColsOfCsvFile(filePath);

      //      String delimiter = (numRows == 1) ? "," : "\n";
      return read1DIntCsvToIntArray(filePath, numRows * numCols, numRows == 1);
   }

   private static Integer[] read1DIntCsvToIntArray(String filePath, int numElement, Boolean isRowVector)
   {

      Integer[] intArray = new Integer[numElement];
      int i = 0;

      String[] split = null;
      try (BufferedReader br = new BufferedReader(new FileReader(filePath)))
      {
         String line;
         while ((line = br.readLine()) != null)
         {
            if (isRowVector)
            {
               split = line.split(",");

               if (numElement != split.length)
               {
                  System.out.println("there is a bug somewhere! The vector length is not consisent");
               }

               for (int j = 0; j < numElement; j++)
               {
                  intArray[j] = Integer.parseInt(split[j]);
               }
            }
            else
            {
               if (numElement == i)
               {
                  System.out.println("there is a bug somewhere! The vector length is not consisent");
               }
               intArray[i] = Integer.parseInt(line);
               i++;
            }
         }
      }
      catch (FileNotFoundException e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
      catch (IOException e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
      return intArray;
   }

   //==========================================================================================================================================
   private String[] jointNameInOrder = new String[] {"LEFT_HIP_Z", "RIGHT_HIP_Z", "SPINE_Z", "LEFT_HIP_X", "RIGHT_HIP_X", "SPINE_X", "LEFT_HIP_Y",
         "RIGHT_HIP_Y", "SPINE_Y", "LEFT_KNEE_Y", "RIGHT_KNEE_Y", "LEFT_SHOULDER_Y", "RIGHT_SHOULDER_Y", "LEFT_ANKLE_Y", "RIGHT_ANKLE_Y", "LEFT_SHOULDER_X",
         "RIGHT_SHOULDER_X", "LEFT_ANKLE_X", "RIGHT_ANKLE_X", "LEFT_SHOULDER_Z", "RIGHT_SHOULDER_Z", "LEFT_ELBOW_Y", "RIGHT_ELBOW_Y", "LEFT_WRIST_Z",
         "RIGHT_WRIST_Z", "LEFT_WRIST_X", "RIGHT_WRIST_X", "LEFT_WRIST_Y", "RIGHT_WRIST_Y"};

   private Integer[] jointsToFit = new Integer[] {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 16, 17, 20, 21, 22, 23};

   public double getQx(double[] q)
   {
      double Qx;
      Qx = (-1.091470e-02) * q[i0] + (-1.075516e-02) * q[i1] + (-6.493713e-04) * q[i2] + (1.312319e-01) * q[i3] + (1.305468e-01) * q[i4]
            + (1.272279e-01) * q[i5] + (3.137786e-03) * q[i6] + (-3.106478e-03) * q[i7] + (-1.146618e-04) * q[i8] + (4.889873e-03) * q[i9]
            + (-4.803546e-03) * q[i10] + (2.683704e-03) * q[i11] + (3.009467e-03) * q[i12] + (-1.968976e-03) * q[i15] + (1.890989e-03) * q[i16]
            + (9.177392e-04) * q[i19] + (-9.533365e-04) * q[i20] + (-2.718647e-03) * q[i21] + (-2.768467e-03) * q[i22] + (-4.763695e-03) * q[i0] * q[i0]
            + (4.731760e-03) * q[i1] * q[i1] + (-1.342401e-05) * q[i2] * q[i2] + (1.348478e-02) * q[i3] * q[i3] + (-1.337858e-02) * q[i4] * q[i4]
            + (-1.522872e-05) * q[i5] * q[i5] + (1.824246e-02) * q[i6] * q[i6] + (-1.817250e-02) * q[i7] * q[i7] + (-7.790112e-06) * q[i8] * q[i8]
            + (4.028609e-03) * q[i9] * q[i9] + (-3.969694e-03) * q[i10] * q[i10] + (5.184729e-03) * q[i11] * q[i11] + (-5.326149e-03) * q[i12] * q[i12]
            + (6.490308e-03) * q[i15] * q[i15] + (-6.544746e-03) * q[i16] * q[i16] + (7.924415e-04) * q[i19] * q[i19] + (-7.808437e-04) * q[i20] * q[i20]
            + (1.668478e-03) * q[i21] * q[i21] + (-1.674249e-03) * q[i22] * q[i22] + (-3.072205e-05) * q[i0] * q[i1] + (-2.215087e-04) * q[i0] * q[i2]
            + (-1.614664e-03) * q[i0] * q[i3] + (-4.964491e-03) * q[i0] * q[i4] + (2.972583e-03) * q[i0] * q[i5] + (9.063135e-02) * q[i0] * q[i6]
            + (-1.051662e-02) * q[i0] * q[i7] + (-3.638438e-03) * q[i0] * q[i8] + (2.931941e-02) * q[i0] * q[i9] + (-1.566182e-03) * q[i0] * q[i10]
            + (2.431821e-03) * q[i0] * q[i11] + (-2.645071e-04) * q[i0] * q[i12] + (-3.495047e-04) * q[i0] * q[i15] + (2.440634e-03) * q[i0] * q[i16]
            + (-2.003494e-03) * q[i0] * q[i19] + (6.647019e-04) * q[i0] * q[i20] + (1.479821e-03) * q[i0] * q[i21] + (3.335293e-04) * q[i0] * q[i22]
            + (1.546791e-04) * q[i1] * q[i2] + (4.911325e-03) * q[i1] * q[i3] + (1.639490e-03) * q[i1] * q[i4] + (-2.945678e-03) * q[i1] * q[i5]
            + (-1.054391e-02) * q[i1] * q[i6] + (9.010674e-02) * q[i1] * q[i7] + (-3.676940e-03) * q[i1] * q[i8] + (-1.585410e-03) * q[i1] * q[i9]
            + (2.896086e-02) * q[i1] * q[i10] + (2.549366e-04) * q[i1] * q[i11] + (-2.437312e-03) * q[i1] * q[i12] + (2.488392e-03) * q[i1] * q[i15]
            + (-3.292644e-04) * q[i1] * q[i16] + (7.010237e-04) * q[i1] * q[i19] + (-1.956615e-03) * q[i1] * q[i20] + (-3.335710e-04) * q[i1] * q[i21]
            + (-1.478069e-03) * q[i1] * q[i22] + (9.295948e-04) * q[i2] * q[i3] + (-1.007355e-03) * q[i2] * q[i4] + (9.527315e-05) * q[i2] * q[i5]
            + (1.591417e-02) * q[i2] * q[i6] + (1.577288e-02) * q[i2] * q[i7] + (-6.770789e-02) * q[i2] * q[i8] + (3.152888e-03) * q[i2] * q[i9]
            + (3.116881e-03) * q[i2] * q[i10] + (5.278294e-03) * q[i2] * q[i11] + (-5.454492e-03) * q[i2] * q[i12] + (2.469027e-03) * q[i2] * q[i15]
            + (2.522821e-03) * q[i2] * q[i16] + (-2.095669e-03) * q[i2] * q[i19] + (-2.100913e-03) * q[i2] * q[i20] + (1.403467e-03) * q[i2] * q[i21]
            + (-1.403050e-03) * q[i2] * q[i22] + (-2.876676e-06) * q[i3] * q[i4] + (-9.794577e-03) * q[i3] * q[i5] + (1.055381e-02) * q[i3] * q[i6]
            + (-4.554090e-03) * q[i3] * q[i7] + (2.986873e-03) * q[i3] * q[i8] + (-7.460434e-03) * q[i3] * q[i9] + (4.861489e-03) * q[i3] * q[i10]
            + (-8.971683e-04) * q[i3] * q[i11] + (2.443215e-03) * q[i3] * q[i12] + (5.579206e-03) * q[i3] * q[i15] + (5.510126e-03) * q[i3] * q[i16]
            + (-2.364092e-03) * q[i3] * q[i19] + (3.470420e-03) * q[i3] * q[i20] + (-9.036793e-04) * q[i3] * q[i21] + (5.059900e-04) * q[i3] * q[i22]
            + (9.743681e-03) * q[i4] * q[i5] + (-4.531510e-03) * q[i4] * q[i6] + (1.051373e-02) * q[i4] * q[i7] + (3.061437e-03) * q[i4] * q[i8]
            + (4.908594e-03) * q[i4] * q[i9] + (-7.410230e-03) * q[i4] * q[i10] + (-2.383645e-03) * q[i4] * q[i11] + (9.079738e-04) * q[i4] * q[i12]
            + (5.425509e-03) * q[i4] * q[i15] + (5.592667e-03) * q[i4] * q[i16] + (3.453573e-03) * q[i4] * q[i19] + (-2.334264e-03) * q[i4] * q[i20]
            + (-5.053012e-04) * q[i4] * q[i21] + (8.770941e-04) * q[i4] * q[i22] + (-1.444991e-03) * q[i5] * q[i6] + (-1.434103e-03) * q[i5] * q[i7]
            + (4.079506e-04) * q[i5] * q[i8] + (2.276604e-03) * q[i5] * q[i9] + (2.289996e-03) * q[i5] * q[i10] + (5.436482e-03) * q[i5] * q[i11]
            + (-5.465597e-03) * q[i5] * q[i12] + (-1.788624e-02) * q[i5] * q[i15] + (-1.806450e-02) * q[i5] * q[i16] + (-5.391021e-04) * q[i5] * q[i19]
            + (-5.439486e-04) * q[i5] * q[i20] + (-8.173700e-04) * q[i5] * q[i21] + (8.332752e-04) * q[i5] * q[i22] + (-5.334098e-05) * q[i6] * q[i7]
            + (2.546911e-03) * q[i6] * q[i8] + (8.509535e-03) * q[i6] * q[i9] + (-4.073500e-04) * q[i6] * q[i10] + (3.803170e-03) * q[i6] * q[i11]
            + (4.725349e-03) * q[i6] * q[i12] + (-9.283013e-04) * q[i6] * q[i15] + (-7.215826e-04) * q[i6] * q[i16] + (2.066508e-03) * q[i6] * q[i19]
            + (1.200329e-03) * q[i6] * q[i20] + (-7.083634e-04) * q[i6] * q[i21] + (-1.230140e-03) * q[i6] * q[i22] + (-2.560472e-03) * q[i7] * q[i8]
            + (3.818315e-04) * q[i7] * q[i9] + (-8.383535e-03) * q[i7] * q[i10] + (4.708322e-03) * q[i7] * q[i11] + (3.765473e-03) * q[i7] * q[i12]
            + (7.197612e-04) * q[i7] * q[i15] + (9.171318e-04) * q[i7] * q[i16] + (-1.185100e-03) * q[i7] * q[i19] + (-2.023647e-03) * q[i7] * q[i20]
            + (-1.223041e-03) * q[i7] * q[i21] + (-6.863549e-04) * q[i7] * q[i22] + (1.827642e-03) * q[i8] * q[i9] + (-1.850971e-03) * q[i8] * q[i10]
            + (1.037983e-02) * q[i8] * q[i11] + (1.046921e-02) * q[i8] * q[i12] + (9.172387e-03) * q[i8] * q[i15] + (-9.271249e-03) * q[i8] * q[i16]
            + (-4.194115e-04) * q[i8] * q[i19] + (4.509890e-04) * q[i8] * q[i20] + (2.241619e-03) * q[i8] * q[i21] + (2.269841e-03) * q[i8] * q[i22]
            + (3.718957e-06) * q[i9] * q[i10] + (1.113186e-03) * q[i9] * q[i11] + (2.158386e-03) * q[i9] * q[i12] + (-1.509882e-03) * q[i9] * q[i15]
            + (3.377185e-04) * q[i9] * q[i16] + (1.013386e-04) * q[i9] * q[i19] + (1.035450e-03) * q[i9] * q[i20] + (2.236456e-04) * q[i9] * q[i21]
            + (-5.577125e-05) * q[i9] * q[i22] + (2.136864e-03) * q[i10] * q[i11] + (1.103518e-03) * q[i10] * q[i12] + (-3.305814e-04) * q[i10] * q[i15]
            + (1.497577e-03) * q[i10] * q[i16] + (-1.040316e-03) * q[i10] * q[i19] + (-8.931737e-05) * q[i10] * q[i20] + (-5.361554e-05) * q[i10] * q[i21]
            + (2.301890e-04) * q[i10] * q[i22] + (1.641208e-05) * q[i11] * q[i12] + (1.869733e-03) * q[i11] * q[i15] + (-5.634301e-05) * q[i11] * q[i16]
            + (-1.497320e-03) * q[i11] * q[i19] + (1.693188e-03) * q[i11] * q[i20] + (4.262676e-04) * q[i11] * q[i21] + (-2.681229e-04) * q[i11] * q[i22]
            + (-3.818853e-05) * q[i12] * q[i15] + (1.944719e-03) * q[i12] * q[i16] + (1.699568e-03) * q[i12] * q[i19] + (-1.523914e-03) * q[i12] * q[i20]
            + (2.662284e-04) * q[i12] * q[i21] + (-4.255399e-04) * q[i12] * q[i22] + (1.381008e-06) * q[i15] * q[i16] + (1.443820e-03) * q[i15] * q[i19]
            + (-1.050440e-04) * q[i15] * q[i20] + (2.450143e-03) * q[i15] * q[i21] + (2.765729e-04) * q[i15] * q[i22] + (9.614661e-05) * q[i16] * q[i19]
            + (-1.418129e-03) * q[i16] * q[i20] + (2.959218e-04) * q[i16] * q[i21] + (2.443170e-03) * q[i16] * q[i22] + (6.413053e-06) * q[i19] * q[i20]
            + (-9.128984e-04) * q[i19] * q[i21] + (-9.662913e-05) * q[i19] * q[i22] + (-1.065760e-04) * q[i20] * q[i21] + (-8.800162e-04) * q[i20] * q[i22]
            + (-9.178308e-06) * q[i21] * q[i22] + (1.626362e-03) * q[i0] * q[i0] * q[i0] + (-3.464843e-03) * q[i0] * q[i0] * q[i1]
            + (2.556321e-03) * q[i0] * q[i0] * q[i2] + (-3.012961e-02) * q[i0] * q[i0] * q[i3] + (-6.835978e-05) * q[i0] * q[i0] * q[i4]
            + (-3.507112e-03) * q[i0] * q[i0] * q[i5] + (1.253647e-03) * q[i0] * q[i0] * q[i6] + (2.032812e-03) * q[i0] * q[i0] * q[i7]
            + (-1.493391e-03) * q[i0] * q[i0] * q[i8] + (2.952615e-03) * q[i0] * q[i0] * q[i9] + (-2.826019e-04) * q[i0] * q[i0] * q[i10]
            + (7.179850e-04) * q[i0] * q[i0] * q[i11] + (4.998808e-06) * q[i0] * q[i0] * q[i12] + (-1.196572e-03) * q[i0] * q[i0] * q[i15]
            + (-2.972802e-04) * q[i0] * q[i0] * q[i16] + (-4.202603e-05) * q[i0] * q[i0] * q[i19] + (-3.548803e-04) * q[i0] * q[i0] * q[i20]
            + (6.331864e-04) * q[i0] * q[i0] * q[i21] + (-2.889568e-04) * q[i0] * q[i0] * q[i22] + (-3.444680e-03) * q[i0] * q[i1] * q[i1]
            + (1.609090e-03) * q[i1] * q[i1] * q[i1] + (2.557233e-03) * q[i1] * q[i1] * q[i2] + (-6.150220e-05) * q[i1] * q[i1] * q[i3]
            + (-3.004084e-02) * q[i1] * q[i1] * q[i4] + (-3.462911e-03) * q[i1] * q[i1] * q[i5] + (-2.010742e-03) * q[i1] * q[i1] * q[i6]
            + (-1.266027e-03) * q[i1] * q[i1] * q[i7] + (1.472293e-03) * q[i1] * q[i1] * q[i8] + (3.075456e-04) * q[i1] * q[i1] * q[i9]
            + (-2.936930e-03) * q[i1] * q[i1] * q[i10] + (2.325739e-05) * q[i1] * q[i1] * q[i11] + (7.082192e-04) * q[i1] * q[i1] * q[i12]
            + (2.994997e-04) * q[i1] * q[i1] * q[i15] + (1.192890e-03) * q[i1] * q[i1] * q[i16] + (3.554884e-04) * q[i1] * q[i1] * q[i19]
            + (5.501578e-05) * q[i1] * q[i1] * q[i20] + (-2.780366e-04) * q[i1] * q[i1] * q[i21] + (6.328589e-04) * q[i1] * q[i1] * q[i22]
            + (1.645876e-04) * q[i0] * q[i2] * q[i2] + (1.444696e-04) * q[i1] * q[i2] * q[i2] + (2.998311e-03) * q[i2] * q[i2] * q[i2]
            + (-1.410196e-03) * q[i2] * q[i2] * q[i3] + (-1.390624e-03) * q[i2] * q[i2] * q[i4] + (-2.711721e-02) * q[i2] * q[i2] * q[i5]
            + (9.419021e-04) * q[i2] * q[i2] * q[i6] + (-9.400876e-04) * q[i2] * q[i2] * q[i7] + (-1.694320e-05) * q[i2] * q[i2] * q[i8]
            + (1.309976e-04) * q[i2] * q[i2] * q[i9] + (-1.301474e-04) * q[i2] * q[i2] * q[i10] + (6.982843e-04) * q[i2] * q[i2] * q[i11]
            + (6.892704e-04) * q[i2] * q[i2] * q[i12] + (-2.546112e-03) * q[i2] * q[i2] * q[i15] + (2.587146e-03) * q[i2] * q[i2] * q[i16]
            + (1.521570e-04) * q[i2] * q[i2] * q[i19] + (-1.591219e-04) * q[i2] * q[i2] * q[i20] + (1.112668e-03) * q[i2] * q[i2] * q[i21]
            + (1.105582e-03) * q[i2] * q[i2] * q[i22] + (-2.153409e-03) * q[i0] * q[i3] * q[i3] + (1.442191e-03) * q[i1] * q[i3] * q[i3]
            + (-2.028160e-03) * q[i2] * q[i3] * q[i3] + (-3.734343e-03) * q[i3] * q[i3] * q[i3] + (-1.103080e-03) * q[i3] * q[i3] * q[i4]
            + (7.210215e-03) * q[i3] * q[i3] * q[i5] + (1.880440e-03) * q[i3] * q[i3] * q[i6] + (2.994120e-04) * q[i3] * q[i3] * q[i7]
            + (7.210461e-04) * q[i3] * q[i3] * q[i8] + (-1.857075e-04) * q[i3] * q[i3] * q[i9] + (-2.367540e-04) * q[i3] * q[i3] * q[i10]
            + (-1.402094e-03) * q[i3] * q[i3] * q[i11] + (-2.855950e-03) * q[i3] * q[i3] * q[i12] + (7.552389e-04) * q[i3] * q[i3] * q[i15]
            + (3.199199e-04) * q[i3] * q[i3] * q[i16] + (-1.116370e-03) * q[i3] * q[i3] * q[i19] + (-2.797243e-04) * q[i3] * q[i3] * q[i20]
            + (1.536098e-04) * q[i3] * q[i3] * q[i21] + (3.081561e-04) * q[i3] * q[i3] * q[i22] + (1.433173e-03) * q[i0] * q[i4] * q[i4]
            + (-2.141928e-03) * q[i1] * q[i4] * q[i4] + (-2.034666e-03) * q[i2] * q[i4] * q[i4] + (-1.147295e-03) * q[i3] * q[i4] * q[i4]
            + (-3.714140e-03) * q[i4] * q[i4] * q[i4] + (7.177278e-03) * q[i4] * q[i4] * q[i5] + (-2.780751e-04) * q[i4] * q[i4] * q[i6]
            + (-1.868758e-03) * q[i4] * q[i4] * q[i7] + (-6.687702e-04) * q[i4] * q[i4] * q[i8] + (2.262692e-04) * q[i4] * q[i4] * q[i9]
            + (1.723745e-04) * q[i4] * q[i4] * q[i10] + (-2.837358e-03) * q[i4] * q[i4] * q[i11] + (-1.394569e-03) * q[i4] * q[i4] * q[i12]
            + (-3.144852e-04) * q[i4] * q[i4] * q[i15] + (-7.444036e-04) * q[i4] * q[i4] * q[i16] + (2.920938e-04) * q[i4] * q[i4] * q[i19]
            + (1.091014e-03) * q[i4] * q[i4] * q[i20] + (3.029220e-04) * q[i4] * q[i4] * q[i21] + (1.453614e-04) * q[i4] * q[i4] * q[i22]
            + (-2.019421e-03) * q[i0] * q[i5] * q[i5] + (-1.986907e-03) * q[i1] * q[i5] * q[i5] + (-4.393894e-04) * q[i2] * q[i5] * q[i5]
            + (-4.753110e-04) * q[i3] * q[i5] * q[i5] + (-4.325963e-04) * q[i4] * q[i5] * q[i5] + (-4.778403e-03) * q[i5] * q[i5] * q[i5]
            + (-1.737774e-04) * q[i5] * q[i5] * q[i6] + (1.739067e-04) * q[i5] * q[i5] * q[i7] + (8.966249e-06) * q[i5] * q[i5] * q[i8]
            + (1.513727e-04) * q[i5] * q[i5] * q[i9] + (-1.422913e-04) * q[i5] * q[i5] * q[i10] + (2.879716e-04) * q[i5] * q[i5] * q[i11]
            + (2.853626e-04) * q[i5] * q[i5] * q[i12] + (1.071036e-04) * q[i5] * q[i5] * q[i15] + (-9.213145e-05) * q[i5] * q[i5] * q[i16]
            + (4.846585e-04) * q[i5] * q[i5] * q[i19] + (-4.678912e-04) * q[i5] * q[i5] * q[i20] + (-4.744346e-04) * q[i5] * q[i5] * q[i21]
            + (-4.877765e-04) * q[i5] * q[i5] * q[i22] + (3.867929e-03) * q[i0] * q[i6] * q[i6] + (-3.455323e-03) * q[i1] * q[i6] * q[i6]
            + (-4.848208e-04) * q[i2] * q[i6] * q[i6] + (-2.187062e-02) * q[i3] * q[i6] * q[i6] + (3.542552e-03) * q[i4] * q[i6] * q[i6]
            + (4.401184e-03) * q[i5] * q[i6] * q[i6] + (-6.368184e-04) * q[i6] * q[i6] * q[i6] + (-1.625084e-03) * q[i6] * q[i6] * q[i7]
            + (-3.501911e-04) * q[i6] * q[i6] * q[i8] + (-2.486045e-03) * q[i6] * q[i6] * q[i9] + (1.582213e-04) * q[i6] * q[i6] * q[i10]
            + (5.171001e-04) * q[i6] * q[i6] * q[i11] + (-6.927001e-04) * q[i6] * q[i6] * q[i12] + (2.228535e-04) * q[i6] * q[i6] * q[i15]
            + (1.106866e-03) * q[i6] * q[i6] * q[i16] + (-1.366856e-04) * q[i6] * q[i6] * q[i19] + (4.183499e-04) * q[i6] * q[i6] * q[i20]
            + (2.497640e-04) * q[i6] * q[i6] * q[i21] + (-6.035456e-05) * q[i6] * q[i6] * q[i22] + (-3.453669e-03) * q[i0] * q[i7] * q[i7]
            + (3.902302e-03) * q[i1] * q[i7] * q[i7] + (-4.773915e-04) * q[i2] * q[i7] * q[i7] + (3.528850e-03) * q[i3] * q[i7] * q[i7]
            + (-2.179241e-02) * q[i4] * q[i7] * q[i7] + (4.394210e-03) * q[i5] * q[i7] * q[i7] + (1.623807e-03) * q[i6] * q[i7] * q[i7]
            + (6.311920e-04) * q[i7] * q[i7] * q[i7] + (3.571365e-04) * q[i7] * q[i7] * q[i8] + (-1.603254e-04) * q[i7] * q[i7] * q[i9]
            + (2.457586e-03) * q[i7] * q[i7] * q[i10] + (-6.901101e-04) * q[i7] * q[i7] * q[i11] + (5.123069e-04) * q[i7] * q[i7] * q[i12]
            + (-1.107298e-03) * q[i7] * q[i7] * q[i15] + (-2.299380e-04) * q[i7] * q[i7] * q[i16] + (-4.174993e-04) * q[i7] * q[i7] * q[i19]
            + (1.400944e-04) * q[i7] * q[i7] * q[i20] + (-5.819971e-05) * q[i7] * q[i7] * q[i21] + (2.482698e-04) * q[i7] * q[i7] * q[i22]
            + (1.790137e-04) * q[i0] * q[i8] * q[i8] + (2.022371e-04) * q[i1] * q[i8] * q[i8] + (1.764850e-03) * q[i2] * q[i8] * q[i8]
            + (5.135405e-03) * q[i3] * q[i8] * q[i8] + (5.120441e-03) * q[i4] * q[i8] * q[i8] + (-2.642023e-02) * q[i5] * q[i8] * q[i8]
            + (-1.543149e-04) * q[i6] * q[i8] * q[i8] + (1.455535e-04) * q[i7] * q[i8] * q[i8] + (1.545282e-05) * q[i8] * q[i8] * q[i8]
            + (-4.521382e-04) * q[i8] * q[i8] * q[i9] + (4.395179e-04) * q[i8] * q[i8] * q[i10] + (-1.781949e-03) * q[i8] * q[i8] * q[i11]
            + (-1.816467e-03) * q[i8] * q[i8] * q[i12] + (4.086490e-03) * q[i8] * q[i8] * q[i15] + (-4.139522e-03) * q[i8] * q[i8] * q[i16]
            + (-2.545432e-05) * q[i8] * q[i8] * q[i19] + (2.835757e-05) * q[i8] * q[i8] * q[i20] + (-6.908828e-04) * q[i8] * q[i8] * q[i21]
            + (-6.994664e-04) * q[i8] * q[i8] * q[i22] + (-6.786792e-03) * q[i0] * q[i9] * q[i9] + (1.277800e-03) * q[i1] * q[i9] * q[i9]
            + (-2.022016e-03) * q[i2] * q[i9] * q[i9] + (-3.546787e-03) * q[i3] * q[i9] * q[i9] + (4.736603e-04) * q[i4] * q[i9] * q[i9]
            + (1.952825e-03) * q[i5] * q[i9] * q[i9] + (-2.439674e-03) * q[i6] * q[i9] * q[i9] + (1.011449e-03) * q[i7] * q[i9] * q[i9]
            + (3.629383e-04) * q[i8] * q[i9] * q[i9] + (-5.922928e-04) * q[i9] * q[i9] * q[i9] + (2.615357e-04) * q[i9] * q[i9] * q[i10]
            + (-7.666270e-05) * q[i9] * q[i9] * q[i11] + (-6.388348e-04) * q[i9] * q[i9] * q[i12] + (2.176835e-04) * q[i9] * q[i9] * q[i15]
            + (1.270710e-04) * q[i9] * q[i9] * q[i16] + (-1.505523e-04) * q[i9] * q[i9] * q[i19] + (-1.073877e-05) * q[i9] * q[i9] * q[i20]
            + (-2.193992e-05) * q[i9] * q[i9] * q[i21] + (-1.976300e-05) * q[i9] * q[i9] * q[i22] + (1.268991e-03) * q[i0] * q[i10] * q[i10]
            + (-6.708023e-03) * q[i1] * q[i10] * q[i10] + (-1.993552e-03) * q[i2] * q[i10] * q[i10] + (4.730181e-04) * q[i3] * q[i10] * q[i10]
            + (-3.499427e-03) * q[i4] * q[i10] * q[i10] + (1.924849e-03) * q[i5] * q[i10] * q[i10] + (-9.914628e-04) * q[i6] * q[i10] * q[i10]
            + (2.404803e-03) * q[i7] * q[i10] * q[i10] + (-3.565921e-04) * q[i8] * q[i10] * q[i10] + (-2.601219e-04) * q[i9] * q[i10] * q[i10]
            + (5.824171e-04) * q[i10] * q[i10] * q[i10] + (-6.364222e-04) * q[i10] * q[i10] * q[i11] + (-7.248281e-05) * q[i10] * q[i10] * q[i12]
            + (-1.262826e-04) * q[i10] * q[i10] * q[i15] + (-2.161482e-04) * q[i10] * q[i10] * q[i16] + (1.223015e-05) * q[i10] * q[i10] * q[i19]
            + (1.445431e-04) * q[i10] * q[i10] * q[i20] + (-2.028236e-05) * q[i10] * q[i10] * q[i21] + (-2.286918e-05) * q[i10] * q[i10] * q[i22]
            + (2.634201e-04) * q[i0] * q[i11] * q[i11] + (3.093027e-04) * q[i1] * q[i11] * q[i11] + (-4.927059e-05) * q[i2] * q[i11] * q[i11]
            + (-3.952900e-04) * q[i3] * q[i11] * q[i11] + (-1.005719e-03) * q[i4] * q[i11] * q[i11] + (-1.257756e-03) * q[i5] * q[i11] * q[i11]
            + (-5.734439e-05) * q[i6] * q[i11] * q[i11] + (-2.701751e-05) * q[i7] * q[i11] * q[i11] + (4.277118e-04) * q[i8] * q[i11] * q[i11]
            + (-2.190168e-04) * q[i9] * q[i11] * q[i11] + (5.507668e-04) * q[i10] * q[i11] * q[i11] + (-7.581977e-05) * q[i11] * q[i11] * q[i11]
            + (2.780140e-04) * q[i11] * q[i11] * q[i12] + (1.788992e-03) * q[i11] * q[i11] * q[i15] + (1.087262e-04) * q[i11] * q[i11] * q[i16]
            + (4.134884e-04) * q[i11] * q[i11] * q[i19] + (-1.291444e-04) * q[i11] * q[i11] * q[i20] + (8.513405e-04) * q[i11] * q[i11] * q[i21]
            + (6.335773e-05) * q[i11] * q[i11] * q[i22] + (2.812292e-04) * q[i0] * q[i12] * q[i12] + (2.557378e-04) * q[i1] * q[i12] * q[i12]
            + (-1.504999e-04) * q[i2] * q[i12] * q[i12] + (-9.821072e-04) * q[i3] * q[i12] * q[i12] + (-3.937433e-04) * q[i4] * q[i12] * q[i12]
            + (-1.331503e-03) * q[i5] * q[i12] * q[i12] + (4.222588e-05) * q[i6] * q[i12] * q[i12] + (6.581599e-05) * q[i7] * q[i12] * q[i12]
            + (-3.407250e-04) * q[i8] * q[i12] * q[i12] + (-5.664863e-04) * q[i9] * q[i12] * q[i12] + (2.103634e-04) * q[i10] * q[i12] * q[i12]
            + (2.928184e-04) * q[i11] * q[i12] * q[i12] + (-1.318355e-04) * q[i12] * q[i12] * q[i12] + (-1.176538e-04) * q[i12] * q[i12] * q[i15]
            + (-1.806532e-03) * q[i12] * q[i12] * q[i16] + (1.260357e-04) * q[i12] * q[i12] * q[i19] + (-4.017068e-04) * q[i12] * q[i12] * q[i20]
            + (5.719601e-05) * q[i12] * q[i12] * q[i21] + (8.551246e-04) * q[i12] * q[i12] * q[i22] + (-9.181986e-05) * q[i0] * q[i15] * q[i15]
            + (7.592240e-05) * q[i1] * q[i15] * q[i15] + (1.748959e-03) * q[i2] * q[i15] * q[i15] + (6.068730e-04) * q[i3] * q[i15] * q[i15]
            + (7.297165e-04) * q[i4] * q[i15] * q[i15] + (-3.172480e-03) * q[i5] * q[i15] * q[i15] + (8.926814e-05) * q[i6] * q[i15] * q[i15]
            + (-5.139503e-05) * q[i7] * q[i15] * q[i15] + (2.670214e-03) * q[i8] * q[i15] * q[i15] + (-7.293982e-05) * q[i9] * q[i15] * q[i15]
            + (-2.545667e-05) * q[i10] * q[i15] * q[i15] + (2.021889e-03) * q[i11] * q[i15] * q[i15] + (1.280547e-04) * q[i12] * q[i15] * q[i15]
            + (7.975157e-04) * q[i15] * q[i15] * q[i15] + (1.442044e-04) * q[i15] * q[i15] * q[i16] + (2.168470e-04) * q[i15] * q[i15] * q[i19]
            + (1.325622e-04) * q[i15] * q[i15] * q[i20] + (3.968051e-04) * q[i15] * q[i15] * q[i21] + (-1.268897e-05) * q[i15] * q[i15] * q[i22]
            + (6.728841e-05) * q[i0] * q[i16] * q[i16] + (-9.215216e-05) * q[i1] * q[i16] * q[i16] + (1.766318e-03) * q[i2] * q[i16] * q[i16]
            + (7.318281e-04) * q[i3] * q[i16] * q[i16] + (6.037906e-04) * q[i4] * q[i16] * q[i16] + (-3.193741e-03) * q[i5] * q[i16] * q[i16]
            + (5.766638e-05) * q[i6] * q[i16] * q[i16] + (-8.659144e-05) * q[i7] * q[i16] * q[i16] + (-2.682642e-03) * q[i8] * q[i16] * q[i16]
            + (2.559521e-05) * q[i9] * q[i16] * q[i16] + (7.266301e-05) * q[i10] * q[i16] * q[i16] + (1.245777e-04) * q[i11] * q[i16] * q[i16]
            + (2.050267e-03) * q[i12] * q[i16] * q[i16] + (-1.452361e-04) * q[i15] * q[i16] * q[i16] + (-7.998110e-04) * q[i16] * q[i16] * q[i16]
            + (-1.343704e-04) * q[i16] * q[i16] * q[i19] + (-2.153955e-04) * q[i16] * q[i16] * q[i20] + (-1.105465e-05) * q[i16] * q[i16] * q[i21]
            + (4.009891e-04) * q[i16] * q[i16] * q[i22] + (-1.738111e-05) * q[i0] * q[i19] * q[i19] + (-1.020428e-04) * q[i1] * q[i19] * q[i19]
            + (-2.030081e-04) * q[i2] * q[i19] * q[i19] + (1.567991e-04) * q[i3] * q[i19] * q[i19] + (1.412430e-04) * q[i4] * q[i19] * q[i19]
            + (3.073737e-04) * q[i5] * q[i19] * q[i19] + (-1.450210e-04) * q[i6] * q[i19] * q[i19] + (3.239674e-04) * q[i7] * q[i19] * q[i19]
            + (8.407745e-04) * q[i8] * q[i19] * q[i19] + (-3.021958e-05) * q[i9] * q[i19] * q[i19] + (4.162951e-05) * q[i10] * q[i19] * q[i19]
            + (-1.188472e-04) * q[i11] * q[i19] * q[i19] + (-3.760187e-05) * q[i12] * q[i19] * q[i19] + (3.503035e-04) * q[i15] * q[i19] * q[i19]
            + (1.705391e-04) * q[i16] * q[i19] * q[i19] + (1.431031e-04) * q[i19] * q[i19] * q[i19] + (-5.105214e-06) * q[i19] * q[i19] * q[i20]
            + (6.685921e-04) * q[i19] * q[i19] * q[i21] + (-7.300996e-05) * q[i19] * q[i19] * q[i22] + (-1.050280e-04) * q[i0] * q[i20] * q[i20]
            + (-2.351599e-05) * q[i1] * q[i20] * q[i20] + (-2.080122e-04) * q[i2] * q[i20] * q[i20] + (1.332810e-04) * q[i3] * q[i20] * q[i20]
            + (1.504221e-04) * q[i4] * q[i20] * q[i20] + (3.262443e-04) * q[i5] * q[i20] * q[i20] + (-3.249408e-04) * q[i6] * q[i20] * q[i20]
            + (1.465960e-04) * q[i7] * q[i20] * q[i20] + (-8.367389e-04) * q[i8] * q[i20] * q[i20] + (-4.193547e-05) * q[i9] * q[i20] * q[i20]
            + (2.972484e-05) * q[i10] * q[i20] * q[i20] + (-3.707643e-05) * q[i11] * q[i20] * q[i20] + (-1.051264e-04) * q[i12] * q[i20] * q[i20]
            + (-1.661363e-04) * q[i15] * q[i20] * q[i20] + (-3.447732e-04) * q[i16] * q[i20] * q[i20] + (4.827284e-06) * q[i19] * q[i20] * q[i20]
            + (-1.374386e-04) * q[i20] * q[i20] * q[i20] + (-7.287472e-05) * q[i20] * q[i20] * q[i21] + (6.647343e-04) * q[i20] * q[i20] * q[i22]
            + (2.005327e-04) * q[i0] * q[i21] * q[i21] + (3.551545e-04) * q[i1] * q[i21] * q[i21] + (6.122352e-04) * q[i2] * q[i21] * q[i21]
            + (2.876675e-04) * q[i3] * q[i21] * q[i21] + (1.871116e-04) * q[i4] * q[i21] * q[i21] + (-1.374830e-03) * q[i5] * q[i21] * q[i21]
            + (-1.487235e-04) * q[i6] * q[i21] * q[i21] + (-3.780352e-04) * q[i7] * q[i21] * q[i21] + (6.811219e-04) * q[i8] * q[i21] * q[i21]
            + (1.256556e-04) * q[i9] * q[i21] * q[i21] + (-1.027291e-04) * q[i10] * q[i21] * q[i21] + (4.836974e-04) * q[i11] * q[i21] * q[i21]
            + (1.767875e-04) * q[i12] * q[i21] * q[i21] + (7.926800e-04) * q[i15] * q[i21] * q[i21] + (-1.365182e-05) * q[i16] * q[i21] * q[i21]
            + (-9.116763e-05) * q[i19] * q[i21] * q[i21] + (-1.229703e-04) * q[i20] * q[i21] * q[i21] + (3.620149e-04) * q[i21] * q[i21] * q[i21]
            + (3.931188e-05) * q[i21] * q[i21] * q[i22] + (3.563537e-04) * q[i0] * q[i22] * q[i22] + (1.959782e-04) * q[i1] * q[i22] * q[i22]
            + (6.068116e-04) * q[i2] * q[i22] * q[i22] + (1.916338e-04) * q[i3] * q[i22] * q[i22] + (2.904623e-04) * q[i4] * q[i22] * q[i22]
            + (-1.379575e-03) * q[i5] * q[i22] * q[i22] + (3.754935e-04) * q[i6] * q[i22] * q[i22] + (1.424931e-04) * q[i7] * q[i22] * q[i22]
            + (-6.902480e-04) * q[i8] * q[i22] * q[i22] + (1.046439e-04) * q[i9] * q[i22] * q[i22] + (-1.250614e-04) * q[i10] * q[i22] * q[i22]
            + (1.762423e-04) * q[i11] * q[i22] * q[i22] + (4.884731e-04) * q[i12] * q[i22] * q[i22] + (1.745091e-05) * q[i15] * q[i22] * q[i22]
            + (-7.943027e-04) * q[i16] * q[i22] * q[i22] + (1.245651e-04) * q[i19] * q[i22] * q[i22] + (8.810742e-05) * q[i20] * q[i22] * q[i22]
            + (4.055755e-05) * q[i21] * q[i22] * q[i22] + (3.661811e-04) * q[i22] * q[i22] * q[i22] + (-4.256968e-03) * q[i0] * q[i1] * q[i2]
            + (7.202945e-03) * q[i0] * q[i1] * q[i3] + (7.154852e-03) * q[i0] * q[i1] * q[i4] + (-5.894587e-03) * q[i0] * q[i1] * q[i5]
            + (1.950107e-03) * q[i0] * q[i1] * q[i6] + (-1.925395e-03) * q[i0] * q[i1] * q[i7] + (4.442005e-06) * q[i0] * q[i1] * q[i8]
            + (-1.418768e-03) * q[i0] * q[i1] * q[i9] + (1.421309e-03) * q[i0] * q[i1] * q[i10] + (-8.110965e-04) * q[i0] * q[i1] * q[i11]
            + (-8.088057e-04) * q[i0] * q[i1] * q[i12] + (1.151046e-03) * q[i0] * q[i1] * q[i15] + (-1.169626e-03) * q[i0] * q[i1] * q[i16]
            + (-5.858271e-04) * q[i0] * q[i1] * q[i19] + (5.868450e-04) * q[i0] * q[i1] * q[i20] + (2.626669e-04) * q[i0] * q[i1] * q[i21]
            + (2.594540e-04) * q[i0] * q[i1] * q[i22] + (-1.283325e-02) * q[i0] * q[i2] * q[i3] + (1.855208e-03) * q[i0] * q[i2] * q[i4]
            + (-1.194723e-02) * q[i0] * q[i2] * q[i5] + (1.345896e-03) * q[i0] * q[i2] * q[i6] + (-2.001371e-03) * q[i0] * q[i2] * q[i7]
            + (-3.725796e-03) * q[i0] * q[i2] * q[i8] + (1.292606e-03) * q[i0] * q[i2] * q[i9] + (5.500446e-04) * q[i0] * q[i2] * q[i10]
            + (6.652454e-04) * q[i0] * q[i2] * q[i11] + (1.396673e-03) * q[i0] * q[i2] * q[i12] + (-6.568085e-05) * q[i0] * q[i2] * q[i15]
            + (1.546440e-03) * q[i0] * q[i2] * q[i16] + (4.345169e-04) * q[i0] * q[i2] * q[i19] + (1.349169e-03) * q[i0] * q[i2] * q[i20]
            + (5.814716e-04) * q[i0] * q[i2] * q[i21] + (1.058040e-04) * q[i0] * q[i2] * q[i22] + (-1.104364e-05) * q[i0] * q[i3] * q[i4]
            + (6.232324e-04) * q[i0] * q[i3] * q[i5] + (-1.220277e-02) * q[i0] * q[i3] * q[i6] + (4.599825e-04) * q[i0] * q[i3] * q[i7]
            + (2.879513e-03) * q[i0] * q[i3] * q[i8] + (-1.020871e-03) * q[i0] * q[i3] * q[i9] + (1.436438e-03) * q[i0] * q[i3] * q[i10]
            + (4.059029e-05) * q[i0] * q[i3] * q[i11] + (-1.685471e-03) * q[i0] * q[i3] * q[i12] + (2.231693e-03) * q[i0] * q[i3] * q[i15]
            + (8.670147e-04) * q[i0] * q[i3] * q[i16] + (1.522527e-03) * q[i0] * q[i3] * q[i19] + (1.245394e-03) * q[i0] * q[i3] * q[i20]
            + (-8.057844e-04) * q[i0] * q[i3] * q[i21] + (-7.652511e-04) * q[i0] * q[i3] * q[i22] + (4.044206e-03) * q[i0] * q[i4] * q[i5]
            + (-3.852007e-03) * q[i0] * q[i4] * q[i6] + (-2.147826e-04) * q[i0] * q[i4] * q[i7] + (-1.860052e-03) * q[i0] * q[i4] * q[i8]
            + (-6.762930e-04) * q[i0] * q[i4] * q[i9] + (7.799416e-04) * q[i0] * q[i4] * q[i10] + (1.098684e-03) * q[i0] * q[i4] * q[i11]
            + (-1.804400e-03) * q[i0] * q[i4] * q[i12] + (-5.569488e-04) * q[i0] * q[i4] * q[i15] + (-3.701069e-04) * q[i0] * q[i4] * q[i16]
            + (-1.016211e-03) * q[i0] * q[i4] * q[i19] + (-9.938613e-04) * q[i0] * q[i4] * q[i20] + (-3.788516e-04) * q[i0] * q[i4] * q[i21]
            + (6.018258e-04) * q[i0] * q[i4] * q[i22] + (-8.060198e-03) * q[i0] * q[i5] * q[i6] + (-4.824738e-04) * q[i0] * q[i5] * q[i7]
            + (-3.618220e-04) * q[i0] * q[i5] * q[i8] + (4.046716e-04) * q[i0] * q[i5] * q[i9] + (-8.990503e-04) * q[i0] * q[i5] * q[i10]
            + (-6.948002e-05) * q[i0] * q[i5] * q[i11] + (2.869565e-04) * q[i0] * q[i5] * q[i12] + (7.623461e-04) * q[i0] * q[i5] * q[i15]
            + (-1.635196e-03) * q[i0] * q[i5] * q[i16] + (-8.694627e-04) * q[i0] * q[i5] * q[i19] + (1.593632e-04) * q[i0] * q[i5] * q[i20]
            + (6.680767e-04) * q[i0] * q[i5] * q[i21] + (-2.400176e-04) * q[i0] * q[i5] * q[i22] + (-4.657516e-04) * q[i0] * q[i6] * q[i7]
            + (7.409014e-03) * q[i0] * q[i6] * q[i8] + (-1.360672e-02) * q[i0] * q[i6] * q[i9] + (1.714814e-03) * q[i0] * q[i6] * q[i10]
            + (-1.493674e-03) * q[i0] * q[i6] * q[i11] + (3.205563e-04) * q[i0] * q[i6] * q[i12] + (3.511050e-04) * q[i0] * q[i6] * q[i15]
            + (2.069774e-03) * q[i0] * q[i6] * q[i16] + (1.222475e-03) * q[i0] * q[i6] * q[i19] + (1.359705e-03) * q[i0] * q[i6] * q[i20]
            + (-8.987898e-04) * q[i0] * q[i6] * q[i21] + (6.300372e-04) * q[i0] * q[i6] * q[i22] + (-1.297956e-03) * q[i0] * q[i7] * q[i8]
            + (-1.327951e-04) * q[i0] * q[i7] * q[i9] + (3.413640e-03) * q[i0] * q[i7] * q[i10] + (-9.047856e-04) * q[i0] * q[i7] * q[i11]
            + (-1.669194e-03) * q[i0] * q[i7] * q[i12] + (-6.133359e-04) * q[i0] * q[i7] * q[i15] + (7.881685e-04) * q[i0] * q[i7] * q[i16]
            + (-6.159552e-04) * q[i0] * q[i7] * q[i19] + (-9.066067e-04) * q[i0] * q[i7] * q[i20] + (-2.006549e-04) * q[i0] * q[i7] * q[i21]
            + (-8.392349e-04) * q[i0] * q[i7] * q[i22] + (7.407671e-04) * q[i0] * q[i8] * q[i9] + (-1.721679e-04) * q[i0] * q[i8] * q[i10]
            + (-5.959634e-04) * q[i0] * q[i8] * q[i11] + (1.649874e-04) * q[i0] * q[i8] * q[i12] + (9.418310e-04) * q[i0] * q[i8] * q[i15]
            + (2.296953e-04) * q[i0] * q[i8] * q[i16] + (1.349733e-03) * q[i0] * q[i8] * q[i19] + (1.054646e-03) * q[i0] * q[i8] * q[i20]
            + (3.697073e-04) * q[i0] * q[i8] * q[i21] + (-3.461998e-04) * q[i0] * q[i8] * q[i22] + (7.379308e-04) * q[i0] * q[i9] * q[i10]
            + (1.292349e-05) * q[i0] * q[i9] * q[i11] + (-5.578639e-04) * q[i0] * q[i9] * q[i12] + (7.242263e-04) * q[i0] * q[i9] * q[i15]
            + (2.030050e-04) * q[i0] * q[i9] * q[i16] + (8.202329e-04) * q[i0] * q[i9] * q[i19] + (5.663061e-04) * q[i0] * q[i9] * q[i20]
            + (-4.827697e-04) * q[i0] * q[i9] * q[i21] + (-5.376229e-05) * q[i0] * q[i9] * q[i22] + (-8.957915e-04) * q[i0] * q[i10] * q[i11]
            + (-5.991964e-05) * q[i0] * q[i10] * q[i12] + (-8.095689e-05) * q[i0] * q[i10] * q[i15] + (1.579682e-04) * q[i0] * q[i10] * q[i16]
            + (-3.351315e-04) * q[i0] * q[i10] * q[i19] + (-6.638931e-04) * q[i0] * q[i10] * q[i20] + (-6.733610e-04) * q[i0] * q[i10] * q[i21]
            + (-4.308028e-04) * q[i0] * q[i10] * q[i22] + (1.037433e-03) * q[i0] * q[i11] * q[i12] + (1.111723e-03) * q[i0] * q[i11] * q[i15]
            + (-6.870321e-05) * q[i0] * q[i11] * q[i16] + (6.155529e-04) * q[i0] * q[i11] * q[i19] + (-2.593303e-04) * q[i0] * q[i11] * q[i20]
            + (7.605467e-04) * q[i0] * q[i11] * q[i21] + (3.498287e-04) * q[i0] * q[i11] * q[i22] + (-3.508971e-04) * q[i0] * q[i12] * q[i15]
            + (-3.126500e-04) * q[i0] * q[i12] * q[i16] + (1.308050e-04) * q[i0] * q[i12] * q[i19] + (-5.335765e-04) * q[i0] * q[i12] * q[i20]
            + (4.240182e-04) * q[i0] * q[i12] * q[i21] + (-7.301080e-04) * q[i0] * q[i12] * q[i22] + (5.148387e-04) * q[i0] * q[i15] * q[i16]
            + (-1.717268e-04) * q[i0] * q[i15] * q[i19] + (4.410306e-04) * q[i0] * q[i15] * q[i20] + (2.398073e-04) * q[i0] * q[i15] * q[i21]
            + (5.474682e-04) * q[i0] * q[i15] * q[i22] + (-1.048530e-03) * q[i0] * q[i16] * q[i19] + (8.003445e-04) * q[i0] * q[i16] * q[i20]
            + (-2.354564e-04) * q[i0] * q[i16] * q[i21] + (-2.929430e-05) * q[i0] * q[i16] * q[i22] + (4.840219e-04) * q[i0] * q[i19] * q[i20]
            + (-2.430971e-04) * q[i0] * q[i19] * q[i21] + (-5.005325e-04) * q[i0] * q[i19] * q[i22] + (7.464442e-04) * q[i0] * q[i20] * q[i21]
            + (8.370573e-04) * q[i0] * q[i20] * q[i22] + (1.707779e-04) * q[i0] * q[i21] * q[i22] + (1.897785e-03) * q[i1] * q[i2] * q[i3]
            + (-1.277614e-02) * q[i1] * q[i2] * q[i4] + (-1.187656e-02) * q[i1] * q[i2] * q[i5] + (2.015449e-03) * q[i1] * q[i2] * q[i6]
            + (-1.331022e-03) * q[i1] * q[i2] * q[i7] + (3.702176e-03) * q[i1] * q[i2] * q[i8] + (-5.259012e-04) * q[i1] * q[i2] * q[i9]
            + (-1.279786e-03) * q[i1] * q[i2] * q[i10] + (1.404683e-03) * q[i1] * q[i2] * q[i11] + (6.646431e-04) * q[i1] * q[i2] * q[i12]
            + (-1.531927e-03) * q[i1] * q[i2] * q[i15] + (6.903362e-05) * q[i1] * q[i2] * q[i16] + (-1.356130e-03) * q[i1] * q[i2] * q[i19]
            + (-4.233245e-04) * q[i1] * q[i2] * q[i20] + (1.109319e-04) * q[i1] * q[i2] * q[i21] + (5.835975e-04) * q[i1] * q[i2] * q[i22]
            + (6.927884e-06) * q[i1] * q[i3] * q[i4] + (4.044748e-03) * q[i1] * q[i3] * q[i5] + (1.776476e-04) * q[i1] * q[i3] * q[i6]
            + (3.785884e-03) * q[i1] * q[i3] * q[i7] + (1.894027e-03) * q[i1] * q[i3] * q[i8] + (-7.905932e-04) * q[i1] * q[i3] * q[i9]
            + (6.715230e-04) * q[i1] * q[i3] * q[i10] + (-1.809140e-03) * q[i1] * q[i3] * q[i11] + (1.062638e-03) * q[i1] * q[i3] * q[i12]
            + (3.459426e-04) * q[i1] * q[i3] * q[i15] + (5.483259e-04) * q[i1] * q[i3] * q[i16] + (9.862217e-04) * q[i1] * q[i3] * q[i19]
            + (1.000061e-03) * q[i1] * q[i3] * q[i20] + (5.962383e-04) * q[i1] * q[i3] * q[i21] + (-3.858473e-04) * q[i1] * q[i3] * q[i22]
            + (6.049404e-04) * q[i1] * q[i4] * q[i5] + (-4.309909e-04) * q[i1] * q[i4] * q[i6] + (1.215547e-02) * q[i1] * q[i4] * q[i7]
            + (-2.884023e-03) * q[i1] * q[i4] * q[i8] + (-1.469909e-03) * q[i1] * q[i4] * q[i9] + (1.007366e-03) * q[i1] * q[i4] * q[i10]
            + (-1.716275e-03) * q[i1] * q[i4] * q[i11] + (6.743792e-05) * q[i1] * q[i4] * q[i12] + (-8.506545e-04) * q[i1] * q[i4] * q[i15]
            + (-2.229472e-03) * q[i1] * q[i4] * q[i16] + (-1.241991e-03) * q[i1] * q[i4] * q[i19] + (-1.501474e-03) * q[i1] * q[i4] * q[i20]
            + (-7.710065e-04) * q[i1] * q[i4] * q[i21] + (-8.268980e-04) * q[i1] * q[i4] * q[i22] + (4.644945e-04) * q[i1] * q[i5] * q[i6]
            + (8.116384e-03) * q[i1] * q[i5] * q[i7] + (3.976265e-04) * q[i1] * q[i5] * q[i8] + (8.545363e-04) * q[i1] * q[i5] * q[i9]
            + (-3.880431e-04) * q[i1] * q[i5] * q[i10] + (3.387693e-04) * q[i1] * q[i5] * q[i11] + (-6.398775e-05) * q[i1] * q[i5] * q[i12]
            + (1.632663e-03) * q[i1] * q[i5] * q[i15] + (-7.627753e-04) * q[i1] * q[i5] * q[i16] + (-1.608649e-04) * q[i1] * q[i5] * q[i19]
            + (8.323787e-04) * q[i1] * q[i5] * q[i20] + (-2.455253e-04) * q[i1] * q[i5] * q[i21] + (6.741223e-04) * q[i1] * q[i5] * q[i22]
            + (-5.072719e-04) * q[i1] * q[i6] * q[i7] + (-1.312707e-03) * q[i1] * q[i6] * q[i8] + (3.417191e-03) * q[i1] * q[i6] * q[i9]
            + (-1.111062e-04) * q[i1] * q[i6] * q[i10] + (1.669093e-03) * q[i1] * q[i6] * q[i11] + (9.281863e-04) * q[i1] * q[i6] * q[i12]
            + (7.933511e-04) * q[i1] * q[i6] * q[i15] + (-6.163040e-04) * q[i1] * q[i6] * q[i16] + (-9.142841e-04) * q[i1] * q[i6] * q[i19]
            + (-6.114105e-04) * q[i1] * q[i6] * q[i20] + (8.439397e-04) * q[i1] * q[i6] * q[i21] + (1.852810e-04) * q[i1] * q[i6] * q[i22]
            + (7.388077e-03) * q[i1] * q[i7] * q[i8] + (1.747766e-03) * q[i1] * q[i7] * q[i9] + (-1.345148e-02) * q[i1] * q[i7] * q[i10]
            + (-2.929429e-04) * q[i1] * q[i7] * q[i11] + (1.519737e-03) * q[i1] * q[i7] * q[i12] + (2.038911e-03) * q[i1] * q[i7] * q[i15]
            + (3.758379e-04) * q[i1] * q[i7] * q[i16] + (1.349245e-03) * q[i1] * q[i7] * q[i19] + (1.199835e-03) * q[i1] * q[i7] * q[i20]
            + (-6.180849e-04) * q[i1] * q[i7] * q[i21] + (9.134953e-04) * q[i1] * q[i7] * q[i22] + (-1.896148e-04) * q[i1] * q[i8] * q[i9]
            + (7.417262e-04) * q[i1] * q[i8] * q[i10] + (-1.253281e-04) * q[i1] * q[i8] * q[i11] + (6.184059e-04) * q[i1] * q[i8] * q[i12]
            + (2.099563e-04) * q[i1] * q[i8] * q[i15] + (9.333350e-04) * q[i1] * q[i8] * q[i16] + (1.060160e-03) * q[i1] * q[i8] * q[i19]
            + (1.348211e-03) * q[i1] * q[i8] * q[i20] + (3.351103e-04) * q[i1] * q[i8] * q[i21] + (-3.669231e-04) * q[i1] * q[i8] * q[i22]
            + (7.396080e-04) * q[i1] * q[i9] * q[i10] + (6.116578e-05) * q[i1] * q[i9] * q[i11] + (9.128566e-04) * q[i1] * q[i9] * q[i12]
            + (1.494251e-04) * q[i1] * q[i9] * q[i15] + (-7.386152e-05) * q[i1] * q[i9] * q[i16] + (-6.724032e-04) * q[i1] * q[i9] * q[i19]
            + (-3.448627e-04) * q[i1] * q[i9] * q[i20] + (4.375698e-04) * q[i1] * q[i9] * q[i21] + (6.739413e-04) * q[i1] * q[i9] * q[i22]
            + (5.542927e-04) * q[i1] * q[i10] * q[i11] + (-1.430565e-05) * q[i1] * q[i10] * q[i12] + (1.871632e-04) * q[i1] * q[i10] * q[i15]
            + (7.150911e-04) * q[i1] * q[i10] * q[i16] + (5.671184e-04) * q[i1] * q[i10] * q[i19] + (8.148392e-04) * q[i1] * q[i10] * q[i20]
            + (5.603105e-05) * q[i1] * q[i10] * q[i21] + (4.743153e-04) * q[i1] * q[i10] * q[i22] + (1.050391e-03) * q[i1] * q[i11] * q[i12]
            + (2.987142e-04) * q[i1] * q[i11] * q[i15] + (3.494695e-04) * q[i1] * q[i11] * q[i16] + (5.263009e-04) * q[i1] * q[i11] * q[i19]
            + (-1.317697e-04) * q[i1] * q[i11] * q[i20] + (-7.379106e-04) * q[i1] * q[i11] * q[i21] + (4.227531e-04) * q[i1] * q[i11] * q[i22]
            + (5.891167e-05) * q[i1] * q[i12] * q[i15] + (-1.117033e-03) * q[i1] * q[i12] * q[i16] + (2.595780e-04) * q[i1] * q[i12] * q[i19]
            + (-6.142505e-04) * q[i1] * q[i12] * q[i20] + (3.496778e-04) * q[i1] * q[i12] * q[i21] + (7.630297e-04) * q[i1] * q[i12] * q[i22]
            + (5.088894e-04) * q[i1] * q[i15] * q[i16] + (8.008115e-04) * q[i1] * q[i15] * q[i19] + (-1.042180e-03) * q[i1] * q[i15] * q[i20]
            + (2.924900e-05) * q[i1] * q[i15] * q[i21] + (2.277305e-04) * q[i1] * q[i15] * q[i22] + (4.527292e-04) * q[i1] * q[i16] * q[i19]
            + (-1.634519e-04) * q[i1] * q[i16] * q[i20] + (-5.340071e-04) * q[i1] * q[i16] * q[i21] + (-2.454685e-04) * q[i1] * q[i16] * q[i22]
            + (4.608439e-04) * q[i1] * q[i19] * q[i20] + (-8.342090e-04) * q[i1] * q[i19] * q[i21] + (-7.396063e-04) * q[i1] * q[i19] * q[i22]
            + (5.044497e-04) * q[i1] * q[i20] * q[i21] + (2.404725e-04) * q[i1] * q[i20] * q[i22] + (1.760598e-04) * q[i1] * q[i21] * q[i22]
            + (2.120604e-03) * q[i2] * q[i3] * q[i4] + (5.379433e-03) * q[i2] * q[i3] * q[i5] + (-3.754849e-03) * q[i2] * q[i3] * q[i6]
            + (1.078962e-03) * q[i2] * q[i3] * q[i7] + (3.084937e-03) * q[i2] * q[i3] * q[i8] + (1.821206e-04) * q[i2] * q[i3] * q[i9]
            + (-4.290648e-04) * q[i2] * q[i3] * q[i10] + (5.062225e-04) * q[i2] * q[i3] * q[i11] + (-4.700165e-04) * q[i2] * q[i3] * q[i12]
            + (1.418867e-03) * q[i2] * q[i3] * q[i15] + (1.015846e-04) * q[i2] * q[i3] * q[i16] + (-9.577654e-04) * q[i2] * q[i3] * q[i19]
            + (5.090413e-04) * q[i2] * q[i3] * q[i20] + (-7.309653e-04) * q[i2] * q[i3] * q[i21] + (1.000350e-03) * q[i2] * q[i3] * q[i22]
            + (5.443058e-03) * q[i2] * q[i4] * q[i5] + (-1.066613e-03) * q[i2] * q[i4] * q[i6] + (3.717713e-03) * q[i2] * q[i4] * q[i7]
            + (-3.042462e-03) * q[i2] * q[i4] * q[i8] + (4.203955e-04) * q[i2] * q[i4] * q[i9] + (-1.730749e-04) * q[i2] * q[i4] * q[i10]
            + (-4.936573e-04) * q[i2] * q[i4] * q[i11] + (5.365713e-04) * q[i2] * q[i4] * q[i12] + (-9.815592e-05) * q[i2] * q[i4] * q[i15]
            + (-1.455408e-03) * q[i2] * q[i4] * q[i16] + (-5.021187e-04) * q[i2] * q[i4] * q[i19] + (9.312876e-04) * q[i2] * q[i4] * q[i20]
            + (9.853955e-04) * q[i2] * q[i4] * q[i21] + (-7.527671e-04) * q[i2] * q[i4] * q[i22] + (-1.897680e-04) * q[i2] * q[i5] * q[i6]
            + (1.789531e-04) * q[i2] * q[i5] * q[i7] + (-2.629788e-05) * q[i2] * q[i5] * q[i8] + (2.116555e-03) * q[i2] * q[i5] * q[i9]
            + (-2.136809e-03) * q[i2] * q[i5] * q[i10] + (1.737642e-03) * q[i2] * q[i5] * q[i11] + (1.689365e-03) * q[i2] * q[i5] * q[i12]
            + (-1.444219e-03) * q[i2] * q[i5] * q[i15] + (1.441259e-03) * q[i2] * q[i5] * q[i16] + (1.186527e-03) * q[i2] * q[i5] * q[i19]
            + (-1.194357e-03) * q[i2] * q[i5] * q[i20] + (5.688177e-04) * q[i2] * q[i5] * q[i21] + (5.556571e-04) * q[i2] * q[i5] * q[i22]
            + (3.344318e-03) * q[i2] * q[i6] * q[i7] + (5.734035e-03) * q[i2] * q[i6] * q[i8] + (-2.505272e-03) * q[i2] * q[i6] * q[i9]
            + (7.022326e-04) * q[i2] * q[i6] * q[i10] + (-7.407081e-04) * q[i2] * q[i6] * q[i11] + (-3.079661e-04) * q[i2] * q[i6] * q[i12]
            + (5.709624e-04) * q[i2] * q[i6] * q[i15] + (2.251452e-04) * q[i2] * q[i6] * q[i16] + (1.268388e-03) * q[i2] * q[i6] * q[i19]
            + (3.018136e-04) * q[i2] * q[i6] * q[i20] + (1.285496e-04) * q[i2] * q[i6] * q[i21] + (3.242108e-04) * q[i2] * q[i6] * q[i22]
            + (5.755823e-03) * q[i2] * q[i7] * q[i8] + (7.057201e-04) * q[i2] * q[i7] * q[i9] + (-2.443772e-03) * q[i2] * q[i7] * q[i10]
            + (3.318719e-04) * q[i2] * q[i7] * q[i11] + (7.383186e-04) * q[i2] * q[i7] * q[i12] + (2.397672e-04) * q[i2] * q[i7] * q[i15]
            + (5.739001e-04) * q[i2] * q[i7] * q[i16] + (2.983729e-04) * q[i2] * q[i7] * q[i19] + (1.274549e-03) * q[i2] * q[i7] * q[i20]
            + (-3.183364e-04) * q[i2] * q[i7] * q[i21] + (-1.257341e-04) * q[i2] * q[i7] * q[i22] + (-2.458956e-03) * q[i2] * q[i8] * q[i9]
            + (-2.408560e-03) * q[i2] * q[i8] * q[i10] + (-1.244990e-03) * q[i2] * q[i8] * q[i11] + (1.411772e-03) * q[i2] * q[i8] * q[i12]
            + (4.348435e-03) * q[i2] * q[i8] * q[i15] + (4.428257e-03) * q[i2] * q[i8] * q[i16] + (1.071222e-03) * q[i2] * q[i8] * q[i19]
            + (1.062139e-03) * q[i2] * q[i8] * q[i20] + (-5.368134e-04) * q[i2] * q[i8] * q[i21] + (5.417197e-04) * q[i2] * q[i8] * q[i22]
            + (-1.138271e-04) * q[i2] * q[i9] * q[i10] + (-4.428682e-04) * q[i2] * q[i9] * q[i11] + (1.445852e-04) * q[i2] * q[i9] * q[i12]
            + (6.495961e-04) * q[i2] * q[i9] * q[i15] + (-4.232450e-04) * q[i2] * q[i9] * q[i16] + (1.164111e-04) * q[i2] * q[i9] * q[i19]
            + (3.097910e-04) * q[i2] * q[i9] * q[i20] + (-3.177849e-04) * q[i2] * q[i9] * q[i21] + (2.041463e-04) * q[i2] * q[i9] * q[i22]
            + (-1.591874e-04) * q[i2] * q[i10] * q[i11] + (4.314493e-04) * q[i2] * q[i10] * q[i12] + (-4.261787e-04) * q[i2] * q[i10] * q[i15]
            + (6.458251e-04) * q[i2] * q[i10] * q[i16] + (3.252194e-04) * q[i2] * q[i10] * q[i19] + (1.234765e-04) * q[i2] * q[i10] * q[i20]
            + (-2.055229e-04) * q[i2] * q[i10] * q[i21] + (3.180438e-04) * q[i2] * q[i10] * q[i22] + (1.565082e-03) * q[i2] * q[i11] * q[i12]
            + (3.600388e-03) * q[i2] * q[i11] * q[i15] + (-6.828608e-04) * q[i2] * q[i11] * q[i16] + (1.696311e-03) * q[i2] * q[i11] * q[i19]
            + (5.165242e-05) * q[i2] * q[i11] * q[i20] + (8.125435e-05) * q[i2] * q[i11] * q[i21] + (-1.375290e-04) * q[i2] * q[i11] * q[i22]
            + (6.583024e-04) * q[i2] * q[i12] * q[i15] + (-3.614837e-03) * q[i2] * q[i12] * q[i16] + (-6.191380e-05) * q[i2] * q[i12] * q[i19]
            + (-1.678354e-03) * q[i2] * q[i12] * q[i20] + (-1.404150e-04) * q[i2] * q[i12] * q[i21] + (8.041627e-05) * q[i2] * q[i12] * q[i22]
            + (4.283540e-05) * q[i2] * q[i15] * q[i16] + (-9.625853e-04) * q[i2] * q[i15] * q[i19] + (-8.625585e-04) * q[i2] * q[i15] * q[i20]
            + (3.715050e-04) * q[i2] * q[i15] * q[i21] + (3.675351e-04) * q[i2] * q[i15] * q[i22] + (-8.608036e-04) * q[i2] * q[i16] * q[i19]
            + (-9.489442e-04) * q[i2] * q[i16] * q[i20] + (-3.632431e-04) * q[i2] * q[i16] * q[i21] + (-3.792889e-04) * q[i2] * q[i16] * q[i22]
            + (1.081195e-03) * q[i2] * q[i19] * q[i20] + (-1.188958e-03) * q[i2] * q[i19] * q[i21] + (-5.315273e-04) * q[i2] * q[i19] * q[i22]
            + (5.348327e-04) * q[i2] * q[i20] * q[i21] + (1.183849e-03) * q[i2] * q[i20] * q[i22] + (2.080626e-04) * q[i2] * q[i21] * q[i22]
            + (-1.625450e-03) * q[i3] * q[i4] * q[i5] + (2.142900e-03) * q[i3] * q[i4] * q[i6] + (-2.125483e-03) * q[i3] * q[i4] * q[i7]
            + (-2.209751e-05) * q[i3] * q[i4] * q[i8] + (-4.865914e-05) * q[i3] * q[i4] * q[i9] + (5.663865e-05) * q[i3] * q[i4] * q[i10]
            + (1.984605e-03) * q[i3] * q[i4] * q[i11] + (1.988030e-03) * q[i3] * q[i4] * q[i12] + (-6.454528e-05) * q[i3] * q[i4] * q[i15]
            + (7.420731e-05) * q[i3] * q[i4] * q[i16] + (-7.927073e-04) * q[i3] * q[i4] * q[i19] + (7.792435e-04) * q[i3] * q[i4] * q[i20]
            + (-7.415448e-04) * q[i3] * q[i4] * q[i21] + (-7.459352e-04) * q[i3] * q[i4] * q[i22] + (-6.349387e-03) * q[i3] * q[i5] * q[i6]
            + (5.954575e-04) * q[i3] * q[i5] * q[i7] + (-4.838856e-04) * q[i3] * q[i5] * q[i8] + (-1.903444e-04) * q[i3] * q[i5] * q[i9]
            + (-1.450961e-03) * q[i3] * q[i5] * q[i10] + (-8.940210e-04) * q[i3] * q[i5] * q[i11] + (-3.053467e-04) * q[i3] * q[i5] * q[i12]
            + (3.455274e-04) * q[i3] * q[i5] * q[i15] + (9.330827e-04) * q[i3] * q[i5] * q[i16] + (-8.253939e-04) * q[i3] * q[i5] * q[i19]
            + (-1.240502e-03) * q[i3] * q[i5] * q[i20] + (3.996984e-05) * q[i3] * q[i5] * q[i21] + (1.160825e-03) * q[i3] * q[i5] * q[i22]
            + (3.245163e-04) * q[i3] * q[i6] * q[i7] + (-3.734237e-03) * q[i3] * q[i6] * q[i8] + (-1.030080e-02) * q[i3] * q[i6] * q[i9]
            + (5.297648e-04) * q[i3] * q[i6] * q[i10] + (1.117085e-04) * q[i3] * q[i6] * q[i11] + (1.625795e-04) * q[i3] * q[i6] * q[i12]
            + (1.331850e-03) * q[i3] * q[i6] * q[i15] + (-1.027732e-03) * q[i3] * q[i6] * q[i16] + (-2.949664e-03) * q[i3] * q[i6] * q[i19]
            + (-1.961914e-04) * q[i3] * q[i6] * q[i20] + (-8.116925e-04) * q[i3] * q[i6] * q[i21] + (3.394242e-04) * q[i3] * q[i6] * q[i22]
            + (-3.796529e-04) * q[i3] * q[i7] * q[i8] + (1.223988e-04) * q[i3] * q[i7] * q[i9] + (1.490958e-03) * q[i3] * q[i7] * q[i10]
            + (-9.071750e-04) * q[i3] * q[i7] * q[i11] + (7.903289e-04) * q[i3] * q[i7] * q[i12] + (-7.167150e-04) * q[i3] * q[i7] * q[i15]
            + (-1.838161e-03) * q[i3] * q[i7] * q[i16] + (-1.571947e-04) * q[i3] * q[i7] * q[i19] + (1.872840e-03) * q[i3] * q[i7] * q[i20]
            + (1.340993e-03) * q[i3] * q[i7] * q[i21] + (7.346822e-04) * q[i3] * q[i7] * q[i22] + (-7.091580e-04) * q[i3] * q[i8] * q[i9]
            + (9.674446e-04) * q[i3] * q[i8] * q[i10] + (-8.620245e-04) * q[i3] * q[i8] * q[i11] + (1.817324e-03) * q[i3] * q[i8] * q[i12]
            + (4.639511e-04) * q[i3] * q[i8] * q[i15] + (-2.946867e-04) * q[i3] * q[i8] * q[i16] + (-1.082728e-03) * q[i3] * q[i8] * q[i19]
            + (4.997872e-05) * q[i3] * q[i8] * q[i20] + (1.835052e-04) * q[i3] * q[i8] * q[i21] + (-1.137578e-04) * q[i3] * q[i8] * q[i22]
            + (-1.125415e-03) * q[i3] * q[i9] * q[i10] + (-5.818825e-04) * q[i3] * q[i9] * q[i11] + (2.452206e-04) * q[i3] * q[i9] * q[i12]
            + (-2.710467e-04) * q[i3] * q[i9] * q[i15] + (-4.947617e-04) * q[i3] * q[i9] * q[i16] + (2.575008e-04) * q[i3] * q[i9] * q[i19]
            + (-1.099831e-03) * q[i3] * q[i9] * q[i20] + (2.429874e-04) * q[i3] * q[i9] * q[i21] + (-1.711662e-04) * q[i3] * q[i9] * q[i22]
            + (-7.761758e-05) * q[i3] * q[i10] * q[i11] + (2.547123e-04) * q[i3] * q[i10] * q[i12] + (-6.593369e-04) * q[i3] * q[i10] * q[i15]
            + (-5.319852e-05) * q[i3] * q[i10] * q[i16] + (9.528878e-04) * q[i3] * q[i10] * q[i19] + (-1.118422e-04) * q[i3] * q[i10] * q[i20]
            + (-2.816300e-04) * q[i3] * q[i10] * q[i21] + (3.857699e-04) * q[i3] * q[i10] * q[i22] + (-9.431192e-04) * q[i3] * q[i11] * q[i12]
            + (-2.399187e-03) * q[i3] * q[i11] * q[i15] + (9.598698e-04) * q[i3] * q[i11] * q[i16] + (3.549022e-04) * q[i3] * q[i11] * q[i19]
            + (-6.758842e-05) * q[i3] * q[i11] * q[i20] + (-8.200449e-04) * q[i3] * q[i11] * q[i21] + (-1.065079e-03) * q[i3] * q[i11] * q[i22]
            + (3.537319e-04) * q[i3] * q[i12] * q[i15] + (-4.212583e-04) * q[i3] * q[i12] * q[i16] + (-5.466593e-04) * q[i3] * q[i12] * q[i19]
            + (8.499335e-04) * q[i3] * q[i12] * q[i20] + (8.870416e-04) * q[i3] * q[i12] * q[i21] + (-2.562754e-04) * q[i3] * q[i12] * q[i22]
            + (6.317741e-04) * q[i3] * q[i15] * q[i16] + (-4.963041e-04) * q[i3] * q[i15] * q[i19] + (-5.463849e-04) * q[i3] * q[i15] * q[i20]
            + (-1.765358e-04) * q[i3] * q[i15] * q[i21] + (2.002703e-04) * q[i3] * q[i15] * q[i22] + (7.135509e-04) * q[i3] * q[i16] * q[i19]
            + (4.429594e-04) * q[i3] * q[i16] * q[i20] + (-2.275339e-05) * q[i3] * q[i16] * q[i21] + (-5.864281e-04) * q[i3] * q[i16] * q[i22]
            + (6.337695e-04) * q[i3] * q[i19] * q[i20] + (1.284728e-03) * q[i3] * q[i19] * q[i21] + (-3.871210e-04) * q[i3] * q[i19] * q[i22]
            + (-3.009105e-04) * q[i3] * q[i20] * q[i21] + (-1.722683e-04) * q[i3] * q[i20] * q[i22] + (2.200795e-04) * q[i3] * q[i21] * q[i22]
            + (-5.931727e-04) * q[i4] * q[i5] * q[i6] + (6.315469e-03) * q[i4] * q[i5] * q[i7] + (4.671618e-04) * q[i4] * q[i5] * q[i8]
            + (1.461121e-03) * q[i4] * q[i5] * q[i9] + (1.876810e-04) * q[i4] * q[i5] * q[i10] + (-3.199444e-04) * q[i4] * q[i5] * q[i11]
            + (-8.964191e-04) * q[i4] * q[i5] * q[i12] + (-9.330154e-04) * q[i4] * q[i5] * q[i15] + (-3.794327e-04) * q[i4] * q[i5] * q[i16]
            + (1.241729e-03) * q[i4] * q[i5] * q[i19] + (8.317277e-04) * q[i4] * q[i5] * q[i20] + (1.183960e-03) * q[i4] * q[i5] * q[i21]
            + (7.428462e-05) * q[i4] * q[i5] * q[i22] + (3.288686e-04) * q[i4] * q[i6] * q[i7] + (-3.885931e-04) * q[i4] * q[i6] * q[i8]
            + (1.505838e-03) * q[i4] * q[i6] * q[i9] + (1.077158e-04) * q[i4] * q[i6] * q[i10] + (-7.801497e-04) * q[i4] * q[i6] * q[i11]
            + (8.819036e-04) * q[i4] * q[i6] * q[i12] + (-1.836521e-03) * q[i4] * q[i6] * q[i15] + (-7.226764e-04) * q[i4] * q[i6] * q[i16]
            + (1.863697e-03) * q[i4] * q[i6] * q[i19] + (-1.470723e-04) * q[i4] * q[i6] * q[i20] + (-7.265601e-04) * q[i4] * q[i6] * q[i21]
            + (-1.339614e-03) * q[i4] * q[i6] * q[i22] + (-3.720154e-03) * q[i4] * q[i7] * q[i8] + (5.630538e-04) * q[i4] * q[i7] * q[i9]
            + (-1.020911e-02) * q[i4] * q[i7] * q[i10] + (-1.659809e-04) * q[i4] * q[i7] * q[i11] + (-1.034888e-04) * q[i4] * q[i7] * q[i12]
            + (-1.031517e-03) * q[i4] * q[i7] * q[i15] + (1.346195e-03) * q[i4] * q[i7] * q[i16] + (-1.888206e-04) * q[i4] * q[i7] * q[i19]
            + (-2.927266e-03) * q[i4] * q[i7] * q[i20] + (-3.443914e-04) * q[i4] * q[i7] * q[i21] + (8.115579e-04) * q[i4] * q[i7] * q[i22]
            + (9.636456e-04) * q[i4] * q[i8] * q[i9] + (-7.088078e-04) * q[i4] * q[i8] * q[i10] + (-1.784835e-03) * q[i4] * q[i8] * q[i11]
            + (8.717079e-04) * q[i4] * q[i8] * q[i12] + (-2.745696e-04) * q[i4] * q[i8] * q[i15] + (4.798022e-04) * q[i4] * q[i8] * q[i16]
            + (5.460890e-05) * q[i4] * q[i8] * q[i19] + (-1.055922e-03) * q[i4] * q[i8] * q[i20] + (1.136042e-04) * q[i4] * q[i8] * q[i21]
            + (-1.860377e-04) * q[i4] * q[i8] * q[i22] + (-1.116048e-03) * q[i4] * q[i9] * q[i10] + (-2.538559e-04) * q[i4] * q[i9] * q[i11]
            + (7.054069e-05) * q[i4] * q[i9] * q[i12] + (-6.173504e-05) * q[i4] * q[i9] * q[i15] + (-6.497450e-04) * q[i4] * q[i9] * q[i16]
            + (-1.113940e-04) * q[i4] * q[i9] * q[i19] + (9.484636e-04) * q[i4] * q[i9] * q[i20] + (-3.795851e-04) * q[i4] * q[i9] * q[i21]
            + (2.792880e-04) * q[i4] * q[i9] * q[i22] + (-2.539604e-04) * q[i4] * q[i10] * q[i11] + (5.829149e-04) * q[i4] * q[i10] * q[i12]
            + (-4.795333e-04) * q[i4] * q[i10] * q[i15] + (-2.686769e-04) * q[i4] * q[i10] * q[i16] + (-1.089708e-03) * q[i4] * q[i10] * q[i19]
            + (2.507049e-04) * q[i4] * q[i10] * q[i20] + (1.679941e-04) * q[i4] * q[i10] * q[i21] + (-2.440550e-04) * q[i4] * q[i10] * q[i22]
            + (-9.341547e-04) * q[i4] * q[i11] * q[i12] + (4.198854e-04) * q[i4] * q[i11] * q[i15] + (-3.551721e-04) * q[i4] * q[i11] * q[i16]
            + (-8.331422e-04) * q[i4] * q[i11] * q[i19] + (5.438664e-04) * q[i4] * q[i11] * q[i20] + (-2.493820e-04) * q[i4] * q[i11] * q[i21]
            + (8.925204e-04) * q[i4] * q[i11] * q[i22] + (-9.618192e-04) * q[i4] * q[i12] * q[i15] + (2.398704e-03) * q[i4] * q[i12] * q[i16]
            + (5.877753e-05) * q[i4] * q[i12] * q[i19] + (-3.604102e-04) * q[i4] * q[i12] * q[i20] + (-1.056132e-03) * q[i4] * q[i12] * q[i21]
            + (-8.202490e-04) * q[i4] * q[i12] * q[i22] + (6.267090e-04) * q[i4] * q[i15] * q[i16] + (4.461442e-04) * q[i4] * q[i15] * q[i19]
            + (7.146477e-04) * q[i4] * q[i15] * q[i20] + (5.753389e-04) * q[i4] * q[i15] * q[i21] + (1.739075e-05) * q[i4] * q[i15] * q[i22]
            + (-5.510075e-04) * q[i4] * q[i16] * q[i19] + (-4.883223e-04) * q[i4] * q[i16] * q[i20] + (-1.986130e-04) * q[i4] * q[i16] * q[i21]
            + (1.665167e-04) * q[i4] * q[i16] * q[i22] + (6.201209e-04) * q[i4] * q[i19] * q[i20] + (1.730511e-04) * q[i4] * q[i19] * q[i21]
            + (3.040057e-04) * q[i4] * q[i19] * q[i22] + (3.878512e-04) * q[i4] * q[i20] * q[i21] + (-1.278620e-03) * q[i4] * q[i20] * q[i22]
            + (2.073554e-04) * q[i4] * q[i21] * q[i22] + (2.967315e-03) * q[i5] * q[i6] * q[i7] + (1.843466e-03) * q[i5] * q[i6] * q[i8]
            + (3.600806e-03) * q[i5] * q[i6] * q[i9] + (-1.788734e-03) * q[i5] * q[i6] * q[i10] + (1.185197e-04) * q[i5] * q[i6] * q[i11]
            + (-1.385588e-03) * q[i5] * q[i6] * q[i12] + (-2.008343e-04) * q[i5] * q[i6] * q[i15] + (3.957857e-04) * q[i5] * q[i6] * q[i16]
            + (2.459765e-04) * q[i5] * q[i6] * q[i19] + (6.275868e-04) * q[i5] * q[i6] * q[i20] + (4.400469e-04) * q[i5] * q[i6] * q[i21]
            + (5.221971e-04) * q[i5] * q[i6] * q[i22] + (1.807979e-03) * q[i5] * q[i7] * q[i8] + (-1.811998e-03) * q[i5] * q[i7] * q[i9]
            + (3.594889e-03) * q[i5] * q[i7] * q[i10] + (1.385137e-03) * q[i5] * q[i7] * q[i11] + (-1.108449e-04) * q[i5] * q[i7] * q[i12]
            + (3.823836e-04) * q[i5] * q[i7] * q[i15] + (-2.080655e-04) * q[i5] * q[i7] * q[i16] + (6.288068e-04) * q[i5] * q[i7] * q[i19]
            + (2.373423e-04) * q[i5] * q[i7] * q[i20] + (-5.079405e-04) * q[i5] * q[i7] * q[i21] + (-4.616325e-04) * q[i5] * q[i7] * q[i22]
            + (-1.520993e-03) * q[i5] * q[i8] * q[i9] + (-1.520071e-03) * q[i5] * q[i8] * q[i10] + (1.744742e-03) * q[i5] * q[i8] * q[i11]
            + (-1.778833e-03) * q[i5] * q[i8] * q[i12] + (1.608008e-03) * q[i5] * q[i8] * q[i15] + (1.625924e-03) * q[i5] * q[i8] * q[i16]
            + (-1.509317e-03) * q[i5] * q[i8] * q[i19] + (-1.517377e-03) * q[i5] * q[i8] * q[i20] + (6.891492e-04) * q[i5] * q[i8] * q[i21]
            + (-7.023000e-04) * q[i5] * q[i8] * q[i22] + (3.736259e-04) * q[i5] * q[i9] * q[i10] + (-3.088273e-04) * q[i5] * q[i9] * q[i11]
            + (-1.763527e-04) * q[i5] * q[i9] * q[i12] + (-1.271194e-05) * q[i5] * q[i9] * q[i15] + (-2.967064e-06) * q[i5] * q[i9] * q[i16]
            + (-1.319905e-04) * q[i5] * q[i9] * q[i19] + (6.840200e-05) * q[i5] * q[i9] * q[i20] + (5.699121e-05) * q[i5] * q[i9] * q[i21]
            + (-1.983944e-04) * q[i5] * q[i9] * q[i22] + (1.690130e-04) * q[i5] * q[i10] * q[i11] + (3.018424e-04) * q[i5] * q[i10] * q[i12]
            + (1.603742e-06) * q[i5] * q[i10] * q[i15] + (-2.413245e-05) * q[i5] * q[i10] * q[i16] + (6.552438e-05) * q[i5] * q[i10] * q[i19]
            + (-1.415325e-04) * q[i5] * q[i10] * q[i20] + (1.988570e-04) * q[i5] * q[i10] * q[i21] + (-7.148407e-05) * q[i5] * q[i10] * q[i22]
            + (1.094104e-03) * q[i5] * q[i11] * q[i12] + (2.062101e-03) * q[i5] * q[i11] * q[i15] + (-6.159860e-04) * q[i5] * q[i11] * q[i16]
            + (6.998644e-04) * q[i5] * q[i11] * q[i19] + (-4.212765e-04) * q[i5] * q[i11] * q[i20] + (-2.158314e-04) * q[i5] * q[i11] * q[i21]
            + (-3.102334e-04) * q[i5] * q[i11] * q[i22] + (6.111405e-04) * q[i5] * q[i12] * q[i15] + (-2.064967e-03) * q[i5] * q[i12] * q[i16]
            + (4.324835e-04) * q[i5] * q[i12] * q[i19] + (-7.160581e-04) * q[i5] * q[i12] * q[i20] + (-3.110414e-04) * q[i5] * q[i12] * q[i21]
            + (-2.057889e-04) * q[i5] * q[i12] * q[i22] + (-1.431438e-04) * q[i5] * q[i15] * q[i16] + (-7.355210e-04) * q[i5] * q[i15] * q[i19]
            + (-3.359705e-04) * q[i5] * q[i15] * q[i20] + (-1.399082e-03) * q[i5] * q[i15] * q[i21] + (1.192572e-04) * q[i5] * q[i15] * q[i22]
            + (-3.321778e-04) * q[i5] * q[i16] * q[i19] + (-7.404865e-04) * q[i5] * q[i16] * q[i20] + (-1.216028e-04) * q[i5] * q[i16] * q[i21]
            + (1.415512e-03) * q[i5] * q[i16] * q[i22] + (-5.472268e-04) * q[i5] * q[i19] * q[i20] + (-1.777132e-03) * q[i5] * q[i19] * q[i21]
            + (5.059876e-04) * q[i5] * q[i19] * q[i22] + (-5.014894e-04) * q[i5] * q[i20] * q[i21] + (1.779544e-03) * q[i5] * q[i20] * q[i22]
            + (-5.758126e-04) * q[i5] * q[i21] * q[i22] + (8.144014e-06) * q[i6] * q[i7] * q[i8] + (1.094756e-03) * q[i6] * q[i7] * q[i9]
            + (-1.072949e-03) * q[i6] * q[i7] * q[i10] + (9.749898e-04) * q[i6] * q[i7] * q[i11] + (9.681896e-04) * q[i6] * q[i7] * q[i12]
            + (-1.479991e-04) * q[i6] * q[i7] * q[i15] + (1.486869e-04) * q[i6] * q[i7] * q[i16] + (3.730544e-04) * q[i6] * q[i7] * q[i19]
            + (-3.841827e-04) * q[i6] * q[i7] * q[i20] + (-2.921076e-04) * q[i6] * q[i7] * q[i21] + (-2.994227e-04) * q[i6] * q[i7] * q[i22]
            + (2.919645e-04) * q[i6] * q[i8] * q[i9] + (5.735390e-05) * q[i6] * q[i8] * q[i10] + (3.118483e-04) * q[i6] * q[i8] * q[i11]
            + (3.574590e-04) * q[i6] * q[i8] * q[i12] + (1.204213e-05) * q[i6] * q[i8] * q[i15] + (8.807517e-04) * q[i6] * q[i8] * q[i16]
            + (-3.218953e-04) * q[i6] * q[i8] * q[i19] + (-2.926902e-06) * q[i6] * q[i8] * q[i20] + (3.936861e-05) * q[i6] * q[i8] * q[i21]
            + (6.272907e-04) * q[i6] * q[i8] * q[i22] + (1.057842e-03) * q[i6] * q[i9] * q[i10] + (-8.023481e-04) * q[i6] * q[i9] * q[i11]
            + (-6.623801e-05) * q[i6] * q[i9] * q[i12] + (-5.787308e-05) * q[i6] * q[i9] * q[i15] + (5.530778e-04) * q[i6] * q[i9] * q[i16]
            + (-5.147763e-04) * q[i6] * q[i9] * q[i19] + (-2.811633e-04) * q[i6] * q[i9] * q[i20] + (-5.704405e-06) * q[i6] * q[i9] * q[i21]
            + (6.697351e-06) * q[i6] * q[i9] * q[i22] + (1.632620e-04) * q[i6] * q[i10] * q[i11] + (1.932459e-04) * q[i6] * q[i10] * q[i12]
            + (5.898438e-05) * q[i6] * q[i10] * q[i15] + (-3.333471e-05) * q[i6] * q[i10] * q[i16] + (-3.367955e-06) * q[i6] * q[i10] * q[i19]
            + (-6.403651e-05) * q[i6] * q[i10] * q[i20] + (6.740169e-04) * q[i6] * q[i10] * q[i21] + (3.724830e-04) * q[i6] * q[i10] * q[i22]
            + (-9.789896e-05) * q[i6] * q[i11] * q[i12] + (1.189101e-03) * q[i6] * q[i11] * q[i15] + (1.504720e-04) * q[i6] * q[i11] * q[i16]
            + (2.453448e-05) * q[i6] * q[i11] * q[i19] + (1.247018e-03) * q[i6] * q[i11] * q[i20] + (-3.542402e-04) * q[i6] * q[i11] * q[i21]
            + (-3.928276e-04) * q[i6] * q[i11] * q[i22] + (1.037932e-03) * q[i6] * q[i12] * q[i15] + (3.039606e-04) * q[i6] * q[i12] * q[i16]
            + (1.155972e-04) * q[i6] * q[i12] * q[i19] + (-1.390614e-04) * q[i6] * q[i12] * q[i20] + (9.065239e-05) * q[i6] * q[i12] * q[i21]
            + (-5.720250e-04) * q[i6] * q[i12] * q[i22] + (9.620974e-05) * q[i6] * q[i15] * q[i16] + (2.382661e-04) * q[i6] * q[i15] * q[i19]
            + (-2.391586e-04) * q[i6] * q[i15] * q[i20] + (-5.152226e-05) * q[i6] * q[i15] * q[i21] + (2.113318e-04) * q[i6] * q[i15] * q[i22]
            + (-4.223975e-04) * q[i6] * q[i16] * q[i19] + (1.109683e-05) * q[i6] * q[i16] * q[i20] + (1.089463e-04) * q[i6] * q[i16] * q[i21]
            + (1.466666e-04) * q[i6] * q[i16] * q[i22] + (1.838184e-04) * q[i6] * q[i19] * q[i20] + (-1.257420e-04) * q[i6] * q[i19] * q[i21]
            + (-7.666956e-05) * q[i6] * q[i19] * q[i22] + (8.421887e-04) * q[i6] * q[i20] * q[i21] + (-6.182464e-04) * q[i6] * q[i20] * q[i22]
            + (-1.593029e-04) * q[i6] * q[i21] * q[i22] + (-5.487595e-05) * q[i7] * q[i8] * q[i9] + (-2.853329e-04) * q[i7] * q[i8] * q[i10]
            + (3.848970e-04) * q[i7] * q[i8] * q[i11] + (3.219046e-04) * q[i7] * q[i8] * q[i12] + (-8.874344e-04) * q[i7] * q[i8] * q[i15]
            + (-6.459206e-07) * q[i7] * q[i8] * q[i16] + (2.425172e-06) * q[i7] * q[i8] * q[i19] + (3.094587e-04) * q[i7] * q[i8] * q[i20]
            + (6.312095e-04) * q[i7] * q[i8] * q[i21] + (3.965294e-05) * q[i7] * q[i8] * q[i22] + (-1.062174e-03) * q[i7] * q[i9] * q[i10]
            + (2.032087e-04) * q[i7] * q[i9] * q[i11] + (1.531247e-04) * q[i7] * q[i9] * q[i12] + (3.024924e-05) * q[i7] * q[i9] * q[i15]
            + (-5.499585e-05) * q[i7] * q[i9] * q[i16] + (6.235673e-05) * q[i7] * q[i9] * q[i19] + (-3.490015e-07) * q[i7] * q[i9] * q[i20]
            + (3.703065e-04) * q[i7] * q[i9] * q[i21] + (6.726312e-04) * q[i7] * q[i9] * q[i22] + (-6.176399e-05) * q[i7] * q[i10] * q[i11]
            + (-7.972379e-04) * q[i7] * q[i10] * q[i12] + (-5.512347e-04) * q[i7] * q[i10] * q[i15] + (6.889978e-05) * q[i7] * q[i10] * q[i16]
            + (2.787552e-04) * q[i7] * q[i10] * q[i19] + (5.044036e-04) * q[i7] * q[i10] * q[i20] + (6.170180e-06) * q[i7] * q[i10] * q[i21]
            + (-1.011493e-05) * q[i7] * q[i10] * q[i22] + (1.092357e-04) * q[i7] * q[i11] * q[i12] + (3.001006e-04) * q[i7] * q[i11] * q[i15]
            + (1.036485e-03) * q[i7] * q[i11] * q[i16] + (-1.439265e-04) * q[i7] * q[i11] * q[i19] + (1.173055e-04) * q[i7] * q[i11] * q[i20]
            + (5.647104e-04) * q[i7] * q[i11] * q[i21] + (-8.827581e-05) * q[i7] * q[i11] * q[i22] + (1.521089e-04) * q[i7] * q[i12] * q[i15]
            + (1.174695e-03) * q[i7] * q[i12] * q[i16] + (1.244186e-03) * q[i7] * q[i12] * q[i19] + (2.493281e-05) * q[i7] * q[i12] * q[i20]
            + (3.914591e-04) * q[i7] * q[i12] * q[i21] + (3.562947e-04) * q[i7] * q[i12] * q[i22] + (-9.605176e-05) * q[i7] * q[i15] * q[i16]
            + (-1.144394e-05) * q[i7] * q[i15] * q[i19] + (4.214893e-04) * q[i7] * q[i15] * q[i20] + (1.458426e-04) * q[i7] * q[i15] * q[i21]
            + (1.105229e-04) * q[i7] * q[i15] * q[i22] + (2.379539e-04) * q[i7] * q[i16] * q[i19] + (-2.380913e-04) * q[i7] * q[i16] * q[i20]
            + (2.063762e-04) * q[i7] * q[i16] * q[i21] + (-5.774850e-05) * q[i7] * q[i16] * q[i22] + (-1.807043e-04) * q[i7] * q[i19] * q[i20]
            + (-6.118894e-04) * q[i7] * q[i19] * q[i21] + (8.430038e-04) * q[i7] * q[i19] * q[i22] + (-8.372730e-05) * q[i7] * q[i20] * q[i21]
            + (-1.313692e-04) * q[i7] * q[i20] * q[i22] + (1.575562e-04) * q[i7] * q[i21] * q[i22] + (3.051791e-06) * q[i8] * q[i9] * q[i10]
            + (3.713144e-04) * q[i8] * q[i9] * q[i11] + (1.890128e-03) * q[i8] * q[i9] * q[i12] + (-6.984873e-04) * q[i8] * q[i9] * q[i15]
            + (5.208003e-04) * q[i8] * q[i9] * q[i16] + (-2.432863e-04) * q[i8] * q[i9] * q[i19] + (5.502630e-05) * q[i8] * q[i9] * q[i20]
            + (1.687497e-04) * q[i8] * q[i9] * q[i21] + (-6.421777e-04) * q[i8] * q[i9] * q[i22] + (1.871301e-03) * q[i8] * q[i10] * q[i11]
            + (3.728237e-04) * q[i8] * q[i10] * q[i12] + (-5.211538e-04) * q[i8] * q[i10] * q[i15] + (6.973763e-04) * q[i8] * q[i10] * q[i16]
            + (-4.806814e-05) * q[i8] * q[i10] * q[i19] + (2.439533e-04) * q[i8] * q[i10] * q[i20] + (-6.443793e-04) * q[i8] * q[i10] * q[i21]
            + (1.681949e-04) * q[i8] * q[i10] * q[i22] + (-1.148201e-05) * q[i8] * q[i11] * q[i12] + (3.140654e-03) * q[i8] * q[i11] * q[i15]
            + (9.446172e-04) * q[i8] * q[i11] * q[i16] + (1.077931e-03) * q[i8] * q[i11] * q[i19] + (-1.831442e-04) * q[i8] * q[i11] * q[i20]
            + (1.827731e-03) * q[i8] * q[i11] * q[i21] + (-3.900571e-05) * q[i8] * q[i11] * q[i22] + (9.501223e-04) * q[i8] * q[i12] * q[i15]
            + (3.159565e-03) * q[i8] * q[i12] * q[i16] + (-1.796733e-04) * q[i8] * q[i12] * q[i19] + (1.059739e-03) * q[i8] * q[i12] * q[i20]
            + (4.025228e-05) * q[i8] * q[i12] * q[i21] + (-1.843622e-03) * q[i8] * q[i12] * q[i22] + (5.080928e-06) * q[i8] * q[i15] * q[i16]
            + (4.603370e-04) * q[i8] * q[i15] * q[i19] + (2.050454e-04) * q[i8] * q[i15] * q[i20] + (-1.066225e-04) * q[i8] * q[i15] * q[i21]
            + (-4.518506e-04) * q[i8] * q[i15] * q[i22] + (-2.102962e-04) * q[i8] * q[i16] * q[i19] + (-4.268456e-04) * q[i8] * q[i16] * q[i20]
            + (-4.533561e-04) * q[i8] * q[i16] * q[i21] + (-8.338577e-05) * q[i8] * q[i16] * q[i22] + (5.811766e-08) * q[i8] * q[i19] * q[i20]
            + (-1.353064e-03) * q[i8] * q[i19] * q[i21] + (-1.799147e-05) * q[i8] * q[i19] * q[i22] + (-8.841363e-06) * q[i8] * q[i20] * q[i21]
            + (-1.338323e-03) * q[i8] * q[i20] * q[i22] + (-1.803648e-06) * q[i8] * q[i21] * q[i22] + (-1.215391e-04) * q[i9] * q[i10] * q[i11]
            + (-1.253624e-04) * q[i9] * q[i10] * q[i12] + (3.373777e-04) * q[i9] * q[i10] * q[i15] + (-3.369648e-04) * q[i9] * q[i10] * q[i16]
            + (-4.452825e-05) * q[i9] * q[i10] * q[i19] + (4.880631e-05) * q[i9] * q[i10] * q[i20] + (1.934615e-04) * q[i9] * q[i10] * q[i21]
            + (1.904536e-04) * q[i9] * q[i10] * q[i22] + (-2.414728e-04) * q[i9] * q[i11] * q[i12] + (1.564612e-04) * q[i9] * q[i11] * q[i15]
            + (1.517742e-04) * q[i9] * q[i11] * q[i16] + (4.364411e-05) * q[i9] * q[i11] * q[i19] + (2.088469e-04) * q[i9] * q[i11] * q[i20]
            + (2.569153e-05) * q[i9] * q[i11] * q[i21] + (-1.682093e-04) * q[i9] * q[i11] * q[i22] + (-1.483531e-04) * q[i9] * q[i12] * q[i15]
            + (6.375883e-05) * q[i9] * q[i12] * q[i16] + (-8.314378e-05) * q[i9] * q[i12] * q[i19] + (-5.902861e-05) * q[i9] * q[i12] * q[i20]
            + (2.179966e-04) * q[i9] * q[i12] * q[i21] + (9.555884e-05) * q[i9] * q[i12] * q[i22] + (-2.402656e-04) * q[i9] * q[i15] * q[i16]
            + (1.483290e-04) * q[i9] * q[i15] * q[i19] + (1.634168e-04) * q[i9] * q[i15] * q[i20] + (6.501105e-05) * q[i9] * q[i15] * q[i21]
            + (-3.990033e-05) * q[i9] * q[i15] * q[i22] + (-2.577399e-04) * q[i9] * q[i16] * q[i19] + (1.908896e-04) * q[i9] * q[i16] * q[i20]
            + (4.393048e-05) * q[i9] * q[i16] * q[i21] + (-1.016466e-04) * q[i9] * q[i16] * q[i22] + (1.544749e-04) * q[i9] * q[i19] * q[i20]
            + (-2.061993e-05) * q[i9] * q[i19] * q[i21] + (1.277229e-04) * q[i9] * q[i19] * q[i22] + (1.741792e-04) * q[i9] * q[i20] * q[i21]
            + (-3.629460e-04) * q[i9] * q[i20] * q[i22] + (-1.726096e-05) * q[i9] * q[i21] * q[i22] + (2.459750e-04) * q[i10] * q[i11] * q[i12]
            + (6.393224e-05) * q[i10] * q[i11] * q[i15] + (-1.507483e-04) * q[i10] * q[i11] * q[i16] + (-4.681708e-05) * q[i10] * q[i11] * q[i19]
            + (-8.202952e-05) * q[i10] * q[i11] * q[i20] + (-9.864996e-05) * q[i10] * q[i11] * q[i21] + (-2.166452e-04) * q[i10] * q[i11] * q[i22]
            + (1.453329e-04) * q[i10] * q[i12] * q[i15] + (1.589430e-04) * q[i10] * q[i12] * q[i16] + (2.078339e-04) * q[i10] * q[i12] * q[i19]
            + (4.660277e-05) * q[i10] * q[i12] * q[i20] + (1.714616e-04) * q[i10] * q[i12] * q[i21] + (-2.717697e-05) * q[i10] * q[i12] * q[i22]
            + (2.381822e-04) * q[i10] * q[i15] * q[i16] + (-1.906830e-04) * q[i10] * q[i15] * q[i19] + (2.572036e-04) * q[i10] * q[i15] * q[i20]
            + (-9.423758e-05) * q[i10] * q[i15] * q[i21] + (4.601349e-05) * q[i10] * q[i15] * q[i22] + (-1.615727e-04) * q[i10] * q[i16] * q[i19]
            + (-1.470767e-04) * q[i10] * q[i16] * q[i20] + (-4.275113e-05) * q[i10] * q[i16] * q[i21] + (6.190540e-05) * q[i10] * q[i16] * q[i22]
            + (-1.553178e-04) * q[i10] * q[i19] * q[i20] + (-3.572332e-04) * q[i10] * q[i19] * q[i21] + (1.728515e-04) * q[i10] * q[i19] * q[i22]
            + (1.273184e-04) * q[i10] * q[i20] * q[i21] + (-2.134184e-05) * q[i10] * q[i20] * q[i22] + (1.970438e-05) * q[i10] * q[i21] * q[i22]
            + (3.146763e-04) * q[i11] * q[i12] * q[i15] + (-3.128072e-04) * q[i11] * q[i12] * q[i16] + (3.545202e-04) * q[i11] * q[i12] * q[i19]
            + (-3.485009e-04) * q[i11] * q[i12] * q[i20] + (-4.512187e-04) * q[i11] * q[i12] * q[i21] + (-4.540651e-04) * q[i11] * q[i12] * q[i22]
            + (-1.783143e-04) * q[i11] * q[i15] * q[i16] + (-4.017164e-04) * q[i11] * q[i15] * q[i19] + (-2.733872e-05) * q[i11] * q[i15] * q[i20]
            + (-4.815746e-04) * q[i11] * q[i15] * q[i21] + (-1.275106e-04) * q[i11] * q[i15] * q[i22] + (5.782590e-05) * q[i11] * q[i16] * q[i19]
            + (1.390104e-04) * q[i11] * q[i16] * q[i20] + (-6.533038e-04) * q[i11] * q[i16] * q[i21] + (-3.496984e-04) * q[i11] * q[i16] * q[i22]
            + (-2.405631e-04) * q[i11] * q[i19] * q[i20] + (-2.057011e-03) * q[i11] * q[i19] * q[i21] + (6.084827e-05) * q[i11] * q[i19] * q[i22]
            + (1.702902e-04) * q[i11] * q[i20] * q[i21] + (-2.498516e-04) * q[i11] * q[i20] * q[i22] + (8.413113e-05) * q[i11] * q[i21] * q[i22]
            + (-1.691708e-04) * q[i12] * q[i15] * q[i16] + (1.354376e-04) * q[i12] * q[i15] * q[i19] + (6.039360e-05) * q[i12] * q[i15] * q[i20]
            + (3.469436e-04) * q[i12] * q[i15] * q[i21] + (6.541224e-04) * q[i12] * q[i15] * q[i22] + (-2.004387e-05) * q[i12] * q[i16] * q[i19]
            + (-4.167561e-04) * q[i12] * q[i16] * q[i20] + (1.295322e-04) * q[i12] * q[i16] * q[i21] + (4.850914e-04) * q[i12] * q[i16] * q[i22]
            + (-2.413345e-04) * q[i12] * q[i19] * q[i20] + (2.520685e-04) * q[i12] * q[i19] * q[i21] + (-1.723724e-04) * q[i12] * q[i19] * q[i22]
            + (-6.266256e-05) * q[i12] * q[i20] * q[i21] + (2.050566e-03) * q[i12] * q[i20] * q[i22] + (8.656308e-05) * q[i12] * q[i21] * q[i22]
            + (2.134674e-04) * q[i15] * q[i16] * q[i19] + (-2.091147e-04) * q[i15] * q[i16] * q[i20] + (1.579448e-04) * q[i15] * q[i16] * q[i21]
            + (1.535920e-04) * q[i15] * q[i16] * q[i22] + (1.092114e-05) * q[i15] * q[i19] * q[i20] + (5.399830e-05) * q[i15] * q[i19] * q[i21]
            + (-8.669724e-05) * q[i15] * q[i19] * q[i22] + (1.848389e-06) * q[i15] * q[i20] * q[i21] + (-2.399921e-05) * q[i15] * q[i20] * q[i22]
            + (-6.705985e-05) * q[i15] * q[i21] * q[i22] + (-7.888197e-06) * q[i16] * q[i19] * q[i20] + (-2.416558e-05) * q[i16] * q[i19] * q[i21]
            + (5.949074e-06) * q[i16] * q[i19] * q[i22] + (-8.441539e-05) * q[i16] * q[i20] * q[i21] + (6.254680e-05) * q[i16] * q[i20] * q[i22]
            + (6.619962e-05) * q[i16] * q[i21] * q[i22] + (1.641668e-04) * q[i19] * q[i20] * q[i21] + (1.603013e-04) * q[i19] * q[i20] * q[i22]
            + (6.385282e-05) * q[i19] * q[i21] * q[i22] + (-5.825105e-05) * q[i20] * q[i21] * q[i22];
      return Qx;
   }

   public double getQy(double[] q)
   {
      double Qy;
      Qy = (-1.118106e-03) * q[i0] + (1.171536e-03) * q[i1] + (1.371481e-04) * q[i2] + (-2.286804e-03) * q[i3] + (2.272458e-03) * q[i4]
            + (-5.235205e-05) * q[i5] + (1.286961e-01) * q[i6] + (1.279249e-01) * q[i7] + (1.062151e-01) * q[i8] + (5.879427e-02) * q[i9]
            + (5.814686e-02) * q[i10] + (-5.222022e-03) * q[i11] + (5.619843e-03) * q[i12] + (5.040684e-03) * q[i15] + (4.959764e-03) * q[i16]
            + (-1.357233e-03) * q[i19] + (-1.260901e-03) * q[i20] + (5.083198e-03) * q[i21] + (-5.078924e-03) * q[i22] + (4.467133e-03) * q[i0] * q[i0]
            + (4.412114e-03) * q[i1] * q[i1] + (3.642564e-04) * q[i2] * q[i2] + (-2.063380e-04) * q[i3] * q[i3] + (-2.102849e-04) * q[i4] * q[i4]
            + (-2.271242e-03) * q[i5] * q[i5] + (-1.276062e-03) * q[i6] * q[i6] + (-1.230130e-03) * q[i7] * q[i7] + (-2.514359e-03) * q[i8] * q[i8]
            + (-3.273431e-03) * q[i9] * q[i9] + (-3.258967e-03) * q[i10] * q[i10] + (-3.690845e-03) * q[i11] * q[i11] + (-3.610286e-03) * q[i12] * q[i12]
            + (-2.841511e-03) * q[i15] * q[i15] + (-2.872103e-03) * q[i16] * q[i16] + (-2.122095e-05) * q[i19] * q[i19] + (-7.609279e-06) * q[i20] * q[i20]
            + (-1.589365e-04) * q[i21] * q[i21] + (-1.608665e-04) * q[i22] * q[i22] + (7.115423e-05) * q[i0] * q[i1] + (2.760604e-03) * q[i0] * q[i2]
            + (-1.082041e-01) * q[i0] * q[i3] + (1.091409e-02) * q[i0] * q[i4] + (1.464916e-03) * q[i0] * q[i5] + (6.008783e-03) * q[i0] * q[i6]
            + (5.937328e-04) * q[i0] * q[i7] + (1.223880e-04) * q[i0] * q[i8] + (-1.577200e-03) * q[i0] * q[i9] + (-2.364382e-03) * q[i0] * q[i10]
            + (-2.101461e-03) * q[i0] * q[i11] + (-6.247272e-04) * q[i0] * q[i12] + (1.763563e-03) * q[i0] * q[i15] + (-5.522394e-03) * q[i0] * q[i16]
            + (-1.921777e-03) * q[i0] * q[i19] + (8.267937e-04) * q[i0] * q[i20] + (2.222365e-03) * q[i0] * q[i21] + (-8.863129e-04) * q[i0] * q[i22]
            + (2.773411e-03) * q[i1] * q[i2] + (1.089383e-02) * q[i1] * q[i3] + (-1.076574e-01) * q[i1] * q[i4] + (1.596455e-03) * q[i1] * q[i5]
            + (-6.207352e-04) * q[i1] * q[i6] + (-6.060779e-03) * q[i1] * q[i7] + (-1.238358e-04) * q[i1] * q[i8] + (2.346357e-03) * q[i1] * q[i9]
            + (1.575809e-03) * q[i1] * q[i10] + (-6.405035e-04) * q[i1] * q[i11] + (-2.114922e-03) * q[i1] * q[i12] + (5.440769e-03) * q[i1] * q[i15]
            + (-1.795247e-03) * q[i1] * q[i16] + (-8.235971e-04) * q[i1] * q[i19] + (1.904894e-03) * q[i1] * q[i20] + (-8.990263e-04) * q[i1] * q[i21]
            + (2.234868e-03) * q[i1] * q[i22] + (-2.528163e-02) * q[i2] * q[i3] + (-2.511238e-02) * q[i2] * q[i4] + (7.861535e-02) * q[i2] * q[i5]
            + (-9.848709e-05) * q[i2] * q[i6] + (9.317147e-05) * q[i2] * q[i7] + (-7.650917e-06) * q[i2] * q[i8] + (1.460264e-03) * q[i2] * q[i9]
            + (-1.435032e-03) * q[i2] * q[i10] + (-2.575455e-03) * q[i2] * q[i11] + (-2.486851e-03) * q[i2] * q[i12] + (5.981859e-03) * q[i2] * q[i15]
            + (-6.084513e-03) * q[i2] * q[i16] + (-2.725771e-03) * q[i2] * q[i19] + (2.709918e-03) * q[i2] * q[i20] + (1.988143e-03) * q[i2] * q[i21]
            + (1.965185e-03) * q[i2] * q[i22] + (-4.914697e-03) * q[i3] * q[i4] + (5.148517e-04) * q[i3] * q[i5] + (-1.325262e-02) * q[i3] * q[i6]
            + (1.496705e-02) * q[i3] * q[i7] + (2.286157e-03) * q[i3] * q[i8] + (-6.041502e-03) * q[i3] * q[i9] + (5.639904e-03) * q[i3] * q[i10]
            + (-4.951514e-03) * q[i3] * q[i11] + (-2.852589e-03) * q[i3] * q[i12] + (1.701516e-04) * q[i3] * q[i15] + (1.983357e-03) * q[i3] * q[i16]
            + (-3.041974e-03) * q[i3] * q[i19] + (-1.632224e-03) * q[i3] * q[i20] + (-2.054358e-03) * q[i3] * q[i21] + (1.763675e-03) * q[i3] * q[i22]
            + (4.679285e-04) * q[i4] * q[i5] + (-1.500024e-02) * q[i4] * q[i6] + (1.316982e-02) * q[i4] * q[i7] + (-2.209310e-03) * q[i4] * q[i8]
            + (-5.699578e-03) * q[i4] * q[i9] + (5.987383e-03) * q[i4] * q[i10] + (-2.811232e-03) * q[i4] * q[i11] + (-4.944247e-03) * q[i4] * q[i12]
            + (-1.936538e-03) * q[i4] * q[i15] + (-1.070322e-04) * q[i4] * q[i16] + (1.584654e-03) * q[i4] * q[i19] + (3.021591e-03) * q[i4] * q[i20]
            + (1.778881e-03) * q[i4] * q[i21] + (-2.025126e-03) * q[i4] * q[i22] + (-1.586590e-02) * q[i5] * q[i6] + (1.586884e-02) * q[i5] * q[i7]
            + (-4.203808e-05) * q[i5] * q[i8] + (-5.528472e-03) * q[i5] * q[i9] + (5.516264e-03) * q[i5] * q[i10] + (-4.741971e-03) * q[i5] * q[i11]
            + (-4.693143e-03) * q[i5] * q[i12] + (1.072417e-03) * q[i5] * q[i15] + (-1.156109e-03) * q[i5] * q[i16] + (-6.688310e-04) * q[i5] * q[i19]
            + (6.817503e-04) * q[i5] * q[i20] + (1.007175e-03) * q[i5] * q[i21] + (1.007538e-03) * q[i5] * q[i22] + (-3.116376e-03) * q[i6] * q[i7]
            + (9.608001e-04) * q[i6] * q[i8] + (-7.158658e-03) * q[i6] * q[i9] + (5.731393e-03) * q[i6] * q[i10] + (2.074065e-03) * q[i6] * q[i11]
            + (9.387117e-04) * q[i6] * q[i12] + (2.506067e-03) * q[i6] * q[i15] + (6.238223e-03) * q[i6] * q[i16] + (-5.073938e-04) * q[i6] * q[i19]
            + (-9.342910e-04) * q[i6] * q[i20] + (-2.525261e-04) * q[i6] * q[i21] + (3.042374e-04) * q[i6] * q[i22] + (9.322740e-04) * q[i7] * q[i8]
            + (5.781920e-03) * q[i7] * q[i9] + (-7.079973e-03) * q[i7] * q[i10] + (-9.094966e-04) * q[i7] * q[i11] + (-2.033062e-03) * q[i7] * q[i12]
            + (6.173416e-03) * q[i7] * q[i15] + (2.507167e-03) * q[i7] * q[i16] + (-9.229792e-04) * q[i7] * q[i19] + (-4.789234e-04) * q[i7] * q[i20]
            + (-3.041800e-04) * q[i7] * q[i21] + (2.464662e-04) * q[i7] * q[i22] + (2.419856e-03) * q[i8] * q[i9] + (2.390816e-03) * q[i8] * q[i10]
            + (-2.250525e-03) * q[i8] * q[i11] + (2.137495e-03) * q[i8] * q[i12] + (-9.332737e-03) * q[i8] * q[i15] + (-9.383865e-03) * q[i8] * q[i16]
            + (-2.190352e-03) * q[i8] * q[i19] + (-2.185457e-03) * q[i8] * q[i20] + (-2.085760e-03) * q[i8] * q[i21] + (2.088335e-03) * q[i8] * q[i22]
            + (4.848303e-03) * q[i9] * q[i10] + (-2.832155e-04) * q[i9] * q[i11] + (3.941600e-04) * q[i9] * q[i12] + (1.616846e-03) * q[i9] * q[i15]
            + (2.078583e-03) * q[i9] * q[i16] + (7.876256e-05) * q[i9] * q[i19] + (1.164829e-03) * q[i9] * q[i20] + (-7.003847e-04) * q[i9] * q[i21]
            + (4.554292e-04) * q[i9] * q[i22] + (-3.697813e-04) * q[i10] * q[i11] + (3.171618e-04) * q[i10] * q[i12] + (2.032891e-03) * q[i10] * q[i15]
            + (1.586893e-03) * q[i10] * q[i16] + (1.160841e-03) * q[i10] * q[i19] + (6.927503e-05) * q[i10] * q[i20] + (-4.558118e-04) * q[i10] * q[i21]
            + (6.953218e-04) * q[i10] * q[i22] + (-1.794156e-04) * q[i11] * q[i12] + (8.438747e-04) * q[i11] * q[i15] + (7.972360e-04) * q[i11] * q[i16]
            + (-9.842817e-04) * q[i11] * q[i19] + (-7.222546e-04) * q[i11] * q[i20] + (-1.187627e-03) * q[i11] * q[i21] + (1.108089e-05) * q[i11] * q[i22]
            + (-7.939226e-04) * q[i12] * q[i15] + (-9.109884e-04) * q[i12] * q[i16] + (7.347940e-04) * q[i12] * q[i19] + (9.576683e-04) * q[i12] * q[i20]
            + (3.132506e-05) * q[i12] * q[i21] + (-1.201508e-03) * q[i12] * q[i22] + (-9.050529e-04) * q[i15] * q[i16] + (6.894907e-04) * q[i15] * q[i19]
            + (-7.798828e-04) * q[i15] * q[i20] + (6.160006e-04) * q[i15] * q[i21] + (-3.030791e-04) * q[i15] * q[i22] + (-8.045054e-04) * q[i16] * q[i19]
            + (6.796655e-04) * q[i16] * q[i20] + (2.968374e-04) * q[i16] * q[i21] + (-6.132847e-04) * q[i16] * q[i22] + (5.521720e-04) * q[i19] * q[i20]
            + (1.012404e-03) * q[i19] * q[i21] + (-1.535273e-04) * q[i19] * q[i22] + (1.546839e-04) * q[i20] * q[i21] + (-1.037990e-03) * q[i20] * q[i22]
            + (2.020928e-04) * q[i21] * q[i22] + (2.699103e-03) * q[i0] * q[i0] * q[i0] + (4.217759e-04) * q[i0] * q[i0] * q[i1]
            + (2.148421e-03) * q[i0] * q[i0] * q[i2] + (-2.877567e-03) * q[i0] * q[i0] * q[i3] + (-1.957202e-03) * q[i0] * q[i0] * q[i4]
            + (-4.799782e-03) * q[i0] * q[i0] * q[i5] + (-2.172771e-02) * q[i0] * q[i0] * q[i6] + (-2.006883e-03) * q[i0] * q[i0] * q[i7]
            + (1.524819e-03) * q[i0] * q[i0] * q[i8] + (-3.620341e-03) * q[i0] * q[i0] * q[i9] + (-4.255330e-04) * q[i0] * q[i0] * q[i10]
            + (1.261540e-03) * q[i0] * q[i0] * q[i11] + (-2.336911e-04) * q[i0] * q[i0] * q[i12] + (7.602174e-04) * q[i0] * q[i0] * q[i15]
            + (-1.045320e-04) * q[i0] * q[i0] * q[i16] + (8.855494e-05) * q[i0] * q[i0] * q[i19] + (7.927038e-04) * q[i0] * q[i0] * q[i20]
            + (1.487935e-04) * q[i0] * q[i0] * q[i21] + (2.358285e-04) * q[i0] * q[i0] * q[i22] + (-3.711844e-04) * q[i0] * q[i1] * q[i1]
            + (-2.713918e-03) * q[i1] * q[i1] * q[i1] + (-2.176803e-03) * q[i1] * q[i1] * q[i2] + (1.966242e-03) * q[i1] * q[i1] * q[i3]
            + (2.900782e-03) * q[i1] * q[i1] * q[i4] + (4.795869e-03) * q[i1] * q[i1] * q[i5] + (-2.025674e-03) * q[i1] * q[i1] * q[i6]
            + (-2.168669e-02) * q[i1] * q[i1] * q[i7] + (1.522468e-03) * q[i1] * q[i1] * q[i8] + (-4.279621e-04) * q[i1] * q[i1] * q[i9]
            + (-3.573820e-03) * q[i1] * q[i1] * q[i10] + (2.313198e-04) * q[i1] * q[i1] * q[i11] + (-1.247452e-03) * q[i1] * q[i1] * q[i12]
            + (-1.123010e-04) * q[i1] * q[i1] * q[i15] + (7.653473e-04) * q[i1] * q[i1] * q[i16] + (7.919462e-04) * q[i1] * q[i1] * q[i19]
            + (9.471634e-05) * q[i1] * q[i1] * q[i20] + (-2.343892e-04) * q[i1] * q[i1] * q[i21] + (-1.491548e-04) * q[i1] * q[i1] * q[i22]
            + (1.270791e-03) * q[i0] * q[i2] * q[i2] + (-1.279295e-03) * q[i1] * q[i2] * q[i2] + (-9.223137e-06) * q[i2] * q[i2] * q[i2]
            + (1.510165e-04) * q[i2] * q[i2] * q[i3] + (-1.471769e-04) * q[i2] * q[i2] * q[i4] + (-1.566487e-05) * q[i2] * q[i2] * q[i5]
            + (2.143538e-05) * q[i2] * q[i2] * q[i6] + (2.407642e-05) * q[i2] * q[i2] * q[i7] + (-3.070144e-02) * q[i2] * q[i2] * q[i8]
            + (-9.801745e-04) * q[i2] * q[i2] * q[i9] + (-9.776732e-04) * q[i2] * q[i2] * q[i10] + (8.314828e-05) * q[i2] * q[i2] * q[i11]
            + (-1.286932e-04) * q[i2] * q[i2] * q[i12] + (-2.236801e-04) * q[i2] * q[i2] * q[i15] + (-2.076048e-04) * q[i2] * q[i2] * q[i16]
            + (-6.277662e-04) * q[i2] * q[i2] * q[i19] + (-6.472335e-04) * q[i2] * q[i2] * q[i20] + (7.020029e-04) * q[i2] * q[i2] * q[i21]
            + (-7.202685e-04) * q[i2] * q[i2] * q[i22] + (7.799925e-03) * q[i0] * q[i3] * q[i3] + (-2.959506e-03) * q[i1] * q[i3] * q[i3]
            + (1.746452e-03) * q[i2] * q[i3] * q[i3] + (9.593527e-04) * q[i3] * q[i3] * q[i3] + (1.781395e-03) * q[i3] * q[i3] * q[i4]
            + (6.591798e-04) * q[i3] * q[i3] * q[i5] + (1.200284e-02) * q[i3] * q[i3] * q[i6] + (-1.176808e-03) * q[i3] * q[i3] * q[i7]
            + (3.685066e-03) * q[i3] * q[i3] * q[i8] + (2.280216e-03) * q[i3] * q[i3] * q[i9] + (-9.699651e-04) * q[i3] * q[i3] * q[i10]
            + (2.476817e-04) * q[i3] * q[i3] * q[i11] + (7.545558e-04) * q[i3] * q[i3] * q[i12] + (3.150266e-04) * q[i3] * q[i3] * q[i15]
            + (1.793124e-04) * q[i3] * q[i3] * q[i16] + (1.215654e-03) * q[i3] * q[i3] * q[i19] + (-4.721584e-04) * q[i3] * q[i3] * q[i20]
            + (-7.272412e-04) * q[i3] * q[i3] * q[i21] + (-6.555301e-04) * q[i3] * q[i3] * q[i22] + (2.965714e-03) * q[i0] * q[i4] * q[i4]
            + (-7.743154e-03) * q[i1] * q[i4] * q[i4] + (-1.715255e-03) * q[i2] * q[i4] * q[i4] + (-1.787267e-03) * q[i3] * q[i4] * q[i4]
            + (-9.534190e-04) * q[i4] * q[i4] * q[i4] + (-6.716760e-04) * q[i4] * q[i4] * q[i5] + (-1.198562e-03) * q[i4] * q[i4] * q[i6]
            + (1.202639e-02) * q[i4] * q[i4] * q[i7] + (3.701925e-03) * q[i4] * q[i4] * q[i8] + (-9.772404e-04) * q[i4] * q[i4] * q[i9]
            + (2.273967e-03) * q[i4] * q[i4] * q[i10] + (-7.619221e-04) * q[i4] * q[i4] * q[i11] + (-2.208289e-04) * q[i4] * q[i4] * q[i12]
            + (1.659287e-04) * q[i4] * q[i4] * q[i15] + (2.896900e-04) * q[i4] * q[i4] * q[i16] + (-4.610605e-04) * q[i4] * q[i4] * q[i19]
            + (1.217230e-03) * q[i4] * q[i4] * q[i20] + (6.508448e-04) * q[i4] * q[i4] * q[i21] + (7.148945e-04) * q[i4] * q[i4] * q[i22]
            + (-3.637586e-04) * q[i0] * q[i5] * q[i5] + (3.673930e-04) * q[i1] * q[i5] * q[i5] + (-3.329987e-05) * q[i2] * q[i5] * q[i5]
            + (-1.147605e-03) * q[i3] * q[i5] * q[i5] + (1.138473e-03) * q[i4] * q[i5] * q[i5] + (1.971547e-05) * q[i5] * q[i5] * q[i5]
            + (1.523657e-03) * q[i5] * q[i5] * q[i6] + (1.525435e-03) * q[i5] * q[i5] * q[i7] + (2.662022e-03) * q[i5] * q[i5] * q[i8]
            + (-2.407482e-04) * q[i5] * q[i5] * q[i9] + (-2.454196e-04) * q[i5] * q[i5] * q[i10] + (6.884126e-05) * q[i5] * q[i5] * q[i11]
            + (-9.857491e-05) * q[i5] * q[i5] * q[i12] + (-2.111881e-03) * q[i5] * q[i5] * q[i15] + (-2.132539e-03) * q[i5] * q[i5] * q[i16]
            + (-1.718998e-03) * q[i5] * q[i5] * q[i19] + (-1.704766e-03) * q[i5] * q[i5] * q[i20] + (-1.473637e-03) * q[i5] * q[i5] * q[i21]
            + (1.495075e-03) * q[i5] * q[i5] * q[i22] + (-4.331391e-03) * q[i0] * q[i6] * q[i6] + (-1.108751e-03) * q[i1] * q[i6] * q[i6]
            + (-3.326144e-04) * q[i2] * q[i6] * q[i6] + (-6.037617e-04) * q[i3] * q[i6] * q[i6] + (8.606353e-04) * q[i4] * q[i6] * q[i6]
            + (-3.680953e-03) * q[i5] * q[i6] * q[i6] + (-2.277140e-03) * q[i6] * q[i6] * q[i6] + (-8.911033e-04) * q[i6] * q[i6] * q[i7]
            + (3.097585e-03) * q[i6] * q[i6] * q[i8] + (-6.738246e-04) * q[i6] * q[i6] * q[i9] + (-4.169451e-04) * q[i6] * q[i6] * q[i10]
            + (-2.863740e-04) * q[i6] * q[i6] * q[i11] + (3.608378e-04) * q[i6] * q[i6] * q[i12] + (-5.963555e-04) * q[i6] * q[i6] * q[i15]
            + (-4.876350e-04) * q[i6] * q[i6] * q[i16] + (-1.054806e-03) * q[i6] * q[i6] * q[i19] + (-5.433964e-05) * q[i6] * q[i6] * q[i20]
            + (-6.000541e-04) * q[i6] * q[i6] * q[i21] + (1.043639e-03) * q[i6] * q[i6] * q[i22] + (1.095570e-03) * q[i0] * q[i7] * q[i7]
            + (4.318104e-03) * q[i1] * q[i7] * q[i7] + (3.392017e-04) * q[i2] * q[i7] * q[i7] + (-8.564796e-04) * q[i3] * q[i7] * q[i7]
            + (6.288183e-04) * q[i4] * q[i7] * q[i7] + (3.688227e-03) * q[i5] * q[i7] * q[i7] + (-8.986901e-04) * q[i6] * q[i7] * q[i7]
            + (-2.277373e-03) * q[i7] * q[i7] * q[i7] + (3.125767e-03) * q[i7] * q[i7] * q[i8] + (-4.165958e-04) * q[i7] * q[i7] * q[i9]
            + (-6.674327e-04) * q[i7] * q[i7] * q[i10] + (-3.531736e-04) * q[i7] * q[i7] * q[i11] + (3.045578e-04) * q[i7] * q[i7] * q[i12]
            + (-4.750761e-04) * q[i7] * q[i7] * q[i15] + (-5.980053e-04) * q[i7] * q[i7] * q[i16] + (-4.924212e-05) * q[i7] * q[i7] * q[i19]
            + (-1.049889e-03) * q[i7] * q[i7] * q[i20] + (-1.035763e-03) * q[i7] * q[i7] * q[i21] + (5.882127e-04) * q[i7] * q[i7] * q[i22]
            + (-1.652844e-03) * q[i0] * q[i8] * q[i8] + (1.667796e-03) * q[i1] * q[i8] * q[i8] + (2.976896e-05) * q[i2] * q[i8] * q[i8]
            + (-5.313342e-04) * q[i3] * q[i8] * q[i8] + (5.051829e-04) * q[i4] * q[i8] * q[i8] + (-1.978580e-07) * q[i5] * q[i8] * q[i8]
            + (1.549358e-03) * q[i6] * q[i8] * q[i8] + (1.578614e-03) * q[i7] * q[i8] * q[i8] + (-3.481256e-03) * q[i8] * q[i8] * q[i8]
            + (8.673059e-04) * q[i8] * q[i8] * q[i9] + (8.654743e-04) * q[i8] * q[i8] * q[i10] + (7.407764e-04) * q[i8] * q[i8] * q[i11]
            + (-7.625813e-04) * q[i8] * q[i8] * q[i12] + (4.076422e-05) * q[i8] * q[i8] * q[i15] + (5.069054e-05) * q[i8] * q[i8] * q[i16]
            + (-1.430933e-05) * q[i8] * q[i8] * q[i19] + (-2.542223e-05) * q[i8] * q[i8] * q[i20] + (-5.663762e-04) * q[i8] * q[i8] * q[i21]
            + (5.723610e-04) * q[i8] * q[i8] * q[i22] + (1.141431e-04) * q[i0] * q[i9] * q[i9] + (-5.281776e-04) * q[i1] * q[i9] * q[i9]
            + (7.392941e-04) * q[i2] * q[i9] * q[i9] + (1.189732e-03) * q[i3] * q[i9] * q[i9] + (1.167596e-03) * q[i4] * q[i9] * q[i9]
            + (8.578853e-04) * q[i5] * q[i9] * q[i9] + (-3.082180e-03) * q[i6] * q[i9] * q[i9] + (-9.292043e-05) * q[i7] * q[i9] * q[i9]
            + (1.081172e-03) * q[i8] * q[i9] * q[i9] + (-1.031580e-03) * q[i9] * q[i9] * q[i9] + (-3.716021e-04) * q[i9] * q[i9] * q[i10]
            + (1.059143e-04) * q[i9] * q[i9] * q[i11] + (1.338060e-04) * q[i9] * q[i9] * q[i12] + (-2.008965e-04) * q[i9] * q[i9] * q[i15]
            + (-2.215306e-04) * q[i9] * q[i9] * q[i16] + (1.068099e-04) * q[i9] * q[i9] * q[i19] + (-2.559619e-04) * q[i9] * q[i9] * q[i20]
            + (5.934105e-06) * q[i9] * q[i9] * q[i21] + (-2.138896e-04) * q[i9] * q[i9] * q[i22] + (5.209430e-04) * q[i0] * q[i10] * q[i10]
            + (-1.151267e-04) * q[i1] * q[i10] * q[i10] + (-7.362072e-04) * q[i2] * q[i10] * q[i10] + (-1.155973e-03) * q[i3] * q[i10] * q[i10]
            + (-1.170908e-03) * q[i4] * q[i10] * q[i10] + (-8.626645e-04) * q[i5] * q[i10] * q[i10] + (-1.009958e-04) * q[i6] * q[i10] * q[i10]
            + (-3.045815e-03) * q[i7] * q[i10] * q[i10] + (1.069369e-03) * q[i8] * q[i10] * q[i10] + (-3.751170e-04) * q[i9] * q[i10] * q[i10]
            + (-1.017551e-03) * q[i10] * q[i10] * q[i10] + (-1.380678e-04) * q[i10] * q[i10] * q[i11] + (-1.062532e-04) * q[i10] * q[i10] * q[i12]
            + (-2.173750e-04) * q[i10] * q[i10] * q[i15] + (-1.998722e-04) * q[i10] * q[i10] * q[i16] + (-2.521337e-04) * q[i10] * q[i10] * q[i19]
            + (1.029168e-04) * q[i10] * q[i10] * q[i20] + (2.119792e-04) * q[i10] * q[i10] * q[i21] + (-6.543407e-06) * q[i10] * q[i10] * q[i22]
            + (2.709956e-04) * q[i0] * q[i11] * q[i11] + (-4.310596e-04) * q[i1] * q[i11] * q[i11] + (1.572873e-03) * q[i2] * q[i11] * q[i11]
            + (1.445009e-03) * q[i3] * q[i11] * q[i11] + (-6.828332e-04) * q[i4] * q[i11] * q[i11] + (3.701298e-04) * q[i5] * q[i11] * q[i11]
            + (-8.192230e-04) * q[i6] * q[i11] * q[i11] + (5.161338e-06) * q[i7] * q[i11] * q[i11] + (1.676774e-03) * q[i8] * q[i11] * q[i11]
            + (-1.130603e-04) * q[i9] * q[i11] * q[i11] + (5.397174e-05) * q[i10] * q[i11] * q[i11] + (8.726313e-04) * q[i11] * q[i11] * q[i11]
            + (-5.712128e-04) * q[i11] * q[i11] * q[i12] + (-2.055705e-03) * q[i11] * q[i11] * q[i15] + (-1.399818e-04) * q[i11] * q[i11] * q[i16]
            + (6.309429e-04) * q[i11] * q[i11] * q[i19] + (-2.318441e-05) * q[i11] * q[i11] * q[i20] + (1.754330e-04) * q[i11] * q[i11] * q[i21]
            + (1.907818e-04) * q[i11] * q[i11] * q[i22] + (4.285072e-04) * q[i0] * q[i12] * q[i12] + (-2.801488e-04) * q[i1] * q[i12] * q[i12]
            + (-1.692362e-03) * q[i2] * q[i12] * q[i12] + (6.728427e-04) * q[i3] * q[i12] * q[i12] + (-1.457032e-03) * q[i4] * q[i12] * q[i12]
            + (-4.291887e-04) * q[i5] * q[i12] * q[i12] + (1.147318e-05) * q[i6] * q[i12] * q[i12] + (-8.323333e-04) * q[i7] * q[i12] * q[i12]
            + (1.706114e-03) * q[i8] * q[i12] * q[i12] + (6.798928e-05) * q[i9] * q[i12] * q[i12] + (-1.124623e-04) * q[i10] * q[i12] * q[i12]
            + (5.751741e-04) * q[i11] * q[i12] * q[i12] + (-9.228392e-04) * q[i12] * q[i12] * q[i12] + (-1.452196e-04) * q[i12] * q[i12] * q[i15]
            + (-2.048617e-03) * q[i12] * q[i12] * q[i16] + (-1.360159e-05) * q[i12] * q[i12] * q[i19] + (6.134458e-04) * q[i12] * q[i12] * q[i20]
            + (-1.951116e-04) * q[i12] * q[i12] * q[i21] + (-1.700646e-04) * q[i12] * q[i12] * q[i22] + (4.538791e-05) * q[i0] * q[i15] * q[i15]
            + (7.123550e-04) * q[i1] * q[i15] * q[i15] + (2.563287e-03) * q[i2] * q[i15] * q[i15] + (-2.638679e-05) * q[i3] * q[i15] * q[i15]
            + (-4.245545e-04) * q[i4] * q[i15] * q[i15] + (-5.541777e-04) * q[i5] * q[i15] * q[i15] + (4.019567e-05) * q[i6] * q[i15] * q[i15]
            + (8.646733e-04) * q[i7] * q[i15] * q[i15] + (6.044488e-05) * q[i8] * q[i15] * q[i15] + (-9.121269e-06) * q[i9] * q[i15] * q[i15]
            + (-1.647609e-04) * q[i10] * q[i15] * q[i15] + (2.137627e-03) * q[i11] * q[i15] * q[i15] + (2.182402e-04) * q[i12] * q[i15] * q[i15]
            + (5.182965e-05) * q[i15] * q[i15] * q[i15] + (-3.212224e-04) * q[i15] * q[i15] * q[i16] + (1.617819e-04) * q[i15] * q[i15] * q[i19]
            + (-3.049154e-04) * q[i15] * q[i15] * q[i20] + (-1.976457e-04) * q[i15] * q[i15] * q[i21] + (9.307695e-07) * q[i15] * q[i15] * q[i22]
            + (-7.146619e-04) * q[i0] * q[i16] * q[i16] + (-4.658708e-05) * q[i1] * q[i16] * q[i16] + (-2.586976e-03) * q[i2] * q[i16] * q[i16]
            + (4.245840e-04) * q[i3] * q[i16] * q[i16] + (4.041991e-05) * q[i4] * q[i16] * q[i16] + (5.385013e-04) * q[i5] * q[i16] * q[i16]
            + (8.658253e-04) * q[i6] * q[i16] * q[i16] + (3.910849e-05) * q[i7] * q[i16] * q[i16] + (7.343281e-05) * q[i8] * q[i16] * q[i16]
            + (-1.646033e-04) * q[i9] * q[i16] * q[i16] + (-1.332429e-05) * q[i10] * q[i16] * q[i16] + (-2.187742e-04) * q[i11] * q[i16] * q[i16]
            + (-2.170813e-03) * q[i12] * q[i16] * q[i16] + (-3.227798e-04) * q[i15] * q[i16] * q[i16] + (5.263756e-05) * q[i16] * q[i16] * q[i16]
            + (-3.098556e-04) * q[i16] * q[i16] * q[i19] + (1.564144e-04) * q[i16] * q[i16] * q[i20] + (-3.724932e-06) * q[i16] * q[i16] * q[i21]
            + (2.048794e-04) * q[i16] * q[i16] * q[i22] + (-1.345468e-04) * q[i0] * q[i19] * q[i19] + (-2.431014e-04) * q[i1] * q[i19] * q[i19]
            + (3.508930e-06) * q[i2] * q[i19] * q[i19] + (-3.874298e-04) * q[i3] * q[i19] * q[i19] + (1.838183e-04) * q[i4] * q[i19] * q[i19]
            + (-8.537189e-04) * q[i5] * q[i19] * q[i19] + (-6.359441e-04) * q[i6] * q[i19] * q[i19] + (3.946263e-04) * q[i7] * q[i19] * q[i19]
            + (-4.900689e-04) * q[i8] * q[i19] * q[i19] + (-1.551092e-04) * q[i9] * q[i19] * q[i19] + (-1.400187e-04) * q[i10] * q[i19] * q[i19]
            + (-3.553330e-04) * q[i11] * q[i19] * q[i19] + (1.844954e-04) * q[i12] * q[i19] * q[i19] + (-1.633945e-04) * q[i15] * q[i19] * q[i19]
            + (-4.874503e-05) * q[i16] * q[i19] * q[i19] + (2.056915e-04) * q[i19] * q[i19] * q[i19] + (5.381839e-05) * q[i19] * q[i19] * q[i20]
            + (-3.067863e-04) * q[i19] * q[i19] * q[i21] + (4.524680e-05) * q[i19] * q[i19] * q[i22] + (2.372272e-04) * q[i0] * q[i20] * q[i20]
            + (1.360576e-04) * q[i1] * q[i20] * q[i20] + (-2.837281e-06) * q[i2] * q[i20] * q[i20] + (-1.862724e-04) * q[i3] * q[i20] * q[i20]
            + (3.691705e-04) * q[i4] * q[i20] * q[i20] + (8.604417e-04) * q[i5] * q[i20] * q[i20] + (3.957655e-04) * q[i6] * q[i20] * q[i20]
            + (-6.289636e-04) * q[i7] * q[i20] * q[i20] + (-4.881255e-04) * q[i8] * q[i20] * q[i20] + (-1.402449e-04) * q[i9] * q[i20] * q[i20]
            + (-1.564151e-04) * q[i10] * q[i20] * q[i20] + (-1.814366e-04) * q[i11] * q[i20] * q[i20] + (3.580127e-04) * q[i12] * q[i20] * q[i20]
            + (-5.054305e-05) * q[i15] * q[i20] * q[i20] + (-1.522340e-04) * q[i16] * q[i20] * q[i20] + (5.310574e-05) * q[i19] * q[i20] * q[i20]
            + (2.032525e-04) * q[i20] * q[i20] * q[i20] + (-4.372817e-05) * q[i20] * q[i20] * q[i21] + (3.035206e-04) * q[i20] * q[i20] * q[i22]
            + (1.887070e-04) * q[i0] * q[i21] * q[i21] + (-6.073279e-05) * q[i1] * q[i21] * q[i21] + (3.209924e-04) * q[i2] * q[i21] * q[i21]
            + (-2.119539e-04) * q[i3] * q[i21] * q[i21] + (5.086029e-04) * q[i4] * q[i21] * q[i21] + (-4.984050e-04) * q[i5] * q[i21] * q[i21]
            + (1.102790e-04) * q[i6] * q[i21] * q[i21] + (6.938603e-04) * q[i7] * q[i21] * q[i21] + (-7.177874e-04) * q[i8] * q[i21] * q[i21]
            + (-3.988740e-05) * q[i9] * q[i21] * q[i21] + (1.020322e-04) * q[i10] * q[i21] * q[i21] + (-4.051638e-04) * q[i11] * q[i21] * q[i21]
            + (-2.526193e-05) * q[i12] * q[i21] * q[i21] + (-1.613204e-04) * q[i15] * q[i21] * q[i21] + (1.226978e-04) * q[i16] * q[i21] * q[i21]
            + (5.306266e-04) * q[i19] * q[i21] * q[i21] + (-1.304610e-04) * q[i20] * q[i21] * q[i21] + (-8.453739e-05) * q[i21] * q[i21] * q[i21]
            + (5.480708e-05) * q[i21] * q[i21] * q[i22] + (4.910673e-05) * q[i0] * q[i22] * q[i22] + (-1.891935e-04) * q[i1] * q[i22] * q[i22]
            + (-3.183774e-04) * q[i2] * q[i22] * q[i22] + (-5.071852e-04) * q[i3] * q[i22] * q[i22] + (2.075443e-04) * q[i4] * q[i22] * q[i22]
            + (5.071164e-04) * q[i5] * q[i22] * q[i22] + (6.934395e-04) * q[i6] * q[i22] * q[i22] + (1.140094e-04) * q[i7] * q[i22] * q[i22]
            + (-7.183218e-04) * q[i8] * q[i22] * q[i22] + (1.021671e-04) * q[i9] * q[i22] * q[i22] + (-4.066959e-05) * q[i10] * q[i22] * q[i22]
            + (2.422408e-05) * q[i11] * q[i22] * q[i22] + (4.178399e-04) * q[i12] * q[i22] * q[i22] + (1.252246e-04) * q[i15] * q[i22] * q[i22]
            + (-1.634418e-04) * q[i16] * q[i22] * q[i22] + (-1.313933e-04) * q[i19] * q[i22] * q[i22] + (5.409582e-04) * q[i20] * q[i22] * q[i22]
            + (-5.651128e-05) * q[i21] * q[i22] * q[i22] + (8.735746e-05) * q[i22] * q[i22] * q[i22] + (2.041900e-05) * q[i0] * q[i1] * q[i2]
            + (-2.203863e-03) * q[i0] * q[i1] * q[i3] + (2.176018e-03) * q[i0] * q[i1] * q[i4] + (-3.023284e-06) * q[i0] * q[i1] * q[i5]
            + (4.607573e-03) * q[i0] * q[i1] * q[i6] + (4.585382e-03) * q[i0] * q[i1] * q[i7] + (3.606869e-03) * q[i0] * q[i1] * q[i8]
            + (3.022323e-04) * q[i0] * q[i1] * q[i9] + (2.846155e-04) * q[i0] * q[i1] * q[i10] + (-7.803878e-04) * q[i0] * q[i1] * q[i11]
            + (7.788289e-04) * q[i0] * q[i1] * q[i12] + (-5.317338e-04) * q[i0] * q[i1] * q[i15] + (-5.364147e-04) * q[i0] * q[i1] * q[i16]
            + (-2.149220e-03) * q[i0] * q[i1] * q[i19] + (-2.164970e-03) * q[i0] * q[i1] * q[i20] + (1.059114e-03) * q[i0] * q[i1] * q[i21]
            + (-1.058629e-03) * q[i0] * q[i1] * q[i22] + (-4.830283e-03) * q[i0] * q[i2] * q[i3] + (-3.669630e-04) * q[i0] * q[i2] * q[i4]
            + (-1.346757e-03) * q[i0] * q[i2] * q[i5] + (-8.850569e-03) * q[i0] * q[i2] * q[i6] + (1.509462e-03) * q[i0] * q[i2] * q[i7]
            + (-2.242384e-03) * q[i0] * q[i2] * q[i8] + (-1.992049e-03) * q[i0] * q[i2] * q[i9] + (2.438050e-03) * q[i0] * q[i2] * q[i10]
            + (6.523406e-04) * q[i0] * q[i2] * q[i11] + (8.603355e-04) * q[i0] * q[i2] * q[i12] + (9.454998e-05) * q[i0] * q[i2] * q[i15]
            + (1.685530e-04) * q[i0] * q[i2] * q[i16] + (-5.308325e-04) * q[i0] * q[i2] * q[i19] + (-7.462804e-04) * q[i0] * q[i2] * q[i20]
            + (1.166162e-03) * q[i0] * q[i2] * q[i21] + (-1.663098e-03) * q[i0] * q[i2] * q[i22] + (1.577433e-02) * q[i0] * q[i3] * q[i4]
            + (8.064864e-03) * q[i0] * q[i3] * q[i5] + (-1.744554e-02) * q[i0] * q[i3] * q[i6] + (7.487355e-04) * q[i0] * q[i3] * q[i7]
            + (-6.385192e-04) * q[i0] * q[i3] * q[i8] + (1.275901e-02) * q[i0] * q[i3] * q[i9] + (-2.628810e-03) * q[i0] * q[i3] * q[i10]
            + (3.415507e-04) * q[i0] * q[i3] * q[i11] + (-1.892927e-03) * q[i0] * q[i3] * q[i12] + (-2.736937e-04) * q[i0] * q[i3] * q[i15]
            + (-4.429769e-03) * q[i0] * q[i3] * q[i16] + (-1.283668e-03) * q[i0] * q[i3] * q[i19] + (-1.352818e-03) * q[i0] * q[i3] * q[i20]
            + (-4.281597e-05) * q[i0] * q[i3] * q[i21] + (-7.913839e-04) * q[i0] * q[i3] * q[i22] + (2.766852e-03) * q[i0] * q[i4] * q[i5]
            + (-6.211916e-04) * q[i0] * q[i4] * q[i6] + (5.575730e-03) * q[i0] * q[i4] * q[i7] + (-2.558086e-03) * q[i0] * q[i4] * q[i8]
            + (-9.015293e-04) * q[i0] * q[i4] * q[i9] + (-4.478219e-04) * q[i0] * q[i4] * q[i10] + (-2.682264e-04) * q[i0] * q[i4] * q[i11]
            + (-1.597540e-03) * q[i0] * q[i4] * q[i12] + (-3.911409e-04) * q[i0] * q[i4] * q[i15] + (2.809789e-04) * q[i0] * q[i4] * q[i16]
            + (-5.715602e-06) * q[i0] * q[i4] * q[i19] + (-6.549558e-04) * q[i0] * q[i4] * q[i20] + (-1.410528e-04) * q[i0] * q[i4] * q[i21]
            + (-6.179272e-04) * q[i0] * q[i4] * q[i22] + (1.989546e-03) * q[i0] * q[i5] * q[i6] + (1.368311e-03) * q[i0] * q[i5] * q[i7]
            + (1.635292e-04) * q[i0] * q[i5] * q[i8] + (2.272778e-03) * q[i0] * q[i5] * q[i9] + (-3.349013e-04) * q[i0] * q[i5] * q[i10]
            + (-2.742440e-03) * q[i0] * q[i5] * q[i11] + (2.213568e-03) * q[i0] * q[i5] * q[i12] + (-1.032333e-03) * q[i0] * q[i5] * q[i15]
            + (-1.635496e-03) * q[i0] * q[i5] * q[i16] + (6.490070e-04) * q[i0] * q[i5] * q[i19] + (-3.652971e-05) * q[i0] * q[i5] * q[i20]
            + (-4.205140e-04) * q[i0] * q[i5] * q[i21] + (6.642214e-04) * q[i0] * q[i5] * q[i22] + (1.017963e-02) * q[i0] * q[i6] * q[i7]
            + (1.651949e-03) * q[i0] * q[i6] * q[i8] + (-5.399697e-04) * q[i0] * q[i6] * q[i9] + (1.844408e-03) * q[i0] * q[i6] * q[i10]
            + (-1.342392e-03) * q[i0] * q[i6] * q[i11] + (-1.443070e-03) * q[i0] * q[i6] * q[i12] + (-3.174323e-04) * q[i0] * q[i6] * q[i15]
            + (1.159499e-03) * q[i0] * q[i6] * q[i16] + (-1.498094e-03) * q[i0] * q[i6] * q[i19] + (7.182522e-04) * q[i0] * q[i6] * q[i20]
            + (5.736209e-04) * q[i0] * q[i6] * q[i21] + (-8.699801e-04) * q[i0] * q[i6] * q[i22] + (2.109935e-04) * q[i0] * q[i7] * q[i8]
            + (1.644994e-03) * q[i0] * q[i7] * q[i9] + (5.517188e-04) * q[i0] * q[i7] * q[i10] + (8.710504e-05) * q[i0] * q[i7] * q[i11]
            + (2.285169e-04) * q[i0] * q[i7] * q[i12] + (7.248336e-04) * q[i0] * q[i7] * q[i15] + (1.887704e-04) * q[i0] * q[i7] * q[i16]
            + (-1.286637e-03) * q[i0] * q[i7] * q[i19] + (3.037250e-04) * q[i0] * q[i7] * q[i20] + (-4.422425e-04) * q[i0] * q[i7] * q[i21]
            + (-3.798863e-04) * q[i0] * q[i7] * q[i22] + (7.119842e-04) * q[i0] * q[i8] * q[i9] + (2.040536e-04) * q[i0] * q[i8] * q[i10]
            + (-9.079045e-04) * q[i0] * q[i8] * q[i11] + (9.702281e-04) * q[i0] * q[i8] * q[i12] + (-7.466604e-04) * q[i0] * q[i8] * q[i15]
            + (2.160150e-03) * q[i0] * q[i8] * q[i16] + (-3.694046e-04) * q[i0] * q[i8] * q[i19] + (-2.317022e-04) * q[i0] * q[i8] * q[i20]
            + (2.642182e-05) * q[i0] * q[i8] * q[i21] + (-4.572423e-04) * q[i0] * q[i8] * q[i22] + (1.360263e-03) * q[i0] * q[i9] * q[i10]
            + (4.597602e-05) * q[i0] * q[i9] * q[i11] + (-1.002433e-03) * q[i0] * q[i9] * q[i12] + (1.017340e-04) * q[i0] * q[i9] * q[i15]
            + (1.127961e-03) * q[i0] * q[i9] * q[i16] + (-1.799168e-04) * q[i0] * q[i9] * q[i19] + (5.617884e-04) * q[i0] * q[i9] * q[i20]
            + (-7.818176e-05) * q[i0] * q[i9] * q[i21] + (-6.660250e-05) * q[i0] * q[i9] * q[i22] + (6.985196e-04) * q[i0] * q[i10] * q[i11]
            + (5.734440e-04) * q[i0] * q[i10] * q[i12] + (5.303826e-04) * q[i0] * q[i10] * q[i15] + (-5.940487e-04) * q[i0] * q[i10] * q[i16]
            + (2.148813e-04) * q[i0] * q[i10] * q[i19] + (-4.319454e-04) * q[i0] * q[i10] * q[i20] + (-1.584458e-04) * q[i0] * q[i10] * q[i21]
            + (-2.132184e-04) * q[i0] * q[i10] * q[i22] + (-5.393562e-04) * q[i0] * q[i11] * q[i12] + (1.859815e-04) * q[i0] * q[i11] * q[i15]
            + (-7.142625e-05) * q[i0] * q[i11] * q[i16] + (5.389142e-04) * q[i0] * q[i11] * q[i19] + (4.192410e-04) * q[i0] * q[i11] * q[i20]
            + (2.893217e-04) * q[i0] * q[i11] * q[i21] + (3.909004e-04) * q[i0] * q[i11] * q[i22] + (-2.571111e-05) * q[i0] * q[i12] * q[i15]
            + (-5.853968e-05) * q[i0] * q[i12] * q[i16] + (-5.457165e-04) * q[i0] * q[i12] * q[i19] + (6.830868e-05) * q[i0] * q[i12] * q[i20]
            + (-5.167213e-04) * q[i0] * q[i12] * q[i21] + (-3.713889e-04) * q[i0] * q[i12] * q[i22] + (-2.180080e-04) * q[i0] * q[i15] * q[i16]
            + (-9.731131e-05) * q[i0] * q[i15] * q[i19] + (-1.329828e-04) * q[i0] * q[i15] * q[i20] + (3.266881e-04) * q[i0] * q[i15] * q[i21]
            + (1.098482e-04) * q[i0] * q[i15] * q[i22] + (-5.863536e-04) * q[i0] * q[i16] * q[i19] + (1.775150e-04) * q[i0] * q[i16] * q[i20]
            + (-2.073675e-04) * q[i0] * q[i16] * q[i21] + (1.769337e-04) * q[i0] * q[i16] * q[i22] + (3.099871e-04) * q[i0] * q[i19] * q[i20]
            + (2.827223e-04) * q[i0] * q[i19] * q[i21] + (2.580421e-04) * q[i0] * q[i19] * q[i22] + (-1.022213e-04) * q[i0] * q[i20] * q[i21]
            + (-2.220662e-05) * q[i0] * q[i20] * q[i22] + (-5.595093e-04) * q[i0] * q[i21] * q[i22] + (3.625418e-04) * q[i1] * q[i2] * q[i3]
            + (4.829084e-03) * q[i1] * q[i2] * q[i4] + (1.352036e-03) * q[i1] * q[i2] * q[i5] + (1.486211e-03) * q[i1] * q[i2] * q[i6]
            + (-8.826749e-03) * q[i1] * q[i2] * q[i7] + (-2.254740e-03) * q[i1] * q[i2] * q[i8] + (2.449361e-03) * q[i1] * q[i2] * q[i9]
            + (-1.976517e-03) * q[i1] * q[i2] * q[i10] + (-8.432124e-04) * q[i1] * q[i2] * q[i11] + (-6.395182e-04) * q[i1] * q[i2] * q[i12]
            + (1.760426e-04) * q[i1] * q[i2] * q[i15] + (1.001514e-04) * q[i1] * q[i2] * q[i16] + (-7.448692e-04) * q[i1] * q[i2] * q[i19]
            + (-5.387324e-04) * q[i1] * q[i2] * q[i20] + (1.664055e-03) * q[i1] * q[i2] * q[i21] + (-1.179484e-03) * q[i1] * q[i2] * q[i22]
            + (-1.580378e-02) * q[i1] * q[i3] * q[i4] + (-2.793112e-03) * q[i1] * q[i3] * q[i5] + (5.600800e-03) * q[i1] * q[i3] * q[i6]
            + (-6.311696e-04) * q[i1] * q[i3] * q[i7] + (-2.588719e-03) * q[i1] * q[i3] * q[i8] + (-4.320600e-04) * q[i1] * q[i3] * q[i9]
            + (-8.867219e-04) * q[i1] * q[i3] * q[i10] + (1.609103e-03) * q[i1] * q[i3] * q[i11] + (2.617932e-04) * q[i1] * q[i3] * q[i12]
            + (2.837263e-04) * q[i1] * q[i3] * q[i15] + (-3.677597e-04) * q[i1] * q[i3] * q[i16] + (-6.588636e-04) * q[i1] * q[i3] * q[i19]
            + (1.403780e-05) * q[i1] * q[i3] * q[i20] + (6.214166e-04) * q[i1] * q[i3] * q[i21] + (1.400097e-04) * q[i1] * q[i3] * q[i22]
            + (-8.075424e-03) * q[i1] * q[i4] * q[i5] + (7.475188e-04) * q[i1] * q[i4] * q[i6] + (-1.749236e-02) * q[i1] * q[i4] * q[i7]
            + (-6.287449e-04) * q[i1] * q[i4] * q[i8] + (-2.675081e-03) * q[i1] * q[i4] * q[i9] + (1.261796e-02) * q[i1] * q[i4] * q[i10]
            + (1.852015e-03) * q[i1] * q[i4] * q[i11] + (-3.495028e-04) * q[i1] * q[i4] * q[i12] + (-4.364390e-03) * q[i1] * q[i4] * q[i15]
            + (-2.788492e-04) * q[i1] * q[i4] * q[i16] + (-1.337410e-03) * q[i1] * q[i4] * q[i19] + (-1.289726e-03) * q[i1] * q[i4] * q[i20]
            + (7.876661e-04) * q[i1] * q[i4] * q[i21] + (4.832829e-05) * q[i1] * q[i4] * q[i22] + (1.352922e-03) * q[i1] * q[i5] * q[i6]
            + (1.951753e-03) * q[i1] * q[i5] * q[i7] + (1.637739e-04) * q[i1] * q[i5] * q[i8] + (-3.407700e-04) * q[i1] * q[i5] * q[i9]
            + (2.248592e-03) * q[i1] * q[i5] * q[i10] + (-2.247199e-03) * q[i1] * q[i5] * q[i11] + (2.677469e-03) * q[i1] * q[i5] * q[i12]
            + (-1.592579e-03) * q[i1] * q[i5] * q[i15] + (-1.026977e-03) * q[i1] * q[i5] * q[i16] + (-5.729034e-05) * q[i1] * q[i5] * q[i19]
            + (6.226303e-04) * q[i1] * q[i5] * q[i20] + (-6.467642e-04) * q[i1] * q[i5] * q[i21] + (4.217495e-04) * q[i1] * q[i5] * q[i22]
            + (-1.015857e-02) * q[i1] * q[i6] * q[i7] + (-2.377159e-04) * q[i1] * q[i6] * q[i8] + (-5.646921e-04) * q[i1] * q[i6] * q[i9]
            + (-1.639768e-03) * q[i1] * q[i6] * q[i10] + (2.123385e-04) * q[i1] * q[i6] * q[i11] + (7.670935e-05) * q[i1] * q[i6] * q[i12]
            + (-1.923527e-04) * q[i1] * q[i6] * q[i15] + (-7.395194e-04) * q[i1] * q[i6] * q[i16] + (-2.937084e-04) * q[i1] * q[i6] * q[i19]
            + (1.270226e-03) * q[i1] * q[i6] * q[i20] + (-3.794178e-04) * q[i1] * q[i6] * q[i21] + (-4.370185e-04) * q[i1] * q[i6] * q[i22]
            + (-1.668724e-03) * q[i1] * q[i7] * q[i8] + (-1.866107e-03) * q[i1] * q[i7] * q[i9] + (5.542534e-04) * q[i1] * q[i7] * q[i10]
            + (-1.426140e-03) * q[i1] * q[i7] * q[i11] + (-1.341927e-03) * q[i1] * q[i7] * q[i12] + (-1.151666e-03) * q[i1] * q[i7] * q[i15]
            + (3.387974e-04) * q[i1] * q[i7] * q[i16] + (-7.119287e-04) * q[i1] * q[i7] * q[i19] + (1.484588e-03) * q[i1] * q[i7] * q[i20]
            + (-8.684813e-04) * q[i1] * q[i7] * q[i21] + (5.826807e-04) * q[i1] * q[i7] * q[i22] + (-2.056903e-04) * q[i1] * q[i8] * q[i9]
            + (-7.043732e-04) * q[i1] * q[i8] * q[i10] + (1.002922e-03) * q[i1] * q[i8] * q[i11] + (-9.243151e-04) * q[i1] * q[i8] * q[i12]
            + (-2.146160e-03) * q[i1] * q[i8] * q[i15] + (7.556071e-04) * q[i1] * q[i8] * q[i16] + (2.228837e-04) * q[i1] * q[i8] * q[i19]
            + (3.807387e-04) * q[i1] * q[i8] * q[i20] + (-4.667108e-04) * q[i1] * q[i8] * q[i21] + (3.449386e-05) * q[i1] * q[i8] * q[i22]
            + (-1.360802e-03) * q[i1] * q[i9] * q[i10] + (5.642974e-04) * q[i1] * q[i9] * q[i11] + (7.066151e-04) * q[i1] * q[i9] * q[i12]
            + (5.905837e-04) * q[i1] * q[i9] * q[i15] + (-5.296953e-04) * q[i1] * q[i9] * q[i16] + (4.403208e-04) * q[i1] * q[i9] * q[i19]
            + (-2.157064e-04) * q[i1] * q[i9] * q[i20] + (-2.141346e-04) * q[i1] * q[i9] * q[i21] + (-1.572557e-04) * q[i1] * q[i9] * q[i22]
            + (-9.983918e-04) * q[i1] * q[i10] * q[i11] + (3.945032e-05) * q[i1] * q[i10] * q[i12] + (-1.115424e-03) * q[i1] * q[i10] * q[i15]
            + (-9.232939e-05) * q[i1] * q[i10] * q[i16] + (-5.613018e-04) * q[i1] * q[i10] * q[i19] + (1.733368e-04) * q[i1] * q[i10] * q[i20]
            + (-6.546301e-05) * q[i1] * q[i10] * q[i21] + (-7.665282e-05) * q[i1] * q[i10] * q[i22] + (5.435814e-04) * q[i1] * q[i11] * q[i12]
            + (-6.513442e-05) * q[i1] * q[i11] * q[i15] + (-2.026858e-05) * q[i1] * q[i11] * q[i16] + (6.352125e-05) * q[i1] * q[i11] * q[i19]
            + (-5.394431e-04) * q[i1] * q[i11] * q[i20] + (3.606199e-04) * q[i1] * q[i11] * q[i21] + (5.179043e-04) * q[i1] * q[i11] * q[i22]
            + (-7.006303e-05) * q[i1] * q[i12] * q[i15] + (1.941888e-04) * q[i1] * q[i12] * q[i16] + (4.266644e-04) * q[i1] * q[i12] * q[i19]
            + (5.463806e-04) * q[i1] * q[i12] * q[i20] + (-3.924730e-04) * q[i1] * q[i12] * q[i21] + (-2.773281e-04) * q[i1] * q[i12] * q[i22]
            + (2.186881e-04) * q[i1] * q[i15] * q[i16] + (-1.743718e-04) * q[i1] * q[i15] * q[i19] + (5.877674e-04) * q[i1] * q[i15] * q[i20]
            + (1.723671e-04) * q[i1] * q[i15] * q[i21] + (-2.020610e-04) * q[i1] * q[i15] * q[i22] + (1.262092e-04) * q[i1] * q[i16] * q[i19]
            + (8.947500e-05) * q[i1] * q[i16] * q[i20] + (1.137587e-04) * q[i1] * q[i16] * q[i21] + (3.327511e-04) * q[i1] * q[i16] * q[i22]
            + (-3.021009e-04) * q[i1] * q[i19] * q[i20] + (-2.477809e-05) * q[i1] * q[i19] * q[i21] + (-1.043606e-04) * q[i1] * q[i19] * q[i22]
            + (2.473619e-04) * q[i1] * q[i20] * q[i21] + (2.811799e-04) * q[i1] * q[i20] * q[i22] + (5.531782e-04) * q[i1] * q[i21] * q[i22]
            + (8.644762e-06) * q[i2] * q[i3] * q[i4] + (8.699480e-03) * q[i2] * q[i3] * q[i5] + (-5.663162e-03) * q[i2] * q[i3] * q[i6]
            + (6.717905e-04) * q[i2] * q[i3] * q[i7] + (-3.571442e-04) * q[i2] * q[i3] * q[i8] + (2.309691e-03) * q[i2] * q[i3] * q[i9]
            + (-2.988670e-04) * q[i2] * q[i3] * q[i10] + (7.361710e-04) * q[i2] * q[i3] * q[i11] + (5.600232e-04) * q[i2] * q[i3] * q[i12]
            + (9.927942e-04) * q[i2] * q[i3] * q[i15] + (-2.515278e-03) * q[i2] * q[i3] * q[i16] + (1.276545e-04) * q[i2] * q[i3] * q[i19]
            + (6.979532e-05) * q[i2] * q[i3] * q[i20] + (-6.013862e-04) * q[i2] * q[i3] * q[i21] + (6.002039e-04) * q[i2] * q[i3] * q[i22]
            + (-8.697755e-03) * q[i2] * q[i4] * q[i5] + (6.851293e-04) * q[i2] * q[i4] * q[i6] + (-5.699442e-03) * q[i2] * q[i4] * q[i7]
            + (-3.418379e-04) * q[i2] * q[i4] * q[i8] + (-3.234076e-04) * q[i2] * q[i4] * q[i9] + (2.270634e-03) * q[i2] * q[i4] * q[i10]
            + (-5.738576e-04) * q[i2] * q[i4] * q[i11] + (-7.830249e-04) * q[i2] * q[i4] * q[i12] + (-2.498836e-03) * q[i2] * q[i4] * q[i15]
            + (1.011946e-03) * q[i2] * q[i4] * q[i16] + (8.152563e-05) * q[i2] * q[i4] * q[i19] + (1.180434e-04) * q[i2] * q[i4] * q[i20]
            + (-5.871365e-04) * q[i2] * q[i4] * q[i21] + (5.962369e-04) * q[i2] * q[i4] * q[i22] + (3.143789e-04) * q[i2] * q[i5] * q[i6]
            + (3.189315e-04) * q[i2] * q[i5] * q[i7] + (-1.046169e-02) * q[i2] * q[i5] * q[i8] + (5.292999e-03) * q[i2] * q[i5] * q[i9]
            + (5.223961e-03) * q[i2] * q[i5] * q[i10] + (1.171327e-03) * q[i2] * q[i5] * q[i11] + (-1.285937e-03) * q[i2] * q[i5] * q[i12]
            + (-6.544554e-03) * q[i2] * q[i5] * q[i15] + (-6.624475e-03) * q[i2] * q[i5] * q[i16] + (1.439647e-04) * q[i2] * q[i5] * q[i19]
            + (1.530245e-04) * q[i2] * q[i5] * q[i20] + (2.058515e-03) * q[i2] * q[i5] * q[i21] + (-2.067730e-03) * q[i2] * q[i5] * q[i22]
            + (-1.321266e-06) * q[i2] * q[i6] * q[i7] + (9.315958e-03) * q[i2] * q[i6] * q[i8] + (1.297394e-03) * q[i2] * q[i6] * q[i9]
            + (-4.481669e-04) * q[i2] * q[i6] * q[i10] + (-7.785610e-04) * q[i2] * q[i6] * q[i11] + (4.258846e-04) * q[i2] * q[i6] * q[i12]
            + (-1.313394e-03) * q[i2] * q[i6] * q[i15] + (4.201134e-04) * q[i2] * q[i6] * q[i16] + (-3.549272e-04) * q[i2] * q[i6] * q[i19]
            + (7.624220e-04) * q[i2] * q[i6] * q[i20] + (4.757560e-04) * q[i2] * q[i6] * q[i21] + (-7.125799e-04) * q[i2] * q[i6] * q[i22]
            + (-9.328918e-03) * q[i2] * q[i7] * q[i8] + (4.524172e-04) * q[i2] * q[i7] * q[i9] + (-1.291531e-03) * q[i2] * q[i7] * q[i10]
            + (4.293556e-04) * q[i2] * q[i7] * q[i11] + (-8.119320e-04) * q[i2] * q[i7] * q[i12] + (-4.246611e-04) * q[i2] * q[i7] * q[i15]
            + (1.335803e-03) * q[i2] * q[i7] * q[i16] + (-7.598168e-04) * q[i2] * q[i7] * q[i19] + (3.408859e-04) * q[i2] * q[i7] * q[i20]
            + (-7.253175e-04) * q[i2] * q[i7] * q[i21] + (4.696598e-04) * q[i2] * q[i7] * q[i22] + (2.149226e-03) * q[i2] * q[i8] * q[i9]
            + (-2.143734e-03) * q[i2] * q[i8] * q[i10] + (4.147071e-03) * q[i2] * q[i8] * q[i11] + (4.154181e-03) * q[i2] * q[i8] * q[i12]
            + (-9.564607e-04) * q[i2] * q[i8] * q[i15] + (9.360359e-04) * q[i2] * q[i8] * q[i16] + (1.023466e-03) * q[i2] * q[i8] * q[i19]
            + (-1.027806e-03) * q[i2] * q[i8] * q[i20] + (4.401218e-04) * q[i2] * q[i8] * q[i21] + (4.437775e-04) * q[i2] * q[i8] * q[i22]
            + (2.516672e-06) * q[i2] * q[i9] * q[i10] + (3.620506e-06) * q[i2] * q[i9] * q[i11] + (-1.799722e-04) * q[i2] * q[i9] * q[i12]
            + (4.049757e-04) * q[i2] * q[i9] * q[i15] + (-5.789794e-05) * q[i2] * q[i9] * q[i16] + (-2.610005e-04) * q[i2] * q[i9] * q[i19]
            + (9.740003e-05) * q[i2] * q[i9] * q[i20] + (-4.083199e-04) * q[i2] * q[i9] * q[i21] + (-3.429924e-04) * q[i2] * q[i9] * q[i22]
            + (-1.926262e-04) * q[i2] * q[i10] * q[i11] + (1.820085e-06) * q[i2] * q[i10] * q[i12] + (6.286513e-05) * q[i2] * q[i10] * q[i15]
            + (-4.083522e-04) * q[i2] * q[i10] * q[i16] + (-1.011537e-04) * q[i2] * q[i10] * q[i19] + (2.619118e-04) * q[i2] * q[i10] * q[i20]
            + (-3.418190e-04) * q[i2] * q[i10] * q[i21] + (-4.035359e-04) * q[i2] * q[i10] * q[i22] + (6.764880e-06) * q[i2] * q[i11] * q[i12]
            + (-2.362006e-03) * q[i2] * q[i11] * q[i15] + (-5.016997e-04) * q[i2] * q[i11] * q[i16] + (6.703634e-04) * q[i2] * q[i11] * q[i19]
            + (-4.443000e-04) * q[i2] * q[i11] * q[i20] + (9.345787e-04) * q[i2] * q[i11] * q[i21] + (1.211605e-03) * q[i2] * q[i11] * q[i22]
            + (-5.111858e-04) * q[i2] * q[i12] * q[i15] + (-2.336272e-03) * q[i2] * q[i12] * q[i16] + (-4.444471e-04) * q[i2] * q[i12] * q[i19]
            + (6.501908e-04) * q[i2] * q[i12] * q[i20] + (-1.204995e-03) * q[i2] * q[i12] * q[i21] + (-9.273983e-04) * q[i2] * q[i12] * q[i22]
            + (-4.108729e-06) * q[i2] * q[i15] * q[i16] + (-3.347044e-04) * q[i2] * q[i15] * q[i19] + (1.346695e-03) * q[i2] * q[i15] * q[i20]
            + (7.605788e-04) * q[i2] * q[i15] * q[i21] + (3.999834e-05) * q[i2] * q[i15] * q[i22] + (-1.355870e-03) * q[i2] * q[i16] * q[i19]
            + (3.529748e-04) * q[i2] * q[i16] * q[i20] + (4.025737e-05) * q[i2] * q[i16] * q[i21] + (7.495992e-04) * q[i2] * q[i16] * q[i22]
            + (9.810085e-06) * q[i2] * q[i19] * q[i20] + (-6.799983e-04) * q[i2] * q[i19] * q[i21] + (-8.525243e-05) * q[i2] * q[i19] * q[i22]
            + (-9.238074e-05) * q[i2] * q[i20] * q[i21] + (-6.741689e-04) * q[i2] * q[i20] * q[i22] + (-1.328435e-06) * q[i2] * q[i21] * q[i22]
            + (-1.528038e-05) * q[i3] * q[i4] * q[i5] + (-4.692084e-03) * q[i3] * q[i4] * q[i6] + (-4.669405e-03) * q[i3] * q[i4] * q[i7]
            + (-2.187009e-03) * q[i3] * q[i4] * q[i8] + (-1.861590e-03) * q[i3] * q[i4] * q[i9] + (-1.869920e-03) * q[i3] * q[i4] * q[i10]
            + (1.272945e-03) * q[i3] * q[i4] * q[i11] + (-1.294124e-03) * q[i3] * q[i4] * q[i12] + (-3.552360e-04) * q[i3] * q[i4] * q[i15]
            + (-3.478787e-04) * q[i3] * q[i4] * q[i16] + (1.436047e-03) * q[i3] * q[i4] * q[i19] + (1.413824e-03) * q[i3] * q[i4] * q[i20]
            + (-1.400512e-03) * q[i3] * q[i4] * q[i21] + (1.412147e-03) * q[i3] * q[i4] * q[i22] + (-9.070857e-03) * q[i3] * q[i5] * q[i6]
            + (1.810798e-03) * q[i3] * q[i5] * q[i7] + (-3.838593e-03) * q[i3] * q[i5] * q[i8] + (-1.723064e-03) * q[i3] * q[i5] * q[i9]
            + (-5.797220e-04) * q[i3] * q[i5] * q[i10] + (-1.448399e-03) * q[i3] * q[i5] * q[i11] + (-1.207768e-03) * q[i3] * q[i5] * q[i12]
            + (-1.703704e-03) * q[i3] * q[i5] * q[i15] + (9.957671e-04) * q[i3] * q[i5] * q[i16] + (-1.447273e-03) * q[i3] * q[i5] * q[i19]
            + (2.859915e-04) * q[i3] * q[i5] * q[i20] + (-1.096375e-03) * q[i3] * q[i5] * q[i21] + (-2.078576e-03) * q[i3] * q[i5] * q[i22]
            + (1.678006e-04) * q[i3] * q[i6] * q[i7] + (5.232857e-04) * q[i3] * q[i6] * q[i8] + (1.313509e-03) * q[i3] * q[i6] * q[i9]
            + (-8.018915e-04) * q[i3] * q[i6] * q[i10] + (-2.051880e-03) * q[i3] * q[i6] * q[i11] + (-2.301127e-04) * q[i3] * q[i6] * q[i12]
            + (-6.109421e-04) * q[i3] * q[i6] * q[i15] + (1.322964e-03) * q[i3] * q[i6] * q[i16] + (5.983288e-04) * q[i3] * q[i6] * q[i19]
            + (-5.325464e-04) * q[i3] * q[i6] * q[i20] + (-1.193203e-03) * q[i3] * q[i6] * q[i21] + (-2.469359e-04) * q[i3] * q[i6] * q[i22]
            + (8.731107e-04) * q[i3] * q[i7] * q[i8] + (-1.450256e-03) * q[i3] * q[i7] * q[i9] + (-2.740255e-03) * q[i3] * q[i7] * q[i10]
            + (5.274222e-04) * q[i3] * q[i7] * q[i11] + (2.305726e-04) * q[i3] * q[i7] * q[i12] + (1.450320e-03) * q[i3] * q[i7] * q[i15]
            + (-7.606796e-04) * q[i3] * q[i7] * q[i16] + (2.271741e-04) * q[i3] * q[i7] * q[i19] + (1.320482e-04) * q[i3] * q[i7] * q[i20]
            + (-7.691139e-04) * q[i3] * q[i7] * q[i21] + (1.421559e-03) * q[i3] * q[i7] * q[i22] + (-1.037956e-03) * q[i3] * q[i8] * q[i9]
            + (2.746779e-04) * q[i3] * q[i8] * q[i10] + (1.522556e-03) * q[i3] * q[i8] * q[i11] + (-3.970226e-05) * q[i3] * q[i8] * q[i12]
            + (-2.196074e-03) * q[i3] * q[i8] * q[i15] + (1.745948e-03) * q[i3] * q[i8] * q[i16] + (-1.824331e-04) * q[i3] * q[i8] * q[i19]
            + (9.336729e-05) * q[i3] * q[i8] * q[i20] + (2.121110e-04) * q[i3] * q[i8] * q[i21] + (4.277614e-04) * q[i3] * q[i8] * q[i22]
            + (-8.110467e-04) * q[i3] * q[i9] * q[i10] + (7.615759e-04) * q[i3] * q[i9] * q[i11] + (6.839198e-04) * q[i3] * q[i9] * q[i12]
            + (-2.617285e-04) * q[i3] * q[i9] * q[i15] + (-7.010736e-04) * q[i3] * q[i9] * q[i16] + (3.544132e-04) * q[i3] * q[i9] * q[i19]
            + (-6.354895e-04) * q[i3] * q[i9] * q[i20] + (-2.459064e-04) * q[i3] * q[i9] * q[i21] + (-3.224698e-04) * q[i3] * q[i9] * q[i22]
            + (-1.161962e-03) * q[i3] * q[i10] * q[i11] + (7.719139e-05) * q[i3] * q[i10] * q[i12] + (-4.930728e-04) * q[i3] * q[i10] * q[i15]
            + (4.747305e-04) * q[i3] * q[i10] * q[i16] + (2.506987e-04) * q[i3] * q[i10] * q[i19] + (6.488983e-04) * q[i3] * q[i10] * q[i20]
            + (1.378146e-05) * q[i3] * q[i10] * q[i21] + (2.411005e-04) * q[i3] * q[i10] * q[i22] + (-7.578032e-04) * q[i3] * q[i11] * q[i12]
            + (-9.027708e-04) * q[i3] * q[i11] * q[i15] + (-3.870236e-05) * q[i3] * q[i11] * q[i16] + (-1.712573e-04) * q[i3] * q[i11] * q[i19]
            + (-8.019534e-04) * q[i3] * q[i11] * q[i20] + (-8.460064e-04) * q[i3] * q[i11] * q[i21] + (4.242010e-04) * q[i3] * q[i11] * q[i22]
            + (1.071918e-03) * q[i3] * q[i12] * q[i15] + (-4.166864e-04) * q[i3] * q[i12] * q[i16] + (8.904234e-05) * q[i3] * q[i12] * q[i19]
            + (-1.091851e-04) * q[i3] * q[i12] * q[i20] + (-2.769233e-05) * q[i3] * q[i12] * q[i21] + (-1.330506e-04) * q[i3] * q[i12] * q[i22]
            + (-4.291182e-04) * q[i3] * q[i15] * q[i16] + (-1.090363e-03) * q[i3] * q[i15] * q[i19] + (-3.773423e-04) * q[i3] * q[i15] * q[i20]
            + (-7.025679e-04) * q[i3] * q[i15] * q[i21] + (-7.839491e-04) * q[i3] * q[i15] * q[i22] + (-4.696699e-04) * q[i3] * q[i16] * q[i19]
            + (1.838969e-04) * q[i3] * q[i16] * q[i20] + (-4.494096e-05) * q[i3] * q[i16] * q[i21] + (2.849656e-04) * q[i3] * q[i16] * q[i22]
            + (7.589383e-04) * q[i3] * q[i19] * q[i20] + (7.291109e-05) * q[i3] * q[i19] * q[i21] + (2.006097e-04) * q[i3] * q[i19] * q[i22]
            + (-7.413924e-04) * q[i3] * q[i20] * q[i21] + (8.525907e-04) * q[i3] * q[i20] * q[i22] + (1.085187e-04) * q[i3] * q[i21] * q[i22]
            + (1.822253e-03) * q[i4] * q[i5] * q[i6] + (-9.067011e-03) * q[i4] * q[i5] * q[i7] + (-3.841741e-03) * q[i4] * q[i5] * q[i8]
            + (-5.934737e-04) * q[i4] * q[i5] * q[i9] + (-1.696389e-03) * q[i4] * q[i5] * q[i10] + (1.201051e-03) * q[i4] * q[i5] * q[i11]
            + (1.420428e-03) * q[i4] * q[i5] * q[i12] + (9.956928e-04) * q[i4] * q[i5] * q[i15] + (-1.694699e-03) * q[i4] * q[i5] * q[i16]
            + (2.752440e-04) * q[i4] * q[i5] * q[i19] + (-1.445958e-03) * q[i4] * q[i5] * q[i20] + (2.063944e-03) * q[i4] * q[i5] * q[i21]
            + (1.112460e-03) * q[i4] * q[i5] * q[i22] + (-1.860803e-04) * q[i4] * q[i6] * q[i7] + (-8.465525e-04) * q[i4] * q[i6] * q[i8]
            + (2.745881e-03) * q[i4] * q[i6] * q[i9] + (1.436831e-03) * q[i4] * q[i6] * q[i10] + (2.222197e-04) * q[i4] * q[i6] * q[i11]
            + (5.484947e-04) * q[i4] * q[i6] * q[i12] + (7.600604e-04) * q[i4] * q[i6] * q[i15] + (-1.475898e-03) * q[i4] * q[i6] * q[i16]
            + (-1.344769e-04) * q[i4] * q[i6] * q[i19] + (-2.211296e-04) * q[i4] * q[i6] * q[i20] + (1.419563e-03) * q[i4] * q[i6] * q[i21]
            + (-7.808593e-04) * q[i4] * q[i6] * q[i22] + (-5.403478e-04) * q[i4] * q[i7] * q[i8] + (8.076992e-04) * q[i4] * q[i7] * q[i9]
            + (-1.282671e-03) * q[i4] * q[i7] * q[i10] + (-2.309113e-04) * q[i4] * q[i7] * q[i11] + (-2.075818e-03) * q[i4] * q[i7] * q[i12]
            + (-1.302155e-03) * q[i4] * q[i7] * q[i15] + (6.214028e-04) * q[i4] * q[i7] * q[i16] + (5.267683e-04) * q[i4] * q[i7] * q[i19]
            + (-6.071990e-04) * q[i4] * q[i7] * q[i20] + (-2.688400e-04) * q[i4] * q[i7] * q[i21] + (-1.188431e-03) * q[i4] * q[i7] * q[i22]
            + (-2.740939e-04) * q[i4] * q[i8] * q[i9] + (1.022948e-03) * q[i4] * q[i8] * q[i10] + (-3.970672e-05) * q[i4] * q[i8] * q[i11]
            + (1.552861e-03) * q[i4] * q[i8] * q[i12] + (-1.735568e-03) * q[i4] * q[i8] * q[i15] + (2.215468e-03) * q[i4] * q[i8] * q[i16]
            + (-9.381795e-05) * q[i4] * q[i8] * q[i19] + (1.747690e-04) * q[i4] * q[i8] * q[i20] + (4.350437e-04) * q[i4] * q[i8] * q[i21]
            + (2.177448e-04) * q[i4] * q[i8] * q[i22] + (8.112286e-04) * q[i4] * q[i9] * q[i10] + (7.421642e-05) * q[i4] * q[i9] * q[i11]
            + (-1.158998e-03) * q[i4] * q[i9] * q[i12] + (-4.751135e-04) * q[i4] * q[i9] * q[i15] + (4.904249e-04) * q[i4] * q[i9] * q[i16]
            + (-6.228559e-04) * q[i4] * q[i9] * q[i19] + (-2.543523e-04) * q[i4] * q[i9] * q[i20] + (2.289478e-04) * q[i4] * q[i9] * q[i21]
            + (-4.351189e-08) * q[i4] * q[i9] * q[i22] + (6.648302e-04) * q[i4] * q[i10] * q[i11] + (7.633476e-04) * q[i4] * q[i10] * q[i12]
            + (6.980060e-04) * q[i4] * q[i10] * q[i15] + (2.553292e-04) * q[i4] * q[i10] * q[i16] + (6.283218e-04) * q[i4] * q[i10] * q[i19]
            + (-3.471171e-04) * q[i4] * q[i10] * q[i20] + (-3.230970e-04) * q[i4] * q[i10] * q[i21] + (-2.538713e-04) * q[i4] * q[i10] * q[i22]
            + (7.642985e-04) * q[i4] * q[i11] * q[i12] + (-4.090085e-04) * q[i4] * q[i11] * q[i15] + (1.070191e-03) * q[i4] * q[i11] * q[i16]
            + (-1.119821e-04) * q[i4] * q[i11] * q[i19] + (9.979069e-05) * q[i4] * q[i11] * q[i20] + (1.227838e-04) * q[i4] * q[i11] * q[i21]
            + (2.703891e-05) * q[i4] * q[i11] * q[i22] + (-2.736292e-05) * q[i4] * q[i12] * q[i15] + (-9.062632e-04) * q[i4] * q[i12] * q[i16]
            + (-7.938514e-04) * q[i4] * q[i12] * q[i19] + (-1.668851e-04) * q[i4] * q[i12] * q[i20] + (-4.133740e-04) * q[i4] * q[i12] * q[i21]
            + (8.567602e-04) * q[i4] * q[i12] * q[i22] + (4.269459e-04) * q[i4] * q[i15] * q[i16] + (-1.936850e-04) * q[i4] * q[i15] * q[i19]
            + (4.801148e-04) * q[i4] * q[i15] * q[i20] + (2.930438e-04) * q[i4] * q[i15] * q[i21] + (-5.318361e-05) * q[i4] * q[i15] * q[i22]
            + (3.716815e-04) * q[i4] * q[i16] * q[i19] + (1.078294e-03) * q[i4] * q[i16] * q[i20] + (-7.797041e-04) * q[i4] * q[i16] * q[i21]
            + (-7.098311e-04) * q[i4] * q[i16] * q[i22] + (-7.567033e-04) * q[i4] * q[i19] * q[i20] + (8.571854e-04) * q[i4] * q[i19] * q[i21]
            + (-7.366768e-04) * q[i4] * q[i19] * q[i22] + (1.948996e-04) * q[i4] * q[i20] * q[i21] + (7.178784e-05) * q[i4] * q[i20] * q[i22]
            + (-1.024263e-04) * q[i4] * q[i21] * q[i22] + (1.324561e-05) * q[i5] * q[i6] * q[i7] + (1.535462e-03) * q[i5] * q[i6] * q[i8]
            + (2.186654e-03) * q[i5] * q[i6] * q[i9] + (-2.477935e-03) * q[i5] * q[i6] * q[i10] + (4.159708e-04) * q[i5] * q[i6] * q[i11]
            + (-1.840572e-03) * q[i5] * q[i6] * q[i12] + (6.391077e-04) * q[i5] * q[i6] * q[i15] + (-3.613898e-04) * q[i5] * q[i6] * q[i16]
            + (-9.828295e-04) * q[i5] * q[i6] * q[i19] + (-2.404216e-04) * q[i5] * q[i6] * q[i20] + (-2.500503e-04) * q[i5] * q[i6] * q[i21]
            + (2.925324e-04) * q[i5] * q[i6] * q[i22] + (-1.558072e-03) * q[i5] * q[i7] * q[i8] + (2.477884e-03) * q[i5] * q[i7] * q[i9]
            + (-2.164833e-03) * q[i5] * q[i7] * q[i10] + (-1.864377e-03) * q[i5] * q[i7] * q[i11] + (4.200824e-04) * q[i5] * q[i7] * q[i12]
            + (3.664296e-04) * q[i5] * q[i7] * q[i15] + (-6.492493e-04) * q[i5] * q[i7] * q[i16] + (2.210279e-04) * q[i5] * q[i7] * q[i19]
            + (9.826121e-04) * q[i5] * q[i7] * q[i20] + (2.803336e-04) * q[i5] * q[i7] * q[i21] + (-2.304339e-04) * q[i5] * q[i7] * q[i22]
            + (2.484928e-03) * q[i5] * q[i8] * q[i9] + (-2.482312e-03) * q[i5] * q[i8] * q[i10] + (2.648866e-03) * q[i5] * q[i8] * q[i11]
            + (2.660963e-03) * q[i5] * q[i8] * q[i12] + (2.435700e-03) * q[i5] * q[i8] * q[i15] + (-2.476418e-03) * q[i5] * q[i8] * q[i16]
            + (1.505927e-04) * q[i5] * q[i8] * q[i19] + (-1.445753e-04) * q[i5] * q[i8] * q[i20] + (-5.209222e-05) * q[i5] * q[i8] * q[i21]
            + (-4.250972e-05) * q[i5] * q[i8] * q[i22] + (-2.280379e-06) * q[i5] * q[i9] * q[i10] + (-9.996856e-04) * q[i5] * q[i9] * q[i11]
            + (2.664558e-04) * q[i5] * q[i9] * q[i12] + (4.065399e-04) * q[i5] * q[i9] * q[i15] + (-8.526925e-04) * q[i5] * q[i9] * q[i16]
            + (6.817273e-04) * q[i5] * q[i9] * q[i19] + (2.758747e-04) * q[i5] * q[i9] * q[i20] + (-5.875879e-04) * q[i5] * q[i9] * q[i21]
            + (-2.189059e-04) * q[i5] * q[i9] * q[i22] + (2.569866e-04) * q[i5] * q[i10] * q[i11] + (-9.864429e-04) * q[i5] * q[i10] * q[i12]
            + (8.599808e-04) * q[i5] * q[i10] * q[i15] + (-4.068580e-04) * q[i5] * q[i10] * q[i16] + (-2.698364e-04) * q[i5] * q[i10] * q[i19]
            + (-7.060816e-04) * q[i5] * q[i10] * q[i20] + (-2.235920e-04) * q[i5] * q[i10] * q[i21] + (-5.826465e-04) * q[i5] * q[i10] * q[i22]
            + (7.873494e-06) * q[i5] * q[i11] * q[i12] + (-2.545878e-03) * q[i5] * q[i11] * q[i15] + (-3.355602e-04) * q[i5] * q[i11] * q[i16]
            + (-3.721245e-04) * q[i5] * q[i11] * q[i19] + (3.566828e-04) * q[i5] * q[i11] * q[i20] + (-5.596208e-04) * q[i5] * q[i11] * q[i21]
            + (-2.649668e-04) * q[i5] * q[i11] * q[i22] + (-3.395477e-04) * q[i5] * q[i12] * q[i15] + (-2.540585e-03) * q[i5] * q[i12] * q[i16]
            + (3.623418e-04) * q[i5] * q[i12] * q[i19] + (-3.867339e-04) * q[i5] * q[i12] * q[i20] + (2.657216e-04) * q[i5] * q[i12] * q[i21]
            + (5.715204e-04) * q[i5] * q[i12] * q[i22] + (-1.584383e-06) * q[i5] * q[i15] * q[i16] + (-4.229998e-04) * q[i5] * q[i15] * q[i19]
            + (-4.258683e-04) * q[i5] * q[i15] * q[i20] + (6.511432e-04) * q[i5] * q[i15] * q[i21] + (4.829936e-04) * q[i5] * q[i15] * q[i22]
            + (4.420311e-04) * q[i5] * q[i16] * q[i19] + (4.188070e-04) * q[i5] * q[i16] * q[i20] + (4.852346e-04) * q[i5] * q[i16] * q[i21]
            + (6.692534e-04) * q[i5] * q[i16] * q[i22] + (2.482564e-06) * q[i5] * q[i19] * q[i20] + (6.058628e-04) * q[i5] * q[i19] * q[i21]
            + (9.127125e-05) * q[i5] * q[i19] * q[i22] + (9.610038e-05) * q[i5] * q[i20] * q[i21] + (6.030247e-04) * q[i5] * q[i20] * q[i22]
            + (-2.869781e-06) * q[i5] * q[i21] * q[i22] + (-7.091101e-03) * q[i6] * q[i7] * q[i8] + (5.066186e-04) * q[i6] * q[i7] * q[i9]
            + (5.131238e-04) * q[i6] * q[i7] * q[i10] + (1.122814e-03) * q[i6] * q[i7] * q[i11] + (-1.119746e-03) * q[i6] * q[i7] * q[i12]
            + (6.383422e-04) * q[i6] * q[i7] * q[i15] + (6.339042e-04) * q[i6] * q[i7] * q[i16] + (2.586580e-04) * q[i6] * q[i7] * q[i19]
            + (2.497187e-04) * q[i6] * q[i7] * q[i20] + (-2.658031e-04) * q[i6] * q[i7] * q[i21] + (2.470120e-04) * q[i6] * q[i7] * q[i22]
            + (-3.254459e-04) * q[i6] * q[i8] * q[i9] + (-1.566180e-03) * q[i6] * q[i8] * q[i10] + (-8.828370e-04) * q[i6] * q[i8] * q[i11]
            + (-1.065433e-03) * q[i6] * q[i8] * q[i12] + (7.782932e-04) * q[i6] * q[i8] * q[i15] + (-7.840902e-04) * q[i6] * q[i8] * q[i16]
            + (-5.448643e-05) * q[i6] * q[i8] * q[i19] + (-2.203541e-04) * q[i6] * q[i8] * q[i20] + (-4.420825e-04) * q[i6] * q[i8] * q[i21]
            + (-1.667029e-04) * q[i6] * q[i8] * q[i22] + (-3.222617e-04) * q[i6] * q[i9] * q[i10] + (3.018579e-04) * q[i6] * q[i9] * q[i11]
            + (3.321788e-04) * q[i6] * q[i9] * q[i12] + (-4.425578e-04) * q[i6] * q[i9] * q[i15] + (-8.358086e-05) * q[i6] * q[i9] * q[i16]
            + (1.934901e-04) * q[i6] * q[i9] * q[i19] + (-2.956798e-04) * q[i6] * q[i9] * q[i20] + (-2.630103e-04) * q[i6] * q[i9] * q[i21]
            + (-2.043316e-04) * q[i6] * q[i9] * q[i22] + (3.046552e-05) * q[i6] * q[i10] * q[i11] + (-5.482079e-04) * q[i6] * q[i10] * q[i12]
            + (7.382468e-06) * q[i6] * q[i10] * q[i15] + (2.798608e-04) * q[i6] * q[i10] * q[i16] + (-1.757855e-05) * q[i6] * q[i10] * q[i19]
            + (4.630654e-04) * q[i6] * q[i10] * q[i20] + (-3.136931e-04) * q[i6] * q[i10] * q[i21] + (-3.502427e-06) * q[i6] * q[i10] * q[i22]
            + (-6.285653e-05) * q[i6] * q[i11] * q[i12] + (4.187876e-04) * q[i6] * q[i11] * q[i15] + (3.723862e-04) * q[i6] * q[i11] * q[i16]
            + (-1.640755e-04) * q[i6] * q[i11] * q[i19] + (-2.434679e-04) * q[i6] * q[i11] * q[i20] + (-2.957810e-04) * q[i6] * q[i11] * q[i21]
            + (-6.079118e-04) * q[i6] * q[i11] * q[i22] + (3.061818e-05) * q[i6] * q[i12] * q[i15] + (-5.408422e-04) * q[i6] * q[i12] * q[i16]
            + (-9.819785e-04) * q[i6] * q[i12] * q[i19] + (8.291197e-05) * q[i6] * q[i12] * q[i20] + (6.580356e-04) * q[i6] * q[i12] * q[i21]
            + (3.425389e-07) * q[i6] * q[i12] * q[i22] + (-2.138078e-05) * q[i6] * q[i15] * q[i16] + (-6.097696e-05) * q[i6] * q[i15] * q[i19]
            + (-4.208301e-04) * q[i6] * q[i15] * q[i20] + (-3.006825e-04) * q[i6] * q[i15] * q[i21] + (6.121058e-05) * q[i6] * q[i15] * q[i22]
            + (1.010537e-03) * q[i6] * q[i16] * q[i19] + (-2.576356e-04) * q[i6] * q[i16] * q[i20] + (1.806325e-04) * q[i6] * q[i16] * q[i21]
            + (-7.857355e-04) * q[i6] * q[i16] * q[i22] + (-3.891080e-04) * q[i6] * q[i19] * q[i20] + (3.961905e-04) * q[i6] * q[i19] * q[i21]
            + (-1.491139e-04) * q[i6] * q[i19] * q[i22] + (1.049546e-04) * q[i6] * q[i20] * q[i21] + (2.197945e-04) * q[i6] * q[i20] * q[i22]
            + (3.274916e-04) * q[i6] * q[i21] * q[i22] + (-1.582883e-03) * q[i7] * q[i8] * q[i9] + (-3.117434e-04) * q[i7] * q[i8] * q[i10]
            + (1.097291e-03) * q[i7] * q[i8] * q[i11] + (8.750085e-04) * q[i7] * q[i8] * q[i12] + (-7.901275e-04) * q[i7] * q[i8] * q[i15]
            + (7.769723e-04) * q[i7] * q[i8] * q[i16] + (-2.197780e-04) * q[i7] * q[i8] * q[i19] + (-3.898825e-05) * q[i7] * q[i8] * q[i20]
            + (1.665584e-04) * q[i7] * q[i8] * q[i21] + (4.488181e-04) * q[i7] * q[i8] * q[i22] + (-3.334738e-04) * q[i7] * q[i9] * q[i10]
            + (5.548080e-04) * q[i7] * q[i9] * q[i11] + (-2.256889e-05) * q[i7] * q[i9] * q[i12] + (2.860588e-04) * q[i7] * q[i9] * q[i15]
            + (1.170306e-05) * q[i7] * q[i9] * q[i16] + (4.661810e-04) * q[i7] * q[i9] * q[i19] + (-2.378912e-05) * q[i7] * q[i9] * q[i20]
            + (2.457164e-06) * q[i7] * q[i9] * q[i21] + (3.049735e-04) * q[i7] * q[i9] * q[i22] + (-3.371485e-04) * q[i7] * q[i10] * q[i11]
            + (-3.108714e-04) * q[i7] * q[i10] * q[i12] + (-7.809335e-05) * q[i7] * q[i10] * q[i15] + (-4.451027e-04) * q[i7] * q[i10] * q[i16]
            + (-2.948753e-04) * q[i7] * q[i10] * q[i19] + (1.864975e-04) * q[i7] * q[i10] * q[i20] + (2.096182e-04) * q[i7] * q[i10] * q[i21]
            + (2.627882e-04) * q[i7] * q[i10] * q[i22] + (-8.108071e-05) * q[i7] * q[i11] * q[i12] + (5.273374e-04) * q[i7] * q[i11] * q[i15]
            + (-3.163262e-05) * q[i7] * q[i11] * q[i16] + (-7.524544e-05) * q[i7] * q[i11] * q[i19] + (9.860693e-04) * q[i7] * q[i11] * q[i20]
            + (-1.315688e-06) * q[i7] * q[i11] * q[i21] + (6.556906e-04) * q[i7] * q[i11] * q[i22] + (-3.739448e-04) * q[i7] * q[i12] * q[i15]
            + (-4.156540e-04) * q[i7] * q[i12] * q[i16] + (2.504454e-04) * q[i7] * q[i12] * q[i19] + (1.631307e-04) * q[i7] * q[i12] * q[i20]
            + (-6.068143e-04) * q[i7] * q[i12] * q[i21] + (-2.887615e-04) * q[i7] * q[i12] * q[i22] + (-2.752398e-05) * q[i7] * q[i15] * q[i16]
            + (-2.515684e-04) * q[i7] * q[i15] * q[i19] + (1.008449e-03) * q[i7] * q[i15] * q[i20] + (7.773756e-04) * q[i7] * q[i15] * q[i21]
            + (-1.900134e-04) * q[i7] * q[i15] * q[i22] + (-4.187241e-04) * q[i7] * q[i16] * q[i19] + (-4.536671e-05) * q[i7] * q[i16] * q[i20]
            + (-4.937533e-05) * q[i7] * q[i16] * q[i21] + (3.086174e-04) * q[i7] * q[i16] * q[i22] + (-3.844589e-04) * q[i7] * q[i19] * q[i20]
            + (-2.231712e-04) * q[i7] * q[i19] * q[i21] + (-1.014804e-04) * q[i7] * q[i19] * q[i22] + (1.437196e-04) * q[i7] * q[i20] * q[i21]
            + (-3.856598e-04) * q[i7] * q[i20] * q[i22] + (3.294441e-04) * q[i7] * q[i21] * q[i22] + (-1.025028e-04) * q[i8] * q[i9] * q[i10]
            + (-4.504523e-04) * q[i8] * q[i9] * q[i11] + (-9.541455e-04) * q[i8] * q[i9] * q[i12] + (-3.011738e-04) * q[i8] * q[i9] * q[i15]
            + (-1.652432e-04) * q[i8] * q[i9] * q[i16] + (-1.399525e-04) * q[i8] * q[i9] * q[i19] + (8.570201e-04) * q[i8] * q[i9] * q[i20]
            + (9.173531e-05) * q[i8] * q[i9] * q[i21] + (-3.030985e-04) * q[i8] * q[i9] * q[i22] + (9.523357e-04) * q[i8] * q[i10] * q[i11]
            + (4.452269e-04) * q[i8] * q[i10] * q[i12] + (-1.570818e-04) * q[i8] * q[i10] * q[i15] + (-3.077152e-04) * q[i8] * q[i10] * q[i16]
            + (8.625016e-04) * q[i8] * q[i10] * q[i19] + (-1.244961e-04) * q[i8] * q[i10] * q[i20] + (2.951818e-04) * q[i8] * q[i10] * q[i21]
            + (-9.019046e-05) * q[i8] * q[i10] * q[i22] + (-1.421608e-03) * q[i8] * q[i11] * q[i12] + (-9.667942e-04) * q[i8] * q[i11] * q[i15]
            + (2.049559e-04) * q[i8] * q[i11] * q[i16] + (7.583491e-04) * q[i8] * q[i11] * q[i19] + (1.177453e-03) * q[i8] * q[i11] * q[i20]
            + (-3.791023e-04) * q[i8] * q[i11] * q[i21] + (-2.098501e-04) * q[i8] * q[i11] * q[i22] + (-2.133103e-04) * q[i8] * q[i12] * q[i15]
            + (9.615786e-04) * q[i8] * q[i12] * q[i16] + (-1.190358e-03) * q[i8] * q[i12] * q[i19] + (-7.548895e-04) * q[i8] * q[i12] * q[i20]
            + (-2.082552e-04) * q[i8] * q[i12] * q[i21] + (-3.871480e-04) * q[i8] * q[i12] * q[i22] + (-2.463027e-04) * q[i8] * q[i15] * q[i16]
            + (1.797364e-04) * q[i8] * q[i15] * q[i19] + (-7.647453e-04) * q[i8] * q[i15] * q[i20] + (-1.503288e-03) * q[i8] * q[i15] * q[i21]
            + (2.291151e-04) * q[i8] * q[i15] * q[i22] + (-7.659774e-04) * q[i8] * q[i16] * q[i19] + (1.855496e-04) * q[i8] * q[i16] * q[i20]
            + (-2.341260e-04) * q[i8] * q[i16] * q[i21] + (1.516622e-03) * q[i8] * q[i16] * q[i22] + (6.328515e-04) * q[i8] * q[i19] * q[i20]
            + (-1.120134e-03) * q[i8] * q[i19] * q[i21] + (-4.248339e-04) * q[i8] * q[i19] * q[i22] + (4.221432e-04) * q[i8] * q[i20] * q[i21]
            + (1.111237e-03) * q[i8] * q[i20] * q[i22] + (-4.557013e-04) * q[i8] * q[i21] * q[i22] + (1.670375e-04) * q[i9] * q[i10] * q[i11]
            + (-1.631895e-04) * q[i9] * q[i10] * q[i12] + (2.964690e-04) * q[i9] * q[i10] * q[i15] + (2.970809e-04) * q[i9] * q[i10] * q[i16]
            + (3.917550e-05) * q[i9] * q[i10] * q[i19] + (4.137510e-05) * q[i9] * q[i10] * q[i20] + (7.523850e-05) * q[i9] * q[i10] * q[i21]
            + (-7.457916e-05) * q[i9] * q[i10] * q[i22] + (2.691510e-04) * q[i9] * q[i11] * q[i12] + (1.353202e-04) * q[i9] * q[i11] * q[i15]
            + (-1.070767e-04) * q[i9] * q[i11] * q[i16] + (1.100914e-04) * q[i9] * q[i11] * q[i19] + (5.449230e-04) * q[i9] * q[i11] * q[i20]
            + (-5.803486e-05) * q[i9] * q[i11] * q[i21] + (8.276334e-05) * q[i9] * q[i11] * q[i22] + (2.551192e-04) * q[i9] * q[i12] * q[i15]
            + (2.425242e-04) * q[i9] * q[i12] * q[i16] + (1.687963e-04) * q[i9] * q[i12] * q[i19] + (3.931066e-04) * q[i9] * q[i12] * q[i20]
            + (-2.334586e-05) * q[i9] * q[i12] * q[i21] + (3.840462e-07) * q[i9] * q[i12] * q[i22] + (2.926568e-04) * q[i9] * q[i15] * q[i16]
            + (-1.205072e-04) * q[i9] * q[i15] * q[i19] + (2.208531e-04) * q[i9] * q[i15] * q[i20] + (5.189756e-06) * q[i9] * q[i15] * q[i21]
            + (-3.313817e-05) * q[i9] * q[i15] * q[i22] + (4.823402e-04) * q[i9] * q[i16] * q[i19] + (2.434512e-04) * q[i9] * q[i16] * q[i20]
            + (-8.190503e-05) * q[i9] * q[i16] * q[i21] + (-1.549280e-04) * q[i9] * q[i16] * q[i22] + (-1.338109e-04) * q[i9] * q[i19] * q[i20]
            + (1.220038e-04) * q[i9] * q[i19] * q[i21] + (1.010255e-04) * q[i9] * q[i19] * q[i22] + (1.195072e-04) * q[i9] * q[i20] * q[i21]
            + (-1.742876e-04) * q[i9] * q[i20] * q[i22] + (6.391691e-05) * q[i9] * q[i21] * q[i22] + (2.763474e-04) * q[i10] * q[i11] * q[i12]
            + (-2.427197e-04) * q[i10] * q[i11] * q[i15] + (-2.551507e-04) * q[i10] * q[i11] * q[i16] + (-3.946347e-04) * q[i10] * q[i11] * q[i19]
            + (-1.611441e-04) * q[i10] * q[i11] * q[i20] + (-1.973640e-06) * q[i10] * q[i11] * q[i21] + (-1.972338e-05) * q[i10] * q[i11] * q[i22]
            + (1.152339e-04) * q[i10] * q[i12] * q[i15] + (-1.365703e-04) * q[i10] * q[i12] * q[i16] + (-5.485815e-04) * q[i10] * q[i12] * q[i19]
            + (-1.178500e-04) * q[i10] * q[i12] * q[i20] + (8.537272e-05) * q[i10] * q[i12] * q[i21] + (-5.462330e-05) * q[i10] * q[i12] * q[i22]
            + (2.893244e-04) * q[i10] * q[i15] * q[i16] + (2.416269e-04) * q[i10] * q[i15] * q[i19] + (4.802103e-04) * q[i10] * q[i15] * q[i20]
            + (1.516216e-04) * q[i10] * q[i15] * q[i21] + (8.125625e-05) * q[i10] * q[i15] * q[i22] + (2.265096e-04) * q[i10] * q[i16] * q[i19]
            + (-1.238974e-04) * q[i10] * q[i16] * q[i20] + (3.186902e-05) * q[i10] * q[i16] * q[i21] + (-2.368354e-06) * q[i10] * q[i16] * q[i22]
            + (-1.316243e-04) * q[i10] * q[i19] * q[i20] + (1.747631e-04) * q[i10] * q[i19] * q[i21] + (-1.190671e-04) * q[i10] * q[i19] * q[i22]
            + (-9.656492e-05) * q[i10] * q[i20] * q[i21] + (-1.132204e-04) * q[i10] * q[i20] * q[i22] + (6.457753e-05) * q[i10] * q[i21] * q[i22]
            + (1.804308e-05) * q[i11] * q[i12] * q[i15] + (2.901597e-05) * q[i11] * q[i12] * q[i16] + (-2.141351e-04) * q[i11] * q[i12] * q[i19]
            + (-2.115579e-04) * q[i11] * q[i12] * q[i20] + (3.290286e-06) * q[i11] * q[i12] * q[i21] + (9.393283e-08) * q[i11] * q[i12] * q[i22]
            + (-1.103244e-04) * q[i11] * q[i15] * q[i16] + (1.222465e-04) * q[i11] * q[i15] * q[i19] + (-5.286889e-04) * q[i11] * q[i15] * q[i20]
            + (-7.061340e-05) * q[i11] * q[i15] * q[i21] + (1.524228e-04) * q[i11] * q[i15] * q[i22] + (-5.783379e-04) * q[i11] * q[i16] * q[i19]
            + (-1.003911e-04) * q[i11] * q[i16] * q[i20] + (1.560478e-04) * q[i11] * q[i16] * q[i21] + (-5.117911e-05) * q[i11] * q[i16] * q[i22]
            + (1.460629e-04) * q[i11] * q[i19] * q[i20] + (1.258352e-04) * q[i11] * q[i19] * q[i21] + (-9.818217e-05) * q[i11] * q[i19] * q[i22]
            + (-8.726795e-05) * q[i11] * q[i20] * q[i21] + (-1.764124e-04) * q[i11] * q[i20] * q[i22] + (-1.611880e-05) * q[i11] * q[i21] * q[i22]
            + (1.079393e-04) * q[i12] * q[i15] * q[i16] + (9.948236e-05) * q[i12] * q[i15] * q[i19] + (5.750484e-04) * q[i12] * q[i15] * q[i20]
            + (-4.713812e-05) * q[i12] * q[i15] * q[i21] + (1.608407e-04) * q[i12] * q[i15] * q[i22] + (5.335886e-04) * q[i12] * q[i16] * q[i19]
            + (-1.164256e-04) * q[i12] * q[i16] * q[i20] + (1.596939e-04) * q[i12] * q[i16] * q[i21] + (-8.637346e-05) * q[i12] * q[i16] * q[i22]
            + (-1.457333e-04) * q[i12] * q[i19] * q[i20] + (-1.774228e-04) * q[i12] * q[i19] * q[i21] + (-8.433190e-05) * q[i12] * q[i19] * q[i22]
            + (-9.988231e-05) * q[i12] * q[i20] * q[i21] + (1.307902e-04) * q[i12] * q[i20] * q[i22] + (1.312943e-05) * q[i12] * q[i21] * q[i22]
            + (2.676551e-04) * q[i15] * q[i16] * q[i19] + (2.627945e-04) * q[i15] * q[i16] * q[i20] + (6.172918e-05) * q[i15] * q[i16] * q[i21]
            + (-6.089486e-05) * q[i15] * q[i16] * q[i22] + (9.631664e-05) * q[i15] * q[i19] * q[i20] + (-8.679254e-04) * q[i15] * q[i19] * q[i21]
            + (-2.501875e-04) * q[i15] * q[i19] * q[i22] + (-1.314745e-04) * q[i15] * q[i20] * q[i21] + (-9.374580e-05) * q[i15] * q[i20] * q[i22]
            + (-4.223805e-05) * q[i15] * q[i21] * q[i22] + (9.661418e-05) * q[i16] * q[i19] * q[i20] + (9.483944e-05) * q[i16] * q[i19] * q[i21]
            + (1.345618e-04) * q[i16] * q[i19] * q[i22] + (2.487113e-04) * q[i16] * q[i20] * q[i21] + (8.598867e-04) * q[i16] * q[i20] * q[i22]
            + (-4.249707e-05) * q[i16] * q[i21] * q[i22] + (3.544925e-04) * q[i19] * q[i20] * q[i21] + (-3.544988e-04) * q[i19] * q[i20] * q[i22]
            + (3.561850e-06) * q[i19] * q[i21] * q[i22] + (3.144747e-06) * q[i20] * q[i21] * q[i22];
      return Qy;
   }

   public double getQz(double[] q)
   {
      double Qz;
      Qz = (-1.133065e-01) * q[i0] + (-1.127153e-01) * q[i1] + (2.139773e-01) * q[i2] + (2.223118e-02) * q[i3] + (2.211737e-02) * q[i4]
            + (-1.718942e-02) * q[i5] + (8.517761e-02) * q[i6] + (-8.492880e-02) * q[i7] + (-2.080289e-04) * q[i8] + (3.172784e-02) * q[i9]
            + (-3.150416e-02) * q[i10] + (4.990228e-02) * q[i11] + (5.017613e-02) * q[i12] + (1.203367e-02) * q[i15] + (-1.233942e-02) * q[i16]
            + (9.749392e-03) * q[i19] + (-9.463679e-03) * q[i20] + (8.252879e-03) * q[i21] + (8.274845e-03) * q[i22] + (6.937143e-03) * q[i0] * q[i0]
            + (-6.826124e-03) * q[i1] * q[i1] + (2.270333e-06) * q[i2] * q[i2] + (-6.406715e-03) * q[i3] * q[i3] + (6.368734e-03) * q[i4] * q[i4]
            + (1.956306e-05) * q[i5] * q[i5] + (-6.686235e-03) * q[i6] * q[i6] + (6.668476e-03) * q[i7] * q[i7] + (9.495748e-05) * q[i8] * q[i8]
            + (-3.316302e-03) * q[i9] * q[i9] + (3.293977e-03) * q[i10] * q[i10] + (-9.116594e-04) * q[i11] * q[i11] + (1.148862e-03) * q[i12] * q[i12]
            + (4.929370e-03) * q[i15] * q[i15] + (-4.954336e-03) * q[i16] * q[i16] + (-1.456905e-03) * q[i19] * q[i19] + (1.449502e-03) * q[i20] * q[i20]
            + (5.218888e-05) * q[i21] * q[i21] + (-7.309604e-05) * q[i22] * q[i22] + (6.714252e-06) * q[i0] * q[i1] + (-3.677595e-03) * q[i0] * q[i2]
            + (-6.042340e-02) * q[i0] * q[i3] + (-5.138662e-02) * q[i0] * q[i4] + (-8.213849e-03) * q[i0] * q[i5] + (1.570326e-02) * q[i0] * q[i6]
            + (-1.074346e-02) * q[i0] * q[i7] + (8.653143e-03) * q[i0] * q[i8] + (-5.261070e-03) * q[i0] * q[i9] + (-1.270144e-03) * q[i0] * q[i10]
            + (6.303760e-04) * q[i0] * q[i11] + (-3.051713e-04) * q[i0] * q[i12] + (-8.068139e-03) * q[i0] * q[i15] + (-7.002062e-03) * q[i0] * q[i16]
            + (-3.021311e-03) * q[i0] * q[i19] + (-2.484627e-05) * q[i0] * q[i20] + (-6.986138e-03) * q[i0] * q[i21] + (1.111550e-03) * q[i0] * q[i22]
            + (3.762040e-03) * q[i1] * q[i2] + (5.143739e-02) * q[i1] * q[i3] + (6.048093e-02) * q[i1] * q[i4] + (8.116596e-03) * q[i1] * q[i5]
            + (-1.079350e-02) * q[i1] * q[i6] + (1.575324e-02) * q[i1] * q[i7] + (8.756454e-03) * q[i1] * q[i8] + (-1.332474e-03) * q[i1] * q[i9]
            + (-5.206619e-03) * q[i1] * q[i10] + (4.869941e-04) * q[i1] * q[i11] + (-5.109633e-04) * q[i1] * q[i12] + (-6.918457e-03) * q[i1] * q[i15]
            + (-8.223930e-03) * q[i1] * q[i16] + (-1.285072e-05) * q[i1] * q[i19] + (-3.012531e-03) * q[i1] * q[i20] + (-1.117623e-03) * q[i1] * q[i21]
            + (6.876052e-03) * q[i1] * q[i22] + (-1.297012e-02) * q[i2] * q[i3] + (1.287635e-02) * q[i2] * q[i4] + (-2.055079e-05) * q[i2] * q[i5]
            + (1.407401e-02) * q[i2] * q[i6] + (1.409484e-02) * q[i2] * q[i7] + (-1.008699e-03) * q[i2] * q[i8] + (-1.280027e-03) * q[i2] * q[i9]
            + (-1.263228e-03) * q[i2] * q[i10] + (2.007064e-03) * q[i2] * q[i11] + (-1.752413e-03) * q[i2] * q[i12] + (-1.796744e-02) * q[i2] * q[i15]
            + (-1.816952e-02) * q[i2] * q[i16] + (2.002263e-03) * q[i2] * q[i19] + (1.964953e-03) * q[i2] * q[i20] + (-3.676468e-03) * q[i2] * q[i21]
            + (3.596080e-03) * q[i2] * q[i22] + (-6.414578e-05) * q[i3] * q[i4] + (3.490649e-03) * q[i3] * q[i5] + (1.711444e-02) * q[i3] * q[i6]
            + (1.131108e-02) * q[i3] * q[i7] + (-9.928663e-04) * q[i3] * q[i8] + (5.648699e-03) * q[i3] * q[i9] + (1.150907e-02) * q[i3] * q[i10]
            + (-1.591195e-03) * q[i3] * q[i11] + (-2.040305e-03) * q[i3] * q[i12] + (7.821852e-03) * q[i3] * q[i15] + (-2.114044e-04) * q[i3] * q[i16]
            + (-1.242713e-02) * q[i3] * q[i19] + (1.146731e-02) * q[i3] * q[i20] + (1.072986e-02) * q[i3] * q[i21] + (3.548932e-03) * q[i3] * q[i22]
            + (-3.470244e-03) * q[i4] * q[i5] + (1.139101e-02) * q[i4] * q[i6] + (1.688534e-02) * q[i4] * q[i7] + (-9.773458e-04) * q[i4] * q[i8]
            + (1.158554e-02) * q[i4] * q[i9] + (5.640228e-03) * q[i4] * q[i10] + (2.082612e-03) * q[i4] * q[i11] + (1.588879e-03) * q[i4] * q[i12]
            + (-2.693480e-04) * q[i4] * q[i15] + (7.795655e-03) * q[i4] * q[i16] + (1.143341e-02) * q[i4] * q[i19] + (-1.241368e-02) * q[i4] * q[i20]
            + (-3.541732e-03) * q[i4] * q[i21] + (-1.061346e-02) * q[i4] * q[i22] + (2.939048e-02) * q[i5] * q[i6] + (2.907652e-02) * q[i5] * q[i7]
            + (5.116424e-02) * q[i5] * q[i8] + (8.933518e-03) * q[i5] * q[i9] + (8.822487e-03) * q[i5] * q[i10] + (-3.697470e-03) * q[i5] * q[i11]
            + (3.888892e-03) * q[i5] * q[i12] + (-1.199885e-03) * q[i5] * q[i15] + (-1.234830e-03) * q[i5] * q[i16] + (5.678138e-03) * q[i5] * q[i19]
            + (5.708390e-03) * q[i5] * q[i20] + (-3.162355e-03) * q[i5] * q[i21] + (3.131660e-03) * q[i5] * q[i22] + (8.185942e-06) * q[i6] * q[i7]
            + (-8.877688e-03) * q[i6] * q[i8] + (-6.607333e-03) * q[i6] * q[i9] + (1.429997e-03) * q[i6] * q[i10] + (8.965253e-04) * q[i6] * q[i11]
            + (1.560899e-03) * q[i6] * q[i12] + (2.112994e-03) * q[i6] * q[i15] + (1.119554e-02) * q[i6] * q[i16] + (-8.683908e-03) * q[i6] * q[i19]
            + (-5.856227e-03) * q[i6] * q[i20] + (-1.736492e-03) * q[i6] * q[i21] + (2.301098e-03) * q[i6] * q[i22] + (8.758807e-03) * q[i7] * q[i8]
            + (-1.487626e-03) * q[i7] * q[i9] + (6.514480e-03) * q[i7] * q[i10] + (1.726801e-03) * q[i7] * q[i11] + (9.740498e-04) * q[i7] * q[i12]
            + (-1.104024e-02) * q[i7] * q[i15] + (-2.191571e-03) * q[i7] * q[i16] + (5.833061e-03) * q[i7] * q[i19] + (8.629763e-03) * q[i7] * q[i20]
            + (2.335451e-03) * q[i7] * q[i21] + (-1.764857e-03) * q[i7] * q[i22] + (-2.700869e-03) * q[i8] * q[i9] + (2.641316e-03) * q[i8] * q[i10]
            + (-3.691522e-03) * q[i8] * q[i11] + (-3.862373e-03) * q[i8] * q[i12] + (1.443567e-02) * q[i8] * q[i15] + (-1.453026e-02) * q[i8] * q[i16]
            + (4.252995e-03) * q[i8] * q[i19] + (-4.199044e-03) * q[i8] * q[i20] + (3.168813e-03) * q[i8] * q[i21] + (3.195922e-03) * q[i8] * q[i22]
            + (-3.860355e-06) * q[i9] * q[i10] + (-1.635497e-03) * q[i9] * q[i11] + (1.497003e-03) * q[i9] * q[i12] + (-2.044194e-03) * q[i9] * q[i15]
            + (1.961958e-03) * q[i9] * q[i16] + (-4.272644e-03) * q[i9] * q[i19] + (-1.320874e-03) * q[i9] * q[i20] + (-4.451931e-04) * q[i9] * q[i21]
            + (2.513326e-03) * q[i9] * q[i22] + (1.510507e-03) * q[i10] * q[i11] + (-1.588390e-03) * q[i10] * q[i12] + (-1.900916e-03) * q[i10] * q[i15]
            + (2.036227e-03) * q[i10] * q[i16] + (1.303528e-03) * q[i10] * q[i19] + (4.239309e-03) * q[i10] * q[i20] + (2.496257e-03) * q[i10] * q[i21]
            + (-4.460657e-04) * q[i10] * q[i22] + (2.681928e-06) * q[i11] * q[i12] + (2.564447e-03) * q[i11] * q[i15] + (2.571983e-03) * q[i11] * q[i16]
            + (4.182586e-03) * q[i11] * q[i19] + (3.689563e-04) * q[i11] * q[i20] + (4.161639e-03) * q[i11] * q[i21] + (4.512438e-04) * q[i11] * q[i22]
            + (2.559983e-03) * q[i12] * q[i15] + (2.497307e-03) * q[i12] * q[i16] + (3.930770e-04) * q[i12] * q[i19] + (4.151972e-03) * q[i12] * q[i20]
            + (-4.494994e-04) * q[i12] * q[i21] + (-4.286199e-03) * q[i12] * q[i22] + (6.650289e-06) * q[i15] * q[i16] + (4.703209e-03) * q[i15] * q[i19]
            + (-1.427964e-03) * q[i15] * q[i20] + (4.141272e-03) * q[i15] * q[i21] + (3.035838e-04) * q[i15] * q[i22] + (1.448494e-03) * q[i16] * q[i19]
            + (-4.613598e-03) * q[i16] * q[i20] + (3.120678e-04) * q[i16] * q[i21] + (4.203977e-03) * q[i16] * q[i22] + (7.985600e-06) * q[i19] * q[i20]
            + (-5.571742e-03) * q[i19] * q[i21] + (-7.097554e-04) * q[i19] * q[i22] + (-6.905349e-04) * q[i20] * q[i21] + (-5.626379e-03) * q[i20] * q[i22]
            + (-3.384314e-06) * q[i21] * q[i22] + (4.892620e-03) * q[i0] * q[i0] * q[i0] + (-1.963697e-03) * q[i0] * q[i0] * q[i1]
            + (-1.779915e-03) * q[i0] * q[i0] * q[i2] + (-7.543770e-03) * q[i0] * q[i0] * q[i3] + (-2.024621e-03) * q[i0] * q[i0] * q[i4]
            + (1.598918e-04) * q[i0] * q[i0] * q[i5] + (-1.235529e-02) * q[i0] * q[i0] * q[i6] + (-3.272551e-03) * q[i0] * q[i0] * q[i7]
            + (-9.077013e-04) * q[i0] * q[i0] * q[i8] + (-2.269364e-03) * q[i0] * q[i0] * q[i9] + (-2.136750e-03) * q[i0] * q[i0] * q[i10]
            + (-6.309729e-05) * q[i0] * q[i0] * q[i11] + (2.519247e-03) * q[i0] * q[i0] * q[i12] + (6.291848e-04) * q[i0] * q[i0] * q[i15]
            + (-1.373595e-06) * q[i0] * q[i0] * q[i16] + (-6.879212e-05) * q[i0] * q[i0] * q[i19] + (3.310978e-04) * q[i0] * q[i0] * q[i20]
            + (-1.531392e-04) * q[i0] * q[i0] * q[i21] + (4.324533e-04) * q[i0] * q[i0] * q[i22] + (-1.922747e-03) * q[i0] * q[i1] * q[i1]
            + (4.864441e-03) * q[i1] * q[i1] * q[i1] + (-1.710495e-03) * q[i1] * q[i1] * q[i2] + (-2.026980e-03) * q[i1] * q[i1] * q[i3]
            + (-7.512135e-03) * q[i1] * q[i1] * q[i4] + (1.282382e-04) * q[i1] * q[i1] * q[i5] + (3.284873e-03) * q[i1] * q[i1] * q[i6]
            + (1.242606e-02) * q[i1] * q[i1] * q[i7] + (8.847795e-04) * q[i1] * q[i1] * q[i8] + (2.143130e-03) * q[i1] * q[i1] * q[i9]
            + (2.239776e-03) * q[i1] * q[i1] * q[i10] + (2.519575e-03) * q[i1] * q[i1] * q[i11] + (-6.138320e-05) * q[i1] * q[i1] * q[i12]
            + (3.195453e-05) * q[i1] * q[i1] * q[i15] + (-6.272335e-04) * q[i1] * q[i1] * q[i16] + (-3.195466e-04) * q[i1] * q[i1] * q[i19]
            + (1.054223e-04) * q[i1] * q[i1] * q[i20] + (4.403735e-04) * q[i1] * q[i1] * q[i21] + (-1.664605e-04) * q[i1] * q[i1] * q[i22]
            + (3.389921e-03) * q[i0] * q[i2] * q[i2] + (3.420126e-03) * q[i1] * q[i2] * q[i2] + (-2.802144e-03) * q[i2] * q[i2] * q[i2]
            + (-3.984365e-03) * q[i2] * q[i2] * q[i3] + (-3.960318e-03) * q[i2] * q[i2] * q[i4] + (5.685596e-03) * q[i2] * q[i2] * q[i5]
            + (-2.160606e-04) * q[i2] * q[i2] * q[i6] + (2.160540e-04) * q[i2] * q[i2] * q[i7] + (6.674590e-06) * q[i2] * q[i2] * q[i8]
            + (4.256555e-04) * q[i2] * q[i2] * q[i9] + (-4.375921e-04) * q[i2] * q[i2] * q[i10] + (-1.959227e-03) * q[i2] * q[i2] * q[i11]
            + (-1.950016e-03) * q[i2] * q[i2] * q[i12] + (-1.460510e-03) * q[i2] * q[i2] * q[i15] + (1.450669e-03) * q[i2] * q[i2] * q[i16]
            + (-1.100006e-03) * q[i2] * q[i2] * q[i19] + (1.085563e-03) * q[i2] * q[i2] * q[i20] + (5.330095e-04) * q[i2] * q[i2] * q[i21]
            + (5.261333e-04) * q[i2] * q[i2] * q[i22] + (-6.693411e-03) * q[i0] * q[i3] * q[i3] + (-2.267421e-03) * q[i1] * q[i3] * q[i3]
            + (-6.304685e-03) * q[i2] * q[i3] * q[i3] + (3.357610e-03) * q[i3] * q[i3] * q[i3] + (1.782037e-03) * q[i3] * q[i3] * q[i4]
            + (-4.691578e-03) * q[i3] * q[i3] * q[i5] + (5.830212e-03) * q[i3] * q[i3] * q[i6] + (9.049718e-03) * q[i3] * q[i3] * q[i7]
            + (4.594219e-03) * q[i3] * q[i3] * q[i8] + (8.430913e-04) * q[i3] * q[i3] * q[i9] + (-2.710877e-03) * q[i3] * q[i3] * q[i10]
            + (3.105474e-03) * q[i3] * q[i3] * q[i11] + (-3.467011e-04) * q[i3] * q[i3] * q[i12] + (-6.010023e-03) * q[i3] * q[i3] * q[i15]
            + (-2.747068e-03) * q[i3] * q[i3] * q[i16] + (8.414819e-04) * q[i3] * q[i3] * q[i19] + (-2.612225e-04) * q[i3] * q[i3] * q[i20]
            + (-1.876124e-03) * q[i3] * q[i3] * q[i21] + (1.019498e-03) * q[i3] * q[i3] * q[i22] + (-2.229401e-03) * q[i0] * q[i4] * q[i4]
            + (-6.662712e-03) * q[i1] * q[i4] * q[i4] + (-6.295773e-03) * q[i2] * q[i4] * q[i4] + (1.758108e-03) * q[i3] * q[i4] * q[i4]
            + (3.280418e-03) * q[i4] * q[i4] * q[i4] + (-4.701985e-03) * q[i4] * q[i4] * q[i5] + (-9.034951e-03) * q[i4] * q[i4] * q[i6]
            + (-5.931101e-03) * q[i4] * q[i4] * q[i7] + (-4.616553e-03) * q[i4] * q[i4] * q[i8] + (2.690506e-03) * q[i4] * q[i4] * q[i9]
            + (-8.586939e-04) * q[i4] * q[i4] * q[i10] + (-3.515045e-04) * q[i4] * q[i4] * q[i11] + (3.047201e-03) * q[i4] * q[i4] * q[i12]
            + (2.749408e-03) * q[i4] * q[i4] * q[i15] + (6.002643e-03) * q[i4] * q[i4] * q[i16] + (2.218625e-04) * q[i4] * q[i4] * q[i19]
            + (-8.482367e-04) * q[i4] * q[i4] * q[i20] + (1.014072e-03) * q[i4] * q[i4] * q[i21] + (-1.839932e-03) * q[i4] * q[i4] * q[i22]
            + (4.455479e-05) * q[i0] * q[i5] * q[i5] + (-6.971971e-05) * q[i1] * q[i5] * q[i5] + (-1.450116e-02) * q[i2] * q[i5] * q[i5]
            + (3.354428e-03) * q[i3] * q[i5] * q[i5] + (3.387415e-03) * q[i4] * q[i5] * q[i5] + (2.352317e-03) * q[i5] * q[i5] * q[i5]
            + (2.022121e-04) * q[i5] * q[i5] * q[i6] + (-2.879824e-04) * q[i5] * q[i5] * q[i7] + (7.539750e-05) * q[i5] * q[i5] * q[i8]
            + (4.889481e-05) * q[i5] * q[i5] * q[i9] + (-8.529623e-05) * q[i5] * q[i5] * q[i10] + (3.295042e-03) * q[i5] * q[i5] * q[i11]
            + (3.401737e-03) * q[i5] * q[i5] * q[i12] + (9.894973e-04) * q[i5] * q[i5] * q[i15] + (-9.961190e-04) * q[i5] * q[i5] * q[i16]
            + (2.131419e-03) * q[i5] * q[i5] * q[i19] + (-2.149900e-03) * q[i5] * q[i5] * q[i20] + (4.273690e-04) * q[i5] * q[i5] * q[i21]
            + (4.067289e-04) * q[i5] * q[i5] * q[i22] + (-1.078988e-02) * q[i0] * q[i6] * q[i6] + (6.680487e-03) * q[i1] * q[i6] * q[i6]
            + (-4.822411e-03) * q[i2] * q[i6] * q[i6] + (-5.211335e-03) * q[i3] * q[i6] * q[i6] + (-3.818273e-03) * q[i4] * q[i6] * q[i6]
            + (8.587558e-03) * q[i5] * q[i6] * q[i6] + (-3.909764e-03) * q[i6] * q[i6] * q[i6] + (1.289841e-03) * q[i6] * q[i6] * q[i7]
            + (7.130810e-03) * q[i6] * q[i6] * q[i8] + (-2.098500e-04) * q[i6] * q[i6] * q[i9] + (2.881432e-03) * q[i6] * q[i6] * q[i10]
            + (-4.684032e-03) * q[i6] * q[i6] * q[i11] + (1.484473e-03) * q[i6] * q[i6] * q[i12] + (2.561257e-03) * q[i6] * q[i6] * q[i15]
            + (-1.554538e-03) * q[i6] * q[i6] * q[i16] + (2.031403e-04) * q[i6] * q[i6] * q[i19] + (-8.974494e-04) * q[i6] * q[i6] * q[i20]
            + (-1.225484e-03) * q[i6] * q[i6] * q[i21] + (-1.812007e-03) * q[i6] * q[i6] * q[i22] + (6.715776e-03) * q[i0] * q[i7] * q[i7]
            + (-1.079080e-02) * q[i1] * q[i7] * q[i7] + (-4.756895e-03) * q[i2] * q[i7] * q[i7] + (-3.817495e-03) * q[i3] * q[i7] * q[i7]
            + (-5.239347e-03) * q[i4] * q[i7] * q[i7] + (8.560408e-03) * q[i5] * q[i7] * q[i7] + (-1.309745e-03) * q[i6] * q[i7] * q[i7]
            + (3.920256e-03) * q[i7] * q[i7] * q[i7] + (-7.145583e-03) * q[i7] * q[i7] * q[i8] + (-2.885807e-03) * q[i7] * q[i7] * q[i9]
            + (2.142158e-04) * q[i7] * q[i7] * q[i10] + (1.484495e-03) * q[i7] * q[i7] * q[i11] + (-4.665546e-03) * q[i7] * q[i7] * q[i12]
            + (1.562721e-03) * q[i7] * q[i7] * q[i15] + (-2.543950e-03) * q[i7] * q[i7] * q[i16] + (8.816005e-04) * q[i7] * q[i7] * q[i19]
            + (-2.149406e-04) * q[i7] * q[i7] * q[i20] + (-1.779634e-03) * q[i7] * q[i7] * q[i21] + (-1.211107e-03) * q[i7] * q[i7] * q[i22]
            + (8.763863e-04) * q[i0] * q[i8] * q[i8] + (8.621596e-04) * q[i1] * q[i8] * q[i8] + (6.311364e-03) * q[i2] * q[i8] * q[i8]
            + (-4.006531e-03) * q[i3] * q[i8] * q[i8] + (-4.035328e-03) * q[i4] * q[i8] * q[i8] + (6.413173e-03) * q[i5] * q[i8] * q[i8]
            + (1.706365e-03) * q[i6] * q[i8] * q[i8] + (-1.682221e-03) * q[i7] * q[i8] * q[i8] + (-2.481958e-05) * q[i8] * q[i8] * q[i8]
            + (3.472965e-04) * q[i8] * q[i8] * q[i9] + (-3.438137e-04) * q[i8] * q[i8] * q[i10] + (-5.372922e-03) * q[i8] * q[i8] * q[i11]
            + (-5.461197e-03) * q[i8] * q[i8] * q[i12] + (-1.147863e-03) * q[i8] * q[i8] * q[i15] + (1.224891e-03) * q[i8] * q[i8] * q[i16]
            + (-1.183642e-03) * q[i8] * q[i8] * q[i19] + (1.153215e-03) * q[i8] * q[i8] * q[i20] + (1.220470e-03) * q[i8] * q[i8] * q[i21]
            + (1.243642e-03) * q[i8] * q[i8] * q[i22] + (3.571000e-03) * q[i0] * q[i9] * q[i9] + (2.252412e-04) * q[i1] * q[i9] * q[i9]
            + (9.906023e-04) * q[i2] * q[i9] * q[i9] + (-1.039539e-03) * q[i3] * q[i9] * q[i9] + (-1.127005e-03) * q[i4] * q[i9] * q[i9]
            + (-1.496355e-03) * q[i5] * q[i9] * q[i9] + (-1.563730e-03) * q[i6] * q[i9] * q[i9] + (-1.879608e-04) * q[i7] * q[i9] * q[i9]
            + (-5.020887e-04) * q[i8] * q[i9] * q[i9] + (-7.815466e-04) * q[i9] * q[i9] * q[i9] + (-4.161969e-04) * q[i9] * q[i9] * q[i10]
            + (7.893895e-04) * q[i9] * q[i9] * q[i11] + (-3.654373e-04) * q[i9] * q[i9] * q[i12] + (-1.776470e-04) * q[i9] * q[i9] * q[i15]
            + (-8.181243e-04) * q[i9] * q[i9] * q[i16] + (5.688618e-04) * q[i9] * q[i9] * q[i19] + (5.050609e-04) * q[i9] * q[i9] * q[i20]
            + (1.956634e-04) * q[i9] * q[i9] * q[i21] + (-7.894434e-04) * q[i9] * q[i9] * q[i22] + (2.137082e-04) * q[i0] * q[i10] * q[i10]
            + (3.526691e-03) * q[i1] * q[i10] * q[i10] + (9.744991e-04) * q[i2] * q[i10] * q[i10] + (-1.109231e-03) * q[i3] * q[i10] * q[i10]
            + (-1.073550e-03) * q[i4] * q[i10] * q[i10] + (-1.465258e-03) * q[i5] * q[i10] * q[i10] + (1.977798e-04) * q[i6] * q[i10] * q[i10]
            + (1.547509e-03) * q[i7] * q[i10] * q[i10] + (5.061291e-04) * q[i8] * q[i10] * q[i10] + (4.154889e-04) * q[i9] * q[i10] * q[i10]
            + (7.701880e-04) * q[i10] * q[i10] * q[i10] + (-3.646132e-04) * q[i10] * q[i10] * q[i11] + (7.796672e-04) * q[i10] * q[i10] * q[i12]
            + (8.040850e-04) * q[i10] * q[i10] * q[i15] + (1.798451e-04) * q[i10] * q[i10] * q[i16] + (-5.029924e-04) * q[i10] * q[i10] * q[i19]
            + (-5.685368e-04) * q[i10] * q[i10] * q[i20] + (-7.848734e-04) * q[i10] * q[i10] * q[i21] + (1.943379e-04) * q[i10] * q[i10] * q[i22]
            + (-3.013394e-04) * q[i0] * q[i11] * q[i11] + (-1.769956e-05) * q[i1] * q[i11] * q[i11] + (-2.203007e-03) * q[i2] * q[i11] * q[i11]
            + (-3.986162e-04) * q[i3] * q[i11] * q[i11] + (-2.803565e-03) * q[i4] * q[i11] * q[i11] + (-4.641605e-04) * q[i5] * q[i11] * q[i11]
            + (2.021229e-03) * q[i6] * q[i11] * q[i11] + (-2.718147e-03) * q[i7] * q[i11] * q[i11] + (-2.565567e-03) * q[i8] * q[i11] * q[i11]
            + (3.303996e-05) * q[i9] * q[i11] * q[i11] + (-8.027005e-04) * q[i10] * q[i11] * q[i11] + (-8.028001e-04) * q[i11] * q[i11] * q[i11]
            + (5.829736e-04) * q[i11] * q[i11] * q[i12] + (1.444886e-03) * q[i11] * q[i11] * q[i15] + (-5.429432e-04) * q[i11] * q[i11] * q[i16]
            + (-1.210977e-03) * q[i11] * q[i11] * q[i19] + (-2.343149e-05) * q[i11] * q[i11] * q[i20] + (-1.088728e-03) * q[i11] * q[i11] * q[i21]
            + (2.058726e-04) * q[i11] * q[i11] * q[i22] + (-4.926891e-06) * q[i0] * q[i12] * q[i12] + (-3.530106e-04) * q[i1] * q[i12] * q[i12]
            + (-2.230185e-03) * q[i2] * q[i12] * q[i12] + (-2.886119e-03) * q[i3] * q[i12] * q[i12] + (-3.361134e-04) * q[i4] * q[i12] * q[i12]
            + (-4.418924e-04) * q[i5] * q[i12] * q[i12] + (2.741810e-03) * q[i6] * q[i12] * q[i12] + (-1.990822e-03) * q[i7] * q[i12] * q[i12]
            + (2.620844e-03) * q[i8] * q[i12] * q[i12] + (8.229866e-04) * q[i9] * q[i12] * q[i12] + (-1.848996e-05) * q[i10] * q[i12] * q[i12]
            + (5.870360e-04) * q[i11] * q[i12] * q[i12] + (-8.365510e-04) * q[i12] * q[i12] * q[i12] + (5.440826e-04) * q[i12] * q[i12] * q[i15]
            + (-1.440262e-03) * q[i12] * q[i12] * q[i16] + (4.530601e-06) * q[i12] * q[i12] * q[i19] + (1.180369e-03) * q[i12] * q[i12] * q[i20]
            + (2.075158e-04) * q[i12] * q[i12] * q[i21] + (-1.071907e-03) * q[i12] * q[i12] * q[i22] + (-1.610159e-03) * q[i0] * q[i15] * q[i15]
            + (-2.590570e-03) * q[i1] * q[i15] * q[i15] + (-4.875913e-03) * q[i2] * q[i15] * q[i15] + (-1.798421e-03) * q[i3] * q[i15] * q[i15]
            + (1.108226e-03) * q[i4] * q[i15] * q[i15] + (-1.275408e-03) * q[i5] * q[i15] * q[i15] + (1.350029e-03) * q[i6] * q[i15] * q[i15]
            + (-1.718359e-03) * q[i7] * q[i15] * q[i15] + (-1.623978e-03) * q[i8] * q[i15] * q[i15] + (-9.476405e-05) * q[i9] * q[i15] * q[i15]
            + (4.446822e-04) * q[i10] * q[i15] * q[i15] + (-5.868728e-03) * q[i11] * q[i15] * q[i15] + (4.680595e-04) * q[i12] * q[i15] * q[i15]
            + (-2.844297e-04) * q[i15] * q[i15] * q[i15] + (-1.891718e-04) * q[i15] * q[i15] * q[i16] + (8.950920e-04) * q[i15] * q[i15] * q[i19]
            + (-3.238596e-05) * q[i15] * q[i15] * q[i20] + (1.447443e-03) * q[i15] * q[i15] * q[i21] + (-2.898229e-04) * q[i15] * q[i15] * q[i22]
            + (-2.629202e-03) * q[i0] * q[i16] * q[i16] + (-1.635661e-03) * q[i1] * q[i16] * q[i16] + (-4.926560e-03) * q[i2] * q[i16] * q[i16]
            + (1.128739e-03) * q[i3] * q[i16] * q[i16] + (-1.787982e-03) * q[i4] * q[i16] * q[i16] + (-1.308459e-03) * q[i5] * q[i16] * q[i16]
            + (1.732938e-03) * q[i6] * q[i16] * q[i16] + (-1.365205e-03) * q[i7] * q[i16] * q[i16] + (1.643932e-03) * q[i8] * q[i16] * q[i16]
            + (-4.486444e-04) * q[i9] * q[i16] * q[i16] + (9.360250e-05) * q[i10] * q[i16] * q[i16] + (4.659210e-04) * q[i11] * q[i16] * q[i16]
            + (-5.938069e-03) * q[i12] * q[i16] * q[i16] + (1.895777e-04) * q[i15] * q[i16] * q[i16] + (2.887173e-04) * q[i16] * q[i16] * q[i16]
            + (3.629187e-05) * q[i16] * q[i16] * q[i19] + (-9.031755e-04) * q[i16] * q[i16] * q[i20] + (-2.888105e-04) * q[i16] * q[i16] * q[i21]
            + (1.464243e-03) * q[i16] * q[i16] * q[i22] + (9.614484e-04) * q[i0] * q[i19] * q[i19] + (-8.908750e-04) * q[i1] * q[i19] * q[i19]
            + (-3.407559e-04) * q[i2] * q[i19] * q[i19] + (-1.618046e-03) * q[i3] * q[i19] * q[i19] + (9.882835e-04) * q[i4] * q[i19] * q[i19]
            + (5.672238e-04) * q[i5] * q[i19] * q[i19] + (1.277015e-03) * q[i6] * q[i19] * q[i19] + (-1.300125e-03) * q[i7] * q[i19] * q[i19]
            + (-2.297524e-04) * q[i8] * q[i19] * q[i19] + (3.851531e-04) * q[i9] * q[i19] * q[i19] + (2.144442e-04) * q[i10] * q[i19] * q[i19]
            + (6.159593e-04) * q[i11] * q[i19] * q[i19] + (-1.264439e-04) * q[i12] * q[i19] * q[i19] + (3.317757e-04) * q[i15] * q[i19] * q[i19]
            + (-1.910473e-04) * q[i16] * q[i19] * q[i19] + (-1.966820e-04) * q[i19] * q[i19] * q[i19] + (-3.966958e-04) * q[i19] * q[i19] * q[i20]
            + (-2.099350e-03) * q[i19] * q[i19] * q[i21] + (-1.815490e-05) * q[i19] * q[i19] * q[i22] + (-8.809152e-04) * q[i0] * q[i20] * q[i20]
            + (9.579766e-04) * q[i1] * q[i20] * q[i20] + (-3.275157e-04) * q[i2] * q[i20] * q[i20] + (9.940037e-04) * q[i3] * q[i20] * q[i20]
            + (-1.590706e-03) * q[i4] * q[i20] * q[i20] + (5.569911e-04) * q[i5] * q[i20] * q[i20] + (1.285111e-03) * q[i6] * q[i20] * q[i20]
            + (-1.281165e-03) * q[i7] * q[i20] * q[i20] + (2.460478e-04) * q[i8] * q[i20] * q[i20] + (-2.195414e-04) * q[i9] * q[i20] * q[i20]
            + (-3.820262e-04) * q[i10] * q[i20] * q[i20] + (-1.191432e-04) * q[i11] * q[i20] * q[i20] + (6.084688e-04) * q[i12] * q[i20] * q[i20]
            + (1.876722e-04) * q[i15] * q[i20] * q[i20] + (-3.153356e-04) * q[i16] * q[i20] * q[i20] + (3.836269e-04) * q[i19] * q[i20] * q[i20]
            + (1.954735e-04) * q[i20] * q[i20] * q[i20] + (-1.573420e-05) * q[i20] * q[i20] * q[i21] + (-2.097085e-03) * q[i20] * q[i20] * q[i22]
            + (-1.759633e-03) * q[i0] * q[i21] * q[i21] + (-3.901743e-04) * q[i1] * q[i21] * q[i21] + (-2.045029e-03) * q[i2] * q[i21] * q[i21]
            + (-7.426308e-04) * q[i3] * q[i21] * q[i21] + (1.312973e-03) * q[i4] * q[i21] * q[i21] + (-3.217971e-04) * q[i5] * q[i21] * q[i21]
            + (1.607276e-03) * q[i6] * q[i21] * q[i21] + (-1.741215e-03) * q[i7] * q[i21] * q[i21] + (4.810772e-04) * q[i8] * q[i21] * q[i21]
            + (3.766099e-04) * q[i9] * q[i21] * q[i21] + (6.369769e-05) * q[i10] * q[i21] * q[i21] + (-6.023134e-04) * q[i11] * q[i21] * q[i21]
            + (-4.094983e-04) * q[i12] * q[i21] * q[i21] + (2.459007e-04) * q[i15] * q[i21] * q[i21] + (-1.195869e-04) * q[i16] * q[i21] * q[i21]
            + (-1.030272e-03) * q[i19] * q[i21] * q[i21] + (1.407896e-04) * q[i20] * q[i21] * q[i21] + (-5.500735e-04) * q[i21] * q[i21] * q[i21]
            + (4.184537e-05) * q[i21] * q[i21] * q[i22] + (-4.010723e-04) * q[i0] * q[i22] * q[i22] + (-1.742359e-03) * q[i1] * q[i22] * q[i22]
            + (-2.031400e-03) * q[i2] * q[i22] * q[i22] + (1.324916e-03) * q[i3] * q[i22] * q[i22] + (-7.465614e-04) * q[i4] * q[i22] * q[i22]
            + (-3.075707e-04) * q[i5] * q[i22] * q[i22] + (1.739386e-03) * q[i6] * q[i22] * q[i22] + (-1.603073e-03) * q[i7] * q[i22] * q[i22]
            + (-4.754923e-04) * q[i8] * q[i22] * q[i22] + (-6.016814e-05) * q[i9] * q[i22] * q[i22] + (-3.702790e-04) * q[i10] * q[i22] * q[i22]
            + (-4.069585e-04) * q[i11] * q[i22] * q[i22] + (-5.864783e-04) * q[i12] * q[i22] * q[i22] + (1.223636e-04) * q[i15] * q[i22] * q[i22]
            + (-2.405727e-04) * q[i16] * q[i22] * q[i22] + (-1.440290e-04) * q[i19] * q[i22] * q[i22] + (1.047033e-03) * q[i20] * q[i22] * q[i22]
            + (3.682205e-05) * q[i21] * q[i22] * q[i22] + (-5.527782e-04) * q[i22] * q[i22] * q[i22] + (-1.034438e-03) * q[i0] * q[i1] * q[i2]
            + (1.107523e-03) * q[i0] * q[i1] * q[i3] + (1.092826e-03) * q[i0] * q[i1] * q[i4] + (5.936415e-03) * q[i0] * q[i1] * q[i5]
            + (-3.535111e-04) * q[i0] * q[i1] * q[i6] + (2.757222e-04) * q[i0] * q[i1] * q[i7] + (5.311938e-05) * q[i0] * q[i1] * q[i8]
            + (-1.650176e-03) * q[i0] * q[i1] * q[i9] + (1.641943e-03) * q[i0] * q[i1] * q[i10] + (-6.978984e-04) * q[i0] * q[i1] * q[i11]
            + (-6.819984e-04) * q[i0] * q[i1] * q[i12] + (8.307751e-05) * q[i0] * q[i1] * q[i15] + (-8.990329e-05) * q[i0] * q[i1] * q[i16]
            + (2.970379e-04) * q[i0] * q[i1] * q[i19] + (-3.106547e-04) * q[i0] * q[i1] * q[i20] + (1.096464e-03) * q[i0] * q[i1] * q[i21]
            + (1.088216e-03) * q[i0] * q[i1] * q[i22] + (2.092116e-03) * q[i0] * q[i2] * q[i3] + (-5.079142e-03) * q[i0] * q[i2] * q[i4]
            + (7.131349e-03) * q[i0] * q[i2] * q[i5] + (-8.584065e-03) * q[i0] * q[i2] * q[i6] + (-2.635861e-03) * q[i0] * q[i2] * q[i7]
            + (5.935676e-03) * q[i0] * q[i2] * q[i8] + (-1.593483e-03) * q[i0] * q[i2] * q[i9] + (-1.246070e-03) * q[i0] * q[i2] * q[i10]
            + (5.106428e-04) * q[i0] * q[i2] * q[i11] + (-8.160733e-04) * q[i0] * q[i2] * q[i12] + (-8.816348e-04) * q[i0] * q[i2] * q[i15]
            + (-1.612249e-03) * q[i0] * q[i2] * q[i16] + (-1.778900e-03) * q[i0] * q[i2] * q[i19] + (3.204858e-04) * q[i0] * q[i2] * q[i20]
            + (-7.543829e-05) * q[i0] * q[i2] * q[i21] + (1.559269e-03) * q[i0] * q[i2] * q[i22] + (1.157804e-02) * q[i0] * q[i3] * q[i4]
            + (-1.175620e-02) * q[i0] * q[i3] * q[i5] + (-1.011579e-02) * q[i0] * q[i3] * q[i6] + (3.106750e-03) * q[i0] * q[i3] * q[i7]
            + (5.752914e-03) * q[i0] * q[i3] * q[i8] + (6.880723e-03) * q[i0] * q[i3] * q[i9] + (4.104927e-04) * q[i0] * q[i3] * q[i10]
            + (-1.905806e-03) * q[i0] * q[i3] * q[i11] + (-4.130384e-03) * q[i0] * q[i3] * q[i12] + (3.737213e-03) * q[i0] * q[i3] * q[i15]
            + (-4.549942e-03) * q[i0] * q[i3] * q[i16] + (-7.239892e-04) * q[i0] * q[i3] * q[i19] + (-4.420090e-04) * q[i0] * q[i3] * q[i20]
            + (2.352586e-03) * q[i0] * q[i3] * q[i21] + (-2.251364e-03) * q[i0] * q[i3] * q[i22] + (3.113038e-04) * q[i0] * q[i4] * q[i5]
            + (2.345123e-03) * q[i0] * q[i4] * q[i6] + (-3.091155e-03) * q[i0] * q[i4] * q[i7] + (2.799478e-03) * q[i0] * q[i4] * q[i8]
            + (5.130394e-03) * q[i0] * q[i4] * q[i9] + (6.354973e-03) * q[i0] * q[i4] * q[i10] + (-4.735293e-04) * q[i0] * q[i4] * q[i11]
            + (3.386984e-03) * q[i0] * q[i4] * q[i12] + (-3.231341e-03) * q[i0] * q[i4] * q[i15] + (-1.271616e-04) * q[i0] * q[i4] * q[i16]
            + (-1.119306e-05) * q[i0] * q[i4] * q[i19] + (-5.171096e-04) * q[i0] * q[i4] * q[i20] + (-2.774221e-03) * q[i0] * q[i4] * q[i21]
            + (-7.930453e-04) * q[i0] * q[i4] * q[i22] + (2.328631e-03) * q[i0] * q[i5] * q[i6] + (-2.363745e-04) * q[i0] * q[i5] * q[i7]
            + (3.065318e-03) * q[i0] * q[i5] * q[i8] + (2.392677e-03) * q[i0] * q[i5] * q[i9] + (1.078840e-03) * q[i0] * q[i5] * q[i10]
            + (1.322023e-03) * q[i0] * q[i5] * q[i11] + (1.282937e-03) * q[i0] * q[i5] * q[i12] + (3.452045e-03) * q[i0] * q[i5] * q[i15]
            + (-2.146369e-03) * q[i0] * q[i5] * q[i16] + (-1.380481e-03) * q[i0] * q[i5] * q[i19] + (-5.019998e-04) * q[i0] * q[i5] * q[i20]
            + (-2.871135e-03) * q[i0] * q[i5] * q[i21] + (-8.908884e-04) * q[i0] * q[i5] * q[i22] + (7.590054e-03) * q[i0] * q[i6] * q[i7]
            + (3.540452e-03) * q[i0] * q[i6] * q[i8] + (-6.566775e-03) * q[i0] * q[i6] * q[i9] + (6.778875e-03) * q[i0] * q[i6] * q[i10]
            + (2.362114e-03) * q[i0] * q[i6] * q[i11] + (1.859035e-03) * q[i0] * q[i6] * q[i12] + (-3.302023e-04) * q[i0] * q[i6] * q[i15]
            + (2.531603e-03) * q[i0] * q[i6] * q[i16] + (-1.968210e-03) * q[i0] * q[i6] * q[i19] + (-7.278929e-04) * q[i0] * q[i6] * q[i20]
            + (-4.359618e-04) * q[i0] * q[i6] * q[i21] + (1.950349e-03) * q[i0] * q[i6] * q[i22] + (-1.160031e-03) * q[i0] * q[i7] * q[i8]
            + (2.473974e-04) * q[i0] * q[i7] * q[i9] + (5.886241e-03) * q[i0] * q[i7] * q[i10] + (-8.944924e-04) * q[i0] * q[i7] * q[i11]
            + (-6.621534e-04) * q[i0] * q[i7] * q[i12] + (1.730123e-03) * q[i0] * q[i7] * q[i15] + (-2.987341e-03) * q[i0] * q[i7] * q[i16]
            + (-2.717691e-03) * q[i0] * q[i7] * q[i19] + (-1.157077e-04) * q[i0] * q[i7] * q[i20] + (4.351924e-04) * q[i0] * q[i7] * q[i21]
            + (-6.437164e-04) * q[i0] * q[i7] * q[i22] + (3.435307e-04) * q[i0] * q[i8] * q[i9] + (-1.449731e-03) * q[i0] * q[i8] * q[i10]
            + (-1.501626e-04) * q[i0] * q[i8] * q[i11] + (3.196186e-04) * q[i0] * q[i8] * q[i12] + (-1.777304e-03) * q[i0] * q[i8] * q[i15]
            + (-1.266048e-03) * q[i0] * q[i8] * q[i16] + (-9.278645e-04) * q[i0] * q[i8] * q[i19] + (2.572781e-03) * q[i0] * q[i8] * q[i20]
            + (5.334901e-04) * q[i0] * q[i8] * q[i21] + (-1.699471e-03) * q[i0] * q[i8] * q[i22] + (1.333336e-03) * q[i0] * q[i9] * q[i10]
            + (9.486425e-06) * q[i0] * q[i9] * q[i11] + (-4.151017e-04) * q[i0] * q[i9] * q[i12] + (1.080065e-04) * q[i0] * q[i9] * q[i15]
            + (-8.717527e-04) * q[i0] * q[i9] * q[i16] + (1.619438e-03) * q[i0] * q[i9] * q[i19] + (-3.476660e-04) * q[i0] * q[i9] * q[i20]
            + (1.984431e-04) * q[i0] * q[i9] * q[i21] + (4.594173e-05) * q[i0] * q[i9] * q[i22] + (-1.415883e-03) * q[i0] * q[i10] * q[i11]
            + (1.398713e-03) * q[i0] * q[i10] * q[i12] + (-1.365183e-03) * q[i0] * q[i10] * q[i15] + (-2.779088e-04) * q[i0] * q[i10] * q[i16]
            + (1.148780e-03) * q[i0] * q[i10] * q[i19] + (-6.231768e-04) * q[i0] * q[i10] * q[i20] + (6.428626e-04) * q[i0] * q[i10] * q[i21]
            + (-1.778341e-04) * q[i0] * q[i10] * q[i22] + (5.165199e-04) * q[i0] * q[i11] * q[i12] + (-1.382556e-03) * q[i0] * q[i11] * q[i15]
            + (-2.685371e-04) * q[i0] * q[i11] * q[i16] + (1.617007e-03) * q[i0] * q[i11] * q[i19] + (-1.578691e-03) * q[i0] * q[i11] * q[i20]
            + (-8.476260e-04) * q[i0] * q[i11] * q[i21] + (-1.043071e-03) * q[i0] * q[i11] * q[i22] + (1.106490e-03) * q[i0] * q[i12] * q[i15]
            + (-3.871843e-03) * q[i0] * q[i12] * q[i16] + (6.751019e-04) * q[i0] * q[i12] * q[i19] + (2.247609e-03) * q[i0] * q[i12] * q[i20]
            + (1.695129e-03) * q[i0] * q[i12] * q[i21] + (6.040034e-04) * q[i0] * q[i12] * q[i22] + (8.868801e-05) * q[i0] * q[i15] * q[i16]
            + (1.068404e-03) * q[i0] * q[i15] * q[i19] + (-6.770629e-05) * q[i0] * q[i15] * q[i20] + (-2.242571e-03) * q[i0] * q[i15] * q[i21]
            + (2.860531e-04) * q[i0] * q[i15] * q[i22] + (1.225508e-03) * q[i0] * q[i16] * q[i19] + (-9.726985e-04) * q[i0] * q[i16] * q[i20]
            + (8.158969e-05) * q[i0] * q[i16] * q[i21] + (2.233752e-04) * q[i0] * q[i16] * q[i22] + (-1.029540e-04) * q[i0] * q[i19] * q[i20]
            + (7.949083e-04) * q[i0] * q[i19] * q[i21] + (-6.577718e-04) * q[i0] * q[i19] * q[i22] + (-8.140101e-04) * q[i0] * q[i20] * q[i21]
            + (1.086136e-04) * q[i0] * q[i20] * q[i22] + (4.544889e-04) * q[i0] * q[i21] * q[i22] + (-5.128699e-03) * q[i1] * q[i2] * q[i3]
            + (2.120396e-03) * q[i1] * q[i2] * q[i4] + (7.188995e-03) * q[i1] * q[i2] * q[i5] + (2.627176e-03) * q[i1] * q[i2] * q[i6]
            + (8.588029e-03) * q[i1] * q[i2] * q[i7] + (-5.944304e-03) * q[i1] * q[i2] * q[i8] + (1.251727e-03) * q[i1] * q[i2] * q[i9]
            + (1.589570e-03) * q[i1] * q[i2] * q[i10] + (-8.452529e-04) * q[i1] * q[i2] * q[i11] + (5.137758e-04) * q[i1] * q[i2] * q[i12]
            + (1.617850e-03) * q[i1] * q[i2] * q[i15] + (8.778651e-04) * q[i1] * q[i2] * q[i16] + (-3.047581e-04) * q[i1] * q[i2] * q[i19]
            + (1.812038e-03) * q[i1] * q[i2] * q[i20] + (1.553031e-03) * q[i1] * q[i2] * q[i21] + (-1.092656e-04) * q[i1] * q[i2] * q[i22]
            + (1.164925e-02) * q[i1] * q[i3] * q[i4] + (3.613486e-04) * q[i1] * q[i3] * q[i5] + (3.009775e-03) * q[i1] * q[i3] * q[i6]
            + (-2.379827e-03) * q[i1] * q[i3] * q[i7] + (-2.802561e-03) * q[i1] * q[i3] * q[i8] + (-6.364877e-03) * q[i1] * q[i3] * q[i9]
            + (-5.111334e-03) * q[i1] * q[i3] * q[i10] + (3.375875e-03) * q[i1] * q[i3] * q[i11] + (-5.158804e-04) * q[i1] * q[i3] * q[i12]
            + (1.375503e-04) * q[i1] * q[i3] * q[i15] + (3.280968e-03) * q[i1] * q[i3] * q[i16] + (5.168160e-04) * q[i1] * q[i3] * q[i19]
            + (-2.998582e-05) * q[i1] * q[i3] * q[i20] + (-7.713824e-04) * q[i1] * q[i3] * q[i21] + (-2.770689e-03) * q[i1] * q[i3] * q[i22]
            + (-1.174509e-02) * q[i1] * q[i4] * q[i5] + (-3.075405e-03) * q[i1] * q[i4] * q[i6] + (1.026329e-02) * q[i1] * q[i4] * q[i7]
            + (-5.652658e-03) * q[i1] * q[i4] * q[i8] + (-4.481237e-04) * q[i1] * q[i4] * q[i9] + (-6.823310e-03) * q[i1] * q[i4] * q[i10]
            + (-4.153900e-03) * q[i1] * q[i4] * q[i11] + (-1.935043e-03) * q[i1] * q[i4] * q[i12] + (4.452672e-03) * q[i1] * q[i4] * q[i15]
            + (-3.699263e-03) * q[i1] * q[i4] * q[i16] + (4.557667e-04) * q[i1] * q[i4] * q[i19] + (6.645825e-04) * q[i1] * q[i4] * q[i20]
            + (-2.249974e-03) * q[i1] * q[i4] * q[i21] + (2.335218e-03) * q[i1] * q[i4] * q[i22] + (2.449185e-04) * q[i1] * q[i5] * q[i6]
            + (-2.324987e-03) * q[i1] * q[i5] * q[i7] + (-3.085534e-03) * q[i1] * q[i5] * q[i8] + (-1.058509e-03) * q[i1] * q[i5] * q[i9]
            + (-2.381350e-03) * q[i1] * q[i5] * q[i10] + (1.337867e-03) * q[i1] * q[i5] * q[i11] + (1.235463e-03) * q[i1] * q[i5] * q[i12]
            + (2.119109e-03) * q[i1] * q[i5] * q[i15] + (-3.452865e-03) * q[i1] * q[i5] * q[i16] + (5.274652e-04) * q[i1] * q[i5] * q[i19]
            + (1.376363e-03) * q[i1] * q[i5] * q[i20] + (-8.784839e-04) * q[i1] * q[i5] * q[i21] + (-2.824797e-03) * q[i1] * q[i5] * q[i22]
            + (7.598896e-03) * q[i1] * q[i6] * q[i7] + (-1.137300e-03) * q[i1] * q[i6] * q[i8] + (5.926933e-03) * q[i1] * q[i6] * q[i9]
            + (2.451999e-04) * q[i1] * q[i6] * q[i10] + (7.141237e-04) * q[i1] * q[i6] * q[i11] + (8.925012e-04) * q[i1] * q[i6] * q[i12]
            + (-2.981604e-03) * q[i1] * q[i6] * q[i15] + (1.727913e-03) * q[i1] * q[i6] * q[i16] + (-1.552690e-04) * q[i1] * q[i6] * q[i19]
            + (-2.713295e-03) * q[i1] * q[i6] * q[i20] + (6.585911e-04) * q[i1] * q[i6] * q[i21] + (-4.110103e-04) * q[i1] * q[i6] * q[i22]
            + (3.540632e-03) * q[i1] * q[i7] * q[i8] + (6.815199e-03) * q[i1] * q[i7] * q[i9] + (-6.555638e-03) * q[i1] * q[i7] * q[i10]
            + (-1.815133e-03) * q[i1] * q[i7] * q[i11] + (-2.364231e-03) * q[i1] * q[i7] * q[i12] + (2.516036e-03) * q[i1] * q[i7] * q[i15]
            + (-2.919010e-04) * q[i1] * q[i7] * q[i16] + (-7.084220e-04) * q[i1] * q[i7] * q[i19] + (-1.945346e-03) * q[i1] * q[i7] * q[i20]
            + (-1.962824e-03) * q[i1] * q[i7] * q[i21] + (4.281154e-04) * q[i1] * q[i7] * q[i22] + (-1.458352e-03) * q[i1] * q[i8] * q[i9]
            + (3.259798e-04) * q[i1] * q[i8] * q[i10] + (-3.038879e-04) * q[i1] * q[i8] * q[i11] + (1.257303e-04) * q[i1] * q[i8] * q[i12]
            + (-1.220725e-03) * q[i1] * q[i8] * q[i15] + (-1.803216e-03) * q[i1] * q[i8] * q[i16] + (2.591051e-03) * q[i1] * q[i8] * q[i19]
            + (-9.210163e-04) * q[i1] * q[i8] * q[i20] + (1.685769e-03) * q[i1] * q[i8] * q[i21] + (-5.526813e-04) * q[i1] * q[i8] * q[i22]
            + (1.366859e-03) * q[i1] * q[i9] * q[i10] + (-1.395787e-03) * q[i1] * q[i9] * q[i11] + (1.418468e-03) * q[i1] * q[i9] * q[i12]
            + (-2.835336e-04) * q[i1] * q[i9] * q[i15] + (-1.345912e-03) * q[i1] * q[i9] * q[i16] + (-6.331721e-04) * q[i1] * q[i9] * q[i19]
            + (1.136149e-03) * q[i1] * q[i9] * q[i20] + (1.872351e-04) * q[i1] * q[i9] * q[i21] + (-6.430545e-04) * q[i1] * q[i9] * q[i22]
            + (4.045033e-04) * q[i1] * q[i10] * q[i11] + (-1.472401e-05) * q[i1] * q[i10] * q[i12] + (-8.608620e-04) * q[i1] * q[i10] * q[i15]
            + (1.288937e-04) * q[i1] * q[i10] * q[i16] + (-3.646995e-04) * q[i1] * q[i10] * q[i19] + (1.601052e-03) * q[i1] * q[i10] * q[i20]
            + (-4.788341e-05) * q[i1] * q[i10] * q[i21] + (-1.747739e-04) * q[i1] * q[i10] * q[i22] + (5.198115e-04) * q[i1] * q[i11] * q[i12]
            + (3.872995e-03) * q[i1] * q[i11] * q[i15] + (-1.095867e-03) * q[i1] * q[i11] * q[i16] + (-2.231571e-03) * q[i1] * q[i11] * q[i19]
            + (-6.580470e-04) * q[i1] * q[i11] * q[i20] + (6.031111e-04) * q[i1] * q[i11] * q[i21] + (1.702132e-03) * q[i1] * q[i11] * q[i22]
            + (2.860590e-04) * q[i1] * q[i12] * q[i15] + (1.377429e-03) * q[i1] * q[i12] * q[i16] + (1.575255e-03) * q[i1] * q[i12] * q[i19]
            + (-1.639693e-03) * q[i1] * q[i12] * q[i20] + (-1.023231e-03) * q[i1] * q[i12] * q[i21] + (-8.373231e-04) * q[i1] * q[i12] * q[i22]
            + (8.015678e-05) * q[i1] * q[i15] * q[i16] + (-9.882359e-04) * q[i1] * q[i15] * q[i19] + (1.207770e-03) * q[i1] * q[i15] * q[i20]
            + (-2.136392e-04) * q[i1] * q[i15] * q[i21] + (-6.772398e-05) * q[i1] * q[i15] * q[i22] + (-6.929032e-05) * q[i1] * q[i16] * q[i19]
            + (1.071461e-03) * q[i1] * q[i16] * q[i20] + (-3.002668e-04) * q[i1] * q[i16] * q[i21] + (2.228063e-03) * q[i1] * q[i16] * q[i22]
            + (-1.173045e-04) * q[i1] * q[i19] * q[i20] + (-1.194085e-04) * q[i1] * q[i19] * q[i21] + (8.183928e-04) * q[i1] * q[i19] * q[i22]
            + (6.573443e-04) * q[i1] * q[i20] * q[i21] + (-7.683510e-04) * q[i1] * q[i20] * q[i22] + (4.280431e-04) * q[i1] * q[i21] * q[i22]
            + (8.438957e-04) * q[i2] * q[i3] * q[i4] + (-1.295654e-02) * q[i2] * q[i3] * q[i5] + (-3.436195e-03) * q[i2] * q[i3] * q[i6]
            + (-5.176599e-05) * q[i2] * q[i3] * q[i7] + (3.351840e-03) * q[i2] * q[i3] * q[i8] + (1.625624e-03) * q[i2] * q[i3] * q[i9]
            + (-2.978439e-03) * q[i2] * q[i3] * q[i10] + (1.251982e-03) * q[i2] * q[i3] * q[i11] + (-3.162736e-03) * q[i2] * q[i3] * q[i12]
            + (7.214720e-06) * q[i2] * q[i3] * q[i15] + (6.397862e-04) * q[i2] * q[i3] * q[i16] + (-9.618666e-04) * q[i2] * q[i3] * q[i19]
            + (-2.681924e-03) * q[i2] * q[i3] * q[i20] + (1.918705e-03) * q[i2] * q[i3] * q[i21] + (-3.860117e-03) * q[i2] * q[i3] * q[i22]
            + (-1.289630e-02) * q[i2] * q[i4] * q[i5] + (8.420013e-06) * q[i2] * q[i4] * q[i6] + (3.457939e-03) * q[i2] * q[i4] * q[i7]
            + (-3.298082e-03) * q[i2] * q[i4] * q[i8] + (2.980178e-03) * q[i2] * q[i4] * q[i9] + (-1.570757e-03) * q[i2] * q[i4] * q[i10]
            + (-3.083760e-03) * q[i2] * q[i4] * q[i11] + (1.235821e-03) * q[i2] * q[i4] * q[i12] + (-6.401135e-04) * q[i2] * q[i4] * q[i15]
            + (-3.348564e-05) * q[i2] * q[i4] * q[i16] + (2.656238e-03) * q[i2] * q[i4] * q[i19] + (9.512730e-04) * q[i2] * q[i4] * q[i20]
            + (-3.841436e-03) * q[i2] * q[i4] * q[i21] + (1.912853e-03) * q[i2] * q[i4] * q[i22] + (-8.390429e-03) * q[i2] * q[i5] * q[i6]
            + (8.471359e-03) * q[i2] * q[i5] * q[i7] + (-1.490702e-05) * q[i2] * q[i5] * q[i8] + (2.289242e-03) * q[i2] * q[i5] * q[i9]
            + (-2.286014e-03) * q[i2] * q[i5] * q[i10] + (-1.072867e-03) * q[i2] * q[i5] * q[i11] + (-1.137662e-03) * q[i2] * q[i5] * q[i12]
            + (7.764108e-03) * q[i2] * q[i5] * q[i15] + (-7.806409e-03) * q[i2] * q[i5] * q[i16] + (-3.188673e-03) * q[i2] * q[i5] * q[i19]
            + (3.189714e-03) * q[i2] * q[i5] * q[i20] + (-2.336499e-03) * q[i2] * q[i5] * q[i21] + (-2.288843e-03) * q[i2] * q[i5] * q[i22]
            + (1.407930e-02) * q[i2] * q[i6] * q[i7] + (-9.756797e-03) * q[i2] * q[i6] * q[i8] + (-4.292913e-03) * q[i2] * q[i6] * q[i9]
            + (1.744585e-03) * q[i2] * q[i6] * q[i10] + (2.110339e-03) * q[i2] * q[i6] * q[i11] + (3.404679e-03) * q[i2] * q[i6] * q[i12]
            + (-2.437519e-03) * q[i2] * q[i6] * q[i15] + (1.432669e-03) * q[i2] * q[i6] * q[i16] + (-9.689233e-04) * q[i2] * q[i6] * q[i19]
            + (1.426482e-03) * q[i2] * q[i6] * q[i20] + (-3.636610e-04) * q[i2] * q[i6] * q[i21] + (4.758287e-04) * q[i2] * q[i6] * q[i22]
            + (-9.730376e-03) * q[i2] * q[i7] * q[i8] + (1.755898e-03) * q[i2] * q[i7] * q[i9] + (-4.274821e-03) * q[i2] * q[i7] * q[i10]
            + (-3.373453e-03) * q[i2] * q[i7] * q[i11] + (-2.075903e-03) * q[i2] * q[i7] * q[i12] + (1.418427e-03) * q[i2] * q[i7] * q[i15]
            + (-2.459923e-03) * q[i2] * q[i7] * q[i16] + (1.409915e-03) * q[i2] * q[i7] * q[i19] + (-9.379687e-04) * q[i2] * q[i7] * q[i20]
            + (-4.657147e-04) * q[i2] * q[i7] * q[i21] + (3.561437e-04) * q[i2] * q[i7] * q[i22] + (3.126228e-04) * q[i2] * q[i8] * q[i9]
            + (3.500331e-04) * q[i2] * q[i8] * q[i10] + (6.417629e-05) * q[i2] * q[i8] * q[i11] + (-1.014369e-04) * q[i2] * q[i8] * q[i12]
            + (-6.401808e-04) * q[i2] * q[i8] * q[i15] + (-6.618732e-04) * q[i2] * q[i8] * q[i16] + (-2.143070e-03) * q[i2] * q[i8] * q[i19]
            + (-2.164839e-03) * q[i2] * q[i8] * q[i20] + (7.322682e-04) * q[i2] * q[i8] * q[i21] + (-7.450876e-04) * q[i2] * q[i8] * q[i22]
            + (1.046591e-03) * q[i2] * q[i9] * q[i10] + (-1.559192e-03) * q[i2] * q[i9] * q[i11] + (7.825747e-04) * q[i2] * q[i9] * q[i12]
            + (1.132058e-03) * q[i2] * q[i9] * q[i15] + (-1.885767e-03) * q[i2] * q[i9] * q[i16] + (-1.062082e-04) * q[i2] * q[i9] * q[i19]
            + (3.898031e-04) * q[i2] * q[i9] * q[i20] + (9.727644e-05) * q[i2] * q[i9] * q[i21] + (-3.653917e-04) * q[i2] * q[i9] * q[i22]
            + (-7.718939e-04) * q[i2] * q[i10] * q[i11] + (1.535351e-03) * q[i2] * q[i10] * q[i12] + (-1.879207e-03) * q[i2] * q[i10] * q[i15]
            + (1.131607e-03) * q[i2] * q[i10] * q[i16] + (3.717942e-04) * q[i2] * q[i10] * q[i19] + (-1.153407e-04) * q[i2] * q[i10] * q[i20]
            + (3.553253e-04) * q[i2] * q[i10] * q[i21] + (-8.714664e-05) * q[i2] * q[i10] * q[i22] + (2.987426e-03) * q[i2] * q[i11] * q[i12]
            + (2.351462e-03) * q[i2] * q[i11] * q[i15] + (-1.103160e-03) * q[i2] * q[i11] * q[i16] + (-7.304907e-04) * q[i2] * q[i11] * q[i19]
            + (6.020709e-04) * q[i2] * q[i11] * q[i20] + (-8.595518e-04) * q[i2] * q[i11] * q[i21] + (-6.161720e-04) * q[i2] * q[i11] * q[i22]
            + (1.115856e-03) * q[i2] * q[i12] * q[i15] + (-2.369277e-03) * q[i2] * q[i12] * q[i16] + (-6.144859e-04) * q[i2] * q[i12] * q[i19]
            + (7.187429e-04) * q[i2] * q[i12] * q[i20] + (-6.115261e-04) * q[i2] * q[i12] * q[i21] + (-8.317397e-04) * q[i2] * q[i12] * q[i22]
            + (5.453522e-04) * q[i2] * q[i15] * q[i16] + (5.335550e-04) * q[i2] * q[i15] * q[i19] + (9.489542e-04) * q[i2] * q[i15] * q[i20]
            + (-1.693832e-03) * q[i2] * q[i15] * q[i21] + (-3.798725e-04) * q[i2] * q[i15] * q[i22] + (9.717815e-04) * q[i2] * q[i16] * q[i19]
            + (5.450892e-04) * q[i2] * q[i16] * q[i20] + (3.833704e-04) * q[i2] * q[i16] * q[i21] + (1.684649e-03) * q[i2] * q[i16] * q[i22]
            + (-8.298723e-04) * q[i2] * q[i19] * q[i20] + (-6.527569e-04) * q[i2] * q[i19] * q[i21] + (3.099660e-04) * q[i2] * q[i19] * q[i22]
            + (-3.104171e-04) * q[i2] * q[i20] * q[i21] + (6.695144e-04) * q[i2] * q[i20] * q[i22] + (1.096783e-03) * q[i2] * q[i21] * q[i22]
            + (8.470272e-03) * q[i3] * q[i4] * q[i5] + (8.451358e-03) * q[i3] * q[i4] * q[i6] + (-8.385159e-03) * q[i3] * q[i4] * q[i7]
            + (-5.215263e-05) * q[i3] * q[i4] * q[i8] + (-2.546138e-03) * q[i3] * q[i4] * q[i9] + (2.573182e-03) * q[i3] * q[i4] * q[i10]
            + (1.651983e-04) * q[i3] * q[i4] * q[i11] + (2.118888e-04) * q[i3] * q[i4] * q[i12] + (1.928381e-03) * q[i3] * q[i4] * q[i15]
            + (-1.965659e-03) * q[i3] * q[i4] * q[i16] + (-1.061026e-03) * q[i3] * q[i4] * q[i19] + (1.098279e-03) * q[i3] * q[i4] * q[i20]
            + (-1.279848e-03) * q[i3] * q[i4] * q[i21] + (-1.279522e-03) * q[i3] * q[i4] * q[i22] + (-6.583003e-03) * q[i3] * q[i5] * q[i6]
            + (-6.469581e-03) * q[i3] * q[i5] * q[i7] + (-2.029328e-02) * q[i3] * q[i5] * q[i8] + (1.136585e-03) * q[i3] * q[i5] * q[i9]
            + (-4.226047e-05) * q[i3] * q[i5] * q[i10] + (1.932446e-04) * q[i3] * q[i5] * q[i11] + (-4.053934e-03) * q[i3] * q[i5] * q[i12]
            + (-8.369577e-04) * q[i3] * q[i5] * q[i15] + (1.036362e-03) * q[i3] * q[i5] * q[i16] + (-3.405769e-03) * q[i3] * q[i5] * q[i19]
            + (-2.451930e-03) * q[i3] * q[i5] * q[i20] + (2.088894e-03) * q[i3] * q[i5] * q[i21] + (-2.087590e-03) * q[i3] * q[i5] * q[i22]
            + (9.574128e-05) * q[i3] * q[i6] * q[i7] + (1.185074e-04) * q[i3] * q[i6] * q[i8] + (-1.053298e-03) * q[i3] * q[i6] * q[i9]
            + (1.761238e-03) * q[i3] * q[i6] * q[i10] + (-3.321811e-03) * q[i3] * q[i6] * q[i11] + (5.265804e-03) * q[i3] * q[i6] * q[i12]
            + (4.697587e-03) * q[i3] * q[i6] * q[i15] + (1.186916e-03) * q[i3] * q[i6] * q[i16] + (-1.181307e-03) * q[i3] * q[i6] * q[i19]
            + (2.162745e-03) * q[i3] * q[i6] * q[i20] + (-1.000557e-03) * q[i3] * q[i6] * q[i21] + (3.790578e-03) * q[i3] * q[i6] * q[i22]
            + (8.310662e-04) * q[i3] * q[i7] * q[i8] + (-5.500530e-04) * q[i3] * q[i7] * q[i9] + (-2.524895e-03) * q[i3] * q[i7] * q[i10]
            + (5.338200e-03) * q[i3] * q[i7] * q[i11] + (-3.673652e-03) * q[i3] * q[i7] * q[i12] + (3.929025e-03) * q[i3] * q[i7] * q[i15]
            + (-4.584537e-03) * q[i3] * q[i7] * q[i16] + (-2.230980e-04) * q[i3] * q[i7] * q[i19] + (5.809542e-03) * q[i3] * q[i7] * q[i20]
            + (1.440824e-03) * q[i3] * q[i7] * q[i21] + (-3.218778e-04) * q[i3] * q[i7] * q[i22] + (8.914568e-04) * q[i3] * q[i8] * q[i9]
            + (-4.275390e-04) * q[i3] * q[i8] * q[i10] + (2.691313e-03) * q[i3] * q[i8] * q[i11] + (2.354166e-03) * q[i3] * q[i8] * q[i12]
            + (-8.223876e-04) * q[i3] * q[i8] * q[i15] + (3.525317e-06) * q[i3] * q[i8] * q[i16] + (5.242117e-03) * q[i3] * q[i8] * q[i19]
            + (-2.827921e-03) * q[i3] * q[i8] * q[i20] + (1.435959e-03) * q[i3] * q[i8] * q[i21] + (2.509093e-03) * q[i3] * q[i8] * q[i22]
            + (4.041426e-04) * q[i3] * q[i9] * q[i10] + (-1.194464e-03) * q[i3] * q[i9] * q[i11] + (-6.590604e-04) * q[i3] * q[i9] * q[i12]
            + (-2.520779e-04) * q[i3] * q[i9] * q[i15] + (1.079600e-03) * q[i3] * q[i9] * q[i16] + (1.489055e-03) * q[i3] * q[i9] * q[i19]
            + (6.169341e-04) * q[i3] * q[i9] * q[i20] + (-1.580515e-03) * q[i3] * q[i9] * q[i21] + (4.432069e-04) * q[i3] * q[i9] * q[i22]
            + (7.508213e-04) * q[i3] * q[i10] * q[i11] + (2.924787e-04) * q[i3] * q[i10] * q[i12] + (-4.046427e-05) * q[i3] * q[i10] * q[i15]
            + (-2.314218e-03) * q[i3] * q[i10] * q[i16] + (1.010646e-03) * q[i3] * q[i10] * q[i19] + (-1.256187e-03) * q[i3] * q[i10] * q[i20]
            + (-2.090453e-03) * q[i3] * q[i10] * q[i21] + (-1.610804e-03) * q[i3] * q[i10] * q[i22] + (4.219337e-04) * q[i3] * q[i11] * q[i12]
            + (-1.423635e-03) * q[i3] * q[i11] * q[i15] + (4.422790e-04) * q[i3] * q[i11] * q[i16] + (-1.174579e-03) * q[i3] * q[i11] * q[i19]
            + (-2.797302e-03) * q[i3] * q[i11] * q[i20] + (-1.517272e-03) * q[i3] * q[i11] * q[i21] + (-7.618473e-04) * q[i3] * q[i11] * q[i22]
            + (-4.134908e-04) * q[i3] * q[i12] * q[i15] + (-4.669852e-04) * q[i3] * q[i12] * q[i16] + (-4.828984e-03) * q[i3] * q[i12] * q[i19]
            + (-2.117439e-03) * q[i3] * q[i12] * q[i20] + (-5.185118e-04) * q[i3] * q[i12] * q[i21] + (9.435594e-05) * q[i3] * q[i12] * q[i22]
            + (2.937128e-04) * q[i3] * q[i15] * q[i16] + (-1.359044e-03) * q[i3] * q[i15] * q[i19] + (1.892597e-03) * q[i3] * q[i15] * q[i20]
            + (2.977472e-04) * q[i3] * q[i15] * q[i21] + (1.001613e-03) * q[i3] * q[i15] * q[i22] + (-2.043554e-03) * q[i3] * q[i16] * q[i19]
            + (2.998594e-03) * q[i3] * q[i16] * q[i20] + (9.347620e-04) * q[i3] * q[i16] * q[i21] + (1.107827e-03) * q[i3] * q[i16] * q[i22]
            + (-2.845084e-04) * q[i3] * q[i19] * q[i20] + (2.922209e-04) * q[i3] * q[i19] * q[i21] + (-7.987878e-04) * q[i3] * q[i19] * q[i22]
            + (8.213946e-04) * q[i3] * q[i20] * q[i21] + (1.826411e-03) * q[i3] * q[i20] * q[i22] + (7.898597e-05) * q[i3] * q[i21] * q[i22]
            + (6.493612e-03) * q[i4] * q[i5] * q[i6] + (6.544851e-03) * q[i4] * q[i5] * q[i7] + (2.026581e-02) * q[i4] * q[i5] * q[i8]
            + (7.047726e-05) * q[i4] * q[i5] * q[i9] + (-1.138860e-03) * q[i4] * q[i5] * q[i10] + (-3.948614e-03) * q[i4] * q[i5] * q[i11]
            + (1.697960e-04) * q[i4] * q[i5] * q[i12] + (-9.759214e-04) * q[i4] * q[i5] * q[i15] + (8.420715e-04) * q[i4] * q[i5] * q[i16]
            + (2.421059e-03) * q[i4] * q[i5] * q[i19] + (3.417518e-03) * q[i4] * q[i5] * q[i20] + (-2.063205e-03) * q[i4] * q[i5] * q[i21]
            + (2.065826e-03) * q[i4] * q[i5] * q[i22] + (1.566079e-04) * q[i4] * q[i6] * q[i7] + (8.183180e-04) * q[i4] * q[i6] * q[i8]
            + (-2.593748e-03) * q[i4] * q[i6] * q[i9] + (-5.370685e-04) * q[i4] * q[i6] * q[i10] + (3.676719e-03) * q[i4] * q[i6] * q[i11]
            + (-5.365699e-03) * q[i4] * q[i6] * q[i12] + (-4.621407e-03) * q[i4] * q[i6] * q[i15] + (3.924413e-03) * q[i4] * q[i6] * q[i16]
            + (5.798120e-03) * q[i4] * q[i6] * q[i19] + (-2.300427e-04) * q[i4] * q[i6] * q[i20] + (3.513589e-04) * q[i4] * q[i6] * q[i21]
            + (-1.427510e-03) * q[i4] * q[i6] * q[i22] + (1.354132e-04) * q[i4] * q[i7] * q[i8] + (1.765688e-03) * q[i4] * q[i7] * q[i9]
            + (-1.119688e-03) * q[i4] * q[i7] * q[i10] + (-5.271561e-03) * q[i4] * q[i7] * q[i11] + (3.306854e-03) * q[i4] * q[i7] * q[i12]
            + (1.152578e-03) * q[i4] * q[i7] * q[i15] + (4.677970e-03) * q[i4] * q[i7] * q[i16] + (2.130440e-03) * q[i4] * q[i7] * q[i19]
            + (-1.210903e-03) * q[i4] * q[i7] * q[i20] + (-3.757145e-03) * q[i4] * q[i7] * q[i21] + (1.006111e-03) * q[i4] * q[i7] * q[i22]
            + (-4.158017e-04) * q[i4] * q[i8] * q[i9] + (8.800150e-04) * q[i4] * q[i8] * q[i10] + (-2.375399e-03) * q[i4] * q[i8] * q[i11]
            + (-2.759669e-03) * q[i4] * q[i8] * q[i12] + (1.167909e-05) * q[i4] * q[i8] * q[i15] + (-8.594415e-04) * q[i4] * q[i8] * q[i16]
            + (-2.804724e-03) * q[i4] * q[i8] * q[i19] + (5.203166e-03) * q[i4] * q[i8] * q[i20] + (-2.477220e-03) * q[i4] * q[i8] * q[i21]
            + (-1.455301e-03) * q[i4] * q[i8] * q[i22] + (3.924239e-04) * q[i4] * q[i9] * q[i10] + (-3.106594e-04) * q[i4] * q[i9] * q[i11]
            + (-7.626711e-04) * q[i4] * q[i9] * q[i12] + (-2.307338e-03) * q[i4] * q[i9] * q[i15] + (-3.622675e-05) * q[i4] * q[i9] * q[i16]
            + (-1.238296e-03) * q[i4] * q[i9] * q[i19] + (1.006603e-03) * q[i4] * q[i9] * q[i20] + (1.596703e-03) * q[i4] * q[i9] * q[i21]
            + (2.096791e-03) * q[i4] * q[i9] * q[i22] + (6.662096e-04) * q[i4] * q[i10] * q[i11] + (1.201299e-03) * q[i4] * q[i10] * q[i12]
            + (1.070567e-03) * q[i4] * q[i10] * q[i15] + (-2.303241e-04) * q[i4] * q[i10] * q[i16] + (5.875405e-04) * q[i4] * q[i10] * q[i19]
            + (1.476427e-03) * q[i4] * q[i10] * q[i20] + (-4.333095e-04) * q[i4] * q[i10] * q[i21] + (1.577342e-03) * q[i4] * q[i10] * q[i22]
            + (4.096850e-04) * q[i4] * q[i11] * q[i12] + (4.705882e-04) * q[i4] * q[i11] * q[i15] + (4.229579e-04) * q[i4] * q[i11] * q[i16]
            + (2.112294e-03) * q[i4] * q[i11] * q[i19] + (4.808062e-03) * q[i4] * q[i11] * q[i20] + (1.044082e-04) * q[i4] * q[i11] * q[i21]
            + (-5.197775e-04) * q[i4] * q[i11] * q[i22] + (-4.336719e-04) * q[i4] * q[i12] * q[i15] + (1.438830e-03) * q[i4] * q[i12] * q[i16]
            + (2.819507e-03) * q[i4] * q[i12] * q[i19] + (1.160681e-03) * q[i4] * q[i12] * q[i20] + (-7.572880e-04) * q[i4] * q[i12] * q[i21]
            + (-1.530939e-03) * q[i4] * q[i12] * q[i22] + (3.093467e-04) * q[i4] * q[i15] * q[i16] + (3.023984e-03) * q[i4] * q[i15] * q[i19]
            + (-2.037393e-03) * q[i4] * q[i15] * q[i20] + (-1.102679e-03) * q[i4] * q[i15] * q[i21] + (-9.009462e-04) * q[i4] * q[i15] * q[i22]
            + (1.869637e-03) * q[i4] * q[i16] * q[i19] + (-1.350580e-03) * q[i4] * q[i16] * q[i20] + (-1.002016e-03) * q[i4] * q[i16] * q[i21]
            + (-2.786692e-04) * q[i4] * q[i16] * q[i22] + (-2.449044e-04) * q[i4] * q[i19] * q[i20] + (-1.807287e-03) * q[i4] * q[i19] * q[i21]
            + (-8.107735e-04) * q[i4] * q[i19] * q[i22] + (7.983853e-04) * q[i4] * q[i20] * q[i21] + (-2.967091e-04) * q[i4] * q[i20] * q[i22]
            + (6.475461e-05) * q[i4] * q[i21] * q[i22] + (-1.077427e-02) * q[i5] * q[i6] * q[i7] + (2.130050e-04) * q[i5] * q[i6] * q[i8]
            + (-3.894283e-03) * q[i5] * q[i6] * q[i9] + (-1.495500e-03) * q[i5] * q[i6] * q[i10] + (-1.098048e-03) * q[i5] * q[i6] * q[i11]
            + (-1.625085e-03) * q[i5] * q[i6] * q[i12] + (-3.531705e-03) * q[i5] * q[i6] * q[i15] + (-2.354698e-03) * q[i5] * q[i6] * q[i16]
            + (2.702330e-03) * q[i5] * q[i6] * q[i19] + (-9.051232e-04) * q[i5] * q[i6] * q[i20] + (1.591960e-03) * q[i5] * q[i6] * q[i21]
            + (-1.461438e-03) * q[i5] * q[i6] * q[i22] + (2.489370e-04) * q[i5] * q[i7] * q[i8] + (-1.484852e-03) * q[i5] * q[i7] * q[i9]
            + (-3.813284e-03) * q[i5] * q[i7] * q[i10] + (1.649069e-03) * q[i5] * q[i7] * q[i11] + (1.139513e-03) * q[i5] * q[i7] * q[i12]
            + (-2.396054e-03) * q[i5] * q[i7] * q[i15] + (-3.510798e-03) * q[i5] * q[i7] * q[i16] + (-8.926671e-04) * q[i5] * q[i7] * q[i19]
            + (2.701717e-03) * q[i5] * q[i7] * q[i20] + (1.449493e-03) * q[i5] * q[i7] * q[i21] + (-1.590293e-03) * q[i5] * q[i7] * q[i22]
            + (2.075229e-03) * q[i5] * q[i8] * q[i9] + (2.087639e-03) * q[i5] * q[i8] * q[i10] + (-5.032207e-03) * q[i5] * q[i8] * q[i11]
            + (5.177861e-03) * q[i5] * q[i8] * q[i12] + (-3.516862e-03) * q[i5] * q[i8] * q[i15] + (-3.532130e-03) * q[i5] * q[i8] * q[i16]
            + (-1.052659e-03) * q[i5] * q[i8] * q[i19] + (-1.009716e-03) * q[i5] * q[i8] * q[i20] + (1.641170e-03) * q[i5] * q[i8] * q[i21]
            + (-1.689967e-03) * q[i5] * q[i8] * q[i22] + (3.383062e-06) * q[i5] * q[i9] * q[i10] + (-4.488976e-04) * q[i5] * q[i9] * q[i11]
            + (3.316518e-04) * q[i5] * q[i9] * q[i12] + (6.361504e-04) * q[i5] * q[i9] * q[i15] + (-8.130769e-04) * q[i5] * q[i9] * q[i16]
            + (3.115368e-04) * q[i5] * q[i9] * q[i19] + (-1.091896e-03) * q[i5] * q[i9] * q[i20] + (7.568795e-04) * q[i5] * q[i9] * q[i21]
            + (-1.920676e-03) * q[i5] * q[i9] * q[i22] + (-3.314254e-04) * q[i5] * q[i10] * q[i11] + (4.846041e-04) * q[i5] * q[i10] * q[i12]
            + (-7.985036e-04) * q[i5] * q[i10] * q[i15] + (6.463539e-04) * q[i5] * q[i10] * q[i16] + (-1.069672e-03) * q[i5] * q[i10] * q[i19]
            + (3.255600e-04) * q[i5] * q[i10] * q[i20] + (1.911686e-03) * q[i5] * q[i10] * q[i21] + (-7.536486e-04) * q[i5] * q[i10] * q[i22]
            + (-5.596421e-05) * q[i5] * q[i11] * q[i12] + (-3.651541e-03) * q[i5] * q[i11] * q[i15] + (1.199525e-03) * q[i5] * q[i11] * q[i16]
            + (-7.826358e-04) * q[i5] * q[i11] * q[i19] + (8.982612e-04) * q[i5] * q[i11] * q[i20] + (-2.270249e-04) * q[i5] * q[i11] * q[i21]
            + (7.702120e-04) * q[i5] * q[i11] * q[i22] + (-1.202181e-03) * q[i5] * q[i12] * q[i15] + (3.666847e-03) * q[i5] * q[i12] * q[i16]
            + (-9.107160e-04) * q[i5] * q[i12] * q[i19] + (7.582725e-04) * q[i5] * q[i12] * q[i20] + (7.650291e-04) * q[i5] * q[i12] * q[i21]
            + (-2.058770e-04) * q[i5] * q[i12] * q[i22] + (1.537212e-03) * q[i5] * q[i15] * q[i16] + (7.910750e-04) * q[i5] * q[i15] * q[i19]
            + (3.173976e-05) * q[i5] * q[i15] * q[i20] + (-1.798743e-04) * q[i5] * q[i15] * q[i21] + (1.713108e-04) * q[i5] * q[i15] * q[i22]
            + (5.531803e-05) * q[i5] * q[i16] * q[i19] + (8.106571e-04) * q[i5] * q[i16] * q[i20] + (-1.722371e-04) * q[i5] * q[i16] * q[i21]
            + (1.665068e-04) * q[i5] * q[i16] * q[i22] + (4.123524e-06) * q[i5] * q[i19] * q[i20] + (2.630843e-04) * q[i5] * q[i19] * q[i21]
            + (5.709591e-04) * q[i5] * q[i19] * q[i22] + (-5.749362e-04) * q[i5] * q[i20] * q[i21] + (-2.760606e-04) * q[i5] * q[i20] * q[i22]
            + (-9.237874e-04) * q[i5] * q[i21] * q[i22] + (1.471681e-06) * q[i6] * q[i7] * q[i8] + (3.538223e-03) * q[i6] * q[i7] * q[i9]
            + (-3.533052e-03) * q[i6] * q[i7] * q[i10] + (7.670973e-04) * q[i6] * q[i7] * q[i11] + (7.360295e-04) * q[i6] * q[i7] * q[i12]
            + (-4.667196e-03) * q[i6] * q[i7] * q[i15] + (4.670192e-03) * q[i6] * q[i7] * q[i16] + (1.347722e-03) * q[i6] * q[i7] * q[i19]
            + (-1.363415e-03) * q[i6] * q[i7] * q[i20] + (1.620153e-03) * q[i6] * q[i7] * q[i21] + (1.609069e-03) * q[i6] * q[i7] * q[i22]
            + (1.728767e-04) * q[i6] * q[i8] * q[i9] + (3.467390e-04) * q[i6] * q[i8] * q[i10] + (2.328102e-03) * q[i6] * q[i8] * q[i11]
            + (-6.499768e-04) * q[i6] * q[i8] * q[i12] + (-1.649729e-03) * q[i6] * q[i8] * q[i15] + (-2.953595e-03) * q[i6] * q[i8] * q[i16]
            + (9.782341e-04) * q[i6] * q[i8] * q[i19] + (3.762517e-03) * q[i6] * q[i8] * q[i20] + (-3.744837e-04) * q[i6] * q[i8] * q[i21]
            + (5.308637e-04) * q[i6] * q[i8] * q[i22] + (-1.387681e-04) * q[i6] * q[i9] * q[i10] + (9.282152e-04) * q[i6] * q[i9] * q[i11]
            + (-1.665129e-03) * q[i6] * q[i9] * q[i12] + (-4.085242e-04) * q[i6] * q[i9] * q[i15] + (-1.681619e-03) * q[i6] * q[i9] * q[i16]
            + (1.523656e-03) * q[i6] * q[i9] * q[i19] + (1.277208e-03) * q[i6] * q[i9] * q[i20] + (1.961747e-04) * q[i6] * q[i9] * q[i21]
            + (-2.454917e-03) * q[i6] * q[i9] * q[i22] + (1.092517e-03) * q[i6] * q[i10] * q[i11] + (-3.319301e-03) * q[i6] * q[i10] * q[i12]
            + (-3.119390e-03) * q[i6] * q[i10] * q[i15] + (8.119270e-04) * q[i6] * q[i10] * q[i16] + (1.324738e-03) * q[i6] * q[i10] * q[i19]
            + (7.252227e-04) * q[i6] * q[i10] * q[i20] + (1.626460e-03) * q[i6] * q[i10] * q[i21] + (-4.929493e-04) * q[i6] * q[i10] * q[i22]
            + (-6.797917e-03) * q[i6] * q[i11] * q[i12] + (-2.988266e-05) * q[i6] * q[i11] * q[i15] + (2.932153e-03) * q[i6] * q[i11] * q[i16]
            + (1.671965e-03) * q[i6] * q[i11] * q[i19] + (-7.149938e-04) * q[i6] * q[i11] * q[i20] + (1.916454e-03) * q[i6] * q[i11] * q[i21]
            + (1.473500e-03) * q[i6] * q[i11] * q[i22] + (-3.450668e-03) * q[i6] * q[i12] * q[i15] + (9.277737e-04) * q[i6] * q[i12] * q[i16]
            + (-3.422618e-04) * q[i6] * q[i12] * q[i19] + (-1.686519e-03) * q[i6] * q[i12] * q[i20] + (-1.191898e-03) * q[i6] * q[i12] * q[i21]
            + (1.967394e-03) * q[i6] * q[i12] * q[i22] + (-3.686725e-04) * q[i6] * q[i15] * q[i16] + (-1.345635e-03) * q[i6] * q[i15] * q[i19]
            + (-1.620104e-03) * q[i6] * q[i15] * q[i20] + (-1.247835e-03) * q[i6] * q[i15] * q[i21] + (9.407677e-04) * q[i6] * q[i15] * q[i22]
            + (-3.332313e-03) * q[i6] * q[i16] * q[i19] + (5.609957e-04) * q[i6] * q[i16] * q[i20] + (5.767641e-04) * q[i6] * q[i16] * q[i21]
            + (-1.211639e-05) * q[i6] * q[i16] * q[i22] + (1.521665e-03) * q[i6] * q[i19] * q[i20] + (-1.122185e-03) * q[i6] * q[i19] * q[i21]
            + (1.952267e-04) * q[i6] * q[i19] * q[i22] + (-4.661550e-05) * q[i6] * q[i20] * q[i21] + (-1.946770e-03) * q[i6] * q[i20] * q[i22]
            + (-3.455159e-04) * q[i6] * q[i21] * q[i22] + (-3.371606e-04) * q[i7] * q[i8] * q[i9] + (-1.569436e-04) * q[i7] * q[i8] * q[i10]
            + (-6.914101e-04) * q[i7] * q[i8] * q[i11] + (2.296906e-03) * q[i7] * q[i8] * q[i12] + (2.943766e-03) * q[i7] * q[i8] * q[i15]
            + (1.613064e-03) * q[i7] * q[i8] * q[i16] + (-3.737186e-03) * q[i7] * q[i8] * q[i19] + (-9.455939e-04) * q[i7] * q[i8] * q[i20]
            + (5.372617e-04) * q[i7] * q[i8] * q[i21] + (-3.697157e-04) * q[i7] * q[i8] * q[i22] + (1.400217e-04) * q[i7] * q[i9] * q[i10]
            + (-3.324927e-03) * q[i7] * q[i9] * q[i11] + (1.092796e-03) * q[i7] * q[i9] * q[i12] + (-8.232981e-04) * q[i7] * q[i9] * q[i15]
            + (3.118214e-03) * q[i7] * q[i9] * q[i16] + (-7.282846e-04) * q[i7] * q[i9] * q[i19] + (-1.328406e-03) * q[i7] * q[i9] * q[i20]
            + (-5.005814e-04) * q[i7] * q[i9] * q[i21] + (1.636119e-03) * q[i7] * q[i9] * q[i22] + (-1.687183e-03) * q[i7] * q[i10] * q[i11]
            + (9.381162e-04) * q[i7] * q[i10] * q[i12] + (1.665228e-03) * q[i7] * q[i10] * q[i15] + (4.276289e-04) * q[i7] * q[i10] * q[i16]
            + (-1.287692e-03) * q[i7] * q[i10] * q[i19] + (-1.523878e-03) * q[i7] * q[i10] * q[i20] + (-2.451621e-03) * q[i7] * q[i10] * q[i21]
            + (1.984686e-04) * q[i7] * q[i10] * q[i22] + (6.788398e-03) * q[i7] * q[i11] * q[i12] + (9.438830e-04) * q[i7] * q[i11] * q[i15]
            + (-3.429429e-03) * q[i7] * q[i11] * q[i16] + (-1.694066e-03) * q[i7] * q[i11] * q[i19] + (-3.149649e-04) * q[i7] * q[i11] * q[i20]
            + (-1.967779e-03) * q[i7] * q[i11] * q[i21] + (1.176254e-03) * q[i7] * q[i11] * q[i22] + (2.947093e-03) * q[i7] * q[i12] * q[i15]
            + (-3.053283e-05) * q[i7] * q[i12] * q[i16] + (-7.212440e-04) * q[i7] * q[i12] * q[i19] + (1.663596e-03) * q[i7] * q[i12] * q[i20]
            + (-1.454366e-03) * q[i7] * q[i12] * q[i21] + (-1.900401e-03) * q[i7] * q[i12] * q[i22] + (3.657008e-04) * q[i7] * q[i15] * q[i16]
            + (-5.565482e-04) * q[i7] * q[i15] * q[i19] + (3.309427e-03) * q[i7] * q[i15] * q[i20] + (-8.648373e-06) * q[i7] * q[i15] * q[i21]
            + (5.829073e-04) * q[i7] * q[i15] * q[i22] + (1.611454e-03) * q[i7] * q[i16] * q[i19] + (1.343040e-03) * q[i7] * q[i16] * q[i20]
            + (9.408520e-04) * q[i7] * q[i16] * q[i21] + (-1.249415e-03) * q[i7] * q[i16] * q[i22] + (-1.519264e-03) * q[i7] * q[i19] * q[i20]
            + (-1.940345e-03) * q[i7] * q[i19] * q[i21] + (-4.046829e-05) * q[i7] * q[i19] * q[i22] + (2.033601e-04) * q[i7] * q[i20] * q[i21]
            + (-1.112846e-03) * q[i7] * q[i20] * q[i22] + (3.555269e-04) * q[i7] * q[i21] * q[i22] + (7.190316e-06) * q[i8] * q[i9] * q[i10]
            + (1.505537e-03) * q[i8] * q[i9] * q[i11] + (7.128823e-04) * q[i8] * q[i9] * q[i12] + (-2.079523e-03) * q[i8] * q[i9] * q[i15]
            + (-2.149540e-03) * q[i8] * q[i9] * q[i16] + (8.208506e-04) * q[i8] * q[i9] * q[i19] + (2.952076e-03) * q[i8] * q[i9] * q[i20]
            + (-1.144188e-03) * q[i8] * q[i9] * q[i21] + (4.128965e-04) * q[i8] * q[i9] * q[i22] + (7.206284e-04) * q[i8] * q[i10] * q[i11]
            + (1.499905e-03) * q[i8] * q[i10] * q[i12] + (2.132471e-03) * q[i8] * q[i10] * q[i15] + (2.067358e-03) * q[i8] * q[i10] * q[i16]
            + (-2.938725e-03) * q[i8] * q[i10] * q[i19] + (-8.147241e-04) * q[i8] * q[i10] * q[i20] + (4.143830e-04) * q[i8] * q[i10] * q[i21]
            + (-1.149477e-03) * q[i8] * q[i10] * q[i22] + (5.213333e-07) * q[i8] * q[i11] * q[i12] + (2.826673e-03) * q[i8] * q[i11] * q[i15]
            + (1.193063e-03) * q[i8] * q[i11] * q[i16] + (-1.131070e-03) * q[i8] * q[i11] * q[i19] + (4.758059e-04) * q[i8] * q[i11] * q[i20]
            + (-3.066098e-04) * q[i8] * q[i11] * q[i21] + (-1.142485e-03) * q[i8] * q[i11] * q[i22] + (1.212328e-03) * q[i8] * q[i12] * q[i15]
            + (2.828037e-03) * q[i8] * q[i12] * q[i16] + (4.809914e-04) * q[i8] * q[i12] * q[i19] + (-1.072583e-03) * q[i8] * q[i12] * q[i20]
            + (1.149213e-03) * q[i8] * q[i12] * q[i21] + (3.046039e-04) * q[i8] * q[i12] * q[i22] + (7.098969e-07) * q[i8] * q[i15] * q[i16]
            + (1.069179e-03) * q[i8] * q[i15] * q[i19] + (-7.574531e-04) * q[i8] * q[i15] * q[i20] + (2.629756e-03) * q[i8] * q[i15] * q[i21]
            + (-3.028679e-04) * q[i8] * q[i15] * q[i22] + (7.779851e-04) * q[i8] * q[i16] * q[i19] + (-1.079222e-03) * q[i8] * q[i16] * q[i20]
            + (-2.999376e-04) * q[i8] * q[i16] * q[i21] + (2.667583e-03) * q[i8] * q[i16] * q[i22] + (5.116259e-06) * q[i8] * q[i19] * q[i20]
            + (3.348708e-03) * q[i8] * q[i19] * q[i21] + (2.725898e-04) * q[i8] * q[i19] * q[i22] + (2.678443e-04) * q[i8] * q[i20] * q[i21]
            + (3.337514e-03) * q[i8] * q[i20] * q[i22] + (7.139614e-06) * q[i8] * q[i21] * q[i22] + (-3.432840e-04) * q[i9] * q[i10] * q[i11]
            + (-3.509618e-04) * q[i9] * q[i10] * q[i12] + (-7.261380e-04) * q[i9] * q[i10] * q[i15] + (7.210693e-04) * q[i9] * q[i10] * q[i16]
            + (3.068323e-04) * q[i9] * q[i10] * q[i19] + (-3.114489e-04) * q[i9] * q[i10] * q[i20] + (2.671950e-04) * q[i9] * q[i10] * q[i21]
            + (2.713456e-04) * q[i9] * q[i10] * q[i22] + (-1.383689e-03) * q[i9] * q[i11] * q[i12] + (4.185205e-04) * q[i9] * q[i11] * q[i15]
            + (6.748590e-05) * q[i9] * q[i11] * q[i16] + (2.570354e-04) * q[i9] * q[i11] * q[i19] + (-2.698155e-04) * q[i9] * q[i11] * q[i20]
            + (5.498505e-04) * q[i9] * q[i11] * q[i21] + (5.739907e-04) * q[i9] * q[i11] * q[i22] + (-5.104271e-04) * q[i9] * q[i12] * q[i15]
            + (8.176412e-04) * q[i9] * q[i12] * q[i16] + (8.131269e-05) * q[i9] * q[i12] * q[i19] + (-9.948959e-06) * q[i9] * q[i12] * q[i20]
            + (-7.537221e-04) * q[i9] * q[i12] * q[i21] + (-2.366834e-04) * q[i9] * q[i12] * q[i22] + (-6.006669e-04) * q[i9] * q[i15] * q[i16]
            + (-1.487320e-04) * q[i9] * q[i15] * q[i19] + (-2.653334e-05) * q[i9] * q[i15] * q[i20] + (-6.297149e-04) * q[i9] * q[i15] * q[i21]
            + (-3.685585e-05) * q[i9] * q[i15] * q[i22] + (-9.418895e-04) * q[i9] * q[i16] * q[i19] + (4.169658e-04) * q[i9] * q[i16] * q[i20]
            + (1.721087e-05) * q[i9] * q[i16] * q[i21] + (2.033018e-05) * q[i9] * q[i16] * q[i22] + (6.983687e-04) * q[i9] * q[i19] * q[i20]
            + (-4.695622e-04) * q[i9] * q[i19] * q[i21] + (2.972330e-04) * q[i9] * q[i19] * q[i22] + (-1.456776e-05) * q[i9] * q[i20] * q[i21]
            + (-3.028418e-04) * q[i9] * q[i20] * q[i22] + (-1.130490e-04) * q[i9] * q[i21] * q[i22] + (1.351670e-03) * q[i10] * q[i11] * q[i12]
            + (8.191657e-04) * q[i10] * q[i11] * q[i15] + (-4.993969e-04) * q[i10] * q[i11] * q[i16] + (-6.430745e-06) * q[i10] * q[i11] * q[i19]
            + (9.173832e-05) * q[i10] * q[i11] * q[i20] + (2.451832e-04) * q[i10] * q[i11] * q[i21] + (7.502987e-04) * q[i10] * q[i11] * q[i22]
            + (6.865590e-05) * q[i10] * q[i12] * q[i15] + (4.180841e-04) * q[i10] * q[i12] * q[i16] + (-2.767815e-04) * q[i10] * q[i12] * q[i19]
            + (2.536174e-04) * q[i10] * q[i12] * q[i20] + (-5.718205e-04) * q[i10] * q[i12] * q[i21] + (-5.451014e-04) * q[i10] * q[i12] * q[i22]
            + (6.058846e-04) * q[i10] * q[i15] * q[i16] + (-4.135366e-04) * q[i10] * q[i15] * q[i19] + (9.230676e-04) * q[i10] * q[i15] * q[i20]
            + (2.476633e-05) * q[i10] * q[i15] * q[i21] + (2.225760e-05) * q[i10] * q[i15] * q[i22] + (2.947857e-05) * q[i10] * q[i16] * q[i19]
            + (1.432046e-04) * q[i10] * q[i16] * q[i20] + (-4.318072e-05) * q[i10] * q[i16] * q[i21] + (-6.301378e-04) * q[i10] * q[i16] * q[i22]
            + (-6.971758e-04) * q[i10] * q[i19] * q[i20] + (-3.065525e-04) * q[i10] * q[i19] * q[i21] + (-1.014092e-05) * q[i10] * q[i19] * q[i22]
            + (2.987073e-04) * q[i10] * q[i20] * q[i21] + (-4.575649e-04) * q[i10] * q[i20] * q[i22] + (1.193717e-04) * q[i10] * q[i21] * q[i22]
            + (-3.824057e-04) * q[i11] * q[i12] * q[i15] + (3.722676e-04) * q[i11] * q[i12] * q[i16] + (-5.943896e-04) * q[i11] * q[i12] * q[i19]
            + (6.070909e-04) * q[i11] * q[i12] * q[i20] + (1.004132e-04) * q[i11] * q[i12] * q[i21] + (1.110204e-04) * q[i11] * q[i12] * q[i22]
            + (7.387768e-04) * q[i11] * q[i15] * q[i16] + (8.488030e-04) * q[i11] * q[i15] * q[i19] + (-1.645095e-04) * q[i11] * q[i15] * q[i20]
            + (1.516783e-03) * q[i11] * q[i15] * q[i21] + (-1.931456e-04) * q[i11] * q[i15] * q[i22] + (5.898340e-04) * q[i11] * q[i16] * q[i19]
            + (-9.083006e-04) * q[i11] * q[i16] * q[i20] + (1.304451e-04) * q[i11] * q[i16] * q[i21] + (-9.794195e-05) * q[i11] * q[i16] * q[i22]
            + (3.248382e-04) * q[i11] * q[i19] * q[i20] + (1.886006e-03) * q[i11] * q[i19] * q[i21] + (7.798466e-04) * q[i11] * q[i19] * q[i22]
            + (3.111280e-04) * q[i11] * q[i20] * q[i21] + (-3.240401e-04) * q[i11] * q[i20] * q[i22] + (2.629995e-04) * q[i11] * q[i21] * q[i22]
            + (7.343239e-04) * q[i12] * q[i15] * q[i16] + (-9.047756e-04) * q[i12] * q[i15] * q[i19] + (5.885397e-04) * q[i12] * q[i15] * q[i20]
            + (9.203270e-05) * q[i12] * q[i15] * q[i21] + (-1.425435e-04) * q[i12] * q[i15] * q[i22] + (-1.627775e-04) * q[i12] * q[i16] * q[i19]
            + (8.694204e-04) * q[i12] * q[i16] * q[i20] + (1.930178e-04) * q[i12] * q[i16] * q[i21] + (-1.559937e-03) * q[i12] * q[i16] * q[i22]
            + (3.196399e-04) * q[i12] * q[i19] * q[i20] + (3.213397e-04) * q[i12] * q[i19] * q[i21] + (-3.151055e-04) * q[i12] * q[i19] * q[i22]
            + (-7.816191e-04) * q[i12] * q[i20] * q[i21] + (-1.866132e-03) * q[i12] * q[i20] * q[i22] + (2.602461e-04) * q[i12] * q[i21] * q[i22]
            + (-7.513823e-05) * q[i15] * q[i16] * q[i19] + (7.803911e-05) * q[i15] * q[i16] * q[i20] + (-2.143389e-04) * q[i15] * q[i16] * q[i21]
            + (-2.106631e-04) * q[i15] * q[i16] * q[i22] + (-5.097665e-04) * q[i15] * q[i19] * q[i20] + (1.186223e-04) * q[i15] * q[i19] * q[i21]
            + (-1.350526e-06) * q[i15] * q[i19] * q[i22] + (-2.515901e-04) * q[i15] * q[i20] * q[i21] + (6.296997e-05) * q[i15] * q[i20] * q[i22]
            + (1.336803e-04) * q[i15] * q[i21] * q[i22] + (5.075237e-04) * q[i16] * q[i19] * q[i20] + (6.470507e-05) * q[i16] * q[i19] * q[i21]
            + (-2.544436e-04) * q[i16] * q[i19] * q[i22] + (5.950866e-06) * q[i16] * q[i20] * q[i21] + (1.024102e-04) * q[i16] * q[i20] * q[i22]
            + (-1.343905e-04) * q[i16] * q[i21] * q[i22] + (-2.439649e-05) * q[i19] * q[i20] * q[i21] + (-2.701687e-05) * q[i19] * q[i20] * q[i22]
            + (8.061874e-05) * q[i19] * q[i21] * q[i22] + (-8.116803e-05) * q[i20] * q[i21] * q[i22];
      return Qz;
   }

   public void getJQx(double[] q, double[][] JQ)
   {
      getJQx0(q, JQ);
      getJQx1(q, JQ);
      getJQx2(q, JQ);
      getJQx3(q, JQ);
      getJQx4(q, JQ);
      getJQx5(q, JQ);
      getJQx6(q, JQ);
      getJQx7(q, JQ);
      getJQx8(q, JQ);
      getJQx9(q, JQ);
      getJQx10(q, JQ);
      getJQx11(q, JQ);
      getJQx12(q, JQ);
      getJQx15(q, JQ);
      getJQx16(q, JQ);
      getJQx19(q, JQ);
      getJQx20(q, JQ);
      getJQx21(q, JQ);
      getJQx22(q, JQ);
   }

   public void getJQx0(double[] q, double[][] JQ)
   {
      JQ[1][i0] = (-1.091470e-02) * (1) + (-4.763695e-03) * ((2) * q[i0]) + (-3.072205e-05) * (q[i1]) + (-2.215087e-04) * (q[i2]) + (-1.614664e-03) * (q[i3])
            + (-4.964491e-03) * (q[i4]) + (2.972583e-03) * (q[i5]) + (9.063135e-02) * (q[i6]) + (-1.051662e-02) * (q[i7]) + (-3.638438e-03) * (q[i8])
            + (2.931941e-02) * (q[i9]) + (-1.566182e-03) * (q[i10]) + (2.431821e-03) * (q[i11]) + (-2.645071e-04) * (q[i12]) + (-3.495047e-04) * (q[i15])
            + (2.440634e-03) * (q[i16]) + (-2.003494e-03) * (q[i19]) + (6.647019e-04) * (q[i20]) + (1.479821e-03) * (q[i21]) + (3.335293e-04) * (q[i22])
            + (1.626362e-03) * ((3) * q[i0] * q[i0]) + (-3.464843e-03) * ((2) * q[i0] * q[i1]) + (2.556321e-03) * ((2) * q[i0] * q[i2])
            + (-3.012961e-02) * ((2) * q[i0] * q[i3]) + (-6.835978e-05) * ((2) * q[i0] * q[i4]) + (-3.507112e-03) * ((2) * q[i0] * q[i5])
            + (1.253647e-03) * ((2) * q[i0] * q[i6]) + (2.032812e-03) * ((2) * q[i0] * q[i7]) + (-1.493391e-03) * ((2) * q[i0] * q[i8])
            + (2.952615e-03) * ((2) * q[i0] * q[i9]) + (-2.826019e-04) * ((2) * q[i0] * q[i10]) + (7.179850e-04) * ((2) * q[i0] * q[i11])
            + (4.998808e-06) * ((2) * q[i0] * q[i12]) + (-1.196572e-03) * ((2) * q[i0] * q[i15]) + (-2.972802e-04) * ((2) * q[i0] * q[i16])
            + (-4.202603e-05) * ((2) * q[i0] * q[i19]) + (-3.548803e-04) * ((2) * q[i0] * q[i20]) + (6.331864e-04) * ((2) * q[i0] * q[i21])
            + (-2.889568e-04) * ((2) * q[i0] * q[i22]) + (-3.444680e-03) * (q[i1] * q[i1]) + (1.645876e-04) * (q[i2] * q[i2])
            + (-2.153409e-03) * (q[i3] * q[i3]) + (1.433173e-03) * (q[i4] * q[i4]) + (-2.019421e-03) * (q[i5] * q[i5]) + (3.867929e-03) * (q[i6] * q[i6])
            + (-3.453669e-03) * (q[i7] * q[i7]) + (1.790137e-04) * (q[i8] * q[i8]) + (-6.786792e-03) * (q[i9] * q[i9]) + (1.268991e-03) * (q[i10] * q[i10])
            + (2.634201e-04) * (q[i11] * q[i11]) + (2.812292e-04) * (q[i12] * q[i12]) + (-9.181986e-05) * (q[i15] * q[i15]) + (6.728841e-05) * (q[i16] * q[i16])
            + (-1.738111e-05) * (q[i19] * q[i19]) + (-1.050280e-04) * (q[i20] * q[i20]) + (2.005327e-04) * (q[i21] * q[i21])
            + (3.563537e-04) * (q[i22] * q[i22]) + (-4.256968e-03) * (q[i1] * q[i2]) + (7.202945e-03) * (q[i1] * q[i3]) + (7.154852e-03) * (q[i1] * q[i4])
            + (-5.894587e-03) * (q[i1] * q[i5]) + (1.950107e-03) * (q[i1] * q[i6]) + (-1.925395e-03) * (q[i1] * q[i7]) + (4.442005e-06) * (q[i1] * q[i8])
            + (-1.418768e-03) * (q[i1] * q[i9]) + (1.421309e-03) * (q[i1] * q[i10]) + (-8.110965e-04) * (q[i1] * q[i11]) + (-8.088057e-04) * (q[i1] * q[i12])
            + (1.151046e-03) * (q[i1] * q[i15]) + (-1.169626e-03) * (q[i1] * q[i16]) + (-5.858271e-04) * (q[i1] * q[i19]) + (5.868450e-04) * (q[i1] * q[i20])
            + (2.626669e-04) * (q[i1] * q[i21]) + (2.594540e-04) * (q[i1] * q[i22]) + (-1.283325e-02) * (q[i2] * q[i3]) + (1.855208e-03) * (q[i2] * q[i4])
            + (-1.194723e-02) * (q[i2] * q[i5]) + (1.345896e-03) * (q[i2] * q[i6]) + (-2.001371e-03) * (q[i2] * q[i7]) + (-3.725796e-03) * (q[i2] * q[i8])
            + (1.292606e-03) * (q[i2] * q[i9]) + (5.500446e-04) * (q[i2] * q[i10]) + (6.652454e-04) * (q[i2] * q[i11]) + (1.396673e-03) * (q[i2] * q[i12])
            + (-6.568085e-05) * (q[i2] * q[i15]) + (1.546440e-03) * (q[i2] * q[i16]) + (4.345169e-04) * (q[i2] * q[i19]) + (1.349169e-03) * (q[i2] * q[i20])
            + (5.814716e-04) * (q[i2] * q[i21]) + (1.058040e-04) * (q[i2] * q[i22]) + (-1.104364e-05) * (q[i3] * q[i4]) + (6.232324e-04) * (q[i3] * q[i5])
            + (-1.220277e-02) * (q[i3] * q[i6]) + (4.599825e-04) * (q[i3] * q[i7]) + (2.879513e-03) * (q[i3] * q[i8]) + (-1.020871e-03) * (q[i3] * q[i9])
            + (1.436438e-03) * (q[i3] * q[i10]) + (4.059029e-05) * (q[i3] * q[i11]) + (-1.685471e-03) * (q[i3] * q[i12]) + (2.231693e-03) * (q[i3] * q[i15])
            + (8.670147e-04) * (q[i3] * q[i16]) + (1.522527e-03) * (q[i3] * q[i19]) + (1.245394e-03) * (q[i3] * q[i20]) + (-8.057844e-04) * (q[i3] * q[i21])
            + (-7.652511e-04) * (q[i3] * q[i22]) + (4.044206e-03) * (q[i4] * q[i5]) + (-3.852007e-03) * (q[i4] * q[i6]) + (-2.147826e-04) * (q[i4] * q[i7])
            + (-1.860052e-03) * (q[i4] * q[i8]) + (-6.762930e-04) * (q[i4] * q[i9]) + (7.799416e-04) * (q[i4] * q[i10]) + (1.098684e-03) * (q[i4] * q[i11])
            + (-1.804400e-03) * (q[i4] * q[i12]) + (-5.569488e-04) * (q[i4] * q[i15]) + (-3.701069e-04) * (q[i4] * q[i16]) + (-1.016211e-03) * (q[i4] * q[i19])
            + (-9.938613e-04) * (q[i4] * q[i20]) + (-3.788516e-04) * (q[i4] * q[i21]) + (6.018258e-04) * (q[i4] * q[i22]) + (-8.060198e-03) * (q[i5] * q[i6])
            + (-4.824738e-04) * (q[i5] * q[i7]) + (-3.618220e-04) * (q[i5] * q[i8]) + (4.046716e-04) * (q[i5] * q[i9]) + (-8.990503e-04) * (q[i5] * q[i10])
            + (-6.948002e-05) * (q[i5] * q[i11]) + (2.869565e-04) * (q[i5] * q[i12]) + (7.623461e-04) * (q[i5] * q[i15]) + (-1.635196e-03) * (q[i5] * q[i16])
            + (-8.694627e-04) * (q[i5] * q[i19]) + (1.593632e-04) * (q[i5] * q[i20]) + (6.680767e-04) * (q[i5] * q[i21]) + (-2.400176e-04) * (q[i5] * q[i22])
            + (-4.657516e-04) * (q[i6] * q[i7]) + (7.409014e-03) * (q[i6] * q[i8]) + (-1.360672e-02) * (q[i6] * q[i9]) + (1.714814e-03) * (q[i6] * q[i10])
            + (-1.493674e-03) * (q[i6] * q[i11]) + (3.205563e-04) * (q[i6] * q[i12]) + (3.511050e-04) * (q[i6] * q[i15]) + (2.069774e-03) * (q[i6] * q[i16])
            + (1.222475e-03) * (q[i6] * q[i19]) + (1.359705e-03) * (q[i6] * q[i20]) + (-8.987898e-04) * (q[i6] * q[i21]) + (6.300372e-04) * (q[i6] * q[i22])
            + (-1.297956e-03) * (q[i7] * q[i8]) + (-1.327951e-04) * (q[i7] * q[i9]) + (3.413640e-03) * (q[i7] * q[i10]) + (-9.047856e-04) * (q[i7] * q[i11])
            + (-1.669194e-03) * (q[i7] * q[i12]) + (-6.133359e-04) * (q[i7] * q[i15]) + (7.881685e-04) * (q[i7] * q[i16]) + (-6.159552e-04) * (q[i7] * q[i19])
            + (-9.066067e-04) * (q[i7] * q[i20]) + (-2.006549e-04) * (q[i7] * q[i21]) + (-8.392349e-04) * (q[i7] * q[i22]) + (7.407671e-04) * (q[i8] * q[i9])
            + (-1.721679e-04) * (q[i8] * q[i10]) + (-5.959634e-04) * (q[i8] * q[i11]) + (1.649874e-04) * (q[i8] * q[i12]) + (9.418310e-04) * (q[i8] * q[i15])
            + (2.296953e-04) * (q[i8] * q[i16]) + (1.349733e-03) * (q[i8] * q[i19]) + (1.054646e-03) * (q[i8] * q[i20]) + (3.697073e-04) * (q[i8] * q[i21])
            + (-3.461998e-04) * (q[i8] * q[i22]) + (7.379308e-04) * (q[i9] * q[i10]) + (1.292349e-05) * (q[i9] * q[i11]) + (-5.578639e-04) * (q[i9] * q[i12])
            + (7.242263e-04) * (q[i9] * q[i15]) + (2.030050e-04) * (q[i9] * q[i16]) + (8.202329e-04) * (q[i9] * q[i19]) + (5.663061e-04) * (q[i9] * q[i20])
            + (-4.827697e-04) * (q[i9] * q[i21]) + (-5.376229e-05) * (q[i9] * q[i22]) + (-8.957915e-04) * (q[i10] * q[i11])
            + (-5.991964e-05) * (q[i10] * q[i12]) + (-8.095689e-05) * (q[i10] * q[i15]) + (1.579682e-04) * (q[i10] * q[i16])
            + (-3.351315e-04) * (q[i10] * q[i19]) + (-6.638931e-04) * (q[i10] * q[i20]) + (-6.733610e-04) * (q[i10] * q[i21])
            + (-4.308028e-04) * (q[i10] * q[i22]) + (1.037433e-03) * (q[i11] * q[i12]) + (1.111723e-03) * (q[i11] * q[i15])
            + (-6.870321e-05) * (q[i11] * q[i16]) + (6.155529e-04) * (q[i11] * q[i19]) + (-2.593303e-04) * (q[i11] * q[i20])
            + (7.605467e-04) * (q[i11] * q[i21]) + (3.498287e-04) * (q[i11] * q[i22]) + (-3.508971e-04) * (q[i12] * q[i15])
            + (-3.126500e-04) * (q[i12] * q[i16]) + (1.308050e-04) * (q[i12] * q[i19]) + (-5.335765e-04) * (q[i12] * q[i20])
            + (4.240182e-04) * (q[i12] * q[i21]) + (-7.301080e-04) * (q[i12] * q[i22]) + (5.148387e-04) * (q[i15] * q[i16])
            + (-1.717268e-04) * (q[i15] * q[i19]) + (4.410306e-04) * (q[i15] * q[i20]) + (2.398073e-04) * (q[i15] * q[i21]) + (5.474682e-04) * (q[i15] * q[i22])
            + (-1.048530e-03) * (q[i16] * q[i19]) + (8.003445e-04) * (q[i16] * q[i20]) + (-2.354564e-04) * (q[i16] * q[i21])
            + (-2.929430e-05) * (q[i16] * q[i22]) + (4.840219e-04) * (q[i19] * q[i20]) + (-2.430971e-04) * (q[i19] * q[i21])
            + (-5.005325e-04) * (q[i19] * q[i22]) + (7.464442e-04) * (q[i20] * q[i21]) + (8.370573e-04) * (q[i20] * q[i22])
            + (1.707779e-04) * (q[i21] * q[i22]);
   }

   public void getJQx1(double[] q, double[][] JQ)
   {
      JQ[1][i1] = (-1.075516e-02) * (1) + (4.731760e-03) * ((2) * q[i1]) + (-3.072205e-05) * (q[i0]) + (1.546791e-04) * (q[i2]) + (4.911325e-03) * (q[i3])
            + (1.639490e-03) * (q[i4]) + (-2.945678e-03) * (q[i5]) + (-1.054391e-02) * (q[i6]) + (9.010674e-02) * (q[i7]) + (-3.676940e-03) * (q[i8])
            + (-1.585410e-03) * (q[i9]) + (2.896086e-02) * (q[i10]) + (2.549366e-04) * (q[i11]) + (-2.437312e-03) * (q[i12]) + (2.488392e-03) * (q[i15])
            + (-3.292644e-04) * (q[i16]) + (7.010237e-04) * (q[i19]) + (-1.956615e-03) * (q[i20]) + (-3.335710e-04) * (q[i21]) + (-1.478069e-03) * (q[i22])
            + (-3.464843e-03) * (q[i0] * q[i0]) + (-3.444680e-03) * ((2) * q[i0] * q[i1]) + (1.609090e-03) * ((3) * q[i1] * q[i1])
            + (2.557233e-03) * ((2) * q[i1] * q[i2]) + (-6.150220e-05) * ((2) * q[i1] * q[i3]) + (-3.004084e-02) * ((2) * q[i1] * q[i4])
            + (-3.462911e-03) * ((2) * q[i1] * q[i5]) + (-2.010742e-03) * ((2) * q[i1] * q[i6]) + (-1.266027e-03) * ((2) * q[i1] * q[i7])
            + (1.472293e-03) * ((2) * q[i1] * q[i8]) + (3.075456e-04) * ((2) * q[i1] * q[i9]) + (-2.936930e-03) * ((2) * q[i1] * q[i10])
            + (2.325739e-05) * ((2) * q[i1] * q[i11]) + (7.082192e-04) * ((2) * q[i1] * q[i12]) + (2.994997e-04) * ((2) * q[i1] * q[i15])
            + (1.192890e-03) * ((2) * q[i1] * q[i16]) + (3.554884e-04) * ((2) * q[i1] * q[i19]) + (5.501578e-05) * ((2) * q[i1] * q[i20])
            + (-2.780366e-04) * ((2) * q[i1] * q[i21]) + (6.328589e-04) * ((2) * q[i1] * q[i22]) + (1.444696e-04) * (q[i2] * q[i2])
            + (1.442191e-03) * (q[i3] * q[i3]) + (-2.141928e-03) * (q[i4] * q[i4]) + (-1.986907e-03) * (q[i5] * q[i5]) + (-3.455323e-03) * (q[i6] * q[i6])
            + (3.902302e-03) * (q[i7] * q[i7]) + (2.022371e-04) * (q[i8] * q[i8]) + (1.277800e-03) * (q[i9] * q[i9]) + (-6.708023e-03) * (q[i10] * q[i10])
            + (3.093027e-04) * (q[i11] * q[i11]) + (2.557378e-04) * (q[i12] * q[i12]) + (7.592240e-05) * (q[i15] * q[i15]) + (-9.215216e-05) * (q[i16] * q[i16])
            + (-1.020428e-04) * (q[i19] * q[i19]) + (-2.351599e-05) * (q[i20] * q[i20]) + (3.551545e-04) * (q[i21] * q[i21])
            + (1.959782e-04) * (q[i22] * q[i22]) + (-4.256968e-03) * (q[i0] * q[i2]) + (7.202945e-03) * (q[i0] * q[i3]) + (7.154852e-03) * (q[i0] * q[i4])
            + (-5.894587e-03) * (q[i0] * q[i5]) + (1.950107e-03) * (q[i0] * q[i6]) + (-1.925395e-03) * (q[i0] * q[i7]) + (4.442005e-06) * (q[i0] * q[i8])
            + (-1.418768e-03) * (q[i0] * q[i9]) + (1.421309e-03) * (q[i0] * q[i10]) + (-8.110965e-04) * (q[i0] * q[i11]) + (-8.088057e-04) * (q[i0] * q[i12])
            + (1.151046e-03) * (q[i0] * q[i15]) + (-1.169626e-03) * (q[i0] * q[i16]) + (-5.858271e-04) * (q[i0] * q[i19]) + (5.868450e-04) * (q[i0] * q[i20])
            + (2.626669e-04) * (q[i0] * q[i21]) + (2.594540e-04) * (q[i0] * q[i22]) + (1.897785e-03) * (q[i2] * q[i3]) + (-1.277614e-02) * (q[i2] * q[i4])
            + (-1.187656e-02) * (q[i2] * q[i5]) + (2.015449e-03) * (q[i2] * q[i6]) + (-1.331022e-03) * (q[i2] * q[i7]) + (3.702176e-03) * (q[i2] * q[i8])
            + (-5.259012e-04) * (q[i2] * q[i9]) + (-1.279786e-03) * (q[i2] * q[i10]) + (1.404683e-03) * (q[i2] * q[i11]) + (6.646431e-04) * (q[i2] * q[i12])
            + (-1.531927e-03) * (q[i2] * q[i15]) + (6.903362e-05) * (q[i2] * q[i16]) + (-1.356130e-03) * (q[i2] * q[i19]) + (-4.233245e-04) * (q[i2] * q[i20])
            + (1.109319e-04) * (q[i2] * q[i21]) + (5.835975e-04) * (q[i2] * q[i22]) + (6.927884e-06) * (q[i3] * q[i4]) + (4.044748e-03) * (q[i3] * q[i5])
            + (1.776476e-04) * (q[i3] * q[i6]) + (3.785884e-03) * (q[i3] * q[i7]) + (1.894027e-03) * (q[i3] * q[i8]) + (-7.905932e-04) * (q[i3] * q[i9])
            + (6.715230e-04) * (q[i3] * q[i10]) + (-1.809140e-03) * (q[i3] * q[i11]) + (1.062638e-03) * (q[i3] * q[i12]) + (3.459426e-04) * (q[i3] * q[i15])
            + (5.483259e-04) * (q[i3] * q[i16]) + (9.862217e-04) * (q[i3] * q[i19]) + (1.000061e-03) * (q[i3] * q[i20]) + (5.962383e-04) * (q[i3] * q[i21])
            + (-3.858473e-04) * (q[i3] * q[i22]) + (6.049404e-04) * (q[i4] * q[i5]) + (-4.309909e-04) * (q[i4] * q[i6]) + (1.215547e-02) * (q[i4] * q[i7])
            + (-2.884023e-03) * (q[i4] * q[i8]) + (-1.469909e-03) * (q[i4] * q[i9]) + (1.007366e-03) * (q[i4] * q[i10]) + (-1.716275e-03) * (q[i4] * q[i11])
            + (6.743792e-05) * (q[i4] * q[i12]) + (-8.506545e-04) * (q[i4] * q[i15]) + (-2.229472e-03) * (q[i4] * q[i16]) + (-1.241991e-03) * (q[i4] * q[i19])
            + (-1.501474e-03) * (q[i4] * q[i20]) + (-7.710065e-04) * (q[i4] * q[i21]) + (-8.268980e-04) * (q[i4] * q[i22]) + (4.644945e-04) * (q[i5] * q[i6])
            + (8.116384e-03) * (q[i5] * q[i7]) + (3.976265e-04) * (q[i5] * q[i8]) + (8.545363e-04) * (q[i5] * q[i9]) + (-3.880431e-04) * (q[i5] * q[i10])
            + (3.387693e-04) * (q[i5] * q[i11]) + (-6.398775e-05) * (q[i5] * q[i12]) + (1.632663e-03) * (q[i5] * q[i15]) + (-7.627753e-04) * (q[i5] * q[i16])
            + (-1.608649e-04) * (q[i5] * q[i19]) + (8.323787e-04) * (q[i5] * q[i20]) + (-2.455253e-04) * (q[i5] * q[i21]) + (6.741223e-04) * (q[i5] * q[i22])
            + (-5.072719e-04) * (q[i6] * q[i7]) + (-1.312707e-03) * (q[i6] * q[i8]) + (3.417191e-03) * (q[i6] * q[i9]) + (-1.111062e-04) * (q[i6] * q[i10])
            + (1.669093e-03) * (q[i6] * q[i11]) + (9.281863e-04) * (q[i6] * q[i12]) + (7.933511e-04) * (q[i6] * q[i15]) + (-6.163040e-04) * (q[i6] * q[i16])
            + (-9.142841e-04) * (q[i6] * q[i19]) + (-6.114105e-04) * (q[i6] * q[i20]) + (8.439397e-04) * (q[i6] * q[i21]) + (1.852810e-04) * (q[i6] * q[i22])
            + (7.388077e-03) * (q[i7] * q[i8]) + (1.747766e-03) * (q[i7] * q[i9]) + (-1.345148e-02) * (q[i7] * q[i10]) + (-2.929429e-04) * (q[i7] * q[i11])
            + (1.519737e-03) * (q[i7] * q[i12]) + (2.038911e-03) * (q[i7] * q[i15]) + (3.758379e-04) * (q[i7] * q[i16]) + (1.349245e-03) * (q[i7] * q[i19])
            + (1.199835e-03) * (q[i7] * q[i20]) + (-6.180849e-04) * (q[i7] * q[i21]) + (9.134953e-04) * (q[i7] * q[i22]) + (-1.896148e-04) * (q[i8] * q[i9])
            + (7.417262e-04) * (q[i8] * q[i10]) + (-1.253281e-04) * (q[i8] * q[i11]) + (6.184059e-04) * (q[i8] * q[i12]) + (2.099563e-04) * (q[i8] * q[i15])
            + (9.333350e-04) * (q[i8] * q[i16]) + (1.060160e-03) * (q[i8] * q[i19]) + (1.348211e-03) * (q[i8] * q[i20]) + (3.351103e-04) * (q[i8] * q[i21])
            + (-3.669231e-04) * (q[i8] * q[i22]) + (7.396080e-04) * (q[i9] * q[i10]) + (6.116578e-05) * (q[i9] * q[i11]) + (9.128566e-04) * (q[i9] * q[i12])
            + (1.494251e-04) * (q[i9] * q[i15]) + (-7.386152e-05) * (q[i9] * q[i16]) + (-6.724032e-04) * (q[i9] * q[i19]) + (-3.448627e-04) * (q[i9] * q[i20])
            + (4.375698e-04) * (q[i9] * q[i21]) + (6.739413e-04) * (q[i9] * q[i22]) + (5.542927e-04) * (q[i10] * q[i11]) + (-1.430565e-05) * (q[i10] * q[i12])
            + (1.871632e-04) * (q[i10] * q[i15]) + (7.150911e-04) * (q[i10] * q[i16]) + (5.671184e-04) * (q[i10] * q[i19]) + (8.148392e-04) * (q[i10] * q[i20])
            + (5.603105e-05) * (q[i10] * q[i21]) + (4.743153e-04) * (q[i10] * q[i22]) + (1.050391e-03) * (q[i11] * q[i12]) + (2.987142e-04) * (q[i11] * q[i15])
            + (3.494695e-04) * (q[i11] * q[i16]) + (5.263009e-04) * (q[i11] * q[i19]) + (-1.317697e-04) * (q[i11] * q[i20])
            + (-7.379106e-04) * (q[i11] * q[i21]) + (4.227531e-04) * (q[i11] * q[i22]) + (5.891167e-05) * (q[i12] * q[i15])
            + (-1.117033e-03) * (q[i12] * q[i16]) + (2.595780e-04) * (q[i12] * q[i19]) + (-6.142505e-04) * (q[i12] * q[i20])
            + (3.496778e-04) * (q[i12] * q[i21]) + (7.630297e-04) * (q[i12] * q[i22]) + (5.088894e-04) * (q[i15] * q[i16]) + (8.008115e-04) * (q[i15] * q[i19])
            + (-1.042180e-03) * (q[i15] * q[i20]) + (2.924900e-05) * (q[i15] * q[i21]) + (2.277305e-04) * (q[i15] * q[i22]) + (4.527292e-04) * (q[i16] * q[i19])
            + (-1.634519e-04) * (q[i16] * q[i20]) + (-5.340071e-04) * (q[i16] * q[i21]) + (-2.454685e-04) * (q[i16] * q[i22])
            + (4.608439e-04) * (q[i19] * q[i20]) + (-8.342090e-04) * (q[i19] * q[i21]) + (-7.396063e-04) * (q[i19] * q[i22])
            + (5.044497e-04) * (q[i20] * q[i21]) + (2.404725e-04) * (q[i20] * q[i22]) + (1.760598e-04) * (q[i21] * q[i22]);
   }

   public void getJQx2(double[] q, double[][] JQ)
   {
      JQ[1][i2] = (-6.493713e-04) * (1) + (-1.342401e-05) * ((2) * q[i2]) + (-2.215087e-04) * (q[i0]) + (1.546791e-04) * (q[i1]) + (9.295948e-04) * (q[i3])
            + (-1.007355e-03) * (q[i4]) + (9.527315e-05) * (q[i5]) + (1.591417e-02) * (q[i6]) + (1.577288e-02) * (q[i7]) + (-6.770789e-02) * (q[i8])
            + (3.152888e-03) * (q[i9]) + (3.116881e-03) * (q[i10]) + (5.278294e-03) * (q[i11]) + (-5.454492e-03) * (q[i12]) + (2.469027e-03) * (q[i15])
            + (2.522821e-03) * (q[i16]) + (-2.095669e-03) * (q[i19]) + (-2.100913e-03) * (q[i20]) + (1.403467e-03) * (q[i21]) + (-1.403050e-03) * (q[i22])
            + (2.556321e-03) * (q[i0] * q[i0]) + (2.557233e-03) * (q[i1] * q[i1]) + (1.645876e-04) * ((2) * q[i0] * q[i2])
            + (1.444696e-04) * ((2) * q[i1] * q[i2]) + (2.998311e-03) * ((3) * q[i2] * q[i2]) + (-1.410196e-03) * ((2) * q[i2] * q[i3])
            + (-1.390624e-03) * ((2) * q[i2] * q[i4]) + (-2.711721e-02) * ((2) * q[i2] * q[i5]) + (9.419021e-04) * ((2) * q[i2] * q[i6])
            + (-9.400876e-04) * ((2) * q[i2] * q[i7]) + (-1.694320e-05) * ((2) * q[i2] * q[i8]) + (1.309976e-04) * ((2) * q[i2] * q[i9])
            + (-1.301474e-04) * ((2) * q[i2] * q[i10]) + (6.982843e-04) * ((2) * q[i2] * q[i11]) + (6.892704e-04) * ((2) * q[i2] * q[i12])
            + (-2.546112e-03) * ((2) * q[i2] * q[i15]) + (2.587146e-03) * ((2) * q[i2] * q[i16]) + (1.521570e-04) * ((2) * q[i2] * q[i19])
            + (-1.591219e-04) * ((2) * q[i2] * q[i20]) + (1.112668e-03) * ((2) * q[i2] * q[i21]) + (1.105582e-03) * ((2) * q[i2] * q[i22])
            + (-2.028160e-03) * (q[i3] * q[i3]) + (-2.034666e-03) * (q[i4] * q[i4]) + (-4.393894e-04) * (q[i5] * q[i5]) + (-4.848208e-04) * (q[i6] * q[i6])
            + (-4.773915e-04) * (q[i7] * q[i7]) + (1.764850e-03) * (q[i8] * q[i8]) + (-2.022016e-03) * (q[i9] * q[i9]) + (-1.993552e-03) * (q[i10] * q[i10])
            + (-4.927059e-05) * (q[i11] * q[i11]) + (-1.504999e-04) * (q[i12] * q[i12]) + (1.748959e-03) * (q[i15] * q[i15])
            + (1.766318e-03) * (q[i16] * q[i16]) + (-2.030081e-04) * (q[i19] * q[i19]) + (-2.080122e-04) * (q[i20] * q[i20])
            + (6.122352e-04) * (q[i21] * q[i21]) + (6.068116e-04) * (q[i22] * q[i22]) + (-4.256968e-03) * (q[i0] * q[i1]) + (-1.283325e-02) * (q[i0] * q[i3])
            + (1.855208e-03) * (q[i0] * q[i4]) + (-1.194723e-02) * (q[i0] * q[i5]) + (1.345896e-03) * (q[i0] * q[i6]) + (-2.001371e-03) * (q[i0] * q[i7])
            + (-3.725796e-03) * (q[i0] * q[i8]) + (1.292606e-03) * (q[i0] * q[i9]) + (5.500446e-04) * (q[i0] * q[i10]) + (6.652454e-04) * (q[i0] * q[i11])
            + (1.396673e-03) * (q[i0] * q[i12]) + (-6.568085e-05) * (q[i0] * q[i15]) + (1.546440e-03) * (q[i0] * q[i16]) + (4.345169e-04) * (q[i0] * q[i19])
            + (1.349169e-03) * (q[i0] * q[i20]) + (5.814716e-04) * (q[i0] * q[i21]) + (1.058040e-04) * (q[i0] * q[i22]) + (1.897785e-03) * (q[i1] * q[i3])
            + (-1.277614e-02) * (q[i1] * q[i4]) + (-1.187656e-02) * (q[i1] * q[i5]) + (2.015449e-03) * (q[i1] * q[i6]) + (-1.331022e-03) * (q[i1] * q[i7])
            + (3.702176e-03) * (q[i1] * q[i8]) + (-5.259012e-04) * (q[i1] * q[i9]) + (-1.279786e-03) * (q[i1] * q[i10]) + (1.404683e-03) * (q[i1] * q[i11])
            + (6.646431e-04) * (q[i1] * q[i12]) + (-1.531927e-03) * (q[i1] * q[i15]) + (6.903362e-05) * (q[i1] * q[i16]) + (-1.356130e-03) * (q[i1] * q[i19])
            + (-4.233245e-04) * (q[i1] * q[i20]) + (1.109319e-04) * (q[i1] * q[i21]) + (5.835975e-04) * (q[i1] * q[i22]) + (2.120604e-03) * (q[i3] * q[i4])
            + (5.379433e-03) * (q[i3] * q[i5]) + (-3.754849e-03) * (q[i3] * q[i6]) + (1.078962e-03) * (q[i3] * q[i7]) + (3.084937e-03) * (q[i3] * q[i8])
            + (1.821206e-04) * (q[i3] * q[i9]) + (-4.290648e-04) * (q[i3] * q[i10]) + (5.062225e-04) * (q[i3] * q[i11]) + (-4.700165e-04) * (q[i3] * q[i12])
            + (1.418867e-03) * (q[i3] * q[i15]) + (1.015846e-04) * (q[i3] * q[i16]) + (-9.577654e-04) * (q[i3] * q[i19]) + (5.090413e-04) * (q[i3] * q[i20])
            + (-7.309653e-04) * (q[i3] * q[i21]) + (1.000350e-03) * (q[i3] * q[i22]) + (5.443058e-03) * (q[i4] * q[i5]) + (-1.066613e-03) * (q[i4] * q[i6])
            + (3.717713e-03) * (q[i4] * q[i7]) + (-3.042462e-03) * (q[i4] * q[i8]) + (4.203955e-04) * (q[i4] * q[i9]) + (-1.730749e-04) * (q[i4] * q[i10])
            + (-4.936573e-04) * (q[i4] * q[i11]) + (5.365713e-04) * (q[i4] * q[i12]) + (-9.815592e-05) * (q[i4] * q[i15]) + (-1.455408e-03) * (q[i4] * q[i16])
            + (-5.021187e-04) * (q[i4] * q[i19]) + (9.312876e-04) * (q[i4] * q[i20]) + (9.853955e-04) * (q[i4] * q[i21]) + (-7.527671e-04) * (q[i4] * q[i22])
            + (-1.897680e-04) * (q[i5] * q[i6]) + (1.789531e-04) * (q[i5] * q[i7]) + (-2.629788e-05) * (q[i5] * q[i8]) + (2.116555e-03) * (q[i5] * q[i9])
            + (-2.136809e-03) * (q[i5] * q[i10]) + (1.737642e-03) * (q[i5] * q[i11]) + (1.689365e-03) * (q[i5] * q[i12]) + (-1.444219e-03) * (q[i5] * q[i15])
            + (1.441259e-03) * (q[i5] * q[i16]) + (1.186527e-03) * (q[i5] * q[i19]) + (-1.194357e-03) * (q[i5] * q[i20]) + (5.688177e-04) * (q[i5] * q[i21])
            + (5.556571e-04) * (q[i5] * q[i22]) + (3.344318e-03) * (q[i6] * q[i7]) + (5.734035e-03) * (q[i6] * q[i8]) + (-2.505272e-03) * (q[i6] * q[i9])
            + (7.022326e-04) * (q[i6] * q[i10]) + (-7.407081e-04) * (q[i6] * q[i11]) + (-3.079661e-04) * (q[i6] * q[i12]) + (5.709624e-04) * (q[i6] * q[i15])
            + (2.251452e-04) * (q[i6] * q[i16]) + (1.268388e-03) * (q[i6] * q[i19]) + (3.018136e-04) * (q[i6] * q[i20]) + (1.285496e-04) * (q[i6] * q[i21])
            + (3.242108e-04) * (q[i6] * q[i22]) + (5.755823e-03) * (q[i7] * q[i8]) + (7.057201e-04) * (q[i7] * q[i9]) + (-2.443772e-03) * (q[i7] * q[i10])
            + (3.318719e-04) * (q[i7] * q[i11]) + (7.383186e-04) * (q[i7] * q[i12]) + (2.397672e-04) * (q[i7] * q[i15]) + (5.739001e-04) * (q[i7] * q[i16])
            + (2.983729e-04) * (q[i7] * q[i19]) + (1.274549e-03) * (q[i7] * q[i20]) + (-3.183364e-04) * (q[i7] * q[i21]) + (-1.257341e-04) * (q[i7] * q[i22])
            + (-2.458956e-03) * (q[i8] * q[i9]) + (-2.408560e-03) * (q[i8] * q[i10]) + (-1.244990e-03) * (q[i8] * q[i11]) + (1.411772e-03) * (q[i8] * q[i12])
            + (4.348435e-03) * (q[i8] * q[i15]) + (4.428257e-03) * (q[i8] * q[i16]) + (1.071222e-03) * (q[i8] * q[i19]) + (1.062139e-03) * (q[i8] * q[i20])
            + (-5.368134e-04) * (q[i8] * q[i21]) + (5.417197e-04) * (q[i8] * q[i22]) + (-1.138271e-04) * (q[i9] * q[i10]) + (-4.428682e-04) * (q[i9] * q[i11])
            + (1.445852e-04) * (q[i9] * q[i12]) + (6.495961e-04) * (q[i9] * q[i15]) + (-4.232450e-04) * (q[i9] * q[i16]) + (1.164111e-04) * (q[i9] * q[i19])
            + (3.097910e-04) * (q[i9] * q[i20]) + (-3.177849e-04) * (q[i9] * q[i21]) + (2.041463e-04) * (q[i9] * q[i22]) + (-1.591874e-04) * (q[i10] * q[i11])
            + (4.314493e-04) * (q[i10] * q[i12]) + (-4.261787e-04) * (q[i10] * q[i15]) + (6.458251e-04) * (q[i10] * q[i16]) + (3.252194e-04) * (q[i10] * q[i19])
            + (1.234765e-04) * (q[i10] * q[i20]) + (-2.055229e-04) * (q[i10] * q[i21]) + (3.180438e-04) * (q[i10] * q[i22]) + (1.565082e-03) * (q[i11] * q[i12])
            + (3.600388e-03) * (q[i11] * q[i15]) + (-6.828608e-04) * (q[i11] * q[i16]) + (1.696311e-03) * (q[i11] * q[i19]) + (5.165242e-05) * (q[i11] * q[i20])
            + (8.125435e-05) * (q[i11] * q[i21]) + (-1.375290e-04) * (q[i11] * q[i22]) + (6.583024e-04) * (q[i12] * q[i15])
            + (-3.614837e-03) * (q[i12] * q[i16]) + (-6.191380e-05) * (q[i12] * q[i19]) + (-1.678354e-03) * (q[i12] * q[i20])
            + (-1.404150e-04) * (q[i12] * q[i21]) + (8.041627e-05) * (q[i12] * q[i22]) + (4.283540e-05) * (q[i15] * q[i16])
            + (-9.625853e-04) * (q[i15] * q[i19]) + (-8.625585e-04) * (q[i15] * q[i20]) + (3.715050e-04) * (q[i15] * q[i21])
            + (3.675351e-04) * (q[i15] * q[i22]) + (-8.608036e-04) * (q[i16] * q[i19]) + (-9.489442e-04) * (q[i16] * q[i20])
            + (-3.632431e-04) * (q[i16] * q[i21]) + (-3.792889e-04) * (q[i16] * q[i22]) + (1.081195e-03) * (q[i19] * q[i20])
            + (-1.188958e-03) * (q[i19] * q[i21]) + (-5.315273e-04) * (q[i19] * q[i22]) + (5.348327e-04) * (q[i20] * q[i21])
            + (1.183849e-03) * (q[i20] * q[i22]) + (2.080626e-04) * (q[i21] * q[i22]);
   }

   public void getJQx3(double[] q, double[][] JQ)
   {
      JQ[1][i3] = (1.312319e-01) * (1) + (1.348478e-02) * ((2) * q[i3]) + (-1.614664e-03) * (q[i0]) + (4.911325e-03) * (q[i1]) + (9.295948e-04) * (q[i2])
            + (-2.876676e-06) * (q[i4]) + (-9.794577e-03) * (q[i5]) + (1.055381e-02) * (q[i6]) + (-4.554090e-03) * (q[i7]) + (2.986873e-03) * (q[i8])
            + (-7.460434e-03) * (q[i9]) + (4.861489e-03) * (q[i10]) + (-8.971683e-04) * (q[i11]) + (2.443215e-03) * (q[i12]) + (5.579206e-03) * (q[i15])
            + (5.510126e-03) * (q[i16]) + (-2.364092e-03) * (q[i19]) + (3.470420e-03) * (q[i20]) + (-9.036793e-04) * (q[i21]) + (5.059900e-04) * (q[i22])
            + (-3.012961e-02) * (q[i0] * q[i0]) + (-6.150220e-05) * (q[i1] * q[i1]) + (-1.410196e-03) * (q[i2] * q[i2])
            + (-2.153409e-03) * ((2) * q[i0] * q[i3]) + (1.442191e-03) * ((2) * q[i1] * q[i3]) + (-2.028160e-03) * ((2) * q[i2] * q[i3])
            + (-3.734343e-03) * ((3) * q[i3] * q[i3]) + (-1.103080e-03) * ((2) * q[i3] * q[i4]) + (7.210215e-03) * ((2) * q[i3] * q[i5])
            + (1.880440e-03) * ((2) * q[i3] * q[i6]) + (2.994120e-04) * ((2) * q[i3] * q[i7]) + (7.210461e-04) * ((2) * q[i3] * q[i8])
            + (-1.857075e-04) * ((2) * q[i3] * q[i9]) + (-2.367540e-04) * ((2) * q[i3] * q[i10]) + (-1.402094e-03) * ((2) * q[i3] * q[i11])
            + (-2.855950e-03) * ((2) * q[i3] * q[i12]) + (7.552389e-04) * ((2) * q[i3] * q[i15]) + (3.199199e-04) * ((2) * q[i3] * q[i16])
            + (-1.116370e-03) * ((2) * q[i3] * q[i19]) + (-2.797243e-04) * ((2) * q[i3] * q[i20]) + (1.536098e-04) * ((2) * q[i3] * q[i21])
            + (3.081561e-04) * ((2) * q[i3] * q[i22]) + (-1.147295e-03) * (q[i4] * q[i4]) + (-4.753110e-04) * (q[i5] * q[i5])
            + (-2.187062e-02) * (q[i6] * q[i6]) + (3.528850e-03) * (q[i7] * q[i7]) + (5.135405e-03) * (q[i8] * q[i8]) + (-3.546787e-03) * (q[i9] * q[i9])
            + (4.730181e-04) * (q[i10] * q[i10]) + (-3.952900e-04) * (q[i11] * q[i11]) + (-9.821072e-04) * (q[i12] * q[i12])
            + (6.068730e-04) * (q[i15] * q[i15]) + (7.318281e-04) * (q[i16] * q[i16]) + (1.567991e-04) * (q[i19] * q[i19]) + (1.332810e-04) * (q[i20] * q[i20])
            + (2.876675e-04) * (q[i21] * q[i21]) + (1.916338e-04) * (q[i22] * q[i22]) + (7.202945e-03) * (q[i0] * q[i1]) + (-1.283325e-02) * (q[i0] * q[i2])
            + (-1.104364e-05) * (q[i0] * q[i4]) + (6.232324e-04) * (q[i0] * q[i5]) + (-1.220277e-02) * (q[i0] * q[i6]) + (4.599825e-04) * (q[i0] * q[i7])
            + (2.879513e-03) * (q[i0] * q[i8]) + (-1.020871e-03) * (q[i0] * q[i9]) + (1.436438e-03) * (q[i0] * q[i10]) + (4.059029e-05) * (q[i0] * q[i11])
            + (-1.685471e-03) * (q[i0] * q[i12]) + (2.231693e-03) * (q[i0] * q[i15]) + (8.670147e-04) * (q[i0] * q[i16]) + (1.522527e-03) * (q[i0] * q[i19])
            + (1.245394e-03) * (q[i0] * q[i20]) + (-8.057844e-04) * (q[i0] * q[i21]) + (-7.652511e-04) * (q[i0] * q[i22]) + (1.897785e-03) * (q[i1] * q[i2])
            + (6.927884e-06) * (q[i1] * q[i4]) + (4.044748e-03) * (q[i1] * q[i5]) + (1.776476e-04) * (q[i1] * q[i6]) + (3.785884e-03) * (q[i1] * q[i7])
            + (1.894027e-03) * (q[i1] * q[i8]) + (-7.905932e-04) * (q[i1] * q[i9]) + (6.715230e-04) * (q[i1] * q[i10]) + (-1.809140e-03) * (q[i1] * q[i11])
            + (1.062638e-03) * (q[i1] * q[i12]) + (3.459426e-04) * (q[i1] * q[i15]) + (5.483259e-04) * (q[i1] * q[i16]) + (9.862217e-04) * (q[i1] * q[i19])
            + (1.000061e-03) * (q[i1] * q[i20]) + (5.962383e-04) * (q[i1] * q[i21]) + (-3.858473e-04) * (q[i1] * q[i22]) + (2.120604e-03) * (q[i2] * q[i4])
            + (5.379433e-03) * (q[i2] * q[i5]) + (-3.754849e-03) * (q[i2] * q[i6]) + (1.078962e-03) * (q[i2] * q[i7]) + (3.084937e-03) * (q[i2] * q[i8])
            + (1.821206e-04) * (q[i2] * q[i9]) + (-4.290648e-04) * (q[i2] * q[i10]) + (5.062225e-04) * (q[i2] * q[i11]) + (-4.700165e-04) * (q[i2] * q[i12])
            + (1.418867e-03) * (q[i2] * q[i15]) + (1.015846e-04) * (q[i2] * q[i16]) + (-9.577654e-04) * (q[i2] * q[i19]) + (5.090413e-04) * (q[i2] * q[i20])
            + (-7.309653e-04) * (q[i2] * q[i21]) + (1.000350e-03) * (q[i2] * q[i22]) + (-1.625450e-03) * (q[i4] * q[i5]) + (2.142900e-03) * (q[i4] * q[i6])
            + (-2.125483e-03) * (q[i4] * q[i7]) + (-2.209751e-05) * (q[i4] * q[i8]) + (-4.865914e-05) * (q[i4] * q[i9]) + (5.663865e-05) * (q[i4] * q[i10])
            + (1.984605e-03) * (q[i4] * q[i11]) + (1.988030e-03) * (q[i4] * q[i12]) + (-6.454528e-05) * (q[i4] * q[i15]) + (7.420731e-05) * (q[i4] * q[i16])
            + (-7.927073e-04) * (q[i4] * q[i19]) + (7.792435e-04) * (q[i4] * q[i20]) + (-7.415448e-04) * (q[i4] * q[i21]) + (-7.459352e-04) * (q[i4] * q[i22])
            + (-6.349387e-03) * (q[i5] * q[i6]) + (5.954575e-04) * (q[i5] * q[i7]) + (-4.838856e-04) * (q[i5] * q[i8]) + (-1.903444e-04) * (q[i5] * q[i9])
            + (-1.450961e-03) * (q[i5] * q[i10]) + (-8.940210e-04) * (q[i5] * q[i11]) + (-3.053467e-04) * (q[i5] * q[i12]) + (3.455274e-04) * (q[i5] * q[i15])
            + (9.330827e-04) * (q[i5] * q[i16]) + (-8.253939e-04) * (q[i5] * q[i19]) + (-1.240502e-03) * (q[i5] * q[i20]) + (3.996984e-05) * (q[i5] * q[i21])
            + (1.160825e-03) * (q[i5] * q[i22]) + (3.245163e-04) * (q[i6] * q[i7]) + (-3.734237e-03) * (q[i6] * q[i8]) + (-1.030080e-02) * (q[i6] * q[i9])
            + (5.297648e-04) * (q[i6] * q[i10]) + (1.117085e-04) * (q[i6] * q[i11]) + (1.625795e-04) * (q[i6] * q[i12]) + (1.331850e-03) * (q[i6] * q[i15])
            + (-1.027732e-03) * (q[i6] * q[i16]) + (-2.949664e-03) * (q[i6] * q[i19]) + (-1.961914e-04) * (q[i6] * q[i20]) + (-8.116925e-04) * (q[i6] * q[i21])
            + (3.394242e-04) * (q[i6] * q[i22]) + (-3.796529e-04) * (q[i7] * q[i8]) + (1.223988e-04) * (q[i7] * q[i9]) + (1.490958e-03) * (q[i7] * q[i10])
            + (-9.071750e-04) * (q[i7] * q[i11]) + (7.903289e-04) * (q[i7] * q[i12]) + (-7.167150e-04) * (q[i7] * q[i15]) + (-1.838161e-03) * (q[i7] * q[i16])
            + (-1.571947e-04) * (q[i7] * q[i19]) + (1.872840e-03) * (q[i7] * q[i20]) + (1.340993e-03) * (q[i7] * q[i21]) + (7.346822e-04) * (q[i7] * q[i22])
            + (-7.091580e-04) * (q[i8] * q[i9]) + (9.674446e-04) * (q[i8] * q[i10]) + (-8.620245e-04) * (q[i8] * q[i11]) + (1.817324e-03) * (q[i8] * q[i12])
            + (4.639511e-04) * (q[i8] * q[i15]) + (-2.946867e-04) * (q[i8] * q[i16]) + (-1.082728e-03) * (q[i8] * q[i19]) + (4.997872e-05) * (q[i8] * q[i20])
            + (1.835052e-04) * (q[i8] * q[i21]) + (-1.137578e-04) * (q[i8] * q[i22]) + (-1.125415e-03) * (q[i9] * q[i10]) + (-5.818825e-04) * (q[i9] * q[i11])
            + (2.452206e-04) * (q[i9] * q[i12]) + (-2.710467e-04) * (q[i9] * q[i15]) + (-4.947617e-04) * (q[i9] * q[i16]) + (2.575008e-04) * (q[i9] * q[i19])
            + (-1.099831e-03) * (q[i9] * q[i20]) + (2.429874e-04) * (q[i9] * q[i21]) + (-1.711662e-04) * (q[i9] * q[i22]) + (-7.761758e-05) * (q[i10] * q[i11])
            + (2.547123e-04) * (q[i10] * q[i12]) + (-6.593369e-04) * (q[i10] * q[i15]) + (-5.319852e-05) * (q[i10] * q[i16])
            + (9.528878e-04) * (q[i10] * q[i19]) + (-1.118422e-04) * (q[i10] * q[i20]) + (-2.816300e-04) * (q[i10] * q[i21])
            + (3.857699e-04) * (q[i10] * q[i22]) + (-9.431192e-04) * (q[i11] * q[i12]) + (-2.399187e-03) * (q[i11] * q[i15])
            + (9.598698e-04) * (q[i11] * q[i16]) + (3.549022e-04) * (q[i11] * q[i19]) + (-6.758842e-05) * (q[i11] * q[i20])
            + (-8.200449e-04) * (q[i11] * q[i21]) + (-1.065079e-03) * (q[i11] * q[i22]) + (3.537319e-04) * (q[i12] * q[i15])
            + (-4.212583e-04) * (q[i12] * q[i16]) + (-5.466593e-04) * (q[i12] * q[i19]) + (8.499335e-04) * (q[i12] * q[i20])
            + (8.870416e-04) * (q[i12] * q[i21]) + (-2.562754e-04) * (q[i12] * q[i22]) + (6.317741e-04) * (q[i15] * q[i16])
            + (-4.963041e-04) * (q[i15] * q[i19]) + (-5.463849e-04) * (q[i15] * q[i20]) + (-1.765358e-04) * (q[i15] * q[i21])
            + (2.002703e-04) * (q[i15] * q[i22]) + (7.135509e-04) * (q[i16] * q[i19]) + (4.429594e-04) * (q[i16] * q[i20]) + (-2.275339e-05) * (q[i16] * q[i21])
            + (-5.864281e-04) * (q[i16] * q[i22]) + (6.337695e-04) * (q[i19] * q[i20]) + (1.284728e-03) * (q[i19] * q[i21])
            + (-3.871210e-04) * (q[i19] * q[i22]) + (-3.009105e-04) * (q[i20] * q[i21]) + (-1.722683e-04) * (q[i20] * q[i22])
            + (2.200795e-04) * (q[i21] * q[i22]);
   }

   public void getJQx4(double[] q, double[][] JQ)
   {
      JQ[1][i4] = (1.305468e-01) * (1) + (-1.337858e-02) * ((2) * q[i4]) + (-4.964491e-03) * (q[i0]) + (1.639490e-03) * (q[i1]) + (-1.007355e-03) * (q[i2])
            + (-2.876676e-06) * (q[i3]) + (9.743681e-03) * (q[i5]) + (-4.531510e-03) * (q[i6]) + (1.051373e-02) * (q[i7]) + (3.061437e-03) * (q[i8])
            + (4.908594e-03) * (q[i9]) + (-7.410230e-03) * (q[i10]) + (-2.383645e-03) * (q[i11]) + (9.079738e-04) * (q[i12]) + (5.425509e-03) * (q[i15])
            + (5.592667e-03) * (q[i16]) + (3.453573e-03) * (q[i19]) + (-2.334264e-03) * (q[i20]) + (-5.053012e-04) * (q[i21]) + (8.770941e-04) * (q[i22])
            + (-6.835978e-05) * (q[i0] * q[i0]) + (-3.004084e-02) * (q[i1] * q[i1]) + (-1.390624e-03) * (q[i2] * q[i2]) + (-1.103080e-03) * (q[i3] * q[i3])
            + (1.433173e-03) * ((2) * q[i0] * q[i4]) + (-2.141928e-03) * ((2) * q[i1] * q[i4]) + (-2.034666e-03) * ((2) * q[i2] * q[i4])
            + (-1.147295e-03) * ((2) * q[i3] * q[i4]) + (-3.714140e-03) * ((3) * q[i4] * q[i4]) + (7.177278e-03) * ((2) * q[i4] * q[i5])
            + (-2.780751e-04) * ((2) * q[i4] * q[i6]) + (-1.868758e-03) * ((2) * q[i4] * q[i7]) + (-6.687702e-04) * ((2) * q[i4] * q[i8])
            + (2.262692e-04) * ((2) * q[i4] * q[i9]) + (1.723745e-04) * ((2) * q[i4] * q[i10]) + (-2.837358e-03) * ((2) * q[i4] * q[i11])
            + (-1.394569e-03) * ((2) * q[i4] * q[i12]) + (-3.144852e-04) * ((2) * q[i4] * q[i15]) + (-7.444036e-04) * ((2) * q[i4] * q[i16])
            + (2.920938e-04) * ((2) * q[i4] * q[i19]) + (1.091014e-03) * ((2) * q[i4] * q[i20]) + (3.029220e-04) * ((2) * q[i4] * q[i21])
            + (1.453614e-04) * ((2) * q[i4] * q[i22]) + (-4.325963e-04) * (q[i5] * q[i5]) + (3.542552e-03) * (q[i6] * q[i6]) + (-2.179241e-02) * (q[i7] * q[i7])
            + (5.120441e-03) * (q[i8] * q[i8]) + (4.736603e-04) * (q[i9] * q[i9]) + (-3.499427e-03) * (q[i10] * q[i10]) + (-1.005719e-03) * (q[i11] * q[i11])
            + (-3.937433e-04) * (q[i12] * q[i12]) + (7.297165e-04) * (q[i15] * q[i15]) + (6.037906e-04) * (q[i16] * q[i16]) + (1.412430e-04) * (q[i19] * q[i19])
            + (1.504221e-04) * (q[i20] * q[i20]) + (1.871116e-04) * (q[i21] * q[i21]) + (2.904623e-04) * (q[i22] * q[i22]) + (7.154852e-03) * (q[i0] * q[i1])
            + (1.855208e-03) * (q[i0] * q[i2]) + (-1.104364e-05) * (q[i0] * q[i3]) + (4.044206e-03) * (q[i0] * q[i5]) + (-3.852007e-03) * (q[i0] * q[i6])
            + (-2.147826e-04) * (q[i0] * q[i7]) + (-1.860052e-03) * (q[i0] * q[i8]) + (-6.762930e-04) * (q[i0] * q[i9]) + (7.799416e-04) * (q[i0] * q[i10])
            + (1.098684e-03) * (q[i0] * q[i11]) + (-1.804400e-03) * (q[i0] * q[i12]) + (-5.569488e-04) * (q[i0] * q[i15]) + (-3.701069e-04) * (q[i0] * q[i16])
            + (-1.016211e-03) * (q[i0] * q[i19]) + (-9.938613e-04) * (q[i0] * q[i20]) + (-3.788516e-04) * (q[i0] * q[i21]) + (6.018258e-04) * (q[i0] * q[i22])
            + (-1.277614e-02) * (q[i1] * q[i2]) + (6.927884e-06) * (q[i1] * q[i3]) + (6.049404e-04) * (q[i1] * q[i5]) + (-4.309909e-04) * (q[i1] * q[i6])
            + (1.215547e-02) * (q[i1] * q[i7]) + (-2.884023e-03) * (q[i1] * q[i8]) + (-1.469909e-03) * (q[i1] * q[i9]) + (1.007366e-03) * (q[i1] * q[i10])
            + (-1.716275e-03) * (q[i1] * q[i11]) + (6.743792e-05) * (q[i1] * q[i12]) + (-8.506545e-04) * (q[i1] * q[i15]) + (-2.229472e-03) * (q[i1] * q[i16])
            + (-1.241991e-03) * (q[i1] * q[i19]) + (-1.501474e-03) * (q[i1] * q[i20]) + (-7.710065e-04) * (q[i1] * q[i21]) + (-8.268980e-04) * (q[i1] * q[i22])
            + (2.120604e-03) * (q[i2] * q[i3]) + (5.443058e-03) * (q[i2] * q[i5]) + (-1.066613e-03) * (q[i2] * q[i6]) + (3.717713e-03) * (q[i2] * q[i7])
            + (-3.042462e-03) * (q[i2] * q[i8]) + (4.203955e-04) * (q[i2] * q[i9]) + (-1.730749e-04) * (q[i2] * q[i10]) + (-4.936573e-04) * (q[i2] * q[i11])
            + (5.365713e-04) * (q[i2] * q[i12]) + (-9.815592e-05) * (q[i2] * q[i15]) + (-1.455408e-03) * (q[i2] * q[i16]) + (-5.021187e-04) * (q[i2] * q[i19])
            + (9.312876e-04) * (q[i2] * q[i20]) + (9.853955e-04) * (q[i2] * q[i21]) + (-7.527671e-04) * (q[i2] * q[i22]) + (-1.625450e-03) * (q[i3] * q[i5])
            + (2.142900e-03) * (q[i3] * q[i6]) + (-2.125483e-03) * (q[i3] * q[i7]) + (-2.209751e-05) * (q[i3] * q[i8]) + (-4.865914e-05) * (q[i3] * q[i9])
            + (5.663865e-05) * (q[i3] * q[i10]) + (1.984605e-03) * (q[i3] * q[i11]) + (1.988030e-03) * (q[i3] * q[i12]) + (-6.454528e-05) * (q[i3] * q[i15])
            + (7.420731e-05) * (q[i3] * q[i16]) + (-7.927073e-04) * (q[i3] * q[i19]) + (7.792435e-04) * (q[i3] * q[i20]) + (-7.415448e-04) * (q[i3] * q[i21])
            + (-7.459352e-04) * (q[i3] * q[i22]) + (-5.931727e-04) * (q[i5] * q[i6]) + (6.315469e-03) * (q[i5] * q[i7]) + (4.671618e-04) * (q[i5] * q[i8])
            + (1.461121e-03) * (q[i5] * q[i9]) + (1.876810e-04) * (q[i5] * q[i10]) + (-3.199444e-04) * (q[i5] * q[i11]) + (-8.964191e-04) * (q[i5] * q[i12])
            + (-9.330154e-04) * (q[i5] * q[i15]) + (-3.794327e-04) * (q[i5] * q[i16]) + (1.241729e-03) * (q[i5] * q[i19]) + (8.317277e-04) * (q[i5] * q[i20])
            + (1.183960e-03) * (q[i5] * q[i21]) + (7.428462e-05) * (q[i5] * q[i22]) + (3.288686e-04) * (q[i6] * q[i7]) + (-3.885931e-04) * (q[i6] * q[i8])
            + (1.505838e-03) * (q[i6] * q[i9]) + (1.077158e-04) * (q[i6] * q[i10]) + (-7.801497e-04) * (q[i6] * q[i11]) + (8.819036e-04) * (q[i6] * q[i12])
            + (-1.836521e-03) * (q[i6] * q[i15]) + (-7.226764e-04) * (q[i6] * q[i16]) + (1.863697e-03) * (q[i6] * q[i19]) + (-1.470723e-04) * (q[i6] * q[i20])
            + (-7.265601e-04) * (q[i6] * q[i21]) + (-1.339614e-03) * (q[i6] * q[i22]) + (-3.720154e-03) * (q[i7] * q[i8]) + (5.630538e-04) * (q[i7] * q[i9])
            + (-1.020911e-02) * (q[i7] * q[i10]) + (-1.659809e-04) * (q[i7] * q[i11]) + (-1.034888e-04) * (q[i7] * q[i12]) + (-1.031517e-03) * (q[i7] * q[i15])
            + (1.346195e-03) * (q[i7] * q[i16]) + (-1.888206e-04) * (q[i7] * q[i19]) + (-2.927266e-03) * (q[i7] * q[i20]) + (-3.443914e-04) * (q[i7] * q[i21])
            + (8.115579e-04) * (q[i7] * q[i22]) + (9.636456e-04) * (q[i8] * q[i9]) + (-7.088078e-04) * (q[i8] * q[i10]) + (-1.784835e-03) * (q[i8] * q[i11])
            + (8.717079e-04) * (q[i8] * q[i12]) + (-2.745696e-04) * (q[i8] * q[i15]) + (4.798022e-04) * (q[i8] * q[i16]) + (5.460890e-05) * (q[i8] * q[i19])
            + (-1.055922e-03) * (q[i8] * q[i20]) + (1.136042e-04) * (q[i8] * q[i21]) + (-1.860377e-04) * (q[i8] * q[i22]) + (-1.116048e-03) * (q[i9] * q[i10])
            + (-2.538559e-04) * (q[i9] * q[i11]) + (7.054069e-05) * (q[i9] * q[i12]) + (-6.173504e-05) * (q[i9] * q[i15]) + (-6.497450e-04) * (q[i9] * q[i16])
            + (-1.113940e-04) * (q[i9] * q[i19]) + (9.484636e-04) * (q[i9] * q[i20]) + (-3.795851e-04) * (q[i9] * q[i21]) + (2.792880e-04) * (q[i9] * q[i22])
            + (-2.539604e-04) * (q[i10] * q[i11]) + (5.829149e-04) * (q[i10] * q[i12]) + (-4.795333e-04) * (q[i10] * q[i15])
            + (-2.686769e-04) * (q[i10] * q[i16]) + (-1.089708e-03) * (q[i10] * q[i19]) + (2.507049e-04) * (q[i10] * q[i20])
            + (1.679941e-04) * (q[i10] * q[i21]) + (-2.440550e-04) * (q[i10] * q[i22]) + (-9.341547e-04) * (q[i11] * q[i12])
            + (4.198854e-04) * (q[i11] * q[i15]) + (-3.551721e-04) * (q[i11] * q[i16]) + (-8.331422e-04) * (q[i11] * q[i19])
            + (5.438664e-04) * (q[i11] * q[i20]) + (-2.493820e-04) * (q[i11] * q[i21]) + (8.925204e-04) * (q[i11] * q[i22])
            + (-9.618192e-04) * (q[i12] * q[i15]) + (2.398704e-03) * (q[i12] * q[i16]) + (5.877753e-05) * (q[i12] * q[i19])
            + (-3.604102e-04) * (q[i12] * q[i20]) + (-1.056132e-03) * (q[i12] * q[i21]) + (-8.202490e-04) * (q[i12] * q[i22])
            + (6.267090e-04) * (q[i15] * q[i16]) + (4.461442e-04) * (q[i15] * q[i19]) + (7.146477e-04) * (q[i15] * q[i20]) + (5.753389e-04) * (q[i15] * q[i21])
            + (1.739075e-05) * (q[i15] * q[i22]) + (-5.510075e-04) * (q[i16] * q[i19]) + (-4.883223e-04) * (q[i16] * q[i20])
            + (-1.986130e-04) * (q[i16] * q[i21]) + (1.665167e-04) * (q[i16] * q[i22]) + (6.201209e-04) * (q[i19] * q[i20]) + (1.730511e-04) * (q[i19] * q[i21])
            + (3.040057e-04) * (q[i19] * q[i22]) + (3.878512e-04) * (q[i20] * q[i21]) + (-1.278620e-03) * (q[i20] * q[i22])
            + (2.073554e-04) * (q[i21] * q[i22]);
   }

   public void getJQx5(double[] q, double[][] JQ)
   {
      JQ[1][i5] = (1.272279e-01) * (1) + (-1.522872e-05) * ((2) * q[i5]) + (2.972583e-03) * (q[i0]) + (-2.945678e-03) * (q[i1]) + (9.527315e-05) * (q[i2])
            + (-9.794577e-03) * (q[i3]) + (9.743681e-03) * (q[i4]) + (-1.444991e-03) * (q[i6]) + (-1.434103e-03) * (q[i7]) + (4.079506e-04) * (q[i8])
            + (2.276604e-03) * (q[i9]) + (2.289996e-03) * (q[i10]) + (5.436482e-03) * (q[i11]) + (-5.465597e-03) * (q[i12]) + (-1.788624e-02) * (q[i15])
            + (-1.806450e-02) * (q[i16]) + (-5.391021e-04) * (q[i19]) + (-5.439486e-04) * (q[i20]) + (-8.173700e-04) * (q[i21]) + (8.332752e-04) * (q[i22])
            + (-3.507112e-03) * (q[i0] * q[i0]) + (-3.462911e-03) * (q[i1] * q[i1]) + (-2.711721e-02) * (q[i2] * q[i2]) + (7.210215e-03) * (q[i3] * q[i3])
            + (7.177278e-03) * (q[i4] * q[i4]) + (-2.019421e-03) * ((2) * q[i0] * q[i5]) + (-1.986907e-03) * ((2) * q[i1] * q[i5])
            + (-4.393894e-04) * ((2) * q[i2] * q[i5]) + (-4.753110e-04) * ((2) * q[i3] * q[i5]) + (-4.325963e-04) * ((2) * q[i4] * q[i5])
            + (-4.778403e-03) * ((3) * q[i5] * q[i5]) + (-1.737774e-04) * ((2) * q[i5] * q[i6]) + (1.739067e-04) * ((2) * q[i5] * q[i7])
            + (8.966249e-06) * ((2) * q[i5] * q[i8]) + (1.513727e-04) * ((2) * q[i5] * q[i9]) + (-1.422913e-04) * ((2) * q[i5] * q[i10])
            + (2.879716e-04) * ((2) * q[i5] * q[i11]) + (2.853626e-04) * ((2) * q[i5] * q[i12]) + (1.071036e-04) * ((2) * q[i5] * q[i15])
            + (-9.213145e-05) * ((2) * q[i5] * q[i16]) + (4.846585e-04) * ((2) * q[i5] * q[i19]) + (-4.678912e-04) * ((2) * q[i5] * q[i20])
            + (-4.744346e-04) * ((2) * q[i5] * q[i21]) + (-4.877765e-04) * ((2) * q[i5] * q[i22]) + (4.401184e-03) * (q[i6] * q[i6])
            + (4.394210e-03) * (q[i7] * q[i7]) + (-2.642023e-02) * (q[i8] * q[i8]) + (1.952825e-03) * (q[i9] * q[i9]) + (1.924849e-03) * (q[i10] * q[i10])
            + (-1.257756e-03) * (q[i11] * q[i11]) + (-1.331503e-03) * (q[i12] * q[i12]) + (-3.172480e-03) * (q[i15] * q[i15])
            + (-3.193741e-03) * (q[i16] * q[i16]) + (3.073737e-04) * (q[i19] * q[i19]) + (3.262443e-04) * (q[i20] * q[i20])
            + (-1.374830e-03) * (q[i21] * q[i21]) + (-1.379575e-03) * (q[i22] * q[i22]) + (-5.894587e-03) * (q[i0] * q[i1]) + (-1.194723e-02) * (q[i0] * q[i2])
            + (6.232324e-04) * (q[i0] * q[i3]) + (4.044206e-03) * (q[i0] * q[i4]) + (-8.060198e-03) * (q[i0] * q[i6]) + (-4.824738e-04) * (q[i0] * q[i7])
            + (-3.618220e-04) * (q[i0] * q[i8]) + (4.046716e-04) * (q[i0] * q[i9]) + (-8.990503e-04) * (q[i0] * q[i10]) + (-6.948002e-05) * (q[i0] * q[i11])
            + (2.869565e-04) * (q[i0] * q[i12]) + (7.623461e-04) * (q[i0] * q[i15]) + (-1.635196e-03) * (q[i0] * q[i16]) + (-8.694627e-04) * (q[i0] * q[i19])
            + (1.593632e-04) * (q[i0] * q[i20]) + (6.680767e-04) * (q[i0] * q[i21]) + (-2.400176e-04) * (q[i0] * q[i22]) + (-1.187656e-02) * (q[i1] * q[i2])
            + (4.044748e-03) * (q[i1] * q[i3]) + (6.049404e-04) * (q[i1] * q[i4]) + (4.644945e-04) * (q[i1] * q[i6]) + (8.116384e-03) * (q[i1] * q[i7])
            + (3.976265e-04) * (q[i1] * q[i8]) + (8.545363e-04) * (q[i1] * q[i9]) + (-3.880431e-04) * (q[i1] * q[i10]) + (3.387693e-04) * (q[i1] * q[i11])
            + (-6.398775e-05) * (q[i1] * q[i12]) + (1.632663e-03) * (q[i1] * q[i15]) + (-7.627753e-04) * (q[i1] * q[i16]) + (-1.608649e-04) * (q[i1] * q[i19])
            + (8.323787e-04) * (q[i1] * q[i20]) + (-2.455253e-04) * (q[i1] * q[i21]) + (6.741223e-04) * (q[i1] * q[i22]) + (5.379433e-03) * (q[i2] * q[i3])
            + (5.443058e-03) * (q[i2] * q[i4]) + (-1.897680e-04) * (q[i2] * q[i6]) + (1.789531e-04) * (q[i2] * q[i7]) + (-2.629788e-05) * (q[i2] * q[i8])
            + (2.116555e-03) * (q[i2] * q[i9]) + (-2.136809e-03) * (q[i2] * q[i10]) + (1.737642e-03) * (q[i2] * q[i11]) + (1.689365e-03) * (q[i2] * q[i12])
            + (-1.444219e-03) * (q[i2] * q[i15]) + (1.441259e-03) * (q[i2] * q[i16]) + (1.186527e-03) * (q[i2] * q[i19]) + (-1.194357e-03) * (q[i2] * q[i20])
            + (5.688177e-04) * (q[i2] * q[i21]) + (5.556571e-04) * (q[i2] * q[i22]) + (-1.625450e-03) * (q[i3] * q[i4]) + (-6.349387e-03) * (q[i3] * q[i6])
            + (5.954575e-04) * (q[i3] * q[i7]) + (-4.838856e-04) * (q[i3] * q[i8]) + (-1.903444e-04) * (q[i3] * q[i9]) + (-1.450961e-03) * (q[i3] * q[i10])
            + (-8.940210e-04) * (q[i3] * q[i11]) + (-3.053467e-04) * (q[i3] * q[i12]) + (3.455274e-04) * (q[i3] * q[i15]) + (9.330827e-04) * (q[i3] * q[i16])
            + (-8.253939e-04) * (q[i3] * q[i19]) + (-1.240502e-03) * (q[i3] * q[i20]) + (3.996984e-05) * (q[i3] * q[i21]) + (1.160825e-03) * (q[i3] * q[i22])
            + (-5.931727e-04) * (q[i4] * q[i6]) + (6.315469e-03) * (q[i4] * q[i7]) + (4.671618e-04) * (q[i4] * q[i8]) + (1.461121e-03) * (q[i4] * q[i9])
            + (1.876810e-04) * (q[i4] * q[i10]) + (-3.199444e-04) * (q[i4] * q[i11]) + (-8.964191e-04) * (q[i4] * q[i12]) + (-9.330154e-04) * (q[i4] * q[i15])
            + (-3.794327e-04) * (q[i4] * q[i16]) + (1.241729e-03) * (q[i4] * q[i19]) + (8.317277e-04) * (q[i4] * q[i20]) + (1.183960e-03) * (q[i4] * q[i21])
            + (7.428462e-05) * (q[i4] * q[i22]) + (2.967315e-03) * (q[i6] * q[i7]) + (1.843466e-03) * (q[i6] * q[i8]) + (3.600806e-03) * (q[i6] * q[i9])
            + (-1.788734e-03) * (q[i6] * q[i10]) + (1.185197e-04) * (q[i6] * q[i11]) + (-1.385588e-03) * (q[i6] * q[i12]) + (-2.008343e-04) * (q[i6] * q[i15])
            + (3.957857e-04) * (q[i6] * q[i16]) + (2.459765e-04) * (q[i6] * q[i19]) + (6.275868e-04) * (q[i6] * q[i20]) + (4.400469e-04) * (q[i6] * q[i21])
            + (5.221971e-04) * (q[i6] * q[i22]) + (1.807979e-03) * (q[i7] * q[i8]) + (-1.811998e-03) * (q[i7] * q[i9]) + (3.594889e-03) * (q[i7] * q[i10])
            + (1.385137e-03) * (q[i7] * q[i11]) + (-1.108449e-04) * (q[i7] * q[i12]) + (3.823836e-04) * (q[i7] * q[i15]) + (-2.080655e-04) * (q[i7] * q[i16])
            + (6.288068e-04) * (q[i7] * q[i19]) + (2.373423e-04) * (q[i7] * q[i20]) + (-5.079405e-04) * (q[i7] * q[i21]) + (-4.616325e-04) * (q[i7] * q[i22])
            + (-1.520993e-03) * (q[i8] * q[i9]) + (-1.520071e-03) * (q[i8] * q[i10]) + (1.744742e-03) * (q[i8] * q[i11]) + (-1.778833e-03) * (q[i8] * q[i12])
            + (1.608008e-03) * (q[i8] * q[i15]) + (1.625924e-03) * (q[i8] * q[i16]) + (-1.509317e-03) * (q[i8] * q[i19]) + (-1.517377e-03) * (q[i8] * q[i20])
            + (6.891492e-04) * (q[i8] * q[i21]) + (-7.023000e-04) * (q[i8] * q[i22]) + (3.736259e-04) * (q[i9] * q[i10]) + (-3.088273e-04) * (q[i9] * q[i11])
            + (-1.763527e-04) * (q[i9] * q[i12]) + (-1.271194e-05) * (q[i9] * q[i15]) + (-2.967064e-06) * (q[i9] * q[i16]) + (-1.319905e-04) * (q[i9] * q[i19])
            + (6.840200e-05) * (q[i9] * q[i20]) + (5.699121e-05) * (q[i9] * q[i21]) + (-1.983944e-04) * (q[i9] * q[i22]) + (1.690130e-04) * (q[i10] * q[i11])
            + (3.018424e-04) * (q[i10] * q[i12]) + (1.603742e-06) * (q[i10] * q[i15]) + (-2.413245e-05) * (q[i10] * q[i16]) + (6.552438e-05) * (q[i10] * q[i19])
            + (-1.415325e-04) * (q[i10] * q[i20]) + (1.988570e-04) * (q[i10] * q[i21]) + (-7.148407e-05) * (q[i10] * q[i22])
            + (1.094104e-03) * (q[i11] * q[i12]) + (2.062101e-03) * (q[i11] * q[i15]) + (-6.159860e-04) * (q[i11] * q[i16]) + (6.998644e-04) * (q[i11] * q[i19])
            + (-4.212765e-04) * (q[i11] * q[i20]) + (-2.158314e-04) * (q[i11] * q[i21]) + (-3.102334e-04) * (q[i11] * q[i22])
            + (6.111405e-04) * (q[i12] * q[i15]) + (-2.064967e-03) * (q[i12] * q[i16]) + (4.324835e-04) * (q[i12] * q[i19])
            + (-7.160581e-04) * (q[i12] * q[i20]) + (-3.110414e-04) * (q[i12] * q[i21]) + (-2.057889e-04) * (q[i12] * q[i22])
            + (-1.431438e-04) * (q[i15] * q[i16]) + (-7.355210e-04) * (q[i15] * q[i19]) + (-3.359705e-04) * (q[i15] * q[i20])
            + (-1.399082e-03) * (q[i15] * q[i21]) + (1.192572e-04) * (q[i15] * q[i22]) + (-3.321778e-04) * (q[i16] * q[i19])
            + (-7.404865e-04) * (q[i16] * q[i20]) + (-1.216028e-04) * (q[i16] * q[i21]) + (1.415512e-03) * (q[i16] * q[i22])
            + (-5.472268e-04) * (q[i19] * q[i20]) + (-1.777132e-03) * (q[i19] * q[i21]) + (5.059876e-04) * (q[i19] * q[i22])
            + (-5.014894e-04) * (q[i20] * q[i21]) + (1.779544e-03) * (q[i20] * q[i22]) + (-5.758126e-04) * (q[i21] * q[i22]);
   }

   public void getJQx6(double[] q, double[][] JQ)
   {
      JQ[1][i6] = (3.137786e-03) * (1) + (1.824246e-02) * ((2) * q[i6]) + (9.063135e-02) * (q[i0]) + (-1.054391e-02) * (q[i1]) + (1.591417e-02) * (q[i2])
            + (1.055381e-02) * (q[i3]) + (-4.531510e-03) * (q[i4]) + (-1.444991e-03) * (q[i5]) + (-5.334098e-05) * (q[i7]) + (2.546911e-03) * (q[i8])
            + (8.509535e-03) * (q[i9]) + (-4.073500e-04) * (q[i10]) + (3.803170e-03) * (q[i11]) + (4.725349e-03) * (q[i12]) + (-9.283013e-04) * (q[i15])
            + (-7.215826e-04) * (q[i16]) + (2.066508e-03) * (q[i19]) + (1.200329e-03) * (q[i20]) + (-7.083634e-04) * (q[i21]) + (-1.230140e-03) * (q[i22])
            + (1.253647e-03) * (q[i0] * q[i0]) + (-2.010742e-03) * (q[i1] * q[i1]) + (9.419021e-04) * (q[i2] * q[i2]) + (1.880440e-03) * (q[i3] * q[i3])
            + (-2.780751e-04) * (q[i4] * q[i4]) + (-1.737774e-04) * (q[i5] * q[i5]) + (3.867929e-03) * ((2) * q[i0] * q[i6])
            + (-3.455323e-03) * ((2) * q[i1] * q[i6]) + (-4.848208e-04) * ((2) * q[i2] * q[i6]) + (-2.187062e-02) * ((2) * q[i3] * q[i6])
            + (3.542552e-03) * ((2) * q[i4] * q[i6]) + (4.401184e-03) * ((2) * q[i5] * q[i6]) + (-6.368184e-04) * ((3) * q[i6] * q[i6])
            + (-1.625084e-03) * ((2) * q[i6] * q[i7]) + (-3.501911e-04) * ((2) * q[i6] * q[i8]) + (-2.486045e-03) * ((2) * q[i6] * q[i9])
            + (1.582213e-04) * ((2) * q[i6] * q[i10]) + (5.171001e-04) * ((2) * q[i6] * q[i11]) + (-6.927001e-04) * ((2) * q[i6] * q[i12])
            + (2.228535e-04) * ((2) * q[i6] * q[i15]) + (1.106866e-03) * ((2) * q[i6] * q[i16]) + (-1.366856e-04) * ((2) * q[i6] * q[i19])
            + (4.183499e-04) * ((2) * q[i6] * q[i20]) + (2.497640e-04) * ((2) * q[i6] * q[i21]) + (-6.035456e-05) * ((2) * q[i6] * q[i22])
            + (1.623807e-03) * (q[i7] * q[i7]) + (-1.543149e-04) * (q[i8] * q[i8]) + (-2.439674e-03) * (q[i9] * q[i9]) + (-9.914628e-04) * (q[i10] * q[i10])
            + (-5.734439e-05) * (q[i11] * q[i11]) + (4.222588e-05) * (q[i12] * q[i12]) + (8.926814e-05) * (q[i15] * q[i15]) + (5.766638e-05) * (q[i16] * q[i16])
            + (-1.450210e-04) * (q[i19] * q[i19]) + (-3.249408e-04) * (q[i20] * q[i20]) + (-1.487235e-04) * (q[i21] * q[i21])
            + (3.754935e-04) * (q[i22] * q[i22]) + (1.950107e-03) * (q[i0] * q[i1]) + (1.345896e-03) * (q[i0] * q[i2]) + (-1.220277e-02) * (q[i0] * q[i3])
            + (-3.852007e-03) * (q[i0] * q[i4]) + (-8.060198e-03) * (q[i0] * q[i5]) + (-4.657516e-04) * (q[i0] * q[i7]) + (7.409014e-03) * (q[i0] * q[i8])
            + (-1.360672e-02) * (q[i0] * q[i9]) + (1.714814e-03) * (q[i0] * q[i10]) + (-1.493674e-03) * (q[i0] * q[i11]) + (3.205563e-04) * (q[i0] * q[i12])
            + (3.511050e-04) * (q[i0] * q[i15]) + (2.069774e-03) * (q[i0] * q[i16]) + (1.222475e-03) * (q[i0] * q[i19]) + (1.359705e-03) * (q[i0] * q[i20])
            + (-8.987898e-04) * (q[i0] * q[i21]) + (6.300372e-04) * (q[i0] * q[i22]) + (2.015449e-03) * (q[i1] * q[i2]) + (1.776476e-04) * (q[i1] * q[i3])
            + (-4.309909e-04) * (q[i1] * q[i4]) + (4.644945e-04) * (q[i1] * q[i5]) + (-5.072719e-04) * (q[i1] * q[i7]) + (-1.312707e-03) * (q[i1] * q[i8])
            + (3.417191e-03) * (q[i1] * q[i9]) + (-1.111062e-04) * (q[i1] * q[i10]) + (1.669093e-03) * (q[i1] * q[i11]) + (9.281863e-04) * (q[i1] * q[i12])
            + (7.933511e-04) * (q[i1] * q[i15]) + (-6.163040e-04) * (q[i1] * q[i16]) + (-9.142841e-04) * (q[i1] * q[i19]) + (-6.114105e-04) * (q[i1] * q[i20])
            + (8.439397e-04) * (q[i1] * q[i21]) + (1.852810e-04) * (q[i1] * q[i22]) + (-3.754849e-03) * (q[i2] * q[i3]) + (-1.066613e-03) * (q[i2] * q[i4])
            + (-1.897680e-04) * (q[i2] * q[i5]) + (3.344318e-03) * (q[i2] * q[i7]) + (5.734035e-03) * (q[i2] * q[i8]) + (-2.505272e-03) * (q[i2] * q[i9])
            + (7.022326e-04) * (q[i2] * q[i10]) + (-7.407081e-04) * (q[i2] * q[i11]) + (-3.079661e-04) * (q[i2] * q[i12]) + (5.709624e-04) * (q[i2] * q[i15])
            + (2.251452e-04) * (q[i2] * q[i16]) + (1.268388e-03) * (q[i2] * q[i19]) + (3.018136e-04) * (q[i2] * q[i20]) + (1.285496e-04) * (q[i2] * q[i21])
            + (3.242108e-04) * (q[i2] * q[i22]) + (2.142900e-03) * (q[i3] * q[i4]) + (-6.349387e-03) * (q[i3] * q[i5]) + (3.245163e-04) * (q[i3] * q[i7])
            + (-3.734237e-03) * (q[i3] * q[i8]) + (-1.030080e-02) * (q[i3] * q[i9]) + (5.297648e-04) * (q[i3] * q[i10]) + (1.117085e-04) * (q[i3] * q[i11])
            + (1.625795e-04) * (q[i3] * q[i12]) + (1.331850e-03) * (q[i3] * q[i15]) + (-1.027732e-03) * (q[i3] * q[i16]) + (-2.949664e-03) * (q[i3] * q[i19])
            + (-1.961914e-04) * (q[i3] * q[i20]) + (-8.116925e-04) * (q[i3] * q[i21]) + (3.394242e-04) * (q[i3] * q[i22]) + (-5.931727e-04) * (q[i4] * q[i5])
            + (3.288686e-04) * (q[i4] * q[i7]) + (-3.885931e-04) * (q[i4] * q[i8]) + (1.505838e-03) * (q[i4] * q[i9]) + (1.077158e-04) * (q[i4] * q[i10])
            + (-7.801497e-04) * (q[i4] * q[i11]) + (8.819036e-04) * (q[i4] * q[i12]) + (-1.836521e-03) * (q[i4] * q[i15]) + (-7.226764e-04) * (q[i4] * q[i16])
            + (1.863697e-03) * (q[i4] * q[i19]) + (-1.470723e-04) * (q[i4] * q[i20]) + (-7.265601e-04) * (q[i4] * q[i21]) + (-1.339614e-03) * (q[i4] * q[i22])
            + (2.967315e-03) * (q[i5] * q[i7]) + (1.843466e-03) * (q[i5] * q[i8]) + (3.600806e-03) * (q[i5] * q[i9]) + (-1.788734e-03) * (q[i5] * q[i10])
            + (1.185197e-04) * (q[i5] * q[i11]) + (-1.385588e-03) * (q[i5] * q[i12]) + (-2.008343e-04) * (q[i5] * q[i15]) + (3.957857e-04) * (q[i5] * q[i16])
            + (2.459765e-04) * (q[i5] * q[i19]) + (6.275868e-04) * (q[i5] * q[i20]) + (4.400469e-04) * (q[i5] * q[i21]) + (5.221971e-04) * (q[i5] * q[i22])
            + (8.144014e-06) * (q[i7] * q[i8]) + (1.094756e-03) * (q[i7] * q[i9]) + (-1.072949e-03) * (q[i7] * q[i10]) + (9.749898e-04) * (q[i7] * q[i11])
            + (9.681896e-04) * (q[i7] * q[i12]) + (-1.479991e-04) * (q[i7] * q[i15]) + (1.486869e-04) * (q[i7] * q[i16]) + (3.730544e-04) * (q[i7] * q[i19])
            + (-3.841827e-04) * (q[i7] * q[i20]) + (-2.921076e-04) * (q[i7] * q[i21]) + (-2.994227e-04) * (q[i7] * q[i22]) + (2.919645e-04) * (q[i8] * q[i9])
            + (5.735390e-05) * (q[i8] * q[i10]) + (3.118483e-04) * (q[i8] * q[i11]) + (3.574590e-04) * (q[i8] * q[i12]) + (1.204213e-05) * (q[i8] * q[i15])
            + (8.807517e-04) * (q[i8] * q[i16]) + (-3.218953e-04) * (q[i8] * q[i19]) + (-2.926902e-06) * (q[i8] * q[i20]) + (3.936861e-05) * (q[i8] * q[i21])
            + (6.272907e-04) * (q[i8] * q[i22]) + (1.057842e-03) * (q[i9] * q[i10]) + (-8.023481e-04) * (q[i9] * q[i11]) + (-6.623801e-05) * (q[i9] * q[i12])
            + (-5.787308e-05) * (q[i9] * q[i15]) + (5.530778e-04) * (q[i9] * q[i16]) + (-5.147763e-04) * (q[i9] * q[i19]) + (-2.811633e-04) * (q[i9] * q[i20])
            + (-5.704405e-06) * (q[i9] * q[i21]) + (6.697351e-06) * (q[i9] * q[i22]) + (1.632620e-04) * (q[i10] * q[i11]) + (1.932459e-04) * (q[i10] * q[i12])
            + (5.898438e-05) * (q[i10] * q[i15]) + (-3.333471e-05) * (q[i10] * q[i16]) + (-3.367955e-06) * (q[i10] * q[i19])
            + (-6.403651e-05) * (q[i10] * q[i20]) + (6.740169e-04) * (q[i10] * q[i21]) + (3.724830e-04) * (q[i10] * q[i22])
            + (-9.789896e-05) * (q[i11] * q[i12]) + (1.189101e-03) * (q[i11] * q[i15]) + (1.504720e-04) * (q[i11] * q[i16]) + (2.453448e-05) * (q[i11] * q[i19])
            + (1.247018e-03) * (q[i11] * q[i20]) + (-3.542402e-04) * (q[i11] * q[i21]) + (-3.928276e-04) * (q[i11] * q[i22])
            + (1.037932e-03) * (q[i12] * q[i15]) + (3.039606e-04) * (q[i12] * q[i16]) + (1.155972e-04) * (q[i12] * q[i19]) + (-1.390614e-04) * (q[i12] * q[i20])
            + (9.065239e-05) * (q[i12] * q[i21]) + (-5.720250e-04) * (q[i12] * q[i22]) + (9.620974e-05) * (q[i15] * q[i16]) + (2.382661e-04) * (q[i15] * q[i19])
            + (-2.391586e-04) * (q[i15] * q[i20]) + (-5.152226e-05) * (q[i15] * q[i21]) + (2.113318e-04) * (q[i15] * q[i22])
            + (-4.223975e-04) * (q[i16] * q[i19]) + (1.109683e-05) * (q[i16] * q[i20]) + (1.089463e-04) * (q[i16] * q[i21]) + (1.466666e-04) * (q[i16] * q[i22])
            + (1.838184e-04) * (q[i19] * q[i20]) + (-1.257420e-04) * (q[i19] * q[i21]) + (-7.666956e-05) * (q[i19] * q[i22])
            + (8.421887e-04) * (q[i20] * q[i21]) + (-6.182464e-04) * (q[i20] * q[i22]) + (-1.593029e-04) * (q[i21] * q[i22]);
   }

   public void getJQx7(double[] q, double[][] JQ)
   {
      JQ[1][i7] = (-3.106478e-03) * (1) + (-1.817250e-02) * ((2) * q[i7]) + (-1.051662e-02) * (q[i0]) + (9.010674e-02) * (q[i1]) + (1.577288e-02) * (q[i2])
            + (-4.554090e-03) * (q[i3]) + (1.051373e-02) * (q[i4]) + (-1.434103e-03) * (q[i5]) + (-5.334098e-05) * (q[i6]) + (-2.560472e-03) * (q[i8])
            + (3.818315e-04) * (q[i9]) + (-8.383535e-03) * (q[i10]) + (4.708322e-03) * (q[i11]) + (3.765473e-03) * (q[i12]) + (7.197612e-04) * (q[i15])
            + (9.171318e-04) * (q[i16]) + (-1.185100e-03) * (q[i19]) + (-2.023647e-03) * (q[i20]) + (-1.223041e-03) * (q[i21]) + (-6.863549e-04) * (q[i22])
            + (2.032812e-03) * (q[i0] * q[i0]) + (-1.266027e-03) * (q[i1] * q[i1]) + (-9.400876e-04) * (q[i2] * q[i2]) + (2.994120e-04) * (q[i3] * q[i3])
            + (-1.868758e-03) * (q[i4] * q[i4]) + (1.739067e-04) * (q[i5] * q[i5]) + (-1.625084e-03) * (q[i6] * q[i6]) + (-3.453669e-03) * ((2) * q[i0] * q[i7])
            + (3.902302e-03) * ((2) * q[i1] * q[i7]) + (-4.773915e-04) * ((2) * q[i2] * q[i7]) + (3.528850e-03) * ((2) * q[i3] * q[i7])
            + (-2.179241e-02) * ((2) * q[i4] * q[i7]) + (4.394210e-03) * ((2) * q[i5] * q[i7]) + (1.623807e-03) * ((2) * q[i6] * q[i7])
            + (6.311920e-04) * ((3) * q[i7] * q[i7]) + (3.571365e-04) * ((2) * q[i7] * q[i8]) + (-1.603254e-04) * ((2) * q[i7] * q[i9])
            + (2.457586e-03) * ((2) * q[i7] * q[i10]) + (-6.901101e-04) * ((2) * q[i7] * q[i11]) + (5.123069e-04) * ((2) * q[i7] * q[i12])
            + (-1.107298e-03) * ((2) * q[i7] * q[i15]) + (-2.299380e-04) * ((2) * q[i7] * q[i16]) + (-4.174993e-04) * ((2) * q[i7] * q[i19])
            + (1.400944e-04) * ((2) * q[i7] * q[i20]) + (-5.819971e-05) * ((2) * q[i7] * q[i21]) + (2.482698e-04) * ((2) * q[i7] * q[i22])
            + (1.455535e-04) * (q[i8] * q[i8]) + (1.011449e-03) * (q[i9] * q[i9]) + (2.404803e-03) * (q[i10] * q[i10]) + (-2.701751e-05) * (q[i11] * q[i11])
            + (6.581599e-05) * (q[i12] * q[i12]) + (-5.139503e-05) * (q[i15] * q[i15]) + (-8.659144e-05) * (q[i16] * q[i16])
            + (3.239674e-04) * (q[i19] * q[i19]) + (1.465960e-04) * (q[i20] * q[i20]) + (-3.780352e-04) * (q[i21] * q[i21]) + (1.424931e-04) * (q[i22] * q[i22])
            + (-1.925395e-03) * (q[i0] * q[i1]) + (-2.001371e-03) * (q[i0] * q[i2]) + (4.599825e-04) * (q[i0] * q[i3]) + (-2.147826e-04) * (q[i0] * q[i4])
            + (-4.824738e-04) * (q[i0] * q[i5]) + (-4.657516e-04) * (q[i0] * q[i6]) + (-1.297956e-03) * (q[i0] * q[i8]) + (-1.327951e-04) * (q[i0] * q[i9])
            + (3.413640e-03) * (q[i0] * q[i10]) + (-9.047856e-04) * (q[i0] * q[i11]) + (-1.669194e-03) * (q[i0] * q[i12]) + (-6.133359e-04) * (q[i0] * q[i15])
            + (7.881685e-04) * (q[i0] * q[i16]) + (-6.159552e-04) * (q[i0] * q[i19]) + (-9.066067e-04) * (q[i0] * q[i20]) + (-2.006549e-04) * (q[i0] * q[i21])
            + (-8.392349e-04) * (q[i0] * q[i22]) + (-1.331022e-03) * (q[i1] * q[i2]) + (3.785884e-03) * (q[i1] * q[i3]) + (1.215547e-02) * (q[i1] * q[i4])
            + (8.116384e-03) * (q[i1] * q[i5]) + (-5.072719e-04) * (q[i1] * q[i6]) + (7.388077e-03) * (q[i1] * q[i8]) + (1.747766e-03) * (q[i1] * q[i9])
            + (-1.345148e-02) * (q[i1] * q[i10]) + (-2.929429e-04) * (q[i1] * q[i11]) + (1.519737e-03) * (q[i1] * q[i12]) + (2.038911e-03) * (q[i1] * q[i15])
            + (3.758379e-04) * (q[i1] * q[i16]) + (1.349245e-03) * (q[i1] * q[i19]) + (1.199835e-03) * (q[i1] * q[i20]) + (-6.180849e-04) * (q[i1] * q[i21])
            + (9.134953e-04) * (q[i1] * q[i22]) + (1.078962e-03) * (q[i2] * q[i3]) + (3.717713e-03) * (q[i2] * q[i4]) + (1.789531e-04) * (q[i2] * q[i5])
            + (3.344318e-03) * (q[i2] * q[i6]) + (5.755823e-03) * (q[i2] * q[i8]) + (7.057201e-04) * (q[i2] * q[i9]) + (-2.443772e-03) * (q[i2] * q[i10])
            + (3.318719e-04) * (q[i2] * q[i11]) + (7.383186e-04) * (q[i2] * q[i12]) + (2.397672e-04) * (q[i2] * q[i15]) + (5.739001e-04) * (q[i2] * q[i16])
            + (2.983729e-04) * (q[i2] * q[i19]) + (1.274549e-03) * (q[i2] * q[i20]) + (-3.183364e-04) * (q[i2] * q[i21]) + (-1.257341e-04) * (q[i2] * q[i22])
            + (-2.125483e-03) * (q[i3] * q[i4]) + (5.954575e-04) * (q[i3] * q[i5]) + (3.245163e-04) * (q[i3] * q[i6]) + (-3.796529e-04) * (q[i3] * q[i8])
            + (1.223988e-04) * (q[i3] * q[i9]) + (1.490958e-03) * (q[i3] * q[i10]) + (-9.071750e-04) * (q[i3] * q[i11]) + (7.903289e-04) * (q[i3] * q[i12])
            + (-7.167150e-04) * (q[i3] * q[i15]) + (-1.838161e-03) * (q[i3] * q[i16]) + (-1.571947e-04) * (q[i3] * q[i19]) + (1.872840e-03) * (q[i3] * q[i20])
            + (1.340993e-03) * (q[i3] * q[i21]) + (7.346822e-04) * (q[i3] * q[i22]) + (6.315469e-03) * (q[i4] * q[i5]) + (3.288686e-04) * (q[i4] * q[i6])
            + (-3.720154e-03) * (q[i4] * q[i8]) + (5.630538e-04) * (q[i4] * q[i9]) + (-1.020911e-02) * (q[i4] * q[i10]) + (-1.659809e-04) * (q[i4] * q[i11])
            + (-1.034888e-04) * (q[i4] * q[i12]) + (-1.031517e-03) * (q[i4] * q[i15]) + (1.346195e-03) * (q[i4] * q[i16]) + (-1.888206e-04) * (q[i4] * q[i19])
            + (-2.927266e-03) * (q[i4] * q[i20]) + (-3.443914e-04) * (q[i4] * q[i21]) + (8.115579e-04) * (q[i4] * q[i22]) + (2.967315e-03) * (q[i5] * q[i6])
            + (1.807979e-03) * (q[i5] * q[i8]) + (-1.811998e-03) * (q[i5] * q[i9]) + (3.594889e-03) * (q[i5] * q[i10]) + (1.385137e-03) * (q[i5] * q[i11])
            + (-1.108449e-04) * (q[i5] * q[i12]) + (3.823836e-04) * (q[i5] * q[i15]) + (-2.080655e-04) * (q[i5] * q[i16]) + (6.288068e-04) * (q[i5] * q[i19])
            + (2.373423e-04) * (q[i5] * q[i20]) + (-5.079405e-04) * (q[i5] * q[i21]) + (-4.616325e-04) * (q[i5] * q[i22]) + (8.144014e-06) * (q[i6] * q[i8])
            + (1.094756e-03) * (q[i6] * q[i9]) + (-1.072949e-03) * (q[i6] * q[i10]) + (9.749898e-04) * (q[i6] * q[i11]) + (9.681896e-04) * (q[i6] * q[i12])
            + (-1.479991e-04) * (q[i6] * q[i15]) + (1.486869e-04) * (q[i6] * q[i16]) + (3.730544e-04) * (q[i6] * q[i19]) + (-3.841827e-04) * (q[i6] * q[i20])
            + (-2.921076e-04) * (q[i6] * q[i21]) + (-2.994227e-04) * (q[i6] * q[i22]) + (-5.487595e-05) * (q[i8] * q[i9]) + (-2.853329e-04) * (q[i8] * q[i10])
            + (3.848970e-04) * (q[i8] * q[i11]) + (3.219046e-04) * (q[i8] * q[i12]) + (-8.874344e-04) * (q[i8] * q[i15]) + (-6.459206e-07) * (q[i8] * q[i16])
            + (2.425172e-06) * (q[i8] * q[i19]) + (3.094587e-04) * (q[i8] * q[i20]) + (6.312095e-04) * (q[i8] * q[i21]) + (3.965294e-05) * (q[i8] * q[i22])
            + (-1.062174e-03) * (q[i9] * q[i10]) + (2.032087e-04) * (q[i9] * q[i11]) + (1.531247e-04) * (q[i9] * q[i12]) + (3.024924e-05) * (q[i9] * q[i15])
            + (-5.499585e-05) * (q[i9] * q[i16]) + (6.235673e-05) * (q[i9] * q[i19]) + (-3.490015e-07) * (q[i9] * q[i20]) + (3.703065e-04) * (q[i9] * q[i21])
            + (6.726312e-04) * (q[i9] * q[i22]) + (-6.176399e-05) * (q[i10] * q[i11]) + (-7.972379e-04) * (q[i10] * q[i12])
            + (-5.512347e-04) * (q[i10] * q[i15]) + (6.889978e-05) * (q[i10] * q[i16]) + (2.787552e-04) * (q[i10] * q[i19]) + (5.044036e-04) * (q[i10] * q[i20])
            + (6.170180e-06) * (q[i10] * q[i21]) + (-1.011493e-05) * (q[i10] * q[i22]) + (1.092357e-04) * (q[i11] * q[i12]) + (3.001006e-04) * (q[i11] * q[i15])
            + (1.036485e-03) * (q[i11] * q[i16]) + (-1.439265e-04) * (q[i11] * q[i19]) + (1.173055e-04) * (q[i11] * q[i20]) + (5.647104e-04) * (q[i11] * q[i21])
            + (-8.827581e-05) * (q[i11] * q[i22]) + (1.521089e-04) * (q[i12] * q[i15]) + (1.174695e-03) * (q[i12] * q[i16]) + (1.244186e-03) * (q[i12] * q[i19])
            + (2.493281e-05) * (q[i12] * q[i20]) + (3.914591e-04) * (q[i12] * q[i21]) + (3.562947e-04) * (q[i12] * q[i22]) + (-9.605176e-05) * (q[i15] * q[i16])
            + (-1.144394e-05) * (q[i15] * q[i19]) + (4.214893e-04) * (q[i15] * q[i20]) + (1.458426e-04) * (q[i15] * q[i21]) + (1.105229e-04) * (q[i15] * q[i22])
            + (2.379539e-04) * (q[i16] * q[i19]) + (-2.380913e-04) * (q[i16] * q[i20]) + (2.063762e-04) * (q[i16] * q[i21])
            + (-5.774850e-05) * (q[i16] * q[i22]) + (-1.807043e-04) * (q[i19] * q[i20]) + (-6.118894e-04) * (q[i19] * q[i21])
            + (8.430038e-04) * (q[i19] * q[i22]) + (-8.372730e-05) * (q[i20] * q[i21]) + (-1.313692e-04) * (q[i20] * q[i22])
            + (1.575562e-04) * (q[i21] * q[i22]);
   }

   public void getJQx8(double[] q, double[][] JQ)
   {
      JQ[1][i8] = (-1.146618e-04) * (1) + (-7.790112e-06) * ((2) * q[i8]) + (-3.638438e-03) * (q[i0]) + (-3.676940e-03) * (q[i1]) + (-6.770789e-02) * (q[i2])
            + (2.986873e-03) * (q[i3]) + (3.061437e-03) * (q[i4]) + (4.079506e-04) * (q[i5]) + (2.546911e-03) * (q[i6]) + (-2.560472e-03) * (q[i7])
            + (1.827642e-03) * (q[i9]) + (-1.850971e-03) * (q[i10]) + (1.037983e-02) * (q[i11]) + (1.046921e-02) * (q[i12]) + (9.172387e-03) * (q[i15])
            + (-9.271249e-03) * (q[i16]) + (-4.194115e-04) * (q[i19]) + (4.509890e-04) * (q[i20]) + (2.241619e-03) * (q[i21]) + (2.269841e-03) * (q[i22])
            + (-1.493391e-03) * (q[i0] * q[i0]) + (1.472293e-03) * (q[i1] * q[i1]) + (-1.694320e-05) * (q[i2] * q[i2]) + (7.210461e-04) * (q[i3] * q[i3])
            + (-6.687702e-04) * (q[i4] * q[i4]) + (8.966249e-06) * (q[i5] * q[i5]) + (-3.501911e-04) * (q[i6] * q[i6]) + (3.571365e-04) * (q[i7] * q[i7])
            + (1.790137e-04) * ((2) * q[i0] * q[i8]) + (2.022371e-04) * ((2) * q[i1] * q[i8]) + (1.764850e-03) * ((2) * q[i2] * q[i8])
            + (5.135405e-03) * ((2) * q[i3] * q[i8]) + (5.120441e-03) * ((2) * q[i4] * q[i8]) + (-2.642023e-02) * ((2) * q[i5] * q[i8])
            + (-1.543149e-04) * ((2) * q[i6] * q[i8]) + (1.455535e-04) * ((2) * q[i7] * q[i8]) + (1.545282e-05) * ((3) * q[i8] * q[i8])
            + (-4.521382e-04) * ((2) * q[i8] * q[i9]) + (4.395179e-04) * ((2) * q[i8] * q[i10]) + (-1.781949e-03) * ((2) * q[i8] * q[i11])
            + (-1.816467e-03) * ((2) * q[i8] * q[i12]) + (4.086490e-03) * ((2) * q[i8] * q[i15]) + (-4.139522e-03) * ((2) * q[i8] * q[i16])
            + (-2.545432e-05) * ((2) * q[i8] * q[i19]) + (2.835757e-05) * ((2) * q[i8] * q[i20]) + (-6.908828e-04) * ((2) * q[i8] * q[i21])
            + (-6.994664e-04) * ((2) * q[i8] * q[i22]) + (3.629383e-04) * (q[i9] * q[i9]) + (-3.565921e-04) * (q[i10] * q[i10])
            + (4.277118e-04) * (q[i11] * q[i11]) + (-3.407250e-04) * (q[i12] * q[i12]) + (2.670214e-03) * (q[i15] * q[i15])
            + (-2.682642e-03) * (q[i16] * q[i16]) + (8.407745e-04) * (q[i19] * q[i19]) + (-8.367389e-04) * (q[i20] * q[i20])
            + (6.811219e-04) * (q[i21] * q[i21]) + (-6.902480e-04) * (q[i22] * q[i22]) + (4.442005e-06) * (q[i0] * q[i1]) + (-3.725796e-03) * (q[i0] * q[i2])
            + (2.879513e-03) * (q[i0] * q[i3]) + (-1.860052e-03) * (q[i0] * q[i4]) + (-3.618220e-04) * (q[i0] * q[i5]) + (7.409014e-03) * (q[i0] * q[i6])
            + (-1.297956e-03) * (q[i0] * q[i7]) + (7.407671e-04) * (q[i0] * q[i9]) + (-1.721679e-04) * (q[i0] * q[i10]) + (-5.959634e-04) * (q[i0] * q[i11])
            + (1.649874e-04) * (q[i0] * q[i12]) + (9.418310e-04) * (q[i0] * q[i15]) + (2.296953e-04) * (q[i0] * q[i16]) + (1.349733e-03) * (q[i0] * q[i19])
            + (1.054646e-03) * (q[i0] * q[i20]) + (3.697073e-04) * (q[i0] * q[i21]) + (-3.461998e-04) * (q[i0] * q[i22]) + (3.702176e-03) * (q[i1] * q[i2])
            + (1.894027e-03) * (q[i1] * q[i3]) + (-2.884023e-03) * (q[i1] * q[i4]) + (3.976265e-04) * (q[i1] * q[i5]) + (-1.312707e-03) * (q[i1] * q[i6])
            + (7.388077e-03) * (q[i1] * q[i7]) + (-1.896148e-04) * (q[i1] * q[i9]) + (7.417262e-04) * (q[i1] * q[i10]) + (-1.253281e-04) * (q[i1] * q[i11])
            + (6.184059e-04) * (q[i1] * q[i12]) + (2.099563e-04) * (q[i1] * q[i15]) + (9.333350e-04) * (q[i1] * q[i16]) + (1.060160e-03) * (q[i1] * q[i19])
            + (1.348211e-03) * (q[i1] * q[i20]) + (3.351103e-04) * (q[i1] * q[i21]) + (-3.669231e-04) * (q[i1] * q[i22]) + (3.084937e-03) * (q[i2] * q[i3])
            + (-3.042462e-03) * (q[i2] * q[i4]) + (-2.629788e-05) * (q[i2] * q[i5]) + (5.734035e-03) * (q[i2] * q[i6]) + (5.755823e-03) * (q[i2] * q[i7])
            + (-2.458956e-03) * (q[i2] * q[i9]) + (-2.408560e-03) * (q[i2] * q[i10]) + (-1.244990e-03) * (q[i2] * q[i11]) + (1.411772e-03) * (q[i2] * q[i12])
            + (4.348435e-03) * (q[i2] * q[i15]) + (4.428257e-03) * (q[i2] * q[i16]) + (1.071222e-03) * (q[i2] * q[i19]) + (1.062139e-03) * (q[i2] * q[i20])
            + (-5.368134e-04) * (q[i2] * q[i21]) + (5.417197e-04) * (q[i2] * q[i22]) + (-2.209751e-05) * (q[i3] * q[i4]) + (-4.838856e-04) * (q[i3] * q[i5])
            + (-3.734237e-03) * (q[i3] * q[i6]) + (-3.796529e-04) * (q[i3] * q[i7]) + (-7.091580e-04) * (q[i3] * q[i9]) + (9.674446e-04) * (q[i3] * q[i10])
            + (-8.620245e-04) * (q[i3] * q[i11]) + (1.817324e-03) * (q[i3] * q[i12]) + (4.639511e-04) * (q[i3] * q[i15]) + (-2.946867e-04) * (q[i3] * q[i16])
            + (-1.082728e-03) * (q[i3] * q[i19]) + (4.997872e-05) * (q[i3] * q[i20]) + (1.835052e-04) * (q[i3] * q[i21]) + (-1.137578e-04) * (q[i3] * q[i22])
            + (4.671618e-04) * (q[i4] * q[i5]) + (-3.885931e-04) * (q[i4] * q[i6]) + (-3.720154e-03) * (q[i4] * q[i7]) + (9.636456e-04) * (q[i4] * q[i9])
            + (-7.088078e-04) * (q[i4] * q[i10]) + (-1.784835e-03) * (q[i4] * q[i11]) + (8.717079e-04) * (q[i4] * q[i12]) + (-2.745696e-04) * (q[i4] * q[i15])
            + (4.798022e-04) * (q[i4] * q[i16]) + (5.460890e-05) * (q[i4] * q[i19]) + (-1.055922e-03) * (q[i4] * q[i20]) + (1.136042e-04) * (q[i4] * q[i21])
            + (-1.860377e-04) * (q[i4] * q[i22]) + (1.843466e-03) * (q[i5] * q[i6]) + (1.807979e-03) * (q[i5] * q[i7]) + (-1.520993e-03) * (q[i5] * q[i9])
            + (-1.520071e-03) * (q[i5] * q[i10]) + (1.744742e-03) * (q[i5] * q[i11]) + (-1.778833e-03) * (q[i5] * q[i12]) + (1.608008e-03) * (q[i5] * q[i15])
            + (1.625924e-03) * (q[i5] * q[i16]) + (-1.509317e-03) * (q[i5] * q[i19]) + (-1.517377e-03) * (q[i5] * q[i20]) + (6.891492e-04) * (q[i5] * q[i21])
            + (-7.023000e-04) * (q[i5] * q[i22]) + (8.144014e-06) * (q[i6] * q[i7]) + (2.919645e-04) * (q[i6] * q[i9]) + (5.735390e-05) * (q[i6] * q[i10])
            + (3.118483e-04) * (q[i6] * q[i11]) + (3.574590e-04) * (q[i6] * q[i12]) + (1.204213e-05) * (q[i6] * q[i15]) + (8.807517e-04) * (q[i6] * q[i16])
            + (-3.218953e-04) * (q[i6] * q[i19]) + (-2.926902e-06) * (q[i6] * q[i20]) + (3.936861e-05) * (q[i6] * q[i21]) + (6.272907e-04) * (q[i6] * q[i22])
            + (-5.487595e-05) * (q[i7] * q[i9]) + (-2.853329e-04) * (q[i7] * q[i10]) + (3.848970e-04) * (q[i7] * q[i11]) + (3.219046e-04) * (q[i7] * q[i12])
            + (-8.874344e-04) * (q[i7] * q[i15]) + (-6.459206e-07) * (q[i7] * q[i16]) + (2.425172e-06) * (q[i7] * q[i19]) + (3.094587e-04) * (q[i7] * q[i20])
            + (6.312095e-04) * (q[i7] * q[i21]) + (3.965294e-05) * (q[i7] * q[i22]) + (3.051791e-06) * (q[i9] * q[i10]) + (3.713144e-04) * (q[i9] * q[i11])
            + (1.890128e-03) * (q[i9] * q[i12]) + (-6.984873e-04) * (q[i9] * q[i15]) + (5.208003e-04) * (q[i9] * q[i16]) + (-2.432863e-04) * (q[i9] * q[i19])
            + (5.502630e-05) * (q[i9] * q[i20]) + (1.687497e-04) * (q[i9] * q[i21]) + (-6.421777e-04) * (q[i9] * q[i22]) + (1.871301e-03) * (q[i10] * q[i11])
            + (3.728237e-04) * (q[i10] * q[i12]) + (-5.211538e-04) * (q[i10] * q[i15]) + (6.973763e-04) * (q[i10] * q[i16])
            + (-4.806814e-05) * (q[i10] * q[i19]) + (2.439533e-04) * (q[i10] * q[i20]) + (-6.443793e-04) * (q[i10] * q[i21])
            + (1.681949e-04) * (q[i10] * q[i22]) + (-1.148201e-05) * (q[i11] * q[i12]) + (3.140654e-03) * (q[i11] * q[i15]) + (9.446172e-04) * (q[i11] * q[i16])
            + (1.077931e-03) * (q[i11] * q[i19]) + (-1.831442e-04) * (q[i11] * q[i20]) + (1.827731e-03) * (q[i11] * q[i21])
            + (-3.900571e-05) * (q[i11] * q[i22]) + (9.501223e-04) * (q[i12] * q[i15]) + (3.159565e-03) * (q[i12] * q[i16])
            + (-1.796733e-04) * (q[i12] * q[i19]) + (1.059739e-03) * (q[i12] * q[i20]) + (4.025228e-05) * (q[i12] * q[i21])
            + (-1.843622e-03) * (q[i12] * q[i22]) + (5.080928e-06) * (q[i15] * q[i16]) + (4.603370e-04) * (q[i15] * q[i19]) + (2.050454e-04) * (q[i15] * q[i20])
            + (-1.066225e-04) * (q[i15] * q[i21]) + (-4.518506e-04) * (q[i15] * q[i22]) + (-2.102962e-04) * (q[i16] * q[i19])
            + (-4.268456e-04) * (q[i16] * q[i20]) + (-4.533561e-04) * (q[i16] * q[i21]) + (-8.338577e-05) * (q[i16] * q[i22])
            + (5.811766e-08) * (q[i19] * q[i20]) + (-1.353064e-03) * (q[i19] * q[i21]) + (-1.799147e-05) * (q[i19] * q[i22])
            + (-8.841363e-06) * (q[i20] * q[i21]) + (-1.338323e-03) * (q[i20] * q[i22]) + (-1.803648e-06) * (q[i21] * q[i22]);
   }

   public void getJQx9(double[] q, double[][] JQ)
   {
      JQ[1][i9] = (4.889873e-03) * (1) + (4.028609e-03) * ((2) * q[i9]) + (2.931941e-02) * (q[i0]) + (-1.585410e-03) * (q[i1]) + (3.152888e-03) * (q[i2])
            + (-7.460434e-03) * (q[i3]) + (4.908594e-03) * (q[i4]) + (2.276604e-03) * (q[i5]) + (8.509535e-03) * (q[i6]) + (3.818315e-04) * (q[i7])
            + (1.827642e-03) * (q[i8]) + (3.718957e-06) * (q[i10]) + (1.113186e-03) * (q[i11]) + (2.158386e-03) * (q[i12]) + (-1.509882e-03) * (q[i15])
            + (3.377185e-04) * (q[i16]) + (1.013386e-04) * (q[i19]) + (1.035450e-03) * (q[i20]) + (2.236456e-04) * (q[i21]) + (-5.577125e-05) * (q[i22])
            + (2.952615e-03) * (q[i0] * q[i0]) + (3.075456e-04) * (q[i1] * q[i1]) + (1.309976e-04) * (q[i2] * q[i2]) + (-1.857075e-04) * (q[i3] * q[i3])
            + (2.262692e-04) * (q[i4] * q[i4]) + (1.513727e-04) * (q[i5] * q[i5]) + (-2.486045e-03) * (q[i6] * q[i6]) + (-1.603254e-04) * (q[i7] * q[i7])
            + (-4.521382e-04) * (q[i8] * q[i8]) + (-6.786792e-03) * ((2) * q[i0] * q[i9]) + (1.277800e-03) * ((2) * q[i1] * q[i9])
            + (-2.022016e-03) * ((2) * q[i2] * q[i9]) + (-3.546787e-03) * ((2) * q[i3] * q[i9]) + (4.736603e-04) * ((2) * q[i4] * q[i9])
            + (1.952825e-03) * ((2) * q[i5] * q[i9]) + (-2.439674e-03) * ((2) * q[i6] * q[i9]) + (1.011449e-03) * ((2) * q[i7] * q[i9])
            + (3.629383e-04) * ((2) * q[i8] * q[i9]) + (-5.922928e-04) * ((3) * q[i9] * q[i9]) + (2.615357e-04) * ((2) * q[i9] * q[i10])
            + (-7.666270e-05) * ((2) * q[i9] * q[i11]) + (-6.388348e-04) * ((2) * q[i9] * q[i12]) + (2.176835e-04) * ((2) * q[i9] * q[i15])
            + (1.270710e-04) * ((2) * q[i9] * q[i16]) + (-1.505523e-04) * ((2) * q[i9] * q[i19]) + (-1.073877e-05) * ((2) * q[i9] * q[i20])
            + (-2.193992e-05) * ((2) * q[i9] * q[i21]) + (-1.976300e-05) * ((2) * q[i9] * q[i22]) + (-2.601219e-04) * (q[i10] * q[i10])
            + (-2.190168e-04) * (q[i11] * q[i11]) + (-5.664863e-04) * (q[i12] * q[i12]) + (-7.293982e-05) * (q[i15] * q[i15])
            + (2.559521e-05) * (q[i16] * q[i16]) + (-3.021958e-05) * (q[i19] * q[i19]) + (-4.193547e-05) * (q[i20] * q[i20])
            + (1.256556e-04) * (q[i21] * q[i21]) + (1.046439e-04) * (q[i22] * q[i22]) + (-1.418768e-03) * (q[i0] * q[i1]) + (1.292606e-03) * (q[i0] * q[i2])
            + (-1.020871e-03) * (q[i0] * q[i3]) + (-6.762930e-04) * (q[i0] * q[i4]) + (4.046716e-04) * (q[i0] * q[i5]) + (-1.360672e-02) * (q[i0] * q[i6])
            + (-1.327951e-04) * (q[i0] * q[i7]) + (7.407671e-04) * (q[i0] * q[i8]) + (7.379308e-04) * (q[i0] * q[i10]) + (1.292349e-05) * (q[i0] * q[i11])
            + (-5.578639e-04) * (q[i0] * q[i12]) + (7.242263e-04) * (q[i0] * q[i15]) + (2.030050e-04) * (q[i0] * q[i16]) + (8.202329e-04) * (q[i0] * q[i19])
            + (5.663061e-04) * (q[i0] * q[i20]) + (-4.827697e-04) * (q[i0] * q[i21]) + (-5.376229e-05) * (q[i0] * q[i22]) + (-5.259012e-04) * (q[i1] * q[i2])
            + (-7.905932e-04) * (q[i1] * q[i3]) + (-1.469909e-03) * (q[i1] * q[i4]) + (8.545363e-04) * (q[i1] * q[i5]) + (3.417191e-03) * (q[i1] * q[i6])
            + (1.747766e-03) * (q[i1] * q[i7]) + (-1.896148e-04) * (q[i1] * q[i8]) + (7.396080e-04) * (q[i1] * q[i10]) + (6.116578e-05) * (q[i1] * q[i11])
            + (9.128566e-04) * (q[i1] * q[i12]) + (1.494251e-04) * (q[i1] * q[i15]) + (-7.386152e-05) * (q[i1] * q[i16]) + (-6.724032e-04) * (q[i1] * q[i19])
            + (-3.448627e-04) * (q[i1] * q[i20]) + (4.375698e-04) * (q[i1] * q[i21]) + (6.739413e-04) * (q[i1] * q[i22]) + (1.821206e-04) * (q[i2] * q[i3])
            + (4.203955e-04) * (q[i2] * q[i4]) + (2.116555e-03) * (q[i2] * q[i5]) + (-2.505272e-03) * (q[i2] * q[i6]) + (7.057201e-04) * (q[i2] * q[i7])
            + (-2.458956e-03) * (q[i2] * q[i8]) + (-1.138271e-04) * (q[i2] * q[i10]) + (-4.428682e-04) * (q[i2] * q[i11]) + (1.445852e-04) * (q[i2] * q[i12])
            + (6.495961e-04) * (q[i2] * q[i15]) + (-4.232450e-04) * (q[i2] * q[i16]) + (1.164111e-04) * (q[i2] * q[i19]) + (3.097910e-04) * (q[i2] * q[i20])
            + (-3.177849e-04) * (q[i2] * q[i21]) + (2.041463e-04) * (q[i2] * q[i22]) + (-4.865914e-05) * (q[i3] * q[i4]) + (-1.903444e-04) * (q[i3] * q[i5])
            + (-1.030080e-02) * (q[i3] * q[i6]) + (1.223988e-04) * (q[i3] * q[i7]) + (-7.091580e-04) * (q[i3] * q[i8]) + (-1.125415e-03) * (q[i3] * q[i10])
            + (-5.818825e-04) * (q[i3] * q[i11]) + (2.452206e-04) * (q[i3] * q[i12]) + (-2.710467e-04) * (q[i3] * q[i15]) + (-4.947617e-04) * (q[i3] * q[i16])
            + (2.575008e-04) * (q[i3] * q[i19]) + (-1.099831e-03) * (q[i3] * q[i20]) + (2.429874e-04) * (q[i3] * q[i21]) + (-1.711662e-04) * (q[i3] * q[i22])
            + (1.461121e-03) * (q[i4] * q[i5]) + (1.505838e-03) * (q[i4] * q[i6]) + (5.630538e-04) * (q[i4] * q[i7]) + (9.636456e-04) * (q[i4] * q[i8])
            + (-1.116048e-03) * (q[i4] * q[i10]) + (-2.538559e-04) * (q[i4] * q[i11]) + (7.054069e-05) * (q[i4] * q[i12]) + (-6.173504e-05) * (q[i4] * q[i15])
            + (-6.497450e-04) * (q[i4] * q[i16]) + (-1.113940e-04) * (q[i4] * q[i19]) + (9.484636e-04) * (q[i4] * q[i20]) + (-3.795851e-04) * (q[i4] * q[i21])
            + (2.792880e-04) * (q[i4] * q[i22]) + (3.600806e-03) * (q[i5] * q[i6]) + (-1.811998e-03) * (q[i5] * q[i7]) + (-1.520993e-03) * (q[i5] * q[i8])
            + (3.736259e-04) * (q[i5] * q[i10]) + (-3.088273e-04) * (q[i5] * q[i11]) + (-1.763527e-04) * (q[i5] * q[i12]) + (-1.271194e-05) * (q[i5] * q[i15])
            + (-2.967064e-06) * (q[i5] * q[i16]) + (-1.319905e-04) * (q[i5] * q[i19]) + (6.840200e-05) * (q[i5] * q[i20]) + (5.699121e-05) * (q[i5] * q[i21])
            + (-1.983944e-04) * (q[i5] * q[i22]) + (1.094756e-03) * (q[i6] * q[i7]) + (2.919645e-04) * (q[i6] * q[i8]) + (1.057842e-03) * (q[i6] * q[i10])
            + (-8.023481e-04) * (q[i6] * q[i11]) + (-6.623801e-05) * (q[i6] * q[i12]) + (-5.787308e-05) * (q[i6] * q[i15]) + (5.530778e-04) * (q[i6] * q[i16])
            + (-5.147763e-04) * (q[i6] * q[i19]) + (-2.811633e-04) * (q[i6] * q[i20]) + (-5.704405e-06) * (q[i6] * q[i21]) + (6.697351e-06) * (q[i6] * q[i22])
            + (-5.487595e-05) * (q[i7] * q[i8]) + (-1.062174e-03) * (q[i7] * q[i10]) + (2.032087e-04) * (q[i7] * q[i11]) + (1.531247e-04) * (q[i7] * q[i12])
            + (3.024924e-05) * (q[i7] * q[i15]) + (-5.499585e-05) * (q[i7] * q[i16]) + (6.235673e-05) * (q[i7] * q[i19]) + (-3.490015e-07) * (q[i7] * q[i20])
            + (3.703065e-04) * (q[i7] * q[i21]) + (6.726312e-04) * (q[i7] * q[i22]) + (3.051791e-06) * (q[i8] * q[i10]) + (3.713144e-04) * (q[i8] * q[i11])
            + (1.890128e-03) * (q[i8] * q[i12]) + (-6.984873e-04) * (q[i8] * q[i15]) + (5.208003e-04) * (q[i8] * q[i16]) + (-2.432863e-04) * (q[i8] * q[i19])
            + (5.502630e-05) * (q[i8] * q[i20]) + (1.687497e-04) * (q[i8] * q[i21]) + (-6.421777e-04) * (q[i8] * q[i22]) + (-1.215391e-04) * (q[i10] * q[i11])
            + (-1.253624e-04) * (q[i10] * q[i12]) + (3.373777e-04) * (q[i10] * q[i15]) + (-3.369648e-04) * (q[i10] * q[i16])
            + (-4.452825e-05) * (q[i10] * q[i19]) + (4.880631e-05) * (q[i10] * q[i20]) + (1.934615e-04) * (q[i10] * q[i21]) + (1.904536e-04) * (q[i10] * q[i22])
            + (-2.414728e-04) * (q[i11] * q[i12]) + (1.564612e-04) * (q[i11] * q[i15]) + (1.517742e-04) * (q[i11] * q[i16]) + (4.364411e-05) * (q[i11] * q[i19])
            + (2.088469e-04) * (q[i11] * q[i20]) + (2.569153e-05) * (q[i11] * q[i21]) + (-1.682093e-04) * (q[i11] * q[i22])
            + (-1.483531e-04) * (q[i12] * q[i15]) + (6.375883e-05) * (q[i12] * q[i16]) + (-8.314378e-05) * (q[i12] * q[i19])
            + (-5.902861e-05) * (q[i12] * q[i20]) + (2.179966e-04) * (q[i12] * q[i21]) + (9.555884e-05) * (q[i12] * q[i22])
            + (-2.402656e-04) * (q[i15] * q[i16]) + (1.483290e-04) * (q[i15] * q[i19]) + (1.634168e-04) * (q[i15] * q[i20]) + (6.501105e-05) * (q[i15] * q[i21])
            + (-3.990033e-05) * (q[i15] * q[i22]) + (-2.577399e-04) * (q[i16] * q[i19]) + (1.908896e-04) * (q[i16] * q[i20])
            + (4.393048e-05) * (q[i16] * q[i21]) + (-1.016466e-04) * (q[i16] * q[i22]) + (1.544749e-04) * (q[i19] * q[i20])
            + (-2.061993e-05) * (q[i19] * q[i21]) + (1.277229e-04) * (q[i19] * q[i22]) + (1.741792e-04) * (q[i20] * q[i21])
            + (-3.629460e-04) * (q[i20] * q[i22]) + (-1.726096e-05) * (q[i21] * q[i22]);
   }

   public void getJQx10(double[] q, double[][] JQ)
   {
      JQ[1][i10] = (-4.803546e-03) * (1) + (-3.969694e-03) * ((2) * q[i10]) + (-1.566182e-03) * (q[i0]) + (2.896086e-02) * (q[i1]) + (3.116881e-03) * (q[i2])
            + (4.861489e-03) * (q[i3]) + (-7.410230e-03) * (q[i4]) + (2.289996e-03) * (q[i5]) + (-4.073500e-04) * (q[i6]) + (-8.383535e-03) * (q[i7])
            + (-1.850971e-03) * (q[i8]) + (3.718957e-06) * (q[i9]) + (2.136864e-03) * (q[i11]) + (1.103518e-03) * (q[i12]) + (-3.305814e-04) * (q[i15])
            + (1.497577e-03) * (q[i16]) + (-1.040316e-03) * (q[i19]) + (-8.931737e-05) * (q[i20]) + (-5.361554e-05) * (q[i21]) + (2.301890e-04) * (q[i22])
            + (-2.826019e-04) * (q[i0] * q[i0]) + (-2.936930e-03) * (q[i1] * q[i1]) + (-1.301474e-04) * (q[i2] * q[i2]) + (-2.367540e-04) * (q[i3] * q[i3])
            + (1.723745e-04) * (q[i4] * q[i4]) + (-1.422913e-04) * (q[i5] * q[i5]) + (1.582213e-04) * (q[i6] * q[i6]) + (2.457586e-03) * (q[i7] * q[i7])
            + (4.395179e-04) * (q[i8] * q[i8]) + (2.615357e-04) * (q[i9] * q[i9]) + (1.268991e-03) * ((2) * q[i0] * q[i10])
            + (-6.708023e-03) * ((2) * q[i1] * q[i10]) + (-1.993552e-03) * ((2) * q[i2] * q[i10]) + (4.730181e-04) * ((2) * q[i3] * q[i10])
            + (-3.499427e-03) * ((2) * q[i4] * q[i10]) + (1.924849e-03) * ((2) * q[i5] * q[i10]) + (-9.914628e-04) * ((2) * q[i6] * q[i10])
            + (2.404803e-03) * ((2) * q[i7] * q[i10]) + (-3.565921e-04) * ((2) * q[i8] * q[i10]) + (-2.601219e-04) * ((2) * q[i9] * q[i10])
            + (5.824171e-04) * ((3) * q[i10] * q[i10]) + (-6.364222e-04) * ((2) * q[i10] * q[i11]) + (-7.248281e-05) * ((2) * q[i10] * q[i12])
            + (-1.262826e-04) * ((2) * q[i10] * q[i15]) + (-2.161482e-04) * ((2) * q[i10] * q[i16]) + (1.223015e-05) * ((2) * q[i10] * q[i19])
            + (1.445431e-04) * ((2) * q[i10] * q[i20]) + (-2.028236e-05) * ((2) * q[i10] * q[i21]) + (-2.286918e-05) * ((2) * q[i10] * q[i22])
            + (5.507668e-04) * (q[i11] * q[i11]) + (2.103634e-04) * (q[i12] * q[i12]) + (-2.545667e-05) * (q[i15] * q[i15]) + (7.266301e-05) * (q[i16] * q[i16])
            + (4.162951e-05) * (q[i19] * q[i19]) + (2.972484e-05) * (q[i20] * q[i20]) + (-1.027291e-04) * (q[i21] * q[i21])
            + (-1.250614e-04) * (q[i22] * q[i22]) + (1.421309e-03) * (q[i0] * q[i1]) + (5.500446e-04) * (q[i0] * q[i2]) + (1.436438e-03) * (q[i0] * q[i3])
            + (7.799416e-04) * (q[i0] * q[i4]) + (-8.990503e-04) * (q[i0] * q[i5]) + (1.714814e-03) * (q[i0] * q[i6]) + (3.413640e-03) * (q[i0] * q[i7])
            + (-1.721679e-04) * (q[i0] * q[i8]) + (7.379308e-04) * (q[i0] * q[i9]) + (-8.957915e-04) * (q[i0] * q[i11]) + (-5.991964e-05) * (q[i0] * q[i12])
            + (-8.095689e-05) * (q[i0] * q[i15]) + (1.579682e-04) * (q[i0] * q[i16]) + (-3.351315e-04) * (q[i0] * q[i19]) + (-6.638931e-04) * (q[i0] * q[i20])
            + (-6.733610e-04) * (q[i0] * q[i21]) + (-4.308028e-04) * (q[i0] * q[i22]) + (-1.279786e-03) * (q[i1] * q[i2]) + (6.715230e-04) * (q[i1] * q[i3])
            + (1.007366e-03) * (q[i1] * q[i4]) + (-3.880431e-04) * (q[i1] * q[i5]) + (-1.111062e-04) * (q[i1] * q[i6]) + (-1.345148e-02) * (q[i1] * q[i7])
            + (7.417262e-04) * (q[i1] * q[i8]) + (7.396080e-04) * (q[i1] * q[i9]) + (5.542927e-04) * (q[i1] * q[i11]) + (-1.430565e-05) * (q[i1] * q[i12])
            + (1.871632e-04) * (q[i1] * q[i15]) + (7.150911e-04) * (q[i1] * q[i16]) + (5.671184e-04) * (q[i1] * q[i19]) + (8.148392e-04) * (q[i1] * q[i20])
            + (5.603105e-05) * (q[i1] * q[i21]) + (4.743153e-04) * (q[i1] * q[i22]) + (-4.290648e-04) * (q[i2] * q[i3]) + (-1.730749e-04) * (q[i2] * q[i4])
            + (-2.136809e-03) * (q[i2] * q[i5]) + (7.022326e-04) * (q[i2] * q[i6]) + (-2.443772e-03) * (q[i2] * q[i7]) + (-2.408560e-03) * (q[i2] * q[i8])
            + (-1.138271e-04) * (q[i2] * q[i9]) + (-1.591874e-04) * (q[i2] * q[i11]) + (4.314493e-04) * (q[i2] * q[i12]) + (-4.261787e-04) * (q[i2] * q[i15])
            + (6.458251e-04) * (q[i2] * q[i16]) + (3.252194e-04) * (q[i2] * q[i19]) + (1.234765e-04) * (q[i2] * q[i20]) + (-2.055229e-04) * (q[i2] * q[i21])
            + (3.180438e-04) * (q[i2] * q[i22]) + (5.663865e-05) * (q[i3] * q[i4]) + (-1.450961e-03) * (q[i3] * q[i5]) + (5.297648e-04) * (q[i3] * q[i6])
            + (1.490958e-03) * (q[i3] * q[i7]) + (9.674446e-04) * (q[i3] * q[i8]) + (-1.125415e-03) * (q[i3] * q[i9]) + (-7.761758e-05) * (q[i3] * q[i11])
            + (2.547123e-04) * (q[i3] * q[i12]) + (-6.593369e-04) * (q[i3] * q[i15]) + (-5.319852e-05) * (q[i3] * q[i16]) + (9.528878e-04) * (q[i3] * q[i19])
            + (-1.118422e-04) * (q[i3] * q[i20]) + (-2.816300e-04) * (q[i3] * q[i21]) + (3.857699e-04) * (q[i3] * q[i22]) + (1.876810e-04) * (q[i4] * q[i5])
            + (1.077158e-04) * (q[i4] * q[i6]) + (-1.020911e-02) * (q[i4] * q[i7]) + (-7.088078e-04) * (q[i4] * q[i8]) + (-1.116048e-03) * (q[i4] * q[i9])
            + (-2.539604e-04) * (q[i4] * q[i11]) + (5.829149e-04) * (q[i4] * q[i12]) + (-4.795333e-04) * (q[i4] * q[i15]) + (-2.686769e-04) * (q[i4] * q[i16])
            + (-1.089708e-03) * (q[i4] * q[i19]) + (2.507049e-04) * (q[i4] * q[i20]) + (1.679941e-04) * (q[i4] * q[i21]) + (-2.440550e-04) * (q[i4] * q[i22])
            + (-1.788734e-03) * (q[i5] * q[i6]) + (3.594889e-03) * (q[i5] * q[i7]) + (-1.520071e-03) * (q[i5] * q[i8]) + (3.736259e-04) * (q[i5] * q[i9])
            + (1.690130e-04) * (q[i5] * q[i11]) + (3.018424e-04) * (q[i5] * q[i12]) + (1.603742e-06) * (q[i5] * q[i15]) + (-2.413245e-05) * (q[i5] * q[i16])
            + (6.552438e-05) * (q[i5] * q[i19]) + (-1.415325e-04) * (q[i5] * q[i20]) + (1.988570e-04) * (q[i5] * q[i21]) + (-7.148407e-05) * (q[i5] * q[i22])
            + (-1.072949e-03) * (q[i6] * q[i7]) + (5.735390e-05) * (q[i6] * q[i8]) + (1.057842e-03) * (q[i6] * q[i9]) + (1.632620e-04) * (q[i6] * q[i11])
            + (1.932459e-04) * (q[i6] * q[i12]) + (5.898438e-05) * (q[i6] * q[i15]) + (-3.333471e-05) * (q[i6] * q[i16]) + (-3.367955e-06) * (q[i6] * q[i19])
            + (-6.403651e-05) * (q[i6] * q[i20]) + (6.740169e-04) * (q[i6] * q[i21]) + (3.724830e-04) * (q[i6] * q[i22]) + (-2.853329e-04) * (q[i7] * q[i8])
            + (-1.062174e-03) * (q[i7] * q[i9]) + (-6.176399e-05) * (q[i7] * q[i11]) + (-7.972379e-04) * (q[i7] * q[i12]) + (-5.512347e-04) * (q[i7] * q[i15])
            + (6.889978e-05) * (q[i7] * q[i16]) + (2.787552e-04) * (q[i7] * q[i19]) + (5.044036e-04) * (q[i7] * q[i20]) + (6.170180e-06) * (q[i7] * q[i21])
            + (-1.011493e-05) * (q[i7] * q[i22]) + (3.051791e-06) * (q[i8] * q[i9]) + (1.871301e-03) * (q[i8] * q[i11]) + (3.728237e-04) * (q[i8] * q[i12])
            + (-5.211538e-04) * (q[i8] * q[i15]) + (6.973763e-04) * (q[i8] * q[i16]) + (-4.806814e-05) * (q[i8] * q[i19]) + (2.439533e-04) * (q[i8] * q[i20])
            + (-6.443793e-04) * (q[i8] * q[i21]) + (1.681949e-04) * (q[i8] * q[i22]) + (-1.215391e-04) * (q[i9] * q[i11]) + (-1.253624e-04) * (q[i9] * q[i12])
            + (3.373777e-04) * (q[i9] * q[i15]) + (-3.369648e-04) * (q[i9] * q[i16]) + (-4.452825e-05) * (q[i9] * q[i19]) + (4.880631e-05) * (q[i9] * q[i20])
            + (1.934615e-04) * (q[i9] * q[i21]) + (1.904536e-04) * (q[i9] * q[i22]) + (2.459750e-04) * (q[i11] * q[i12]) + (6.393224e-05) * (q[i11] * q[i15])
            + (-1.507483e-04) * (q[i11] * q[i16]) + (-4.681708e-05) * (q[i11] * q[i19]) + (-8.202952e-05) * (q[i11] * q[i20])
            + (-9.864996e-05) * (q[i11] * q[i21]) + (-2.166452e-04) * (q[i11] * q[i22]) + (1.453329e-04) * (q[i12] * q[i15])
            + (1.589430e-04) * (q[i12] * q[i16]) + (2.078339e-04) * (q[i12] * q[i19]) + (4.660277e-05) * (q[i12] * q[i20]) + (1.714616e-04) * (q[i12] * q[i21])
            + (-2.717697e-05) * (q[i12] * q[i22]) + (2.381822e-04) * (q[i15] * q[i16]) + (-1.906830e-04) * (q[i15] * q[i19])
            + (2.572036e-04) * (q[i15] * q[i20]) + (-9.423758e-05) * (q[i15] * q[i21]) + (4.601349e-05) * (q[i15] * q[i22])
            + (-1.615727e-04) * (q[i16] * q[i19]) + (-1.470767e-04) * (q[i16] * q[i20]) + (-4.275113e-05) * (q[i16] * q[i21])
            + (6.190540e-05) * (q[i16] * q[i22]) + (-1.553178e-04) * (q[i19] * q[i20]) + (-3.572332e-04) * (q[i19] * q[i21])
            + (1.728515e-04) * (q[i19] * q[i22]) + (1.273184e-04) * (q[i20] * q[i21]) + (-2.134184e-05) * (q[i20] * q[i22])
            + (1.970438e-05) * (q[i21] * q[i22]);
   }

   public void getJQx11(double[] q, double[][] JQ)
   {
      JQ[1][i11] = (2.683704e-03) * (1) + (5.184729e-03) * ((2) * q[i11]) + (2.431821e-03) * (q[i0]) + (2.549366e-04) * (q[i1]) + (5.278294e-03) * (q[i2])
            + (-8.971683e-04) * (q[i3]) + (-2.383645e-03) * (q[i4]) + (5.436482e-03) * (q[i5]) + (3.803170e-03) * (q[i6]) + (4.708322e-03) * (q[i7])
            + (1.037983e-02) * (q[i8]) + (1.113186e-03) * (q[i9]) + (2.136864e-03) * (q[i10]) + (1.641208e-05) * (q[i12]) + (1.869733e-03) * (q[i15])
            + (-5.634301e-05) * (q[i16]) + (-1.497320e-03) * (q[i19]) + (1.693188e-03) * (q[i20]) + (4.262676e-04) * (q[i21]) + (-2.681229e-04) * (q[i22])
            + (7.179850e-04) * (q[i0] * q[i0]) + (2.325739e-05) * (q[i1] * q[i1]) + (6.982843e-04) * (q[i2] * q[i2]) + (-1.402094e-03) * (q[i3] * q[i3])
            + (-2.837358e-03) * (q[i4] * q[i4]) + (2.879716e-04) * (q[i5] * q[i5]) + (5.171001e-04) * (q[i6] * q[i6]) + (-6.901101e-04) * (q[i7] * q[i7])
            + (-1.781949e-03) * (q[i8] * q[i8]) + (-7.666270e-05) * (q[i9] * q[i9]) + (-6.364222e-04) * (q[i10] * q[i10])
            + (2.634201e-04) * ((2) * q[i0] * q[i11]) + (3.093027e-04) * ((2) * q[i1] * q[i11]) + (-4.927059e-05) * ((2) * q[i2] * q[i11])
            + (-3.952900e-04) * ((2) * q[i3] * q[i11]) + (-1.005719e-03) * ((2) * q[i4] * q[i11]) + (-1.257756e-03) * ((2) * q[i5] * q[i11])
            + (-5.734439e-05) * ((2) * q[i6] * q[i11]) + (-2.701751e-05) * ((2) * q[i7] * q[i11]) + (4.277118e-04) * ((2) * q[i8] * q[i11])
            + (-2.190168e-04) * ((2) * q[i9] * q[i11]) + (5.507668e-04) * ((2) * q[i10] * q[i11]) + (-7.581977e-05) * ((3) * q[i11] * q[i11])
            + (2.780140e-04) * ((2) * q[i11] * q[i12]) + (1.788992e-03) * ((2) * q[i11] * q[i15]) + (1.087262e-04) * ((2) * q[i11] * q[i16])
            + (4.134884e-04) * ((2) * q[i11] * q[i19]) + (-1.291444e-04) * ((2) * q[i11] * q[i20]) + (8.513405e-04) * ((2) * q[i11] * q[i21])
            + (6.335773e-05) * ((2) * q[i11] * q[i22]) + (2.928184e-04) * (q[i12] * q[i12]) + (2.021889e-03) * (q[i15] * q[i15])
            + (1.245777e-04) * (q[i16] * q[i16]) + (-1.188472e-04) * (q[i19] * q[i19]) + (-3.707643e-05) * (q[i20] * q[i20])
            + (4.836974e-04) * (q[i21] * q[i21]) + (1.762423e-04) * (q[i22] * q[i22]) + (-8.110965e-04) * (q[i0] * q[i1]) + (6.652454e-04) * (q[i0] * q[i2])
            + (4.059029e-05) * (q[i0] * q[i3]) + (1.098684e-03) * (q[i0] * q[i4]) + (-6.948002e-05) * (q[i0] * q[i5]) + (-1.493674e-03) * (q[i0] * q[i6])
            + (-9.047856e-04) * (q[i0] * q[i7]) + (-5.959634e-04) * (q[i0] * q[i8]) + (1.292349e-05) * (q[i0] * q[i9]) + (-8.957915e-04) * (q[i0] * q[i10])
            + (1.037433e-03) * (q[i0] * q[i12]) + (1.111723e-03) * (q[i0] * q[i15]) + (-6.870321e-05) * (q[i0] * q[i16]) + (6.155529e-04) * (q[i0] * q[i19])
            + (-2.593303e-04) * (q[i0] * q[i20]) + (7.605467e-04) * (q[i0] * q[i21]) + (3.498287e-04) * (q[i0] * q[i22]) + (1.404683e-03) * (q[i1] * q[i2])
            + (-1.809140e-03) * (q[i1] * q[i3]) + (-1.716275e-03) * (q[i1] * q[i4]) + (3.387693e-04) * (q[i1] * q[i5]) + (1.669093e-03) * (q[i1] * q[i6])
            + (-2.929429e-04) * (q[i1] * q[i7]) + (-1.253281e-04) * (q[i1] * q[i8]) + (6.116578e-05) * (q[i1] * q[i9]) + (5.542927e-04) * (q[i1] * q[i10])
            + (1.050391e-03) * (q[i1] * q[i12]) + (2.987142e-04) * (q[i1] * q[i15]) + (3.494695e-04) * (q[i1] * q[i16]) + (5.263009e-04) * (q[i1] * q[i19])
            + (-1.317697e-04) * (q[i1] * q[i20]) + (-7.379106e-04) * (q[i1] * q[i21]) + (4.227531e-04) * (q[i1] * q[i22]) + (5.062225e-04) * (q[i2] * q[i3])
            + (-4.936573e-04) * (q[i2] * q[i4]) + (1.737642e-03) * (q[i2] * q[i5]) + (-7.407081e-04) * (q[i2] * q[i6]) + (3.318719e-04) * (q[i2] * q[i7])
            + (-1.244990e-03) * (q[i2] * q[i8]) + (-4.428682e-04) * (q[i2] * q[i9]) + (-1.591874e-04) * (q[i2] * q[i10]) + (1.565082e-03) * (q[i2] * q[i12])
            + (3.600388e-03) * (q[i2] * q[i15]) + (-6.828608e-04) * (q[i2] * q[i16]) + (1.696311e-03) * (q[i2] * q[i19]) + (5.165242e-05) * (q[i2] * q[i20])
            + (8.125435e-05) * (q[i2] * q[i21]) + (-1.375290e-04) * (q[i2] * q[i22]) + (1.984605e-03) * (q[i3] * q[i4]) + (-8.940210e-04) * (q[i3] * q[i5])
            + (1.117085e-04) * (q[i3] * q[i6]) + (-9.071750e-04) * (q[i3] * q[i7]) + (-8.620245e-04) * (q[i3] * q[i8]) + (-5.818825e-04) * (q[i3] * q[i9])
            + (-7.761758e-05) * (q[i3] * q[i10]) + (-9.431192e-04) * (q[i3] * q[i12]) + (-2.399187e-03) * (q[i3] * q[i15]) + (9.598698e-04) * (q[i3] * q[i16])
            + (3.549022e-04) * (q[i3] * q[i19]) + (-6.758842e-05) * (q[i3] * q[i20]) + (-8.200449e-04) * (q[i3] * q[i21]) + (-1.065079e-03) * (q[i3] * q[i22])
            + (-3.199444e-04) * (q[i4] * q[i5]) + (-7.801497e-04) * (q[i4] * q[i6]) + (-1.659809e-04) * (q[i4] * q[i7]) + (-1.784835e-03) * (q[i4] * q[i8])
            + (-2.538559e-04) * (q[i4] * q[i9]) + (-2.539604e-04) * (q[i4] * q[i10]) + (-9.341547e-04) * (q[i4] * q[i12]) + (4.198854e-04) * (q[i4] * q[i15])
            + (-3.551721e-04) * (q[i4] * q[i16]) + (-8.331422e-04) * (q[i4] * q[i19]) + (5.438664e-04) * (q[i4] * q[i20]) + (-2.493820e-04) * (q[i4] * q[i21])
            + (8.925204e-04) * (q[i4] * q[i22]) + (1.185197e-04) * (q[i5] * q[i6]) + (1.385137e-03) * (q[i5] * q[i7]) + (1.744742e-03) * (q[i5] * q[i8])
            + (-3.088273e-04) * (q[i5] * q[i9]) + (1.690130e-04) * (q[i5] * q[i10]) + (1.094104e-03) * (q[i5] * q[i12]) + (2.062101e-03) * (q[i5] * q[i15])
            + (-6.159860e-04) * (q[i5] * q[i16]) + (6.998644e-04) * (q[i5] * q[i19]) + (-4.212765e-04) * (q[i5] * q[i20]) + (-2.158314e-04) * (q[i5] * q[i21])
            + (-3.102334e-04) * (q[i5] * q[i22]) + (9.749898e-04) * (q[i6] * q[i7]) + (3.118483e-04) * (q[i6] * q[i8]) + (-8.023481e-04) * (q[i6] * q[i9])
            + (1.632620e-04) * (q[i6] * q[i10]) + (-9.789896e-05) * (q[i6] * q[i12]) + (1.189101e-03) * (q[i6] * q[i15]) + (1.504720e-04) * (q[i6] * q[i16])
            + (2.453448e-05) * (q[i6] * q[i19]) + (1.247018e-03) * (q[i6] * q[i20]) + (-3.542402e-04) * (q[i6] * q[i21]) + (-3.928276e-04) * (q[i6] * q[i22])
            + (3.848970e-04) * (q[i7] * q[i8]) + (2.032087e-04) * (q[i7] * q[i9]) + (-6.176399e-05) * (q[i7] * q[i10]) + (1.092357e-04) * (q[i7] * q[i12])
            + (3.001006e-04) * (q[i7] * q[i15]) + (1.036485e-03) * (q[i7] * q[i16]) + (-1.439265e-04) * (q[i7] * q[i19]) + (1.173055e-04) * (q[i7] * q[i20])
            + (5.647104e-04) * (q[i7] * q[i21]) + (-8.827581e-05) * (q[i7] * q[i22]) + (3.713144e-04) * (q[i8] * q[i9]) + (1.871301e-03) * (q[i8] * q[i10])
            + (-1.148201e-05) * (q[i8] * q[i12]) + (3.140654e-03) * (q[i8] * q[i15]) + (9.446172e-04) * (q[i8] * q[i16]) + (1.077931e-03) * (q[i8] * q[i19])
            + (-1.831442e-04) * (q[i8] * q[i20]) + (1.827731e-03) * (q[i8] * q[i21]) + (-3.900571e-05) * (q[i8] * q[i22]) + (-1.215391e-04) * (q[i9] * q[i10])
            + (-2.414728e-04) * (q[i9] * q[i12]) + (1.564612e-04) * (q[i9] * q[i15]) + (1.517742e-04) * (q[i9] * q[i16]) + (4.364411e-05) * (q[i9] * q[i19])
            + (2.088469e-04) * (q[i9] * q[i20]) + (2.569153e-05) * (q[i9] * q[i21]) + (-1.682093e-04) * (q[i9] * q[i22]) + (2.459750e-04) * (q[i10] * q[i12])
            + (6.393224e-05) * (q[i10] * q[i15]) + (-1.507483e-04) * (q[i10] * q[i16]) + (-4.681708e-05) * (q[i10] * q[i19])
            + (-8.202952e-05) * (q[i10] * q[i20]) + (-9.864996e-05) * (q[i10] * q[i21]) + (-2.166452e-04) * (q[i10] * q[i22])
            + (3.146763e-04) * (q[i12] * q[i15]) + (-3.128072e-04) * (q[i12] * q[i16]) + (3.545202e-04) * (q[i12] * q[i19])
            + (-3.485009e-04) * (q[i12] * q[i20]) + (-4.512187e-04) * (q[i12] * q[i21]) + (-4.540651e-04) * (q[i12] * q[i22])
            + (-1.783143e-04) * (q[i15] * q[i16]) + (-4.017164e-04) * (q[i15] * q[i19]) + (-2.733872e-05) * (q[i15] * q[i20])
            + (-4.815746e-04) * (q[i15] * q[i21]) + (-1.275106e-04) * (q[i15] * q[i22]) + (5.782590e-05) * (q[i16] * q[i19])
            + (1.390104e-04) * (q[i16] * q[i20]) + (-6.533038e-04) * (q[i16] * q[i21]) + (-3.496984e-04) * (q[i16] * q[i22])
            + (-2.405631e-04) * (q[i19] * q[i20]) + (-2.057011e-03) * (q[i19] * q[i21]) + (6.084827e-05) * (q[i19] * q[i22])
            + (1.702902e-04) * (q[i20] * q[i21]) + (-2.498516e-04) * (q[i20] * q[i22]) + (8.413113e-05) * (q[i21] * q[i22]);
   }

   public void getJQx12(double[] q, double[][] JQ)
   {
      JQ[1][i12] = (3.009467e-03) * (1) + (-5.326149e-03) * ((2) * q[i12]) + (-2.645071e-04) * (q[i0]) + (-2.437312e-03) * (q[i1]) + (-5.454492e-03) * (q[i2])
            + (2.443215e-03) * (q[i3]) + (9.079738e-04) * (q[i4]) + (-5.465597e-03) * (q[i5]) + (4.725349e-03) * (q[i6]) + (3.765473e-03) * (q[i7])
            + (1.046921e-02) * (q[i8]) + (2.158386e-03) * (q[i9]) + (1.103518e-03) * (q[i10]) + (1.641208e-05) * (q[i11]) + (-3.818853e-05) * (q[i15])
            + (1.944719e-03) * (q[i16]) + (1.699568e-03) * (q[i19]) + (-1.523914e-03) * (q[i20]) + (2.662284e-04) * (q[i21]) + (-4.255399e-04) * (q[i22])
            + (4.998808e-06) * (q[i0] * q[i0]) + (7.082192e-04) * (q[i1] * q[i1]) + (6.892704e-04) * (q[i2] * q[i2]) + (-2.855950e-03) * (q[i3] * q[i3])
            + (-1.394569e-03) * (q[i4] * q[i4]) + (2.853626e-04) * (q[i5] * q[i5]) + (-6.927001e-04) * (q[i6] * q[i6]) + (5.123069e-04) * (q[i7] * q[i7])
            + (-1.816467e-03) * (q[i8] * q[i8]) + (-6.388348e-04) * (q[i9] * q[i9]) + (-7.248281e-05) * (q[i10] * q[i10]) + (2.780140e-04) * (q[i11] * q[i11])
            + (2.812292e-04) * ((2) * q[i0] * q[i12]) + (2.557378e-04) * ((2) * q[i1] * q[i12]) + (-1.504999e-04) * ((2) * q[i2] * q[i12])
            + (-9.821072e-04) * ((2) * q[i3] * q[i12]) + (-3.937433e-04) * ((2) * q[i4] * q[i12]) + (-1.331503e-03) * ((2) * q[i5] * q[i12])
            + (4.222588e-05) * ((2) * q[i6] * q[i12]) + (6.581599e-05) * ((2) * q[i7] * q[i12]) + (-3.407250e-04) * ((2) * q[i8] * q[i12])
            + (-5.664863e-04) * ((2) * q[i9] * q[i12]) + (2.103634e-04) * ((2) * q[i10] * q[i12]) + (2.928184e-04) * ((2) * q[i11] * q[i12])
            + (-1.318355e-04) * ((3) * q[i12] * q[i12]) + (-1.176538e-04) * ((2) * q[i12] * q[i15]) + (-1.806532e-03) * ((2) * q[i12] * q[i16])
            + (1.260357e-04) * ((2) * q[i12] * q[i19]) + (-4.017068e-04) * ((2) * q[i12] * q[i20]) + (5.719601e-05) * ((2) * q[i12] * q[i21])
            + (8.551246e-04) * ((2) * q[i12] * q[i22]) + (1.280547e-04) * (q[i15] * q[i15]) + (2.050267e-03) * (q[i16] * q[i16])
            + (-3.760187e-05) * (q[i19] * q[i19]) + (-1.051264e-04) * (q[i20] * q[i20]) + (1.767875e-04) * (q[i21] * q[i21])
            + (4.884731e-04) * (q[i22] * q[i22]) + (-8.088057e-04) * (q[i0] * q[i1]) + (1.396673e-03) * (q[i0] * q[i2]) + (-1.685471e-03) * (q[i0] * q[i3])
            + (-1.804400e-03) * (q[i0] * q[i4]) + (2.869565e-04) * (q[i0] * q[i5]) + (3.205563e-04) * (q[i0] * q[i6]) + (-1.669194e-03) * (q[i0] * q[i7])
            + (1.649874e-04) * (q[i0] * q[i8]) + (-5.578639e-04) * (q[i0] * q[i9]) + (-5.991964e-05) * (q[i0] * q[i10]) + (1.037433e-03) * (q[i0] * q[i11])
            + (-3.508971e-04) * (q[i0] * q[i15]) + (-3.126500e-04) * (q[i0] * q[i16]) + (1.308050e-04) * (q[i0] * q[i19]) + (-5.335765e-04) * (q[i0] * q[i20])
            + (4.240182e-04) * (q[i0] * q[i21]) + (-7.301080e-04) * (q[i0] * q[i22]) + (6.646431e-04) * (q[i1] * q[i2]) + (1.062638e-03) * (q[i1] * q[i3])
            + (6.743792e-05) * (q[i1] * q[i4]) + (-6.398775e-05) * (q[i1] * q[i5]) + (9.281863e-04) * (q[i1] * q[i6]) + (1.519737e-03) * (q[i1] * q[i7])
            + (6.184059e-04) * (q[i1] * q[i8]) + (9.128566e-04) * (q[i1] * q[i9]) + (-1.430565e-05) * (q[i1] * q[i10]) + (1.050391e-03) * (q[i1] * q[i11])
            + (5.891167e-05) * (q[i1] * q[i15]) + (-1.117033e-03) * (q[i1] * q[i16]) + (2.595780e-04) * (q[i1] * q[i19]) + (-6.142505e-04) * (q[i1] * q[i20])
            + (3.496778e-04) * (q[i1] * q[i21]) + (7.630297e-04) * (q[i1] * q[i22]) + (-4.700165e-04) * (q[i2] * q[i3]) + (5.365713e-04) * (q[i2] * q[i4])
            + (1.689365e-03) * (q[i2] * q[i5]) + (-3.079661e-04) * (q[i2] * q[i6]) + (7.383186e-04) * (q[i2] * q[i7]) + (1.411772e-03) * (q[i2] * q[i8])
            + (1.445852e-04) * (q[i2] * q[i9]) + (4.314493e-04) * (q[i2] * q[i10]) + (1.565082e-03) * (q[i2] * q[i11]) + (6.583024e-04) * (q[i2] * q[i15])
            + (-3.614837e-03) * (q[i2] * q[i16]) + (-6.191380e-05) * (q[i2] * q[i19]) + (-1.678354e-03) * (q[i2] * q[i20]) + (-1.404150e-04) * (q[i2] * q[i21])
            + (8.041627e-05) * (q[i2] * q[i22]) + (1.988030e-03) * (q[i3] * q[i4]) + (-3.053467e-04) * (q[i3] * q[i5]) + (1.625795e-04) * (q[i3] * q[i6])
            + (7.903289e-04) * (q[i3] * q[i7]) + (1.817324e-03) * (q[i3] * q[i8]) + (2.452206e-04) * (q[i3] * q[i9]) + (2.547123e-04) * (q[i3] * q[i10])
            + (-9.431192e-04) * (q[i3] * q[i11]) + (3.537319e-04) * (q[i3] * q[i15]) + (-4.212583e-04) * (q[i3] * q[i16]) + (-5.466593e-04) * (q[i3] * q[i19])
            + (8.499335e-04) * (q[i3] * q[i20]) + (8.870416e-04) * (q[i3] * q[i21]) + (-2.562754e-04) * (q[i3] * q[i22]) + (-8.964191e-04) * (q[i4] * q[i5])
            + (8.819036e-04) * (q[i4] * q[i6]) + (-1.034888e-04) * (q[i4] * q[i7]) + (8.717079e-04) * (q[i4] * q[i8]) + (7.054069e-05) * (q[i4] * q[i9])
            + (5.829149e-04) * (q[i4] * q[i10]) + (-9.341547e-04) * (q[i4] * q[i11]) + (-9.618192e-04) * (q[i4] * q[i15]) + (2.398704e-03) * (q[i4] * q[i16])
            + (5.877753e-05) * (q[i4] * q[i19]) + (-3.604102e-04) * (q[i4] * q[i20]) + (-1.056132e-03) * (q[i4] * q[i21]) + (-8.202490e-04) * (q[i4] * q[i22])
            + (-1.385588e-03) * (q[i5] * q[i6]) + (-1.108449e-04) * (q[i5] * q[i7]) + (-1.778833e-03) * (q[i5] * q[i8]) + (-1.763527e-04) * (q[i5] * q[i9])
            + (3.018424e-04) * (q[i5] * q[i10]) + (1.094104e-03) * (q[i5] * q[i11]) + (6.111405e-04) * (q[i5] * q[i15]) + (-2.064967e-03) * (q[i5] * q[i16])
            + (4.324835e-04) * (q[i5] * q[i19]) + (-7.160581e-04) * (q[i5] * q[i20]) + (-3.110414e-04) * (q[i5] * q[i21]) + (-2.057889e-04) * (q[i5] * q[i22])
            + (9.681896e-04) * (q[i6] * q[i7]) + (3.574590e-04) * (q[i6] * q[i8]) + (-6.623801e-05) * (q[i6] * q[i9]) + (1.932459e-04) * (q[i6] * q[i10])
            + (-9.789896e-05) * (q[i6] * q[i11]) + (1.037932e-03) * (q[i6] * q[i15]) + (3.039606e-04) * (q[i6] * q[i16]) + (1.155972e-04) * (q[i6] * q[i19])
            + (-1.390614e-04) * (q[i6] * q[i20]) + (9.065239e-05) * (q[i6] * q[i21]) + (-5.720250e-04) * (q[i6] * q[i22]) + (3.219046e-04) * (q[i7] * q[i8])
            + (1.531247e-04) * (q[i7] * q[i9]) + (-7.972379e-04) * (q[i7] * q[i10]) + (1.092357e-04) * (q[i7] * q[i11]) + (1.521089e-04) * (q[i7] * q[i15])
            + (1.174695e-03) * (q[i7] * q[i16]) + (1.244186e-03) * (q[i7] * q[i19]) + (2.493281e-05) * (q[i7] * q[i20]) + (3.914591e-04) * (q[i7] * q[i21])
            + (3.562947e-04) * (q[i7] * q[i22]) + (1.890128e-03) * (q[i8] * q[i9]) + (3.728237e-04) * (q[i8] * q[i10]) + (-1.148201e-05) * (q[i8] * q[i11])
            + (9.501223e-04) * (q[i8] * q[i15]) + (3.159565e-03) * (q[i8] * q[i16]) + (-1.796733e-04) * (q[i8] * q[i19]) + (1.059739e-03) * (q[i8] * q[i20])
            + (4.025228e-05) * (q[i8] * q[i21]) + (-1.843622e-03) * (q[i8] * q[i22]) + (-1.253624e-04) * (q[i9] * q[i10]) + (-2.414728e-04) * (q[i9] * q[i11])
            + (-1.483531e-04) * (q[i9] * q[i15]) + (6.375883e-05) * (q[i9] * q[i16]) + (-8.314378e-05) * (q[i9] * q[i19]) + (-5.902861e-05) * (q[i9] * q[i20])
            + (2.179966e-04) * (q[i9] * q[i21]) + (9.555884e-05) * (q[i9] * q[i22]) + (2.459750e-04) * (q[i10] * q[i11]) + (1.453329e-04) * (q[i10] * q[i15])
            + (1.589430e-04) * (q[i10] * q[i16]) + (2.078339e-04) * (q[i10] * q[i19]) + (4.660277e-05) * (q[i10] * q[i20]) + (1.714616e-04) * (q[i10] * q[i21])
            + (-2.717697e-05) * (q[i10] * q[i22]) + (3.146763e-04) * (q[i11] * q[i15]) + (-3.128072e-04) * (q[i11] * q[i16])
            + (3.545202e-04) * (q[i11] * q[i19]) + (-3.485009e-04) * (q[i11] * q[i20]) + (-4.512187e-04) * (q[i11] * q[i21])
            + (-4.540651e-04) * (q[i11] * q[i22]) + (-1.691708e-04) * (q[i15] * q[i16]) + (1.354376e-04) * (q[i15] * q[i19])
            + (6.039360e-05) * (q[i15] * q[i20]) + (3.469436e-04) * (q[i15] * q[i21]) + (6.541224e-04) * (q[i15] * q[i22]) + (-2.004387e-05) * (q[i16] * q[i19])
            + (-4.167561e-04) * (q[i16] * q[i20]) + (1.295322e-04) * (q[i16] * q[i21]) + (4.850914e-04) * (q[i16] * q[i22])
            + (-2.413345e-04) * (q[i19] * q[i20]) + (2.520685e-04) * (q[i19] * q[i21]) + (-1.723724e-04) * (q[i19] * q[i22])
            + (-6.266256e-05) * (q[i20] * q[i21]) + (2.050566e-03) * (q[i20] * q[i22]) + (8.656308e-05) * (q[i21] * q[i22]);
   }

   public void getJQx15(double[] q, double[][] JQ)
   {
      JQ[1][i15] = (-1.968976e-03) * (1) + (6.490308e-03) * ((2) * q[i15]) + (-3.495047e-04) * (q[i0]) + (2.488392e-03) * (q[i1]) + (2.469027e-03) * (q[i2])
            + (5.579206e-03) * (q[i3]) + (5.425509e-03) * (q[i4]) + (-1.788624e-02) * (q[i5]) + (-9.283013e-04) * (q[i6]) + (7.197612e-04) * (q[i7])
            + (9.172387e-03) * (q[i8]) + (-1.509882e-03) * (q[i9]) + (-3.305814e-04) * (q[i10]) + (1.869733e-03) * (q[i11]) + (-3.818853e-05) * (q[i12])
            + (1.381008e-06) * (q[i16]) + (1.443820e-03) * (q[i19]) + (-1.050440e-04) * (q[i20]) + (2.450143e-03) * (q[i21]) + (2.765729e-04) * (q[i22])
            + (-1.196572e-03) * (q[i0] * q[i0]) + (2.994997e-04) * (q[i1] * q[i1]) + (-2.546112e-03) * (q[i2] * q[i2]) + (7.552389e-04) * (q[i3] * q[i3])
            + (-3.144852e-04) * (q[i4] * q[i4]) + (1.071036e-04) * (q[i5] * q[i5]) + (2.228535e-04) * (q[i6] * q[i6]) + (-1.107298e-03) * (q[i7] * q[i7])
            + (4.086490e-03) * (q[i8] * q[i8]) + (2.176835e-04) * (q[i9] * q[i9]) + (-1.262826e-04) * (q[i10] * q[i10]) + (1.788992e-03) * (q[i11] * q[i11])
            + (-1.176538e-04) * (q[i12] * q[i12]) + (-9.181986e-05) * ((2) * q[i0] * q[i15]) + (7.592240e-05) * ((2) * q[i1] * q[i15])
            + (1.748959e-03) * ((2) * q[i2] * q[i15]) + (6.068730e-04) * ((2) * q[i3] * q[i15]) + (7.297165e-04) * ((2) * q[i4] * q[i15])
            + (-3.172480e-03) * ((2) * q[i5] * q[i15]) + (8.926814e-05) * ((2) * q[i6] * q[i15]) + (-5.139503e-05) * ((2) * q[i7] * q[i15])
            + (2.670214e-03) * ((2) * q[i8] * q[i15]) + (-7.293982e-05) * ((2) * q[i9] * q[i15]) + (-2.545667e-05) * ((2) * q[i10] * q[i15])
            + (2.021889e-03) * ((2) * q[i11] * q[i15]) + (1.280547e-04) * ((2) * q[i12] * q[i15]) + (7.975157e-04) * ((3) * q[i15] * q[i15])
            + (1.442044e-04) * ((2) * q[i15] * q[i16]) + (2.168470e-04) * ((2) * q[i15] * q[i19]) + (1.325622e-04) * ((2) * q[i15] * q[i20])
            + (3.968051e-04) * ((2) * q[i15] * q[i21]) + (-1.268897e-05) * ((2) * q[i15] * q[i22]) + (-1.452361e-04) * (q[i16] * q[i16])
            + (3.503035e-04) * (q[i19] * q[i19]) + (-1.661363e-04) * (q[i20] * q[i20]) + (7.926800e-04) * (q[i21] * q[i21]) + (1.745091e-05) * (q[i22] * q[i22])
            + (1.151046e-03) * (q[i0] * q[i1]) + (-6.568085e-05) * (q[i0] * q[i2]) + (2.231693e-03) * (q[i0] * q[i3]) + (-5.569488e-04) * (q[i0] * q[i4])
            + (7.623461e-04) * (q[i0] * q[i5]) + (3.511050e-04) * (q[i0] * q[i6]) + (-6.133359e-04) * (q[i0] * q[i7]) + (9.418310e-04) * (q[i0] * q[i8])
            + (7.242263e-04) * (q[i0] * q[i9]) + (-8.095689e-05) * (q[i0] * q[i10]) + (1.111723e-03) * (q[i0] * q[i11]) + (-3.508971e-04) * (q[i0] * q[i12])
            + (5.148387e-04) * (q[i0] * q[i16]) + (-1.717268e-04) * (q[i0] * q[i19]) + (4.410306e-04) * (q[i0] * q[i20]) + (2.398073e-04) * (q[i0] * q[i21])
            + (5.474682e-04) * (q[i0] * q[i22]) + (-1.531927e-03) * (q[i1] * q[i2]) + (3.459426e-04) * (q[i1] * q[i3]) + (-8.506545e-04) * (q[i1] * q[i4])
            + (1.632663e-03) * (q[i1] * q[i5]) + (7.933511e-04) * (q[i1] * q[i6]) + (2.038911e-03) * (q[i1] * q[i7]) + (2.099563e-04) * (q[i1] * q[i8])
            + (1.494251e-04) * (q[i1] * q[i9]) + (1.871632e-04) * (q[i1] * q[i10]) + (2.987142e-04) * (q[i1] * q[i11]) + (5.891167e-05) * (q[i1] * q[i12])
            + (5.088894e-04) * (q[i1] * q[i16]) + (8.008115e-04) * (q[i1] * q[i19]) + (-1.042180e-03) * (q[i1] * q[i20]) + (2.924900e-05) * (q[i1] * q[i21])
            + (2.277305e-04) * (q[i1] * q[i22]) + (1.418867e-03) * (q[i2] * q[i3]) + (-9.815592e-05) * (q[i2] * q[i4]) + (-1.444219e-03) * (q[i2] * q[i5])
            + (5.709624e-04) * (q[i2] * q[i6]) + (2.397672e-04) * (q[i2] * q[i7]) + (4.348435e-03) * (q[i2] * q[i8]) + (6.495961e-04) * (q[i2] * q[i9])
            + (-4.261787e-04) * (q[i2] * q[i10]) + (3.600388e-03) * (q[i2] * q[i11]) + (6.583024e-04) * (q[i2] * q[i12]) + (4.283540e-05) * (q[i2] * q[i16])
            + (-9.625853e-04) * (q[i2] * q[i19]) + (-8.625585e-04) * (q[i2] * q[i20]) + (3.715050e-04) * (q[i2] * q[i21]) + (3.675351e-04) * (q[i2] * q[i22])
            + (-6.454528e-05) * (q[i3] * q[i4]) + (3.455274e-04) * (q[i3] * q[i5]) + (1.331850e-03) * (q[i3] * q[i6]) + (-7.167150e-04) * (q[i3] * q[i7])
            + (4.639511e-04) * (q[i3] * q[i8]) + (-2.710467e-04) * (q[i3] * q[i9]) + (-6.593369e-04) * (q[i3] * q[i10]) + (-2.399187e-03) * (q[i3] * q[i11])
            + (3.537319e-04) * (q[i3] * q[i12]) + (6.317741e-04) * (q[i3] * q[i16]) + (-4.963041e-04) * (q[i3] * q[i19]) + (-5.463849e-04) * (q[i3] * q[i20])
            + (-1.765358e-04) * (q[i3] * q[i21]) + (2.002703e-04) * (q[i3] * q[i22]) + (-9.330154e-04) * (q[i4] * q[i5]) + (-1.836521e-03) * (q[i4] * q[i6])
            + (-1.031517e-03) * (q[i4] * q[i7]) + (-2.745696e-04) * (q[i4] * q[i8]) + (-6.173504e-05) * (q[i4] * q[i9]) + (-4.795333e-04) * (q[i4] * q[i10])
            + (4.198854e-04) * (q[i4] * q[i11]) + (-9.618192e-04) * (q[i4] * q[i12]) + (6.267090e-04) * (q[i4] * q[i16]) + (4.461442e-04) * (q[i4] * q[i19])
            + (7.146477e-04) * (q[i4] * q[i20]) + (5.753389e-04) * (q[i4] * q[i21]) + (1.739075e-05) * (q[i4] * q[i22]) + (-2.008343e-04) * (q[i5] * q[i6])
            + (3.823836e-04) * (q[i5] * q[i7]) + (1.608008e-03) * (q[i5] * q[i8]) + (-1.271194e-05) * (q[i5] * q[i9]) + (1.603742e-06) * (q[i5] * q[i10])
            + (2.062101e-03) * (q[i5] * q[i11]) + (6.111405e-04) * (q[i5] * q[i12]) + (-1.431438e-04) * (q[i5] * q[i16]) + (-7.355210e-04) * (q[i5] * q[i19])
            + (-3.359705e-04) * (q[i5] * q[i20]) + (-1.399082e-03) * (q[i5] * q[i21]) + (1.192572e-04) * (q[i5] * q[i22]) + (-1.479991e-04) * (q[i6] * q[i7])
            + (1.204213e-05) * (q[i6] * q[i8]) + (-5.787308e-05) * (q[i6] * q[i9]) + (5.898438e-05) * (q[i6] * q[i10]) + (1.189101e-03) * (q[i6] * q[i11])
            + (1.037932e-03) * (q[i6] * q[i12]) + (9.620974e-05) * (q[i6] * q[i16]) + (2.382661e-04) * (q[i6] * q[i19]) + (-2.391586e-04) * (q[i6] * q[i20])
            + (-5.152226e-05) * (q[i6] * q[i21]) + (2.113318e-04) * (q[i6] * q[i22]) + (-8.874344e-04) * (q[i7] * q[i8]) + (3.024924e-05) * (q[i7] * q[i9])
            + (-5.512347e-04) * (q[i7] * q[i10]) + (3.001006e-04) * (q[i7] * q[i11]) + (1.521089e-04) * (q[i7] * q[i12]) + (-9.605176e-05) * (q[i7] * q[i16])
            + (-1.144394e-05) * (q[i7] * q[i19]) + (4.214893e-04) * (q[i7] * q[i20]) + (1.458426e-04) * (q[i7] * q[i21]) + (1.105229e-04) * (q[i7] * q[i22])
            + (-6.984873e-04) * (q[i8] * q[i9]) + (-5.211538e-04) * (q[i8] * q[i10]) + (3.140654e-03) * (q[i8] * q[i11]) + (9.501223e-04) * (q[i8] * q[i12])
            + (5.080928e-06) * (q[i8] * q[i16]) + (4.603370e-04) * (q[i8] * q[i19]) + (2.050454e-04) * (q[i8] * q[i20]) + (-1.066225e-04) * (q[i8] * q[i21])
            + (-4.518506e-04) * (q[i8] * q[i22]) + (3.373777e-04) * (q[i9] * q[i10]) + (1.564612e-04) * (q[i9] * q[i11]) + (-1.483531e-04) * (q[i9] * q[i12])
            + (-2.402656e-04) * (q[i9] * q[i16]) + (1.483290e-04) * (q[i9] * q[i19]) + (1.634168e-04) * (q[i9] * q[i20]) + (6.501105e-05) * (q[i9] * q[i21])
            + (-3.990033e-05) * (q[i9] * q[i22]) + (6.393224e-05) * (q[i10] * q[i11]) + (1.453329e-04) * (q[i10] * q[i12]) + (2.381822e-04) * (q[i10] * q[i16])
            + (-1.906830e-04) * (q[i10] * q[i19]) + (2.572036e-04) * (q[i10] * q[i20]) + (-9.423758e-05) * (q[i10] * q[i21])
            + (4.601349e-05) * (q[i10] * q[i22]) + (3.146763e-04) * (q[i11] * q[i12]) + (-1.783143e-04) * (q[i11] * q[i16])
            + (-4.017164e-04) * (q[i11] * q[i19]) + (-2.733872e-05) * (q[i11] * q[i20]) + (-4.815746e-04) * (q[i11] * q[i21])
            + (-1.275106e-04) * (q[i11] * q[i22]) + (-1.691708e-04) * (q[i12] * q[i16]) + (1.354376e-04) * (q[i12] * q[i19])
            + (6.039360e-05) * (q[i12] * q[i20]) + (3.469436e-04) * (q[i12] * q[i21]) + (6.541224e-04) * (q[i12] * q[i22]) + (2.134674e-04) * (q[i16] * q[i19])
            + (-2.091147e-04) * (q[i16] * q[i20]) + (1.579448e-04) * (q[i16] * q[i21]) + (1.535920e-04) * (q[i16] * q[i22]) + (1.092114e-05) * (q[i19] * q[i20])
            + (5.399830e-05) * (q[i19] * q[i21]) + (-8.669724e-05) * (q[i19] * q[i22]) + (1.848389e-06) * (q[i20] * q[i21])
            + (-2.399921e-05) * (q[i20] * q[i22]) + (-6.705985e-05) * (q[i21] * q[i22]);
   }

   public void getJQx16(double[] q, double[][] JQ)
   {
      JQ[1][i16] = (1.890989e-03) * (1) + (-6.544746e-03) * ((2) * q[i16]) + (2.440634e-03) * (q[i0]) + (-3.292644e-04) * (q[i1]) + (2.522821e-03) * (q[i2])
            + (5.510126e-03) * (q[i3]) + (5.592667e-03) * (q[i4]) + (-1.806450e-02) * (q[i5]) + (-7.215826e-04) * (q[i6]) + (9.171318e-04) * (q[i7])
            + (-9.271249e-03) * (q[i8]) + (3.377185e-04) * (q[i9]) + (1.497577e-03) * (q[i10]) + (-5.634301e-05) * (q[i11]) + (1.944719e-03) * (q[i12])
            + (1.381008e-06) * (q[i15]) + (9.614661e-05) * (q[i19]) + (-1.418129e-03) * (q[i20]) + (2.959218e-04) * (q[i21]) + (2.443170e-03) * (q[i22])
            + (-2.972802e-04) * (q[i0] * q[i0]) + (1.192890e-03) * (q[i1] * q[i1]) + (2.587146e-03) * (q[i2] * q[i2]) + (3.199199e-04) * (q[i3] * q[i3])
            + (-7.444036e-04) * (q[i4] * q[i4]) + (-9.213145e-05) * (q[i5] * q[i5]) + (1.106866e-03) * (q[i6] * q[i6]) + (-2.299380e-04) * (q[i7] * q[i7])
            + (-4.139522e-03) * (q[i8] * q[i8]) + (1.270710e-04) * (q[i9] * q[i9]) + (-2.161482e-04) * (q[i10] * q[i10]) + (1.087262e-04) * (q[i11] * q[i11])
            + (-1.806532e-03) * (q[i12] * q[i12]) + (1.442044e-04) * (q[i15] * q[i15]) + (6.728841e-05) * ((2) * q[i0] * q[i16])
            + (-9.215216e-05) * ((2) * q[i1] * q[i16]) + (1.766318e-03) * ((2) * q[i2] * q[i16]) + (7.318281e-04) * ((2) * q[i3] * q[i16])
            + (6.037906e-04) * ((2) * q[i4] * q[i16]) + (-3.193741e-03) * ((2) * q[i5] * q[i16]) + (5.766638e-05) * ((2) * q[i6] * q[i16])
            + (-8.659144e-05) * ((2) * q[i7] * q[i16]) + (-2.682642e-03) * ((2) * q[i8] * q[i16]) + (2.559521e-05) * ((2) * q[i9] * q[i16])
            + (7.266301e-05) * ((2) * q[i10] * q[i16]) + (1.245777e-04) * ((2) * q[i11] * q[i16]) + (2.050267e-03) * ((2) * q[i12] * q[i16])
            + (-1.452361e-04) * ((2) * q[i15] * q[i16]) + (-7.998110e-04) * ((3) * q[i16] * q[i16]) + (-1.343704e-04) * ((2) * q[i16] * q[i19])
            + (-2.153955e-04) * ((2) * q[i16] * q[i20]) + (-1.105465e-05) * ((2) * q[i16] * q[i21]) + (4.009891e-04) * ((2) * q[i16] * q[i22])
            + (1.705391e-04) * (q[i19] * q[i19]) + (-3.447732e-04) * (q[i20] * q[i20]) + (-1.365182e-05) * (q[i21] * q[i21])
            + (-7.943027e-04) * (q[i22] * q[i22]) + (-1.169626e-03) * (q[i0] * q[i1]) + (1.546440e-03) * (q[i0] * q[i2]) + (8.670147e-04) * (q[i0] * q[i3])
            + (-3.701069e-04) * (q[i0] * q[i4]) + (-1.635196e-03) * (q[i0] * q[i5]) + (2.069774e-03) * (q[i0] * q[i6]) + (7.881685e-04) * (q[i0] * q[i7])
            + (2.296953e-04) * (q[i0] * q[i8]) + (2.030050e-04) * (q[i0] * q[i9]) + (1.579682e-04) * (q[i0] * q[i10]) + (-6.870321e-05) * (q[i0] * q[i11])
            + (-3.126500e-04) * (q[i0] * q[i12]) + (5.148387e-04) * (q[i0] * q[i15]) + (-1.048530e-03) * (q[i0] * q[i19]) + (8.003445e-04) * (q[i0] * q[i20])
            + (-2.354564e-04) * (q[i0] * q[i21]) + (-2.929430e-05) * (q[i0] * q[i22]) + (6.903362e-05) * (q[i1] * q[i2]) + (5.483259e-04) * (q[i1] * q[i3])
            + (-2.229472e-03) * (q[i1] * q[i4]) + (-7.627753e-04) * (q[i1] * q[i5]) + (-6.163040e-04) * (q[i1] * q[i6]) + (3.758379e-04) * (q[i1] * q[i7])
            + (9.333350e-04) * (q[i1] * q[i8]) + (-7.386152e-05) * (q[i1] * q[i9]) + (7.150911e-04) * (q[i1] * q[i10]) + (3.494695e-04) * (q[i1] * q[i11])
            + (-1.117033e-03) * (q[i1] * q[i12]) + (5.088894e-04) * (q[i1] * q[i15]) + (4.527292e-04) * (q[i1] * q[i19]) + (-1.634519e-04) * (q[i1] * q[i20])
            + (-5.340071e-04) * (q[i1] * q[i21]) + (-2.454685e-04) * (q[i1] * q[i22]) + (1.015846e-04) * (q[i2] * q[i3]) + (-1.455408e-03) * (q[i2] * q[i4])
            + (1.441259e-03) * (q[i2] * q[i5]) + (2.251452e-04) * (q[i2] * q[i6]) + (5.739001e-04) * (q[i2] * q[i7]) + (4.428257e-03) * (q[i2] * q[i8])
            + (-4.232450e-04) * (q[i2] * q[i9]) + (6.458251e-04) * (q[i2] * q[i10]) + (-6.828608e-04) * (q[i2] * q[i11]) + (-3.614837e-03) * (q[i2] * q[i12])
            + (4.283540e-05) * (q[i2] * q[i15]) + (-8.608036e-04) * (q[i2] * q[i19]) + (-9.489442e-04) * (q[i2] * q[i20]) + (-3.632431e-04) * (q[i2] * q[i21])
            + (-3.792889e-04) * (q[i2] * q[i22]) + (7.420731e-05) * (q[i3] * q[i4]) + (9.330827e-04) * (q[i3] * q[i5]) + (-1.027732e-03) * (q[i3] * q[i6])
            + (-1.838161e-03) * (q[i3] * q[i7]) + (-2.946867e-04) * (q[i3] * q[i8]) + (-4.947617e-04) * (q[i3] * q[i9]) + (-5.319852e-05) * (q[i3] * q[i10])
            + (9.598698e-04) * (q[i3] * q[i11]) + (-4.212583e-04) * (q[i3] * q[i12]) + (6.317741e-04) * (q[i3] * q[i15]) + (7.135509e-04) * (q[i3] * q[i19])
            + (4.429594e-04) * (q[i3] * q[i20]) + (-2.275339e-05) * (q[i3] * q[i21]) + (-5.864281e-04) * (q[i3] * q[i22]) + (-3.794327e-04) * (q[i4] * q[i5])
            + (-7.226764e-04) * (q[i4] * q[i6]) + (1.346195e-03) * (q[i4] * q[i7]) + (4.798022e-04) * (q[i4] * q[i8]) + (-6.497450e-04) * (q[i4] * q[i9])
            + (-2.686769e-04) * (q[i4] * q[i10]) + (-3.551721e-04) * (q[i4] * q[i11]) + (2.398704e-03) * (q[i4] * q[i12]) + (6.267090e-04) * (q[i4] * q[i15])
            + (-5.510075e-04) * (q[i4] * q[i19]) + (-4.883223e-04) * (q[i4] * q[i20]) + (-1.986130e-04) * (q[i4] * q[i21]) + (1.665167e-04) * (q[i4] * q[i22])
            + (3.957857e-04) * (q[i5] * q[i6]) + (-2.080655e-04) * (q[i5] * q[i7]) + (1.625924e-03) * (q[i5] * q[i8]) + (-2.967064e-06) * (q[i5] * q[i9])
            + (-2.413245e-05) * (q[i5] * q[i10]) + (-6.159860e-04) * (q[i5] * q[i11]) + (-2.064967e-03) * (q[i5] * q[i12]) + (-1.431438e-04) * (q[i5] * q[i15])
            + (-3.321778e-04) * (q[i5] * q[i19]) + (-7.404865e-04) * (q[i5] * q[i20]) + (-1.216028e-04) * (q[i5] * q[i21]) + (1.415512e-03) * (q[i5] * q[i22])
            + (1.486869e-04) * (q[i6] * q[i7]) + (8.807517e-04) * (q[i6] * q[i8]) + (5.530778e-04) * (q[i6] * q[i9]) + (-3.333471e-05) * (q[i6] * q[i10])
            + (1.504720e-04) * (q[i6] * q[i11]) + (3.039606e-04) * (q[i6] * q[i12]) + (9.620974e-05) * (q[i6] * q[i15]) + (-4.223975e-04) * (q[i6] * q[i19])
            + (1.109683e-05) * (q[i6] * q[i20]) + (1.089463e-04) * (q[i6] * q[i21]) + (1.466666e-04) * (q[i6] * q[i22]) + (-6.459206e-07) * (q[i7] * q[i8])
            + (-5.499585e-05) * (q[i7] * q[i9]) + (6.889978e-05) * (q[i7] * q[i10]) + (1.036485e-03) * (q[i7] * q[i11]) + (1.174695e-03) * (q[i7] * q[i12])
            + (-9.605176e-05) * (q[i7] * q[i15]) + (2.379539e-04) * (q[i7] * q[i19]) + (-2.380913e-04) * (q[i7] * q[i20]) + (2.063762e-04) * (q[i7] * q[i21])
            + (-5.774850e-05) * (q[i7] * q[i22]) + (5.208003e-04) * (q[i8] * q[i9]) + (6.973763e-04) * (q[i8] * q[i10]) + (9.446172e-04) * (q[i8] * q[i11])
            + (3.159565e-03) * (q[i8] * q[i12]) + (5.080928e-06) * (q[i8] * q[i15]) + (-2.102962e-04) * (q[i8] * q[i19]) + (-4.268456e-04) * (q[i8] * q[i20])
            + (-4.533561e-04) * (q[i8] * q[i21]) + (-8.338577e-05) * (q[i8] * q[i22]) + (-3.369648e-04) * (q[i9] * q[i10]) + (1.517742e-04) * (q[i9] * q[i11])
            + (6.375883e-05) * (q[i9] * q[i12]) + (-2.402656e-04) * (q[i9] * q[i15]) + (-2.577399e-04) * (q[i9] * q[i19]) + (1.908896e-04) * (q[i9] * q[i20])
            + (4.393048e-05) * (q[i9] * q[i21]) + (-1.016466e-04) * (q[i9] * q[i22]) + (-1.507483e-04) * (q[i10] * q[i11]) + (1.589430e-04) * (q[i10] * q[i12])
            + (2.381822e-04) * (q[i10] * q[i15]) + (-1.615727e-04) * (q[i10] * q[i19]) + (-1.470767e-04) * (q[i10] * q[i20])
            + (-4.275113e-05) * (q[i10] * q[i21]) + (6.190540e-05) * (q[i10] * q[i22]) + (-3.128072e-04) * (q[i11] * q[i12])
            + (-1.783143e-04) * (q[i11] * q[i15]) + (5.782590e-05) * (q[i11] * q[i19]) + (1.390104e-04) * (q[i11] * q[i20])
            + (-6.533038e-04) * (q[i11] * q[i21]) + (-3.496984e-04) * (q[i11] * q[i22]) + (-1.691708e-04) * (q[i12] * q[i15])
            + (-2.004387e-05) * (q[i12] * q[i19]) + (-4.167561e-04) * (q[i12] * q[i20]) + (1.295322e-04) * (q[i12] * q[i21])
            + (4.850914e-04) * (q[i12] * q[i22]) + (2.134674e-04) * (q[i15] * q[i19]) + (-2.091147e-04) * (q[i15] * q[i20]) + (1.579448e-04) * (q[i15] * q[i21])
            + (1.535920e-04) * (q[i15] * q[i22]) + (-7.888197e-06) * (q[i19] * q[i20]) + (-2.416558e-05) * (q[i19] * q[i21])
            + (5.949074e-06) * (q[i19] * q[i22]) + (-8.441539e-05) * (q[i20] * q[i21]) + (6.254680e-05) * (q[i20] * q[i22])
            + (6.619962e-05) * (q[i21] * q[i22]);
   }

   public void getJQx19(double[] q, double[][] JQ)
   {
      JQ[1][i19] = (9.177392e-04) * (1) + (7.924415e-04) * ((2) * q[i19]) + (-2.003494e-03) * (q[i0]) + (7.010237e-04) * (q[i1]) + (-2.095669e-03) * (q[i2])
            + (-2.364092e-03) * (q[i3]) + (3.453573e-03) * (q[i4]) + (-5.391021e-04) * (q[i5]) + (2.066508e-03) * (q[i6]) + (-1.185100e-03) * (q[i7])
            + (-4.194115e-04) * (q[i8]) + (1.013386e-04) * (q[i9]) + (-1.040316e-03) * (q[i10]) + (-1.497320e-03) * (q[i11]) + (1.699568e-03) * (q[i12])
            + (1.443820e-03) * (q[i15]) + (9.614661e-05) * (q[i16]) + (6.413053e-06) * (q[i20]) + (-9.128984e-04) * (q[i21]) + (-9.662913e-05) * (q[i22])
            + (-4.202603e-05) * (q[i0] * q[i0]) + (3.554884e-04) * (q[i1] * q[i1]) + (1.521570e-04) * (q[i2] * q[i2]) + (-1.116370e-03) * (q[i3] * q[i3])
            + (2.920938e-04) * (q[i4] * q[i4]) + (4.846585e-04) * (q[i5] * q[i5]) + (-1.366856e-04) * (q[i6] * q[i6]) + (-4.174993e-04) * (q[i7] * q[i7])
            + (-2.545432e-05) * (q[i8] * q[i8]) + (-1.505523e-04) * (q[i9] * q[i9]) + (1.223015e-05) * (q[i10] * q[i10]) + (4.134884e-04) * (q[i11] * q[i11])
            + (1.260357e-04) * (q[i12] * q[i12]) + (2.168470e-04) * (q[i15] * q[i15]) + (-1.343704e-04) * (q[i16] * q[i16])
            + (-1.738111e-05) * ((2) * q[i0] * q[i19]) + (-1.020428e-04) * ((2) * q[i1] * q[i19]) + (-2.030081e-04) * ((2) * q[i2] * q[i19])
            + (1.567991e-04) * ((2) * q[i3] * q[i19]) + (1.412430e-04) * ((2) * q[i4] * q[i19]) + (3.073737e-04) * ((2) * q[i5] * q[i19])
            + (-1.450210e-04) * ((2) * q[i6] * q[i19]) + (3.239674e-04) * ((2) * q[i7] * q[i19]) + (8.407745e-04) * ((2) * q[i8] * q[i19])
            + (-3.021958e-05) * ((2) * q[i9] * q[i19]) + (4.162951e-05) * ((2) * q[i10] * q[i19]) + (-1.188472e-04) * ((2) * q[i11] * q[i19])
            + (-3.760187e-05) * ((2) * q[i12] * q[i19]) + (3.503035e-04) * ((2) * q[i15] * q[i19]) + (1.705391e-04) * ((2) * q[i16] * q[i19])
            + (1.431031e-04) * ((3) * q[i19] * q[i19]) + (-5.105214e-06) * ((2) * q[i19] * q[i20]) + (6.685921e-04) * ((2) * q[i19] * q[i21])
            + (-7.300996e-05) * ((2) * q[i19] * q[i22]) + (4.827284e-06) * (q[i20] * q[i20]) + (-9.116763e-05) * (q[i21] * q[i21])
            + (1.245651e-04) * (q[i22] * q[i22]) + (-5.858271e-04) * (q[i0] * q[i1]) + (4.345169e-04) * (q[i0] * q[i2]) + (1.522527e-03) * (q[i0] * q[i3])
            + (-1.016211e-03) * (q[i0] * q[i4]) + (-8.694627e-04) * (q[i0] * q[i5]) + (1.222475e-03) * (q[i0] * q[i6]) + (-6.159552e-04) * (q[i0] * q[i7])
            + (1.349733e-03) * (q[i0] * q[i8]) + (8.202329e-04) * (q[i0] * q[i9]) + (-3.351315e-04) * (q[i0] * q[i10]) + (6.155529e-04) * (q[i0] * q[i11])
            + (1.308050e-04) * (q[i0] * q[i12]) + (-1.717268e-04) * (q[i0] * q[i15]) + (-1.048530e-03) * (q[i0] * q[i16]) + (4.840219e-04) * (q[i0] * q[i20])
            + (-2.430971e-04) * (q[i0] * q[i21]) + (-5.005325e-04) * (q[i0] * q[i22]) + (-1.356130e-03) * (q[i1] * q[i2]) + (9.862217e-04) * (q[i1] * q[i3])
            + (-1.241991e-03) * (q[i1] * q[i4]) + (-1.608649e-04) * (q[i1] * q[i5]) + (-9.142841e-04) * (q[i1] * q[i6]) + (1.349245e-03) * (q[i1] * q[i7])
            + (1.060160e-03) * (q[i1] * q[i8]) + (-6.724032e-04) * (q[i1] * q[i9]) + (5.671184e-04) * (q[i1] * q[i10]) + (5.263009e-04) * (q[i1] * q[i11])
            + (2.595780e-04) * (q[i1] * q[i12]) + (8.008115e-04) * (q[i1] * q[i15]) + (4.527292e-04) * (q[i1] * q[i16]) + (4.608439e-04) * (q[i1] * q[i20])
            + (-8.342090e-04) * (q[i1] * q[i21]) + (-7.396063e-04) * (q[i1] * q[i22]) + (-9.577654e-04) * (q[i2] * q[i3]) + (-5.021187e-04) * (q[i2] * q[i4])
            + (1.186527e-03) * (q[i2] * q[i5]) + (1.268388e-03) * (q[i2] * q[i6]) + (2.983729e-04) * (q[i2] * q[i7]) + (1.071222e-03) * (q[i2] * q[i8])
            + (1.164111e-04) * (q[i2] * q[i9]) + (3.252194e-04) * (q[i2] * q[i10]) + (1.696311e-03) * (q[i2] * q[i11]) + (-6.191380e-05) * (q[i2] * q[i12])
            + (-9.625853e-04) * (q[i2] * q[i15]) + (-8.608036e-04) * (q[i2] * q[i16]) + (1.081195e-03) * (q[i2] * q[i20]) + (-1.188958e-03) * (q[i2] * q[i21])
            + (-5.315273e-04) * (q[i2] * q[i22]) + (-7.927073e-04) * (q[i3] * q[i4]) + (-8.253939e-04) * (q[i3] * q[i5]) + (-2.949664e-03) * (q[i3] * q[i6])
            + (-1.571947e-04) * (q[i3] * q[i7]) + (-1.082728e-03) * (q[i3] * q[i8]) + (2.575008e-04) * (q[i3] * q[i9]) + (9.528878e-04) * (q[i3] * q[i10])
            + (3.549022e-04) * (q[i3] * q[i11]) + (-5.466593e-04) * (q[i3] * q[i12]) + (-4.963041e-04) * (q[i3] * q[i15]) + (7.135509e-04) * (q[i3] * q[i16])
            + (6.337695e-04) * (q[i3] * q[i20]) + (1.284728e-03) * (q[i3] * q[i21]) + (-3.871210e-04) * (q[i3] * q[i22]) + (1.241729e-03) * (q[i4] * q[i5])
            + (1.863697e-03) * (q[i4] * q[i6]) + (-1.888206e-04) * (q[i4] * q[i7]) + (5.460890e-05) * (q[i4] * q[i8]) + (-1.113940e-04) * (q[i4] * q[i9])
            + (-1.089708e-03) * (q[i4] * q[i10]) + (-8.331422e-04) * (q[i4] * q[i11]) + (5.877753e-05) * (q[i4] * q[i12]) + (4.461442e-04) * (q[i4] * q[i15])
            + (-5.510075e-04) * (q[i4] * q[i16]) + (6.201209e-04) * (q[i4] * q[i20]) + (1.730511e-04) * (q[i4] * q[i21]) + (3.040057e-04) * (q[i4] * q[i22])
            + (2.459765e-04) * (q[i5] * q[i6]) + (6.288068e-04) * (q[i5] * q[i7]) + (-1.509317e-03) * (q[i5] * q[i8]) + (-1.319905e-04) * (q[i5] * q[i9])
            + (6.552438e-05) * (q[i5] * q[i10]) + (6.998644e-04) * (q[i5] * q[i11]) + (4.324835e-04) * (q[i5] * q[i12]) + (-7.355210e-04) * (q[i5] * q[i15])
            + (-3.321778e-04) * (q[i5] * q[i16]) + (-5.472268e-04) * (q[i5] * q[i20]) + (-1.777132e-03) * (q[i5] * q[i21]) + (5.059876e-04) * (q[i5] * q[i22])
            + (3.730544e-04) * (q[i6] * q[i7]) + (-3.218953e-04) * (q[i6] * q[i8]) + (-5.147763e-04) * (q[i6] * q[i9]) + (-3.367955e-06) * (q[i6] * q[i10])
            + (2.453448e-05) * (q[i6] * q[i11]) + (1.155972e-04) * (q[i6] * q[i12]) + (2.382661e-04) * (q[i6] * q[i15]) + (-4.223975e-04) * (q[i6] * q[i16])
            + (1.838184e-04) * (q[i6] * q[i20]) + (-1.257420e-04) * (q[i6] * q[i21]) + (-7.666956e-05) * (q[i6] * q[i22]) + (2.425172e-06) * (q[i7] * q[i8])
            + (6.235673e-05) * (q[i7] * q[i9]) + (2.787552e-04) * (q[i7] * q[i10]) + (-1.439265e-04) * (q[i7] * q[i11]) + (1.244186e-03) * (q[i7] * q[i12])
            + (-1.144394e-05) * (q[i7] * q[i15]) + (2.379539e-04) * (q[i7] * q[i16]) + (-1.807043e-04) * (q[i7] * q[i20]) + (-6.118894e-04) * (q[i7] * q[i21])
            + (8.430038e-04) * (q[i7] * q[i22]) + (-2.432863e-04) * (q[i8] * q[i9]) + (-4.806814e-05) * (q[i8] * q[i10]) + (1.077931e-03) * (q[i8] * q[i11])
            + (-1.796733e-04) * (q[i8] * q[i12]) + (4.603370e-04) * (q[i8] * q[i15]) + (-2.102962e-04) * (q[i8] * q[i16]) + (5.811766e-08) * (q[i8] * q[i20])
            + (-1.353064e-03) * (q[i8] * q[i21]) + (-1.799147e-05) * (q[i8] * q[i22]) + (-4.452825e-05) * (q[i9] * q[i10]) + (4.364411e-05) * (q[i9] * q[i11])
            + (-8.314378e-05) * (q[i9] * q[i12]) + (1.483290e-04) * (q[i9] * q[i15]) + (-2.577399e-04) * (q[i9] * q[i16]) + (1.544749e-04) * (q[i9] * q[i20])
            + (-2.061993e-05) * (q[i9] * q[i21]) + (1.277229e-04) * (q[i9] * q[i22]) + (-4.681708e-05) * (q[i10] * q[i11]) + (2.078339e-04) * (q[i10] * q[i12])
            + (-1.906830e-04) * (q[i10] * q[i15]) + (-1.615727e-04) * (q[i10] * q[i16]) + (-1.553178e-04) * (q[i10] * q[i20])
            + (-3.572332e-04) * (q[i10] * q[i21]) + (1.728515e-04) * (q[i10] * q[i22]) + (3.545202e-04) * (q[i11] * q[i12])
            + (-4.017164e-04) * (q[i11] * q[i15]) + (5.782590e-05) * (q[i11] * q[i16]) + (-2.405631e-04) * (q[i11] * q[i20])
            + (-2.057011e-03) * (q[i11] * q[i21]) + (6.084827e-05) * (q[i11] * q[i22]) + (1.354376e-04) * (q[i12] * q[i15])
            + (-2.004387e-05) * (q[i12] * q[i16]) + (-2.413345e-04) * (q[i12] * q[i20]) + (2.520685e-04) * (q[i12] * q[i21])
            + (-1.723724e-04) * (q[i12] * q[i22]) + (2.134674e-04) * (q[i15] * q[i16]) + (1.092114e-05) * (q[i15] * q[i20]) + (5.399830e-05) * (q[i15] * q[i21])
            + (-8.669724e-05) * (q[i15] * q[i22]) + (-7.888197e-06) * (q[i16] * q[i20]) + (-2.416558e-05) * (q[i16] * q[i21])
            + (5.949074e-06) * (q[i16] * q[i22]) + (1.641668e-04) * (q[i20] * q[i21]) + (1.603013e-04) * (q[i20] * q[i22]) + (6.385282e-05) * (q[i21] * q[i22]);
   }

   public void getJQx20(double[] q, double[][] JQ)
   {
      JQ[1][i20] = (-9.533365e-04) * (1) + (-7.808437e-04) * ((2) * q[i20]) + (6.647019e-04) * (q[i0]) + (-1.956615e-03) * (q[i1]) + (-2.100913e-03) * (q[i2])
            + (3.470420e-03) * (q[i3]) + (-2.334264e-03) * (q[i4]) + (-5.439486e-04) * (q[i5]) + (1.200329e-03) * (q[i6]) + (-2.023647e-03) * (q[i7])
            + (4.509890e-04) * (q[i8]) + (1.035450e-03) * (q[i9]) + (-8.931737e-05) * (q[i10]) + (1.693188e-03) * (q[i11]) + (-1.523914e-03) * (q[i12])
            + (-1.050440e-04) * (q[i15]) + (-1.418129e-03) * (q[i16]) + (6.413053e-06) * (q[i19]) + (-1.065760e-04) * (q[i21]) + (-8.800162e-04) * (q[i22])
            + (-3.548803e-04) * (q[i0] * q[i0]) + (5.501578e-05) * (q[i1] * q[i1]) + (-1.591219e-04) * (q[i2] * q[i2]) + (-2.797243e-04) * (q[i3] * q[i3])
            + (1.091014e-03) * (q[i4] * q[i4]) + (-4.678912e-04) * (q[i5] * q[i5]) + (4.183499e-04) * (q[i6] * q[i6]) + (1.400944e-04) * (q[i7] * q[i7])
            + (2.835757e-05) * (q[i8] * q[i8]) + (-1.073877e-05) * (q[i9] * q[i9]) + (1.445431e-04) * (q[i10] * q[i10]) + (-1.291444e-04) * (q[i11] * q[i11])
            + (-4.017068e-04) * (q[i12] * q[i12]) + (1.325622e-04) * (q[i15] * q[i15]) + (-2.153955e-04) * (q[i16] * q[i16])
            + (-5.105214e-06) * (q[i19] * q[i19]) + (-1.050280e-04) * ((2) * q[i0] * q[i20]) + (-2.351599e-05) * ((2) * q[i1] * q[i20])
            + (-2.080122e-04) * ((2) * q[i2] * q[i20]) + (1.332810e-04) * ((2) * q[i3] * q[i20]) + (1.504221e-04) * ((2) * q[i4] * q[i20])
            + (3.262443e-04) * ((2) * q[i5] * q[i20]) + (-3.249408e-04) * ((2) * q[i6] * q[i20]) + (1.465960e-04) * ((2) * q[i7] * q[i20])
            + (-8.367389e-04) * ((2) * q[i8] * q[i20]) + (-4.193547e-05) * ((2) * q[i9] * q[i20]) + (2.972484e-05) * ((2) * q[i10] * q[i20])
            + (-3.707643e-05) * ((2) * q[i11] * q[i20]) + (-1.051264e-04) * ((2) * q[i12] * q[i20]) + (-1.661363e-04) * ((2) * q[i15] * q[i20])
            + (-3.447732e-04) * ((2) * q[i16] * q[i20]) + (4.827284e-06) * ((2) * q[i19] * q[i20]) + (-1.374386e-04) * ((3) * q[i20] * q[i20])
            + (-7.287472e-05) * ((2) * q[i20] * q[i21]) + (6.647343e-04) * ((2) * q[i20] * q[i22]) + (-1.229703e-04) * (q[i21] * q[i21])
            + (8.810742e-05) * (q[i22] * q[i22]) + (5.868450e-04) * (q[i0] * q[i1]) + (1.349169e-03) * (q[i0] * q[i2]) + (1.245394e-03) * (q[i0] * q[i3])
            + (-9.938613e-04) * (q[i0] * q[i4]) + (1.593632e-04) * (q[i0] * q[i5]) + (1.359705e-03) * (q[i0] * q[i6]) + (-9.066067e-04) * (q[i0] * q[i7])
            + (1.054646e-03) * (q[i0] * q[i8]) + (5.663061e-04) * (q[i0] * q[i9]) + (-6.638931e-04) * (q[i0] * q[i10]) + (-2.593303e-04) * (q[i0] * q[i11])
            + (-5.335765e-04) * (q[i0] * q[i12]) + (4.410306e-04) * (q[i0] * q[i15]) + (8.003445e-04) * (q[i0] * q[i16]) + (4.840219e-04) * (q[i0] * q[i19])
            + (7.464442e-04) * (q[i0] * q[i21]) + (8.370573e-04) * (q[i0] * q[i22]) + (-4.233245e-04) * (q[i1] * q[i2]) + (1.000061e-03) * (q[i1] * q[i3])
            + (-1.501474e-03) * (q[i1] * q[i4]) + (8.323787e-04) * (q[i1] * q[i5]) + (-6.114105e-04) * (q[i1] * q[i6]) + (1.199835e-03) * (q[i1] * q[i7])
            + (1.348211e-03) * (q[i1] * q[i8]) + (-3.448627e-04) * (q[i1] * q[i9]) + (8.148392e-04) * (q[i1] * q[i10]) + (-1.317697e-04) * (q[i1] * q[i11])
            + (-6.142505e-04) * (q[i1] * q[i12]) + (-1.042180e-03) * (q[i1] * q[i15]) + (-1.634519e-04) * (q[i1] * q[i16]) + (4.608439e-04) * (q[i1] * q[i19])
            + (5.044497e-04) * (q[i1] * q[i21]) + (2.404725e-04) * (q[i1] * q[i22]) + (5.090413e-04) * (q[i2] * q[i3]) + (9.312876e-04) * (q[i2] * q[i4])
            + (-1.194357e-03) * (q[i2] * q[i5]) + (3.018136e-04) * (q[i2] * q[i6]) + (1.274549e-03) * (q[i2] * q[i7]) + (1.062139e-03) * (q[i2] * q[i8])
            + (3.097910e-04) * (q[i2] * q[i9]) + (1.234765e-04) * (q[i2] * q[i10]) + (5.165242e-05) * (q[i2] * q[i11]) + (-1.678354e-03) * (q[i2] * q[i12])
            + (-8.625585e-04) * (q[i2] * q[i15]) + (-9.489442e-04) * (q[i2] * q[i16]) + (1.081195e-03) * (q[i2] * q[i19]) + (5.348327e-04) * (q[i2] * q[i21])
            + (1.183849e-03) * (q[i2] * q[i22]) + (7.792435e-04) * (q[i3] * q[i4]) + (-1.240502e-03) * (q[i3] * q[i5]) + (-1.961914e-04) * (q[i3] * q[i6])
            + (1.872840e-03) * (q[i3] * q[i7]) + (4.997872e-05) * (q[i3] * q[i8]) + (-1.099831e-03) * (q[i3] * q[i9]) + (-1.118422e-04) * (q[i3] * q[i10])
            + (-6.758842e-05) * (q[i3] * q[i11]) + (8.499335e-04) * (q[i3] * q[i12]) + (-5.463849e-04) * (q[i3] * q[i15]) + (4.429594e-04) * (q[i3] * q[i16])
            + (6.337695e-04) * (q[i3] * q[i19]) + (-3.009105e-04) * (q[i3] * q[i21]) + (-1.722683e-04) * (q[i3] * q[i22]) + (8.317277e-04) * (q[i4] * q[i5])
            + (-1.470723e-04) * (q[i4] * q[i6]) + (-2.927266e-03) * (q[i4] * q[i7]) + (-1.055922e-03) * (q[i4] * q[i8]) + (9.484636e-04) * (q[i4] * q[i9])
            + (2.507049e-04) * (q[i4] * q[i10]) + (5.438664e-04) * (q[i4] * q[i11]) + (-3.604102e-04) * (q[i4] * q[i12]) + (7.146477e-04) * (q[i4] * q[i15])
            + (-4.883223e-04) * (q[i4] * q[i16]) + (6.201209e-04) * (q[i4] * q[i19]) + (3.878512e-04) * (q[i4] * q[i21]) + (-1.278620e-03) * (q[i4] * q[i22])
            + (6.275868e-04) * (q[i5] * q[i6]) + (2.373423e-04) * (q[i5] * q[i7]) + (-1.517377e-03) * (q[i5] * q[i8]) + (6.840200e-05) * (q[i5] * q[i9])
            + (-1.415325e-04) * (q[i5] * q[i10]) + (-4.212765e-04) * (q[i5] * q[i11]) + (-7.160581e-04) * (q[i5] * q[i12]) + (-3.359705e-04) * (q[i5] * q[i15])
            + (-7.404865e-04) * (q[i5] * q[i16]) + (-5.472268e-04) * (q[i5] * q[i19]) + (-5.014894e-04) * (q[i5] * q[i21]) + (1.779544e-03) * (q[i5] * q[i22])
            + (-3.841827e-04) * (q[i6] * q[i7]) + (-2.926902e-06) * (q[i6] * q[i8]) + (-2.811633e-04) * (q[i6] * q[i9]) + (-6.403651e-05) * (q[i6] * q[i10])
            + (1.247018e-03) * (q[i6] * q[i11]) + (-1.390614e-04) * (q[i6] * q[i12]) + (-2.391586e-04) * (q[i6] * q[i15]) + (1.109683e-05) * (q[i6] * q[i16])
            + (1.838184e-04) * (q[i6] * q[i19]) + (8.421887e-04) * (q[i6] * q[i21]) + (-6.182464e-04) * (q[i6] * q[i22]) + (3.094587e-04) * (q[i7] * q[i8])
            + (-3.490015e-07) * (q[i7] * q[i9]) + (5.044036e-04) * (q[i7] * q[i10]) + (1.173055e-04) * (q[i7] * q[i11]) + (2.493281e-05) * (q[i7] * q[i12])
            + (4.214893e-04) * (q[i7] * q[i15]) + (-2.380913e-04) * (q[i7] * q[i16]) + (-1.807043e-04) * (q[i7] * q[i19]) + (-8.372730e-05) * (q[i7] * q[i21])
            + (-1.313692e-04) * (q[i7] * q[i22]) + (5.502630e-05) * (q[i8] * q[i9]) + (2.439533e-04) * (q[i8] * q[i10]) + (-1.831442e-04) * (q[i8] * q[i11])
            + (1.059739e-03) * (q[i8] * q[i12]) + (2.050454e-04) * (q[i8] * q[i15]) + (-4.268456e-04) * (q[i8] * q[i16]) + (5.811766e-08) * (q[i8] * q[i19])
            + (-8.841363e-06) * (q[i8] * q[i21]) + (-1.338323e-03) * (q[i8] * q[i22]) + (4.880631e-05) * (q[i9] * q[i10]) + (2.088469e-04) * (q[i9] * q[i11])
            + (-5.902861e-05) * (q[i9] * q[i12]) + (1.634168e-04) * (q[i9] * q[i15]) + (1.908896e-04) * (q[i9] * q[i16]) + (1.544749e-04) * (q[i9] * q[i19])
            + (1.741792e-04) * (q[i9] * q[i21]) + (-3.629460e-04) * (q[i9] * q[i22]) + (-8.202952e-05) * (q[i10] * q[i11]) + (4.660277e-05) * (q[i10] * q[i12])
            + (2.572036e-04) * (q[i10] * q[i15]) + (-1.470767e-04) * (q[i10] * q[i16]) + (-1.553178e-04) * (q[i10] * q[i19])
            + (1.273184e-04) * (q[i10] * q[i21]) + (-2.134184e-05) * (q[i10] * q[i22]) + (-3.485009e-04) * (q[i11] * q[i12])
            + (-2.733872e-05) * (q[i11] * q[i15]) + (1.390104e-04) * (q[i11] * q[i16]) + (-2.405631e-04) * (q[i11] * q[i19])
            + (1.702902e-04) * (q[i11] * q[i21]) + (-2.498516e-04) * (q[i11] * q[i22]) + (6.039360e-05) * (q[i12] * q[i15])
            + (-4.167561e-04) * (q[i12] * q[i16]) + (-2.413345e-04) * (q[i12] * q[i19]) + (-6.266256e-05) * (q[i12] * q[i21])
            + (2.050566e-03) * (q[i12] * q[i22]) + (-2.091147e-04) * (q[i15] * q[i16]) + (1.092114e-05) * (q[i15] * q[i19]) + (1.848389e-06) * (q[i15] * q[i21])
            + (-2.399921e-05) * (q[i15] * q[i22]) + (-7.888197e-06) * (q[i16] * q[i19]) + (-8.441539e-05) * (q[i16] * q[i21])
            + (6.254680e-05) * (q[i16] * q[i22]) + (1.641668e-04) * (q[i19] * q[i21]) + (1.603013e-04) * (q[i19] * q[i22])
            + (-5.825105e-05) * (q[i21] * q[i22]);
   }

   public void getJQx21(double[] q, double[][] JQ)
   {
      JQ[1][i21] = (-2.718647e-03) * (1) + (1.668478e-03) * ((2) * q[i21]) + (1.479821e-03) * (q[i0]) + (-3.335710e-04) * (q[i1]) + (1.403467e-03) * (q[i2])
            + (-9.036793e-04) * (q[i3]) + (-5.053012e-04) * (q[i4]) + (-8.173700e-04) * (q[i5]) + (-7.083634e-04) * (q[i6]) + (-1.223041e-03) * (q[i7])
            + (2.241619e-03) * (q[i8]) + (2.236456e-04) * (q[i9]) + (-5.361554e-05) * (q[i10]) + (4.262676e-04) * (q[i11]) + (2.662284e-04) * (q[i12])
            + (2.450143e-03) * (q[i15]) + (2.959218e-04) * (q[i16]) + (-9.128984e-04) * (q[i19]) + (-1.065760e-04) * (q[i20]) + (-9.178308e-06) * (q[i22])
            + (6.331864e-04) * (q[i0] * q[i0]) + (-2.780366e-04) * (q[i1] * q[i1]) + (1.112668e-03) * (q[i2] * q[i2]) + (1.536098e-04) * (q[i3] * q[i3])
            + (3.029220e-04) * (q[i4] * q[i4]) + (-4.744346e-04) * (q[i5] * q[i5]) + (2.497640e-04) * (q[i6] * q[i6]) + (-5.819971e-05) * (q[i7] * q[i7])
            + (-6.908828e-04) * (q[i8] * q[i8]) + (-2.193992e-05) * (q[i9] * q[i9]) + (-2.028236e-05) * (q[i10] * q[i10]) + (8.513405e-04) * (q[i11] * q[i11])
            + (5.719601e-05) * (q[i12] * q[i12]) + (3.968051e-04) * (q[i15] * q[i15]) + (-1.105465e-05) * (q[i16] * q[i16]) + (6.685921e-04) * (q[i19] * q[i19])
            + (-7.287472e-05) * (q[i20] * q[i20]) + (2.005327e-04) * ((2) * q[i0] * q[i21]) + (3.551545e-04) * ((2) * q[i1] * q[i21])
            + (6.122352e-04) * ((2) * q[i2] * q[i21]) + (2.876675e-04) * ((2) * q[i3] * q[i21]) + (1.871116e-04) * ((2) * q[i4] * q[i21])
            + (-1.374830e-03) * ((2) * q[i5] * q[i21]) + (-1.487235e-04) * ((2) * q[i6] * q[i21]) + (-3.780352e-04) * ((2) * q[i7] * q[i21])
            + (6.811219e-04) * ((2) * q[i8] * q[i21]) + (1.256556e-04) * ((2) * q[i9] * q[i21]) + (-1.027291e-04) * ((2) * q[i10] * q[i21])
            + (4.836974e-04) * ((2) * q[i11] * q[i21]) + (1.767875e-04) * ((2) * q[i12] * q[i21]) + (7.926800e-04) * ((2) * q[i15] * q[i21])
            + (-1.365182e-05) * ((2) * q[i16] * q[i21]) + (-9.116763e-05) * ((2) * q[i19] * q[i21]) + (-1.229703e-04) * ((2) * q[i20] * q[i21])
            + (3.620149e-04) * ((3) * q[i21] * q[i21]) + (3.931188e-05) * ((2) * q[i21] * q[i22]) + (4.055755e-05) * (q[i22] * q[i22])
            + (2.626669e-04) * (q[i0] * q[i1]) + (5.814716e-04) * (q[i0] * q[i2]) + (-8.057844e-04) * (q[i0] * q[i3]) + (-3.788516e-04) * (q[i0] * q[i4])
            + (6.680767e-04) * (q[i0] * q[i5]) + (-8.987898e-04) * (q[i0] * q[i6]) + (-2.006549e-04) * (q[i0] * q[i7]) + (3.697073e-04) * (q[i0] * q[i8])
            + (-4.827697e-04) * (q[i0] * q[i9]) + (-6.733610e-04) * (q[i0] * q[i10]) + (7.605467e-04) * (q[i0] * q[i11]) + (4.240182e-04) * (q[i0] * q[i12])
            + (2.398073e-04) * (q[i0] * q[i15]) + (-2.354564e-04) * (q[i0] * q[i16]) + (-2.430971e-04) * (q[i0] * q[i19]) + (7.464442e-04) * (q[i0] * q[i20])
            + (1.707779e-04) * (q[i0] * q[i22]) + (1.109319e-04) * (q[i1] * q[i2]) + (5.962383e-04) * (q[i1] * q[i3]) + (-7.710065e-04) * (q[i1] * q[i4])
            + (-2.455253e-04) * (q[i1] * q[i5]) + (8.439397e-04) * (q[i1] * q[i6]) + (-6.180849e-04) * (q[i1] * q[i7]) + (3.351103e-04) * (q[i1] * q[i8])
            + (4.375698e-04) * (q[i1] * q[i9]) + (5.603105e-05) * (q[i1] * q[i10]) + (-7.379106e-04) * (q[i1] * q[i11]) + (3.496778e-04) * (q[i1] * q[i12])
            + (2.924900e-05) * (q[i1] * q[i15]) + (-5.340071e-04) * (q[i1] * q[i16]) + (-8.342090e-04) * (q[i1] * q[i19]) + (5.044497e-04) * (q[i1] * q[i20])
            + (1.760598e-04) * (q[i1] * q[i22]) + (-7.309653e-04) * (q[i2] * q[i3]) + (9.853955e-04) * (q[i2] * q[i4]) + (5.688177e-04) * (q[i2] * q[i5])
            + (1.285496e-04) * (q[i2] * q[i6]) + (-3.183364e-04) * (q[i2] * q[i7]) + (-5.368134e-04) * (q[i2] * q[i8]) + (-3.177849e-04) * (q[i2] * q[i9])
            + (-2.055229e-04) * (q[i2] * q[i10]) + (8.125435e-05) * (q[i2] * q[i11]) + (-1.404150e-04) * (q[i2] * q[i12]) + (3.715050e-04) * (q[i2] * q[i15])
            + (-3.632431e-04) * (q[i2] * q[i16]) + (-1.188958e-03) * (q[i2] * q[i19]) + (5.348327e-04) * (q[i2] * q[i20]) + (2.080626e-04) * (q[i2] * q[i22])
            + (-7.415448e-04) * (q[i3] * q[i4]) + (3.996984e-05) * (q[i3] * q[i5]) + (-8.116925e-04) * (q[i3] * q[i6]) + (1.340993e-03) * (q[i3] * q[i7])
            + (1.835052e-04) * (q[i3] * q[i8]) + (2.429874e-04) * (q[i3] * q[i9]) + (-2.816300e-04) * (q[i3] * q[i10]) + (-8.200449e-04) * (q[i3] * q[i11])
            + (8.870416e-04) * (q[i3] * q[i12]) + (-1.765358e-04) * (q[i3] * q[i15]) + (-2.275339e-05) * (q[i3] * q[i16]) + (1.284728e-03) * (q[i3] * q[i19])
            + (-3.009105e-04) * (q[i3] * q[i20]) + (2.200795e-04) * (q[i3] * q[i22]) + (1.183960e-03) * (q[i4] * q[i5]) + (-7.265601e-04) * (q[i4] * q[i6])
            + (-3.443914e-04) * (q[i4] * q[i7]) + (1.136042e-04) * (q[i4] * q[i8]) + (-3.795851e-04) * (q[i4] * q[i9]) + (1.679941e-04) * (q[i4] * q[i10])
            + (-2.493820e-04) * (q[i4] * q[i11]) + (-1.056132e-03) * (q[i4] * q[i12]) + (5.753389e-04) * (q[i4] * q[i15]) + (-1.986130e-04) * (q[i4] * q[i16])
            + (1.730511e-04) * (q[i4] * q[i19]) + (3.878512e-04) * (q[i4] * q[i20]) + (2.073554e-04) * (q[i4] * q[i22]) + (4.400469e-04) * (q[i5] * q[i6])
            + (-5.079405e-04) * (q[i5] * q[i7]) + (6.891492e-04) * (q[i5] * q[i8]) + (5.699121e-05) * (q[i5] * q[i9]) + (1.988570e-04) * (q[i5] * q[i10])
            + (-2.158314e-04) * (q[i5] * q[i11]) + (-3.110414e-04) * (q[i5] * q[i12]) + (-1.399082e-03) * (q[i5] * q[i15]) + (-1.216028e-04) * (q[i5] * q[i16])
            + (-1.777132e-03) * (q[i5] * q[i19]) + (-5.014894e-04) * (q[i5] * q[i20]) + (-5.758126e-04) * (q[i5] * q[i22]) + (-2.921076e-04) * (q[i6] * q[i7])
            + (3.936861e-05) * (q[i6] * q[i8]) + (-5.704405e-06) * (q[i6] * q[i9]) + (6.740169e-04) * (q[i6] * q[i10]) + (-3.542402e-04) * (q[i6] * q[i11])
            + (9.065239e-05) * (q[i6] * q[i12]) + (-5.152226e-05) * (q[i6] * q[i15]) + (1.089463e-04) * (q[i6] * q[i16]) + (-1.257420e-04) * (q[i6] * q[i19])
            + (8.421887e-04) * (q[i6] * q[i20]) + (-1.593029e-04) * (q[i6] * q[i22]) + (6.312095e-04) * (q[i7] * q[i8]) + (3.703065e-04) * (q[i7] * q[i9])
            + (6.170180e-06) * (q[i7] * q[i10]) + (5.647104e-04) * (q[i7] * q[i11]) + (3.914591e-04) * (q[i7] * q[i12]) + (1.458426e-04) * (q[i7] * q[i15])
            + (2.063762e-04) * (q[i7] * q[i16]) + (-6.118894e-04) * (q[i7] * q[i19]) + (-8.372730e-05) * (q[i7] * q[i20]) + (1.575562e-04) * (q[i7] * q[i22])
            + (1.687497e-04) * (q[i8] * q[i9]) + (-6.443793e-04) * (q[i8] * q[i10]) + (1.827731e-03) * (q[i8] * q[i11]) + (4.025228e-05) * (q[i8] * q[i12])
            + (-1.066225e-04) * (q[i8] * q[i15]) + (-4.533561e-04) * (q[i8] * q[i16]) + (-1.353064e-03) * (q[i8] * q[i19]) + (-8.841363e-06) * (q[i8] * q[i20])
            + (-1.803648e-06) * (q[i8] * q[i22]) + (1.934615e-04) * (q[i9] * q[i10]) + (2.569153e-05) * (q[i9] * q[i11]) + (2.179966e-04) * (q[i9] * q[i12])
            + (6.501105e-05) * (q[i9] * q[i15]) + (4.393048e-05) * (q[i9] * q[i16]) + (-2.061993e-05) * (q[i9] * q[i19]) + (1.741792e-04) * (q[i9] * q[i20])
            + (-1.726096e-05) * (q[i9] * q[i22]) + (-9.864996e-05) * (q[i10] * q[i11]) + (1.714616e-04) * (q[i10] * q[i12])
            + (-9.423758e-05) * (q[i10] * q[i15]) + (-4.275113e-05) * (q[i10] * q[i16]) + (-3.572332e-04) * (q[i10] * q[i19])
            + (1.273184e-04) * (q[i10] * q[i20]) + (1.970438e-05) * (q[i10] * q[i22]) + (-4.512187e-04) * (q[i11] * q[i12])
            + (-4.815746e-04) * (q[i11] * q[i15]) + (-6.533038e-04) * (q[i11] * q[i16]) + (-2.057011e-03) * (q[i11] * q[i19])
            + (1.702902e-04) * (q[i11] * q[i20]) + (8.413113e-05) * (q[i11] * q[i22]) + (3.469436e-04) * (q[i12] * q[i15]) + (1.295322e-04) * (q[i12] * q[i16])
            + (2.520685e-04) * (q[i12] * q[i19]) + (-6.266256e-05) * (q[i12] * q[i20]) + (8.656308e-05) * (q[i12] * q[i22]) + (1.579448e-04) * (q[i15] * q[i16])
            + (5.399830e-05) * (q[i15] * q[i19]) + (1.848389e-06) * (q[i15] * q[i20]) + (-6.705985e-05) * (q[i15] * q[i22])
            + (-2.416558e-05) * (q[i16] * q[i19]) + (-8.441539e-05) * (q[i16] * q[i20]) + (6.619962e-05) * (q[i16] * q[i22])
            + (1.641668e-04) * (q[i19] * q[i20]) + (6.385282e-05) * (q[i19] * q[i22]) + (-5.825105e-05) * (q[i20] * q[i22]);
   }

   public void getJQx22(double[] q, double[][] JQ)
   {
      JQ[1][i22] = (-2.768467e-03) * (1) + (-1.674249e-03) * ((2) * q[i22]) + (3.335293e-04) * (q[i0]) + (-1.478069e-03) * (q[i1]) + (-1.403050e-03) * (q[i2])
            + (5.059900e-04) * (q[i3]) + (8.770941e-04) * (q[i4]) + (8.332752e-04) * (q[i5]) + (-1.230140e-03) * (q[i6]) + (-6.863549e-04) * (q[i7])
            + (2.269841e-03) * (q[i8]) + (-5.577125e-05) * (q[i9]) + (2.301890e-04) * (q[i10]) + (-2.681229e-04) * (q[i11]) + (-4.255399e-04) * (q[i12])
            + (2.765729e-04) * (q[i15]) + (2.443170e-03) * (q[i16]) + (-9.662913e-05) * (q[i19]) + (-8.800162e-04) * (q[i20]) + (-9.178308e-06) * (q[i21])
            + (-2.889568e-04) * (q[i0] * q[i0]) + (6.328589e-04) * (q[i1] * q[i1]) + (1.105582e-03) * (q[i2] * q[i2]) + (3.081561e-04) * (q[i3] * q[i3])
            + (1.453614e-04) * (q[i4] * q[i4]) + (-4.877765e-04) * (q[i5] * q[i5]) + (-6.035456e-05) * (q[i6] * q[i6]) + (2.482698e-04) * (q[i7] * q[i7])
            + (-6.994664e-04) * (q[i8] * q[i8]) + (-1.976300e-05) * (q[i9] * q[i9]) + (-2.286918e-05) * (q[i10] * q[i10]) + (6.335773e-05) * (q[i11] * q[i11])
            + (8.551246e-04) * (q[i12] * q[i12]) + (-1.268897e-05) * (q[i15] * q[i15]) + (4.009891e-04) * (q[i16] * q[i16])
            + (-7.300996e-05) * (q[i19] * q[i19]) + (6.647343e-04) * (q[i20] * q[i20]) + (3.931188e-05) * (q[i21] * q[i21])
            + (3.563537e-04) * ((2) * q[i0] * q[i22]) + (1.959782e-04) * ((2) * q[i1] * q[i22]) + (6.068116e-04) * ((2) * q[i2] * q[i22])
            + (1.916338e-04) * ((2) * q[i3] * q[i22]) + (2.904623e-04) * ((2) * q[i4] * q[i22]) + (-1.379575e-03) * ((2) * q[i5] * q[i22])
            + (3.754935e-04) * ((2) * q[i6] * q[i22]) + (1.424931e-04) * ((2) * q[i7] * q[i22]) + (-6.902480e-04) * ((2) * q[i8] * q[i22])
            + (1.046439e-04) * ((2) * q[i9] * q[i22]) + (-1.250614e-04) * ((2) * q[i10] * q[i22]) + (1.762423e-04) * ((2) * q[i11] * q[i22])
            + (4.884731e-04) * ((2) * q[i12] * q[i22]) + (1.745091e-05) * ((2) * q[i15] * q[i22]) + (-7.943027e-04) * ((2) * q[i16] * q[i22])
            + (1.245651e-04) * ((2) * q[i19] * q[i22]) + (8.810742e-05) * ((2) * q[i20] * q[i22]) + (4.055755e-05) * ((2) * q[i21] * q[i22])
            + (3.661811e-04) * ((3) * q[i22] * q[i22]) + (2.594540e-04) * (q[i0] * q[i1]) + (1.058040e-04) * (q[i0] * q[i2]) + (-7.652511e-04) * (q[i0] * q[i3])
            + (6.018258e-04) * (q[i0] * q[i4]) + (-2.400176e-04) * (q[i0] * q[i5]) + (6.300372e-04) * (q[i0] * q[i6]) + (-8.392349e-04) * (q[i0] * q[i7])
            + (-3.461998e-04) * (q[i0] * q[i8]) + (-5.376229e-05) * (q[i0] * q[i9]) + (-4.308028e-04) * (q[i0] * q[i10]) + (3.498287e-04) * (q[i0] * q[i11])
            + (-7.301080e-04) * (q[i0] * q[i12]) + (5.474682e-04) * (q[i0] * q[i15]) + (-2.929430e-05) * (q[i0] * q[i16]) + (-5.005325e-04) * (q[i0] * q[i19])
            + (8.370573e-04) * (q[i0] * q[i20]) + (1.707779e-04) * (q[i0] * q[i21]) + (5.835975e-04) * (q[i1] * q[i2]) + (-3.858473e-04) * (q[i1] * q[i3])
            + (-8.268980e-04) * (q[i1] * q[i4]) + (6.741223e-04) * (q[i1] * q[i5]) + (1.852810e-04) * (q[i1] * q[i6]) + (9.134953e-04) * (q[i1] * q[i7])
            + (-3.669231e-04) * (q[i1] * q[i8]) + (6.739413e-04) * (q[i1] * q[i9]) + (4.743153e-04) * (q[i1] * q[i10]) + (4.227531e-04) * (q[i1] * q[i11])
            + (7.630297e-04) * (q[i1] * q[i12]) + (2.277305e-04) * (q[i1] * q[i15]) + (-2.454685e-04) * (q[i1] * q[i16]) + (-7.396063e-04) * (q[i1] * q[i19])
            + (2.404725e-04) * (q[i1] * q[i20]) + (1.760598e-04) * (q[i1] * q[i21]) + (1.000350e-03) * (q[i2] * q[i3]) + (-7.527671e-04) * (q[i2] * q[i4])
            + (5.556571e-04) * (q[i2] * q[i5]) + (3.242108e-04) * (q[i2] * q[i6]) + (-1.257341e-04) * (q[i2] * q[i7]) + (5.417197e-04) * (q[i2] * q[i8])
            + (2.041463e-04) * (q[i2] * q[i9]) + (3.180438e-04) * (q[i2] * q[i10]) + (-1.375290e-04) * (q[i2] * q[i11]) + (8.041627e-05) * (q[i2] * q[i12])
            + (3.675351e-04) * (q[i2] * q[i15]) + (-3.792889e-04) * (q[i2] * q[i16]) + (-5.315273e-04) * (q[i2] * q[i19]) + (1.183849e-03) * (q[i2] * q[i20])
            + (2.080626e-04) * (q[i2] * q[i21]) + (-7.459352e-04) * (q[i3] * q[i4]) + (1.160825e-03) * (q[i3] * q[i5]) + (3.394242e-04) * (q[i3] * q[i6])
            + (7.346822e-04) * (q[i3] * q[i7]) + (-1.137578e-04) * (q[i3] * q[i8]) + (-1.711662e-04) * (q[i3] * q[i9]) + (3.857699e-04) * (q[i3] * q[i10])
            + (-1.065079e-03) * (q[i3] * q[i11]) + (-2.562754e-04) * (q[i3] * q[i12]) + (2.002703e-04) * (q[i3] * q[i15]) + (-5.864281e-04) * (q[i3] * q[i16])
            + (-3.871210e-04) * (q[i3] * q[i19]) + (-1.722683e-04) * (q[i3] * q[i20]) + (2.200795e-04) * (q[i3] * q[i21]) + (7.428462e-05) * (q[i4] * q[i5])
            + (-1.339614e-03) * (q[i4] * q[i6]) + (8.115579e-04) * (q[i4] * q[i7]) + (-1.860377e-04) * (q[i4] * q[i8]) + (2.792880e-04) * (q[i4] * q[i9])
            + (-2.440550e-04) * (q[i4] * q[i10]) + (8.925204e-04) * (q[i4] * q[i11]) + (-8.202490e-04) * (q[i4] * q[i12]) + (1.739075e-05) * (q[i4] * q[i15])
            + (1.665167e-04) * (q[i4] * q[i16]) + (3.040057e-04) * (q[i4] * q[i19]) + (-1.278620e-03) * (q[i4] * q[i20]) + (2.073554e-04) * (q[i4] * q[i21])
            + (5.221971e-04) * (q[i5] * q[i6]) + (-4.616325e-04) * (q[i5] * q[i7]) + (-7.023000e-04) * (q[i5] * q[i8]) + (-1.983944e-04) * (q[i5] * q[i9])
            + (-7.148407e-05) * (q[i5] * q[i10]) + (-3.102334e-04) * (q[i5] * q[i11]) + (-2.057889e-04) * (q[i5] * q[i12]) + (1.192572e-04) * (q[i5] * q[i15])
            + (1.415512e-03) * (q[i5] * q[i16]) + (5.059876e-04) * (q[i5] * q[i19]) + (1.779544e-03) * (q[i5] * q[i20]) + (-5.758126e-04) * (q[i5] * q[i21])
            + (-2.994227e-04) * (q[i6] * q[i7]) + (6.272907e-04) * (q[i6] * q[i8]) + (6.697351e-06) * (q[i6] * q[i9]) + (3.724830e-04) * (q[i6] * q[i10])
            + (-3.928276e-04) * (q[i6] * q[i11]) + (-5.720250e-04) * (q[i6] * q[i12]) + (2.113318e-04) * (q[i6] * q[i15]) + (1.466666e-04) * (q[i6] * q[i16])
            + (-7.666956e-05) * (q[i6] * q[i19]) + (-6.182464e-04) * (q[i6] * q[i20]) + (-1.593029e-04) * (q[i6] * q[i21]) + (3.965294e-05) * (q[i7] * q[i8])
            + (6.726312e-04) * (q[i7] * q[i9]) + (-1.011493e-05) * (q[i7] * q[i10]) + (-8.827581e-05) * (q[i7] * q[i11]) + (3.562947e-04) * (q[i7] * q[i12])
            + (1.105229e-04) * (q[i7] * q[i15]) + (-5.774850e-05) * (q[i7] * q[i16]) + (8.430038e-04) * (q[i7] * q[i19]) + (-1.313692e-04) * (q[i7] * q[i20])
            + (1.575562e-04) * (q[i7] * q[i21]) + (-6.421777e-04) * (q[i8] * q[i9]) + (1.681949e-04) * (q[i8] * q[i10]) + (-3.900571e-05) * (q[i8] * q[i11])
            + (-1.843622e-03) * (q[i8] * q[i12]) + (-4.518506e-04) * (q[i8] * q[i15]) + (-8.338577e-05) * (q[i8] * q[i16]) + (-1.799147e-05) * (q[i8] * q[i19])
            + (-1.338323e-03) * (q[i8] * q[i20]) + (-1.803648e-06) * (q[i8] * q[i21]) + (1.904536e-04) * (q[i9] * q[i10]) + (-1.682093e-04) * (q[i9] * q[i11])
            + (9.555884e-05) * (q[i9] * q[i12]) + (-3.990033e-05) * (q[i9] * q[i15]) + (-1.016466e-04) * (q[i9] * q[i16]) + (1.277229e-04) * (q[i9] * q[i19])
            + (-3.629460e-04) * (q[i9] * q[i20]) + (-1.726096e-05) * (q[i9] * q[i21]) + (-2.166452e-04) * (q[i10] * q[i11])
            + (-2.717697e-05) * (q[i10] * q[i12]) + (4.601349e-05) * (q[i10] * q[i15]) + (6.190540e-05) * (q[i10] * q[i16]) + (1.728515e-04) * (q[i10] * q[i19])
            + (-2.134184e-05) * (q[i10] * q[i20]) + (1.970438e-05) * (q[i10] * q[i21]) + (-4.540651e-04) * (q[i11] * q[i12])
            + (-1.275106e-04) * (q[i11] * q[i15]) + (-3.496984e-04) * (q[i11] * q[i16]) + (6.084827e-05) * (q[i11] * q[i19])
            + (-2.498516e-04) * (q[i11] * q[i20]) + (8.413113e-05) * (q[i11] * q[i21]) + (6.541224e-04) * (q[i12] * q[i15]) + (4.850914e-04) * (q[i12] * q[i16])
            + (-1.723724e-04) * (q[i12] * q[i19]) + (2.050566e-03) * (q[i12] * q[i20]) + (8.656308e-05) * (q[i12] * q[i21]) + (1.535920e-04) * (q[i15] * q[i16])
            + (-8.669724e-05) * (q[i15] * q[i19]) + (-2.399921e-05) * (q[i15] * q[i20]) + (-6.705985e-05) * (q[i15] * q[i21])
            + (5.949074e-06) * (q[i16] * q[i19]) + (6.254680e-05) * (q[i16] * q[i20]) + (6.619962e-05) * (q[i16] * q[i21]) + (1.603013e-04) * (q[i19] * q[i20])
            + (6.385282e-05) * (q[i19] * q[i21]) + (-5.825105e-05) * (q[i20] * q[i21]);
   }

   public void getJQy(double[] q, double[][] JQ)
   {
      getJQy0(q, JQ);
      getJQy1(q, JQ);
      getJQy2(q, JQ);
      getJQy3(q, JQ);
      getJQy4(q, JQ);
      getJQy5(q, JQ);
      getJQy6(q, JQ);
      getJQy7(q, JQ);
      getJQy8(q, JQ);
      getJQy9(q, JQ);
      getJQy10(q, JQ);
      getJQy11(q, JQ);
      getJQy12(q, JQ);
      getJQy15(q, JQ);
      getJQy16(q, JQ);
      getJQy19(q, JQ);
      getJQy20(q, JQ);
      getJQy21(q, JQ);
      getJQy22(q, JQ);
   }

   public void getJQy0(double[] q, double[][] JQ)
   {
      JQ[2][i0] = (-1.118106e-03) * (1) + (4.467133e-03) * ((2) * q[i0]) + (7.115423e-05) * (q[i1]) + (2.760604e-03) * (q[i2]) + (-1.082041e-01) * (q[i3])
            + (1.091409e-02) * (q[i4]) + (1.464916e-03) * (q[i5]) + (6.008783e-03) * (q[i6]) + (5.937328e-04) * (q[i7]) + (1.223880e-04) * (q[i8])
            + (-1.577200e-03) * (q[i9]) + (-2.364382e-03) * (q[i10]) + (-2.101461e-03) * (q[i11]) + (-6.247272e-04) * (q[i12]) + (1.763563e-03) * (q[i15])
            + (-5.522394e-03) * (q[i16]) + (-1.921777e-03) * (q[i19]) + (8.267937e-04) * (q[i20]) + (2.222365e-03) * (q[i21]) + (-8.863129e-04) * (q[i22])
            + (2.699103e-03) * ((3) * q[i0] * q[i0]) + (4.217759e-04) * ((2) * q[i0] * q[i1]) + (2.148421e-03) * ((2) * q[i0] * q[i2])
            + (-2.877567e-03) * ((2) * q[i0] * q[i3]) + (-1.957202e-03) * ((2) * q[i0] * q[i4]) + (-4.799782e-03) * ((2) * q[i0] * q[i5])
            + (-2.172771e-02) * ((2) * q[i0] * q[i6]) + (-2.006883e-03) * ((2) * q[i0] * q[i7]) + (1.524819e-03) * ((2) * q[i0] * q[i8])
            + (-3.620341e-03) * ((2) * q[i0] * q[i9]) + (-4.255330e-04) * ((2) * q[i0] * q[i10]) + (1.261540e-03) * ((2) * q[i0] * q[i11])
            + (-2.336911e-04) * ((2) * q[i0] * q[i12]) + (7.602174e-04) * ((2) * q[i0] * q[i15]) + (-1.045320e-04) * ((2) * q[i0] * q[i16])
            + (8.855494e-05) * ((2) * q[i0] * q[i19]) + (7.927038e-04) * ((2) * q[i0] * q[i20]) + (1.487935e-04) * ((2) * q[i0] * q[i21])
            + (2.358285e-04) * ((2) * q[i0] * q[i22]) + (-3.711844e-04) * (q[i1] * q[i1]) + (1.270791e-03) * (q[i2] * q[i2]) + (7.799925e-03) * (q[i3] * q[i3])
            + (2.965714e-03) * (q[i4] * q[i4]) + (-3.637586e-04) * (q[i5] * q[i5]) + (-4.331391e-03) * (q[i6] * q[i6]) + (1.095570e-03) * (q[i7] * q[i7])
            + (-1.652844e-03) * (q[i8] * q[i8]) + (1.141431e-04) * (q[i9] * q[i9]) + (5.209430e-04) * (q[i10] * q[i10]) + (2.709956e-04) * (q[i11] * q[i11])
            + (4.285072e-04) * (q[i12] * q[i12]) + (4.538791e-05) * (q[i15] * q[i15]) + (-7.146619e-04) * (q[i16] * q[i16])
            + (-1.345468e-04) * (q[i19] * q[i19]) + (2.372272e-04) * (q[i20] * q[i20]) + (1.887070e-04) * (q[i21] * q[i21]) + (4.910673e-05) * (q[i22] * q[i22])
            + (2.041900e-05) * (q[i1] * q[i2]) + (-2.203863e-03) * (q[i1] * q[i3]) + (2.176018e-03) * (q[i1] * q[i4]) + (-3.023284e-06) * (q[i1] * q[i5])
            + (4.607573e-03) * (q[i1] * q[i6]) + (4.585382e-03) * (q[i1] * q[i7]) + (3.606869e-03) * (q[i1] * q[i8]) + (3.022323e-04) * (q[i1] * q[i9])
            + (2.846155e-04) * (q[i1] * q[i10]) + (-7.803878e-04) * (q[i1] * q[i11]) + (7.788289e-04) * (q[i1] * q[i12]) + (-5.317338e-04) * (q[i1] * q[i15])
            + (-5.364147e-04) * (q[i1] * q[i16]) + (-2.149220e-03) * (q[i1] * q[i19]) + (-2.164970e-03) * (q[i1] * q[i20]) + (1.059114e-03) * (q[i1] * q[i21])
            + (-1.058629e-03) * (q[i1] * q[i22]) + (-4.830283e-03) * (q[i2] * q[i3]) + (-3.669630e-04) * (q[i2] * q[i4]) + (-1.346757e-03) * (q[i2] * q[i5])
            + (-8.850569e-03) * (q[i2] * q[i6]) + (1.509462e-03) * (q[i2] * q[i7]) + (-2.242384e-03) * (q[i2] * q[i8]) + (-1.992049e-03) * (q[i2] * q[i9])
            + (2.438050e-03) * (q[i2] * q[i10]) + (6.523406e-04) * (q[i2] * q[i11]) + (8.603355e-04) * (q[i2] * q[i12]) + (9.454998e-05) * (q[i2] * q[i15])
            + (1.685530e-04) * (q[i2] * q[i16]) + (-5.308325e-04) * (q[i2] * q[i19]) + (-7.462804e-04) * (q[i2] * q[i20]) + (1.166162e-03) * (q[i2] * q[i21])
            + (-1.663098e-03) * (q[i2] * q[i22]) + (1.577433e-02) * (q[i3] * q[i4]) + (8.064864e-03) * (q[i3] * q[i5]) + (-1.744554e-02) * (q[i3] * q[i6])
            + (7.487355e-04) * (q[i3] * q[i7]) + (-6.385192e-04) * (q[i3] * q[i8]) + (1.275901e-02) * (q[i3] * q[i9]) + (-2.628810e-03) * (q[i3] * q[i10])
            + (3.415507e-04) * (q[i3] * q[i11]) + (-1.892927e-03) * (q[i3] * q[i12]) + (-2.736937e-04) * (q[i3] * q[i15]) + (-4.429769e-03) * (q[i3] * q[i16])
            + (-1.283668e-03) * (q[i3] * q[i19]) + (-1.352818e-03) * (q[i3] * q[i20]) + (-4.281597e-05) * (q[i3] * q[i21]) + (-7.913839e-04) * (q[i3] * q[i22])
            + (2.766852e-03) * (q[i4] * q[i5]) + (-6.211916e-04) * (q[i4] * q[i6]) + (5.575730e-03) * (q[i4] * q[i7]) + (-2.558086e-03) * (q[i4] * q[i8])
            + (-9.015293e-04) * (q[i4] * q[i9]) + (-4.478219e-04) * (q[i4] * q[i10]) + (-2.682264e-04) * (q[i4] * q[i11]) + (-1.597540e-03) * (q[i4] * q[i12])
            + (-3.911409e-04) * (q[i4] * q[i15]) + (2.809789e-04) * (q[i4] * q[i16]) + (-5.715602e-06) * (q[i4] * q[i19]) + (-6.549558e-04) * (q[i4] * q[i20])
            + (-1.410528e-04) * (q[i4] * q[i21]) + (-6.179272e-04) * (q[i4] * q[i22]) + (1.989546e-03) * (q[i5] * q[i6]) + (1.368311e-03) * (q[i5] * q[i7])
            + (1.635292e-04) * (q[i5] * q[i8]) + (2.272778e-03) * (q[i5] * q[i9]) + (-3.349013e-04) * (q[i5] * q[i10]) + (-2.742440e-03) * (q[i5] * q[i11])
            + (2.213568e-03) * (q[i5] * q[i12]) + (-1.032333e-03) * (q[i5] * q[i15]) + (-1.635496e-03) * (q[i5] * q[i16]) + (6.490070e-04) * (q[i5] * q[i19])
            + (-3.652971e-05) * (q[i5] * q[i20]) + (-4.205140e-04) * (q[i5] * q[i21]) + (6.642214e-04) * (q[i5] * q[i22]) + (1.017963e-02) * (q[i6] * q[i7])
            + (1.651949e-03) * (q[i6] * q[i8]) + (-5.399697e-04) * (q[i6] * q[i9]) + (1.844408e-03) * (q[i6] * q[i10]) + (-1.342392e-03) * (q[i6] * q[i11])
            + (-1.443070e-03) * (q[i6] * q[i12]) + (-3.174323e-04) * (q[i6] * q[i15]) + (1.159499e-03) * (q[i6] * q[i16]) + (-1.498094e-03) * (q[i6] * q[i19])
            + (7.182522e-04) * (q[i6] * q[i20]) + (5.736209e-04) * (q[i6] * q[i21]) + (-8.699801e-04) * (q[i6] * q[i22]) + (2.109935e-04) * (q[i7] * q[i8])
            + (1.644994e-03) * (q[i7] * q[i9]) + (5.517188e-04) * (q[i7] * q[i10]) + (8.710504e-05) * (q[i7] * q[i11]) + (2.285169e-04) * (q[i7] * q[i12])
            + (7.248336e-04) * (q[i7] * q[i15]) + (1.887704e-04) * (q[i7] * q[i16]) + (-1.286637e-03) * (q[i7] * q[i19]) + (3.037250e-04) * (q[i7] * q[i20])
            + (-4.422425e-04) * (q[i7] * q[i21]) + (-3.798863e-04) * (q[i7] * q[i22]) + (7.119842e-04) * (q[i8] * q[i9]) + (2.040536e-04) * (q[i8] * q[i10])
            + (-9.079045e-04) * (q[i8] * q[i11]) + (9.702281e-04) * (q[i8] * q[i12]) + (-7.466604e-04) * (q[i8] * q[i15]) + (2.160150e-03) * (q[i8] * q[i16])
            + (-3.694046e-04) * (q[i8] * q[i19]) + (-2.317022e-04) * (q[i8] * q[i20]) + (2.642182e-05) * (q[i8] * q[i21]) + (-4.572423e-04) * (q[i8] * q[i22])
            + (1.360263e-03) * (q[i9] * q[i10]) + (4.597602e-05) * (q[i9] * q[i11]) + (-1.002433e-03) * (q[i9] * q[i12]) + (1.017340e-04) * (q[i9] * q[i15])
            + (1.127961e-03) * (q[i9] * q[i16]) + (-1.799168e-04) * (q[i9] * q[i19]) + (5.617884e-04) * (q[i9] * q[i20]) + (-7.818176e-05) * (q[i9] * q[i21])
            + (-6.660250e-05) * (q[i9] * q[i22]) + (6.985196e-04) * (q[i10] * q[i11]) + (5.734440e-04) * (q[i10] * q[i12]) + (5.303826e-04) * (q[i10] * q[i15])
            + (-5.940487e-04) * (q[i10] * q[i16]) + (2.148813e-04) * (q[i10] * q[i19]) + (-4.319454e-04) * (q[i10] * q[i20])
            + (-1.584458e-04) * (q[i10] * q[i21]) + (-2.132184e-04) * (q[i10] * q[i22]) + (-5.393562e-04) * (q[i11] * q[i12])
            + (1.859815e-04) * (q[i11] * q[i15]) + (-7.142625e-05) * (q[i11] * q[i16]) + (5.389142e-04) * (q[i11] * q[i19]) + (4.192410e-04) * (q[i11] * q[i20])
            + (2.893217e-04) * (q[i11] * q[i21]) + (3.909004e-04) * (q[i11] * q[i22]) + (-2.571111e-05) * (q[i12] * q[i15])
            + (-5.853968e-05) * (q[i12] * q[i16]) + (-5.457165e-04) * (q[i12] * q[i19]) + (6.830868e-05) * (q[i12] * q[i20])
            + (-5.167213e-04) * (q[i12] * q[i21]) + (-3.713889e-04) * (q[i12] * q[i22]) + (-2.180080e-04) * (q[i15] * q[i16])
            + (-9.731131e-05) * (q[i15] * q[i19]) + (-1.329828e-04) * (q[i15] * q[i20]) + (3.266881e-04) * (q[i15] * q[i21])
            + (1.098482e-04) * (q[i15] * q[i22]) + (-5.863536e-04) * (q[i16] * q[i19]) + (1.775150e-04) * (q[i16] * q[i20])
            + (-2.073675e-04) * (q[i16] * q[i21]) + (1.769337e-04) * (q[i16] * q[i22]) + (3.099871e-04) * (q[i19] * q[i20]) + (2.827223e-04) * (q[i19] * q[i21])
            + (2.580421e-04) * (q[i19] * q[i22]) + (-1.022213e-04) * (q[i20] * q[i21]) + (-2.220662e-05) * (q[i20] * q[i22])
            + (-5.595093e-04) * (q[i21] * q[i22]);
   }

   public void getJQy1(double[] q, double[][] JQ)
   {
      JQ[2][i1] = (1.171536e-03) * (1) + (4.412114e-03) * ((2) * q[i1]) + (7.115423e-05) * (q[i0]) + (2.773411e-03) * (q[i2]) + (1.089383e-02) * (q[i3])
            + (-1.076574e-01) * (q[i4]) + (1.596455e-03) * (q[i5]) + (-6.207352e-04) * (q[i6]) + (-6.060779e-03) * (q[i7]) + (-1.238358e-04) * (q[i8])
            + (2.346357e-03) * (q[i9]) + (1.575809e-03) * (q[i10]) + (-6.405035e-04) * (q[i11]) + (-2.114922e-03) * (q[i12]) + (5.440769e-03) * (q[i15])
            + (-1.795247e-03) * (q[i16]) + (-8.235971e-04) * (q[i19]) + (1.904894e-03) * (q[i20]) + (-8.990263e-04) * (q[i21]) + (2.234868e-03) * (q[i22])
            + (4.217759e-04) * (q[i0] * q[i0]) + (-3.711844e-04) * ((2) * q[i0] * q[i1]) + (-2.713918e-03) * ((3) * q[i1] * q[i1])
            + (-2.176803e-03) * ((2) * q[i1] * q[i2]) + (1.966242e-03) * ((2) * q[i1] * q[i3]) + (2.900782e-03) * ((2) * q[i1] * q[i4])
            + (4.795869e-03) * ((2) * q[i1] * q[i5]) + (-2.025674e-03) * ((2) * q[i1] * q[i6]) + (-2.168669e-02) * ((2) * q[i1] * q[i7])
            + (1.522468e-03) * ((2) * q[i1] * q[i8]) + (-4.279621e-04) * ((2) * q[i1] * q[i9]) + (-3.573820e-03) * ((2) * q[i1] * q[i10])
            + (2.313198e-04) * ((2) * q[i1] * q[i11]) + (-1.247452e-03) * ((2) * q[i1] * q[i12]) + (-1.123010e-04) * ((2) * q[i1] * q[i15])
            + (7.653473e-04) * ((2) * q[i1] * q[i16]) + (7.919462e-04) * ((2) * q[i1] * q[i19]) + (9.471634e-05) * ((2) * q[i1] * q[i20])
            + (-2.343892e-04) * ((2) * q[i1] * q[i21]) + (-1.491548e-04) * ((2) * q[i1] * q[i22]) + (-1.279295e-03) * (q[i2] * q[i2])
            + (-2.959506e-03) * (q[i3] * q[i3]) + (-7.743154e-03) * (q[i4] * q[i4]) + (3.673930e-04) * (q[i5] * q[i5]) + (-1.108751e-03) * (q[i6] * q[i6])
            + (4.318104e-03) * (q[i7] * q[i7]) + (1.667796e-03) * (q[i8] * q[i8]) + (-5.281776e-04) * (q[i9] * q[i9]) + (-1.151267e-04) * (q[i10] * q[i10])
            + (-4.310596e-04) * (q[i11] * q[i11]) + (-2.801488e-04) * (q[i12] * q[i12]) + (7.123550e-04) * (q[i15] * q[i15])
            + (-4.658708e-05) * (q[i16] * q[i16]) + (-2.431014e-04) * (q[i19] * q[i19]) + (1.360576e-04) * (q[i20] * q[i20])
            + (-6.073279e-05) * (q[i21] * q[i21]) + (-1.891935e-04) * (q[i22] * q[i22]) + (2.041900e-05) * (q[i0] * q[i2]) + (-2.203863e-03) * (q[i0] * q[i3])
            + (2.176018e-03) * (q[i0] * q[i4]) + (-3.023284e-06) * (q[i0] * q[i5]) + (4.607573e-03) * (q[i0] * q[i6]) + (4.585382e-03) * (q[i0] * q[i7])
            + (3.606869e-03) * (q[i0] * q[i8]) + (3.022323e-04) * (q[i0] * q[i9]) + (2.846155e-04) * (q[i0] * q[i10]) + (-7.803878e-04) * (q[i0] * q[i11])
            + (7.788289e-04) * (q[i0] * q[i12]) + (-5.317338e-04) * (q[i0] * q[i15]) + (-5.364147e-04) * (q[i0] * q[i16]) + (-2.149220e-03) * (q[i0] * q[i19])
            + (-2.164970e-03) * (q[i0] * q[i20]) + (1.059114e-03) * (q[i0] * q[i21]) + (-1.058629e-03) * (q[i0] * q[i22]) + (3.625418e-04) * (q[i2] * q[i3])
            + (4.829084e-03) * (q[i2] * q[i4]) + (1.352036e-03) * (q[i2] * q[i5]) + (1.486211e-03) * (q[i2] * q[i6]) + (-8.826749e-03) * (q[i2] * q[i7])
            + (-2.254740e-03) * (q[i2] * q[i8]) + (2.449361e-03) * (q[i2] * q[i9]) + (-1.976517e-03) * (q[i2] * q[i10]) + (-8.432124e-04) * (q[i2] * q[i11])
            + (-6.395182e-04) * (q[i2] * q[i12]) + (1.760426e-04) * (q[i2] * q[i15]) + (1.001514e-04) * (q[i2] * q[i16]) + (-7.448692e-04) * (q[i2] * q[i19])
            + (-5.387324e-04) * (q[i2] * q[i20]) + (1.664055e-03) * (q[i2] * q[i21]) + (-1.179484e-03) * (q[i2] * q[i22]) + (-1.580378e-02) * (q[i3] * q[i4])
            + (-2.793112e-03) * (q[i3] * q[i5]) + (5.600800e-03) * (q[i3] * q[i6]) + (-6.311696e-04) * (q[i3] * q[i7]) + (-2.588719e-03) * (q[i3] * q[i8])
            + (-4.320600e-04) * (q[i3] * q[i9]) + (-8.867219e-04) * (q[i3] * q[i10]) + (1.609103e-03) * (q[i3] * q[i11]) + (2.617932e-04) * (q[i3] * q[i12])
            + (2.837263e-04) * (q[i3] * q[i15]) + (-3.677597e-04) * (q[i3] * q[i16]) + (-6.588636e-04) * (q[i3] * q[i19]) + (1.403780e-05) * (q[i3] * q[i20])
            + (6.214166e-04) * (q[i3] * q[i21]) + (1.400097e-04) * (q[i3] * q[i22]) + (-8.075424e-03) * (q[i4] * q[i5]) + (7.475188e-04) * (q[i4] * q[i6])
            + (-1.749236e-02) * (q[i4] * q[i7]) + (-6.287449e-04) * (q[i4] * q[i8]) + (-2.675081e-03) * (q[i4] * q[i9]) + (1.261796e-02) * (q[i4] * q[i10])
            + (1.852015e-03) * (q[i4] * q[i11]) + (-3.495028e-04) * (q[i4] * q[i12]) + (-4.364390e-03) * (q[i4] * q[i15]) + (-2.788492e-04) * (q[i4] * q[i16])
            + (-1.337410e-03) * (q[i4] * q[i19]) + (-1.289726e-03) * (q[i4] * q[i20]) + (7.876661e-04) * (q[i4] * q[i21]) + (4.832829e-05) * (q[i4] * q[i22])
            + (1.352922e-03) * (q[i5] * q[i6]) + (1.951753e-03) * (q[i5] * q[i7]) + (1.637739e-04) * (q[i5] * q[i8]) + (-3.407700e-04) * (q[i5] * q[i9])
            + (2.248592e-03) * (q[i5] * q[i10]) + (-2.247199e-03) * (q[i5] * q[i11]) + (2.677469e-03) * (q[i5] * q[i12]) + (-1.592579e-03) * (q[i5] * q[i15])
            + (-1.026977e-03) * (q[i5] * q[i16]) + (-5.729034e-05) * (q[i5] * q[i19]) + (6.226303e-04) * (q[i5] * q[i20]) + (-6.467642e-04) * (q[i5] * q[i21])
            + (4.217495e-04) * (q[i5] * q[i22]) + (-1.015857e-02) * (q[i6] * q[i7]) + (-2.377159e-04) * (q[i6] * q[i8]) + (-5.646921e-04) * (q[i6] * q[i9])
            + (-1.639768e-03) * (q[i6] * q[i10]) + (2.123385e-04) * (q[i6] * q[i11]) + (7.670935e-05) * (q[i6] * q[i12]) + (-1.923527e-04) * (q[i6] * q[i15])
            + (-7.395194e-04) * (q[i6] * q[i16]) + (-2.937084e-04) * (q[i6] * q[i19]) + (1.270226e-03) * (q[i6] * q[i20]) + (-3.794178e-04) * (q[i6] * q[i21])
            + (-4.370185e-04) * (q[i6] * q[i22]) + (-1.668724e-03) * (q[i7] * q[i8]) + (-1.866107e-03) * (q[i7] * q[i9]) + (5.542534e-04) * (q[i7] * q[i10])
            + (-1.426140e-03) * (q[i7] * q[i11]) + (-1.341927e-03) * (q[i7] * q[i12]) + (-1.151666e-03) * (q[i7] * q[i15]) + (3.387974e-04) * (q[i7] * q[i16])
            + (-7.119287e-04) * (q[i7] * q[i19]) + (1.484588e-03) * (q[i7] * q[i20]) + (-8.684813e-04) * (q[i7] * q[i21]) + (5.826807e-04) * (q[i7] * q[i22])
            + (-2.056903e-04) * (q[i8] * q[i9]) + (-7.043732e-04) * (q[i8] * q[i10]) + (1.002922e-03) * (q[i8] * q[i11]) + (-9.243151e-04) * (q[i8] * q[i12])
            + (-2.146160e-03) * (q[i8] * q[i15]) + (7.556071e-04) * (q[i8] * q[i16]) + (2.228837e-04) * (q[i8] * q[i19]) + (3.807387e-04) * (q[i8] * q[i20])
            + (-4.667108e-04) * (q[i8] * q[i21]) + (3.449386e-05) * (q[i8] * q[i22]) + (-1.360802e-03) * (q[i9] * q[i10]) + (5.642974e-04) * (q[i9] * q[i11])
            + (7.066151e-04) * (q[i9] * q[i12]) + (5.905837e-04) * (q[i9] * q[i15]) + (-5.296953e-04) * (q[i9] * q[i16]) + (4.403208e-04) * (q[i9] * q[i19])
            + (-2.157064e-04) * (q[i9] * q[i20]) + (-2.141346e-04) * (q[i9] * q[i21]) + (-1.572557e-04) * (q[i9] * q[i22]) + (-9.983918e-04) * (q[i10] * q[i11])
            + (3.945032e-05) * (q[i10] * q[i12]) + (-1.115424e-03) * (q[i10] * q[i15]) + (-9.232939e-05) * (q[i10] * q[i16])
            + (-5.613018e-04) * (q[i10] * q[i19]) + (1.733368e-04) * (q[i10] * q[i20]) + (-6.546301e-05) * (q[i10] * q[i21])
            + (-7.665282e-05) * (q[i10] * q[i22]) + (5.435814e-04) * (q[i11] * q[i12]) + (-6.513442e-05) * (q[i11] * q[i15])
            + (-2.026858e-05) * (q[i11] * q[i16]) + (6.352125e-05) * (q[i11] * q[i19]) + (-5.394431e-04) * (q[i11] * q[i20])
            + (3.606199e-04) * (q[i11] * q[i21]) + (5.179043e-04) * (q[i11] * q[i22]) + (-7.006303e-05) * (q[i12] * q[i15]) + (1.941888e-04) * (q[i12] * q[i16])
            + (4.266644e-04) * (q[i12] * q[i19]) + (5.463806e-04) * (q[i12] * q[i20]) + (-3.924730e-04) * (q[i12] * q[i21])
            + (-2.773281e-04) * (q[i12] * q[i22]) + (2.186881e-04) * (q[i15] * q[i16]) + (-1.743718e-04) * (q[i15] * q[i19])
            + (5.877674e-04) * (q[i15] * q[i20]) + (1.723671e-04) * (q[i15] * q[i21]) + (-2.020610e-04) * (q[i15] * q[i22]) + (1.262092e-04) * (q[i16] * q[i19])
            + (8.947500e-05) * (q[i16] * q[i20]) + (1.137587e-04) * (q[i16] * q[i21]) + (3.327511e-04) * (q[i16] * q[i22]) + (-3.021009e-04) * (q[i19] * q[i20])
            + (-2.477809e-05) * (q[i19] * q[i21]) + (-1.043606e-04) * (q[i19] * q[i22]) + (2.473619e-04) * (q[i20] * q[i21])
            + (2.811799e-04) * (q[i20] * q[i22]) + (5.531782e-04) * (q[i21] * q[i22]);
   }

   public void getJQy2(double[] q, double[][] JQ)
   {
      JQ[2][i2] = (1.371481e-04) * (1) + (3.642564e-04) * ((2) * q[i2]) + (2.760604e-03) * (q[i0]) + (2.773411e-03) * (q[i1]) + (-2.528163e-02) * (q[i3])
            + (-2.511238e-02) * (q[i4]) + (7.861535e-02) * (q[i5]) + (-9.848709e-05) * (q[i6]) + (9.317147e-05) * (q[i7]) + (-7.650917e-06) * (q[i8])
            + (1.460264e-03) * (q[i9]) + (-1.435032e-03) * (q[i10]) + (-2.575455e-03) * (q[i11]) + (-2.486851e-03) * (q[i12]) + (5.981859e-03) * (q[i15])
            + (-6.084513e-03) * (q[i16]) + (-2.725771e-03) * (q[i19]) + (2.709918e-03) * (q[i20]) + (1.988143e-03) * (q[i21]) + (1.965185e-03) * (q[i22])
            + (2.148421e-03) * (q[i0] * q[i0]) + (-2.176803e-03) * (q[i1] * q[i1]) + (1.270791e-03) * ((2) * q[i0] * q[i2])
            + (-1.279295e-03) * ((2) * q[i1] * q[i2]) + (-9.223137e-06) * ((3) * q[i2] * q[i2]) + (1.510165e-04) * ((2) * q[i2] * q[i3])
            + (-1.471769e-04) * ((2) * q[i2] * q[i4]) + (-1.566487e-05) * ((2) * q[i2] * q[i5]) + (2.143538e-05) * ((2) * q[i2] * q[i6])
            + (2.407642e-05) * ((2) * q[i2] * q[i7]) + (-3.070144e-02) * ((2) * q[i2] * q[i8]) + (-9.801745e-04) * ((2) * q[i2] * q[i9])
            + (-9.776732e-04) * ((2) * q[i2] * q[i10]) + (8.314828e-05) * ((2) * q[i2] * q[i11]) + (-1.286932e-04) * ((2) * q[i2] * q[i12])
            + (-2.236801e-04) * ((2) * q[i2] * q[i15]) + (-2.076048e-04) * ((2) * q[i2] * q[i16]) + (-6.277662e-04) * ((2) * q[i2] * q[i19])
            + (-6.472335e-04) * ((2) * q[i2] * q[i20]) + (7.020029e-04) * ((2) * q[i2] * q[i21]) + (-7.202685e-04) * ((2) * q[i2] * q[i22])
            + (1.746452e-03) * (q[i3] * q[i3]) + (-1.715255e-03) * (q[i4] * q[i4]) + (-3.329987e-05) * (q[i5] * q[i5]) + (-3.326144e-04) * (q[i6] * q[i6])
            + (3.392017e-04) * (q[i7] * q[i7]) + (2.976896e-05) * (q[i8] * q[i8]) + (7.392941e-04) * (q[i9] * q[i9]) + (-7.362072e-04) * (q[i10] * q[i10])
            + (1.572873e-03) * (q[i11] * q[i11]) + (-1.692362e-03) * (q[i12] * q[i12]) + (2.563287e-03) * (q[i15] * q[i15])
            + (-2.586976e-03) * (q[i16] * q[i16]) + (3.508930e-06) * (q[i19] * q[i19]) + (-2.837281e-06) * (q[i20] * q[i20])
            + (3.209924e-04) * (q[i21] * q[i21]) + (-3.183774e-04) * (q[i22] * q[i22]) + (2.041900e-05) * (q[i0] * q[i1]) + (-4.830283e-03) * (q[i0] * q[i3])
            + (-3.669630e-04) * (q[i0] * q[i4]) + (-1.346757e-03) * (q[i0] * q[i5]) + (-8.850569e-03) * (q[i0] * q[i6]) + (1.509462e-03) * (q[i0] * q[i7])
            + (-2.242384e-03) * (q[i0] * q[i8]) + (-1.992049e-03) * (q[i0] * q[i9]) + (2.438050e-03) * (q[i0] * q[i10]) + (6.523406e-04) * (q[i0] * q[i11])
            + (8.603355e-04) * (q[i0] * q[i12]) + (9.454998e-05) * (q[i0] * q[i15]) + (1.685530e-04) * (q[i0] * q[i16]) + (-5.308325e-04) * (q[i0] * q[i19])
            + (-7.462804e-04) * (q[i0] * q[i20]) + (1.166162e-03) * (q[i0] * q[i21]) + (-1.663098e-03) * (q[i0] * q[i22]) + (3.625418e-04) * (q[i1] * q[i3])
            + (4.829084e-03) * (q[i1] * q[i4]) + (1.352036e-03) * (q[i1] * q[i5]) + (1.486211e-03) * (q[i1] * q[i6]) + (-8.826749e-03) * (q[i1] * q[i7])
            + (-2.254740e-03) * (q[i1] * q[i8]) + (2.449361e-03) * (q[i1] * q[i9]) + (-1.976517e-03) * (q[i1] * q[i10]) + (-8.432124e-04) * (q[i1] * q[i11])
            + (-6.395182e-04) * (q[i1] * q[i12]) + (1.760426e-04) * (q[i1] * q[i15]) + (1.001514e-04) * (q[i1] * q[i16]) + (-7.448692e-04) * (q[i1] * q[i19])
            + (-5.387324e-04) * (q[i1] * q[i20]) + (1.664055e-03) * (q[i1] * q[i21]) + (-1.179484e-03) * (q[i1] * q[i22]) + (8.644762e-06) * (q[i3] * q[i4])
            + (8.699480e-03) * (q[i3] * q[i5]) + (-5.663162e-03) * (q[i3] * q[i6]) + (6.717905e-04) * (q[i3] * q[i7]) + (-3.571442e-04) * (q[i3] * q[i8])
            + (2.309691e-03) * (q[i3] * q[i9]) + (-2.988670e-04) * (q[i3] * q[i10]) + (7.361710e-04) * (q[i3] * q[i11]) + (5.600232e-04) * (q[i3] * q[i12])
            + (9.927942e-04) * (q[i3] * q[i15]) + (-2.515278e-03) * (q[i3] * q[i16]) + (1.276545e-04) * (q[i3] * q[i19]) + (6.979532e-05) * (q[i3] * q[i20])
            + (-6.013862e-04) * (q[i3] * q[i21]) + (6.002039e-04) * (q[i3] * q[i22]) + (-8.697755e-03) * (q[i4] * q[i5]) + (6.851293e-04) * (q[i4] * q[i6])
            + (-5.699442e-03) * (q[i4] * q[i7]) + (-3.418379e-04) * (q[i4] * q[i8]) + (-3.234076e-04) * (q[i4] * q[i9]) + (2.270634e-03) * (q[i4] * q[i10])
            + (-5.738576e-04) * (q[i4] * q[i11]) + (-7.830249e-04) * (q[i4] * q[i12]) + (-2.498836e-03) * (q[i4] * q[i15]) + (1.011946e-03) * (q[i4] * q[i16])
            + (8.152563e-05) * (q[i4] * q[i19]) + (1.180434e-04) * (q[i4] * q[i20]) + (-5.871365e-04) * (q[i4] * q[i21]) + (5.962369e-04) * (q[i4] * q[i22])
            + (3.143789e-04) * (q[i5] * q[i6]) + (3.189315e-04) * (q[i5] * q[i7]) + (-1.046169e-02) * (q[i5] * q[i8]) + (5.292999e-03) * (q[i5] * q[i9])
            + (5.223961e-03) * (q[i5] * q[i10]) + (1.171327e-03) * (q[i5] * q[i11]) + (-1.285937e-03) * (q[i5] * q[i12]) + (-6.544554e-03) * (q[i5] * q[i15])
            + (-6.624475e-03) * (q[i5] * q[i16]) + (1.439647e-04) * (q[i5] * q[i19]) + (1.530245e-04) * (q[i5] * q[i20]) + (2.058515e-03) * (q[i5] * q[i21])
            + (-2.067730e-03) * (q[i5] * q[i22]) + (-1.321266e-06) * (q[i6] * q[i7]) + (9.315958e-03) * (q[i6] * q[i8]) + (1.297394e-03) * (q[i6] * q[i9])
            + (-4.481669e-04) * (q[i6] * q[i10]) + (-7.785610e-04) * (q[i6] * q[i11]) + (4.258846e-04) * (q[i6] * q[i12]) + (-1.313394e-03) * (q[i6] * q[i15])
            + (4.201134e-04) * (q[i6] * q[i16]) + (-3.549272e-04) * (q[i6] * q[i19]) + (7.624220e-04) * (q[i6] * q[i20]) + (4.757560e-04) * (q[i6] * q[i21])
            + (-7.125799e-04) * (q[i6] * q[i22]) + (-9.328918e-03) * (q[i7] * q[i8]) + (4.524172e-04) * (q[i7] * q[i9]) + (-1.291531e-03) * (q[i7] * q[i10])
            + (4.293556e-04) * (q[i7] * q[i11]) + (-8.119320e-04) * (q[i7] * q[i12]) + (-4.246611e-04) * (q[i7] * q[i15]) + (1.335803e-03) * (q[i7] * q[i16])
            + (-7.598168e-04) * (q[i7] * q[i19]) + (3.408859e-04) * (q[i7] * q[i20]) + (-7.253175e-04) * (q[i7] * q[i21]) + (4.696598e-04) * (q[i7] * q[i22])
            + (2.149226e-03) * (q[i8] * q[i9]) + (-2.143734e-03) * (q[i8] * q[i10]) + (4.147071e-03) * (q[i8] * q[i11]) + (4.154181e-03) * (q[i8] * q[i12])
            + (-9.564607e-04) * (q[i8] * q[i15]) + (9.360359e-04) * (q[i8] * q[i16]) + (1.023466e-03) * (q[i8] * q[i19]) + (-1.027806e-03) * (q[i8] * q[i20])
            + (4.401218e-04) * (q[i8] * q[i21]) + (4.437775e-04) * (q[i8] * q[i22]) + (2.516672e-06) * (q[i9] * q[i10]) + (3.620506e-06) * (q[i9] * q[i11])
            + (-1.799722e-04) * (q[i9] * q[i12]) + (4.049757e-04) * (q[i9] * q[i15]) + (-5.789794e-05) * (q[i9] * q[i16]) + (-2.610005e-04) * (q[i9] * q[i19])
            + (9.740003e-05) * (q[i9] * q[i20]) + (-4.083199e-04) * (q[i9] * q[i21]) + (-3.429924e-04) * (q[i9] * q[i22]) + (-1.926262e-04) * (q[i10] * q[i11])
            + (1.820085e-06) * (q[i10] * q[i12]) + (6.286513e-05) * (q[i10] * q[i15]) + (-4.083522e-04) * (q[i10] * q[i16])
            + (-1.011537e-04) * (q[i10] * q[i19]) + (2.619118e-04) * (q[i10] * q[i20]) + (-3.418190e-04) * (q[i10] * q[i21])
            + (-4.035359e-04) * (q[i10] * q[i22]) + (6.764880e-06) * (q[i11] * q[i12]) + (-2.362006e-03) * (q[i11] * q[i15])
            + (-5.016997e-04) * (q[i11] * q[i16]) + (6.703634e-04) * (q[i11] * q[i19]) + (-4.443000e-04) * (q[i11] * q[i20])
            + (9.345787e-04) * (q[i11] * q[i21]) + (1.211605e-03) * (q[i11] * q[i22]) + (-5.111858e-04) * (q[i12] * q[i15])
            + (-2.336272e-03) * (q[i12] * q[i16]) + (-4.444471e-04) * (q[i12] * q[i19]) + (6.501908e-04) * (q[i12] * q[i20])
            + (-1.204995e-03) * (q[i12] * q[i21]) + (-9.273983e-04) * (q[i12] * q[i22]) + (-4.108729e-06) * (q[i15] * q[i16])
            + (-3.347044e-04) * (q[i15] * q[i19]) + (1.346695e-03) * (q[i15] * q[i20]) + (7.605788e-04) * (q[i15] * q[i21]) + (3.999834e-05) * (q[i15] * q[i22])
            + (-1.355870e-03) * (q[i16] * q[i19]) + (3.529748e-04) * (q[i16] * q[i20]) + (4.025737e-05) * (q[i16] * q[i21]) + (7.495992e-04) * (q[i16] * q[i22])
            + (9.810085e-06) * (q[i19] * q[i20]) + (-6.799983e-04) * (q[i19] * q[i21]) + (-8.525243e-05) * (q[i19] * q[i22])
            + (-9.238074e-05) * (q[i20] * q[i21]) + (-6.741689e-04) * (q[i20] * q[i22]) + (-1.328435e-06) * (q[i21] * q[i22]);
   }

   public void getJQy3(double[] q, double[][] JQ)
   {
      JQ[2][i3] = (-2.286804e-03) * (1) + (-2.063380e-04) * ((2) * q[i3]) + (-1.082041e-01) * (q[i0]) + (1.089383e-02) * (q[i1]) + (-2.528163e-02) * (q[i2])
            + (-4.914697e-03) * (q[i4]) + (5.148517e-04) * (q[i5]) + (-1.325262e-02) * (q[i6]) + (1.496705e-02) * (q[i7]) + (2.286157e-03) * (q[i8])
            + (-6.041502e-03) * (q[i9]) + (5.639904e-03) * (q[i10]) + (-4.951514e-03) * (q[i11]) + (-2.852589e-03) * (q[i12]) + (1.701516e-04) * (q[i15])
            + (1.983357e-03) * (q[i16]) + (-3.041974e-03) * (q[i19]) + (-1.632224e-03) * (q[i20]) + (-2.054358e-03) * (q[i21]) + (1.763675e-03) * (q[i22])
            + (-2.877567e-03) * (q[i0] * q[i0]) + (1.966242e-03) * (q[i1] * q[i1]) + (1.510165e-04) * (q[i2] * q[i2]) + (7.799925e-03) * ((2) * q[i0] * q[i3])
            + (-2.959506e-03) * ((2) * q[i1] * q[i3]) + (1.746452e-03) * ((2) * q[i2] * q[i3]) + (9.593527e-04) * ((3) * q[i3] * q[i3])
            + (1.781395e-03) * ((2) * q[i3] * q[i4]) + (6.591798e-04) * ((2) * q[i3] * q[i5]) + (1.200284e-02) * ((2) * q[i3] * q[i6])
            + (-1.176808e-03) * ((2) * q[i3] * q[i7]) + (3.685066e-03) * ((2) * q[i3] * q[i8]) + (2.280216e-03) * ((2) * q[i3] * q[i9])
            + (-9.699651e-04) * ((2) * q[i3] * q[i10]) + (2.476817e-04) * ((2) * q[i3] * q[i11]) + (7.545558e-04) * ((2) * q[i3] * q[i12])
            + (3.150266e-04) * ((2) * q[i3] * q[i15]) + (1.793124e-04) * ((2) * q[i3] * q[i16]) + (1.215654e-03) * ((2) * q[i3] * q[i19])
            + (-4.721584e-04) * ((2) * q[i3] * q[i20]) + (-7.272412e-04) * ((2) * q[i3] * q[i21]) + (-6.555301e-04) * ((2) * q[i3] * q[i22])
            + (-1.787267e-03) * (q[i4] * q[i4]) + (-1.147605e-03) * (q[i5] * q[i5]) + (-6.037617e-04) * (q[i6] * q[i6]) + (-8.564796e-04) * (q[i7] * q[i7])
            + (-5.313342e-04) * (q[i8] * q[i8]) + (1.189732e-03) * (q[i9] * q[i9]) + (-1.155973e-03) * (q[i10] * q[i10]) + (1.445009e-03) * (q[i11] * q[i11])
            + (6.728427e-04) * (q[i12] * q[i12]) + (-2.638679e-05) * (q[i15] * q[i15]) + (4.245840e-04) * (q[i16] * q[i16])
            + (-3.874298e-04) * (q[i19] * q[i19]) + (-1.862724e-04) * (q[i20] * q[i20]) + (-2.119539e-04) * (q[i21] * q[i21])
            + (-5.071852e-04) * (q[i22] * q[i22]) + (-2.203863e-03) * (q[i0] * q[i1]) + (-4.830283e-03) * (q[i0] * q[i2]) + (1.577433e-02) * (q[i0] * q[i4])
            + (8.064864e-03) * (q[i0] * q[i5]) + (-1.744554e-02) * (q[i0] * q[i6]) + (7.487355e-04) * (q[i0] * q[i7]) + (-6.385192e-04) * (q[i0] * q[i8])
            + (1.275901e-02) * (q[i0] * q[i9]) + (-2.628810e-03) * (q[i0] * q[i10]) + (3.415507e-04) * (q[i0] * q[i11]) + (-1.892927e-03) * (q[i0] * q[i12])
            + (-2.736937e-04) * (q[i0] * q[i15]) + (-4.429769e-03) * (q[i0] * q[i16]) + (-1.283668e-03) * (q[i0] * q[i19]) + (-1.352818e-03) * (q[i0] * q[i20])
            + (-4.281597e-05) * (q[i0] * q[i21]) + (-7.913839e-04) * (q[i0] * q[i22]) + (3.625418e-04) * (q[i1] * q[i2]) + (-1.580378e-02) * (q[i1] * q[i4])
            + (-2.793112e-03) * (q[i1] * q[i5]) + (5.600800e-03) * (q[i1] * q[i6]) + (-6.311696e-04) * (q[i1] * q[i7]) + (-2.588719e-03) * (q[i1] * q[i8])
            + (-4.320600e-04) * (q[i1] * q[i9]) + (-8.867219e-04) * (q[i1] * q[i10]) + (1.609103e-03) * (q[i1] * q[i11]) + (2.617932e-04) * (q[i1] * q[i12])
            + (2.837263e-04) * (q[i1] * q[i15]) + (-3.677597e-04) * (q[i1] * q[i16]) + (-6.588636e-04) * (q[i1] * q[i19]) + (1.403780e-05) * (q[i1] * q[i20])
            + (6.214166e-04) * (q[i1] * q[i21]) + (1.400097e-04) * (q[i1] * q[i22]) + (8.644762e-06) * (q[i2] * q[i4]) + (8.699480e-03) * (q[i2] * q[i5])
            + (-5.663162e-03) * (q[i2] * q[i6]) + (6.717905e-04) * (q[i2] * q[i7]) + (-3.571442e-04) * (q[i2] * q[i8]) + (2.309691e-03) * (q[i2] * q[i9])
            + (-2.988670e-04) * (q[i2] * q[i10]) + (7.361710e-04) * (q[i2] * q[i11]) + (5.600232e-04) * (q[i2] * q[i12]) + (9.927942e-04) * (q[i2] * q[i15])
            + (-2.515278e-03) * (q[i2] * q[i16]) + (1.276545e-04) * (q[i2] * q[i19]) + (6.979532e-05) * (q[i2] * q[i20]) + (-6.013862e-04) * (q[i2] * q[i21])
            + (6.002039e-04) * (q[i2] * q[i22]) + (-1.528038e-05) * (q[i4] * q[i5]) + (-4.692084e-03) * (q[i4] * q[i6]) + (-4.669405e-03) * (q[i4] * q[i7])
            + (-2.187009e-03) * (q[i4] * q[i8]) + (-1.861590e-03) * (q[i4] * q[i9]) + (-1.869920e-03) * (q[i4] * q[i10]) + (1.272945e-03) * (q[i4] * q[i11])
            + (-1.294124e-03) * (q[i4] * q[i12]) + (-3.552360e-04) * (q[i4] * q[i15]) + (-3.478787e-04) * (q[i4] * q[i16]) + (1.436047e-03) * (q[i4] * q[i19])
            + (1.413824e-03) * (q[i4] * q[i20]) + (-1.400512e-03) * (q[i4] * q[i21]) + (1.412147e-03) * (q[i4] * q[i22]) + (-9.070857e-03) * (q[i5] * q[i6])
            + (1.810798e-03) * (q[i5] * q[i7]) + (-3.838593e-03) * (q[i5] * q[i8]) + (-1.723064e-03) * (q[i5] * q[i9]) + (-5.797220e-04) * (q[i5] * q[i10])
            + (-1.448399e-03) * (q[i5] * q[i11]) + (-1.207768e-03) * (q[i5] * q[i12]) + (-1.703704e-03) * (q[i5] * q[i15]) + (9.957671e-04) * (q[i5] * q[i16])
            + (-1.447273e-03) * (q[i5] * q[i19]) + (2.859915e-04) * (q[i5] * q[i20]) + (-1.096375e-03) * (q[i5] * q[i21]) + (-2.078576e-03) * (q[i5] * q[i22])
            + (1.678006e-04) * (q[i6] * q[i7]) + (5.232857e-04) * (q[i6] * q[i8]) + (1.313509e-03) * (q[i6] * q[i9]) + (-8.018915e-04) * (q[i6] * q[i10])
            + (-2.051880e-03) * (q[i6] * q[i11]) + (-2.301127e-04) * (q[i6] * q[i12]) + (-6.109421e-04) * (q[i6] * q[i15]) + (1.322964e-03) * (q[i6] * q[i16])
            + (5.983288e-04) * (q[i6] * q[i19]) + (-5.325464e-04) * (q[i6] * q[i20]) + (-1.193203e-03) * (q[i6] * q[i21]) + (-2.469359e-04) * (q[i6] * q[i22])
            + (8.731107e-04) * (q[i7] * q[i8]) + (-1.450256e-03) * (q[i7] * q[i9]) + (-2.740255e-03) * (q[i7] * q[i10]) + (5.274222e-04) * (q[i7] * q[i11])
            + (2.305726e-04) * (q[i7] * q[i12]) + (1.450320e-03) * (q[i7] * q[i15]) + (-7.606796e-04) * (q[i7] * q[i16]) + (2.271741e-04) * (q[i7] * q[i19])
            + (1.320482e-04) * (q[i7] * q[i20]) + (-7.691139e-04) * (q[i7] * q[i21]) + (1.421559e-03) * (q[i7] * q[i22]) + (-1.037956e-03) * (q[i8] * q[i9])
            + (2.746779e-04) * (q[i8] * q[i10]) + (1.522556e-03) * (q[i8] * q[i11]) + (-3.970226e-05) * (q[i8] * q[i12]) + (-2.196074e-03) * (q[i8] * q[i15])
            + (1.745948e-03) * (q[i8] * q[i16]) + (-1.824331e-04) * (q[i8] * q[i19]) + (9.336729e-05) * (q[i8] * q[i20]) + (2.121110e-04) * (q[i8] * q[i21])
            + (4.277614e-04) * (q[i8] * q[i22]) + (-8.110467e-04) * (q[i9] * q[i10]) + (7.615759e-04) * (q[i9] * q[i11]) + (6.839198e-04) * (q[i9] * q[i12])
            + (-2.617285e-04) * (q[i9] * q[i15]) + (-7.010736e-04) * (q[i9] * q[i16]) + (3.544132e-04) * (q[i9] * q[i19]) + (-6.354895e-04) * (q[i9] * q[i20])
            + (-2.459064e-04) * (q[i9] * q[i21]) + (-3.224698e-04) * (q[i9] * q[i22]) + (-1.161962e-03) * (q[i10] * q[i11]) + (7.719139e-05) * (q[i10] * q[i12])
            + (-4.930728e-04) * (q[i10] * q[i15]) + (4.747305e-04) * (q[i10] * q[i16]) + (2.506987e-04) * (q[i10] * q[i19]) + (6.488983e-04) * (q[i10] * q[i20])
            + (1.378146e-05) * (q[i10] * q[i21]) + (2.411005e-04) * (q[i10] * q[i22]) + (-7.578032e-04) * (q[i11] * q[i12])
            + (-9.027708e-04) * (q[i11] * q[i15]) + (-3.870236e-05) * (q[i11] * q[i16]) + (-1.712573e-04) * (q[i11] * q[i19])
            + (-8.019534e-04) * (q[i11] * q[i20]) + (-8.460064e-04) * (q[i11] * q[i21]) + (4.242010e-04) * (q[i11] * q[i22])
            + (1.071918e-03) * (q[i12] * q[i15]) + (-4.166864e-04) * (q[i12] * q[i16]) + (8.904234e-05) * (q[i12] * q[i19])
            + (-1.091851e-04) * (q[i12] * q[i20]) + (-2.769233e-05) * (q[i12] * q[i21]) + (-1.330506e-04) * (q[i12] * q[i22])
            + (-4.291182e-04) * (q[i15] * q[i16]) + (-1.090363e-03) * (q[i15] * q[i19]) + (-3.773423e-04) * (q[i15] * q[i20])
            + (-7.025679e-04) * (q[i15] * q[i21]) + (-7.839491e-04) * (q[i15] * q[i22]) + (-4.696699e-04) * (q[i16] * q[i19])
            + (1.838969e-04) * (q[i16] * q[i20]) + (-4.494096e-05) * (q[i16] * q[i21]) + (2.849656e-04) * (q[i16] * q[i22]) + (7.589383e-04) * (q[i19] * q[i20])
            + (7.291109e-05) * (q[i19] * q[i21]) + (2.006097e-04) * (q[i19] * q[i22]) + (-7.413924e-04) * (q[i20] * q[i21]) + (8.525907e-04) * (q[i20] * q[i22])
            + (1.085187e-04) * (q[i21] * q[i22]);
   }

   public void getJQy4(double[] q, double[][] JQ)
   {
      JQ[2][i4] = (2.272458e-03) * (1) + (-2.102849e-04) * ((2) * q[i4]) + (1.091409e-02) * (q[i0]) + (-1.076574e-01) * (q[i1]) + (-2.511238e-02) * (q[i2])
            + (-4.914697e-03) * (q[i3]) + (4.679285e-04) * (q[i5]) + (-1.500024e-02) * (q[i6]) + (1.316982e-02) * (q[i7]) + (-2.209310e-03) * (q[i8])
            + (-5.699578e-03) * (q[i9]) + (5.987383e-03) * (q[i10]) + (-2.811232e-03) * (q[i11]) + (-4.944247e-03) * (q[i12]) + (-1.936538e-03) * (q[i15])
            + (-1.070322e-04) * (q[i16]) + (1.584654e-03) * (q[i19]) + (3.021591e-03) * (q[i20]) + (1.778881e-03) * (q[i21]) + (-2.025126e-03) * (q[i22])
            + (-1.957202e-03) * (q[i0] * q[i0]) + (2.900782e-03) * (q[i1] * q[i1]) + (-1.471769e-04) * (q[i2] * q[i2]) + (1.781395e-03) * (q[i3] * q[i3])
            + (2.965714e-03) * ((2) * q[i0] * q[i4]) + (-7.743154e-03) * ((2) * q[i1] * q[i4]) + (-1.715255e-03) * ((2) * q[i2] * q[i4])
            + (-1.787267e-03) * ((2) * q[i3] * q[i4]) + (-9.534190e-04) * ((3) * q[i4] * q[i4]) + (-6.716760e-04) * ((2) * q[i4] * q[i5])
            + (-1.198562e-03) * ((2) * q[i4] * q[i6]) + (1.202639e-02) * ((2) * q[i4] * q[i7]) + (3.701925e-03) * ((2) * q[i4] * q[i8])
            + (-9.772404e-04) * ((2) * q[i4] * q[i9]) + (2.273967e-03) * ((2) * q[i4] * q[i10]) + (-7.619221e-04) * ((2) * q[i4] * q[i11])
            + (-2.208289e-04) * ((2) * q[i4] * q[i12]) + (1.659287e-04) * ((2) * q[i4] * q[i15]) + (2.896900e-04) * ((2) * q[i4] * q[i16])
            + (-4.610605e-04) * ((2) * q[i4] * q[i19]) + (1.217230e-03) * ((2) * q[i4] * q[i20]) + (6.508448e-04) * ((2) * q[i4] * q[i21])
            + (7.148945e-04) * ((2) * q[i4] * q[i22]) + (1.138473e-03) * (q[i5] * q[i5]) + (8.606353e-04) * (q[i6] * q[i6]) + (6.288183e-04) * (q[i7] * q[i7])
            + (5.051829e-04) * (q[i8] * q[i8]) + (1.167596e-03) * (q[i9] * q[i9]) + (-1.170908e-03) * (q[i10] * q[i10]) + (-6.828332e-04) * (q[i11] * q[i11])
            + (-1.457032e-03) * (q[i12] * q[i12]) + (-4.245545e-04) * (q[i15] * q[i15]) + (4.041991e-05) * (q[i16] * q[i16])
            + (1.838183e-04) * (q[i19] * q[i19]) + (3.691705e-04) * (q[i20] * q[i20]) + (5.086029e-04) * (q[i21] * q[i21]) + (2.075443e-04) * (q[i22] * q[i22])
            + (2.176018e-03) * (q[i0] * q[i1]) + (-3.669630e-04) * (q[i0] * q[i2]) + (1.577433e-02) * (q[i0] * q[i3]) + (2.766852e-03) * (q[i0] * q[i5])
            + (-6.211916e-04) * (q[i0] * q[i6]) + (5.575730e-03) * (q[i0] * q[i7]) + (-2.558086e-03) * (q[i0] * q[i8]) + (-9.015293e-04) * (q[i0] * q[i9])
            + (-4.478219e-04) * (q[i0] * q[i10]) + (-2.682264e-04) * (q[i0] * q[i11]) + (-1.597540e-03) * (q[i0] * q[i12]) + (-3.911409e-04) * (q[i0] * q[i15])
            + (2.809789e-04) * (q[i0] * q[i16]) + (-5.715602e-06) * (q[i0] * q[i19]) + (-6.549558e-04) * (q[i0] * q[i20]) + (-1.410528e-04) * (q[i0] * q[i21])
            + (-6.179272e-04) * (q[i0] * q[i22]) + (4.829084e-03) * (q[i1] * q[i2]) + (-1.580378e-02) * (q[i1] * q[i3]) + (-8.075424e-03) * (q[i1] * q[i5])
            + (7.475188e-04) * (q[i1] * q[i6]) + (-1.749236e-02) * (q[i1] * q[i7]) + (-6.287449e-04) * (q[i1] * q[i8]) + (-2.675081e-03) * (q[i1] * q[i9])
            + (1.261796e-02) * (q[i1] * q[i10]) + (1.852015e-03) * (q[i1] * q[i11]) + (-3.495028e-04) * (q[i1] * q[i12]) + (-4.364390e-03) * (q[i1] * q[i15])
            + (-2.788492e-04) * (q[i1] * q[i16]) + (-1.337410e-03) * (q[i1] * q[i19]) + (-1.289726e-03) * (q[i1] * q[i20]) + (7.876661e-04) * (q[i1] * q[i21])
            + (4.832829e-05) * (q[i1] * q[i22]) + (8.644762e-06) * (q[i2] * q[i3]) + (-8.697755e-03) * (q[i2] * q[i5]) + (6.851293e-04) * (q[i2] * q[i6])
            + (-5.699442e-03) * (q[i2] * q[i7]) + (-3.418379e-04) * (q[i2] * q[i8]) + (-3.234076e-04) * (q[i2] * q[i9]) + (2.270634e-03) * (q[i2] * q[i10])
            + (-5.738576e-04) * (q[i2] * q[i11]) + (-7.830249e-04) * (q[i2] * q[i12]) + (-2.498836e-03) * (q[i2] * q[i15]) + (1.011946e-03) * (q[i2] * q[i16])
            + (8.152563e-05) * (q[i2] * q[i19]) + (1.180434e-04) * (q[i2] * q[i20]) + (-5.871365e-04) * (q[i2] * q[i21]) + (5.962369e-04) * (q[i2] * q[i22])
            + (-1.528038e-05) * (q[i3] * q[i5]) + (-4.692084e-03) * (q[i3] * q[i6]) + (-4.669405e-03) * (q[i3] * q[i7]) + (-2.187009e-03) * (q[i3] * q[i8])
            + (-1.861590e-03) * (q[i3] * q[i9]) + (-1.869920e-03) * (q[i3] * q[i10]) + (1.272945e-03) * (q[i3] * q[i11]) + (-1.294124e-03) * (q[i3] * q[i12])
            + (-3.552360e-04) * (q[i3] * q[i15]) + (-3.478787e-04) * (q[i3] * q[i16]) + (1.436047e-03) * (q[i3] * q[i19]) + (1.413824e-03) * (q[i3] * q[i20])
            + (-1.400512e-03) * (q[i3] * q[i21]) + (1.412147e-03) * (q[i3] * q[i22]) + (1.822253e-03) * (q[i5] * q[i6]) + (-9.067011e-03) * (q[i5] * q[i7])
            + (-3.841741e-03) * (q[i5] * q[i8]) + (-5.934737e-04) * (q[i5] * q[i9]) + (-1.696389e-03) * (q[i5] * q[i10]) + (1.201051e-03) * (q[i5] * q[i11])
            + (1.420428e-03) * (q[i5] * q[i12]) + (9.956928e-04) * (q[i5] * q[i15]) + (-1.694699e-03) * (q[i5] * q[i16]) + (2.752440e-04) * (q[i5] * q[i19])
            + (-1.445958e-03) * (q[i5] * q[i20]) + (2.063944e-03) * (q[i5] * q[i21]) + (1.112460e-03) * (q[i5] * q[i22]) + (-1.860803e-04) * (q[i6] * q[i7])
            + (-8.465525e-04) * (q[i6] * q[i8]) + (2.745881e-03) * (q[i6] * q[i9]) + (1.436831e-03) * (q[i6] * q[i10]) + (2.222197e-04) * (q[i6] * q[i11])
            + (5.484947e-04) * (q[i6] * q[i12]) + (7.600604e-04) * (q[i6] * q[i15]) + (-1.475898e-03) * (q[i6] * q[i16]) + (-1.344769e-04) * (q[i6] * q[i19])
            + (-2.211296e-04) * (q[i6] * q[i20]) + (1.419563e-03) * (q[i6] * q[i21]) + (-7.808593e-04) * (q[i6] * q[i22]) + (-5.403478e-04) * (q[i7] * q[i8])
            + (8.076992e-04) * (q[i7] * q[i9]) + (-1.282671e-03) * (q[i7] * q[i10]) + (-2.309113e-04) * (q[i7] * q[i11]) + (-2.075818e-03) * (q[i7] * q[i12])
            + (-1.302155e-03) * (q[i7] * q[i15]) + (6.214028e-04) * (q[i7] * q[i16]) + (5.267683e-04) * (q[i7] * q[i19]) + (-6.071990e-04) * (q[i7] * q[i20])
            + (-2.688400e-04) * (q[i7] * q[i21]) + (-1.188431e-03) * (q[i7] * q[i22]) + (-2.740939e-04) * (q[i8] * q[i9]) + (1.022948e-03) * (q[i8] * q[i10])
            + (-3.970672e-05) * (q[i8] * q[i11]) + (1.552861e-03) * (q[i8] * q[i12]) + (-1.735568e-03) * (q[i8] * q[i15]) + (2.215468e-03) * (q[i8] * q[i16])
            + (-9.381795e-05) * (q[i8] * q[i19]) + (1.747690e-04) * (q[i8] * q[i20]) + (4.350437e-04) * (q[i8] * q[i21]) + (2.177448e-04) * (q[i8] * q[i22])
            + (8.112286e-04) * (q[i9] * q[i10]) + (7.421642e-05) * (q[i9] * q[i11]) + (-1.158998e-03) * (q[i9] * q[i12]) + (-4.751135e-04) * (q[i9] * q[i15])
            + (4.904249e-04) * (q[i9] * q[i16]) + (-6.228559e-04) * (q[i9] * q[i19]) + (-2.543523e-04) * (q[i9] * q[i20]) + (2.289478e-04) * (q[i9] * q[i21])
            + (-4.351189e-08) * (q[i9] * q[i22]) + (6.648302e-04) * (q[i10] * q[i11]) + (7.633476e-04) * (q[i10] * q[i12]) + (6.980060e-04) * (q[i10] * q[i15])
            + (2.553292e-04) * (q[i10] * q[i16]) + (6.283218e-04) * (q[i10] * q[i19]) + (-3.471171e-04) * (q[i10] * q[i20])
            + (-3.230970e-04) * (q[i10] * q[i21]) + (-2.538713e-04) * (q[i10] * q[i22]) + (7.642985e-04) * (q[i11] * q[i12])
            + (-4.090085e-04) * (q[i11] * q[i15]) + (1.070191e-03) * (q[i11] * q[i16]) + (-1.119821e-04) * (q[i11] * q[i19])
            + (9.979069e-05) * (q[i11] * q[i20]) + (1.227838e-04) * (q[i11] * q[i21]) + (2.703891e-05) * (q[i11] * q[i22]) + (-2.736292e-05) * (q[i12] * q[i15])
            + (-9.062632e-04) * (q[i12] * q[i16]) + (-7.938514e-04) * (q[i12] * q[i19]) + (-1.668851e-04) * (q[i12] * q[i20])
            + (-4.133740e-04) * (q[i12] * q[i21]) + (8.567602e-04) * (q[i12] * q[i22]) + (4.269459e-04) * (q[i15] * q[i16])
            + (-1.936850e-04) * (q[i15] * q[i19]) + (4.801148e-04) * (q[i15] * q[i20]) + (2.930438e-04) * (q[i15] * q[i21])
            + (-5.318361e-05) * (q[i15] * q[i22]) + (3.716815e-04) * (q[i16] * q[i19]) + (1.078294e-03) * (q[i16] * q[i20])
            + (-7.797041e-04) * (q[i16] * q[i21]) + (-7.098311e-04) * (q[i16] * q[i22]) + (-7.567033e-04) * (q[i19] * q[i20])
            + (8.571854e-04) * (q[i19] * q[i21]) + (-7.366768e-04) * (q[i19] * q[i22]) + (1.948996e-04) * (q[i20] * q[i21]) + (7.178784e-05) * (q[i20] * q[i22])
            + (-1.024263e-04) * (q[i21] * q[i22]);
   }

   public void getJQy5(double[] q, double[][] JQ)
   {
      JQ[2][i5] = (-5.235205e-05) * (1) + (-2.271242e-03) * ((2) * q[i5]) + (1.464916e-03) * (q[i0]) + (1.596455e-03) * (q[i1]) + (7.861535e-02) * (q[i2])
            + (5.148517e-04) * (q[i3]) + (4.679285e-04) * (q[i4]) + (-1.586590e-02) * (q[i6]) + (1.586884e-02) * (q[i7]) + (-4.203808e-05) * (q[i8])
            + (-5.528472e-03) * (q[i9]) + (5.516264e-03) * (q[i10]) + (-4.741971e-03) * (q[i11]) + (-4.693143e-03) * (q[i12]) + (1.072417e-03) * (q[i15])
            + (-1.156109e-03) * (q[i16]) + (-6.688310e-04) * (q[i19]) + (6.817503e-04) * (q[i20]) + (1.007175e-03) * (q[i21]) + (1.007538e-03) * (q[i22])
            + (-4.799782e-03) * (q[i0] * q[i0]) + (4.795869e-03) * (q[i1] * q[i1]) + (-1.566487e-05) * (q[i2] * q[i2]) + (6.591798e-04) * (q[i3] * q[i3])
            + (-6.716760e-04) * (q[i4] * q[i4]) + (-3.637586e-04) * ((2) * q[i0] * q[i5]) + (3.673930e-04) * ((2) * q[i1] * q[i5])
            + (-3.329987e-05) * ((2) * q[i2] * q[i5]) + (-1.147605e-03) * ((2) * q[i3] * q[i5]) + (1.138473e-03) * ((2) * q[i4] * q[i5])
            + (1.971547e-05) * ((3) * q[i5] * q[i5]) + (1.523657e-03) * ((2) * q[i5] * q[i6]) + (1.525435e-03) * ((2) * q[i5] * q[i7])
            + (2.662022e-03) * ((2) * q[i5] * q[i8]) + (-2.407482e-04) * ((2) * q[i5] * q[i9]) + (-2.454196e-04) * ((2) * q[i5] * q[i10])
            + (6.884126e-05) * ((2) * q[i5] * q[i11]) + (-9.857491e-05) * ((2) * q[i5] * q[i12]) + (-2.111881e-03) * ((2) * q[i5] * q[i15])
            + (-2.132539e-03) * ((2) * q[i5] * q[i16]) + (-1.718998e-03) * ((2) * q[i5] * q[i19]) + (-1.704766e-03) * ((2) * q[i5] * q[i20])
            + (-1.473637e-03) * ((2) * q[i5] * q[i21]) + (1.495075e-03) * ((2) * q[i5] * q[i22]) + (-3.680953e-03) * (q[i6] * q[i6])
            + (3.688227e-03) * (q[i7] * q[i7]) + (-1.978580e-07) * (q[i8] * q[i8]) + (8.578853e-04) * (q[i9] * q[i9]) + (-8.626645e-04) * (q[i10] * q[i10])
            + (3.701298e-04) * (q[i11] * q[i11]) + (-4.291887e-04) * (q[i12] * q[i12]) + (-5.541777e-04) * (q[i15] * q[i15])
            + (5.385013e-04) * (q[i16] * q[i16]) + (-8.537189e-04) * (q[i19] * q[i19]) + (8.604417e-04) * (q[i20] * q[i20])
            + (-4.984050e-04) * (q[i21] * q[i21]) + (5.071164e-04) * (q[i22] * q[i22]) + (-3.023284e-06) * (q[i0] * q[i1]) + (-1.346757e-03) * (q[i0] * q[i2])
            + (8.064864e-03) * (q[i0] * q[i3]) + (2.766852e-03) * (q[i0] * q[i4]) + (1.989546e-03) * (q[i0] * q[i6]) + (1.368311e-03) * (q[i0] * q[i7])
            + (1.635292e-04) * (q[i0] * q[i8]) + (2.272778e-03) * (q[i0] * q[i9]) + (-3.349013e-04) * (q[i0] * q[i10]) + (-2.742440e-03) * (q[i0] * q[i11])
            + (2.213568e-03) * (q[i0] * q[i12]) + (-1.032333e-03) * (q[i0] * q[i15]) + (-1.635496e-03) * (q[i0] * q[i16]) + (6.490070e-04) * (q[i0] * q[i19])
            + (-3.652971e-05) * (q[i0] * q[i20]) + (-4.205140e-04) * (q[i0] * q[i21]) + (6.642214e-04) * (q[i0] * q[i22]) + (1.352036e-03) * (q[i1] * q[i2])
            + (-2.793112e-03) * (q[i1] * q[i3]) + (-8.075424e-03) * (q[i1] * q[i4]) + (1.352922e-03) * (q[i1] * q[i6]) + (1.951753e-03) * (q[i1] * q[i7])
            + (1.637739e-04) * (q[i1] * q[i8]) + (-3.407700e-04) * (q[i1] * q[i9]) + (2.248592e-03) * (q[i1] * q[i10]) + (-2.247199e-03) * (q[i1] * q[i11])
            + (2.677469e-03) * (q[i1] * q[i12]) + (-1.592579e-03) * (q[i1] * q[i15]) + (-1.026977e-03) * (q[i1] * q[i16]) + (-5.729034e-05) * (q[i1] * q[i19])
            + (6.226303e-04) * (q[i1] * q[i20]) + (-6.467642e-04) * (q[i1] * q[i21]) + (4.217495e-04) * (q[i1] * q[i22]) + (8.699480e-03) * (q[i2] * q[i3])
            + (-8.697755e-03) * (q[i2] * q[i4]) + (3.143789e-04) * (q[i2] * q[i6]) + (3.189315e-04) * (q[i2] * q[i7]) + (-1.046169e-02) * (q[i2] * q[i8])
            + (5.292999e-03) * (q[i2] * q[i9]) + (5.223961e-03) * (q[i2] * q[i10]) + (1.171327e-03) * (q[i2] * q[i11]) + (-1.285937e-03) * (q[i2] * q[i12])
            + (-6.544554e-03) * (q[i2] * q[i15]) + (-6.624475e-03) * (q[i2] * q[i16]) + (1.439647e-04) * (q[i2] * q[i19]) + (1.530245e-04) * (q[i2] * q[i20])
            + (2.058515e-03) * (q[i2] * q[i21]) + (-2.067730e-03) * (q[i2] * q[i22]) + (-1.528038e-05) * (q[i3] * q[i4]) + (-9.070857e-03) * (q[i3] * q[i6])
            + (1.810798e-03) * (q[i3] * q[i7]) + (-3.838593e-03) * (q[i3] * q[i8]) + (-1.723064e-03) * (q[i3] * q[i9]) + (-5.797220e-04) * (q[i3] * q[i10])
            + (-1.448399e-03) * (q[i3] * q[i11]) + (-1.207768e-03) * (q[i3] * q[i12]) + (-1.703704e-03) * (q[i3] * q[i15]) + (9.957671e-04) * (q[i3] * q[i16])
            + (-1.447273e-03) * (q[i3] * q[i19]) + (2.859915e-04) * (q[i3] * q[i20]) + (-1.096375e-03) * (q[i3] * q[i21]) + (-2.078576e-03) * (q[i3] * q[i22])
            + (1.822253e-03) * (q[i4] * q[i6]) + (-9.067011e-03) * (q[i4] * q[i7]) + (-3.841741e-03) * (q[i4] * q[i8]) + (-5.934737e-04) * (q[i4] * q[i9])
            + (-1.696389e-03) * (q[i4] * q[i10]) + (1.201051e-03) * (q[i4] * q[i11]) + (1.420428e-03) * (q[i4] * q[i12]) + (9.956928e-04) * (q[i4] * q[i15])
            + (-1.694699e-03) * (q[i4] * q[i16]) + (2.752440e-04) * (q[i4] * q[i19]) + (-1.445958e-03) * (q[i4] * q[i20]) + (2.063944e-03) * (q[i4] * q[i21])
            + (1.112460e-03) * (q[i4] * q[i22]) + (1.324561e-05) * (q[i6] * q[i7]) + (1.535462e-03) * (q[i6] * q[i8]) + (2.186654e-03) * (q[i6] * q[i9])
            + (-2.477935e-03) * (q[i6] * q[i10]) + (4.159708e-04) * (q[i6] * q[i11]) + (-1.840572e-03) * (q[i6] * q[i12]) + (6.391077e-04) * (q[i6] * q[i15])
            + (-3.613898e-04) * (q[i6] * q[i16]) + (-9.828295e-04) * (q[i6] * q[i19]) + (-2.404216e-04) * (q[i6] * q[i20]) + (-2.500503e-04) * (q[i6] * q[i21])
            + (2.925324e-04) * (q[i6] * q[i22]) + (-1.558072e-03) * (q[i7] * q[i8]) + (2.477884e-03) * (q[i7] * q[i9]) + (-2.164833e-03) * (q[i7] * q[i10])
            + (-1.864377e-03) * (q[i7] * q[i11]) + (4.200824e-04) * (q[i7] * q[i12]) + (3.664296e-04) * (q[i7] * q[i15]) + (-6.492493e-04) * (q[i7] * q[i16])
            + (2.210279e-04) * (q[i7] * q[i19]) + (9.826121e-04) * (q[i7] * q[i20]) + (2.803336e-04) * (q[i7] * q[i21]) + (-2.304339e-04) * (q[i7] * q[i22])
            + (2.484928e-03) * (q[i8] * q[i9]) + (-2.482312e-03) * (q[i8] * q[i10]) + (2.648866e-03) * (q[i8] * q[i11]) + (2.660963e-03) * (q[i8] * q[i12])
            + (2.435700e-03) * (q[i8] * q[i15]) + (-2.476418e-03) * (q[i8] * q[i16]) + (1.505927e-04) * (q[i8] * q[i19]) + (-1.445753e-04) * (q[i8] * q[i20])
            + (-5.209222e-05) * (q[i8] * q[i21]) + (-4.250972e-05) * (q[i8] * q[i22]) + (-2.280379e-06) * (q[i9] * q[i10]) + (-9.996856e-04) * (q[i9] * q[i11])
            + (2.664558e-04) * (q[i9] * q[i12]) + (4.065399e-04) * (q[i9] * q[i15]) + (-8.526925e-04) * (q[i9] * q[i16]) + (6.817273e-04) * (q[i9] * q[i19])
            + (2.758747e-04) * (q[i9] * q[i20]) + (-5.875879e-04) * (q[i9] * q[i21]) + (-2.189059e-04) * (q[i9] * q[i22]) + (2.569866e-04) * (q[i10] * q[i11])
            + (-9.864429e-04) * (q[i10] * q[i12]) + (8.599808e-04) * (q[i10] * q[i15]) + (-4.068580e-04) * (q[i10] * q[i16])
            + (-2.698364e-04) * (q[i10] * q[i19]) + (-7.060816e-04) * (q[i10] * q[i20]) + (-2.235920e-04) * (q[i10] * q[i21])
            + (-5.826465e-04) * (q[i10] * q[i22]) + (7.873494e-06) * (q[i11] * q[i12]) + (-2.545878e-03) * (q[i11] * q[i15])
            + (-3.355602e-04) * (q[i11] * q[i16]) + (-3.721245e-04) * (q[i11] * q[i19]) + (3.566828e-04) * (q[i11] * q[i20])
            + (-5.596208e-04) * (q[i11] * q[i21]) + (-2.649668e-04) * (q[i11] * q[i22]) + (-3.395477e-04) * (q[i12] * q[i15])
            + (-2.540585e-03) * (q[i12] * q[i16]) + (3.623418e-04) * (q[i12] * q[i19]) + (-3.867339e-04) * (q[i12] * q[i20])
            + (2.657216e-04) * (q[i12] * q[i21]) + (5.715204e-04) * (q[i12] * q[i22]) + (-1.584383e-06) * (q[i15] * q[i16])
            + (-4.229998e-04) * (q[i15] * q[i19]) + (-4.258683e-04) * (q[i15] * q[i20]) + (6.511432e-04) * (q[i15] * q[i21])
            + (4.829936e-04) * (q[i15] * q[i22]) + (4.420311e-04) * (q[i16] * q[i19]) + (4.188070e-04) * (q[i16] * q[i20]) + (4.852346e-04) * (q[i16] * q[i21])
            + (6.692534e-04) * (q[i16] * q[i22]) + (2.482564e-06) * (q[i19] * q[i20]) + (6.058628e-04) * (q[i19] * q[i21]) + (9.127125e-05) * (q[i19] * q[i22])
            + (9.610038e-05) * (q[i20] * q[i21]) + (6.030247e-04) * (q[i20] * q[i22]) + (-2.869781e-06) * (q[i21] * q[i22]);
   }

   public void getJQy6(double[] q, double[][] JQ)
   {
      JQ[2][i6] = (1.286961e-01) * (1) + (-1.276062e-03) * ((2) * q[i6]) + (6.008783e-03) * (q[i0]) + (-6.207352e-04) * (q[i1]) + (-9.848709e-05) * (q[i2])
            + (-1.325262e-02) * (q[i3]) + (-1.500024e-02) * (q[i4]) + (-1.586590e-02) * (q[i5]) + (-3.116376e-03) * (q[i7]) + (9.608001e-04) * (q[i8])
            + (-7.158658e-03) * (q[i9]) + (5.731393e-03) * (q[i10]) + (2.074065e-03) * (q[i11]) + (9.387117e-04) * (q[i12]) + (2.506067e-03) * (q[i15])
            + (6.238223e-03) * (q[i16]) + (-5.073938e-04) * (q[i19]) + (-9.342910e-04) * (q[i20]) + (-2.525261e-04) * (q[i21]) + (3.042374e-04) * (q[i22])
            + (-2.172771e-02) * (q[i0] * q[i0]) + (-2.025674e-03) * (q[i1] * q[i1]) + (2.143538e-05) * (q[i2] * q[i2]) + (1.200284e-02) * (q[i3] * q[i3])
            + (-1.198562e-03) * (q[i4] * q[i4]) + (1.523657e-03) * (q[i5] * q[i5]) + (-4.331391e-03) * ((2) * q[i0] * q[i6])
            + (-1.108751e-03) * ((2) * q[i1] * q[i6]) + (-3.326144e-04) * ((2) * q[i2] * q[i6]) + (-6.037617e-04) * ((2) * q[i3] * q[i6])
            + (8.606353e-04) * ((2) * q[i4] * q[i6]) + (-3.680953e-03) * ((2) * q[i5] * q[i6]) + (-2.277140e-03) * ((3) * q[i6] * q[i6])
            + (-8.911033e-04) * ((2) * q[i6] * q[i7]) + (3.097585e-03) * ((2) * q[i6] * q[i8]) + (-6.738246e-04) * ((2) * q[i6] * q[i9])
            + (-4.169451e-04) * ((2) * q[i6] * q[i10]) + (-2.863740e-04) * ((2) * q[i6] * q[i11]) + (3.608378e-04) * ((2) * q[i6] * q[i12])
            + (-5.963555e-04) * ((2) * q[i6] * q[i15]) + (-4.876350e-04) * ((2) * q[i6] * q[i16]) + (-1.054806e-03) * ((2) * q[i6] * q[i19])
            + (-5.433964e-05) * ((2) * q[i6] * q[i20]) + (-6.000541e-04) * ((2) * q[i6] * q[i21]) + (1.043639e-03) * ((2) * q[i6] * q[i22])
            + (-8.986901e-04) * (q[i7] * q[i7]) + (1.549358e-03) * (q[i8] * q[i8]) + (-3.082180e-03) * (q[i9] * q[i9]) + (-1.009958e-04) * (q[i10] * q[i10])
            + (-8.192230e-04) * (q[i11] * q[i11]) + (1.147318e-05) * (q[i12] * q[i12]) + (4.019567e-05) * (q[i15] * q[i15]) + (8.658253e-04) * (q[i16] * q[i16])
            + (-6.359441e-04) * (q[i19] * q[i19]) + (3.957655e-04) * (q[i20] * q[i20]) + (1.102790e-04) * (q[i21] * q[i21]) + (6.934395e-04) * (q[i22] * q[i22])
            + (4.607573e-03) * (q[i0] * q[i1]) + (-8.850569e-03) * (q[i0] * q[i2]) + (-1.744554e-02) * (q[i0] * q[i3]) + (-6.211916e-04) * (q[i0] * q[i4])
            + (1.989546e-03) * (q[i0] * q[i5]) + (1.017963e-02) * (q[i0] * q[i7]) + (1.651949e-03) * (q[i0] * q[i8]) + (-5.399697e-04) * (q[i0] * q[i9])
            + (1.844408e-03) * (q[i0] * q[i10]) + (-1.342392e-03) * (q[i0] * q[i11]) + (-1.443070e-03) * (q[i0] * q[i12]) + (-3.174323e-04) * (q[i0] * q[i15])
            + (1.159499e-03) * (q[i0] * q[i16]) + (-1.498094e-03) * (q[i0] * q[i19]) + (7.182522e-04) * (q[i0] * q[i20]) + (5.736209e-04) * (q[i0] * q[i21])
            + (-8.699801e-04) * (q[i0] * q[i22]) + (1.486211e-03) * (q[i1] * q[i2]) + (5.600800e-03) * (q[i1] * q[i3]) + (7.475188e-04) * (q[i1] * q[i4])
            + (1.352922e-03) * (q[i1] * q[i5]) + (-1.015857e-02) * (q[i1] * q[i7]) + (-2.377159e-04) * (q[i1] * q[i8]) + (-5.646921e-04) * (q[i1] * q[i9])
            + (-1.639768e-03) * (q[i1] * q[i10]) + (2.123385e-04) * (q[i1] * q[i11]) + (7.670935e-05) * (q[i1] * q[i12]) + (-1.923527e-04) * (q[i1] * q[i15])
            + (-7.395194e-04) * (q[i1] * q[i16]) + (-2.937084e-04) * (q[i1] * q[i19]) + (1.270226e-03) * (q[i1] * q[i20]) + (-3.794178e-04) * (q[i1] * q[i21])
            + (-4.370185e-04) * (q[i1] * q[i22]) + (-5.663162e-03) * (q[i2] * q[i3]) + (6.851293e-04) * (q[i2] * q[i4]) + (3.143789e-04) * (q[i2] * q[i5])
            + (-1.321266e-06) * (q[i2] * q[i7]) + (9.315958e-03) * (q[i2] * q[i8]) + (1.297394e-03) * (q[i2] * q[i9]) + (-4.481669e-04) * (q[i2] * q[i10])
            + (-7.785610e-04) * (q[i2] * q[i11]) + (4.258846e-04) * (q[i2] * q[i12]) + (-1.313394e-03) * (q[i2] * q[i15]) + (4.201134e-04) * (q[i2] * q[i16])
            + (-3.549272e-04) * (q[i2] * q[i19]) + (7.624220e-04) * (q[i2] * q[i20]) + (4.757560e-04) * (q[i2] * q[i21]) + (-7.125799e-04) * (q[i2] * q[i22])
            + (-4.692084e-03) * (q[i3] * q[i4]) + (-9.070857e-03) * (q[i3] * q[i5]) + (1.678006e-04) * (q[i3] * q[i7]) + (5.232857e-04) * (q[i3] * q[i8])
            + (1.313509e-03) * (q[i3] * q[i9]) + (-8.018915e-04) * (q[i3] * q[i10]) + (-2.051880e-03) * (q[i3] * q[i11]) + (-2.301127e-04) * (q[i3] * q[i12])
            + (-6.109421e-04) * (q[i3] * q[i15]) + (1.322964e-03) * (q[i3] * q[i16]) + (5.983288e-04) * (q[i3] * q[i19]) + (-5.325464e-04) * (q[i3] * q[i20])
            + (-1.193203e-03) * (q[i3] * q[i21]) + (-2.469359e-04) * (q[i3] * q[i22]) + (1.822253e-03) * (q[i4] * q[i5]) + (-1.860803e-04) * (q[i4] * q[i7])
            + (-8.465525e-04) * (q[i4] * q[i8]) + (2.745881e-03) * (q[i4] * q[i9]) + (1.436831e-03) * (q[i4] * q[i10]) + (2.222197e-04) * (q[i4] * q[i11])
            + (5.484947e-04) * (q[i4] * q[i12]) + (7.600604e-04) * (q[i4] * q[i15]) + (-1.475898e-03) * (q[i4] * q[i16]) + (-1.344769e-04) * (q[i4] * q[i19])
            + (-2.211296e-04) * (q[i4] * q[i20]) + (1.419563e-03) * (q[i4] * q[i21]) + (-7.808593e-04) * (q[i4] * q[i22]) + (1.324561e-05) * (q[i5] * q[i7])
            + (1.535462e-03) * (q[i5] * q[i8]) + (2.186654e-03) * (q[i5] * q[i9]) + (-2.477935e-03) * (q[i5] * q[i10]) + (4.159708e-04) * (q[i5] * q[i11])
            + (-1.840572e-03) * (q[i5] * q[i12]) + (6.391077e-04) * (q[i5] * q[i15]) + (-3.613898e-04) * (q[i5] * q[i16]) + (-9.828295e-04) * (q[i5] * q[i19])
            + (-2.404216e-04) * (q[i5] * q[i20]) + (-2.500503e-04) * (q[i5] * q[i21]) + (2.925324e-04) * (q[i5] * q[i22]) + (-7.091101e-03) * (q[i7] * q[i8])
            + (5.066186e-04) * (q[i7] * q[i9]) + (5.131238e-04) * (q[i7] * q[i10]) + (1.122814e-03) * (q[i7] * q[i11]) + (-1.119746e-03) * (q[i7] * q[i12])
            + (6.383422e-04) * (q[i7] * q[i15]) + (6.339042e-04) * (q[i7] * q[i16]) + (2.586580e-04) * (q[i7] * q[i19]) + (2.497187e-04) * (q[i7] * q[i20])
            + (-2.658031e-04) * (q[i7] * q[i21]) + (2.470120e-04) * (q[i7] * q[i22]) + (-3.254459e-04) * (q[i8] * q[i9]) + (-1.566180e-03) * (q[i8] * q[i10])
            + (-8.828370e-04) * (q[i8] * q[i11]) + (-1.065433e-03) * (q[i8] * q[i12]) + (7.782932e-04) * (q[i8] * q[i15]) + (-7.840902e-04) * (q[i8] * q[i16])
            + (-5.448643e-05) * (q[i8] * q[i19]) + (-2.203541e-04) * (q[i8] * q[i20]) + (-4.420825e-04) * (q[i8] * q[i21]) + (-1.667029e-04) * (q[i8] * q[i22])
            + (-3.222617e-04) * (q[i9] * q[i10]) + (3.018579e-04) * (q[i9] * q[i11]) + (3.321788e-04) * (q[i9] * q[i12]) + (-4.425578e-04) * (q[i9] * q[i15])
            + (-8.358086e-05) * (q[i9] * q[i16]) + (1.934901e-04) * (q[i9] * q[i19]) + (-2.956798e-04) * (q[i9] * q[i20]) + (-2.630103e-04) * (q[i9] * q[i21])
            + (-2.043316e-04) * (q[i9] * q[i22]) + (3.046552e-05) * (q[i10] * q[i11]) + (-5.482079e-04) * (q[i10] * q[i12]) + (7.382468e-06) * (q[i10] * q[i15])
            + (2.798608e-04) * (q[i10] * q[i16]) + (-1.757855e-05) * (q[i10] * q[i19]) + (4.630654e-04) * (q[i10] * q[i20])
            + (-3.136931e-04) * (q[i10] * q[i21]) + (-3.502427e-06) * (q[i10] * q[i22]) + (-6.285653e-05) * (q[i11] * q[i12])
            + (4.187876e-04) * (q[i11] * q[i15]) + (3.723862e-04) * (q[i11] * q[i16]) + (-1.640755e-04) * (q[i11] * q[i19])
            + (-2.434679e-04) * (q[i11] * q[i20]) + (-2.957810e-04) * (q[i11] * q[i21]) + (-6.079118e-04) * (q[i11] * q[i22])
            + (3.061818e-05) * (q[i12] * q[i15]) + (-5.408422e-04) * (q[i12] * q[i16]) + (-9.819785e-04) * (q[i12] * q[i19])
            + (8.291197e-05) * (q[i12] * q[i20]) + (6.580356e-04) * (q[i12] * q[i21]) + (3.425389e-07) * (q[i12] * q[i22]) + (-2.138078e-05) * (q[i15] * q[i16])
            + (-6.097696e-05) * (q[i15] * q[i19]) + (-4.208301e-04) * (q[i15] * q[i20]) + (-3.006825e-04) * (q[i15] * q[i21])
            + (6.121058e-05) * (q[i15] * q[i22]) + (1.010537e-03) * (q[i16] * q[i19]) + (-2.576356e-04) * (q[i16] * q[i20]) + (1.806325e-04) * (q[i16] * q[i21])
            + (-7.857355e-04) * (q[i16] * q[i22]) + (-3.891080e-04) * (q[i19] * q[i20]) + (3.961905e-04) * (q[i19] * q[i21])
            + (-1.491139e-04) * (q[i19] * q[i22]) + (1.049546e-04) * (q[i20] * q[i21]) + (2.197945e-04) * (q[i20] * q[i22])
            + (3.274916e-04) * (q[i21] * q[i22]);
   }

   public void getJQy7(double[] q, double[][] JQ)
   {
      JQ[2][i7] = (1.279249e-01) * (1) + (-1.230130e-03) * ((2) * q[i7]) + (5.937328e-04) * (q[i0]) + (-6.060779e-03) * (q[i1]) + (9.317147e-05) * (q[i2])
            + (1.496705e-02) * (q[i3]) + (1.316982e-02) * (q[i4]) + (1.586884e-02) * (q[i5]) + (-3.116376e-03) * (q[i6]) + (9.322740e-04) * (q[i8])
            + (5.781920e-03) * (q[i9]) + (-7.079973e-03) * (q[i10]) + (-9.094966e-04) * (q[i11]) + (-2.033062e-03) * (q[i12]) + (6.173416e-03) * (q[i15])
            + (2.507167e-03) * (q[i16]) + (-9.229792e-04) * (q[i19]) + (-4.789234e-04) * (q[i20]) + (-3.041800e-04) * (q[i21]) + (2.464662e-04) * (q[i22])
            + (-2.006883e-03) * (q[i0] * q[i0]) + (-2.168669e-02) * (q[i1] * q[i1]) + (2.407642e-05) * (q[i2] * q[i2]) + (-1.176808e-03) * (q[i3] * q[i3])
            + (1.202639e-02) * (q[i4] * q[i4]) + (1.525435e-03) * (q[i5] * q[i5]) + (-8.911033e-04) * (q[i6] * q[i6]) + (1.095570e-03) * ((2) * q[i0] * q[i7])
            + (4.318104e-03) * ((2) * q[i1] * q[i7]) + (3.392017e-04) * ((2) * q[i2] * q[i7]) + (-8.564796e-04) * ((2) * q[i3] * q[i7])
            + (6.288183e-04) * ((2) * q[i4] * q[i7]) + (3.688227e-03) * ((2) * q[i5] * q[i7]) + (-8.986901e-04) * ((2) * q[i6] * q[i7])
            + (-2.277373e-03) * ((3) * q[i7] * q[i7]) + (3.125767e-03) * ((2) * q[i7] * q[i8]) + (-4.165958e-04) * ((2) * q[i7] * q[i9])
            + (-6.674327e-04) * ((2) * q[i7] * q[i10]) + (-3.531736e-04) * ((2) * q[i7] * q[i11]) + (3.045578e-04) * ((2) * q[i7] * q[i12])
            + (-4.750761e-04) * ((2) * q[i7] * q[i15]) + (-5.980053e-04) * ((2) * q[i7] * q[i16]) + (-4.924212e-05) * ((2) * q[i7] * q[i19])
            + (-1.049889e-03) * ((2) * q[i7] * q[i20]) + (-1.035763e-03) * ((2) * q[i7] * q[i21]) + (5.882127e-04) * ((2) * q[i7] * q[i22])
            + (1.578614e-03) * (q[i8] * q[i8]) + (-9.292043e-05) * (q[i9] * q[i9]) + (-3.045815e-03) * (q[i10] * q[i10]) + (5.161338e-06) * (q[i11] * q[i11])
            + (-8.323333e-04) * (q[i12] * q[i12]) + (8.646733e-04) * (q[i15] * q[i15]) + (3.910849e-05) * (q[i16] * q[i16]) + (3.946263e-04) * (q[i19] * q[i19])
            + (-6.289636e-04) * (q[i20] * q[i20]) + (6.938603e-04) * (q[i21] * q[i21]) + (1.140094e-04) * (q[i22] * q[i22]) + (4.585382e-03) * (q[i0] * q[i1])
            + (1.509462e-03) * (q[i0] * q[i2]) + (7.487355e-04) * (q[i0] * q[i3]) + (5.575730e-03) * (q[i0] * q[i4]) + (1.368311e-03) * (q[i0] * q[i5])
            + (1.017963e-02) * (q[i0] * q[i6]) + (2.109935e-04) * (q[i0] * q[i8]) + (1.644994e-03) * (q[i0] * q[i9]) + (5.517188e-04) * (q[i0] * q[i10])
            + (8.710504e-05) * (q[i0] * q[i11]) + (2.285169e-04) * (q[i0] * q[i12]) + (7.248336e-04) * (q[i0] * q[i15]) + (1.887704e-04) * (q[i0] * q[i16])
            + (-1.286637e-03) * (q[i0] * q[i19]) + (3.037250e-04) * (q[i0] * q[i20]) + (-4.422425e-04) * (q[i0] * q[i21]) + (-3.798863e-04) * (q[i0] * q[i22])
            + (-8.826749e-03) * (q[i1] * q[i2]) + (-6.311696e-04) * (q[i1] * q[i3]) + (-1.749236e-02) * (q[i1] * q[i4]) + (1.951753e-03) * (q[i1] * q[i5])
            + (-1.015857e-02) * (q[i1] * q[i6]) + (-1.668724e-03) * (q[i1] * q[i8]) + (-1.866107e-03) * (q[i1] * q[i9]) + (5.542534e-04) * (q[i1] * q[i10])
            + (-1.426140e-03) * (q[i1] * q[i11]) + (-1.341927e-03) * (q[i1] * q[i12]) + (-1.151666e-03) * (q[i1] * q[i15]) + (3.387974e-04) * (q[i1] * q[i16])
            + (-7.119287e-04) * (q[i1] * q[i19]) + (1.484588e-03) * (q[i1] * q[i20]) + (-8.684813e-04) * (q[i1] * q[i21]) + (5.826807e-04) * (q[i1] * q[i22])
            + (6.717905e-04) * (q[i2] * q[i3]) + (-5.699442e-03) * (q[i2] * q[i4]) + (3.189315e-04) * (q[i2] * q[i5]) + (-1.321266e-06) * (q[i2] * q[i6])
            + (-9.328918e-03) * (q[i2] * q[i8]) + (4.524172e-04) * (q[i2] * q[i9]) + (-1.291531e-03) * (q[i2] * q[i10]) + (4.293556e-04) * (q[i2] * q[i11])
            + (-8.119320e-04) * (q[i2] * q[i12]) + (-4.246611e-04) * (q[i2] * q[i15]) + (1.335803e-03) * (q[i2] * q[i16]) + (-7.598168e-04) * (q[i2] * q[i19])
            + (3.408859e-04) * (q[i2] * q[i20]) + (-7.253175e-04) * (q[i2] * q[i21]) + (4.696598e-04) * (q[i2] * q[i22]) + (-4.669405e-03) * (q[i3] * q[i4])
            + (1.810798e-03) * (q[i3] * q[i5]) + (1.678006e-04) * (q[i3] * q[i6]) + (8.731107e-04) * (q[i3] * q[i8]) + (-1.450256e-03) * (q[i3] * q[i9])
            + (-2.740255e-03) * (q[i3] * q[i10]) + (5.274222e-04) * (q[i3] * q[i11]) + (2.305726e-04) * (q[i3] * q[i12]) + (1.450320e-03) * (q[i3] * q[i15])
            + (-7.606796e-04) * (q[i3] * q[i16]) + (2.271741e-04) * (q[i3] * q[i19]) + (1.320482e-04) * (q[i3] * q[i20]) + (-7.691139e-04) * (q[i3] * q[i21])
            + (1.421559e-03) * (q[i3] * q[i22]) + (-9.067011e-03) * (q[i4] * q[i5]) + (-1.860803e-04) * (q[i4] * q[i6]) + (-5.403478e-04) * (q[i4] * q[i8])
            + (8.076992e-04) * (q[i4] * q[i9]) + (-1.282671e-03) * (q[i4] * q[i10]) + (-2.309113e-04) * (q[i4] * q[i11]) + (-2.075818e-03) * (q[i4] * q[i12])
            + (-1.302155e-03) * (q[i4] * q[i15]) + (6.214028e-04) * (q[i4] * q[i16]) + (5.267683e-04) * (q[i4] * q[i19]) + (-6.071990e-04) * (q[i4] * q[i20])
            + (-2.688400e-04) * (q[i4] * q[i21]) + (-1.188431e-03) * (q[i4] * q[i22]) + (1.324561e-05) * (q[i5] * q[i6]) + (-1.558072e-03) * (q[i5] * q[i8])
            + (2.477884e-03) * (q[i5] * q[i9]) + (-2.164833e-03) * (q[i5] * q[i10]) + (-1.864377e-03) * (q[i5] * q[i11]) + (4.200824e-04) * (q[i5] * q[i12])
            + (3.664296e-04) * (q[i5] * q[i15]) + (-6.492493e-04) * (q[i5] * q[i16]) + (2.210279e-04) * (q[i5] * q[i19]) + (9.826121e-04) * (q[i5] * q[i20])
            + (2.803336e-04) * (q[i5] * q[i21]) + (-2.304339e-04) * (q[i5] * q[i22]) + (-7.091101e-03) * (q[i6] * q[i8]) + (5.066186e-04) * (q[i6] * q[i9])
            + (5.131238e-04) * (q[i6] * q[i10]) + (1.122814e-03) * (q[i6] * q[i11]) + (-1.119746e-03) * (q[i6] * q[i12]) + (6.383422e-04) * (q[i6] * q[i15])
            + (6.339042e-04) * (q[i6] * q[i16]) + (2.586580e-04) * (q[i6] * q[i19]) + (2.497187e-04) * (q[i6] * q[i20]) + (-2.658031e-04) * (q[i6] * q[i21])
            + (2.470120e-04) * (q[i6] * q[i22]) + (-1.582883e-03) * (q[i8] * q[i9]) + (-3.117434e-04) * (q[i8] * q[i10]) + (1.097291e-03) * (q[i8] * q[i11])
            + (8.750085e-04) * (q[i8] * q[i12]) + (-7.901275e-04) * (q[i8] * q[i15]) + (7.769723e-04) * (q[i8] * q[i16]) + (-2.197780e-04) * (q[i8] * q[i19])
            + (-3.898825e-05) * (q[i8] * q[i20]) + (1.665584e-04) * (q[i8] * q[i21]) + (4.488181e-04) * (q[i8] * q[i22]) + (-3.334738e-04) * (q[i9] * q[i10])
            + (5.548080e-04) * (q[i9] * q[i11]) + (-2.256889e-05) * (q[i9] * q[i12]) + (2.860588e-04) * (q[i9] * q[i15]) + (1.170306e-05) * (q[i9] * q[i16])
            + (4.661810e-04) * (q[i9] * q[i19]) + (-2.378912e-05) * (q[i9] * q[i20]) + (2.457164e-06) * (q[i9] * q[i21]) + (3.049735e-04) * (q[i9] * q[i22])
            + (-3.371485e-04) * (q[i10] * q[i11]) + (-3.108714e-04) * (q[i10] * q[i12]) + (-7.809335e-05) * (q[i10] * q[i15])
            + (-4.451027e-04) * (q[i10] * q[i16]) + (-2.948753e-04) * (q[i10] * q[i19]) + (1.864975e-04) * (q[i10] * q[i20])
            + (2.096182e-04) * (q[i10] * q[i21]) + (2.627882e-04) * (q[i10] * q[i22]) + (-8.108071e-05) * (q[i11] * q[i12]) + (5.273374e-04) * (q[i11] * q[i15])
            + (-3.163262e-05) * (q[i11] * q[i16]) + (-7.524544e-05) * (q[i11] * q[i19]) + (9.860693e-04) * (q[i11] * q[i20])
            + (-1.315688e-06) * (q[i11] * q[i21]) + (6.556906e-04) * (q[i11] * q[i22]) + (-3.739448e-04) * (q[i12] * q[i15])
            + (-4.156540e-04) * (q[i12] * q[i16]) + (2.504454e-04) * (q[i12] * q[i19]) + (1.631307e-04) * (q[i12] * q[i20])
            + (-6.068143e-04) * (q[i12] * q[i21]) + (-2.887615e-04) * (q[i12] * q[i22]) + (-2.752398e-05) * (q[i15] * q[i16])
            + (-2.515684e-04) * (q[i15] * q[i19]) + (1.008449e-03) * (q[i15] * q[i20]) + (7.773756e-04) * (q[i15] * q[i21])
            + (-1.900134e-04) * (q[i15] * q[i22]) + (-4.187241e-04) * (q[i16] * q[i19]) + (-4.536671e-05) * (q[i16] * q[i20])
            + (-4.937533e-05) * (q[i16] * q[i21]) + (3.086174e-04) * (q[i16] * q[i22]) + (-3.844589e-04) * (q[i19] * q[i20])
            + (-2.231712e-04) * (q[i19] * q[i21]) + (-1.014804e-04) * (q[i19] * q[i22]) + (1.437196e-04) * (q[i20] * q[i21])
            + (-3.856598e-04) * (q[i20] * q[i22]) + (3.294441e-04) * (q[i21] * q[i22]);
   }

   public void getJQy8(double[] q, double[][] JQ)
   {
      JQ[2][i8] = (1.062151e-01) * (1) + (-2.514359e-03) * ((2) * q[i8]) + (1.223880e-04) * (q[i0]) + (-1.238358e-04) * (q[i1]) + (-7.650917e-06) * (q[i2])
            + (2.286157e-03) * (q[i3]) + (-2.209310e-03) * (q[i4]) + (-4.203808e-05) * (q[i5]) + (9.608001e-04) * (q[i6]) + (9.322740e-04) * (q[i7])
            + (2.419856e-03) * (q[i9]) + (2.390816e-03) * (q[i10]) + (-2.250525e-03) * (q[i11]) + (2.137495e-03) * (q[i12]) + (-9.332737e-03) * (q[i15])
            + (-9.383865e-03) * (q[i16]) + (-2.190352e-03) * (q[i19]) + (-2.185457e-03) * (q[i20]) + (-2.085760e-03) * (q[i21]) + (2.088335e-03) * (q[i22])
            + (1.524819e-03) * (q[i0] * q[i0]) + (1.522468e-03) * (q[i1] * q[i1]) + (-3.070144e-02) * (q[i2] * q[i2]) + (3.685066e-03) * (q[i3] * q[i3])
            + (3.701925e-03) * (q[i4] * q[i4]) + (2.662022e-03) * (q[i5] * q[i5]) + (3.097585e-03) * (q[i6] * q[i6]) + (3.125767e-03) * (q[i7] * q[i7])
            + (-1.652844e-03) * ((2) * q[i0] * q[i8]) + (1.667796e-03) * ((2) * q[i1] * q[i8]) + (2.976896e-05) * ((2) * q[i2] * q[i8])
            + (-5.313342e-04) * ((2) * q[i3] * q[i8]) + (5.051829e-04) * ((2) * q[i4] * q[i8]) + (-1.978580e-07) * ((2) * q[i5] * q[i8])
            + (1.549358e-03) * ((2) * q[i6] * q[i8]) + (1.578614e-03) * ((2) * q[i7] * q[i8]) + (-3.481256e-03) * ((3) * q[i8] * q[i8])
            + (8.673059e-04) * ((2) * q[i8] * q[i9]) + (8.654743e-04) * ((2) * q[i8] * q[i10]) + (7.407764e-04) * ((2) * q[i8] * q[i11])
            + (-7.625813e-04) * ((2) * q[i8] * q[i12]) + (4.076422e-05) * ((2) * q[i8] * q[i15]) + (5.069054e-05) * ((2) * q[i8] * q[i16])
            + (-1.430933e-05) * ((2) * q[i8] * q[i19]) + (-2.542223e-05) * ((2) * q[i8] * q[i20]) + (-5.663762e-04) * ((2) * q[i8] * q[i21])
            + (5.723610e-04) * ((2) * q[i8] * q[i22]) + (1.081172e-03) * (q[i9] * q[i9]) + (1.069369e-03) * (q[i10] * q[i10])
            + (1.676774e-03) * (q[i11] * q[i11]) + (1.706114e-03) * (q[i12] * q[i12]) + (6.044488e-05) * (q[i15] * q[i15]) + (7.343281e-05) * (q[i16] * q[i16])
            + (-4.900689e-04) * (q[i19] * q[i19]) + (-4.881255e-04) * (q[i20] * q[i20]) + (-7.177874e-04) * (q[i21] * q[i21])
            + (-7.183218e-04) * (q[i22] * q[i22]) + (3.606869e-03) * (q[i0] * q[i1]) + (-2.242384e-03) * (q[i0] * q[i2]) + (-6.385192e-04) * (q[i0] * q[i3])
            + (-2.558086e-03) * (q[i0] * q[i4]) + (1.635292e-04) * (q[i0] * q[i5]) + (1.651949e-03) * (q[i0] * q[i6]) + (2.109935e-04) * (q[i0] * q[i7])
            + (7.119842e-04) * (q[i0] * q[i9]) + (2.040536e-04) * (q[i0] * q[i10]) + (-9.079045e-04) * (q[i0] * q[i11]) + (9.702281e-04) * (q[i0] * q[i12])
            + (-7.466604e-04) * (q[i0] * q[i15]) + (2.160150e-03) * (q[i0] * q[i16]) + (-3.694046e-04) * (q[i0] * q[i19]) + (-2.317022e-04) * (q[i0] * q[i20])
            + (2.642182e-05) * (q[i0] * q[i21]) + (-4.572423e-04) * (q[i0] * q[i22]) + (-2.254740e-03) * (q[i1] * q[i2]) + (-2.588719e-03) * (q[i1] * q[i3])
            + (-6.287449e-04) * (q[i1] * q[i4]) + (1.637739e-04) * (q[i1] * q[i5]) + (-2.377159e-04) * (q[i1] * q[i6]) + (-1.668724e-03) * (q[i1] * q[i7])
            + (-2.056903e-04) * (q[i1] * q[i9]) + (-7.043732e-04) * (q[i1] * q[i10]) + (1.002922e-03) * (q[i1] * q[i11]) + (-9.243151e-04) * (q[i1] * q[i12])
            + (-2.146160e-03) * (q[i1] * q[i15]) + (7.556071e-04) * (q[i1] * q[i16]) + (2.228837e-04) * (q[i1] * q[i19]) + (3.807387e-04) * (q[i1] * q[i20])
            + (-4.667108e-04) * (q[i1] * q[i21]) + (3.449386e-05) * (q[i1] * q[i22]) + (-3.571442e-04) * (q[i2] * q[i3]) + (-3.418379e-04) * (q[i2] * q[i4])
            + (-1.046169e-02) * (q[i2] * q[i5]) + (9.315958e-03) * (q[i2] * q[i6]) + (-9.328918e-03) * (q[i2] * q[i7]) + (2.149226e-03) * (q[i2] * q[i9])
            + (-2.143734e-03) * (q[i2] * q[i10]) + (4.147071e-03) * (q[i2] * q[i11]) + (4.154181e-03) * (q[i2] * q[i12]) + (-9.564607e-04) * (q[i2] * q[i15])
            + (9.360359e-04) * (q[i2] * q[i16]) + (1.023466e-03) * (q[i2] * q[i19]) + (-1.027806e-03) * (q[i2] * q[i20]) + (4.401218e-04) * (q[i2] * q[i21])
            + (4.437775e-04) * (q[i2] * q[i22]) + (-2.187009e-03) * (q[i3] * q[i4]) + (-3.838593e-03) * (q[i3] * q[i5]) + (5.232857e-04) * (q[i3] * q[i6])
            + (8.731107e-04) * (q[i3] * q[i7]) + (-1.037956e-03) * (q[i3] * q[i9]) + (2.746779e-04) * (q[i3] * q[i10]) + (1.522556e-03) * (q[i3] * q[i11])
            + (-3.970226e-05) * (q[i3] * q[i12]) + (-2.196074e-03) * (q[i3] * q[i15]) + (1.745948e-03) * (q[i3] * q[i16]) + (-1.824331e-04) * (q[i3] * q[i19])
            + (9.336729e-05) * (q[i3] * q[i20]) + (2.121110e-04) * (q[i3] * q[i21]) + (4.277614e-04) * (q[i3] * q[i22]) + (-3.841741e-03) * (q[i4] * q[i5])
            + (-8.465525e-04) * (q[i4] * q[i6]) + (-5.403478e-04) * (q[i4] * q[i7]) + (-2.740939e-04) * (q[i4] * q[i9]) + (1.022948e-03) * (q[i4] * q[i10])
            + (-3.970672e-05) * (q[i4] * q[i11]) + (1.552861e-03) * (q[i4] * q[i12]) + (-1.735568e-03) * (q[i4] * q[i15]) + (2.215468e-03) * (q[i4] * q[i16])
            + (-9.381795e-05) * (q[i4] * q[i19]) + (1.747690e-04) * (q[i4] * q[i20]) + (4.350437e-04) * (q[i4] * q[i21]) + (2.177448e-04) * (q[i4] * q[i22])
            + (1.535462e-03) * (q[i5] * q[i6]) + (-1.558072e-03) * (q[i5] * q[i7]) + (2.484928e-03) * (q[i5] * q[i9]) + (-2.482312e-03) * (q[i5] * q[i10])
            + (2.648866e-03) * (q[i5] * q[i11]) + (2.660963e-03) * (q[i5] * q[i12]) + (2.435700e-03) * (q[i5] * q[i15]) + (-2.476418e-03) * (q[i5] * q[i16])
            + (1.505927e-04) * (q[i5] * q[i19]) + (-1.445753e-04) * (q[i5] * q[i20]) + (-5.209222e-05) * (q[i5] * q[i21]) + (-4.250972e-05) * (q[i5] * q[i22])
            + (-7.091101e-03) * (q[i6] * q[i7]) + (-3.254459e-04) * (q[i6] * q[i9]) + (-1.566180e-03) * (q[i6] * q[i10]) + (-8.828370e-04) * (q[i6] * q[i11])
            + (-1.065433e-03) * (q[i6] * q[i12]) + (7.782932e-04) * (q[i6] * q[i15]) + (-7.840902e-04) * (q[i6] * q[i16]) + (-5.448643e-05) * (q[i6] * q[i19])
            + (-2.203541e-04) * (q[i6] * q[i20]) + (-4.420825e-04) * (q[i6] * q[i21]) + (-1.667029e-04) * (q[i6] * q[i22]) + (-1.582883e-03) * (q[i7] * q[i9])
            + (-3.117434e-04) * (q[i7] * q[i10]) + (1.097291e-03) * (q[i7] * q[i11]) + (8.750085e-04) * (q[i7] * q[i12]) + (-7.901275e-04) * (q[i7] * q[i15])
            + (7.769723e-04) * (q[i7] * q[i16]) + (-2.197780e-04) * (q[i7] * q[i19]) + (-3.898825e-05) * (q[i7] * q[i20]) + (1.665584e-04) * (q[i7] * q[i21])
            + (4.488181e-04) * (q[i7] * q[i22]) + (-1.025028e-04) * (q[i9] * q[i10]) + (-4.504523e-04) * (q[i9] * q[i11]) + (-9.541455e-04) * (q[i9] * q[i12])
            + (-3.011738e-04) * (q[i9] * q[i15]) + (-1.652432e-04) * (q[i9] * q[i16]) + (-1.399525e-04) * (q[i9] * q[i19]) + (8.570201e-04) * (q[i9] * q[i20])
            + (9.173531e-05) * (q[i9] * q[i21]) + (-3.030985e-04) * (q[i9] * q[i22]) + (9.523357e-04) * (q[i10] * q[i11]) + (4.452269e-04) * (q[i10] * q[i12])
            + (-1.570818e-04) * (q[i10] * q[i15]) + (-3.077152e-04) * (q[i10] * q[i16]) + (8.625016e-04) * (q[i10] * q[i19])
            + (-1.244961e-04) * (q[i10] * q[i20]) + (2.951818e-04) * (q[i10] * q[i21]) + (-9.019046e-05) * (q[i10] * q[i22])
            + (-1.421608e-03) * (q[i11] * q[i12]) + (-9.667942e-04) * (q[i11] * q[i15]) + (2.049559e-04) * (q[i11] * q[i16])
            + (7.583491e-04) * (q[i11] * q[i19]) + (1.177453e-03) * (q[i11] * q[i20]) + (-3.791023e-04) * (q[i11] * q[i21])
            + (-2.098501e-04) * (q[i11] * q[i22]) + (-2.133103e-04) * (q[i12] * q[i15]) + (9.615786e-04) * (q[i12] * q[i16])
            + (-1.190358e-03) * (q[i12] * q[i19]) + (-7.548895e-04) * (q[i12] * q[i20]) + (-2.082552e-04) * (q[i12] * q[i21])
            + (-3.871480e-04) * (q[i12] * q[i22]) + (-2.463027e-04) * (q[i15] * q[i16]) + (1.797364e-04) * (q[i15] * q[i19])
            + (-7.647453e-04) * (q[i15] * q[i20]) + (-1.503288e-03) * (q[i15] * q[i21]) + (2.291151e-04) * (q[i15] * q[i22])
            + (-7.659774e-04) * (q[i16] * q[i19]) + (1.855496e-04) * (q[i16] * q[i20]) + (-2.341260e-04) * (q[i16] * q[i21])
            + (1.516622e-03) * (q[i16] * q[i22]) + (6.328515e-04) * (q[i19] * q[i20]) + (-1.120134e-03) * (q[i19] * q[i21])
            + (-4.248339e-04) * (q[i19] * q[i22]) + (4.221432e-04) * (q[i20] * q[i21]) + (1.111237e-03) * (q[i20] * q[i22])
            + (-4.557013e-04) * (q[i21] * q[i22]);
   }

   public void getJQy9(double[] q, double[][] JQ)
   {
      JQ[2][i9] = (5.879427e-02) * (1) + (-3.273431e-03) * ((2) * q[i9]) + (-1.577200e-03) * (q[i0]) + (2.346357e-03) * (q[i1]) + (1.460264e-03) * (q[i2])
            + (-6.041502e-03) * (q[i3]) + (-5.699578e-03) * (q[i4]) + (-5.528472e-03) * (q[i5]) + (-7.158658e-03) * (q[i6]) + (5.781920e-03) * (q[i7])
            + (2.419856e-03) * (q[i8]) + (4.848303e-03) * (q[i10]) + (-2.832155e-04) * (q[i11]) + (3.941600e-04) * (q[i12]) + (1.616846e-03) * (q[i15])
            + (2.078583e-03) * (q[i16]) + (7.876256e-05) * (q[i19]) + (1.164829e-03) * (q[i20]) + (-7.003847e-04) * (q[i21]) + (4.554292e-04) * (q[i22])
            + (-3.620341e-03) * (q[i0] * q[i0]) + (-4.279621e-04) * (q[i1] * q[i1]) + (-9.801745e-04) * (q[i2] * q[i2]) + (2.280216e-03) * (q[i3] * q[i3])
            + (-9.772404e-04) * (q[i4] * q[i4]) + (-2.407482e-04) * (q[i5] * q[i5]) + (-6.738246e-04) * (q[i6] * q[i6]) + (-4.165958e-04) * (q[i7] * q[i7])
            + (8.673059e-04) * (q[i8] * q[i8]) + (1.141431e-04) * ((2) * q[i0] * q[i9]) + (-5.281776e-04) * ((2) * q[i1] * q[i9])
            + (7.392941e-04) * ((2) * q[i2] * q[i9]) + (1.189732e-03) * ((2) * q[i3] * q[i9]) + (1.167596e-03) * ((2) * q[i4] * q[i9])
            + (8.578853e-04) * ((2) * q[i5] * q[i9]) + (-3.082180e-03) * ((2) * q[i6] * q[i9]) + (-9.292043e-05) * ((2) * q[i7] * q[i9])
            + (1.081172e-03) * ((2) * q[i8] * q[i9]) + (-1.031580e-03) * ((3) * q[i9] * q[i9]) + (-3.716021e-04) * ((2) * q[i9] * q[i10])
            + (1.059143e-04) * ((2) * q[i9] * q[i11]) + (1.338060e-04) * ((2) * q[i9] * q[i12]) + (-2.008965e-04) * ((2) * q[i9] * q[i15])
            + (-2.215306e-04) * ((2) * q[i9] * q[i16]) + (1.068099e-04) * ((2) * q[i9] * q[i19]) + (-2.559619e-04) * ((2) * q[i9] * q[i20])
            + (5.934105e-06) * ((2) * q[i9] * q[i21]) + (-2.138896e-04) * ((2) * q[i9] * q[i22]) + (-3.751170e-04) * (q[i10] * q[i10])
            + (-1.130603e-04) * (q[i11] * q[i11]) + (6.798928e-05) * (q[i12] * q[i12]) + (-9.121269e-06) * (q[i15] * q[i15])
            + (-1.646033e-04) * (q[i16] * q[i16]) + (-1.551092e-04) * (q[i19] * q[i19]) + (-1.402449e-04) * (q[i20] * q[i20])
            + (-3.988740e-05) * (q[i21] * q[i21]) + (1.021671e-04) * (q[i22] * q[i22]) + (3.022323e-04) * (q[i0] * q[i1]) + (-1.992049e-03) * (q[i0] * q[i2])
            + (1.275901e-02) * (q[i0] * q[i3]) + (-9.015293e-04) * (q[i0] * q[i4]) + (2.272778e-03) * (q[i0] * q[i5]) + (-5.399697e-04) * (q[i0] * q[i6])
            + (1.644994e-03) * (q[i0] * q[i7]) + (7.119842e-04) * (q[i0] * q[i8]) + (1.360263e-03) * (q[i0] * q[i10]) + (4.597602e-05) * (q[i0] * q[i11])
            + (-1.002433e-03) * (q[i0] * q[i12]) + (1.017340e-04) * (q[i0] * q[i15]) + (1.127961e-03) * (q[i0] * q[i16]) + (-1.799168e-04) * (q[i0] * q[i19])
            + (5.617884e-04) * (q[i0] * q[i20]) + (-7.818176e-05) * (q[i0] * q[i21]) + (-6.660250e-05) * (q[i0] * q[i22]) + (2.449361e-03) * (q[i1] * q[i2])
            + (-4.320600e-04) * (q[i1] * q[i3]) + (-2.675081e-03) * (q[i1] * q[i4]) + (-3.407700e-04) * (q[i1] * q[i5]) + (-5.646921e-04) * (q[i1] * q[i6])
            + (-1.866107e-03) * (q[i1] * q[i7]) + (-2.056903e-04) * (q[i1] * q[i8]) + (-1.360802e-03) * (q[i1] * q[i10]) + (5.642974e-04) * (q[i1] * q[i11])
            + (7.066151e-04) * (q[i1] * q[i12]) + (5.905837e-04) * (q[i1] * q[i15]) + (-5.296953e-04) * (q[i1] * q[i16]) + (4.403208e-04) * (q[i1] * q[i19])
            + (-2.157064e-04) * (q[i1] * q[i20]) + (-2.141346e-04) * (q[i1] * q[i21]) + (-1.572557e-04) * (q[i1] * q[i22]) + (2.309691e-03) * (q[i2] * q[i3])
            + (-3.234076e-04) * (q[i2] * q[i4]) + (5.292999e-03) * (q[i2] * q[i5]) + (1.297394e-03) * (q[i2] * q[i6]) + (4.524172e-04) * (q[i2] * q[i7])
            + (2.149226e-03) * (q[i2] * q[i8]) + (2.516672e-06) * (q[i2] * q[i10]) + (3.620506e-06) * (q[i2] * q[i11]) + (-1.799722e-04) * (q[i2] * q[i12])
            + (4.049757e-04) * (q[i2] * q[i15]) + (-5.789794e-05) * (q[i2] * q[i16]) + (-2.610005e-04) * (q[i2] * q[i19]) + (9.740003e-05) * (q[i2] * q[i20])
            + (-4.083199e-04) * (q[i2] * q[i21]) + (-3.429924e-04) * (q[i2] * q[i22]) + (-1.861590e-03) * (q[i3] * q[i4]) + (-1.723064e-03) * (q[i3] * q[i5])
            + (1.313509e-03) * (q[i3] * q[i6]) + (-1.450256e-03) * (q[i3] * q[i7]) + (-1.037956e-03) * (q[i3] * q[i8]) + (-8.110467e-04) * (q[i3] * q[i10])
            + (7.615759e-04) * (q[i3] * q[i11]) + (6.839198e-04) * (q[i3] * q[i12]) + (-2.617285e-04) * (q[i3] * q[i15]) + (-7.010736e-04) * (q[i3] * q[i16])
            + (3.544132e-04) * (q[i3] * q[i19]) + (-6.354895e-04) * (q[i3] * q[i20]) + (-2.459064e-04) * (q[i3] * q[i21]) + (-3.224698e-04) * (q[i3] * q[i22])
            + (-5.934737e-04) * (q[i4] * q[i5]) + (2.745881e-03) * (q[i4] * q[i6]) + (8.076992e-04) * (q[i4] * q[i7]) + (-2.740939e-04) * (q[i4] * q[i8])
            + (8.112286e-04) * (q[i4] * q[i10]) + (7.421642e-05) * (q[i4] * q[i11]) + (-1.158998e-03) * (q[i4] * q[i12]) + (-4.751135e-04) * (q[i4] * q[i15])
            + (4.904249e-04) * (q[i4] * q[i16]) + (-6.228559e-04) * (q[i4] * q[i19]) + (-2.543523e-04) * (q[i4] * q[i20]) + (2.289478e-04) * (q[i4] * q[i21])
            + (-4.351189e-08) * (q[i4] * q[i22]) + (2.186654e-03) * (q[i5] * q[i6]) + (2.477884e-03) * (q[i5] * q[i7]) + (2.484928e-03) * (q[i5] * q[i8])
            + (-2.280379e-06) * (q[i5] * q[i10]) + (-9.996856e-04) * (q[i5] * q[i11]) + (2.664558e-04) * (q[i5] * q[i12]) + (4.065399e-04) * (q[i5] * q[i15])
            + (-8.526925e-04) * (q[i5] * q[i16]) + (6.817273e-04) * (q[i5] * q[i19]) + (2.758747e-04) * (q[i5] * q[i20]) + (-5.875879e-04) * (q[i5] * q[i21])
            + (-2.189059e-04) * (q[i5] * q[i22]) + (5.066186e-04) * (q[i6] * q[i7]) + (-3.254459e-04) * (q[i6] * q[i8]) + (-3.222617e-04) * (q[i6] * q[i10])
            + (3.018579e-04) * (q[i6] * q[i11]) + (3.321788e-04) * (q[i6] * q[i12]) + (-4.425578e-04) * (q[i6] * q[i15]) + (-8.358086e-05) * (q[i6] * q[i16])
            + (1.934901e-04) * (q[i6] * q[i19]) + (-2.956798e-04) * (q[i6] * q[i20]) + (-2.630103e-04) * (q[i6] * q[i21]) + (-2.043316e-04) * (q[i6] * q[i22])
            + (-1.582883e-03) * (q[i7] * q[i8]) + (-3.334738e-04) * (q[i7] * q[i10]) + (5.548080e-04) * (q[i7] * q[i11]) + (-2.256889e-05) * (q[i7] * q[i12])
            + (2.860588e-04) * (q[i7] * q[i15]) + (1.170306e-05) * (q[i7] * q[i16]) + (4.661810e-04) * (q[i7] * q[i19]) + (-2.378912e-05) * (q[i7] * q[i20])
            + (2.457164e-06) * (q[i7] * q[i21]) + (3.049735e-04) * (q[i7] * q[i22]) + (-1.025028e-04) * (q[i8] * q[i10]) + (-4.504523e-04) * (q[i8] * q[i11])
            + (-9.541455e-04) * (q[i8] * q[i12]) + (-3.011738e-04) * (q[i8] * q[i15]) + (-1.652432e-04) * (q[i8] * q[i16]) + (-1.399525e-04) * (q[i8] * q[i19])
            + (8.570201e-04) * (q[i8] * q[i20]) + (9.173531e-05) * (q[i8] * q[i21]) + (-3.030985e-04) * (q[i8] * q[i22]) + (1.670375e-04) * (q[i10] * q[i11])
            + (-1.631895e-04) * (q[i10] * q[i12]) + (2.964690e-04) * (q[i10] * q[i15]) + (2.970809e-04) * (q[i10] * q[i16]) + (3.917550e-05) * (q[i10] * q[i19])
            + (4.137510e-05) * (q[i10] * q[i20]) + (7.523850e-05) * (q[i10] * q[i21]) + (-7.457916e-05) * (q[i10] * q[i22]) + (2.691510e-04) * (q[i11] * q[i12])
            + (1.353202e-04) * (q[i11] * q[i15]) + (-1.070767e-04) * (q[i11] * q[i16]) + (1.100914e-04) * (q[i11] * q[i19]) + (5.449230e-04) * (q[i11] * q[i20])
            + (-5.803486e-05) * (q[i11] * q[i21]) + (8.276334e-05) * (q[i11] * q[i22]) + (2.551192e-04) * (q[i12] * q[i15]) + (2.425242e-04) * (q[i12] * q[i16])
            + (1.687963e-04) * (q[i12] * q[i19]) + (3.931066e-04) * (q[i12] * q[i20]) + (-2.334586e-05) * (q[i12] * q[i21]) + (3.840462e-07) * (q[i12] * q[i22])
            + (2.926568e-04) * (q[i15] * q[i16]) + (-1.205072e-04) * (q[i15] * q[i19]) + (2.208531e-04) * (q[i15] * q[i20]) + (5.189756e-06) * (q[i15] * q[i21])
            + (-3.313817e-05) * (q[i15] * q[i22]) + (4.823402e-04) * (q[i16] * q[i19]) + (2.434512e-04) * (q[i16] * q[i20])
            + (-8.190503e-05) * (q[i16] * q[i21]) + (-1.549280e-04) * (q[i16] * q[i22]) + (-1.338109e-04) * (q[i19] * q[i20])
            + (1.220038e-04) * (q[i19] * q[i21]) + (1.010255e-04) * (q[i19] * q[i22]) + (1.195072e-04) * (q[i20] * q[i21]) + (-1.742876e-04) * (q[i20] * q[i22])
            + (6.391691e-05) * (q[i21] * q[i22]);
   }

   public void getJQy10(double[] q, double[][] JQ)
   {
      JQ[2][i10] = (5.814686e-02) * (1) + (-3.258967e-03) * ((2) * q[i10]) + (-2.364382e-03) * (q[i0]) + (1.575809e-03) * (q[i1]) + (-1.435032e-03) * (q[i2])
            + (5.639904e-03) * (q[i3]) + (5.987383e-03) * (q[i4]) + (5.516264e-03) * (q[i5]) + (5.731393e-03) * (q[i6]) + (-7.079973e-03) * (q[i7])
            + (2.390816e-03) * (q[i8]) + (4.848303e-03) * (q[i9]) + (-3.697813e-04) * (q[i11]) + (3.171618e-04) * (q[i12]) + (2.032891e-03) * (q[i15])
            + (1.586893e-03) * (q[i16]) + (1.160841e-03) * (q[i19]) + (6.927503e-05) * (q[i20]) + (-4.558118e-04) * (q[i21]) + (6.953218e-04) * (q[i22])
            + (-4.255330e-04) * (q[i0] * q[i0]) + (-3.573820e-03) * (q[i1] * q[i1]) + (-9.776732e-04) * (q[i2] * q[i2]) + (-9.699651e-04) * (q[i3] * q[i3])
            + (2.273967e-03) * (q[i4] * q[i4]) + (-2.454196e-04) * (q[i5] * q[i5]) + (-4.169451e-04) * (q[i6] * q[i6]) + (-6.674327e-04) * (q[i7] * q[i7])
            + (8.654743e-04) * (q[i8] * q[i8]) + (-3.716021e-04) * (q[i9] * q[i9]) + (5.209430e-04) * ((2) * q[i0] * q[i10])
            + (-1.151267e-04) * ((2) * q[i1] * q[i10]) + (-7.362072e-04) * ((2) * q[i2] * q[i10]) + (-1.155973e-03) * ((2) * q[i3] * q[i10])
            + (-1.170908e-03) * ((2) * q[i4] * q[i10]) + (-8.626645e-04) * ((2) * q[i5] * q[i10]) + (-1.009958e-04) * ((2) * q[i6] * q[i10])
            + (-3.045815e-03) * ((2) * q[i7] * q[i10]) + (1.069369e-03) * ((2) * q[i8] * q[i10]) + (-3.751170e-04) * ((2) * q[i9] * q[i10])
            + (-1.017551e-03) * ((3) * q[i10] * q[i10]) + (-1.380678e-04) * ((2) * q[i10] * q[i11]) + (-1.062532e-04) * ((2) * q[i10] * q[i12])
            + (-2.173750e-04) * ((2) * q[i10] * q[i15]) + (-1.998722e-04) * ((2) * q[i10] * q[i16]) + (-2.521337e-04) * ((2) * q[i10] * q[i19])
            + (1.029168e-04) * ((2) * q[i10] * q[i20]) + (2.119792e-04) * ((2) * q[i10] * q[i21]) + (-6.543407e-06) * ((2) * q[i10] * q[i22])
            + (5.397174e-05) * (q[i11] * q[i11]) + (-1.124623e-04) * (q[i12] * q[i12]) + (-1.647609e-04) * (q[i15] * q[i15])
            + (-1.332429e-05) * (q[i16] * q[i16]) + (-1.400187e-04) * (q[i19] * q[i19]) + (-1.564151e-04) * (q[i20] * q[i20])
            + (1.020322e-04) * (q[i21] * q[i21]) + (-4.066959e-05) * (q[i22] * q[i22]) + (2.846155e-04) * (q[i0] * q[i1]) + (2.438050e-03) * (q[i0] * q[i2])
            + (-2.628810e-03) * (q[i0] * q[i3]) + (-4.478219e-04) * (q[i0] * q[i4]) + (-3.349013e-04) * (q[i0] * q[i5]) + (1.844408e-03) * (q[i0] * q[i6])
            + (5.517188e-04) * (q[i0] * q[i7]) + (2.040536e-04) * (q[i0] * q[i8]) + (1.360263e-03) * (q[i0] * q[i9]) + (6.985196e-04) * (q[i0] * q[i11])
            + (5.734440e-04) * (q[i0] * q[i12]) + (5.303826e-04) * (q[i0] * q[i15]) + (-5.940487e-04) * (q[i0] * q[i16]) + (2.148813e-04) * (q[i0] * q[i19])
            + (-4.319454e-04) * (q[i0] * q[i20]) + (-1.584458e-04) * (q[i0] * q[i21]) + (-2.132184e-04) * (q[i0] * q[i22]) + (-1.976517e-03) * (q[i1] * q[i2])
            + (-8.867219e-04) * (q[i1] * q[i3]) + (1.261796e-02) * (q[i1] * q[i4]) + (2.248592e-03) * (q[i1] * q[i5]) + (-1.639768e-03) * (q[i1] * q[i6])
            + (5.542534e-04) * (q[i1] * q[i7]) + (-7.043732e-04) * (q[i1] * q[i8]) + (-1.360802e-03) * (q[i1] * q[i9]) + (-9.983918e-04) * (q[i1] * q[i11])
            + (3.945032e-05) * (q[i1] * q[i12]) + (-1.115424e-03) * (q[i1] * q[i15]) + (-9.232939e-05) * (q[i1] * q[i16]) + (-5.613018e-04) * (q[i1] * q[i19])
            + (1.733368e-04) * (q[i1] * q[i20]) + (-6.546301e-05) * (q[i1] * q[i21]) + (-7.665282e-05) * (q[i1] * q[i22]) + (-2.988670e-04) * (q[i2] * q[i3])
            + (2.270634e-03) * (q[i2] * q[i4]) + (5.223961e-03) * (q[i2] * q[i5]) + (-4.481669e-04) * (q[i2] * q[i6]) + (-1.291531e-03) * (q[i2] * q[i7])
            + (-2.143734e-03) * (q[i2] * q[i8]) + (2.516672e-06) * (q[i2] * q[i9]) + (-1.926262e-04) * (q[i2] * q[i11]) + (1.820085e-06) * (q[i2] * q[i12])
            + (6.286513e-05) * (q[i2] * q[i15]) + (-4.083522e-04) * (q[i2] * q[i16]) + (-1.011537e-04) * (q[i2] * q[i19]) + (2.619118e-04) * (q[i2] * q[i20])
            + (-3.418190e-04) * (q[i2] * q[i21]) + (-4.035359e-04) * (q[i2] * q[i22]) + (-1.869920e-03) * (q[i3] * q[i4]) + (-5.797220e-04) * (q[i3] * q[i5])
            + (-8.018915e-04) * (q[i3] * q[i6]) + (-2.740255e-03) * (q[i3] * q[i7]) + (2.746779e-04) * (q[i3] * q[i8]) + (-8.110467e-04) * (q[i3] * q[i9])
            + (-1.161962e-03) * (q[i3] * q[i11]) + (7.719139e-05) * (q[i3] * q[i12]) + (-4.930728e-04) * (q[i3] * q[i15]) + (4.747305e-04) * (q[i3] * q[i16])
            + (2.506987e-04) * (q[i3] * q[i19]) + (6.488983e-04) * (q[i3] * q[i20]) + (1.378146e-05) * (q[i3] * q[i21]) + (2.411005e-04) * (q[i3] * q[i22])
            + (-1.696389e-03) * (q[i4] * q[i5]) + (1.436831e-03) * (q[i4] * q[i6]) + (-1.282671e-03) * (q[i4] * q[i7]) + (1.022948e-03) * (q[i4] * q[i8])
            + (8.112286e-04) * (q[i4] * q[i9]) + (6.648302e-04) * (q[i4] * q[i11]) + (7.633476e-04) * (q[i4] * q[i12]) + (6.980060e-04) * (q[i4] * q[i15])
            + (2.553292e-04) * (q[i4] * q[i16]) + (6.283218e-04) * (q[i4] * q[i19]) + (-3.471171e-04) * (q[i4] * q[i20]) + (-3.230970e-04) * (q[i4] * q[i21])
            + (-2.538713e-04) * (q[i4] * q[i22]) + (-2.477935e-03) * (q[i5] * q[i6]) + (-2.164833e-03) * (q[i5] * q[i7]) + (-2.482312e-03) * (q[i5] * q[i8])
            + (-2.280379e-06) * (q[i5] * q[i9]) + (2.569866e-04) * (q[i5] * q[i11]) + (-9.864429e-04) * (q[i5] * q[i12]) + (8.599808e-04) * (q[i5] * q[i15])
            + (-4.068580e-04) * (q[i5] * q[i16]) + (-2.698364e-04) * (q[i5] * q[i19]) + (-7.060816e-04) * (q[i5] * q[i20]) + (-2.235920e-04) * (q[i5] * q[i21])
            + (-5.826465e-04) * (q[i5] * q[i22]) + (5.131238e-04) * (q[i6] * q[i7]) + (-1.566180e-03) * (q[i6] * q[i8]) + (-3.222617e-04) * (q[i6] * q[i9])
            + (3.046552e-05) * (q[i6] * q[i11]) + (-5.482079e-04) * (q[i6] * q[i12]) + (7.382468e-06) * (q[i6] * q[i15]) + (2.798608e-04) * (q[i6] * q[i16])
            + (-1.757855e-05) * (q[i6] * q[i19]) + (4.630654e-04) * (q[i6] * q[i20]) + (-3.136931e-04) * (q[i6] * q[i21]) + (-3.502427e-06) * (q[i6] * q[i22])
            + (-3.117434e-04) * (q[i7] * q[i8]) + (-3.334738e-04) * (q[i7] * q[i9]) + (-3.371485e-04) * (q[i7] * q[i11]) + (-3.108714e-04) * (q[i7] * q[i12])
            + (-7.809335e-05) * (q[i7] * q[i15]) + (-4.451027e-04) * (q[i7] * q[i16]) + (-2.948753e-04) * (q[i7] * q[i19]) + (1.864975e-04) * (q[i7] * q[i20])
            + (2.096182e-04) * (q[i7] * q[i21]) + (2.627882e-04) * (q[i7] * q[i22]) + (-1.025028e-04) * (q[i8] * q[i9]) + (9.523357e-04) * (q[i8] * q[i11])
            + (4.452269e-04) * (q[i8] * q[i12]) + (-1.570818e-04) * (q[i8] * q[i15]) + (-3.077152e-04) * (q[i8] * q[i16]) + (8.625016e-04) * (q[i8] * q[i19])
            + (-1.244961e-04) * (q[i8] * q[i20]) + (2.951818e-04) * (q[i8] * q[i21]) + (-9.019046e-05) * (q[i8] * q[i22]) + (1.670375e-04) * (q[i9] * q[i11])
            + (-1.631895e-04) * (q[i9] * q[i12]) + (2.964690e-04) * (q[i9] * q[i15]) + (2.970809e-04) * (q[i9] * q[i16]) + (3.917550e-05) * (q[i9] * q[i19])
            + (4.137510e-05) * (q[i9] * q[i20]) + (7.523850e-05) * (q[i9] * q[i21]) + (-7.457916e-05) * (q[i9] * q[i22]) + (2.763474e-04) * (q[i11] * q[i12])
            + (-2.427197e-04) * (q[i11] * q[i15]) + (-2.551507e-04) * (q[i11] * q[i16]) + (-3.946347e-04) * (q[i11] * q[i19])
            + (-1.611441e-04) * (q[i11] * q[i20]) + (-1.973640e-06) * (q[i11] * q[i21]) + (-1.972338e-05) * (q[i11] * q[i22])
            + (1.152339e-04) * (q[i12] * q[i15]) + (-1.365703e-04) * (q[i12] * q[i16]) + (-5.485815e-04) * (q[i12] * q[i19])
            + (-1.178500e-04) * (q[i12] * q[i20]) + (8.537272e-05) * (q[i12] * q[i21]) + (-5.462330e-05) * (q[i12] * q[i22])
            + (2.893244e-04) * (q[i15] * q[i16]) + (2.416269e-04) * (q[i15] * q[i19]) + (4.802103e-04) * (q[i15] * q[i20]) + (1.516216e-04) * (q[i15] * q[i21])
            + (8.125625e-05) * (q[i15] * q[i22]) + (2.265096e-04) * (q[i16] * q[i19]) + (-1.238974e-04) * (q[i16] * q[i20]) + (3.186902e-05) * (q[i16] * q[i21])
            + (-2.368354e-06) * (q[i16] * q[i22]) + (-1.316243e-04) * (q[i19] * q[i20]) + (1.747631e-04) * (q[i19] * q[i21])
            + (-1.190671e-04) * (q[i19] * q[i22]) + (-9.656492e-05) * (q[i20] * q[i21]) + (-1.132204e-04) * (q[i20] * q[i22])
            + (6.457753e-05) * (q[i21] * q[i22]);
   }

   public void getJQy11(double[] q, double[][] JQ)
   {
      JQ[2][i11] = (-5.222022e-03) * (1) + (-3.690845e-03) * ((2) * q[i11]) + (-2.101461e-03) * (q[i0]) + (-6.405035e-04) * (q[i1]) + (-2.575455e-03) * (q[i2])
            + (-4.951514e-03) * (q[i3]) + (-2.811232e-03) * (q[i4]) + (-4.741971e-03) * (q[i5]) + (2.074065e-03) * (q[i6]) + (-9.094966e-04) * (q[i7])
            + (-2.250525e-03) * (q[i8]) + (-2.832155e-04) * (q[i9]) + (-3.697813e-04) * (q[i10]) + (-1.794156e-04) * (q[i12]) + (8.438747e-04) * (q[i15])
            + (7.972360e-04) * (q[i16]) + (-9.842817e-04) * (q[i19]) + (-7.222546e-04) * (q[i20]) + (-1.187627e-03) * (q[i21]) + (1.108089e-05) * (q[i22])
            + (1.261540e-03) * (q[i0] * q[i0]) + (2.313198e-04) * (q[i1] * q[i1]) + (8.314828e-05) * (q[i2] * q[i2]) + (2.476817e-04) * (q[i3] * q[i3])
            + (-7.619221e-04) * (q[i4] * q[i4]) + (6.884126e-05) * (q[i5] * q[i5]) + (-2.863740e-04) * (q[i6] * q[i6]) + (-3.531736e-04) * (q[i7] * q[i7])
            + (7.407764e-04) * (q[i8] * q[i8]) + (1.059143e-04) * (q[i9] * q[i9]) + (-1.380678e-04) * (q[i10] * q[i10])
            + (2.709956e-04) * ((2) * q[i0] * q[i11]) + (-4.310596e-04) * ((2) * q[i1] * q[i11]) + (1.572873e-03) * ((2) * q[i2] * q[i11])
            + (1.445009e-03) * ((2) * q[i3] * q[i11]) + (-6.828332e-04) * ((2) * q[i4] * q[i11]) + (3.701298e-04) * ((2) * q[i5] * q[i11])
            + (-8.192230e-04) * ((2) * q[i6] * q[i11]) + (5.161338e-06) * ((2) * q[i7] * q[i11]) + (1.676774e-03) * ((2) * q[i8] * q[i11])
            + (-1.130603e-04) * ((2) * q[i9] * q[i11]) + (5.397174e-05) * ((2) * q[i10] * q[i11]) + (8.726313e-04) * ((3) * q[i11] * q[i11])
            + (-5.712128e-04) * ((2) * q[i11] * q[i12]) + (-2.055705e-03) * ((2) * q[i11] * q[i15]) + (-1.399818e-04) * ((2) * q[i11] * q[i16])
            + (6.309429e-04) * ((2) * q[i11] * q[i19]) + (-2.318441e-05) * ((2) * q[i11] * q[i20]) + (1.754330e-04) * ((2) * q[i11] * q[i21])
            + (1.907818e-04) * ((2) * q[i11] * q[i22]) + (5.751741e-04) * (q[i12] * q[i12]) + (2.137627e-03) * (q[i15] * q[i15])
            + (-2.187742e-04) * (q[i16] * q[i16]) + (-3.553330e-04) * (q[i19] * q[i19]) + (-1.814366e-04) * (q[i20] * q[i20])
            + (-4.051638e-04) * (q[i21] * q[i21]) + (2.422408e-05) * (q[i22] * q[i22]) + (-7.803878e-04) * (q[i0] * q[i1]) + (6.523406e-04) * (q[i0] * q[i2])
            + (3.415507e-04) * (q[i0] * q[i3]) + (-2.682264e-04) * (q[i0] * q[i4]) + (-2.742440e-03) * (q[i0] * q[i5]) + (-1.342392e-03) * (q[i0] * q[i6])
            + (8.710504e-05) * (q[i0] * q[i7]) + (-9.079045e-04) * (q[i0] * q[i8]) + (4.597602e-05) * (q[i0] * q[i9]) + (6.985196e-04) * (q[i0] * q[i10])
            + (-5.393562e-04) * (q[i0] * q[i12]) + (1.859815e-04) * (q[i0] * q[i15]) + (-7.142625e-05) * (q[i0] * q[i16]) + (5.389142e-04) * (q[i0] * q[i19])
            + (4.192410e-04) * (q[i0] * q[i20]) + (2.893217e-04) * (q[i0] * q[i21]) + (3.909004e-04) * (q[i0] * q[i22]) + (-8.432124e-04) * (q[i1] * q[i2])
            + (1.609103e-03) * (q[i1] * q[i3]) + (1.852015e-03) * (q[i1] * q[i4]) + (-2.247199e-03) * (q[i1] * q[i5]) + (2.123385e-04) * (q[i1] * q[i6])
            + (-1.426140e-03) * (q[i1] * q[i7]) + (1.002922e-03) * (q[i1] * q[i8]) + (5.642974e-04) * (q[i1] * q[i9]) + (-9.983918e-04) * (q[i1] * q[i10])
            + (5.435814e-04) * (q[i1] * q[i12]) + (-6.513442e-05) * (q[i1] * q[i15]) + (-2.026858e-05) * (q[i1] * q[i16]) + (6.352125e-05) * (q[i1] * q[i19])
            + (-5.394431e-04) * (q[i1] * q[i20]) + (3.606199e-04) * (q[i1] * q[i21]) + (5.179043e-04) * (q[i1] * q[i22]) + (7.361710e-04) * (q[i2] * q[i3])
            + (-5.738576e-04) * (q[i2] * q[i4]) + (1.171327e-03) * (q[i2] * q[i5]) + (-7.785610e-04) * (q[i2] * q[i6]) + (4.293556e-04) * (q[i2] * q[i7])
            + (4.147071e-03) * (q[i2] * q[i8]) + (3.620506e-06) * (q[i2] * q[i9]) + (-1.926262e-04) * (q[i2] * q[i10]) + (6.764880e-06) * (q[i2] * q[i12])
            + (-2.362006e-03) * (q[i2] * q[i15]) + (-5.016997e-04) * (q[i2] * q[i16]) + (6.703634e-04) * (q[i2] * q[i19]) + (-4.443000e-04) * (q[i2] * q[i20])
            + (9.345787e-04) * (q[i2] * q[i21]) + (1.211605e-03) * (q[i2] * q[i22]) + (1.272945e-03) * (q[i3] * q[i4]) + (-1.448399e-03) * (q[i3] * q[i5])
            + (-2.051880e-03) * (q[i3] * q[i6]) + (5.274222e-04) * (q[i3] * q[i7]) + (1.522556e-03) * (q[i3] * q[i8]) + (7.615759e-04) * (q[i3] * q[i9])
            + (-1.161962e-03) * (q[i3] * q[i10]) + (-7.578032e-04) * (q[i3] * q[i12]) + (-9.027708e-04) * (q[i3] * q[i15]) + (-3.870236e-05) * (q[i3] * q[i16])
            + (-1.712573e-04) * (q[i3] * q[i19]) + (-8.019534e-04) * (q[i3] * q[i20]) + (-8.460064e-04) * (q[i3] * q[i21]) + (4.242010e-04) * (q[i3] * q[i22])
            + (1.201051e-03) * (q[i4] * q[i5]) + (2.222197e-04) * (q[i4] * q[i6]) + (-2.309113e-04) * (q[i4] * q[i7]) + (-3.970672e-05) * (q[i4] * q[i8])
            + (7.421642e-05) * (q[i4] * q[i9]) + (6.648302e-04) * (q[i4] * q[i10]) + (7.642985e-04) * (q[i4] * q[i12]) + (-4.090085e-04) * (q[i4] * q[i15])
            + (1.070191e-03) * (q[i4] * q[i16]) + (-1.119821e-04) * (q[i4] * q[i19]) + (9.979069e-05) * (q[i4] * q[i20]) + (1.227838e-04) * (q[i4] * q[i21])
            + (2.703891e-05) * (q[i4] * q[i22]) + (4.159708e-04) * (q[i5] * q[i6]) + (-1.864377e-03) * (q[i5] * q[i7]) + (2.648866e-03) * (q[i5] * q[i8])
            + (-9.996856e-04) * (q[i5] * q[i9]) + (2.569866e-04) * (q[i5] * q[i10]) + (7.873494e-06) * (q[i5] * q[i12]) + (-2.545878e-03) * (q[i5] * q[i15])
            + (-3.355602e-04) * (q[i5] * q[i16]) + (-3.721245e-04) * (q[i5] * q[i19]) + (3.566828e-04) * (q[i5] * q[i20]) + (-5.596208e-04) * (q[i5] * q[i21])
            + (-2.649668e-04) * (q[i5] * q[i22]) + (1.122814e-03) * (q[i6] * q[i7]) + (-8.828370e-04) * (q[i6] * q[i8]) + (3.018579e-04) * (q[i6] * q[i9])
            + (3.046552e-05) * (q[i6] * q[i10]) + (-6.285653e-05) * (q[i6] * q[i12]) + (4.187876e-04) * (q[i6] * q[i15]) + (3.723862e-04) * (q[i6] * q[i16])
            + (-1.640755e-04) * (q[i6] * q[i19]) + (-2.434679e-04) * (q[i6] * q[i20]) + (-2.957810e-04) * (q[i6] * q[i21]) + (-6.079118e-04) * (q[i6] * q[i22])
            + (1.097291e-03) * (q[i7] * q[i8]) + (5.548080e-04) * (q[i7] * q[i9]) + (-3.371485e-04) * (q[i7] * q[i10]) + (-8.108071e-05) * (q[i7] * q[i12])
            + (5.273374e-04) * (q[i7] * q[i15]) + (-3.163262e-05) * (q[i7] * q[i16]) + (-7.524544e-05) * (q[i7] * q[i19]) + (9.860693e-04) * (q[i7] * q[i20])
            + (-1.315688e-06) * (q[i7] * q[i21]) + (6.556906e-04) * (q[i7] * q[i22]) + (-4.504523e-04) * (q[i8] * q[i9]) + (9.523357e-04) * (q[i8] * q[i10])
            + (-1.421608e-03) * (q[i8] * q[i12]) + (-9.667942e-04) * (q[i8] * q[i15]) + (2.049559e-04) * (q[i8] * q[i16]) + (7.583491e-04) * (q[i8] * q[i19])
            + (1.177453e-03) * (q[i8] * q[i20]) + (-3.791023e-04) * (q[i8] * q[i21]) + (-2.098501e-04) * (q[i8] * q[i22]) + (1.670375e-04) * (q[i9] * q[i10])
            + (2.691510e-04) * (q[i9] * q[i12]) + (1.353202e-04) * (q[i9] * q[i15]) + (-1.070767e-04) * (q[i9] * q[i16]) + (1.100914e-04) * (q[i9] * q[i19])
            + (5.449230e-04) * (q[i9] * q[i20]) + (-5.803486e-05) * (q[i9] * q[i21]) + (8.276334e-05) * (q[i9] * q[i22]) + (2.763474e-04) * (q[i10] * q[i12])
            + (-2.427197e-04) * (q[i10] * q[i15]) + (-2.551507e-04) * (q[i10] * q[i16]) + (-3.946347e-04) * (q[i10] * q[i19])
            + (-1.611441e-04) * (q[i10] * q[i20]) + (-1.973640e-06) * (q[i10] * q[i21]) + (-1.972338e-05) * (q[i10] * q[i22])
            + (1.804308e-05) * (q[i12] * q[i15]) + (2.901597e-05) * (q[i12] * q[i16]) + (-2.141351e-04) * (q[i12] * q[i19])
            + (-2.115579e-04) * (q[i12] * q[i20]) + (3.290286e-06) * (q[i12] * q[i21]) + (9.393283e-08) * (q[i12] * q[i22])
            + (-1.103244e-04) * (q[i15] * q[i16]) + (1.222465e-04) * (q[i15] * q[i19]) + (-5.286889e-04) * (q[i15] * q[i20])
            + (-7.061340e-05) * (q[i15] * q[i21]) + (1.524228e-04) * (q[i15] * q[i22]) + (-5.783379e-04) * (q[i16] * q[i19])
            + (-1.003911e-04) * (q[i16] * q[i20]) + (1.560478e-04) * (q[i16] * q[i21]) + (-5.117911e-05) * (q[i16] * q[i22])
            + (1.460629e-04) * (q[i19] * q[i20]) + (1.258352e-04) * (q[i19] * q[i21]) + (-9.818217e-05) * (q[i19] * q[i22])
            + (-8.726795e-05) * (q[i20] * q[i21]) + (-1.764124e-04) * (q[i20] * q[i22]) + (-1.611880e-05) * (q[i21] * q[i22]);
   }

   public void getJQy12(double[] q, double[][] JQ)
   {
      JQ[2][i12] = (5.619843e-03) * (1) + (-3.610286e-03) * ((2) * q[i12]) + (-6.247272e-04) * (q[i0]) + (-2.114922e-03) * (q[i1]) + (-2.486851e-03) * (q[i2])
            + (-2.852589e-03) * (q[i3]) + (-4.944247e-03) * (q[i4]) + (-4.693143e-03) * (q[i5]) + (9.387117e-04) * (q[i6]) + (-2.033062e-03) * (q[i7])
            + (2.137495e-03) * (q[i8]) + (3.941600e-04) * (q[i9]) + (3.171618e-04) * (q[i10]) + (-1.794156e-04) * (q[i11]) + (-7.939226e-04) * (q[i15])
            + (-9.109884e-04) * (q[i16]) + (7.347940e-04) * (q[i19]) + (9.576683e-04) * (q[i20]) + (3.132506e-05) * (q[i21]) + (-1.201508e-03) * (q[i22])
            + (-2.336911e-04) * (q[i0] * q[i0]) + (-1.247452e-03) * (q[i1] * q[i1]) + (-1.286932e-04) * (q[i2] * q[i2]) + (7.545558e-04) * (q[i3] * q[i3])
            + (-2.208289e-04) * (q[i4] * q[i4]) + (-9.857491e-05) * (q[i5] * q[i5]) + (3.608378e-04) * (q[i6] * q[i6]) + (3.045578e-04) * (q[i7] * q[i7])
            + (-7.625813e-04) * (q[i8] * q[i8]) + (1.338060e-04) * (q[i9] * q[i9]) + (-1.062532e-04) * (q[i10] * q[i10]) + (-5.712128e-04) * (q[i11] * q[i11])
            + (4.285072e-04) * ((2) * q[i0] * q[i12]) + (-2.801488e-04) * ((2) * q[i1] * q[i12]) + (-1.692362e-03) * ((2) * q[i2] * q[i12])
            + (6.728427e-04) * ((2) * q[i3] * q[i12]) + (-1.457032e-03) * ((2) * q[i4] * q[i12]) + (-4.291887e-04) * ((2) * q[i5] * q[i12])
            + (1.147318e-05) * ((2) * q[i6] * q[i12]) + (-8.323333e-04) * ((2) * q[i7] * q[i12]) + (1.706114e-03) * ((2) * q[i8] * q[i12])
            + (6.798928e-05) * ((2) * q[i9] * q[i12]) + (-1.124623e-04) * ((2) * q[i10] * q[i12]) + (5.751741e-04) * ((2) * q[i11] * q[i12])
            + (-9.228392e-04) * ((3) * q[i12] * q[i12]) + (-1.452196e-04) * ((2) * q[i12] * q[i15]) + (-2.048617e-03) * ((2) * q[i12] * q[i16])
            + (-1.360159e-05) * ((2) * q[i12] * q[i19]) + (6.134458e-04) * ((2) * q[i12] * q[i20]) + (-1.951116e-04) * ((2) * q[i12] * q[i21])
            + (-1.700646e-04) * ((2) * q[i12] * q[i22]) + (2.182402e-04) * (q[i15] * q[i15]) + (-2.170813e-03) * (q[i16] * q[i16])
            + (1.844954e-04) * (q[i19] * q[i19]) + (3.580127e-04) * (q[i20] * q[i20]) + (-2.526193e-05) * (q[i21] * q[i21]) + (4.178399e-04) * (q[i22] * q[i22])
            + (7.788289e-04) * (q[i0] * q[i1]) + (8.603355e-04) * (q[i0] * q[i2]) + (-1.892927e-03) * (q[i0] * q[i3]) + (-1.597540e-03) * (q[i0] * q[i4])
            + (2.213568e-03) * (q[i0] * q[i5]) + (-1.443070e-03) * (q[i0] * q[i6]) + (2.285169e-04) * (q[i0] * q[i7]) + (9.702281e-04) * (q[i0] * q[i8])
            + (-1.002433e-03) * (q[i0] * q[i9]) + (5.734440e-04) * (q[i0] * q[i10]) + (-5.393562e-04) * (q[i0] * q[i11]) + (-2.571111e-05) * (q[i0] * q[i15])
            + (-5.853968e-05) * (q[i0] * q[i16]) + (-5.457165e-04) * (q[i0] * q[i19]) + (6.830868e-05) * (q[i0] * q[i20]) + (-5.167213e-04) * (q[i0] * q[i21])
            + (-3.713889e-04) * (q[i0] * q[i22]) + (-6.395182e-04) * (q[i1] * q[i2]) + (2.617932e-04) * (q[i1] * q[i3]) + (-3.495028e-04) * (q[i1] * q[i4])
            + (2.677469e-03) * (q[i1] * q[i5]) + (7.670935e-05) * (q[i1] * q[i6]) + (-1.341927e-03) * (q[i1] * q[i7]) + (-9.243151e-04) * (q[i1] * q[i8])
            + (7.066151e-04) * (q[i1] * q[i9]) + (3.945032e-05) * (q[i1] * q[i10]) + (5.435814e-04) * (q[i1] * q[i11]) + (-7.006303e-05) * (q[i1] * q[i15])
            + (1.941888e-04) * (q[i1] * q[i16]) + (4.266644e-04) * (q[i1] * q[i19]) + (5.463806e-04) * (q[i1] * q[i20]) + (-3.924730e-04) * (q[i1] * q[i21])
            + (-2.773281e-04) * (q[i1] * q[i22]) + (5.600232e-04) * (q[i2] * q[i3]) + (-7.830249e-04) * (q[i2] * q[i4]) + (-1.285937e-03) * (q[i2] * q[i5])
            + (4.258846e-04) * (q[i2] * q[i6]) + (-8.119320e-04) * (q[i2] * q[i7]) + (4.154181e-03) * (q[i2] * q[i8]) + (-1.799722e-04) * (q[i2] * q[i9])
            + (1.820085e-06) * (q[i2] * q[i10]) + (6.764880e-06) * (q[i2] * q[i11]) + (-5.111858e-04) * (q[i2] * q[i15]) + (-2.336272e-03) * (q[i2] * q[i16])
            + (-4.444471e-04) * (q[i2] * q[i19]) + (6.501908e-04) * (q[i2] * q[i20]) + (-1.204995e-03) * (q[i2] * q[i21]) + (-9.273983e-04) * (q[i2] * q[i22])
            + (-1.294124e-03) * (q[i3] * q[i4]) + (-1.207768e-03) * (q[i3] * q[i5]) + (-2.301127e-04) * (q[i3] * q[i6]) + (2.305726e-04) * (q[i3] * q[i7])
            + (-3.970226e-05) * (q[i3] * q[i8]) + (6.839198e-04) * (q[i3] * q[i9]) + (7.719139e-05) * (q[i3] * q[i10]) + (-7.578032e-04) * (q[i3] * q[i11])
            + (1.071918e-03) * (q[i3] * q[i15]) + (-4.166864e-04) * (q[i3] * q[i16]) + (8.904234e-05) * (q[i3] * q[i19]) + (-1.091851e-04) * (q[i3] * q[i20])
            + (-2.769233e-05) * (q[i3] * q[i21]) + (-1.330506e-04) * (q[i3] * q[i22]) + (1.420428e-03) * (q[i4] * q[i5]) + (5.484947e-04) * (q[i4] * q[i6])
            + (-2.075818e-03) * (q[i4] * q[i7]) + (1.552861e-03) * (q[i4] * q[i8]) + (-1.158998e-03) * (q[i4] * q[i9]) + (7.633476e-04) * (q[i4] * q[i10])
            + (7.642985e-04) * (q[i4] * q[i11]) + (-2.736292e-05) * (q[i4] * q[i15]) + (-9.062632e-04) * (q[i4] * q[i16]) + (-7.938514e-04) * (q[i4] * q[i19])
            + (-1.668851e-04) * (q[i4] * q[i20]) + (-4.133740e-04) * (q[i4] * q[i21]) + (8.567602e-04) * (q[i4] * q[i22]) + (-1.840572e-03) * (q[i5] * q[i6])
            + (4.200824e-04) * (q[i5] * q[i7]) + (2.660963e-03) * (q[i5] * q[i8]) + (2.664558e-04) * (q[i5] * q[i9]) + (-9.864429e-04) * (q[i5] * q[i10])
            + (7.873494e-06) * (q[i5] * q[i11]) + (-3.395477e-04) * (q[i5] * q[i15]) + (-2.540585e-03) * (q[i5] * q[i16]) + (3.623418e-04) * (q[i5] * q[i19])
            + (-3.867339e-04) * (q[i5] * q[i20]) + (2.657216e-04) * (q[i5] * q[i21]) + (5.715204e-04) * (q[i5] * q[i22]) + (-1.119746e-03) * (q[i6] * q[i7])
            + (-1.065433e-03) * (q[i6] * q[i8]) + (3.321788e-04) * (q[i6] * q[i9]) + (-5.482079e-04) * (q[i6] * q[i10]) + (-6.285653e-05) * (q[i6] * q[i11])
            + (3.061818e-05) * (q[i6] * q[i15]) + (-5.408422e-04) * (q[i6] * q[i16]) + (-9.819785e-04) * (q[i6] * q[i19]) + (8.291197e-05) * (q[i6] * q[i20])
            + (6.580356e-04) * (q[i6] * q[i21]) + (3.425389e-07) * (q[i6] * q[i22]) + (8.750085e-04) * (q[i7] * q[i8]) + (-2.256889e-05) * (q[i7] * q[i9])
            + (-3.108714e-04) * (q[i7] * q[i10]) + (-8.108071e-05) * (q[i7] * q[i11]) + (-3.739448e-04) * (q[i7] * q[i15]) + (-4.156540e-04) * (q[i7] * q[i16])
            + (2.504454e-04) * (q[i7] * q[i19]) + (1.631307e-04) * (q[i7] * q[i20]) + (-6.068143e-04) * (q[i7] * q[i21]) + (-2.887615e-04) * (q[i7] * q[i22])
            + (-9.541455e-04) * (q[i8] * q[i9]) + (4.452269e-04) * (q[i8] * q[i10]) + (-1.421608e-03) * (q[i8] * q[i11]) + (-2.133103e-04) * (q[i8] * q[i15])
            + (9.615786e-04) * (q[i8] * q[i16]) + (-1.190358e-03) * (q[i8] * q[i19]) + (-7.548895e-04) * (q[i8] * q[i20]) + (-2.082552e-04) * (q[i8] * q[i21])
            + (-3.871480e-04) * (q[i8] * q[i22]) + (-1.631895e-04) * (q[i9] * q[i10]) + (2.691510e-04) * (q[i9] * q[i11]) + (2.551192e-04) * (q[i9] * q[i15])
            + (2.425242e-04) * (q[i9] * q[i16]) + (1.687963e-04) * (q[i9] * q[i19]) + (3.931066e-04) * (q[i9] * q[i20]) + (-2.334586e-05) * (q[i9] * q[i21])
            + (3.840462e-07) * (q[i9] * q[i22]) + (2.763474e-04) * (q[i10] * q[i11]) + (1.152339e-04) * (q[i10] * q[i15]) + (-1.365703e-04) * (q[i10] * q[i16])
            + (-5.485815e-04) * (q[i10] * q[i19]) + (-1.178500e-04) * (q[i10] * q[i20]) + (8.537272e-05) * (q[i10] * q[i21])
            + (-5.462330e-05) * (q[i10] * q[i22]) + (1.804308e-05) * (q[i11] * q[i15]) + (2.901597e-05) * (q[i11] * q[i16])
            + (-2.141351e-04) * (q[i11] * q[i19]) + (-2.115579e-04) * (q[i11] * q[i20]) + (3.290286e-06) * (q[i11] * q[i21])
            + (9.393283e-08) * (q[i11] * q[i22]) + (1.079393e-04) * (q[i15] * q[i16]) + (9.948236e-05) * (q[i15] * q[i19]) + (5.750484e-04) * (q[i15] * q[i20])
            + (-4.713812e-05) * (q[i15] * q[i21]) + (1.608407e-04) * (q[i15] * q[i22]) + (5.335886e-04) * (q[i16] * q[i19])
            + (-1.164256e-04) * (q[i16] * q[i20]) + (1.596939e-04) * (q[i16] * q[i21]) + (-8.637346e-05) * (q[i16] * q[i22])
            + (-1.457333e-04) * (q[i19] * q[i20]) + (-1.774228e-04) * (q[i19] * q[i21]) + (-8.433190e-05) * (q[i19] * q[i22])
            + (-9.988231e-05) * (q[i20] * q[i21]) + (1.307902e-04) * (q[i20] * q[i22]) + (1.312943e-05) * (q[i21] * q[i22]);
   }

   public void getJQy15(double[] q, double[][] JQ)
   {
      JQ[2][i15] = (5.040684e-03) * (1) + (-2.841511e-03) * ((2) * q[i15]) + (1.763563e-03) * (q[i0]) + (5.440769e-03) * (q[i1]) + (5.981859e-03) * (q[i2])
            + (1.701516e-04) * (q[i3]) + (-1.936538e-03) * (q[i4]) + (1.072417e-03) * (q[i5]) + (2.506067e-03) * (q[i6]) + (6.173416e-03) * (q[i7])
            + (-9.332737e-03) * (q[i8]) + (1.616846e-03) * (q[i9]) + (2.032891e-03) * (q[i10]) + (8.438747e-04) * (q[i11]) + (-7.939226e-04) * (q[i12])
            + (-9.050529e-04) * (q[i16]) + (6.894907e-04) * (q[i19]) + (-7.798828e-04) * (q[i20]) + (6.160006e-04) * (q[i21]) + (-3.030791e-04) * (q[i22])
            + (7.602174e-04) * (q[i0] * q[i0]) + (-1.123010e-04) * (q[i1] * q[i1]) + (-2.236801e-04) * (q[i2] * q[i2]) + (3.150266e-04) * (q[i3] * q[i3])
            + (1.659287e-04) * (q[i4] * q[i4]) + (-2.111881e-03) * (q[i5] * q[i5]) + (-5.963555e-04) * (q[i6] * q[i6]) + (-4.750761e-04) * (q[i7] * q[i7])
            + (4.076422e-05) * (q[i8] * q[i8]) + (-2.008965e-04) * (q[i9] * q[i9]) + (-2.173750e-04) * (q[i10] * q[i10]) + (-2.055705e-03) * (q[i11] * q[i11])
            + (-1.452196e-04) * (q[i12] * q[i12]) + (4.538791e-05) * ((2) * q[i0] * q[i15]) + (7.123550e-04) * ((2) * q[i1] * q[i15])
            + (2.563287e-03) * ((2) * q[i2] * q[i15]) + (-2.638679e-05) * ((2) * q[i3] * q[i15]) + (-4.245545e-04) * ((2) * q[i4] * q[i15])
            + (-5.541777e-04) * ((2) * q[i5] * q[i15]) + (4.019567e-05) * ((2) * q[i6] * q[i15]) + (8.646733e-04) * ((2) * q[i7] * q[i15])
            + (6.044488e-05) * ((2) * q[i8] * q[i15]) + (-9.121269e-06) * ((2) * q[i9] * q[i15]) + (-1.647609e-04) * ((2) * q[i10] * q[i15])
            + (2.137627e-03) * ((2) * q[i11] * q[i15]) + (2.182402e-04) * ((2) * q[i12] * q[i15]) + (5.182965e-05) * ((3) * q[i15] * q[i15])
            + (-3.212224e-04) * ((2) * q[i15] * q[i16]) + (1.617819e-04) * ((2) * q[i15] * q[i19]) + (-3.049154e-04) * ((2) * q[i15] * q[i20])
            + (-1.976457e-04) * ((2) * q[i15] * q[i21]) + (9.307695e-07) * ((2) * q[i15] * q[i22]) + (-3.227798e-04) * (q[i16] * q[i16])
            + (-1.633945e-04) * (q[i19] * q[i19]) + (-5.054305e-05) * (q[i20] * q[i20]) + (-1.613204e-04) * (q[i21] * q[i21])
            + (1.252246e-04) * (q[i22] * q[i22]) + (-5.317338e-04) * (q[i0] * q[i1]) + (9.454998e-05) * (q[i0] * q[i2]) + (-2.736937e-04) * (q[i0] * q[i3])
            + (-3.911409e-04) * (q[i0] * q[i4]) + (-1.032333e-03) * (q[i0] * q[i5]) + (-3.174323e-04) * (q[i0] * q[i6]) + (7.248336e-04) * (q[i0] * q[i7])
            + (-7.466604e-04) * (q[i0] * q[i8]) + (1.017340e-04) * (q[i0] * q[i9]) + (5.303826e-04) * (q[i0] * q[i10]) + (1.859815e-04) * (q[i0] * q[i11])
            + (-2.571111e-05) * (q[i0] * q[i12]) + (-2.180080e-04) * (q[i0] * q[i16]) + (-9.731131e-05) * (q[i0] * q[i19]) + (-1.329828e-04) * (q[i0] * q[i20])
            + (3.266881e-04) * (q[i0] * q[i21]) + (1.098482e-04) * (q[i0] * q[i22]) + (1.760426e-04) * (q[i1] * q[i2]) + (2.837263e-04) * (q[i1] * q[i3])
            + (-4.364390e-03) * (q[i1] * q[i4]) + (-1.592579e-03) * (q[i1] * q[i5]) + (-1.923527e-04) * (q[i1] * q[i6]) + (-1.151666e-03) * (q[i1] * q[i7])
            + (-2.146160e-03) * (q[i1] * q[i8]) + (5.905837e-04) * (q[i1] * q[i9]) + (-1.115424e-03) * (q[i1] * q[i10]) + (-6.513442e-05) * (q[i1] * q[i11])
            + (-7.006303e-05) * (q[i1] * q[i12]) + (2.186881e-04) * (q[i1] * q[i16]) + (-1.743718e-04) * (q[i1] * q[i19]) + (5.877674e-04) * (q[i1] * q[i20])
            + (1.723671e-04) * (q[i1] * q[i21]) + (-2.020610e-04) * (q[i1] * q[i22]) + (9.927942e-04) * (q[i2] * q[i3]) + (-2.498836e-03) * (q[i2] * q[i4])
            + (-6.544554e-03) * (q[i2] * q[i5]) + (-1.313394e-03) * (q[i2] * q[i6]) + (-4.246611e-04) * (q[i2] * q[i7]) + (-9.564607e-04) * (q[i2] * q[i8])
            + (4.049757e-04) * (q[i2] * q[i9]) + (6.286513e-05) * (q[i2] * q[i10]) + (-2.362006e-03) * (q[i2] * q[i11]) + (-5.111858e-04) * (q[i2] * q[i12])
            + (-4.108729e-06) * (q[i2] * q[i16]) + (-3.347044e-04) * (q[i2] * q[i19]) + (1.346695e-03) * (q[i2] * q[i20]) + (7.605788e-04) * (q[i2] * q[i21])
            + (3.999834e-05) * (q[i2] * q[i22]) + (-3.552360e-04) * (q[i3] * q[i4]) + (-1.703704e-03) * (q[i3] * q[i5]) + (-6.109421e-04) * (q[i3] * q[i6])
            + (1.450320e-03) * (q[i3] * q[i7]) + (-2.196074e-03) * (q[i3] * q[i8]) + (-2.617285e-04) * (q[i3] * q[i9]) + (-4.930728e-04) * (q[i3] * q[i10])
            + (-9.027708e-04) * (q[i3] * q[i11]) + (1.071918e-03) * (q[i3] * q[i12]) + (-4.291182e-04) * (q[i3] * q[i16]) + (-1.090363e-03) * (q[i3] * q[i19])
            + (-3.773423e-04) * (q[i3] * q[i20]) + (-7.025679e-04) * (q[i3] * q[i21]) + (-7.839491e-04) * (q[i3] * q[i22]) + (9.956928e-04) * (q[i4] * q[i5])
            + (7.600604e-04) * (q[i4] * q[i6]) + (-1.302155e-03) * (q[i4] * q[i7]) + (-1.735568e-03) * (q[i4] * q[i8]) + (-4.751135e-04) * (q[i4] * q[i9])
            + (6.980060e-04) * (q[i4] * q[i10]) + (-4.090085e-04) * (q[i4] * q[i11]) + (-2.736292e-05) * (q[i4] * q[i12]) + (4.269459e-04) * (q[i4] * q[i16])
            + (-1.936850e-04) * (q[i4] * q[i19]) + (4.801148e-04) * (q[i4] * q[i20]) + (2.930438e-04) * (q[i4] * q[i21]) + (-5.318361e-05) * (q[i4] * q[i22])
            + (6.391077e-04) * (q[i5] * q[i6]) + (3.664296e-04) * (q[i5] * q[i7]) + (2.435700e-03) * (q[i5] * q[i8]) + (4.065399e-04) * (q[i5] * q[i9])
            + (8.599808e-04) * (q[i5] * q[i10]) + (-2.545878e-03) * (q[i5] * q[i11]) + (-3.395477e-04) * (q[i5] * q[i12]) + (-1.584383e-06) * (q[i5] * q[i16])
            + (-4.229998e-04) * (q[i5] * q[i19]) + (-4.258683e-04) * (q[i5] * q[i20]) + (6.511432e-04) * (q[i5] * q[i21]) + (4.829936e-04) * (q[i5] * q[i22])
            + (6.383422e-04) * (q[i6] * q[i7]) + (7.782932e-04) * (q[i6] * q[i8]) + (-4.425578e-04) * (q[i6] * q[i9]) + (7.382468e-06) * (q[i6] * q[i10])
            + (4.187876e-04) * (q[i6] * q[i11]) + (3.061818e-05) * (q[i6] * q[i12]) + (-2.138078e-05) * (q[i6] * q[i16]) + (-6.097696e-05) * (q[i6] * q[i19])
            + (-4.208301e-04) * (q[i6] * q[i20]) + (-3.006825e-04) * (q[i6] * q[i21]) + (6.121058e-05) * (q[i6] * q[i22]) + (-7.901275e-04) * (q[i7] * q[i8])
            + (2.860588e-04) * (q[i7] * q[i9]) + (-7.809335e-05) * (q[i7] * q[i10]) + (5.273374e-04) * (q[i7] * q[i11]) + (-3.739448e-04) * (q[i7] * q[i12])
            + (-2.752398e-05) * (q[i7] * q[i16]) + (-2.515684e-04) * (q[i7] * q[i19]) + (1.008449e-03) * (q[i7] * q[i20]) + (7.773756e-04) * (q[i7] * q[i21])
            + (-1.900134e-04) * (q[i7] * q[i22]) + (-3.011738e-04) * (q[i8] * q[i9]) + (-1.570818e-04) * (q[i8] * q[i10]) + (-9.667942e-04) * (q[i8] * q[i11])
            + (-2.133103e-04) * (q[i8] * q[i12]) + (-2.463027e-04) * (q[i8] * q[i16]) + (1.797364e-04) * (q[i8] * q[i19]) + (-7.647453e-04) * (q[i8] * q[i20])
            + (-1.503288e-03) * (q[i8] * q[i21]) + (2.291151e-04) * (q[i8] * q[i22]) + (2.964690e-04) * (q[i9] * q[i10]) + (1.353202e-04) * (q[i9] * q[i11])
            + (2.551192e-04) * (q[i9] * q[i12]) + (2.926568e-04) * (q[i9] * q[i16]) + (-1.205072e-04) * (q[i9] * q[i19]) + (2.208531e-04) * (q[i9] * q[i20])
            + (5.189756e-06) * (q[i9] * q[i21]) + (-3.313817e-05) * (q[i9] * q[i22]) + (-2.427197e-04) * (q[i10] * q[i11]) + (1.152339e-04) * (q[i10] * q[i12])
            + (2.893244e-04) * (q[i10] * q[i16]) + (2.416269e-04) * (q[i10] * q[i19]) + (4.802103e-04) * (q[i10] * q[i20]) + (1.516216e-04) * (q[i10] * q[i21])
            + (8.125625e-05) * (q[i10] * q[i22]) + (1.804308e-05) * (q[i11] * q[i12]) + (-1.103244e-04) * (q[i11] * q[i16]) + (1.222465e-04) * (q[i11] * q[i19])
            + (-5.286889e-04) * (q[i11] * q[i20]) + (-7.061340e-05) * (q[i11] * q[i21]) + (1.524228e-04) * (q[i11] * q[i22])
            + (1.079393e-04) * (q[i12] * q[i16]) + (9.948236e-05) * (q[i12] * q[i19]) + (5.750484e-04) * (q[i12] * q[i20]) + (-4.713812e-05) * (q[i12] * q[i21])
            + (1.608407e-04) * (q[i12] * q[i22]) + (2.676551e-04) * (q[i16] * q[i19]) + (2.627945e-04) * (q[i16] * q[i20]) + (6.172918e-05) * (q[i16] * q[i21])
            + (-6.089486e-05) * (q[i16] * q[i22]) + (9.631664e-05) * (q[i19] * q[i20]) + (-8.679254e-04) * (q[i19] * q[i21])
            + (-2.501875e-04) * (q[i19] * q[i22]) + (-1.314745e-04) * (q[i20] * q[i21]) + (-9.374580e-05) * (q[i20] * q[i22])
            + (-4.223805e-05) * (q[i21] * q[i22]);
   }

   public void getJQy16(double[] q, double[][] JQ)
   {
      JQ[2][i16] = (4.959764e-03) * (1) + (-2.872103e-03) * ((2) * q[i16]) + (-5.522394e-03) * (q[i0]) + (-1.795247e-03) * (q[i1]) + (-6.084513e-03) * (q[i2])
            + (1.983357e-03) * (q[i3]) + (-1.070322e-04) * (q[i4]) + (-1.156109e-03) * (q[i5]) + (6.238223e-03) * (q[i6]) + (2.507167e-03) * (q[i7])
            + (-9.383865e-03) * (q[i8]) + (2.078583e-03) * (q[i9]) + (1.586893e-03) * (q[i10]) + (7.972360e-04) * (q[i11]) + (-9.109884e-04) * (q[i12])
            + (-9.050529e-04) * (q[i15]) + (-8.045054e-04) * (q[i19]) + (6.796655e-04) * (q[i20]) + (2.968374e-04) * (q[i21]) + (-6.132847e-04) * (q[i22])
            + (-1.045320e-04) * (q[i0] * q[i0]) + (7.653473e-04) * (q[i1] * q[i1]) + (-2.076048e-04) * (q[i2] * q[i2]) + (1.793124e-04) * (q[i3] * q[i3])
            + (2.896900e-04) * (q[i4] * q[i4]) + (-2.132539e-03) * (q[i5] * q[i5]) + (-4.876350e-04) * (q[i6] * q[i6]) + (-5.980053e-04) * (q[i7] * q[i7])
            + (5.069054e-05) * (q[i8] * q[i8]) + (-2.215306e-04) * (q[i9] * q[i9]) + (-1.998722e-04) * (q[i10] * q[i10]) + (-1.399818e-04) * (q[i11] * q[i11])
            + (-2.048617e-03) * (q[i12] * q[i12]) + (-3.212224e-04) * (q[i15] * q[i15]) + (-7.146619e-04) * ((2) * q[i0] * q[i16])
            + (-4.658708e-05) * ((2) * q[i1] * q[i16]) + (-2.586976e-03) * ((2) * q[i2] * q[i16]) + (4.245840e-04) * ((2) * q[i3] * q[i16])
            + (4.041991e-05) * ((2) * q[i4] * q[i16]) + (5.385013e-04) * ((2) * q[i5] * q[i16]) + (8.658253e-04) * ((2) * q[i6] * q[i16])
            + (3.910849e-05) * ((2) * q[i7] * q[i16]) + (7.343281e-05) * ((2) * q[i8] * q[i16]) + (-1.646033e-04) * ((2) * q[i9] * q[i16])
            + (-1.332429e-05) * ((2) * q[i10] * q[i16]) + (-2.187742e-04) * ((2) * q[i11] * q[i16]) + (-2.170813e-03) * ((2) * q[i12] * q[i16])
            + (-3.227798e-04) * ((2) * q[i15] * q[i16]) + (5.263756e-05) * ((3) * q[i16] * q[i16]) + (-3.098556e-04) * ((2) * q[i16] * q[i19])
            + (1.564144e-04) * ((2) * q[i16] * q[i20]) + (-3.724932e-06) * ((2) * q[i16] * q[i21]) + (2.048794e-04) * ((2) * q[i16] * q[i22])
            + (-4.874503e-05) * (q[i19] * q[i19]) + (-1.522340e-04) * (q[i20] * q[i20]) + (1.226978e-04) * (q[i21] * q[i21])
            + (-1.634418e-04) * (q[i22] * q[i22]) + (-5.364147e-04) * (q[i0] * q[i1]) + (1.685530e-04) * (q[i0] * q[i2]) + (-4.429769e-03) * (q[i0] * q[i3])
            + (2.809789e-04) * (q[i0] * q[i4]) + (-1.635496e-03) * (q[i0] * q[i5]) + (1.159499e-03) * (q[i0] * q[i6]) + (1.887704e-04) * (q[i0] * q[i7])
            + (2.160150e-03) * (q[i0] * q[i8]) + (1.127961e-03) * (q[i0] * q[i9]) + (-5.940487e-04) * (q[i0] * q[i10]) + (-7.142625e-05) * (q[i0] * q[i11])
            + (-5.853968e-05) * (q[i0] * q[i12]) + (-2.180080e-04) * (q[i0] * q[i15]) + (-5.863536e-04) * (q[i0] * q[i19]) + (1.775150e-04) * (q[i0] * q[i20])
            + (-2.073675e-04) * (q[i0] * q[i21]) + (1.769337e-04) * (q[i0] * q[i22]) + (1.001514e-04) * (q[i1] * q[i2]) + (-3.677597e-04) * (q[i1] * q[i3])
            + (-2.788492e-04) * (q[i1] * q[i4]) + (-1.026977e-03) * (q[i1] * q[i5]) + (-7.395194e-04) * (q[i1] * q[i6]) + (3.387974e-04) * (q[i1] * q[i7])
            + (7.556071e-04) * (q[i1] * q[i8]) + (-5.296953e-04) * (q[i1] * q[i9]) + (-9.232939e-05) * (q[i1] * q[i10]) + (-2.026858e-05) * (q[i1] * q[i11])
            + (1.941888e-04) * (q[i1] * q[i12]) + (2.186881e-04) * (q[i1] * q[i15]) + (1.262092e-04) * (q[i1] * q[i19]) + (8.947500e-05) * (q[i1] * q[i20])
            + (1.137587e-04) * (q[i1] * q[i21]) + (3.327511e-04) * (q[i1] * q[i22]) + (-2.515278e-03) * (q[i2] * q[i3]) + (1.011946e-03) * (q[i2] * q[i4])
            + (-6.624475e-03) * (q[i2] * q[i5]) + (4.201134e-04) * (q[i2] * q[i6]) + (1.335803e-03) * (q[i2] * q[i7]) + (9.360359e-04) * (q[i2] * q[i8])
            + (-5.789794e-05) * (q[i2] * q[i9]) + (-4.083522e-04) * (q[i2] * q[i10]) + (-5.016997e-04) * (q[i2] * q[i11]) + (-2.336272e-03) * (q[i2] * q[i12])
            + (-4.108729e-06) * (q[i2] * q[i15]) + (-1.355870e-03) * (q[i2] * q[i19]) + (3.529748e-04) * (q[i2] * q[i20]) + (4.025737e-05) * (q[i2] * q[i21])
            + (7.495992e-04) * (q[i2] * q[i22]) + (-3.478787e-04) * (q[i3] * q[i4]) + (9.957671e-04) * (q[i3] * q[i5]) + (1.322964e-03) * (q[i3] * q[i6])
            + (-7.606796e-04) * (q[i3] * q[i7]) + (1.745948e-03) * (q[i3] * q[i8]) + (-7.010736e-04) * (q[i3] * q[i9]) + (4.747305e-04) * (q[i3] * q[i10])
            + (-3.870236e-05) * (q[i3] * q[i11]) + (-4.166864e-04) * (q[i3] * q[i12]) + (-4.291182e-04) * (q[i3] * q[i15]) + (-4.696699e-04) * (q[i3] * q[i19])
            + (1.838969e-04) * (q[i3] * q[i20]) + (-4.494096e-05) * (q[i3] * q[i21]) + (2.849656e-04) * (q[i3] * q[i22]) + (-1.694699e-03) * (q[i4] * q[i5])
            + (-1.475898e-03) * (q[i4] * q[i6]) + (6.214028e-04) * (q[i4] * q[i7]) + (2.215468e-03) * (q[i4] * q[i8]) + (4.904249e-04) * (q[i4] * q[i9])
            + (2.553292e-04) * (q[i4] * q[i10]) + (1.070191e-03) * (q[i4] * q[i11]) + (-9.062632e-04) * (q[i4] * q[i12]) + (4.269459e-04) * (q[i4] * q[i15])
            + (3.716815e-04) * (q[i4] * q[i19]) + (1.078294e-03) * (q[i4] * q[i20]) + (-7.797041e-04) * (q[i4] * q[i21]) + (-7.098311e-04) * (q[i4] * q[i22])
            + (-3.613898e-04) * (q[i5] * q[i6]) + (-6.492493e-04) * (q[i5] * q[i7]) + (-2.476418e-03) * (q[i5] * q[i8]) + (-8.526925e-04) * (q[i5] * q[i9])
            + (-4.068580e-04) * (q[i5] * q[i10]) + (-3.355602e-04) * (q[i5] * q[i11]) + (-2.540585e-03) * (q[i5] * q[i12]) + (-1.584383e-06) * (q[i5] * q[i15])
            + (4.420311e-04) * (q[i5] * q[i19]) + (4.188070e-04) * (q[i5] * q[i20]) + (4.852346e-04) * (q[i5] * q[i21]) + (6.692534e-04) * (q[i5] * q[i22])
            + (6.339042e-04) * (q[i6] * q[i7]) + (-7.840902e-04) * (q[i6] * q[i8]) + (-8.358086e-05) * (q[i6] * q[i9]) + (2.798608e-04) * (q[i6] * q[i10])
            + (3.723862e-04) * (q[i6] * q[i11]) + (-5.408422e-04) * (q[i6] * q[i12]) + (-2.138078e-05) * (q[i6] * q[i15]) + (1.010537e-03) * (q[i6] * q[i19])
            + (-2.576356e-04) * (q[i6] * q[i20]) + (1.806325e-04) * (q[i6] * q[i21]) + (-7.857355e-04) * (q[i6] * q[i22]) + (7.769723e-04) * (q[i7] * q[i8])
            + (1.170306e-05) * (q[i7] * q[i9]) + (-4.451027e-04) * (q[i7] * q[i10]) + (-3.163262e-05) * (q[i7] * q[i11]) + (-4.156540e-04) * (q[i7] * q[i12])
            + (-2.752398e-05) * (q[i7] * q[i15]) + (-4.187241e-04) * (q[i7] * q[i19]) + (-4.536671e-05) * (q[i7] * q[i20]) + (-4.937533e-05) * (q[i7] * q[i21])
            + (3.086174e-04) * (q[i7] * q[i22]) + (-1.652432e-04) * (q[i8] * q[i9]) + (-3.077152e-04) * (q[i8] * q[i10]) + (2.049559e-04) * (q[i8] * q[i11])
            + (9.615786e-04) * (q[i8] * q[i12]) + (-2.463027e-04) * (q[i8] * q[i15]) + (-7.659774e-04) * (q[i8] * q[i19]) + (1.855496e-04) * (q[i8] * q[i20])
            + (-2.341260e-04) * (q[i8] * q[i21]) + (1.516622e-03) * (q[i8] * q[i22]) + (2.970809e-04) * (q[i9] * q[i10]) + (-1.070767e-04) * (q[i9] * q[i11])
            + (2.425242e-04) * (q[i9] * q[i12]) + (2.926568e-04) * (q[i9] * q[i15]) + (4.823402e-04) * (q[i9] * q[i19]) + (2.434512e-04) * (q[i9] * q[i20])
            + (-8.190503e-05) * (q[i9] * q[i21]) + (-1.549280e-04) * (q[i9] * q[i22]) + (-2.551507e-04) * (q[i10] * q[i11])
            + (-1.365703e-04) * (q[i10] * q[i12]) + (2.893244e-04) * (q[i10] * q[i15]) + (2.265096e-04) * (q[i10] * q[i19])
            + (-1.238974e-04) * (q[i10] * q[i20]) + (3.186902e-05) * (q[i10] * q[i21]) + (-2.368354e-06) * (q[i10] * q[i22])
            + (2.901597e-05) * (q[i11] * q[i12]) + (-1.103244e-04) * (q[i11] * q[i15]) + (-5.783379e-04) * (q[i11] * q[i19])
            + (-1.003911e-04) * (q[i11] * q[i20]) + (1.560478e-04) * (q[i11] * q[i21]) + (-5.117911e-05) * (q[i11] * q[i22])
            + (1.079393e-04) * (q[i12] * q[i15]) + (5.335886e-04) * (q[i12] * q[i19]) + (-1.164256e-04) * (q[i12] * q[i20]) + (1.596939e-04) * (q[i12] * q[i21])
            + (-8.637346e-05) * (q[i12] * q[i22]) + (2.676551e-04) * (q[i15] * q[i19]) + (2.627945e-04) * (q[i15] * q[i20]) + (6.172918e-05) * (q[i15] * q[i21])
            + (-6.089486e-05) * (q[i15] * q[i22]) + (9.661418e-05) * (q[i19] * q[i20]) + (9.483944e-05) * (q[i19] * q[i21]) + (1.345618e-04) * (q[i19] * q[i22])
            + (2.487113e-04) * (q[i20] * q[i21]) + (8.598867e-04) * (q[i20] * q[i22]) + (-4.249707e-05) * (q[i21] * q[i22]);
   }

   public void getJQy19(double[] q, double[][] JQ)
   {
      JQ[2][i19] = (-1.357233e-03) * (1) + (-2.122095e-05) * ((2) * q[i19]) + (-1.921777e-03) * (q[i0]) + (-8.235971e-04) * (q[i1]) + (-2.725771e-03) * (q[i2])
            + (-3.041974e-03) * (q[i3]) + (1.584654e-03) * (q[i4]) + (-6.688310e-04) * (q[i5]) + (-5.073938e-04) * (q[i6]) + (-9.229792e-04) * (q[i7])
            + (-2.190352e-03) * (q[i8]) + (7.876256e-05) * (q[i9]) + (1.160841e-03) * (q[i10]) + (-9.842817e-04) * (q[i11]) + (7.347940e-04) * (q[i12])
            + (6.894907e-04) * (q[i15]) + (-8.045054e-04) * (q[i16]) + (5.521720e-04) * (q[i20]) + (1.012404e-03) * (q[i21]) + (-1.535273e-04) * (q[i22])
            + (8.855494e-05) * (q[i0] * q[i0]) + (7.919462e-04) * (q[i1] * q[i1]) + (-6.277662e-04) * (q[i2] * q[i2]) + (1.215654e-03) * (q[i3] * q[i3])
            + (-4.610605e-04) * (q[i4] * q[i4]) + (-1.718998e-03) * (q[i5] * q[i5]) + (-1.054806e-03) * (q[i6] * q[i6]) + (-4.924212e-05) * (q[i7] * q[i7])
            + (-1.430933e-05) * (q[i8] * q[i8]) + (1.068099e-04) * (q[i9] * q[i9]) + (-2.521337e-04) * (q[i10] * q[i10]) + (6.309429e-04) * (q[i11] * q[i11])
            + (-1.360159e-05) * (q[i12] * q[i12]) + (1.617819e-04) * (q[i15] * q[i15]) + (-3.098556e-04) * (q[i16] * q[i16])
            + (-1.345468e-04) * ((2) * q[i0] * q[i19]) + (-2.431014e-04) * ((2) * q[i1] * q[i19]) + (3.508930e-06) * ((2) * q[i2] * q[i19])
            + (-3.874298e-04) * ((2) * q[i3] * q[i19]) + (1.838183e-04) * ((2) * q[i4] * q[i19]) + (-8.537189e-04) * ((2) * q[i5] * q[i19])
            + (-6.359441e-04) * ((2) * q[i6] * q[i19]) + (3.946263e-04) * ((2) * q[i7] * q[i19]) + (-4.900689e-04) * ((2) * q[i8] * q[i19])
            + (-1.551092e-04) * ((2) * q[i9] * q[i19]) + (-1.400187e-04) * ((2) * q[i10] * q[i19]) + (-3.553330e-04) * ((2) * q[i11] * q[i19])
            + (1.844954e-04) * ((2) * q[i12] * q[i19]) + (-1.633945e-04) * ((2) * q[i15] * q[i19]) + (-4.874503e-05) * ((2) * q[i16] * q[i19])
            + (2.056915e-04) * ((3) * q[i19] * q[i19]) + (5.381839e-05) * ((2) * q[i19] * q[i20]) + (-3.067863e-04) * ((2) * q[i19] * q[i21])
            + (4.524680e-05) * ((2) * q[i19] * q[i22]) + (5.310574e-05) * (q[i20] * q[i20]) + (5.306266e-04) * (q[i21] * q[i21])
            + (-1.313933e-04) * (q[i22] * q[i22]) + (-2.149220e-03) * (q[i0] * q[i1]) + (-5.308325e-04) * (q[i0] * q[i2]) + (-1.283668e-03) * (q[i0] * q[i3])
            + (-5.715602e-06) * (q[i0] * q[i4]) + (6.490070e-04) * (q[i0] * q[i5]) + (-1.498094e-03) * (q[i0] * q[i6]) + (-1.286637e-03) * (q[i0] * q[i7])
            + (-3.694046e-04) * (q[i0] * q[i8]) + (-1.799168e-04) * (q[i0] * q[i9]) + (2.148813e-04) * (q[i0] * q[i10]) + (5.389142e-04) * (q[i0] * q[i11])
            + (-5.457165e-04) * (q[i0] * q[i12]) + (-9.731131e-05) * (q[i0] * q[i15]) + (-5.863536e-04) * (q[i0] * q[i16]) + (3.099871e-04) * (q[i0] * q[i20])
            + (2.827223e-04) * (q[i0] * q[i21]) + (2.580421e-04) * (q[i0] * q[i22]) + (-7.448692e-04) * (q[i1] * q[i2]) + (-6.588636e-04) * (q[i1] * q[i3])
            + (-1.337410e-03) * (q[i1] * q[i4]) + (-5.729034e-05) * (q[i1] * q[i5]) + (-2.937084e-04) * (q[i1] * q[i6]) + (-7.119287e-04) * (q[i1] * q[i7])
            + (2.228837e-04) * (q[i1] * q[i8]) + (4.403208e-04) * (q[i1] * q[i9]) + (-5.613018e-04) * (q[i1] * q[i10]) + (6.352125e-05) * (q[i1] * q[i11])
            + (4.266644e-04) * (q[i1] * q[i12]) + (-1.743718e-04) * (q[i1] * q[i15]) + (1.262092e-04) * (q[i1] * q[i16]) + (-3.021009e-04) * (q[i1] * q[i20])
            + (-2.477809e-05) * (q[i1] * q[i21]) + (-1.043606e-04) * (q[i1] * q[i22]) + (1.276545e-04) * (q[i2] * q[i3]) + (8.152563e-05) * (q[i2] * q[i4])
            + (1.439647e-04) * (q[i2] * q[i5]) + (-3.549272e-04) * (q[i2] * q[i6]) + (-7.598168e-04) * (q[i2] * q[i7]) + (1.023466e-03) * (q[i2] * q[i8])
            + (-2.610005e-04) * (q[i2] * q[i9]) + (-1.011537e-04) * (q[i2] * q[i10]) + (6.703634e-04) * (q[i2] * q[i11]) + (-4.444471e-04) * (q[i2] * q[i12])
            + (-3.347044e-04) * (q[i2] * q[i15]) + (-1.355870e-03) * (q[i2] * q[i16]) + (9.810085e-06) * (q[i2] * q[i20]) + (-6.799983e-04) * (q[i2] * q[i21])
            + (-8.525243e-05) * (q[i2] * q[i22]) + (1.436047e-03) * (q[i3] * q[i4]) + (-1.447273e-03) * (q[i3] * q[i5]) + (5.983288e-04) * (q[i3] * q[i6])
            + (2.271741e-04) * (q[i3] * q[i7]) + (-1.824331e-04) * (q[i3] * q[i8]) + (3.544132e-04) * (q[i3] * q[i9]) + (2.506987e-04) * (q[i3] * q[i10])
            + (-1.712573e-04) * (q[i3] * q[i11]) + (8.904234e-05) * (q[i3] * q[i12]) + (-1.090363e-03) * (q[i3] * q[i15]) + (-4.696699e-04) * (q[i3] * q[i16])
            + (7.589383e-04) * (q[i3] * q[i20]) + (7.291109e-05) * (q[i3] * q[i21]) + (2.006097e-04) * (q[i3] * q[i22]) + (2.752440e-04) * (q[i4] * q[i5])
            + (-1.344769e-04) * (q[i4] * q[i6]) + (5.267683e-04) * (q[i4] * q[i7]) + (-9.381795e-05) * (q[i4] * q[i8]) + (-6.228559e-04) * (q[i4] * q[i9])
            + (6.283218e-04) * (q[i4] * q[i10]) + (-1.119821e-04) * (q[i4] * q[i11]) + (-7.938514e-04) * (q[i4] * q[i12]) + (-1.936850e-04) * (q[i4] * q[i15])
            + (3.716815e-04) * (q[i4] * q[i16]) + (-7.567033e-04) * (q[i4] * q[i20]) + (8.571854e-04) * (q[i4] * q[i21]) + (-7.366768e-04) * (q[i4] * q[i22])
            + (-9.828295e-04) * (q[i5] * q[i6]) + (2.210279e-04) * (q[i5] * q[i7]) + (1.505927e-04) * (q[i5] * q[i8]) + (6.817273e-04) * (q[i5] * q[i9])
            + (-2.698364e-04) * (q[i5] * q[i10]) + (-3.721245e-04) * (q[i5] * q[i11]) + (3.623418e-04) * (q[i5] * q[i12]) + (-4.229998e-04) * (q[i5] * q[i15])
            + (4.420311e-04) * (q[i5] * q[i16]) + (2.482564e-06) * (q[i5] * q[i20]) + (6.058628e-04) * (q[i5] * q[i21]) + (9.127125e-05) * (q[i5] * q[i22])
            + (2.586580e-04) * (q[i6] * q[i7]) + (-5.448643e-05) * (q[i6] * q[i8]) + (1.934901e-04) * (q[i6] * q[i9]) + (-1.757855e-05) * (q[i6] * q[i10])
            + (-1.640755e-04) * (q[i6] * q[i11]) + (-9.819785e-04) * (q[i6] * q[i12]) + (-6.097696e-05) * (q[i6] * q[i15]) + (1.010537e-03) * (q[i6] * q[i16])
            + (-3.891080e-04) * (q[i6] * q[i20]) + (3.961905e-04) * (q[i6] * q[i21]) + (-1.491139e-04) * (q[i6] * q[i22]) + (-2.197780e-04) * (q[i7] * q[i8])
            + (4.661810e-04) * (q[i7] * q[i9]) + (-2.948753e-04) * (q[i7] * q[i10]) + (-7.524544e-05) * (q[i7] * q[i11]) + (2.504454e-04) * (q[i7] * q[i12])
            + (-2.515684e-04) * (q[i7] * q[i15]) + (-4.187241e-04) * (q[i7] * q[i16]) + (-3.844589e-04) * (q[i7] * q[i20]) + (-2.231712e-04) * (q[i7] * q[i21])
            + (-1.014804e-04) * (q[i7] * q[i22]) + (-1.399525e-04) * (q[i8] * q[i9]) + (8.625016e-04) * (q[i8] * q[i10]) + (7.583491e-04) * (q[i8] * q[i11])
            + (-1.190358e-03) * (q[i8] * q[i12]) + (1.797364e-04) * (q[i8] * q[i15]) + (-7.659774e-04) * (q[i8] * q[i16]) + (6.328515e-04) * (q[i8] * q[i20])
            + (-1.120134e-03) * (q[i8] * q[i21]) + (-4.248339e-04) * (q[i8] * q[i22]) + (3.917550e-05) * (q[i9] * q[i10]) + (1.100914e-04) * (q[i9] * q[i11])
            + (1.687963e-04) * (q[i9] * q[i12]) + (-1.205072e-04) * (q[i9] * q[i15]) + (4.823402e-04) * (q[i9] * q[i16]) + (-1.338109e-04) * (q[i9] * q[i20])
            + (1.220038e-04) * (q[i9] * q[i21]) + (1.010255e-04) * (q[i9] * q[i22]) + (-3.946347e-04) * (q[i10] * q[i11]) + (-5.485815e-04) * (q[i10] * q[i12])
            + (2.416269e-04) * (q[i10] * q[i15]) + (2.265096e-04) * (q[i10] * q[i16]) + (-1.316243e-04) * (q[i10] * q[i20]) + (1.747631e-04) * (q[i10] * q[i21])
            + (-1.190671e-04) * (q[i10] * q[i22]) + (-2.141351e-04) * (q[i11] * q[i12]) + (1.222465e-04) * (q[i11] * q[i15])
            + (-5.783379e-04) * (q[i11] * q[i16]) + (1.460629e-04) * (q[i11] * q[i20]) + (1.258352e-04) * (q[i11] * q[i21])
            + (-9.818217e-05) * (q[i11] * q[i22]) + (9.948236e-05) * (q[i12] * q[i15]) + (5.335886e-04) * (q[i12] * q[i16])
            + (-1.457333e-04) * (q[i12] * q[i20]) + (-1.774228e-04) * (q[i12] * q[i21]) + (-8.433190e-05) * (q[i12] * q[i22])
            + (2.676551e-04) * (q[i15] * q[i16]) + (9.631664e-05) * (q[i15] * q[i20]) + (-8.679254e-04) * (q[i15] * q[i21])
            + (-2.501875e-04) * (q[i15] * q[i22]) + (9.661418e-05) * (q[i16] * q[i20]) + (9.483944e-05) * (q[i16] * q[i21]) + (1.345618e-04) * (q[i16] * q[i22])
            + (3.544925e-04) * (q[i20] * q[i21]) + (-3.544988e-04) * (q[i20] * q[i22]) + (3.561850e-06) * (q[i21] * q[i22]);
   }

   public void getJQy20(double[] q, double[][] JQ)
   {
      JQ[2][i20] = (-1.260901e-03) * (1) + (-7.609279e-06) * ((2) * q[i20]) + (8.267937e-04) * (q[i0]) + (1.904894e-03) * (q[i1]) + (2.709918e-03) * (q[i2])
            + (-1.632224e-03) * (q[i3]) + (3.021591e-03) * (q[i4]) + (6.817503e-04) * (q[i5]) + (-9.342910e-04) * (q[i6]) + (-4.789234e-04) * (q[i7])
            + (-2.185457e-03) * (q[i8]) + (1.164829e-03) * (q[i9]) + (6.927503e-05) * (q[i10]) + (-7.222546e-04) * (q[i11]) + (9.576683e-04) * (q[i12])
            + (-7.798828e-04) * (q[i15]) + (6.796655e-04) * (q[i16]) + (5.521720e-04) * (q[i19]) + (1.546839e-04) * (q[i21]) + (-1.037990e-03) * (q[i22])
            + (7.927038e-04) * (q[i0] * q[i0]) + (9.471634e-05) * (q[i1] * q[i1]) + (-6.472335e-04) * (q[i2] * q[i2]) + (-4.721584e-04) * (q[i3] * q[i3])
            + (1.217230e-03) * (q[i4] * q[i4]) + (-1.704766e-03) * (q[i5] * q[i5]) + (-5.433964e-05) * (q[i6] * q[i6]) + (-1.049889e-03) * (q[i7] * q[i7])
            + (-2.542223e-05) * (q[i8] * q[i8]) + (-2.559619e-04) * (q[i9] * q[i9]) + (1.029168e-04) * (q[i10] * q[i10]) + (-2.318441e-05) * (q[i11] * q[i11])
            + (6.134458e-04) * (q[i12] * q[i12]) + (-3.049154e-04) * (q[i15] * q[i15]) + (1.564144e-04) * (q[i16] * q[i16]) + (5.381839e-05) * (q[i19] * q[i19])
            + (2.372272e-04) * ((2) * q[i0] * q[i20]) + (1.360576e-04) * ((2) * q[i1] * q[i20]) + (-2.837281e-06) * ((2) * q[i2] * q[i20])
            + (-1.862724e-04) * ((2) * q[i3] * q[i20]) + (3.691705e-04) * ((2) * q[i4] * q[i20]) + (8.604417e-04) * ((2) * q[i5] * q[i20])
            + (3.957655e-04) * ((2) * q[i6] * q[i20]) + (-6.289636e-04) * ((2) * q[i7] * q[i20]) + (-4.881255e-04) * ((2) * q[i8] * q[i20])
            + (-1.402449e-04) * ((2) * q[i9] * q[i20]) + (-1.564151e-04) * ((2) * q[i10] * q[i20]) + (-1.814366e-04) * ((2) * q[i11] * q[i20])
            + (3.580127e-04) * ((2) * q[i12] * q[i20]) + (-5.054305e-05) * ((2) * q[i15] * q[i20]) + (-1.522340e-04) * ((2) * q[i16] * q[i20])
            + (5.310574e-05) * ((2) * q[i19] * q[i20]) + (2.032525e-04) * ((3) * q[i20] * q[i20]) + (-4.372817e-05) * ((2) * q[i20] * q[i21])
            + (3.035206e-04) * ((2) * q[i20] * q[i22]) + (-1.304610e-04) * (q[i21] * q[i21]) + (5.409582e-04) * (q[i22] * q[i22])
            + (-2.164970e-03) * (q[i0] * q[i1]) + (-7.462804e-04) * (q[i0] * q[i2]) + (-1.352818e-03) * (q[i0] * q[i3]) + (-6.549558e-04) * (q[i0] * q[i4])
            + (-3.652971e-05) * (q[i0] * q[i5]) + (7.182522e-04) * (q[i0] * q[i6]) + (3.037250e-04) * (q[i0] * q[i7]) + (-2.317022e-04) * (q[i0] * q[i8])
            + (5.617884e-04) * (q[i0] * q[i9]) + (-4.319454e-04) * (q[i0] * q[i10]) + (4.192410e-04) * (q[i0] * q[i11]) + (6.830868e-05) * (q[i0] * q[i12])
            + (-1.329828e-04) * (q[i0] * q[i15]) + (1.775150e-04) * (q[i0] * q[i16]) + (3.099871e-04) * (q[i0] * q[i19]) + (-1.022213e-04) * (q[i0] * q[i21])
            + (-2.220662e-05) * (q[i0] * q[i22]) + (-5.387324e-04) * (q[i1] * q[i2]) + (1.403780e-05) * (q[i1] * q[i3]) + (-1.289726e-03) * (q[i1] * q[i4])
            + (6.226303e-04) * (q[i1] * q[i5]) + (1.270226e-03) * (q[i1] * q[i6]) + (1.484588e-03) * (q[i1] * q[i7]) + (3.807387e-04) * (q[i1] * q[i8])
            + (-2.157064e-04) * (q[i1] * q[i9]) + (1.733368e-04) * (q[i1] * q[i10]) + (-5.394431e-04) * (q[i1] * q[i11]) + (5.463806e-04) * (q[i1] * q[i12])
            + (5.877674e-04) * (q[i1] * q[i15]) + (8.947500e-05) * (q[i1] * q[i16]) + (-3.021009e-04) * (q[i1] * q[i19]) + (2.473619e-04) * (q[i1] * q[i21])
            + (2.811799e-04) * (q[i1] * q[i22]) + (6.979532e-05) * (q[i2] * q[i3]) + (1.180434e-04) * (q[i2] * q[i4]) + (1.530245e-04) * (q[i2] * q[i5])
            + (7.624220e-04) * (q[i2] * q[i6]) + (3.408859e-04) * (q[i2] * q[i7]) + (-1.027806e-03) * (q[i2] * q[i8]) + (9.740003e-05) * (q[i2] * q[i9])
            + (2.619118e-04) * (q[i2] * q[i10]) + (-4.443000e-04) * (q[i2] * q[i11]) + (6.501908e-04) * (q[i2] * q[i12]) + (1.346695e-03) * (q[i2] * q[i15])
            + (3.529748e-04) * (q[i2] * q[i16]) + (9.810085e-06) * (q[i2] * q[i19]) + (-9.238074e-05) * (q[i2] * q[i21]) + (-6.741689e-04) * (q[i2] * q[i22])
            + (1.413824e-03) * (q[i3] * q[i4]) + (2.859915e-04) * (q[i3] * q[i5]) + (-5.325464e-04) * (q[i3] * q[i6]) + (1.320482e-04) * (q[i3] * q[i7])
            + (9.336729e-05) * (q[i3] * q[i8]) + (-6.354895e-04) * (q[i3] * q[i9]) + (6.488983e-04) * (q[i3] * q[i10]) + (-8.019534e-04) * (q[i3] * q[i11])
            + (-1.091851e-04) * (q[i3] * q[i12]) + (-3.773423e-04) * (q[i3] * q[i15]) + (1.838969e-04) * (q[i3] * q[i16]) + (7.589383e-04) * (q[i3] * q[i19])
            + (-7.413924e-04) * (q[i3] * q[i21]) + (8.525907e-04) * (q[i3] * q[i22]) + (-1.445958e-03) * (q[i4] * q[i5]) + (-2.211296e-04) * (q[i4] * q[i6])
            + (-6.071990e-04) * (q[i4] * q[i7]) + (1.747690e-04) * (q[i4] * q[i8]) + (-2.543523e-04) * (q[i4] * q[i9]) + (-3.471171e-04) * (q[i4] * q[i10])
            + (9.979069e-05) * (q[i4] * q[i11]) + (-1.668851e-04) * (q[i4] * q[i12]) + (4.801148e-04) * (q[i4] * q[i15]) + (1.078294e-03) * (q[i4] * q[i16])
            + (-7.567033e-04) * (q[i4] * q[i19]) + (1.948996e-04) * (q[i4] * q[i21]) + (7.178784e-05) * (q[i4] * q[i22]) + (-2.404216e-04) * (q[i5] * q[i6])
            + (9.826121e-04) * (q[i5] * q[i7]) + (-1.445753e-04) * (q[i5] * q[i8]) + (2.758747e-04) * (q[i5] * q[i9]) + (-7.060816e-04) * (q[i5] * q[i10])
            + (3.566828e-04) * (q[i5] * q[i11]) + (-3.867339e-04) * (q[i5] * q[i12]) + (-4.258683e-04) * (q[i5] * q[i15]) + (4.188070e-04) * (q[i5] * q[i16])
            + (2.482564e-06) * (q[i5] * q[i19]) + (9.610038e-05) * (q[i5] * q[i21]) + (6.030247e-04) * (q[i5] * q[i22]) + (2.497187e-04) * (q[i6] * q[i7])
            + (-2.203541e-04) * (q[i6] * q[i8]) + (-2.956798e-04) * (q[i6] * q[i9]) + (4.630654e-04) * (q[i6] * q[i10]) + (-2.434679e-04) * (q[i6] * q[i11])
            + (8.291197e-05) * (q[i6] * q[i12]) + (-4.208301e-04) * (q[i6] * q[i15]) + (-2.576356e-04) * (q[i6] * q[i16]) + (-3.891080e-04) * (q[i6] * q[i19])
            + (1.049546e-04) * (q[i6] * q[i21]) + (2.197945e-04) * (q[i6] * q[i22]) + (-3.898825e-05) * (q[i7] * q[i8]) + (-2.378912e-05) * (q[i7] * q[i9])
            + (1.864975e-04) * (q[i7] * q[i10]) + (9.860693e-04) * (q[i7] * q[i11]) + (1.631307e-04) * (q[i7] * q[i12]) + (1.008449e-03) * (q[i7] * q[i15])
            + (-4.536671e-05) * (q[i7] * q[i16]) + (-3.844589e-04) * (q[i7] * q[i19]) + (1.437196e-04) * (q[i7] * q[i21]) + (-3.856598e-04) * (q[i7] * q[i22])
            + (8.570201e-04) * (q[i8] * q[i9]) + (-1.244961e-04) * (q[i8] * q[i10]) + (1.177453e-03) * (q[i8] * q[i11]) + (-7.548895e-04) * (q[i8] * q[i12])
            + (-7.647453e-04) * (q[i8] * q[i15]) + (1.855496e-04) * (q[i8] * q[i16]) + (6.328515e-04) * (q[i8] * q[i19]) + (4.221432e-04) * (q[i8] * q[i21])
            + (1.111237e-03) * (q[i8] * q[i22]) + (4.137510e-05) * (q[i9] * q[i10]) + (5.449230e-04) * (q[i9] * q[i11]) + (3.931066e-04) * (q[i9] * q[i12])
            + (2.208531e-04) * (q[i9] * q[i15]) + (2.434512e-04) * (q[i9] * q[i16]) + (-1.338109e-04) * (q[i9] * q[i19]) + (1.195072e-04) * (q[i9] * q[i21])
            + (-1.742876e-04) * (q[i9] * q[i22]) + (-1.611441e-04) * (q[i10] * q[i11]) + (-1.178500e-04) * (q[i10] * q[i12])
            + (4.802103e-04) * (q[i10] * q[i15]) + (-1.238974e-04) * (q[i10] * q[i16]) + (-1.316243e-04) * (q[i10] * q[i19])
            + (-9.656492e-05) * (q[i10] * q[i21]) + (-1.132204e-04) * (q[i10] * q[i22]) + (-2.115579e-04) * (q[i11] * q[i12])
            + (-5.286889e-04) * (q[i11] * q[i15]) + (-1.003911e-04) * (q[i11] * q[i16]) + (1.460629e-04) * (q[i11] * q[i19])
            + (-8.726795e-05) * (q[i11] * q[i21]) + (-1.764124e-04) * (q[i11] * q[i22]) + (5.750484e-04) * (q[i12] * q[i15])
            + (-1.164256e-04) * (q[i12] * q[i16]) + (-1.457333e-04) * (q[i12] * q[i19]) + (-9.988231e-05) * (q[i12] * q[i21])
            + (1.307902e-04) * (q[i12] * q[i22]) + (2.627945e-04) * (q[i15] * q[i16]) + (9.631664e-05) * (q[i15] * q[i19]) + (-1.314745e-04) * (q[i15] * q[i21])
            + (-9.374580e-05) * (q[i15] * q[i22]) + (9.661418e-05) * (q[i16] * q[i19]) + (2.487113e-04) * (q[i16] * q[i21]) + (8.598867e-04) * (q[i16] * q[i22])
            + (3.544925e-04) * (q[i19] * q[i21]) + (-3.544988e-04) * (q[i19] * q[i22]) + (3.144747e-06) * (q[i21] * q[i22]);
   }

   public void getJQy21(double[] q, double[][] JQ)
   {
      JQ[2][i21] = (5.083198e-03) * (1) + (-1.589365e-04) * ((2) * q[i21]) + (2.222365e-03) * (q[i0]) + (-8.990263e-04) * (q[i1]) + (1.988143e-03) * (q[i2])
            + (-2.054358e-03) * (q[i3]) + (1.778881e-03) * (q[i4]) + (1.007175e-03) * (q[i5]) + (-2.525261e-04) * (q[i6]) + (-3.041800e-04) * (q[i7])
            + (-2.085760e-03) * (q[i8]) + (-7.003847e-04) * (q[i9]) + (-4.558118e-04) * (q[i10]) + (-1.187627e-03) * (q[i11]) + (3.132506e-05) * (q[i12])
            + (6.160006e-04) * (q[i15]) + (2.968374e-04) * (q[i16]) + (1.012404e-03) * (q[i19]) + (1.546839e-04) * (q[i20]) + (2.020928e-04) * (q[i22])
            + (1.487935e-04) * (q[i0] * q[i0]) + (-2.343892e-04) * (q[i1] * q[i1]) + (7.020029e-04) * (q[i2] * q[i2]) + (-7.272412e-04) * (q[i3] * q[i3])
            + (6.508448e-04) * (q[i4] * q[i4]) + (-1.473637e-03) * (q[i5] * q[i5]) + (-6.000541e-04) * (q[i6] * q[i6]) + (-1.035763e-03) * (q[i7] * q[i7])
            + (-5.663762e-04) * (q[i8] * q[i8]) + (5.934105e-06) * (q[i9] * q[i9]) + (2.119792e-04) * (q[i10] * q[i10]) + (1.754330e-04) * (q[i11] * q[i11])
            + (-1.951116e-04) * (q[i12] * q[i12]) + (-1.976457e-04) * (q[i15] * q[i15]) + (-3.724932e-06) * (q[i16] * q[i16])
            + (-3.067863e-04) * (q[i19] * q[i19]) + (-4.372817e-05) * (q[i20] * q[i20]) + (1.887070e-04) * ((2) * q[i0] * q[i21])
            + (-6.073279e-05) * ((2) * q[i1] * q[i21]) + (3.209924e-04) * ((2) * q[i2] * q[i21]) + (-2.119539e-04) * ((2) * q[i3] * q[i21])
            + (5.086029e-04) * ((2) * q[i4] * q[i21]) + (-4.984050e-04) * ((2) * q[i5] * q[i21]) + (1.102790e-04) * ((2) * q[i6] * q[i21])
            + (6.938603e-04) * ((2) * q[i7] * q[i21]) + (-7.177874e-04) * ((2) * q[i8] * q[i21]) + (-3.988740e-05) * ((2) * q[i9] * q[i21])
            + (1.020322e-04) * ((2) * q[i10] * q[i21]) + (-4.051638e-04) * ((2) * q[i11] * q[i21]) + (-2.526193e-05) * ((2) * q[i12] * q[i21])
            + (-1.613204e-04) * ((2) * q[i15] * q[i21]) + (1.226978e-04) * ((2) * q[i16] * q[i21]) + (5.306266e-04) * ((2) * q[i19] * q[i21])
            + (-1.304610e-04) * ((2) * q[i20] * q[i21]) + (-8.453739e-05) * ((3) * q[i21] * q[i21]) + (5.480708e-05) * ((2) * q[i21] * q[i22])
            + (-5.651128e-05) * (q[i22] * q[i22]) + (1.059114e-03) * (q[i0] * q[i1]) + (1.166162e-03) * (q[i0] * q[i2]) + (-4.281597e-05) * (q[i0] * q[i3])
            + (-1.410528e-04) * (q[i0] * q[i4]) + (-4.205140e-04) * (q[i0] * q[i5]) + (5.736209e-04) * (q[i0] * q[i6]) + (-4.422425e-04) * (q[i0] * q[i7])
            + (2.642182e-05) * (q[i0] * q[i8]) + (-7.818176e-05) * (q[i0] * q[i9]) + (-1.584458e-04) * (q[i0] * q[i10]) + (2.893217e-04) * (q[i0] * q[i11])
            + (-5.167213e-04) * (q[i0] * q[i12]) + (3.266881e-04) * (q[i0] * q[i15]) + (-2.073675e-04) * (q[i0] * q[i16]) + (2.827223e-04) * (q[i0] * q[i19])
            + (-1.022213e-04) * (q[i0] * q[i20]) + (-5.595093e-04) * (q[i0] * q[i22]) + (1.664055e-03) * (q[i1] * q[i2]) + (6.214166e-04) * (q[i1] * q[i3])
            + (7.876661e-04) * (q[i1] * q[i4]) + (-6.467642e-04) * (q[i1] * q[i5]) + (-3.794178e-04) * (q[i1] * q[i6]) + (-8.684813e-04) * (q[i1] * q[i7])
            + (-4.667108e-04) * (q[i1] * q[i8]) + (-2.141346e-04) * (q[i1] * q[i9]) + (-6.546301e-05) * (q[i1] * q[i10]) + (3.606199e-04) * (q[i1] * q[i11])
            + (-3.924730e-04) * (q[i1] * q[i12]) + (1.723671e-04) * (q[i1] * q[i15]) + (1.137587e-04) * (q[i1] * q[i16]) + (-2.477809e-05) * (q[i1] * q[i19])
            + (2.473619e-04) * (q[i1] * q[i20]) + (5.531782e-04) * (q[i1] * q[i22]) + (-6.013862e-04) * (q[i2] * q[i3]) + (-5.871365e-04) * (q[i2] * q[i4])
            + (2.058515e-03) * (q[i2] * q[i5]) + (4.757560e-04) * (q[i2] * q[i6]) + (-7.253175e-04) * (q[i2] * q[i7]) + (4.401218e-04) * (q[i2] * q[i8])
            + (-4.083199e-04) * (q[i2] * q[i9]) + (-3.418190e-04) * (q[i2] * q[i10]) + (9.345787e-04) * (q[i2] * q[i11]) + (-1.204995e-03) * (q[i2] * q[i12])
            + (7.605788e-04) * (q[i2] * q[i15]) + (4.025737e-05) * (q[i2] * q[i16]) + (-6.799983e-04) * (q[i2] * q[i19]) + (-9.238074e-05) * (q[i2] * q[i20])
            + (-1.328435e-06) * (q[i2] * q[i22]) + (-1.400512e-03) * (q[i3] * q[i4]) + (-1.096375e-03) * (q[i3] * q[i5]) + (-1.193203e-03) * (q[i3] * q[i6])
            + (-7.691139e-04) * (q[i3] * q[i7]) + (2.121110e-04) * (q[i3] * q[i8]) + (-2.459064e-04) * (q[i3] * q[i9]) + (1.378146e-05) * (q[i3] * q[i10])
            + (-8.460064e-04) * (q[i3] * q[i11]) + (-2.769233e-05) * (q[i3] * q[i12]) + (-7.025679e-04) * (q[i3] * q[i15]) + (-4.494096e-05) * (q[i3] * q[i16])
            + (7.291109e-05) * (q[i3] * q[i19]) + (-7.413924e-04) * (q[i3] * q[i20]) + (1.085187e-04) * (q[i3] * q[i22]) + (2.063944e-03) * (q[i4] * q[i5])
            + (1.419563e-03) * (q[i4] * q[i6]) + (-2.688400e-04) * (q[i4] * q[i7]) + (4.350437e-04) * (q[i4] * q[i8]) + (2.289478e-04) * (q[i4] * q[i9])
            + (-3.230970e-04) * (q[i4] * q[i10]) + (1.227838e-04) * (q[i4] * q[i11]) + (-4.133740e-04) * (q[i4] * q[i12]) + (2.930438e-04) * (q[i4] * q[i15])
            + (-7.797041e-04) * (q[i4] * q[i16]) + (8.571854e-04) * (q[i4] * q[i19]) + (1.948996e-04) * (q[i4] * q[i20]) + (-1.024263e-04) * (q[i4] * q[i22])
            + (-2.500503e-04) * (q[i5] * q[i6]) + (2.803336e-04) * (q[i5] * q[i7]) + (-5.209222e-05) * (q[i5] * q[i8]) + (-5.875879e-04) * (q[i5] * q[i9])
            + (-2.235920e-04) * (q[i5] * q[i10]) + (-5.596208e-04) * (q[i5] * q[i11]) + (2.657216e-04) * (q[i5] * q[i12]) + (6.511432e-04) * (q[i5] * q[i15])
            + (4.852346e-04) * (q[i5] * q[i16]) + (6.058628e-04) * (q[i5] * q[i19]) + (9.610038e-05) * (q[i5] * q[i20]) + (-2.869781e-06) * (q[i5] * q[i22])
            + (-2.658031e-04) * (q[i6] * q[i7]) + (-4.420825e-04) * (q[i6] * q[i8]) + (-2.630103e-04) * (q[i6] * q[i9]) + (-3.136931e-04) * (q[i6] * q[i10])
            + (-2.957810e-04) * (q[i6] * q[i11]) + (6.580356e-04) * (q[i6] * q[i12]) + (-3.006825e-04) * (q[i6] * q[i15]) + (1.806325e-04) * (q[i6] * q[i16])
            + (3.961905e-04) * (q[i6] * q[i19]) + (1.049546e-04) * (q[i6] * q[i20]) + (3.274916e-04) * (q[i6] * q[i22]) + (1.665584e-04) * (q[i7] * q[i8])
            + (2.457164e-06) * (q[i7] * q[i9]) + (2.096182e-04) * (q[i7] * q[i10]) + (-1.315688e-06) * (q[i7] * q[i11]) + (-6.068143e-04) * (q[i7] * q[i12])
            + (7.773756e-04) * (q[i7] * q[i15]) + (-4.937533e-05) * (q[i7] * q[i16]) + (-2.231712e-04) * (q[i7] * q[i19]) + (1.437196e-04) * (q[i7] * q[i20])
            + (3.294441e-04) * (q[i7] * q[i22]) + (9.173531e-05) * (q[i8] * q[i9]) + (2.951818e-04) * (q[i8] * q[i10]) + (-3.791023e-04) * (q[i8] * q[i11])
            + (-2.082552e-04) * (q[i8] * q[i12]) + (-1.503288e-03) * (q[i8] * q[i15]) + (-2.341260e-04) * (q[i8] * q[i16]) + (-1.120134e-03) * (q[i8] * q[i19])
            + (4.221432e-04) * (q[i8] * q[i20]) + (-4.557013e-04) * (q[i8] * q[i22]) + (7.523850e-05) * (q[i9] * q[i10]) + (-5.803486e-05) * (q[i9] * q[i11])
            + (-2.334586e-05) * (q[i9] * q[i12]) + (5.189756e-06) * (q[i9] * q[i15]) + (-8.190503e-05) * (q[i9] * q[i16]) + (1.220038e-04) * (q[i9] * q[i19])
            + (1.195072e-04) * (q[i9] * q[i20]) + (6.391691e-05) * (q[i9] * q[i22]) + (-1.973640e-06) * (q[i10] * q[i11]) + (8.537272e-05) * (q[i10] * q[i12])
            + (1.516216e-04) * (q[i10] * q[i15]) + (3.186902e-05) * (q[i10] * q[i16]) + (1.747631e-04) * (q[i10] * q[i19]) + (-9.656492e-05) * (q[i10] * q[i20])
            + (6.457753e-05) * (q[i10] * q[i22]) + (3.290286e-06) * (q[i11] * q[i12]) + (-7.061340e-05) * (q[i11] * q[i15]) + (1.560478e-04) * (q[i11] * q[i16])
            + (1.258352e-04) * (q[i11] * q[i19]) + (-8.726795e-05) * (q[i11] * q[i20]) + (-1.611880e-05) * (q[i11] * q[i22])
            + (-4.713812e-05) * (q[i12] * q[i15]) + (1.596939e-04) * (q[i12] * q[i16]) + (-1.774228e-04) * (q[i12] * q[i19])
            + (-9.988231e-05) * (q[i12] * q[i20]) + (1.312943e-05) * (q[i12] * q[i22]) + (6.172918e-05) * (q[i15] * q[i16])
            + (-8.679254e-04) * (q[i15] * q[i19]) + (-1.314745e-04) * (q[i15] * q[i20]) + (-4.223805e-05) * (q[i15] * q[i22])
            + (9.483944e-05) * (q[i16] * q[i19]) + (2.487113e-04) * (q[i16] * q[i20]) + (-4.249707e-05) * (q[i16] * q[i22]) + (3.544925e-04) * (q[i19] * q[i20])
            + (3.561850e-06) * (q[i19] * q[i22]) + (3.144747e-06) * (q[i20] * q[i22]);
   }

   public void getJQy22(double[] q, double[][] JQ)
   {
      JQ[2][i22] = (-5.078924e-03) * (1) + (-1.608665e-04) * ((2) * q[i22]) + (-8.863129e-04) * (q[i0]) + (2.234868e-03) * (q[i1]) + (1.965185e-03) * (q[i2])
            + (1.763675e-03) * (q[i3]) + (-2.025126e-03) * (q[i4]) + (1.007538e-03) * (q[i5]) + (3.042374e-04) * (q[i6]) + (2.464662e-04) * (q[i7])
            + (2.088335e-03) * (q[i8]) + (4.554292e-04) * (q[i9]) + (6.953218e-04) * (q[i10]) + (1.108089e-05) * (q[i11]) + (-1.201508e-03) * (q[i12])
            + (-3.030791e-04) * (q[i15]) + (-6.132847e-04) * (q[i16]) + (-1.535273e-04) * (q[i19]) + (-1.037990e-03) * (q[i20]) + (2.020928e-04) * (q[i21])
            + (2.358285e-04) * (q[i0] * q[i0]) + (-1.491548e-04) * (q[i1] * q[i1]) + (-7.202685e-04) * (q[i2] * q[i2]) + (-6.555301e-04) * (q[i3] * q[i3])
            + (7.148945e-04) * (q[i4] * q[i4]) + (1.495075e-03) * (q[i5] * q[i5]) + (1.043639e-03) * (q[i6] * q[i6]) + (5.882127e-04) * (q[i7] * q[i7])
            + (5.723610e-04) * (q[i8] * q[i8]) + (-2.138896e-04) * (q[i9] * q[i9]) + (-6.543407e-06) * (q[i10] * q[i10]) + (1.907818e-04) * (q[i11] * q[i11])
            + (-1.700646e-04) * (q[i12] * q[i12]) + (9.307695e-07) * (q[i15] * q[i15]) + (2.048794e-04) * (q[i16] * q[i16]) + (4.524680e-05) * (q[i19] * q[i19])
            + (3.035206e-04) * (q[i20] * q[i20]) + (5.480708e-05) * (q[i21] * q[i21]) + (4.910673e-05) * ((2) * q[i0] * q[i22])
            + (-1.891935e-04) * ((2) * q[i1] * q[i22]) + (-3.183774e-04) * ((2) * q[i2] * q[i22]) + (-5.071852e-04) * ((2) * q[i3] * q[i22])
            + (2.075443e-04) * ((2) * q[i4] * q[i22]) + (5.071164e-04) * ((2) * q[i5] * q[i22]) + (6.934395e-04) * ((2) * q[i6] * q[i22])
            + (1.140094e-04) * ((2) * q[i7] * q[i22]) + (-7.183218e-04) * ((2) * q[i8] * q[i22]) + (1.021671e-04) * ((2) * q[i9] * q[i22])
            + (-4.066959e-05) * ((2) * q[i10] * q[i22]) + (2.422408e-05) * ((2) * q[i11] * q[i22]) + (4.178399e-04) * ((2) * q[i12] * q[i22])
            + (1.252246e-04) * ((2) * q[i15] * q[i22]) + (-1.634418e-04) * ((2) * q[i16] * q[i22]) + (-1.313933e-04) * ((2) * q[i19] * q[i22])
            + (5.409582e-04) * ((2) * q[i20] * q[i22]) + (-5.651128e-05) * ((2) * q[i21] * q[i22]) + (8.735746e-05) * ((3) * q[i22] * q[i22])
            + (-1.058629e-03) * (q[i0] * q[i1]) + (-1.663098e-03) * (q[i0] * q[i2]) + (-7.913839e-04) * (q[i0] * q[i3]) + (-6.179272e-04) * (q[i0] * q[i4])
            + (6.642214e-04) * (q[i0] * q[i5]) + (-8.699801e-04) * (q[i0] * q[i6]) + (-3.798863e-04) * (q[i0] * q[i7]) + (-4.572423e-04) * (q[i0] * q[i8])
            + (-6.660250e-05) * (q[i0] * q[i9]) + (-2.132184e-04) * (q[i0] * q[i10]) + (3.909004e-04) * (q[i0] * q[i11]) + (-3.713889e-04) * (q[i0] * q[i12])
            + (1.098482e-04) * (q[i0] * q[i15]) + (1.769337e-04) * (q[i0] * q[i16]) + (2.580421e-04) * (q[i0] * q[i19]) + (-2.220662e-05) * (q[i0] * q[i20])
            + (-5.595093e-04) * (q[i0] * q[i21]) + (-1.179484e-03) * (q[i1] * q[i2]) + (1.400097e-04) * (q[i1] * q[i3]) + (4.832829e-05) * (q[i1] * q[i4])
            + (4.217495e-04) * (q[i1] * q[i5]) + (-4.370185e-04) * (q[i1] * q[i6]) + (5.826807e-04) * (q[i1] * q[i7]) + (3.449386e-05) * (q[i1] * q[i8])
            + (-1.572557e-04) * (q[i1] * q[i9]) + (-7.665282e-05) * (q[i1] * q[i10]) + (5.179043e-04) * (q[i1] * q[i11]) + (-2.773281e-04) * (q[i1] * q[i12])
            + (-2.020610e-04) * (q[i1] * q[i15]) + (3.327511e-04) * (q[i1] * q[i16]) + (-1.043606e-04) * (q[i1] * q[i19]) + (2.811799e-04) * (q[i1] * q[i20])
            + (5.531782e-04) * (q[i1] * q[i21]) + (6.002039e-04) * (q[i2] * q[i3]) + (5.962369e-04) * (q[i2] * q[i4]) + (-2.067730e-03) * (q[i2] * q[i5])
            + (-7.125799e-04) * (q[i2] * q[i6]) + (4.696598e-04) * (q[i2] * q[i7]) + (4.437775e-04) * (q[i2] * q[i8]) + (-3.429924e-04) * (q[i2] * q[i9])
            + (-4.035359e-04) * (q[i2] * q[i10]) + (1.211605e-03) * (q[i2] * q[i11]) + (-9.273983e-04) * (q[i2] * q[i12]) + (3.999834e-05) * (q[i2] * q[i15])
            + (7.495992e-04) * (q[i2] * q[i16]) + (-8.525243e-05) * (q[i2] * q[i19]) + (-6.741689e-04) * (q[i2] * q[i20]) + (-1.328435e-06) * (q[i2] * q[i21])
            + (1.412147e-03) * (q[i3] * q[i4]) + (-2.078576e-03) * (q[i3] * q[i5]) + (-2.469359e-04) * (q[i3] * q[i6]) + (1.421559e-03) * (q[i3] * q[i7])
            + (4.277614e-04) * (q[i3] * q[i8]) + (-3.224698e-04) * (q[i3] * q[i9]) + (2.411005e-04) * (q[i3] * q[i10]) + (4.242010e-04) * (q[i3] * q[i11])
            + (-1.330506e-04) * (q[i3] * q[i12]) + (-7.839491e-04) * (q[i3] * q[i15]) + (2.849656e-04) * (q[i3] * q[i16]) + (2.006097e-04) * (q[i3] * q[i19])
            + (8.525907e-04) * (q[i3] * q[i20]) + (1.085187e-04) * (q[i3] * q[i21]) + (1.112460e-03) * (q[i4] * q[i5]) + (-7.808593e-04) * (q[i4] * q[i6])
            + (-1.188431e-03) * (q[i4] * q[i7]) + (2.177448e-04) * (q[i4] * q[i8]) + (-4.351189e-08) * (q[i4] * q[i9]) + (-2.538713e-04) * (q[i4] * q[i10])
            + (2.703891e-05) * (q[i4] * q[i11]) + (8.567602e-04) * (q[i4] * q[i12]) + (-5.318361e-05) * (q[i4] * q[i15]) + (-7.098311e-04) * (q[i4] * q[i16])
            + (-7.366768e-04) * (q[i4] * q[i19]) + (7.178784e-05) * (q[i4] * q[i20]) + (-1.024263e-04) * (q[i4] * q[i21]) + (2.925324e-04) * (q[i5] * q[i6])
            + (-2.304339e-04) * (q[i5] * q[i7]) + (-4.250972e-05) * (q[i5] * q[i8]) + (-2.189059e-04) * (q[i5] * q[i9]) + (-5.826465e-04) * (q[i5] * q[i10])
            + (-2.649668e-04) * (q[i5] * q[i11]) + (5.715204e-04) * (q[i5] * q[i12]) + (4.829936e-04) * (q[i5] * q[i15]) + (6.692534e-04) * (q[i5] * q[i16])
            + (9.127125e-05) * (q[i5] * q[i19]) + (6.030247e-04) * (q[i5] * q[i20]) + (-2.869781e-06) * (q[i5] * q[i21]) + (2.470120e-04) * (q[i6] * q[i7])
            + (-1.667029e-04) * (q[i6] * q[i8]) + (-2.043316e-04) * (q[i6] * q[i9]) + (-3.502427e-06) * (q[i6] * q[i10]) + (-6.079118e-04) * (q[i6] * q[i11])
            + (3.425389e-07) * (q[i6] * q[i12]) + (6.121058e-05) * (q[i6] * q[i15]) + (-7.857355e-04) * (q[i6] * q[i16]) + (-1.491139e-04) * (q[i6] * q[i19])
            + (2.197945e-04) * (q[i6] * q[i20]) + (3.274916e-04) * (q[i6] * q[i21]) + (4.488181e-04) * (q[i7] * q[i8]) + (3.049735e-04) * (q[i7] * q[i9])
            + (2.627882e-04) * (q[i7] * q[i10]) + (6.556906e-04) * (q[i7] * q[i11]) + (-2.887615e-04) * (q[i7] * q[i12]) + (-1.900134e-04) * (q[i7] * q[i15])
            + (3.086174e-04) * (q[i7] * q[i16]) + (-1.014804e-04) * (q[i7] * q[i19]) + (-3.856598e-04) * (q[i7] * q[i20]) + (3.294441e-04) * (q[i7] * q[i21])
            + (-3.030985e-04) * (q[i8] * q[i9]) + (-9.019046e-05) * (q[i8] * q[i10]) + (-2.098501e-04) * (q[i8] * q[i11]) + (-3.871480e-04) * (q[i8] * q[i12])
            + (2.291151e-04) * (q[i8] * q[i15]) + (1.516622e-03) * (q[i8] * q[i16]) + (-4.248339e-04) * (q[i8] * q[i19]) + (1.111237e-03) * (q[i8] * q[i20])
            + (-4.557013e-04) * (q[i8] * q[i21]) + (-7.457916e-05) * (q[i9] * q[i10]) + (8.276334e-05) * (q[i9] * q[i11]) + (3.840462e-07) * (q[i9] * q[i12])
            + (-3.313817e-05) * (q[i9] * q[i15]) + (-1.549280e-04) * (q[i9] * q[i16]) + (1.010255e-04) * (q[i9] * q[i19]) + (-1.742876e-04) * (q[i9] * q[i20])
            + (6.391691e-05) * (q[i9] * q[i21]) + (-1.972338e-05) * (q[i10] * q[i11]) + (-5.462330e-05) * (q[i10] * q[i12]) + (8.125625e-05) * (q[i10] * q[i15])
            + (-2.368354e-06) * (q[i10] * q[i16]) + (-1.190671e-04) * (q[i10] * q[i19]) + (-1.132204e-04) * (q[i10] * q[i20])
            + (6.457753e-05) * (q[i10] * q[i21]) + (9.393283e-08) * (q[i11] * q[i12]) + (1.524228e-04) * (q[i11] * q[i15]) + (-5.117911e-05) * (q[i11] * q[i16])
            + (-9.818217e-05) * (q[i11] * q[i19]) + (-1.764124e-04) * (q[i11] * q[i20]) + (-1.611880e-05) * (q[i11] * q[i21])
            + (1.608407e-04) * (q[i12] * q[i15]) + (-8.637346e-05) * (q[i12] * q[i16]) + (-8.433190e-05) * (q[i12] * q[i19])
            + (1.307902e-04) * (q[i12] * q[i20]) + (1.312943e-05) * (q[i12] * q[i21]) + (-6.089486e-05) * (q[i15] * q[i16])
            + (-2.501875e-04) * (q[i15] * q[i19]) + (-9.374580e-05) * (q[i15] * q[i20]) + (-4.223805e-05) * (q[i15] * q[i21])
            + (1.345618e-04) * (q[i16] * q[i19]) + (8.598867e-04) * (q[i16] * q[i20]) + (-4.249707e-05) * (q[i16] * q[i21])
            + (-3.544988e-04) * (q[i19] * q[i20]) + (3.561850e-06) * (q[i19] * q[i21]) + (3.144747e-06) * (q[i20] * q[i21]);
   }

   public void getJQz(double[] q, double[][] JQ)
   {
      getJQz0(q, JQ);
      getJQz1(q, JQ);
      getJQz2(q, JQ);
      getJQz3(q, JQ);
      getJQz4(q, JQ);
      getJQz5(q, JQ);
      getJQz6(q, JQ);
      getJQz7(q, JQ);
      getJQz8(q, JQ);
      getJQz9(q, JQ);
      getJQz10(q, JQ);
      getJQz11(q, JQ);
      getJQz12(q, JQ);
      getJQz15(q, JQ);
      getJQz16(q, JQ);
      getJQz19(q, JQ);
      getJQz20(q, JQ);
      getJQz21(q, JQ);
      getJQz22(q, JQ);
   }

   public void getJQz0(double[] q, double[][] JQ)
   {
      JQ[3][i0] = (-1.133065e-01) * (1) + (6.937143e-03) * ((2) * q[i0]) + (6.714252e-06) * (q[i1]) + (-3.677595e-03) * (q[i2]) + (-6.042340e-02) * (q[i3])
            + (-5.138662e-02) * (q[i4]) + (-8.213849e-03) * (q[i5]) + (1.570326e-02) * (q[i6]) + (-1.074346e-02) * (q[i7]) + (8.653143e-03) * (q[i8])
            + (-5.261070e-03) * (q[i9]) + (-1.270144e-03) * (q[i10]) + (6.303760e-04) * (q[i11]) + (-3.051713e-04) * (q[i12]) + (-8.068139e-03) * (q[i15])
            + (-7.002062e-03) * (q[i16]) + (-3.021311e-03) * (q[i19]) + (-2.484627e-05) * (q[i20]) + (-6.986138e-03) * (q[i21]) + (1.111550e-03) * (q[i22])
            + (4.892620e-03) * ((3) * q[i0] * q[i0]) + (-1.963697e-03) * ((2) * q[i0] * q[i1]) + (-1.779915e-03) * ((2) * q[i0] * q[i2])
            + (-7.543770e-03) * ((2) * q[i0] * q[i3]) + (-2.024621e-03) * ((2) * q[i0] * q[i4]) + (1.598918e-04) * ((2) * q[i0] * q[i5])
            + (-1.235529e-02) * ((2) * q[i0] * q[i6]) + (-3.272551e-03) * ((2) * q[i0] * q[i7]) + (-9.077013e-04) * ((2) * q[i0] * q[i8])
            + (-2.269364e-03) * ((2) * q[i0] * q[i9]) + (-2.136750e-03) * ((2) * q[i0] * q[i10]) + (-6.309729e-05) * ((2) * q[i0] * q[i11])
            + (2.519247e-03) * ((2) * q[i0] * q[i12]) + (6.291848e-04) * ((2) * q[i0] * q[i15]) + (-1.373595e-06) * ((2) * q[i0] * q[i16])
            + (-6.879212e-05) * ((2) * q[i0] * q[i19]) + (3.310978e-04) * ((2) * q[i0] * q[i20]) + (-1.531392e-04) * ((2) * q[i0] * q[i21])
            + (4.324533e-04) * ((2) * q[i0] * q[i22]) + (-1.922747e-03) * (q[i1] * q[i1]) + (3.389921e-03) * (q[i2] * q[i2]) + (-6.693411e-03) * (q[i3] * q[i3])
            + (-2.229401e-03) * (q[i4] * q[i4]) + (4.455479e-05) * (q[i5] * q[i5]) + (-1.078988e-02) * (q[i6] * q[i6]) + (6.715776e-03) * (q[i7] * q[i7])
            + (8.763863e-04) * (q[i8] * q[i8]) + (3.571000e-03) * (q[i9] * q[i9]) + (2.137082e-04) * (q[i10] * q[i10]) + (-3.013394e-04) * (q[i11] * q[i11])
            + (-4.926891e-06) * (q[i12] * q[i12]) + (-1.610159e-03) * (q[i15] * q[i15]) + (-2.629202e-03) * (q[i16] * q[i16])
            + (9.614484e-04) * (q[i19] * q[i19]) + (-8.809152e-04) * (q[i20] * q[i20]) + (-1.759633e-03) * (q[i21] * q[i21])
            + (-4.010723e-04) * (q[i22] * q[i22]) + (-1.034438e-03) * (q[i1] * q[i2]) + (1.107523e-03) * (q[i1] * q[i3]) + (1.092826e-03) * (q[i1] * q[i4])
            + (5.936415e-03) * (q[i1] * q[i5]) + (-3.535111e-04) * (q[i1] * q[i6]) + (2.757222e-04) * (q[i1] * q[i7]) + (5.311938e-05) * (q[i1] * q[i8])
            + (-1.650176e-03) * (q[i1] * q[i9]) + (1.641943e-03) * (q[i1] * q[i10]) + (-6.978984e-04) * (q[i1] * q[i11]) + (-6.819984e-04) * (q[i1] * q[i12])
            + (8.307751e-05) * (q[i1] * q[i15]) + (-8.990329e-05) * (q[i1] * q[i16]) + (2.970379e-04) * (q[i1] * q[i19]) + (-3.106547e-04) * (q[i1] * q[i20])
            + (1.096464e-03) * (q[i1] * q[i21]) + (1.088216e-03) * (q[i1] * q[i22]) + (2.092116e-03) * (q[i2] * q[i3]) + (-5.079142e-03) * (q[i2] * q[i4])
            + (7.131349e-03) * (q[i2] * q[i5]) + (-8.584065e-03) * (q[i2] * q[i6]) + (-2.635861e-03) * (q[i2] * q[i7]) + (5.935676e-03) * (q[i2] * q[i8])
            + (-1.593483e-03) * (q[i2] * q[i9]) + (-1.246070e-03) * (q[i2] * q[i10]) + (5.106428e-04) * (q[i2] * q[i11]) + (-8.160733e-04) * (q[i2] * q[i12])
            + (-8.816348e-04) * (q[i2] * q[i15]) + (-1.612249e-03) * (q[i2] * q[i16]) + (-1.778900e-03) * (q[i2] * q[i19]) + (3.204858e-04) * (q[i2] * q[i20])
            + (-7.543829e-05) * (q[i2] * q[i21]) + (1.559269e-03) * (q[i2] * q[i22]) + (1.157804e-02) * (q[i3] * q[i4]) + (-1.175620e-02) * (q[i3] * q[i5])
            + (-1.011579e-02) * (q[i3] * q[i6]) + (3.106750e-03) * (q[i3] * q[i7]) + (5.752914e-03) * (q[i3] * q[i8]) + (6.880723e-03) * (q[i3] * q[i9])
            + (4.104927e-04) * (q[i3] * q[i10]) + (-1.905806e-03) * (q[i3] * q[i11]) + (-4.130384e-03) * (q[i3] * q[i12]) + (3.737213e-03) * (q[i3] * q[i15])
            + (-4.549942e-03) * (q[i3] * q[i16]) + (-7.239892e-04) * (q[i3] * q[i19]) + (-4.420090e-04) * (q[i3] * q[i20]) + (2.352586e-03) * (q[i3] * q[i21])
            + (-2.251364e-03) * (q[i3] * q[i22]) + (3.113038e-04) * (q[i4] * q[i5]) + (2.345123e-03) * (q[i4] * q[i6]) + (-3.091155e-03) * (q[i4] * q[i7])
            + (2.799478e-03) * (q[i4] * q[i8]) + (5.130394e-03) * (q[i4] * q[i9]) + (6.354973e-03) * (q[i4] * q[i10]) + (-4.735293e-04) * (q[i4] * q[i11])
            + (3.386984e-03) * (q[i4] * q[i12]) + (-3.231341e-03) * (q[i4] * q[i15]) + (-1.271616e-04) * (q[i4] * q[i16]) + (-1.119306e-05) * (q[i4] * q[i19])
            + (-5.171096e-04) * (q[i4] * q[i20]) + (-2.774221e-03) * (q[i4] * q[i21]) + (-7.930453e-04) * (q[i4] * q[i22]) + (2.328631e-03) * (q[i5] * q[i6])
            + (-2.363745e-04) * (q[i5] * q[i7]) + (3.065318e-03) * (q[i5] * q[i8]) + (2.392677e-03) * (q[i5] * q[i9]) + (1.078840e-03) * (q[i5] * q[i10])
            + (1.322023e-03) * (q[i5] * q[i11]) + (1.282937e-03) * (q[i5] * q[i12]) + (3.452045e-03) * (q[i5] * q[i15]) + (-2.146369e-03) * (q[i5] * q[i16])
            + (-1.380481e-03) * (q[i5] * q[i19]) + (-5.019998e-04) * (q[i5] * q[i20]) + (-2.871135e-03) * (q[i5] * q[i21]) + (-8.908884e-04) * (q[i5] * q[i22])
            + (7.590054e-03) * (q[i6] * q[i7]) + (3.540452e-03) * (q[i6] * q[i8]) + (-6.566775e-03) * (q[i6] * q[i9]) + (6.778875e-03) * (q[i6] * q[i10])
            + (2.362114e-03) * (q[i6] * q[i11]) + (1.859035e-03) * (q[i6] * q[i12]) + (-3.302023e-04) * (q[i6] * q[i15]) + (2.531603e-03) * (q[i6] * q[i16])
            + (-1.968210e-03) * (q[i6] * q[i19]) + (-7.278929e-04) * (q[i6] * q[i20]) + (-4.359618e-04) * (q[i6] * q[i21]) + (1.950349e-03) * (q[i6] * q[i22])
            + (-1.160031e-03) * (q[i7] * q[i8]) + (2.473974e-04) * (q[i7] * q[i9]) + (5.886241e-03) * (q[i7] * q[i10]) + (-8.944924e-04) * (q[i7] * q[i11])
            + (-6.621534e-04) * (q[i7] * q[i12]) + (1.730123e-03) * (q[i7] * q[i15]) + (-2.987341e-03) * (q[i7] * q[i16]) + (-2.717691e-03) * (q[i7] * q[i19])
            + (-1.157077e-04) * (q[i7] * q[i20]) + (4.351924e-04) * (q[i7] * q[i21]) + (-6.437164e-04) * (q[i7] * q[i22]) + (3.435307e-04) * (q[i8] * q[i9])
            + (-1.449731e-03) * (q[i8] * q[i10]) + (-1.501626e-04) * (q[i8] * q[i11]) + (3.196186e-04) * (q[i8] * q[i12]) + (-1.777304e-03) * (q[i8] * q[i15])
            + (-1.266048e-03) * (q[i8] * q[i16]) + (-9.278645e-04) * (q[i8] * q[i19]) + (2.572781e-03) * (q[i8] * q[i20]) + (5.334901e-04) * (q[i8] * q[i21])
            + (-1.699471e-03) * (q[i8] * q[i22]) + (1.333336e-03) * (q[i9] * q[i10]) + (9.486425e-06) * (q[i9] * q[i11]) + (-4.151017e-04) * (q[i9] * q[i12])
            + (1.080065e-04) * (q[i9] * q[i15]) + (-8.717527e-04) * (q[i9] * q[i16]) + (1.619438e-03) * (q[i9] * q[i19]) + (-3.476660e-04) * (q[i9] * q[i20])
            + (1.984431e-04) * (q[i9] * q[i21]) + (4.594173e-05) * (q[i9] * q[i22]) + (-1.415883e-03) * (q[i10] * q[i11]) + (1.398713e-03) * (q[i10] * q[i12])
            + (-1.365183e-03) * (q[i10] * q[i15]) + (-2.779088e-04) * (q[i10] * q[i16]) + (1.148780e-03) * (q[i10] * q[i19])
            + (-6.231768e-04) * (q[i10] * q[i20]) + (6.428626e-04) * (q[i10] * q[i21]) + (-1.778341e-04) * (q[i10] * q[i22])
            + (5.165199e-04) * (q[i11] * q[i12]) + (-1.382556e-03) * (q[i11] * q[i15]) + (-2.685371e-04) * (q[i11] * q[i16])
            + (1.617007e-03) * (q[i11] * q[i19]) + (-1.578691e-03) * (q[i11] * q[i20]) + (-8.476260e-04) * (q[i11] * q[i21])
            + (-1.043071e-03) * (q[i11] * q[i22]) + (1.106490e-03) * (q[i12] * q[i15]) + (-3.871843e-03) * (q[i12] * q[i16])
            + (6.751019e-04) * (q[i12] * q[i19]) + (2.247609e-03) * (q[i12] * q[i20]) + (1.695129e-03) * (q[i12] * q[i21]) + (6.040034e-04) * (q[i12] * q[i22])
            + (8.868801e-05) * (q[i15] * q[i16]) + (1.068404e-03) * (q[i15] * q[i19]) + (-6.770629e-05) * (q[i15] * q[i20])
            + (-2.242571e-03) * (q[i15] * q[i21]) + (2.860531e-04) * (q[i15] * q[i22]) + (1.225508e-03) * (q[i16] * q[i19])
            + (-9.726985e-04) * (q[i16] * q[i20]) + (8.158969e-05) * (q[i16] * q[i21]) + (2.233752e-04) * (q[i16] * q[i22])
            + (-1.029540e-04) * (q[i19] * q[i20]) + (7.949083e-04) * (q[i19] * q[i21]) + (-6.577718e-04) * (q[i19] * q[i22])
            + (-8.140101e-04) * (q[i20] * q[i21]) + (1.086136e-04) * (q[i20] * q[i22]) + (4.544889e-04) * (q[i21] * q[i22]);
   }

   public void getJQz1(double[] q, double[][] JQ)
   {
      JQ[3][i1] = (-1.127153e-01) * (1) + (-6.826124e-03) * ((2) * q[i1]) + (6.714252e-06) * (q[i0]) + (3.762040e-03) * (q[i2]) + (5.143739e-02) * (q[i3])
            + (6.048093e-02) * (q[i4]) + (8.116596e-03) * (q[i5]) + (-1.079350e-02) * (q[i6]) + (1.575324e-02) * (q[i7]) + (8.756454e-03) * (q[i8])
            + (-1.332474e-03) * (q[i9]) + (-5.206619e-03) * (q[i10]) + (4.869941e-04) * (q[i11]) + (-5.109633e-04) * (q[i12]) + (-6.918457e-03) * (q[i15])
            + (-8.223930e-03) * (q[i16]) + (-1.285072e-05) * (q[i19]) + (-3.012531e-03) * (q[i20]) + (-1.117623e-03) * (q[i21]) + (6.876052e-03) * (q[i22])
            + (-1.963697e-03) * (q[i0] * q[i0]) + (-1.922747e-03) * ((2) * q[i0] * q[i1]) + (4.864441e-03) * ((3) * q[i1] * q[i1])
            + (-1.710495e-03) * ((2) * q[i1] * q[i2]) + (-2.026980e-03) * ((2) * q[i1] * q[i3]) + (-7.512135e-03) * ((2) * q[i1] * q[i4])
            + (1.282382e-04) * ((2) * q[i1] * q[i5]) + (3.284873e-03) * ((2) * q[i1] * q[i6]) + (1.242606e-02) * ((2) * q[i1] * q[i7])
            + (8.847795e-04) * ((2) * q[i1] * q[i8]) + (2.143130e-03) * ((2) * q[i1] * q[i9]) + (2.239776e-03) * ((2) * q[i1] * q[i10])
            + (2.519575e-03) * ((2) * q[i1] * q[i11]) + (-6.138320e-05) * ((2) * q[i1] * q[i12]) + (3.195453e-05) * ((2) * q[i1] * q[i15])
            + (-6.272335e-04) * ((2) * q[i1] * q[i16]) + (-3.195466e-04) * ((2) * q[i1] * q[i19]) + (1.054223e-04) * ((2) * q[i1] * q[i20])
            + (4.403735e-04) * ((2) * q[i1] * q[i21]) + (-1.664605e-04) * ((2) * q[i1] * q[i22]) + (3.420126e-03) * (q[i2] * q[i2])
            + (-2.267421e-03) * (q[i3] * q[i3]) + (-6.662712e-03) * (q[i4] * q[i4]) + (-6.971971e-05) * (q[i5] * q[i5]) + (6.680487e-03) * (q[i6] * q[i6])
            + (-1.079080e-02) * (q[i7] * q[i7]) + (8.621596e-04) * (q[i8] * q[i8]) + (2.252412e-04) * (q[i9] * q[i9]) + (3.526691e-03) * (q[i10] * q[i10])
            + (-1.769956e-05) * (q[i11] * q[i11]) + (-3.530106e-04) * (q[i12] * q[i12]) + (-2.590570e-03) * (q[i15] * q[i15])
            + (-1.635661e-03) * (q[i16] * q[i16]) + (-8.908750e-04) * (q[i19] * q[i19]) + (9.579766e-04) * (q[i20] * q[i20])
            + (-3.901743e-04) * (q[i21] * q[i21]) + (-1.742359e-03) * (q[i22] * q[i22]) + (-1.034438e-03) * (q[i0] * q[i2]) + (1.107523e-03) * (q[i0] * q[i3])
            + (1.092826e-03) * (q[i0] * q[i4]) + (5.936415e-03) * (q[i0] * q[i5]) + (-3.535111e-04) * (q[i0] * q[i6]) + (2.757222e-04) * (q[i0] * q[i7])
            + (5.311938e-05) * (q[i0] * q[i8]) + (-1.650176e-03) * (q[i0] * q[i9]) + (1.641943e-03) * (q[i0] * q[i10]) + (-6.978984e-04) * (q[i0] * q[i11])
            + (-6.819984e-04) * (q[i0] * q[i12]) + (8.307751e-05) * (q[i0] * q[i15]) + (-8.990329e-05) * (q[i0] * q[i16]) + (2.970379e-04) * (q[i0] * q[i19])
            + (-3.106547e-04) * (q[i0] * q[i20]) + (1.096464e-03) * (q[i0] * q[i21]) + (1.088216e-03) * (q[i0] * q[i22]) + (-5.128699e-03) * (q[i2] * q[i3])
            + (2.120396e-03) * (q[i2] * q[i4]) + (7.188995e-03) * (q[i2] * q[i5]) + (2.627176e-03) * (q[i2] * q[i6]) + (8.588029e-03) * (q[i2] * q[i7])
            + (-5.944304e-03) * (q[i2] * q[i8]) + (1.251727e-03) * (q[i2] * q[i9]) + (1.589570e-03) * (q[i2] * q[i10]) + (-8.452529e-04) * (q[i2] * q[i11])
            + (5.137758e-04) * (q[i2] * q[i12]) + (1.617850e-03) * (q[i2] * q[i15]) + (8.778651e-04) * (q[i2] * q[i16]) + (-3.047581e-04) * (q[i2] * q[i19])
            + (1.812038e-03) * (q[i2] * q[i20]) + (1.553031e-03) * (q[i2] * q[i21]) + (-1.092656e-04) * (q[i2] * q[i22]) + (1.164925e-02) * (q[i3] * q[i4])
            + (3.613486e-04) * (q[i3] * q[i5]) + (3.009775e-03) * (q[i3] * q[i6]) + (-2.379827e-03) * (q[i3] * q[i7]) + (-2.802561e-03) * (q[i3] * q[i8])
            + (-6.364877e-03) * (q[i3] * q[i9]) + (-5.111334e-03) * (q[i3] * q[i10]) + (3.375875e-03) * (q[i3] * q[i11]) + (-5.158804e-04) * (q[i3] * q[i12])
            + (1.375503e-04) * (q[i3] * q[i15]) + (3.280968e-03) * (q[i3] * q[i16]) + (5.168160e-04) * (q[i3] * q[i19]) + (-2.998582e-05) * (q[i3] * q[i20])
            + (-7.713824e-04) * (q[i3] * q[i21]) + (-2.770689e-03) * (q[i3] * q[i22]) + (-1.174509e-02) * (q[i4] * q[i5]) + (-3.075405e-03) * (q[i4] * q[i6])
            + (1.026329e-02) * (q[i4] * q[i7]) + (-5.652658e-03) * (q[i4] * q[i8]) + (-4.481237e-04) * (q[i4] * q[i9]) + (-6.823310e-03) * (q[i4] * q[i10])
            + (-4.153900e-03) * (q[i4] * q[i11]) + (-1.935043e-03) * (q[i4] * q[i12]) + (4.452672e-03) * (q[i4] * q[i15]) + (-3.699263e-03) * (q[i4] * q[i16])
            + (4.557667e-04) * (q[i4] * q[i19]) + (6.645825e-04) * (q[i4] * q[i20]) + (-2.249974e-03) * (q[i4] * q[i21]) + (2.335218e-03) * (q[i4] * q[i22])
            + (2.449185e-04) * (q[i5] * q[i6]) + (-2.324987e-03) * (q[i5] * q[i7]) + (-3.085534e-03) * (q[i5] * q[i8]) + (-1.058509e-03) * (q[i5] * q[i9])
            + (-2.381350e-03) * (q[i5] * q[i10]) + (1.337867e-03) * (q[i5] * q[i11]) + (1.235463e-03) * (q[i5] * q[i12]) + (2.119109e-03) * (q[i5] * q[i15])
            + (-3.452865e-03) * (q[i5] * q[i16]) + (5.274652e-04) * (q[i5] * q[i19]) + (1.376363e-03) * (q[i5] * q[i20]) + (-8.784839e-04) * (q[i5] * q[i21])
            + (-2.824797e-03) * (q[i5] * q[i22]) + (7.598896e-03) * (q[i6] * q[i7]) + (-1.137300e-03) * (q[i6] * q[i8]) + (5.926933e-03) * (q[i6] * q[i9])
            + (2.451999e-04) * (q[i6] * q[i10]) + (7.141237e-04) * (q[i6] * q[i11]) + (8.925012e-04) * (q[i6] * q[i12]) + (-2.981604e-03) * (q[i6] * q[i15])
            + (1.727913e-03) * (q[i6] * q[i16]) + (-1.552690e-04) * (q[i6] * q[i19]) + (-2.713295e-03) * (q[i6] * q[i20]) + (6.585911e-04) * (q[i6] * q[i21])
            + (-4.110103e-04) * (q[i6] * q[i22]) + (3.540632e-03) * (q[i7] * q[i8]) + (6.815199e-03) * (q[i7] * q[i9]) + (-6.555638e-03) * (q[i7] * q[i10])
            + (-1.815133e-03) * (q[i7] * q[i11]) + (-2.364231e-03) * (q[i7] * q[i12]) + (2.516036e-03) * (q[i7] * q[i15]) + (-2.919010e-04) * (q[i7] * q[i16])
            + (-7.084220e-04) * (q[i7] * q[i19]) + (-1.945346e-03) * (q[i7] * q[i20]) + (-1.962824e-03) * (q[i7] * q[i21]) + (4.281154e-04) * (q[i7] * q[i22])
            + (-1.458352e-03) * (q[i8] * q[i9]) + (3.259798e-04) * (q[i8] * q[i10]) + (-3.038879e-04) * (q[i8] * q[i11]) + (1.257303e-04) * (q[i8] * q[i12])
            + (-1.220725e-03) * (q[i8] * q[i15]) + (-1.803216e-03) * (q[i8] * q[i16]) + (2.591051e-03) * (q[i8] * q[i19]) + (-9.210163e-04) * (q[i8] * q[i20])
            + (1.685769e-03) * (q[i8] * q[i21]) + (-5.526813e-04) * (q[i8] * q[i22]) + (1.366859e-03) * (q[i9] * q[i10]) + (-1.395787e-03) * (q[i9] * q[i11])
            + (1.418468e-03) * (q[i9] * q[i12]) + (-2.835336e-04) * (q[i9] * q[i15]) + (-1.345912e-03) * (q[i9] * q[i16]) + (-6.331721e-04) * (q[i9] * q[i19])
            + (1.136149e-03) * (q[i9] * q[i20]) + (1.872351e-04) * (q[i9] * q[i21]) + (-6.430545e-04) * (q[i9] * q[i22]) + (4.045033e-04) * (q[i10] * q[i11])
            + (-1.472401e-05) * (q[i10] * q[i12]) + (-8.608620e-04) * (q[i10] * q[i15]) + (1.288937e-04) * (q[i10] * q[i16])
            + (-3.646995e-04) * (q[i10] * q[i19]) + (1.601052e-03) * (q[i10] * q[i20]) + (-4.788341e-05) * (q[i10] * q[i21])
            + (-1.747739e-04) * (q[i10] * q[i22]) + (5.198115e-04) * (q[i11] * q[i12]) + (3.872995e-03) * (q[i11] * q[i15])
            + (-1.095867e-03) * (q[i11] * q[i16]) + (-2.231571e-03) * (q[i11] * q[i19]) + (-6.580470e-04) * (q[i11] * q[i20])
            + (6.031111e-04) * (q[i11] * q[i21]) + (1.702132e-03) * (q[i11] * q[i22]) + (2.860590e-04) * (q[i12] * q[i15]) + (1.377429e-03) * (q[i12] * q[i16])
            + (1.575255e-03) * (q[i12] * q[i19]) + (-1.639693e-03) * (q[i12] * q[i20]) + (-1.023231e-03) * (q[i12] * q[i21])
            + (-8.373231e-04) * (q[i12] * q[i22]) + (8.015678e-05) * (q[i15] * q[i16]) + (-9.882359e-04) * (q[i15] * q[i19])
            + (1.207770e-03) * (q[i15] * q[i20]) + (-2.136392e-04) * (q[i15] * q[i21]) + (-6.772398e-05) * (q[i15] * q[i22])
            + (-6.929032e-05) * (q[i16] * q[i19]) + (1.071461e-03) * (q[i16] * q[i20]) + (-3.002668e-04) * (q[i16] * q[i21])
            + (2.228063e-03) * (q[i16] * q[i22]) + (-1.173045e-04) * (q[i19] * q[i20]) + (-1.194085e-04) * (q[i19] * q[i21])
            + (8.183928e-04) * (q[i19] * q[i22]) + (6.573443e-04) * (q[i20] * q[i21]) + (-7.683510e-04) * (q[i20] * q[i22])
            + (4.280431e-04) * (q[i21] * q[i22]);
   }

   public void getJQz2(double[] q, double[][] JQ)
   {
      JQ[3][i2] = (2.139773e-01) * (1) + (2.270333e-06) * ((2) * q[i2]) + (-3.677595e-03) * (q[i0]) + (3.762040e-03) * (q[i1]) + (-1.297012e-02) * (q[i3])
            + (1.287635e-02) * (q[i4]) + (-2.055079e-05) * (q[i5]) + (1.407401e-02) * (q[i6]) + (1.409484e-02) * (q[i7]) + (-1.008699e-03) * (q[i8])
            + (-1.280027e-03) * (q[i9]) + (-1.263228e-03) * (q[i10]) + (2.007064e-03) * (q[i11]) + (-1.752413e-03) * (q[i12]) + (-1.796744e-02) * (q[i15])
            + (-1.816952e-02) * (q[i16]) + (2.002263e-03) * (q[i19]) + (1.964953e-03) * (q[i20]) + (-3.676468e-03) * (q[i21]) + (3.596080e-03) * (q[i22])
            + (-1.779915e-03) * (q[i0] * q[i0]) + (-1.710495e-03) * (q[i1] * q[i1]) + (3.389921e-03) * ((2) * q[i0] * q[i2])
            + (3.420126e-03) * ((2) * q[i1] * q[i2]) + (-2.802144e-03) * ((3) * q[i2] * q[i2]) + (-3.984365e-03) * ((2) * q[i2] * q[i3])
            + (-3.960318e-03) * ((2) * q[i2] * q[i4]) + (5.685596e-03) * ((2) * q[i2] * q[i5]) + (-2.160606e-04) * ((2) * q[i2] * q[i6])
            + (2.160540e-04) * ((2) * q[i2] * q[i7]) + (6.674590e-06) * ((2) * q[i2] * q[i8]) + (4.256555e-04) * ((2) * q[i2] * q[i9])
            + (-4.375921e-04) * ((2) * q[i2] * q[i10]) + (-1.959227e-03) * ((2) * q[i2] * q[i11]) + (-1.950016e-03) * ((2) * q[i2] * q[i12])
            + (-1.460510e-03) * ((2) * q[i2] * q[i15]) + (1.450669e-03) * ((2) * q[i2] * q[i16]) + (-1.100006e-03) * ((2) * q[i2] * q[i19])
            + (1.085563e-03) * ((2) * q[i2] * q[i20]) + (5.330095e-04) * ((2) * q[i2] * q[i21]) + (5.261333e-04) * ((2) * q[i2] * q[i22])
            + (-6.304685e-03) * (q[i3] * q[i3]) + (-6.295773e-03) * (q[i4] * q[i4]) + (-1.450116e-02) * (q[i5] * q[i5]) + (-4.822411e-03) * (q[i6] * q[i6])
            + (-4.756895e-03) * (q[i7] * q[i7]) + (6.311364e-03) * (q[i8] * q[i8]) + (9.906023e-04) * (q[i9] * q[i9]) + (9.744991e-04) * (q[i10] * q[i10])
            + (-2.203007e-03) * (q[i11] * q[i11]) + (-2.230185e-03) * (q[i12] * q[i12]) + (-4.875913e-03) * (q[i15] * q[i15])
            + (-4.926560e-03) * (q[i16] * q[i16]) + (-3.407559e-04) * (q[i19] * q[i19]) + (-3.275157e-04) * (q[i20] * q[i20])
            + (-2.045029e-03) * (q[i21] * q[i21]) + (-2.031400e-03) * (q[i22] * q[i22]) + (-1.034438e-03) * (q[i0] * q[i1]) + (2.092116e-03) * (q[i0] * q[i3])
            + (-5.079142e-03) * (q[i0] * q[i4]) + (7.131349e-03) * (q[i0] * q[i5]) + (-8.584065e-03) * (q[i0] * q[i6]) + (-2.635861e-03) * (q[i0] * q[i7])
            + (5.935676e-03) * (q[i0] * q[i8]) + (-1.593483e-03) * (q[i0] * q[i9]) + (-1.246070e-03) * (q[i0] * q[i10]) + (5.106428e-04) * (q[i0] * q[i11])
            + (-8.160733e-04) * (q[i0] * q[i12]) + (-8.816348e-04) * (q[i0] * q[i15]) + (-1.612249e-03) * (q[i0] * q[i16]) + (-1.778900e-03) * (q[i0] * q[i19])
            + (3.204858e-04) * (q[i0] * q[i20]) + (-7.543829e-05) * (q[i0] * q[i21]) + (1.559269e-03) * (q[i0] * q[i22]) + (-5.128699e-03) * (q[i1] * q[i3])
            + (2.120396e-03) * (q[i1] * q[i4]) + (7.188995e-03) * (q[i1] * q[i5]) + (2.627176e-03) * (q[i1] * q[i6]) + (8.588029e-03) * (q[i1] * q[i7])
            + (-5.944304e-03) * (q[i1] * q[i8]) + (1.251727e-03) * (q[i1] * q[i9]) + (1.589570e-03) * (q[i1] * q[i10]) + (-8.452529e-04) * (q[i1] * q[i11])
            + (5.137758e-04) * (q[i1] * q[i12]) + (1.617850e-03) * (q[i1] * q[i15]) + (8.778651e-04) * (q[i1] * q[i16]) + (-3.047581e-04) * (q[i1] * q[i19])
            + (1.812038e-03) * (q[i1] * q[i20]) + (1.553031e-03) * (q[i1] * q[i21]) + (-1.092656e-04) * (q[i1] * q[i22]) + (8.438957e-04) * (q[i3] * q[i4])
            + (-1.295654e-02) * (q[i3] * q[i5]) + (-3.436195e-03) * (q[i3] * q[i6]) + (-5.176599e-05) * (q[i3] * q[i7]) + (3.351840e-03) * (q[i3] * q[i8])
            + (1.625624e-03) * (q[i3] * q[i9]) + (-2.978439e-03) * (q[i3] * q[i10]) + (1.251982e-03) * (q[i3] * q[i11]) + (-3.162736e-03) * (q[i3] * q[i12])
            + (7.214720e-06) * (q[i3] * q[i15]) + (6.397862e-04) * (q[i3] * q[i16]) + (-9.618666e-04) * (q[i3] * q[i19]) + (-2.681924e-03) * (q[i3] * q[i20])
            + (1.918705e-03) * (q[i3] * q[i21]) + (-3.860117e-03) * (q[i3] * q[i22]) + (-1.289630e-02) * (q[i4] * q[i5]) + (8.420013e-06) * (q[i4] * q[i6])
            + (3.457939e-03) * (q[i4] * q[i7]) + (-3.298082e-03) * (q[i4] * q[i8]) + (2.980178e-03) * (q[i4] * q[i9]) + (-1.570757e-03) * (q[i4] * q[i10])
            + (-3.083760e-03) * (q[i4] * q[i11]) + (1.235821e-03) * (q[i4] * q[i12]) + (-6.401135e-04) * (q[i4] * q[i15]) + (-3.348564e-05) * (q[i4] * q[i16])
            + (2.656238e-03) * (q[i4] * q[i19]) + (9.512730e-04) * (q[i4] * q[i20]) + (-3.841436e-03) * (q[i4] * q[i21]) + (1.912853e-03) * (q[i4] * q[i22])
            + (-8.390429e-03) * (q[i5] * q[i6]) + (8.471359e-03) * (q[i5] * q[i7]) + (-1.490702e-05) * (q[i5] * q[i8]) + (2.289242e-03) * (q[i5] * q[i9])
            + (-2.286014e-03) * (q[i5] * q[i10]) + (-1.072867e-03) * (q[i5] * q[i11]) + (-1.137662e-03) * (q[i5] * q[i12]) + (7.764108e-03) * (q[i5] * q[i15])
            + (-7.806409e-03) * (q[i5] * q[i16]) + (-3.188673e-03) * (q[i5] * q[i19]) + (3.189714e-03) * (q[i5] * q[i20]) + (-2.336499e-03) * (q[i5] * q[i21])
            + (-2.288843e-03) * (q[i5] * q[i22]) + (1.407930e-02) * (q[i6] * q[i7]) + (-9.756797e-03) * (q[i6] * q[i8]) + (-4.292913e-03) * (q[i6] * q[i9])
            + (1.744585e-03) * (q[i6] * q[i10]) + (2.110339e-03) * (q[i6] * q[i11]) + (3.404679e-03) * (q[i6] * q[i12]) + (-2.437519e-03) * (q[i6] * q[i15])
            + (1.432669e-03) * (q[i6] * q[i16]) + (-9.689233e-04) * (q[i6] * q[i19]) + (1.426482e-03) * (q[i6] * q[i20]) + (-3.636610e-04) * (q[i6] * q[i21])
            + (4.758287e-04) * (q[i6] * q[i22]) + (-9.730376e-03) * (q[i7] * q[i8]) + (1.755898e-03) * (q[i7] * q[i9]) + (-4.274821e-03) * (q[i7] * q[i10])
            + (-3.373453e-03) * (q[i7] * q[i11]) + (-2.075903e-03) * (q[i7] * q[i12]) + (1.418427e-03) * (q[i7] * q[i15]) + (-2.459923e-03) * (q[i7] * q[i16])
            + (1.409915e-03) * (q[i7] * q[i19]) + (-9.379687e-04) * (q[i7] * q[i20]) + (-4.657147e-04) * (q[i7] * q[i21]) + (3.561437e-04) * (q[i7] * q[i22])
            + (3.126228e-04) * (q[i8] * q[i9]) + (3.500331e-04) * (q[i8] * q[i10]) + (6.417629e-05) * (q[i8] * q[i11]) + (-1.014369e-04) * (q[i8] * q[i12])
            + (-6.401808e-04) * (q[i8] * q[i15]) + (-6.618732e-04) * (q[i8] * q[i16]) + (-2.143070e-03) * (q[i8] * q[i19]) + (-2.164839e-03) * (q[i8] * q[i20])
            + (7.322682e-04) * (q[i8] * q[i21]) + (-7.450876e-04) * (q[i8] * q[i22]) + (1.046591e-03) * (q[i9] * q[i10]) + (-1.559192e-03) * (q[i9] * q[i11])
            + (7.825747e-04) * (q[i9] * q[i12]) + (1.132058e-03) * (q[i9] * q[i15]) + (-1.885767e-03) * (q[i9] * q[i16]) + (-1.062082e-04) * (q[i9] * q[i19])
            + (3.898031e-04) * (q[i9] * q[i20]) + (9.727644e-05) * (q[i9] * q[i21]) + (-3.653917e-04) * (q[i9] * q[i22]) + (-7.718939e-04) * (q[i10] * q[i11])
            + (1.535351e-03) * (q[i10] * q[i12]) + (-1.879207e-03) * (q[i10] * q[i15]) + (1.131607e-03) * (q[i10] * q[i16]) + (3.717942e-04) * (q[i10] * q[i19])
            + (-1.153407e-04) * (q[i10] * q[i20]) + (3.553253e-04) * (q[i10] * q[i21]) + (-8.714664e-05) * (q[i10] * q[i22])
            + (2.987426e-03) * (q[i11] * q[i12]) + (2.351462e-03) * (q[i11] * q[i15]) + (-1.103160e-03) * (q[i11] * q[i16])
            + (-7.304907e-04) * (q[i11] * q[i19]) + (6.020709e-04) * (q[i11] * q[i20]) + (-8.595518e-04) * (q[i11] * q[i21])
            + (-6.161720e-04) * (q[i11] * q[i22]) + (1.115856e-03) * (q[i12] * q[i15]) + (-2.369277e-03) * (q[i12] * q[i16])
            + (-6.144859e-04) * (q[i12] * q[i19]) + (7.187429e-04) * (q[i12] * q[i20]) + (-6.115261e-04) * (q[i12] * q[i21])
            + (-8.317397e-04) * (q[i12] * q[i22]) + (5.453522e-04) * (q[i15] * q[i16]) + (5.335550e-04) * (q[i15] * q[i19]) + (9.489542e-04) * (q[i15] * q[i20])
            + (-1.693832e-03) * (q[i15] * q[i21]) + (-3.798725e-04) * (q[i15] * q[i22]) + (9.717815e-04) * (q[i16] * q[i19])
            + (5.450892e-04) * (q[i16] * q[i20]) + (3.833704e-04) * (q[i16] * q[i21]) + (1.684649e-03) * (q[i16] * q[i22]) + (-8.298723e-04) * (q[i19] * q[i20])
            + (-6.527569e-04) * (q[i19] * q[i21]) + (3.099660e-04) * (q[i19] * q[i22]) + (-3.104171e-04) * (q[i20] * q[i21])
            + (6.695144e-04) * (q[i20] * q[i22]) + (1.096783e-03) * (q[i21] * q[i22]);
   }

   public void getJQz3(double[] q, double[][] JQ)
   {
      JQ[3][i3] = (2.223118e-02) * (1) + (-6.406715e-03) * ((2) * q[i3]) + (-6.042340e-02) * (q[i0]) + (5.143739e-02) * (q[i1]) + (-1.297012e-02) * (q[i2])
            + (-6.414578e-05) * (q[i4]) + (3.490649e-03) * (q[i5]) + (1.711444e-02) * (q[i6]) + (1.131108e-02) * (q[i7]) + (-9.928663e-04) * (q[i8])
            + (5.648699e-03) * (q[i9]) + (1.150907e-02) * (q[i10]) + (-1.591195e-03) * (q[i11]) + (-2.040305e-03) * (q[i12]) + (7.821852e-03) * (q[i15])
            + (-2.114044e-04) * (q[i16]) + (-1.242713e-02) * (q[i19]) + (1.146731e-02) * (q[i20]) + (1.072986e-02) * (q[i21]) + (3.548932e-03) * (q[i22])
            + (-7.543770e-03) * (q[i0] * q[i0]) + (-2.026980e-03) * (q[i1] * q[i1]) + (-3.984365e-03) * (q[i2] * q[i2])
            + (-6.693411e-03) * ((2) * q[i0] * q[i3]) + (-2.267421e-03) * ((2) * q[i1] * q[i3]) + (-6.304685e-03) * ((2) * q[i2] * q[i3])
            + (3.357610e-03) * ((3) * q[i3] * q[i3]) + (1.782037e-03) * ((2) * q[i3] * q[i4]) + (-4.691578e-03) * ((2) * q[i3] * q[i5])
            + (5.830212e-03) * ((2) * q[i3] * q[i6]) + (9.049718e-03) * ((2) * q[i3] * q[i7]) + (4.594219e-03) * ((2) * q[i3] * q[i8])
            + (8.430913e-04) * ((2) * q[i3] * q[i9]) + (-2.710877e-03) * ((2) * q[i3] * q[i10]) + (3.105474e-03) * ((2) * q[i3] * q[i11])
            + (-3.467011e-04) * ((2) * q[i3] * q[i12]) + (-6.010023e-03) * ((2) * q[i3] * q[i15]) + (-2.747068e-03) * ((2) * q[i3] * q[i16])
            + (8.414819e-04) * ((2) * q[i3] * q[i19]) + (-2.612225e-04) * ((2) * q[i3] * q[i20]) + (-1.876124e-03) * ((2) * q[i3] * q[i21])
            + (1.019498e-03) * ((2) * q[i3] * q[i22]) + (1.758108e-03) * (q[i4] * q[i4]) + (3.354428e-03) * (q[i5] * q[i5]) + (-5.211335e-03) * (q[i6] * q[i6])
            + (-3.817495e-03) * (q[i7] * q[i7]) + (-4.006531e-03) * (q[i8] * q[i8]) + (-1.039539e-03) * (q[i9] * q[i9]) + (-1.109231e-03) * (q[i10] * q[i10])
            + (-3.986162e-04) * (q[i11] * q[i11]) + (-2.886119e-03) * (q[i12] * q[i12]) + (-1.798421e-03) * (q[i15] * q[i15])
            + (1.128739e-03) * (q[i16] * q[i16]) + (-1.618046e-03) * (q[i19] * q[i19]) + (9.940037e-04) * (q[i20] * q[i20])
            + (-7.426308e-04) * (q[i21] * q[i21]) + (1.324916e-03) * (q[i22] * q[i22]) + (1.107523e-03) * (q[i0] * q[i1]) + (2.092116e-03) * (q[i0] * q[i2])
            + (1.157804e-02) * (q[i0] * q[i4]) + (-1.175620e-02) * (q[i0] * q[i5]) + (-1.011579e-02) * (q[i0] * q[i6]) + (3.106750e-03) * (q[i0] * q[i7])
            + (5.752914e-03) * (q[i0] * q[i8]) + (6.880723e-03) * (q[i0] * q[i9]) + (4.104927e-04) * (q[i0] * q[i10]) + (-1.905806e-03) * (q[i0] * q[i11])
            + (-4.130384e-03) * (q[i0] * q[i12]) + (3.737213e-03) * (q[i0] * q[i15]) + (-4.549942e-03) * (q[i0] * q[i16]) + (-7.239892e-04) * (q[i0] * q[i19])
            + (-4.420090e-04) * (q[i0] * q[i20]) + (2.352586e-03) * (q[i0] * q[i21]) + (-2.251364e-03) * (q[i0] * q[i22]) + (-5.128699e-03) * (q[i1] * q[i2])
            + (1.164925e-02) * (q[i1] * q[i4]) + (3.613486e-04) * (q[i1] * q[i5]) + (3.009775e-03) * (q[i1] * q[i6]) + (-2.379827e-03) * (q[i1] * q[i7])
            + (-2.802561e-03) * (q[i1] * q[i8]) + (-6.364877e-03) * (q[i1] * q[i9]) + (-5.111334e-03) * (q[i1] * q[i10]) + (3.375875e-03) * (q[i1] * q[i11])
            + (-5.158804e-04) * (q[i1] * q[i12]) + (1.375503e-04) * (q[i1] * q[i15]) + (3.280968e-03) * (q[i1] * q[i16]) + (5.168160e-04) * (q[i1] * q[i19])
            + (-2.998582e-05) * (q[i1] * q[i20]) + (-7.713824e-04) * (q[i1] * q[i21]) + (-2.770689e-03) * (q[i1] * q[i22]) + (8.438957e-04) * (q[i2] * q[i4])
            + (-1.295654e-02) * (q[i2] * q[i5]) + (-3.436195e-03) * (q[i2] * q[i6]) + (-5.176599e-05) * (q[i2] * q[i7]) + (3.351840e-03) * (q[i2] * q[i8])
            + (1.625624e-03) * (q[i2] * q[i9]) + (-2.978439e-03) * (q[i2] * q[i10]) + (1.251982e-03) * (q[i2] * q[i11]) + (-3.162736e-03) * (q[i2] * q[i12])
            + (7.214720e-06) * (q[i2] * q[i15]) + (6.397862e-04) * (q[i2] * q[i16]) + (-9.618666e-04) * (q[i2] * q[i19]) + (-2.681924e-03) * (q[i2] * q[i20])
            + (1.918705e-03) * (q[i2] * q[i21]) + (-3.860117e-03) * (q[i2] * q[i22]) + (8.470272e-03) * (q[i4] * q[i5]) + (8.451358e-03) * (q[i4] * q[i6])
            + (-8.385159e-03) * (q[i4] * q[i7]) + (-5.215263e-05) * (q[i4] * q[i8]) + (-2.546138e-03) * (q[i4] * q[i9]) + (2.573182e-03) * (q[i4] * q[i10])
            + (1.651983e-04) * (q[i4] * q[i11]) + (2.118888e-04) * (q[i4] * q[i12]) + (1.928381e-03) * (q[i4] * q[i15]) + (-1.965659e-03) * (q[i4] * q[i16])
            + (-1.061026e-03) * (q[i4] * q[i19]) + (1.098279e-03) * (q[i4] * q[i20]) + (-1.279848e-03) * (q[i4] * q[i21]) + (-1.279522e-03) * (q[i4] * q[i22])
            + (-6.583003e-03) * (q[i5] * q[i6]) + (-6.469581e-03) * (q[i5] * q[i7]) + (-2.029328e-02) * (q[i5] * q[i8]) + (1.136585e-03) * (q[i5] * q[i9])
            + (-4.226047e-05) * (q[i5] * q[i10]) + (1.932446e-04) * (q[i5] * q[i11]) + (-4.053934e-03) * (q[i5] * q[i12]) + (-8.369577e-04) * (q[i5] * q[i15])
            + (1.036362e-03) * (q[i5] * q[i16]) + (-3.405769e-03) * (q[i5] * q[i19]) + (-2.451930e-03) * (q[i5] * q[i20]) + (2.088894e-03) * (q[i5] * q[i21])
            + (-2.087590e-03) * (q[i5] * q[i22]) + (9.574128e-05) * (q[i6] * q[i7]) + (1.185074e-04) * (q[i6] * q[i8]) + (-1.053298e-03) * (q[i6] * q[i9])
            + (1.761238e-03) * (q[i6] * q[i10]) + (-3.321811e-03) * (q[i6] * q[i11]) + (5.265804e-03) * (q[i6] * q[i12]) + (4.697587e-03) * (q[i6] * q[i15])
            + (1.186916e-03) * (q[i6] * q[i16]) + (-1.181307e-03) * (q[i6] * q[i19]) + (2.162745e-03) * (q[i6] * q[i20]) + (-1.000557e-03) * (q[i6] * q[i21])
            + (3.790578e-03) * (q[i6] * q[i22]) + (8.310662e-04) * (q[i7] * q[i8]) + (-5.500530e-04) * (q[i7] * q[i9]) + (-2.524895e-03) * (q[i7] * q[i10])
            + (5.338200e-03) * (q[i7] * q[i11]) + (-3.673652e-03) * (q[i7] * q[i12]) + (3.929025e-03) * (q[i7] * q[i15]) + (-4.584537e-03) * (q[i7] * q[i16])
            + (-2.230980e-04) * (q[i7] * q[i19]) + (5.809542e-03) * (q[i7] * q[i20]) + (1.440824e-03) * (q[i7] * q[i21]) + (-3.218778e-04) * (q[i7] * q[i22])
            + (8.914568e-04) * (q[i8] * q[i9]) + (-4.275390e-04) * (q[i8] * q[i10]) + (2.691313e-03) * (q[i8] * q[i11]) + (2.354166e-03) * (q[i8] * q[i12])
            + (-8.223876e-04) * (q[i8] * q[i15]) + (3.525317e-06) * (q[i8] * q[i16]) + (5.242117e-03) * (q[i8] * q[i19]) + (-2.827921e-03) * (q[i8] * q[i20])
            + (1.435959e-03) * (q[i8] * q[i21]) + (2.509093e-03) * (q[i8] * q[i22]) + (4.041426e-04) * (q[i9] * q[i10]) + (-1.194464e-03) * (q[i9] * q[i11])
            + (-6.590604e-04) * (q[i9] * q[i12]) + (-2.520779e-04) * (q[i9] * q[i15]) + (1.079600e-03) * (q[i9] * q[i16]) + (1.489055e-03) * (q[i9] * q[i19])
            + (6.169341e-04) * (q[i9] * q[i20]) + (-1.580515e-03) * (q[i9] * q[i21]) + (4.432069e-04) * (q[i9] * q[i22]) + (7.508213e-04) * (q[i10] * q[i11])
            + (2.924787e-04) * (q[i10] * q[i12]) + (-4.046427e-05) * (q[i10] * q[i15]) + (-2.314218e-03) * (q[i10] * q[i16])
            + (1.010646e-03) * (q[i10] * q[i19]) + (-1.256187e-03) * (q[i10] * q[i20]) + (-2.090453e-03) * (q[i10] * q[i21])
            + (-1.610804e-03) * (q[i10] * q[i22]) + (4.219337e-04) * (q[i11] * q[i12]) + (-1.423635e-03) * (q[i11] * q[i15])
            + (4.422790e-04) * (q[i11] * q[i16]) + (-1.174579e-03) * (q[i11] * q[i19]) + (-2.797302e-03) * (q[i11] * q[i20])
            + (-1.517272e-03) * (q[i11] * q[i21]) + (-7.618473e-04) * (q[i11] * q[i22]) + (-4.134908e-04) * (q[i12] * q[i15])
            + (-4.669852e-04) * (q[i12] * q[i16]) + (-4.828984e-03) * (q[i12] * q[i19]) + (-2.117439e-03) * (q[i12] * q[i20])
            + (-5.185118e-04) * (q[i12] * q[i21]) + (9.435594e-05) * (q[i12] * q[i22]) + (2.937128e-04) * (q[i15] * q[i16])
            + (-1.359044e-03) * (q[i15] * q[i19]) + (1.892597e-03) * (q[i15] * q[i20]) + (2.977472e-04) * (q[i15] * q[i21]) + (1.001613e-03) * (q[i15] * q[i22])
            + (-2.043554e-03) * (q[i16] * q[i19]) + (2.998594e-03) * (q[i16] * q[i20]) + (9.347620e-04) * (q[i16] * q[i21]) + (1.107827e-03) * (q[i16] * q[i22])
            + (-2.845084e-04) * (q[i19] * q[i20]) + (2.922209e-04) * (q[i19] * q[i21]) + (-7.987878e-04) * (q[i19] * q[i22])
            + (8.213946e-04) * (q[i20] * q[i21]) + (1.826411e-03) * (q[i20] * q[i22]) + (7.898597e-05) * (q[i21] * q[i22]);
   }

   public void getJQz4(double[] q, double[][] JQ)
   {
      JQ[3][i4] = (2.211737e-02) * (1) + (6.368734e-03) * ((2) * q[i4]) + (-5.138662e-02) * (q[i0]) + (6.048093e-02) * (q[i1]) + (1.287635e-02) * (q[i2])
            + (-6.414578e-05) * (q[i3]) + (-3.470244e-03) * (q[i5]) + (1.139101e-02) * (q[i6]) + (1.688534e-02) * (q[i7]) + (-9.773458e-04) * (q[i8])
            + (1.158554e-02) * (q[i9]) + (5.640228e-03) * (q[i10]) + (2.082612e-03) * (q[i11]) + (1.588879e-03) * (q[i12]) + (-2.693480e-04) * (q[i15])
            + (7.795655e-03) * (q[i16]) + (1.143341e-02) * (q[i19]) + (-1.241368e-02) * (q[i20]) + (-3.541732e-03) * (q[i21]) + (-1.061346e-02) * (q[i22])
            + (-2.024621e-03) * (q[i0] * q[i0]) + (-7.512135e-03) * (q[i1] * q[i1]) + (-3.960318e-03) * (q[i2] * q[i2]) + (1.782037e-03) * (q[i3] * q[i3])
            + (-2.229401e-03) * ((2) * q[i0] * q[i4]) + (-6.662712e-03) * ((2) * q[i1] * q[i4]) + (-6.295773e-03) * ((2) * q[i2] * q[i4])
            + (1.758108e-03) * ((2) * q[i3] * q[i4]) + (3.280418e-03) * ((3) * q[i4] * q[i4]) + (-4.701985e-03) * ((2) * q[i4] * q[i5])
            + (-9.034951e-03) * ((2) * q[i4] * q[i6]) + (-5.931101e-03) * ((2) * q[i4] * q[i7]) + (-4.616553e-03) * ((2) * q[i4] * q[i8])
            + (2.690506e-03) * ((2) * q[i4] * q[i9]) + (-8.586939e-04) * ((2) * q[i4] * q[i10]) + (-3.515045e-04) * ((2) * q[i4] * q[i11])
            + (3.047201e-03) * ((2) * q[i4] * q[i12]) + (2.749408e-03) * ((2) * q[i4] * q[i15]) + (6.002643e-03) * ((2) * q[i4] * q[i16])
            + (2.218625e-04) * ((2) * q[i4] * q[i19]) + (-8.482367e-04) * ((2) * q[i4] * q[i20]) + (1.014072e-03) * ((2) * q[i4] * q[i21])
            + (-1.839932e-03) * ((2) * q[i4] * q[i22]) + (3.387415e-03) * (q[i5] * q[i5]) + (-3.818273e-03) * (q[i6] * q[i6])
            + (-5.239347e-03) * (q[i7] * q[i7]) + (-4.035328e-03) * (q[i8] * q[i8]) + (-1.127005e-03) * (q[i9] * q[i9]) + (-1.073550e-03) * (q[i10] * q[i10])
            + (-2.803565e-03) * (q[i11] * q[i11]) + (-3.361134e-04) * (q[i12] * q[i12]) + (1.108226e-03) * (q[i15] * q[i15])
            + (-1.787982e-03) * (q[i16] * q[i16]) + (9.882835e-04) * (q[i19] * q[i19]) + (-1.590706e-03) * (q[i20] * q[i20])
            + (1.312973e-03) * (q[i21] * q[i21]) + (-7.465614e-04) * (q[i22] * q[i22]) + (1.092826e-03) * (q[i0] * q[i1]) + (-5.079142e-03) * (q[i0] * q[i2])
            + (1.157804e-02) * (q[i0] * q[i3]) + (3.113038e-04) * (q[i0] * q[i5]) + (2.345123e-03) * (q[i0] * q[i6]) + (-3.091155e-03) * (q[i0] * q[i7])
            + (2.799478e-03) * (q[i0] * q[i8]) + (5.130394e-03) * (q[i0] * q[i9]) + (6.354973e-03) * (q[i0] * q[i10]) + (-4.735293e-04) * (q[i0] * q[i11])
            + (3.386984e-03) * (q[i0] * q[i12]) + (-3.231341e-03) * (q[i0] * q[i15]) + (-1.271616e-04) * (q[i0] * q[i16]) + (-1.119306e-05) * (q[i0] * q[i19])
            + (-5.171096e-04) * (q[i0] * q[i20]) + (-2.774221e-03) * (q[i0] * q[i21]) + (-7.930453e-04) * (q[i0] * q[i22]) + (2.120396e-03) * (q[i1] * q[i2])
            + (1.164925e-02) * (q[i1] * q[i3]) + (-1.174509e-02) * (q[i1] * q[i5]) + (-3.075405e-03) * (q[i1] * q[i6]) + (1.026329e-02) * (q[i1] * q[i7])
            + (-5.652658e-03) * (q[i1] * q[i8]) + (-4.481237e-04) * (q[i1] * q[i9]) + (-6.823310e-03) * (q[i1] * q[i10]) + (-4.153900e-03) * (q[i1] * q[i11])
            + (-1.935043e-03) * (q[i1] * q[i12]) + (4.452672e-03) * (q[i1] * q[i15]) + (-3.699263e-03) * (q[i1] * q[i16]) + (4.557667e-04) * (q[i1] * q[i19])
            + (6.645825e-04) * (q[i1] * q[i20]) + (-2.249974e-03) * (q[i1] * q[i21]) + (2.335218e-03) * (q[i1] * q[i22]) + (8.438957e-04) * (q[i2] * q[i3])
            + (-1.289630e-02) * (q[i2] * q[i5]) + (8.420013e-06) * (q[i2] * q[i6]) + (3.457939e-03) * (q[i2] * q[i7]) + (-3.298082e-03) * (q[i2] * q[i8])
            + (2.980178e-03) * (q[i2] * q[i9]) + (-1.570757e-03) * (q[i2] * q[i10]) + (-3.083760e-03) * (q[i2] * q[i11]) + (1.235821e-03) * (q[i2] * q[i12])
            + (-6.401135e-04) * (q[i2] * q[i15]) + (-3.348564e-05) * (q[i2] * q[i16]) + (2.656238e-03) * (q[i2] * q[i19]) + (9.512730e-04) * (q[i2] * q[i20])
            + (-3.841436e-03) * (q[i2] * q[i21]) + (1.912853e-03) * (q[i2] * q[i22]) + (8.470272e-03) * (q[i3] * q[i5]) + (8.451358e-03) * (q[i3] * q[i6])
            + (-8.385159e-03) * (q[i3] * q[i7]) + (-5.215263e-05) * (q[i3] * q[i8]) + (-2.546138e-03) * (q[i3] * q[i9]) + (2.573182e-03) * (q[i3] * q[i10])
            + (1.651983e-04) * (q[i3] * q[i11]) + (2.118888e-04) * (q[i3] * q[i12]) + (1.928381e-03) * (q[i3] * q[i15]) + (-1.965659e-03) * (q[i3] * q[i16])
            + (-1.061026e-03) * (q[i3] * q[i19]) + (1.098279e-03) * (q[i3] * q[i20]) + (-1.279848e-03) * (q[i3] * q[i21]) + (-1.279522e-03) * (q[i3] * q[i22])
            + (6.493612e-03) * (q[i5] * q[i6]) + (6.544851e-03) * (q[i5] * q[i7]) + (2.026581e-02) * (q[i5] * q[i8]) + (7.047726e-05) * (q[i5] * q[i9])
            + (-1.138860e-03) * (q[i5] * q[i10]) + (-3.948614e-03) * (q[i5] * q[i11]) + (1.697960e-04) * (q[i5] * q[i12]) + (-9.759214e-04) * (q[i5] * q[i15])
            + (8.420715e-04) * (q[i5] * q[i16]) + (2.421059e-03) * (q[i5] * q[i19]) + (3.417518e-03) * (q[i5] * q[i20]) + (-2.063205e-03) * (q[i5] * q[i21])
            + (2.065826e-03) * (q[i5] * q[i22]) + (1.566079e-04) * (q[i6] * q[i7]) + (8.183180e-04) * (q[i6] * q[i8]) + (-2.593748e-03) * (q[i6] * q[i9])
            + (-5.370685e-04) * (q[i6] * q[i10]) + (3.676719e-03) * (q[i6] * q[i11]) + (-5.365699e-03) * (q[i6] * q[i12]) + (-4.621407e-03) * (q[i6] * q[i15])
            + (3.924413e-03) * (q[i6] * q[i16]) + (5.798120e-03) * (q[i6] * q[i19]) + (-2.300427e-04) * (q[i6] * q[i20]) + (3.513589e-04) * (q[i6] * q[i21])
            + (-1.427510e-03) * (q[i6] * q[i22]) + (1.354132e-04) * (q[i7] * q[i8]) + (1.765688e-03) * (q[i7] * q[i9]) + (-1.119688e-03) * (q[i7] * q[i10])
            + (-5.271561e-03) * (q[i7] * q[i11]) + (3.306854e-03) * (q[i7] * q[i12]) + (1.152578e-03) * (q[i7] * q[i15]) + (4.677970e-03) * (q[i7] * q[i16])
            + (2.130440e-03) * (q[i7] * q[i19]) + (-1.210903e-03) * (q[i7] * q[i20]) + (-3.757145e-03) * (q[i7] * q[i21]) + (1.006111e-03) * (q[i7] * q[i22])
            + (-4.158017e-04) * (q[i8] * q[i9]) + (8.800150e-04) * (q[i8] * q[i10]) + (-2.375399e-03) * (q[i8] * q[i11]) + (-2.759669e-03) * (q[i8] * q[i12])
            + (1.167909e-05) * (q[i8] * q[i15]) + (-8.594415e-04) * (q[i8] * q[i16]) + (-2.804724e-03) * (q[i8] * q[i19]) + (5.203166e-03) * (q[i8] * q[i20])
            + (-2.477220e-03) * (q[i8] * q[i21]) + (-1.455301e-03) * (q[i8] * q[i22]) + (3.924239e-04) * (q[i9] * q[i10]) + (-3.106594e-04) * (q[i9] * q[i11])
            + (-7.626711e-04) * (q[i9] * q[i12]) + (-2.307338e-03) * (q[i9] * q[i15]) + (-3.622675e-05) * (q[i9] * q[i16]) + (-1.238296e-03) * (q[i9] * q[i19])
            + (1.006603e-03) * (q[i9] * q[i20]) + (1.596703e-03) * (q[i9] * q[i21]) + (2.096791e-03) * (q[i9] * q[i22]) + (6.662096e-04) * (q[i10] * q[i11])
            + (1.201299e-03) * (q[i10] * q[i12]) + (1.070567e-03) * (q[i10] * q[i15]) + (-2.303241e-04) * (q[i10] * q[i16]) + (5.875405e-04) * (q[i10] * q[i19])
            + (1.476427e-03) * (q[i10] * q[i20]) + (-4.333095e-04) * (q[i10] * q[i21]) + (1.577342e-03) * (q[i10] * q[i22]) + (4.096850e-04) * (q[i11] * q[i12])
            + (4.705882e-04) * (q[i11] * q[i15]) + (4.229579e-04) * (q[i11] * q[i16]) + (2.112294e-03) * (q[i11] * q[i19]) + (4.808062e-03) * (q[i11] * q[i20])
            + (1.044082e-04) * (q[i11] * q[i21]) + (-5.197775e-04) * (q[i11] * q[i22]) + (-4.336719e-04) * (q[i12] * q[i15])
            + (1.438830e-03) * (q[i12] * q[i16]) + (2.819507e-03) * (q[i12] * q[i19]) + (1.160681e-03) * (q[i12] * q[i20]) + (-7.572880e-04) * (q[i12] * q[i21])
            + (-1.530939e-03) * (q[i12] * q[i22]) + (3.093467e-04) * (q[i15] * q[i16]) + (3.023984e-03) * (q[i15] * q[i19])
            + (-2.037393e-03) * (q[i15] * q[i20]) + (-1.102679e-03) * (q[i15] * q[i21]) + (-9.009462e-04) * (q[i15] * q[i22])
            + (1.869637e-03) * (q[i16] * q[i19]) + (-1.350580e-03) * (q[i16] * q[i20]) + (-1.002016e-03) * (q[i16] * q[i21])
            + (-2.786692e-04) * (q[i16] * q[i22]) + (-2.449044e-04) * (q[i19] * q[i20]) + (-1.807287e-03) * (q[i19] * q[i21])
            + (-8.107735e-04) * (q[i19] * q[i22]) + (7.983853e-04) * (q[i20] * q[i21]) + (-2.967091e-04) * (q[i20] * q[i22])
            + (6.475461e-05) * (q[i21] * q[i22]);
   }

   public void getJQz5(double[] q, double[][] JQ)
   {
      JQ[3][i5] = (-1.718942e-02) * (1) + (1.956306e-05) * ((2) * q[i5]) + (-8.213849e-03) * (q[i0]) + (8.116596e-03) * (q[i1]) + (-2.055079e-05) * (q[i2])
            + (3.490649e-03) * (q[i3]) + (-3.470244e-03) * (q[i4]) + (2.939048e-02) * (q[i6]) + (2.907652e-02) * (q[i7]) + (5.116424e-02) * (q[i8])
            + (8.933518e-03) * (q[i9]) + (8.822487e-03) * (q[i10]) + (-3.697470e-03) * (q[i11]) + (3.888892e-03) * (q[i12]) + (-1.199885e-03) * (q[i15])
            + (-1.234830e-03) * (q[i16]) + (5.678138e-03) * (q[i19]) + (5.708390e-03) * (q[i20]) + (-3.162355e-03) * (q[i21]) + (3.131660e-03) * (q[i22])
            + (1.598918e-04) * (q[i0] * q[i0]) + (1.282382e-04) * (q[i1] * q[i1]) + (5.685596e-03) * (q[i2] * q[i2]) + (-4.691578e-03) * (q[i3] * q[i3])
            + (-4.701985e-03) * (q[i4] * q[i4]) + (4.455479e-05) * ((2) * q[i0] * q[i5]) + (-6.971971e-05) * ((2) * q[i1] * q[i5])
            + (-1.450116e-02) * ((2) * q[i2] * q[i5]) + (3.354428e-03) * ((2) * q[i3] * q[i5]) + (3.387415e-03) * ((2) * q[i4] * q[i5])
            + (2.352317e-03) * ((3) * q[i5] * q[i5]) + (2.022121e-04) * ((2) * q[i5] * q[i6]) + (-2.879824e-04) * ((2) * q[i5] * q[i7])
            + (7.539750e-05) * ((2) * q[i5] * q[i8]) + (4.889481e-05) * ((2) * q[i5] * q[i9]) + (-8.529623e-05) * ((2) * q[i5] * q[i10])
            + (3.295042e-03) * ((2) * q[i5] * q[i11]) + (3.401737e-03) * ((2) * q[i5] * q[i12]) + (9.894973e-04) * ((2) * q[i5] * q[i15])
            + (-9.961190e-04) * ((2) * q[i5] * q[i16]) + (2.131419e-03) * ((2) * q[i5] * q[i19]) + (-2.149900e-03) * ((2) * q[i5] * q[i20])
            + (4.273690e-04) * ((2) * q[i5] * q[i21]) + (4.067289e-04) * ((2) * q[i5] * q[i22]) + (8.587558e-03) * (q[i6] * q[i6])
            + (8.560408e-03) * (q[i7] * q[i7]) + (6.413173e-03) * (q[i8] * q[i8]) + (-1.496355e-03) * (q[i9] * q[i9]) + (-1.465258e-03) * (q[i10] * q[i10])
            + (-4.641605e-04) * (q[i11] * q[i11]) + (-4.418924e-04) * (q[i12] * q[i12]) + (-1.275408e-03) * (q[i15] * q[i15])
            + (-1.308459e-03) * (q[i16] * q[i16]) + (5.672238e-04) * (q[i19] * q[i19]) + (5.569911e-04) * (q[i20] * q[i20])
            + (-3.217971e-04) * (q[i21] * q[i21]) + (-3.075707e-04) * (q[i22] * q[i22]) + (5.936415e-03) * (q[i0] * q[i1]) + (7.131349e-03) * (q[i0] * q[i2])
            + (-1.175620e-02) * (q[i0] * q[i3]) + (3.113038e-04) * (q[i0] * q[i4]) + (2.328631e-03) * (q[i0] * q[i6]) + (-2.363745e-04) * (q[i0] * q[i7])
            + (3.065318e-03) * (q[i0] * q[i8]) + (2.392677e-03) * (q[i0] * q[i9]) + (1.078840e-03) * (q[i0] * q[i10]) + (1.322023e-03) * (q[i0] * q[i11])
            + (1.282937e-03) * (q[i0] * q[i12]) + (3.452045e-03) * (q[i0] * q[i15]) + (-2.146369e-03) * (q[i0] * q[i16]) + (-1.380481e-03) * (q[i0] * q[i19])
            + (-5.019998e-04) * (q[i0] * q[i20]) + (-2.871135e-03) * (q[i0] * q[i21]) + (-8.908884e-04) * (q[i0] * q[i22]) + (7.188995e-03) * (q[i1] * q[i2])
            + (3.613486e-04) * (q[i1] * q[i3]) + (-1.174509e-02) * (q[i1] * q[i4]) + (2.449185e-04) * (q[i1] * q[i6]) + (-2.324987e-03) * (q[i1] * q[i7])
            + (-3.085534e-03) * (q[i1] * q[i8]) + (-1.058509e-03) * (q[i1] * q[i9]) + (-2.381350e-03) * (q[i1] * q[i10]) + (1.337867e-03) * (q[i1] * q[i11])
            + (1.235463e-03) * (q[i1] * q[i12]) + (2.119109e-03) * (q[i1] * q[i15]) + (-3.452865e-03) * (q[i1] * q[i16]) + (5.274652e-04) * (q[i1] * q[i19])
            + (1.376363e-03) * (q[i1] * q[i20]) + (-8.784839e-04) * (q[i1] * q[i21]) + (-2.824797e-03) * (q[i1] * q[i22]) + (-1.295654e-02) * (q[i2] * q[i3])
            + (-1.289630e-02) * (q[i2] * q[i4]) + (-8.390429e-03) * (q[i2] * q[i6]) + (8.471359e-03) * (q[i2] * q[i7]) + (-1.490702e-05) * (q[i2] * q[i8])
            + (2.289242e-03) * (q[i2] * q[i9]) + (-2.286014e-03) * (q[i2] * q[i10]) + (-1.072867e-03) * (q[i2] * q[i11]) + (-1.137662e-03) * (q[i2] * q[i12])
            + (7.764108e-03) * (q[i2] * q[i15]) + (-7.806409e-03) * (q[i2] * q[i16]) + (-3.188673e-03) * (q[i2] * q[i19]) + (3.189714e-03) * (q[i2] * q[i20])
            + (-2.336499e-03) * (q[i2] * q[i21]) + (-2.288843e-03) * (q[i2] * q[i22]) + (8.470272e-03) * (q[i3] * q[i4]) + (-6.583003e-03) * (q[i3] * q[i6])
            + (-6.469581e-03) * (q[i3] * q[i7]) + (-2.029328e-02) * (q[i3] * q[i8]) + (1.136585e-03) * (q[i3] * q[i9]) + (-4.226047e-05) * (q[i3] * q[i10])
            + (1.932446e-04) * (q[i3] * q[i11]) + (-4.053934e-03) * (q[i3] * q[i12]) + (-8.369577e-04) * (q[i3] * q[i15]) + (1.036362e-03) * (q[i3] * q[i16])
            + (-3.405769e-03) * (q[i3] * q[i19]) + (-2.451930e-03) * (q[i3] * q[i20]) + (2.088894e-03) * (q[i3] * q[i21]) + (-2.087590e-03) * (q[i3] * q[i22])
            + (6.493612e-03) * (q[i4] * q[i6]) + (6.544851e-03) * (q[i4] * q[i7]) + (2.026581e-02) * (q[i4] * q[i8]) + (7.047726e-05) * (q[i4] * q[i9])
            + (-1.138860e-03) * (q[i4] * q[i10]) + (-3.948614e-03) * (q[i4] * q[i11]) + (1.697960e-04) * (q[i4] * q[i12]) + (-9.759214e-04) * (q[i4] * q[i15])
            + (8.420715e-04) * (q[i4] * q[i16]) + (2.421059e-03) * (q[i4] * q[i19]) + (3.417518e-03) * (q[i4] * q[i20]) + (-2.063205e-03) * (q[i4] * q[i21])
            + (2.065826e-03) * (q[i4] * q[i22]) + (-1.077427e-02) * (q[i6] * q[i7]) + (2.130050e-04) * (q[i6] * q[i8]) + (-3.894283e-03) * (q[i6] * q[i9])
            + (-1.495500e-03) * (q[i6] * q[i10]) + (-1.098048e-03) * (q[i6] * q[i11]) + (-1.625085e-03) * (q[i6] * q[i12]) + (-3.531705e-03) * (q[i6] * q[i15])
            + (-2.354698e-03) * (q[i6] * q[i16]) + (2.702330e-03) * (q[i6] * q[i19]) + (-9.051232e-04) * (q[i6] * q[i20]) + (1.591960e-03) * (q[i6] * q[i21])
            + (-1.461438e-03) * (q[i6] * q[i22]) + (2.489370e-04) * (q[i7] * q[i8]) + (-1.484852e-03) * (q[i7] * q[i9]) + (-3.813284e-03) * (q[i7] * q[i10])
            + (1.649069e-03) * (q[i7] * q[i11]) + (1.139513e-03) * (q[i7] * q[i12]) + (-2.396054e-03) * (q[i7] * q[i15]) + (-3.510798e-03) * (q[i7] * q[i16])
            + (-8.926671e-04) * (q[i7] * q[i19]) + (2.701717e-03) * (q[i7] * q[i20]) + (1.449493e-03) * (q[i7] * q[i21]) + (-1.590293e-03) * (q[i7] * q[i22])
            + (2.075229e-03) * (q[i8] * q[i9]) + (2.087639e-03) * (q[i8] * q[i10]) + (-5.032207e-03) * (q[i8] * q[i11]) + (5.177861e-03) * (q[i8] * q[i12])
            + (-3.516862e-03) * (q[i8] * q[i15]) + (-3.532130e-03) * (q[i8] * q[i16]) + (-1.052659e-03) * (q[i8] * q[i19]) + (-1.009716e-03) * (q[i8] * q[i20])
            + (1.641170e-03) * (q[i8] * q[i21]) + (-1.689967e-03) * (q[i8] * q[i22]) + (3.383062e-06) * (q[i9] * q[i10]) + (-4.488976e-04) * (q[i9] * q[i11])
            + (3.316518e-04) * (q[i9] * q[i12]) + (6.361504e-04) * (q[i9] * q[i15]) + (-8.130769e-04) * (q[i9] * q[i16]) + (3.115368e-04) * (q[i9] * q[i19])
            + (-1.091896e-03) * (q[i9] * q[i20]) + (7.568795e-04) * (q[i9] * q[i21]) + (-1.920676e-03) * (q[i9] * q[i22]) + (-3.314254e-04) * (q[i10] * q[i11])
            + (4.846041e-04) * (q[i10] * q[i12]) + (-7.985036e-04) * (q[i10] * q[i15]) + (6.463539e-04) * (q[i10] * q[i16])
            + (-1.069672e-03) * (q[i10] * q[i19]) + (3.255600e-04) * (q[i10] * q[i20]) + (1.911686e-03) * (q[i10] * q[i21])
            + (-7.536486e-04) * (q[i10] * q[i22]) + (-5.596421e-05) * (q[i11] * q[i12]) + (-3.651541e-03) * (q[i11] * q[i15])
            + (1.199525e-03) * (q[i11] * q[i16]) + (-7.826358e-04) * (q[i11] * q[i19]) + (8.982612e-04) * (q[i11] * q[i20])
            + (-2.270249e-04) * (q[i11] * q[i21]) + (7.702120e-04) * (q[i11] * q[i22]) + (-1.202181e-03) * (q[i12] * q[i15])
            + (3.666847e-03) * (q[i12] * q[i16]) + (-9.107160e-04) * (q[i12] * q[i19]) + (7.582725e-04) * (q[i12] * q[i20]) + (7.650291e-04) * (q[i12] * q[i21])
            + (-2.058770e-04) * (q[i12] * q[i22]) + (1.537212e-03) * (q[i15] * q[i16]) + (7.910750e-04) * (q[i15] * q[i19]) + (3.173976e-05) * (q[i15] * q[i20])
            + (-1.798743e-04) * (q[i15] * q[i21]) + (1.713108e-04) * (q[i15] * q[i22]) + (5.531803e-05) * (q[i16] * q[i19]) + (8.106571e-04) * (q[i16] * q[i20])
            + (-1.722371e-04) * (q[i16] * q[i21]) + (1.665068e-04) * (q[i16] * q[i22]) + (4.123524e-06) * (q[i19] * q[i20]) + (2.630843e-04) * (q[i19] * q[i21])
            + (5.709591e-04) * (q[i19] * q[i22]) + (-5.749362e-04) * (q[i20] * q[i21]) + (-2.760606e-04) * (q[i20] * q[i22])
            + (-9.237874e-04) * (q[i21] * q[i22]);
   }

   public void getJQz6(double[] q, double[][] JQ)
   {
      JQ[3][i6] = (8.517761e-02) * (1) + (-6.686235e-03) * ((2) * q[i6]) + (1.570326e-02) * (q[i0]) + (-1.079350e-02) * (q[i1]) + (1.407401e-02) * (q[i2])
            + (1.711444e-02) * (q[i3]) + (1.139101e-02) * (q[i4]) + (2.939048e-02) * (q[i5]) + (8.185942e-06) * (q[i7]) + (-8.877688e-03) * (q[i8])
            + (-6.607333e-03) * (q[i9]) + (1.429997e-03) * (q[i10]) + (8.965253e-04) * (q[i11]) + (1.560899e-03) * (q[i12]) + (2.112994e-03) * (q[i15])
            + (1.119554e-02) * (q[i16]) + (-8.683908e-03) * (q[i19]) + (-5.856227e-03) * (q[i20]) + (-1.736492e-03) * (q[i21]) + (2.301098e-03) * (q[i22])
            + (-1.235529e-02) * (q[i0] * q[i0]) + (3.284873e-03) * (q[i1] * q[i1]) + (-2.160606e-04) * (q[i2] * q[i2]) + (5.830212e-03) * (q[i3] * q[i3])
            + (-9.034951e-03) * (q[i4] * q[i4]) + (2.022121e-04) * (q[i5] * q[i5]) + (-1.078988e-02) * ((2) * q[i0] * q[i6])
            + (6.680487e-03) * ((2) * q[i1] * q[i6]) + (-4.822411e-03) * ((2) * q[i2] * q[i6]) + (-5.211335e-03) * ((2) * q[i3] * q[i6])
            + (-3.818273e-03) * ((2) * q[i4] * q[i6]) + (8.587558e-03) * ((2) * q[i5] * q[i6]) + (-3.909764e-03) * ((3) * q[i6] * q[i6])
            + (1.289841e-03) * ((2) * q[i6] * q[i7]) + (7.130810e-03) * ((2) * q[i6] * q[i8]) + (-2.098500e-04) * ((2) * q[i6] * q[i9])
            + (2.881432e-03) * ((2) * q[i6] * q[i10]) + (-4.684032e-03) * ((2) * q[i6] * q[i11]) + (1.484473e-03) * ((2) * q[i6] * q[i12])
            + (2.561257e-03) * ((2) * q[i6] * q[i15]) + (-1.554538e-03) * ((2) * q[i6] * q[i16]) + (2.031403e-04) * ((2) * q[i6] * q[i19])
            + (-8.974494e-04) * ((2) * q[i6] * q[i20]) + (-1.225484e-03) * ((2) * q[i6] * q[i21]) + (-1.812007e-03) * ((2) * q[i6] * q[i22])
            + (-1.309745e-03) * (q[i7] * q[i7]) + (1.706365e-03) * (q[i8] * q[i8]) + (-1.563730e-03) * (q[i9] * q[i9]) + (1.977798e-04) * (q[i10] * q[i10])
            + (2.021229e-03) * (q[i11] * q[i11]) + (2.741810e-03) * (q[i12] * q[i12]) + (1.350029e-03) * (q[i15] * q[i15]) + (1.732938e-03) * (q[i16] * q[i16])
            + (1.277015e-03) * (q[i19] * q[i19]) + (1.285111e-03) * (q[i20] * q[i20]) + (1.607276e-03) * (q[i21] * q[i21]) + (1.739386e-03) * (q[i22] * q[i22])
            + (-3.535111e-04) * (q[i0] * q[i1]) + (-8.584065e-03) * (q[i0] * q[i2]) + (-1.011579e-02) * (q[i0] * q[i3]) + (2.345123e-03) * (q[i0] * q[i4])
            + (2.328631e-03) * (q[i0] * q[i5]) + (7.590054e-03) * (q[i0] * q[i7]) + (3.540452e-03) * (q[i0] * q[i8]) + (-6.566775e-03) * (q[i0] * q[i9])
            + (6.778875e-03) * (q[i0] * q[i10]) + (2.362114e-03) * (q[i0] * q[i11]) + (1.859035e-03) * (q[i0] * q[i12]) + (-3.302023e-04) * (q[i0] * q[i15])
            + (2.531603e-03) * (q[i0] * q[i16]) + (-1.968210e-03) * (q[i0] * q[i19]) + (-7.278929e-04) * (q[i0] * q[i20]) + (-4.359618e-04) * (q[i0] * q[i21])
            + (1.950349e-03) * (q[i0] * q[i22]) + (2.627176e-03) * (q[i1] * q[i2]) + (3.009775e-03) * (q[i1] * q[i3]) + (-3.075405e-03) * (q[i1] * q[i4])
            + (2.449185e-04) * (q[i1] * q[i5]) + (7.598896e-03) * (q[i1] * q[i7]) + (-1.137300e-03) * (q[i1] * q[i8]) + (5.926933e-03) * (q[i1] * q[i9])
            + (2.451999e-04) * (q[i1] * q[i10]) + (7.141237e-04) * (q[i1] * q[i11]) + (8.925012e-04) * (q[i1] * q[i12]) + (-2.981604e-03) * (q[i1] * q[i15])
            + (1.727913e-03) * (q[i1] * q[i16]) + (-1.552690e-04) * (q[i1] * q[i19]) + (-2.713295e-03) * (q[i1] * q[i20]) + (6.585911e-04) * (q[i1] * q[i21])
            + (-4.110103e-04) * (q[i1] * q[i22]) + (-3.436195e-03) * (q[i2] * q[i3]) + (8.420013e-06) * (q[i2] * q[i4]) + (-8.390429e-03) * (q[i2] * q[i5])
            + (1.407930e-02) * (q[i2] * q[i7]) + (-9.756797e-03) * (q[i2] * q[i8]) + (-4.292913e-03) * (q[i2] * q[i9]) + (1.744585e-03) * (q[i2] * q[i10])
            + (2.110339e-03) * (q[i2] * q[i11]) + (3.404679e-03) * (q[i2] * q[i12]) + (-2.437519e-03) * (q[i2] * q[i15]) + (1.432669e-03) * (q[i2] * q[i16])
            + (-9.689233e-04) * (q[i2] * q[i19]) + (1.426482e-03) * (q[i2] * q[i20]) + (-3.636610e-04) * (q[i2] * q[i21]) + (4.758287e-04) * (q[i2] * q[i22])
            + (8.451358e-03) * (q[i3] * q[i4]) + (-6.583003e-03) * (q[i3] * q[i5]) + (9.574128e-05) * (q[i3] * q[i7]) + (1.185074e-04) * (q[i3] * q[i8])
            + (-1.053298e-03) * (q[i3] * q[i9]) + (1.761238e-03) * (q[i3] * q[i10]) + (-3.321811e-03) * (q[i3] * q[i11]) + (5.265804e-03) * (q[i3] * q[i12])
            + (4.697587e-03) * (q[i3] * q[i15]) + (1.186916e-03) * (q[i3] * q[i16]) + (-1.181307e-03) * (q[i3] * q[i19]) + (2.162745e-03) * (q[i3] * q[i20])
            + (-1.000557e-03) * (q[i3] * q[i21]) + (3.790578e-03) * (q[i3] * q[i22]) + (6.493612e-03) * (q[i4] * q[i5]) + (1.566079e-04) * (q[i4] * q[i7])
            + (8.183180e-04) * (q[i4] * q[i8]) + (-2.593748e-03) * (q[i4] * q[i9]) + (-5.370685e-04) * (q[i4] * q[i10]) + (3.676719e-03) * (q[i4] * q[i11])
            + (-5.365699e-03) * (q[i4] * q[i12]) + (-4.621407e-03) * (q[i4] * q[i15]) + (3.924413e-03) * (q[i4] * q[i16]) + (5.798120e-03) * (q[i4] * q[i19])
            + (-2.300427e-04) * (q[i4] * q[i20]) + (3.513589e-04) * (q[i4] * q[i21]) + (-1.427510e-03) * (q[i4] * q[i22]) + (-1.077427e-02) * (q[i5] * q[i7])
            + (2.130050e-04) * (q[i5] * q[i8]) + (-3.894283e-03) * (q[i5] * q[i9]) + (-1.495500e-03) * (q[i5] * q[i10]) + (-1.098048e-03) * (q[i5] * q[i11])
            + (-1.625085e-03) * (q[i5] * q[i12]) + (-3.531705e-03) * (q[i5] * q[i15]) + (-2.354698e-03) * (q[i5] * q[i16]) + (2.702330e-03) * (q[i5] * q[i19])
            + (-9.051232e-04) * (q[i5] * q[i20]) + (1.591960e-03) * (q[i5] * q[i21]) + (-1.461438e-03) * (q[i5] * q[i22]) + (1.471681e-06) * (q[i7] * q[i8])
            + (3.538223e-03) * (q[i7] * q[i9]) + (-3.533052e-03) * (q[i7] * q[i10]) + (7.670973e-04) * (q[i7] * q[i11]) + (7.360295e-04) * (q[i7] * q[i12])
            + (-4.667196e-03) * (q[i7] * q[i15]) + (4.670192e-03) * (q[i7] * q[i16]) + (1.347722e-03) * (q[i7] * q[i19]) + (-1.363415e-03) * (q[i7] * q[i20])
            + (1.620153e-03) * (q[i7] * q[i21]) + (1.609069e-03) * (q[i7] * q[i22]) + (1.728767e-04) * (q[i8] * q[i9]) + (3.467390e-04) * (q[i8] * q[i10])
            + (2.328102e-03) * (q[i8] * q[i11]) + (-6.499768e-04) * (q[i8] * q[i12]) + (-1.649729e-03) * (q[i8] * q[i15]) + (-2.953595e-03) * (q[i8] * q[i16])
            + (9.782341e-04) * (q[i8] * q[i19]) + (3.762517e-03) * (q[i8] * q[i20]) + (-3.744837e-04) * (q[i8] * q[i21]) + (5.308637e-04) * (q[i8] * q[i22])
            + (-1.387681e-04) * (q[i9] * q[i10]) + (9.282152e-04) * (q[i9] * q[i11]) + (-1.665129e-03) * (q[i9] * q[i12]) + (-4.085242e-04) * (q[i9] * q[i15])
            + (-1.681619e-03) * (q[i9] * q[i16]) + (1.523656e-03) * (q[i9] * q[i19]) + (1.277208e-03) * (q[i9] * q[i20]) + (1.961747e-04) * (q[i9] * q[i21])
            + (-2.454917e-03) * (q[i9] * q[i22]) + (1.092517e-03) * (q[i10] * q[i11]) + (-3.319301e-03) * (q[i10] * q[i12])
            + (-3.119390e-03) * (q[i10] * q[i15]) + (8.119270e-04) * (q[i10] * q[i16]) + (1.324738e-03) * (q[i10] * q[i19]) + (7.252227e-04) * (q[i10] * q[i20])
            + (1.626460e-03) * (q[i10] * q[i21]) + (-4.929493e-04) * (q[i10] * q[i22]) + (-6.797917e-03) * (q[i11] * q[i12])
            + (-2.988266e-05) * (q[i11] * q[i15]) + (2.932153e-03) * (q[i11] * q[i16]) + (1.671965e-03) * (q[i11] * q[i19])
            + (-7.149938e-04) * (q[i11] * q[i20]) + (1.916454e-03) * (q[i11] * q[i21]) + (1.473500e-03) * (q[i11] * q[i22])
            + (-3.450668e-03) * (q[i12] * q[i15]) + (9.277737e-04) * (q[i12] * q[i16]) + (-3.422618e-04) * (q[i12] * q[i19])
            + (-1.686519e-03) * (q[i12] * q[i20]) + (-1.191898e-03) * (q[i12] * q[i21]) + (1.967394e-03) * (q[i12] * q[i22])
            + (-3.686725e-04) * (q[i15] * q[i16]) + (-1.345635e-03) * (q[i15] * q[i19]) + (-1.620104e-03) * (q[i15] * q[i20])
            + (-1.247835e-03) * (q[i15] * q[i21]) + (9.407677e-04) * (q[i15] * q[i22]) + (-3.332313e-03) * (q[i16] * q[i19])
            + (5.609957e-04) * (q[i16] * q[i20]) + (5.767641e-04) * (q[i16] * q[i21]) + (-1.211639e-05) * (q[i16] * q[i22]) + (1.521665e-03) * (q[i19] * q[i20])
            + (-1.122185e-03) * (q[i19] * q[i21]) + (1.952267e-04) * (q[i19] * q[i22]) + (-4.661550e-05) * (q[i20] * q[i21])
            + (-1.946770e-03) * (q[i20] * q[i22]) + (-3.455159e-04) * (q[i21] * q[i22]);
   }

   public void getJQz7(double[] q, double[][] JQ)
   {
      JQ[3][i7] = (-8.492880e-02) * (1) + (6.668476e-03) * ((2) * q[i7]) + (-1.074346e-02) * (q[i0]) + (1.575324e-02) * (q[i1]) + (1.409484e-02) * (q[i2])
            + (1.131108e-02) * (q[i3]) + (1.688534e-02) * (q[i4]) + (2.907652e-02) * (q[i5]) + (8.185942e-06) * (q[i6]) + (8.758807e-03) * (q[i8])
            + (-1.487626e-03) * (q[i9]) + (6.514480e-03) * (q[i10]) + (1.726801e-03) * (q[i11]) + (9.740498e-04) * (q[i12]) + (-1.104024e-02) * (q[i15])
            + (-2.191571e-03) * (q[i16]) + (5.833061e-03) * (q[i19]) + (8.629763e-03) * (q[i20]) + (2.335451e-03) * (q[i21]) + (-1.764857e-03) * (q[i22])
            + (-3.272551e-03) * (q[i0] * q[i0]) + (1.242606e-02) * (q[i1] * q[i1]) + (2.160540e-04) * (q[i2] * q[i2]) + (9.049718e-03) * (q[i3] * q[i3])
            + (-5.931101e-03) * (q[i4] * q[i4]) + (-2.879824e-04) * (q[i5] * q[i5]) + (1.289841e-03) * (q[i6] * q[i6]) + (6.715776e-03) * ((2) * q[i0] * q[i7])
            + (-1.079080e-02) * ((2) * q[i1] * q[i7]) + (-4.756895e-03) * ((2) * q[i2] * q[i7]) + (-3.817495e-03) * ((2) * q[i3] * q[i7])
            + (-5.239347e-03) * ((2) * q[i4] * q[i7]) + (8.560408e-03) * ((2) * q[i5] * q[i7]) + (-1.309745e-03) * ((2) * q[i6] * q[i7])
            + (3.920256e-03) * ((3) * q[i7] * q[i7]) + (-7.145583e-03) * ((2) * q[i7] * q[i8]) + (-2.885807e-03) * ((2) * q[i7] * q[i9])
            + (2.142158e-04) * ((2) * q[i7] * q[i10]) + (1.484495e-03) * ((2) * q[i7] * q[i11]) + (-4.665546e-03) * ((2) * q[i7] * q[i12])
            + (1.562721e-03) * ((2) * q[i7] * q[i15]) + (-2.543950e-03) * ((2) * q[i7] * q[i16]) + (8.816005e-04) * ((2) * q[i7] * q[i19])
            + (-2.149406e-04) * ((2) * q[i7] * q[i20]) + (-1.779634e-03) * ((2) * q[i7] * q[i21]) + (-1.211107e-03) * ((2) * q[i7] * q[i22])
            + (-1.682221e-03) * (q[i8] * q[i8]) + (-1.879608e-04) * (q[i9] * q[i9]) + (1.547509e-03) * (q[i10] * q[i10]) + (-2.718147e-03) * (q[i11] * q[i11])
            + (-1.990822e-03) * (q[i12] * q[i12]) + (-1.718359e-03) * (q[i15] * q[i15]) + (-1.365205e-03) * (q[i16] * q[i16])
            + (-1.300125e-03) * (q[i19] * q[i19]) + (-1.281165e-03) * (q[i20] * q[i20]) + (-1.741215e-03) * (q[i21] * q[i21])
            + (-1.603073e-03) * (q[i22] * q[i22]) + (2.757222e-04) * (q[i0] * q[i1]) + (-2.635861e-03) * (q[i0] * q[i2]) + (3.106750e-03) * (q[i0] * q[i3])
            + (-3.091155e-03) * (q[i0] * q[i4]) + (-2.363745e-04) * (q[i0] * q[i5]) + (7.590054e-03) * (q[i0] * q[i6]) + (-1.160031e-03) * (q[i0] * q[i8])
            + (2.473974e-04) * (q[i0] * q[i9]) + (5.886241e-03) * (q[i0] * q[i10]) + (-8.944924e-04) * (q[i0] * q[i11]) + (-6.621534e-04) * (q[i0] * q[i12])
            + (1.730123e-03) * (q[i0] * q[i15]) + (-2.987341e-03) * (q[i0] * q[i16]) + (-2.717691e-03) * (q[i0] * q[i19]) + (-1.157077e-04) * (q[i0] * q[i20])
            + (4.351924e-04) * (q[i0] * q[i21]) + (-6.437164e-04) * (q[i0] * q[i22]) + (8.588029e-03) * (q[i1] * q[i2]) + (-2.379827e-03) * (q[i1] * q[i3])
            + (1.026329e-02) * (q[i1] * q[i4]) + (-2.324987e-03) * (q[i1] * q[i5]) + (7.598896e-03) * (q[i1] * q[i6]) + (3.540632e-03) * (q[i1] * q[i8])
            + (6.815199e-03) * (q[i1] * q[i9]) + (-6.555638e-03) * (q[i1] * q[i10]) + (-1.815133e-03) * (q[i1] * q[i11]) + (-2.364231e-03) * (q[i1] * q[i12])
            + (2.516036e-03) * (q[i1] * q[i15]) + (-2.919010e-04) * (q[i1] * q[i16]) + (-7.084220e-04) * (q[i1] * q[i19]) + (-1.945346e-03) * (q[i1] * q[i20])
            + (-1.962824e-03) * (q[i1] * q[i21]) + (4.281154e-04) * (q[i1] * q[i22]) + (-5.176599e-05) * (q[i2] * q[i3]) + (3.457939e-03) * (q[i2] * q[i4])
            + (8.471359e-03) * (q[i2] * q[i5]) + (1.407930e-02) * (q[i2] * q[i6]) + (-9.730376e-03) * (q[i2] * q[i8]) + (1.755898e-03) * (q[i2] * q[i9])
            + (-4.274821e-03) * (q[i2] * q[i10]) + (-3.373453e-03) * (q[i2] * q[i11]) + (-2.075903e-03) * (q[i2] * q[i12]) + (1.418427e-03) * (q[i2] * q[i15])
            + (-2.459923e-03) * (q[i2] * q[i16]) + (1.409915e-03) * (q[i2] * q[i19]) + (-9.379687e-04) * (q[i2] * q[i20]) + (-4.657147e-04) * (q[i2] * q[i21])
            + (3.561437e-04) * (q[i2] * q[i22]) + (-8.385159e-03) * (q[i3] * q[i4]) + (-6.469581e-03) * (q[i3] * q[i5]) + (9.574128e-05) * (q[i3] * q[i6])
            + (8.310662e-04) * (q[i3] * q[i8]) + (-5.500530e-04) * (q[i3] * q[i9]) + (-2.524895e-03) * (q[i3] * q[i10]) + (5.338200e-03) * (q[i3] * q[i11])
            + (-3.673652e-03) * (q[i3] * q[i12]) + (3.929025e-03) * (q[i3] * q[i15]) + (-4.584537e-03) * (q[i3] * q[i16]) + (-2.230980e-04) * (q[i3] * q[i19])
            + (5.809542e-03) * (q[i3] * q[i20]) + (1.440824e-03) * (q[i3] * q[i21]) + (-3.218778e-04) * (q[i3] * q[i22]) + (6.544851e-03) * (q[i4] * q[i5])
            + (1.566079e-04) * (q[i4] * q[i6]) + (1.354132e-04) * (q[i4] * q[i8]) + (1.765688e-03) * (q[i4] * q[i9]) + (-1.119688e-03) * (q[i4] * q[i10])
            + (-5.271561e-03) * (q[i4] * q[i11]) + (3.306854e-03) * (q[i4] * q[i12]) + (1.152578e-03) * (q[i4] * q[i15]) + (4.677970e-03) * (q[i4] * q[i16])
            + (2.130440e-03) * (q[i4] * q[i19]) + (-1.210903e-03) * (q[i4] * q[i20]) + (-3.757145e-03) * (q[i4] * q[i21]) + (1.006111e-03) * (q[i4] * q[i22])
            + (-1.077427e-02) * (q[i5] * q[i6]) + (2.489370e-04) * (q[i5] * q[i8]) + (-1.484852e-03) * (q[i5] * q[i9]) + (-3.813284e-03) * (q[i5] * q[i10])
            + (1.649069e-03) * (q[i5] * q[i11]) + (1.139513e-03) * (q[i5] * q[i12]) + (-2.396054e-03) * (q[i5] * q[i15]) + (-3.510798e-03) * (q[i5] * q[i16])
            + (-8.926671e-04) * (q[i5] * q[i19]) + (2.701717e-03) * (q[i5] * q[i20]) + (1.449493e-03) * (q[i5] * q[i21]) + (-1.590293e-03) * (q[i5] * q[i22])
            + (1.471681e-06) * (q[i6] * q[i8]) + (3.538223e-03) * (q[i6] * q[i9]) + (-3.533052e-03) * (q[i6] * q[i10]) + (7.670973e-04) * (q[i6] * q[i11])
            + (7.360295e-04) * (q[i6] * q[i12]) + (-4.667196e-03) * (q[i6] * q[i15]) + (4.670192e-03) * (q[i6] * q[i16]) + (1.347722e-03) * (q[i6] * q[i19])
            + (-1.363415e-03) * (q[i6] * q[i20]) + (1.620153e-03) * (q[i6] * q[i21]) + (1.609069e-03) * (q[i6] * q[i22]) + (-3.371606e-04) * (q[i8] * q[i9])
            + (-1.569436e-04) * (q[i8] * q[i10]) + (-6.914101e-04) * (q[i8] * q[i11]) + (2.296906e-03) * (q[i8] * q[i12]) + (2.943766e-03) * (q[i8] * q[i15])
            + (1.613064e-03) * (q[i8] * q[i16]) + (-3.737186e-03) * (q[i8] * q[i19]) + (-9.455939e-04) * (q[i8] * q[i20]) + (5.372617e-04) * (q[i8] * q[i21])
            + (-3.697157e-04) * (q[i8] * q[i22]) + (1.400217e-04) * (q[i9] * q[i10]) + (-3.324927e-03) * (q[i9] * q[i11]) + (1.092796e-03) * (q[i9] * q[i12])
            + (-8.232981e-04) * (q[i9] * q[i15]) + (3.118214e-03) * (q[i9] * q[i16]) + (-7.282846e-04) * (q[i9] * q[i19]) + (-1.328406e-03) * (q[i9] * q[i20])
            + (-5.005814e-04) * (q[i9] * q[i21]) + (1.636119e-03) * (q[i9] * q[i22]) + (-1.687183e-03) * (q[i10] * q[i11]) + (9.381162e-04) * (q[i10] * q[i12])
            + (1.665228e-03) * (q[i10] * q[i15]) + (4.276289e-04) * (q[i10] * q[i16]) + (-1.287692e-03) * (q[i10] * q[i19])
            + (-1.523878e-03) * (q[i10] * q[i20]) + (-2.451621e-03) * (q[i10] * q[i21]) + (1.984686e-04) * (q[i10] * q[i22])
            + (6.788398e-03) * (q[i11] * q[i12]) + (9.438830e-04) * (q[i11] * q[i15]) + (-3.429429e-03) * (q[i11] * q[i16])
            + (-1.694066e-03) * (q[i11] * q[i19]) + (-3.149649e-04) * (q[i11] * q[i20]) + (-1.967779e-03) * (q[i11] * q[i21])
            + (1.176254e-03) * (q[i11] * q[i22]) + (2.947093e-03) * (q[i12] * q[i15]) + (-3.053283e-05) * (q[i12] * q[i16])
            + (-7.212440e-04) * (q[i12] * q[i19]) + (1.663596e-03) * (q[i12] * q[i20]) + (-1.454366e-03) * (q[i12] * q[i21])
            + (-1.900401e-03) * (q[i12] * q[i22]) + (3.657008e-04) * (q[i15] * q[i16]) + (-5.565482e-04) * (q[i15] * q[i19])
            + (3.309427e-03) * (q[i15] * q[i20]) + (-8.648373e-06) * (q[i15] * q[i21]) + (5.829073e-04) * (q[i15] * q[i22]) + (1.611454e-03) * (q[i16] * q[i19])
            + (1.343040e-03) * (q[i16] * q[i20]) + (9.408520e-04) * (q[i16] * q[i21]) + (-1.249415e-03) * (q[i16] * q[i22])
            + (-1.519264e-03) * (q[i19] * q[i20]) + (-1.940345e-03) * (q[i19] * q[i21]) + (-4.046829e-05) * (q[i19] * q[i22])
            + (2.033601e-04) * (q[i20] * q[i21]) + (-1.112846e-03) * (q[i20] * q[i22]) + (3.555269e-04) * (q[i21] * q[i22]);
   }

   public void getJQz8(double[] q, double[][] JQ)
   {
      JQ[3][i8] = (-2.080289e-04) * (1) + (9.495748e-05) * ((2) * q[i8]) + (8.653143e-03) * (q[i0]) + (8.756454e-03) * (q[i1]) + (-1.008699e-03) * (q[i2])
            + (-9.928663e-04) * (q[i3]) + (-9.773458e-04) * (q[i4]) + (5.116424e-02) * (q[i5]) + (-8.877688e-03) * (q[i6]) + (8.758807e-03) * (q[i7])
            + (-2.700869e-03) * (q[i9]) + (2.641316e-03) * (q[i10]) + (-3.691522e-03) * (q[i11]) + (-3.862373e-03) * (q[i12]) + (1.443567e-02) * (q[i15])
            + (-1.453026e-02) * (q[i16]) + (4.252995e-03) * (q[i19]) + (-4.199044e-03) * (q[i20]) + (3.168813e-03) * (q[i21]) + (3.195922e-03) * (q[i22])
            + (-9.077013e-04) * (q[i0] * q[i0]) + (8.847795e-04) * (q[i1] * q[i1]) + (6.674590e-06) * (q[i2] * q[i2]) + (4.594219e-03) * (q[i3] * q[i3])
            + (-4.616553e-03) * (q[i4] * q[i4]) + (7.539750e-05) * (q[i5] * q[i5]) + (7.130810e-03) * (q[i6] * q[i6]) + (-7.145583e-03) * (q[i7] * q[i7])
            + (8.763863e-04) * ((2) * q[i0] * q[i8]) + (8.621596e-04) * ((2) * q[i1] * q[i8]) + (6.311364e-03) * ((2) * q[i2] * q[i8])
            + (-4.006531e-03) * ((2) * q[i3] * q[i8]) + (-4.035328e-03) * ((2) * q[i4] * q[i8]) + (6.413173e-03) * ((2) * q[i5] * q[i8])
            + (1.706365e-03) * ((2) * q[i6] * q[i8]) + (-1.682221e-03) * ((2) * q[i7] * q[i8]) + (-2.481958e-05) * ((3) * q[i8] * q[i8])
            + (3.472965e-04) * ((2) * q[i8] * q[i9]) + (-3.438137e-04) * ((2) * q[i8] * q[i10]) + (-5.372922e-03) * ((2) * q[i8] * q[i11])
            + (-5.461197e-03) * ((2) * q[i8] * q[i12]) + (-1.147863e-03) * ((2) * q[i8] * q[i15]) + (1.224891e-03) * ((2) * q[i8] * q[i16])
            + (-1.183642e-03) * ((2) * q[i8] * q[i19]) + (1.153215e-03) * ((2) * q[i8] * q[i20]) + (1.220470e-03) * ((2) * q[i8] * q[i21])
            + (1.243642e-03) * ((2) * q[i8] * q[i22]) + (-5.020887e-04) * (q[i9] * q[i9]) + (5.061291e-04) * (q[i10] * q[i10])
            + (-2.565567e-03) * (q[i11] * q[i11]) + (2.620844e-03) * (q[i12] * q[i12]) + (-1.623978e-03) * (q[i15] * q[i15])
            + (1.643932e-03) * (q[i16] * q[i16]) + (-2.297524e-04) * (q[i19] * q[i19]) + (2.460478e-04) * (q[i20] * q[i20]) + (4.810772e-04) * (q[i21] * q[i21])
            + (-4.754923e-04) * (q[i22] * q[i22]) + (5.311938e-05) * (q[i0] * q[i1]) + (5.935676e-03) * (q[i0] * q[i2]) + (5.752914e-03) * (q[i0] * q[i3])
            + (2.799478e-03) * (q[i0] * q[i4]) + (3.065318e-03) * (q[i0] * q[i5]) + (3.540452e-03) * (q[i0] * q[i6]) + (-1.160031e-03) * (q[i0] * q[i7])
            + (3.435307e-04) * (q[i0] * q[i9]) + (-1.449731e-03) * (q[i0] * q[i10]) + (-1.501626e-04) * (q[i0] * q[i11]) + (3.196186e-04) * (q[i0] * q[i12])
            + (-1.777304e-03) * (q[i0] * q[i15]) + (-1.266048e-03) * (q[i0] * q[i16]) + (-9.278645e-04) * (q[i0] * q[i19]) + (2.572781e-03) * (q[i0] * q[i20])
            + (5.334901e-04) * (q[i0] * q[i21]) + (-1.699471e-03) * (q[i0] * q[i22]) + (-5.944304e-03) * (q[i1] * q[i2]) + (-2.802561e-03) * (q[i1] * q[i3])
            + (-5.652658e-03) * (q[i1] * q[i4]) + (-3.085534e-03) * (q[i1] * q[i5]) + (-1.137300e-03) * (q[i1] * q[i6]) + (3.540632e-03) * (q[i1] * q[i7])
            + (-1.458352e-03) * (q[i1] * q[i9]) + (3.259798e-04) * (q[i1] * q[i10]) + (-3.038879e-04) * (q[i1] * q[i11]) + (1.257303e-04) * (q[i1] * q[i12])
            + (-1.220725e-03) * (q[i1] * q[i15]) + (-1.803216e-03) * (q[i1] * q[i16]) + (2.591051e-03) * (q[i1] * q[i19]) + (-9.210163e-04) * (q[i1] * q[i20])
            + (1.685769e-03) * (q[i1] * q[i21]) + (-5.526813e-04) * (q[i1] * q[i22]) + (3.351840e-03) * (q[i2] * q[i3]) + (-3.298082e-03) * (q[i2] * q[i4])
            + (-1.490702e-05) * (q[i2] * q[i5]) + (-9.756797e-03) * (q[i2] * q[i6]) + (-9.730376e-03) * (q[i2] * q[i7]) + (3.126228e-04) * (q[i2] * q[i9])
            + (3.500331e-04) * (q[i2] * q[i10]) + (6.417629e-05) * (q[i2] * q[i11]) + (-1.014369e-04) * (q[i2] * q[i12]) + (-6.401808e-04) * (q[i2] * q[i15])
            + (-6.618732e-04) * (q[i2] * q[i16]) + (-2.143070e-03) * (q[i2] * q[i19]) + (-2.164839e-03) * (q[i2] * q[i20]) + (7.322682e-04) * (q[i2] * q[i21])
            + (-7.450876e-04) * (q[i2] * q[i22]) + (-5.215263e-05) * (q[i3] * q[i4]) + (-2.029328e-02) * (q[i3] * q[i5]) + (1.185074e-04) * (q[i3] * q[i6])
            + (8.310662e-04) * (q[i3] * q[i7]) + (8.914568e-04) * (q[i3] * q[i9]) + (-4.275390e-04) * (q[i3] * q[i10]) + (2.691313e-03) * (q[i3] * q[i11])
            + (2.354166e-03) * (q[i3] * q[i12]) + (-8.223876e-04) * (q[i3] * q[i15]) + (3.525317e-06) * (q[i3] * q[i16]) + (5.242117e-03) * (q[i3] * q[i19])
            + (-2.827921e-03) * (q[i3] * q[i20]) + (1.435959e-03) * (q[i3] * q[i21]) + (2.509093e-03) * (q[i3] * q[i22]) + (2.026581e-02) * (q[i4] * q[i5])
            + (8.183180e-04) * (q[i4] * q[i6]) + (1.354132e-04) * (q[i4] * q[i7]) + (-4.158017e-04) * (q[i4] * q[i9]) + (8.800150e-04) * (q[i4] * q[i10])
            + (-2.375399e-03) * (q[i4] * q[i11]) + (-2.759669e-03) * (q[i4] * q[i12]) + (1.167909e-05) * (q[i4] * q[i15]) + (-8.594415e-04) * (q[i4] * q[i16])
            + (-2.804724e-03) * (q[i4] * q[i19]) + (5.203166e-03) * (q[i4] * q[i20]) + (-2.477220e-03) * (q[i4] * q[i21]) + (-1.455301e-03) * (q[i4] * q[i22])
            + (2.130050e-04) * (q[i5] * q[i6]) + (2.489370e-04) * (q[i5] * q[i7]) + (2.075229e-03) * (q[i5] * q[i9]) + (2.087639e-03) * (q[i5] * q[i10])
            + (-5.032207e-03) * (q[i5] * q[i11]) + (5.177861e-03) * (q[i5] * q[i12]) + (-3.516862e-03) * (q[i5] * q[i15]) + (-3.532130e-03) * (q[i5] * q[i16])
            + (-1.052659e-03) * (q[i5] * q[i19]) + (-1.009716e-03) * (q[i5] * q[i20]) + (1.641170e-03) * (q[i5] * q[i21]) + (-1.689967e-03) * (q[i5] * q[i22])
            + (1.471681e-06) * (q[i6] * q[i7]) + (1.728767e-04) * (q[i6] * q[i9]) + (3.467390e-04) * (q[i6] * q[i10]) + (2.328102e-03) * (q[i6] * q[i11])
            + (-6.499768e-04) * (q[i6] * q[i12]) + (-1.649729e-03) * (q[i6] * q[i15]) + (-2.953595e-03) * (q[i6] * q[i16]) + (9.782341e-04) * (q[i6] * q[i19])
            + (3.762517e-03) * (q[i6] * q[i20]) + (-3.744837e-04) * (q[i6] * q[i21]) + (5.308637e-04) * (q[i6] * q[i22]) + (-3.371606e-04) * (q[i7] * q[i9])
            + (-1.569436e-04) * (q[i7] * q[i10]) + (-6.914101e-04) * (q[i7] * q[i11]) + (2.296906e-03) * (q[i7] * q[i12]) + (2.943766e-03) * (q[i7] * q[i15])
            + (1.613064e-03) * (q[i7] * q[i16]) + (-3.737186e-03) * (q[i7] * q[i19]) + (-9.455939e-04) * (q[i7] * q[i20]) + (5.372617e-04) * (q[i7] * q[i21])
            + (-3.697157e-04) * (q[i7] * q[i22]) + (7.190316e-06) * (q[i9] * q[i10]) + (1.505537e-03) * (q[i9] * q[i11]) + (7.128823e-04) * (q[i9] * q[i12])
            + (-2.079523e-03) * (q[i9] * q[i15]) + (-2.149540e-03) * (q[i9] * q[i16]) + (8.208506e-04) * (q[i9] * q[i19]) + (2.952076e-03) * (q[i9] * q[i20])
            + (-1.144188e-03) * (q[i9] * q[i21]) + (4.128965e-04) * (q[i9] * q[i22]) + (7.206284e-04) * (q[i10] * q[i11]) + (1.499905e-03) * (q[i10] * q[i12])
            + (2.132471e-03) * (q[i10] * q[i15]) + (2.067358e-03) * (q[i10] * q[i16]) + (-2.938725e-03) * (q[i10] * q[i19])
            + (-8.147241e-04) * (q[i10] * q[i20]) + (4.143830e-04) * (q[i10] * q[i21]) + (-1.149477e-03) * (q[i10] * q[i22])
            + (5.213333e-07) * (q[i11] * q[i12]) + (2.826673e-03) * (q[i11] * q[i15]) + (1.193063e-03) * (q[i11] * q[i16]) + (-1.131070e-03) * (q[i11] * q[i19])
            + (4.758059e-04) * (q[i11] * q[i20]) + (-3.066098e-04) * (q[i11] * q[i21]) + (-1.142485e-03) * (q[i11] * q[i22])
            + (1.212328e-03) * (q[i12] * q[i15]) + (2.828037e-03) * (q[i12] * q[i16]) + (4.809914e-04) * (q[i12] * q[i19]) + (-1.072583e-03) * (q[i12] * q[i20])
            + (1.149213e-03) * (q[i12] * q[i21]) + (3.046039e-04) * (q[i12] * q[i22]) + (7.098969e-07) * (q[i15] * q[i16]) + (1.069179e-03) * (q[i15] * q[i19])
            + (-7.574531e-04) * (q[i15] * q[i20]) + (2.629756e-03) * (q[i15] * q[i21]) + (-3.028679e-04) * (q[i15] * q[i22])
            + (7.779851e-04) * (q[i16] * q[i19]) + (-1.079222e-03) * (q[i16] * q[i20]) + (-2.999376e-04) * (q[i16] * q[i21])
            + (2.667583e-03) * (q[i16] * q[i22]) + (5.116259e-06) * (q[i19] * q[i20]) + (3.348708e-03) * (q[i19] * q[i21]) + (2.725898e-04) * (q[i19] * q[i22])
            + (2.678443e-04) * (q[i20] * q[i21]) + (3.337514e-03) * (q[i20] * q[i22]) + (7.139614e-06) * (q[i21] * q[i22]);
   }

   public void getJQz9(double[] q, double[][] JQ)
   {
      JQ[3][i9] = (3.172784e-02) * (1) + (-3.316302e-03) * ((2) * q[i9]) + (-5.261070e-03) * (q[i0]) + (-1.332474e-03) * (q[i1]) + (-1.280027e-03) * (q[i2])
            + (5.648699e-03) * (q[i3]) + (1.158554e-02) * (q[i4]) + (8.933518e-03) * (q[i5]) + (-6.607333e-03) * (q[i6]) + (-1.487626e-03) * (q[i7])
            + (-2.700869e-03) * (q[i8]) + (-3.860355e-06) * (q[i10]) + (-1.635497e-03) * (q[i11]) + (1.497003e-03) * (q[i12]) + (-2.044194e-03) * (q[i15])
            + (1.961958e-03) * (q[i16]) + (-4.272644e-03) * (q[i19]) + (-1.320874e-03) * (q[i20]) + (-4.451931e-04) * (q[i21]) + (2.513326e-03) * (q[i22])
            + (-2.269364e-03) * (q[i0] * q[i0]) + (2.143130e-03) * (q[i1] * q[i1]) + (4.256555e-04) * (q[i2] * q[i2]) + (8.430913e-04) * (q[i3] * q[i3])
            + (2.690506e-03) * (q[i4] * q[i4]) + (4.889481e-05) * (q[i5] * q[i5]) + (-2.098500e-04) * (q[i6] * q[i6]) + (-2.885807e-03) * (q[i7] * q[i7])
            + (3.472965e-04) * (q[i8] * q[i8]) + (3.571000e-03) * ((2) * q[i0] * q[i9]) + (2.252412e-04) * ((2) * q[i1] * q[i9])
            + (9.906023e-04) * ((2) * q[i2] * q[i9]) + (-1.039539e-03) * ((2) * q[i3] * q[i9]) + (-1.127005e-03) * ((2) * q[i4] * q[i9])
            + (-1.496355e-03) * ((2) * q[i5] * q[i9]) + (-1.563730e-03) * ((2) * q[i6] * q[i9]) + (-1.879608e-04) * ((2) * q[i7] * q[i9])
            + (-5.020887e-04) * ((2) * q[i8] * q[i9]) + (-7.815466e-04) * ((3) * q[i9] * q[i9]) + (-4.161969e-04) * ((2) * q[i9] * q[i10])
            + (7.893895e-04) * ((2) * q[i9] * q[i11]) + (-3.654373e-04) * ((2) * q[i9] * q[i12]) + (-1.776470e-04) * ((2) * q[i9] * q[i15])
            + (-8.181243e-04) * ((2) * q[i9] * q[i16]) + (5.688618e-04) * ((2) * q[i9] * q[i19]) + (5.050609e-04) * ((2) * q[i9] * q[i20])
            + (1.956634e-04) * ((2) * q[i9] * q[i21]) + (-7.894434e-04) * ((2) * q[i9] * q[i22]) + (4.154889e-04) * (q[i10] * q[i10])
            + (3.303996e-05) * (q[i11] * q[i11]) + (8.229866e-04) * (q[i12] * q[i12]) + (-9.476405e-05) * (q[i15] * q[i15])
            + (-4.486444e-04) * (q[i16] * q[i16]) + (3.851531e-04) * (q[i19] * q[i19]) + (-2.195414e-04) * (q[i20] * q[i20])
            + (3.766099e-04) * (q[i21] * q[i21]) + (-6.016814e-05) * (q[i22] * q[i22]) + (-1.650176e-03) * (q[i0] * q[i1]) + (-1.593483e-03) * (q[i0] * q[i2])
            + (6.880723e-03) * (q[i0] * q[i3]) + (5.130394e-03) * (q[i0] * q[i4]) + (2.392677e-03) * (q[i0] * q[i5]) + (-6.566775e-03) * (q[i0] * q[i6])
            + (2.473974e-04) * (q[i0] * q[i7]) + (3.435307e-04) * (q[i0] * q[i8]) + (1.333336e-03) * (q[i0] * q[i10]) + (9.486425e-06) * (q[i0] * q[i11])
            + (-4.151017e-04) * (q[i0] * q[i12]) + (1.080065e-04) * (q[i0] * q[i15]) + (-8.717527e-04) * (q[i0] * q[i16]) + (1.619438e-03) * (q[i0] * q[i19])
            + (-3.476660e-04) * (q[i0] * q[i20]) + (1.984431e-04) * (q[i0] * q[i21]) + (4.594173e-05) * (q[i0] * q[i22]) + (1.251727e-03) * (q[i1] * q[i2])
            + (-6.364877e-03) * (q[i1] * q[i3]) + (-4.481237e-04) * (q[i1] * q[i4]) + (-1.058509e-03) * (q[i1] * q[i5]) + (5.926933e-03) * (q[i1] * q[i6])
            + (6.815199e-03) * (q[i1] * q[i7]) + (-1.458352e-03) * (q[i1] * q[i8]) + (1.366859e-03) * (q[i1] * q[i10]) + (-1.395787e-03) * (q[i1] * q[i11])
            + (1.418468e-03) * (q[i1] * q[i12]) + (-2.835336e-04) * (q[i1] * q[i15]) + (-1.345912e-03) * (q[i1] * q[i16]) + (-6.331721e-04) * (q[i1] * q[i19])
            + (1.136149e-03) * (q[i1] * q[i20]) + (1.872351e-04) * (q[i1] * q[i21]) + (-6.430545e-04) * (q[i1] * q[i22]) + (1.625624e-03) * (q[i2] * q[i3])
            + (2.980178e-03) * (q[i2] * q[i4]) + (2.289242e-03) * (q[i2] * q[i5]) + (-4.292913e-03) * (q[i2] * q[i6]) + (1.755898e-03) * (q[i2] * q[i7])
            + (3.126228e-04) * (q[i2] * q[i8]) + (1.046591e-03) * (q[i2] * q[i10]) + (-1.559192e-03) * (q[i2] * q[i11]) + (7.825747e-04) * (q[i2] * q[i12])
            + (1.132058e-03) * (q[i2] * q[i15]) + (-1.885767e-03) * (q[i2] * q[i16]) + (-1.062082e-04) * (q[i2] * q[i19]) + (3.898031e-04) * (q[i2] * q[i20])
            + (9.727644e-05) * (q[i2] * q[i21]) + (-3.653917e-04) * (q[i2] * q[i22]) + (-2.546138e-03) * (q[i3] * q[i4]) + (1.136585e-03) * (q[i3] * q[i5])
            + (-1.053298e-03) * (q[i3] * q[i6]) + (-5.500530e-04) * (q[i3] * q[i7]) + (8.914568e-04) * (q[i3] * q[i8]) + (4.041426e-04) * (q[i3] * q[i10])
            + (-1.194464e-03) * (q[i3] * q[i11]) + (-6.590604e-04) * (q[i3] * q[i12]) + (-2.520779e-04) * (q[i3] * q[i15]) + (1.079600e-03) * (q[i3] * q[i16])
            + (1.489055e-03) * (q[i3] * q[i19]) + (6.169341e-04) * (q[i3] * q[i20]) + (-1.580515e-03) * (q[i3] * q[i21]) + (4.432069e-04) * (q[i3] * q[i22])
            + (7.047726e-05) * (q[i4] * q[i5]) + (-2.593748e-03) * (q[i4] * q[i6]) + (1.765688e-03) * (q[i4] * q[i7]) + (-4.158017e-04) * (q[i4] * q[i8])
            + (3.924239e-04) * (q[i4] * q[i10]) + (-3.106594e-04) * (q[i4] * q[i11]) + (-7.626711e-04) * (q[i4] * q[i12]) + (-2.307338e-03) * (q[i4] * q[i15])
            + (-3.622675e-05) * (q[i4] * q[i16]) + (-1.238296e-03) * (q[i4] * q[i19]) + (1.006603e-03) * (q[i4] * q[i20]) + (1.596703e-03) * (q[i4] * q[i21])
            + (2.096791e-03) * (q[i4] * q[i22]) + (-3.894283e-03) * (q[i5] * q[i6]) + (-1.484852e-03) * (q[i5] * q[i7]) + (2.075229e-03) * (q[i5] * q[i8])
            + (3.383062e-06) * (q[i5] * q[i10]) + (-4.488976e-04) * (q[i5] * q[i11]) + (3.316518e-04) * (q[i5] * q[i12]) + (6.361504e-04) * (q[i5] * q[i15])
            + (-8.130769e-04) * (q[i5] * q[i16]) + (3.115368e-04) * (q[i5] * q[i19]) + (-1.091896e-03) * (q[i5] * q[i20]) + (7.568795e-04) * (q[i5] * q[i21])
            + (-1.920676e-03) * (q[i5] * q[i22]) + (3.538223e-03) * (q[i6] * q[i7]) + (1.728767e-04) * (q[i6] * q[i8]) + (-1.387681e-04) * (q[i6] * q[i10])
            + (9.282152e-04) * (q[i6] * q[i11]) + (-1.665129e-03) * (q[i6] * q[i12]) + (-4.085242e-04) * (q[i6] * q[i15]) + (-1.681619e-03) * (q[i6] * q[i16])
            + (1.523656e-03) * (q[i6] * q[i19]) + (1.277208e-03) * (q[i6] * q[i20]) + (1.961747e-04) * (q[i6] * q[i21]) + (-2.454917e-03) * (q[i6] * q[i22])
            + (-3.371606e-04) * (q[i7] * q[i8]) + (1.400217e-04) * (q[i7] * q[i10]) + (-3.324927e-03) * (q[i7] * q[i11]) + (1.092796e-03) * (q[i7] * q[i12])
            + (-8.232981e-04) * (q[i7] * q[i15]) + (3.118214e-03) * (q[i7] * q[i16]) + (-7.282846e-04) * (q[i7] * q[i19]) + (-1.328406e-03) * (q[i7] * q[i20])
            + (-5.005814e-04) * (q[i7] * q[i21]) + (1.636119e-03) * (q[i7] * q[i22]) + (7.190316e-06) * (q[i8] * q[i10]) + (1.505537e-03) * (q[i8] * q[i11])
            + (7.128823e-04) * (q[i8] * q[i12]) + (-2.079523e-03) * (q[i8] * q[i15]) + (-2.149540e-03) * (q[i8] * q[i16]) + (8.208506e-04) * (q[i8] * q[i19])
            + (2.952076e-03) * (q[i8] * q[i20]) + (-1.144188e-03) * (q[i8] * q[i21]) + (4.128965e-04) * (q[i8] * q[i22]) + (-3.432840e-04) * (q[i10] * q[i11])
            + (-3.509618e-04) * (q[i10] * q[i12]) + (-7.261380e-04) * (q[i10] * q[i15]) + (7.210693e-04) * (q[i10] * q[i16])
            + (3.068323e-04) * (q[i10] * q[i19]) + (-3.114489e-04) * (q[i10] * q[i20]) + (2.671950e-04) * (q[i10] * q[i21]) + (2.713456e-04) * (q[i10] * q[i22])
            + (-1.383689e-03) * (q[i11] * q[i12]) + (4.185205e-04) * (q[i11] * q[i15]) + (6.748590e-05) * (q[i11] * q[i16]) + (2.570354e-04) * (q[i11] * q[i19])
            + (-2.698155e-04) * (q[i11] * q[i20]) + (5.498505e-04) * (q[i11] * q[i21]) + (5.739907e-04) * (q[i11] * q[i22])
            + (-5.104271e-04) * (q[i12] * q[i15]) + (8.176412e-04) * (q[i12] * q[i16]) + (8.131269e-05) * (q[i12] * q[i19])
            + (-9.948959e-06) * (q[i12] * q[i20]) + (-7.537221e-04) * (q[i12] * q[i21]) + (-2.366834e-04) * (q[i12] * q[i22])
            + (-6.006669e-04) * (q[i15] * q[i16]) + (-1.487320e-04) * (q[i15] * q[i19]) + (-2.653334e-05) * (q[i15] * q[i20])
            + (-6.297149e-04) * (q[i15] * q[i21]) + (-3.685585e-05) * (q[i15] * q[i22]) + (-9.418895e-04) * (q[i16] * q[i19])
            + (4.169658e-04) * (q[i16] * q[i20]) + (1.721087e-05) * (q[i16] * q[i21]) + (2.033018e-05) * (q[i16] * q[i22]) + (6.983687e-04) * (q[i19] * q[i20])
            + (-4.695622e-04) * (q[i19] * q[i21]) + (2.972330e-04) * (q[i19] * q[i22]) + (-1.456776e-05) * (q[i20] * q[i21])
            + (-3.028418e-04) * (q[i20] * q[i22]) + (-1.130490e-04) * (q[i21] * q[i22]);
   }

   public void getJQz10(double[] q, double[][] JQ)
   {
      JQ[3][i10] = (-3.150416e-02) * (1) + (3.293977e-03) * ((2) * q[i10]) + (-1.270144e-03) * (q[i0]) + (-5.206619e-03) * (q[i1]) + (-1.263228e-03) * (q[i2])
            + (1.150907e-02) * (q[i3]) + (5.640228e-03) * (q[i4]) + (8.822487e-03) * (q[i5]) + (1.429997e-03) * (q[i6]) + (6.514480e-03) * (q[i7])
            + (2.641316e-03) * (q[i8]) + (-3.860355e-06) * (q[i9]) + (1.510507e-03) * (q[i11]) + (-1.588390e-03) * (q[i12]) + (-1.900916e-03) * (q[i15])
            + (2.036227e-03) * (q[i16]) + (1.303528e-03) * (q[i19]) + (4.239309e-03) * (q[i20]) + (2.496257e-03) * (q[i21]) + (-4.460657e-04) * (q[i22])
            + (-2.136750e-03) * (q[i0] * q[i0]) + (2.239776e-03) * (q[i1] * q[i1]) + (-4.375921e-04) * (q[i2] * q[i2]) + (-2.710877e-03) * (q[i3] * q[i3])
            + (-8.586939e-04) * (q[i4] * q[i4]) + (-8.529623e-05) * (q[i5] * q[i5]) + (2.881432e-03) * (q[i6] * q[i6]) + (2.142158e-04) * (q[i7] * q[i7])
            + (-3.438137e-04) * (q[i8] * q[i8]) + (-4.161969e-04) * (q[i9] * q[i9]) + (2.137082e-04) * ((2) * q[i0] * q[i10])
            + (3.526691e-03) * ((2) * q[i1] * q[i10]) + (9.744991e-04) * ((2) * q[i2] * q[i10]) + (-1.109231e-03) * ((2) * q[i3] * q[i10])
            + (-1.073550e-03) * ((2) * q[i4] * q[i10]) + (-1.465258e-03) * ((2) * q[i5] * q[i10]) + (1.977798e-04) * ((2) * q[i6] * q[i10])
            + (1.547509e-03) * ((2) * q[i7] * q[i10]) + (5.061291e-04) * ((2) * q[i8] * q[i10]) + (4.154889e-04) * ((2) * q[i9] * q[i10])
            + (7.701880e-04) * ((3) * q[i10] * q[i10]) + (-3.646132e-04) * ((2) * q[i10] * q[i11]) + (7.796672e-04) * ((2) * q[i10] * q[i12])
            + (8.040850e-04) * ((2) * q[i10] * q[i15]) + (1.798451e-04) * ((2) * q[i10] * q[i16]) + (-5.029924e-04) * ((2) * q[i10] * q[i19])
            + (-5.685368e-04) * ((2) * q[i10] * q[i20]) + (-7.848734e-04) * ((2) * q[i10] * q[i21]) + (1.943379e-04) * ((2) * q[i10] * q[i22])
            + (-8.027005e-04) * (q[i11] * q[i11]) + (-1.848996e-05) * (q[i12] * q[i12]) + (4.446822e-04) * (q[i15] * q[i15])
            + (9.360250e-05) * (q[i16] * q[i16]) + (2.144442e-04) * (q[i19] * q[i19]) + (-3.820262e-04) * (q[i20] * q[i20]) + (6.369769e-05) * (q[i21] * q[i21])
            + (-3.702790e-04) * (q[i22] * q[i22]) + (1.641943e-03) * (q[i0] * q[i1]) + (-1.246070e-03) * (q[i0] * q[i2]) + (4.104927e-04) * (q[i0] * q[i3])
            + (6.354973e-03) * (q[i0] * q[i4]) + (1.078840e-03) * (q[i0] * q[i5]) + (6.778875e-03) * (q[i0] * q[i6]) + (5.886241e-03) * (q[i0] * q[i7])
            + (-1.449731e-03) * (q[i0] * q[i8]) + (1.333336e-03) * (q[i0] * q[i9]) + (-1.415883e-03) * (q[i0] * q[i11]) + (1.398713e-03) * (q[i0] * q[i12])
            + (-1.365183e-03) * (q[i0] * q[i15]) + (-2.779088e-04) * (q[i0] * q[i16]) + (1.148780e-03) * (q[i0] * q[i19]) + (-6.231768e-04) * (q[i0] * q[i20])
            + (6.428626e-04) * (q[i0] * q[i21]) + (-1.778341e-04) * (q[i0] * q[i22]) + (1.589570e-03) * (q[i1] * q[i2]) + (-5.111334e-03) * (q[i1] * q[i3])
            + (-6.823310e-03) * (q[i1] * q[i4]) + (-2.381350e-03) * (q[i1] * q[i5]) + (2.451999e-04) * (q[i1] * q[i6]) + (-6.555638e-03) * (q[i1] * q[i7])
            + (3.259798e-04) * (q[i1] * q[i8]) + (1.366859e-03) * (q[i1] * q[i9]) + (4.045033e-04) * (q[i1] * q[i11]) + (-1.472401e-05) * (q[i1] * q[i12])
            + (-8.608620e-04) * (q[i1] * q[i15]) + (1.288937e-04) * (q[i1] * q[i16]) + (-3.646995e-04) * (q[i1] * q[i19]) + (1.601052e-03) * (q[i1] * q[i20])
            + (-4.788341e-05) * (q[i1] * q[i21]) + (-1.747739e-04) * (q[i1] * q[i22]) + (-2.978439e-03) * (q[i2] * q[i3]) + (-1.570757e-03) * (q[i2] * q[i4])
            + (-2.286014e-03) * (q[i2] * q[i5]) + (1.744585e-03) * (q[i2] * q[i6]) + (-4.274821e-03) * (q[i2] * q[i7]) + (3.500331e-04) * (q[i2] * q[i8])
            + (1.046591e-03) * (q[i2] * q[i9]) + (-7.718939e-04) * (q[i2] * q[i11]) + (1.535351e-03) * (q[i2] * q[i12]) + (-1.879207e-03) * (q[i2] * q[i15])
            + (1.131607e-03) * (q[i2] * q[i16]) + (3.717942e-04) * (q[i2] * q[i19]) + (-1.153407e-04) * (q[i2] * q[i20]) + (3.553253e-04) * (q[i2] * q[i21])
            + (-8.714664e-05) * (q[i2] * q[i22]) + (2.573182e-03) * (q[i3] * q[i4]) + (-4.226047e-05) * (q[i3] * q[i5]) + (1.761238e-03) * (q[i3] * q[i6])
            + (-2.524895e-03) * (q[i3] * q[i7]) + (-4.275390e-04) * (q[i3] * q[i8]) + (4.041426e-04) * (q[i3] * q[i9]) + (7.508213e-04) * (q[i3] * q[i11])
            + (2.924787e-04) * (q[i3] * q[i12]) + (-4.046427e-05) * (q[i3] * q[i15]) + (-2.314218e-03) * (q[i3] * q[i16]) + (1.010646e-03) * (q[i3] * q[i19])
            + (-1.256187e-03) * (q[i3] * q[i20]) + (-2.090453e-03) * (q[i3] * q[i21]) + (-1.610804e-03) * (q[i3] * q[i22]) + (-1.138860e-03) * (q[i4] * q[i5])
            + (-5.370685e-04) * (q[i4] * q[i6]) + (-1.119688e-03) * (q[i4] * q[i7]) + (8.800150e-04) * (q[i4] * q[i8]) + (3.924239e-04) * (q[i4] * q[i9])
            + (6.662096e-04) * (q[i4] * q[i11]) + (1.201299e-03) * (q[i4] * q[i12]) + (1.070567e-03) * (q[i4] * q[i15]) + (-2.303241e-04) * (q[i4] * q[i16])
            + (5.875405e-04) * (q[i4] * q[i19]) + (1.476427e-03) * (q[i4] * q[i20]) + (-4.333095e-04) * (q[i4] * q[i21]) + (1.577342e-03) * (q[i4] * q[i22])
            + (-1.495500e-03) * (q[i5] * q[i6]) + (-3.813284e-03) * (q[i5] * q[i7]) + (2.087639e-03) * (q[i5] * q[i8]) + (3.383062e-06) * (q[i5] * q[i9])
            + (-3.314254e-04) * (q[i5] * q[i11]) + (4.846041e-04) * (q[i5] * q[i12]) + (-7.985036e-04) * (q[i5] * q[i15]) + (6.463539e-04) * (q[i5] * q[i16])
            + (-1.069672e-03) * (q[i5] * q[i19]) + (3.255600e-04) * (q[i5] * q[i20]) + (1.911686e-03) * (q[i5] * q[i21]) + (-7.536486e-04) * (q[i5] * q[i22])
            + (-3.533052e-03) * (q[i6] * q[i7]) + (3.467390e-04) * (q[i6] * q[i8]) + (-1.387681e-04) * (q[i6] * q[i9]) + (1.092517e-03) * (q[i6] * q[i11])
            + (-3.319301e-03) * (q[i6] * q[i12]) + (-3.119390e-03) * (q[i6] * q[i15]) + (8.119270e-04) * (q[i6] * q[i16]) + (1.324738e-03) * (q[i6] * q[i19])
            + (7.252227e-04) * (q[i6] * q[i20]) + (1.626460e-03) * (q[i6] * q[i21]) + (-4.929493e-04) * (q[i6] * q[i22]) + (-1.569436e-04) * (q[i7] * q[i8])
            + (1.400217e-04) * (q[i7] * q[i9]) + (-1.687183e-03) * (q[i7] * q[i11]) + (9.381162e-04) * (q[i7] * q[i12]) + (1.665228e-03) * (q[i7] * q[i15])
            + (4.276289e-04) * (q[i7] * q[i16]) + (-1.287692e-03) * (q[i7] * q[i19]) + (-1.523878e-03) * (q[i7] * q[i20]) + (-2.451621e-03) * (q[i7] * q[i21])
            + (1.984686e-04) * (q[i7] * q[i22]) + (7.190316e-06) * (q[i8] * q[i9]) + (7.206284e-04) * (q[i8] * q[i11]) + (1.499905e-03) * (q[i8] * q[i12])
            + (2.132471e-03) * (q[i8] * q[i15]) + (2.067358e-03) * (q[i8] * q[i16]) + (-2.938725e-03) * (q[i8] * q[i19]) + (-8.147241e-04) * (q[i8] * q[i20])
            + (4.143830e-04) * (q[i8] * q[i21]) + (-1.149477e-03) * (q[i8] * q[i22]) + (-3.432840e-04) * (q[i9] * q[i11]) + (-3.509618e-04) * (q[i9] * q[i12])
            + (-7.261380e-04) * (q[i9] * q[i15]) + (7.210693e-04) * (q[i9] * q[i16]) + (3.068323e-04) * (q[i9] * q[i19]) + (-3.114489e-04) * (q[i9] * q[i20])
            + (2.671950e-04) * (q[i9] * q[i21]) + (2.713456e-04) * (q[i9] * q[i22]) + (1.351670e-03) * (q[i11] * q[i12]) + (8.191657e-04) * (q[i11] * q[i15])
            + (-4.993969e-04) * (q[i11] * q[i16]) + (-6.430745e-06) * (q[i11] * q[i19]) + (9.173832e-05) * (q[i11] * q[i20])
            + (2.451832e-04) * (q[i11] * q[i21]) + (7.502987e-04) * (q[i11] * q[i22]) + (6.865590e-05) * (q[i12] * q[i15]) + (4.180841e-04) * (q[i12] * q[i16])
            + (-2.767815e-04) * (q[i12] * q[i19]) + (2.536174e-04) * (q[i12] * q[i20]) + (-5.718205e-04) * (q[i12] * q[i21])
            + (-5.451014e-04) * (q[i12] * q[i22]) + (6.058846e-04) * (q[i15] * q[i16]) + (-4.135366e-04) * (q[i15] * q[i19])
            + (9.230676e-04) * (q[i15] * q[i20]) + (2.476633e-05) * (q[i15] * q[i21]) + (2.225760e-05) * (q[i15] * q[i22]) + (2.947857e-05) * (q[i16] * q[i19])
            + (1.432046e-04) * (q[i16] * q[i20]) + (-4.318072e-05) * (q[i16] * q[i21]) + (-6.301378e-04) * (q[i16] * q[i22])
            + (-6.971758e-04) * (q[i19] * q[i20]) + (-3.065525e-04) * (q[i19] * q[i21]) + (-1.014092e-05) * (q[i19] * q[i22])
            + (2.987073e-04) * (q[i20] * q[i21]) + (-4.575649e-04) * (q[i20] * q[i22]) + (1.193717e-04) * (q[i21] * q[i22]);
   }

   public void getJQz11(double[] q, double[][] JQ)
   {
      JQ[3][i11] = (4.990228e-02) * (1) + (-9.116594e-04) * ((2) * q[i11]) + (6.303760e-04) * (q[i0]) + (4.869941e-04) * (q[i1]) + (2.007064e-03) * (q[i2])
            + (-1.591195e-03) * (q[i3]) + (2.082612e-03) * (q[i4]) + (-3.697470e-03) * (q[i5]) + (8.965253e-04) * (q[i6]) + (1.726801e-03) * (q[i7])
            + (-3.691522e-03) * (q[i8]) + (-1.635497e-03) * (q[i9]) + (1.510507e-03) * (q[i10]) + (2.681928e-06) * (q[i12]) + (2.564447e-03) * (q[i15])
            + (2.571983e-03) * (q[i16]) + (4.182586e-03) * (q[i19]) + (3.689563e-04) * (q[i20]) + (4.161639e-03) * (q[i21]) + (4.512438e-04) * (q[i22])
            + (-6.309729e-05) * (q[i0] * q[i0]) + (2.519575e-03) * (q[i1] * q[i1]) + (-1.959227e-03) * (q[i2] * q[i2]) + (3.105474e-03) * (q[i3] * q[i3])
            + (-3.515045e-04) * (q[i4] * q[i4]) + (3.295042e-03) * (q[i5] * q[i5]) + (-4.684032e-03) * (q[i6] * q[i6]) + (1.484495e-03) * (q[i7] * q[i7])
            + (-5.372922e-03) * (q[i8] * q[i8]) + (7.893895e-04) * (q[i9] * q[i9]) + (-3.646132e-04) * (q[i10] * q[i10])
            + (-3.013394e-04) * ((2) * q[i0] * q[i11]) + (-1.769956e-05) * ((2) * q[i1] * q[i11]) + (-2.203007e-03) * ((2) * q[i2] * q[i11])
            + (-3.986162e-04) * ((2) * q[i3] * q[i11]) + (-2.803565e-03) * ((2) * q[i4] * q[i11]) + (-4.641605e-04) * ((2) * q[i5] * q[i11])
            + (2.021229e-03) * ((2) * q[i6] * q[i11]) + (-2.718147e-03) * ((2) * q[i7] * q[i11]) + (-2.565567e-03) * ((2) * q[i8] * q[i11])
            + (3.303996e-05) * ((2) * q[i9] * q[i11]) + (-8.027005e-04) * ((2) * q[i10] * q[i11]) + (-8.028001e-04) * ((3) * q[i11] * q[i11])
            + (5.829736e-04) * ((2) * q[i11] * q[i12]) + (1.444886e-03) * ((2) * q[i11] * q[i15]) + (-5.429432e-04) * ((2) * q[i11] * q[i16])
            + (-1.210977e-03) * ((2) * q[i11] * q[i19]) + (-2.343149e-05) * ((2) * q[i11] * q[i20]) + (-1.088728e-03) * ((2) * q[i11] * q[i21])
            + (2.058726e-04) * ((2) * q[i11] * q[i22]) + (5.870360e-04) * (q[i12] * q[i12]) + (-5.868728e-03) * (q[i15] * q[i15])
            + (4.659210e-04) * (q[i16] * q[i16]) + (6.159593e-04) * (q[i19] * q[i19]) + (-1.191432e-04) * (q[i20] * q[i20])
            + (-6.023134e-04) * (q[i21] * q[i21]) + (-4.069585e-04) * (q[i22] * q[i22]) + (-6.978984e-04) * (q[i0] * q[i1]) + (5.106428e-04) * (q[i0] * q[i2])
            + (-1.905806e-03) * (q[i0] * q[i3]) + (-4.735293e-04) * (q[i0] * q[i4]) + (1.322023e-03) * (q[i0] * q[i5]) + (2.362114e-03) * (q[i0] * q[i6])
            + (-8.944924e-04) * (q[i0] * q[i7]) + (-1.501626e-04) * (q[i0] * q[i8]) + (9.486425e-06) * (q[i0] * q[i9]) + (-1.415883e-03) * (q[i0] * q[i10])
            + (5.165199e-04) * (q[i0] * q[i12]) + (-1.382556e-03) * (q[i0] * q[i15]) + (-2.685371e-04) * (q[i0] * q[i16]) + (1.617007e-03) * (q[i0] * q[i19])
            + (-1.578691e-03) * (q[i0] * q[i20]) + (-8.476260e-04) * (q[i0] * q[i21]) + (-1.043071e-03) * (q[i0] * q[i22]) + (-8.452529e-04) * (q[i1] * q[i2])
            + (3.375875e-03) * (q[i1] * q[i3]) + (-4.153900e-03) * (q[i1] * q[i4]) + (1.337867e-03) * (q[i1] * q[i5]) + (7.141237e-04) * (q[i1] * q[i6])
            + (-1.815133e-03) * (q[i1] * q[i7]) + (-3.038879e-04) * (q[i1] * q[i8]) + (-1.395787e-03) * (q[i1] * q[i9]) + (4.045033e-04) * (q[i1] * q[i10])
            + (5.198115e-04) * (q[i1] * q[i12]) + (3.872995e-03) * (q[i1] * q[i15]) + (-1.095867e-03) * (q[i1] * q[i16]) + (-2.231571e-03) * (q[i1] * q[i19])
            + (-6.580470e-04) * (q[i1] * q[i20]) + (6.031111e-04) * (q[i1] * q[i21]) + (1.702132e-03) * (q[i1] * q[i22]) + (1.251982e-03) * (q[i2] * q[i3])
            + (-3.083760e-03) * (q[i2] * q[i4]) + (-1.072867e-03) * (q[i2] * q[i5]) + (2.110339e-03) * (q[i2] * q[i6]) + (-3.373453e-03) * (q[i2] * q[i7])
            + (6.417629e-05) * (q[i2] * q[i8]) + (-1.559192e-03) * (q[i2] * q[i9]) + (-7.718939e-04) * (q[i2] * q[i10]) + (2.987426e-03) * (q[i2] * q[i12])
            + (2.351462e-03) * (q[i2] * q[i15]) + (-1.103160e-03) * (q[i2] * q[i16]) + (-7.304907e-04) * (q[i2] * q[i19]) + (6.020709e-04) * (q[i2] * q[i20])
            + (-8.595518e-04) * (q[i2] * q[i21]) + (-6.161720e-04) * (q[i2] * q[i22]) + (1.651983e-04) * (q[i3] * q[i4]) + (1.932446e-04) * (q[i3] * q[i5])
            + (-3.321811e-03) * (q[i3] * q[i6]) + (5.338200e-03) * (q[i3] * q[i7]) + (2.691313e-03) * (q[i3] * q[i8]) + (-1.194464e-03) * (q[i3] * q[i9])
            + (7.508213e-04) * (q[i3] * q[i10]) + (4.219337e-04) * (q[i3] * q[i12]) + (-1.423635e-03) * (q[i3] * q[i15]) + (4.422790e-04) * (q[i3] * q[i16])
            + (-1.174579e-03) * (q[i3] * q[i19]) + (-2.797302e-03) * (q[i3] * q[i20]) + (-1.517272e-03) * (q[i3] * q[i21]) + (-7.618473e-04) * (q[i3] * q[i22])
            + (-3.948614e-03) * (q[i4] * q[i5]) + (3.676719e-03) * (q[i4] * q[i6]) + (-5.271561e-03) * (q[i4] * q[i7]) + (-2.375399e-03) * (q[i4] * q[i8])
            + (-3.106594e-04) * (q[i4] * q[i9]) + (6.662096e-04) * (q[i4] * q[i10]) + (4.096850e-04) * (q[i4] * q[i12]) + (4.705882e-04) * (q[i4] * q[i15])
            + (4.229579e-04) * (q[i4] * q[i16]) + (2.112294e-03) * (q[i4] * q[i19]) + (4.808062e-03) * (q[i4] * q[i20]) + (1.044082e-04) * (q[i4] * q[i21])
            + (-5.197775e-04) * (q[i4] * q[i22]) + (-1.098048e-03) * (q[i5] * q[i6]) + (1.649069e-03) * (q[i5] * q[i7]) + (-5.032207e-03) * (q[i5] * q[i8])
            + (-4.488976e-04) * (q[i5] * q[i9]) + (-3.314254e-04) * (q[i5] * q[i10]) + (-5.596421e-05) * (q[i5] * q[i12]) + (-3.651541e-03) * (q[i5] * q[i15])
            + (1.199525e-03) * (q[i5] * q[i16]) + (-7.826358e-04) * (q[i5] * q[i19]) + (8.982612e-04) * (q[i5] * q[i20]) + (-2.270249e-04) * (q[i5] * q[i21])
            + (7.702120e-04) * (q[i5] * q[i22]) + (7.670973e-04) * (q[i6] * q[i7]) + (2.328102e-03) * (q[i6] * q[i8]) + (9.282152e-04) * (q[i6] * q[i9])
            + (1.092517e-03) * (q[i6] * q[i10]) + (-6.797917e-03) * (q[i6] * q[i12]) + (-2.988266e-05) * (q[i6] * q[i15]) + (2.932153e-03) * (q[i6] * q[i16])
            + (1.671965e-03) * (q[i6] * q[i19]) + (-7.149938e-04) * (q[i6] * q[i20]) + (1.916454e-03) * (q[i6] * q[i21]) + (1.473500e-03) * (q[i6] * q[i22])
            + (-6.914101e-04) * (q[i7] * q[i8]) + (-3.324927e-03) * (q[i7] * q[i9]) + (-1.687183e-03) * (q[i7] * q[i10]) + (6.788398e-03) * (q[i7] * q[i12])
            + (9.438830e-04) * (q[i7] * q[i15]) + (-3.429429e-03) * (q[i7] * q[i16]) + (-1.694066e-03) * (q[i7] * q[i19]) + (-3.149649e-04) * (q[i7] * q[i20])
            + (-1.967779e-03) * (q[i7] * q[i21]) + (1.176254e-03) * (q[i7] * q[i22]) + (1.505537e-03) * (q[i8] * q[i9]) + (7.206284e-04) * (q[i8] * q[i10])
            + (5.213333e-07) * (q[i8] * q[i12]) + (2.826673e-03) * (q[i8] * q[i15]) + (1.193063e-03) * (q[i8] * q[i16]) + (-1.131070e-03) * (q[i8] * q[i19])
            + (4.758059e-04) * (q[i8] * q[i20]) + (-3.066098e-04) * (q[i8] * q[i21]) + (-1.142485e-03) * (q[i8] * q[i22]) + (-3.432840e-04) * (q[i9] * q[i10])
            + (-1.383689e-03) * (q[i9] * q[i12]) + (4.185205e-04) * (q[i9] * q[i15]) + (6.748590e-05) * (q[i9] * q[i16]) + (2.570354e-04) * (q[i9] * q[i19])
            + (-2.698155e-04) * (q[i9] * q[i20]) + (5.498505e-04) * (q[i9] * q[i21]) + (5.739907e-04) * (q[i9] * q[i22]) + (1.351670e-03) * (q[i10] * q[i12])
            + (8.191657e-04) * (q[i10] * q[i15]) + (-4.993969e-04) * (q[i10] * q[i16]) + (-6.430745e-06) * (q[i10] * q[i19])
            + (9.173832e-05) * (q[i10] * q[i20]) + (2.451832e-04) * (q[i10] * q[i21]) + (7.502987e-04) * (q[i10] * q[i22]) + (-3.824057e-04) * (q[i12] * q[i15])
            + (3.722676e-04) * (q[i12] * q[i16]) + (-5.943896e-04) * (q[i12] * q[i19]) + (6.070909e-04) * (q[i12] * q[i20]) + (1.004132e-04) * (q[i12] * q[i21])
            + (1.110204e-04) * (q[i12] * q[i22]) + (7.387768e-04) * (q[i15] * q[i16]) + (8.488030e-04) * (q[i15] * q[i19]) + (-1.645095e-04) * (q[i15] * q[i20])
            + (1.516783e-03) * (q[i15] * q[i21]) + (-1.931456e-04) * (q[i15] * q[i22]) + (5.898340e-04) * (q[i16] * q[i19])
            + (-9.083006e-04) * (q[i16] * q[i20]) + (1.304451e-04) * (q[i16] * q[i21]) + (-9.794195e-05) * (q[i16] * q[i22])
            + (3.248382e-04) * (q[i19] * q[i20]) + (1.886006e-03) * (q[i19] * q[i21]) + (7.798466e-04) * (q[i19] * q[i22]) + (3.111280e-04) * (q[i20] * q[i21])
            + (-3.240401e-04) * (q[i20] * q[i22]) + (2.629995e-04) * (q[i21] * q[i22]);
   }

   public void getJQz12(double[] q, double[][] JQ)
   {
      JQ[3][i12] = (5.017613e-02) * (1) + (1.148862e-03) * ((2) * q[i12]) + (-3.051713e-04) * (q[i0]) + (-5.109633e-04) * (q[i1]) + (-1.752413e-03) * (q[i2])
            + (-2.040305e-03) * (q[i3]) + (1.588879e-03) * (q[i4]) + (3.888892e-03) * (q[i5]) + (1.560899e-03) * (q[i6]) + (9.740498e-04) * (q[i7])
            + (-3.862373e-03) * (q[i8]) + (1.497003e-03) * (q[i9]) + (-1.588390e-03) * (q[i10]) + (2.681928e-06) * (q[i11]) + (2.559983e-03) * (q[i15])
            + (2.497307e-03) * (q[i16]) + (3.930770e-04) * (q[i19]) + (4.151972e-03) * (q[i20]) + (-4.494994e-04) * (q[i21]) + (-4.286199e-03) * (q[i22])
            + (2.519247e-03) * (q[i0] * q[i0]) + (-6.138320e-05) * (q[i1] * q[i1]) + (-1.950016e-03) * (q[i2] * q[i2]) + (-3.467011e-04) * (q[i3] * q[i3])
            + (3.047201e-03) * (q[i4] * q[i4]) + (3.401737e-03) * (q[i5] * q[i5]) + (1.484473e-03) * (q[i6] * q[i6]) + (-4.665546e-03) * (q[i7] * q[i7])
            + (-5.461197e-03) * (q[i8] * q[i8]) + (-3.654373e-04) * (q[i9] * q[i9]) + (7.796672e-04) * (q[i10] * q[i10]) + (5.829736e-04) * (q[i11] * q[i11])
            + (-4.926891e-06) * ((2) * q[i0] * q[i12]) + (-3.530106e-04) * ((2) * q[i1] * q[i12]) + (-2.230185e-03) * ((2) * q[i2] * q[i12])
            + (-2.886119e-03) * ((2) * q[i3] * q[i12]) + (-3.361134e-04) * ((2) * q[i4] * q[i12]) + (-4.418924e-04) * ((2) * q[i5] * q[i12])
            + (2.741810e-03) * ((2) * q[i6] * q[i12]) + (-1.990822e-03) * ((2) * q[i7] * q[i12]) + (2.620844e-03) * ((2) * q[i8] * q[i12])
            + (8.229866e-04) * ((2) * q[i9] * q[i12]) + (-1.848996e-05) * ((2) * q[i10] * q[i12]) + (5.870360e-04) * ((2) * q[i11] * q[i12])
            + (-8.365510e-04) * ((3) * q[i12] * q[i12]) + (5.440826e-04) * ((2) * q[i12] * q[i15]) + (-1.440262e-03) * ((2) * q[i12] * q[i16])
            + (4.530601e-06) * ((2) * q[i12] * q[i19]) + (1.180369e-03) * ((2) * q[i12] * q[i20]) + (2.075158e-04) * ((2) * q[i12] * q[i21])
            + (-1.071907e-03) * ((2) * q[i12] * q[i22]) + (4.680595e-04) * (q[i15] * q[i15]) + (-5.938069e-03) * (q[i16] * q[i16])
            + (-1.264439e-04) * (q[i19] * q[i19]) + (6.084688e-04) * (q[i20] * q[i20]) + (-4.094983e-04) * (q[i21] * q[i21])
            + (-5.864783e-04) * (q[i22] * q[i22]) + (-6.819984e-04) * (q[i0] * q[i1]) + (-8.160733e-04) * (q[i0] * q[i2]) + (-4.130384e-03) * (q[i0] * q[i3])
            + (3.386984e-03) * (q[i0] * q[i4]) + (1.282937e-03) * (q[i0] * q[i5]) + (1.859035e-03) * (q[i0] * q[i6]) + (-6.621534e-04) * (q[i0] * q[i7])
            + (3.196186e-04) * (q[i0] * q[i8]) + (-4.151017e-04) * (q[i0] * q[i9]) + (1.398713e-03) * (q[i0] * q[i10]) + (5.165199e-04) * (q[i0] * q[i11])
            + (1.106490e-03) * (q[i0] * q[i15]) + (-3.871843e-03) * (q[i0] * q[i16]) + (6.751019e-04) * (q[i0] * q[i19]) + (2.247609e-03) * (q[i0] * q[i20])
            + (1.695129e-03) * (q[i0] * q[i21]) + (6.040034e-04) * (q[i0] * q[i22]) + (5.137758e-04) * (q[i1] * q[i2]) + (-5.158804e-04) * (q[i1] * q[i3])
            + (-1.935043e-03) * (q[i1] * q[i4]) + (1.235463e-03) * (q[i1] * q[i5]) + (8.925012e-04) * (q[i1] * q[i6]) + (-2.364231e-03) * (q[i1] * q[i7])
            + (1.257303e-04) * (q[i1] * q[i8]) + (1.418468e-03) * (q[i1] * q[i9]) + (-1.472401e-05) * (q[i1] * q[i10]) + (5.198115e-04) * (q[i1] * q[i11])
            + (2.860590e-04) * (q[i1] * q[i15]) + (1.377429e-03) * (q[i1] * q[i16]) + (1.575255e-03) * (q[i1] * q[i19]) + (-1.639693e-03) * (q[i1] * q[i20])
            + (-1.023231e-03) * (q[i1] * q[i21]) + (-8.373231e-04) * (q[i1] * q[i22]) + (-3.162736e-03) * (q[i2] * q[i3]) + (1.235821e-03) * (q[i2] * q[i4])
            + (-1.137662e-03) * (q[i2] * q[i5]) + (3.404679e-03) * (q[i2] * q[i6]) + (-2.075903e-03) * (q[i2] * q[i7]) + (-1.014369e-04) * (q[i2] * q[i8])
            + (7.825747e-04) * (q[i2] * q[i9]) + (1.535351e-03) * (q[i2] * q[i10]) + (2.987426e-03) * (q[i2] * q[i11]) + (1.115856e-03) * (q[i2] * q[i15])
            + (-2.369277e-03) * (q[i2] * q[i16]) + (-6.144859e-04) * (q[i2] * q[i19]) + (7.187429e-04) * (q[i2] * q[i20]) + (-6.115261e-04) * (q[i2] * q[i21])
            + (-8.317397e-04) * (q[i2] * q[i22]) + (2.118888e-04) * (q[i3] * q[i4]) + (-4.053934e-03) * (q[i3] * q[i5]) + (5.265804e-03) * (q[i3] * q[i6])
            + (-3.673652e-03) * (q[i3] * q[i7]) + (2.354166e-03) * (q[i3] * q[i8]) + (-6.590604e-04) * (q[i3] * q[i9]) + (2.924787e-04) * (q[i3] * q[i10])
            + (4.219337e-04) * (q[i3] * q[i11]) + (-4.134908e-04) * (q[i3] * q[i15]) + (-4.669852e-04) * (q[i3] * q[i16]) + (-4.828984e-03) * (q[i3] * q[i19])
            + (-2.117439e-03) * (q[i3] * q[i20]) + (-5.185118e-04) * (q[i3] * q[i21]) + (9.435594e-05) * (q[i3] * q[i22]) + (1.697960e-04) * (q[i4] * q[i5])
            + (-5.365699e-03) * (q[i4] * q[i6]) + (3.306854e-03) * (q[i4] * q[i7]) + (-2.759669e-03) * (q[i4] * q[i8]) + (-7.626711e-04) * (q[i4] * q[i9])
            + (1.201299e-03) * (q[i4] * q[i10]) + (4.096850e-04) * (q[i4] * q[i11]) + (-4.336719e-04) * (q[i4] * q[i15]) + (1.438830e-03) * (q[i4] * q[i16])
            + (2.819507e-03) * (q[i4] * q[i19]) + (1.160681e-03) * (q[i4] * q[i20]) + (-7.572880e-04) * (q[i4] * q[i21]) + (-1.530939e-03) * (q[i4] * q[i22])
            + (-1.625085e-03) * (q[i5] * q[i6]) + (1.139513e-03) * (q[i5] * q[i7]) + (5.177861e-03) * (q[i5] * q[i8]) + (3.316518e-04) * (q[i5] * q[i9])
            + (4.846041e-04) * (q[i5] * q[i10]) + (-5.596421e-05) * (q[i5] * q[i11]) + (-1.202181e-03) * (q[i5] * q[i15]) + (3.666847e-03) * (q[i5] * q[i16])
            + (-9.107160e-04) * (q[i5] * q[i19]) + (7.582725e-04) * (q[i5] * q[i20]) + (7.650291e-04) * (q[i5] * q[i21]) + (-2.058770e-04) * (q[i5] * q[i22])
            + (7.360295e-04) * (q[i6] * q[i7]) + (-6.499768e-04) * (q[i6] * q[i8]) + (-1.665129e-03) * (q[i6] * q[i9]) + (-3.319301e-03) * (q[i6] * q[i10])
            + (-6.797917e-03) * (q[i6] * q[i11]) + (-3.450668e-03) * (q[i6] * q[i15]) + (9.277737e-04) * (q[i6] * q[i16]) + (-3.422618e-04) * (q[i6] * q[i19])
            + (-1.686519e-03) * (q[i6] * q[i20]) + (-1.191898e-03) * (q[i6] * q[i21]) + (1.967394e-03) * (q[i6] * q[i22]) + (2.296906e-03) * (q[i7] * q[i8])
            + (1.092796e-03) * (q[i7] * q[i9]) + (9.381162e-04) * (q[i7] * q[i10]) + (6.788398e-03) * (q[i7] * q[i11]) + (2.947093e-03) * (q[i7] * q[i15])
            + (-3.053283e-05) * (q[i7] * q[i16]) + (-7.212440e-04) * (q[i7] * q[i19]) + (1.663596e-03) * (q[i7] * q[i20]) + (-1.454366e-03) * (q[i7] * q[i21])
            + (-1.900401e-03) * (q[i7] * q[i22]) + (7.128823e-04) * (q[i8] * q[i9]) + (1.499905e-03) * (q[i8] * q[i10]) + (5.213333e-07) * (q[i8] * q[i11])
            + (1.212328e-03) * (q[i8] * q[i15]) + (2.828037e-03) * (q[i8] * q[i16]) + (4.809914e-04) * (q[i8] * q[i19]) + (-1.072583e-03) * (q[i8] * q[i20])
            + (1.149213e-03) * (q[i8] * q[i21]) + (3.046039e-04) * (q[i8] * q[i22]) + (-3.509618e-04) * (q[i9] * q[i10]) + (-1.383689e-03) * (q[i9] * q[i11])
            + (-5.104271e-04) * (q[i9] * q[i15]) + (8.176412e-04) * (q[i9] * q[i16]) + (8.131269e-05) * (q[i9] * q[i19]) + (-9.948959e-06) * (q[i9] * q[i20])
            + (-7.537221e-04) * (q[i9] * q[i21]) + (-2.366834e-04) * (q[i9] * q[i22]) + (1.351670e-03) * (q[i10] * q[i11]) + (6.865590e-05) * (q[i10] * q[i15])
            + (4.180841e-04) * (q[i10] * q[i16]) + (-2.767815e-04) * (q[i10] * q[i19]) + (2.536174e-04) * (q[i10] * q[i20])
            + (-5.718205e-04) * (q[i10] * q[i21]) + (-5.451014e-04) * (q[i10] * q[i22]) + (-3.824057e-04) * (q[i11] * q[i15])
            + (3.722676e-04) * (q[i11] * q[i16]) + (-5.943896e-04) * (q[i11] * q[i19]) + (6.070909e-04) * (q[i11] * q[i20]) + (1.004132e-04) * (q[i11] * q[i21])
            + (1.110204e-04) * (q[i11] * q[i22]) + (7.343239e-04) * (q[i15] * q[i16]) + (-9.047756e-04) * (q[i15] * q[i19]) + (5.885397e-04) * (q[i15] * q[i20])
            + (9.203270e-05) * (q[i15] * q[i21]) + (-1.425435e-04) * (q[i15] * q[i22]) + (-1.627775e-04) * (q[i16] * q[i19])
            + (8.694204e-04) * (q[i16] * q[i20]) + (1.930178e-04) * (q[i16] * q[i21]) + (-1.559937e-03) * (q[i16] * q[i22]) + (3.196399e-04) * (q[i19] * q[i20])
            + (3.213397e-04) * (q[i19] * q[i21]) + (-3.151055e-04) * (q[i19] * q[i22]) + (-7.816191e-04) * (q[i20] * q[i21])
            + (-1.866132e-03) * (q[i20] * q[i22]) + (2.602461e-04) * (q[i21] * q[i22]);
   }

   public void getJQz15(double[] q, double[][] JQ)
   {
      JQ[3][i15] = (1.203367e-02) * (1) + (4.929370e-03) * ((2) * q[i15]) + (-8.068139e-03) * (q[i0]) + (-6.918457e-03) * (q[i1]) + (-1.796744e-02) * (q[i2])
            + (7.821852e-03) * (q[i3]) + (-2.693480e-04) * (q[i4]) + (-1.199885e-03) * (q[i5]) + (2.112994e-03) * (q[i6]) + (-1.104024e-02) * (q[i7])
            + (1.443567e-02) * (q[i8]) + (-2.044194e-03) * (q[i9]) + (-1.900916e-03) * (q[i10]) + (2.564447e-03) * (q[i11]) + (2.559983e-03) * (q[i12])
            + (6.650289e-06) * (q[i16]) + (4.703209e-03) * (q[i19]) + (-1.427964e-03) * (q[i20]) + (4.141272e-03) * (q[i21]) + (3.035838e-04) * (q[i22])
            + (6.291848e-04) * (q[i0] * q[i0]) + (3.195453e-05) * (q[i1] * q[i1]) + (-1.460510e-03) * (q[i2] * q[i2]) + (-6.010023e-03) * (q[i3] * q[i3])
            + (2.749408e-03) * (q[i4] * q[i4]) + (9.894973e-04) * (q[i5] * q[i5]) + (2.561257e-03) * (q[i6] * q[i6]) + (1.562721e-03) * (q[i7] * q[i7])
            + (-1.147863e-03) * (q[i8] * q[i8]) + (-1.776470e-04) * (q[i9] * q[i9]) + (8.040850e-04) * (q[i10] * q[i10]) + (1.444886e-03) * (q[i11] * q[i11])
            + (5.440826e-04) * (q[i12] * q[i12]) + (-1.610159e-03) * ((2) * q[i0] * q[i15]) + (-2.590570e-03) * ((2) * q[i1] * q[i15])
            + (-4.875913e-03) * ((2) * q[i2] * q[i15]) + (-1.798421e-03) * ((2) * q[i3] * q[i15]) + (1.108226e-03) * ((2) * q[i4] * q[i15])
            + (-1.275408e-03) * ((2) * q[i5] * q[i15]) + (1.350029e-03) * ((2) * q[i6] * q[i15]) + (-1.718359e-03) * ((2) * q[i7] * q[i15])
            + (-1.623978e-03) * ((2) * q[i8] * q[i15]) + (-9.476405e-05) * ((2) * q[i9] * q[i15]) + (4.446822e-04) * ((2) * q[i10] * q[i15])
            + (-5.868728e-03) * ((2) * q[i11] * q[i15]) + (4.680595e-04) * ((2) * q[i12] * q[i15]) + (-2.844297e-04) * ((3) * q[i15] * q[i15])
            + (-1.891718e-04) * ((2) * q[i15] * q[i16]) + (8.950920e-04) * ((2) * q[i15] * q[i19]) + (-3.238596e-05) * ((2) * q[i15] * q[i20])
            + (1.447443e-03) * ((2) * q[i15] * q[i21]) + (-2.898229e-04) * ((2) * q[i15] * q[i22]) + (1.895777e-04) * (q[i16] * q[i16])
            + (3.317757e-04) * (q[i19] * q[i19]) + (1.876722e-04) * (q[i20] * q[i20]) + (2.459007e-04) * (q[i21] * q[i21]) + (1.223636e-04) * (q[i22] * q[i22])
            + (8.307751e-05) * (q[i0] * q[i1]) + (-8.816348e-04) * (q[i0] * q[i2]) + (3.737213e-03) * (q[i0] * q[i3]) + (-3.231341e-03) * (q[i0] * q[i4])
            + (3.452045e-03) * (q[i0] * q[i5]) + (-3.302023e-04) * (q[i0] * q[i6]) + (1.730123e-03) * (q[i0] * q[i7]) + (-1.777304e-03) * (q[i0] * q[i8])
            + (1.080065e-04) * (q[i0] * q[i9]) + (-1.365183e-03) * (q[i0] * q[i10]) + (-1.382556e-03) * (q[i0] * q[i11]) + (1.106490e-03) * (q[i0] * q[i12])
            + (8.868801e-05) * (q[i0] * q[i16]) + (1.068404e-03) * (q[i0] * q[i19]) + (-6.770629e-05) * (q[i0] * q[i20]) + (-2.242571e-03) * (q[i0] * q[i21])
            + (2.860531e-04) * (q[i0] * q[i22]) + (1.617850e-03) * (q[i1] * q[i2]) + (1.375503e-04) * (q[i1] * q[i3]) + (4.452672e-03) * (q[i1] * q[i4])
            + (2.119109e-03) * (q[i1] * q[i5]) + (-2.981604e-03) * (q[i1] * q[i6]) + (2.516036e-03) * (q[i1] * q[i7]) + (-1.220725e-03) * (q[i1] * q[i8])
            + (-2.835336e-04) * (q[i1] * q[i9]) + (-8.608620e-04) * (q[i1] * q[i10]) + (3.872995e-03) * (q[i1] * q[i11]) + (2.860590e-04) * (q[i1] * q[i12])
            + (8.015678e-05) * (q[i1] * q[i16]) + (-9.882359e-04) * (q[i1] * q[i19]) + (1.207770e-03) * (q[i1] * q[i20]) + (-2.136392e-04) * (q[i1] * q[i21])
            + (-6.772398e-05) * (q[i1] * q[i22]) + (7.214720e-06) * (q[i2] * q[i3]) + (-6.401135e-04) * (q[i2] * q[i4]) + (7.764108e-03) * (q[i2] * q[i5])
            + (-2.437519e-03) * (q[i2] * q[i6]) + (1.418427e-03) * (q[i2] * q[i7]) + (-6.401808e-04) * (q[i2] * q[i8]) + (1.132058e-03) * (q[i2] * q[i9])
            + (-1.879207e-03) * (q[i2] * q[i10]) + (2.351462e-03) * (q[i2] * q[i11]) + (1.115856e-03) * (q[i2] * q[i12]) + (5.453522e-04) * (q[i2] * q[i16])
            + (5.335550e-04) * (q[i2] * q[i19]) + (9.489542e-04) * (q[i2] * q[i20]) + (-1.693832e-03) * (q[i2] * q[i21]) + (-3.798725e-04) * (q[i2] * q[i22])
            + (1.928381e-03) * (q[i3] * q[i4]) + (-8.369577e-04) * (q[i3] * q[i5]) + (4.697587e-03) * (q[i3] * q[i6]) + (3.929025e-03) * (q[i3] * q[i7])
            + (-8.223876e-04) * (q[i3] * q[i8]) + (-2.520779e-04) * (q[i3] * q[i9]) + (-4.046427e-05) * (q[i3] * q[i10]) + (-1.423635e-03) * (q[i3] * q[i11])
            + (-4.134908e-04) * (q[i3] * q[i12]) + (2.937128e-04) * (q[i3] * q[i16]) + (-1.359044e-03) * (q[i3] * q[i19]) + (1.892597e-03) * (q[i3] * q[i20])
            + (2.977472e-04) * (q[i3] * q[i21]) + (1.001613e-03) * (q[i3] * q[i22]) + (-9.759214e-04) * (q[i4] * q[i5]) + (-4.621407e-03) * (q[i4] * q[i6])
            + (1.152578e-03) * (q[i4] * q[i7]) + (1.167909e-05) * (q[i4] * q[i8]) + (-2.307338e-03) * (q[i4] * q[i9]) + (1.070567e-03) * (q[i4] * q[i10])
            + (4.705882e-04) * (q[i4] * q[i11]) + (-4.336719e-04) * (q[i4] * q[i12]) + (3.093467e-04) * (q[i4] * q[i16]) + (3.023984e-03) * (q[i4] * q[i19])
            + (-2.037393e-03) * (q[i4] * q[i20]) + (-1.102679e-03) * (q[i4] * q[i21]) + (-9.009462e-04) * (q[i4] * q[i22]) + (-3.531705e-03) * (q[i5] * q[i6])
            + (-2.396054e-03) * (q[i5] * q[i7]) + (-3.516862e-03) * (q[i5] * q[i8]) + (6.361504e-04) * (q[i5] * q[i9]) + (-7.985036e-04) * (q[i5] * q[i10])
            + (-3.651541e-03) * (q[i5] * q[i11]) + (-1.202181e-03) * (q[i5] * q[i12]) + (1.537212e-03) * (q[i5] * q[i16]) + (7.910750e-04) * (q[i5] * q[i19])
            + (3.173976e-05) * (q[i5] * q[i20]) + (-1.798743e-04) * (q[i5] * q[i21]) + (1.713108e-04) * (q[i5] * q[i22]) + (-4.667196e-03) * (q[i6] * q[i7])
            + (-1.649729e-03) * (q[i6] * q[i8]) + (-4.085242e-04) * (q[i6] * q[i9]) + (-3.119390e-03) * (q[i6] * q[i10]) + (-2.988266e-05) * (q[i6] * q[i11])
            + (-3.450668e-03) * (q[i6] * q[i12]) + (-3.686725e-04) * (q[i6] * q[i16]) + (-1.345635e-03) * (q[i6] * q[i19]) + (-1.620104e-03) * (q[i6] * q[i20])
            + (-1.247835e-03) * (q[i6] * q[i21]) + (9.407677e-04) * (q[i6] * q[i22]) + (2.943766e-03) * (q[i7] * q[i8]) + (-8.232981e-04) * (q[i7] * q[i9])
            + (1.665228e-03) * (q[i7] * q[i10]) + (9.438830e-04) * (q[i7] * q[i11]) + (2.947093e-03) * (q[i7] * q[i12]) + (3.657008e-04) * (q[i7] * q[i16])
            + (-5.565482e-04) * (q[i7] * q[i19]) + (3.309427e-03) * (q[i7] * q[i20]) + (-8.648373e-06) * (q[i7] * q[i21]) + (5.829073e-04) * (q[i7] * q[i22])
            + (-2.079523e-03) * (q[i8] * q[i9]) + (2.132471e-03) * (q[i8] * q[i10]) + (2.826673e-03) * (q[i8] * q[i11]) + (1.212328e-03) * (q[i8] * q[i12])
            + (7.098969e-07) * (q[i8] * q[i16]) + (1.069179e-03) * (q[i8] * q[i19]) + (-7.574531e-04) * (q[i8] * q[i20]) + (2.629756e-03) * (q[i8] * q[i21])
            + (-3.028679e-04) * (q[i8] * q[i22]) + (-7.261380e-04) * (q[i9] * q[i10]) + (4.185205e-04) * (q[i9] * q[i11]) + (-5.104271e-04) * (q[i9] * q[i12])
            + (-6.006669e-04) * (q[i9] * q[i16]) + (-1.487320e-04) * (q[i9] * q[i19]) + (-2.653334e-05) * (q[i9] * q[i20]) + (-6.297149e-04) * (q[i9] * q[i21])
            + (-3.685585e-05) * (q[i9] * q[i22]) + (8.191657e-04) * (q[i10] * q[i11]) + (6.865590e-05) * (q[i10] * q[i12]) + (6.058846e-04) * (q[i10] * q[i16])
            + (-4.135366e-04) * (q[i10] * q[i19]) + (9.230676e-04) * (q[i10] * q[i20]) + (2.476633e-05) * (q[i10] * q[i21]) + (2.225760e-05) * (q[i10] * q[i22])
            + (-3.824057e-04) * (q[i11] * q[i12]) + (7.387768e-04) * (q[i11] * q[i16]) + (8.488030e-04) * (q[i11] * q[i19])
            + (-1.645095e-04) * (q[i11] * q[i20]) + (1.516783e-03) * (q[i11] * q[i21]) + (-1.931456e-04) * (q[i11] * q[i22])
            + (7.343239e-04) * (q[i12] * q[i16]) + (-9.047756e-04) * (q[i12] * q[i19]) + (5.885397e-04) * (q[i12] * q[i20]) + (9.203270e-05) * (q[i12] * q[i21])
            + (-1.425435e-04) * (q[i12] * q[i22]) + (-7.513823e-05) * (q[i16] * q[i19]) + (7.803911e-05) * (q[i16] * q[i20])
            + (-2.143389e-04) * (q[i16] * q[i21]) + (-2.106631e-04) * (q[i16] * q[i22]) + (-5.097665e-04) * (q[i19] * q[i20])
            + (1.186223e-04) * (q[i19] * q[i21]) + (-1.350526e-06) * (q[i19] * q[i22]) + (-2.515901e-04) * (q[i20] * q[i21])
            + (6.296997e-05) * (q[i20] * q[i22]) + (1.336803e-04) * (q[i21] * q[i22]);
   }

   public void getJQz16(double[] q, double[][] JQ)
   {
      JQ[3][i16] = (-1.233942e-02) * (1) + (-4.954336e-03) * ((2) * q[i16]) + (-7.002062e-03) * (q[i0]) + (-8.223930e-03) * (q[i1]) + (-1.816952e-02) * (q[i2])
            + (-2.114044e-04) * (q[i3]) + (7.795655e-03) * (q[i4]) + (-1.234830e-03) * (q[i5]) + (1.119554e-02) * (q[i6]) + (-2.191571e-03) * (q[i7])
            + (-1.453026e-02) * (q[i8]) + (1.961958e-03) * (q[i9]) + (2.036227e-03) * (q[i10]) + (2.571983e-03) * (q[i11]) + (2.497307e-03) * (q[i12])
            + (6.650289e-06) * (q[i15]) + (1.448494e-03) * (q[i19]) + (-4.613598e-03) * (q[i20]) + (3.120678e-04) * (q[i21]) + (4.203977e-03) * (q[i22])
            + (-1.373595e-06) * (q[i0] * q[i0]) + (-6.272335e-04) * (q[i1] * q[i1]) + (1.450669e-03) * (q[i2] * q[i2]) + (-2.747068e-03) * (q[i3] * q[i3])
            + (6.002643e-03) * (q[i4] * q[i4]) + (-9.961190e-04) * (q[i5] * q[i5]) + (-1.554538e-03) * (q[i6] * q[i6]) + (-2.543950e-03) * (q[i7] * q[i7])
            + (1.224891e-03) * (q[i8] * q[i8]) + (-8.181243e-04) * (q[i9] * q[i9]) + (1.798451e-04) * (q[i10] * q[i10]) + (-5.429432e-04) * (q[i11] * q[i11])
            + (-1.440262e-03) * (q[i12] * q[i12]) + (-1.891718e-04) * (q[i15] * q[i15]) + (-2.629202e-03) * ((2) * q[i0] * q[i16])
            + (-1.635661e-03) * ((2) * q[i1] * q[i16]) + (-4.926560e-03) * ((2) * q[i2] * q[i16]) + (1.128739e-03) * ((2) * q[i3] * q[i16])
            + (-1.787982e-03) * ((2) * q[i4] * q[i16]) + (-1.308459e-03) * ((2) * q[i5] * q[i16]) + (1.732938e-03) * ((2) * q[i6] * q[i16])
            + (-1.365205e-03) * ((2) * q[i7] * q[i16]) + (1.643932e-03) * ((2) * q[i8] * q[i16]) + (-4.486444e-04) * ((2) * q[i9] * q[i16])
            + (9.360250e-05) * ((2) * q[i10] * q[i16]) + (4.659210e-04) * ((2) * q[i11] * q[i16]) + (-5.938069e-03) * ((2) * q[i12] * q[i16])
            + (1.895777e-04) * ((2) * q[i15] * q[i16]) + (2.887173e-04) * ((3) * q[i16] * q[i16]) + (3.629187e-05) * ((2) * q[i16] * q[i19])
            + (-9.031755e-04) * ((2) * q[i16] * q[i20]) + (-2.888105e-04) * ((2) * q[i16] * q[i21]) + (1.464243e-03) * ((2) * q[i16] * q[i22])
            + (-1.910473e-04) * (q[i19] * q[i19]) + (-3.153356e-04) * (q[i20] * q[i20]) + (-1.195869e-04) * (q[i21] * q[i21])
            + (-2.405727e-04) * (q[i22] * q[i22]) + (-8.990329e-05) * (q[i0] * q[i1]) + (-1.612249e-03) * (q[i0] * q[i2]) + (-4.549942e-03) * (q[i0] * q[i3])
            + (-1.271616e-04) * (q[i0] * q[i4]) + (-2.146369e-03) * (q[i0] * q[i5]) + (2.531603e-03) * (q[i0] * q[i6]) + (-2.987341e-03) * (q[i0] * q[i7])
            + (-1.266048e-03) * (q[i0] * q[i8]) + (-8.717527e-04) * (q[i0] * q[i9]) + (-2.779088e-04) * (q[i0] * q[i10]) + (-2.685371e-04) * (q[i0] * q[i11])
            + (-3.871843e-03) * (q[i0] * q[i12]) + (8.868801e-05) * (q[i0] * q[i15]) + (1.225508e-03) * (q[i0] * q[i19]) + (-9.726985e-04) * (q[i0] * q[i20])
            + (8.158969e-05) * (q[i0] * q[i21]) + (2.233752e-04) * (q[i0] * q[i22]) + (8.778651e-04) * (q[i1] * q[i2]) + (3.280968e-03) * (q[i1] * q[i3])
            + (-3.699263e-03) * (q[i1] * q[i4]) + (-3.452865e-03) * (q[i1] * q[i5]) + (1.727913e-03) * (q[i1] * q[i6]) + (-2.919010e-04) * (q[i1] * q[i7])
            + (-1.803216e-03) * (q[i1] * q[i8]) + (-1.345912e-03) * (q[i1] * q[i9]) + (1.288937e-04) * (q[i1] * q[i10]) + (-1.095867e-03) * (q[i1] * q[i11])
            + (1.377429e-03) * (q[i1] * q[i12]) + (8.015678e-05) * (q[i1] * q[i15]) + (-6.929032e-05) * (q[i1] * q[i19]) + (1.071461e-03) * (q[i1] * q[i20])
            + (-3.002668e-04) * (q[i1] * q[i21]) + (2.228063e-03) * (q[i1] * q[i22]) + (6.397862e-04) * (q[i2] * q[i3]) + (-3.348564e-05) * (q[i2] * q[i4])
            + (-7.806409e-03) * (q[i2] * q[i5]) + (1.432669e-03) * (q[i2] * q[i6]) + (-2.459923e-03) * (q[i2] * q[i7]) + (-6.618732e-04) * (q[i2] * q[i8])
            + (-1.885767e-03) * (q[i2] * q[i9]) + (1.131607e-03) * (q[i2] * q[i10]) + (-1.103160e-03) * (q[i2] * q[i11]) + (-2.369277e-03) * (q[i2] * q[i12])
            + (5.453522e-04) * (q[i2] * q[i15]) + (9.717815e-04) * (q[i2] * q[i19]) + (5.450892e-04) * (q[i2] * q[i20]) + (3.833704e-04) * (q[i2] * q[i21])
            + (1.684649e-03) * (q[i2] * q[i22]) + (-1.965659e-03) * (q[i3] * q[i4]) + (1.036362e-03) * (q[i3] * q[i5]) + (1.186916e-03) * (q[i3] * q[i6])
            + (-4.584537e-03) * (q[i3] * q[i7]) + (3.525317e-06) * (q[i3] * q[i8]) + (1.079600e-03) * (q[i3] * q[i9]) + (-2.314218e-03) * (q[i3] * q[i10])
            + (4.422790e-04) * (q[i3] * q[i11]) + (-4.669852e-04) * (q[i3] * q[i12]) + (2.937128e-04) * (q[i3] * q[i15]) + (-2.043554e-03) * (q[i3] * q[i19])
            + (2.998594e-03) * (q[i3] * q[i20]) + (9.347620e-04) * (q[i3] * q[i21]) + (1.107827e-03) * (q[i3] * q[i22]) + (8.420715e-04) * (q[i4] * q[i5])
            + (3.924413e-03) * (q[i4] * q[i6]) + (4.677970e-03) * (q[i4] * q[i7]) + (-8.594415e-04) * (q[i4] * q[i8]) + (-3.622675e-05) * (q[i4] * q[i9])
            + (-2.303241e-04) * (q[i4] * q[i10]) + (4.229579e-04) * (q[i4] * q[i11]) + (1.438830e-03) * (q[i4] * q[i12]) + (3.093467e-04) * (q[i4] * q[i15])
            + (1.869637e-03) * (q[i4] * q[i19]) + (-1.350580e-03) * (q[i4] * q[i20]) + (-1.002016e-03) * (q[i4] * q[i21]) + (-2.786692e-04) * (q[i4] * q[i22])
            + (-2.354698e-03) * (q[i5] * q[i6]) + (-3.510798e-03) * (q[i5] * q[i7]) + (-3.532130e-03) * (q[i5] * q[i8]) + (-8.130769e-04) * (q[i5] * q[i9])
            + (6.463539e-04) * (q[i5] * q[i10]) + (1.199525e-03) * (q[i5] * q[i11]) + (3.666847e-03) * (q[i5] * q[i12]) + (1.537212e-03) * (q[i5] * q[i15])
            + (5.531803e-05) * (q[i5] * q[i19]) + (8.106571e-04) * (q[i5] * q[i20]) + (-1.722371e-04) * (q[i5] * q[i21]) + (1.665068e-04) * (q[i5] * q[i22])
            + (4.670192e-03) * (q[i6] * q[i7]) + (-2.953595e-03) * (q[i6] * q[i8]) + (-1.681619e-03) * (q[i6] * q[i9]) + (8.119270e-04) * (q[i6] * q[i10])
            + (2.932153e-03) * (q[i6] * q[i11]) + (9.277737e-04) * (q[i6] * q[i12]) + (-3.686725e-04) * (q[i6] * q[i15]) + (-3.332313e-03) * (q[i6] * q[i19])
            + (5.609957e-04) * (q[i6] * q[i20]) + (5.767641e-04) * (q[i6] * q[i21]) + (-1.211639e-05) * (q[i6] * q[i22]) + (1.613064e-03) * (q[i7] * q[i8])
            + (3.118214e-03) * (q[i7] * q[i9]) + (4.276289e-04) * (q[i7] * q[i10]) + (-3.429429e-03) * (q[i7] * q[i11]) + (-3.053283e-05) * (q[i7] * q[i12])
            + (3.657008e-04) * (q[i7] * q[i15]) + (1.611454e-03) * (q[i7] * q[i19]) + (1.343040e-03) * (q[i7] * q[i20]) + (9.408520e-04) * (q[i7] * q[i21])
            + (-1.249415e-03) * (q[i7] * q[i22]) + (-2.149540e-03) * (q[i8] * q[i9]) + (2.067358e-03) * (q[i8] * q[i10]) + (1.193063e-03) * (q[i8] * q[i11])
            + (2.828037e-03) * (q[i8] * q[i12]) + (7.098969e-07) * (q[i8] * q[i15]) + (7.779851e-04) * (q[i8] * q[i19]) + (-1.079222e-03) * (q[i8] * q[i20])
            + (-2.999376e-04) * (q[i8] * q[i21]) + (2.667583e-03) * (q[i8] * q[i22]) + (7.210693e-04) * (q[i9] * q[i10]) + (6.748590e-05) * (q[i9] * q[i11])
            + (8.176412e-04) * (q[i9] * q[i12]) + (-6.006669e-04) * (q[i9] * q[i15]) + (-9.418895e-04) * (q[i9] * q[i19]) + (4.169658e-04) * (q[i9] * q[i20])
            + (1.721087e-05) * (q[i9] * q[i21]) + (2.033018e-05) * (q[i9] * q[i22]) + (-4.993969e-04) * (q[i10] * q[i11]) + (4.180841e-04) * (q[i10] * q[i12])
            + (6.058846e-04) * (q[i10] * q[i15]) + (2.947857e-05) * (q[i10] * q[i19]) + (1.432046e-04) * (q[i10] * q[i20]) + (-4.318072e-05) * (q[i10] * q[i21])
            + (-6.301378e-04) * (q[i10] * q[i22]) + (3.722676e-04) * (q[i11] * q[i12]) + (7.387768e-04) * (q[i11] * q[i15]) + (5.898340e-04) * (q[i11] * q[i19])
            + (-9.083006e-04) * (q[i11] * q[i20]) + (1.304451e-04) * (q[i11] * q[i21]) + (-9.794195e-05) * (q[i11] * q[i22])
            + (7.343239e-04) * (q[i12] * q[i15]) + (-1.627775e-04) * (q[i12] * q[i19]) + (8.694204e-04) * (q[i12] * q[i20]) + (1.930178e-04) * (q[i12] * q[i21])
            + (-1.559937e-03) * (q[i12] * q[i22]) + (-7.513823e-05) * (q[i15] * q[i19]) + (7.803911e-05) * (q[i15] * q[i20])
            + (-2.143389e-04) * (q[i15] * q[i21]) + (-2.106631e-04) * (q[i15] * q[i22]) + (5.075237e-04) * (q[i19] * q[i20])
            + (6.470507e-05) * (q[i19] * q[i21]) + (-2.544436e-04) * (q[i19] * q[i22]) + (5.950866e-06) * (q[i20] * q[i21]) + (1.024102e-04) * (q[i20] * q[i22])
            + (-1.343905e-04) * (q[i21] * q[i22]);
   }

   public void getJQz19(double[] q, double[][] JQ)
   {
      JQ[3][i19] = (9.749392e-03) * (1) + (-1.456905e-03) * ((2) * q[i19]) + (-3.021311e-03) * (q[i0]) + (-1.285072e-05) * (q[i1]) + (2.002263e-03) * (q[i2])
            + (-1.242713e-02) * (q[i3]) + (1.143341e-02) * (q[i4]) + (5.678138e-03) * (q[i5]) + (-8.683908e-03) * (q[i6]) + (5.833061e-03) * (q[i7])
            + (4.252995e-03) * (q[i8]) + (-4.272644e-03) * (q[i9]) + (1.303528e-03) * (q[i10]) + (4.182586e-03) * (q[i11]) + (3.930770e-04) * (q[i12])
            + (4.703209e-03) * (q[i15]) + (1.448494e-03) * (q[i16]) + (7.985600e-06) * (q[i20]) + (-5.571742e-03) * (q[i21]) + (-7.097554e-04) * (q[i22])
            + (-6.879212e-05) * (q[i0] * q[i0]) + (-3.195466e-04) * (q[i1] * q[i1]) + (-1.100006e-03) * (q[i2] * q[i2]) + (8.414819e-04) * (q[i3] * q[i3])
            + (2.218625e-04) * (q[i4] * q[i4]) + (2.131419e-03) * (q[i5] * q[i5]) + (2.031403e-04) * (q[i6] * q[i6]) + (8.816005e-04) * (q[i7] * q[i7])
            + (-1.183642e-03) * (q[i8] * q[i8]) + (5.688618e-04) * (q[i9] * q[i9]) + (-5.029924e-04) * (q[i10] * q[i10]) + (-1.210977e-03) * (q[i11] * q[i11])
            + (4.530601e-06) * (q[i12] * q[i12]) + (8.950920e-04) * (q[i15] * q[i15]) + (3.629187e-05) * (q[i16] * q[i16])
            + (9.614484e-04) * ((2) * q[i0] * q[i19]) + (-8.908750e-04) * ((2) * q[i1] * q[i19]) + (-3.407559e-04) * ((2) * q[i2] * q[i19])
            + (-1.618046e-03) * ((2) * q[i3] * q[i19]) + (9.882835e-04) * ((2) * q[i4] * q[i19]) + (5.672238e-04) * ((2) * q[i5] * q[i19])
            + (1.277015e-03) * ((2) * q[i6] * q[i19]) + (-1.300125e-03) * ((2) * q[i7] * q[i19]) + (-2.297524e-04) * ((2) * q[i8] * q[i19])
            + (3.851531e-04) * ((2) * q[i9] * q[i19]) + (2.144442e-04) * ((2) * q[i10] * q[i19]) + (6.159593e-04) * ((2) * q[i11] * q[i19])
            + (-1.264439e-04) * ((2) * q[i12] * q[i19]) + (3.317757e-04) * ((2) * q[i15] * q[i19]) + (-1.910473e-04) * ((2) * q[i16] * q[i19])
            + (-1.966820e-04) * ((3) * q[i19] * q[i19]) + (-3.966958e-04) * ((2) * q[i19] * q[i20]) + (-2.099350e-03) * ((2) * q[i19] * q[i21])
            + (-1.815490e-05) * ((2) * q[i19] * q[i22]) + (3.836269e-04) * (q[i20] * q[i20]) + (-1.030272e-03) * (q[i21] * q[i21])
            + (-1.440290e-04) * (q[i22] * q[i22]) + (2.970379e-04) * (q[i0] * q[i1]) + (-1.778900e-03) * (q[i0] * q[i2]) + (-7.239892e-04) * (q[i0] * q[i3])
            + (-1.119306e-05) * (q[i0] * q[i4]) + (-1.380481e-03) * (q[i0] * q[i5]) + (-1.968210e-03) * (q[i0] * q[i6]) + (-2.717691e-03) * (q[i0] * q[i7])
            + (-9.278645e-04) * (q[i0] * q[i8]) + (1.619438e-03) * (q[i0] * q[i9]) + (1.148780e-03) * (q[i0] * q[i10]) + (1.617007e-03) * (q[i0] * q[i11])
            + (6.751019e-04) * (q[i0] * q[i12]) + (1.068404e-03) * (q[i0] * q[i15]) + (1.225508e-03) * (q[i0] * q[i16]) + (-1.029540e-04) * (q[i0] * q[i20])
            + (7.949083e-04) * (q[i0] * q[i21]) + (-6.577718e-04) * (q[i0] * q[i22]) + (-3.047581e-04) * (q[i1] * q[i2]) + (5.168160e-04) * (q[i1] * q[i3])
            + (4.557667e-04) * (q[i1] * q[i4]) + (5.274652e-04) * (q[i1] * q[i5]) + (-1.552690e-04) * (q[i1] * q[i6]) + (-7.084220e-04) * (q[i1] * q[i7])
            + (2.591051e-03) * (q[i1] * q[i8]) + (-6.331721e-04) * (q[i1] * q[i9]) + (-3.646995e-04) * (q[i1] * q[i10]) + (-2.231571e-03) * (q[i1] * q[i11])
            + (1.575255e-03) * (q[i1] * q[i12]) + (-9.882359e-04) * (q[i1] * q[i15]) + (-6.929032e-05) * (q[i1] * q[i16]) + (-1.173045e-04) * (q[i1] * q[i20])
            + (-1.194085e-04) * (q[i1] * q[i21]) + (8.183928e-04) * (q[i1] * q[i22]) + (-9.618666e-04) * (q[i2] * q[i3]) + (2.656238e-03) * (q[i2] * q[i4])
            + (-3.188673e-03) * (q[i2] * q[i5]) + (-9.689233e-04) * (q[i2] * q[i6]) + (1.409915e-03) * (q[i2] * q[i7]) + (-2.143070e-03) * (q[i2] * q[i8])
            + (-1.062082e-04) * (q[i2] * q[i9]) + (3.717942e-04) * (q[i2] * q[i10]) + (-7.304907e-04) * (q[i2] * q[i11]) + (-6.144859e-04) * (q[i2] * q[i12])
            + (5.335550e-04) * (q[i2] * q[i15]) + (9.717815e-04) * (q[i2] * q[i16]) + (-8.298723e-04) * (q[i2] * q[i20]) + (-6.527569e-04) * (q[i2] * q[i21])
            + (3.099660e-04) * (q[i2] * q[i22]) + (-1.061026e-03) * (q[i3] * q[i4]) + (-3.405769e-03) * (q[i3] * q[i5]) + (-1.181307e-03) * (q[i3] * q[i6])
            + (-2.230980e-04) * (q[i3] * q[i7]) + (5.242117e-03) * (q[i3] * q[i8]) + (1.489055e-03) * (q[i3] * q[i9]) + (1.010646e-03) * (q[i3] * q[i10])
            + (-1.174579e-03) * (q[i3] * q[i11]) + (-4.828984e-03) * (q[i3] * q[i12]) + (-1.359044e-03) * (q[i3] * q[i15]) + (-2.043554e-03) * (q[i3] * q[i16])
            + (-2.845084e-04) * (q[i3] * q[i20]) + (2.922209e-04) * (q[i3] * q[i21]) + (-7.987878e-04) * (q[i3] * q[i22]) + (2.421059e-03) * (q[i4] * q[i5])
            + (5.798120e-03) * (q[i4] * q[i6]) + (2.130440e-03) * (q[i4] * q[i7]) + (-2.804724e-03) * (q[i4] * q[i8]) + (-1.238296e-03) * (q[i4] * q[i9])
            + (5.875405e-04) * (q[i4] * q[i10]) + (2.112294e-03) * (q[i4] * q[i11]) + (2.819507e-03) * (q[i4] * q[i12]) + (3.023984e-03) * (q[i4] * q[i15])
            + (1.869637e-03) * (q[i4] * q[i16]) + (-2.449044e-04) * (q[i4] * q[i20]) + (-1.807287e-03) * (q[i4] * q[i21]) + (-8.107735e-04) * (q[i4] * q[i22])
            + (2.702330e-03) * (q[i5] * q[i6]) + (-8.926671e-04) * (q[i5] * q[i7]) + (-1.052659e-03) * (q[i5] * q[i8]) + (3.115368e-04) * (q[i5] * q[i9])
            + (-1.069672e-03) * (q[i5] * q[i10]) + (-7.826358e-04) * (q[i5] * q[i11]) + (-9.107160e-04) * (q[i5] * q[i12]) + (7.910750e-04) * (q[i5] * q[i15])
            + (5.531803e-05) * (q[i5] * q[i16]) + (4.123524e-06) * (q[i5] * q[i20]) + (2.630843e-04) * (q[i5] * q[i21]) + (5.709591e-04) * (q[i5] * q[i22])
            + (1.347722e-03) * (q[i6] * q[i7]) + (9.782341e-04) * (q[i6] * q[i8]) + (1.523656e-03) * (q[i6] * q[i9]) + (1.324738e-03) * (q[i6] * q[i10])
            + (1.671965e-03) * (q[i6] * q[i11]) + (-3.422618e-04) * (q[i6] * q[i12]) + (-1.345635e-03) * (q[i6] * q[i15]) + (-3.332313e-03) * (q[i6] * q[i16])
            + (1.521665e-03) * (q[i6] * q[i20]) + (-1.122185e-03) * (q[i6] * q[i21]) + (1.952267e-04) * (q[i6] * q[i22]) + (-3.737186e-03) * (q[i7] * q[i8])
            + (-7.282846e-04) * (q[i7] * q[i9]) + (-1.287692e-03) * (q[i7] * q[i10]) + (-1.694066e-03) * (q[i7] * q[i11]) + (-7.212440e-04) * (q[i7] * q[i12])
            + (-5.565482e-04) * (q[i7] * q[i15]) + (1.611454e-03) * (q[i7] * q[i16]) + (-1.519264e-03) * (q[i7] * q[i20]) + (-1.940345e-03) * (q[i7] * q[i21])
            + (-4.046829e-05) * (q[i7] * q[i22]) + (8.208506e-04) * (q[i8] * q[i9]) + (-2.938725e-03) * (q[i8] * q[i10]) + (-1.131070e-03) * (q[i8] * q[i11])
            + (4.809914e-04) * (q[i8] * q[i12]) + (1.069179e-03) * (q[i8] * q[i15]) + (7.779851e-04) * (q[i8] * q[i16]) + (5.116259e-06) * (q[i8] * q[i20])
            + (3.348708e-03) * (q[i8] * q[i21]) + (2.725898e-04) * (q[i8] * q[i22]) + (3.068323e-04) * (q[i9] * q[i10]) + (2.570354e-04) * (q[i9] * q[i11])
            + (8.131269e-05) * (q[i9] * q[i12]) + (-1.487320e-04) * (q[i9] * q[i15]) + (-9.418895e-04) * (q[i9] * q[i16]) + (6.983687e-04) * (q[i9] * q[i20])
            + (-4.695622e-04) * (q[i9] * q[i21]) + (2.972330e-04) * (q[i9] * q[i22]) + (-6.430745e-06) * (q[i10] * q[i11]) + (-2.767815e-04) * (q[i10] * q[i12])
            + (-4.135366e-04) * (q[i10] * q[i15]) + (2.947857e-05) * (q[i10] * q[i16]) + (-6.971758e-04) * (q[i10] * q[i20])
            + (-3.065525e-04) * (q[i10] * q[i21]) + (-1.014092e-05) * (q[i10] * q[i22]) + (-5.943896e-04) * (q[i11] * q[i12])
            + (8.488030e-04) * (q[i11] * q[i15]) + (5.898340e-04) * (q[i11] * q[i16]) + (3.248382e-04) * (q[i11] * q[i20]) + (1.886006e-03) * (q[i11] * q[i21])
            + (7.798466e-04) * (q[i11] * q[i22]) + (-9.047756e-04) * (q[i12] * q[i15]) + (-1.627775e-04) * (q[i12] * q[i16])
            + (3.196399e-04) * (q[i12] * q[i20]) + (3.213397e-04) * (q[i12] * q[i21]) + (-3.151055e-04) * (q[i12] * q[i22])
            + (-7.513823e-05) * (q[i15] * q[i16]) + (-5.097665e-04) * (q[i15] * q[i20]) + (1.186223e-04) * (q[i15] * q[i21])
            + (-1.350526e-06) * (q[i15] * q[i22]) + (5.075237e-04) * (q[i16] * q[i20]) + (6.470507e-05) * (q[i16] * q[i21])
            + (-2.544436e-04) * (q[i16] * q[i22]) + (-2.439649e-05) * (q[i20] * q[i21]) + (-2.701687e-05) * (q[i20] * q[i22])
            + (8.061874e-05) * (q[i21] * q[i22]);
   }

   public void getJQz20(double[] q, double[][] JQ)
   {
      JQ[3][i20] = (-9.463679e-03) * (1) + (1.449502e-03) * ((2) * q[i20]) + (-2.484627e-05) * (q[i0]) + (-3.012531e-03) * (q[i1]) + (1.964953e-03) * (q[i2])
            + (1.146731e-02) * (q[i3]) + (-1.241368e-02) * (q[i4]) + (5.708390e-03) * (q[i5]) + (-5.856227e-03) * (q[i6]) + (8.629763e-03) * (q[i7])
            + (-4.199044e-03) * (q[i8]) + (-1.320874e-03) * (q[i9]) + (4.239309e-03) * (q[i10]) + (3.689563e-04) * (q[i11]) + (4.151972e-03) * (q[i12])
            + (-1.427964e-03) * (q[i15]) + (-4.613598e-03) * (q[i16]) + (7.985600e-06) * (q[i19]) + (-6.905349e-04) * (q[i21]) + (-5.626379e-03) * (q[i22])
            + (3.310978e-04) * (q[i0] * q[i0]) + (1.054223e-04) * (q[i1] * q[i1]) + (1.085563e-03) * (q[i2] * q[i2]) + (-2.612225e-04) * (q[i3] * q[i3])
            + (-8.482367e-04) * (q[i4] * q[i4]) + (-2.149900e-03) * (q[i5] * q[i5]) + (-8.974494e-04) * (q[i6] * q[i6]) + (-2.149406e-04) * (q[i7] * q[i7])
            + (1.153215e-03) * (q[i8] * q[i8]) + (5.050609e-04) * (q[i9] * q[i9]) + (-5.685368e-04) * (q[i10] * q[i10]) + (-2.343149e-05) * (q[i11] * q[i11])
            + (1.180369e-03) * (q[i12] * q[i12]) + (-3.238596e-05) * (q[i15] * q[i15]) + (-9.031755e-04) * (q[i16] * q[i16])
            + (-3.966958e-04) * (q[i19] * q[i19]) + (-8.809152e-04) * ((2) * q[i0] * q[i20]) + (9.579766e-04) * ((2) * q[i1] * q[i20])
            + (-3.275157e-04) * ((2) * q[i2] * q[i20]) + (9.940037e-04) * ((2) * q[i3] * q[i20]) + (-1.590706e-03) * ((2) * q[i4] * q[i20])
            + (5.569911e-04) * ((2) * q[i5] * q[i20]) + (1.285111e-03) * ((2) * q[i6] * q[i20]) + (-1.281165e-03) * ((2) * q[i7] * q[i20])
            + (2.460478e-04) * ((2) * q[i8] * q[i20]) + (-2.195414e-04) * ((2) * q[i9] * q[i20]) + (-3.820262e-04) * ((2) * q[i10] * q[i20])
            + (-1.191432e-04) * ((2) * q[i11] * q[i20]) + (6.084688e-04) * ((2) * q[i12] * q[i20]) + (1.876722e-04) * ((2) * q[i15] * q[i20])
            + (-3.153356e-04) * ((2) * q[i16] * q[i20]) + (3.836269e-04) * ((2) * q[i19] * q[i20]) + (1.954735e-04) * ((3) * q[i20] * q[i20])
            + (-1.573420e-05) * ((2) * q[i20] * q[i21]) + (-2.097085e-03) * ((2) * q[i20] * q[i22]) + (1.407896e-04) * (q[i21] * q[i21])
            + (1.047033e-03) * (q[i22] * q[i22]) + (-3.106547e-04) * (q[i0] * q[i1]) + (3.204858e-04) * (q[i0] * q[i2]) + (-4.420090e-04) * (q[i0] * q[i3])
            + (-5.171096e-04) * (q[i0] * q[i4]) + (-5.019998e-04) * (q[i0] * q[i5]) + (-7.278929e-04) * (q[i0] * q[i6]) + (-1.157077e-04) * (q[i0] * q[i7])
            + (2.572781e-03) * (q[i0] * q[i8]) + (-3.476660e-04) * (q[i0] * q[i9]) + (-6.231768e-04) * (q[i0] * q[i10]) + (-1.578691e-03) * (q[i0] * q[i11])
            + (2.247609e-03) * (q[i0] * q[i12]) + (-6.770629e-05) * (q[i0] * q[i15]) + (-9.726985e-04) * (q[i0] * q[i16]) + (-1.029540e-04) * (q[i0] * q[i19])
            + (-8.140101e-04) * (q[i0] * q[i21]) + (1.086136e-04) * (q[i0] * q[i22]) + (1.812038e-03) * (q[i1] * q[i2]) + (-2.998582e-05) * (q[i1] * q[i3])
            + (6.645825e-04) * (q[i1] * q[i4]) + (1.376363e-03) * (q[i1] * q[i5]) + (-2.713295e-03) * (q[i1] * q[i6]) + (-1.945346e-03) * (q[i1] * q[i7])
            + (-9.210163e-04) * (q[i1] * q[i8]) + (1.136149e-03) * (q[i1] * q[i9]) + (1.601052e-03) * (q[i1] * q[i10]) + (-6.580470e-04) * (q[i1] * q[i11])
            + (-1.639693e-03) * (q[i1] * q[i12]) + (1.207770e-03) * (q[i1] * q[i15]) + (1.071461e-03) * (q[i1] * q[i16]) + (-1.173045e-04) * (q[i1] * q[i19])
            + (6.573443e-04) * (q[i1] * q[i21]) + (-7.683510e-04) * (q[i1] * q[i22]) + (-2.681924e-03) * (q[i2] * q[i3]) + (9.512730e-04) * (q[i2] * q[i4])
            + (3.189714e-03) * (q[i2] * q[i5]) + (1.426482e-03) * (q[i2] * q[i6]) + (-9.379687e-04) * (q[i2] * q[i7]) + (-2.164839e-03) * (q[i2] * q[i8])
            + (3.898031e-04) * (q[i2] * q[i9]) + (-1.153407e-04) * (q[i2] * q[i10]) + (6.020709e-04) * (q[i2] * q[i11]) + (7.187429e-04) * (q[i2] * q[i12])
            + (9.489542e-04) * (q[i2] * q[i15]) + (5.450892e-04) * (q[i2] * q[i16]) + (-8.298723e-04) * (q[i2] * q[i19]) + (-3.104171e-04) * (q[i2] * q[i21])
            + (6.695144e-04) * (q[i2] * q[i22]) + (1.098279e-03) * (q[i3] * q[i4]) + (-2.451930e-03) * (q[i3] * q[i5]) + (2.162745e-03) * (q[i3] * q[i6])
            + (5.809542e-03) * (q[i3] * q[i7]) + (-2.827921e-03) * (q[i3] * q[i8]) + (6.169341e-04) * (q[i3] * q[i9]) + (-1.256187e-03) * (q[i3] * q[i10])
            + (-2.797302e-03) * (q[i3] * q[i11]) + (-2.117439e-03) * (q[i3] * q[i12]) + (1.892597e-03) * (q[i3] * q[i15]) + (2.998594e-03) * (q[i3] * q[i16])
            + (-2.845084e-04) * (q[i3] * q[i19]) + (8.213946e-04) * (q[i3] * q[i21]) + (1.826411e-03) * (q[i3] * q[i22]) + (3.417518e-03) * (q[i4] * q[i5])
            + (-2.300427e-04) * (q[i4] * q[i6]) + (-1.210903e-03) * (q[i4] * q[i7]) + (5.203166e-03) * (q[i4] * q[i8]) + (1.006603e-03) * (q[i4] * q[i9])
            + (1.476427e-03) * (q[i4] * q[i10]) + (4.808062e-03) * (q[i4] * q[i11]) + (1.160681e-03) * (q[i4] * q[i12]) + (-2.037393e-03) * (q[i4] * q[i15])
            + (-1.350580e-03) * (q[i4] * q[i16]) + (-2.449044e-04) * (q[i4] * q[i19]) + (7.983853e-04) * (q[i4] * q[i21]) + (-2.967091e-04) * (q[i4] * q[i22])
            + (-9.051232e-04) * (q[i5] * q[i6]) + (2.701717e-03) * (q[i5] * q[i7]) + (-1.009716e-03) * (q[i5] * q[i8]) + (-1.091896e-03) * (q[i5] * q[i9])
            + (3.255600e-04) * (q[i5] * q[i10]) + (8.982612e-04) * (q[i5] * q[i11]) + (7.582725e-04) * (q[i5] * q[i12]) + (3.173976e-05) * (q[i5] * q[i15])
            + (8.106571e-04) * (q[i5] * q[i16]) + (4.123524e-06) * (q[i5] * q[i19]) + (-5.749362e-04) * (q[i5] * q[i21]) + (-2.760606e-04) * (q[i5] * q[i22])
            + (-1.363415e-03) * (q[i6] * q[i7]) + (3.762517e-03) * (q[i6] * q[i8]) + (1.277208e-03) * (q[i6] * q[i9]) + (7.252227e-04) * (q[i6] * q[i10])
            + (-7.149938e-04) * (q[i6] * q[i11]) + (-1.686519e-03) * (q[i6] * q[i12]) + (-1.620104e-03) * (q[i6] * q[i15]) + (5.609957e-04) * (q[i6] * q[i16])
            + (1.521665e-03) * (q[i6] * q[i19]) + (-4.661550e-05) * (q[i6] * q[i21]) + (-1.946770e-03) * (q[i6] * q[i22]) + (-9.455939e-04) * (q[i7] * q[i8])
            + (-1.328406e-03) * (q[i7] * q[i9]) + (-1.523878e-03) * (q[i7] * q[i10]) + (-3.149649e-04) * (q[i7] * q[i11]) + (1.663596e-03) * (q[i7] * q[i12])
            + (3.309427e-03) * (q[i7] * q[i15]) + (1.343040e-03) * (q[i7] * q[i16]) + (-1.519264e-03) * (q[i7] * q[i19]) + (2.033601e-04) * (q[i7] * q[i21])
            + (-1.112846e-03) * (q[i7] * q[i22]) + (2.952076e-03) * (q[i8] * q[i9]) + (-8.147241e-04) * (q[i8] * q[i10]) + (4.758059e-04) * (q[i8] * q[i11])
            + (-1.072583e-03) * (q[i8] * q[i12]) + (-7.574531e-04) * (q[i8] * q[i15]) + (-1.079222e-03) * (q[i8] * q[i16]) + (5.116259e-06) * (q[i8] * q[i19])
            + (2.678443e-04) * (q[i8] * q[i21]) + (3.337514e-03) * (q[i8] * q[i22]) + (-3.114489e-04) * (q[i9] * q[i10]) + (-2.698155e-04) * (q[i9] * q[i11])
            + (-9.948959e-06) * (q[i9] * q[i12]) + (-2.653334e-05) * (q[i9] * q[i15]) + (4.169658e-04) * (q[i9] * q[i16]) + (6.983687e-04) * (q[i9] * q[i19])
            + (-1.456776e-05) * (q[i9] * q[i21]) + (-3.028418e-04) * (q[i9] * q[i22]) + (9.173832e-05) * (q[i10] * q[i11]) + (2.536174e-04) * (q[i10] * q[i12])
            + (9.230676e-04) * (q[i10] * q[i15]) + (1.432046e-04) * (q[i10] * q[i16]) + (-6.971758e-04) * (q[i10] * q[i19]) + (2.987073e-04) * (q[i10] * q[i21])
            + (-4.575649e-04) * (q[i10] * q[i22]) + (6.070909e-04) * (q[i11] * q[i12]) + (-1.645095e-04) * (q[i11] * q[i15])
            + (-9.083006e-04) * (q[i11] * q[i16]) + (3.248382e-04) * (q[i11] * q[i19]) + (3.111280e-04) * (q[i11] * q[i21])
            + (-3.240401e-04) * (q[i11] * q[i22]) + (5.885397e-04) * (q[i12] * q[i15]) + (8.694204e-04) * (q[i12] * q[i16]) + (3.196399e-04) * (q[i12] * q[i19])
            + (-7.816191e-04) * (q[i12] * q[i21]) + (-1.866132e-03) * (q[i12] * q[i22]) + (7.803911e-05) * (q[i15] * q[i16])
            + (-5.097665e-04) * (q[i15] * q[i19]) + (-2.515901e-04) * (q[i15] * q[i21]) + (6.296997e-05) * (q[i15] * q[i22])
            + (5.075237e-04) * (q[i16] * q[i19]) + (5.950866e-06) * (q[i16] * q[i21]) + (1.024102e-04) * (q[i16] * q[i22]) + (-2.439649e-05) * (q[i19] * q[i21])
            + (-2.701687e-05) * (q[i19] * q[i22]) + (-8.116803e-05) * (q[i21] * q[i22]);
   }

   public void getJQz21(double[] q, double[][] JQ)
   {
      JQ[3][i21] = (8.252879e-03) * (1) + (5.218888e-05) * ((2) * q[i21]) + (-6.986138e-03) * (q[i0]) + (-1.117623e-03) * (q[i1]) + (-3.676468e-03) * (q[i2])
            + (1.072986e-02) * (q[i3]) + (-3.541732e-03) * (q[i4]) + (-3.162355e-03) * (q[i5]) + (-1.736492e-03) * (q[i6]) + (2.335451e-03) * (q[i7])
            + (3.168813e-03) * (q[i8]) + (-4.451931e-04) * (q[i9]) + (2.496257e-03) * (q[i10]) + (4.161639e-03) * (q[i11]) + (-4.494994e-04) * (q[i12])
            + (4.141272e-03) * (q[i15]) + (3.120678e-04) * (q[i16]) + (-5.571742e-03) * (q[i19]) + (-6.905349e-04) * (q[i20]) + (-3.384314e-06) * (q[i22])
            + (-1.531392e-04) * (q[i0] * q[i0]) + (4.403735e-04) * (q[i1] * q[i1]) + (5.330095e-04) * (q[i2] * q[i2]) + (-1.876124e-03) * (q[i3] * q[i3])
            + (1.014072e-03) * (q[i4] * q[i4]) + (4.273690e-04) * (q[i5] * q[i5]) + (-1.225484e-03) * (q[i6] * q[i6]) + (-1.779634e-03) * (q[i7] * q[i7])
            + (1.220470e-03) * (q[i8] * q[i8]) + (1.956634e-04) * (q[i9] * q[i9]) + (-7.848734e-04) * (q[i10] * q[i10]) + (-1.088728e-03) * (q[i11] * q[i11])
            + (2.075158e-04) * (q[i12] * q[i12]) + (1.447443e-03) * (q[i15] * q[i15]) + (-2.888105e-04) * (q[i16] * q[i16])
            + (-2.099350e-03) * (q[i19] * q[i19]) + (-1.573420e-05) * (q[i20] * q[i20]) + (-1.759633e-03) * ((2) * q[i0] * q[i21])
            + (-3.901743e-04) * ((2) * q[i1] * q[i21]) + (-2.045029e-03) * ((2) * q[i2] * q[i21]) + (-7.426308e-04) * ((2) * q[i3] * q[i21])
            + (1.312973e-03) * ((2) * q[i4] * q[i21]) + (-3.217971e-04) * ((2) * q[i5] * q[i21]) + (1.607276e-03) * ((2) * q[i6] * q[i21])
            + (-1.741215e-03) * ((2) * q[i7] * q[i21]) + (4.810772e-04) * ((2) * q[i8] * q[i21]) + (3.766099e-04) * ((2) * q[i9] * q[i21])
            + (6.369769e-05) * ((2) * q[i10] * q[i21]) + (-6.023134e-04) * ((2) * q[i11] * q[i21]) + (-4.094983e-04) * ((2) * q[i12] * q[i21])
            + (2.459007e-04) * ((2) * q[i15] * q[i21]) + (-1.195869e-04) * ((2) * q[i16] * q[i21]) + (-1.030272e-03) * ((2) * q[i19] * q[i21])
            + (1.407896e-04) * ((2) * q[i20] * q[i21]) + (-5.500735e-04) * ((3) * q[i21] * q[i21]) + (4.184537e-05) * ((2) * q[i21] * q[i22])
            + (3.682205e-05) * (q[i22] * q[i22]) + (1.096464e-03) * (q[i0] * q[i1]) + (-7.543829e-05) * (q[i0] * q[i2]) + (2.352586e-03) * (q[i0] * q[i3])
            + (-2.774221e-03) * (q[i0] * q[i4]) + (-2.871135e-03) * (q[i0] * q[i5]) + (-4.359618e-04) * (q[i0] * q[i6]) + (4.351924e-04) * (q[i0] * q[i7])
            + (5.334901e-04) * (q[i0] * q[i8]) + (1.984431e-04) * (q[i0] * q[i9]) + (6.428626e-04) * (q[i0] * q[i10]) + (-8.476260e-04) * (q[i0] * q[i11])
            + (1.695129e-03) * (q[i0] * q[i12]) + (-2.242571e-03) * (q[i0] * q[i15]) + (8.158969e-05) * (q[i0] * q[i16]) + (7.949083e-04) * (q[i0] * q[i19])
            + (-8.140101e-04) * (q[i0] * q[i20]) + (4.544889e-04) * (q[i0] * q[i22]) + (1.553031e-03) * (q[i1] * q[i2]) + (-7.713824e-04) * (q[i1] * q[i3])
            + (-2.249974e-03) * (q[i1] * q[i4]) + (-8.784839e-04) * (q[i1] * q[i5]) + (6.585911e-04) * (q[i1] * q[i6]) + (-1.962824e-03) * (q[i1] * q[i7])
            + (1.685769e-03) * (q[i1] * q[i8]) + (1.872351e-04) * (q[i1] * q[i9]) + (-4.788341e-05) * (q[i1] * q[i10]) + (6.031111e-04) * (q[i1] * q[i11])
            + (-1.023231e-03) * (q[i1] * q[i12]) + (-2.136392e-04) * (q[i1] * q[i15]) + (-3.002668e-04) * (q[i1] * q[i16]) + (-1.194085e-04) * (q[i1] * q[i19])
            + (6.573443e-04) * (q[i1] * q[i20]) + (4.280431e-04) * (q[i1] * q[i22]) + (1.918705e-03) * (q[i2] * q[i3]) + (-3.841436e-03) * (q[i2] * q[i4])
            + (-2.336499e-03) * (q[i2] * q[i5]) + (-3.636610e-04) * (q[i2] * q[i6]) + (-4.657147e-04) * (q[i2] * q[i7]) + (7.322682e-04) * (q[i2] * q[i8])
            + (9.727644e-05) * (q[i2] * q[i9]) + (3.553253e-04) * (q[i2] * q[i10]) + (-8.595518e-04) * (q[i2] * q[i11]) + (-6.115261e-04) * (q[i2] * q[i12])
            + (-1.693832e-03) * (q[i2] * q[i15]) + (3.833704e-04) * (q[i2] * q[i16]) + (-6.527569e-04) * (q[i2] * q[i19]) + (-3.104171e-04) * (q[i2] * q[i20])
            + (1.096783e-03) * (q[i2] * q[i22]) + (-1.279848e-03) * (q[i3] * q[i4]) + (2.088894e-03) * (q[i3] * q[i5]) + (-1.000557e-03) * (q[i3] * q[i6])
            + (1.440824e-03) * (q[i3] * q[i7]) + (1.435959e-03) * (q[i3] * q[i8]) + (-1.580515e-03) * (q[i3] * q[i9]) + (-2.090453e-03) * (q[i3] * q[i10])
            + (-1.517272e-03) * (q[i3] * q[i11]) + (-5.185118e-04) * (q[i3] * q[i12]) + (2.977472e-04) * (q[i3] * q[i15]) + (9.347620e-04) * (q[i3] * q[i16])
            + (2.922209e-04) * (q[i3] * q[i19]) + (8.213946e-04) * (q[i3] * q[i20]) + (7.898597e-05) * (q[i3] * q[i22]) + (-2.063205e-03) * (q[i4] * q[i5])
            + (3.513589e-04) * (q[i4] * q[i6]) + (-3.757145e-03) * (q[i4] * q[i7]) + (-2.477220e-03) * (q[i4] * q[i8]) + (1.596703e-03) * (q[i4] * q[i9])
            + (-4.333095e-04) * (q[i4] * q[i10]) + (1.044082e-04) * (q[i4] * q[i11]) + (-7.572880e-04) * (q[i4] * q[i12]) + (-1.102679e-03) * (q[i4] * q[i15])
            + (-1.002016e-03) * (q[i4] * q[i16]) + (-1.807287e-03) * (q[i4] * q[i19]) + (7.983853e-04) * (q[i4] * q[i20]) + (6.475461e-05) * (q[i4] * q[i22])
            + (1.591960e-03) * (q[i5] * q[i6]) + (1.449493e-03) * (q[i5] * q[i7]) + (1.641170e-03) * (q[i5] * q[i8]) + (7.568795e-04) * (q[i5] * q[i9])
            + (1.911686e-03) * (q[i5] * q[i10]) + (-2.270249e-04) * (q[i5] * q[i11]) + (7.650291e-04) * (q[i5] * q[i12]) + (-1.798743e-04) * (q[i5] * q[i15])
            + (-1.722371e-04) * (q[i5] * q[i16]) + (2.630843e-04) * (q[i5] * q[i19]) + (-5.749362e-04) * (q[i5] * q[i20]) + (-9.237874e-04) * (q[i5] * q[i22])
            + (1.620153e-03) * (q[i6] * q[i7]) + (-3.744837e-04) * (q[i6] * q[i8]) + (1.961747e-04) * (q[i6] * q[i9]) + (1.626460e-03) * (q[i6] * q[i10])
            + (1.916454e-03) * (q[i6] * q[i11]) + (-1.191898e-03) * (q[i6] * q[i12]) + (-1.247835e-03) * (q[i6] * q[i15]) + (5.767641e-04) * (q[i6] * q[i16])
            + (-1.122185e-03) * (q[i6] * q[i19]) + (-4.661550e-05) * (q[i6] * q[i20]) + (-3.455159e-04) * (q[i6] * q[i22]) + (5.372617e-04) * (q[i7] * q[i8])
            + (-5.005814e-04) * (q[i7] * q[i9]) + (-2.451621e-03) * (q[i7] * q[i10]) + (-1.967779e-03) * (q[i7] * q[i11]) + (-1.454366e-03) * (q[i7] * q[i12])
            + (-8.648373e-06) * (q[i7] * q[i15]) + (9.408520e-04) * (q[i7] * q[i16]) + (-1.940345e-03) * (q[i7] * q[i19]) + (2.033601e-04) * (q[i7] * q[i20])
            + (3.555269e-04) * (q[i7] * q[i22]) + (-1.144188e-03) * (q[i8] * q[i9]) + (4.143830e-04) * (q[i8] * q[i10]) + (-3.066098e-04) * (q[i8] * q[i11])
            + (1.149213e-03) * (q[i8] * q[i12]) + (2.629756e-03) * (q[i8] * q[i15]) + (-2.999376e-04) * (q[i8] * q[i16]) + (3.348708e-03) * (q[i8] * q[i19])
            + (2.678443e-04) * (q[i8] * q[i20]) + (7.139614e-06) * (q[i8] * q[i22]) + (2.671950e-04) * (q[i9] * q[i10]) + (5.498505e-04) * (q[i9] * q[i11])
            + (-7.537221e-04) * (q[i9] * q[i12]) + (-6.297149e-04) * (q[i9] * q[i15]) + (1.721087e-05) * (q[i9] * q[i16]) + (-4.695622e-04) * (q[i9] * q[i19])
            + (-1.456776e-05) * (q[i9] * q[i20]) + (-1.130490e-04) * (q[i9] * q[i22]) + (2.451832e-04) * (q[i10] * q[i11]) + (-5.718205e-04) * (q[i10] * q[i12])
            + (2.476633e-05) * (q[i10] * q[i15]) + (-4.318072e-05) * (q[i10] * q[i16]) + (-3.065525e-04) * (q[i10] * q[i19])
            + (2.987073e-04) * (q[i10] * q[i20]) + (1.193717e-04) * (q[i10] * q[i22]) + (1.004132e-04) * (q[i11] * q[i12]) + (1.516783e-03) * (q[i11] * q[i15])
            + (1.304451e-04) * (q[i11] * q[i16]) + (1.886006e-03) * (q[i11] * q[i19]) + (3.111280e-04) * (q[i11] * q[i20]) + (2.629995e-04) * (q[i11] * q[i22])
            + (9.203270e-05) * (q[i12] * q[i15]) + (1.930178e-04) * (q[i12] * q[i16]) + (3.213397e-04) * (q[i12] * q[i19]) + (-7.816191e-04) * (q[i12] * q[i20])
            + (2.602461e-04) * (q[i12] * q[i22]) + (-2.143389e-04) * (q[i15] * q[i16]) + (1.186223e-04) * (q[i15] * q[i19])
            + (-2.515901e-04) * (q[i15] * q[i20]) + (1.336803e-04) * (q[i15] * q[i22]) + (6.470507e-05) * (q[i16] * q[i19]) + (5.950866e-06) * (q[i16] * q[i20])
            + (-1.343905e-04) * (q[i16] * q[i22]) + (-2.439649e-05) * (q[i19] * q[i20]) + (8.061874e-05) * (q[i19] * q[i22])
            + (-8.116803e-05) * (q[i20] * q[i22]);
   }

   public void getJQz22(double[] q, double[][] JQ)
   {
      JQ[3][i22] = (8.274845e-03) * (1) + (-7.309604e-05) * ((2) * q[i22]) + (1.111550e-03) * (q[i0]) + (6.876052e-03) * (q[i1]) + (3.596080e-03) * (q[i2])
            + (3.548932e-03) * (q[i3]) + (-1.061346e-02) * (q[i4]) + (3.131660e-03) * (q[i5]) + (2.301098e-03) * (q[i6]) + (-1.764857e-03) * (q[i7])
            + (3.195922e-03) * (q[i8]) + (2.513326e-03) * (q[i9]) + (-4.460657e-04) * (q[i10]) + (4.512438e-04) * (q[i11]) + (-4.286199e-03) * (q[i12])
            + (3.035838e-04) * (q[i15]) + (4.203977e-03) * (q[i16]) + (-7.097554e-04) * (q[i19]) + (-5.626379e-03) * (q[i20]) + (-3.384314e-06) * (q[i21])
            + (4.324533e-04) * (q[i0] * q[i0]) + (-1.664605e-04) * (q[i1] * q[i1]) + (5.261333e-04) * (q[i2] * q[i2]) + (1.019498e-03) * (q[i3] * q[i3])
            + (-1.839932e-03) * (q[i4] * q[i4]) + (4.067289e-04) * (q[i5] * q[i5]) + (-1.812007e-03) * (q[i6] * q[i6]) + (-1.211107e-03) * (q[i7] * q[i7])
            + (1.243642e-03) * (q[i8] * q[i8]) + (-7.894434e-04) * (q[i9] * q[i9]) + (1.943379e-04) * (q[i10] * q[i10]) + (2.058726e-04) * (q[i11] * q[i11])
            + (-1.071907e-03) * (q[i12] * q[i12]) + (-2.898229e-04) * (q[i15] * q[i15]) + (1.464243e-03) * (q[i16] * q[i16])
            + (-1.815490e-05) * (q[i19] * q[i19]) + (-2.097085e-03) * (q[i20] * q[i20]) + (4.184537e-05) * (q[i21] * q[i21])
            + (-4.010723e-04) * ((2) * q[i0] * q[i22]) + (-1.742359e-03) * ((2) * q[i1] * q[i22]) + (-2.031400e-03) * ((2) * q[i2] * q[i22])
            + (1.324916e-03) * ((2) * q[i3] * q[i22]) + (-7.465614e-04) * ((2) * q[i4] * q[i22]) + (-3.075707e-04) * ((2) * q[i5] * q[i22])
            + (1.739386e-03) * ((2) * q[i6] * q[i22]) + (-1.603073e-03) * ((2) * q[i7] * q[i22]) + (-4.754923e-04) * ((2) * q[i8] * q[i22])
            + (-6.016814e-05) * ((2) * q[i9] * q[i22]) + (-3.702790e-04) * ((2) * q[i10] * q[i22]) + (-4.069585e-04) * ((2) * q[i11] * q[i22])
            + (-5.864783e-04) * ((2) * q[i12] * q[i22]) + (1.223636e-04) * ((2) * q[i15] * q[i22]) + (-2.405727e-04) * ((2) * q[i16] * q[i22])
            + (-1.440290e-04) * ((2) * q[i19] * q[i22]) + (1.047033e-03) * ((2) * q[i20] * q[i22]) + (3.682205e-05) * ((2) * q[i21] * q[i22])
            + (-5.527782e-04) * ((3) * q[i22] * q[i22]) + (1.088216e-03) * (q[i0] * q[i1]) + (1.559269e-03) * (q[i0] * q[i2])
            + (-2.251364e-03) * (q[i0] * q[i3]) + (-7.930453e-04) * (q[i0] * q[i4]) + (-8.908884e-04) * (q[i0] * q[i5]) + (1.950349e-03) * (q[i0] * q[i6])
            + (-6.437164e-04) * (q[i0] * q[i7]) + (-1.699471e-03) * (q[i0] * q[i8]) + (4.594173e-05) * (q[i0] * q[i9]) + (-1.778341e-04) * (q[i0] * q[i10])
            + (-1.043071e-03) * (q[i0] * q[i11]) + (6.040034e-04) * (q[i0] * q[i12]) + (2.860531e-04) * (q[i0] * q[i15]) + (2.233752e-04) * (q[i0] * q[i16])
            + (-6.577718e-04) * (q[i0] * q[i19]) + (1.086136e-04) * (q[i0] * q[i20]) + (4.544889e-04) * (q[i0] * q[i21]) + (-1.092656e-04) * (q[i1] * q[i2])
            + (-2.770689e-03) * (q[i1] * q[i3]) + (2.335218e-03) * (q[i1] * q[i4]) + (-2.824797e-03) * (q[i1] * q[i5]) + (-4.110103e-04) * (q[i1] * q[i6])
            + (4.281154e-04) * (q[i1] * q[i7]) + (-5.526813e-04) * (q[i1] * q[i8]) + (-6.430545e-04) * (q[i1] * q[i9]) + (-1.747739e-04) * (q[i1] * q[i10])
            + (1.702132e-03) * (q[i1] * q[i11]) + (-8.373231e-04) * (q[i1] * q[i12]) + (-6.772398e-05) * (q[i1] * q[i15]) + (2.228063e-03) * (q[i1] * q[i16])
            + (8.183928e-04) * (q[i1] * q[i19]) + (-7.683510e-04) * (q[i1] * q[i20]) + (4.280431e-04) * (q[i1] * q[i21]) + (-3.860117e-03) * (q[i2] * q[i3])
            + (1.912853e-03) * (q[i2] * q[i4]) + (-2.288843e-03) * (q[i2] * q[i5]) + (4.758287e-04) * (q[i2] * q[i6]) + (3.561437e-04) * (q[i2] * q[i7])
            + (-7.450876e-04) * (q[i2] * q[i8]) + (-3.653917e-04) * (q[i2] * q[i9]) + (-8.714664e-05) * (q[i2] * q[i10]) + (-6.161720e-04) * (q[i2] * q[i11])
            + (-8.317397e-04) * (q[i2] * q[i12]) + (-3.798725e-04) * (q[i2] * q[i15]) + (1.684649e-03) * (q[i2] * q[i16]) + (3.099660e-04) * (q[i2] * q[i19])
            + (6.695144e-04) * (q[i2] * q[i20]) + (1.096783e-03) * (q[i2] * q[i21]) + (-1.279522e-03) * (q[i3] * q[i4]) + (-2.087590e-03) * (q[i3] * q[i5])
            + (3.790578e-03) * (q[i3] * q[i6]) + (-3.218778e-04) * (q[i3] * q[i7]) + (2.509093e-03) * (q[i3] * q[i8]) + (4.432069e-04) * (q[i3] * q[i9])
            + (-1.610804e-03) * (q[i3] * q[i10]) + (-7.618473e-04) * (q[i3] * q[i11]) + (9.435594e-05) * (q[i3] * q[i12]) + (1.001613e-03) * (q[i3] * q[i15])
            + (1.107827e-03) * (q[i3] * q[i16]) + (-7.987878e-04) * (q[i3] * q[i19]) + (1.826411e-03) * (q[i3] * q[i20]) + (7.898597e-05) * (q[i3] * q[i21])
            + (2.065826e-03) * (q[i4] * q[i5]) + (-1.427510e-03) * (q[i4] * q[i6]) + (1.006111e-03) * (q[i4] * q[i7]) + (-1.455301e-03) * (q[i4] * q[i8])
            + (2.096791e-03) * (q[i4] * q[i9]) + (1.577342e-03) * (q[i4] * q[i10]) + (-5.197775e-04) * (q[i4] * q[i11]) + (-1.530939e-03) * (q[i4] * q[i12])
            + (-9.009462e-04) * (q[i4] * q[i15]) + (-2.786692e-04) * (q[i4] * q[i16]) + (-8.107735e-04) * (q[i4] * q[i19]) + (-2.967091e-04) * (q[i4] * q[i20])
            + (6.475461e-05) * (q[i4] * q[i21]) + (-1.461438e-03) * (q[i5] * q[i6]) + (-1.590293e-03) * (q[i5] * q[i7]) + (-1.689967e-03) * (q[i5] * q[i8])
            + (-1.920676e-03) * (q[i5] * q[i9]) + (-7.536486e-04) * (q[i5] * q[i10]) + (7.702120e-04) * (q[i5] * q[i11]) + (-2.058770e-04) * (q[i5] * q[i12])
            + (1.713108e-04) * (q[i5] * q[i15]) + (1.665068e-04) * (q[i5] * q[i16]) + (5.709591e-04) * (q[i5] * q[i19]) + (-2.760606e-04) * (q[i5] * q[i20])
            + (-9.237874e-04) * (q[i5] * q[i21]) + (1.609069e-03) * (q[i6] * q[i7]) + (5.308637e-04) * (q[i6] * q[i8]) + (-2.454917e-03) * (q[i6] * q[i9])
            + (-4.929493e-04) * (q[i6] * q[i10]) + (1.473500e-03) * (q[i6] * q[i11]) + (1.967394e-03) * (q[i6] * q[i12]) + (9.407677e-04) * (q[i6] * q[i15])
            + (-1.211639e-05) * (q[i6] * q[i16]) + (1.952267e-04) * (q[i6] * q[i19]) + (-1.946770e-03) * (q[i6] * q[i20]) + (-3.455159e-04) * (q[i6] * q[i21])
            + (-3.697157e-04) * (q[i7] * q[i8]) + (1.636119e-03) * (q[i7] * q[i9]) + (1.984686e-04) * (q[i7] * q[i10]) + (1.176254e-03) * (q[i7] * q[i11])
            + (-1.900401e-03) * (q[i7] * q[i12]) + (5.829073e-04) * (q[i7] * q[i15]) + (-1.249415e-03) * (q[i7] * q[i16]) + (-4.046829e-05) * (q[i7] * q[i19])
            + (-1.112846e-03) * (q[i7] * q[i20]) + (3.555269e-04) * (q[i7] * q[i21]) + (4.128965e-04) * (q[i8] * q[i9]) + (-1.149477e-03) * (q[i8] * q[i10])
            + (-1.142485e-03) * (q[i8] * q[i11]) + (3.046039e-04) * (q[i8] * q[i12]) + (-3.028679e-04) * (q[i8] * q[i15]) + (2.667583e-03) * (q[i8] * q[i16])
            + (2.725898e-04) * (q[i8] * q[i19]) + (3.337514e-03) * (q[i8] * q[i20]) + (7.139614e-06) * (q[i8] * q[i21]) + (2.713456e-04) * (q[i9] * q[i10])
            + (5.739907e-04) * (q[i9] * q[i11]) + (-2.366834e-04) * (q[i9] * q[i12]) + (-3.685585e-05) * (q[i9] * q[i15]) + (2.033018e-05) * (q[i9] * q[i16])
            + (2.972330e-04) * (q[i9] * q[i19]) + (-3.028418e-04) * (q[i9] * q[i20]) + (-1.130490e-04) * (q[i9] * q[i21]) + (7.502987e-04) * (q[i10] * q[i11])
            + (-5.451014e-04) * (q[i10] * q[i12]) + (2.225760e-05) * (q[i10] * q[i15]) + (-6.301378e-04) * (q[i10] * q[i16])
            + (-1.014092e-05) * (q[i10] * q[i19]) + (-4.575649e-04) * (q[i10] * q[i20]) + (1.193717e-04) * (q[i10] * q[i21])
            + (1.110204e-04) * (q[i11] * q[i12]) + (-1.931456e-04) * (q[i11] * q[i15]) + (-9.794195e-05) * (q[i11] * q[i16])
            + (7.798466e-04) * (q[i11] * q[i19]) + (-3.240401e-04) * (q[i11] * q[i20]) + (2.629995e-04) * (q[i11] * q[i21])
            + (-1.425435e-04) * (q[i12] * q[i15]) + (-1.559937e-03) * (q[i12] * q[i16]) + (-3.151055e-04) * (q[i12] * q[i19])
            + (-1.866132e-03) * (q[i12] * q[i20]) + (2.602461e-04) * (q[i12] * q[i21]) + (-2.106631e-04) * (q[i15] * q[i16])
            + (-1.350526e-06) * (q[i15] * q[i19]) + (6.296997e-05) * (q[i15] * q[i20]) + (1.336803e-04) * (q[i15] * q[i21])
            + (-2.544436e-04) * (q[i16] * q[i19]) + (1.024102e-04) * (q[i16] * q[i20]) + (-1.343905e-04) * (q[i16] * q[i21])
            + (-2.701687e-05) * (q[i19] * q[i20]) + (8.061874e-05) * (q[i19] * q[i21]) + (-8.116803e-05) * (q[i20] * q[i21]);
   }

   //==========================================================================================================================================
}
