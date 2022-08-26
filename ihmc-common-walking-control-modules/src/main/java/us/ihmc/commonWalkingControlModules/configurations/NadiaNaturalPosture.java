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

   // Testing -- track local yaw angle
   // Note that the base (pelvis) has to turn in order for the upper body to turn together
   private Boolean trackingLocalYaw = true;

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
      // Testing -- do local yaw control
      Quaternion Q_world_base_with_zero_yaw = new Quaternion(Q_world_base);
      if (trackingLocalYaw)
      {
         Q_world_base_with_zero_yaw.setUnsafe(Q_world_base_with_zero_yaw.getX(), Q_world_base_with_zero_yaw.getY(), 0, Q_world_base_with_zero_yaw.getS());
         Q_world_base_with_zero_yaw.normalize();
      }

      // Get the NP quaternion r.t. the base(pelvis) frame:
      computeQuaternionNPrtBase(q, this.Q_Base_NP);

      // Express the NP quaternion in the world-frame:
      this.Q_World_NP.set(Q_world_base_with_zero_yaw);
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
         yoQuaternionIdent.set(Q_world_base_with_zero_yaw);
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
            if (trackingLocalYaw && (i == 2))
            {
               // Testing -- do local yaw control (zero out the third row of R^T because it corresponds to the term of angular vel of pelvis rt world 
               jacobianToPack.set(i, j, 0.0);
            }
            else
            {
               jacobianToPack.set(i, j, CnpBase.getElement(i, j)); // GMN: Need to see if we have submatrix operators...
            }
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

   //==== auto-generated functions below ======================================================================================================================================

   private String[] jointNameInOrder = new String[] {"LEFT_HIP_Z",
                                                     "RIGHT_HIP_Z",
                                                     "SPINE_Z",
                                                     "LEFT_HIP_X",
                                                     "RIGHT_HIP_X",
                                                     "SPINE_X",
                                                     "LEFT_HIP_Y",
                                                     "RIGHT_HIP_Y",
                                                     "SPINE_Y",
                                                     "LEFT_KNEE_Y",
                                                     "RIGHT_KNEE_Y",
                                                     "LEFT_SHOULDER_Y",
                                                     "RIGHT_SHOULDER_Y",
                                                     "LEFT_ANKLE_Y",
                                                     "RIGHT_ANKLE_Y",
                                                     "LEFT_SHOULDER_X",
                                                     "RIGHT_SHOULDER_X",
                                                     "LEFT_ANKLE_X",
                                                     "RIGHT_ANKLE_X",
                                                     "LEFT_SHOULDER_Z",
                                                     "RIGHT_SHOULDER_Z",
                                                     "LEFT_ELBOW_Y",
                                                     "RIGHT_ELBOW_Y",
                                                     "LEFT_WRIST_Z",
                                                     "RIGHT_WRIST_Z",
                                                     "LEFT_WRIST_X",
                                                     "RIGHT_WRIST_X",
                                                     "LEFT_WRIST_Y",
                                                     "RIGHT_WRIST_Y"};

   private Integer[] jointsToFit = new Integer[] {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 16, 17, 20, 21, 22, 23};

   public double getQx(double[] q)
   {
      double Qx;
      Qx = (-1.010091e-02) * q[i0] + (-1.004714e-02) * q[i1] + (1.441417e-03) * q[i2] + (1.310380e-01) * q[i3] + (1.304297e-01) * q[i4] + (1.312501e-01) * q[i5]
            + (1.517551e-03) * q[i6] + (-1.474994e-03) * q[i7] + (-1.043223e-04) * q[i8] + (5.489318e-03) * q[i9] + (-5.369225e-03) * q[i10]
            + (3.027550e-03) * q[i11] + (3.341064e-03) * q[i12] + (-2.513163e-03) * q[i15] + (2.431806e-03) * q[i16] + (-3.287858e-04) * q[i19]
            + (2.730810e-04) * q[i20] + (-3.412418e-03) * q[i21] + (-3.452125e-03) * q[i22] + (-2.367657e-03) * q[i0] * q[i0] + (2.331259e-03) * q[i1] * q[i1]
            + (-1.981026e-05) * q[i2] * q[i2] + (1.070055e-02) * q[i3] * q[i3] + (-1.064344e-02) * q[i4] * q[i4] + (-1.137564e-05) * q[i5] * q[i5]
            + (1.696235e-02) * q[i6] * q[i6] + (-1.690152e-02) * q[i7] * q[i7] + (-1.866490e-06) * q[i8] * q[i8] + (4.313477e-03) * q[i9] * q[i9]
            + (-4.259048e-03) * q[i10] * q[i10] + (5.266495e-03) * q[i11] * q[i11] + (-5.416358e-03) * q[i12] * q[i12] + (5.216814e-03) * q[i15] * q[i15]
            + (-5.259577e-03) * q[i16] * q[i16] + (8.486198e-04) * q[i19] * q[i19] + (-8.468018e-04) * q[i20] * q[i20] + (1.953283e-03) * q[i21] * q[i21]
            + (-1.955954e-03) * q[i22] * q[i22] + (1.383805e-05) * q[i0] * q[i1] + (-2.185157e-04) * q[i0] * q[i2] + (-2.598147e-04) * q[i0] * q[i3]
            + (-4.940217e-03) * q[i0] * q[i4] + (1.179885e-03) * q[i0] * q[i5] + (8.925438e-02) * q[i0] * q[i6] + (-1.064739e-02) * q[i0] * q[i7]
            + (-2.129090e-03) * q[i0] * q[i8] + (2.868910e-02) * q[i0] * q[i9] + (-1.991742e-03) * q[i0] * q[i10] + (2.180248e-03) * q[i0] * q[i11]
            + (-1.271582e-03) * q[i0] * q[i12] + (6.716320e-04) * q[i0] * q[i15] + (2.434471e-03) * q[i0] * q[i16] + (-1.256073e-03) * q[i0] * q[i19]
            + (4.558196e-04) * q[i0] * q[i20] + (1.785382e-03) * q[i0] * q[i21] + (7.426365e-04) * q[i0] * q[i22] + (1.845959e-04) * q[i1] * q[i2]
            + (4.926447e-03) * q[i1] * q[i3] + (2.307347e-04) * q[i1] * q[i4] + (-1.160811e-03) * q[i1] * q[i5] + (-1.065993e-02) * q[i1] * q[i6]
            + (8.875536e-02) * q[i1] * q[i7] + (-2.140533e-03) * q[i1] * q[i8] + (-2.013038e-03) * q[i1] * q[i9] + (2.838285e-02) * q[i1] * q[i10]
            + (1.245275e-03) * q[i1] * q[i11] + (-2.172879e-03) * q[i1] * q[i12] + (2.446439e-03) * q[i1] * q[i15] + (6.808327e-04) * q[i1] * q[i16]
            + (4.686219e-04) * q[i1] * q[i19] + (-1.242329e-03) * q[i1] * q[i20] + (-7.433376e-04) * q[i1] * q[i21] + (-1.775374e-03) * q[i1] * q[i22]
            + (-2.013881e-05) * q[i2] * q[i3] + (-3.492671e-05) * q[i2] * q[i4] + (7.287988e-05) * q[i2] * q[i5] + (1.497659e-02) * q[i2] * q[i6]
            + (1.484636e-02) * q[i2] * q[i7] + (-6.425181e-02) * q[i2] * q[i8] + (1.110663e-03) * q[i2] * q[i9] + (1.109812e-03) * q[i2] * q[i10]
            + (5.822078e-03) * q[i2] * q[i11] + (-5.992994e-03) * q[i2] * q[i12] + (3.217467e-03) * q[i2] * q[i15] + (3.282658e-03) * q[i2] * q[i16]
            + (-1.830097e-03) * q[i2] * q[i19] + (-1.825938e-03) * q[i2] * q[i20] + (8.509005e-04) * q[i2] * q[i21] + (-8.366886e-04) * q[i2] * q[i22]
            + (-8.651343e-07) * q[i3] * q[i4] + (-9.554202e-03) * q[i3] * q[i5] + (1.090700e-02) * q[i3] * q[i6] + (-5.409597e-03) * q[i3] * q[i7]
            + (2.761238e-03) * q[i3] * q[i8] + (-8.353152e-03) * q[i3] * q[i9] + (4.679714e-03) * q[i3] * q[i10] + (-1.397326e-03) * q[i3] * q[i11]
            + (1.370022e-03) * q[i3] * q[i12] + (4.420613e-03) * q[i3] * q[i15] + (4.782723e-03) * q[i3] * q[i16] + (-7.273987e-04) * q[i3] * q[i19]
            + (1.193762e-03) * q[i3] * q[i20] + (-6.352294e-04) * q[i3] * q[i21] + (3.072453e-04) * q[i3] * q[i22] + (9.530658e-03) * q[i4] * q[i5]
            + (-5.418235e-03) * q[i4] * q[i6] + (1.087283e-02) * q[i4] * q[i7] + (2.762834e-03) * q[i4] * q[i8] + (4.705070e-03) * q[i4] * q[i9]
            + (-8.281846e-03) * q[i4] * q[i10] + (-1.337685e-03) * q[i4] * q[i11] + (1.413711e-03) * q[i4] * q[i12] + (4.708550e-03) * q[i4] * q[i15]
            + (4.447938e-03) * q[i4] * q[i16] + (1.189330e-03) * q[i4] * q[i19] + (-7.088497e-04) * q[i4] * q[i20] + (-2.964353e-04) * q[i4] * q[i21]
            + (6.134833e-04) * q[i4] * q[i22] + (7.151259e-04) * q[i5] * q[i6] + (7.129366e-04) * q[i5] * q[i7] + (-1.648139e-03) * q[i5] * q[i8]
            + (3.173301e-03) * q[i5] * q[i9] + (3.166160e-03) * q[i5] * q[i10] + (5.353168e-03) * q[i5] * q[i11] + (-5.393804e-03) * q[i5] * q[i12]
            + (-1.472151e-02) * q[i5] * q[i15] + (-1.488733e-02) * q[i5] * q[i16] + (-2.978448e-04) * q[i5] * q[i19] + (-3.149214e-04) * q[i5] * q[i20]
            + (-5.199303e-04) * q[i5] * q[i21] + (5.237748e-04) * q[i5] * q[i22] + (-4.978121e-05) * q[i6] * q[i7] + (1.125088e-03) * q[i6] * q[i8]
            + (8.561279e-03) * q[i6] * q[i9] + (1.671832e-04) * q[i6] * q[i10] + (3.825531e-03) * q[i6] * q[i11] + (4.177021e-03) * q[i6] * q[i12]
            + (-9.606252e-04) * q[i6] * q[i15] + (-1.464468e-03) * q[i6] * q[i16] + (9.621823e-04) * q[i6] * q[i19] + (2.620063e-04) * q[i6] * q[i20]
            + (-1.346279e-03) * q[i6] * q[i21] + (-4.101186e-04) * q[i6] * q[i22] + (-1.136944e-03) * q[i7] * q[i8] + (-1.806331e-04) * q[i7] * q[i9]
            + (-8.450414e-03) * q[i7] * q[i10] + (4.162477e-03) * q[i7] * q[i11] + (3.783637e-03) * q[i7] * q[i12] + (1.467304e-03) * q[i7] * q[i15]
            + (9.646926e-04) * q[i7] * q[i16] + (-2.502844e-04) * q[i7] * q[i19] + (-9.387683e-04) * q[i7] * q[i20] + (-4.092206e-04) * q[i7] * q[i21]
            + (-1.333653e-03) * q[i7] * q[i22] + (1.078814e-03) * q[i8] * q[i9] + (-1.089813e-03) * q[i8] * q[i10] + (9.864555e-03) * q[i8] * q[i11]
            + (9.943276e-03) * q[i8] * q[i12] + (7.993956e-03) * q[i8] * q[i15] + (-8.089714e-03) * q[i8] * q[i16] + (6.607573e-04) * q[i8] * q[i19]
            + (-6.266187e-04) * q[i8] * q[i20] + (2.642570e-03) * q[i8] * q[i21] + (2.659965e-03) * q[i8] * q[i22] + (-1.742694e-06) * q[i9] * q[i10]
            + (3.720471e-04) * q[i9] * q[i11] + (7.644718e-04) * q[i9] * q[i12] + (-4.095399e-04) * q[i9] * q[i15] + (4.936475e-04) * q[i9] * q[i16]
            + (2.404055e-04) * q[i9] * q[i19] + (4.906908e-04) * q[i9] * q[i20] + (3.530960e-04) * q[i9] * q[i21] + (-8.328203e-05) * q[i9] * q[i22]
            + (7.565401e-04) * q[i10] * q[i11] + (3.643949e-04) * q[i10] * q[i12] + (-4.787359e-04) * q[i10] * q[i15] + (4.054355e-04) * q[i10] * q[i16]
            + (-4.933374e-04) * q[i10] * q[i19] + (-2.292215e-04) * q[i10] * q[i20] + (-7.698729e-05) * q[i10] * q[i21] + (3.507823e-04) * q[i10] * q[i22]
            + (4.677954e-06) * q[i11] * q[i12] + (2.482377e-03) * q[i11] * q[i15] + (7.708882e-05) * q[i11] * q[i16] + (-1.381336e-03) * q[i11] * q[i19]
            + (8.973351e-04) * q[i11] * q[i20] + (2.252492e-04) * q[i11] * q[i21] + (-2.830865e-04) * q[i11] * q[i22] + (8.698621e-05) * q[i12] * q[i15]
            + (2.562287e-03) * q[i12] * q[i16] + (9.009046e-04) * q[i12] * q[i19] + (-1.403589e-03) * q[i12] * q[i20] + (2.860958e-04) * q[i12] * q[i21]
            + (-2.284272e-04) * q[i12] * q[i22] + (-3.239600e-06) * q[i15] * q[i16] + (1.028355e-03) * q[i15] * q[i19] + (-4.449288e-04) * q[i15] * q[i20]
            + (2.043079e-03) * q[i15] * q[i21] + (-1.155256e-04) * q[i15] * q[i22] + (4.367905e-04) * q[i16] * q[i19] + (-9.992207e-04) * q[i16] * q[i20]
            + (-1.113394e-04) * q[i16] * q[i21] + (2.038371e-03) * q[i16] * q[i22] + (-4.093346e-06) * q[i19] * q[i20] + (-7.656317e-04) * q[i19] * q[i21]
            + (3.017889e-05) * q[i19] * q[i22] + (2.101496e-05) * q[i20] * q[i21] + (-7.428983e-04) * q[i20] * q[i22] + (-5.025617e-06) * q[i21] * q[i22]
            + (5.669814e-04) * q[i0] * q[i0] * q[i0] + (-9.825471e-04) * q[i0] * q[i0] * q[i1] + (1.184207e-03) * q[i0] * q[i0] * q[i2]
            + (-2.992206e-02) * q[i0] * q[i0] * q[i3] + (-2.018558e-03) * q[i0] * q[i0] * q[i4] + (-2.422992e-03) * q[i0] * q[i0] * q[i5]
            + (3.158567e-04) * q[i0] * q[i0] * q[i6] + (6.750960e-04) * q[i0] * q[i0] * q[i7] + (-1.325909e-03) * q[i0] * q[i0] * q[i8]
            + (2.091410e-03) * q[i0] * q[i0] * q[i9] + (-4.548212e-04) * q[i0] * q[i0] * q[i10] + (5.786766e-04) * q[i0] * q[i0] * q[i11]
            + (4.670338e-04) * q[i0] * q[i0] * q[i12] + (-5.193199e-04) * q[i0] * q[i0] * q[i15] + (6.033945e-04) * q[i0] * q[i0] * q[i16]
            + (3.062619e-04) * q[i0] * q[i0] * q[i19] + (-9.033055e-04) * q[i0] * q[i0] * q[i20] + (5.415450e-04) * q[i0] * q[i0] * q[i21]
            + (-5.642650e-04) * q[i0] * q[i0] * q[i22] + (-9.887655e-04) * q[i0] * q[i1] * q[i1] + (5.713060e-04) * q[i1] * q[i1] * q[i1]
            + (1.177124e-03) * q[i1] * q[i1] * q[i2] + (-2.003344e-03) * q[i1] * q[i1] * q[i3] + (-2.984955e-02) * q[i1] * q[i1] * q[i4]
            + (-2.404036e-03) * q[i1] * q[i1] * q[i5] + (-6.653566e-04) * q[i1] * q[i1] * q[i6] + (-3.216427e-04) * q[i1] * q[i1] * q[i7]
            + (1.317663e-03) * q[i1] * q[i1] * q[i8] + (4.706956e-04) * q[i1] * q[i1] * q[i9] + (-2.075597e-03) * q[i1] * q[i1] * q[i10]
            + (4.802014e-04) * q[i1] * q[i1] * q[i11] + (5.728565e-04) * q[i1] * q[i1] * q[i12] + (-6.010900e-04) * q[i1] * q[i1] * q[i15]
            + (5.145223e-04) * q[i1] * q[i1] * q[i16] + (8.886021e-04) * q[i1] * q[i1] * q[i19] + (-2.973511e-04) * q[i1] * q[i1] * q[i20]
            + (-5.628567e-04) * q[i1] * q[i1] * q[i21] + (5.422990e-04) * q[i1] * q[i1] * q[i22] + (3.324110e-04) * q[i0] * q[i2] * q[i2]
            + (3.103255e-04) * q[i1] * q[i2] * q[i2] + (2.386629e-03) * q[i2] * q[i2] * q[i2] + (-2.506449e-03) * q[i2] * q[i2] * q[i3]
            + (-2.486643e-03) * q[i2] * q[i2] * q[i4] + (-3.008131e-02) * q[i2] * q[i2] * q[i5] + (9.338505e-05) * q[i2] * q[i2] * q[i6]
            + (-9.476205e-05) * q[i2] * q[i2] * q[i7] + (-2.584377e-05) * q[i2] * q[i2] * q[i8] + (-7.424691e-05) * q[i2] * q[i2] * q[i9]
            + (6.551763e-05) * q[i2] * q[i2] * q[i10] + (6.818229e-04) * q[i2] * q[i2] * q[i11] + (6.679535e-04) * q[i2] * q[i2] * q[i12]
            + (-3.073569e-03) * q[i2] * q[i2] * q[i15] + (3.108143e-03) * q[i2] * q[i2] * q[i16] + (-2.805107e-04) * q[i2] * q[i2] * q[i19]
            + (2.853590e-04) * q[i2] * q[i2] * q[i20] + (4.678450e-04) * q[i2] * q[i2] * q[i21] + (4.736763e-04) * q[i2] * q[i2] * q[i22]
            + (-1.536166e-03) * q[i0] * q[i3] * q[i3] + (-1.951041e-03) * q[i1] * q[i3] * q[i3] + (-2.706366e-03) * q[i2] * q[i3] * q[i3]
            + (-2.965197e-03) * q[i3] * q[i3] * q[i3] + (-1.202879e-03) * q[i3] * q[i3] * q[i4] + (4.928136e-03) * q[i3] * q[i3] * q[i5]
            + (6.705721e-04) * q[i3] * q[i3] * q[i6] + (-1.513164e-03) * q[i3] * q[i3] * q[i7] + (5.034326e-04) * q[i3] * q[i3] * q[i8]
            + (-5.224901e-04) * q[i3] * q[i3] * q[i9] + (-8.534385e-04) * q[i3] * q[i3] * q[i10] + (-8.827741e-04) * q[i3] * q[i3] * q[i11]
            + (-1.418222e-03) * q[i3] * q[i3] * q[i12] + (8.495688e-05) * q[i3] * q[i3] * q[i15] + (7.292597e-05) * q[i3] * q[i3] * q[i16]
            + (-4.504642e-04) * q[i3] * q[i3] * q[i19] + (4.050251e-04) * q[i3] * q[i3] * q[i20] + (-5.546236e-05) * q[i3] * q[i3] * q[i21]
            + (1.992528e-04) * q[i3] * q[i3] * q[i22] + (-1.931539e-03) * q[i0] * q[i4] * q[i4] + (-1.536351e-03) * q[i1] * q[i4] * q[i4]
            + (-2.704664e-03) * q[i2] * q[i4] * q[i4] + (-1.222937e-03) * q[i3] * q[i4] * q[i4] + (-2.979427e-03) * q[i4] * q[i4] * q[i4]
            + (4.909451e-03) * q[i4] * q[i4] * q[i5] + (1.513751e-03) * q[i4] * q[i4] * q[i6] + (-6.565926e-04) * q[i4] * q[i4] * q[i7]
            + (-4.924872e-04) * q[i4] * q[i4] * q[i8] + (8.403889e-04) * q[i4] * q[i4] * q[i9] + (5.077441e-04) * q[i4] * q[i4] * q[i10]
            + (-1.417677e-03) * q[i4] * q[i4] * q[i11] + (-8.833817e-04) * q[i4] * q[i4] * q[i12] + (-6.731487e-05) * q[i4] * q[i4] * q[i15]
            + (-8.986242e-05) * q[i4] * q[i4] * q[i16] + (-3.920504e-04) * q[i4] * q[i4] * q[i19] + (4.323403e-04) * q[i4] * q[i4] * q[i20]
            + (2.000950e-04) * q[i4] * q[i4] * q[i21] + (-6.013280e-05) * q[i4] * q[i4] * q[i22] + (-1.039680e-03) * q[i0] * q[i5] * q[i5]
            + (-1.022238e-03) * q[i1] * q[i5] * q[i5] + (-9.399917e-04) * q[i2] * q[i5] * q[i5] + (4.794269e-05) * q[i3] * q[i5] * q[i5]
            + (8.117997e-05) * q[i4] * q[i5] * q[i5] + (-5.570426e-03) * q[i5] * q[i5] * q[i5] + (1.849501e-04) * q[i5] * q[i5] * q[i6]
            + (-1.648071e-04) * q[i5] * q[i5] * q[i7] + (-1.426346e-05) * q[i5] * q[i5] * q[i8] + (-5.364463e-04) * q[i5] * q[i5] * q[i9]
            + (5.496565e-04) * q[i5] * q[i5] * q[i10] + (1.577033e-05) * q[i5] * q[i5] * q[i11] + (1.886787e-05) * q[i5] * q[i5] * q[i12]
            + (-5.557385e-04) * q[i5] * q[i5] * q[i15] + (5.689679e-04) * q[i5] * q[i5] * q[i16] + (6.193059e-04) * q[i5] * q[i5] * q[i19]
            + (-6.123879e-04) * q[i5] * q[i5] * q[i20] + (-2.430805e-04) * q[i5] * q[i5] * q[i21] + (-2.432180e-04) * q[i5] * q[i5] * q[i22]
            + (5.723209e-03) * q[i0] * q[i6] * q[i6] + (-3.361309e-03) * q[i1] * q[i6] * q[i6] + (6.404787e-04) * q[i2] * q[i6] * q[i6]
            + (-2.172440e-02) * q[i3] * q[i6] * q[i6] + (3.718044e-03) * q[i4] * q[i6] * q[i6] + (6.099329e-03) * q[i5] * q[i6] * q[i6]
            + (-2.270557e-04) * q[i6] * q[i6] * q[i6] + (-9.038252e-04) * q[i6] * q[i6] * q[i7] + (4.100959e-04) * q[i6] * q[i6] * q[i8]
            + (-2.264878e-03) * q[i6] * q[i6] * q[i9] + (5.487510e-04) * q[i6] * q[i6] * q[i10] + (4.218537e-04) * q[i6] * q[i6] * q[i11]
            + (-3.891096e-04) * q[i6] * q[i6] * q[i12] + (-9.299073e-05) * q[i6] * q[i6] * q[i15] + (6.497508e-04) * q[i6] * q[i6] * q[i16]
            + (4.450975e-05) * q[i6] * q[i6] * q[i19] + (1.616839e-04) * q[i6] * q[i6] * q[i20] + (-3.261228e-06) * q[i6] * q[i6] * q[i21]
            + (-1.632641e-05) * q[i6] * q[i6] * q[i22] + (-3.356871e-03) * q[i0] * q[i7] * q[i7] + (5.746949e-03) * q[i1] * q[i7] * q[i7]
            + (6.425832e-04) * q[i2] * q[i7] * q[i7] + (3.708187e-03) * q[i3] * q[i7] * q[i7] + (-2.165774e-02) * q[i4] * q[i7] * q[i7]
            + (6.091801e-03) * q[i5] * q[i7] * q[i7] + (9.049236e-04) * q[i6] * q[i7] * q[i7] + (2.205851e-04) * q[i7] * q[i7] * q[i7]
            + (-4.012663e-04) * q[i7] * q[i7] * q[i8] + (-5.476560e-04) * q[i7] * q[i7] * q[i9] + (2.235544e-03) * q[i7] * q[i7] * q[i10]
            + (-3.949178e-04) * q[i7] * q[i7] * q[i11] + (4.162036e-04) * q[i7] * q[i7] * q[i12] + (-6.495229e-04) * q[i7] * q[i7] * q[i15]
            + (8.193634e-05) * q[i7] * q[i7] * q[i16] + (-1.559224e-04) * q[i7] * q[i7] * q[i19] + (-5.194834e-05) * q[i7] * q[i7] * q[i20]
            + (-2.092241e-05) * q[i7] * q[i7] * q[i21] + (-5.552237e-06) * q[i7] * q[i7] * q[i22] + (-7.268526e-05) * q[i0] * q[i8] * q[i8]
            + (-6.196483e-05) * q[i1] * q[i8] * q[i8] + (2.673863e-03) * q[i2] * q[i8] * q[i8] + (4.515987e-03) * q[i3] * q[i8] * q[i8]
            + (4.518467e-03) * q[i4] * q[i8] * q[i8] + (-2.254533e-02) * q[i5] * q[i8] * q[i8] + (-7.129223e-04) * q[i6] * q[i8] * q[i8]
            + (7.053321e-04) * q[i7] * q[i8] * q[i8] + (1.276820e-05) * q[i8] * q[i8] * q[i8] + (-1.608917e-04) * q[i8] * q[i8] * q[i9]
            + (1.536928e-04) * q[i8] * q[i8] * q[i10] + (-1.381044e-03) * q[i8] * q[i8] * q[i11] + (-1.417740e-03) * q[i8] * q[i8] * q[i12]
            + (3.791046e-03) * q[i8] * q[i8] * q[i15] + (-3.834838e-03) * q[i8] * q[i8] * q[i16] + (2.476959e-06) * q[i8] * q[i8] * q[i19]
            + (6.359908e-07) * q[i8] * q[i8] * q[i20] + (-1.303810e-04) * q[i8] * q[i8] * q[i21] + (-1.347525e-04) * q[i8] * q[i8] * q[i22]
            + (-6.680454e-03) * q[i0] * q[i9] * q[i9] + (1.242082e-03) * q[i1] * q[i9] * q[i9] + (-1.726621e-03) * q[i2] * q[i9] * q[i9]
            + (-3.512466e-03) * q[i3] * q[i9] * q[i9] + (7.681636e-04) * q[i4] * q[i9] * q[i9] + (1.550665e-03) * q[i5] * q[i9] * q[i9]
            + (-2.245750e-03) * q[i6] * q[i9] * q[i9] + (8.848611e-04) * q[i7] * q[i9] * q[i9] + (4.676212e-04) * q[i8] * q[i9] * q[i9]
            + (-6.165831e-04) * q[i9] * q[i9] * q[i9] + (1.651695e-04) * q[i9] * q[i9] * q[i10] + (-1.470574e-04) * q[i9] * q[i9] * q[i11]
            + (-2.736369e-04) * q[i9] * q[i9] * q[i12] + (2.042519e-04) * q[i9] * q[i9] * q[i15] + (1.160582e-04) * q[i9] * q[i9] * q[i16]
            + (-1.989461e-04) * q[i9] * q[i9] * q[i19] + (1.046171e-04) * q[i9] * q[i9] * q[i20] + (-5.645742e-05) * q[i9] * q[i9] * q[i21]
            + (-8.623598e-06) * q[i9] * q[i9] * q[i22] + (1.232645e-03) * q[i0] * q[i10] * q[i10] + (-6.606172e-03) * q[i1] * q[i10] * q[i10]
            + (-1.701862e-03) * q[i2] * q[i10] * q[i10] + (7.635137e-04) * q[i3] * q[i10] * q[i10] + (-3.469145e-03) * q[i4] * q[i10] * q[i10]
            + (1.532201e-03) * q[i5] * q[i10] * q[i10] + (-8.673242e-04) * q[i6] * q[i10] * q[i10] + (2.211956e-03) * q[i7] * q[i10] * q[i10]
            + (-4.618617e-04) * q[i8] * q[i10] * q[i10] + (-1.636058e-04) * q[i9] * q[i10] * q[i10] + (6.074131e-04) * q[i10] * q[i10] * q[i10]
            + (-2.739555e-04) * q[i10] * q[i10] * q[i11] + (-1.446726e-04) * q[i10] * q[i10] * q[i12] + (-1.148960e-04) * q[i10] * q[i10] * q[i15]
            + (-2.034701e-04) * q[i10] * q[i10] * q[i16] + (-1.036331e-04) * q[i10] * q[i10] * q[i19] + (1.934299e-04) * q[i10] * q[i10] * q[i20]
            + (-9.143866e-06) * q[i10] * q[i10] * q[i21] + (-5.630073e-05) * q[i10] * q[i10] * q[i22] + (3.231714e-04) * q[i0] * q[i11] * q[i11]
            + (1.890957e-04) * q[i1] * q[i11] * q[i11] + (8.918757e-05) * q[i2] * q[i11] * q[i11] + (-3.944303e-04) * q[i3] * q[i11] * q[i11]
            + (-9.777769e-06) * q[i4] * q[i11] * q[i11] + (-1.498545e-03) * q[i5] * q[i11] * q[i11] + (-3.333575e-04) * q[i6] * q[i11] * q[i11]
            + (-5.456559e-05) * q[i7] * q[i11] * q[i11] + (1.947496e-04) * q[i8] * q[i11] * q[i11] + (-7.500403e-05) * q[i9] * q[i11] * q[i11]
            + (3.721239e-04) * q[i10] * q[i11] * q[i11] + (2.842080e-05) * q[i11] * q[i11] * q[i11] + (3.174098e-05) * q[i11] * q[i11] * q[i12]
            + (1.694161e-03) * q[i11] * q[i11] * q[i15] + (1.845193e-04) * q[i11] * q[i11] * q[i16] + (5.351644e-04) * q[i11] * q[i11] * q[i19]
            + (-2.024991e-04) * q[i11] * q[i11] * q[i20] + (7.965434e-04) * q[i11] * q[i11] * q[i21] + (-9.622360e-05) * q[i11] * q[i11] * q[i22]
            + (1.600572e-04) * q[i0] * q[i12] * q[i12] + (3.179734e-04) * q[i1] * q[i12] * q[i12] + (-4.404651e-06) * q[i2] * q[i12] * q[i12]
            + (1.547685e-05) * q[i3] * q[i12] * q[i12] + (-4.022853e-04) * q[i4] * q[i12] * q[i12] + (-1.566259e-03) * q[i5] * q[i12] * q[i12]
            + (7.173906e-05) * q[i6] * q[i12] * q[i12] + (3.419957e-04) * q[i7] * q[i12] * q[i12] + (-1.097498e-04) * q[i8] * q[i12] * q[i12]
            + (-3.851676e-04) * q[i9] * q[i12] * q[i12] + (6.770298e-05) * q[i10] * q[i12] * q[i12] + (3.942835e-05) * q[i11] * q[i12] * q[i12]
            + (-2.357827e-05) * q[i12] * q[i12] * q[i12] + (-1.939265e-04) * q[i12] * q[i12] * q[i15] + (-1.712646e-03) * q[i12] * q[i12] * q[i16]
            + (1.989071e-04) * q[i12] * q[i12] * q[i19] + (-5.191837e-04) * q[i12] * q[i12] * q[i20] + (-9.978708e-05) * q[i12] * q[i12] * q[i21]
            + (8.008094e-04) * q[i12] * q[i12] * q[i22] + (1.483345e-04) * q[i0] * q[i15] * q[i15] + (1.062923e-04) * q[i1] * q[i15] * q[i15]
            + (1.988861e-03) * q[i2] * q[i15] * q[i15] + (2.427431e-04) * q[i3] * q[i15] * q[i15] + (6.388368e-04) * q[i4] * q[i15] * q[i15]
            + (-2.441580e-03) * q[i5] * q[i15] * q[i15] + (-1.249697e-05) * q[i6] * q[i15] * q[i15] + (-7.857293e-05) * q[i7] * q[i15] * q[i15]
            + (2.194621e-03) * q[i8] * q[i15] * q[i15] + (7.354129e-05) * q[i9] * q[i15] * q[i15] + (7.832320e-05) * q[i10] * q[i15] * q[i15]
            + (2.172944e-03) * q[i11] * q[i15] * q[i15] + (1.105513e-04) * q[i12] * q[i15] * q[i15] + (5.377181e-04) * q[i15] * q[i15] * q[i15]
            + (-1.811882e-05) * q[i15] * q[i15] * q[i16] + (1.538827e-04) * q[i15] * q[i15] * q[i19] + (3.743298e-05) * q[i15] * q[i15] * q[i20]
            + (5.407792e-04) * q[i15] * q[i15] * q[i21] + (6.319877e-05) * q[i15] * q[i15] * q[i22] + (1.020646e-04) * q[i0] * q[i16] * q[i16]
            + (1.516463e-04) * q[i1] * q[i16] * q[i16] + (2.009771e-03) * q[i2] * q[i16] * q[i16] + (6.422297e-04) * q[i3] * q[i16] * q[i16]
            + (2.455638e-04) * q[i4] * q[i16] * q[i16] + (-2.458229e-03) * q[i5] * q[i16] * q[i16] + (8.264544e-05) * q[i6] * q[i16] * q[i16]
            + (1.906856e-05) * q[i7] * q[i16] * q[i16] + (-2.207795e-03) * q[i8] * q[i16] * q[i16] + (-7.777213e-05) * q[i9] * q[i16] * q[i16]
            + (-7.392163e-05) * q[i10] * q[i16] * q[i16] + (1.075104e-04) * q[i11] * q[i16] * q[i16] + (2.204910e-03) * q[i12] * q[i16] * q[i16]
            + (1.829172e-05) * q[i15] * q[i16] * q[i16] + (-5.379419e-04) * q[i16] * q[i16] * q[i16] + (-3.690873e-05) * q[i16] * q[i16] * q[i19]
            + (-1.493813e-04) * q[i16] * q[i16] * q[i20] + (6.428493e-05) * q[i16] * q[i16] * q[i21] + (5.458667e-04) * q[i16] * q[i16] * q[i22]
            + (-1.449182e-04) * q[i0] * q[i19] * q[i19] + (5.154156e-05) * q[i1] * q[i19] * q[i19] + (-1.105789e-04) * q[i2] * q[i19] * q[i19]
            + (-3.432702e-05) * q[i3] * q[i19] * q[i19] + (3.278650e-05) * q[i4] * q[i19] * q[i19] + (5.381257e-06) * q[i5] * q[i19] * q[i19]
            + (-1.282560e-05) * q[i6] * q[i19] * q[i19] + (1.689597e-04) * q[i7] * q[i19] * q[i19] + (3.474673e-04) * q[i8] * q[i19] * q[i19]
            + (1.758296e-05) * q[i9] * q[i19] * q[i19] + (-3.597388e-05) * q[i10] * q[i19] * q[i19] + (1.771287e-05) * q[i11] * q[i19] * q[i19]
            + (-1.250546e-05) * q[i12] * q[i19] * q[i19] + (2.848873e-04) * q[i15] * q[i19] * q[i19] + (1.441866e-04) * q[i16] * q[i19] * q[i19]
            + (1.591417e-04) * q[i19] * q[i19] * q[i19] + (-5.491691e-05) * q[i19] * q[i19] * q[i20] + (6.623013e-04) * q[i19] * q[i19] * q[i21]
            + (-8.550832e-05) * q[i19] * q[i19] * q[i22] + (5.001423e-05) * q[i0] * q[i20] * q[i20] + (-1.397175e-04) * q[i1] * q[i20] * q[i20]
            + (-1.112106e-04) * q[i2] * q[i20] * q[i20] + (2.891431e-05) * q[i3] * q[i20] * q[i20] + (-3.742291e-05) * q[i4] * q[i20] * q[i20]
            + (1.593998e-05) * q[i5] * q[i20] * q[i20] + (-1.731079e-04) * q[i6] * q[i20] * q[i20] + (1.524099e-05) * q[i7] * q[i20] * q[i20]
            + (-3.542810e-04) * q[i8] * q[i20] * q[i20] + (3.629395e-05) * q[i9] * q[i20] * q[i20] + (-1.600149e-05) * q[i10] * q[i20] * q[i20]
            + (-1.225013e-05) * q[i11] * q[i20] * q[i20] + (3.205542e-05) * q[i12] * q[i20] * q[i20] + (-1.412690e-04) * q[i15] * q[i20] * q[i20]
            + (-2.797231e-04) * q[i16] * q[i20] * q[i20] + (5.428668e-05) * q[i19] * q[i20] * q[i20] + (-1.525967e-04) * q[i20] * q[i20] * q[i20]
            + (-8.564171e-05) * q[i20] * q[i20] * q[i21] + (6.618400e-04) * q[i20] * q[i20] * q[i22] + (6.503951e-05) * q[i0] * q[i21] * q[i21]
            + (4.499750e-06) * q[i1] * q[i21] * q[i21] + (1.895148e-04) * q[i2] * q[i21] * q[i21] + (4.343329e-05) * q[i3] * q[i21] * q[i21]
            + (5.999856e-04) * q[i4] * q[i21] * q[i21] + (-1.538097e-03) * q[i5] * q[i21] * q[i21] + (-5.732377e-05) * q[i6] * q[i21] * q[i21]
            + (-3.043743e-04) * q[i7] * q[i21] * q[i21] + (7.875672e-04) * q[i8] * q[i21] * q[i21] + (6.271647e-05) * q[i9] * q[i21] * q[i21]
            + (-1.086686e-04) * q[i10] * q[i21] * q[i21] + (3.680343e-04) * q[i11] * q[i21] * q[i21] + (5.457835e-05) * q[i12] * q[i21] * q[i21]
            + (7.180441e-04) * q[i15] * q[i21] * q[i21] + (-3.764340e-05) * q[i16] * q[i21] * q[i21] + (-1.431363e-04) * q[i19] * q[i21] * q[i21]
            + (-2.911553e-05) * q[i20] * q[i21] * q[i21] + (3.875186e-04) * q[i21] * q[i21] * q[i21] + (5.605760e-05) * q[i21] * q[i21] * q[i22]
            + (4.423919e-06) * q[i0] * q[i22] * q[i22] + (6.438361e-05) * q[i1] * q[i22] * q[i22] + (1.868505e-04) * q[i2] * q[i22] * q[i22]
            + (6.041209e-04) * q[i3] * q[i22] * q[i22] + (4.531129e-05) * q[i4] * q[i22] * q[i22] + (-1.540498e-03) * q[i5] * q[i22] * q[i22]
            + (3.050303e-04) * q[i6] * q[i22] * q[i22] + (5.152297e-05) * q[i7] * q[i22] * q[i22] + (-7.910245e-04) * q[i8] * q[i22] * q[i22]
            + (1.106597e-04) * q[i9] * q[i22] * q[i22] + (-6.206295e-05) * q[i10] * q[i22] * q[i22] + (5.465612e-05) * q[i11] * q[i22] * q[i22]
            + (3.705402e-04) * q[i12] * q[i22] * q[i22] + (4.040670e-05) * q[i15] * q[i22] * q[i22] + (-7.192742e-04) * q[i16] * q[i22] * q[i22]
            + (3.088854e-05) * q[i19] * q[i22] * q[i22] + (1.418221e-04) * q[i20] * q[i22] * q[i22] + (5.699931e-05) * q[i21] * q[i22] * q[i22]
            + (3.898745e-04) * q[i22] * q[i22] * q[i22] + (-1.687786e-03) * q[i0] * q[i1] * q[i2] + (8.098341e-03) * q[i0] * q[i1] * q[i3]
            + (8.106983e-03) * q[i0] * q[i1] * q[i4] + (-2.589524e-03) * q[i0] * q[i1] * q[i5] + (1.561763e-03) * q[i0] * q[i1] * q[i6]
            + (-1.557649e-03) * q[i0] * q[i1] * q[i7] + (3.874993e-06) * q[i0] * q[i1] * q[i8] + (-1.265387e-03) * q[i0] * q[i1] * q[i9]
            + (1.262403e-03) * q[i0] * q[i1] * q[i10] + (-5.326159e-04) * q[i0] * q[i1] * q[i11] + (-5.398613e-04) * q[i0] * q[i1] * q[i12]
            + (1.070189e-03) * q[i0] * q[i1] * q[i15] + (-1.067377e-03) * q[i0] * q[i1] * q[i16] + (-3.577786e-04) * q[i0] * q[i1] * q[i19]
            + (3.680753e-04) * q[i0] * q[i1] * q[i20] + (6.375455e-04) * q[i0] * q[i1] * q[i21] + (6.293637e-04) * q[i0] * q[i1] * q[i22]
            + (-1.333621e-02) * q[i0] * q[i2] * q[i3] + (-8.794458e-04) * q[i0] * q[i2] * q[i4] + (-7.566281e-03) * q[i0] * q[i2] * q[i5]
            + (1.107415e-03) * q[i0] * q[i2] * q[i6] + (-5.434355e-04) * q[i0] * q[i2] * q[i7] + (-3.435710e-03) * q[i0] * q[i2] * q[i8]
            + (6.688534e-04) * q[i0] * q[i2] * q[i9] + (1.871623e-04) * q[i0] * q[i2] * q[i10] + (4.646967e-04) * q[i0] * q[i2] * q[i11]
            + (1.638474e-03) * q[i0] * q[i2] * q[i12] + (-1.319058e-03) * q[i0] * q[i2] * q[i15] + (2.183712e-03) * q[i0] * q[i2] * q[i16]
            + (5.679102e-04) * q[i0] * q[i2] * q[i19] + (9.169973e-04) * q[i0] * q[i2] * q[i20] + (4.389959e-04) * q[i0] * q[i2] * q[i21]
            + (1.853998e-04) * q[i0] * q[i2] * q[i22] + (4.782162e-04) * q[i0] * q[i3] * q[i4] + (2.791141e-03) * q[i0] * q[i3] * q[i5]
            + (-1.013608e-02) * q[i0] * q[i3] * q[i6] + (3.139681e-03) * q[i0] * q[i3] * q[i7] + (2.277535e-03) * q[i0] * q[i3] * q[i8]
            + (-5.963577e-04) * q[i0] * q[i3] * q[i9] + (2.242324e-03) * q[i0] * q[i3] * q[i10] + (-8.594092e-04) * q[i0] * q[i3] * q[i11]
            + (-1.465488e-03) * q[i0] * q[i3] * q[i12] + (2.222247e-03) * q[i0] * q[i3] * q[i15] + (6.065313e-04) * q[i0] * q[i3] * q[i16]
            + (9.987913e-04) * q[i0] * q[i3] * q[i19] + (3.553240e-04) * q[i0] * q[i3] * q[i20] + (-3.321404e-04) * q[i0] * q[i3] * q[i21]
            + (-8.913986e-04) * q[i0] * q[i3] * q[i22] + (3.717767e-03) * q[i0] * q[i4] * q[i5] + (-2.233771e-03) * q[i0] * q[i4] * q[i6]
            + (9.178476e-04) * q[i0] * q[i4] * q[i7] + (-2.001233e-03) * q[i0] * q[i4] * q[i8] + (5.563906e-04) * q[i0] * q[i4] * q[i9]
            + (6.042899e-04) * q[i0] * q[i4] * q[i10] + (1.327075e-03) * q[i0] * q[i4] * q[i11] + (-1.168324e-03) * q[i0] * q[i4] * q[i12]
            + (2.300713e-04) * q[i0] * q[i4] * q[i15] + (-2.979545e-04) * q[i0] * q[i4] * q[i16] + (-9.249756e-05) * q[i0] * q[i4] * q[i19]
            + (-3.000105e-04) * q[i0] * q[i4] * q[i20] + (1.880298e-05) * q[i0] * q[i4] * q[i21] + (-2.734117e-05) * q[i0] * q[i4] * q[i22]
            + (-6.703300e-03) * q[i0] * q[i5] * q[i6] + (-2.394482e-04) * q[i0] * q[i5] * q[i7] + (3.941155e-04) * q[i0] * q[i5] * q[i8]
            + (-5.443572e-04) * q[i0] * q[i5] * q[i9] + (-5.315843e-04) * q[i0] * q[i5] * q[i10] + (8.634613e-04) * q[i0] * q[i5] * q[i11]
            + (4.483416e-04) * q[i0] * q[i5] * q[i12] + (-6.115209e-04) * q[i0] * q[i5] * q[i15] + (-1.147948e-03) * q[i0] * q[i5] * q[i16]
            + (-6.164790e-04) * q[i0] * q[i5] * q[i19] + (-3.585268e-04) * q[i0] * q[i5] * q[i20] + (2.820474e-04) * q[i0] * q[i5] * q[i21]
            + (4.204164e-04) * q[i0] * q[i5] * q[i22] + (-1.802536e-03) * q[i0] * q[i6] * q[i7] + (7.060447e-03) * q[i0] * q[i6] * q[i8]
            + (-1.365839e-02) * q[i0] * q[i6] * q[i9] + (2.730496e-03) * q[i0] * q[i6] * q[i10] + (-1.453379e-03) * q[i0] * q[i6] * q[i11]
            + (-1.077763e-03) * q[i0] * q[i6] * q[i12] + (7.646598e-04) * q[i0] * q[i6] * q[i15] + (1.406846e-03) * q[i0] * q[i6] * q[i16]
            + (6.747908e-04) * q[i0] * q[i6] * q[i19] + (1.608036e-04) * q[i0] * q[i6] * q[i20] + (-7.451047e-04) * q[i0] * q[i6] * q[i21]
            + (8.910689e-04) * q[i0] * q[i6] * q[i22] + (3.706215e-04) * q[i0] * q[i7] * q[i8] + (3.080839e-04) * q[i0] * q[i7] * q[i9]
            + (1.284695e-03) * q[i0] * q[i7] * q[i10] + (-1.745295e-03) * q[i0] * q[i7] * q[i11] + (-9.625702e-04) * q[i0] * q[i7] * q[i12]
            + (-6.808599e-04) * q[i0] * q[i7] * q[i15] + (1.844087e-04) * q[i0] * q[i7] * q[i16] + (-2.213321e-04) * q[i0] * q[i7] * q[i19]
            + (-1.875617e-04) * q[i0] * q[i7] * q[i20] + (1.814035e-05) * q[i0] * q[i7] * q[i21] + (-3.477295e-04) * q[i0] * q[i7] * q[i22]
            + (7.704389e-04) * q[i0] * q[i8] * q[i9] + (-3.394240e-04) * q[i0] * q[i8] * q[i10] + (-1.841148e-04) * q[i0] * q[i8] * q[i11]
            + (1.194779e-04) * q[i0] * q[i8] * q[i12] + (7.729900e-04) * q[i0] * q[i8] * q[i15] + (7.457658e-04) * q[i0] * q[i8] * q[i16]
            + (6.306952e-04) * q[i0] * q[i8] * q[i19] + (5.005176e-04) * q[i0] * q[i8] * q[i20] + (4.348939e-04) * q[i0] * q[i8] * q[i21]
            + (-1.241166e-04) * q[i0] * q[i8] * q[i22] + (4.473814e-04) * q[i0] * q[i9] * q[i10] + (-8.576594e-05) * q[i0] * q[i9] * q[i11]
            + (-3.243681e-04) * q[i0] * q[i9] * q[i12] + (1.634297e-04) * q[i0] * q[i9] * q[i15] + (2.365968e-05) * q[i0] * q[i9] * q[i16]
            + (6.612424e-04) * q[i0] * q[i9] * q[i19] + (2.772090e-04) * q[i0] * q[i9] * q[i20] + (-3.066195e-04) * q[i0] * q[i9] * q[i21]
            + (-1.234293e-04) * q[i0] * q[i9] * q[i22] + (-5.594774e-04) * q[i0] * q[i10] * q[i11] + (-7.310737e-05) * q[i0] * q[i10] * q[i12]
            + (3.888543e-04) * q[i0] * q[i10] * q[i15] + (-1.663085e-05) * q[i0] * q[i10] * q[i16] + (-1.775852e-04) * q[i0] * q[i10] * q[i19]
            + (-4.583215e-04) * q[i0] * q[i10] * q[i20] + (-5.004782e-04) * q[i0] * q[i10] * q[i21] + (-7.364061e-05) * q[i0] * q[i10] * q[i22]
            + (6.593555e-04) * q[i0] * q[i11] * q[i12] + (4.933105e-04) * q[i0] * q[i11] * q[i15] + (5.187873e-04) * q[i0] * q[i11] * q[i16]
            + (5.942319e-04) * q[i0] * q[i11] * q[i19] + (-5.535935e-05) * q[i0] * q[i11] * q[i20] + (4.090679e-04) * q[i0] * q[i11] * q[i21]
            + (-3.014434e-04) * q[i0] * q[i11] * q[i22] + (-2.343484e-06) * q[i0] * q[i12] * q[i15] + (-6.241420e-04) * q[i0] * q[i12] * q[i16]
            + (-1.363830e-04) * q[i0] * q[i12] * q[i19] + (-3.798038e-04) * q[i0] * q[i12] * q[i20] + (1.552164e-04) * q[i0] * q[i12] * q[i21]
            + (-3.450505e-04) * q[i0] * q[i12] * q[i22] + (5.167769e-04) * q[i0] * q[i15] * q[i16] + (3.401895e-05) * q[i0] * q[i15] * q[i19]
            + (8.032369e-06) * q[i0] * q[i15] * q[i20] + (2.456465e-04) * q[i0] * q[i15] * q[i21] + (3.832412e-04) * q[i0] * q[i15] * q[i22]
            + (-6.703630e-04) * q[i0] * q[i16] * q[i19] + (1.521333e-04) * q[i0] * q[i16] * q[i20] + (2.922396e-05) * q[i0] * q[i16] * q[i21]
            + (7.041708e-05) * q[i0] * q[i16] * q[i22] + (-3.255676e-05) * q[i0] * q[i19] * q[i20] + (-2.073554e-04) * q[i0] * q[i19] * q[i21]
            + (-1.515405e-04) * q[i0] * q[i19] * q[i22] + (5.198957e-04) * q[i0] * q[i20] * q[i21] + (5.406636e-05) * q[i0] * q[i20] * q[i22]
            + (8.145290e-05) * q[i0] * q[i21] * q[i22] + (-8.596076e-04) * q[i1] * q[i2] * q[i3] + (-1.326255e-02) * q[i1] * q[i2] * q[i4]
            + (-7.496941e-03) * q[i1] * q[i2] * q[i5] + (5.407253e-04) * q[i1] * q[i2] * q[i6] + (-1.096792e-03) * q[i1] * q[i2] * q[i7]
            + (3.407788e-03) * q[i1] * q[i2] * q[i8] + (-1.748730e-04) * q[i1] * q[i2] * q[i9] + (-6.674760e-04) * q[i1] * q[i2] * q[i10]
            + (1.645258e-03) * q[i1] * q[i2] * q[i11] + (4.574802e-04) * q[i1] * q[i2] * q[i12] + (-2.156562e-03) * q[i1] * q[i2] * q[i15]
            + (1.311596e-03) * q[i1] * q[i2] * q[i16] + (-9.215505e-04) * q[i1] * q[i2] * q[i19] + (-5.653053e-04) * q[i1] * q[i2] * q[i20]
            + (1.822513e-04) * q[i1] * q[i2] * q[i21] + (4.438889e-04) * q[i1] * q[i2] * q[i22] + (4.826837e-04) * q[i1] * q[i3] * q[i4]
            + (3.747658e-03) * q[i1] * q[i3] * q[i5] + (-9.422555e-04) * q[i1] * q[i3] * q[i6] + (2.198485e-03) * q[i1] * q[i3] * q[i7]
            + (2.014678e-03) * q[i1] * q[i3] * q[i8] + (-6.174670e-04) * q[i1] * q[i3] * q[i9] + (-5.566213e-04) * q[i1] * q[i3] * q[i10]
            + (-1.167348e-03) * q[i1] * q[i3] * q[i11] + (1.317098e-03) * q[i1] * q[i3] * q[i12] + (2.786214e-04) * q[i1] * q[i3] * q[i15]
            + (-2.264120e-04) * q[i1] * q[i3] * q[i16] + (2.977698e-04) * q[i1] * q[i3] * q[i19] + (8.919473e-05) * q[i1] * q[i3] * q[i20]
            + (-2.511312e-05) * q[i1] * q[i3] * q[i21] + (2.563931e-05) * q[i1] * q[i3] * q[i22] + (2.791861e-03) * q[i1] * q[i4] * q[i5]
            + (-3.108244e-03) * q[i1] * q[i4] * q[i6] + (1.007526e-02) * q[i1] * q[i4] * q[i7] + (-2.251224e-03) * q[i1] * q[i4] * q[i8]
            + (-2.275981e-03) * q[i1] * q[i4] * q[i9] + (6.026894e-04) * q[i1] * q[i4] * q[i10] + (-1.489637e-03) * q[i1] * q[i4] * q[i11]
            + (-8.532372e-04) * q[i1] * q[i4] * q[i12] + (-6.116358e-04) * q[i1] * q[i4] * q[i15] + (-2.220515e-03) * q[i1] * q[i4] * q[i16]
            + (-3.527346e-04) * q[i1] * q[i4] * q[i19] + (-9.849510e-04) * q[i1] * q[i4] * q[i20] + (-8.889815e-04) * q[i1] * q[i4] * q[i21]
            + (-3.475137e-04) * q[i1] * q[i4] * q[i22] + (2.446108e-04) * q[i1] * q[i5] * q[i6] + (6.748017e-03) * q[i1] * q[i5] * q[i7]
            + (-3.875300e-04) * q[i1] * q[i5] * q[i8] + (5.165634e-04) * q[i1] * q[i5] * q[i9] + (5.554145e-04) * q[i1] * q[i5] * q[i10]
            + (4.673067e-04) * q[i1] * q[i5] * q[i11] + (8.766858e-04) * q[i1] * q[i5] * q[i12] + (1.168227e-03) * q[i1] * q[i5] * q[i15]
            + (6.108258e-04) * q[i1] * q[i5] * q[i16] + (3.734309e-04) * q[i1] * q[i5] * q[i19] + (5.978117e-04) * q[i1] * q[i5] * q[i20]
            + (4.016762e-04) * q[i1] * q[i5] * q[i21] + (2.796101e-04) * q[i1] * q[i5] * q[i22] + (-1.795773e-03) * q[i1] * q[i6] * q[i7]
            + (3.638126e-04) * q[i1] * q[i6] * q[i8] + (1.300291e-03) * q[i1] * q[i6] * q[i9] + (3.132138e-04) * q[i1] * q[i6] * q[i10]
            + (9.677180e-04) * q[i1] * q[i6] * q[i11] + (1.764341e-03) * q[i1] * q[i6] * q[i12] + (1.927100e-04) * q[i1] * q[i6] * q[i15]
            + (-6.800327e-04) * q[i1] * q[i6] * q[i16] + (-1.865585e-04) * q[i1] * q[i6] * q[i19] + (-2.142370e-04) * q[i1] * q[i6] * q[i20]
            + (3.399222e-04) * q[i1] * q[i6] * q[i21] + (-2.211472e-05) * q[i1] * q[i6] * q[i22] + (7.035108e-03) * q[i1] * q[i7] * q[i8]
            + (2.754172e-03) * q[i1] * q[i7] * q[i9] + (-1.349584e-02) * q[i1] * q[i7] * q[i10] + (1.099982e-03) * q[i1] * q[i7] * q[i11]
            + (1.474185e-03) * q[i1] * q[i7] * q[i12] + (1.383560e-03) * q[i1] * q[i7] * q[i15] + (7.824298e-04) * q[i1] * q[i7] * q[i16]
            + (1.553686e-04) * q[i1] * q[i7] * q[i19] + (6.718480e-04) * q[i1] * q[i7] * q[i20] + (-8.756166e-04) * q[i1] * q[i7] * q[i21]
            + (7.638772e-04) * q[i1] * q[i7] * q[i22] + (-3.468415e-04) * q[i1] * q[i8] * q[i9] + (7.726777e-04) * q[i1] * q[i8] * q[i10]
            + (-7.515004e-05) * q[i1] * q[i8] * q[i11] + (2.155798e-04) * q[i1] * q[i8] * q[i12] + (7.261643e-04) * q[i1] * q[i8] * q[i15]
            + (7.741995e-04) * q[i1] * q[i8] * q[i16] + (5.053310e-04) * q[i1] * q[i8] * q[i19] + (6.299696e-04) * q[i1] * q[i8] * q[i20]
            + (1.145841e-04) * q[i1] * q[i8] * q[i21] + (-4.322135e-04) * q[i1] * q[i8] * q[i22] + (4.457035e-04) * q[i1] * q[i9] * q[i10]
            + (7.662144e-05) * q[i1] * q[i9] * q[i11] + (5.675472e-04) * q[i1] * q[i9] * q[i12] + (-2.443037e-05) * q[i1] * q[i9] * q[i15]
            + (3.857545e-04) * q[i1] * q[i9] * q[i16] + (-4.597970e-04) * q[i1] * q[i9] * q[i19] + (-1.791769e-04) * q[i1] * q[i9] * q[i20]
            + (7.667513e-05) * q[i1] * q[i9] * q[i21] + (5.041266e-04) * q[i1] * q[i9] * q[i22] + (3.202169e-04) * q[i1] * q[i10] * q[i11]
            + (8.348494e-05) * q[i1] * q[i10] * q[i12] + (1.561949e-05) * q[i1] * q[i10] * q[i15] + (1.650535e-04) * q[i1] * q[i10] * q[i16]
            + (2.768153e-04) * q[i1] * q[i10] * q[i19] + (6.625426e-04) * q[i1] * q[i10] * q[i20] + (1.233639e-04) * q[i1] * q[i10] * q[i21]
            + (2.979040e-04) * q[i1] * q[i10] * q[i22] + (6.634929e-04) * q[i1] * q[i11] * q[i12] + (6.052294e-04) * q[i1] * q[i11] * q[i15]
            + (8.497943e-07) * q[i1] * q[i11] * q[i16] + (3.752991e-04) * q[i1] * q[i11] * q[i19] + (1.283739e-04) * q[i1] * q[i11] * q[i20]
            + (-3.525242e-04) * q[i1] * q[i11] * q[i21] + (1.552551e-04) * q[i1] * q[i11] * q[i22] + (-5.293381e-04) * q[i1] * q[i12] * q[i15]
            + (-4.937368e-04) * q[i1] * q[i12] * q[i16] + (5.602229e-05) * q[i1] * q[i12] * q[i19] + (-5.984771e-04) * q[i1] * q[i12] * q[i20]
            + (-2.965218e-04) * q[i1] * q[i12] * q[i21] + (4.076207e-04) * q[i1] * q[i12] * q[i22] + (5.077336e-04) * q[i1] * q[i15] * q[i16]
            + (1.600298e-04) * q[i1] * q[i15] * q[i19] + (-6.659789e-04) * q[i1] * q[i15] * q[i20] + (-7.249146e-05) * q[i1] * q[i15] * q[i21]
            + (-3.137002e-05) * q[i1] * q[i15] * q[i22] + (1.494955e-05) * q[i1] * q[i16] * q[i19] + (3.872082e-05) * q[i1] * q[i16] * q[i20]
            + (-3.747243e-04) * q[i1] * q[i16] * q[i21] + (-2.484144e-04) * q[i1] * q[i16] * q[i22] + (-4.926675e-05) * q[i1] * q[i19] * q[i20]
            + (-5.847086e-05) * q[i1] * q[i19] * q[i21] + (-5.140003e-04) * q[i1] * q[i19] * q[i22] + (1.523064e-04) * q[i1] * q[i20] * q[i21]
            + (2.056473e-04) * q[i1] * q[i20] * q[i22] + (8.272157e-05) * q[i1] * q[i21] * q[i22] + (9.854905e-04) * q[i2] * q[i3] * q[i4]
            + (3.739732e-03) * q[i2] * q[i3] * q[i5] + (-2.893629e-03) * q[i2] * q[i3] * q[i6] + (4.047955e-04) * q[i2] * q[i3] * q[i7]
            + (2.254073e-03) * q[i2] * q[i3] * q[i8] + (8.912089e-05) * q[i2] * q[i3] * q[i9] + (1.396181e-04) * q[i2] * q[i3] * q[i10]
            + (-1.113976e-04) * q[i2] * q[i3] * q[i11] + (2.118474e-04) * q[i2] * q[i3] * q[i12] + (1.898353e-03) * q[i2] * q[i3] * q[i15]
            + (-4.379823e-04) * q[i2] * q[i3] * q[i16] + (-6.730639e-04) * q[i2] * q[i3] * q[i19] + (-1.564297e-04) * q[i2] * q[i3] * q[i20]
            + (-8.784506e-04) * q[i2] * q[i3] * q[i21] + (5.838644e-04) * q[i2] * q[i3] * q[i22] + (3.749981e-03) * q[i2] * q[i4] * q[i5]
            + (-3.893894e-04) * q[i2] * q[i4] * q[i6] + (2.878982e-03) * q[i2] * q[i4] * q[i7] + (-2.213145e-03) * q[i2] * q[i4] * q[i8]
            + (-1.504482e-04) * q[i2] * q[i4] * q[i9] + (-7.647034e-05) * q[i2] * q[i4] * q[i10] + (1.860122e-04) * q[i2] * q[i4] * q[i11]
            + (-8.613907e-05) * q[i2] * q[i4] * q[i12] + (4.359900e-04) * q[i2] * q[i4] * q[i15] + (-1.911338e-03) * q[i2] * q[i4] * q[i16]
            + (1.579609e-04) * q[i2] * q[i4] * q[i19] + (6.575876e-04) * q[i2] * q[i4] * q[i20] + (5.731572e-04) * q[i2] * q[i4] * q[i21]
            + (-8.803963e-04) * q[i2] * q[i4] * q[i22] + (8.400495e-04) * q[i2] * q[i5] * q[i6] + (-8.526404e-04) * q[i2] * q[i5] * q[i7]
            + (-4.747920e-05) * q[i2] * q[i5] * q[i8] + (1.378678e-03) * q[i2] * q[i5] * q[i9] + (-1.394716e-03) * q[i2] * q[i5] * q[i10]
            + (1.879672e-03) * q[i2] * q[i5] * q[i11] + (1.853627e-03) * q[i2] * q[i5] * q[i12] + (-2.309920e-03) * q[i2] * q[i5] * q[i15]
            + (2.307184e-03) * q[i2] * q[i5] * q[i16] + (1.426304e-03) * q[i2] * q[i5] * q[i19] + (-1.426429e-03) * q[i2] * q[i5] * q[i20]
            + (8.077700e-04) * q[i2] * q[i5] * q[i21] + (7.973751e-04) * q[i2] * q[i5] * q[i22] + (3.171142e-04) * q[i2] * q[i6] * q[i7]
            + (6.081805e-03) * q[i2] * q[i6] * q[i8] + (-3.521530e-03) * q[i2] * q[i6] * q[i9] + (1.440729e-03) * q[i2] * q[i6] * q[i10]
            + (-8.145888e-04) * q[i2] * q[i6] * q[i11] + (-1.063741e-04) * q[i2] * q[i6] * q[i12] + (6.365126e-04) * q[i2] * q[i6] * q[i15]
            + (-4.380484e-04) * q[i2] * q[i6] * q[i16] + (1.961629e-04) * q[i2] * q[i6] * q[i19] + (3.576040e-05) * q[i2] * q[i6] * q[i20]
            + (3.222426e-04) * q[i2] * q[i6] * q[i21] + (2.455748e-04) * q[i2] * q[i6] * q[i22] + (6.089197e-03) * q[i2] * q[i7] * q[i8]
            + (1.449597e-03) * q[i2] * q[i7] * q[i9] + (-3.469230e-03) * q[i2] * q[i7] * q[i10] + (1.189835e-04) * q[i2] * q[i7] * q[i11]
            + (8.198402e-04) * q[i2] * q[i7] * q[i12] + (-4.190663e-04) * q[i2] * q[i7] * q[i15] + (6.364879e-04) * q[i2] * q[i7] * q[i16]
            + (3.483468e-05) * q[i2] * q[i7] * q[i19] + (2.011984e-04) * q[i2] * q[i7] * q[i20] + (-2.421668e-04) * q[i2] * q[i7] * q[i21]
            + (-3.191185e-04) * q[i2] * q[i7] * q[i22] + (-2.776862e-03) * q[i2] * q[i8] * q[i9] + (-2.736449e-03) * q[i2] * q[i8] * q[i10]
            + (-6.921279e-04) * q[i2] * q[i8] * q[i11] + (8.612909e-04) * q[i2] * q[i8] * q[i12] + (5.179766e-03) * q[i2] * q[i8] * q[i15]
            + (5.254888e-03) * q[i2] * q[i8] * q[i16] + (4.322037e-04) * q[i2] * q[i8] * q[i19] + (4.214895e-04) * q[i2] * q[i8] * q[i20]
            + (-3.998145e-04) * q[i2] * q[i8] * q[i21] + (4.081732e-04) * q[i2] * q[i8] * q[i22] + (-1.672587e-05) * q[i2] * q[i9] * q[i10]
            + (-5.373416e-04) * q[i2] * q[i9] * q[i11] + (3.698614e-06) * q[i2] * q[i9] * q[i12] + (1.200645e-04) * q[i2] * q[i9] * q[i15]
            + (-2.390041e-04) * q[i2] * q[i9] * q[i16] + (1.394536e-04) * q[i2] * q[i9] * q[i19] + (2.306254e-04) * q[i2] * q[i9] * q[i20]
            + (-2.424145e-04) * q[i2] * q[i9] * q[i21] + (7.810359e-05) * q[i2] * q[i9] * q[i22] + (-2.125501e-05) * q[i2] * q[i10] * q[i11]
            + (5.232176e-04) * q[i2] * q[i10] * q[i12] + (-2.334638e-04) * q[i2] * q[i10] * q[i15] + (1.254069e-04) * q[i2] * q[i10] * q[i16]
            + (2.362313e-04) * q[i2] * q[i10] * q[i19] + (1.414497e-04) * q[i2] * q[i10] * q[i20] + (-7.869362e-05) * q[i2] * q[i10] * q[i21]
            + (2.386402e-04) * q[i2] * q[i10] * q[i22] + (6.074138e-04) * q[i2] * q[i11] * q[i12] + (3.323139e-03) * q[i2] * q[i11] * q[i15]
            + (-4.022756e-04) * q[i2] * q[i11] * q[i16] + (1.742883e-03) * q[i2] * q[i11] * q[i19] + (5.947629e-05) * q[i2] * q[i11] * q[i20]
            + (2.158425e-04) * q[i2] * q[i11] * q[i21] + (-4.507722e-04) * q[i2] * q[i11] * q[i22] + (3.809445e-04) * q[i2] * q[i12] * q[i15]
            + (-3.340560e-03) * q[i2] * q[i12] * q[i16] + (-7.327170e-05) * q[i2] * q[i12] * q[i19] + (-1.726330e-03) * q[i2] * q[i12] * q[i20]
            + (-4.512358e-04) * q[i2] * q[i12] * q[i21] + (2.083152e-04) * q[i2] * q[i12] * q[i22] + (5.847767e-05) * q[i2] * q[i15] * q[i16]
            + (-9.102059e-04) * q[i2] * q[i15] * q[i19] + (-7.358046e-04) * q[i2] * q[i15] * q[i20] + (1.149993e-04) * q[i2] * q[i15] * q[i21]
            + (2.249188e-04) * q[i2] * q[i15] * q[i22] + (-7.419841e-04) * q[i2] * q[i16] * q[i19] + (-9.052824e-04) * q[i2] * q[i16] * q[i20]
            + (-2.212101e-04) * q[i2] * q[i16] * q[i21] + (-1.195472e-04) * q[i2] * q[i16] * q[i22] + (1.839152e-04) * q[i2] * q[i19] * q[i20]
            + (-6.358527e-04) * q[i2] * q[i19] * q[i21] + (-3.780284e-04) * q[i2] * q[i19] * q[i22] + (3.801782e-04) * q[i2] * q[i20] * q[i21]
            + (6.258241e-04) * q[i2] * q[i20] * q[i22] + (8.527480e-05) * q[i2] * q[i21] * q[i22] + (-1.760935e-04) * q[i3] * q[i4] * q[i5]
            + (1.609779e-03) * q[i3] * q[i4] * q[i6] + (-1.611773e-03) * q[i3] * q[i4] * q[i7] + (5.305136e-06) * q[i3] * q[i4] * q[i8]
            + (-9.559773e-04) * q[i3] * q[i4] * q[i9] + (9.555538e-04) * q[i3] * q[i4] * q[i10] + (1.488707e-03) * q[i3] * q[i4] * q[i11]
            + (1.482513e-03) * q[i3] * q[i4] * q[i12] + (-2.339222e-05) * q[i3] * q[i4] * q[i15] + (3.199603e-05) * q[i3] * q[i4] * q[i16]
            + (-1.143179e-03) * q[i3] * q[i4] * q[i19] + (1.148663e-03) * q[i3] * q[i4] * q[i20] + (-5.856657e-04) * q[i3] * q[i4] * q[i21]
            + (-5.908238e-04) * q[i3] * q[i4] * q[i22] + (-3.556932e-03) * q[i3] * q[i5] * q[i6] + (-5.519686e-04) * q[i3] * q[i5] * q[i7]
            + (3.825928e-04) * q[i3] * q[i5] * q[i8] + (6.208945e-04) * q[i3] * q[i5] * q[i9] + (-9.887132e-04) * q[i3] * q[i5] * q[i10]
            + (-2.169193e-03) * q[i3] * q[i5] * q[i11] + (5.292480e-04) * q[i3] * q[i5] * q[i12] + (1.139528e-03) * q[i3] * q[i5] * q[i15]
            + (3.817097e-04) * q[i3] * q[i5] * q[i16] + (6.850017e-06) * q[i3] * q[i5] * q[i19] + (-1.682617e-03) * q[i3] * q[i5] * q[i20]
            + (9.284001e-05) * q[i3] * q[i5] * q[i21] + (2.950487e-04) * q[i3] * q[i5] * q[i22] + (7.620428e-05) * q[i3] * q[i6] * q[i7]
            + (-1.833501e-03) * q[i3] * q[i6] * q[i8] + (-1.084537e-02) * q[i3] * q[i6] * q[i9] + (5.088915e-04) * q[i3] * q[i6] * q[i10]
            + (-5.300235e-04) * q[i3] * q[i6] * q[i11] + (1.113857e-03) * q[i3] * q[i6] * q[i12] + (8.047113e-04) * q[i3] * q[i6] * q[i15]
            + (-7.632352e-04) * q[i3] * q[i6] * q[i16] + (-1.925811e-03) * q[i3] * q[i6] * q[i19] + (4.617432e-04) * q[i3] * q[i6] * q[i20]
            + (-3.234240e-04) * q[i3] * q[i6] * q[i21] + (9.169762e-04) * q[i3] * q[i6] * q[i22] + (-1.648222e-03) * q[i3] * q[i7] * q[i8]
            + (4.427378e-04) * q[i3] * q[i7] * q[i9] + (2.244056e-03) * q[i3] * q[i7] * q[i10] + (3.831200e-04) * q[i3] * q[i7] * q[i11]
            + (5.663898e-04) * q[i3] * q[i7] * q[i12] + (-1.599024e-03) * q[i3] * q[i7] * q[i15] + (-9.085843e-04) * q[i3] * q[i7] * q[i16]
            + (-1.873487e-04) * q[i3] * q[i7] * q[i19] + (4.093100e-04) * q[i3] * q[i7] * q[i20] + (6.429751e-04) * q[i3] * q[i7] * q[i21]
            + (4.012098e-04) * q[i3] * q[i7] * q[i22] + (-7.396036e-04) * q[i3] * q[i8] * q[i9] + (1.941091e-04) * q[i3] * q[i8] * q[i10]
            + (-1.732590e-03) * q[i3] * q[i8] * q[i11] + (6.654939e-04) * q[i3] * q[i8] * q[i12] + (-6.313806e-04) * q[i3] * q[i8] * q[i15]
            + (-7.923113e-05) * q[i3] * q[i8] * q[i16] + (-4.347447e-05) * q[i3] * q[i8] * q[i19] + (-6.697361e-05) * q[i3] * q[i8] * q[i20]
            + (6.391445e-04) * q[i3] * q[i8] * q[i21] + (-4.429403e-04) * q[i3] * q[i8] * q[i22] + (-4.205340e-04) * q[i3] * q[i9] * q[i10]
            + (-2.376146e-04) * q[i3] * q[i9] * q[i11] + (3.003056e-04) * q[i3] * q[i9] * q[i12] + (-2.719674e-04) * q[i3] * q[i9] * q[i15]
            + (-5.925024e-04) * q[i3] * q[i9] * q[i16] + (-3.112606e-04) * q[i3] * q[i9] * q[i19] + (-1.554147e-04) * q[i3] * q[i9] * q[i20]
            + (1.320513e-04) * q[i3] * q[i9] * q[i21] + (-3.187391e-04) * q[i3] * q[i9] * q[i22] + (2.053841e-04) * q[i3] * q[i10] * q[i11]
            + (4.718133e-04) * q[i3] * q[i10] * q[i12] + (-3.623644e-04) * q[i3] * q[i10] * q[i15] + (3.846383e-04) * q[i3] * q[i10] * q[i16]
            + (2.522698e-04) * q[i3] * q[i10] * q[i19] + (3.036936e-05) * q[i3] * q[i10] * q[i20] + (-1.085302e-04) * q[i3] * q[i10] * q[i21]
            + (1.945716e-04) * q[i3] * q[i10] * q[i22] + (-1.561455e-04) * q[i3] * q[i11] * q[i12] + (-1.538416e-03) * q[i3] * q[i11] * q[i15]
            + (7.868216e-04) * q[i3] * q[i11] * q[i16] + (4.310239e-05) * q[i3] * q[i11] * q[i19] + (3.062487e-05) * q[i3] * q[i11] * q[i20]
            + (-4.160914e-04) * q[i3] * q[i11] * q[i21] + (-2.337426e-04) * q[i3] * q[i11] * q[i22] + (5.428375e-04) * q[i3] * q[i12] * q[i15]
            + (-5.822735e-05) * q[i3] * q[i12] * q[i16] + (2.744032e-05) * q[i3] * q[i12] * q[i19] + (4.514248e-04) * q[i3] * q[i12] * q[i20]
            + (1.865373e-04) * q[i3] * q[i12] * q[i21] + (-2.017979e-04) * q[i3] * q[i12] * q[i22] + (3.354747e-04) * q[i3] * q[i15] * q[i16]
            + (-6.221902e-04) * q[i3] * q[i15] * q[i19] + (1.336437e-04) * q[i3] * q[i15] * q[i20] + (1.475200e-04) * q[i3] * q[i15] * q[i21]
            + (2.283656e-04) * q[i3] * q[i15] * q[i22] + (3.829431e-04) * q[i3] * q[i16] * q[i19] + (3.285342e-04) * q[i3] * q[i16] * q[i20]
            + (-8.378908e-05) * q[i3] * q[i16] * q[i21] + (-4.900784e-04) * q[i3] * q[i16] * q[i22] + (4.188005e-04) * q[i3] * q[i19] * q[i20]
            + (5.394042e-04) * q[i3] * q[i19] * q[i21] + (-6.136038e-04) * q[i3] * q[i19] * q[i22] + (-3.209858e-04) * q[i3] * q[i20] * q[i21]
            + (-3.933443e-04) * q[i3] * q[i20] * q[i22] + (8.998727e-05) * q[i3] * q[i21] * q[i22] + (5.506200e-04) * q[i4] * q[i5] * q[i6]
            + (3.544684e-03) * q[i4] * q[i5] * q[i7] + (-3.975660e-04) * q[i4] * q[i5] * q[i8] + (9.979039e-04) * q[i4] * q[i5] * q[i9]
            + (-6.191354e-04) * q[i4] * q[i5] * q[i10] + (5.179589e-04) * q[i4] * q[i5] * q[i11] + (-2.150434e-03) * q[i4] * q[i5] * q[i12]
            + (-3.856767e-04) * q[i4] * q[i5] * q[i15] + (-1.157941e-03) * q[i4] * q[i5] * q[i16] + (1.691194e-03) * q[i4] * q[i5] * q[i19]
            + (-1.832415e-06) * q[i4] * q[i5] * q[i20] + (2.917031e-04) * q[i4] * q[i5] * q[i21] + (9.782708e-05) * q[i4] * q[i5] * q[i22]
            + (7.334453e-05) * q[i4] * q[i6] * q[i7] + (-1.661487e-03) * q[i4] * q[i6] * q[i8] + (2.256587e-03) * q[i4] * q[i6] * q[i9]
            + (4.444126e-04) * q[i4] * q[i6] * q[i10] + (-5.527570e-04) * q[i4] * q[i6] * q[i11] + (-4.070025e-04) * q[i4] * q[i6] * q[i12]
            + (-9.036454e-04) * q[i4] * q[i6] * q[i15] + (-1.607747e-03) * q[i4] * q[i6] * q[i16] + (4.110395e-04) * q[i4] * q[i6] * q[i19]
            + (-1.805649e-04) * q[i4] * q[i6] * q[i20] + (-3.987744e-04) * q[i4] * q[i6] * q[i21] + (-6.479182e-04) * q[i4] * q[i6] * q[i22]
            + (-1.843450e-03) * q[i4] * q[i7] * q[i8] + (5.375296e-04) * q[i4] * q[i7] * q[i9] + (-1.075749e-02) * q[i4] * q[i7] * q[i10]
            + (-1.102490e-03) * q[i4] * q[i7] * q[i11] + (5.165377e-04) * q[i4] * q[i7] * q[i12] + (-7.693415e-04) * q[i4] * q[i7] * q[i15]
            + (8.235890e-04) * q[i4] * q[i7] * q[i16] + (4.637157e-04) * q[i4] * q[i7] * q[i19] + (-1.908558e-03) * q[i4] * q[i7] * q[i20]
            + (-9.108623e-04) * q[i4] * q[i7] * q[i21] + (3.232710e-04) * q[i4] * q[i7] * q[i22] + (1.938854e-04) * q[i4] * q[i8] * q[i9]
            + (-7.384261e-04) * q[i4] * q[i8] * q[i10] + (-6.265955e-04) * q[i4] * q[i8] * q[i11] + (1.743262e-03) * q[i4] * q[i8] * q[i12]
            + (-6.749888e-05) * q[i4] * q[i8] * q[i15] + (-6.186600e-04) * q[i4] * q[i8] * q[i16] + (-7.238836e-05) * q[i4] * q[i8] * q[i19]
            + (-3.067289e-05) * q[i4] * q[i8] * q[i20] + (4.369758e-04) * q[i4] * q[i8] * q[i21] + (-6.370546e-04) * q[i4] * q[i8] * q[i22]
            + (-4.183442e-04) * q[i4] * q[i9] * q[i10] + (-4.617177e-04) * q[i4] * q[i9] * q[i11] + (-2.095858e-04) * q[i4] * q[i9] * q[i12]
            + (3.734594e-04) * q[i4] * q[i9] * q[i15] + (-3.542908e-04) * q[i4] * q[i9] * q[i16] + (2.444100e-05) * q[i4] * q[i9] * q[i19]
            + (2.528863e-04) * q[i4] * q[i9] * q[i20] + (-1.937380e-04) * q[i4] * q[i9] * q[i21] + (1.095770e-04) * q[i4] * q[i9] * q[i22]
            + (-3.095683e-04) * q[i4] * q[i10] * q[i11] + (2.313153e-04) * q[i4] * q[i10] * q[i12] + (-5.753068e-04) * q[i4] * q[i10] * q[i15]
            + (-2.718006e-04) * q[i4] * q[i10] * q[i16] + (-1.542513e-04) * q[i4] * q[i10] * q[i19] + (-3.165382e-04) * q[i4] * q[i10] * q[i20]
            + (3.152262e-04) * q[i4] * q[i10] * q[i21] + (-1.301767e-04) * q[i4] * q[i10] * q[i22] + (-1.573023e-04) * q[i4] * q[i11] * q[i12]
            + (5.701258e-05) * q[i4] * q[i11] * q[i15] + (-5.416789e-04) * q[i4] * q[i11] * q[i16] + (-4.461548e-04) * q[i4] * q[i11] * q[i19]
            + (-2.933765e-05) * q[i4] * q[i11] * q[i20] + (-1.990022e-04) * q[i4] * q[i11] * q[i21] + (1.956478e-04) * q[i4] * q[i11] * q[i22]
            + (-7.825143e-04) * q[i4] * q[i12] * q[i15] + (1.539207e-03) * q[i4] * q[i12] * q[i16] + (-3.089071e-05) * q[i4] * q[i12] * q[i19]
            + (-5.005504e-05) * q[i4] * q[i12] * q[i20] + (-2.380424e-04) * q[i4] * q[i12] * q[i21] + (-4.228184e-04) * q[i4] * q[i12] * q[i22]
            + (3.344007e-04) * q[i4] * q[i15] * q[i16] + (3.299048e-04) * q[i4] * q[i15] * q[i19] + (3.810390e-04) * q[i4] * q[i15] * q[i20]
            + (4.816924e-04) * q[i4] * q[i15] * q[i21] + (7.991379e-05) * q[i4] * q[i15] * q[i22] + (1.239034e-04) * q[i4] * q[i16] * q[i19]
            + (-6.142677e-04) * q[i4] * q[i16] * q[i20] + (-2.286239e-04) * q[i4] * q[i16] * q[i21] + (-1.533807e-04) * q[i4] * q[i16] * q[i22]
            + (4.128263e-04) * q[i4] * q[i19] * q[i20] + (3.958918e-04) * q[i4] * q[i19] * q[i21] + (3.210236e-04) * q[i4] * q[i19] * q[i22]
            + (6.100004e-04) * q[i4] * q[i20] * q[i21] + (-5.392119e-04) * q[i4] * q[i20] * q[i22] + (8.042876e-05) * q[i4] * q[i21] * q[i22]
            + (8.525738e-04) * q[i5] * q[i6] * q[i7] + (3.660053e-04) * q[i5] * q[i6] * q[i8] + (2.230545e-03) * q[i5] * q[i6] * q[i9]
            + (-1.157632e-03) * q[i5] * q[i6] * q[i10] + (-2.735815e-04) * q[i5] * q[i6] * q[i11] + (-6.490981e-05) * q[i5] * q[i6] * q[i12]
            + (7.044322e-04) * q[i5] * q[i6] * q[i15] + (4.955465e-04) * q[i5] * q[i6] * q[i16] + (-2.268588e-04) * q[i5] * q[i6] * q[i19]
            + (7.898259e-04) * q[i5] * q[i6] * q[i20] + (1.469943e-04) * q[i5] * q[i6] * q[i21] + (-9.274461e-05) * q[i5] * q[i6] * q[i22]
            + (3.475794e-04) * q[i5] * q[i7] * q[i8] + (-1.172213e-03) * q[i5] * q[i7] * q[i9] + (2.225263e-03) * q[i5] * q[i7] * q[i10]
            + (4.523701e-05) * q[i5] * q[i7] * q[i11] + (2.827897e-04) * q[i5] * q[i7] * q[i12] + (4.890998e-04) * q[i5] * q[i7] * q[i15]
            + (6.914289e-04) * q[i5] * q[i7] * q[i16] + (7.971804e-04) * q[i5] * q[i7] * q[i19] + (-2.328081e-04) * q[i5] * q[i7] * q[i20]
            + (1.016375e-04) * q[i5] * q[i7] * q[i21] + (-1.583401e-04) * q[i5] * q[i7] * q[i22] + (-1.557785e-03) * q[i5] * q[i8] * q[i9]
            + (-1.550726e-03) * q[i5] * q[i8] * q[i10] + (2.422300e-03) * q[i5] * q[i8] * q[i11] + (-2.488921e-03) * q[i5] * q[i8] * q[i12]
            + (2.049749e-03) * q[i5] * q[i8] * q[i15] + (2.082281e-03) * q[i5] * q[i8] * q[i16] + (-4.989687e-05) * q[i5] * q[i8] * q[i19]
            + (-6.700013e-05) * q[i5] * q[i8] * q[i20] + (3.568588e-04) * q[i5] * q[i8] * q[i21] + (-3.555156e-04) * q[i5] * q[i8] * q[i22]
            + (-3.448780e-04) * q[i5] * q[i9] * q[i10] + (-1.836317e-04) * q[i5] * q[i9] * q[i11] + (-2.499222e-04) * q[i5] * q[i9] * q[i12]
            + (-1.230037e-04) * q[i5] * q[i9] * q[i15] + (-2.549192e-04) * q[i5] * q[i9] * q[i16] + (3.467943e-04) * q[i5] * q[i9] * q[i19]
            + (-2.783076e-04) * q[i5] * q[i9] * q[i20] + (3.350566e-05) * q[i5] * q[i9] * q[i21] + (5.021899e-05) * q[i5] * q[i9] * q[i22]
            + (2.410171e-04) * q[i5] * q[i10] * q[i11] + (1.793394e-04) * q[i5] * q[i10] * q[i12] + (-2.451128e-04) * q[i5] * q[i10] * q[i15]
            + (-1.284285e-04) * q[i5] * q[i10] * q[i16] + (-2.752782e-04) * q[i5] * q[i10] * q[i19] + (3.378551e-04) * q[i5] * q[i10] * q[i20]
            + (-5.018386e-05) * q[i5] * q[i10] * q[i21] + (-4.503147e-05) * q[i5] * q[i10] * q[i22] + (-6.758906e-05) * q[i5] * q[i11] * q[i12]
            + (2.045775e-03) * q[i5] * q[i11] * q[i15] + (-6.262812e-04) * q[i5] * q[i11] * q[i16] + (1.178573e-03) * q[i5] * q[i11] * q[i19]
            + (-4.248532e-04) * q[i5] * q[i11] * q[i20] + (1.428402e-04) * q[i5] * q[i11] * q[i21] + (-6.106314e-04) * q[i5] * q[i11] * q[i22]
            + (6.146331e-04) * q[i5] * q[i12] * q[i15] + (-2.053211e-03) * q[i5] * q[i12] * q[i16] + (4.292040e-04) * q[i5] * q[i12] * q[i19]
            + (-1.178317e-03) * q[i5] * q[i12] * q[i20] + (-6.021615e-04) * q[i5] * q[i12] * q[i21] + (1.624863e-04) * q[i5] * q[i12] * q[i22]
            + (-1.212710e-05) * q[i5] * q[i15] * q[i16] + (-3.948275e-04) * q[i5] * q[i15] * q[i19] + (-3.213971e-04) * q[i5] * q[i15] * q[i20]
            + (-1.647586e-03) * q[i5] * q[i15] * q[i21] + (-3.132046e-04) * q[i5] * q[i15] * q[i22] + (-3.121259e-04) * q[i5] * q[i16] * q[i19]
            + (-3.933553e-04) * q[i5] * q[i16] * q[i20] + (3.135076e-04) * q[i5] * q[i16] * q[i21] + (1.668104e-03) * q[i5] * q[i16] * q[i22]
            + (-1.699453e-04) * q[i5] * q[i19] * q[i20] + (-1.532837e-03) * q[i5] * q[i19] * q[i21] + (3.425438e-04) * q[i5] * q[i19] * q[i22]
            + (-3.402388e-04) * q[i5] * q[i20] * q[i21] + (1.535302e-03) * q[i5] * q[i20] * q[i22] + (-4.511650e-04) * q[i5] * q[i21] * q[i22]
            + (9.935497e-06) * q[i6] * q[i7] * q[i8] + (1.051171e-03) * q[i6] * q[i7] * q[i9] + (-1.026063e-03) * q[i6] * q[i7] * q[i10]
            + (8.457240e-04) * q[i6] * q[i7] * q[i11] + (8.422181e-04) * q[i6] * q[i7] * q[i12] + (1.090265e-04) * q[i6] * q[i7] * q[i15]
            + (-1.037743e-04) * q[i6] * q[i7] * q[i16] + (1.300454e-04) * q[i6] * q[i7] * q[i19] + (-1.368505e-04) * q[i6] * q[i7] * q[i20]
            + (-8.415065e-04) * q[i6] * q[i7] * q[i21] + (-8.425326e-04) * q[i6] * q[i7] * q[i22] + (7.096309e-04) * q[i6] * q[i8] * q[i9]
            + (5.071955e-04) * q[i6] * q[i8] * q[i10] + (-9.533636e-04) * q[i6] * q[i8] * q[i11] + (6.504106e-04) * q[i6] * q[i8] * q[i12]
            + (-2.518575e-04) * q[i6] * q[i8] * q[i15] + (5.991215e-04) * q[i6] * q[i8] * q[i16] + (-1.606921e-04) * q[i6] * q[i8] * q[i19]
            + (2.586692e-04) * q[i6] * q[i8] * q[i20] + (2.789672e-04) * q[i6] * q[i8] * q[i21] + (1.065966e-04) * q[i6] * q[i8] * q[i22]
            + (4.232231e-04) * q[i6] * q[i9] * q[i10] + (-6.709169e-04) * q[i6] * q[i9] * q[i11] + (-4.706229e-04) * q[i6] * q[i9] * q[i12]
            + (-9.224733e-05) * q[i6] * q[i9] * q[i15] + (-9.940779e-05) * q[i6] * q[i9] * q[i16] + (-4.476959e-04) * q[i6] * q[i9] * q[i19]
            + (1.079777e-04) * q[i6] * q[i9] * q[i20] + (-1.558414e-04) * q[i6] * q[i9] * q[i21] + (-1.983248e-04) * q[i6] * q[i9] * q[i22]
            + (1.559962e-04) * q[i6] * q[i10] * q[i11] + (1.286611e-05) * q[i6] * q[i10] * q[i12] + (-3.926582e-04) * q[i6] * q[i10] * q[i15]
            + (-1.307033e-05) * q[i6] * q[i10] * q[i16] + (7.761660e-05) * q[i6] * q[i10] * q[i19] + (-3.088570e-04) * q[i6] * q[i10] * q[i20]
            + (4.445187e-04) * q[i6] * q[i10] * q[i21] + (1.511723e-04) * q[i6] * q[i10] * q[i22] + (6.787517e-06) * q[i6] * q[i11] * q[i12]
            + (1.036661e-03) * q[i6] * q[i11] * q[i15] + (-6.698623e-05) * q[i6] * q[i11] * q[i16] + (-7.845989e-05) * q[i6] * q[i11] * q[i19]
            + (4.371767e-04) * q[i6] * q[i11] * q[i20] + (3.351916e-04) * q[i6] * q[i11] * q[i21] + (1.009963e-04) * q[i6] * q[i11] * q[i22]
            + (6.488194e-04) * q[i6] * q[i12] * q[i15] + (6.449914e-04) * q[i6] * q[i12] * q[i16] + (-1.415264e-04) * q[i6] * q[i12] * q[i19]
            + (2.387709e-04) * q[i6] * q[i12] * q[i20] + (-2.481534e-04) * q[i6] * q[i12] * q[i21] + (-4.363672e-04) * q[i6] * q[i12] * q[i22]
            + (-3.029794e-04) * q[i6] * q[i15] * q[i16] + (3.436680e-04) * q[i6] * q[i15] * q[i19] + (-5.554971e-05) * q[i6] * q[i15] * q[i20]
            + (-2.115197e-04) * q[i6] * q[i15] * q[i21] + (1.826301e-04) * q[i6] * q[i15] * q[i22] + (-4.558552e-04) * q[i6] * q[i16] * q[i19]
            + (-1.235554e-04) * q[i6] * q[i16] * q[i20] + (-4.327594e-04) * q[i6] * q[i16] * q[i21] + (3.514698e-04) * q[i6] * q[i16] * q[i22]
            + (-9.645676e-06) * q[i6] * q[i19] * q[i20] + (-3.073957e-04) * q[i6] * q[i19] * q[i21] + (-7.130947e-05) * q[i6] * q[i19] * q[i22]
            + (2.802752e-04) * q[i6] * q[i20] * q[i21] + (-2.261390e-04) * q[i6] * q[i20] * q[i22] + (1.855035e-04) * q[i6] * q[i21] * q[i22]
            + (-5.016305e-04) * q[i7] * q[i8] * q[i9] + (-6.984106e-04) * q[i7] * q[i8] * q[i10] + (6.851689e-04) * q[i7] * q[i8] * q[i11]
            + (-9.460906e-04) * q[i7] * q[i8] * q[i12] + (-5.965860e-04) * q[i7] * q[i8] * q[i15] + (2.638945e-04) * q[i7] * q[i8] * q[i16]
            + (-2.536953e-04) * q[i7] * q[i8] * q[i19] + (1.514588e-04) * q[i7] * q[i8] * q[i20] + (1.148712e-04) * q[i7] * q[i8] * q[i21]
            + (2.757292e-04) * q[i7] * q[i8] * q[i22] + (-4.224340e-04) * q[i7] * q[i9] * q[i10] + (1.739491e-05) * q[i7] * q[i9] * q[i11]
            + (1.543717e-04) * q[i7] * q[i9] * q[i12] + (1.165673e-05) * q[i7] * q[i9] * q[i15] + (3.950348e-04) * q[i7] * q[i9] * q[i16]
            + (3.069477e-04) * q[i7] * q[i9] * q[i19] + (-7.608729e-05) * q[i7] * q[i9] * q[i20] + (1.492711e-04) * q[i7] * q[i9] * q[i21]
            + (4.424179e-04) * q[i7] * q[i9] * q[i22] + (-4.669284e-04) * q[i7] * q[i10] * q[i11] + (-6.696082e-04) * q[i7] * q[i10] * q[i12]
            + (9.726486e-05) * q[i7] * q[i10] * q[i15] + (1.002896e-04) * q[i7] * q[i10] * q[i16] + (-1.083990e-04) * q[i7] * q[i10] * q[i19]
            + (4.362536e-04) * q[i7] * q[i10] * q[i20] + (-2.008247e-04) * q[i7] * q[i10] * q[i21] + (-1.599506e-04) * q[i7] * q[i10] * q[i22]
            + (-5.702689e-06) * q[i7] * q[i11] * q[i12] + (6.396486e-04) * q[i7] * q[i11] * q[i15] + (6.519627e-04) * q[i7] * q[i11] * q[i16]
            + (2.386295e-04) * q[i7] * q[i11] * q[i19] + (-1.468150e-04) * q[i7] * q[i11] * q[i20] + (4.271983e-04) * q[i7] * q[i11] * q[i21]
            + (2.477358e-04) * q[i7] * q[i11] * q[i22] + (-6.701922e-05) * q[i7] * q[i12] * q[i15] + (1.026269e-03) * q[i7] * q[i12] * q[i16]
            + (4.347151e-04) * q[i7] * q[i12] * q[i19] + (-7.737333e-05) * q[i7] * q[i12] * q[i20] + (-1.026845e-04) * q[i7] * q[i12] * q[i21]
            + (-3.307243e-04) * q[i7] * q[i12] * q[i22] + (3.025784e-04) * q[i7] * q[i15] * q[i16] + (1.200177e-04) * q[i7] * q[i15] * q[i19]
            + (4.572318e-04) * q[i7] * q[i15] * q[i20] + (3.478372e-04) * q[i7] * q[i15] * q[i21] + (-4.318086e-04) * q[i7] * q[i15] * q[i22]
            + (5.480453e-05) * q[i7] * q[i16] * q[i19] + (-3.419594e-04) * q[i7] * q[i16] * q[i20] + (1.779659e-04) * q[i7] * q[i16] * q[i21]
            + (-2.200355e-04) * q[i7] * q[i16] * q[i22] + (6.585923e-06) * q[i7] * q[i19] * q[i20] + (-2.169687e-04) * q[i7] * q[i19] * q[i21]
            + (2.842097e-04) * q[i7] * q[i19] * q[i22] + (-7.294358e-05) * q[i7] * q[i20] * q[i21] + (-3.075305e-04) * q[i7] * q[i20] * q[i22]
            + (-1.864748e-04) * q[i7] * q[i21] * q[i22] + (1.159071e-06) * q[i8] * q[i9] * q[i10] + (3.203332e-04) * q[i8] * q[i9] * q[i11]
            + (1.751888e-03) * q[i8] * q[i9] * q[i12] + (-5.982581e-04) * q[i8] * q[i9] * q[i15] + (2.623590e-04) * q[i8] * q[i9] * q[i16]
            + (-3.346973e-04) * q[i8] * q[i9] * q[i19] + (1.654978e-04) * q[i8] * q[i9] * q[i20] + (3.364259e-04) * q[i8] * q[i9] * q[i21]
            + (-5.882717e-04) * q[i8] * q[i9] * q[i22] + (1.736864e-03) * q[i8] * q[i10] * q[i11] + (3.206554e-04) * q[i8] * q[i10] * q[i12]
            + (-2.604991e-04) * q[i8] * q[i10] * q[i15] + (5.953452e-04) * q[i8] * q[i10] * q[i16] + (-1.589789e-04) * q[i8] * q[i10] * q[i19]
            + (3.364738e-04) * q[i8] * q[i10] * q[i20] + (-5.852228e-04) * q[i8] * q[i10] * q[i21] + (3.368740e-04) * q[i8] * q[i10] * q[i22]
            + (-1.723473e-06) * q[i8] * q[i11] * q[i12] + (3.377785e-03) * q[i8] * q[i11] * q[i15] + (3.779367e-04) * q[i8] * q[i11] * q[i16]
            + (2.604282e-04) * q[i8] * q[i11] * q[i19] + (-1.029660e-04) * q[i8] * q[i11] * q[i20] + (1.627077e-03) * q[i8] * q[i11] * q[i21]
            + (-1.225490e-04) * q[i8] * q[i11] * q[i22] + (3.782355e-04) * q[i8] * q[i12] * q[i15] + (3.393794e-03) * q[i8] * q[i12] * q[i16]
            + (-9.972883e-05) * q[i8] * q[i12] * q[i19] + (2.450668e-04) * q[i8] * q[i12] * q[i20] + (1.240925e-04) * q[i8] * q[i12] * q[i21]
            + (-1.640942e-03) * q[i8] * q[i12] * q[i22] + (1.146822e-06) * q[i8] * q[i15] * q[i16] + (4.156413e-04) * q[i8] * q[i15] * q[i19]
            + (7.485298e-05) * q[i8] * q[i15] * q[i20] + (2.092922e-04) * q[i8] * q[i15] * q[i21] + (3.882708e-05) * q[i8] * q[i15] * q[i22]
            + (-7.327515e-05) * q[i8] * q[i16] * q[i19] + (-3.833072e-04) * q[i8] * q[i16] * q[i20] + (3.885455e-05) * q[i8] * q[i16] * q[i21]
            + (2.265615e-04) * q[i8] * q[i16] * q[i22] + (4.605962e-08) * q[i8] * q[i19] * q[i20] + (-1.010121e-03) * q[i8] * q[i19] * q[i21]
            + (-1.827928e-07) * q[i8] * q[i19] * q[i22] + (6.841617e-06) * q[i8] * q[i20] * q[i21] + (-9.942643e-04) * q[i8] * q[i20] * q[i22]
            + (-2.021385e-07) * q[i8] * q[i21] * q[i22] + (-2.964199e-05) * q[i9] * q[i10] * q[i11] + (-2.893927e-05) * q[i9] * q[i10] * q[i12]
            + (1.243970e-04) * q[i9] * q[i10] * q[i15] + (-1.266824e-04) * q[i9] * q[i10] * q[i16] + (1.701359e-04) * q[i9] * q[i10] * q[i19]
            + (-1.680864e-04) * q[i9] * q[i10] * q[i20] + (1.291903e-05) * q[i9] * q[i10] * q[i21] + (1.231432e-05) * q[i9] * q[i10] * q[i22]
            + (-9.748947e-06) * q[i9] * q[i11] * q[i12] + (-2.354124e-04) * q[i9] * q[i11] * q[i15] + (5.645389e-05) * q[i9] * q[i11] * q[i16]
            + (4.054558e-05) * q[i9] * q[i11] * q[i19] + (3.056139e-05) * q[i9] * q[i11] * q[i20] + (2.158245e-04) * q[i9] * q[i11] * q[i21]
            + (-7.281869e-05) * q[i9] * q[i11] * q[i22] + (-1.428266e-04) * q[i9] * q[i12] * q[i15] + (-2.587763e-05) * q[i9] * q[i12] * q[i16]
            + (-5.674719e-05) * q[i9] * q[i12] * q[i19] + (1.614984e-04) * q[i9] * q[i12] * q[i20] + (3.176837e-05) * q[i9] * q[i12] * q[i21]
            + (-5.370242e-05) * q[i9] * q[i12] * q[i22] + (6.629910e-05) * q[i9] * q[i15] * q[i16] + (1.229711e-04) * q[i9] * q[i15] * q[i19]
            + (1.753397e-04) * q[i9] * q[i15] * q[i20] + (9.140971e-05) * q[i9] * q[i15] * q[i21] + (-3.221827e-05) * q[i9] * q[i15] * q[i22]
            + (-1.553824e-04) * q[i9] * q[i16] * q[i19] + (-3.935135e-05) * q[i9] * q[i16] * q[i20] + (-2.204518e-05) * q[i9] * q[i16] * q[i21]
            + (-5.194976e-05) * q[i9] * q[i16] * q[i22] + (2.941614e-04) * q[i9] * q[i19] * q[i20] + (-6.484038e-05) * q[i9] * q[i19] * q[i21]
            + (4.005138e-05) * q[i9] * q[i19] * q[i22] + (8.318399e-05) * q[i9] * q[i20] * q[i21] + (-2.331863e-04) * q[i9] * q[i20] * q[i22]
            + (4.165778e-05) * q[i9] * q[i21] * q[i22] + (1.347356e-05) * q[i10] * q[i11] * q[i12] + (-2.594386e-05) * q[i10] * q[i11] * q[i15]
            + (-1.444610e-04) * q[i10] * q[i11] * q[i16] + (1.695191e-04) * q[i10] * q[i11] * q[i19] + (-5.633695e-05) * q[i10] * q[i11] * q[i20]
            + (5.205945e-05) * q[i10] * q[i11] * q[i21] + (-3.360076e-05) * q[i10] * q[i11] * q[i22] + (5.493540e-05) * q[i10] * q[i12] * q[i15]
            + (-2.329611e-04) * q[i10] * q[i12] * q[i16] + (3.057630e-05) * q[i10] * q[i12] * q[i19] + (4.205834e-05) * q[i10] * q[i12] * q[i20]
            + (7.259000e-05) * q[i10] * q[i12] * q[i21] + (-2.155244e-04) * q[i10] * q[i12] * q[i22] + (-6.673015e-05) * q[i10] * q[i15] * q[i16]
            + (3.761990e-05) * q[i10] * q[i15] * q[i19] + (1.563051e-04) * q[i10] * q[i15] * q[i20] + (-4.727244e-05) * q[i10] * q[i15] * q[i21]
            + (-2.014826e-05) * q[i10] * q[i15] * q[i22] + (-1.741995e-04) * q[i10] * q[i16] * q[i19] + (-1.220272e-04) * q[i10] * q[i16] * q[i20]
            + (-3.285843e-05) * q[i10] * q[i16] * q[i21] + (8.839866e-05) * q[i10] * q[i16] * q[i22] + (-2.943001e-04) * q[i10] * q[i19] * q[i20]
            + (-2.292217e-04) * q[i10] * q[i19] * q[i21] + (8.241691e-05) * q[i10] * q[i19] * q[i22] + (4.174105e-05) * q[i10] * q[i20] * q[i21]
            + (-6.584737e-05) * q[i10] * q[i20] * q[i22] + (-4.007233e-05) * q[i10] * q[i21] * q[i22] + (2.662090e-04) * q[i11] * q[i12] * q[i15]
            + (-2.646526e-04) * q[i11] * q[i12] * q[i16] + (3.777457e-04) * q[i11] * q[i12] * q[i19] + (-3.773006e-04) * q[i11] * q[i12] * q[i20]
            + (-2.386592e-04) * q[i11] * q[i12] * q[i21] + (-2.399177e-04) * q[i11] * q[i12] * q[i22] + (-1.718668e-04) * q[i11] * q[i15] * q[i16]
            + (-3.795920e-04) * q[i11] * q[i15] * q[i19] + (-3.056488e-05) * q[i11] * q[i15] * q[i20] + (-4.355309e-04) * q[i11] * q[i15] * q[i21]
            + (-7.896743e-05) * q[i11] * q[i15] * q[i22] + (-1.546858e-04) * q[i11] * q[i16] * q[i19] + (1.975458e-04) * q[i11] * q[i16] * q[i20]
            + (-3.401647e-04) * q[i11] * q[i16] * q[i21] + (-3.060890e-04) * q[i11] * q[i16] * q[i22] + (-1.446436e-04) * q[i11] * q[i19] * q[i20]
            + (-1.896972e-03) * q[i11] * q[i19] * q[i21] + (1.041701e-04) * q[i11] * q[i19] * q[i22] + (2.664331e-05) * q[i11] * q[i20] * q[i21]
            + (-2.688682e-04) * q[i11] * q[i20] * q[i22] + (1.066602e-04) * q[i11] * q[i21] * q[i22] + (-1.650153e-04) * q[i12] * q[i15] * q[i16]
            + (1.966239e-04) * q[i12] * q[i15] * q[i19] + (-1.517018e-04) * q[i12] * q[i15] * q[i20] + (3.056551e-04) * q[i12] * q[i15] * q[i21]
            + (3.410488e-04) * q[i12] * q[i15] * q[i22] + (-2.977686e-05) * q[i12] * q[i16] * q[i19] + (-3.921195e-04) * q[i12] * q[i16] * q[i20]
            + (7.957061e-05) * q[i12] * q[i16] * q[i21] + (4.378143e-04) * q[i12] * q[i16] * q[i22] + (-1.457396e-04) * q[i12] * q[i19] * q[i20]
            + (2.692456e-04) * q[i12] * q[i19] * q[i21] + (-2.546263e-05) * q[i12] * q[i19] * q[i22] + (-1.057655e-04) * q[i12] * q[i20] * q[i21]
            + (1.885072e-03) * q[i12] * q[i20] * q[i22] + (1.076934e-04) * q[i12] * q[i21] * q[i22] + (2.180806e-04) * q[i15] * q[i16] * q[i19]
            + (-2.195016e-04) * q[i15] * q[i16] * q[i20] + (-4.443231e-05) * q[i15] * q[i16] * q[i21] + (-4.519659e-05) * q[i15] * q[i16] * q[i22]
            + (-4.297183e-05) * q[i15] * q[i19] * q[i20] + (8.424679e-05) * q[i15] * q[i19] * q[i21] + (-9.699551e-05) * q[i15] * q[i19] * q[i22]
            + (5.359315e-06) * q[i15] * q[i20] * q[i21] + (-5.234313e-06) * q[i15] * q[i20] * q[i22] + (-2.632293e-05) * q[i15] * q[i21] * q[i22]
            + (4.326723e-05) * q[i16] * q[i19] * q[i20] + (-3.497949e-06) * q[i16] * q[i19] * q[i21] + (8.001428e-06) * q[i16] * q[i19] * q[i22]
            + (-9.522871e-05) * q[i16] * q[i20] * q[i21] + (9.050152e-05) * q[i16] * q[i20] * q[i22] + (2.710906e-05) * q[i16] * q[i21] * q[i22]
            + (1.342922e-04) * q[i19] * q[i20] * q[i21] + (1.338215e-04) * q[i19] * q[i20] * q[i22] + (1.658186e-05) * q[i19] * q[i21] * q[i22]
            + (-1.409546e-05) * q[i20] * q[i21] * q[i22];
      return Qx;
   }

   public double getQy(double[] q)
   {
      double Qy;
      Qy = (-3.419960e-03) * q[i0] + (3.490960e-03) * q[i1] + (1.501351e-04) * q[i2] + (1.558602e-03) * q[i3] + (-1.540007e-03) * q[i4]
            + (-3.710266e-05) * q[i5] + (1.330128e-01) * q[i6] + (1.322580e-01) * q[i7] + (1.077327e-01) * q[i8] + (5.828438e-02) * q[i9]
            + (5.766145e-02) * q[i10] + (-6.514705e-03) * q[i11] + (6.950164e-03) * q[i12] + (5.348609e-03) * q[i15] + (5.286144e-03) * q[i16]
            + (-1.190229e-03) * q[i19] + (-1.099037e-03) * q[i20] + (4.107771e-03) * q[i21] + (-4.119158e-03) * q[i22] + (2.105685e-03) * q[i0] * q[i0]
            + (2.094272e-03) * q[i1] * q[i1] + (-1.136704e-03) * q[i2] * q[i2] + (6.271335e-04) * q[i3] * q[i3] + (6.097098e-04) * q[i4] * q[i4]
            + (-2.102908e-03) * q[i5] * q[i5] + (-9.506830e-04) * q[i6] * q[i6] + (-9.205158e-04) * q[i7] * q[i7] + (-1.279854e-04) * q[i8] * q[i8]
            + (-3.456945e-03) * q[i9] * q[i9] + (-3.442015e-03) * q[i10] * q[i10] + (-3.430255e-03) * q[i11] * q[i11] + (-3.350182e-03) * q[i12] * q[i12]
            + (-1.300503e-03) * q[i15] * q[i15] + (-1.319721e-03) * q[i16] * q[i16] + (-1.560022e-04) * q[i19] * q[i19] + (-1.454103e-04) * q[i20] * q[i20]
            + (-6.221178e-04) * q[i21] * q[i21] + (-6.224303e-04) * q[i22] * q[i22] + (5.464848e-04) * q[i0] * q[i1] + (-2.370456e-04) * q[i0] * q[i2]
            + (-1.042953e-01) * q[i0] * q[i3] + (8.473045e-03) * q[i0] * q[i4] + (3.996197e-03) * q[i0] * q[i5] + (6.862044e-03) * q[i0] * q[i6]
            + (1.114285e-03) * q[i0] * q[i7] + (-1.908835e-03) * q[i0] * q[i8] + (4.586587e-04) * q[i0] * q[i9] + (6.756558e-04) * q[i0] * q[i10]
            + (-2.883306e-03) * q[i0] * q[i11] + (-9.513398e-04) * q[i0] * q[i12] + (3.946335e-03) * q[i0] * q[i15] + (-3.339468e-03) * q[i0] * q[i16]
            + (-6.674361e-04) * q[i0] * q[i19] + (-6.149881e-04) * q[i0] * q[i20] + (1.730498e-03) * q[i0] * q[i21] + (1.026309e-03) * q[i0] * q[i22]
            + (-2.264149e-04) * q[i1] * q[i2] + (8.490752e-03) * q[i1] * q[i3] + (-1.037722e-01) * q[i1] * q[i4] + (4.080226e-03) * q[i1] * q[i5]
            + (-1.099342e-03) * q[i1] * q[i6] + (-6.925039e-03) * q[i1] * q[i7] + (1.898408e-03) * q[i1] * q[i8] + (-6.650083e-04) * q[i1] * q[i9]
            + (-4.832283e-04) * q[i1] * q[i10] + (-9.367681e-04) * q[i1] * q[i11] + (-2.903912e-03) * q[i1] * q[i12] + (3.281194e-03) * q[i1] * q[i15]
            + (-3.948355e-03) * q[i1] * q[i16] + (6.088204e-04) * q[i1] * q[i19] + (6.722230e-04) * q[i1] * q[i20] + (1.017126e-03) * q[i1] * q[i21]
            + (1.730079e-03) * q[i1] * q[i22] + (-2.393672e-02) * q[i2] * q[i3] + (-2.378894e-02) * q[i2] * q[i4] + (7.407930e-02) * q[i2] * q[i5]
            + (8.590408e-04) * q[i2] * q[i6] + (-8.558570e-04) * q[i2] * q[i7] + (-2.238345e-05) * q[i2] * q[i8] + (2.099167e-03) * q[i2] * q[i9]
            + (-2.077411e-03) * q[i2] * q[i10] + (-3.180395e-03) * q[i2] * q[i11] + (-3.118885e-03) * q[i2] * q[i12] + (6.499997e-03) * q[i2] * q[i15]
            + (-6.603870e-03) * q[i2] * q[i16] + (-1.564712e-03) * q[i2] * q[i19] + (1.557533e-03) * q[i2] * q[i20] + (1.801057e-03) * q[i2] * q[i21]
            + (1.770597e-03) * q[i2] * q[i22] + (-2.404377e-03) * q[i3] * q[i4] + (5.817271e-04) * q[i3] * q[i5] + (-1.221548e-02) * q[i3] * q[i6]
            + (1.650458e-02) * q[i3] * q[i7] + (1.532152e-03) * q[i3] * q[i8] + (-5.205169e-03) * q[i3] * q[i9] + (4.690330e-03) * q[i3] * q[i10]
            + (-3.008612e-03) * q[i3] * q[i11] + (-5.443482e-03) * q[i3] * q[i12] + (3.868940e-04) * q[i3] * q[i15] + (1.664877e-03) * q[i3] * q[i16]
            + (-2.862405e-03) * q[i3] * q[i19] + (-6.835361e-04) * q[i3] * q[i20] + (-1.083523e-03) * q[i3] * q[i21] + (1.513340e-04) * q[i3] * q[i22]
            + (5.476178e-04) * q[i4] * q[i5] + (-1.650476e-02) * q[i4] * q[i6] + (1.214653e-02) * q[i4] * q[i7] + (-1.465220e-03) * q[i4] * q[i8]
            + (-4.747030e-03) * q[i4] * q[i9] + (5.152011e-03) * q[i4] * q[i10] + (-5.395995e-03) * q[i4] * q[i11] + (-3.023244e-03) * q[i4] * q[i12]
            + (-1.631447e-03) * q[i4] * q[i15] + (-3.370856e-04) * q[i4] * q[i16] + (6.483261e-04) * q[i4] * q[i19] + (2.841136e-03) * q[i4] * q[i20]
            + (1.670042e-04) * q[i4] * q[i21] + (-1.072809e-03) * q[i4] * q[i22] + (-9.827014e-03) * q[i5] * q[i6] + (9.830779e-03) * q[i5] * q[i7]
            + (-1.302839e-05) * q[i5] * q[i8] + (-5.307797e-03) * q[i5] * q[i9] + (5.287685e-03) * q[i5] * q[i10] + (-3.371008e-03) * q[i5] * q[i11]
            + (-3.296339e-03) * q[i5] * q[i12] + (6.842918e-04) * q[i5] * q[i15] + (-7.180200e-04) * q[i5] * q[i16] + (-7.658719e-04) * q[i5] * q[i19]
            + (7.757564e-04) * q[i5] * q[i20] + (1.357518e-03) * q[i5] * q[i21] + (1.335329e-03) * q[i5] * q[i22] + (-3.088028e-03) * q[i6] * q[i7]
            + (1.958398e-03) * q[i6] * q[i8] + (-1.037082e-02) * q[i6] * q[i9] + (5.144393e-03) * q[i6] * q[i10] + (4.540958e-04) * q[i6] * q[i11]
            + (3.960625e-04) * q[i6] * q[i12] + (2.847519e-03) * q[i6] * q[i15] + (5.580052e-03) * q[i6] * q[i16] + (-3.673866e-04) * q[i6] * q[i19]
            + (-1.263329e-03) * q[i6] * q[i20] + (5.394388e-04) * q[i6] * q[i21] + (8.421602e-05) * q[i6] * q[i22] + (1.917137e-03) * q[i7] * q[i8]
            + (5.192358e-03) * q[i7] * q[i9] + (-1.027846e-02) * q[i7] * q[i10] + (-3.654866e-04) * q[i7] * q[i11] + (-4.179029e-04) * q[i7] * q[i12]
            + (5.496503e-03) * q[i7] * q[i15] + (2.848602e-03) * q[i7] * q[i16] + (-1.261602e-03) * q[i7] * q[i19] + (-3.554908e-04) * q[i7] * q[i20]
            + (-8.707822e-05) * q[i7] * q[i21] + (-5.215120e-04) * q[i7] * q[i22] + (3.952787e-03) * q[i8] * q[i9] + (3.915407e-03) * q[i8] * q[i10]
            + (-5.004001e-04) * q[i8] * q[i11] + (3.912603e-04) * q[i8] * q[i12] + (-8.566462e-03) * q[i8] * q[i15] + (-8.621797e-03) * q[i8] * q[i16]
            + (-5.022812e-04) * q[i8] * q[i19] + (-4.814287e-04) * q[i8] * q[i20] + (-2.099477e-03) * q[i8] * q[i21] + (2.088474e-03) * q[i8] * q[i22]
            + (3.393551e-03) * q[i9] * q[i10] + (-5.751373e-04) * q[i9] * q[i11] + (1.103258e-04) * q[i9] * q[i12] + (1.040421e-03) * q[i9] * q[i15]
            + (1.937559e-03) * q[i9] * q[i16] + (1.589063e-04) * q[i9] * q[i19] + (-1.205392e-04) * q[i9] * q[i20] + (-6.516309e-04) * q[i9] * q[i21]
            + (5.353539e-04) * q[i9] * q[i22] + (-8.699660e-05) * q[i10] * q[i11] + (6.043354e-04) * q[i10] * q[i12] + (1.905084e-03) * q[i10] * q[i15]
            + (1.021186e-03) * q[i10] * q[i16] + (-1.201700e-04) * q[i10] * q[i19] + (1.449772e-04) * q[i10] * q[i20] + (-5.336681e-04) * q[i10] * q[i21]
            + (6.467833e-04) * q[i10] * q[i22] + (6.946383e-05) * q[i11] * q[i12] + (8.759169e-04) * q[i11] * q[i15] + (9.139358e-04) * q[i11] * q[i16]
            + (-4.620093e-04) * q[i11] * q[i19] + (-6.228889e-04) * q[i11] * q[i20] + (-1.495999e-03) * q[i11] * q[i21] + (3.249641e-04) * q[i11] * q[i22]
            + (-9.042461e-04) * q[i12] * q[i15] + (-9.498034e-04) * q[i12] * q[i16] + (6.370554e-04) * q[i12] * q[i19] + (4.295278e-04) * q[i12] * q[i20]
            + (3.438006e-04) * q[i12] * q[i21] + (-1.516772e-03) * q[i12] * q[i22] + (-3.944256e-04) * q[i15] * q[i16] + (7.792923e-04) * q[i15] * q[i19]
            + (5.134609e-05) * q[i15] * q[i20] + (3.841905e-04) * q[i15] * q[i21] + (2.592284e-04) * q[i15] * q[i22] + (3.609896e-05) * q[i16] * q[i19]
            + (7.638382e-04) * q[i16] * q[i20] + (-2.607071e-04) * q[i16] * q[i21] + (-3.870956e-04) * q[i16] * q[i22] + (1.147045e-04) * q[i19] * q[i20]
            + (1.353645e-03) * q[i19] * q[i21] + (-3.655234e-04) * q[i19] * q[i22] + (3.754518e-04) * q[i20] * q[i21] + (-1.381182e-03) * q[i20] * q[i22]
            + (-9.737342e-05) * q[i21] * q[i22] + (2.103703e-03) * q[i0] * q[i0] * q[i0] + (9.010064e-04) * q[i0] * q[i0] * q[i1]
            + (2.649180e-03) * q[i0] * q[i0] * q[i2] + (-2.712331e-03) * q[i0] * q[i0] * q[i3] + (-1.903058e-03) * q[i0] * q[i0] * q[i4]
            + (-2.381163e-03) * q[i0] * q[i0] * q[i5] + (-2.368336e-02) * q[i0] * q[i0] * q[i6] + (-1.687363e-03) * q[i0] * q[i0] * q[i7]
            + (3.544038e-04) * q[i0] * q[i0] * q[i8] + (-3.140358e-03) * q[i0] * q[i0] * q[i9] + (-7.828196e-05) * q[i0] * q[i0] * q[i10]
            + (6.092797e-04) * q[i0] * q[i0] * q[i11] + (1.224367e-04) * q[i0] * q[i0] * q[i12] + (6.393383e-04) * q[i0] * q[i0] * q[i15]
            + (-1.798960e-05) * q[i0] * q[i0] * q[i16] + (9.390891e-05) * q[i0] * q[i0] * q[i19] + (4.563936e-04) * q[i0] * q[i0] * q[i20]
            + (1.672795e-04) * q[i0] * q[i0] * q[i21] + (1.590001e-04) * q[i0] * q[i0] * q[i22] + (-8.664399e-04) * q[i0] * q[i1] * q[i1]
            + (-2.111054e-03) * q[i1] * q[i1] * q[i1] + (-2.648981e-03) * q[i1] * q[i1] * q[i2] + (1.908193e-03) * q[i1] * q[i1] * q[i3]
            + (2.724853e-03) * q[i1] * q[i1] * q[i4] + (2.403961e-03) * q[i1] * q[i1] * q[i5] + (-1.694718e-03) * q[i1] * q[i1] * q[i6]
            + (-2.364088e-02) * q[i1] * q[i1] * q[i7] + (3.581257e-04) * q[i1] * q[i1] * q[i8] + (-7.962768e-05) * q[i1] * q[i1] * q[i9]
            + (-3.113276e-03) * q[i1] * q[i1] * q[i10] + (-1.286764e-04) * q[i1] * q[i1] * q[i11] + (-6.013921e-04) * q[i1] * q[i1] * q[i12]
            + (-2.069750e-05) * q[i1] * q[i1] * q[i15] + (6.446987e-04) * q[i1] * q[i1] * q[i16] + (4.612640e-04) * q[i1] * q[i1] * q[i19]
            + (9.646236e-05) * q[i1] * q[i1] * q[i20] + (-1.517087e-04) * q[i1] * q[i1] * q[i21] + (-1.698214e-04) * q[i1] * q[i1] * q[i22]
            + (1.848346e-03) * q[i0] * q[i2] * q[i2] + (-1.845939e-03) * q[i1] * q[i2] * q[i2] + (-1.336101e-05) * q[i2] * q[i2] * q[i2]
            + (-1.172723e-03) * q[i2] * q[i2] * q[i3] + (1.181290e-03) * q[i2] * q[i2] * q[i4] + (-9.250102e-06) * q[i2] * q[i2] * q[i5]
            + (-1.935695e-03) * q[i2] * q[i2] * q[i6] + (-1.946621e-03) * q[i2] * q[i2] * q[i7] + (-3.025856e-02) * q[i2] * q[i2] * q[i8]
            + (-1.384378e-03) * q[i2] * q[i2] * q[i9] + (-1.382920e-03) * q[i2] * q[i2] * q[i10] + (8.277614e-04) * q[i2] * q[i2] * q[i11]
            + (-8.804478e-04) * q[i2] * q[i2] * q[i12] + (-5.454598e-05) * q[i2] * q[i2] * q[i15] + (-4.365303e-05) * q[i2] * q[i2] * q[i16]
            + (6.886514e-04) * q[i2] * q[i2] * q[i19] + (6.695697e-04) * q[i2] * q[i2] * q[i20] + (1.455581e-04) * q[i2] * q[i2] * q[i21]
            + (-1.499381e-04) * q[i2] * q[i2] * q[i22] + (7.395034e-03) * q[i0] * q[i3] * q[i3] + (-2.389710e-03) * q[i1] * q[i3] * q[i3]
            + (2.295159e-03) * q[i2] * q[i3] * q[i3] + (-2.073203e-04) * q[i3] * q[i3] * q[i3] + (2.238555e-03) * q[i3] * q[i3] * q[i4]
            + (1.725926e-03) * q[i3] * q[i3] * q[i5] + (1.004020e-02) * q[i3] * q[i3] * q[i6] + (-3.008768e-03) * q[i3] * q[i3] * q[i7]
            + (3.782819e-03) * q[i3] * q[i3] * q[i8] + (1.625013e-03) * q[i3] * q[i3] * q[i9] + (-1.056831e-03) * q[i3] * q[i3] * q[i10]
            + (2.469390e-04) * q[i3] * q[i3] * q[i11] + (-1.931397e-04) * q[i3] * q[i3] * q[i12] + (5.459118e-04) * q[i3] * q[i3] * q[i15]
            + (-2.540502e-05) * q[i3] * q[i3] * q[i16] + (6.712668e-04) * q[i3] * q[i3] * q[i19] + (-6.506100e-04) * q[i3] * q[i3] * q[i20]
            + (-2.625181e-04) * q[i3] * q[i3] * q[i21] + (1.063758e-04) * q[i3] * q[i3] * q[i22] + (2.391480e-03) * q[i0] * q[i4] * q[i4]
            + (-7.337485e-03) * q[i1] * q[i4] * q[i4] + (-2.268454e-03) * q[i2] * q[i4] * q[i4] + (-2.231748e-03) * q[i3] * q[i4] * q[i4]
            + (1.951299e-04) * q[i4] * q[i4] * q[i4] + (-1.735576e-03) * q[i4] * q[i4] * q[i5] + (-3.015428e-03) * q[i4] * q[i4] * q[i6]
            + (1.006275e-02) * q[i4] * q[i4] * q[i7] + (3.800462e-03) * q[i4] * q[i4] * q[i8] + (-1.062881e-03) * q[i4] * q[i4] * q[i9]
            + (1.624405e-03) * q[i4] * q[i4] * q[i10] + (1.929410e-04) * q[i4] * q[i4] * q[i11] + (-2.181179e-04) * q[i4] * q[i4] * q[i12]
            + (-2.829543e-05) * q[i4] * q[i4] * q[i15] + (5.306661e-04) * q[i4] * q[i4] * q[i16] + (-6.442424e-04) * q[i4] * q[i4] * q[i19]
            + (6.694240e-04) * q[i4] * q[i4] * q[i20] + (-1.053468e-04) * q[i4] * q[i4] * q[i21] + (2.582947e-04) * q[i4] * q[i4] * q[i22]
            + (-9.545015e-04) * q[i0] * q[i5] * q[i5] + (9.680336e-04) * q[i1] * q[i5] * q[i5] + (-2.336732e-05) * q[i2] * q[i5] * q[i5]
            + (-4.359289e-04) * q[i3] * q[i5] * q[i5] + (4.155623e-04) * q[i4] * q[i5] * q[i5] + (2.164887e-05) * q[i5] * q[i5] * q[i5]
            + (1.256812e-03) * q[i5] * q[i5] * q[i6] + (1.240186e-03) * q[i5] * q[i5] * q[i7] + (3.513279e-03) * q[i5] * q[i5] * q[i8]
            + (4.900245e-04) * q[i5] * q[i5] * q[i9] + (4.921660e-04) * q[i5] * q[i5] * q[i10] + (-1.414232e-04) * q[i5] * q[i5] * q[i11]
            + (1.351142e-04) * q[i5] * q[i5] * q[i12] + (-1.570381e-03) * q[i5] * q[i5] * q[i15] + (-1.588174e-03) * q[i5] * q[i5] * q[i16]
            + (-7.619498e-04) * q[i5] * q[i5] * q[i19] + (-7.367319e-04) * q[i5] * q[i5] * q[i20] + (-3.578230e-04) * q[i5] * q[i5] * q[i21]
            + (3.657845e-04) * q[i5] * q[i5] * q[i22] + (-4.341833e-03) * q[i0] * q[i6] * q[i6] + (-8.176774e-04) * q[i1] * q[i6] * q[i6]
            + (-5.110815e-04) * q[i2] * q[i6] * q[i6] + (-1.290716e-03) * q[i3] * q[i6] * q[i6] + (9.079333e-04) * q[i4] * q[i6] * q[i6]
            + (-8.333533e-05) * q[i5] * q[i6] * q[i6] + (-2.142596e-03) * q[i6] * q[i6] * q[i6] + (-1.413375e-03) * q[i6] * q[i6] * q[i7]
            + (2.408843e-03) * q[i6] * q[i6] * q[i8] + (-8.203321e-04) * q[i6] * q[i6] * q[i9] + (-4.106816e-04) * q[i6] * q[i6] * q[i10]
            + (-4.244388e-04) * q[i6] * q[i6] * q[i11] + (-2.558142e-04) * q[i6] * q[i6] * q[i12] + (-5.124145e-04) * q[i6] * q[i6] * q[i15]
            + (-5.331069e-04) * q[i6] * q[i6] * q[i16] + (-8.688893e-04) * q[i6] * q[i6] * q[i19] + (-2.521222e-04) * q[i6] * q[i6] * q[i20]
            + (-3.175184e-04) * q[i6] * q[i6] * q[i21] + (4.531219e-04) * q[i6] * q[i6] * q[i22] + (8.112818e-04) * q[i0] * q[i7] * q[i7]
            + (4.321724e-03) * q[i1] * q[i7] * q[i7] + (5.175746e-04) * q[i2] * q[i7] * q[i7] + (-9.041409e-04) * q[i3] * q[i7] * q[i7]
            + (1.320553e-03) * q[i4] * q[i7] * q[i7] + (8.674062e-05) * q[i5] * q[i7] * q[i7] + (-1.409038e-03) * q[i6] * q[i7] * q[i7]
            + (-2.152157e-03) * q[i7] * q[i7] * q[i7] + (2.430478e-03) * q[i7] * q[i7] * q[i8] + (-4.139885e-04) * q[i7] * q[i7] * q[i9]
            + (-8.165828e-04) * q[i7] * q[i7] * q[i10] + (2.617575e-04) * q[i7] * q[i7] * q[i11] + (4.257258e-04) * q[i7] * q[i7] * q[i12]
            + (-5.255415e-04) * q[i7] * q[i7] * q[i15] + (-5.192656e-04) * q[i7] * q[i7] * q[i16] + (-2.492046e-04) * q[i7] * q[i7] * q[i19]
            + (-8.649150e-04) * q[i7] * q[i7] * q[i20] + (-4.481974e-04) * q[i7] * q[i7] * q[i21] + (3.133449e-04) * q[i7] * q[i7] * q[i22]
            + (-1.456665e-03) * q[i0] * q[i8] * q[i8] + (1.468737e-03) * q[i1] * q[i8] * q[i8] + (2.545932e-05) * q[i2] * q[i8] * q[i8]
            + (-1.144956e-03) * q[i3] * q[i8] * q[i8] + (1.125159e-03) * q[i4] * q[i8] * q[i8] + (4.126801e-06) * q[i5] * q[i8] * q[i8]
            + (1.105897e-03) * q[i6] * q[i8] * q[i8] + (1.110014e-03) * q[i7] * q[i8] * q[i8] + (-4.060335e-03) * q[i8] * q[i8] * q[i8]
            + (2.916677e-04) * q[i8] * q[i8] * q[i9] + (2.897784e-04) * q[i8] * q[i8] * q[i10] + (5.352001e-04) * q[i8] * q[i8] * q[i11]
            + (-5.628161e-04) * q[i8] * q[i8] * q[i12] + (2.523868e-04) * q[i8] * q[i8] * q[i15] + (2.591024e-04) * q[i8] * q[i8] * q[i16]
            + (6.735998e-05) * q[i8] * q[i8] * q[i19] + (5.945936e-05) * q[i8] * q[i8] * q[i20] + (-4.600819e-04) * q[i8] * q[i8] * q[i21]
            + (4.633328e-04) * q[i8] * q[i8] * q[i22] + (-1.972233e-04) * q[i0] * q[i9] * q[i9] + (-3.188681e-04) * q[i1] * q[i9] * q[i9]
            + (5.600260e-04) * q[i2] * q[i9] * q[i9] + (8.883241e-04) * q[i3] * q[i9] * q[i9] + (8.941807e-04) * q[i4] * q[i9] * q[i9]
            + (1.103610e-03) * q[i5] * q[i9] * q[i9] + (-2.560459e-03) * q[i6] * q[i9] * q[i9] + (4.640644e-05) * q[i7] * q[i9] * q[i9]
            + (6.011000e-04) * q[i8] * q[i9] * q[i9] + (-9.366195e-04) * q[i9] * q[i9] * q[i9] + (-2.172592e-04) * q[i9] * q[i9] * q[i10]
            + (3.615142e-05) * q[i9] * q[i9] * q[i11] + (3.414539e-04) * q[i9] * q[i9] * q[i12] + (-9.578398e-05) * q[i9] * q[i9] * q[i15]
            + (-2.517296e-04) * q[i9] * q[i9] * q[i16] + (5.892796e-05) * q[i9] * q[i9] * q[i19] + (7.640899e-05) * q[i9] * q[i9] * q[i20]
            + (7.150478e-05) * q[i9] * q[i9] * q[i21] + (-1.594535e-04) * q[i9] * q[i9] * q[i22] + (3.095510e-04) * q[i0] * q[i10] * q[i10]
            + (1.998999e-04) * q[i1] * q[i10] * q[i10] + (-5.555149e-04) * q[i2] * q[i10] * q[i10] + (-8.852734e-04) * q[i3] * q[i10] * q[i10]
            + (-8.770639e-04) * q[i4] * q[i10] * q[i10] + (-1.104244e-03) * q[i5] * q[i10] * q[i10] + (4.343192e-05) * q[i6] * q[i10] * q[i10]
            + (-2.533437e-03) * q[i7] * q[i10] * q[i10] + (5.970276e-04) * q[i8] * q[i10] * q[i10] + (-2.201108e-04) * q[i9] * q[i10] * q[i10]
            + (-9.242922e-04) * q[i10] * q[i10] * q[i10] + (-3.427192e-04) * q[i10] * q[i10] * q[i11] + (-3.808000e-05) * q[i10] * q[i10] * q[i12]
            + (-2.469388e-04) * q[i10] * q[i10] * q[i15] + (-9.715316e-05) * q[i10] * q[i10] * q[i16] + (7.674123e-05) * q[i10] * q[i10] * q[i19]
            + (5.754956e-05) * q[i10] * q[i10] * q[i20] + (1.589391e-04) * q[i10] * q[i10] * q[i21] + (-7.038176e-05) * q[i10] * q[i10] * q[i22]
            + (2.080339e-04) * q[i0] * q[i11] * q[i11] + (-3.758514e-05) * q[i1] * q[i11] * q[i11] + (2.278370e-03) * q[i2] * q[i11] * q[i11]
            + (6.860758e-04) * q[i3] * q[i11] * q[i11] + (-4.984141e-04) * q[i4] * q[i11] * q[i11] + (9.546430e-04) * q[i5] * q[i11] * q[i11]
            + (-6.108288e-04) * q[i6] * q[i11] * q[i11] + (8.434333e-05) * q[i7] * q[i11] * q[i11] + (1.581777e-03) * q[i8] * q[i11] * q[i11]
            + (-1.448847e-04) * q[i9] * q[i11] * q[i11] + (-7.667511e-06) * q[i10] * q[i11] * q[i11] + (1.164990e-03) * q[i11] * q[i11] * q[i11]
            + (-4.182149e-05) * q[i11] * q[i11] * q[i12] + (-2.020515e-03) * q[i11] * q[i11] * q[i15] + (-1.803217e-04) * q[i11] * q[i11] * q[i16]
            + (6.447725e-04) * q[i11] * q[i11] * q[i19] + (-8.472641e-05) * q[i11] * q[i11] * q[i20] + (1.849186e-05) * q[i11] * q[i11] * q[i21]
            + (6.243942e-05) * q[i11] * q[i11] * q[i22] + (3.540226e-05) * q[i0] * q[i12] * q[i12] + (-2.165943e-04) * q[i1] * q[i12] * q[i12]
            + (-2.398656e-03) * q[i2] * q[i12] * q[i12] + (4.919326e-04) * q[i3] * q[i12] * q[i12] + (-7.033265e-04) * q[i4] * q[i12] * q[i12]
            + (-1.014978e-03) * q[i5] * q[i12] * q[i12] + (8.812535e-05) * q[i6] * q[i12] * q[i12] + (-6.252605e-04) * q[i7] * q[i12] * q[i12]
            + (1.612018e-03) * q[i8] * q[i12] * q[i12] + (6.421360e-06) * q[i9] * q[i12] * q[i12] + (-1.478634e-04) * q[i10] * q[i12] * q[i12]
            + (4.737131e-05) * q[i11] * q[i12] * q[i12] + (-1.218456e-03) * q[i12] * q[i12] * q[i12] + (-1.863703e-04) * q[i12] * q[i12] * q[i15]
            + (-2.013504e-03) * q[i12] * q[i12] * q[i16] + (-8.000025e-05) * q[i12] * q[i12] * q[i19] + (6.312400e-04) * q[i12] * q[i12] * q[i20]
            + (-6.533212e-05) * q[i12] * q[i12] * q[i21] + (-1.222734e-05) * q[i12] * q[i12] * q[i22] + (2.058150e-04) * q[i0] * q[i15] * q[i15]
            + (3.545634e-04) * q[i1] * q[i15] * q[i15] + (2.669399e-03) * q[i2] * q[i15] * q[i15] + (6.707951e-05) * q[i3] * q[i15] * q[i15]
            + (-4.986179e-05) * q[i4] * q[i15] * q[i15] + (-5.327172e-04) * q[i5] * q[i15] * q[i15] + (6.372224e-05) * q[i6] * q[i15] * q[i15]
            + (7.534292e-04) * q[i7] * q[i15] * q[i15] + (1.903655e-04) * q[i8] * q[i15] * q[i15] + (-1.399477e-04) * q[i9] * q[i15] * q[i15]
            + (-1.180544e-04) * q[i10] * q[i15] * q[i15] + (2.029845e-03) * q[i11] * q[i15] * q[i15] + (1.936846e-04) * q[i12] * q[i15] * q[i15]
            + (3.132070e-04) * q[i15] * q[i15] * q[i15] + (-8.299577e-05) * q[i15] * q[i15] * q[i16] + (2.742122e-04) * q[i15] * q[i15] * q[i19]
            + (-1.641042e-04) * q[i15] * q[i15] * q[i20] + (-2.893991e-04) * q[i15] * q[i15] * q[i21] + (8.124079e-05) * q[i15] * q[i15] * q[i22]
            + (-3.579373e-04) * q[i0] * q[i16] * q[i16] + (-2.042699e-04) * q[i1] * q[i16] * q[i16] + (-2.696586e-03) * q[i2] * q[i16] * q[i16]
            + (4.782192e-05) * q[i3] * q[i16] * q[i16] + (-6.367990e-05) * q[i4] * q[i16] * q[i16] + (5.323044e-04) * q[i5] * q[i16] * q[i16]
            + (7.546963e-04) * q[i6] * q[i16] * q[i16] + (6.457506e-05) * q[i7] * q[i16] * q[i16] + (2.011981e-04) * q[i8] * q[i16] * q[i16]
            + (-1.199851e-04) * q[i9] * q[i16] * q[i16] + (-1.432445e-04) * q[i10] * q[i16] * q[i16] + (-1.955120e-04) * q[i11] * q[i16] * q[i16]
            + (-2.063879e-03) * q[i12] * q[i16] * q[i16] + (-8.439846e-05) * q[i15] * q[i16] * q[i16] + (3.163447e-04) * q[i16] * q[i16] * q[i16]
            + (-1.660521e-04) * q[i16] * q[i16] * q[i19] + (2.685030e-04) * q[i16] * q[i16] * q[i20] + (-8.377281e-05) * q[i16] * q[i16] * q[i21]
            + (2.956557e-04) * q[i16] * q[i16] * q[i22] + (1.713251e-04) * q[i0] * q[i19] * q[i19] + (2.274663e-05) * q[i1] * q[i19] * q[i19]
            + (-2.059287e-04) * q[i2] * q[i19] * q[i19] + (-6.031514e-04) * q[i3] * q[i19] * q[i19] + (6.606077e-05) * q[i4] * q[i19] * q[i19]
            + (-6.602141e-04) * q[i5] * q[i19] * q[i19] + (-3.221905e-04) * q[i6] * q[i19] * q[i19] + (1.443314e-04) * q[i7] * q[i19] * q[i19]
            + (-2.638921e-04) * q[i8] * q[i19] * q[i19] + (-3.016804e-05) * q[i9] * q[i19] * q[i19] + (-1.226696e-04) * q[i10] * q[i19] * q[i19]
            + (-4.295102e-04) * q[i11] * q[i19] * q[i19] + (8.348564e-05) * q[i12] * q[i19] * q[i19] + (-2.575059e-04) * q[i15] * q[i19] * q[i19]
            + (-3.148092e-05) * q[i16] * q[i19] * q[i19] + (2.139490e-04) * q[i19] * q[i19] * q[i19] + (3.710345e-05) * q[i19] * q[i19] * q[i20]
            + (-1.694254e-04) * q[i19] * q[i19] * q[i21] + (3.185691e-05) * q[i19] * q[i19] * q[i22] + (-2.899285e-05) * q[i0] * q[i20] * q[i20]
            + (-1.739136e-04) * q[i1] * q[i20] * q[i20] + (2.077010e-04) * q[i2] * q[i20] * q[i20] + (-6.408954e-05) * q[i3] * q[i20] * q[i20]
            + (5.927220e-04) * q[i4] * q[i20] * q[i20] + (6.640720e-04) * q[i5] * q[i20] * q[i20] + (1.437983e-04) * q[i6] * q[i20] * q[i20]
            + (-3.164190e-04) * q[i7] * q[i20] * q[i20] + (-2.617898e-04) * q[i8] * q[i20] * q[i20] + (-1.238043e-04) * q[i9] * q[i20] * q[i20]
            + (-3.093279e-05) * q[i10] * q[i20] * q[i20] + (-8.136337e-05) * q[i11] * q[i20] * q[i20] + (4.310497e-04) * q[i12] * q[i20] * q[i20]
            + (-3.405803e-05) * q[i15] * q[i20] * q[i20] + (-2.465934e-04) * q[i16] * q[i20] * q[i20] + (3.678517e-05) * q[i19] * q[i20] * q[i20]
            + (2.096029e-04) * q[i20] * q[i20] * q[i20] + (-2.987384e-05) * q[i20] * q[i20] * q[i21] + (1.685686e-04) * q[i20] * q[i20] * q[i22]
            + (6.082433e-05) * q[i0] * q[i21] * q[i21] + (1.276385e-04) * q[i1] * q[i21] * q[i21] + (3.646947e-04) * q[i2] * q[i21] * q[i21]
            + (1.764318e-04) * q[i3] * q[i21] * q[i21] + (2.558276e-04) * q[i4] * q[i21] * q[i21] + (-6.442669e-04) * q[i5] * q[i21] * q[i21]
            + (1.089340e-04) * q[i6] * q[i21] * q[i21] + (4.041422e-04) * q[i7] * q[i21] * q[i21] + (-8.752838e-04) * q[i8] * q[i21] * q[i21]
            + (2.229091e-06) * q[i9] * q[i21] * q[i21] + (8.894858e-06) * q[i10] * q[i21] * q[i21] + (-3.551176e-04) * q[i11] * q[i21] * q[i21]
            + (-9.283926e-05) * q[i12] * q[i21] * q[i21] + (-1.400345e-04) * q[i15] * q[i21] * q[i21] + (-4.668007e-05) * q[i16] * q[i21] * q[i21]
            + (5.828106e-04) * q[i19] * q[i21] * q[i21] + (-9.207259e-05) * q[i20] * q[i21] * q[i21] + (-7.129647e-05) * q[i21] * q[i21] * q[i21]
            + (-1.070164e-05) * q[i21] * q[i21] * q[i22] + (-1.294115e-04) * q[i0] * q[i22] * q[i22] + (-5.942282e-05) * q[i1] * q[i22] * q[i22]
            + (-3.571494e-04) * q[i2] * q[i22] * q[i22] + (-2.551087e-04) * q[i3] * q[i22] * q[i22] + (-1.807228e-04) * q[i4] * q[i22] * q[i22]
            + (6.537732e-04) * q[i5] * q[i22] * q[i22] + (4.046895e-04) * q[i6] * q[i22] * q[i22] + (1.068850e-04) * q[i7] * q[i22] * q[i22]
            + (-8.719102e-04) * q[i8] * q[i22] * q[i22] + (7.763999e-06) * q[i9] * q[i22] * q[i22] + (6.995825e-07) * q[i10] * q[i22] * q[i22]
            + (9.220215e-05) * q[i11] * q[i22] * q[i22] + (3.677878e-04) * q[i12] * q[i22] * q[i22] + (-4.611770e-05) * q[i15] * q[i22] * q[i22]
            + (-1.417031e-04) * q[i16] * q[i22] * q[i22] + (-9.226347e-05) * q[i19] * q[i22] * q[i22] + (5.922148e-04) * q[i20] * q[i22] * q[i22]
            + (9.836954e-06) * q[i21] * q[i22] * q[i22] + (7.309101e-05) * q[i22] * q[i22] * q[i22] + (1.132544e-05) * q[i0] * q[i1] * q[i2]
            + (-3.003652e-03) * q[i0] * q[i1] * q[i3] + (2.966210e-03) * q[i0] * q[i1] * q[i4] + (-6.624666e-06) * q[i0] * q[i1] * q[i5]
            + (5.883122e-03) * q[i0] * q[i1] * q[i6] + (5.865579e-03) * q[i0] * q[i1] * q[i7] + (2.177889e-03) * q[i0] * q[i1] * q[i8]
            + (1.000905e-03) * q[i0] * q[i1] * q[i9] + (9.960849e-04) * q[i0] * q[i1] * q[i10] + (5.025431e-04) * q[i0] * q[i1] * q[i11]
            + (-4.917774e-04) * q[i0] * q[i1] * q[i12] + (-2.196508e-04) * q[i0] * q[i1] * q[i15] + (-2.161718e-04) * q[i0] * q[i1] * q[i16]
            + (-8.633194e-04) * q[i0] * q[i1] * q[i19] + (-8.676393e-04) * q[i0] * q[i1] * q[i20] + (4.477294e-04) * q[i0] * q[i1] * q[i21]
            + (-4.469170e-04) * q[i0] * q[i1] * q[i22] + (-4.418969e-03) * q[i0] * q[i2] * q[i3] + (-9.307455e-04) * q[i0] * q[i2] * q[i4]
            + (-1.340187e-03) * q[i0] * q[i2] * q[i5] + (-1.042584e-02) * q[i0] * q[i2] * q[i6] + (6.692984e-04) * q[i0] * q[i2] * q[i7]
            + (-4.592705e-03) * q[i0] * q[i2] * q[i8] + (-1.144177e-03) * q[i0] * q[i2] * q[i9] + (1.910390e-03) * q[i0] * q[i2] * q[i10]
            + (1.065594e-03) * q[i0] * q[i2] * q[i11] + (-1.294444e-04) * q[i0] * q[i2] * q[i12] + (2.449058e-04) * q[i0] * q[i2] * q[i15]
            + (2.344968e-04) * q[i0] * q[i2] * q[i16] + (5.742787e-04) * q[i0] * q[i2] * q[i19] + (-5.652489e-04) * q[i0] * q[i2] * q[i20]
            + (2.171232e-04) * q[i0] * q[i2] * q[i21] + (-8.866820e-04) * q[i0] * q[i2] * q[i22] + (1.369951e-02) * q[i0] * q[i3] * q[i4]
            + (6.985472e-03) * q[i0] * q[i3] * q[i5] + (-1.610863e-02) * q[i0] * q[i3] * q[i6] + (3.069103e-03) * q[i0] * q[i3] * q[i7]
            + (-2.993308e-04) * q[i0] * q[i3] * q[i8] + (1.178320e-02) * q[i0] * q[i3] * q[i9] + (-3.278556e-03) * q[i0] * q[i3] * q[i10]
            + (4.723161e-04) * q[i0] * q[i3] * q[i11] + (-4.202721e-04) * q[i0] * q[i3] * q[i12] + (-1.399009e-03) * q[i0] * q[i3] * q[i15]
            + (-4.426984e-03) * q[i0] * q[i3] * q[i16] + (-1.230233e-03) * q[i0] * q[i3] * q[i19] + (-4.987277e-04) * q[i0] * q[i3] * q[i20]
            + (-4.396539e-04) * q[i0] * q[i3] * q[i21] + (-1.155803e-03) * q[i0] * q[i3] * q[i22] + (4.170673e-03) * q[i0] * q[i4] * q[i5]
            + (-4.381152e-04) * q[i0] * q[i4] * q[i6] + (3.905466e-03) * q[i0] * q[i4] * q[i7] + (-1.282934e-03) * q[i0] * q[i4] * q[i8]
            + (3.261963e-04) * q[i0] * q[i4] * q[i9] + (-7.603548e-04) * q[i0] * q[i4] * q[i10] + (-2.240502e-04) * q[i0] * q[i4] * q[i11]
            + (-8.096605e-04) * q[i0] * q[i4] * q[i12] + (3.336301e-05) * q[i0] * q[i4] * q[i15] + (1.829189e-05) * q[i0] * q[i4] * q[i16]
            + (6.015720e-04) * q[i0] * q[i4] * q[i19] + (8.366387e-06) * q[i0] * q[i4] * q[i20] + (-2.850994e-04) * q[i0] * q[i4] * q[i21]
            + (-6.946741e-04) * q[i0] * q[i4] * q[i22] + (2.766211e-03) * q[i0] * q[i5] * q[i6] + (-4.716225e-04) * q[i0] * q[i5] * q[i7]
            + (-2.733731e-05) * q[i0] * q[i5] * q[i8] + (5.309420e-04) * q[i0] * q[i5] * q[i9] + (7.873888e-04) * q[i0] * q[i5] * q[i10]
            + (-1.103549e-03) * q[i0] * q[i5] * q[i11] + (1.435492e-03) * q[i0] * q[i5] * q[i12] + (-7.085482e-04) * q[i0] * q[i5] * q[i15]
            + (-2.215502e-04) * q[i0] * q[i5] * q[i16] + (2.208192e-05) * q[i0] * q[i5] * q[i19] + (-6.243246e-04) * q[i0] * q[i5] * q[i20]
            + (-5.182215e-04) * q[i0] * q[i5] * q[i21] + (6.836454e-04) * q[i0] * q[i5] * q[i22] + (1.146902e-02) * q[i0] * q[i6] * q[i7]
            + (1.381052e-03) * q[i0] * q[i6] * q[i8] + (-1.529254e-03) * q[i0] * q[i6] * q[i9] + (2.113707e-03) * q[i0] * q[i6] * q[i10]
            + (-1.800672e-03) * q[i0] * q[i6] * q[i11] + (-1.143893e-03) * q[i0] * q[i6] * q[i12] + (-6.248782e-05) * q[i0] * q[i6] * q[i15]
            + (1.073967e-03) * q[i0] * q[i6] * q[i16] + (-1.418968e-03) * q[i0] * q[i6] * q[i19] + (1.062149e-03) * q[i0] * q[i6] * q[i20]
            + (4.109724e-04) * q[i0] * q[i6] * q[i21] + (2.203350e-04) * q[i0] * q[i6] * q[i22] + (-2.645940e-04) * q[i0] * q[i7] * q[i8]
            + (2.004623e-03) * q[i0] * q[i7] * q[i9] + (8.376530e-05) * q[i0] * q[i7] * q[i10] + (1.402177e-04) * q[i0] * q[i7] * q[i11]
            + (-1.169007e-04) * q[i0] * q[i7] * q[i12] + (6.449001e-04) * q[i0] * q[i7] * q[i15] + (3.597124e-04) * q[i0] * q[i7] * q[i16]
            + (-1.804753e-04) * q[i0] * q[i7] * q[i19] + (6.153951e-05) * q[i0] * q[i7] * q[i20] + (-2.018885e-04) * q[i0] * q[i7] * q[i21]
            + (-5.363128e-04) * q[i0] * q[i7] * q[i22] + (1.232822e-03) * q[i0] * q[i8] * q[i9] + (1.771793e-04) * q[i0] * q[i8] * q[i10]
            + (-5.257208e-04) * q[i0] * q[i8] * q[i11] + (6.450441e-06) * q[i0] * q[i8] * q[i12] + (-2.445200e-04) * q[i0] * q[i8] * q[i15]
            + (6.730540e-04) * q[i0] * q[i8] * q[i16] + (1.101120e-04) * q[i0] * q[i8] * q[i19] + (6.534280e-04) * q[i0] * q[i8] * q[i20]
            + (-1.374253e-04) * q[i0] * q[i8] * q[i21] + (3.502593e-05) * q[i0] * q[i8] * q[i22] + (2.411114e-04) * q[i0] * q[i9] * q[i10]
            + (8.550443e-05) * q[i0] * q[i9] * q[i11] + (-4.645134e-04) * q[i0] * q[i9] * q[i12] + (-3.216631e-04) * q[i0] * q[i9] * q[i15]
            + (3.160296e-04) * q[i0] * q[i9] * q[i16] + (-2.978277e-04) * q[i0] * q[i9] * q[i19] + (6.484969e-04) * q[i0] * q[i9] * q[i20]
            + (-1.291727e-04) * q[i0] * q[i9] * q[i21] + (-1.041448e-04) * q[i0] * q[i9] * q[i22] + (9.875970e-04) * q[i0] * q[i10] * q[i11]
            + (4.411800e-04) * q[i0] * q[i10] * q[i12] + (3.368562e-04) * q[i0] * q[i10] * q[i15] + (-3.942323e-04) * q[i0] * q[i10] * q[i16]
            + (3.867514e-05) * q[i0] * q[i10] * q[i19] + (3.980839e-05) * q[i0] * q[i10] * q[i20] + (-5.573833e-05) * q[i0] * q[i10] * q[i21]
            + (-2.403846e-04) * q[i0] * q[i10] * q[i22] + (1.048394e-04) * q[i0] * q[i11] * q[i12] + (1.167051e-04) * q[i0] * q[i11] * q[i15]
            + (-3.487901e-04) * q[i0] * q[i11] * q[i16] + (2.393338e-04) * q[i0] * q[i11] * q[i19] + (1.528574e-04) * q[i0] * q[i11] * q[i20]
            + (-1.610796e-04) * q[i0] * q[i11] * q[i21] + (3.191131e-04) * q[i0] * q[i11] * q[i22] + (1.398222e-04) * q[i0] * q[i12] * q[i15]
            + (-7.891911e-05) * q[i0] * q[i12] * q[i16] + (-1.259175e-04) * q[i0] * q[i12] * q[i19] + (-1.010733e-05) * q[i0] * q[i12] * q[i20]
            + (-1.028225e-04) * q[i0] * q[i12] * q[i21] + (-1.881295e-04) * q[i0] * q[i12] * q[i22] + (2.723890e-04) * q[i0] * q[i15] * q[i16]
            + (-9.798634e-05) * q[i0] * q[i15] * q[i19] + (-5.472361e-05) * q[i0] * q[i15] * q[i20] + (2.581117e-04) * q[i0] * q[i15] * q[i21]
            + (8.102358e-05) * q[i0] * q[i15] * q[i22] + (-5.580437e-04) * q[i0] * q[i16] * q[i19] + (-3.439001e-04) * q[i0] * q[i16] * q[i20]
            + (-1.036131e-04) * q[i0] * q[i16] * q[i21] + (6.974141e-04) * q[i0] * q[i16] * q[i22] + (-3.581878e-04) * q[i0] * q[i19] * q[i20]
            + (-1.261794e-04) * q[i0] * q[i19] * q[i21] + (3.998468e-06) * q[i0] * q[i19] * q[i22] + (1.815776e-04) * q[i0] * q[i20] * q[i21]
            + (-2.077979e-04) * q[i0] * q[i20] * q[i22] + (-4.586401e-04) * q[i0] * q[i21] * q[i22] + (9.468376e-04) * q[i1] * q[i2] * q[i3]
            + (4.420908e-03) * q[i1] * q[i2] * q[i4] + (1.337284e-03) * q[i1] * q[i2] * q[i5] + (6.545506e-04) * q[i1] * q[i2] * q[i6]
            + (-1.043075e-02) * q[i1] * q[i2] * q[i7] + (-4.582863e-03) * q[i1] * q[i2] * q[i8] + (1.919334e-03) * q[i1] * q[i2] * q[i9]
            + (-1.144844e-03) * q[i1] * q[i2] * q[i10] + (1.398578e-04) * q[i1] * q[i2] * q[i11] + (-1.061259e-03) * q[i1] * q[i2] * q[i12]
            + (2.364509e-04) * q[i1] * q[i2] * q[i15] + (2.506379e-04) * q[i1] * q[i2] * q[i16] + (-5.593445e-04) * q[i1] * q[i2] * q[i19]
            + (5.744647e-04) * q[i1] * q[i2] * q[i20] + (8.894065e-04) * q[i1] * q[i2] * q[i21] + (-2.208451e-04) * q[i1] * q[i2] * q[i22]
            + (-1.371216e-02) * q[i1] * q[i3] * q[i4] + (-4.197114e-03) * q[i1] * q[i3] * q[i5] + (3.907965e-03) * q[i1] * q[i3] * q[i6]
            + (-4.458325e-04) * q[i1] * q[i3] * q[i7] + (-1.308695e-03) * q[i1] * q[i3] * q[i8] + (-7.588419e-04) * q[i1] * q[i3] * q[i9]
            + (3.247102e-04) * q[i1] * q[i3] * q[i10] + (8.214066e-04) * q[i1] * q[i3] * q[i11] + (2.298352e-04) * q[i1] * q[i3] * q[i12]
            + (2.560312e-05) * q[i1] * q[i3] * q[i15] + (5.096420e-05) * q[i1] * q[i3] * q[i16] + (7.963403e-06) * q[i1] * q[i3] * q[i19]
            + (6.092463e-04) * q[i1] * q[i3] * q[i20] + (7.005373e-04) * q[i1] * q[i3] * q[i21] + (2.812319e-04) * q[i1] * q[i3] * q[i22]
            + (-7.012161e-03) * q[i1] * q[i4] * q[i5] + (3.068551e-03) * q[i1] * q[i4] * q[i6] + (-1.615989e-02) * q[i1] * q[i4] * q[i7]
            + (-2.867031e-04) * q[i1] * q[i4] * q[i8] + (-3.326402e-03) * q[i1] * q[i4] * q[i9] + (1.165198e-02) * q[i1] * q[i4] * q[i10]
            + (3.772506e-04) * q[i1] * q[i4] * q[i11] + (-4.866641e-04) * q[i1] * q[i4] * q[i12] + (-4.366051e-03) * q[i1] * q[i4] * q[i15]
            + (-1.393776e-03) * q[i1] * q[i4] * q[i16] + (-5.003371e-04) * q[i1] * q[i4] * q[i19] + (-1.224390e-03) * q[i1] * q[i4] * q[i20]
            + (1.144783e-03) * q[i1] * q[i4] * q[i21] + (4.340912e-04) * q[i1] * q[i4] * q[i22] + (-4.758118e-04) * q[i1] * q[i5] * q[i6]
            + (2.736718e-03) * q[i1] * q[i5] * q[i7] + (-5.661351e-06) * q[i1] * q[i5] * q[i8] + (7.878519e-04) * q[i1] * q[i5] * q[i9]
            + (5.176418e-04) * q[i1] * q[i5] * q[i10] + (-1.469162e-03) * q[i1] * q[i5] * q[i11] + (1.040694e-03) * q[i1] * q[i5] * q[i12]
            + (-2.057310e-04) * q[i1] * q[i5] * q[i15] + (-7.129600e-04) * q[i1] * q[i5] * q[i16] + (-6.364053e-04) * q[i1] * q[i5] * q[i19]
            + (3.675125e-06) * q[i1] * q[i5] * q[i20] + (-6.627542e-04) * q[i1] * q[i5] * q[i21] + (5.078706e-04) * q[i1] * q[i5] * q[i22]
            + (-1.145363e-02) * q[i1] * q[i6] * q[i7] + (2.431201e-04) * q[i1] * q[i6] * q[i8] + (-9.416981e-05) * q[i1] * q[i6] * q[i9]
            + (-2.003019e-03) * q[i1] * q[i6] * q[i10] + (-1.167237e-04) * q[i1] * q[i6] * q[i11] + (1.400456e-04) * q[i1] * q[i6] * q[i12]
            + (-3.584953e-04) * q[i1] * q[i6] * q[i15] + (-6.491164e-04) * q[i1] * q[i6] * q[i16] + (-5.207828e-05) * q[i1] * q[i6] * q[i19]
            + (1.743489e-04) * q[i1] * q[i6] * q[i20] + (-5.344456e-04) * q[i1] * q[i6] * q[i21] + (-1.932415e-04) * q[i1] * q[i6] * q[i22]
            + (-1.377890e-03) * q[i1] * q[i7] * q[i8] + (-2.129486e-03) * q[i1] * q[i7] * q[i9] + (1.539917e-03) * q[i1] * q[i7] * q[i10]
            + (-1.131840e-03) * q[i1] * q[i7] * q[i11] + (-1.800183e-03) * q[i1] * q[i7] * q[i12] + (-1.062855e-03) * q[i1] * q[i7] * q[i15]
            + (7.710115e-05) * q[i1] * q[i7] * q[i16] + (-1.066193e-03) * q[i1] * q[i7] * q[i19] + (1.409631e-03) * q[i1] * q[i7] * q[i20]
            + (2.247653e-04) * q[i1] * q[i7] * q[i21] + (4.137149e-04) * q[i1] * q[i7] * q[i22] + (-1.719627e-04) * q[i1] * q[i8] * q[i9]
            + (-1.222720e-03) * q[i1] * q[i8] * q[i10] + (3.963438e-05) * q[i1] * q[i8] * q[i11] + (-5.528362e-04) * q[i1] * q[i8] * q[i12]
            + (-6.650239e-04) * q[i1] * q[i8] * q[i15] + (2.444242e-04) * q[i1] * q[i8] * q[i16] + (-6.624098e-04) * q[i1] * q[i8] * q[i19]
            + (-1.026070e-04) * q[i1] * q[i8] * q[i20] + (2.896848e-05) * q[i1] * q[i8] * q[i21] + (-1.338097e-04) * q[i1] * q[i8] * q[i22]
            + (-2.445345e-04) * q[i1] * q[i9] * q[i10] + (4.395493e-04) * q[i1] * q[i9] * q[i11] + (9.939138e-04) * q[i1] * q[i9] * q[i12]
            + (3.954244e-04) * q[i1] * q[i9] * q[i15] + (-3.384103e-04) * q[i1] * q[i9] * q[i16] + (-3.654445e-05) * q[i1] * q[i9] * q[i19]
            + (-3.530560e-05) * q[i1] * q[i9] * q[i20] + (-2.388953e-04) * q[i1] * q[i9] * q[i21] + (-5.476873e-05) * q[i1] * q[i9] * q[i22]
            + (-4.684288e-04) * q[i1] * q[i10] * q[i11] + (8.078761e-05) * q[i1] * q[i10] * q[i12] + (-3.101763e-04) * q[i1] * q[i10] * q[i15]
            + (3.223845e-04) * q[i1] * q[i10] * q[i16] + (-6.479919e-04) * q[i1] * q[i10] * q[i19] + (2.965278e-04) * q[i1] * q[i10] * q[i20]
            + (-1.044877e-04) * q[i1] * q[i10] * q[i21] + (-1.251050e-04) * q[i1] * q[i10] * q[i22] + (-1.051395e-04) * q[i1] * q[i11] * q[i12]
            + (-8.438281e-05) * q[i1] * q[i11] * q[i15] + (1.474048e-04) * q[i1] * q[i11] * q[i16] + (-1.130834e-05) * q[i1] * q[i11] * q[i19]
            + (-1.253673e-04) * q[i1] * q[i11] * q[i20] + (1.847853e-04) * q[i1] * q[i11] * q[i21] + (1.039093e-04) * q[i1] * q[i11] * q[i22]
            + (-3.490332e-04) * q[i1] * q[i12] * q[i15] + (1.179321e-04) * q[i1] * q[i12] * q[i16] + (1.497031e-04) * q[i1] * q[i12] * q[i19]
            + (2.454978e-04) * q[i1] * q[i12] * q[i20] + (-3.210245e-04) * q[i1] * q[i12] * q[i21] + (1.659558e-04) * q[i1] * q[i12] * q[i22]
            + (-2.649109e-04) * q[i1] * q[i15] * q[i16] + (3.470410e-04) * q[i1] * q[i15] * q[i19] + (5.617706e-04) * q[i1] * q[i15] * q[i20]
            + (6.902543e-04) * q[i1] * q[i15] * q[i21] + (-1.032416e-04) * q[i1] * q[i15] * q[i22] + (4.275607e-05) * q[i1] * q[i16] * q[i19]
            + (1.031505e-04) * q[i1] * q[i16] * q[i20] + (7.983962e-05) * q[i1] * q[i16] * q[i21] + (2.655944e-04) * q[i1] * q[i16] * q[i22]
            + (3.652302e-04) * q[i1] * q[i19] * q[i20] + (-2.098154e-04) * q[i1] * q[i19] * q[i21] + (1.805125e-04) * q[i1] * q[i19] * q[i22]
            + (6.865431e-06) * q[i1] * q[i20] * q[i21] + (-1.258166e-04) * q[i1] * q[i20] * q[i22] + (4.540418e-04) * q[i1] * q[i21] * q[i22]
            + (-2.037684e-06) * q[i2] * q[i3] * q[i4] + (8.190087e-03) * q[i2] * q[i3] * q[i5] + (-3.135415e-03) * q[i2] * q[i3] * q[i6]
            + (1.534195e-03) * q[i2] * q[i3] * q[i7] + (1.040342e-03) * q[i2] * q[i3] * q[i8] + (2.559771e-03) * q[i2] * q[i3] * q[i9]
            + (-1.802805e-04) * q[i2] * q[i3] * q[i10] + (4.280872e-04) * q[i2] * q[i3] * q[i11] + (1.319566e-03) * q[i2] * q[i3] * q[i12]
            + (7.515078e-04) * q[i2] * q[i3] * q[i15] + (-2.011711e-03) * q[i2] * q[i3] * q[i16] + (-1.015795e-03) * q[i2] * q[i3] * q[i19]
            + (-2.322656e-04) * q[i2] * q[i3] * q[i20] + (-7.376429e-04) * q[i2] * q[i3] * q[i21] + (1.911578e-04) * q[i2] * q[i3] * q[i22]
            + (-8.205304e-03) * q[i2] * q[i4] * q[i5] + (1.537182e-03) * q[i2] * q[i4] * q[i6] + (-3.158159e-03) * q[i2] * q[i4] * q[i7]
            + (1.062334e-03) * q[i2] * q[i4] * q[i8] + (-2.018845e-04) * q[i2] * q[i4] * q[i9] + (2.528187e-03) * q[i2] * q[i4] * q[i10]
            + (-1.337421e-03) * q[i2] * q[i4] * q[i11] + (-4.544515e-04) * q[i2] * q[i4] * q[i12] + (-1.998887e-03) * q[i2] * q[i4] * q[i15]
            + (7.697633e-04) * q[i2] * q[i4] * q[i16] + (-2.286704e-04) * q[i2] * q[i4] * q[i19] + (-1.016435e-03) * q[i2] * q[i4] * q[i20]
            + (-1.849737e-04) * q[i2] * q[i4] * q[i21] + (7.318842e-04) * q[i2] * q[i4] * q[i22] + (-2.330496e-03) * q[i2] * q[i5] * q[i6]
            + (-2.332766e-03) * q[i2] * q[i5] * q[i7] + (-1.009762e-02) * q[i2] * q[i5] * q[i8] + (5.265570e-03) * q[i2] * q[i5] * q[i9]
            + (5.205727e-03) * q[i2] * q[i5] * q[i10] + (1.832586e-03) * q[i2] * q[i5] * q[i11] + (-1.945874e-03) * q[i2] * q[i5] * q[i12]
            + (-6.397119e-03) * q[i2] * q[i5] * q[i15] + (-6.482519e-03) * q[i2] * q[i5] * q[i16] + (2.895049e-04) * q[i2] * q[i5] * q[i19]
            + (2.973489e-04) * q[i2] * q[i5] * q[i20] + (8.994120e-04) * q[i2] * q[i5] * q[i21] + (-9.153289e-04) * q[i2] * q[i5] * q[i22]
            + (-7.705269e-06) * q[i2] * q[i6] * q[i7] + (7.640783e-03) * q[i2] * q[i6] * q[i8] + (9.360254e-04) * q[i2] * q[i6] * q[i9]
            + (-9.274168e-04) * q[i2] * q[i6] * q[i10] + (-1.253649e-03) * q[i2] * q[i6] * q[i11] + (3.816430e-05) * q[i2] * q[i6] * q[i12]
            + (-6.280712e-04) * q[i2] * q[i6] * q[i15] + (4.216966e-04) * q[i2] * q[i6] * q[i16] + (-2.789274e-04) * q[i2] * q[i6] * q[i19]
            + (8.815289e-04) * q[i2] * q[i6] * q[i20] + (4.687912e-04) * q[i2] * q[i6] * q[i21] + (-7.638846e-05) * q[i2] * q[i6] * q[i22]
            + (-7.652251e-03) * q[i2] * q[i7] * q[i8] + (9.302299e-04) * q[i2] * q[i7] * q[i9] + (-9.301464e-04) * q[i2] * q[i7] * q[i10]
            + (5.285828e-05) * q[i2] * q[i7] * q[i11] + (-1.292311e-03) * q[i2] * q[i7] * q[i12] + (-4.168032e-04) * q[i2] * q[i7] * q[i15]
            + (6.447545e-04) * q[i2] * q[i7] * q[i16] + (-8.905544e-04) * q[i2] * q[i7] * q[i19] + (2.646359e-04) * q[i2] * q[i7] * q[i20]
            + (-7.990804e-05) * q[i2] * q[i7] * q[i21] + (4.671243e-04) * q[i2] * q[i7] * q[i22] + (2.822017e-03) * q[i2] * q[i8] * q[i9]
            + (-2.806527e-03) * q[i2] * q[i8] * q[i10] + (4.744048e-03) * q[i2] * q[i8] * q[i11] + (4.759668e-03) * q[i2] * q[i8] * q[i12]
            + (5.980369e-05) * q[i2] * q[i8] * q[i15] + (-8.999536e-05) * q[i2] * q[i8] * q[i16] + (-1.453997e-05) * q[i2] * q[i8] * q[i19]
            + (2.125620e-05) * q[i2] * q[i8] * q[i20] + (-2.847479e-04) * q[i2] * q[i8] * q[i21] + (-2.826212e-04) * q[i2] * q[i8] * q[i22]
            + (2.633729e-07) * q[i2] * q[i9] * q[i10] + (-2.112597e-04) * q[i2] * q[i9] * q[i11] + (2.393001e-04) * q[i2] * q[i9] * q[i12]
            + (2.405074e-04) * q[i2] * q[i9] * q[i15] + (-2.254603e-04) * q[i2] * q[i9] * q[i16] + (-3.864767e-04) * q[i2] * q[i9] * q[i19]
            + (-1.316529e-07) * q[i2] * q[i9] * q[i20] + (-2.489416e-04) * q[i2] * q[i9] * q[i21] + (-2.295709e-04) * q[i2] * q[i9] * q[i22]
            + (2.285840e-04) * q[i2] * q[i10] * q[i11] + (-2.200809e-04) * q[i2] * q[i10] * q[i12] + (2.278704e-04) * q[i2] * q[i10] * q[i15]
            + (-2.443248e-04) * q[i2] * q[i10] * q[i16] + (-5.623259e-06) * q[i2] * q[i10] * q[i19] + (3.850246e-04) * q[i2] * q[i10] * q[i20]
            + (-2.336341e-04) * q[i2] * q[i10] * q[i21] + (-2.438366e-04) * q[i2] * q[i10] * q[i22] + (4.690415e-06) * q[i2] * q[i11] * q[i12]
            + (-2.637985e-03) * q[i2] * q[i11] * q[i15] + (-6.450256e-04) * q[i2] * q[i11] * q[i16] + (8.071551e-04) * q[i2] * q[i11] * q[i19]
            + (-4.746056e-04) * q[i2] * q[i11] * q[i20] + (6.157953e-04) * q[i2] * q[i11] * q[i21] + (4.651000e-04) * q[i2] * q[i11] * q[i22]
            + (-6.507329e-04) * q[i2] * q[i12] * q[i15] + (-2.629174e-03) * q[i2] * q[i12] * q[i16] + (-4.714424e-04) * q[i2] * q[i12] * q[i19]
            + (7.961283e-04) * q[i2] * q[i12] * q[i20] + (-4.648714e-04) * q[i2] * q[i12] * q[i21] + (-6.117027e-04) * q[i2] * q[i12] * q[i22]
            + (-7.984367e-08) * q[i2] * q[i15] * q[i16] + (2.449223e-04) * q[i2] * q[i15] * q[i19] + (6.778326e-04) * q[i2] * q[i15] * q[i20]
            + (6.348229e-04) * q[i2] * q[i15] * q[i21] + (2.914068e-05) * q[i2] * q[i15] * q[i22] + (-6.885542e-04) * q[i2] * q[i16] * q[i19]
            + (-2.187713e-04) * q[i2] * q[i16] * q[i20] + (2.597040e-05) * q[i2] * q[i16] * q[i21] + (6.360450e-04) * q[i2] * q[i16] * q[i22]
            + (7.832015e-06) * q[i2] * q[i19] * q[i20] + (-7.995270e-04) * q[i2] * q[i19] * q[i21] + (2.315073e-05) * q[i2] * q[i19] * q[i22]
            + (2.628485e-05) * q[i2] * q[i20] * q[i21] + (-7.908005e-04) * q[i2] * q[i20] * q[i22] + (-2.930688e-06) * q[i2] * q[i21] * q[i22]
            + (8.074756e-06) * q[i3] * q[i4] * q[i5] + (-3.397184e-03) * q[i3] * q[i4] * q[i6] + (-3.358738e-03) * q[i3] * q[i4] * q[i7]
            + (-1.285209e-03) * q[i3] * q[i4] * q[i8] + (-1.819951e-03) * q[i3] * q[i4] * q[i9] + (-1.819332e-03) * q[i3] * q[i4] * q[i10]
            + (5.681483e-04) * q[i3] * q[i4] * q[i11] + (-6.056154e-04) * q[i3] * q[i4] * q[i12] + (-1.645833e-04) * q[i3] * q[i4] * q[i15]
            + (-1.473190e-04) * q[i3] * q[i4] * q[i16] + (1.669453e-03) * q[i3] * q[i4] * q[i19] + (1.654502e-03) * q[i3] * q[i4] * q[i20]
            + (-7.266920e-04) * q[i3] * q[i4] * q[i21] + (7.298847e-04) * q[i3] * q[i4] * q[i22] + (-6.396838e-03) * q[i3] * q[i5] * q[i6]
            + (2.099809e-03) * q[i3] * q[i5] * q[i7] + (-6.112289e-03) * q[i3] * q[i5] * q[i8] + (-1.097910e-03) * q[i3] * q[i5] * q[i9]
            + (-3.088242e-04) * q[i3] * q[i5] * q[i10] + (-1.929560e-03) * q[i3] * q[i5] * q[i11] + (-1.744299e-03) * q[i3] * q[i5] * q[i12]
            + (-1.171593e-03) * q[i3] * q[i5] * q[i15] + (1.103235e-03) * q[i3] * q[i5] * q[i16] + (1.788667e-04) * q[i3] * q[i5] * q[i19]
            + (-1.627474e-03) * q[i3] * q[i5] * q[i20] + (2.501059e-04) * q[i3] * q[i5] * q[i21] + (-1.042687e-03) * q[i3] * q[i5] * q[i22]
            + (1.714701e-03) * q[i3] * q[i6] * q[i7] + (4.703570e-04) * q[i3] * q[i6] * q[i8] + (1.244003e-03) * q[i3] * q[i6] * q[i9]
            + (-1.035583e-03) * q[i3] * q[i6] * q[i10] + (-1.374444e-03) * q[i3] * q[i6] * q[i11] + (-3.581392e-04) * q[i3] * q[i6] * q[i12]
            + (-6.172732e-04) * q[i3] * q[i6] * q[i15] + (7.018440e-04) * q[i3] * q[i6] * q[i16] + (6.625531e-04) * q[i3] * q[i6] * q[i19]
            + (2.179492e-04) * q[i3] * q[i6] * q[i20] + (-7.358712e-06) * q[i3] * q[i6] * q[i21] + (1.187888e-04) * q[i3] * q[i6] * q[i22]
            + (1.178482e-03) * q[i3] * q[i7] * q[i8] + (-1.221219e-03) * q[i3] * q[i7] * q[i9] + (-1.806832e-03) * q[i3] * q[i7] * q[i10]
            + (2.956707e-04) * q[i3] * q[i7] * q[i11] + (-1.233349e-04) * q[i3] * q[i7] * q[i12] + (7.007633e-04) * q[i3] * q[i7] * q[i15]
            + (-9.436389e-05) * q[i3] * q[i7] * q[i16] + (-9.165213e-05) * q[i3] * q[i7] * q[i19] + (4.630918e-04) * q[i3] * q[i7] * q[i20]
            + (-9.779688e-05) * q[i3] * q[i7] * q[i21] + (7.096689e-04) * q[i3] * q[i7] * q[i22] + (-1.304794e-03) * q[i3] * q[i8] * q[i9]
            + (7.952767e-04) * q[i3] * q[i8] * q[i10] + (1.033786e-03) * q[i3] * q[i8] * q[i11] + (8.022163e-04) * q[i3] * q[i8] * q[i12]
            + (-1.804215e-03) * q[i3] * q[i8] * q[i15] + (9.682878e-04) * q[i3] * q[i8] * q[i16] + (2.773115e-05) * q[i3] * q[i8] * q[i19]
            + (8.255406e-04) * q[i3] * q[i8] * q[i20] + (1.261761e-04) * q[i3] * q[i8] * q[i21] + (6.748273e-04) * q[i3] * q[i8] * q[i22]
            + (-4.577861e-04) * q[i3] * q[i9] * q[i10] + (2.531383e-04) * q[i3] * q[i9] * q[i11] + (5.033502e-04) * q[i3] * q[i9] * q[i12]
            + (-3.873931e-04) * q[i3] * q[i9] * q[i15] + (-6.382520e-04) * q[i3] * q[i9] * q[i16] + (1.814252e-04) * q[i3] * q[i9] * q[i19]
            + (-2.641220e-05) * q[i3] * q[i9] * q[i20] + (2.187432e-04) * q[i3] * q[i9] * q[i21] + (2.044403e-04) * q[i3] * q[i9] * q[i22]
            + (-8.193292e-04) * q[i3] * q[i10] * q[i11] + (3.973499e-04) * q[i3] * q[i10] * q[i12] + (-4.077110e-04) * q[i3] * q[i10] * q[i15]
            + (4.492608e-04) * q[i3] * q[i10] * q[i16] + (2.930732e-04) * q[i3] * q[i10] * q[i19] + (4.740704e-04) * q[i3] * q[i10] * q[i20]
            + (-8.036297e-05) * q[i3] * q[i10] * q[i21] + (-1.194938e-04) * q[i3] * q[i10] * q[i22] + (-5.889736e-04) * q[i3] * q[i11] * q[i12]
            + (-7.788329e-04) * q[i3] * q[i11] * q[i15] + (3.615160e-04) * q[i3] * q[i11] * q[i16] + (-6.880792e-04) * q[i3] * q[i11] * q[i19]
            + (-5.652306e-05) * q[i3] * q[i11] * q[i20] + (-8.779510e-04) * q[i3] * q[i11] * q[i21] + (-2.416514e-04) * q[i3] * q[i11] * q[i22]
            + (-8.186351e-05) * q[i3] * q[i12] * q[i15] + (-1.012331e-03) * q[i3] * q[i12] * q[i16] + (3.051342e-04) * q[i3] * q[i12] * q[i19]
            + (3.018533e-04) * q[i3] * q[i12] * q[i20] + (2.076512e-04) * q[i3] * q[i12] * q[i21] + (2.997615e-04) * q[i3] * q[i12] * q[i22]
            + (-1.571992e-04) * q[i3] * q[i15] * q[i16] + (-5.973864e-04) * q[i3] * q[i15] * q[i19] + (-2.824218e-04) * q[i3] * q[i15] * q[i20]
            + (-4.609274e-04) * q[i3] * q[i15] * q[i21] + (-3.611729e-04) * q[i3] * q[i15] * q[i22] + (-6.815973e-04) * q[i3] * q[i16] * q[i19]
            + (6.176811e-05) * q[i3] * q[i16] * q[i20] + (-1.660415e-05) * q[i3] * q[i16] * q[i21] + (-1.861034e-04) * q[i3] * q[i16] * q[i22]
            + (1.477329e-04) * q[i3] * q[i19] * q[i20] + (-1.040570e-04) * q[i3] * q[i19] * q[i21] + (4.657604e-04) * q[i3] * q[i19] * q[i22]
            + (-6.336138e-04) * q[i3] * q[i20] * q[i21] + (3.488995e-04) * q[i3] * q[i20] * q[i22] + (1.227214e-04) * q[i3] * q[i21] * q[i22]
            + (2.088206e-03) * q[i4] * q[i5] * q[i6] + (-6.389809e-03) * q[i4] * q[i5] * q[i7] + (-6.133572e-03) * q[i4] * q[i5] * q[i8]
            + (-3.236227e-04) * q[i4] * q[i5] * q[i9] + (-1.076212e-03) * q[i4] * q[i5] * q[i10] + (1.749562e-03) * q[i4] * q[i5] * q[i11]
            + (1.903223e-03) * q[i4] * q[i5] * q[i12] + (1.105931e-03) * q[i4] * q[i5] * q[i15] + (-1.172226e-03) * q[i4] * q[i5] * q[i16]
            + (-1.633060e-03) * q[i4] * q[i5] * q[i19] + (1.739405e-04) * q[i4] * q[i5] * q[i20] + (1.047099e-03) * q[i4] * q[i5] * q[i21]
            + (-2.370044e-04) * q[i4] * q[i5] * q[i22] + (-1.735335e-03) * q[i4] * q[i6] * q[i7] + (-1.170429e-03) * q[i4] * q[i6] * q[i8]
            + (1.819040e-03) * q[i4] * q[i6] * q[i9] + (1.206311e-03) * q[i4] * q[i6] * q[i10] + (-1.265048e-04) * q[i4] * q[i6] * q[i11]
            + (3.047392e-04) * q[i4] * q[i6] * q[i12] + (9.692188e-05) * q[i4] * q[i6] * q[i15] + (-7.136097e-04) * q[i4] * q[i6] * q[i16]
            + (-4.620975e-04) * q[i4] * q[i6] * q[i19] + (9.937660e-05) * q[i4] * q[i6] * q[i20] + (7.054472e-04) * q[i4] * q[i6] * q[i21]
            + (-1.034558e-04) * q[i4] * q[i6] * q[i22] + (-4.796848e-04) * q[i4] * q[i7] * q[i8] + (1.030994e-03) * q[i4] * q[i7] * q[i9]
            + (-1.225452e-03) * q[i4] * q[i7] * q[i10] + (-3.746156e-04) * q[i4] * q[i7] * q[i11] + (-1.389476e-03) * q[i4] * q[i7] * q[i12]
            + (-6.976053e-04) * q[i4] * q[i7] * q[i15] + (6.232834e-04) * q[i4] * q[i7] * q[i16] + (-2.170230e-04) * q[i4] * q[i7] * q[i19]
            + (-6.638632e-04) * q[i4] * q[i7] * q[i20] + (1.071998e-04) * q[i4] * q[i7] * q[i21] + (-5.496944e-06) * q[i4] * q[i7] * q[i22]
            + (-7.931301e-04) * q[i4] * q[i8] * q[i9] + (1.294842e-03) * q[i4] * q[i8] * q[i10] + (7.959779e-04) * q[i4] * q[i8] * q[i11]
            + (1.067223e-03) * q[i4] * q[i8] * q[i12] + (-9.611609e-04) * q[i4] * q[i8] * q[i15] + (1.829801e-03) * q[i4] * q[i8] * q[i16]
            + (-8.232891e-04) * q[i4] * q[i8] * q[i19] + (-3.856416e-05) * q[i4] * q[i8] * q[i20] + (6.825732e-04) * q[i4] * q[i8] * q[i21]
            + (1.317616e-04) * q[i4] * q[i8] * q[i22] + (4.573698e-04) * q[i4] * q[i9] * q[i10] + (3.937067e-04) * q[i4] * q[i9] * q[i11]
            + (-8.228550e-04) * q[i4] * q[i9] * q[i12] + (-4.485047e-04) * q[i4] * q[i9] * q[i15] + (3.978682e-04) * q[i4] * q[i9] * q[i16]
            + (-4.595379e-04) * q[i4] * q[i9] * q[i19] + (-2.947117e-04) * q[i4] * q[i9] * q[i20] + (-1.234635e-04) * q[i4] * q[i9] * q[i21]
            + (-8.447726e-05) * q[i4] * q[i9] * q[i22] + (4.898072e-04) * q[i4] * q[i10] * q[i11] + (2.566401e-04) * q[i4] * q[i10] * q[i12]
            + (6.345679e-04) * q[i4] * q[i10] * q[i15] + (3.787644e-04) * q[i4] * q[i10] * q[i16] + (2.490965e-05) * q[i4] * q[i10] * q[i19]
            + (-1.768820e-04) * q[i4] * q[i10] * q[i20] + (2.002999e-04) * q[i4] * q[i10] * q[i21] + (2.178710e-04) * q[i4] * q[i10] * q[i22]
            + (5.899136e-04) * q[i4] * q[i11] * q[i12] + (-1.003587e-03) * q[i4] * q[i11] * q[i15] + (-8.476519e-05) * q[i4] * q[i11] * q[i16]
            + (2.910371e-04) * q[i4] * q[i11] * q[i19] + (3.013094e-04) * q[i4] * q[i11] * q[i20] + (-3.079492e-04) * q[i4] * q[i11] * q[i21]
            + (-2.066231e-04) * q[i4] * q[i11] * q[i22] + (3.631967e-04) * q[i4] * q[i12] * q[i15] + (-7.855407e-04) * q[i4] * q[i12] * q[i16]
            + (-6.787080e-05) * q[i4] * q[i12] * q[i19] + (-6.834487e-04) * q[i4] * q[i12] * q[i20] + (2.436982e-04) * q[i4] * q[i12] * q[i21]
            + (8.872700e-04) * q[i4] * q[i12] * q[i22] + (1.578929e-04) * q[i4] * q[i15] * q[i16] + (-7.381783e-05) * q[i4] * q[i15] * q[i19]
            + (6.806984e-04) * q[i4] * q[i15] * q[i20] + (-1.828871e-04) * q[i4] * q[i15] * q[i21] + (-2.002160e-05) * q[i4] * q[i15] * q[i22]
            + (2.789459e-04) * q[i4] * q[i16] * q[i19] + (5.880863e-04) * q[i4] * q[i16] * q[i20] + (-3.602423e-04) * q[i4] * q[i16] * q[i21]
            + (-4.693425e-04) * q[i4] * q[i16] * q[i22] + (-1.476584e-04) * q[i4] * q[i19] * q[i20] + (3.561740e-04) * q[i4] * q[i19] * q[i21]
            + (-6.224446e-04) * q[i4] * q[i19] * q[i22] + (4.618507e-04) * q[i4] * q[i20] * q[i21] + (-1.077073e-04) * q[i4] * q[i20] * q[i22]
            + (-1.220856e-04) * q[i4] * q[i21] * q[i22] + (1.085024e-05) * q[i5] * q[i6] * q[i7] + (1.075929e-03) * q[i5] * q[i6] * q[i8]
            + (1.891117e-03) * q[i5] * q[i6] * q[i9] + (-2.309741e-03) * q[i5] * q[i6] * q[i10] + (5.856359e-04) * q[i5] * q[i6] * q[i11]
            + (-1.821301e-03) * q[i5] * q[i6] * q[i12] + (1.114467e-03) * q[i5] * q[i6] * q[i15] + (1.956003e-04) * q[i5] * q[i6] * q[i16]
            + (4.064740e-04) * q[i5] * q[i6] * q[i19] + (6.454644e-06) * q[i5] * q[i6] * q[i20] + (-1.304700e-04) * q[i5] * q[i6] * q[i21]
            + (1.018957e-04) * q[i5] * q[i6] * q[i22] + (-1.076203e-03) * q[i5] * q[i7] * q[i8] + (2.316417e-03) * q[i5] * q[i7] * q[i9]
            + (-1.875516e-03) * q[i5] * q[i7] * q[i10] + (-1.821218e-03) * q[i5] * q[i7] * q[i11] + (5.918864e-04) * q[i5] * q[i7] * q[i12]
            + (-1.913683e-04) * q[i5] * q[i7] * q[i15] + (-1.117954e-03) * q[i5] * q[i7] * q[i16] + (-2.409192e-05) * q[i5] * q[i7] * q[i19]
            + (-3.989730e-04) * q[i5] * q[i7] * q[i20] + (9.784063e-05) * q[i5] * q[i7] * q[i21] + (-1.198239e-04) * q[i5] * q[i7] * q[i22]
            + (2.015299e-03) * q[i5] * q[i8] * q[i9] + (-2.008047e-03) * q[i5] * q[i8] * q[i10] + (1.329208e-03) * q[i5] * q[i8] * q[i11]
            + (1.336449e-03) * q[i5] * q[i8] * q[i12] + (1.798920e-03) * q[i5] * q[i8] * q[i15] + (-1.834742e-03) * q[i5] * q[i8] * q[i16]
            + (-6.928540e-05) * q[i5] * q[i8] * q[i19] + (7.243788e-05) * q[i5] * q[i8] * q[i20] + (1.485025e-04) * q[i5] * q[i8] * q[i21]
            + (1.510359e-04) * q[i5] * q[i8] * q[i22] + (-3.608466e-07) * q[i5] * q[i9] * q[i10] + (-6.350464e-04) * q[i5] * q[i9] * q[i11]
            + (-4.927454e-04) * q[i5] * q[i9] * q[i12] + (9.078374e-04) * q[i5] * q[i9] * q[i15] + (-2.821584e-04) * q[i5] * q[i9] * q[i16]
            + (5.246723e-04) * q[i5] * q[i9] * q[i19] + (1.785773e-05) * q[i5] * q[i9] * q[i20] + (-5.585385e-04) * q[i5] * q[i9] * q[i21]
            + (-2.062318e-04) * q[i5] * q[i9] * q[i22] + (-4.946957e-04) * q[i5] * q[i10] * q[i11] + (-6.254464e-04) * q[i5] * q[i10] * q[i12]
            + (2.874019e-04) * q[i5] * q[i10] * q[i15] + (-9.036813e-04) * q[i5] * q[i10] * q[i16] + (-1.686565e-05) * q[i5] * q[i10] * q[i19]
            + (-5.345133e-04) * q[i5] * q[i10] * q[i20] + (-2.027584e-04) * q[i5] * q[i10] * q[i21] + (-5.498457e-04) * q[i5] * q[i10] * q[i22]
            + (1.301077e-05) * q[i5] * q[i11] * q[i12] + (-2.286334e-03) * q[i5] * q[i11] * q[i15] + (-1.740660e-04) * q[i5] * q[i11] * q[i16]
            + (-2.437107e-04) * q[i5] * q[i11] * q[i19] + (-2.532027e-04) * q[i5] * q[i11] * q[i20] + (-1.165125e-04) * q[i5] * q[i11] * q[i21]
            + (1.027526e-04) * q[i5] * q[i11] * q[i22] + (-1.615513e-04) * q[i5] * q[i12] * q[i15] + (-2.283075e-03) * q[i5] * q[i12] * q[i16]
            + (-2.503387e-04) * q[i5] * q[i12] * q[i19] + (-2.516082e-04) * q[i5] * q[i12] * q[i20] + (-1.002124e-04) * q[i5] * q[i12] * q[i21]
            + (1.310858e-04) * q[i5] * q[i12] * q[i22] + (-8.659480e-07) * q[i5] * q[i15] * q[i16] + (-7.873197e-04) * q[i5] * q[i15] * q[i19]
            + (-3.741474e-04) * q[i5] * q[i15] * q[i20] + (1.155737e-03) * q[i5] * q[i15] * q[i21] + (4.391866e-04) * q[i5] * q[i15] * q[i22]
            + (3.876892e-04) * q[i5] * q[i16] * q[i19] + (7.816045e-04) * q[i5] * q[i16] * q[i20] + (4.446953e-04) * q[i5] * q[i16] * q[i21]
            + (1.163210e-03) * q[i5] * q[i16] * q[i22] + (2.007867e-06) * q[i5] * q[i19] * q[i20] + (8.735954e-04) * q[i5] * q[i19] * q[i21]
            + (-1.434840e-05) * q[i5] * q[i19] * q[i22] + (-7.502974e-06) * q[i5] * q[i20] * q[i21] + (8.731083e-04) * q[i5] * q[i20] * q[i22]
            + (-4.321930e-06) * q[i5] * q[i21] * q[i22] + (-3.365718e-03) * q[i6] * q[i7] * q[i8] + (-2.538085e-04) * q[i6] * q[i7] * q[i9]
            + (-2.458370e-04) * q[i6] * q[i7] * q[i10] + (-3.422639e-04) * q[i6] * q[i7] * q[i11] + (3.447562e-04) * q[i6] * q[i7] * q[i12]
            + (4.058894e-04) * q[i6] * q[i7] * q[i15] + (4.062233e-04) * q[i6] * q[i7] * q[i16] + (7.244586e-04) * q[i6] * q[i7] * q[i19]
            + (7.125068e-04) * q[i6] * q[i7] * q[i20] + (-3.170226e-04) * q[i6] * q[i7] * q[i21] + (3.047756e-04) * q[i6] * q[i7] * q[i22]
            + (-4.515444e-04) * q[i6] * q[i8] * q[i9] + (-1.345699e-03) * q[i6] * q[i8] * q[i10] + (-3.970293e-04) * q[i6] * q[i8] * q[i11]
            + (-3.383890e-04) * q[i6] * q[i8] * q[i12] + (-6.477363e-06) * q[i6] * q[i8] * q[i15] + (-2.702556e-04) * q[i6] * q[i8] * q[i16]
            + (-4.789547e-05) * q[i6] * q[i8] * q[i19] + (8.448996e-05) * q[i6] * q[i8] * q[i20] + (9.735872e-05) * q[i6] * q[i8] * q[i21]
            + (1.362510e-04) * q[i6] * q[i8] * q[i22] + (-4.095732e-04) * q[i6] * q[i9] * q[i10] + (6.405275e-05) * q[i6] * q[i9] * q[i11]
            + (1.787217e-04) * q[i6] * q[i9] * q[i12] + (-5.165831e-04) * q[i6] * q[i9] * q[i15] + (-2.863813e-04) * q[i6] * q[i9] * q[i16]
            + (7.655790e-05) * q[i6] * q[i9] * q[i19] + (2.288140e-04) * q[i6] * q[i9] * q[i20] + (-4.923126e-04) * q[i6] * q[i9] * q[i21]
            + (-7.035101e-05) * q[i6] * q[i9] * q[i22] + (-3.145454e-04) * q[i6] * q[i10] * q[i11] + (-1.802055e-04) * q[i6] * q[i10] * q[i12]
            + (-4.340690e-05) * q[i6] * q[i10] * q[i15] + (5.037950e-04) * q[i6] * q[i10] * q[i16] + (-1.027714e-04) * q[i6] * q[i10] * q[i19]
            + (3.982749e-04) * q[i6] * q[i10] * q[i20] + (-1.984213e-04) * q[i6] * q[i10] * q[i21] + (1.941054e-04) * q[i6] * q[i10] * q[i22]
            + (-4.465450e-05) * q[i6] * q[i11] * q[i12] + (-3.946318e-05) * q[i6] * q[i11] * q[i15] + (-2.434706e-05) * q[i6] * q[i11] * q[i16]
            + (-1.572757e-04) * q[i6] * q[i11] * q[i19] + (2.184652e-04) * q[i6] * q[i11] * q[i20] + (-6.219832e-05) * q[i6] * q[i11] * q[i21]
            + (-1.467622e-04) * q[i6] * q[i11] * q[i22] + (-7.233044e-05) * q[i6] * q[i12] * q[i15] + (-1.767328e-04) * q[i6] * q[i12] * q[i16]
            + (-2.336362e-04) * q[i6] * q[i12] * q[i19] + (1.541868e-04) * q[i6] * q[i12] * q[i20] + (2.803087e-04) * q[i6] * q[i12] * q[i21]
            + (-2.579998e-04) * q[i6] * q[i12] * q[i22] + (-6.177311e-05) * q[i6] * q[i15] * q[i16] + (-4.611163e-04) * q[i6] * q[i15] * q[i19]
            + (-3.996412e-04) * q[i6] * q[i15] * q[i20] + (7.036249e-05) * q[i6] * q[i15] * q[i21] + (7.993119e-05) * q[i6] * q[i15] * q[i22]
            + (4.438298e-04) * q[i6] * q[i16] * q[i19] + (1.861985e-04) * q[i6] * q[i16] * q[i20] + (-4.163240e-05) * q[i6] * q[i16] * q[i21]
            + (-5.419059e-04) * q[i6] * q[i16] * q[i22] + (-3.501248e-04) * q[i6] * q[i19] * q[i20] + (4.017529e-04) * q[i6] * q[i19] * q[i21]
            + (-1.984777e-04) * q[i6] * q[i19] * q[i22] + (5.300866e-05) * q[i6] * q[i20] * q[i21] + (2.983694e-06) * q[i6] * q[i20] * q[i22]
            + (1.353639e-04) * q[i6] * q[i21] * q[i22] + (-1.356350e-03) * q[i7] * q[i8] * q[i9] + (-4.340547e-04) * q[i7] * q[i8] * q[i10]
            + (3.746026e-04) * q[i7] * q[i8] * q[i11] + (3.996764e-04) * q[i7] * q[i8] * q[i12] + (-2.735794e-04) * q[i7] * q[i8] * q[i15]
            + (-7.646881e-06) * q[i7] * q[i8] * q[i16] + (8.224123e-05) * q[i7] * q[i8] * q[i19] + (-3.455847e-05) * q[i7] * q[i8] * q[i20]
            + (-1.377254e-04) * q[i7] * q[i8] * q[i21] + (-8.658386e-05) * q[i7] * q[i8] * q[i22] + (-4.172767e-04) * q[i7] * q[i9] * q[i10]
            + (1.817600e-04) * q[i7] * q[i9] * q[i11] + (3.201946e-04) * q[i7] * q[i9] * q[i12] + (5.054034e-04) * q[i7] * q[i9] * q[i15]
            + (-3.701206e-05) * q[i7] * q[i9] * q[i16] + (4.038065e-04) * q[i7] * q[i9] * q[i19] + (-1.078364e-04) * q[i7] * q[i9] * q[i20]
            + (-1.936129e-04) * q[i7] * q[i9] * q[i21] + (1.932345e-04) * q[i7] * q[i9] * q[i22] + (-1.821436e-04) * q[i7] * q[i10] * q[i11]
            + (-7.359981e-05) * q[i7] * q[i10] * q[i12] + (-2.776757e-04) * q[i7] * q[i10] * q[i15] + (-5.209704e-04) * q[i7] * q[i10] * q[i16]
            + (2.316224e-04) * q[i7] * q[i10] * q[i19] + (7.333906e-05) * q[i7] * q[i10] * q[i20] + (7.155125e-05) * q[i7] * q[i10] * q[i21]
            + (4.919634e-04) * q[i7] * q[i10] * q[i22] + (-5.278493e-05) * q[i7] * q[i11] * q[i12] + (1.699183e-04) * q[i7] * q[i11] * q[i15]
            + (7.544601e-05) * q[i7] * q[i11] * q[i16] + (-1.569050e-04) * q[i7] * q[i11] * q[i19] + (2.369777e-04) * q[i7] * q[i11] * q[i20]
            + (-2.603405e-04) * q[i7] * q[i11] * q[i21] + (2.806518e-04) * q[i7] * q[i11] * q[i22] + (2.746136e-05) * q[i7] * q[i12] * q[i15]
            + (4.293758e-05) * q[i7] * q[i12] * q[i16] + (-2.168496e-04) * q[i7] * q[i12] * q[i19] + (1.536198e-04) * q[i7] * q[i12] * q[i20]
            + (-1.445232e-04) * q[i7] * q[i12] * q[i21] + (-5.375323e-05) * q[i7] * q[i12] * q[i22] + (-6.415069e-05) * q[i7] * q[i15] * q[i16]
            + (1.873814e-04) * q[i7] * q[i15] * q[i19] + (4.450782e-04) * q[i7] * q[i15] * q[i20] + (5.336785e-04) * q[i7] * q[i15] * q[i21]
            + (4.000195e-05) * q[i7] * q[i15] * q[i22] + (-3.961685e-04) * q[i7] * q[i16] * q[i19] + (-4.551602e-04) * q[i7] * q[i16] * q[i20]
            + (-7.387613e-05) * q[i7] * q[i16] * q[i21] + (-6.504092e-05) * q[i7] * q[i16] * q[i22] + (-3.432400e-04) * q[i7] * q[i19] * q[i20]
            + (-3.889145e-06) * q[i7] * q[i19] * q[i21] + (-5.439509e-05) * q[i7] * q[i19] * q[i22] + (2.009673e-04) * q[i7] * q[i20] * q[i21]
            + (-3.973034e-04) * q[i7] * q[i20] * q[i22] + (1.395011e-04) * q[i7] * q[i21] * q[i22] + (-1.062765e-04) * q[i8] * q[i9] * q[i10]
            + (-3.790616e-04) * q[i8] * q[i9] * q[i11] + (-2.503545e-04) * q[i8] * q[i9] * q[i12] + (-5.608626e-04) * q[i8] * q[i9] * q[i15]
            + (2.161125e-05) * q[i8] * q[i9] * q[i16] + (-2.332290e-04) * q[i8] * q[i9] * q[i19] + (2.934882e-04) * q[i8] * q[i9] * q[i20]
            + (1.329135e-04) * q[i8] * q[i9] * q[i21] + (-1.614691e-04) * q[i8] * q[i9] * q[i22] + (2.480893e-04) * q[i8] * q[i10] * q[i11]
            + (3.753363e-04) * q[i8] * q[i10] * q[i12] + (2.855460e-05) * q[i8] * q[i10] * q[i15] + (-5.650428e-04) * q[i8] * q[i10] * q[i16]
            + (2.962574e-04) * q[i8] * q[i10] * q[i19] + (-2.234525e-04) * q[i8] * q[i10] * q[i20] + (1.593766e-04) * q[i8] * q[i10] * q[i21]
            + (-1.333644e-04) * q[i8] * q[i10] * q[i22] + (-5.578371e-04) * q[i8] * q[i11] * q[i12] + (-7.542863e-04) * q[i8] * q[i11] * q[i15]
            + (3.434920e-05) * q[i8] * q[i11] * q[i16] + (4.733749e-04) * q[i8] * q[i11] * q[i19] + (5.237842e-04) * q[i8] * q[i11] * q[i20]
            + (-1.172496e-04) * q[i8] * q[i11] * q[i21] + (1.615886e-06) * q[i8] * q[i11] * q[i22] + (-4.209219e-05) * q[i8] * q[i12] * q[i15]
            + (7.420413e-04) * q[i8] * q[i12] * q[i16] + (-5.292212e-04) * q[i8] * q[i12] * q[i19] + (-4.641388e-04) * q[i8] * q[i12] * q[i20]
            + (4.989940e-06) * q[i8] * q[i12] * q[i21] + (-1.251570e-04) * q[i8] * q[i12] * q[i22] + (-3.166315e-04) * q[i8] * q[i15] * q[i16]
            + (4.024530e-04) * q[i8] * q[i15] * q[i19] + (-6.814829e-05) * q[i8] * q[i15] * q[i20] + (-1.336037e-03) * q[i8] * q[i15] * q[i21]
            + (8.554276e-05) * q[i8] * q[i15] * q[i22] + (-7.197157e-05) * q[i8] * q[i16] * q[i19] + (4.059137e-04) * q[i8] * q[i16] * q[i20]
            + (-8.746154e-05) * q[i8] * q[i16] * q[i21] + (1.347250e-03) * q[i8] * q[i16] * q[i22] + (5.735726e-04) * q[i8] * q[i19] * q[i20]
            + (-7.246433e-04) * q[i8] * q[i19] * q[i21] + (-2.846715e-04) * q[i8] * q[i19] * q[i22] + (2.859612e-04) * q[i8] * q[i20] * q[i21]
            + (7.191502e-04) * q[i8] * q[i20] * q[i22] + (-9.113027e-05) * q[i8] * q[i21] * q[i22] + (8.938416e-05) * q[i9] * q[i10] * q[i11]
            + (-8.833383e-05) * q[i9] * q[i10] * q[i12] + (1.485611e-04) * q[i9] * q[i10] * q[i15] + (1.512407e-04) * q[i9] * q[i10] * q[i16]
            + (-3.864186e-05) * q[i9] * q[i10] * q[i19] + (-3.865974e-05) * q[i9] * q[i10] * q[i20] + (-5.422781e-05) * q[i9] * q[i10] * q[i21]
            + (5.440258e-05) * q[i9] * q[i10] * q[i22] + (3.372127e-04) * q[i9] * q[i11] * q[i12] + (3.807597e-05) * q[i9] * q[i11] * q[i15]
            + (-2.562664e-04) * q[i9] * q[i11] * q[i16] + (-2.355869e-04) * q[i9] * q[i11] * q[i19] + (4.099562e-04) * q[i9] * q[i11] * q[i20]
            + (-1.556964e-04) * q[i9] * q[i11] * q[i21] + (2.365417e-05) * q[i9] * q[i11] * q[i22] + (2.298135e-04) * q[i9] * q[i12] * q[i15]
            + (4.424267e-04) * q[i9] * q[i12] * q[i16] + (1.301948e-04) * q[i9] * q[i12] * q[i19] + (2.181249e-04) * q[i9] * q[i12] * q[i20]
            + (-1.273016e-04) * q[i9] * q[i12] * q[i21] + (6.292190e-06) * q[i9] * q[i12] * q[i22] + (1.216782e-04) * q[i9] * q[i15] * q[i16]
            + (-2.019897e-04) * q[i9] * q[i15] * q[i19] + (5.364623e-05) * q[i9] * q[i15] * q[i20] + (6.224095e-05) * q[i9] * q[i15] * q[i21]
            + (5.498908e-05) * q[i9] * q[i15] * q[i22] + (2.454538e-04) * q[i9] * q[i16] * q[i19] + (7.094283e-05) * q[i9] * q[i16] * q[i20]
            + (6.556024e-05) * q[i9] * q[i16] * q[i21] + (-1.441560e-04) * q[i9] * q[i16] * q[i22] + (4.018498e-05) * q[i9] * q[i19] * q[i20]
            + (7.119420e-05) * q[i9] * q[i19] * q[i21] + (5.045095e-06) * q[i9] * q[i19] * q[i22] + (2.715758e-05) * q[i9] * q[i20] * q[i21]
            + (-9.038603e-05) * q[i9] * q[i20] * q[i22] + (4.638653e-05) * q[i9] * q[i21] * q[i22] + (3.405033e-04) * q[i10] * q[i11] * q[i12]
            + (-4.379871e-04) * q[i10] * q[i11] * q[i15] + (-2.290909e-04) * q[i10] * q[i11] * q[i16] + (-2.225971e-04) * q[i10] * q[i11] * q[i19]
            + (-1.278631e-04) * q[i10] * q[i11] * q[i20] + (2.454380e-06) * q[i10] * q[i11] * q[i21] + (-1.242732e-04) * q[i10] * q[i11] * q[i22]
            + (2.620049e-04) * q[i10] * q[i12] * q[i15] + (-3.719982e-05) * q[i10] * q[i12] * q[i16] + (-4.135810e-04) * q[i10] * q[i12] * q[i19]
            + (2.268484e-04) * q[i10] * q[i12] * q[i20] + (2.327850e-05) * q[i10] * q[i12] * q[i21] + (-1.499846e-04) * q[i10] * q[i12] * q[i22]
            + (1.207736e-04) * q[i10] * q[i15] * q[i16] + (6.720933e-05) * q[i10] * q[i15] * q[i19] + (2.440403e-04) * q[i10] * q[i15] * q[i20]
            + (1.405013e-04) * q[i10] * q[i15] * q[i21] + (-6.600514e-05) * q[i10] * q[i15] * q[i22] + (5.760996e-05) * q[i10] * q[i16] * q[i19]
            + (-2.040246e-04) * q[i10] * q[i16] * q[i20] + (-5.506688e-05) * q[i10] * q[i16] * q[i21] + (-6.104315e-05) * q[i10] * q[i16] * q[i22]
            + (4.563640e-05) * q[i10] * q[i19] * q[i20] + (9.115209e-05) * q[i10] * q[i19] * q[i21] + (-2.845212e-05) * q[i10] * q[i19] * q[i22]
            + (-2.515620e-06) * q[i10] * q[i20] * q[i21] + (-6.493948e-05) * q[i10] * q[i20] * q[i22] + (4.477543e-05) * q[i10] * q[i21] * q[i22]
            + (2.104712e-04) * q[i11] * q[i12] * q[i15] + (2.136035e-04) * q[i11] * q[i12] * q[i16] + (-2.892577e-04) * q[i11] * q[i12] * q[i19]
            + (-2.910180e-04) * q[i11] * q[i12] * q[i20] + (-2.512629e-05) * q[i11] * q[i12] * q[i21] + (2.919643e-05) * q[i11] * q[i12] * q[i22]
            + (1.670981e-04) * q[i11] * q[i15] * q[i16] + (-7.325050e-05) * q[i11] * q[i15] * q[i19] + (-1.247904e-04) * q[i11] * q[i15] * q[i20]
            + (-1.182786e-04) * q[i11] * q[i15] * q[i21] + (1.123203e-04) * q[i11] * q[i15] * q[i22] + (-1.722355e-04) * q[i11] * q[i16] * q[i19]
            + (-2.718841e-04) * q[i11] * q[i16] * q[i20] + (8.398881e-05) * q[i11] * q[i16] * q[i21] + (-1.625631e-04) * q[i11] * q[i16] * q[i22]
            + (1.578352e-04) * q[i11] * q[i19] * q[i20] + (9.781332e-05) * q[i11] * q[i19] * q[i21] + (-1.896632e-05) * q[i11] * q[i19] * q[i22]
            + (-7.191706e-05) * q[i11] * q[i20] * q[i21] + (-1.398552e-04) * q[i11] * q[i20] * q[i22] + (1.085750e-04) * q[i11] * q[i21] * q[i22]
            + (-1.695531e-04) * q[i12] * q[i15] * q[i16] + (2.719574e-04) * q[i12] * q[i15] * q[i19] + (1.686572e-04) * q[i12] * q[i15] * q[i20]
            + (-1.579868e-04) * q[i12] * q[i15] * q[i21] + (8.610818e-05) * q[i12] * q[i15] * q[i22] + (1.297550e-04) * q[i12] * q[i16] * q[i19]
            + (7.923498e-05) * q[i12] * q[i16] * q[i20] + (1.141716e-04) * q[i12] * q[i16] * q[i21] + (-1.321650e-04) * q[i12] * q[i16] * q[i22]
            + (-1.579112e-04) * q[i12] * q[i19] * q[i20] + (-1.418660e-04) * q[i12] * q[i19] * q[i21] + (-7.484854e-05) * q[i12] * q[i19] * q[i22]
            + (-2.069570e-05) * q[i12] * q[i20] * q[i21] + (1.036630e-04) * q[i12] * q[i20] * q[i22] + (-1.105546e-04) * q[i12] * q[i21] * q[i22]
            + (2.415881e-04) * q[i15] * q[i16] * q[i19] + (2.378795e-04) * q[i15] * q[i16] * q[i20] + (5.657996e-05) * q[i15] * q[i16] * q[i21]
            + (-5.735289e-05) * q[i15] * q[i16] * q[i22] + (1.267131e-04) * q[i15] * q[i19] * q[i20] + (-9.391578e-04) * q[i15] * q[i19] * q[i21]
            + (-1.083471e-04) * q[i15] * q[i19] * q[i22] + (4.373890e-05) * q[i15] * q[i20] * q[i21] + (-9.581062e-05) * q[i15] * q[i20] * q[i22]
            + (-3.576275e-05) * q[i15] * q[i21] * q[i22] + (1.244008e-04) * q[i16] * q[i19] * q[i20] + (9.520474e-05) * q[i16] * q[i19] * q[i21]
            + (-4.180483e-05) * q[i16] * q[i19] * q[i22] + (1.104112e-04) * q[i16] * q[i20] * q[i21] + (9.323239e-04) * q[i16] * q[i20] * q[i22]
            + (-3.703898e-05) * q[i16] * q[i21] * q[i22] + (1.039629e-04) * q[i19] * q[i20] * q[i21] + (-1.062036e-04) * q[i19] * q[i20] * q[i22]
            + (1.894616e-05) * q[i19] * q[i21] * q[i22] + (1.709314e-05) * q[i20] * q[i21] * q[i22];
      return Qy;
   }

   public double getQz(double[] q)
   {
      double Qz;
      Qz = (-1.155305e-01) * q[i0] + (-1.150095e-01) * q[i1] + (2.023910e-01) * q[i2] + (1.933653e-02) * q[i3] + (1.929621e-02) * q[i4]
            + (-1.727993e-02) * q[i5] + (8.482997e-02) * q[i6] + (-8.463776e-02) * q[i7] + (-1.984033e-04) * q[i8] + (3.090555e-02) * q[i9]
            + (-3.070071e-02) * q[i10] + (4.625223e-02) * q[i11] + (4.647797e-02) * q[i12] + (1.086699e-02) * q[i15] + (-1.112881e-02) * q[i16]
            + (1.147939e-02) * q[i19] + (-1.124376e-02) * q[i20] + (6.412442e-03) * q[i21] + (6.439572e-03) * q[i22] + (5.784739e-03) * q[i0] * q[i0]
            + (-5.709468e-03) * q[i1] * q[i1] + (8.432394e-06) * q[i2] * q[i2] + (-4.255974e-03) * q[i3] * q[i3] + (4.233750e-03) * q[i4] * q[i4]
            + (8.832997e-07) * q[i5] * q[i5] + (-2.480046e-03) * q[i6] * q[i6] + (2.461622e-03) * q[i7] * q[i7] + (8.717667e-05) * q[i8] * q[i8]
            + (-2.102622e-03) * q[i9] * q[i9] + (2.094341e-03) * q[i10] * q[i10] + (-6.034145e-04) * q[i11] * q[i11] + (8.135552e-04) * q[i12] * q[i12]
            + (3.708246e-03) * q[i15] * q[i15] + (-3.722115e-03) * q[i16] * q[i16] + (-1.825673e-03) * q[i19] * q[i19] + (1.814904e-03) * q[i20] * q[i20]
            + (6.871610e-04) * q[i21] * q[i21] + (-7.022029e-04) * q[i22] * q[i22] + (2.036691e-05) * q[i0] * q[i1] + (1.047292e-03) * q[i0] * q[i2]
            + (-6.074749e-02) * q[i0] * q[i3] + (-4.186895e-02) * q[i0] * q[i4] + (-4.252679e-03) * q[i0] * q[i5] + (2.489028e-02) * q[i0] * q[i6]
            + (-9.974459e-03) * q[i0] * q[i7] + (8.124199e-03) * q[i0] * q[i8] + (-2.182358e-03) * q[i0] * q[i9] + (-5.529312e-04) * q[i0] * q[i10]
            + (-7.457680e-04) * q[i0] * q[i11] + (-3.888310e-03) * q[i0] * q[i12] + (-7.918036e-03) * q[i0] * q[i15] + (-1.004595e-02) * q[i0] * q[i16]
            + (1.977376e-03) * q[i0] * q[i19] + (-2.009966e-03) * q[i0] * q[i20] + (-3.807450e-03) * q[i0] * q[i21] + (-3.142940e-04) * q[i0] * q[i22]
            + (-9.686204e-04) * q[i1] * q[i2] + (4.182345e-02) * q[i1] * q[i3] + (6.077392e-02) * q[i1] * q[i4] + (4.187169e-03) * q[i1] * q[i5]
            + (-1.000115e-02) * q[i1] * q[i6] + (2.483707e-02) * q[i1] * q[i7] + (8.120267e-03) * q[i1] * q[i8] + (-5.844495e-04) * q[i1] * q[i9]
            + (-2.087825e-03) * q[i1] * q[i10] + (4.029661e-03) * q[i1] * q[i11] + (7.970926e-04) * q[i1] * q[i12] + (-9.929541e-03) * q[i1] * q[i15]
            + (-8.015240e-03) * q[i1] * q[i16] + (-1.988935e-03) * q[i1] * q[i19] + (1.956409e-03) * q[i1] * q[i20] + (2.742735e-04) * q[i1] * q[i21]
            + (3.725649e-03) * q[i1] * q[i22] + (-1.580835e-02) * q[i2] * q[i3] + (1.576084e-02) * q[i2] * q[i4] + (-6.851522e-05) * q[i2] * q[i5]
            + (1.301239e-02) * q[i2] * q[i6] + (1.298224e-02) * q[i2] * q[i7] + (9.419529e-03) * q[i2] * q[i8] + (5.744760e-05) * q[i2] * q[i9]
            + (1.044127e-04) * q[i2] * q[i10] + (3.681198e-03) * q[i2] * q[i11] + (-3.495712e-03) * q[i2] * q[i12] + (-2.036958e-02) * q[i2] * q[i15]
            + (-2.057607e-02) * q[i2] * q[i16] + (2.362708e-03) * q[i2] * q[i19] + (2.297146e-03) * q[i2] * q[i20] + (-1.734093e-03) * q[i2] * q[i21]
            + (1.651024e-03) * q[i2] * q[i22] + (-4.628329e-05) * q[i3] * q[i4] + (-3.369455e-03) * q[i3] * q[i5] + (1.660043e-02) * q[i3] * q[i6]
            + (2.155436e-02) * q[i3] * q[i7] + (-4.288859e-03) * q[i3] * q[i8] + (1.400853e-03) * q[i3] * q[i9] + (1.098379e-02) * q[i3] * q[i10]
            + (4.454410e-03) * q[i3] * q[i11] + (1.894654e-03) * q[i3] * q[i12] + (4.513642e-03) * q[i3] * q[i15] + (9.240801e-04) * q[i3] * q[i16]
            + (-1.019916e-02) * q[i3] * q[i19] + (1.230024e-02) * q[i3] * q[i20] + (4.898164e-03) * q[i3] * q[i21] + (-2.220548e-03) * q[i3] * q[i22]
            + (3.369217e-03) * q[i4] * q[i5] + (2.152863e-02) * q[i4] * q[i6] + (1.642073e-02) * q[i4] * q[i7] + (-4.240503e-03) * q[i4] * q[i8]
            + (1.108682e-02) * q[i4] * q[i9] + (1.371122e-03) * q[i4] * q[i10] + (-1.862121e-03) * q[i4] * q[i11] + (-4.441355e-03) * q[i4] * q[i12]
            + (8.733209e-04) * q[i4] * q[i15] + (4.485459e-03) * q[i4] * q[i16] + (1.228612e-02) * q[i4] * q[i19] + (-1.017810e-02) * q[i4] * q[i20]
            + (2.181625e-03) * q[i4] * q[i21] + (-4.804226e-03) * q[i4] * q[i22] + (2.376484e-02) * q[i5] * q[i6] + (2.352113e-02) * q[i5] * q[i7]
            + (4.538288e-02) * q[i5] * q[i8] + (8.665537e-03) * q[i5] * q[i9] + (8.572595e-03) * q[i5] * q[i10] + (-4.070402e-03) * q[i5] * q[i11]
            + (4.230532e-03) * q[i5] * q[i12] + (-3.330178e-03) * q[i5] * q[i15] + (-3.352356e-03) * q[i5] * q[i16] + (4.195663e-03) * q[i5] * q[i19]
            + (4.206466e-03) * q[i5] * q[i20] + (-3.430954e-03) * q[i5] * q[i21] + (3.408243e-03) * q[i5] * q[i22] + (-5.543573e-06) * q[i6] * q[i7]
            + (-8.516581e-03) * q[i6] * q[i8] + (-2.719946e-03) * q[i6] * q[i9] + (3.286257e-03) * q[i6] * q[i10] + (7.732070e-04) * q[i6] * q[i11]
            + (4.117008e-03) * q[i6] * q[i12] + (-9.829345e-05) * q[i6] * q[i15] + (9.207379e-03) * q[i6] * q[i16] + (-3.224428e-03) * q[i6] * q[i19]
            + (-1.961197e-03) * q[i6] * q[i20] + (6.853790e-04) * q[i6] * q[i21] + (7.736524e-04) * q[i6] * q[i22] + (8.428740e-03) * q[i7] * q[i8]
            + (-3.306397e-03) * q[i7] * q[i9] + (2.648012e-03) * q[i7] * q[i10] + (4.191025e-03) * q[i7] * q[i11] + (8.121325e-04) * q[i7] * q[i12]
            + (-9.059258e-03) * q[i7] * q[i15] + (5.056939e-05) * q[i7] * q[i16] + (1.962425e-03) * q[i7] * q[i19] + (3.172150e-03) * q[i7] * q[i20]
            + (7.724307e-04) * q[i7] * q[i21] + (6.615569e-04) * q[i7] * q[i22] + (-1.762728e-03) * q[i8] * q[i9] + (1.709825e-03) * q[i8] * q[i10]
            + (-2.119828e-03) * q[i8] * q[i11] + (-2.327962e-03) * q[i8] * q[i12] + (1.183765e-02) * q[i8] * q[i15] + (-1.188977e-02) * q[i8] * q[i16]
            + (3.065881e-03) * q[i8] * q[i19] + (-3.045078e-03) * q[i8] * q[i20] + (1.498093e-03) * q[i8] * q[i21] + (1.520628e-03) * q[i8] * q[i22]
            + (4.878573e-06) * q[i9] * q[i10] + (2.194359e-04) * q[i9] * q[i11] + (1.848397e-03) * q[i9] * q[i12] + (-3.109319e-03) * q[i9] * q[i15]
            + (1.225156e-03) * q[i9] * q[i16] + (-1.853988e-03) * q[i9] * q[i19] + (-1.021110e-03) * q[i9] * q[i20] + (7.322895e-04) * q[i9] * q[i21]
            + (1.643148e-03) * q[i9] * q[i22] + (1.839012e-03) * q[i10] * q[i11] + (2.560436e-04) * q[i10] * q[i12] + (-1.183486e-03) * q[i10] * q[i15]
            + (3.101136e-03) * q[i10] * q[i16] + (1.017924e-03) * q[i10] * q[i19] + (1.831189e-03) * q[i10] * q[i20] + (1.629888e-03) * q[i10] * q[i21]
            + (7.292084e-04) * q[i10] * q[i22] + (-3.334952e-05) * q[i11] * q[i12] + (7.124364e-04) * q[i11] * q[i15] + (1.153869e-03) * q[i11] * q[i16]
            + (3.057393e-03) * q[i11] * q[i19] + (3.689554e-04) * q[i11] * q[i20] + (5.455647e-03) * q[i11] * q[i21] + (9.597047e-04) * q[i11] * q[i22]
            + (1.139107e-03) * q[i12] * q[i15] + (6.336331e-04) * q[i12] * q[i16] + (3.944362e-04) * q[i12] * q[i19] + (3.032000e-03) * q[i12] * q[i20]
            + (-9.732808e-04) * q[i12] * q[i21] + (-5.564965e-03) * q[i12] * q[i22] + (9.802463e-07) * q[i15] * q[i16] + (5.300136e-03) * q[i15] * q[i19]
            + (-1.283342e-03) * q[i15] * q[i20] + (3.836362e-03) * q[i15] * q[i21] + (-8.313458e-05) * q[i15] * q[i22] + (1.305496e-03) * q[i16] * q[i19]
            + (-5.222701e-03) * q[i16] * q[i20] + (-8.271314e-05) * q[i16] * q[i21] + (3.876542e-03) * q[i16] * q[i22] + (-1.310761e-06) * q[i19] * q[i20]
            + (-5.192123e-03) * q[i19] * q[i21] + (-1.044434e-03) * q[i19] * q[i22] + (-1.033220e-03) * q[i20] * q[i21] + (-5.224340e-03) * q[i20] * q[i22]
            + (-5.880013e-06) * q[i21] * q[i22] + (4.351645e-03) * q[i0] * q[i0] * q[i0] + (-7.216278e-04) * q[i0] * q[i0] * q[i1]
            + (-1.101905e-03) * q[i0] * q[i0] * q[i2] + (-6.938060e-03) * q[i0] * q[i0] * q[i3] + (-5.348986e-04) * q[i0] * q[i0] * q[i4]
            + (1.705448e-04) * q[i0] * q[i0] * q[i5] + (-1.109336e-02) * q[i0] * q[i0] * q[i6] + (-1.924448e-03) * q[i0] * q[i0] * q[i7]
            + (-3.690608e-04) * q[i0] * q[i0] * q[i8] + (-1.714045e-03) * q[i0] * q[i0] * q[i9] + (-1.578440e-03) * q[i0] * q[i0] * q[i10]
            + (-1.942903e-04) * q[i0] * q[i0] * q[i11] + (1.862733e-03) * q[i0] * q[i0] * q[i12] + (1.696342e-04) * q[i0] * q[i0] * q[i15]
            + (-1.053388e-03) * q[i0] * q[i0] * q[i16] + (7.116120e-04) * q[i0] * q[i0] * q[i19] + (3.588934e-04) * q[i0] * q[i0] * q[i20]
            + (8.216099e-04) * q[i0] * q[i0] * q[i21] + (1.011065e-03) * q[i0] * q[i0] * q[i22] + (-7.246073e-04) * q[i0] * q[i1] * q[i1]
            + (4.319157e-03) * q[i1] * q[i1] * q[i1] + (-1.056982e-03) * q[i1] * q[i1] * q[i2] + (-5.286147e-04) * q[i1] * q[i1] * q[i3]
            + (-6.889231e-03) * q[i1] * q[i1] * q[i4] + (1.741400e-04) * q[i1] * q[i1] * q[i5] + (1.946229e-03) * q[i1] * q[i1] * q[i6]
            + (1.113704e-02) * q[i1] * q[i1] * q[i7] + (3.509609e-04) * q[i1] * q[i1] * q[i8] + (1.584619e-03) * q[i1] * q[i1] * q[i9]
            + (1.710830e-03) * q[i1] * q[i1] * q[i10] + (1.852546e-03) * q[i1] * q[i1] * q[i11] + (-2.041879e-04) * q[i1] * q[i1] * q[i12]
            + (1.069579e-03) * q[i1] * q[i1] * q[i15] + (-1.540970e-04) * q[i1] * q[i1] * q[i16] + (-3.622465e-04) * q[i1] * q[i1] * q[i19]
            + (-6.913918e-04) * q[i1] * q[i1] * q[i20] + (1.014259e-03) * q[i1] * q[i1] * q[i21] + (7.871221e-04) * q[i1] * q[i1] * q[i22]
            + (4.185170e-03) * q[i0] * q[i2] * q[i2] + (4.196143e-03) * q[i1] * q[i2] * q[i2] + (-3.011909e-03) * q[i2] * q[i2] * q[i2]
            + (-2.273799e-03) * q[i2] * q[i2] * q[i3] + (-2.291724e-03) * q[i2] * q[i2] * q[i4] + (4.036315e-03) * q[i2] * q[i2] * q[i5]
            + (-1.087924e-05) * q[i2] * q[i2] * q[i6] + (9.450271e-06) * q[i2] * q[i2] * q[i7] + (-4.182231e-06) * q[i2] * q[i2] * q[i8]
            + (-5.289894e-04) * q[i2] * q[i2] * q[i9] + (5.168931e-04) * q[i2] * q[i2] * q[i10] + (-7.185042e-04) * q[i2] * q[i2] * q[i11]
            + (-7.079538e-04) * q[i2] * q[i2] * q[i12] + (-2.672006e-04) * q[i2] * q[i2] * q[i15] + (2.779448e-04) * q[i2] * q[i2] * q[i16]
            + (-5.728309e-04) * q[i2] * q[i2] * q[i19] + (5.599040e-04) * q[i2] * q[i2] * q[i20] + (6.728289e-04) * q[i2] * q[i2] * q[i21]
            + (6.704102e-04) * q[i2] * q[i2] * q[i22] + (-7.665853e-03) * q[i0] * q[i3] * q[i3] + (2.662532e-03) * q[i1] * q[i3] * q[i3]
            + (-1.359760e-03) * q[i2] * q[i3] * q[i3] + (1.195013e-03) * q[i3] * q[i3] * q[i3] + (1.952111e-03) * q[i3] * q[i3] * q[i4]
            + (1.556399e-03) * q[i3] * q[i3] * q[i5] + (9.172461e-03) * q[i3] * q[i3] * q[i6] + (4.204319e-03) * q[i3] * q[i3] * q[i7]
            + (3.506601e-03) * q[i3] * q[i3] * q[i8] + (1.159972e-03) * q[i3] * q[i3] * q[i9] + (1.046336e-04) * q[i3] * q[i3] * q[i10]
            + (8.607750e-05) * q[i3] * q[i3] * q[i11] + (-1.082842e-03) * q[i3] * q[i3] * q[i12] + (-3.190496e-03) * q[i3] * q[i3] * q[i15]
            + (-1.417495e-03) * q[i3] * q[i3] * q[i16] + (2.284390e-03) * q[i3] * q[i3] * q[i19] + (-2.929116e-03) * q[i3] * q[i3] * q[i20]
            + (-1.159429e-04) * q[i3] * q[i3] * q[i21] + (1.880048e-03) * q[i3] * q[i3] * q[i22] + (2.659158e-03) * q[i0] * q[i4] * q[i4]
            + (-7.650379e-03) * q[i1] * q[i4] * q[i4] + (-1.354439e-03) * q[i2] * q[i4] * q[i4] + (1.972930e-03) * q[i3] * q[i4] * q[i4]
            + (1.171165e-03) * q[i4] * q[i4] * q[i4] + (1.526571e-03) * q[i4] * q[i4] * q[i5] + (-4.233610e-03) * q[i4] * q[i4] * q[i6]
            + (-9.219435e-03) * q[i4] * q[i4] * q[i7] + (-3.510551e-03) * q[i4] * q[i4] * q[i8] + (-1.080638e-04) * q[i4] * q[i4] * q[i9]
            + (-1.172863e-03) * q[i4] * q[i4] * q[i10] + (-1.091657e-03) * q[i4] * q[i4] * q[i11] + (7.709892e-05) * q[i4] * q[i4] * q[i12]
            + (1.420788e-03) * q[i4] * q[i4] * q[i15] + (3.186542e-03) * q[i4] * q[i4] * q[i16] + (2.887153e-03) * q[i4] * q[i4] * q[i19]
            + (-2.277595e-03) * q[i4] * q[i4] * q[i20] + (1.868968e-03) * q[i4] * q[i4] * q[i21] + (-1.108247e-04) * q[i4] * q[i4] * q[i22]
            + (-1.317582e-03) * q[i0] * q[i5] * q[i5] + (-1.370099e-03) * q[i1] * q[i5] * q[i5] + (-9.658555e-03) * q[i2] * q[i5] * q[i5]
            + (1.226351e-03) * q[i3] * q[i5] * q[i5] + (1.262891e-03) * q[i4] * q[i5] * q[i5] + (5.050062e-03) * q[i5] * q[i5] * q[i5]
            + (-1.825219e-03) * q[i5] * q[i5] * q[i6] + (1.814481e-03) * q[i5] * q[i5] * q[i7] + (6.752906e-05) * q[i5] * q[i5] * q[i8]
            + (-7.996915e-04) * q[i5] * q[i5] * q[i9] + (7.831560e-04) * q[i5] * q[i5] * q[i10] + (1.764104e-03) * q[i5] * q[i5] * q[i11]
            + (1.870958e-03) * q[i5] * q[i5] * q[i12] + (2.081274e-03) * q[i5] * q[i5] * q[i15] + (-2.095174e-03) * q[i5] * q[i5] * q[i16]
            + (3.035728e-06) * q[i5] * q[i5] * q[i19] + (1.337627e-06) * q[i5] * q[i5] * q[i20] + (5.800218e-04) * q[i5] * q[i5] * q[i21]
            + (5.814167e-04) * q[i5] * q[i5] * q[i22] + (-1.145720e-02) * q[i0] * q[i6] * q[i6] + (6.238411e-03) * q[i1] * q[i6] * q[i6]
            + (-6.167978e-03) * q[i2] * q[i6] * q[i6] + (-7.821550e-03) * q[i3] * q[i6] * q[i6] + (1.972131e-03) * q[i4] * q[i6] * q[i6]
            + (6.330235e-03) * q[i5] * q[i6] * q[i6] + (-6.023597e-03) * q[i6] * q[i6] * q[i6] + (5.546016e-03) * q[i6] * q[i6] * q[i7]
            + (3.682597e-03) * q[i6] * q[i6] * q[i8] + (-2.824449e-04) * q[i6] * q[i6] * q[i9] + (2.238024e-03) * q[i6] * q[i6] * q[i10]
            + (-1.910259e-03) * q[i6] * q[i6] * q[i11] + (-8.133748e-04) * q[i6] * q[i6] * q[i12] + (1.964660e-03) * q[i6] * q[i6] * q[i15]
            + (-1.063728e-03) * q[i6] * q[i6] * q[i16] + (-1.662778e-03) * q[i6] * q[i6] * q[i19] + (3.625732e-04) * q[i6] * q[i6] * q[i20]
            + (-7.256285e-04) * q[i6] * q[i6] * q[i21] + (-1.595204e-03) * q[i6] * q[i6] * q[i22] + (6.246037e-03) * q[i0] * q[i7] * q[i7]
            + (-1.145755e-02) * q[i1] * q[i7] * q[i7] + (-6.130194e-03) * q[i2] * q[i7] * q[i7] + (1.974651e-03) * q[i3] * q[i7] * q[i7]
            + (-7.832752e-03) * q[i4] * q[i7] * q[i7] + (6.327663e-03) * q[i5] * q[i7] * q[i7] + (-5.540731e-03) * q[i6] * q[i7] * q[i7]
            + (6.023584e-03) * q[i7] * q[i7] * q[i7] + (-3.694758e-03) * q[i7] * q[i7] * q[i8] + (-2.250312e-03) * q[i7] * q[i7] * q[i9]
            + (2.785022e-04) * q[i7] * q[i7] * q[i10] + (-8.050146e-04) * q[i7] * q[i7] * q[i11] + (-1.907086e-03) * q[i7] * q[i7] * q[i12]
            + (1.072717e-03) * q[i7] * q[i7] * q[i15] + (-1.961758e-03) * q[i7] * q[i7] * q[i16] + (-3.645672e-04) * q[i7] * q[i7] * q[i19]
            + (1.653534e-03) * q[i7] * q[i7] * q[i20] + (-1.584634e-03) * q[i7] * q[i7] * q[i21] + (-7.159547e-04) * q[i7] * q[i7] * q[i22]
            + (2.224777e-03) * q[i0] * q[i8] * q[i8] + (2.240031e-03) * q[i1] * q[i8] * q[i8] + (4.451997e-03) * q[i2] * q[i8] * q[i8]
            + (-2.793614e-03) * q[i3] * q[i8] * q[i8] + (-2.809552e-03) * q[i4] * q[i8] * q[i8] + (2.052357e-03) * q[i5] * q[i8] * q[i8]
            + (-1.045167e-03) * q[i6] * q[i8] * q[i8] + (1.055293e-03) * q[i7] * q[i8] * q[i8] + (-7.670018e-06) * q[i8] * q[i8] * q[i8]
            + (-6.465699e-04) * q[i8] * q[i8] * q[i9] + (6.442480e-04) * q[i8] * q[i8] * q[i10] + (-3.818482e-03) * q[i8] * q[i8] * q[i11]
            + (-3.917756e-03) * q[i8] * q[i8] * q[i12] + (-3.006871e-03) * q[i8] * q[i8] * q[i15] + (3.063677e-03) * q[i8] * q[i8] * q[i16]
            + (1.206806e-04) * q[i8] * q[i8] * q[i19] + (-1.346143e-04) * q[i8] * q[i8] * q[i20] + (1.297573e-03) * q[i8] * q[i8] * q[i21]
            + (1.305954e-03) * q[i8] * q[i8] * q[i22] + (3.295889e-03) * q[i0] * q[i9] * q[i9] + (3.194498e-04) * q[i1] * q[i9] * q[i9]
            + (1.033256e-03) * q[i2] * q[i9] * q[i9] + (7.747477e-04) * q[i3] * q[i9] * q[i9] + (-1.002610e-03) * q[i4] * q[i9] * q[i9]
            + (-1.284342e-03) * q[i5] * q[i9] * q[i9] + (-1.925311e-03) * q[i6] * q[i9] * q[i9] + (2.180055e-04) * q[i7] * q[i9] * q[i9]
            + (-3.126090e-04) * q[i8] * q[i9] * q[i9] + (-9.753753e-04) * q[i9] * q[i9] * q[i9] + (-1.636086e-04) * q[i9] * q[i9] * q[i10]
            + (4.127428e-04) * q[i9] * q[i9] * q[i11] + (6.475281e-05) * q[i9] * q[i9] * q[i12] + (2.367505e-04) * q[i9] * q[i9] * q[i15]
            + (-4.382164e-04) * q[i9] * q[i9] * q[i16] + (-6.611220e-05) * q[i9] * q[i9] * q[i19] + (6.369948e-04) * q[i9] * q[i9] * q[i20]
            + (6.274636e-05) * q[i9] * q[i9] * q[i21] + (-2.526185e-04) * q[i9] * q[i9] * q[i22] + (3.135397e-04) * q[i0] * q[i10] * q[i10]
            + (3.248302e-03) * q[i1] * q[i10] * q[i10] + (1.014726e-03) * q[i2] * q[i10] * q[i10] + (-9.855731e-04) * q[i3] * q[i10] * q[i10]
            + (7.347479e-04) * q[i4] * q[i10] * q[i10] + (-1.257176e-03) * q[i5] * q[i10] * q[i10] + (-2.083735e-04) * q[i6] * q[i10] * q[i10]
            + (1.908832e-03) * q[i7] * q[i10] * q[i10] + (3.205144e-04) * q[i8] * q[i10] * q[i10] + (1.638362e-04) * q[i9] * q[i10] * q[i10]
            + (9.624813e-04) * q[i10] * q[i10] * q[i10] + (6.127784e-05) * q[i10] * q[i10] * q[i11] + (4.079915e-04) * q[i10] * q[i10] * q[i12]
            + (4.309461e-04) * q[i10] * q[i10] * q[i15] + (-2.335088e-04) * q[i10] * q[i10] * q[i16] + (-6.300817e-04) * q[i10] * q[i10] * q[i19]
            + (6.601364e-05) * q[i10] * q[i10] * q[i20] + (-2.492530e-04) * q[i10] * q[i10] * q[i21] + (6.552225e-05) * q[i10] * q[i10] * q[i22]
            + (-2.853861e-04) * q[i0] * q[i11] * q[i11] + (-1.800565e-04) * q[i1] * q[i11] * q[i11] + (-8.427844e-04) * q[i2] * q[i11] * q[i11]
            + (-9.812237e-04) * q[i3] * q[i11] * q[i11] + (-9.955130e-04) * q[i4] * q[i11] * q[i11] + (-5.658690e-05) * q[i5] * q[i11] * q[i11]
            + (1.614308e-03) * q[i6] * q[i11] * q[i11] + (-1.541249e-03) * q[i7] * q[i11] * q[i11] + (-3.244212e-03) * q[i8] * q[i11] * q[i11]
            + (1.587662e-04) * q[i9] * q[i11] * q[i11] + (-3.784368e-04) * q[i10] * q[i11] * q[i11] + (-9.349521e-04) * q[i11] * q[i11] * q[i11]
            + (1.293442e-04) * q[i11] * q[i11] * q[i12] + (1.423557e-03) * q[i11] * q[i11] * q[i15] + (-2.826463e-04) * q[i11] * q[i11] * q[i16]
            + (-1.238077e-03) * q[i11] * q[i11] * q[i19] + (2.976851e-04) * q[i11] * q[i11] * q[i20] + (-8.182068e-04) * q[i11] * q[i11] * q[i21]
            + (1.101684e-04) * q[i11] * q[i11] * q[i22] + (-1.852411e-04) * q[i0] * q[i12] * q[i12] + (-3.294122e-04) * q[i1] * q[i12] * q[i12]
            + (-8.871160e-04) * q[i2] * q[i12] * q[i12] + (-1.043350e-03) * q[i3] * q[i12] * q[i12] + (-9.536163e-04) * q[i4] * q[i12] * q[i12]
            + (-3.241012e-05) * q[i5] * q[i12] * q[i12] + (1.569816e-03) * q[i6] * q[i12] * q[i12] + (-1.602720e-03) * q[i7] * q[i12] * q[i12]
            + (3.310083e-03) * q[i8] * q[i12] * q[i12] + (3.902955e-04) * q[i9] * q[i12] * q[i12] + (-1.483041e-04) * q[i10] * q[i12] * q[i12]
            + (1.353468e-04) * q[i11] * q[i12] * q[i12] + (-9.692197e-04) * q[i12] * q[i12] * q[i12] + (2.788632e-04) * q[i12] * q[i12] * q[i15]
            + (-1.424586e-03) * q[i12] * q[i12] * q[i16] + (-2.953930e-04) * q[i12] * q[i12] * q[i19] + (1.201686e-03) * q[i12] * q[i12] * q[i20]
            + (1.117641e-04) * q[i12] * q[i12] * q[i21] + (-8.072142e-04) * q[i12] * q[i12] * q[i22] + (-1.907664e-03) * q[i0] * q[i15] * q[i15]
            + (-2.048487e-03) * q[i1] * q[i15] * q[i15] + (-4.605599e-03) * q[i2] * q[i15] * q[i15] + (-8.918000e-04) * q[i3] * q[i15] * q[i15]
            + (4.768543e-04) * q[i4] * q[i15] * q[i15] + (-1.741244e-03) * q[i5] * q[i15] * q[i15] + (1.854455e-03) * q[i6] * q[i15] * q[i15]
            + (-1.486156e-03) * q[i7] * q[i15] * q[i15] + (-2.453704e-03) * q[i8] * q[i15] * q[i15] + (-7.700225e-05) * q[i9] * q[i15] * q[i15]
            + (3.441780e-04) * q[i10] * q[i15] * q[i15] + (-5.886223e-03) * q[i11] * q[i15] * q[i15] + (2.535798e-04) * q[i12] * q[i15] * q[i15]
            + (-3.729371e-04) * q[i15] * q[i15] * q[i15] + (-1.696360e-04) * q[i15] * q[i15] * q[i16] + (7.238608e-04) * q[i15] * q[i15] * q[i19]
            + (1.245525e-04) * q[i15] * q[i15] * q[i20] + (1.243250e-03) * q[i15] * q[i15] * q[i21] + (-5.914336e-05) * q[i15] * q[i15] * q[i22]
            + (-2.086639e-03) * q[i0] * q[i16] * q[i16] + (-1.917866e-03) * q[i1] * q[i16] * q[i16] + (-4.648974e-03) * q[i2] * q[i16] * q[i16]
            + (4.890517e-04) * q[i3] * q[i16] * q[i16] + (-8.897604e-04) * q[i4] * q[i16] * q[i16] + (-1.767671e-03) * q[i5] * q[i16] * q[i16]
            + (1.498998e-03) * q[i6] * q[i16] * q[i16] + (-1.862477e-03) * q[i7] * q[i16] * q[i16] + (2.480365e-03) * q[i8] * q[i16] * q[i16]
            + (-3.474733e-04) * q[i9] * q[i16] * q[i16] + (7.707608e-05) * q[i10] * q[i16] * q[i16] + (2.513389e-04) * q[i11] * q[i16] * q[i16]
            + (-5.953830e-03) * q[i12] * q[i16] * q[i16] + (1.689590e-04) * q[i15] * q[i16] * q[i16] + (3.780196e-04) * q[i16] * q[i16] * q[i16]
            + (-1.198530e-04) * q[i16] * q[i16] * q[i19] + (-7.271708e-04) * q[i16] * q[i16] * q[i20] + (-5.805207e-05) * q[i16] * q[i16] * q[i21]
            + (1.255390e-03) * q[i16] * q[i16] * q[i22] + (3.747856e-04) * q[i0] * q[i19] * q[i19] + (-6.184213e-04) * q[i1] * q[i19] * q[i19]
            + (-4.758407e-04) * q[i2] * q[i19] * q[i19] + (-5.499706e-04) * q[i3] * q[i19] * q[i19] + (4.783957e-04) * q[i4] * q[i19] * q[i19]
            + (5.838992e-04) * q[i5] * q[i19] * q[i19] + (1.224872e-03) * q[i6] * q[i19] * q[i19] + (-1.188255e-03) * q[i7] * q[i19] * q[i19]
            + (6.152050e-04) * q[i8] * q[i19] * q[i19] + (3.324393e-04) * q[i9] * q[i19] * q[i19] + (1.273070e-04) * q[i10] * q[i19] * q[i19]
            + (7.898908e-04) * q[i11] * q[i19] * q[i19] + (-2.068457e-04) * q[i12] * q[i19] * q[i19] + (2.784032e-04) * q[i15] * q[i19] * q[i19]
            + (-2.679767e-04) * q[i16] * q[i19] * q[i19] + (-3.305556e-04) * q[i19] * q[i19] * q[i19] + (1.467082e-05) * q[i19] * q[i19] * q[i20]
            + (-1.887431e-03) * q[i19] * q[i19] * q[i21] + (8.010234e-05) * q[i19] * q[i19] * q[i22] + (-6.009789e-04) * q[i0] * q[i20] * q[i20]
            + (3.759789e-04) * q[i1] * q[i20] * q[i20] + (-4.632956e-04) * q[i2] * q[i20] * q[i20] + (4.830417e-04) * q[i3] * q[i20] * q[i20]
            + (-5.486979e-04) * q[i4] * q[i20] * q[i20] + (5.822137e-04) * q[i5] * q[i20] * q[i20] + (1.177861e-03) * q[i6] * q[i20] * q[i20]
            + (-1.234309e-03) * q[i7] * q[i20] * q[i20] + (-5.973724e-04) * q[i8] * q[i20] * q[i20] + (-1.292999e-04) * q[i9] * q[i20] * q[i20]
            + (-3.326441e-04) * q[i10] * q[i20] * q[i20] + (-2.012535e-04) * q[i11] * q[i20] * q[i20] + (7.762578e-04) * q[i12] * q[i20] * q[i20]
            + (2.665662e-04) * q[i15] * q[i20] * q[i20] + (-2.648461e-04) * q[i16] * q[i20] * q[i20] + (-2.308392e-05) * q[i19] * q[i20] * q[i20]
            + (3.206918e-04) * q[i20] * q[i20] * q[i20] + (8.445291e-05) * q[i20] * q[i20] * q[i21] + (-1.884380e-03) * q[i20] * q[i20] * q[i22]
            + (-1.631851e-03) * q[i0] * q[i21] * q[i21] + (3.043680e-04) * q[i1] * q[i21] * q[i21] + (-1.526069e-03) * q[i2] * q[i21] * q[i21]
            + (-2.297460e-04) * q[i3] * q[i21] * q[i21] + (1.052418e-03) * q[i4] * q[i21] * q[i21] + (2.817858e-04) * q[i5] * q[i21] * q[i21]
            + (1.271774e-03) * q[i6] * q[i21] * q[i21] + (-1.401571e-03) * q[i7] * q[i21] * q[i21] + (2.904282e-04) * q[i8] * q[i21] * q[i21]
            + (2.991626e-04) * q[i9] * q[i21] * q[i21] + (7.848120e-05) * q[i10] * q[i21] * q[i21] + (-3.634334e-04) * q[i11] * q[i21] * q[i21]
            + (-1.363000e-04) * q[i12] * q[i21] * q[i21] + (2.829903e-04) * q[i15] * q[i21] * q[i21] + (-1.146371e-04) * q[i16] * q[i21] * q[i21]
            + (-1.248131e-03) * q[i19] * q[i21] * q[i21] + (-2.553853e-05) * q[i20] * q[i21] * q[i21] + (-2.864912e-04) * q[i21] * q[i21] * q[i21]
            + (9.754488e-05) * q[i21] * q[i21] * q[i22] + (3.073992e-04) * q[i0] * q[i22] * q[i22] + (-1.623037e-03) * q[i1] * q[i22] * q[i22]
            + (-1.514669e-03) * q[i2] * q[i22] * q[i22] + (1.064119e-03) * q[i3] * q[i22] * q[i22] + (-2.329584e-04) * q[i4] * q[i22] * q[i22]
            + (2.937040e-04) * q[i5] * q[i22] * q[i22] + (1.395523e-03) * q[i6] * q[i22] * q[i22] + (-1.274019e-03) * q[i7] * q[i22] * q[i22]
            + (-2.898016e-04) * q[i8] * q[i22] * q[i22] + (-7.665134e-05) * q[i9] * q[i22] * q[i22] + (-3.000039e-04) * q[i10] * q[i22] * q[i22]
            + (-1.354532e-04) * q[i11] * q[i22] * q[i22] + (-3.473568e-04) * q[i12] * q[i22] * q[i22] + (1.136117e-04) * q[i15] * q[i22] * q[i22]
            + (-2.800429e-04) * q[i16] * q[i22] * q[i22] + (2.622147e-05) * q[i19] * q[i22] * q[i22] + (1.266536e-03) * q[i20] * q[i22] * q[i22]
            + (9.629361e-05) * q[i21] * q[i22] * q[i22] + (-2.891843e-04) * q[i22] * q[i22] * q[i22] + (-8.346531e-04) * q[i0] * q[i1] * q[i2]
            + (1.427112e-03) * q[i0] * q[i1] * q[i3] + (1.405527e-03) * q[i0] * q[i1] * q[i4] + (-1.495583e-03) * q[i0] * q[i1] * q[i5]
            + (2.440913e-03) * q[i0] * q[i1] * q[i6] + (-2.509785e-03) * q[i0] * q[i1] * q[i7] + (3.772037e-05) * q[i0] * q[i1] * q[i8]
            + (-9.410769e-04) * q[i0] * q[i1] * q[i9] + (9.334965e-04) * q[i0] * q[i1] * q[i10] + (-8.990105e-04) * q[i0] * q[i1] * q[i11]
            + (-8.987392e-04) * q[i0] * q[i1] * q[i12] + (-1.695188e-03) * q[i0] * q[i1] * q[i15] + (1.701661e-03) * q[i0] * q[i1] * q[i16]
            + (-2.723746e-04) * q[i0] * q[i1] * q[i19] + (2.669963e-04) * q[i0] * q[i1] * q[i20] + (-9.051953e-04) * q[i0] * q[i1] * q[i21]
            + (-8.994086e-04) * q[i0] * q[i1] * q[i22] + (3.394724e-05) * q[i0] * q[i2] * q[i3] + (-1.094924e-03) * q[i0] * q[i2] * q[i4]
            + (3.503458e-03) * q[i0] * q[i2] * q[i5] + (-9.255762e-03) * q[i0] * q[i2] * q[i6] + (-9.079879e-04) * q[i0] * q[i2] * q[i7]
            + (2.392972e-03) * q[i0] * q[i2] * q[i8] + (-1.221234e-03) * q[i0] * q[i2] * q[i9] + (-1.089578e-04) * q[i0] * q[i2] * q[i10]
            + (9.579792e-04) * q[i0] * q[i2] * q[i11] + (-7.160882e-04) * q[i0] * q[i2] * q[i12] + (3.870245e-04) * q[i0] * q[i2] * q[i15]
            + (-6.183652e-04) * q[i0] * q[i2] * q[i16] + (-1.829261e-04) * q[i0] * q[i2] * q[i19] + (1.314575e-04) * q[i0] * q[i2] * q[i20]
            + (1.036193e-04) * q[i0] * q[i2] * q[i21] + (3.914916e-04) * q[i0] * q[i2] * q[i22] + (6.346759e-03) * q[i0] * q[i3] * q[i4]
            + (-1.038624e-02) * q[i0] * q[i3] * q[i5] + (-1.448655e-02) * q[i0] * q[i3] * q[i6] + (-8.385057e-05) * q[i0] * q[i3] * q[i7]
            + (3.691852e-03) * q[i0] * q[i3] * q[i8] + (4.641062e-03) * q[i0] * q[i3] * q[i9] + (-4.279734e-04) * q[i0] * q[i3] * q[i10]
            + (-1.397372e-03) * q[i0] * q[i3] * q[i11] + (-1.202876e-03) * q[i0] * q[i3] * q[i12] + (2.500203e-03) * q[i0] * q[i3] * q[i15]
            + (-2.838333e-03) * q[i0] * q[i3] * q[i16] + (-2.411072e-03) * q[i0] * q[i3] * q[i19] + (-3.702059e-04) * q[i0] * q[i3] * q[i20]
            + (1.223852e-03) * q[i0] * q[i3] * q[i21] + (-1.502629e-03) * q[i0] * q[i3] * q[i22] + (2.166891e-03) * q[i0] * q[i4] * q[i5]
            + (2.077978e-03) * q[i0] * q[i4] * q[i6] + (-3.874965e-03) * q[i0] * q[i4] * q[i7] + (1.921058e-03) * q[i0] * q[i4] * q[i8]
            + (4.471828e-03) * q[i0] * q[i4] * q[i9] + (3.653177e-03) * q[i0] * q[i4] * q[i10] + (-7.361672e-05) * q[i0] * q[i4] * q[i11]
            + (6.362083e-04) * q[i0] * q[i4] * q[i12] + (-4.425119e-03) * q[i0] * q[i4] * q[i15] + (-1.259237e-03) * q[i0] * q[i4] * q[i16]
            + (1.293175e-03) * q[i0] * q[i4] * q[i19] + (-1.629772e-04) * q[i0] * q[i4] * q[i20] + (-2.991066e-04) * q[i0] * q[i4] * q[i21]
            + (-1.857760e-03) * q[i0] * q[i4] * q[i22] + (1.889204e-03) * q[i0] * q[i5] * q[i6] + (-1.951168e-03) * q[i0] * q[i5] * q[i7]
            + (4.131932e-03) * q[i0] * q[i5] * q[i8] + (1.215783e-03) * q[i0] * q[i5] * q[i9] + (5.609378e-04) * q[i0] * q[i5] * q[i10]
            + (1.385829e-03) * q[i0] * q[i5] * q[i11] + (3.532723e-04) * q[i0] * q[i5] * q[i12] + (3.217083e-03) * q[i0] * q[i5] * q[i15]
            + (-1.774397e-03) * q[i0] * q[i5] * q[i16] + (-4.666657e-04) * q[i0] * q[i5] * q[i19] + (-4.053795e-04) * q[i0] * q[i5] * q[i20]
            + (-1.545775e-03) * q[i0] * q[i5] * q[i21] + (-1.651426e-03) * q[i0] * q[i5] * q[i22] + (1.064443e-02) * q[i0] * q[i6] * q[i7]
            + (2.018270e-03) * q[i0] * q[i6] * q[i8] + (-8.093356e-03) * q[i0] * q[i6] * q[i9] + (4.731231e-03) * q[i0] * q[i6] * q[i10]
            + (1.256146e-03) * q[i0] * q[i6] * q[i11] + (3.351741e-03) * q[i0] * q[i6] * q[i12] + (-4.457664e-04) * q[i0] * q[i6] * q[i15]
            + (2.525945e-03) * q[i0] * q[i6] * q[i16] + (-3.448775e-03) * q[i0] * q[i6] * q[i19] + (5.469725e-04) * q[i0] * q[i6] * q[i20]
            + (1.146224e-03) * q[i0] * q[i6] * q[i21] + (8.591253e-04) * q[i0] * q[i6] * q[i22] + (-5.674026e-04) * q[i0] * q[i7] * q[i8]
            + (7.027701e-04) * q[i0] * q[i7] * q[i9] + (6.223994e-03) * q[i0] * q[i7] * q[i10] + (6.106089e-05) * q[i0] * q[i7] * q[i11]
            + (-1.928167e-03) * q[i0] * q[i7] * q[i12] + (1.597382e-03) * q[i0] * q[i7] * q[i15] + (-2.070965e-03) * q[i0] * q[i7] * q[i16]
            + (-1.237260e-03) * q[i0] * q[i7] * q[i19] + (-1.390933e-04) * q[i0] * q[i7] * q[i20] + (-7.828759e-04) * q[i0] * q[i7] * q[i21]
            + (-6.802556e-04) * q[i0] * q[i7] * q[i22] + (4.403004e-04) * q[i0] * q[i8] * q[i9] + (-2.309188e-03) * q[i0] * q[i8] * q[i10]
            + (-9.840143e-04) * q[i0] * q[i8] * q[i11] + (-1.802440e-05) * q[i0] * q[i8] * q[i12] + (-1.237752e-03) * q[i0] * q[i8] * q[i15]
            + (3.349766e-05) * q[i0] * q[i8] * q[i16] + (1.515999e-03) * q[i0] * q[i8] * q[i19] + (-1.020506e-03) * q[i0] * q[i8] * q[i20]
            + (6.531737e-04) * q[i0] * q[i8] * q[i21] + (4.179213e-04) * q[i0] * q[i8] * q[i22] + (7.059487e-04) * q[i0] * q[i9] * q[i10]
            + (2.171805e-04) * q[i0] * q[i9] * q[i11] + (8.206057e-04) * q[i0] * q[i9] * q[i12] + (-2.962509e-04) * q[i0] * q[i9] * q[i15]
            + (3.266708e-04) * q[i0] * q[i9] * q[i16] + (-1.340145e-04) * q[i0] * q[i9] * q[i19] + (4.477149e-04) * q[i0] * q[i9] * q[i20]
            + (-9.313780e-05) * q[i0] * q[i9] * q[i21] + (-3.259272e-05) * q[i0] * q[i9] * q[i22] + (7.615470e-04) * q[i0] * q[i10] * q[i11]
            + (9.834727e-04) * q[i0] * q[i10] * q[i12] + (-1.050272e-03) * q[i0] * q[i10] * q[i15] + (5.576206e-04) * q[i0] * q[i10] * q[i16]
            + (-6.144819e-04) * q[i0] * q[i10] * q[i19] + (1.285622e-04) * q[i0] * q[i10] * q[i20] + (1.549669e-04) * q[i0] * q[i10] * q[i21]
            + (-3.355837e-04) * q[i0] * q[i10] * q[i22] + (2.378756e-05) * q[i0] * q[i11] * q[i12] + (2.347744e-04) * q[i0] * q[i11] * q[i15]
            + (-2.017318e-04) * q[i0] * q[i11] * q[i16] + (6.241591e-04) * q[i0] * q[i11] * q[i19] + (-9.831590e-04) * q[i0] * q[i11] * q[i20]
            + (2.935356e-04) * q[i0] * q[i11] * q[i21] + (-2.104644e-04) * q[i0] * q[i11] * q[i22] + (9.361919e-04) * q[i0] * q[i12] * q[i15]
            + (-2.527649e-03) * q[i0] * q[i12] * q[i16] + (1.297013e-03) * q[i0] * q[i12] * q[i19] + (9.315936e-04) * q[i0] * q[i12] * q[i20]
            + (4.288031e-04) * q[i0] * q[i12] * q[i21] + (-3.299089e-05) * q[i0] * q[i12] * q[i22] + (2.377203e-06) * q[i0] * q[i15] * q[i16]
            + (-4.759495e-05) * q[i0] * q[i15] * q[i19] + (-3.216543e-04) * q[i0] * q[i15] * q[i20] + (-1.686942e-03) * q[i0] * q[i15] * q[i21]
            + (3.592464e-04) * q[i0] * q[i15] * q[i22] + (1.059564e-03) * q[i0] * q[i16] * q[i19] + (-2.461122e-04) * q[i0] * q[i16] * q[i20]
            + (-4.163082e-04) * q[i0] * q[i16] * q[i21] + (5.712635e-04) * q[i0] * q[i16] * q[i22] + (2.088757e-04) * q[i0] * q[i19] * q[i20]
            + (-6.207025e-06) * q[i0] * q[i19] * q[i21] + (-4.942090e-04) * q[i0] * q[i19] * q[i22] + (-7.676975e-04) * q[i0] * q[i20] * q[i21]
            + (3.850536e-04) * q[i0] * q[i20] * q[i22] + (3.870924e-04) * q[i0] * q[i21] * q[i22] + (-1.052106e-03) * q[i1] * q[i2] * q[i3]
            + (5.408919e-05) * q[i1] * q[i2] * q[i4] + (3.533647e-03) * q[i1] * q[i2] * q[i5] + (8.852707e-04) * q[i1] * q[i2] * q[i6]
            + (9.255482e-03) * q[i1] * q[i2] * q[i7] + (-2.397383e-03) * q[i1] * q[i2] * q[i8] + (1.123347e-04) * q[i1] * q[i2] * q[i9]
            + (1.213317e-03) * q[i1] * q[i2] * q[i10] + (-7.466897e-04) * q[i1] * q[i2] * q[i11] + (9.808607e-04) * q[i1] * q[i2] * q[i12]
            + (6.267690e-04) * q[i1] * q[i2] * q[i15] + (-3.589615e-04) * q[i1] * q[i2] * q[i16] + (-1.329226e-04) * q[i1] * q[i2] * q[i19]
            + (1.997256e-04) * q[i1] * q[i2] * q[i20] + (3.816037e-04) * q[i1] * q[i2] * q[i21] + (7.688619e-05) * q[i1] * q[i2] * q[i22]
            + (6.358675e-03) * q[i1] * q[i3] * q[i4] + (2.140390e-03) * q[i1] * q[i3] * q[i5] + (3.777589e-03) * q[i1] * q[i3] * q[i6]
            + (-2.093520e-03) * q[i1] * q[i3] * q[i7] + (-1.897335e-03) * q[i1] * q[i3] * q[i8] + (-3.666251e-03) * q[i1] * q[i3] * q[i9]
            + (-4.470188e-03) * q[i1] * q[i3] * q[i10] + (5.982896e-04) * q[i1] * q[i3] * q[i11] + (-1.060551e-04) * q[i1] * q[i3] * q[i12]
            + (1.236331e-03) * q[i1] * q[i3] * q[i15] + (4.454264e-03) * q[i1] * q[i3] * q[i16] + (1.676639e-04) * q[i1] * q[i3] * q[i19]
            + (-1.299664e-03) * q[i1] * q[i3] * q[i20] + (-1.828656e-03) * q[i1] * q[i3] * q[i21] + (-2.798875e-04) * q[i1] * q[i3] * q[i22]
            + (-1.032987e-02) * q[i1] * q[i4] * q[i5] + (7.430544e-05) * q[i1] * q[i4] * q[i6] + (1.461029e-02) * q[i1] * q[i4] * q[i7]
            + (-3.676329e-03) * q[i1] * q[i4] * q[i8] + (4.109857e-04) * q[i1] * q[i4] * q[i9] + (-4.582681e-03) * q[i1] * q[i4] * q[i10]
            + (-1.219388e-03) * q[i1] * q[i4] * q[i11] + (-1.448425e-03) * q[i1] * q[i4] * q[i12] + (2.785258e-03) * q[i1] * q[i4] * q[i15]
            + (-2.476091e-03) * q[i1] * q[i4] * q[i16] + (3.806077e-04) * q[i1] * q[i4] * q[i19] + (2.369193e-03) * q[i1] * q[i4] * q[i20]
            + (-1.502616e-03) * q[i1] * q[i4] * q[i21] + (1.220969e-03) * q[i1] * q[i4] * q[i22] + (1.989037e-03) * q[i1] * q[i5] * q[i6]
            + (-1.894315e-03) * q[i1] * q[i5] * q[i7] + (-4.131924e-03) * q[i1] * q[i5] * q[i8] + (-5.376108e-04) * q[i1] * q[i5] * q[i9]
            + (-1.196318e-03) * q[i1] * q[i5] * q[i10] + (3.883544e-04) * q[i1] * q[i5] * q[i11] + (1.359287e-03) * q[i1] * q[i5] * q[i12]
            + (1.774251e-03) * q[i1] * q[i5] * q[i15] + (-3.250903e-03) * q[i1] * q[i5] * q[i16] + (3.999837e-04) * q[i1] * q[i5] * q[i19]
            + (4.605860e-04) * q[i1] * q[i5] * q[i20] + (-1.623950e-03) * q[i1] * q[i5] * q[i21] + (-1.506938e-03) * q[i1] * q[i5] * q[i22]
            + (1.066394e-02) * q[i1] * q[i6] * q[i7] + (-5.539103e-04) * q[i1] * q[i6] * q[i8] + (6.241155e-03) * q[i1] * q[i6] * q[i9]
            + (7.006525e-04) * q[i1] * q[i6] * q[i10] + (1.968200e-03) * q[i1] * q[i6] * q[i11] + (-8.547104e-05) * q[i1] * q[i6] * q[i12]
            + (-2.089086e-03) * q[i1] * q[i6] * q[i15] + (1.587016e-03) * q[i1] * q[i6] * q[i16] + (-1.365403e-04) * q[i1] * q[i6] * q[i19]
            + (-1.235479e-03) * q[i1] * q[i6] * q[i20] + (6.942189e-04) * q[i1] * q[i6] * q[i21] + (7.825132e-04) * q[i1] * q[i6] * q[i22]
            + (1.990251e-03) * q[i1] * q[i7] * q[i8] + (4.768331e-03) * q[i1] * q[i7] * q[i9] + (-8.073847e-03) * q[i1] * q[i7] * q[i10]
            + (-3.302676e-03) * q[i1] * q[i7] * q[i11] + (-1.266723e-03) * q[i1] * q[i7] * q[i12] + (2.515716e-03) * q[i1] * q[i7] * q[i15]
            + (-4.501706e-04) * q[i1] * q[i7] * q[i16] + (5.669661e-04) * q[i1] * q[i7] * q[i19] + (-3.425146e-03) * q[i1] * q[i7] * q[i20]
            + (-8.601597e-04) * q[i1] * q[i7] * q[i21] + (-1.137522e-03) * q[i1] * q[i7] * q[i22] + (-2.300075e-03) * q[i1] * q[i8] * q[i9]
            + (4.317355e-04) * q[i1] * q[i8] * q[i10] + (3.076759e-05) * q[i1] * q[i8] * q[i11] + (9.556149e-04) * q[i1] * q[i8] * q[i12]
            + (4.747636e-05) * q[i1] * q[i8] * q[i15] + (-1.247925e-03) * q[i1] * q[i8] * q[i16] + (-1.008339e-03) * q[i1] * q[i8] * q[i19]
            + (1.523498e-03) * q[i1] * q[i8] * q[i20] + (-4.188210e-04) * q[i1] * q[i8] * q[i21] + (-6.807220e-04) * q[i1] * q[i8] * q[i22]
            + (7.159339e-04) * q[i1] * q[i9] * q[i10] + (-9.588975e-04) * q[i1] * q[i9] * q[i11] + (-7.617789e-04) * q[i1] * q[i9] * q[i12]
            + (5.426493e-04) * q[i1] * q[i9] * q[i15] + (-1.040909e-03) * q[i1] * q[i9] * q[i16] + (1.209350e-04) * q[i1] * q[i9] * q[i19]
            + (-6.194793e-04) * q[i1] * q[i9] * q[i20] + (3.461663e-04) * q[i1] * q[i9] * q[i21] + (-1.466367e-04) * q[i1] * q[i9] * q[i22]
            + (-8.165858e-04) * q[i1] * q[i10] * q[i11] + (-2.143419e-04) * q[i1] * q[i10] * q[i12] + (3.368781e-04) * q[i1] * q[i10] * q[i15]
            + (-2.906111e-04) * q[i1] * q[i10] * q[i16] + (4.356600e-04) * q[i1] * q[i10] * q[i19] + (-1.314759e-04) * q[i1] * q[i10] * q[i20]
            + (3.046122e-05) * q[i1] * q[i10] * q[i21] + (1.084647e-04) * q[i1] * q[i10] * q[i22] + (1.206100e-05) * q[i1] * q[i11] * q[i12]
            + (2.540007e-03) * q[i1] * q[i11] * q[i15] + (-9.214786e-04) * q[i1] * q[i11] * q[i16] + (-9.333654e-04) * q[i1] * q[i11] * q[i19]
            + (-1.281242e-03) * q[i1] * q[i11] * q[i20] + (-2.952990e-05) * q[i1] * q[i11] * q[i21] + (4.311889e-04) * q[i1] * q[i11] * q[i22]
            + (2.050431e-04) * q[i1] * q[i12] * q[i15] + (-2.460959e-04) * q[i1] * q[i12] * q[i16] + (9.758899e-04) * q[i1] * q[i12] * q[i19]
            + (-6.428755e-04) * q[i1] * q[i12] * q[i20] + (-2.075818e-04) * q[i1] * q[i12] * q[i21] + (2.939070e-04) * q[i1] * q[i12] * q[i22]
            + (-3.547933e-06) * q[i1] * q[i15] * q[i16] + (-2.625495e-04) * q[i1] * q[i15] * q[i19] + (1.049304e-03) * q[i1] * q[i15] * q[i20]
            + (-5.695855e-04) * q[i1] * q[i15] * q[i21] + (4.220954e-04) * q[i1] * q[i15] * q[i22] + (-3.201155e-04) * q[i1] * q[i16] * q[i19]
            + (-3.407726e-05) * q[i1] * q[i16] * q[i20] + (-3.615603e-04) * q[i1] * q[i16] * q[i21] + (1.679648e-03) * q[i1] * q[i16] * q[i22]
            + (2.035014e-04) * q[i1] * q[i19] * q[i20] + (-3.969437e-04) * q[i1] * q[i19] * q[i21] + (7.699027e-04) * q[i1] * q[i19] * q[i22]
            + (4.882128e-04) * q[i1] * q[i20] * q[i21] + (1.876201e-05) * q[i1] * q[i20] * q[i22] + (3.737421e-04) * q[i1] * q[i21] * q[i22]
            + (-3.414844e-03) * q[i2] * q[i3] * q[i4] + (-1.548406e-02) * q[i2] * q[i3] * q[i5] + (-4.403246e-03) * q[i2] * q[i3] * q[i6]
            + (-2.472890e-03) * q[i2] * q[i3] * q[i7] + (-1.970023e-04) * q[i2] * q[i3] * q[i8] + (1.065266e-03) * q[i2] * q[i3] * q[i9]
            + (-2.928779e-03) * q[i2] * q[i3] * q[i10] + (-2.067182e-03) * q[i2] * q[i3] * q[i11] + (5.248740e-04) * q[i2] * q[i3] * q[i12]
            + (3.432405e-03) * q[i2] * q[i3] * q[i15] + (6.102899e-04) * q[i2] * q[i3] * q[i16] + (-2.403196e-03) * q[i2] * q[i3] * q[i19]
            + (-8.265750e-04) * q[i2] * q[i3] * q[i20] + (8.494775e-04) * q[i2] * q[i3] * q[i21] + (-1.339271e-03) * q[i2] * q[i3] * q[i22]
            + (-1.538618e-02) * q[i2] * q[i4] * q[i5] + (2.431319e-03) * q[i2] * q[i4] * q[i6] + (4.423881e-03) * q[i2] * q[i4] * q[i7]
            + (2.195300e-04) * q[i2] * q[i4] * q[i8] + (2.925550e-03) * q[i2] * q[i4] * q[i9] + (-1.013973e-03) * q[i2] * q[i4] * q[i10]
            + (5.717113e-04) * q[i2] * q[i4] * q[i11] + (-2.060311e-03) * q[i2] * q[i4] * q[i12] + (-6.114839e-04) * q[i2] * q[i4] * q[i15]
            + (-3.448970e-03) * q[i2] * q[i4] * q[i16] + (8.164762e-04) * q[i2] * q[i4] * q[i19] + (2.400144e-03) * q[i2] * q[i4] * q[i20]
            + (-1.327809e-03) * q[i2] * q[i4] * q[i21] + (8.300638e-04) * q[i2] * q[i4] * q[i22] + (-2.733054e-03) * q[i2] * q[i5] * q[i6]
            + (2.764813e-03) * q[i2] * q[i5] * q[i7] + (-1.664079e-05) * q[i2] * q[i5] * q[i8] + (1.866223e-03) * q[i2] * q[i5] * q[i9]
            + (-1.855923e-03) * q[i2] * q[i5] * q[i10] + (-6.183201e-04) * q[i2] * q[i5] * q[i11] + (-6.954248e-04) * q[i2] * q[i5] * q[i12]
            + (5.859833e-03) * q[i2] * q[i5] * q[i15] + (-5.913097e-03) * q[i2] * q[i5] * q[i16] + (-1.748491e-03) * q[i2] * q[i5] * q[i19]
            + (1.751262e-03) * q[i2] * q[i5] * q[i20] + (-1.068426e-03) * q[i2] * q[i5] * q[i21] + (-1.038283e-03) * q[i2] * q[i5] * q[i22]
            + (1.259057e-02) * q[i2] * q[i6] * q[i7] + (-7.272094e-03) * q[i2] * q[i6] * q[i8] + (-4.764478e-03) * q[i2] * q[i6] * q[i9]
            + (7.771415e-04) * q[i2] * q[i6] * q[i10] + (8.556703e-06) * q[i2] * q[i6] * q[i11] + (2.338387e-03) * q[i2] * q[i6] * q[i12]
            + (-1.378391e-03) * q[i2] * q[i6] * q[i15] + (9.848270e-04) * q[i2] * q[i6] * q[i16] + (-2.454018e-05) * q[i2] * q[i6] * q[i19]
            + (3.393243e-04) * q[i2] * q[i6] * q[i20] + (-9.109117e-04) * q[i2] * q[i6] * q[i21] + (7.190790e-04) * q[i2] * q[i6] * q[i22]
            + (-7.247755e-03) * q[i2] * q[i7] * q[i8] + (7.925011e-04) * q[i2] * q[i7] * q[i9] + (-4.733092e-03) * q[i2] * q[i7] * q[i10]
            + (-2.324831e-03) * q[i2] * q[i7] * q[i11] + (-3.349756e-06) * q[i2] * q[i7] * q[i12] + (9.778492e-04) * q[i2] * q[i7] * q[i15]
            + (-1.405652e-03) * q[i2] * q[i7] * q[i16] + (3.461047e-04) * q[i2] * q[i7] * q[i19] + (-1.300624e-05) * q[i2] * q[i7] * q[i20]
            + (-7.129322e-04) * q[i2] * q[i7] * q[i21] + (9.094359e-04) * q[i2] * q[i7] * q[i22] + (-2.026978e-03) * q[i2] * q[i8] * q[i9]
            + (-2.007572e-03) * q[i2] * q[i8] * q[i10] + (7.690454e-05) * q[i2] * q[i8] * q[i11] + (-1.277173e-04) * q[i2] * q[i8] * q[i12]
            + (-1.013031e-03) * q[i2] * q[i8] * q[i15] + (-1.017945e-03) * q[i2] * q[i8] * q[i16] + (-1.213964e-03) * q[i2] * q[i8] * q[i19]
            + (-1.202537e-03) * q[i2] * q[i8] * q[i20] + (1.080224e-03) * q[i2] * q[i8] * q[i21] + (-1.089511e-03) * q[i2] * q[i8] * q[i22]
            + (4.356090e-04) * q[i2] * q[i9] * q[i10] + (-1.279139e-03) * q[i2] * q[i9] * q[i11] + (-3.126844e-04) * q[i2] * q[i9] * q[i12]
            + (1.020416e-03) * q[i2] * q[i9] * q[i15] + (-1.131341e-03) * q[i2] * q[i9] * q[i16] + (2.814179e-04) * q[i2] * q[i9] * q[i19]
            + (-4.580965e-04) * q[i2] * q[i9] * q[i20] + (-1.082832e-04) * q[i2] * q[i9] * q[i21] + (-4.297117e-05) * q[i2] * q[i9] * q[i22]
            + (3.151909e-04) * q[i2] * q[i10] * q[i11] + (1.280565e-03) * q[i2] * q[i10] * q[i12] + (-1.123029e-03) * q[i2] * q[i10] * q[i15]
            + (1.026945e-03) * q[i2] * q[i10] * q[i16] + (-4.671196e-04) * q[i2] * q[i10] * q[i19] + (2.916659e-04) * q[i2] * q[i10] * q[i20]
            + (4.605087e-05) * q[i2] * q[i10] * q[i21] + (1.157773e-04) * q[i2] * q[i10] * q[i22] + (8.537937e-04) * q[i2] * q[i11] * q[i12]
            + (2.076426e-03) * q[i2] * q[i11] * q[i15] + (-5.741165e-04) * q[i2] * q[i11] * q[i16] + (-6.186795e-04) * q[i2] * q[i11] * q[i19]
            + (-4.964284e-04) * q[i2] * q[i11] * q[i20] + (-5.254483e-05) * q[i2] * q[i11] * q[i21] + (-1.053333e-04) * q[i2] * q[i11] * q[i22]
            + (5.847939e-04) * q[i2] * q[i12] * q[i15] + (-2.091056e-03) * q[i2] * q[i12] * q[i16] + (4.856886e-04) * q[i2] * q[i12] * q[i19]
            + (5.954556e-04) * q[i2] * q[i12] * q[i20] + (-1.037075e-04) * q[i2] * q[i12] * q[i21] + (-3.184955e-05) * q[i2] * q[i12] * q[i22]
            + (-4.683864e-04) * q[i2] * q[i15] * q[i16] + (7.724403e-04) * q[i2] * q[i15] * q[i19] + (4.007489e-04) * q[i2] * q[i15] * q[i20]
            + (-1.745349e-03) * q[i2] * q[i15] * q[i21] + (4.714810e-05) * q[i2] * q[i15] * q[i22] + (4.203856e-04) * q[i2] * q[i16] * q[i19]
            + (7.946630e-04) * q[i2] * q[i16] * q[i20] + (-3.777557e-05) * q[i2] * q[i16] * q[i21] + (1.742322e-03) * q[i2] * q[i16] * q[i22]
            + (-4.370205e-04) * q[i2] * q[i19] * q[i20] + (-1.674023e-03) * q[i2] * q[i19] * q[i21] + (1.799295e-05) * q[i2] * q[i19] * q[i22]
            + (-2.370410e-05) * q[i2] * q[i20] * q[i21] + (1.679019e-03) * q[i2] * q[i20] * q[i22] + (8.492379e-04) * q[i2] * q[i21] * q[i22]
            + (1.174176e-03) * q[i3] * q[i4] * q[i5] + (8.426780e-03) * q[i3] * q[i4] * q[i6] + (-8.359363e-03) * q[i3] * q[i4] * q[i7]
            + (-1.122263e-05) * q[i3] * q[i4] * q[i8] + (-8.118305e-04) * q[i3] * q[i4] * q[i9] + (8.529342e-04) * q[i3] * q[i4] * q[i10]
            + (3.597437e-03) * q[i3] * q[i4] * q[i11] + (3.627772e-03) * q[i3] * q[i4] * q[i12] + (3.793315e-03) * q[i3] * q[i4] * q[i15]
            + (-3.798990e-03) * q[i3] * q[i4] * q[i16] + (-6.057769e-03) * q[i3] * q[i4] * q[i19] + (6.060812e-03) * q[i3] * q[i4] * q[i20]
            + (-5.361392e-04) * q[i3] * q[i4] * q[i21] + (-5.363010e-04) * q[i3] * q[i4] * q[i22] + (-7.018283e-03) * q[i3] * q[i5] * q[i6]
            + (-3.106559e-03) * q[i3] * q[i5] * q[i7] + (-1.497916e-02) * q[i3] * q[i5] * q[i8] + (-1.775080e-04) * q[i3] * q[i5] * q[i9]
            + (1.140135e-03) * q[i3] * q[i5] * q[i10] + (-9.524564e-04) * q[i3] * q[i5] * q[i11] + (-2.029475e-03) * q[i3] * q[i5] * q[i12]
            + (-1.225580e-03) * q[i3] * q[i5] * q[i15] + (3.048870e-03) * q[i3] * q[i5] * q[i16] + (-1.357805e-03) * q[i3] * q[i5] * q[i19]
            + (-1.363373e-03) * q[i3] * q[i5] * q[i20] + (1.755457e-03) * q[i3] * q[i5] * q[i21] + (-1.061229e-03) * q[i3] * q[i5] * q[i22]
            + (1.009055e-03) * q[i3] * q[i6] * q[i7] + (-2.132648e-03) * q[i3] * q[i6] * q[i8] + (-1.094750e-03) * q[i3] * q[i6] * q[i9]
            + (3.304197e-04) * q[i3] * q[i6] * q[i10] + (6.582829e-04) * q[i3] * q[i6] * q[i11] + (4.444862e-04) * q[i3] * q[i6] * q[i12]
            + (3.373408e-03) * q[i3] * q[i6] * q[i15] + (3.345002e-03) * q[i3] * q[i6] * q[i16] + (-1.681426e-03) * q[i3] * q[i6] * q[i19]
            + (-9.682098e-04) * q[i3] * q[i6] * q[i20] + (-2.165034e-03) * q[i3] * q[i6] * q[i21] + (9.942412e-04) * q[i3] * q[i6] * q[i22]
            + (1.143908e-03) * q[i3] * q[i7] * q[i8] + (-9.034955e-04) * q[i3] * q[i7] * q[i9] + (-2.223610e-03) * q[i3] * q[i7] * q[i10]
            + (3.926526e-04) * q[i3] * q[i7] * q[i11] + (1.952225e-03) * q[i3] * q[i7] * q[i12] + (1.778172e-03) * q[i3] * q[i7] * q[i15]
            + (-2.089276e-03) * q[i3] * q[i7] * q[i16] + (1.133573e-03) * q[i3] * q[i7] * q[i19] + (4.211431e-03) * q[i3] * q[i7] * q[i20]
            + (2.645970e-03) * q[i3] * q[i7] * q[i21] + (-1.204513e-03) * q[i3] * q[i7] * q[i22] + (1.282765e-03) * q[i3] * q[i8] * q[i9]
            + (2.358254e-05) * q[i3] * q[i8] * q[i10] + (7.829275e-04) * q[i3] * q[i8] * q[i11] + (1.143059e-03) * q[i3] * q[i8] * q[i12]
            + (-2.119369e-03) * q[i3] * q[i8] * q[i15] + (4.871352e-04) * q[i3] * q[i8] * q[i16] + (3.770535e-03) * q[i3] * q[i8] * q[i19]
            + (-2.285660e-03) * q[i3] * q[i8] * q[i20] + (6.368776e-04) * q[i3] * q[i8] * q[i21] + (1.847937e-03) * q[i3] * q[i8] * q[i22]
            + (-5.890720e-04) * q[i3] * q[i9] * q[i10] + (-1.284133e-03) * q[i3] * q[i9] * q[i11] + (-1.399436e-03) * q[i3] * q[i9] * q[i12]
            + (2.049416e-04) * q[i3] * q[i9] * q[i15] + (4.223724e-04) * q[i3] * q[i9] * q[i16] + (1.406502e-03) * q[i3] * q[i9] * q[i19]
            + (-1.023814e-03) * q[i3] * q[i9] * q[i20] + (-1.350166e-03) * q[i3] * q[i9] * q[i21] + (7.085870e-04) * q[i3] * q[i9] * q[i22]
            + (-6.521371e-04) * q[i3] * q[i10] * q[i11] + (5.601645e-04) * q[i3] * q[i10] * q[i12] + (3.035723e-04) * q[i3] * q[i10] * q[i15]
            + (-1.849129e-03) * q[i3] * q[i10] * q[i16] + (6.035180e-04) * q[i3] * q[i10] * q[i19] + (-1.015407e-03) * q[i3] * q[i10] * q[i20]
            + (-6.701134e-05) * q[i3] * q[i10] * q[i21] + (-6.866560e-04) * q[i3] * q[i10] * q[i22] + (4.141827e-04) * q[i3] * q[i11] * q[i12]
            + (8.568046e-05) * q[i3] * q[i11] * q[i15] + (-3.801519e-04) * q[i3] * q[i11] * q[i16] + (3.596755e-04) * q[i3] * q[i11] * q[i19]
            + (-2.743292e-03) * q[i3] * q[i11] * q[i20] + (-7.055388e-04) * q[i3] * q[i11] * q[i21] + (3.205368e-04) * q[i3] * q[i11] * q[i22]
            + (-7.205732e-04) * q[i3] * q[i12] * q[i15] + (-9.703579e-04) * q[i3] * q[i12] * q[i16] + (-4.043702e-03) * q[i3] * q[i12] * q[i19]
            + (-1.148295e-03) * q[i3] * q[i12] * q[i20] + (-3.508251e-04) * q[i3] * q[i12] * q[i21] + (-1.932807e-05) * q[i3] * q[i12] * q[i22]
            + (9.508782e-05) * q[i3] * q[i15] * q[i16] + (-5.166564e-04) * q[i3] * q[i15] * q[i19] + (1.693989e-03) * q[i3] * q[i15] * q[i20]
            + (-6.911407e-06) * q[i3] * q[i15] * q[i21] + (-2.669654e-04) * q[i3] * q[i15] * q[i22] + (-2.059293e-03) * q[i3] * q[i16] * q[i19]
            + (2.422904e-03) * q[i3] * q[i16] * q[i20] + (1.136429e-04) * q[i3] * q[i16] * q[i21] + (6.164245e-04) * q[i3] * q[i16] * q[i22]
            + (-5.725087e-04) * q[i3] * q[i19] * q[i20] + (3.786057e-04) * q[i3] * q[i19] * q[i21] + (-1.428575e-03) * q[i3] * q[i19] * q[i22]
            + (1.415799e-03) * q[i3] * q[i20] * q[i21] + (2.036116e-03) * q[i3] * q[i20] * q[i22] + (-1.514408e-04) * q[i3] * q[i21] * q[i22]
            + (3.088027e-03) * q[i4] * q[i5] * q[i6] + (7.010348e-03) * q[i4] * q[i5] * q[i7] + (1.495923e-02) * q[i4] * q[i5] * q[i8]
            + (-1.137618e-03) * q[i4] * q[i5] * q[i9] + (1.745041e-04) * q[i4] * q[i5] * q[i10] + (-1.941690e-03) * q[i4] * q[i5] * q[i11]
            + (-9.799941e-04) * q[i4] * q[i5] * q[i12] + (-3.038147e-03) * q[i4] * q[i5] * q[i15] + (1.249266e-03) * q[i4] * q[i5] * q[i16]
            + (1.341935e-03) * q[i4] * q[i5] * q[i19] + (1.377475e-03) * q[i4] * q[i5] * q[i20] + (-1.064690e-03) * q[i4] * q[i5] * q[i21]
            + (1.727949e-03) * q[i4] * q[i5] * q[i22] + (1.063625e-03) * q[i4] * q[i6] * q[i7] + (1.137326e-03) * q[i4] * q[i6] * q[i8]
            + (-2.258961e-03) * q[i4] * q[i6] * q[i9] + (-8.680651e-04) * q[i4] * q[i6] * q[i10] + (-1.959240e-03) * q[i4] * q[i6] * q[i11]
            + (-4.171843e-04) * q[i4] * q[i6] * q[i12] + (-2.126183e-03) * q[i4] * q[i6] * q[i15] + (1.788607e-03) * q[i4] * q[i6] * q[i16]
            + (4.188873e-03) * q[i4] * q[i6] * q[i19] + (1.145180e-03) * q[i4] * q[i6] * q[i20] + (1.227218e-03) * q[i4] * q[i6] * q[i21]
            + (-2.634513e-03) * q[i4] * q[i6] * q[i22] + (-2.108197e-03) * q[i4] * q[i7] * q[i8] + (3.591907e-04) * q[i4] * q[i7] * q[i9]
            + (-1.167074e-03) * q[i4] * q[i7] * q[i10] + (-4.589427e-04) * q[i4] * q[i7] * q[i11] + (-6.756111e-04) * q[i4] * q[i7] * q[i12]
            + (3.312536e-03) * q[i4] * q[i7] * q[i15] + (3.367054e-03) * q[i4] * q[i7] * q[i16] + (-9.830795e-04) * q[i4] * q[i7] * q[i19]
            + (-1.690906e-03) * q[i4] * q[i7] * q[i20] + (-9.733502e-04) * q[i4] * q[i7] * q[i21] + (2.156502e-03) * q[i4] * q[i7] * q[i22]
            + (1.912230e-05) * q[i4] * q[i8] * q[i9] + (1.274215e-03) * q[i4] * q[i8] * q[i10] + (-1.178815e-03) * q[i4] * q[i8] * q[i11]
            + (-8.375840e-04) * q[i4] * q[i8] * q[i12] + (4.996041e-04) * q[i4] * q[i8] * q[i15] + (-2.130278e-03) * q[i4] * q[i8] * q[i16]
            + (-2.264376e-03) * q[i4] * q[i8] * q[i19] + (3.739202e-03) * q[i4] * q[i8] * q[i20] + (-1.838598e-03) * q[i4] * q[i8] * q[i21]
            + (-6.206917e-04) * q[i4] * q[i8] * q[i22] + (-5.872507e-04) * q[i4] * q[i9] * q[i10] + (-5.744456e-04) * q[i4] * q[i9] * q[i11]
            + (6.472719e-04) * q[i4] * q[i9] * q[i12] + (-1.838502e-03) * q[i4] * q[i9] * q[i15] + (3.111986e-04) * q[i4] * q[i9] * q[i16]
            + (-1.010468e-03) * q[i4] * q[i9] * q[i19] + (6.126302e-04) * q[i4] * q[i9] * q[i20] + (6.892232e-04) * q[i4] * q[i9] * q[i21]
            + (6.536468e-05) * q[i4] * q[i9] * q[i22] + (1.395561e-03) * q[i4] * q[i10] * q[i11] + (1.288199e-03) * q[i4] * q[i10] * q[i12]
            + (4.064886e-04) * q[i4] * q[i10] * q[i15] + (2.140959e-04) * q[i4] * q[i10] * q[i16] + (-1.031287e-03) * q[i4] * q[i10] * q[i19]
            + (1.394813e-03) * q[i4] * q[i10] * q[i20] + (-7.013622e-04) * q[i4] * q[i10] * q[i21] + (1.339857e-03) * q[i4] * q[i10] * q[i22]
            + (4.122995e-04) * q[i4] * q[i11] * q[i12] + (9.640143e-04) * q[i4] * q[i11] * q[i15] + (7.220888e-04) * q[i4] * q[i11] * q[i16]
            + (1.144213e-03) * q[i4] * q[i11] * q[i19] + (4.028919e-03) * q[i4] * q[i11] * q[i20] + (-1.348688e-05) * q[i4] * q[i11] * q[i21]
            + (-3.507120e-04) * q[i4] * q[i11] * q[i22] + (3.887333e-04) * q[i4] * q[i12] * q[i15] + (-7.257782e-05) * q[i4] * q[i12] * q[i16]
            + (2.760975e-03) * q[i4] * q[i12] * q[i19] + (-3.517347e-04) * q[i4] * q[i12] * q[i20] + (3.232380e-04) * q[i4] * q[i12] * q[i21]
            + (-7.104490e-04) * q[i4] * q[i12] * q[i22] + (1.030262e-04) * q[i4] * q[i15] * q[i16] + (2.427859e-03) * q[i4] * q[i15] * q[i19]
            + (-2.048765e-03) * q[i4] * q[i15] * q[i20] + (-6.219502e-04) * q[i4] * q[i15] * q[i21] + (-9.526688e-05) * q[i4] * q[i15] * q[i22]
            + (1.697302e-03) * q[i4] * q[i16] * q[i19] + (-5.161785e-04) * q[i4] * q[i16] * q[i20] + (2.637954e-04) * q[i4] * q[i16] * q[i21]
            + (3.106142e-05) * q[i4] * q[i16] * q[i22] + (-5.556551e-04) * q[i4] * q[i19] * q[i20] + (-2.016544e-03) * q[i4] * q[i19] * q[i21]
            + (-1.405249e-03) * q[i4] * q[i19] * q[i22] + (1.415346e-03) * q[i4] * q[i20] * q[i21] + (-3.705879e-04) * q[i4] * q[i20] * q[i22]
            + (-1.554328e-04) * q[i4] * q[i21] * q[i22] + (-5.513636e-03) * q[i5] * q[i6] * q[i7] + (-4.061282e-04) * q[i5] * q[i6] * q[i8]
            + (-2.733507e-03) * q[i5] * q[i6] * q[i9] + (2.534805e-05) * q[i5] * q[i6] * q[i10] + (2.757766e-04) * q[i5] * q[i6] * q[i11]
            + (-2.044780e-03) * q[i5] * q[i6] * q[i12] + (-3.466603e-03) * q[i5] * q[i6] * q[i15] + (-1.234001e-03) * q[i5] * q[i6] * q[i16]
            + (3.728839e-03) * q[i5] * q[i6] * q[i19] + (-2.791426e-03) * q[i5] * q[i6] * q[i20] + (2.373988e-03) * q[i5] * q[i6] * q[i21]
            + (7.504333e-04) * q[i5] * q[i6] * q[i22] + (-3.849809e-04) * q[i5] * q[i7] * q[i8] + (3.793233e-05) * q[i5] * q[i7] * q[i9]
            + (-2.653394e-03) * q[i5] * q[i7] * q[i10] + (2.087253e-03) * q[i5] * q[i7] * q[i11] + (-2.436332e-04) * q[i5] * q[i7] * q[i12]
            + (-1.246730e-03) * q[i5] * q[i7] * q[i15] + (-3.479245e-03) * q[i5] * q[i7] * q[i16] + (-2.770813e-03) * q[i5] * q[i7] * q[i19]
            + (3.707763e-03) * q[i5] * q[i7] * q[i20] + (-7.583026e-04) * q[i5] * q[i7] * q[i21] + (-2.373746e-03) * q[i5] * q[i7] * q[i22]
            + (2.207665e-03) * q[i5] * q[i8] * q[i9] + (2.197406e-03) * q[i5] * q[i8] * q[i10] + (-3.876869e-03) * q[i5] * q[i8] * q[i11]
            + (4.035952e-03) * q[i5] * q[i8] * q[i12] + (-2.887911e-03) * q[i5] * q[i8] * q[i15] + (-2.914658e-03) * q[i5] * q[i8] * q[i16]
            + (1.483302e-05) * q[i5] * q[i8] * q[i19] + (5.371657e-05) * q[i5] * q[i8] * q[i20] + (7.720520e-05) * q[i5] * q[i8] * q[i21]
            + (-1.235063e-04) * q[i5] * q[i8] * q[i22] + (1.099198e-03) * q[i5] * q[i9] * q[i10] + (3.362156e-04) * q[i5] * q[i9] * q[i11]
            + (-1.014929e-04) * q[i5] * q[i9] * q[i12] + (4.149904e-04) * q[i5] * q[i9] * q[i15] + (1.719742e-04) * q[i5] * q[i9] * q[i16]
            + (-6.854388e-04) * q[i5] * q[i9] * q[i19] + (-5.408115e-04) * q[i5] * q[i9] * q[i20] + (9.070192e-04) * q[i5] * q[i9] * q[i21]
            + (-8.920332e-04) * q[i5] * q[i9] * q[i22] + (1.083697e-04) * q[i5] * q[i10] * q[i11] + (-3.021511e-04) * q[i5] * q[i10] * q[i12]
            + (1.927727e-04) * q[i5] * q[i10] * q[i15] + (4.171139e-04) * q[i5] * q[i10] * q[i16] + (-5.324809e-04) * q[i5] * q[i10] * q[i19]
            + (-6.700685e-04) * q[i5] * q[i10] * q[i20] + (8.840609e-04) * q[i5] * q[i10] * q[i21] + (-9.083725e-04) * q[i5] * q[i10] * q[i22]
            + (-1.144017e-04) * q[i5] * q[i11] * q[i12] + (-3.131738e-03) * q[i5] * q[i11] * q[i15] + (8.952596e-04) * q[i5] * q[i11] * q[i16]
            + (-2.765632e-03) * q[i5] * q[i11] * q[i19] + (1.098577e-03) * q[i5] * q[i11] * q[i20] + (-1.005917e-03) * q[i5] * q[i11] * q[i21]
            + (4.014403e-04) * q[i5] * q[i11] * q[i22] + (-9.011617e-04) * q[i5] * q[i12] * q[i15] + (3.142238e-03) * q[i5] * q[i12] * q[i16]
            + (-1.107020e-03) * q[i5] * q[i12] * q[i19] + (2.738680e-03) * q[i5] * q[i12] * q[i20] + (4.047226e-04) * q[i5] * q[i12] * q[i21]
            + (-9.949296e-04) * q[i5] * q[i12] * q[i22] + (2.660128e-04) * q[i5] * q[i15] * q[i16] + (1.010086e-04) * q[i5] * q[i15] * q[i19]
            + (1.702609e-05) * q[i5] * q[i15] * q[i20] + (-3.191987e-04) * q[i5] * q[i15] * q[i21] + (3.559548e-04) * q[i5] * q[i15] * q[i22]
            + (1.734017e-05) * q[i5] * q[i16] * q[i19] + (1.067308e-04) * q[i5] * q[i16] * q[i20] + (-3.566293e-04) * q[i5] * q[i16] * q[i21]
            + (3.086819e-04) * q[i5] * q[i16] * q[i22] + (1.162797e-03) * q[i5] * q[i19] * q[i20] + (9.746688e-04) * q[i5] * q[i19] * q[i21]
            + (8.000354e-04) * q[i5] * q[i19] * q[i22] + (-7.983432e-04) * q[i5] * q[i20] * q[i21] + (-9.931816e-04) * q[i5] * q[i20] * q[i22]
            + (7.407358e-04) * q[i5] * q[i21] * q[i22] + (4.983881e-07) * q[i6] * q[i7] * q[i8] + (3.256982e-03) * q[i6] * q[i7] * q[i9]
            + (-3.232325e-03) * q[i6] * q[i7] * q[i10] + (9.011420e-04) * q[i6] * q[i7] * q[i11] + (8.889800e-04) * q[i6] * q[i7] * q[i12]
            + (-4.010950e-03) * q[i6] * q[i7] * q[i15] + (4.011652e-03) * q[i6] * q[i7] * q[i16] + (3.888448e-03) * q[i6] * q[i7] * q[i19]
            + (-3.886024e-03) * q[i6] * q[i7] * q[i20] + (1.910748e-03) * q[i6] * q[i7] * q[i21] + (1.899129e-03) * q[i6] * q[i7] * q[i22]
            + (6.320989e-04) * q[i6] * q[i8] * q[i9] + (8.281645e-04) * q[i6] * q[i8] * q[i10] + (1.858958e-03) * q[i6] * q[i8] * q[i11]
            + (5.226429e-04) * q[i6] * q[i8] * q[i12] + (8.525784e-04) * q[i6] * q[i8] * q[i15] + (-3.242813e-03) * q[i6] * q[i8] * q[i16]
            + (-7.519003e-04) * q[i6] * q[i8] * q[i19] + (2.245990e-03) * q[i6] * q[i8] * q[i20] + (-2.577027e-03) * q[i6] * q[i8] * q[i21]
            + (5.465064e-05) * q[i6] * q[i8] * q[i22] + (3.559246e-05) * q[i6] * q[i9] * q[i10] + (4.895574e-04) * q[i6] * q[i9] * q[i11]
            + (-1.295664e-03) * q[i6] * q[i9] * q[i12] + (9.019470e-04) * q[i6] * q[i9] * q[i15] + (-5.992658e-04) * q[i6] * q[i9] * q[i16]
            + (-2.982457e-04) * q[i6] * q[i9] * q[i19] + (1.733925e-03) * q[i6] * q[i9] * q[i20] + (5.865629e-05) * q[i6] * q[i9] * q[i21]
            + (-8.798216e-04) * q[i6] * q[i9] * q[i22] + (6.311207e-04) * q[i6] * q[i10] * q[i11] + (-2.058049e-03) * q[i6] * q[i10] * q[i12]
            + (-8.672907e-04) * q[i6] * q[i10] * q[i15] + (3.818487e-04) * q[i6] * q[i10] * q[i16] + (1.426584e-03) * q[i6] * q[i10] * q[i19]
            + (9.999129e-05) * q[i6] * q[i10] * q[i20] + (7.224236e-04) * q[i6] * q[i10] * q[i21] + (-1.862710e-04) * q[i6] * q[i10] * q[i22]
            + (-3.388114e-03) * q[i6] * q[i11] * q[i12] + (2.391041e-04) * q[i6] * q[i11] * q[i15] + (1.804663e-03) * q[i6] * q[i11] * q[i16]
            + (1.196569e-03) * q[i6] * q[i11] * q[i19] + (-8.816488e-04) * q[i6] * q[i11] * q[i20] + (1.821923e-03) * q[i6] * q[i11] * q[i21]
            + (1.017119e-03) * q[i6] * q[i11] * q[i22] + (-1.811602e-03) * q[i6] * q[i12] * q[i15] + (6.003288e-04) * q[i6] * q[i12] * q[i16]
            + (4.007865e-05) * q[i6] * q[i12] * q[i19] + (-1.552227e-03) * q[i6] * q[i12] * q[i20] + (-7.942234e-04) * q[i6] * q[i12] * q[i21]
            + (3.878863e-04) * q[i6] * q[i12] * q[i22] + (2.550293e-04) * q[i6] * q[i15] * q[i16] + (-1.196648e-03) * q[i6] * q[i15] * q[i19]
            + (1.111495e-03) * q[i6] * q[i15] * q[i20] + (-1.324569e-03) * q[i6] * q[i15] * q[i21] + (1.150947e-03) * q[i6] * q[i15] * q[i22]
            + (-3.283877e-04) * q[i6] * q[i16] * q[i19] + (-2.231642e-04) * q[i6] * q[i16] * q[i20] + (4.686602e-04) * q[i6] * q[i16] * q[i21]
            + (-3.558965e-04) * q[i6] * q[i16] * q[i22] + (2.706007e-03) * q[i6] * q[i19] * q[i20] + (-5.003558e-04) * q[i6] * q[i19] * q[i21]
            + (7.818853e-04) * q[i6] * q[i19] * q[i22] + (-2.724772e-04) * q[i6] * q[i20] * q[i21] + (-2.384966e-03) * q[i6] * q[i20] * q[i22]
            + (-4.921650e-04) * q[i6] * q[i21] * q[i22] + (-8.157522e-04) * q[i7] * q[i8] * q[i9] + (-6.236508e-04) * q[i7] * q[i8] * q[i10]
            + (4.908779e-04) * q[i7] * q[i8] * q[i11] + (1.854941e-03) * q[i7] * q[i8] * q[i12] + (3.240030e-03) * q[i7] * q[i8] * q[i15]
            + (-8.830871e-04) * q[i7] * q[i8] * q[i16] + (-2.249042e-03) * q[i7] * q[i8] * q[i19] + (7.715338e-04) * q[i7] * q[i8] * q[i20]
            + (5.454105e-05) * q[i7] * q[i8] * q[i21] + (-2.572290e-03) * q[i7] * q[i8] * q[i22] + (-3.614415e-05) * q[i7] * q[i9] * q[i10]
            + (-2.056986e-03) * q[i7] * q[i9] * q[i11] + (6.258440e-04) * q[i7] * q[i9] * q[i12] + (-3.932300e-04) * q[i7] * q[i9] * q[i15]
            + (8.676878e-04) * q[i7] * q[i9] * q[i16] + (-9.565985e-05) * q[i7] * q[i9] * q[i19] + (-1.426565e-03) * q[i7] * q[i9] * q[i20]
            + (-1.815872e-04) * q[i7] * q[i9] * q[i21] + (7.214727e-04) * q[i7] * q[i9] * q[i22] + (-1.299163e-03) * q[i7] * q[i10] * q[i11]
            + (5.004541e-04) * q[i7] * q[i10] * q[i12] + (5.882058e-04) * q[i7] * q[i10] * q[i15] + (-8.908161e-04) * q[i7] * q[i10] * q[i16]
            + (-1.729129e-03) * q[i7] * q[i10] * q[i19] + (2.999787e-04) * q[i7] * q[i10] * q[i20] + (-8.749950e-04) * q[i7] * q[i10] * q[i21]
            + (6.418850e-05) * q[i7] * q[i10] * q[i22] + (3.371243e-03) * q[i7] * q[i11] * q[i12] + (6.059683e-04) * q[i7] * q[i11] * q[i15]
            + (-1.812968e-03) * q[i7] * q[i11] * q[i16] + (-1.546261e-03) * q[i7] * q[i11] * q[i19] + (4.480873e-05) * q[i7] * q[i11] * q[i20]
            + (-3.867935e-04) * q[i7] * q[i11] * q[i21] + (7.851733e-04) * q[i7] * q[i11] * q[i22] + (1.800135e-03) * q[i7] * q[i12] * q[i15]
            + (2.377230e-04) * q[i7] * q[i12] * q[i16] + (-8.816813e-04) * q[i7] * q[i12] * q[i19] + (1.192935e-03) * q[i7] * q[i12] * q[i20]
            + (-1.011054e-03) * q[i7] * q[i12] * q[i21] + (-1.816761e-03) * q[i7] * q[i12] * q[i22] + (-2.521683e-04) * q[i7] * q[i15] * q[i16]
            + (2.188180e-04) * q[i7] * q[i15] * q[i19] + (3.129368e-04) * q[i7] * q[i15] * q[i20] + (-3.455207e-04) * q[i7] * q[i15] * q[i21]
            + (4.722414e-04) * q[i7] * q[i15] * q[i22] + (-1.106108e-03) * q[i7] * q[i16] * q[i19] + (1.191575e-03) * q[i7] * q[i16] * q[i20]
            + (1.154882e-03) * q[i7] * q[i16] * q[i21] + (-1.324447e-03) * q[i7] * q[i16] * q[i22] + (-2.697755e-03) * q[i7] * q[i19] * q[i20]
            + (-2.370805e-03) * q[i7] * q[i19] * q[i21] + (-2.713702e-04) * q[i7] * q[i19] * q[i22] + (7.750717e-04) * q[i7] * q[i20] * q[i21]
            + (-4.975113e-04) * q[i7] * q[i20] * q[i22] + (4.933090e-04) * q[i7] * q[i21] * q[i22] + (4.158587e-06) * q[i8] * q[i9] * q[i10]
            + (8.326286e-04) * q[i8] * q[i9] * q[i11] + (-4.350046e-04) * q[i8] * q[i9] * q[i12] + (-1.185753e-03) * q[i8] * q[i9] * q[i15]
            + (-2.455940e-03) * q[i8] * q[i9] * q[i16] + (-2.689159e-05) * q[i8] * q[i9] * q[i19] + (1.587067e-03) * q[i8] * q[i9] * q[i20]
            + (-7.147197e-04) * q[i8] * q[i9] * q[i21] + (2.601415e-04) * q[i8] * q[i9] * q[i22] + (-4.307190e-04) * q[i8] * q[i10] * q[i11]
            + (8.374422e-04) * q[i8] * q[i10] * q[i12] + (2.442853e-03) * q[i8] * q[i10] * q[i15] + (1.178279e-03) * q[i8] * q[i10] * q[i16]
            + (-1.579250e-03) * q[i8] * q[i10] * q[i19] + (3.105124e-05) * q[i8] * q[i10] * q[i20] + (2.566545e-04) * q[i8] * q[i10] * q[i21]
            + (-7.149864e-04) * q[i8] * q[i10] * q[i22] + (9.045604e-07) * q[i8] * q[i11] * q[i12] + (2.710908e-03) * q[i8] * q[i11] * q[i15]
            + (-2.294426e-04) * q[i8] * q[i11] * q[i16] + (-1.246248e-03) * q[i8] * q[i11] * q[i19] + (5.569696e-04) * q[i8] * q[i11] * q[i20]
            + (-6.084515e-04) * q[i8] * q[i11] * q[i21] + (-1.323098e-03) * q[i8] * q[i11] * q[i22] + (-2.242985e-04) * q[i8] * q[i12] * q[i15]
            + (2.704199e-03) * q[i8] * q[i12] * q[i16] + (5.618584e-04) * q[i8] * q[i12] * q[i19] + (-1.202175e-03) * q[i8] * q[i12] * q[i20]
            + (1.327864e-03) * q[i8] * q[i12] * q[i21] + (6.132567e-04) * q[i8] * q[i12] * q[i22] + (3.408042e-06) * q[i8] * q[i15] * q[i16]
            + (3.239668e-04) * q[i8] * q[i15] * q[i19] + (-3.857058e-04) * q[i8] * q[i15] * q[i20] + (2.597813e-03) * q[i8] * q[i15] * q[i21]
            + (-5.794559e-05) * q[i8] * q[i15] * q[i22] + (3.936908e-04) * q[i8] * q[i16] * q[i19] + (-3.308385e-04) * q[i8] * q[i16] * q[i20]
            + (-5.266935e-05) * q[i8] * q[i16] * q[i21] + (2.635654e-03) * q[i8] * q[i16] * q[i22] + (-4.539242e-07) * q[i8] * q[i19] * q[i20]
            + (2.748206e-03) * q[i8] * q[i19] * q[i21] + (-7.039320e-04) * q[i8] * q[i19] * q[i22] + (-7.043087e-04) * q[i8] * q[i20] * q[i21]
            + (2.734584e-03) * q[i8] * q[i20] * q[i22] + (1.592038e-06) * q[i8] * q[i21] * q[i22] + (-1.399252e-04) * q[i9] * q[i10] * q[i11]
            + (-1.486001e-04) * q[i9] * q[i10] * q[i12] + (-1.735716e-04) * q[i9] * q[i10] * q[i15] + (1.709716e-04) * q[i9] * q[i10] * q[i16]
            + (2.079621e-04) * q[i9] * q[i10] * q[i19] + (-2.078830e-04) * q[i9] * q[i10] * q[i20] + (7.075739e-05) * q[i9] * q[i10] * q[i21]
            + (6.639323e-05) * q[i9] * q[i10] * q[i22] + (-6.702575e-04) * q[i9] * q[i11] * q[i12] + (4.938244e-04) * q[i9] * q[i11] * q[i15]
            + (2.749411e-04) * q[i9] * q[i11] * q[i16] + (1.018745e-04) * q[i9] * q[i11] * q[i19] + (-2.517958e-04) * q[i9] * q[i11] * q[i20]
            + (5.632330e-04) * q[i9] * q[i11] * q[i21] + (2.707479e-04) * q[i9] * q[i11] * q[i22] + (-1.756373e-04) * q[i9] * q[i12] * q[i15]
            + (7.380747e-04) * q[i9] * q[i12] * q[i16] + (-8.797610e-05) * q[i9] * q[i12] * q[i19] + (-3.981859e-04) * q[i9] * q[i12] * q[i20]
            + (-2.561373e-04) * q[i9] * q[i12] * q[i21] + (-2.763025e-04) * q[i9] * q[i12] * q[i22] + (-5.587725e-04) * q[i9] * q[i15] * q[i16]
            + (-6.830786e-05) * q[i9] * q[i15] * q[i19] + (3.258650e-04) * q[i9] * q[i15] * q[i20] + (-5.439368e-04) * q[i9] * q[i15] * q[i21]
            + (4.191311e-04) * q[i9] * q[i15] * q[i22] + (-2.823206e-04) * q[i9] * q[i16] * q[i19] + (1.364311e-04) * q[i9] * q[i16] * q[i20]
            + (3.010307e-04) * q[i9] * q[i16] * q[i21] + (-6.196279e-05) * q[i9] * q[i16] * q[i22] + (7.958606e-04) * q[i9] * q[i19] * q[i20]
            + (-4.528721e-04) * q[i9] * q[i19] * q[i21] + (2.472474e-04) * q[i9] * q[i19] * q[i22] + (-5.562106e-05) * q[i9] * q[i20] * q[i21]
            + (-2.840447e-04) * q[i9] * q[i20] * q[i22] + (-1.108698e-04) * q[i9] * q[i21] * q[i22] + (6.578010e-04) * q[i10] * q[i11] * q[i12]
            + (7.359768e-04) * q[i10] * q[i11] * q[i15] + (-1.702980e-04) * q[i10] * q[i11] * q[i16] + (-3.876385e-04) * q[i10] * q[i11] * q[i19]
            + (-7.496709e-05) * q[i10] * q[i11] * q[i20] + (2.774399e-04) * q[i10] * q[i11] * q[i21] + (2.573533e-04) * q[i10] * q[i11] * q[i22]
            + (2.738951e-04) * q[i10] * q[i12] * q[i15] + (4.983328e-04) * q[i10] * q[i12] * q[i16] + (-2.560389e-04) * q[i10] * q[i12] * q[i19]
            + (1.014790e-04) * q[i10] * q[i12] * q[i20] + (-2.674550e-04) * q[i10] * q[i12] * q[i21] + (-5.623680e-04) * q[i10] * q[i12] * q[i22]
            + (5.650456e-04) * q[i10] * q[i15] * q[i16] + (-1.323603e-04) * q[i10] * q[i15] * q[i19] + (2.752441e-04) * q[i10] * q[i15] * q[i20]
            + (-5.644669e-05) * q[i10] * q[i15] * q[i21] + (3.015409e-04) * q[i10] * q[i15] * q[i22] + (-3.197884e-04) * q[i10] * q[i16] * q[i19]
            + (6.513024e-05) * q[i10] * q[i16] * q[i20] + (4.160347e-04) * q[i10] * q[i16] * q[i21] + (-5.392491e-04) * q[i10] * q[i16] * q[i22]
            + (-7.909997e-04) * q[i10] * q[i19] * q[i20] + (-2.803785e-04) * q[i10] * q[i19] * q[i21] + (-5.321534e-05) * q[i10] * q[i19] * q[i22]
            + (2.432303e-04) * q[i10] * q[i20] * q[i21] + (-4.450342e-04) * q[i10] * q[i20] * q[i22] + (1.143337e-04) * q[i10] * q[i21] * q[i22]
            + (-3.694582e-04) * q[i11] * q[i12] * q[i15] + (3.527266e-04) * q[i11] * q[i12] * q[i16] + (1.785714e-04) * q[i11] * q[i12] * q[i19]
            + (-1.718031e-04) * q[i11] * q[i12] * q[i20] + (-1.055158e-04) * q[i11] * q[i12] * q[i21] + (-1.007766e-04) * q[i11] * q[i12] * q[i22]
            + (-1.547781e-04) * q[i11] * q[i15] * q[i16] + (6.189937e-04) * q[i11] * q[i15] * q[i19] + (-5.593809e-04) * q[i11] * q[i15] * q[i20]
            + (1.712960e-03) * q[i11] * q[i15] * q[i21] + (2.039076e-04) * q[i11] * q[i15] * q[i22] + (1.117545e-04) * q[i11] * q[i16] * q[i19]
            + (-2.979092e-04) * q[i11] * q[i16] * q[i20] + (1.930622e-04) * q[i11] * q[i16] * q[i21] + (-6.535191e-05) * q[i11] * q[i16] * q[i22]
            + (2.243249e-04) * q[i11] * q[i19] * q[i20] + (1.306839e-03) * q[i11] * q[i19] * q[i21] + (4.637868e-04) * q[i11] * q[i19] * q[i22]
            + (2.009443e-05) * q[i11] * q[i20] * q[i21] + (-1.081577e-04) * q[i11] * q[i20] * q[i22] + (1.321018e-04) * q[i11] * q[i21] * q[i22]
            + (-1.589321e-04) * q[i12] * q[i15] * q[i16] + (-3.018339e-04) * q[i12] * q[i15] * q[i19] + (1.066293e-04) * q[i12] * q[i15] * q[i20]
            + (6.021189e-05) * q[i12] * q[i15] * q[i21] + (-2.014812e-04) * q[i12] * q[i15] * q[i22] + (-5.592825e-04) * q[i12] * q[i16] * q[i19]
            + (6.358665e-04) * q[i12] * q[i16] * q[i20] + (-2.030599e-04) * q[i12] * q[i16] * q[i21] + (-1.759297e-03) * q[i12] * q[i16] * q[i22]
            + (2.304211e-04) * q[i12] * q[i19] * q[i20] + (1.028135e-04) * q[i12] * q[i19] * q[i21] + (-1.867936e-05) * q[i12] * q[i19] * q[i22]
            + (-4.651508e-04) * q[i12] * q[i20] * q[i21] + (-1.291617e-03) * q[i12] * q[i20] * q[i22] + (1.320225e-04) * q[i12] * q[i21] * q[i22]
            + (3.772262e-04) * q[i15] * q[i16] * q[i19] + (-3.754970e-04) * q[i15] * q[i16] * q[i20] + (-1.755289e-04) * q[i15] * q[i16] * q[i21]
            + (-1.772365e-04) * q[i15] * q[i16] * q[i22] + (-1.729091e-04) * q[i15] * q[i19] * q[i20] + (3.298283e-04) * q[i15] * q[i19] * q[i21]
            + (-3.501043e-04) * q[i15] * q[i19] * q[i22] + (-7.171487e-05) * q[i15] * q[i20] * q[i21] + (8.631228e-05) * q[i15] * q[i20] * q[i22]
            + (1.734183e-04) * q[i15] * q[i21] * q[i22] + (1.727719e-04) * q[i16] * q[i19] * q[i20] + (8.513549e-05) * q[i16] * q[i19] * q[i21]
            + (-7.545387e-05) * q[i16] * q[i19] * q[i22] + (-3.464083e-04) * q[i16] * q[i20] * q[i21] + (3.163393e-04) * q[i16] * q[i20] * q[i22]
            + (-1.737211e-04) * q[i16] * q[i21] * q[i22] + (-1.371422e-04) * q[i19] * q[i20] * q[i21] + (-1.367320e-04) * q[i19] * q[i20] * q[i22]
            + (-6.391255e-05) * q[i19] * q[i21] * q[i22] + (6.553016e-05) * q[i20] * q[i21] * q[i22];
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
      JQ[1][i0] = (-1.010091e-02) * (1) + (-2.367657e-03) * ((2) * q[i0]) + (1.383805e-05) * (q[i1]) + (-2.185157e-04) * (q[i2]) + (-2.598147e-04) * (q[i3])
            + (-4.940217e-03) * (q[i4]) + (1.179885e-03) * (q[i5]) + (8.925438e-02) * (q[i6]) + (-1.064739e-02) * (q[i7]) + (-2.129090e-03) * (q[i8])
            + (2.868910e-02) * (q[i9]) + (-1.991742e-03) * (q[i10]) + (2.180248e-03) * (q[i11]) + (-1.271582e-03) * (q[i12]) + (6.716320e-04) * (q[i15])
            + (2.434471e-03) * (q[i16]) + (-1.256073e-03) * (q[i19]) + (4.558196e-04) * (q[i20]) + (1.785382e-03) * (q[i21]) + (7.426365e-04) * (q[i22])
            + (5.669814e-04) * ((3) * q[i0] * q[i0]) + (-9.825471e-04) * ((2) * q[i0] * q[i1]) + (1.184207e-03) * ((2) * q[i0] * q[i2])
            + (-2.992206e-02) * ((2) * q[i0] * q[i3]) + (-2.018558e-03) * ((2) * q[i0] * q[i4]) + (-2.422992e-03) * ((2) * q[i0] * q[i5])
            + (3.158567e-04) * ((2) * q[i0] * q[i6]) + (6.750960e-04) * ((2) * q[i0] * q[i7]) + (-1.325909e-03) * ((2) * q[i0] * q[i8])
            + (2.091410e-03) * ((2) * q[i0] * q[i9]) + (-4.548212e-04) * ((2) * q[i0] * q[i10]) + (5.786766e-04) * ((2) * q[i0] * q[i11])
            + (4.670338e-04) * ((2) * q[i0] * q[i12]) + (-5.193199e-04) * ((2) * q[i0] * q[i15]) + (6.033945e-04) * ((2) * q[i0] * q[i16])
            + (3.062619e-04) * ((2) * q[i0] * q[i19]) + (-9.033055e-04) * ((2) * q[i0] * q[i20]) + (5.415450e-04) * ((2) * q[i0] * q[i21])
            + (-5.642650e-04) * ((2) * q[i0] * q[i22]) + (-9.887655e-04) * (q[i1] * q[i1]) + (3.324110e-04) * (q[i2] * q[i2])
            + (-1.536166e-03) * (q[i3] * q[i3]) + (-1.931539e-03) * (q[i4] * q[i4]) + (-1.039680e-03) * (q[i5] * q[i5]) + (5.723209e-03) * (q[i6] * q[i6])
            + (-3.356871e-03) * (q[i7] * q[i7]) + (-7.268526e-05) * (q[i8] * q[i8]) + (-6.680454e-03) * (q[i9] * q[i9]) + (1.232645e-03) * (q[i10] * q[i10])
            + (3.231714e-04) * (q[i11] * q[i11]) + (1.600572e-04) * (q[i12] * q[i12]) + (1.483345e-04) * (q[i15] * q[i15]) + (1.020646e-04) * (q[i16] * q[i16])
            + (-1.449182e-04) * (q[i19] * q[i19]) + (5.001423e-05) * (q[i20] * q[i20]) + (6.503951e-05) * (q[i21] * q[i21]) + (4.423919e-06) * (q[i22] * q[i22])
            + (-1.687786e-03) * (q[i1] * q[i2]) + (8.098341e-03) * (q[i1] * q[i3]) + (8.106983e-03) * (q[i1] * q[i4]) + (-2.589524e-03) * (q[i1] * q[i5])
            + (1.561763e-03) * (q[i1] * q[i6]) + (-1.557649e-03) * (q[i1] * q[i7]) + (3.874993e-06) * (q[i1] * q[i8]) + (-1.265387e-03) * (q[i1] * q[i9])
            + (1.262403e-03) * (q[i1] * q[i10]) + (-5.326159e-04) * (q[i1] * q[i11]) + (-5.398613e-04) * (q[i1] * q[i12]) + (1.070189e-03) * (q[i1] * q[i15])
            + (-1.067377e-03) * (q[i1] * q[i16]) + (-3.577786e-04) * (q[i1] * q[i19]) + (3.680753e-04) * (q[i1] * q[i20]) + (6.375455e-04) * (q[i1] * q[i21])
            + (6.293637e-04) * (q[i1] * q[i22]) + (-1.333621e-02) * (q[i2] * q[i3]) + (-8.794458e-04) * (q[i2] * q[i4]) + (-7.566281e-03) * (q[i2] * q[i5])
            + (1.107415e-03) * (q[i2] * q[i6]) + (-5.434355e-04) * (q[i2] * q[i7]) + (-3.435710e-03) * (q[i2] * q[i8]) + (6.688534e-04) * (q[i2] * q[i9])
            + (1.871623e-04) * (q[i2] * q[i10]) + (4.646967e-04) * (q[i2] * q[i11]) + (1.638474e-03) * (q[i2] * q[i12]) + (-1.319058e-03) * (q[i2] * q[i15])
            + (2.183712e-03) * (q[i2] * q[i16]) + (5.679102e-04) * (q[i2] * q[i19]) + (9.169973e-04) * (q[i2] * q[i20]) + (4.389959e-04) * (q[i2] * q[i21])
            + (1.853998e-04) * (q[i2] * q[i22]) + (4.782162e-04) * (q[i3] * q[i4]) + (2.791141e-03) * (q[i3] * q[i5]) + (-1.013608e-02) * (q[i3] * q[i6])
            + (3.139681e-03) * (q[i3] * q[i7]) + (2.277535e-03) * (q[i3] * q[i8]) + (-5.963577e-04) * (q[i3] * q[i9]) + (2.242324e-03) * (q[i3] * q[i10])
            + (-8.594092e-04) * (q[i3] * q[i11]) + (-1.465488e-03) * (q[i3] * q[i12]) + (2.222247e-03) * (q[i3] * q[i15]) + (6.065313e-04) * (q[i3] * q[i16])
            + (9.987913e-04) * (q[i3] * q[i19]) + (3.553240e-04) * (q[i3] * q[i20]) + (-3.321404e-04) * (q[i3] * q[i21]) + (-8.913986e-04) * (q[i3] * q[i22])
            + (3.717767e-03) * (q[i4] * q[i5]) + (-2.233771e-03) * (q[i4] * q[i6]) + (9.178476e-04) * (q[i4] * q[i7]) + (-2.001233e-03) * (q[i4] * q[i8])
            + (5.563906e-04) * (q[i4] * q[i9]) + (6.042899e-04) * (q[i4] * q[i10]) + (1.327075e-03) * (q[i4] * q[i11]) + (-1.168324e-03) * (q[i4] * q[i12])
            + (2.300713e-04) * (q[i4] * q[i15]) + (-2.979545e-04) * (q[i4] * q[i16]) + (-9.249756e-05) * (q[i4] * q[i19]) + (-3.000105e-04) * (q[i4] * q[i20])
            + (1.880298e-05) * (q[i4] * q[i21]) + (-2.734117e-05) * (q[i4] * q[i22]) + (-6.703300e-03) * (q[i5] * q[i6]) + (-2.394482e-04) * (q[i5] * q[i7])
            + (3.941155e-04) * (q[i5] * q[i8]) + (-5.443572e-04) * (q[i5] * q[i9]) + (-5.315843e-04) * (q[i5] * q[i10]) + (8.634613e-04) * (q[i5] * q[i11])
            + (4.483416e-04) * (q[i5] * q[i12]) + (-6.115209e-04) * (q[i5] * q[i15]) + (-1.147948e-03) * (q[i5] * q[i16]) + (-6.164790e-04) * (q[i5] * q[i19])
            + (-3.585268e-04) * (q[i5] * q[i20]) + (2.820474e-04) * (q[i5] * q[i21]) + (4.204164e-04) * (q[i5] * q[i22]) + (-1.802536e-03) * (q[i6] * q[i7])
            + (7.060447e-03) * (q[i6] * q[i8]) + (-1.365839e-02) * (q[i6] * q[i9]) + (2.730496e-03) * (q[i6] * q[i10]) + (-1.453379e-03) * (q[i6] * q[i11])
            + (-1.077763e-03) * (q[i6] * q[i12]) + (7.646598e-04) * (q[i6] * q[i15]) + (1.406846e-03) * (q[i6] * q[i16]) + (6.747908e-04) * (q[i6] * q[i19])
            + (1.608036e-04) * (q[i6] * q[i20]) + (-7.451047e-04) * (q[i6] * q[i21]) + (8.910689e-04) * (q[i6] * q[i22]) + (3.706215e-04) * (q[i7] * q[i8])
            + (3.080839e-04) * (q[i7] * q[i9]) + (1.284695e-03) * (q[i7] * q[i10]) + (-1.745295e-03) * (q[i7] * q[i11]) + (-9.625702e-04) * (q[i7] * q[i12])
            + (-6.808599e-04) * (q[i7] * q[i15]) + (1.844087e-04) * (q[i7] * q[i16]) + (-2.213321e-04) * (q[i7] * q[i19]) + (-1.875617e-04) * (q[i7] * q[i20])
            + (1.814035e-05) * (q[i7] * q[i21]) + (-3.477295e-04) * (q[i7] * q[i22]) + (7.704389e-04) * (q[i8] * q[i9]) + (-3.394240e-04) * (q[i8] * q[i10])
            + (-1.841148e-04) * (q[i8] * q[i11]) + (1.194779e-04) * (q[i8] * q[i12]) + (7.729900e-04) * (q[i8] * q[i15]) + (7.457658e-04) * (q[i8] * q[i16])
            + (6.306952e-04) * (q[i8] * q[i19]) + (5.005176e-04) * (q[i8] * q[i20]) + (4.348939e-04) * (q[i8] * q[i21]) + (-1.241166e-04) * (q[i8] * q[i22])
            + (4.473814e-04) * (q[i9] * q[i10]) + (-8.576594e-05) * (q[i9] * q[i11]) + (-3.243681e-04) * (q[i9] * q[i12]) + (1.634297e-04) * (q[i9] * q[i15])
            + (2.365968e-05) * (q[i9] * q[i16]) + (6.612424e-04) * (q[i9] * q[i19]) + (2.772090e-04) * (q[i9] * q[i20]) + (-3.066195e-04) * (q[i9] * q[i21])
            + (-1.234293e-04) * (q[i9] * q[i22]) + (-5.594774e-04) * (q[i10] * q[i11]) + (-7.310737e-05) * (q[i10] * q[i12])
            + (3.888543e-04) * (q[i10] * q[i15]) + (-1.663085e-05) * (q[i10] * q[i16]) + (-1.775852e-04) * (q[i10] * q[i19])
            + (-4.583215e-04) * (q[i10] * q[i20]) + (-5.004782e-04) * (q[i10] * q[i21]) + (-7.364061e-05) * (q[i10] * q[i22])
            + (6.593555e-04) * (q[i11] * q[i12]) + (4.933105e-04) * (q[i11] * q[i15]) + (5.187873e-04) * (q[i11] * q[i16]) + (5.942319e-04) * (q[i11] * q[i19])
            + (-5.535935e-05) * (q[i11] * q[i20]) + (4.090679e-04) * (q[i11] * q[i21]) + (-3.014434e-04) * (q[i11] * q[i22])
            + (-2.343484e-06) * (q[i12] * q[i15]) + (-6.241420e-04) * (q[i12] * q[i16]) + (-1.363830e-04) * (q[i12] * q[i19])
            + (-3.798038e-04) * (q[i12] * q[i20]) + (1.552164e-04) * (q[i12] * q[i21]) + (-3.450505e-04) * (q[i12] * q[i22])
            + (5.167769e-04) * (q[i15] * q[i16]) + (3.401895e-05) * (q[i15] * q[i19]) + (8.032369e-06) * (q[i15] * q[i20]) + (2.456465e-04) * (q[i15] * q[i21])
            + (3.832412e-04) * (q[i15] * q[i22]) + (-6.703630e-04) * (q[i16] * q[i19]) + (1.521333e-04) * (q[i16] * q[i20]) + (2.922396e-05) * (q[i16] * q[i21])
            + (7.041708e-05) * (q[i16] * q[i22]) + (-3.255676e-05) * (q[i19] * q[i20]) + (-2.073554e-04) * (q[i19] * q[i21])
            + (-1.515405e-04) * (q[i19] * q[i22]) + (5.198957e-04) * (q[i20] * q[i21]) + (5.406636e-05) * (q[i20] * q[i22])
            + (8.145290e-05) * (q[i21] * q[i22]);
   }

   public void getJQx1(double[] q, double[][] JQ)
   {
      JQ[1][i1] = (-1.004714e-02) * (1) + (2.331259e-03) * ((2) * q[i1]) + (1.383805e-05) * (q[i0]) + (1.845959e-04) * (q[i2]) + (4.926447e-03) * (q[i3])
            + (2.307347e-04) * (q[i4]) + (-1.160811e-03) * (q[i5]) + (-1.065993e-02) * (q[i6]) + (8.875536e-02) * (q[i7]) + (-2.140533e-03) * (q[i8])
            + (-2.013038e-03) * (q[i9]) + (2.838285e-02) * (q[i10]) + (1.245275e-03) * (q[i11]) + (-2.172879e-03) * (q[i12]) + (2.446439e-03) * (q[i15])
            + (6.808327e-04) * (q[i16]) + (4.686219e-04) * (q[i19]) + (-1.242329e-03) * (q[i20]) + (-7.433376e-04) * (q[i21]) + (-1.775374e-03) * (q[i22])
            + (-9.825471e-04) * (q[i0] * q[i0]) + (-9.887655e-04) * ((2) * q[i0] * q[i1]) + (5.713060e-04) * ((3) * q[i1] * q[i1])
            + (1.177124e-03) * ((2) * q[i1] * q[i2]) + (-2.003344e-03) * ((2) * q[i1] * q[i3]) + (-2.984955e-02) * ((2) * q[i1] * q[i4])
            + (-2.404036e-03) * ((2) * q[i1] * q[i5]) + (-6.653566e-04) * ((2) * q[i1] * q[i6]) + (-3.216427e-04) * ((2) * q[i1] * q[i7])
            + (1.317663e-03) * ((2) * q[i1] * q[i8]) + (4.706956e-04) * ((2) * q[i1] * q[i9]) + (-2.075597e-03) * ((2) * q[i1] * q[i10])
            + (4.802014e-04) * ((2) * q[i1] * q[i11]) + (5.728565e-04) * ((2) * q[i1] * q[i12]) + (-6.010900e-04) * ((2) * q[i1] * q[i15])
            + (5.145223e-04) * ((2) * q[i1] * q[i16]) + (8.886021e-04) * ((2) * q[i1] * q[i19]) + (-2.973511e-04) * ((2) * q[i1] * q[i20])
            + (-5.628567e-04) * ((2) * q[i1] * q[i21]) + (5.422990e-04) * ((2) * q[i1] * q[i22]) + (3.103255e-04) * (q[i2] * q[i2])
            + (-1.951041e-03) * (q[i3] * q[i3]) + (-1.536351e-03) * (q[i4] * q[i4]) + (-1.022238e-03) * (q[i5] * q[i5]) + (-3.361309e-03) * (q[i6] * q[i6])
            + (5.746949e-03) * (q[i7] * q[i7]) + (-6.196483e-05) * (q[i8] * q[i8]) + (1.242082e-03) * (q[i9] * q[i9]) + (-6.606172e-03) * (q[i10] * q[i10])
            + (1.890957e-04) * (q[i11] * q[i11]) + (3.179734e-04) * (q[i12] * q[i12]) + (1.062923e-04) * (q[i15] * q[i15]) + (1.516463e-04) * (q[i16] * q[i16])
            + (5.154156e-05) * (q[i19] * q[i19]) + (-1.397175e-04) * (q[i20] * q[i20]) + (4.499750e-06) * (q[i21] * q[i21]) + (6.438361e-05) * (q[i22] * q[i22])
            + (-1.687786e-03) * (q[i0] * q[i2]) + (8.098341e-03) * (q[i0] * q[i3]) + (8.106983e-03) * (q[i0] * q[i4]) + (-2.589524e-03) * (q[i0] * q[i5])
            + (1.561763e-03) * (q[i0] * q[i6]) + (-1.557649e-03) * (q[i0] * q[i7]) + (3.874993e-06) * (q[i0] * q[i8]) + (-1.265387e-03) * (q[i0] * q[i9])
            + (1.262403e-03) * (q[i0] * q[i10]) + (-5.326159e-04) * (q[i0] * q[i11]) + (-5.398613e-04) * (q[i0] * q[i12]) + (1.070189e-03) * (q[i0] * q[i15])
            + (-1.067377e-03) * (q[i0] * q[i16]) + (-3.577786e-04) * (q[i0] * q[i19]) + (3.680753e-04) * (q[i0] * q[i20]) + (6.375455e-04) * (q[i0] * q[i21])
            + (6.293637e-04) * (q[i0] * q[i22]) + (-8.596076e-04) * (q[i2] * q[i3]) + (-1.326255e-02) * (q[i2] * q[i4]) + (-7.496941e-03) * (q[i2] * q[i5])
            + (5.407253e-04) * (q[i2] * q[i6]) + (-1.096792e-03) * (q[i2] * q[i7]) + (3.407788e-03) * (q[i2] * q[i8]) + (-1.748730e-04) * (q[i2] * q[i9])
            + (-6.674760e-04) * (q[i2] * q[i10]) + (1.645258e-03) * (q[i2] * q[i11]) + (4.574802e-04) * (q[i2] * q[i12]) + (-2.156562e-03) * (q[i2] * q[i15])
            + (1.311596e-03) * (q[i2] * q[i16]) + (-9.215505e-04) * (q[i2] * q[i19]) + (-5.653053e-04) * (q[i2] * q[i20]) + (1.822513e-04) * (q[i2] * q[i21])
            + (4.438889e-04) * (q[i2] * q[i22]) + (4.826837e-04) * (q[i3] * q[i4]) + (3.747658e-03) * (q[i3] * q[i5]) + (-9.422555e-04) * (q[i3] * q[i6])
            + (2.198485e-03) * (q[i3] * q[i7]) + (2.014678e-03) * (q[i3] * q[i8]) + (-6.174670e-04) * (q[i3] * q[i9]) + (-5.566213e-04) * (q[i3] * q[i10])
            + (-1.167348e-03) * (q[i3] * q[i11]) + (1.317098e-03) * (q[i3] * q[i12]) + (2.786214e-04) * (q[i3] * q[i15]) + (-2.264120e-04) * (q[i3] * q[i16])
            + (2.977698e-04) * (q[i3] * q[i19]) + (8.919473e-05) * (q[i3] * q[i20]) + (-2.511312e-05) * (q[i3] * q[i21]) + (2.563931e-05) * (q[i3] * q[i22])
            + (2.791861e-03) * (q[i4] * q[i5]) + (-3.108244e-03) * (q[i4] * q[i6]) + (1.007526e-02) * (q[i4] * q[i7]) + (-2.251224e-03) * (q[i4] * q[i8])
            + (-2.275981e-03) * (q[i4] * q[i9]) + (6.026894e-04) * (q[i4] * q[i10]) + (-1.489637e-03) * (q[i4] * q[i11]) + (-8.532372e-04) * (q[i4] * q[i12])
            + (-6.116358e-04) * (q[i4] * q[i15]) + (-2.220515e-03) * (q[i4] * q[i16]) + (-3.527346e-04) * (q[i4] * q[i19]) + (-9.849510e-04) * (q[i4] * q[i20])
            + (-8.889815e-04) * (q[i4] * q[i21]) + (-3.475137e-04) * (q[i4] * q[i22]) + (2.446108e-04) * (q[i5] * q[i6]) + (6.748017e-03) * (q[i5] * q[i7])
            + (-3.875300e-04) * (q[i5] * q[i8]) + (5.165634e-04) * (q[i5] * q[i9]) + (5.554145e-04) * (q[i5] * q[i10]) + (4.673067e-04) * (q[i5] * q[i11])
            + (8.766858e-04) * (q[i5] * q[i12]) + (1.168227e-03) * (q[i5] * q[i15]) + (6.108258e-04) * (q[i5] * q[i16]) + (3.734309e-04) * (q[i5] * q[i19])
            + (5.978117e-04) * (q[i5] * q[i20]) + (4.016762e-04) * (q[i5] * q[i21]) + (2.796101e-04) * (q[i5] * q[i22]) + (-1.795773e-03) * (q[i6] * q[i7])
            + (3.638126e-04) * (q[i6] * q[i8]) + (1.300291e-03) * (q[i6] * q[i9]) + (3.132138e-04) * (q[i6] * q[i10]) + (9.677180e-04) * (q[i6] * q[i11])
            + (1.764341e-03) * (q[i6] * q[i12]) + (1.927100e-04) * (q[i6] * q[i15]) + (-6.800327e-04) * (q[i6] * q[i16]) + (-1.865585e-04) * (q[i6] * q[i19])
            + (-2.142370e-04) * (q[i6] * q[i20]) + (3.399222e-04) * (q[i6] * q[i21]) + (-2.211472e-05) * (q[i6] * q[i22]) + (7.035108e-03) * (q[i7] * q[i8])
            + (2.754172e-03) * (q[i7] * q[i9]) + (-1.349584e-02) * (q[i7] * q[i10]) + (1.099982e-03) * (q[i7] * q[i11]) + (1.474185e-03) * (q[i7] * q[i12])
            + (1.383560e-03) * (q[i7] * q[i15]) + (7.824298e-04) * (q[i7] * q[i16]) + (1.553686e-04) * (q[i7] * q[i19]) + (6.718480e-04) * (q[i7] * q[i20])
            + (-8.756166e-04) * (q[i7] * q[i21]) + (7.638772e-04) * (q[i7] * q[i22]) + (-3.468415e-04) * (q[i8] * q[i9]) + (7.726777e-04) * (q[i8] * q[i10])
            + (-7.515004e-05) * (q[i8] * q[i11]) + (2.155798e-04) * (q[i8] * q[i12]) + (7.261643e-04) * (q[i8] * q[i15]) + (7.741995e-04) * (q[i8] * q[i16])
            + (5.053310e-04) * (q[i8] * q[i19]) + (6.299696e-04) * (q[i8] * q[i20]) + (1.145841e-04) * (q[i8] * q[i21]) + (-4.322135e-04) * (q[i8] * q[i22])
            + (4.457035e-04) * (q[i9] * q[i10]) + (7.662144e-05) * (q[i9] * q[i11]) + (5.675472e-04) * (q[i9] * q[i12]) + (-2.443037e-05) * (q[i9] * q[i15])
            + (3.857545e-04) * (q[i9] * q[i16]) + (-4.597970e-04) * (q[i9] * q[i19]) + (-1.791769e-04) * (q[i9] * q[i20]) + (7.667513e-05) * (q[i9] * q[i21])
            + (5.041266e-04) * (q[i9] * q[i22]) + (3.202169e-04) * (q[i10] * q[i11]) + (8.348494e-05) * (q[i10] * q[i12]) + (1.561949e-05) * (q[i10] * q[i15])
            + (1.650535e-04) * (q[i10] * q[i16]) + (2.768153e-04) * (q[i10] * q[i19]) + (6.625426e-04) * (q[i10] * q[i20]) + (1.233639e-04) * (q[i10] * q[i21])
            + (2.979040e-04) * (q[i10] * q[i22]) + (6.634929e-04) * (q[i11] * q[i12]) + (6.052294e-04) * (q[i11] * q[i15]) + (8.497943e-07) * (q[i11] * q[i16])
            + (3.752991e-04) * (q[i11] * q[i19]) + (1.283739e-04) * (q[i11] * q[i20]) + (-3.525242e-04) * (q[i11] * q[i21]) + (1.552551e-04) * (q[i11] * q[i22])
            + (-5.293381e-04) * (q[i12] * q[i15]) + (-4.937368e-04) * (q[i12] * q[i16]) + (5.602229e-05) * (q[i12] * q[i19])
            + (-5.984771e-04) * (q[i12] * q[i20]) + (-2.965218e-04) * (q[i12] * q[i21]) + (4.076207e-04) * (q[i12] * q[i22])
            + (5.077336e-04) * (q[i15] * q[i16]) + (1.600298e-04) * (q[i15] * q[i19]) + (-6.659789e-04) * (q[i15] * q[i20])
            + (-7.249146e-05) * (q[i15] * q[i21]) + (-3.137002e-05) * (q[i15] * q[i22]) + (1.494955e-05) * (q[i16] * q[i19])
            + (3.872082e-05) * (q[i16] * q[i20]) + (-3.747243e-04) * (q[i16] * q[i21]) + (-2.484144e-04) * (q[i16] * q[i22])
            + (-4.926675e-05) * (q[i19] * q[i20]) + (-5.847086e-05) * (q[i19] * q[i21]) + (-5.140003e-04) * (q[i19] * q[i22])
            + (1.523064e-04) * (q[i20] * q[i21]) + (2.056473e-04) * (q[i20] * q[i22]) + (8.272157e-05) * (q[i21] * q[i22]);
   }

   public void getJQx2(double[] q, double[][] JQ)
   {
      JQ[1][i2] = (1.441417e-03) * (1) + (-1.981026e-05) * ((2) * q[i2]) + (-2.185157e-04) * (q[i0]) + (1.845959e-04) * (q[i1]) + (-2.013881e-05) * (q[i3])
            + (-3.492671e-05) * (q[i4]) + (7.287988e-05) * (q[i5]) + (1.497659e-02) * (q[i6]) + (1.484636e-02) * (q[i7]) + (-6.425181e-02) * (q[i8])
            + (1.110663e-03) * (q[i9]) + (1.109812e-03) * (q[i10]) + (5.822078e-03) * (q[i11]) + (-5.992994e-03) * (q[i12]) + (3.217467e-03) * (q[i15])
            + (3.282658e-03) * (q[i16]) + (-1.830097e-03) * (q[i19]) + (-1.825938e-03) * (q[i20]) + (8.509005e-04) * (q[i21]) + (-8.366886e-04) * (q[i22])
            + (1.184207e-03) * (q[i0] * q[i0]) + (1.177124e-03) * (q[i1] * q[i1]) + (3.324110e-04) * ((2) * q[i0] * q[i2])
            + (3.103255e-04) * ((2) * q[i1] * q[i2]) + (2.386629e-03) * ((3) * q[i2] * q[i2]) + (-2.506449e-03) * ((2) * q[i2] * q[i3])
            + (-2.486643e-03) * ((2) * q[i2] * q[i4]) + (-3.008131e-02) * ((2) * q[i2] * q[i5]) + (9.338505e-05) * ((2) * q[i2] * q[i6])
            + (-9.476205e-05) * ((2) * q[i2] * q[i7]) + (-2.584377e-05) * ((2) * q[i2] * q[i8]) + (-7.424691e-05) * ((2) * q[i2] * q[i9])
            + (6.551763e-05) * ((2) * q[i2] * q[i10]) + (6.818229e-04) * ((2) * q[i2] * q[i11]) + (6.679535e-04) * ((2) * q[i2] * q[i12])
            + (-3.073569e-03) * ((2) * q[i2] * q[i15]) + (3.108143e-03) * ((2) * q[i2] * q[i16]) + (-2.805107e-04) * ((2) * q[i2] * q[i19])
            + (2.853590e-04) * ((2) * q[i2] * q[i20]) + (4.678450e-04) * ((2) * q[i2] * q[i21]) + (4.736763e-04) * ((2) * q[i2] * q[i22])
            + (-2.706366e-03) * (q[i3] * q[i3]) + (-2.704664e-03) * (q[i4] * q[i4]) + (-9.399917e-04) * (q[i5] * q[i5]) + (6.404787e-04) * (q[i6] * q[i6])
            + (6.425832e-04) * (q[i7] * q[i7]) + (2.673863e-03) * (q[i8] * q[i8]) + (-1.726621e-03) * (q[i9] * q[i9]) + (-1.701862e-03) * (q[i10] * q[i10])
            + (8.918757e-05) * (q[i11] * q[i11]) + (-4.404651e-06) * (q[i12] * q[i12]) + (1.988861e-03) * (q[i15] * q[i15]) + (2.009771e-03) * (q[i16] * q[i16])
            + (-1.105789e-04) * (q[i19] * q[i19]) + (-1.112106e-04) * (q[i20] * q[i20]) + (1.895148e-04) * (q[i21] * q[i21])
            + (1.868505e-04) * (q[i22] * q[i22]) + (-1.687786e-03) * (q[i0] * q[i1]) + (-1.333621e-02) * (q[i0] * q[i3]) + (-8.794458e-04) * (q[i0] * q[i4])
            + (-7.566281e-03) * (q[i0] * q[i5]) + (1.107415e-03) * (q[i0] * q[i6]) + (-5.434355e-04) * (q[i0] * q[i7]) + (-3.435710e-03) * (q[i0] * q[i8])
            + (6.688534e-04) * (q[i0] * q[i9]) + (1.871623e-04) * (q[i0] * q[i10]) + (4.646967e-04) * (q[i0] * q[i11]) + (1.638474e-03) * (q[i0] * q[i12])
            + (-1.319058e-03) * (q[i0] * q[i15]) + (2.183712e-03) * (q[i0] * q[i16]) + (5.679102e-04) * (q[i0] * q[i19]) + (9.169973e-04) * (q[i0] * q[i20])
            + (4.389959e-04) * (q[i0] * q[i21]) + (1.853998e-04) * (q[i0] * q[i22]) + (-8.596076e-04) * (q[i1] * q[i3]) + (-1.326255e-02) * (q[i1] * q[i4])
            + (-7.496941e-03) * (q[i1] * q[i5]) + (5.407253e-04) * (q[i1] * q[i6]) + (-1.096792e-03) * (q[i1] * q[i7]) + (3.407788e-03) * (q[i1] * q[i8])
            + (-1.748730e-04) * (q[i1] * q[i9]) + (-6.674760e-04) * (q[i1] * q[i10]) + (1.645258e-03) * (q[i1] * q[i11]) + (4.574802e-04) * (q[i1] * q[i12])
            + (-2.156562e-03) * (q[i1] * q[i15]) + (1.311596e-03) * (q[i1] * q[i16]) + (-9.215505e-04) * (q[i1] * q[i19]) + (-5.653053e-04) * (q[i1] * q[i20])
            + (1.822513e-04) * (q[i1] * q[i21]) + (4.438889e-04) * (q[i1] * q[i22]) + (9.854905e-04) * (q[i3] * q[i4]) + (3.739732e-03) * (q[i3] * q[i5])
            + (-2.893629e-03) * (q[i3] * q[i6]) + (4.047955e-04) * (q[i3] * q[i7]) + (2.254073e-03) * (q[i3] * q[i8]) + (8.912089e-05) * (q[i3] * q[i9])
            + (1.396181e-04) * (q[i3] * q[i10]) + (-1.113976e-04) * (q[i3] * q[i11]) + (2.118474e-04) * (q[i3] * q[i12]) + (1.898353e-03) * (q[i3] * q[i15])
            + (-4.379823e-04) * (q[i3] * q[i16]) + (-6.730639e-04) * (q[i3] * q[i19]) + (-1.564297e-04) * (q[i3] * q[i20]) + (-8.784506e-04) * (q[i3] * q[i21])
            + (5.838644e-04) * (q[i3] * q[i22]) + (3.749981e-03) * (q[i4] * q[i5]) + (-3.893894e-04) * (q[i4] * q[i6]) + (2.878982e-03) * (q[i4] * q[i7])
            + (-2.213145e-03) * (q[i4] * q[i8]) + (-1.504482e-04) * (q[i4] * q[i9]) + (-7.647034e-05) * (q[i4] * q[i10]) + (1.860122e-04) * (q[i4] * q[i11])
            + (-8.613907e-05) * (q[i4] * q[i12]) + (4.359900e-04) * (q[i4] * q[i15]) + (-1.911338e-03) * (q[i4] * q[i16]) + (1.579609e-04) * (q[i4] * q[i19])
            + (6.575876e-04) * (q[i4] * q[i20]) + (5.731572e-04) * (q[i4] * q[i21]) + (-8.803963e-04) * (q[i4] * q[i22]) + (8.400495e-04) * (q[i5] * q[i6])
            + (-8.526404e-04) * (q[i5] * q[i7]) + (-4.747920e-05) * (q[i5] * q[i8]) + (1.378678e-03) * (q[i5] * q[i9]) + (-1.394716e-03) * (q[i5] * q[i10])
            + (1.879672e-03) * (q[i5] * q[i11]) + (1.853627e-03) * (q[i5] * q[i12]) + (-2.309920e-03) * (q[i5] * q[i15]) + (2.307184e-03) * (q[i5] * q[i16])
            + (1.426304e-03) * (q[i5] * q[i19]) + (-1.426429e-03) * (q[i5] * q[i20]) + (8.077700e-04) * (q[i5] * q[i21]) + (7.973751e-04) * (q[i5] * q[i22])
            + (3.171142e-04) * (q[i6] * q[i7]) + (6.081805e-03) * (q[i6] * q[i8]) + (-3.521530e-03) * (q[i6] * q[i9]) + (1.440729e-03) * (q[i6] * q[i10])
            + (-8.145888e-04) * (q[i6] * q[i11]) + (-1.063741e-04) * (q[i6] * q[i12]) + (6.365126e-04) * (q[i6] * q[i15]) + (-4.380484e-04) * (q[i6] * q[i16])
            + (1.961629e-04) * (q[i6] * q[i19]) + (3.576040e-05) * (q[i6] * q[i20]) + (3.222426e-04) * (q[i6] * q[i21]) + (2.455748e-04) * (q[i6] * q[i22])
            + (6.089197e-03) * (q[i7] * q[i8]) + (1.449597e-03) * (q[i7] * q[i9]) + (-3.469230e-03) * (q[i7] * q[i10]) + (1.189835e-04) * (q[i7] * q[i11])
            + (8.198402e-04) * (q[i7] * q[i12]) + (-4.190663e-04) * (q[i7] * q[i15]) + (6.364879e-04) * (q[i7] * q[i16]) + (3.483468e-05) * (q[i7] * q[i19])
            + (2.011984e-04) * (q[i7] * q[i20]) + (-2.421668e-04) * (q[i7] * q[i21]) + (-3.191185e-04) * (q[i7] * q[i22]) + (-2.776862e-03) * (q[i8] * q[i9])
            + (-2.736449e-03) * (q[i8] * q[i10]) + (-6.921279e-04) * (q[i8] * q[i11]) + (8.612909e-04) * (q[i8] * q[i12]) + (5.179766e-03) * (q[i8] * q[i15])
            + (5.254888e-03) * (q[i8] * q[i16]) + (4.322037e-04) * (q[i8] * q[i19]) + (4.214895e-04) * (q[i8] * q[i20]) + (-3.998145e-04) * (q[i8] * q[i21])
            + (4.081732e-04) * (q[i8] * q[i22]) + (-1.672587e-05) * (q[i9] * q[i10]) + (-5.373416e-04) * (q[i9] * q[i11]) + (3.698614e-06) * (q[i9] * q[i12])
            + (1.200645e-04) * (q[i9] * q[i15]) + (-2.390041e-04) * (q[i9] * q[i16]) + (1.394536e-04) * (q[i9] * q[i19]) + (2.306254e-04) * (q[i9] * q[i20])
            + (-2.424145e-04) * (q[i9] * q[i21]) + (7.810359e-05) * (q[i9] * q[i22]) + (-2.125501e-05) * (q[i10] * q[i11]) + (5.232176e-04) * (q[i10] * q[i12])
            + (-2.334638e-04) * (q[i10] * q[i15]) + (1.254069e-04) * (q[i10] * q[i16]) + (2.362313e-04) * (q[i10] * q[i19]) + (1.414497e-04) * (q[i10] * q[i20])
            + (-7.869362e-05) * (q[i10] * q[i21]) + (2.386402e-04) * (q[i10] * q[i22]) + (6.074138e-04) * (q[i11] * q[i12]) + (3.323139e-03) * (q[i11] * q[i15])
            + (-4.022756e-04) * (q[i11] * q[i16]) + (1.742883e-03) * (q[i11] * q[i19]) + (5.947629e-05) * (q[i11] * q[i20]) + (2.158425e-04) * (q[i11] * q[i21])
            + (-4.507722e-04) * (q[i11] * q[i22]) + (3.809445e-04) * (q[i12] * q[i15]) + (-3.340560e-03) * (q[i12] * q[i16])
            + (-7.327170e-05) * (q[i12] * q[i19]) + (-1.726330e-03) * (q[i12] * q[i20]) + (-4.512358e-04) * (q[i12] * q[i21])
            + (2.083152e-04) * (q[i12] * q[i22]) + (5.847767e-05) * (q[i15] * q[i16]) + (-9.102059e-04) * (q[i15] * q[i19])
            + (-7.358046e-04) * (q[i15] * q[i20]) + (1.149993e-04) * (q[i15] * q[i21]) + (2.249188e-04) * (q[i15] * q[i22])
            + (-7.419841e-04) * (q[i16] * q[i19]) + (-9.052824e-04) * (q[i16] * q[i20]) + (-2.212101e-04) * (q[i16] * q[i21])
            + (-1.195472e-04) * (q[i16] * q[i22]) + (1.839152e-04) * (q[i19] * q[i20]) + (-6.358527e-04) * (q[i19] * q[i21])
            + (-3.780284e-04) * (q[i19] * q[i22]) + (3.801782e-04) * (q[i20] * q[i21]) + (6.258241e-04) * (q[i20] * q[i22])
            + (8.527480e-05) * (q[i21] * q[i22]);
   }

   public void getJQx3(double[] q, double[][] JQ)
   {
      JQ[1][i3] = (1.310380e-01) * (1) + (1.070055e-02) * ((2) * q[i3]) + (-2.598147e-04) * (q[i0]) + (4.926447e-03) * (q[i1]) + (-2.013881e-05) * (q[i2])
            + (-8.651343e-07) * (q[i4]) + (-9.554202e-03) * (q[i5]) + (1.090700e-02) * (q[i6]) + (-5.409597e-03) * (q[i7]) + (2.761238e-03) * (q[i8])
            + (-8.353152e-03) * (q[i9]) + (4.679714e-03) * (q[i10]) + (-1.397326e-03) * (q[i11]) + (1.370022e-03) * (q[i12]) + (4.420613e-03) * (q[i15])
            + (4.782723e-03) * (q[i16]) + (-7.273987e-04) * (q[i19]) + (1.193762e-03) * (q[i20]) + (-6.352294e-04) * (q[i21]) + (3.072453e-04) * (q[i22])
            + (-2.992206e-02) * (q[i0] * q[i0]) + (-2.003344e-03) * (q[i1] * q[i1]) + (-2.506449e-03) * (q[i2] * q[i2])
            + (-1.536166e-03) * ((2) * q[i0] * q[i3]) + (-1.951041e-03) * ((2) * q[i1] * q[i3]) + (-2.706366e-03) * ((2) * q[i2] * q[i3])
            + (-2.965197e-03) * ((3) * q[i3] * q[i3]) + (-1.202879e-03) * ((2) * q[i3] * q[i4]) + (4.928136e-03) * ((2) * q[i3] * q[i5])
            + (6.705721e-04) * ((2) * q[i3] * q[i6]) + (-1.513164e-03) * ((2) * q[i3] * q[i7]) + (5.034326e-04) * ((2) * q[i3] * q[i8])
            + (-5.224901e-04) * ((2) * q[i3] * q[i9]) + (-8.534385e-04) * ((2) * q[i3] * q[i10]) + (-8.827741e-04) * ((2) * q[i3] * q[i11])
            + (-1.418222e-03) * ((2) * q[i3] * q[i12]) + (8.495688e-05) * ((2) * q[i3] * q[i15]) + (7.292597e-05) * ((2) * q[i3] * q[i16])
            + (-4.504642e-04) * ((2) * q[i3] * q[i19]) + (4.050251e-04) * ((2) * q[i3] * q[i20]) + (-5.546236e-05) * ((2) * q[i3] * q[i21])
            + (1.992528e-04) * ((2) * q[i3] * q[i22]) + (-1.222937e-03) * (q[i4] * q[i4]) + (4.794269e-05) * (q[i5] * q[i5]) + (-2.172440e-02) * (q[i6] * q[i6])
            + (3.708187e-03) * (q[i7] * q[i7]) + (4.515987e-03) * (q[i8] * q[i8]) + (-3.512466e-03) * (q[i9] * q[i9]) + (7.635137e-04) * (q[i10] * q[i10])
            + (-3.944303e-04) * (q[i11] * q[i11]) + (1.547685e-05) * (q[i12] * q[i12]) + (2.427431e-04) * (q[i15] * q[i15]) + (6.422297e-04) * (q[i16] * q[i16])
            + (-3.432702e-05) * (q[i19] * q[i19]) + (2.891431e-05) * (q[i20] * q[i20]) + (4.343329e-05) * (q[i21] * q[i21]) + (6.041209e-04) * (q[i22] * q[i22])
            + (8.098341e-03) * (q[i0] * q[i1]) + (-1.333621e-02) * (q[i0] * q[i2]) + (4.782162e-04) * (q[i0] * q[i4]) + (2.791141e-03) * (q[i0] * q[i5])
            + (-1.013608e-02) * (q[i0] * q[i6]) + (3.139681e-03) * (q[i0] * q[i7]) + (2.277535e-03) * (q[i0] * q[i8]) + (-5.963577e-04) * (q[i0] * q[i9])
            + (2.242324e-03) * (q[i0] * q[i10]) + (-8.594092e-04) * (q[i0] * q[i11]) + (-1.465488e-03) * (q[i0] * q[i12]) + (2.222247e-03) * (q[i0] * q[i15])
            + (6.065313e-04) * (q[i0] * q[i16]) + (9.987913e-04) * (q[i0] * q[i19]) + (3.553240e-04) * (q[i0] * q[i20]) + (-3.321404e-04) * (q[i0] * q[i21])
            + (-8.913986e-04) * (q[i0] * q[i22]) + (-8.596076e-04) * (q[i1] * q[i2]) + (4.826837e-04) * (q[i1] * q[i4]) + (3.747658e-03) * (q[i1] * q[i5])
            + (-9.422555e-04) * (q[i1] * q[i6]) + (2.198485e-03) * (q[i1] * q[i7]) + (2.014678e-03) * (q[i1] * q[i8]) + (-6.174670e-04) * (q[i1] * q[i9])
            + (-5.566213e-04) * (q[i1] * q[i10]) + (-1.167348e-03) * (q[i1] * q[i11]) + (1.317098e-03) * (q[i1] * q[i12]) + (2.786214e-04) * (q[i1] * q[i15])
            + (-2.264120e-04) * (q[i1] * q[i16]) + (2.977698e-04) * (q[i1] * q[i19]) + (8.919473e-05) * (q[i1] * q[i20]) + (-2.511312e-05) * (q[i1] * q[i21])
            + (2.563931e-05) * (q[i1] * q[i22]) + (9.854905e-04) * (q[i2] * q[i4]) + (3.739732e-03) * (q[i2] * q[i5]) + (-2.893629e-03) * (q[i2] * q[i6])
            + (4.047955e-04) * (q[i2] * q[i7]) + (2.254073e-03) * (q[i2] * q[i8]) + (8.912089e-05) * (q[i2] * q[i9]) + (1.396181e-04) * (q[i2] * q[i10])
            + (-1.113976e-04) * (q[i2] * q[i11]) + (2.118474e-04) * (q[i2] * q[i12]) + (1.898353e-03) * (q[i2] * q[i15]) + (-4.379823e-04) * (q[i2] * q[i16])
            + (-6.730639e-04) * (q[i2] * q[i19]) + (-1.564297e-04) * (q[i2] * q[i20]) + (-8.784506e-04) * (q[i2] * q[i21]) + (5.838644e-04) * (q[i2] * q[i22])
            + (-1.760935e-04) * (q[i4] * q[i5]) + (1.609779e-03) * (q[i4] * q[i6]) + (-1.611773e-03) * (q[i4] * q[i7]) + (5.305136e-06) * (q[i4] * q[i8])
            + (-9.559773e-04) * (q[i4] * q[i9]) + (9.555538e-04) * (q[i4] * q[i10]) + (1.488707e-03) * (q[i4] * q[i11]) + (1.482513e-03) * (q[i4] * q[i12])
            + (-2.339222e-05) * (q[i4] * q[i15]) + (3.199603e-05) * (q[i4] * q[i16]) + (-1.143179e-03) * (q[i4] * q[i19]) + (1.148663e-03) * (q[i4] * q[i20])
            + (-5.856657e-04) * (q[i4] * q[i21]) + (-5.908238e-04) * (q[i4] * q[i22]) + (-3.556932e-03) * (q[i5] * q[i6]) + (-5.519686e-04) * (q[i5] * q[i7])
            + (3.825928e-04) * (q[i5] * q[i8]) + (6.208945e-04) * (q[i5] * q[i9]) + (-9.887132e-04) * (q[i5] * q[i10]) + (-2.169193e-03) * (q[i5] * q[i11])
            + (5.292480e-04) * (q[i5] * q[i12]) + (1.139528e-03) * (q[i5] * q[i15]) + (3.817097e-04) * (q[i5] * q[i16]) + (6.850017e-06) * (q[i5] * q[i19])
            + (-1.682617e-03) * (q[i5] * q[i20]) + (9.284001e-05) * (q[i5] * q[i21]) + (2.950487e-04) * (q[i5] * q[i22]) + (7.620428e-05) * (q[i6] * q[i7])
            + (-1.833501e-03) * (q[i6] * q[i8]) + (-1.084537e-02) * (q[i6] * q[i9]) + (5.088915e-04) * (q[i6] * q[i10]) + (-5.300235e-04) * (q[i6] * q[i11])
            + (1.113857e-03) * (q[i6] * q[i12]) + (8.047113e-04) * (q[i6] * q[i15]) + (-7.632352e-04) * (q[i6] * q[i16]) + (-1.925811e-03) * (q[i6] * q[i19])
            + (4.617432e-04) * (q[i6] * q[i20]) + (-3.234240e-04) * (q[i6] * q[i21]) + (9.169762e-04) * (q[i6] * q[i22]) + (-1.648222e-03) * (q[i7] * q[i8])
            + (4.427378e-04) * (q[i7] * q[i9]) + (2.244056e-03) * (q[i7] * q[i10]) + (3.831200e-04) * (q[i7] * q[i11]) + (5.663898e-04) * (q[i7] * q[i12])
            + (-1.599024e-03) * (q[i7] * q[i15]) + (-9.085843e-04) * (q[i7] * q[i16]) + (-1.873487e-04) * (q[i7] * q[i19]) + (4.093100e-04) * (q[i7] * q[i20])
            + (6.429751e-04) * (q[i7] * q[i21]) + (4.012098e-04) * (q[i7] * q[i22]) + (-7.396036e-04) * (q[i8] * q[i9]) + (1.941091e-04) * (q[i8] * q[i10])
            + (-1.732590e-03) * (q[i8] * q[i11]) + (6.654939e-04) * (q[i8] * q[i12]) + (-6.313806e-04) * (q[i8] * q[i15]) + (-7.923113e-05) * (q[i8] * q[i16])
            + (-4.347447e-05) * (q[i8] * q[i19]) + (-6.697361e-05) * (q[i8] * q[i20]) + (6.391445e-04) * (q[i8] * q[i21]) + (-4.429403e-04) * (q[i8] * q[i22])
            + (-4.205340e-04) * (q[i9] * q[i10]) + (-2.376146e-04) * (q[i9] * q[i11]) + (3.003056e-04) * (q[i9] * q[i12]) + (-2.719674e-04) * (q[i9] * q[i15])
            + (-5.925024e-04) * (q[i9] * q[i16]) + (-3.112606e-04) * (q[i9] * q[i19]) + (-1.554147e-04) * (q[i9] * q[i20]) + (1.320513e-04) * (q[i9] * q[i21])
            + (-3.187391e-04) * (q[i9] * q[i22]) + (2.053841e-04) * (q[i10] * q[i11]) + (4.718133e-04) * (q[i10] * q[i12]) + (-3.623644e-04) * (q[i10] * q[i15])
            + (3.846383e-04) * (q[i10] * q[i16]) + (2.522698e-04) * (q[i10] * q[i19]) + (3.036936e-05) * (q[i10] * q[i20]) + (-1.085302e-04) * (q[i10] * q[i21])
            + (1.945716e-04) * (q[i10] * q[i22]) + (-1.561455e-04) * (q[i11] * q[i12]) + (-1.538416e-03) * (q[i11] * q[i15])
            + (7.868216e-04) * (q[i11] * q[i16]) + (4.310239e-05) * (q[i11] * q[i19]) + (3.062487e-05) * (q[i11] * q[i20]) + (-4.160914e-04) * (q[i11] * q[i21])
            + (-2.337426e-04) * (q[i11] * q[i22]) + (5.428375e-04) * (q[i12] * q[i15]) + (-5.822735e-05) * (q[i12] * q[i16])
            + (2.744032e-05) * (q[i12] * q[i19]) + (4.514248e-04) * (q[i12] * q[i20]) + (1.865373e-04) * (q[i12] * q[i21]) + (-2.017979e-04) * (q[i12] * q[i22])
            + (3.354747e-04) * (q[i15] * q[i16]) + (-6.221902e-04) * (q[i15] * q[i19]) + (1.336437e-04) * (q[i15] * q[i20]) + (1.475200e-04) * (q[i15] * q[i21])
            + (2.283656e-04) * (q[i15] * q[i22]) + (3.829431e-04) * (q[i16] * q[i19]) + (3.285342e-04) * (q[i16] * q[i20]) + (-8.378908e-05) * (q[i16] * q[i21])
            + (-4.900784e-04) * (q[i16] * q[i22]) + (4.188005e-04) * (q[i19] * q[i20]) + (5.394042e-04) * (q[i19] * q[i21])
            + (-6.136038e-04) * (q[i19] * q[i22]) + (-3.209858e-04) * (q[i20] * q[i21]) + (-3.933443e-04) * (q[i20] * q[i22])
            + (8.998727e-05) * (q[i21] * q[i22]);
   }

   public void getJQx4(double[] q, double[][] JQ)
   {
      JQ[1][i4] = (1.304297e-01) * (1) + (-1.064344e-02) * ((2) * q[i4]) + (-4.940217e-03) * (q[i0]) + (2.307347e-04) * (q[i1]) + (-3.492671e-05) * (q[i2])
            + (-8.651343e-07) * (q[i3]) + (9.530658e-03) * (q[i5]) + (-5.418235e-03) * (q[i6]) + (1.087283e-02) * (q[i7]) + (2.762834e-03) * (q[i8])
            + (4.705070e-03) * (q[i9]) + (-8.281846e-03) * (q[i10]) + (-1.337685e-03) * (q[i11]) + (1.413711e-03) * (q[i12]) + (4.708550e-03) * (q[i15])
            + (4.447938e-03) * (q[i16]) + (1.189330e-03) * (q[i19]) + (-7.088497e-04) * (q[i20]) + (-2.964353e-04) * (q[i21]) + (6.134833e-04) * (q[i22])
            + (-2.018558e-03) * (q[i0] * q[i0]) + (-2.984955e-02) * (q[i1] * q[i1]) + (-2.486643e-03) * (q[i2] * q[i2]) + (-1.202879e-03) * (q[i3] * q[i3])
            + (-1.931539e-03) * ((2) * q[i0] * q[i4]) + (-1.536351e-03) * ((2) * q[i1] * q[i4]) + (-2.704664e-03) * ((2) * q[i2] * q[i4])
            + (-1.222937e-03) * ((2) * q[i3] * q[i4]) + (-2.979427e-03) * ((3) * q[i4] * q[i4]) + (4.909451e-03) * ((2) * q[i4] * q[i5])
            + (1.513751e-03) * ((2) * q[i4] * q[i6]) + (-6.565926e-04) * ((2) * q[i4] * q[i7]) + (-4.924872e-04) * ((2) * q[i4] * q[i8])
            + (8.403889e-04) * ((2) * q[i4] * q[i9]) + (5.077441e-04) * ((2) * q[i4] * q[i10]) + (-1.417677e-03) * ((2) * q[i4] * q[i11])
            + (-8.833817e-04) * ((2) * q[i4] * q[i12]) + (-6.731487e-05) * ((2) * q[i4] * q[i15]) + (-8.986242e-05) * ((2) * q[i4] * q[i16])
            + (-3.920504e-04) * ((2) * q[i4] * q[i19]) + (4.323403e-04) * ((2) * q[i4] * q[i20]) + (2.000950e-04) * ((2) * q[i4] * q[i21])
            + (-6.013280e-05) * ((2) * q[i4] * q[i22]) + (8.117997e-05) * (q[i5] * q[i5]) + (3.718044e-03) * (q[i6] * q[i6]) + (-2.165774e-02) * (q[i7] * q[i7])
            + (4.518467e-03) * (q[i8] * q[i8]) + (7.681636e-04) * (q[i9] * q[i9]) + (-3.469145e-03) * (q[i10] * q[i10]) + (-9.777769e-06) * (q[i11] * q[i11])
            + (-4.022853e-04) * (q[i12] * q[i12]) + (6.388368e-04) * (q[i15] * q[i15]) + (2.455638e-04) * (q[i16] * q[i16]) + (3.278650e-05) * (q[i19] * q[i19])
            + (-3.742291e-05) * (q[i20] * q[i20]) + (5.999856e-04) * (q[i21] * q[i21]) + (4.531129e-05) * (q[i22] * q[i22]) + (8.106983e-03) * (q[i0] * q[i1])
            + (-8.794458e-04) * (q[i0] * q[i2]) + (4.782162e-04) * (q[i0] * q[i3]) + (3.717767e-03) * (q[i0] * q[i5]) + (-2.233771e-03) * (q[i0] * q[i6])
            + (9.178476e-04) * (q[i0] * q[i7]) + (-2.001233e-03) * (q[i0] * q[i8]) + (5.563906e-04) * (q[i0] * q[i9]) + (6.042899e-04) * (q[i0] * q[i10])
            + (1.327075e-03) * (q[i0] * q[i11]) + (-1.168324e-03) * (q[i0] * q[i12]) + (2.300713e-04) * (q[i0] * q[i15]) + (-2.979545e-04) * (q[i0] * q[i16])
            + (-9.249756e-05) * (q[i0] * q[i19]) + (-3.000105e-04) * (q[i0] * q[i20]) + (1.880298e-05) * (q[i0] * q[i21]) + (-2.734117e-05) * (q[i0] * q[i22])
            + (-1.326255e-02) * (q[i1] * q[i2]) + (4.826837e-04) * (q[i1] * q[i3]) + (2.791861e-03) * (q[i1] * q[i5]) + (-3.108244e-03) * (q[i1] * q[i6])
            + (1.007526e-02) * (q[i1] * q[i7]) + (-2.251224e-03) * (q[i1] * q[i8]) + (-2.275981e-03) * (q[i1] * q[i9]) + (6.026894e-04) * (q[i1] * q[i10])
            + (-1.489637e-03) * (q[i1] * q[i11]) + (-8.532372e-04) * (q[i1] * q[i12]) + (-6.116358e-04) * (q[i1] * q[i15]) + (-2.220515e-03) * (q[i1] * q[i16])
            + (-3.527346e-04) * (q[i1] * q[i19]) + (-9.849510e-04) * (q[i1] * q[i20]) + (-8.889815e-04) * (q[i1] * q[i21]) + (-3.475137e-04) * (q[i1] * q[i22])
            + (9.854905e-04) * (q[i2] * q[i3]) + (3.749981e-03) * (q[i2] * q[i5]) + (-3.893894e-04) * (q[i2] * q[i6]) + (2.878982e-03) * (q[i2] * q[i7])
            + (-2.213145e-03) * (q[i2] * q[i8]) + (-1.504482e-04) * (q[i2] * q[i9]) + (-7.647034e-05) * (q[i2] * q[i10]) + (1.860122e-04) * (q[i2] * q[i11])
            + (-8.613907e-05) * (q[i2] * q[i12]) + (4.359900e-04) * (q[i2] * q[i15]) + (-1.911338e-03) * (q[i2] * q[i16]) + (1.579609e-04) * (q[i2] * q[i19])
            + (6.575876e-04) * (q[i2] * q[i20]) + (5.731572e-04) * (q[i2] * q[i21]) + (-8.803963e-04) * (q[i2] * q[i22]) + (-1.760935e-04) * (q[i3] * q[i5])
            + (1.609779e-03) * (q[i3] * q[i6]) + (-1.611773e-03) * (q[i3] * q[i7]) + (5.305136e-06) * (q[i3] * q[i8]) + (-9.559773e-04) * (q[i3] * q[i9])
            + (9.555538e-04) * (q[i3] * q[i10]) + (1.488707e-03) * (q[i3] * q[i11]) + (1.482513e-03) * (q[i3] * q[i12]) + (-2.339222e-05) * (q[i3] * q[i15])
            + (3.199603e-05) * (q[i3] * q[i16]) + (-1.143179e-03) * (q[i3] * q[i19]) + (1.148663e-03) * (q[i3] * q[i20]) + (-5.856657e-04) * (q[i3] * q[i21])
            + (-5.908238e-04) * (q[i3] * q[i22]) + (5.506200e-04) * (q[i5] * q[i6]) + (3.544684e-03) * (q[i5] * q[i7]) + (-3.975660e-04) * (q[i5] * q[i8])
            + (9.979039e-04) * (q[i5] * q[i9]) + (-6.191354e-04) * (q[i5] * q[i10]) + (5.179589e-04) * (q[i5] * q[i11]) + (-2.150434e-03) * (q[i5] * q[i12])
            + (-3.856767e-04) * (q[i5] * q[i15]) + (-1.157941e-03) * (q[i5] * q[i16]) + (1.691194e-03) * (q[i5] * q[i19]) + (-1.832415e-06) * (q[i5] * q[i20])
            + (2.917031e-04) * (q[i5] * q[i21]) + (9.782708e-05) * (q[i5] * q[i22]) + (7.334453e-05) * (q[i6] * q[i7]) + (-1.661487e-03) * (q[i6] * q[i8])
            + (2.256587e-03) * (q[i6] * q[i9]) + (4.444126e-04) * (q[i6] * q[i10]) + (-5.527570e-04) * (q[i6] * q[i11]) + (-4.070025e-04) * (q[i6] * q[i12])
            + (-9.036454e-04) * (q[i6] * q[i15]) + (-1.607747e-03) * (q[i6] * q[i16]) + (4.110395e-04) * (q[i6] * q[i19]) + (-1.805649e-04) * (q[i6] * q[i20])
            + (-3.987744e-04) * (q[i6] * q[i21]) + (-6.479182e-04) * (q[i6] * q[i22]) + (-1.843450e-03) * (q[i7] * q[i8]) + (5.375296e-04) * (q[i7] * q[i9])
            + (-1.075749e-02) * (q[i7] * q[i10]) + (-1.102490e-03) * (q[i7] * q[i11]) + (5.165377e-04) * (q[i7] * q[i12]) + (-7.693415e-04) * (q[i7] * q[i15])
            + (8.235890e-04) * (q[i7] * q[i16]) + (4.637157e-04) * (q[i7] * q[i19]) + (-1.908558e-03) * (q[i7] * q[i20]) + (-9.108623e-04) * (q[i7] * q[i21])
            + (3.232710e-04) * (q[i7] * q[i22]) + (1.938854e-04) * (q[i8] * q[i9]) + (-7.384261e-04) * (q[i8] * q[i10]) + (-6.265955e-04) * (q[i8] * q[i11])
            + (1.743262e-03) * (q[i8] * q[i12]) + (-6.749888e-05) * (q[i8] * q[i15]) + (-6.186600e-04) * (q[i8] * q[i16]) + (-7.238836e-05) * (q[i8] * q[i19])
            + (-3.067289e-05) * (q[i8] * q[i20]) + (4.369758e-04) * (q[i8] * q[i21]) + (-6.370546e-04) * (q[i8] * q[i22]) + (-4.183442e-04) * (q[i9] * q[i10])
            + (-4.617177e-04) * (q[i9] * q[i11]) + (-2.095858e-04) * (q[i9] * q[i12]) + (3.734594e-04) * (q[i9] * q[i15]) + (-3.542908e-04) * (q[i9] * q[i16])
            + (2.444100e-05) * (q[i9] * q[i19]) + (2.528863e-04) * (q[i9] * q[i20]) + (-1.937380e-04) * (q[i9] * q[i21]) + (1.095770e-04) * (q[i9] * q[i22])
            + (-3.095683e-04) * (q[i10] * q[i11]) + (2.313153e-04) * (q[i10] * q[i12]) + (-5.753068e-04) * (q[i10] * q[i15])
            + (-2.718006e-04) * (q[i10] * q[i16]) + (-1.542513e-04) * (q[i10] * q[i19]) + (-3.165382e-04) * (q[i10] * q[i20])
            + (3.152262e-04) * (q[i10] * q[i21]) + (-1.301767e-04) * (q[i10] * q[i22]) + (-1.573023e-04) * (q[i11] * q[i12])
            + (5.701258e-05) * (q[i11] * q[i15]) + (-5.416789e-04) * (q[i11] * q[i16]) + (-4.461548e-04) * (q[i11] * q[i19])
            + (-2.933765e-05) * (q[i11] * q[i20]) + (-1.990022e-04) * (q[i11] * q[i21]) + (1.956478e-04) * (q[i11] * q[i22])
            + (-7.825143e-04) * (q[i12] * q[i15]) + (1.539207e-03) * (q[i12] * q[i16]) + (-3.089071e-05) * (q[i12] * q[i19])
            + (-5.005504e-05) * (q[i12] * q[i20]) + (-2.380424e-04) * (q[i12] * q[i21]) + (-4.228184e-04) * (q[i12] * q[i22])
            + (3.344007e-04) * (q[i15] * q[i16]) + (3.299048e-04) * (q[i15] * q[i19]) + (3.810390e-04) * (q[i15] * q[i20]) + (4.816924e-04) * (q[i15] * q[i21])
            + (7.991379e-05) * (q[i15] * q[i22]) + (1.239034e-04) * (q[i16] * q[i19]) + (-6.142677e-04) * (q[i16] * q[i20])
            + (-2.286239e-04) * (q[i16] * q[i21]) + (-1.533807e-04) * (q[i16] * q[i22]) + (4.128263e-04) * (q[i19] * q[i20])
            + (3.958918e-04) * (q[i19] * q[i21]) + (3.210236e-04) * (q[i19] * q[i22]) + (6.100004e-04) * (q[i20] * q[i21]) + (-5.392119e-04) * (q[i20] * q[i22])
            + (8.042876e-05) * (q[i21] * q[i22]);
   }

   public void getJQx5(double[] q, double[][] JQ)
   {
      JQ[1][i5] = (1.312501e-01) * (1) + (-1.137564e-05) * ((2) * q[i5]) + (1.179885e-03) * (q[i0]) + (-1.160811e-03) * (q[i1]) + (7.287988e-05) * (q[i2])
            + (-9.554202e-03) * (q[i3]) + (9.530658e-03) * (q[i4]) + (7.151259e-04) * (q[i6]) + (7.129366e-04) * (q[i7]) + (-1.648139e-03) * (q[i8])
            + (3.173301e-03) * (q[i9]) + (3.166160e-03) * (q[i10]) + (5.353168e-03) * (q[i11]) + (-5.393804e-03) * (q[i12]) + (-1.472151e-02) * (q[i15])
            + (-1.488733e-02) * (q[i16]) + (-2.978448e-04) * (q[i19]) + (-3.149214e-04) * (q[i20]) + (-5.199303e-04) * (q[i21]) + (5.237748e-04) * (q[i22])
            + (-2.422992e-03) * (q[i0] * q[i0]) + (-2.404036e-03) * (q[i1] * q[i1]) + (-3.008131e-02) * (q[i2] * q[i2]) + (4.928136e-03) * (q[i3] * q[i3])
            + (4.909451e-03) * (q[i4] * q[i4]) + (-1.039680e-03) * ((2) * q[i0] * q[i5]) + (-1.022238e-03) * ((2) * q[i1] * q[i5])
            + (-9.399917e-04) * ((2) * q[i2] * q[i5]) + (4.794269e-05) * ((2) * q[i3] * q[i5]) + (8.117997e-05) * ((2) * q[i4] * q[i5])
            + (-5.570426e-03) * ((3) * q[i5] * q[i5]) + (1.849501e-04) * ((2) * q[i5] * q[i6]) + (-1.648071e-04) * ((2) * q[i5] * q[i7])
            + (-1.426346e-05) * ((2) * q[i5] * q[i8]) + (-5.364463e-04) * ((2) * q[i5] * q[i9]) + (5.496565e-04) * ((2) * q[i5] * q[i10])
            + (1.577033e-05) * ((2) * q[i5] * q[i11]) + (1.886787e-05) * ((2) * q[i5] * q[i12]) + (-5.557385e-04) * ((2) * q[i5] * q[i15])
            + (5.689679e-04) * ((2) * q[i5] * q[i16]) + (6.193059e-04) * ((2) * q[i5] * q[i19]) + (-6.123879e-04) * ((2) * q[i5] * q[i20])
            + (-2.430805e-04) * ((2) * q[i5] * q[i21]) + (-2.432180e-04) * ((2) * q[i5] * q[i22]) + (6.099329e-03) * (q[i6] * q[i6])
            + (6.091801e-03) * (q[i7] * q[i7]) + (-2.254533e-02) * (q[i8] * q[i8]) + (1.550665e-03) * (q[i9] * q[i9]) + (1.532201e-03) * (q[i10] * q[i10])
            + (-1.498545e-03) * (q[i11] * q[i11]) + (-1.566259e-03) * (q[i12] * q[i12]) + (-2.441580e-03) * (q[i15] * q[i15])
            + (-2.458229e-03) * (q[i16] * q[i16]) + (5.381257e-06) * (q[i19] * q[i19]) + (1.593998e-05) * (q[i20] * q[i20])
            + (-1.538097e-03) * (q[i21] * q[i21]) + (-1.540498e-03) * (q[i22] * q[i22]) + (-2.589524e-03) * (q[i0] * q[i1]) + (-7.566281e-03) * (q[i0] * q[i2])
            + (2.791141e-03) * (q[i0] * q[i3]) + (3.717767e-03) * (q[i0] * q[i4]) + (-6.703300e-03) * (q[i0] * q[i6]) + (-2.394482e-04) * (q[i0] * q[i7])
            + (3.941155e-04) * (q[i0] * q[i8]) + (-5.443572e-04) * (q[i0] * q[i9]) + (-5.315843e-04) * (q[i0] * q[i10]) + (8.634613e-04) * (q[i0] * q[i11])
            + (4.483416e-04) * (q[i0] * q[i12]) + (-6.115209e-04) * (q[i0] * q[i15]) + (-1.147948e-03) * (q[i0] * q[i16]) + (-6.164790e-04) * (q[i0] * q[i19])
            + (-3.585268e-04) * (q[i0] * q[i20]) + (2.820474e-04) * (q[i0] * q[i21]) + (4.204164e-04) * (q[i0] * q[i22]) + (-7.496941e-03) * (q[i1] * q[i2])
            + (3.747658e-03) * (q[i1] * q[i3]) + (2.791861e-03) * (q[i1] * q[i4]) + (2.446108e-04) * (q[i1] * q[i6]) + (6.748017e-03) * (q[i1] * q[i7])
            + (-3.875300e-04) * (q[i1] * q[i8]) + (5.165634e-04) * (q[i1] * q[i9]) + (5.554145e-04) * (q[i1] * q[i10]) + (4.673067e-04) * (q[i1] * q[i11])
            + (8.766858e-04) * (q[i1] * q[i12]) + (1.168227e-03) * (q[i1] * q[i15]) + (6.108258e-04) * (q[i1] * q[i16]) + (3.734309e-04) * (q[i1] * q[i19])
            + (5.978117e-04) * (q[i1] * q[i20]) + (4.016762e-04) * (q[i1] * q[i21]) + (2.796101e-04) * (q[i1] * q[i22]) + (3.739732e-03) * (q[i2] * q[i3])
            + (3.749981e-03) * (q[i2] * q[i4]) + (8.400495e-04) * (q[i2] * q[i6]) + (-8.526404e-04) * (q[i2] * q[i7]) + (-4.747920e-05) * (q[i2] * q[i8])
            + (1.378678e-03) * (q[i2] * q[i9]) + (-1.394716e-03) * (q[i2] * q[i10]) + (1.879672e-03) * (q[i2] * q[i11]) + (1.853627e-03) * (q[i2] * q[i12])
            + (-2.309920e-03) * (q[i2] * q[i15]) + (2.307184e-03) * (q[i2] * q[i16]) + (1.426304e-03) * (q[i2] * q[i19]) + (-1.426429e-03) * (q[i2] * q[i20])
            + (8.077700e-04) * (q[i2] * q[i21]) + (7.973751e-04) * (q[i2] * q[i22]) + (-1.760935e-04) * (q[i3] * q[i4]) + (-3.556932e-03) * (q[i3] * q[i6])
            + (-5.519686e-04) * (q[i3] * q[i7]) + (3.825928e-04) * (q[i3] * q[i8]) + (6.208945e-04) * (q[i3] * q[i9]) + (-9.887132e-04) * (q[i3] * q[i10])
            + (-2.169193e-03) * (q[i3] * q[i11]) + (5.292480e-04) * (q[i3] * q[i12]) + (1.139528e-03) * (q[i3] * q[i15]) + (3.817097e-04) * (q[i3] * q[i16])
            + (6.850017e-06) * (q[i3] * q[i19]) + (-1.682617e-03) * (q[i3] * q[i20]) + (9.284001e-05) * (q[i3] * q[i21]) + (2.950487e-04) * (q[i3] * q[i22])
            + (5.506200e-04) * (q[i4] * q[i6]) + (3.544684e-03) * (q[i4] * q[i7]) + (-3.975660e-04) * (q[i4] * q[i8]) + (9.979039e-04) * (q[i4] * q[i9])
            + (-6.191354e-04) * (q[i4] * q[i10]) + (5.179589e-04) * (q[i4] * q[i11]) + (-2.150434e-03) * (q[i4] * q[i12]) + (-3.856767e-04) * (q[i4] * q[i15])
            + (-1.157941e-03) * (q[i4] * q[i16]) + (1.691194e-03) * (q[i4] * q[i19]) + (-1.832415e-06) * (q[i4] * q[i20]) + (2.917031e-04) * (q[i4] * q[i21])
            + (9.782708e-05) * (q[i4] * q[i22]) + (8.525738e-04) * (q[i6] * q[i7]) + (3.660053e-04) * (q[i6] * q[i8]) + (2.230545e-03) * (q[i6] * q[i9])
            + (-1.157632e-03) * (q[i6] * q[i10]) + (-2.735815e-04) * (q[i6] * q[i11]) + (-6.490981e-05) * (q[i6] * q[i12]) + (7.044322e-04) * (q[i6] * q[i15])
            + (4.955465e-04) * (q[i6] * q[i16]) + (-2.268588e-04) * (q[i6] * q[i19]) + (7.898259e-04) * (q[i6] * q[i20]) + (1.469943e-04) * (q[i6] * q[i21])
            + (-9.274461e-05) * (q[i6] * q[i22]) + (3.475794e-04) * (q[i7] * q[i8]) + (-1.172213e-03) * (q[i7] * q[i9]) + (2.225263e-03) * (q[i7] * q[i10])
            + (4.523701e-05) * (q[i7] * q[i11]) + (2.827897e-04) * (q[i7] * q[i12]) + (4.890998e-04) * (q[i7] * q[i15]) + (6.914289e-04) * (q[i7] * q[i16])
            + (7.971804e-04) * (q[i7] * q[i19]) + (-2.328081e-04) * (q[i7] * q[i20]) + (1.016375e-04) * (q[i7] * q[i21]) + (-1.583401e-04) * (q[i7] * q[i22])
            + (-1.557785e-03) * (q[i8] * q[i9]) + (-1.550726e-03) * (q[i8] * q[i10]) + (2.422300e-03) * (q[i8] * q[i11]) + (-2.488921e-03) * (q[i8] * q[i12])
            + (2.049749e-03) * (q[i8] * q[i15]) + (2.082281e-03) * (q[i8] * q[i16]) + (-4.989687e-05) * (q[i8] * q[i19]) + (-6.700013e-05) * (q[i8] * q[i20])
            + (3.568588e-04) * (q[i8] * q[i21]) + (-3.555156e-04) * (q[i8] * q[i22]) + (-3.448780e-04) * (q[i9] * q[i10]) + (-1.836317e-04) * (q[i9] * q[i11])
            + (-2.499222e-04) * (q[i9] * q[i12]) + (-1.230037e-04) * (q[i9] * q[i15]) + (-2.549192e-04) * (q[i9] * q[i16]) + (3.467943e-04) * (q[i9] * q[i19])
            + (-2.783076e-04) * (q[i9] * q[i20]) + (3.350566e-05) * (q[i9] * q[i21]) + (5.021899e-05) * (q[i9] * q[i22]) + (2.410171e-04) * (q[i10] * q[i11])
            + (1.793394e-04) * (q[i10] * q[i12]) + (-2.451128e-04) * (q[i10] * q[i15]) + (-1.284285e-04) * (q[i10] * q[i16])
            + (-2.752782e-04) * (q[i10] * q[i19]) + (3.378551e-04) * (q[i10] * q[i20]) + (-5.018386e-05) * (q[i10] * q[i21])
            + (-4.503147e-05) * (q[i10] * q[i22]) + (-6.758906e-05) * (q[i11] * q[i12]) + (2.045775e-03) * (q[i11] * q[i15])
            + (-6.262812e-04) * (q[i11] * q[i16]) + (1.178573e-03) * (q[i11] * q[i19]) + (-4.248532e-04) * (q[i11] * q[i20])
            + (1.428402e-04) * (q[i11] * q[i21]) + (-6.106314e-04) * (q[i11] * q[i22]) + (6.146331e-04) * (q[i12] * q[i15])
            + (-2.053211e-03) * (q[i12] * q[i16]) + (4.292040e-04) * (q[i12] * q[i19]) + (-1.178317e-03) * (q[i12] * q[i20])
            + (-6.021615e-04) * (q[i12] * q[i21]) + (1.624863e-04) * (q[i12] * q[i22]) + (-1.212710e-05) * (q[i15] * q[i16])
            + (-3.948275e-04) * (q[i15] * q[i19]) + (-3.213971e-04) * (q[i15] * q[i20]) + (-1.647586e-03) * (q[i15] * q[i21])
            + (-3.132046e-04) * (q[i15] * q[i22]) + (-3.121259e-04) * (q[i16] * q[i19]) + (-3.933553e-04) * (q[i16] * q[i20])
            + (3.135076e-04) * (q[i16] * q[i21]) + (1.668104e-03) * (q[i16] * q[i22]) + (-1.699453e-04) * (q[i19] * q[i20])
            + (-1.532837e-03) * (q[i19] * q[i21]) + (3.425438e-04) * (q[i19] * q[i22]) + (-3.402388e-04) * (q[i20] * q[i21])
            + (1.535302e-03) * (q[i20] * q[i22]) + (-4.511650e-04) * (q[i21] * q[i22]);
   }

   public void getJQx6(double[] q, double[][] JQ)
   {
      JQ[1][i6] = (1.517551e-03) * (1) + (1.696235e-02) * ((2) * q[i6]) + (8.925438e-02) * (q[i0]) + (-1.065993e-02) * (q[i1]) + (1.497659e-02) * (q[i2])
            + (1.090700e-02) * (q[i3]) + (-5.418235e-03) * (q[i4]) + (7.151259e-04) * (q[i5]) + (-4.978121e-05) * (q[i7]) + (1.125088e-03) * (q[i8])
            + (8.561279e-03) * (q[i9]) + (1.671832e-04) * (q[i10]) + (3.825531e-03) * (q[i11]) + (4.177021e-03) * (q[i12]) + (-9.606252e-04) * (q[i15])
            + (-1.464468e-03) * (q[i16]) + (9.621823e-04) * (q[i19]) + (2.620063e-04) * (q[i20]) + (-1.346279e-03) * (q[i21]) + (-4.101186e-04) * (q[i22])
            + (3.158567e-04) * (q[i0] * q[i0]) + (-6.653566e-04) * (q[i1] * q[i1]) + (9.338505e-05) * (q[i2] * q[i2]) + (6.705721e-04) * (q[i3] * q[i3])
            + (1.513751e-03) * (q[i4] * q[i4]) + (1.849501e-04) * (q[i5] * q[i5]) + (5.723209e-03) * ((2) * q[i0] * q[i6])
            + (-3.361309e-03) * ((2) * q[i1] * q[i6]) + (6.404787e-04) * ((2) * q[i2] * q[i6]) + (-2.172440e-02) * ((2) * q[i3] * q[i6])
            + (3.718044e-03) * ((2) * q[i4] * q[i6]) + (6.099329e-03) * ((2) * q[i5] * q[i6]) + (-2.270557e-04) * ((3) * q[i6] * q[i6])
            + (-9.038252e-04) * ((2) * q[i6] * q[i7]) + (4.100959e-04) * ((2) * q[i6] * q[i8]) + (-2.264878e-03) * ((2) * q[i6] * q[i9])
            + (5.487510e-04) * ((2) * q[i6] * q[i10]) + (4.218537e-04) * ((2) * q[i6] * q[i11]) + (-3.891096e-04) * ((2) * q[i6] * q[i12])
            + (-9.299073e-05) * ((2) * q[i6] * q[i15]) + (6.497508e-04) * ((2) * q[i6] * q[i16]) + (4.450975e-05) * ((2) * q[i6] * q[i19])
            + (1.616839e-04) * ((2) * q[i6] * q[i20]) + (-3.261228e-06) * ((2) * q[i6] * q[i21]) + (-1.632641e-05) * ((2) * q[i6] * q[i22])
            + (9.049236e-04) * (q[i7] * q[i7]) + (-7.129223e-04) * (q[i8] * q[i8]) + (-2.245750e-03) * (q[i9] * q[i9]) + (-8.673242e-04) * (q[i10] * q[i10])
            + (-3.333575e-04) * (q[i11] * q[i11]) + (7.173906e-05) * (q[i12] * q[i12]) + (-1.249697e-05) * (q[i15] * q[i15])
            + (8.264544e-05) * (q[i16] * q[i16]) + (-1.282560e-05) * (q[i19] * q[i19]) + (-1.731079e-04) * (q[i20] * q[i20])
            + (-5.732377e-05) * (q[i21] * q[i21]) + (3.050303e-04) * (q[i22] * q[i22]) + (1.561763e-03) * (q[i0] * q[i1]) + (1.107415e-03) * (q[i0] * q[i2])
            + (-1.013608e-02) * (q[i0] * q[i3]) + (-2.233771e-03) * (q[i0] * q[i4]) + (-6.703300e-03) * (q[i0] * q[i5]) + (-1.802536e-03) * (q[i0] * q[i7])
            + (7.060447e-03) * (q[i0] * q[i8]) + (-1.365839e-02) * (q[i0] * q[i9]) + (2.730496e-03) * (q[i0] * q[i10]) + (-1.453379e-03) * (q[i0] * q[i11])
            + (-1.077763e-03) * (q[i0] * q[i12]) + (7.646598e-04) * (q[i0] * q[i15]) + (1.406846e-03) * (q[i0] * q[i16]) + (6.747908e-04) * (q[i0] * q[i19])
            + (1.608036e-04) * (q[i0] * q[i20]) + (-7.451047e-04) * (q[i0] * q[i21]) + (8.910689e-04) * (q[i0] * q[i22]) + (5.407253e-04) * (q[i1] * q[i2])
            + (-9.422555e-04) * (q[i1] * q[i3]) + (-3.108244e-03) * (q[i1] * q[i4]) + (2.446108e-04) * (q[i1] * q[i5]) + (-1.795773e-03) * (q[i1] * q[i7])
            + (3.638126e-04) * (q[i1] * q[i8]) + (1.300291e-03) * (q[i1] * q[i9]) + (3.132138e-04) * (q[i1] * q[i10]) + (9.677180e-04) * (q[i1] * q[i11])
            + (1.764341e-03) * (q[i1] * q[i12]) + (1.927100e-04) * (q[i1] * q[i15]) + (-6.800327e-04) * (q[i1] * q[i16]) + (-1.865585e-04) * (q[i1] * q[i19])
            + (-2.142370e-04) * (q[i1] * q[i20]) + (3.399222e-04) * (q[i1] * q[i21]) + (-2.211472e-05) * (q[i1] * q[i22]) + (-2.893629e-03) * (q[i2] * q[i3])
            + (-3.893894e-04) * (q[i2] * q[i4]) + (8.400495e-04) * (q[i2] * q[i5]) + (3.171142e-04) * (q[i2] * q[i7]) + (6.081805e-03) * (q[i2] * q[i8])
            + (-3.521530e-03) * (q[i2] * q[i9]) + (1.440729e-03) * (q[i2] * q[i10]) + (-8.145888e-04) * (q[i2] * q[i11]) + (-1.063741e-04) * (q[i2] * q[i12])
            + (6.365126e-04) * (q[i2] * q[i15]) + (-4.380484e-04) * (q[i2] * q[i16]) + (1.961629e-04) * (q[i2] * q[i19]) + (3.576040e-05) * (q[i2] * q[i20])
            + (3.222426e-04) * (q[i2] * q[i21]) + (2.455748e-04) * (q[i2] * q[i22]) + (1.609779e-03) * (q[i3] * q[i4]) + (-3.556932e-03) * (q[i3] * q[i5])
            + (7.620428e-05) * (q[i3] * q[i7]) + (-1.833501e-03) * (q[i3] * q[i8]) + (-1.084537e-02) * (q[i3] * q[i9]) + (5.088915e-04) * (q[i3] * q[i10])
            + (-5.300235e-04) * (q[i3] * q[i11]) + (1.113857e-03) * (q[i3] * q[i12]) + (8.047113e-04) * (q[i3] * q[i15]) + (-7.632352e-04) * (q[i3] * q[i16])
            + (-1.925811e-03) * (q[i3] * q[i19]) + (4.617432e-04) * (q[i3] * q[i20]) + (-3.234240e-04) * (q[i3] * q[i21]) + (9.169762e-04) * (q[i3] * q[i22])
            + (5.506200e-04) * (q[i4] * q[i5]) + (7.334453e-05) * (q[i4] * q[i7]) + (-1.661487e-03) * (q[i4] * q[i8]) + (2.256587e-03) * (q[i4] * q[i9])
            + (4.444126e-04) * (q[i4] * q[i10]) + (-5.527570e-04) * (q[i4] * q[i11]) + (-4.070025e-04) * (q[i4] * q[i12]) + (-9.036454e-04) * (q[i4] * q[i15])
            + (-1.607747e-03) * (q[i4] * q[i16]) + (4.110395e-04) * (q[i4] * q[i19]) + (-1.805649e-04) * (q[i4] * q[i20]) + (-3.987744e-04) * (q[i4] * q[i21])
            + (-6.479182e-04) * (q[i4] * q[i22]) + (8.525738e-04) * (q[i5] * q[i7]) + (3.660053e-04) * (q[i5] * q[i8]) + (2.230545e-03) * (q[i5] * q[i9])
            + (-1.157632e-03) * (q[i5] * q[i10]) + (-2.735815e-04) * (q[i5] * q[i11]) + (-6.490981e-05) * (q[i5] * q[i12]) + (7.044322e-04) * (q[i5] * q[i15])
            + (4.955465e-04) * (q[i5] * q[i16]) + (-2.268588e-04) * (q[i5] * q[i19]) + (7.898259e-04) * (q[i5] * q[i20]) + (1.469943e-04) * (q[i5] * q[i21])
            + (-9.274461e-05) * (q[i5] * q[i22]) + (9.935497e-06) * (q[i7] * q[i8]) + (1.051171e-03) * (q[i7] * q[i9]) + (-1.026063e-03) * (q[i7] * q[i10])
            + (8.457240e-04) * (q[i7] * q[i11]) + (8.422181e-04) * (q[i7] * q[i12]) + (1.090265e-04) * (q[i7] * q[i15]) + (-1.037743e-04) * (q[i7] * q[i16])
            + (1.300454e-04) * (q[i7] * q[i19]) + (-1.368505e-04) * (q[i7] * q[i20]) + (-8.415065e-04) * (q[i7] * q[i21]) + (-8.425326e-04) * (q[i7] * q[i22])
            + (7.096309e-04) * (q[i8] * q[i9]) + (5.071955e-04) * (q[i8] * q[i10]) + (-9.533636e-04) * (q[i8] * q[i11]) + (6.504106e-04) * (q[i8] * q[i12])
            + (-2.518575e-04) * (q[i8] * q[i15]) + (5.991215e-04) * (q[i8] * q[i16]) + (-1.606921e-04) * (q[i8] * q[i19]) + (2.586692e-04) * (q[i8] * q[i20])
            + (2.789672e-04) * (q[i8] * q[i21]) + (1.065966e-04) * (q[i8] * q[i22]) + (4.232231e-04) * (q[i9] * q[i10]) + (-6.709169e-04) * (q[i9] * q[i11])
            + (-4.706229e-04) * (q[i9] * q[i12]) + (-9.224733e-05) * (q[i9] * q[i15]) + (-9.940779e-05) * (q[i9] * q[i16]) + (-4.476959e-04) * (q[i9] * q[i19])
            + (1.079777e-04) * (q[i9] * q[i20]) + (-1.558414e-04) * (q[i9] * q[i21]) + (-1.983248e-04) * (q[i9] * q[i22]) + (1.559962e-04) * (q[i10] * q[i11])
            + (1.286611e-05) * (q[i10] * q[i12]) + (-3.926582e-04) * (q[i10] * q[i15]) + (-1.307033e-05) * (q[i10] * q[i16])
            + (7.761660e-05) * (q[i10] * q[i19]) + (-3.088570e-04) * (q[i10] * q[i20]) + (4.445187e-04) * (q[i10] * q[i21]) + (1.511723e-04) * (q[i10] * q[i22])
            + (6.787517e-06) * (q[i11] * q[i12]) + (1.036661e-03) * (q[i11] * q[i15]) + (-6.698623e-05) * (q[i11] * q[i16])
            + (-7.845989e-05) * (q[i11] * q[i19]) + (4.371767e-04) * (q[i11] * q[i20]) + (3.351916e-04) * (q[i11] * q[i21]) + (1.009963e-04) * (q[i11] * q[i22])
            + (6.488194e-04) * (q[i12] * q[i15]) + (6.449914e-04) * (q[i12] * q[i16]) + (-1.415264e-04) * (q[i12] * q[i19]) + (2.387709e-04) * (q[i12] * q[i20])
            + (-2.481534e-04) * (q[i12] * q[i21]) + (-4.363672e-04) * (q[i12] * q[i22]) + (-3.029794e-04) * (q[i15] * q[i16])
            + (3.436680e-04) * (q[i15] * q[i19]) + (-5.554971e-05) * (q[i15] * q[i20]) + (-2.115197e-04) * (q[i15] * q[i21])
            + (1.826301e-04) * (q[i15] * q[i22]) + (-4.558552e-04) * (q[i16] * q[i19]) + (-1.235554e-04) * (q[i16] * q[i20])
            + (-4.327594e-04) * (q[i16] * q[i21]) + (3.514698e-04) * (q[i16] * q[i22]) + (-9.645676e-06) * (q[i19] * q[i20])
            + (-3.073957e-04) * (q[i19] * q[i21]) + (-7.130947e-05) * (q[i19] * q[i22]) + (2.802752e-04) * (q[i20] * q[i21])
            + (-2.261390e-04) * (q[i20] * q[i22]) + (1.855035e-04) * (q[i21] * q[i22]);
   }

   public void getJQx7(double[] q, double[][] JQ)
   {
      JQ[1][i7] = (-1.474994e-03) * (1) + (-1.690152e-02) * ((2) * q[i7]) + (-1.064739e-02) * (q[i0]) + (8.875536e-02) * (q[i1]) + (1.484636e-02) * (q[i2])
            + (-5.409597e-03) * (q[i3]) + (1.087283e-02) * (q[i4]) + (7.129366e-04) * (q[i5]) + (-4.978121e-05) * (q[i6]) + (-1.136944e-03) * (q[i8])
            + (-1.806331e-04) * (q[i9]) + (-8.450414e-03) * (q[i10]) + (4.162477e-03) * (q[i11]) + (3.783637e-03) * (q[i12]) + (1.467304e-03) * (q[i15])
            + (9.646926e-04) * (q[i16]) + (-2.502844e-04) * (q[i19]) + (-9.387683e-04) * (q[i20]) + (-4.092206e-04) * (q[i21]) + (-1.333653e-03) * (q[i22])
            + (6.750960e-04) * (q[i0] * q[i0]) + (-3.216427e-04) * (q[i1] * q[i1]) + (-9.476205e-05) * (q[i2] * q[i2]) + (-1.513164e-03) * (q[i3] * q[i3])
            + (-6.565926e-04) * (q[i4] * q[i4]) + (-1.648071e-04) * (q[i5] * q[i5]) + (-9.038252e-04) * (q[i6] * q[i6])
            + (-3.356871e-03) * ((2) * q[i0] * q[i7]) + (5.746949e-03) * ((2) * q[i1] * q[i7]) + (6.425832e-04) * ((2) * q[i2] * q[i7])
            + (3.708187e-03) * ((2) * q[i3] * q[i7]) + (-2.165774e-02) * ((2) * q[i4] * q[i7]) + (6.091801e-03) * ((2) * q[i5] * q[i7])
            + (9.049236e-04) * ((2) * q[i6] * q[i7]) + (2.205851e-04) * ((3) * q[i7] * q[i7]) + (-4.012663e-04) * ((2) * q[i7] * q[i8])
            + (-5.476560e-04) * ((2) * q[i7] * q[i9]) + (2.235544e-03) * ((2) * q[i7] * q[i10]) + (-3.949178e-04) * ((2) * q[i7] * q[i11])
            + (4.162036e-04) * ((2) * q[i7] * q[i12]) + (-6.495229e-04) * ((2) * q[i7] * q[i15]) + (8.193634e-05) * ((2) * q[i7] * q[i16])
            + (-1.559224e-04) * ((2) * q[i7] * q[i19]) + (-5.194834e-05) * ((2) * q[i7] * q[i20]) + (-2.092241e-05) * ((2) * q[i7] * q[i21])
            + (-5.552237e-06) * ((2) * q[i7] * q[i22]) + (7.053321e-04) * (q[i8] * q[i8]) + (8.848611e-04) * (q[i9] * q[i9])
            + (2.211956e-03) * (q[i10] * q[i10]) + (-5.456559e-05) * (q[i11] * q[i11]) + (3.419957e-04) * (q[i12] * q[i12])
            + (-7.857293e-05) * (q[i15] * q[i15]) + (1.906856e-05) * (q[i16] * q[i16]) + (1.689597e-04) * (q[i19] * q[i19]) + (1.524099e-05) * (q[i20] * q[i20])
            + (-3.043743e-04) * (q[i21] * q[i21]) + (5.152297e-05) * (q[i22] * q[i22]) + (-1.557649e-03) * (q[i0] * q[i1]) + (-5.434355e-04) * (q[i0] * q[i2])
            + (3.139681e-03) * (q[i0] * q[i3]) + (9.178476e-04) * (q[i0] * q[i4]) + (-2.394482e-04) * (q[i0] * q[i5]) + (-1.802536e-03) * (q[i0] * q[i6])
            + (3.706215e-04) * (q[i0] * q[i8]) + (3.080839e-04) * (q[i0] * q[i9]) + (1.284695e-03) * (q[i0] * q[i10]) + (-1.745295e-03) * (q[i0] * q[i11])
            + (-9.625702e-04) * (q[i0] * q[i12]) + (-6.808599e-04) * (q[i0] * q[i15]) + (1.844087e-04) * (q[i0] * q[i16]) + (-2.213321e-04) * (q[i0] * q[i19])
            + (-1.875617e-04) * (q[i0] * q[i20]) + (1.814035e-05) * (q[i0] * q[i21]) + (-3.477295e-04) * (q[i0] * q[i22]) + (-1.096792e-03) * (q[i1] * q[i2])
            + (2.198485e-03) * (q[i1] * q[i3]) + (1.007526e-02) * (q[i1] * q[i4]) + (6.748017e-03) * (q[i1] * q[i5]) + (-1.795773e-03) * (q[i1] * q[i6])
            + (7.035108e-03) * (q[i1] * q[i8]) + (2.754172e-03) * (q[i1] * q[i9]) + (-1.349584e-02) * (q[i1] * q[i10]) + (1.099982e-03) * (q[i1] * q[i11])
            + (1.474185e-03) * (q[i1] * q[i12]) + (1.383560e-03) * (q[i1] * q[i15]) + (7.824298e-04) * (q[i1] * q[i16]) + (1.553686e-04) * (q[i1] * q[i19])
            + (6.718480e-04) * (q[i1] * q[i20]) + (-8.756166e-04) * (q[i1] * q[i21]) + (7.638772e-04) * (q[i1] * q[i22]) + (4.047955e-04) * (q[i2] * q[i3])
            + (2.878982e-03) * (q[i2] * q[i4]) + (-8.526404e-04) * (q[i2] * q[i5]) + (3.171142e-04) * (q[i2] * q[i6]) + (6.089197e-03) * (q[i2] * q[i8])
            + (1.449597e-03) * (q[i2] * q[i9]) + (-3.469230e-03) * (q[i2] * q[i10]) + (1.189835e-04) * (q[i2] * q[i11]) + (8.198402e-04) * (q[i2] * q[i12])
            + (-4.190663e-04) * (q[i2] * q[i15]) + (6.364879e-04) * (q[i2] * q[i16]) + (3.483468e-05) * (q[i2] * q[i19]) + (2.011984e-04) * (q[i2] * q[i20])
            + (-2.421668e-04) * (q[i2] * q[i21]) + (-3.191185e-04) * (q[i2] * q[i22]) + (-1.611773e-03) * (q[i3] * q[i4]) + (-5.519686e-04) * (q[i3] * q[i5])
            + (7.620428e-05) * (q[i3] * q[i6]) + (-1.648222e-03) * (q[i3] * q[i8]) + (4.427378e-04) * (q[i3] * q[i9]) + (2.244056e-03) * (q[i3] * q[i10])
            + (3.831200e-04) * (q[i3] * q[i11]) + (5.663898e-04) * (q[i3] * q[i12]) + (-1.599024e-03) * (q[i3] * q[i15]) + (-9.085843e-04) * (q[i3] * q[i16])
            + (-1.873487e-04) * (q[i3] * q[i19]) + (4.093100e-04) * (q[i3] * q[i20]) + (6.429751e-04) * (q[i3] * q[i21]) + (4.012098e-04) * (q[i3] * q[i22])
            + (3.544684e-03) * (q[i4] * q[i5]) + (7.334453e-05) * (q[i4] * q[i6]) + (-1.843450e-03) * (q[i4] * q[i8]) + (5.375296e-04) * (q[i4] * q[i9])
            + (-1.075749e-02) * (q[i4] * q[i10]) + (-1.102490e-03) * (q[i4] * q[i11]) + (5.165377e-04) * (q[i4] * q[i12]) + (-7.693415e-04) * (q[i4] * q[i15])
            + (8.235890e-04) * (q[i4] * q[i16]) + (4.637157e-04) * (q[i4] * q[i19]) + (-1.908558e-03) * (q[i4] * q[i20]) + (-9.108623e-04) * (q[i4] * q[i21])
            + (3.232710e-04) * (q[i4] * q[i22]) + (8.525738e-04) * (q[i5] * q[i6]) + (3.475794e-04) * (q[i5] * q[i8]) + (-1.172213e-03) * (q[i5] * q[i9])
            + (2.225263e-03) * (q[i5] * q[i10]) + (4.523701e-05) * (q[i5] * q[i11]) + (2.827897e-04) * (q[i5] * q[i12]) + (4.890998e-04) * (q[i5] * q[i15])
            + (6.914289e-04) * (q[i5] * q[i16]) + (7.971804e-04) * (q[i5] * q[i19]) + (-2.328081e-04) * (q[i5] * q[i20]) + (1.016375e-04) * (q[i5] * q[i21])
            + (-1.583401e-04) * (q[i5] * q[i22]) + (9.935497e-06) * (q[i6] * q[i8]) + (1.051171e-03) * (q[i6] * q[i9]) + (-1.026063e-03) * (q[i6] * q[i10])
            + (8.457240e-04) * (q[i6] * q[i11]) + (8.422181e-04) * (q[i6] * q[i12]) + (1.090265e-04) * (q[i6] * q[i15]) + (-1.037743e-04) * (q[i6] * q[i16])
            + (1.300454e-04) * (q[i6] * q[i19]) + (-1.368505e-04) * (q[i6] * q[i20]) + (-8.415065e-04) * (q[i6] * q[i21]) + (-8.425326e-04) * (q[i6] * q[i22])
            + (-5.016305e-04) * (q[i8] * q[i9]) + (-6.984106e-04) * (q[i8] * q[i10]) + (6.851689e-04) * (q[i8] * q[i11]) + (-9.460906e-04) * (q[i8] * q[i12])
            + (-5.965860e-04) * (q[i8] * q[i15]) + (2.638945e-04) * (q[i8] * q[i16]) + (-2.536953e-04) * (q[i8] * q[i19]) + (1.514588e-04) * (q[i8] * q[i20])
            + (1.148712e-04) * (q[i8] * q[i21]) + (2.757292e-04) * (q[i8] * q[i22]) + (-4.224340e-04) * (q[i9] * q[i10]) + (1.739491e-05) * (q[i9] * q[i11])
            + (1.543717e-04) * (q[i9] * q[i12]) + (1.165673e-05) * (q[i9] * q[i15]) + (3.950348e-04) * (q[i9] * q[i16]) + (3.069477e-04) * (q[i9] * q[i19])
            + (-7.608729e-05) * (q[i9] * q[i20]) + (1.492711e-04) * (q[i9] * q[i21]) + (4.424179e-04) * (q[i9] * q[i22]) + (-4.669284e-04) * (q[i10] * q[i11])
            + (-6.696082e-04) * (q[i10] * q[i12]) + (9.726486e-05) * (q[i10] * q[i15]) + (1.002896e-04) * (q[i10] * q[i16])
            + (-1.083990e-04) * (q[i10] * q[i19]) + (4.362536e-04) * (q[i10] * q[i20]) + (-2.008247e-04) * (q[i10] * q[i21])
            + (-1.599506e-04) * (q[i10] * q[i22]) + (-5.702689e-06) * (q[i11] * q[i12]) + (6.396486e-04) * (q[i11] * q[i15])
            + (6.519627e-04) * (q[i11] * q[i16]) + (2.386295e-04) * (q[i11] * q[i19]) + (-1.468150e-04) * (q[i11] * q[i20]) + (4.271983e-04) * (q[i11] * q[i21])
            + (2.477358e-04) * (q[i11] * q[i22]) + (-6.701922e-05) * (q[i12] * q[i15]) + (1.026269e-03) * (q[i12] * q[i16]) + (4.347151e-04) * (q[i12] * q[i19])
            + (-7.737333e-05) * (q[i12] * q[i20]) + (-1.026845e-04) * (q[i12] * q[i21]) + (-3.307243e-04) * (q[i12] * q[i22])
            + (3.025784e-04) * (q[i15] * q[i16]) + (1.200177e-04) * (q[i15] * q[i19]) + (4.572318e-04) * (q[i15] * q[i20]) + (3.478372e-04) * (q[i15] * q[i21])
            + (-4.318086e-04) * (q[i15] * q[i22]) + (5.480453e-05) * (q[i16] * q[i19]) + (-3.419594e-04) * (q[i16] * q[i20])
            + (1.779659e-04) * (q[i16] * q[i21]) + (-2.200355e-04) * (q[i16] * q[i22]) + (6.585923e-06) * (q[i19] * q[i20])
            + (-2.169687e-04) * (q[i19] * q[i21]) + (2.842097e-04) * (q[i19] * q[i22]) + (-7.294358e-05) * (q[i20] * q[i21])
            + (-3.075305e-04) * (q[i20] * q[i22]) + (-1.864748e-04) * (q[i21] * q[i22]);
   }

   public void getJQx8(double[] q, double[][] JQ)
   {
      JQ[1][i8] = (-1.043223e-04) * (1) + (-1.866490e-06) * ((2) * q[i8]) + (-2.129090e-03) * (q[i0]) + (-2.140533e-03) * (q[i1]) + (-6.425181e-02) * (q[i2])
            + (2.761238e-03) * (q[i3]) + (2.762834e-03) * (q[i4]) + (-1.648139e-03) * (q[i5]) + (1.125088e-03) * (q[i6]) + (-1.136944e-03) * (q[i7])
            + (1.078814e-03) * (q[i9]) + (-1.089813e-03) * (q[i10]) + (9.864555e-03) * (q[i11]) + (9.943276e-03) * (q[i12]) + (7.993956e-03) * (q[i15])
            + (-8.089714e-03) * (q[i16]) + (6.607573e-04) * (q[i19]) + (-6.266187e-04) * (q[i20]) + (2.642570e-03) * (q[i21]) + (2.659965e-03) * (q[i22])
            + (-1.325909e-03) * (q[i0] * q[i0]) + (1.317663e-03) * (q[i1] * q[i1]) + (-2.584377e-05) * (q[i2] * q[i2]) + (5.034326e-04) * (q[i3] * q[i3])
            + (-4.924872e-04) * (q[i4] * q[i4]) + (-1.426346e-05) * (q[i5] * q[i5]) + (4.100959e-04) * (q[i6] * q[i6]) + (-4.012663e-04) * (q[i7] * q[i7])
            + (-7.268526e-05) * ((2) * q[i0] * q[i8]) + (-6.196483e-05) * ((2) * q[i1] * q[i8]) + (2.673863e-03) * ((2) * q[i2] * q[i8])
            + (4.515987e-03) * ((2) * q[i3] * q[i8]) + (4.518467e-03) * ((2) * q[i4] * q[i8]) + (-2.254533e-02) * ((2) * q[i5] * q[i8])
            + (-7.129223e-04) * ((2) * q[i6] * q[i8]) + (7.053321e-04) * ((2) * q[i7] * q[i8]) + (1.276820e-05) * ((3) * q[i8] * q[i8])
            + (-1.608917e-04) * ((2) * q[i8] * q[i9]) + (1.536928e-04) * ((2) * q[i8] * q[i10]) + (-1.381044e-03) * ((2) * q[i8] * q[i11])
            + (-1.417740e-03) * ((2) * q[i8] * q[i12]) + (3.791046e-03) * ((2) * q[i8] * q[i15]) + (-3.834838e-03) * ((2) * q[i8] * q[i16])
            + (2.476959e-06) * ((2) * q[i8] * q[i19]) + (6.359908e-07) * ((2) * q[i8] * q[i20]) + (-1.303810e-04) * ((2) * q[i8] * q[i21])
            + (-1.347525e-04) * ((2) * q[i8] * q[i22]) + (4.676212e-04) * (q[i9] * q[i9]) + (-4.618617e-04) * (q[i10] * q[i10])
            + (1.947496e-04) * (q[i11] * q[i11]) + (-1.097498e-04) * (q[i12] * q[i12]) + (2.194621e-03) * (q[i15] * q[i15])
            + (-2.207795e-03) * (q[i16] * q[i16]) + (3.474673e-04) * (q[i19] * q[i19]) + (-3.542810e-04) * (q[i20] * q[i20])
            + (7.875672e-04) * (q[i21] * q[i21]) + (-7.910245e-04) * (q[i22] * q[i22]) + (3.874993e-06) * (q[i0] * q[i1]) + (-3.435710e-03) * (q[i0] * q[i2])
            + (2.277535e-03) * (q[i0] * q[i3]) + (-2.001233e-03) * (q[i0] * q[i4]) + (3.941155e-04) * (q[i0] * q[i5]) + (7.060447e-03) * (q[i0] * q[i6])
            + (3.706215e-04) * (q[i0] * q[i7]) + (7.704389e-04) * (q[i0] * q[i9]) + (-3.394240e-04) * (q[i0] * q[i10]) + (-1.841148e-04) * (q[i0] * q[i11])
            + (1.194779e-04) * (q[i0] * q[i12]) + (7.729900e-04) * (q[i0] * q[i15]) + (7.457658e-04) * (q[i0] * q[i16]) + (6.306952e-04) * (q[i0] * q[i19])
            + (5.005176e-04) * (q[i0] * q[i20]) + (4.348939e-04) * (q[i0] * q[i21]) + (-1.241166e-04) * (q[i0] * q[i22]) + (3.407788e-03) * (q[i1] * q[i2])
            + (2.014678e-03) * (q[i1] * q[i3]) + (-2.251224e-03) * (q[i1] * q[i4]) + (-3.875300e-04) * (q[i1] * q[i5]) + (3.638126e-04) * (q[i1] * q[i6])
            + (7.035108e-03) * (q[i1] * q[i7]) + (-3.468415e-04) * (q[i1] * q[i9]) + (7.726777e-04) * (q[i1] * q[i10]) + (-7.515004e-05) * (q[i1] * q[i11])
            + (2.155798e-04) * (q[i1] * q[i12]) + (7.261643e-04) * (q[i1] * q[i15]) + (7.741995e-04) * (q[i1] * q[i16]) + (5.053310e-04) * (q[i1] * q[i19])
            + (6.299696e-04) * (q[i1] * q[i20]) + (1.145841e-04) * (q[i1] * q[i21]) + (-4.322135e-04) * (q[i1] * q[i22]) + (2.254073e-03) * (q[i2] * q[i3])
            + (-2.213145e-03) * (q[i2] * q[i4]) + (-4.747920e-05) * (q[i2] * q[i5]) + (6.081805e-03) * (q[i2] * q[i6]) + (6.089197e-03) * (q[i2] * q[i7])
            + (-2.776862e-03) * (q[i2] * q[i9]) + (-2.736449e-03) * (q[i2] * q[i10]) + (-6.921279e-04) * (q[i2] * q[i11]) + (8.612909e-04) * (q[i2] * q[i12])
            + (5.179766e-03) * (q[i2] * q[i15]) + (5.254888e-03) * (q[i2] * q[i16]) + (4.322037e-04) * (q[i2] * q[i19]) + (4.214895e-04) * (q[i2] * q[i20])
            + (-3.998145e-04) * (q[i2] * q[i21]) + (4.081732e-04) * (q[i2] * q[i22]) + (5.305136e-06) * (q[i3] * q[i4]) + (3.825928e-04) * (q[i3] * q[i5])
            + (-1.833501e-03) * (q[i3] * q[i6]) + (-1.648222e-03) * (q[i3] * q[i7]) + (-7.396036e-04) * (q[i3] * q[i9]) + (1.941091e-04) * (q[i3] * q[i10])
            + (-1.732590e-03) * (q[i3] * q[i11]) + (6.654939e-04) * (q[i3] * q[i12]) + (-6.313806e-04) * (q[i3] * q[i15]) + (-7.923113e-05) * (q[i3] * q[i16])
            + (-4.347447e-05) * (q[i3] * q[i19]) + (-6.697361e-05) * (q[i3] * q[i20]) + (6.391445e-04) * (q[i3] * q[i21]) + (-4.429403e-04) * (q[i3] * q[i22])
            + (-3.975660e-04) * (q[i4] * q[i5]) + (-1.661487e-03) * (q[i4] * q[i6]) + (-1.843450e-03) * (q[i4] * q[i7]) + (1.938854e-04) * (q[i4] * q[i9])
            + (-7.384261e-04) * (q[i4] * q[i10]) + (-6.265955e-04) * (q[i4] * q[i11]) + (1.743262e-03) * (q[i4] * q[i12]) + (-6.749888e-05) * (q[i4] * q[i15])
            + (-6.186600e-04) * (q[i4] * q[i16]) + (-7.238836e-05) * (q[i4] * q[i19]) + (-3.067289e-05) * (q[i4] * q[i20]) + (4.369758e-04) * (q[i4] * q[i21])
            + (-6.370546e-04) * (q[i4] * q[i22]) + (3.660053e-04) * (q[i5] * q[i6]) + (3.475794e-04) * (q[i5] * q[i7]) + (-1.557785e-03) * (q[i5] * q[i9])
            + (-1.550726e-03) * (q[i5] * q[i10]) + (2.422300e-03) * (q[i5] * q[i11]) + (-2.488921e-03) * (q[i5] * q[i12]) + (2.049749e-03) * (q[i5] * q[i15])
            + (2.082281e-03) * (q[i5] * q[i16]) + (-4.989687e-05) * (q[i5] * q[i19]) + (-6.700013e-05) * (q[i5] * q[i20]) + (3.568588e-04) * (q[i5] * q[i21])
            + (-3.555156e-04) * (q[i5] * q[i22]) + (9.935497e-06) * (q[i6] * q[i7]) + (7.096309e-04) * (q[i6] * q[i9]) + (5.071955e-04) * (q[i6] * q[i10])
            + (-9.533636e-04) * (q[i6] * q[i11]) + (6.504106e-04) * (q[i6] * q[i12]) + (-2.518575e-04) * (q[i6] * q[i15]) + (5.991215e-04) * (q[i6] * q[i16])
            + (-1.606921e-04) * (q[i6] * q[i19]) + (2.586692e-04) * (q[i6] * q[i20]) + (2.789672e-04) * (q[i6] * q[i21]) + (1.065966e-04) * (q[i6] * q[i22])
            + (-5.016305e-04) * (q[i7] * q[i9]) + (-6.984106e-04) * (q[i7] * q[i10]) + (6.851689e-04) * (q[i7] * q[i11]) + (-9.460906e-04) * (q[i7] * q[i12])
            + (-5.965860e-04) * (q[i7] * q[i15]) + (2.638945e-04) * (q[i7] * q[i16]) + (-2.536953e-04) * (q[i7] * q[i19]) + (1.514588e-04) * (q[i7] * q[i20])
            + (1.148712e-04) * (q[i7] * q[i21]) + (2.757292e-04) * (q[i7] * q[i22]) + (1.159071e-06) * (q[i9] * q[i10]) + (3.203332e-04) * (q[i9] * q[i11])
            + (1.751888e-03) * (q[i9] * q[i12]) + (-5.982581e-04) * (q[i9] * q[i15]) + (2.623590e-04) * (q[i9] * q[i16]) + (-3.346973e-04) * (q[i9] * q[i19])
            + (1.654978e-04) * (q[i9] * q[i20]) + (3.364259e-04) * (q[i9] * q[i21]) + (-5.882717e-04) * (q[i9] * q[i22]) + (1.736864e-03) * (q[i10] * q[i11])
            + (3.206554e-04) * (q[i10] * q[i12]) + (-2.604991e-04) * (q[i10] * q[i15]) + (5.953452e-04) * (q[i10] * q[i16])
            + (-1.589789e-04) * (q[i10] * q[i19]) + (3.364738e-04) * (q[i10] * q[i20]) + (-5.852228e-04) * (q[i10] * q[i21])
            + (3.368740e-04) * (q[i10] * q[i22]) + (-1.723473e-06) * (q[i11] * q[i12]) + (3.377785e-03) * (q[i11] * q[i15]) + (3.779367e-04) * (q[i11] * q[i16])
            + (2.604282e-04) * (q[i11] * q[i19]) + (-1.029660e-04) * (q[i11] * q[i20]) + (1.627077e-03) * (q[i11] * q[i21])
            + (-1.225490e-04) * (q[i11] * q[i22]) + (3.782355e-04) * (q[i12] * q[i15]) + (3.393794e-03) * (q[i12] * q[i16])
            + (-9.972883e-05) * (q[i12] * q[i19]) + (2.450668e-04) * (q[i12] * q[i20]) + (1.240925e-04) * (q[i12] * q[i21])
            + (-1.640942e-03) * (q[i12] * q[i22]) + (1.146822e-06) * (q[i15] * q[i16]) + (4.156413e-04) * (q[i15] * q[i19]) + (7.485298e-05) * (q[i15] * q[i20])
            + (2.092922e-04) * (q[i15] * q[i21]) + (3.882708e-05) * (q[i15] * q[i22]) + (-7.327515e-05) * (q[i16] * q[i19])
            + (-3.833072e-04) * (q[i16] * q[i20]) + (3.885455e-05) * (q[i16] * q[i21]) + (2.265615e-04) * (q[i16] * q[i22]) + (4.605962e-08) * (q[i19] * q[i20])
            + (-1.010121e-03) * (q[i19] * q[i21]) + (-1.827928e-07) * (q[i19] * q[i22]) + (6.841617e-06) * (q[i20] * q[i21])
            + (-9.942643e-04) * (q[i20] * q[i22]) + (-2.021385e-07) * (q[i21] * q[i22]);
   }

   public void getJQx9(double[] q, double[][] JQ)
   {
      JQ[1][i9] = (5.489318e-03) * (1) + (4.313477e-03) * ((2) * q[i9]) + (2.868910e-02) * (q[i0]) + (-2.013038e-03) * (q[i1]) + (1.110663e-03) * (q[i2])
            + (-8.353152e-03) * (q[i3]) + (4.705070e-03) * (q[i4]) + (3.173301e-03) * (q[i5]) + (8.561279e-03) * (q[i6]) + (-1.806331e-04) * (q[i7])
            + (1.078814e-03) * (q[i8]) + (-1.742694e-06) * (q[i10]) + (3.720471e-04) * (q[i11]) + (7.644718e-04) * (q[i12]) + (-4.095399e-04) * (q[i15])
            + (4.936475e-04) * (q[i16]) + (2.404055e-04) * (q[i19]) + (4.906908e-04) * (q[i20]) + (3.530960e-04) * (q[i21]) + (-8.328203e-05) * (q[i22])
            + (2.091410e-03) * (q[i0] * q[i0]) + (4.706956e-04) * (q[i1] * q[i1]) + (-7.424691e-05) * (q[i2] * q[i2]) + (-5.224901e-04) * (q[i3] * q[i3])
            + (8.403889e-04) * (q[i4] * q[i4]) + (-5.364463e-04) * (q[i5] * q[i5]) + (-2.264878e-03) * (q[i6] * q[i6]) + (-5.476560e-04) * (q[i7] * q[i7])
            + (-1.608917e-04) * (q[i8] * q[i8]) + (-6.680454e-03) * ((2) * q[i0] * q[i9]) + (1.242082e-03) * ((2) * q[i1] * q[i9])
            + (-1.726621e-03) * ((2) * q[i2] * q[i9]) + (-3.512466e-03) * ((2) * q[i3] * q[i9]) + (7.681636e-04) * ((2) * q[i4] * q[i9])
            + (1.550665e-03) * ((2) * q[i5] * q[i9]) + (-2.245750e-03) * ((2) * q[i6] * q[i9]) + (8.848611e-04) * ((2) * q[i7] * q[i9])
            + (4.676212e-04) * ((2) * q[i8] * q[i9]) + (-6.165831e-04) * ((3) * q[i9] * q[i9]) + (1.651695e-04) * ((2) * q[i9] * q[i10])
            + (-1.470574e-04) * ((2) * q[i9] * q[i11]) + (-2.736369e-04) * ((2) * q[i9] * q[i12]) + (2.042519e-04) * ((2) * q[i9] * q[i15])
            + (1.160582e-04) * ((2) * q[i9] * q[i16]) + (-1.989461e-04) * ((2) * q[i9] * q[i19]) + (1.046171e-04) * ((2) * q[i9] * q[i20])
            + (-5.645742e-05) * ((2) * q[i9] * q[i21]) + (-8.623598e-06) * ((2) * q[i9] * q[i22]) + (-1.636058e-04) * (q[i10] * q[i10])
            + (-7.500403e-05) * (q[i11] * q[i11]) + (-3.851676e-04) * (q[i12] * q[i12]) + (7.354129e-05) * (q[i15] * q[i15])
            + (-7.777213e-05) * (q[i16] * q[i16]) + (1.758296e-05) * (q[i19] * q[i19]) + (3.629395e-05) * (q[i20] * q[i20]) + (6.271647e-05) * (q[i21] * q[i21])
            + (1.106597e-04) * (q[i22] * q[i22]) + (-1.265387e-03) * (q[i0] * q[i1]) + (6.688534e-04) * (q[i0] * q[i2]) + (-5.963577e-04) * (q[i0] * q[i3])
            + (5.563906e-04) * (q[i0] * q[i4]) + (-5.443572e-04) * (q[i0] * q[i5]) + (-1.365839e-02) * (q[i0] * q[i6]) + (3.080839e-04) * (q[i0] * q[i7])
            + (7.704389e-04) * (q[i0] * q[i8]) + (4.473814e-04) * (q[i0] * q[i10]) + (-8.576594e-05) * (q[i0] * q[i11]) + (-3.243681e-04) * (q[i0] * q[i12])
            + (1.634297e-04) * (q[i0] * q[i15]) + (2.365968e-05) * (q[i0] * q[i16]) + (6.612424e-04) * (q[i0] * q[i19]) + (2.772090e-04) * (q[i0] * q[i20])
            + (-3.066195e-04) * (q[i0] * q[i21]) + (-1.234293e-04) * (q[i0] * q[i22]) + (-1.748730e-04) * (q[i1] * q[i2]) + (-6.174670e-04) * (q[i1] * q[i3])
            + (-2.275981e-03) * (q[i1] * q[i4]) + (5.165634e-04) * (q[i1] * q[i5]) + (1.300291e-03) * (q[i1] * q[i6]) + (2.754172e-03) * (q[i1] * q[i7])
            + (-3.468415e-04) * (q[i1] * q[i8]) + (4.457035e-04) * (q[i1] * q[i10]) + (7.662144e-05) * (q[i1] * q[i11]) + (5.675472e-04) * (q[i1] * q[i12])
            + (-2.443037e-05) * (q[i1] * q[i15]) + (3.857545e-04) * (q[i1] * q[i16]) + (-4.597970e-04) * (q[i1] * q[i19]) + (-1.791769e-04) * (q[i1] * q[i20])
            + (7.667513e-05) * (q[i1] * q[i21]) + (5.041266e-04) * (q[i1] * q[i22]) + (8.912089e-05) * (q[i2] * q[i3]) + (-1.504482e-04) * (q[i2] * q[i4])
            + (1.378678e-03) * (q[i2] * q[i5]) + (-3.521530e-03) * (q[i2] * q[i6]) + (1.449597e-03) * (q[i2] * q[i7]) + (-2.776862e-03) * (q[i2] * q[i8])
            + (-1.672587e-05) * (q[i2] * q[i10]) + (-5.373416e-04) * (q[i2] * q[i11]) + (3.698614e-06) * (q[i2] * q[i12]) + (1.200645e-04) * (q[i2] * q[i15])
            + (-2.390041e-04) * (q[i2] * q[i16]) + (1.394536e-04) * (q[i2] * q[i19]) + (2.306254e-04) * (q[i2] * q[i20]) + (-2.424145e-04) * (q[i2] * q[i21])
            + (7.810359e-05) * (q[i2] * q[i22]) + (-9.559773e-04) * (q[i3] * q[i4]) + (6.208945e-04) * (q[i3] * q[i5]) + (-1.084537e-02) * (q[i3] * q[i6])
            + (4.427378e-04) * (q[i3] * q[i7]) + (-7.396036e-04) * (q[i3] * q[i8]) + (-4.205340e-04) * (q[i3] * q[i10]) + (-2.376146e-04) * (q[i3] * q[i11])
            + (3.003056e-04) * (q[i3] * q[i12]) + (-2.719674e-04) * (q[i3] * q[i15]) + (-5.925024e-04) * (q[i3] * q[i16]) + (-3.112606e-04) * (q[i3] * q[i19])
            + (-1.554147e-04) * (q[i3] * q[i20]) + (1.320513e-04) * (q[i3] * q[i21]) + (-3.187391e-04) * (q[i3] * q[i22]) + (9.979039e-04) * (q[i4] * q[i5])
            + (2.256587e-03) * (q[i4] * q[i6]) + (5.375296e-04) * (q[i4] * q[i7]) + (1.938854e-04) * (q[i4] * q[i8]) + (-4.183442e-04) * (q[i4] * q[i10])
            + (-4.617177e-04) * (q[i4] * q[i11]) + (-2.095858e-04) * (q[i4] * q[i12]) + (3.734594e-04) * (q[i4] * q[i15]) + (-3.542908e-04) * (q[i4] * q[i16])
            + (2.444100e-05) * (q[i4] * q[i19]) + (2.528863e-04) * (q[i4] * q[i20]) + (-1.937380e-04) * (q[i4] * q[i21]) + (1.095770e-04) * (q[i4] * q[i22])
            + (2.230545e-03) * (q[i5] * q[i6]) + (-1.172213e-03) * (q[i5] * q[i7]) + (-1.557785e-03) * (q[i5] * q[i8]) + (-3.448780e-04) * (q[i5] * q[i10])
            + (-1.836317e-04) * (q[i5] * q[i11]) + (-2.499222e-04) * (q[i5] * q[i12]) + (-1.230037e-04) * (q[i5] * q[i15]) + (-2.549192e-04) * (q[i5] * q[i16])
            + (3.467943e-04) * (q[i5] * q[i19]) + (-2.783076e-04) * (q[i5] * q[i20]) + (3.350566e-05) * (q[i5] * q[i21]) + (5.021899e-05) * (q[i5] * q[i22])
            + (1.051171e-03) * (q[i6] * q[i7]) + (7.096309e-04) * (q[i6] * q[i8]) + (4.232231e-04) * (q[i6] * q[i10]) + (-6.709169e-04) * (q[i6] * q[i11])
            + (-4.706229e-04) * (q[i6] * q[i12]) + (-9.224733e-05) * (q[i6] * q[i15]) + (-9.940779e-05) * (q[i6] * q[i16]) + (-4.476959e-04) * (q[i6] * q[i19])
            + (1.079777e-04) * (q[i6] * q[i20]) + (-1.558414e-04) * (q[i6] * q[i21]) + (-1.983248e-04) * (q[i6] * q[i22]) + (-5.016305e-04) * (q[i7] * q[i8])
            + (-4.224340e-04) * (q[i7] * q[i10]) + (1.739491e-05) * (q[i7] * q[i11]) + (1.543717e-04) * (q[i7] * q[i12]) + (1.165673e-05) * (q[i7] * q[i15])
            + (3.950348e-04) * (q[i7] * q[i16]) + (3.069477e-04) * (q[i7] * q[i19]) + (-7.608729e-05) * (q[i7] * q[i20]) + (1.492711e-04) * (q[i7] * q[i21])
            + (4.424179e-04) * (q[i7] * q[i22]) + (1.159071e-06) * (q[i8] * q[i10]) + (3.203332e-04) * (q[i8] * q[i11]) + (1.751888e-03) * (q[i8] * q[i12])
            + (-5.982581e-04) * (q[i8] * q[i15]) + (2.623590e-04) * (q[i8] * q[i16]) + (-3.346973e-04) * (q[i8] * q[i19]) + (1.654978e-04) * (q[i8] * q[i20])
            + (3.364259e-04) * (q[i8] * q[i21]) + (-5.882717e-04) * (q[i8] * q[i22]) + (-2.964199e-05) * (q[i10] * q[i11]) + (-2.893927e-05) * (q[i10] * q[i12])
            + (1.243970e-04) * (q[i10] * q[i15]) + (-1.266824e-04) * (q[i10] * q[i16]) + (1.701359e-04) * (q[i10] * q[i19])
            + (-1.680864e-04) * (q[i10] * q[i20]) + (1.291903e-05) * (q[i10] * q[i21]) + (1.231432e-05) * (q[i10] * q[i22])
            + (-9.748947e-06) * (q[i11] * q[i12]) + (-2.354124e-04) * (q[i11] * q[i15]) + (5.645389e-05) * (q[i11] * q[i16])
            + (4.054558e-05) * (q[i11] * q[i19]) + (3.056139e-05) * (q[i11] * q[i20]) + (2.158245e-04) * (q[i11] * q[i21]) + (-7.281869e-05) * (q[i11] * q[i22])
            + (-1.428266e-04) * (q[i12] * q[i15]) + (-2.587763e-05) * (q[i12] * q[i16]) + (-5.674719e-05) * (q[i12] * q[i19])
            + (1.614984e-04) * (q[i12] * q[i20]) + (3.176837e-05) * (q[i12] * q[i21]) + (-5.370242e-05) * (q[i12] * q[i22]) + (6.629910e-05) * (q[i15] * q[i16])
            + (1.229711e-04) * (q[i15] * q[i19]) + (1.753397e-04) * (q[i15] * q[i20]) + (9.140971e-05) * (q[i15] * q[i21]) + (-3.221827e-05) * (q[i15] * q[i22])
            + (-1.553824e-04) * (q[i16] * q[i19]) + (-3.935135e-05) * (q[i16] * q[i20]) + (-2.204518e-05) * (q[i16] * q[i21])
            + (-5.194976e-05) * (q[i16] * q[i22]) + (2.941614e-04) * (q[i19] * q[i20]) + (-6.484038e-05) * (q[i19] * q[i21])
            + (4.005138e-05) * (q[i19] * q[i22]) + (8.318399e-05) * (q[i20] * q[i21]) + (-2.331863e-04) * (q[i20] * q[i22])
            + (4.165778e-05) * (q[i21] * q[i22]);
   }

   public void getJQx10(double[] q, double[][] JQ)
   {
      JQ[1][i10] = (-5.369225e-03) * (1) + (-4.259048e-03) * ((2) * q[i10]) + (-1.991742e-03) * (q[i0]) + (2.838285e-02) * (q[i1]) + (1.109812e-03) * (q[i2])
            + (4.679714e-03) * (q[i3]) + (-8.281846e-03) * (q[i4]) + (3.166160e-03) * (q[i5]) + (1.671832e-04) * (q[i6]) + (-8.450414e-03) * (q[i7])
            + (-1.089813e-03) * (q[i8]) + (-1.742694e-06) * (q[i9]) + (7.565401e-04) * (q[i11]) + (3.643949e-04) * (q[i12]) + (-4.787359e-04) * (q[i15])
            + (4.054355e-04) * (q[i16]) + (-4.933374e-04) * (q[i19]) + (-2.292215e-04) * (q[i20]) + (-7.698729e-05) * (q[i21]) + (3.507823e-04) * (q[i22])
            + (-4.548212e-04) * (q[i0] * q[i0]) + (-2.075597e-03) * (q[i1] * q[i1]) + (6.551763e-05) * (q[i2] * q[i2]) + (-8.534385e-04) * (q[i3] * q[i3])
            + (5.077441e-04) * (q[i4] * q[i4]) + (5.496565e-04) * (q[i5] * q[i5]) + (5.487510e-04) * (q[i6] * q[i6]) + (2.235544e-03) * (q[i7] * q[i7])
            + (1.536928e-04) * (q[i8] * q[i8]) + (1.651695e-04) * (q[i9] * q[i9]) + (1.232645e-03) * ((2) * q[i0] * q[i10])
            + (-6.606172e-03) * ((2) * q[i1] * q[i10]) + (-1.701862e-03) * ((2) * q[i2] * q[i10]) + (7.635137e-04) * ((2) * q[i3] * q[i10])
            + (-3.469145e-03) * ((2) * q[i4] * q[i10]) + (1.532201e-03) * ((2) * q[i5] * q[i10]) + (-8.673242e-04) * ((2) * q[i6] * q[i10])
            + (2.211956e-03) * ((2) * q[i7] * q[i10]) + (-4.618617e-04) * ((2) * q[i8] * q[i10]) + (-1.636058e-04) * ((2) * q[i9] * q[i10])
            + (6.074131e-04) * ((3) * q[i10] * q[i10]) + (-2.739555e-04) * ((2) * q[i10] * q[i11]) + (-1.446726e-04) * ((2) * q[i10] * q[i12])
            + (-1.148960e-04) * ((2) * q[i10] * q[i15]) + (-2.034701e-04) * ((2) * q[i10] * q[i16]) + (-1.036331e-04) * ((2) * q[i10] * q[i19])
            + (1.934299e-04) * ((2) * q[i10] * q[i20]) + (-9.143866e-06) * ((2) * q[i10] * q[i21]) + (-5.630073e-05) * ((2) * q[i10] * q[i22])
            + (3.721239e-04) * (q[i11] * q[i11]) + (6.770298e-05) * (q[i12] * q[i12]) + (7.832320e-05) * (q[i15] * q[i15]) + (-7.392163e-05) * (q[i16] * q[i16])
            + (-3.597388e-05) * (q[i19] * q[i19]) + (-1.600149e-05) * (q[i20] * q[i20]) + (-1.086686e-04) * (q[i21] * q[i21])
            + (-6.206295e-05) * (q[i22] * q[i22]) + (1.262403e-03) * (q[i0] * q[i1]) + (1.871623e-04) * (q[i0] * q[i2]) + (2.242324e-03) * (q[i0] * q[i3])
            + (6.042899e-04) * (q[i0] * q[i4]) + (-5.315843e-04) * (q[i0] * q[i5]) + (2.730496e-03) * (q[i0] * q[i6]) + (1.284695e-03) * (q[i0] * q[i7])
            + (-3.394240e-04) * (q[i0] * q[i8]) + (4.473814e-04) * (q[i0] * q[i9]) + (-5.594774e-04) * (q[i0] * q[i11]) + (-7.310737e-05) * (q[i0] * q[i12])
            + (3.888543e-04) * (q[i0] * q[i15]) + (-1.663085e-05) * (q[i0] * q[i16]) + (-1.775852e-04) * (q[i0] * q[i19]) + (-4.583215e-04) * (q[i0] * q[i20])
            + (-5.004782e-04) * (q[i0] * q[i21]) + (-7.364061e-05) * (q[i0] * q[i22]) + (-6.674760e-04) * (q[i1] * q[i2]) + (-5.566213e-04) * (q[i1] * q[i3])
            + (6.026894e-04) * (q[i1] * q[i4]) + (5.554145e-04) * (q[i1] * q[i5]) + (3.132138e-04) * (q[i1] * q[i6]) + (-1.349584e-02) * (q[i1] * q[i7])
            + (7.726777e-04) * (q[i1] * q[i8]) + (4.457035e-04) * (q[i1] * q[i9]) + (3.202169e-04) * (q[i1] * q[i11]) + (8.348494e-05) * (q[i1] * q[i12])
            + (1.561949e-05) * (q[i1] * q[i15]) + (1.650535e-04) * (q[i1] * q[i16]) + (2.768153e-04) * (q[i1] * q[i19]) + (6.625426e-04) * (q[i1] * q[i20])
            + (1.233639e-04) * (q[i1] * q[i21]) + (2.979040e-04) * (q[i1] * q[i22]) + (1.396181e-04) * (q[i2] * q[i3]) + (-7.647034e-05) * (q[i2] * q[i4])
            + (-1.394716e-03) * (q[i2] * q[i5]) + (1.440729e-03) * (q[i2] * q[i6]) + (-3.469230e-03) * (q[i2] * q[i7]) + (-2.736449e-03) * (q[i2] * q[i8])
            + (-1.672587e-05) * (q[i2] * q[i9]) + (-2.125501e-05) * (q[i2] * q[i11]) + (5.232176e-04) * (q[i2] * q[i12]) + (-2.334638e-04) * (q[i2] * q[i15])
            + (1.254069e-04) * (q[i2] * q[i16]) + (2.362313e-04) * (q[i2] * q[i19]) + (1.414497e-04) * (q[i2] * q[i20]) + (-7.869362e-05) * (q[i2] * q[i21])
            + (2.386402e-04) * (q[i2] * q[i22]) + (9.555538e-04) * (q[i3] * q[i4]) + (-9.887132e-04) * (q[i3] * q[i5]) + (5.088915e-04) * (q[i3] * q[i6])
            + (2.244056e-03) * (q[i3] * q[i7]) + (1.941091e-04) * (q[i3] * q[i8]) + (-4.205340e-04) * (q[i3] * q[i9]) + (2.053841e-04) * (q[i3] * q[i11])
            + (4.718133e-04) * (q[i3] * q[i12]) + (-3.623644e-04) * (q[i3] * q[i15]) + (3.846383e-04) * (q[i3] * q[i16]) + (2.522698e-04) * (q[i3] * q[i19])
            + (3.036936e-05) * (q[i3] * q[i20]) + (-1.085302e-04) * (q[i3] * q[i21]) + (1.945716e-04) * (q[i3] * q[i22]) + (-6.191354e-04) * (q[i4] * q[i5])
            + (4.444126e-04) * (q[i4] * q[i6]) + (-1.075749e-02) * (q[i4] * q[i7]) + (-7.384261e-04) * (q[i4] * q[i8]) + (-4.183442e-04) * (q[i4] * q[i9])
            + (-3.095683e-04) * (q[i4] * q[i11]) + (2.313153e-04) * (q[i4] * q[i12]) + (-5.753068e-04) * (q[i4] * q[i15]) + (-2.718006e-04) * (q[i4] * q[i16])
            + (-1.542513e-04) * (q[i4] * q[i19]) + (-3.165382e-04) * (q[i4] * q[i20]) + (3.152262e-04) * (q[i4] * q[i21]) + (-1.301767e-04) * (q[i4] * q[i22])
            + (-1.157632e-03) * (q[i5] * q[i6]) + (2.225263e-03) * (q[i5] * q[i7]) + (-1.550726e-03) * (q[i5] * q[i8]) + (-3.448780e-04) * (q[i5] * q[i9])
            + (2.410171e-04) * (q[i5] * q[i11]) + (1.793394e-04) * (q[i5] * q[i12]) + (-2.451128e-04) * (q[i5] * q[i15]) + (-1.284285e-04) * (q[i5] * q[i16])
            + (-2.752782e-04) * (q[i5] * q[i19]) + (3.378551e-04) * (q[i5] * q[i20]) + (-5.018386e-05) * (q[i5] * q[i21]) + (-4.503147e-05) * (q[i5] * q[i22])
            + (-1.026063e-03) * (q[i6] * q[i7]) + (5.071955e-04) * (q[i6] * q[i8]) + (4.232231e-04) * (q[i6] * q[i9]) + (1.559962e-04) * (q[i6] * q[i11])
            + (1.286611e-05) * (q[i6] * q[i12]) + (-3.926582e-04) * (q[i6] * q[i15]) + (-1.307033e-05) * (q[i6] * q[i16]) + (7.761660e-05) * (q[i6] * q[i19])
            + (-3.088570e-04) * (q[i6] * q[i20]) + (4.445187e-04) * (q[i6] * q[i21]) + (1.511723e-04) * (q[i6] * q[i22]) + (-6.984106e-04) * (q[i7] * q[i8])
            + (-4.224340e-04) * (q[i7] * q[i9]) + (-4.669284e-04) * (q[i7] * q[i11]) + (-6.696082e-04) * (q[i7] * q[i12]) + (9.726486e-05) * (q[i7] * q[i15])
            + (1.002896e-04) * (q[i7] * q[i16]) + (-1.083990e-04) * (q[i7] * q[i19]) + (4.362536e-04) * (q[i7] * q[i20]) + (-2.008247e-04) * (q[i7] * q[i21])
            + (-1.599506e-04) * (q[i7] * q[i22]) + (1.159071e-06) * (q[i8] * q[i9]) + (1.736864e-03) * (q[i8] * q[i11]) + (3.206554e-04) * (q[i8] * q[i12])
            + (-2.604991e-04) * (q[i8] * q[i15]) + (5.953452e-04) * (q[i8] * q[i16]) + (-1.589789e-04) * (q[i8] * q[i19]) + (3.364738e-04) * (q[i8] * q[i20])
            + (-5.852228e-04) * (q[i8] * q[i21]) + (3.368740e-04) * (q[i8] * q[i22]) + (-2.964199e-05) * (q[i9] * q[i11]) + (-2.893927e-05) * (q[i9] * q[i12])
            + (1.243970e-04) * (q[i9] * q[i15]) + (-1.266824e-04) * (q[i9] * q[i16]) + (1.701359e-04) * (q[i9] * q[i19]) + (-1.680864e-04) * (q[i9] * q[i20])
            + (1.291903e-05) * (q[i9] * q[i21]) + (1.231432e-05) * (q[i9] * q[i22]) + (1.347356e-05) * (q[i11] * q[i12]) + (-2.594386e-05) * (q[i11] * q[i15])
            + (-1.444610e-04) * (q[i11] * q[i16]) + (1.695191e-04) * (q[i11] * q[i19]) + (-5.633695e-05) * (q[i11] * q[i20])
            + (5.205945e-05) * (q[i11] * q[i21]) + (-3.360076e-05) * (q[i11] * q[i22]) + (5.493540e-05) * (q[i12] * q[i15])
            + (-2.329611e-04) * (q[i12] * q[i16]) + (3.057630e-05) * (q[i12] * q[i19]) + (4.205834e-05) * (q[i12] * q[i20]) + (7.259000e-05) * (q[i12] * q[i21])
            + (-2.155244e-04) * (q[i12] * q[i22]) + (-6.673015e-05) * (q[i15] * q[i16]) + (3.761990e-05) * (q[i15] * q[i19])
            + (1.563051e-04) * (q[i15] * q[i20]) + (-4.727244e-05) * (q[i15] * q[i21]) + (-2.014826e-05) * (q[i15] * q[i22])
            + (-1.741995e-04) * (q[i16] * q[i19]) + (-1.220272e-04) * (q[i16] * q[i20]) + (-3.285843e-05) * (q[i16] * q[i21])
            + (8.839866e-05) * (q[i16] * q[i22]) + (-2.943001e-04) * (q[i19] * q[i20]) + (-2.292217e-04) * (q[i19] * q[i21])
            + (8.241691e-05) * (q[i19] * q[i22]) + (4.174105e-05) * (q[i20] * q[i21]) + (-6.584737e-05) * (q[i20] * q[i22])
            + (-4.007233e-05) * (q[i21] * q[i22]);
   }

   public void getJQx11(double[] q, double[][] JQ)
   {
      JQ[1][i11] = (3.027550e-03) * (1) + (5.266495e-03) * ((2) * q[i11]) + (2.180248e-03) * (q[i0]) + (1.245275e-03) * (q[i1]) + (5.822078e-03) * (q[i2])
            + (-1.397326e-03) * (q[i3]) + (-1.337685e-03) * (q[i4]) + (5.353168e-03) * (q[i5]) + (3.825531e-03) * (q[i6]) + (4.162477e-03) * (q[i7])
            + (9.864555e-03) * (q[i8]) + (3.720471e-04) * (q[i9]) + (7.565401e-04) * (q[i10]) + (4.677954e-06) * (q[i12]) + (2.482377e-03) * (q[i15])
            + (7.708882e-05) * (q[i16]) + (-1.381336e-03) * (q[i19]) + (8.973351e-04) * (q[i20]) + (2.252492e-04) * (q[i21]) + (-2.830865e-04) * (q[i22])
            + (5.786766e-04) * (q[i0] * q[i0]) + (4.802014e-04) * (q[i1] * q[i1]) + (6.818229e-04) * (q[i2] * q[i2]) + (-8.827741e-04) * (q[i3] * q[i3])
            + (-1.417677e-03) * (q[i4] * q[i4]) + (1.577033e-05) * (q[i5] * q[i5]) + (4.218537e-04) * (q[i6] * q[i6]) + (-3.949178e-04) * (q[i7] * q[i7])
            + (-1.381044e-03) * (q[i8] * q[i8]) + (-1.470574e-04) * (q[i9] * q[i9]) + (-2.739555e-04) * (q[i10] * q[i10])
            + (3.231714e-04) * ((2) * q[i0] * q[i11]) + (1.890957e-04) * ((2) * q[i1] * q[i11]) + (8.918757e-05) * ((2) * q[i2] * q[i11])
            + (-3.944303e-04) * ((2) * q[i3] * q[i11]) + (-9.777769e-06) * ((2) * q[i4] * q[i11]) + (-1.498545e-03) * ((2) * q[i5] * q[i11])
            + (-3.333575e-04) * ((2) * q[i6] * q[i11]) + (-5.456559e-05) * ((2) * q[i7] * q[i11]) + (1.947496e-04) * ((2) * q[i8] * q[i11])
            + (-7.500403e-05) * ((2) * q[i9] * q[i11]) + (3.721239e-04) * ((2) * q[i10] * q[i11]) + (2.842080e-05) * ((3) * q[i11] * q[i11])
            + (3.174098e-05) * ((2) * q[i11] * q[i12]) + (1.694161e-03) * ((2) * q[i11] * q[i15]) + (1.845193e-04) * ((2) * q[i11] * q[i16])
            + (5.351644e-04) * ((2) * q[i11] * q[i19]) + (-2.024991e-04) * ((2) * q[i11] * q[i20]) + (7.965434e-04) * ((2) * q[i11] * q[i21])
            + (-9.622360e-05) * ((2) * q[i11] * q[i22]) + (3.942835e-05) * (q[i12] * q[i12]) + (2.172944e-03) * (q[i15] * q[i15])
            + (1.075104e-04) * (q[i16] * q[i16]) + (1.771287e-05) * (q[i19] * q[i19]) + (-1.225013e-05) * (q[i20] * q[i20]) + (3.680343e-04) * (q[i21] * q[i21])
            + (5.465612e-05) * (q[i22] * q[i22]) + (-5.326159e-04) * (q[i0] * q[i1]) + (4.646967e-04) * (q[i0] * q[i2]) + (-8.594092e-04) * (q[i0] * q[i3])
            + (1.327075e-03) * (q[i0] * q[i4]) + (8.634613e-04) * (q[i0] * q[i5]) + (-1.453379e-03) * (q[i0] * q[i6]) + (-1.745295e-03) * (q[i0] * q[i7])
            + (-1.841148e-04) * (q[i0] * q[i8]) + (-8.576594e-05) * (q[i0] * q[i9]) + (-5.594774e-04) * (q[i0] * q[i10]) + (6.593555e-04) * (q[i0] * q[i12])
            + (4.933105e-04) * (q[i0] * q[i15]) + (5.187873e-04) * (q[i0] * q[i16]) + (5.942319e-04) * (q[i0] * q[i19]) + (-5.535935e-05) * (q[i0] * q[i20])
            + (4.090679e-04) * (q[i0] * q[i21]) + (-3.014434e-04) * (q[i0] * q[i22]) + (1.645258e-03) * (q[i1] * q[i2]) + (-1.167348e-03) * (q[i1] * q[i3])
            + (-1.489637e-03) * (q[i1] * q[i4]) + (4.673067e-04) * (q[i1] * q[i5]) + (9.677180e-04) * (q[i1] * q[i6]) + (1.099982e-03) * (q[i1] * q[i7])
            + (-7.515004e-05) * (q[i1] * q[i8]) + (7.662144e-05) * (q[i1] * q[i9]) + (3.202169e-04) * (q[i1] * q[i10]) + (6.634929e-04) * (q[i1] * q[i12])
            + (6.052294e-04) * (q[i1] * q[i15]) + (8.497943e-07) * (q[i1] * q[i16]) + (3.752991e-04) * (q[i1] * q[i19]) + (1.283739e-04) * (q[i1] * q[i20])
            + (-3.525242e-04) * (q[i1] * q[i21]) + (1.552551e-04) * (q[i1] * q[i22]) + (-1.113976e-04) * (q[i2] * q[i3]) + (1.860122e-04) * (q[i2] * q[i4])
            + (1.879672e-03) * (q[i2] * q[i5]) + (-8.145888e-04) * (q[i2] * q[i6]) + (1.189835e-04) * (q[i2] * q[i7]) + (-6.921279e-04) * (q[i2] * q[i8])
            + (-5.373416e-04) * (q[i2] * q[i9]) + (-2.125501e-05) * (q[i2] * q[i10]) + (6.074138e-04) * (q[i2] * q[i12]) + (3.323139e-03) * (q[i2] * q[i15])
            + (-4.022756e-04) * (q[i2] * q[i16]) + (1.742883e-03) * (q[i2] * q[i19]) + (5.947629e-05) * (q[i2] * q[i20]) + (2.158425e-04) * (q[i2] * q[i21])
            + (-4.507722e-04) * (q[i2] * q[i22]) + (1.488707e-03) * (q[i3] * q[i4]) + (-2.169193e-03) * (q[i3] * q[i5]) + (-5.300235e-04) * (q[i3] * q[i6])
            + (3.831200e-04) * (q[i3] * q[i7]) + (-1.732590e-03) * (q[i3] * q[i8]) + (-2.376146e-04) * (q[i3] * q[i9]) + (2.053841e-04) * (q[i3] * q[i10])
            + (-1.561455e-04) * (q[i3] * q[i12]) + (-1.538416e-03) * (q[i3] * q[i15]) + (7.868216e-04) * (q[i3] * q[i16]) + (4.310239e-05) * (q[i3] * q[i19])
            + (3.062487e-05) * (q[i3] * q[i20]) + (-4.160914e-04) * (q[i3] * q[i21]) + (-2.337426e-04) * (q[i3] * q[i22]) + (5.179589e-04) * (q[i4] * q[i5])
            + (-5.527570e-04) * (q[i4] * q[i6]) + (-1.102490e-03) * (q[i4] * q[i7]) + (-6.265955e-04) * (q[i4] * q[i8]) + (-4.617177e-04) * (q[i4] * q[i9])
            + (-3.095683e-04) * (q[i4] * q[i10]) + (-1.573023e-04) * (q[i4] * q[i12]) + (5.701258e-05) * (q[i4] * q[i15]) + (-5.416789e-04) * (q[i4] * q[i16])
            + (-4.461548e-04) * (q[i4] * q[i19]) + (-2.933765e-05) * (q[i4] * q[i20]) + (-1.990022e-04) * (q[i4] * q[i21]) + (1.956478e-04) * (q[i4] * q[i22])
            + (-2.735815e-04) * (q[i5] * q[i6]) + (4.523701e-05) * (q[i5] * q[i7]) + (2.422300e-03) * (q[i5] * q[i8]) + (-1.836317e-04) * (q[i5] * q[i9])
            + (2.410171e-04) * (q[i5] * q[i10]) + (-6.758906e-05) * (q[i5] * q[i12]) + (2.045775e-03) * (q[i5] * q[i15]) + (-6.262812e-04) * (q[i5] * q[i16])
            + (1.178573e-03) * (q[i5] * q[i19]) + (-4.248532e-04) * (q[i5] * q[i20]) + (1.428402e-04) * (q[i5] * q[i21]) + (-6.106314e-04) * (q[i5] * q[i22])
            + (8.457240e-04) * (q[i6] * q[i7]) + (-9.533636e-04) * (q[i6] * q[i8]) + (-6.709169e-04) * (q[i6] * q[i9]) + (1.559962e-04) * (q[i6] * q[i10])
            + (6.787517e-06) * (q[i6] * q[i12]) + (1.036661e-03) * (q[i6] * q[i15]) + (-6.698623e-05) * (q[i6] * q[i16]) + (-7.845989e-05) * (q[i6] * q[i19])
            + (4.371767e-04) * (q[i6] * q[i20]) + (3.351916e-04) * (q[i6] * q[i21]) + (1.009963e-04) * (q[i6] * q[i22]) + (6.851689e-04) * (q[i7] * q[i8])
            + (1.739491e-05) * (q[i7] * q[i9]) + (-4.669284e-04) * (q[i7] * q[i10]) + (-5.702689e-06) * (q[i7] * q[i12]) + (6.396486e-04) * (q[i7] * q[i15])
            + (6.519627e-04) * (q[i7] * q[i16]) + (2.386295e-04) * (q[i7] * q[i19]) + (-1.468150e-04) * (q[i7] * q[i20]) + (4.271983e-04) * (q[i7] * q[i21])
            + (2.477358e-04) * (q[i7] * q[i22]) + (3.203332e-04) * (q[i8] * q[i9]) + (1.736864e-03) * (q[i8] * q[i10]) + (-1.723473e-06) * (q[i8] * q[i12])
            + (3.377785e-03) * (q[i8] * q[i15]) + (3.779367e-04) * (q[i8] * q[i16]) + (2.604282e-04) * (q[i8] * q[i19]) + (-1.029660e-04) * (q[i8] * q[i20])
            + (1.627077e-03) * (q[i8] * q[i21]) + (-1.225490e-04) * (q[i8] * q[i22]) + (-2.964199e-05) * (q[i9] * q[i10]) + (-9.748947e-06) * (q[i9] * q[i12])
            + (-2.354124e-04) * (q[i9] * q[i15]) + (5.645389e-05) * (q[i9] * q[i16]) + (4.054558e-05) * (q[i9] * q[i19]) + (3.056139e-05) * (q[i9] * q[i20])
            + (2.158245e-04) * (q[i9] * q[i21]) + (-7.281869e-05) * (q[i9] * q[i22]) + (1.347356e-05) * (q[i10] * q[i12]) + (-2.594386e-05) * (q[i10] * q[i15])
            + (-1.444610e-04) * (q[i10] * q[i16]) + (1.695191e-04) * (q[i10] * q[i19]) + (-5.633695e-05) * (q[i10] * q[i20])
            + (5.205945e-05) * (q[i10] * q[i21]) + (-3.360076e-05) * (q[i10] * q[i22]) + (2.662090e-04) * (q[i12] * q[i15])
            + (-2.646526e-04) * (q[i12] * q[i16]) + (3.777457e-04) * (q[i12] * q[i19]) + (-3.773006e-04) * (q[i12] * q[i20])
            + (-2.386592e-04) * (q[i12] * q[i21]) + (-2.399177e-04) * (q[i12] * q[i22]) + (-1.718668e-04) * (q[i15] * q[i16])
            + (-3.795920e-04) * (q[i15] * q[i19]) + (-3.056488e-05) * (q[i15] * q[i20]) + (-4.355309e-04) * (q[i15] * q[i21])
            + (-7.896743e-05) * (q[i15] * q[i22]) + (-1.546858e-04) * (q[i16] * q[i19]) + (1.975458e-04) * (q[i16] * q[i20])
            + (-3.401647e-04) * (q[i16] * q[i21]) + (-3.060890e-04) * (q[i16] * q[i22]) + (-1.446436e-04) * (q[i19] * q[i20])
            + (-1.896972e-03) * (q[i19] * q[i21]) + (1.041701e-04) * (q[i19] * q[i22]) + (2.664331e-05) * (q[i20] * q[i21])
            + (-2.688682e-04) * (q[i20] * q[i22]) + (1.066602e-04) * (q[i21] * q[i22]);
   }

   public void getJQx12(double[] q, double[][] JQ)
   {
      JQ[1][i12] = (3.341064e-03) * (1) + (-5.416358e-03) * ((2) * q[i12]) + (-1.271582e-03) * (q[i0]) + (-2.172879e-03) * (q[i1]) + (-5.992994e-03) * (q[i2])
            + (1.370022e-03) * (q[i3]) + (1.413711e-03) * (q[i4]) + (-5.393804e-03) * (q[i5]) + (4.177021e-03) * (q[i6]) + (3.783637e-03) * (q[i7])
            + (9.943276e-03) * (q[i8]) + (7.644718e-04) * (q[i9]) + (3.643949e-04) * (q[i10]) + (4.677954e-06) * (q[i11]) + (8.698621e-05) * (q[i15])
            + (2.562287e-03) * (q[i16]) + (9.009046e-04) * (q[i19]) + (-1.403589e-03) * (q[i20]) + (2.860958e-04) * (q[i21]) + (-2.284272e-04) * (q[i22])
            + (4.670338e-04) * (q[i0] * q[i0]) + (5.728565e-04) * (q[i1] * q[i1]) + (6.679535e-04) * (q[i2] * q[i2]) + (-1.418222e-03) * (q[i3] * q[i3])
            + (-8.833817e-04) * (q[i4] * q[i4]) + (1.886787e-05) * (q[i5] * q[i5]) + (-3.891096e-04) * (q[i6] * q[i6]) + (4.162036e-04) * (q[i7] * q[i7])
            + (-1.417740e-03) * (q[i8] * q[i8]) + (-2.736369e-04) * (q[i9] * q[i9]) + (-1.446726e-04) * (q[i10] * q[i10]) + (3.174098e-05) * (q[i11] * q[i11])
            + (1.600572e-04) * ((2) * q[i0] * q[i12]) + (3.179734e-04) * ((2) * q[i1] * q[i12]) + (-4.404651e-06) * ((2) * q[i2] * q[i12])
            + (1.547685e-05) * ((2) * q[i3] * q[i12]) + (-4.022853e-04) * ((2) * q[i4] * q[i12]) + (-1.566259e-03) * ((2) * q[i5] * q[i12])
            + (7.173906e-05) * ((2) * q[i6] * q[i12]) + (3.419957e-04) * ((2) * q[i7] * q[i12]) + (-1.097498e-04) * ((2) * q[i8] * q[i12])
            + (-3.851676e-04) * ((2) * q[i9] * q[i12]) + (6.770298e-05) * ((2) * q[i10] * q[i12]) + (3.942835e-05) * ((2) * q[i11] * q[i12])
            + (-2.357827e-05) * ((3) * q[i12] * q[i12]) + (-1.939265e-04) * ((2) * q[i12] * q[i15]) + (-1.712646e-03) * ((2) * q[i12] * q[i16])
            + (1.989071e-04) * ((2) * q[i12] * q[i19]) + (-5.191837e-04) * ((2) * q[i12] * q[i20]) + (-9.978708e-05) * ((2) * q[i12] * q[i21])
            + (8.008094e-04) * ((2) * q[i12] * q[i22]) + (1.105513e-04) * (q[i15] * q[i15]) + (2.204910e-03) * (q[i16] * q[i16])
            + (-1.250546e-05) * (q[i19] * q[i19]) + (3.205542e-05) * (q[i20] * q[i20]) + (5.457835e-05) * (q[i21] * q[i21]) + (3.705402e-04) * (q[i22] * q[i22])
            + (-5.398613e-04) * (q[i0] * q[i1]) + (1.638474e-03) * (q[i0] * q[i2]) + (-1.465488e-03) * (q[i0] * q[i3]) + (-1.168324e-03) * (q[i0] * q[i4])
            + (4.483416e-04) * (q[i0] * q[i5]) + (-1.077763e-03) * (q[i0] * q[i6]) + (-9.625702e-04) * (q[i0] * q[i7]) + (1.194779e-04) * (q[i0] * q[i8])
            + (-3.243681e-04) * (q[i0] * q[i9]) + (-7.310737e-05) * (q[i0] * q[i10]) + (6.593555e-04) * (q[i0] * q[i11]) + (-2.343484e-06) * (q[i0] * q[i15])
            + (-6.241420e-04) * (q[i0] * q[i16]) + (-1.363830e-04) * (q[i0] * q[i19]) + (-3.798038e-04) * (q[i0] * q[i20]) + (1.552164e-04) * (q[i0] * q[i21])
            + (-3.450505e-04) * (q[i0] * q[i22]) + (4.574802e-04) * (q[i1] * q[i2]) + (1.317098e-03) * (q[i1] * q[i3]) + (-8.532372e-04) * (q[i1] * q[i4])
            + (8.766858e-04) * (q[i1] * q[i5]) + (1.764341e-03) * (q[i1] * q[i6]) + (1.474185e-03) * (q[i1] * q[i7]) + (2.155798e-04) * (q[i1] * q[i8])
            + (5.675472e-04) * (q[i1] * q[i9]) + (8.348494e-05) * (q[i1] * q[i10]) + (6.634929e-04) * (q[i1] * q[i11]) + (-5.293381e-04) * (q[i1] * q[i15])
            + (-4.937368e-04) * (q[i1] * q[i16]) + (5.602229e-05) * (q[i1] * q[i19]) + (-5.984771e-04) * (q[i1] * q[i20]) + (-2.965218e-04) * (q[i1] * q[i21])
            + (4.076207e-04) * (q[i1] * q[i22]) + (2.118474e-04) * (q[i2] * q[i3]) + (-8.613907e-05) * (q[i2] * q[i4]) + (1.853627e-03) * (q[i2] * q[i5])
            + (-1.063741e-04) * (q[i2] * q[i6]) + (8.198402e-04) * (q[i2] * q[i7]) + (8.612909e-04) * (q[i2] * q[i8]) + (3.698614e-06) * (q[i2] * q[i9])
            + (5.232176e-04) * (q[i2] * q[i10]) + (6.074138e-04) * (q[i2] * q[i11]) + (3.809445e-04) * (q[i2] * q[i15]) + (-3.340560e-03) * (q[i2] * q[i16])
            + (-7.327170e-05) * (q[i2] * q[i19]) + (-1.726330e-03) * (q[i2] * q[i20]) + (-4.512358e-04) * (q[i2] * q[i21]) + (2.083152e-04) * (q[i2] * q[i22])
            + (1.482513e-03) * (q[i3] * q[i4]) + (5.292480e-04) * (q[i3] * q[i5]) + (1.113857e-03) * (q[i3] * q[i6]) + (5.663898e-04) * (q[i3] * q[i7])
            + (6.654939e-04) * (q[i3] * q[i8]) + (3.003056e-04) * (q[i3] * q[i9]) + (4.718133e-04) * (q[i3] * q[i10]) + (-1.561455e-04) * (q[i3] * q[i11])
            + (5.428375e-04) * (q[i3] * q[i15]) + (-5.822735e-05) * (q[i3] * q[i16]) + (2.744032e-05) * (q[i3] * q[i19]) + (4.514248e-04) * (q[i3] * q[i20])
            + (1.865373e-04) * (q[i3] * q[i21]) + (-2.017979e-04) * (q[i3] * q[i22]) + (-2.150434e-03) * (q[i4] * q[i5]) + (-4.070025e-04) * (q[i4] * q[i6])
            + (5.165377e-04) * (q[i4] * q[i7]) + (1.743262e-03) * (q[i4] * q[i8]) + (-2.095858e-04) * (q[i4] * q[i9]) + (2.313153e-04) * (q[i4] * q[i10])
            + (-1.573023e-04) * (q[i4] * q[i11]) + (-7.825143e-04) * (q[i4] * q[i15]) + (1.539207e-03) * (q[i4] * q[i16]) + (-3.089071e-05) * (q[i4] * q[i19])
            + (-5.005504e-05) * (q[i4] * q[i20]) + (-2.380424e-04) * (q[i4] * q[i21]) + (-4.228184e-04) * (q[i4] * q[i22]) + (-6.490981e-05) * (q[i5] * q[i6])
            + (2.827897e-04) * (q[i5] * q[i7]) + (-2.488921e-03) * (q[i5] * q[i8]) + (-2.499222e-04) * (q[i5] * q[i9]) + (1.793394e-04) * (q[i5] * q[i10])
            + (-6.758906e-05) * (q[i5] * q[i11]) + (6.146331e-04) * (q[i5] * q[i15]) + (-2.053211e-03) * (q[i5] * q[i16]) + (4.292040e-04) * (q[i5] * q[i19])
            + (-1.178317e-03) * (q[i5] * q[i20]) + (-6.021615e-04) * (q[i5] * q[i21]) + (1.624863e-04) * (q[i5] * q[i22]) + (8.422181e-04) * (q[i6] * q[i7])
            + (6.504106e-04) * (q[i6] * q[i8]) + (-4.706229e-04) * (q[i6] * q[i9]) + (1.286611e-05) * (q[i6] * q[i10]) + (6.787517e-06) * (q[i6] * q[i11])
            + (6.488194e-04) * (q[i6] * q[i15]) + (6.449914e-04) * (q[i6] * q[i16]) + (-1.415264e-04) * (q[i6] * q[i19]) + (2.387709e-04) * (q[i6] * q[i20])
            + (-2.481534e-04) * (q[i6] * q[i21]) + (-4.363672e-04) * (q[i6] * q[i22]) + (-9.460906e-04) * (q[i7] * q[i8]) + (1.543717e-04) * (q[i7] * q[i9])
            + (-6.696082e-04) * (q[i7] * q[i10]) + (-5.702689e-06) * (q[i7] * q[i11]) + (-6.701922e-05) * (q[i7] * q[i15]) + (1.026269e-03) * (q[i7] * q[i16])
            + (4.347151e-04) * (q[i7] * q[i19]) + (-7.737333e-05) * (q[i7] * q[i20]) + (-1.026845e-04) * (q[i7] * q[i21]) + (-3.307243e-04) * (q[i7] * q[i22])
            + (1.751888e-03) * (q[i8] * q[i9]) + (3.206554e-04) * (q[i8] * q[i10]) + (-1.723473e-06) * (q[i8] * q[i11]) + (3.782355e-04) * (q[i8] * q[i15])
            + (3.393794e-03) * (q[i8] * q[i16]) + (-9.972883e-05) * (q[i8] * q[i19]) + (2.450668e-04) * (q[i8] * q[i20]) + (1.240925e-04) * (q[i8] * q[i21])
            + (-1.640942e-03) * (q[i8] * q[i22]) + (-2.893927e-05) * (q[i9] * q[i10]) + (-9.748947e-06) * (q[i9] * q[i11]) + (-1.428266e-04) * (q[i9] * q[i15])
            + (-2.587763e-05) * (q[i9] * q[i16]) + (-5.674719e-05) * (q[i9] * q[i19]) + (1.614984e-04) * (q[i9] * q[i20]) + (3.176837e-05) * (q[i9] * q[i21])
            + (-5.370242e-05) * (q[i9] * q[i22]) + (1.347356e-05) * (q[i10] * q[i11]) + (5.493540e-05) * (q[i10] * q[i15]) + (-2.329611e-04) * (q[i10] * q[i16])
            + (3.057630e-05) * (q[i10] * q[i19]) + (4.205834e-05) * (q[i10] * q[i20]) + (7.259000e-05) * (q[i10] * q[i21]) + (-2.155244e-04) * (q[i10] * q[i22])
            + (2.662090e-04) * (q[i11] * q[i15]) + (-2.646526e-04) * (q[i11] * q[i16]) + (3.777457e-04) * (q[i11] * q[i19])
            + (-3.773006e-04) * (q[i11] * q[i20]) + (-2.386592e-04) * (q[i11] * q[i21]) + (-2.399177e-04) * (q[i11] * q[i22])
            + (-1.650153e-04) * (q[i15] * q[i16]) + (1.966239e-04) * (q[i15] * q[i19]) + (-1.517018e-04) * (q[i15] * q[i20])
            + (3.056551e-04) * (q[i15] * q[i21]) + (3.410488e-04) * (q[i15] * q[i22]) + (-2.977686e-05) * (q[i16] * q[i19])
            + (-3.921195e-04) * (q[i16] * q[i20]) + (7.957061e-05) * (q[i16] * q[i21]) + (4.378143e-04) * (q[i16] * q[i22])
            + (-1.457396e-04) * (q[i19] * q[i20]) + (2.692456e-04) * (q[i19] * q[i21]) + (-2.546263e-05) * (q[i19] * q[i22])
            + (-1.057655e-04) * (q[i20] * q[i21]) + (1.885072e-03) * (q[i20] * q[i22]) + (1.076934e-04) * (q[i21] * q[i22]);
   }

   public void getJQx15(double[] q, double[][] JQ)
   {
      JQ[1][i15] = (-2.513163e-03) * (1) + (5.216814e-03) * ((2) * q[i15]) + (6.716320e-04) * (q[i0]) + (2.446439e-03) * (q[i1]) + (3.217467e-03) * (q[i2])
            + (4.420613e-03) * (q[i3]) + (4.708550e-03) * (q[i4]) + (-1.472151e-02) * (q[i5]) + (-9.606252e-04) * (q[i6]) + (1.467304e-03) * (q[i7])
            + (7.993956e-03) * (q[i8]) + (-4.095399e-04) * (q[i9]) + (-4.787359e-04) * (q[i10]) + (2.482377e-03) * (q[i11]) + (8.698621e-05) * (q[i12])
            + (-3.239600e-06) * (q[i16]) + (1.028355e-03) * (q[i19]) + (-4.449288e-04) * (q[i20]) + (2.043079e-03) * (q[i21]) + (-1.155256e-04) * (q[i22])
            + (-5.193199e-04) * (q[i0] * q[i0]) + (-6.010900e-04) * (q[i1] * q[i1]) + (-3.073569e-03) * (q[i2] * q[i2]) + (8.495688e-05) * (q[i3] * q[i3])
            + (-6.731487e-05) * (q[i4] * q[i4]) + (-5.557385e-04) * (q[i5] * q[i5]) + (-9.299073e-05) * (q[i6] * q[i6]) + (-6.495229e-04) * (q[i7] * q[i7])
            + (3.791046e-03) * (q[i8] * q[i8]) + (2.042519e-04) * (q[i9] * q[i9]) + (-1.148960e-04) * (q[i10] * q[i10]) + (1.694161e-03) * (q[i11] * q[i11])
            + (-1.939265e-04) * (q[i12] * q[i12]) + (1.483345e-04) * ((2) * q[i0] * q[i15]) + (1.062923e-04) * ((2) * q[i1] * q[i15])
            + (1.988861e-03) * ((2) * q[i2] * q[i15]) + (2.427431e-04) * ((2) * q[i3] * q[i15]) + (6.388368e-04) * ((2) * q[i4] * q[i15])
            + (-2.441580e-03) * ((2) * q[i5] * q[i15]) + (-1.249697e-05) * ((2) * q[i6] * q[i15]) + (-7.857293e-05) * ((2) * q[i7] * q[i15])
            + (2.194621e-03) * ((2) * q[i8] * q[i15]) + (7.354129e-05) * ((2) * q[i9] * q[i15]) + (7.832320e-05) * ((2) * q[i10] * q[i15])
            + (2.172944e-03) * ((2) * q[i11] * q[i15]) + (1.105513e-04) * ((2) * q[i12] * q[i15]) + (5.377181e-04) * ((3) * q[i15] * q[i15])
            + (-1.811882e-05) * ((2) * q[i15] * q[i16]) + (1.538827e-04) * ((2) * q[i15] * q[i19]) + (3.743298e-05) * ((2) * q[i15] * q[i20])
            + (5.407792e-04) * ((2) * q[i15] * q[i21]) + (6.319877e-05) * ((2) * q[i15] * q[i22]) + (1.829172e-05) * (q[i16] * q[i16])
            + (2.848873e-04) * (q[i19] * q[i19]) + (-1.412690e-04) * (q[i20] * q[i20]) + (7.180441e-04) * (q[i21] * q[i21]) + (4.040670e-05) * (q[i22] * q[i22])
            + (1.070189e-03) * (q[i0] * q[i1]) + (-1.319058e-03) * (q[i0] * q[i2]) + (2.222247e-03) * (q[i0] * q[i3]) + (2.300713e-04) * (q[i0] * q[i4])
            + (-6.115209e-04) * (q[i0] * q[i5]) + (7.646598e-04) * (q[i0] * q[i6]) + (-6.808599e-04) * (q[i0] * q[i7]) + (7.729900e-04) * (q[i0] * q[i8])
            + (1.634297e-04) * (q[i0] * q[i9]) + (3.888543e-04) * (q[i0] * q[i10]) + (4.933105e-04) * (q[i0] * q[i11]) + (-2.343484e-06) * (q[i0] * q[i12])
            + (5.167769e-04) * (q[i0] * q[i16]) + (3.401895e-05) * (q[i0] * q[i19]) + (8.032369e-06) * (q[i0] * q[i20]) + (2.456465e-04) * (q[i0] * q[i21])
            + (3.832412e-04) * (q[i0] * q[i22]) + (-2.156562e-03) * (q[i1] * q[i2]) + (2.786214e-04) * (q[i1] * q[i3]) + (-6.116358e-04) * (q[i1] * q[i4])
            + (1.168227e-03) * (q[i1] * q[i5]) + (1.927100e-04) * (q[i1] * q[i6]) + (1.383560e-03) * (q[i1] * q[i7]) + (7.261643e-04) * (q[i1] * q[i8])
            + (-2.443037e-05) * (q[i1] * q[i9]) + (1.561949e-05) * (q[i1] * q[i10]) + (6.052294e-04) * (q[i1] * q[i11]) + (-5.293381e-04) * (q[i1] * q[i12])
            + (5.077336e-04) * (q[i1] * q[i16]) + (1.600298e-04) * (q[i1] * q[i19]) + (-6.659789e-04) * (q[i1] * q[i20]) + (-7.249146e-05) * (q[i1] * q[i21])
            + (-3.137002e-05) * (q[i1] * q[i22]) + (1.898353e-03) * (q[i2] * q[i3]) + (4.359900e-04) * (q[i2] * q[i4]) + (-2.309920e-03) * (q[i2] * q[i5])
            + (6.365126e-04) * (q[i2] * q[i6]) + (-4.190663e-04) * (q[i2] * q[i7]) + (5.179766e-03) * (q[i2] * q[i8]) + (1.200645e-04) * (q[i2] * q[i9])
            + (-2.334638e-04) * (q[i2] * q[i10]) + (3.323139e-03) * (q[i2] * q[i11]) + (3.809445e-04) * (q[i2] * q[i12]) + (5.847767e-05) * (q[i2] * q[i16])
            + (-9.102059e-04) * (q[i2] * q[i19]) + (-7.358046e-04) * (q[i2] * q[i20]) + (1.149993e-04) * (q[i2] * q[i21]) + (2.249188e-04) * (q[i2] * q[i22])
            + (-2.339222e-05) * (q[i3] * q[i4]) + (1.139528e-03) * (q[i3] * q[i5]) + (8.047113e-04) * (q[i3] * q[i6]) + (-1.599024e-03) * (q[i3] * q[i7])
            + (-6.313806e-04) * (q[i3] * q[i8]) + (-2.719674e-04) * (q[i3] * q[i9]) + (-3.623644e-04) * (q[i3] * q[i10]) + (-1.538416e-03) * (q[i3] * q[i11])
            + (5.428375e-04) * (q[i3] * q[i12]) + (3.354747e-04) * (q[i3] * q[i16]) + (-6.221902e-04) * (q[i3] * q[i19]) + (1.336437e-04) * (q[i3] * q[i20])
            + (1.475200e-04) * (q[i3] * q[i21]) + (2.283656e-04) * (q[i3] * q[i22]) + (-3.856767e-04) * (q[i4] * q[i5]) + (-9.036454e-04) * (q[i4] * q[i6])
            + (-7.693415e-04) * (q[i4] * q[i7]) + (-6.749888e-05) * (q[i4] * q[i8]) + (3.734594e-04) * (q[i4] * q[i9]) + (-5.753068e-04) * (q[i4] * q[i10])
            + (5.701258e-05) * (q[i4] * q[i11]) + (-7.825143e-04) * (q[i4] * q[i12]) + (3.344007e-04) * (q[i4] * q[i16]) + (3.299048e-04) * (q[i4] * q[i19])
            + (3.810390e-04) * (q[i4] * q[i20]) + (4.816924e-04) * (q[i4] * q[i21]) + (7.991379e-05) * (q[i4] * q[i22]) + (7.044322e-04) * (q[i5] * q[i6])
            + (4.890998e-04) * (q[i5] * q[i7]) + (2.049749e-03) * (q[i5] * q[i8]) + (-1.230037e-04) * (q[i5] * q[i9]) + (-2.451128e-04) * (q[i5] * q[i10])
            + (2.045775e-03) * (q[i5] * q[i11]) + (6.146331e-04) * (q[i5] * q[i12]) + (-1.212710e-05) * (q[i5] * q[i16]) + (-3.948275e-04) * (q[i5] * q[i19])
            + (-3.213971e-04) * (q[i5] * q[i20]) + (-1.647586e-03) * (q[i5] * q[i21]) + (-3.132046e-04) * (q[i5] * q[i22]) + (1.090265e-04) * (q[i6] * q[i7])
            + (-2.518575e-04) * (q[i6] * q[i8]) + (-9.224733e-05) * (q[i6] * q[i9]) + (-3.926582e-04) * (q[i6] * q[i10]) + (1.036661e-03) * (q[i6] * q[i11])
            + (6.488194e-04) * (q[i6] * q[i12]) + (-3.029794e-04) * (q[i6] * q[i16]) + (3.436680e-04) * (q[i6] * q[i19]) + (-5.554971e-05) * (q[i6] * q[i20])
            + (-2.115197e-04) * (q[i6] * q[i21]) + (1.826301e-04) * (q[i6] * q[i22]) + (-5.965860e-04) * (q[i7] * q[i8]) + (1.165673e-05) * (q[i7] * q[i9])
            + (9.726486e-05) * (q[i7] * q[i10]) + (6.396486e-04) * (q[i7] * q[i11]) + (-6.701922e-05) * (q[i7] * q[i12]) + (3.025784e-04) * (q[i7] * q[i16])
            + (1.200177e-04) * (q[i7] * q[i19]) + (4.572318e-04) * (q[i7] * q[i20]) + (3.478372e-04) * (q[i7] * q[i21]) + (-4.318086e-04) * (q[i7] * q[i22])
            + (-5.982581e-04) * (q[i8] * q[i9]) + (-2.604991e-04) * (q[i8] * q[i10]) + (3.377785e-03) * (q[i8] * q[i11]) + (3.782355e-04) * (q[i8] * q[i12])
            + (1.146822e-06) * (q[i8] * q[i16]) + (4.156413e-04) * (q[i8] * q[i19]) + (7.485298e-05) * (q[i8] * q[i20]) + (2.092922e-04) * (q[i8] * q[i21])
            + (3.882708e-05) * (q[i8] * q[i22]) + (1.243970e-04) * (q[i9] * q[i10]) + (-2.354124e-04) * (q[i9] * q[i11]) + (-1.428266e-04) * (q[i9] * q[i12])
            + (6.629910e-05) * (q[i9] * q[i16]) + (1.229711e-04) * (q[i9] * q[i19]) + (1.753397e-04) * (q[i9] * q[i20]) + (9.140971e-05) * (q[i9] * q[i21])
            + (-3.221827e-05) * (q[i9] * q[i22]) + (-2.594386e-05) * (q[i10] * q[i11]) + (5.493540e-05) * (q[i10] * q[i12])
            + (-6.673015e-05) * (q[i10] * q[i16]) + (3.761990e-05) * (q[i10] * q[i19]) + (1.563051e-04) * (q[i10] * q[i20])
            + (-4.727244e-05) * (q[i10] * q[i21]) + (-2.014826e-05) * (q[i10] * q[i22]) + (2.662090e-04) * (q[i11] * q[i12])
            + (-1.718668e-04) * (q[i11] * q[i16]) + (-3.795920e-04) * (q[i11] * q[i19]) + (-3.056488e-05) * (q[i11] * q[i20])
            + (-4.355309e-04) * (q[i11] * q[i21]) + (-7.896743e-05) * (q[i11] * q[i22]) + (-1.650153e-04) * (q[i12] * q[i16])
            + (1.966239e-04) * (q[i12] * q[i19]) + (-1.517018e-04) * (q[i12] * q[i20]) + (3.056551e-04) * (q[i12] * q[i21]) + (3.410488e-04) * (q[i12] * q[i22])
            + (2.180806e-04) * (q[i16] * q[i19]) + (-2.195016e-04) * (q[i16] * q[i20]) + (-4.443231e-05) * (q[i16] * q[i21])
            + (-4.519659e-05) * (q[i16] * q[i22]) + (-4.297183e-05) * (q[i19] * q[i20]) + (8.424679e-05) * (q[i19] * q[i21])
            + (-9.699551e-05) * (q[i19] * q[i22]) + (5.359315e-06) * (q[i20] * q[i21]) + (-5.234313e-06) * (q[i20] * q[i22])
            + (-2.632293e-05) * (q[i21] * q[i22]);
   }

   public void getJQx16(double[] q, double[][] JQ)
   {
      JQ[1][i16] = (2.431806e-03) * (1) + (-5.259577e-03) * ((2) * q[i16]) + (2.434471e-03) * (q[i0]) + (6.808327e-04) * (q[i1]) + (3.282658e-03) * (q[i2])
            + (4.782723e-03) * (q[i3]) + (4.447938e-03) * (q[i4]) + (-1.488733e-02) * (q[i5]) + (-1.464468e-03) * (q[i6]) + (9.646926e-04) * (q[i7])
            + (-8.089714e-03) * (q[i8]) + (4.936475e-04) * (q[i9]) + (4.054355e-04) * (q[i10]) + (7.708882e-05) * (q[i11]) + (2.562287e-03) * (q[i12])
            + (-3.239600e-06) * (q[i15]) + (4.367905e-04) * (q[i19]) + (-9.992207e-04) * (q[i20]) + (-1.113394e-04) * (q[i21]) + (2.038371e-03) * (q[i22])
            + (6.033945e-04) * (q[i0] * q[i0]) + (5.145223e-04) * (q[i1] * q[i1]) + (3.108143e-03) * (q[i2] * q[i2]) + (7.292597e-05) * (q[i3] * q[i3])
            + (-8.986242e-05) * (q[i4] * q[i4]) + (5.689679e-04) * (q[i5] * q[i5]) + (6.497508e-04) * (q[i6] * q[i6]) + (8.193634e-05) * (q[i7] * q[i7])
            + (-3.834838e-03) * (q[i8] * q[i8]) + (1.160582e-04) * (q[i9] * q[i9]) + (-2.034701e-04) * (q[i10] * q[i10]) + (1.845193e-04) * (q[i11] * q[i11])
            + (-1.712646e-03) * (q[i12] * q[i12]) + (-1.811882e-05) * (q[i15] * q[i15]) + (1.020646e-04) * ((2) * q[i0] * q[i16])
            + (1.516463e-04) * ((2) * q[i1] * q[i16]) + (2.009771e-03) * ((2) * q[i2] * q[i16]) + (6.422297e-04) * ((2) * q[i3] * q[i16])
            + (2.455638e-04) * ((2) * q[i4] * q[i16]) + (-2.458229e-03) * ((2) * q[i5] * q[i16]) + (8.264544e-05) * ((2) * q[i6] * q[i16])
            + (1.906856e-05) * ((2) * q[i7] * q[i16]) + (-2.207795e-03) * ((2) * q[i8] * q[i16]) + (-7.777213e-05) * ((2) * q[i9] * q[i16])
            + (-7.392163e-05) * ((2) * q[i10] * q[i16]) + (1.075104e-04) * ((2) * q[i11] * q[i16]) + (2.204910e-03) * ((2) * q[i12] * q[i16])
            + (1.829172e-05) * ((2) * q[i15] * q[i16]) + (-5.379419e-04) * ((3) * q[i16] * q[i16]) + (-3.690873e-05) * ((2) * q[i16] * q[i19])
            + (-1.493813e-04) * ((2) * q[i16] * q[i20]) + (6.428493e-05) * ((2) * q[i16] * q[i21]) + (5.458667e-04) * ((2) * q[i16] * q[i22])
            + (1.441866e-04) * (q[i19] * q[i19]) + (-2.797231e-04) * (q[i20] * q[i20]) + (-3.764340e-05) * (q[i21] * q[i21])
            + (-7.192742e-04) * (q[i22] * q[i22]) + (-1.067377e-03) * (q[i0] * q[i1]) + (2.183712e-03) * (q[i0] * q[i2]) + (6.065313e-04) * (q[i0] * q[i3])
            + (-2.979545e-04) * (q[i0] * q[i4]) + (-1.147948e-03) * (q[i0] * q[i5]) + (1.406846e-03) * (q[i0] * q[i6]) + (1.844087e-04) * (q[i0] * q[i7])
            + (7.457658e-04) * (q[i0] * q[i8]) + (2.365968e-05) * (q[i0] * q[i9]) + (-1.663085e-05) * (q[i0] * q[i10]) + (5.187873e-04) * (q[i0] * q[i11])
            + (-6.241420e-04) * (q[i0] * q[i12]) + (5.167769e-04) * (q[i0] * q[i15]) + (-6.703630e-04) * (q[i0] * q[i19]) + (1.521333e-04) * (q[i0] * q[i20])
            + (2.922396e-05) * (q[i0] * q[i21]) + (7.041708e-05) * (q[i0] * q[i22]) + (1.311596e-03) * (q[i1] * q[i2]) + (-2.264120e-04) * (q[i1] * q[i3])
            + (-2.220515e-03) * (q[i1] * q[i4]) + (6.108258e-04) * (q[i1] * q[i5]) + (-6.800327e-04) * (q[i1] * q[i6]) + (7.824298e-04) * (q[i1] * q[i7])
            + (7.741995e-04) * (q[i1] * q[i8]) + (3.857545e-04) * (q[i1] * q[i9]) + (1.650535e-04) * (q[i1] * q[i10]) + (8.497943e-07) * (q[i1] * q[i11])
            + (-4.937368e-04) * (q[i1] * q[i12]) + (5.077336e-04) * (q[i1] * q[i15]) + (1.494955e-05) * (q[i1] * q[i19]) + (3.872082e-05) * (q[i1] * q[i20])
            + (-3.747243e-04) * (q[i1] * q[i21]) + (-2.484144e-04) * (q[i1] * q[i22]) + (-4.379823e-04) * (q[i2] * q[i3]) + (-1.911338e-03) * (q[i2] * q[i4])
            + (2.307184e-03) * (q[i2] * q[i5]) + (-4.380484e-04) * (q[i2] * q[i6]) + (6.364879e-04) * (q[i2] * q[i7]) + (5.254888e-03) * (q[i2] * q[i8])
            + (-2.390041e-04) * (q[i2] * q[i9]) + (1.254069e-04) * (q[i2] * q[i10]) + (-4.022756e-04) * (q[i2] * q[i11]) + (-3.340560e-03) * (q[i2] * q[i12])
            + (5.847767e-05) * (q[i2] * q[i15]) + (-7.419841e-04) * (q[i2] * q[i19]) + (-9.052824e-04) * (q[i2] * q[i20]) + (-2.212101e-04) * (q[i2] * q[i21])
            + (-1.195472e-04) * (q[i2] * q[i22]) + (3.199603e-05) * (q[i3] * q[i4]) + (3.817097e-04) * (q[i3] * q[i5]) + (-7.632352e-04) * (q[i3] * q[i6])
            + (-9.085843e-04) * (q[i3] * q[i7]) + (-7.923113e-05) * (q[i3] * q[i8]) + (-5.925024e-04) * (q[i3] * q[i9]) + (3.846383e-04) * (q[i3] * q[i10])
            + (7.868216e-04) * (q[i3] * q[i11]) + (-5.822735e-05) * (q[i3] * q[i12]) + (3.354747e-04) * (q[i3] * q[i15]) + (3.829431e-04) * (q[i3] * q[i19])
            + (3.285342e-04) * (q[i3] * q[i20]) + (-8.378908e-05) * (q[i3] * q[i21]) + (-4.900784e-04) * (q[i3] * q[i22]) + (-1.157941e-03) * (q[i4] * q[i5])
            + (-1.607747e-03) * (q[i4] * q[i6]) + (8.235890e-04) * (q[i4] * q[i7]) + (-6.186600e-04) * (q[i4] * q[i8]) + (-3.542908e-04) * (q[i4] * q[i9])
            + (-2.718006e-04) * (q[i4] * q[i10]) + (-5.416789e-04) * (q[i4] * q[i11]) + (1.539207e-03) * (q[i4] * q[i12]) + (3.344007e-04) * (q[i4] * q[i15])
            + (1.239034e-04) * (q[i4] * q[i19]) + (-6.142677e-04) * (q[i4] * q[i20]) + (-2.286239e-04) * (q[i4] * q[i21]) + (-1.533807e-04) * (q[i4] * q[i22])
            + (4.955465e-04) * (q[i5] * q[i6]) + (6.914289e-04) * (q[i5] * q[i7]) + (2.082281e-03) * (q[i5] * q[i8]) + (-2.549192e-04) * (q[i5] * q[i9])
            + (-1.284285e-04) * (q[i5] * q[i10]) + (-6.262812e-04) * (q[i5] * q[i11]) + (-2.053211e-03) * (q[i5] * q[i12]) + (-1.212710e-05) * (q[i5] * q[i15])
            + (-3.121259e-04) * (q[i5] * q[i19]) + (-3.933553e-04) * (q[i5] * q[i20]) + (3.135076e-04) * (q[i5] * q[i21]) + (1.668104e-03) * (q[i5] * q[i22])
            + (-1.037743e-04) * (q[i6] * q[i7]) + (5.991215e-04) * (q[i6] * q[i8]) + (-9.940779e-05) * (q[i6] * q[i9]) + (-1.307033e-05) * (q[i6] * q[i10])
            + (-6.698623e-05) * (q[i6] * q[i11]) + (6.449914e-04) * (q[i6] * q[i12]) + (-3.029794e-04) * (q[i6] * q[i15]) + (-4.558552e-04) * (q[i6] * q[i19])
            + (-1.235554e-04) * (q[i6] * q[i20]) + (-4.327594e-04) * (q[i6] * q[i21]) + (3.514698e-04) * (q[i6] * q[i22]) + (2.638945e-04) * (q[i7] * q[i8])
            + (3.950348e-04) * (q[i7] * q[i9]) + (1.002896e-04) * (q[i7] * q[i10]) + (6.519627e-04) * (q[i7] * q[i11]) + (1.026269e-03) * (q[i7] * q[i12])
            + (3.025784e-04) * (q[i7] * q[i15]) + (5.480453e-05) * (q[i7] * q[i19]) + (-3.419594e-04) * (q[i7] * q[i20]) + (1.779659e-04) * (q[i7] * q[i21])
            + (-2.200355e-04) * (q[i7] * q[i22]) + (2.623590e-04) * (q[i8] * q[i9]) + (5.953452e-04) * (q[i8] * q[i10]) + (3.779367e-04) * (q[i8] * q[i11])
            + (3.393794e-03) * (q[i8] * q[i12]) + (1.146822e-06) * (q[i8] * q[i15]) + (-7.327515e-05) * (q[i8] * q[i19]) + (-3.833072e-04) * (q[i8] * q[i20])
            + (3.885455e-05) * (q[i8] * q[i21]) + (2.265615e-04) * (q[i8] * q[i22]) + (-1.266824e-04) * (q[i9] * q[i10]) + (5.645389e-05) * (q[i9] * q[i11])
            + (-2.587763e-05) * (q[i9] * q[i12]) + (6.629910e-05) * (q[i9] * q[i15]) + (-1.553824e-04) * (q[i9] * q[i19]) + (-3.935135e-05) * (q[i9] * q[i20])
            + (-2.204518e-05) * (q[i9] * q[i21]) + (-5.194976e-05) * (q[i9] * q[i22]) + (-1.444610e-04) * (q[i10] * q[i11])
            + (-2.329611e-04) * (q[i10] * q[i12]) + (-6.673015e-05) * (q[i10] * q[i15]) + (-1.741995e-04) * (q[i10] * q[i19])
            + (-1.220272e-04) * (q[i10] * q[i20]) + (-3.285843e-05) * (q[i10] * q[i21]) + (8.839866e-05) * (q[i10] * q[i22])
            + (-2.646526e-04) * (q[i11] * q[i12]) + (-1.718668e-04) * (q[i11] * q[i15]) + (-1.546858e-04) * (q[i11] * q[i19])
            + (1.975458e-04) * (q[i11] * q[i20]) + (-3.401647e-04) * (q[i11] * q[i21]) + (-3.060890e-04) * (q[i11] * q[i22])
            + (-1.650153e-04) * (q[i12] * q[i15]) + (-2.977686e-05) * (q[i12] * q[i19]) + (-3.921195e-04) * (q[i12] * q[i20])
            + (7.957061e-05) * (q[i12] * q[i21]) + (4.378143e-04) * (q[i12] * q[i22]) + (2.180806e-04) * (q[i15] * q[i19]) + (-2.195016e-04) * (q[i15] * q[i20])
            + (-4.443231e-05) * (q[i15] * q[i21]) + (-4.519659e-05) * (q[i15] * q[i22]) + (4.326723e-05) * (q[i19] * q[i20])
            + (-3.497949e-06) * (q[i19] * q[i21]) + (8.001428e-06) * (q[i19] * q[i22]) + (-9.522871e-05) * (q[i20] * q[i21])
            + (9.050152e-05) * (q[i20] * q[i22]) + (2.710906e-05) * (q[i21] * q[i22]);
   }

   public void getJQx19(double[] q, double[][] JQ)
   {
      JQ[1][i19] = (-3.287858e-04) * (1) + (8.486198e-04) * ((2) * q[i19]) + (-1.256073e-03) * (q[i0]) + (4.686219e-04) * (q[i1]) + (-1.830097e-03) * (q[i2])
            + (-7.273987e-04) * (q[i3]) + (1.189330e-03) * (q[i4]) + (-2.978448e-04) * (q[i5]) + (9.621823e-04) * (q[i6]) + (-2.502844e-04) * (q[i7])
            + (6.607573e-04) * (q[i8]) + (2.404055e-04) * (q[i9]) + (-4.933374e-04) * (q[i10]) + (-1.381336e-03) * (q[i11]) + (9.009046e-04) * (q[i12])
            + (1.028355e-03) * (q[i15]) + (4.367905e-04) * (q[i16]) + (-4.093346e-06) * (q[i20]) + (-7.656317e-04) * (q[i21]) + (3.017889e-05) * (q[i22])
            + (3.062619e-04) * (q[i0] * q[i0]) + (8.886021e-04) * (q[i1] * q[i1]) + (-2.805107e-04) * (q[i2] * q[i2]) + (-4.504642e-04) * (q[i3] * q[i3])
            + (-3.920504e-04) * (q[i4] * q[i4]) + (6.193059e-04) * (q[i5] * q[i5]) + (4.450975e-05) * (q[i6] * q[i6]) + (-1.559224e-04) * (q[i7] * q[i7])
            + (2.476959e-06) * (q[i8] * q[i8]) + (-1.989461e-04) * (q[i9] * q[i9]) + (-1.036331e-04) * (q[i10] * q[i10]) + (5.351644e-04) * (q[i11] * q[i11])
            + (1.989071e-04) * (q[i12] * q[i12]) + (1.538827e-04) * (q[i15] * q[i15]) + (-3.690873e-05) * (q[i16] * q[i16])
            + (-1.449182e-04) * ((2) * q[i0] * q[i19]) + (5.154156e-05) * ((2) * q[i1] * q[i19]) + (-1.105789e-04) * ((2) * q[i2] * q[i19])
            + (-3.432702e-05) * ((2) * q[i3] * q[i19]) + (3.278650e-05) * ((2) * q[i4] * q[i19]) + (5.381257e-06) * ((2) * q[i5] * q[i19])
            + (-1.282560e-05) * ((2) * q[i6] * q[i19]) + (1.689597e-04) * ((2) * q[i7] * q[i19]) + (3.474673e-04) * ((2) * q[i8] * q[i19])
            + (1.758296e-05) * ((2) * q[i9] * q[i19]) + (-3.597388e-05) * ((2) * q[i10] * q[i19]) + (1.771287e-05) * ((2) * q[i11] * q[i19])
            + (-1.250546e-05) * ((2) * q[i12] * q[i19]) + (2.848873e-04) * ((2) * q[i15] * q[i19]) + (1.441866e-04) * ((2) * q[i16] * q[i19])
            + (1.591417e-04) * ((3) * q[i19] * q[i19]) + (-5.491691e-05) * ((2) * q[i19] * q[i20]) + (6.623013e-04) * ((2) * q[i19] * q[i21])
            + (-8.550832e-05) * ((2) * q[i19] * q[i22]) + (5.428668e-05) * (q[i20] * q[i20]) + (-1.431363e-04) * (q[i21] * q[i21])
            + (3.088854e-05) * (q[i22] * q[i22]) + (-3.577786e-04) * (q[i0] * q[i1]) + (5.679102e-04) * (q[i0] * q[i2]) + (9.987913e-04) * (q[i0] * q[i3])
            + (-9.249756e-05) * (q[i0] * q[i4]) + (-6.164790e-04) * (q[i0] * q[i5]) + (6.747908e-04) * (q[i0] * q[i6]) + (-2.213321e-04) * (q[i0] * q[i7])
            + (6.306952e-04) * (q[i0] * q[i8]) + (6.612424e-04) * (q[i0] * q[i9]) + (-1.775852e-04) * (q[i0] * q[i10]) + (5.942319e-04) * (q[i0] * q[i11])
            + (-1.363830e-04) * (q[i0] * q[i12]) + (3.401895e-05) * (q[i0] * q[i15]) + (-6.703630e-04) * (q[i0] * q[i16]) + (-3.255676e-05) * (q[i0] * q[i20])
            + (-2.073554e-04) * (q[i0] * q[i21]) + (-1.515405e-04) * (q[i0] * q[i22]) + (-9.215505e-04) * (q[i1] * q[i2]) + (2.977698e-04) * (q[i1] * q[i3])
            + (-3.527346e-04) * (q[i1] * q[i4]) + (3.734309e-04) * (q[i1] * q[i5]) + (-1.865585e-04) * (q[i1] * q[i6]) + (1.553686e-04) * (q[i1] * q[i7])
            + (5.053310e-04) * (q[i1] * q[i8]) + (-4.597970e-04) * (q[i1] * q[i9]) + (2.768153e-04) * (q[i1] * q[i10]) + (3.752991e-04) * (q[i1] * q[i11])
            + (5.602229e-05) * (q[i1] * q[i12]) + (1.600298e-04) * (q[i1] * q[i15]) + (1.494955e-05) * (q[i1] * q[i16]) + (-4.926675e-05) * (q[i1] * q[i20])
            + (-5.847086e-05) * (q[i1] * q[i21]) + (-5.140003e-04) * (q[i1] * q[i22]) + (-6.730639e-04) * (q[i2] * q[i3]) + (1.579609e-04) * (q[i2] * q[i4])
            + (1.426304e-03) * (q[i2] * q[i5]) + (1.961629e-04) * (q[i2] * q[i6]) + (3.483468e-05) * (q[i2] * q[i7]) + (4.322037e-04) * (q[i2] * q[i8])
            + (1.394536e-04) * (q[i2] * q[i9]) + (2.362313e-04) * (q[i2] * q[i10]) + (1.742883e-03) * (q[i2] * q[i11]) + (-7.327170e-05) * (q[i2] * q[i12])
            + (-9.102059e-04) * (q[i2] * q[i15]) + (-7.419841e-04) * (q[i2] * q[i16]) + (1.839152e-04) * (q[i2] * q[i20]) + (-6.358527e-04) * (q[i2] * q[i21])
            + (-3.780284e-04) * (q[i2] * q[i22]) + (-1.143179e-03) * (q[i3] * q[i4]) + (6.850017e-06) * (q[i3] * q[i5]) + (-1.925811e-03) * (q[i3] * q[i6])
            + (-1.873487e-04) * (q[i3] * q[i7]) + (-4.347447e-05) * (q[i3] * q[i8]) + (-3.112606e-04) * (q[i3] * q[i9]) + (2.522698e-04) * (q[i3] * q[i10])
            + (4.310239e-05) * (q[i3] * q[i11]) + (2.744032e-05) * (q[i3] * q[i12]) + (-6.221902e-04) * (q[i3] * q[i15]) + (3.829431e-04) * (q[i3] * q[i16])
            + (4.188005e-04) * (q[i3] * q[i20]) + (5.394042e-04) * (q[i3] * q[i21]) + (-6.136038e-04) * (q[i3] * q[i22]) + (1.691194e-03) * (q[i4] * q[i5])
            + (4.110395e-04) * (q[i4] * q[i6]) + (4.637157e-04) * (q[i4] * q[i7]) + (-7.238836e-05) * (q[i4] * q[i8]) + (2.444100e-05) * (q[i4] * q[i9])
            + (-1.542513e-04) * (q[i4] * q[i10]) + (-4.461548e-04) * (q[i4] * q[i11]) + (-3.089071e-05) * (q[i4] * q[i12]) + (3.299048e-04) * (q[i4] * q[i15])
            + (1.239034e-04) * (q[i4] * q[i16]) + (4.128263e-04) * (q[i4] * q[i20]) + (3.958918e-04) * (q[i4] * q[i21]) + (3.210236e-04) * (q[i4] * q[i22])
            + (-2.268588e-04) * (q[i5] * q[i6]) + (7.971804e-04) * (q[i5] * q[i7]) + (-4.989687e-05) * (q[i5] * q[i8]) + (3.467943e-04) * (q[i5] * q[i9])
            + (-2.752782e-04) * (q[i5] * q[i10]) + (1.178573e-03) * (q[i5] * q[i11]) + (4.292040e-04) * (q[i5] * q[i12]) + (-3.948275e-04) * (q[i5] * q[i15])
            + (-3.121259e-04) * (q[i5] * q[i16]) + (-1.699453e-04) * (q[i5] * q[i20]) + (-1.532837e-03) * (q[i5] * q[i21]) + (3.425438e-04) * (q[i5] * q[i22])
            + (1.300454e-04) * (q[i6] * q[i7]) + (-1.606921e-04) * (q[i6] * q[i8]) + (-4.476959e-04) * (q[i6] * q[i9]) + (7.761660e-05) * (q[i6] * q[i10])
            + (-7.845989e-05) * (q[i6] * q[i11]) + (-1.415264e-04) * (q[i6] * q[i12]) + (3.436680e-04) * (q[i6] * q[i15]) + (-4.558552e-04) * (q[i6] * q[i16])
            + (-9.645676e-06) * (q[i6] * q[i20]) + (-3.073957e-04) * (q[i6] * q[i21]) + (-7.130947e-05) * (q[i6] * q[i22]) + (-2.536953e-04) * (q[i7] * q[i8])
            + (3.069477e-04) * (q[i7] * q[i9]) + (-1.083990e-04) * (q[i7] * q[i10]) + (2.386295e-04) * (q[i7] * q[i11]) + (4.347151e-04) * (q[i7] * q[i12])
            + (1.200177e-04) * (q[i7] * q[i15]) + (5.480453e-05) * (q[i7] * q[i16]) + (6.585923e-06) * (q[i7] * q[i20]) + (-2.169687e-04) * (q[i7] * q[i21])
            + (2.842097e-04) * (q[i7] * q[i22]) + (-3.346973e-04) * (q[i8] * q[i9]) + (-1.589789e-04) * (q[i8] * q[i10]) + (2.604282e-04) * (q[i8] * q[i11])
            + (-9.972883e-05) * (q[i8] * q[i12]) + (4.156413e-04) * (q[i8] * q[i15]) + (-7.327515e-05) * (q[i8] * q[i16]) + (4.605962e-08) * (q[i8] * q[i20])
            + (-1.010121e-03) * (q[i8] * q[i21]) + (-1.827928e-07) * (q[i8] * q[i22]) + (1.701359e-04) * (q[i9] * q[i10]) + (4.054558e-05) * (q[i9] * q[i11])
            + (-5.674719e-05) * (q[i9] * q[i12]) + (1.229711e-04) * (q[i9] * q[i15]) + (-1.553824e-04) * (q[i9] * q[i16]) + (2.941614e-04) * (q[i9] * q[i20])
            + (-6.484038e-05) * (q[i9] * q[i21]) + (4.005138e-05) * (q[i9] * q[i22]) + (1.695191e-04) * (q[i10] * q[i11]) + (3.057630e-05) * (q[i10] * q[i12])
            + (3.761990e-05) * (q[i10] * q[i15]) + (-1.741995e-04) * (q[i10] * q[i16]) + (-2.943001e-04) * (q[i10] * q[i20])
            + (-2.292217e-04) * (q[i10] * q[i21]) + (8.241691e-05) * (q[i10] * q[i22]) + (3.777457e-04) * (q[i11] * q[i12])
            + (-3.795920e-04) * (q[i11] * q[i15]) + (-1.546858e-04) * (q[i11] * q[i16]) + (-1.446436e-04) * (q[i11] * q[i20])
            + (-1.896972e-03) * (q[i11] * q[i21]) + (1.041701e-04) * (q[i11] * q[i22]) + (1.966239e-04) * (q[i12] * q[i15])
            + (-2.977686e-05) * (q[i12] * q[i16]) + (-1.457396e-04) * (q[i12] * q[i20]) + (2.692456e-04) * (q[i12] * q[i21])
            + (-2.546263e-05) * (q[i12] * q[i22]) + (2.180806e-04) * (q[i15] * q[i16]) + (-4.297183e-05) * (q[i15] * q[i20])
            + (8.424679e-05) * (q[i15] * q[i21]) + (-9.699551e-05) * (q[i15] * q[i22]) + (4.326723e-05) * (q[i16] * q[i20])
            + (-3.497949e-06) * (q[i16] * q[i21]) + (8.001428e-06) * (q[i16] * q[i22]) + (1.342922e-04) * (q[i20] * q[i21]) + (1.338215e-04) * (q[i20] * q[i22])
            + (1.658186e-05) * (q[i21] * q[i22]);
   }

   public void getJQx20(double[] q, double[][] JQ)
   {
      JQ[1][i20] = (2.730810e-04) * (1) + (-8.468018e-04) * ((2) * q[i20]) + (4.558196e-04) * (q[i0]) + (-1.242329e-03) * (q[i1]) + (-1.825938e-03) * (q[i2])
            + (1.193762e-03) * (q[i3]) + (-7.088497e-04) * (q[i4]) + (-3.149214e-04) * (q[i5]) + (2.620063e-04) * (q[i6]) + (-9.387683e-04) * (q[i7])
            + (-6.266187e-04) * (q[i8]) + (4.906908e-04) * (q[i9]) + (-2.292215e-04) * (q[i10]) + (8.973351e-04) * (q[i11]) + (-1.403589e-03) * (q[i12])
            + (-4.449288e-04) * (q[i15]) + (-9.992207e-04) * (q[i16]) + (-4.093346e-06) * (q[i19]) + (2.101496e-05) * (q[i21]) + (-7.428983e-04) * (q[i22])
            + (-9.033055e-04) * (q[i0] * q[i0]) + (-2.973511e-04) * (q[i1] * q[i1]) + (2.853590e-04) * (q[i2] * q[i2]) + (4.050251e-04) * (q[i3] * q[i3])
            + (4.323403e-04) * (q[i4] * q[i4]) + (-6.123879e-04) * (q[i5] * q[i5]) + (1.616839e-04) * (q[i6] * q[i6]) + (-5.194834e-05) * (q[i7] * q[i7])
            + (6.359908e-07) * (q[i8] * q[i8]) + (1.046171e-04) * (q[i9] * q[i9]) + (1.934299e-04) * (q[i10] * q[i10]) + (-2.024991e-04) * (q[i11] * q[i11])
            + (-5.191837e-04) * (q[i12] * q[i12]) + (3.743298e-05) * (q[i15] * q[i15]) + (-1.493813e-04) * (q[i16] * q[i16])
            + (-5.491691e-05) * (q[i19] * q[i19]) + (5.001423e-05) * ((2) * q[i0] * q[i20]) + (-1.397175e-04) * ((2) * q[i1] * q[i20])
            + (-1.112106e-04) * ((2) * q[i2] * q[i20]) + (2.891431e-05) * ((2) * q[i3] * q[i20]) + (-3.742291e-05) * ((2) * q[i4] * q[i20])
            + (1.593998e-05) * ((2) * q[i5] * q[i20]) + (-1.731079e-04) * ((2) * q[i6] * q[i20]) + (1.524099e-05) * ((2) * q[i7] * q[i20])
            + (-3.542810e-04) * ((2) * q[i8] * q[i20]) + (3.629395e-05) * ((2) * q[i9] * q[i20]) + (-1.600149e-05) * ((2) * q[i10] * q[i20])
            + (-1.225013e-05) * ((2) * q[i11] * q[i20]) + (3.205542e-05) * ((2) * q[i12] * q[i20]) + (-1.412690e-04) * ((2) * q[i15] * q[i20])
            + (-2.797231e-04) * ((2) * q[i16] * q[i20]) + (5.428668e-05) * ((2) * q[i19] * q[i20]) + (-1.525967e-04) * ((3) * q[i20] * q[i20])
            + (-8.564171e-05) * ((2) * q[i20] * q[i21]) + (6.618400e-04) * ((2) * q[i20] * q[i22]) + (-2.911553e-05) * (q[i21] * q[i21])
            + (1.418221e-04) * (q[i22] * q[i22]) + (3.680753e-04) * (q[i0] * q[i1]) + (9.169973e-04) * (q[i0] * q[i2]) + (3.553240e-04) * (q[i0] * q[i3])
            + (-3.000105e-04) * (q[i0] * q[i4]) + (-3.585268e-04) * (q[i0] * q[i5]) + (1.608036e-04) * (q[i0] * q[i6]) + (-1.875617e-04) * (q[i0] * q[i7])
            + (5.005176e-04) * (q[i0] * q[i8]) + (2.772090e-04) * (q[i0] * q[i9]) + (-4.583215e-04) * (q[i0] * q[i10]) + (-5.535935e-05) * (q[i0] * q[i11])
            + (-3.798038e-04) * (q[i0] * q[i12]) + (8.032369e-06) * (q[i0] * q[i15]) + (1.521333e-04) * (q[i0] * q[i16]) + (-3.255676e-05) * (q[i0] * q[i19])
            + (5.198957e-04) * (q[i0] * q[i21]) + (5.406636e-05) * (q[i0] * q[i22]) + (-5.653053e-04) * (q[i1] * q[i2]) + (8.919473e-05) * (q[i1] * q[i3])
            + (-9.849510e-04) * (q[i1] * q[i4]) + (5.978117e-04) * (q[i1] * q[i5]) + (-2.142370e-04) * (q[i1] * q[i6]) + (6.718480e-04) * (q[i1] * q[i7])
            + (6.299696e-04) * (q[i1] * q[i8]) + (-1.791769e-04) * (q[i1] * q[i9]) + (6.625426e-04) * (q[i1] * q[i10]) + (1.283739e-04) * (q[i1] * q[i11])
            + (-5.984771e-04) * (q[i1] * q[i12]) + (-6.659789e-04) * (q[i1] * q[i15]) + (3.872082e-05) * (q[i1] * q[i16]) + (-4.926675e-05) * (q[i1] * q[i19])
            + (1.523064e-04) * (q[i1] * q[i21]) + (2.056473e-04) * (q[i1] * q[i22]) + (-1.564297e-04) * (q[i2] * q[i3]) + (6.575876e-04) * (q[i2] * q[i4])
            + (-1.426429e-03) * (q[i2] * q[i5]) + (3.576040e-05) * (q[i2] * q[i6]) + (2.011984e-04) * (q[i2] * q[i7]) + (4.214895e-04) * (q[i2] * q[i8])
            + (2.306254e-04) * (q[i2] * q[i9]) + (1.414497e-04) * (q[i2] * q[i10]) + (5.947629e-05) * (q[i2] * q[i11]) + (-1.726330e-03) * (q[i2] * q[i12])
            + (-7.358046e-04) * (q[i2] * q[i15]) + (-9.052824e-04) * (q[i2] * q[i16]) + (1.839152e-04) * (q[i2] * q[i19]) + (3.801782e-04) * (q[i2] * q[i21])
            + (6.258241e-04) * (q[i2] * q[i22]) + (1.148663e-03) * (q[i3] * q[i4]) + (-1.682617e-03) * (q[i3] * q[i5]) + (4.617432e-04) * (q[i3] * q[i6])
            + (4.093100e-04) * (q[i3] * q[i7]) + (-6.697361e-05) * (q[i3] * q[i8]) + (-1.554147e-04) * (q[i3] * q[i9]) + (3.036936e-05) * (q[i3] * q[i10])
            + (3.062487e-05) * (q[i3] * q[i11]) + (4.514248e-04) * (q[i3] * q[i12]) + (1.336437e-04) * (q[i3] * q[i15]) + (3.285342e-04) * (q[i3] * q[i16])
            + (4.188005e-04) * (q[i3] * q[i19]) + (-3.209858e-04) * (q[i3] * q[i21]) + (-3.933443e-04) * (q[i3] * q[i22]) + (-1.832415e-06) * (q[i4] * q[i5])
            + (-1.805649e-04) * (q[i4] * q[i6]) + (-1.908558e-03) * (q[i4] * q[i7]) + (-3.067289e-05) * (q[i4] * q[i8]) + (2.528863e-04) * (q[i4] * q[i9])
            + (-3.165382e-04) * (q[i4] * q[i10]) + (-2.933765e-05) * (q[i4] * q[i11]) + (-5.005504e-05) * (q[i4] * q[i12]) + (3.810390e-04) * (q[i4] * q[i15])
            + (-6.142677e-04) * (q[i4] * q[i16]) + (4.128263e-04) * (q[i4] * q[i19]) + (6.100004e-04) * (q[i4] * q[i21]) + (-5.392119e-04) * (q[i4] * q[i22])
            + (7.898259e-04) * (q[i5] * q[i6]) + (-2.328081e-04) * (q[i5] * q[i7]) + (-6.700013e-05) * (q[i5] * q[i8]) + (-2.783076e-04) * (q[i5] * q[i9])
            + (3.378551e-04) * (q[i5] * q[i10]) + (-4.248532e-04) * (q[i5] * q[i11]) + (-1.178317e-03) * (q[i5] * q[i12]) + (-3.213971e-04) * (q[i5] * q[i15])
            + (-3.933553e-04) * (q[i5] * q[i16]) + (-1.699453e-04) * (q[i5] * q[i19]) + (-3.402388e-04) * (q[i5] * q[i21]) + (1.535302e-03) * (q[i5] * q[i22])
            + (-1.368505e-04) * (q[i6] * q[i7]) + (2.586692e-04) * (q[i6] * q[i8]) + (1.079777e-04) * (q[i6] * q[i9]) + (-3.088570e-04) * (q[i6] * q[i10])
            + (4.371767e-04) * (q[i6] * q[i11]) + (2.387709e-04) * (q[i6] * q[i12]) + (-5.554971e-05) * (q[i6] * q[i15]) + (-1.235554e-04) * (q[i6] * q[i16])
            + (-9.645676e-06) * (q[i6] * q[i19]) + (2.802752e-04) * (q[i6] * q[i21]) + (-2.261390e-04) * (q[i6] * q[i22]) + (1.514588e-04) * (q[i7] * q[i8])
            + (-7.608729e-05) * (q[i7] * q[i9]) + (4.362536e-04) * (q[i7] * q[i10]) + (-1.468150e-04) * (q[i7] * q[i11]) + (-7.737333e-05) * (q[i7] * q[i12])
            + (4.572318e-04) * (q[i7] * q[i15]) + (-3.419594e-04) * (q[i7] * q[i16]) + (6.585923e-06) * (q[i7] * q[i19]) + (-7.294358e-05) * (q[i7] * q[i21])
            + (-3.075305e-04) * (q[i7] * q[i22]) + (1.654978e-04) * (q[i8] * q[i9]) + (3.364738e-04) * (q[i8] * q[i10]) + (-1.029660e-04) * (q[i8] * q[i11])
            + (2.450668e-04) * (q[i8] * q[i12]) + (7.485298e-05) * (q[i8] * q[i15]) + (-3.833072e-04) * (q[i8] * q[i16]) + (4.605962e-08) * (q[i8] * q[i19])
            + (6.841617e-06) * (q[i8] * q[i21]) + (-9.942643e-04) * (q[i8] * q[i22]) + (-1.680864e-04) * (q[i9] * q[i10]) + (3.056139e-05) * (q[i9] * q[i11])
            + (1.614984e-04) * (q[i9] * q[i12]) + (1.753397e-04) * (q[i9] * q[i15]) + (-3.935135e-05) * (q[i9] * q[i16]) + (2.941614e-04) * (q[i9] * q[i19])
            + (8.318399e-05) * (q[i9] * q[i21]) + (-2.331863e-04) * (q[i9] * q[i22]) + (-5.633695e-05) * (q[i10] * q[i11]) + (4.205834e-05) * (q[i10] * q[i12])
            + (1.563051e-04) * (q[i10] * q[i15]) + (-1.220272e-04) * (q[i10] * q[i16]) + (-2.943001e-04) * (q[i10] * q[i19])
            + (4.174105e-05) * (q[i10] * q[i21]) + (-6.584737e-05) * (q[i10] * q[i22]) + (-3.773006e-04) * (q[i11] * q[i12])
            + (-3.056488e-05) * (q[i11] * q[i15]) + (1.975458e-04) * (q[i11] * q[i16]) + (-1.446436e-04) * (q[i11] * q[i19])
            + (2.664331e-05) * (q[i11] * q[i21]) + (-2.688682e-04) * (q[i11] * q[i22]) + (-1.517018e-04) * (q[i12] * q[i15])
            + (-3.921195e-04) * (q[i12] * q[i16]) + (-1.457396e-04) * (q[i12] * q[i19]) + (-1.057655e-04) * (q[i12] * q[i21])
            + (1.885072e-03) * (q[i12] * q[i22]) + (-2.195016e-04) * (q[i15] * q[i16]) + (-4.297183e-05) * (q[i15] * q[i19])
            + (5.359315e-06) * (q[i15] * q[i21]) + (-5.234313e-06) * (q[i15] * q[i22]) + (4.326723e-05) * (q[i16] * q[i19])
            + (-9.522871e-05) * (q[i16] * q[i21]) + (9.050152e-05) * (q[i16] * q[i22]) + (1.342922e-04) * (q[i19] * q[i21]) + (1.338215e-04) * (q[i19] * q[i22])
            + (-1.409546e-05) * (q[i21] * q[i22]);
   }

   public void getJQx21(double[] q, double[][] JQ)
   {
      JQ[1][i21] = (-3.412418e-03) * (1) + (1.953283e-03) * ((2) * q[i21]) + (1.785382e-03) * (q[i0]) + (-7.433376e-04) * (q[i1]) + (8.509005e-04) * (q[i2])
            + (-6.352294e-04) * (q[i3]) + (-2.964353e-04) * (q[i4]) + (-5.199303e-04) * (q[i5]) + (-1.346279e-03) * (q[i6]) + (-4.092206e-04) * (q[i7])
            + (2.642570e-03) * (q[i8]) + (3.530960e-04) * (q[i9]) + (-7.698729e-05) * (q[i10]) + (2.252492e-04) * (q[i11]) + (2.860958e-04) * (q[i12])
            + (2.043079e-03) * (q[i15]) + (-1.113394e-04) * (q[i16]) + (-7.656317e-04) * (q[i19]) + (2.101496e-05) * (q[i20]) + (-5.025617e-06) * (q[i22])
            + (5.415450e-04) * (q[i0] * q[i0]) + (-5.628567e-04) * (q[i1] * q[i1]) + (4.678450e-04) * (q[i2] * q[i2]) + (-5.546236e-05) * (q[i3] * q[i3])
            + (2.000950e-04) * (q[i4] * q[i4]) + (-2.430805e-04) * (q[i5] * q[i5]) + (-3.261228e-06) * (q[i6] * q[i6]) + (-2.092241e-05) * (q[i7] * q[i7])
            + (-1.303810e-04) * (q[i8] * q[i8]) + (-5.645742e-05) * (q[i9] * q[i9]) + (-9.143866e-06) * (q[i10] * q[i10]) + (7.965434e-04) * (q[i11] * q[i11])
            + (-9.978708e-05) * (q[i12] * q[i12]) + (5.407792e-04) * (q[i15] * q[i15]) + (6.428493e-05) * (q[i16] * q[i16]) + (6.623013e-04) * (q[i19] * q[i19])
            + (-8.564171e-05) * (q[i20] * q[i20]) + (6.503951e-05) * ((2) * q[i0] * q[i21]) + (4.499750e-06) * ((2) * q[i1] * q[i21])
            + (1.895148e-04) * ((2) * q[i2] * q[i21]) + (4.343329e-05) * ((2) * q[i3] * q[i21]) + (5.999856e-04) * ((2) * q[i4] * q[i21])
            + (-1.538097e-03) * ((2) * q[i5] * q[i21]) + (-5.732377e-05) * ((2) * q[i6] * q[i21]) + (-3.043743e-04) * ((2) * q[i7] * q[i21])
            + (7.875672e-04) * ((2) * q[i8] * q[i21]) + (6.271647e-05) * ((2) * q[i9] * q[i21]) + (-1.086686e-04) * ((2) * q[i10] * q[i21])
            + (3.680343e-04) * ((2) * q[i11] * q[i21]) + (5.457835e-05) * ((2) * q[i12] * q[i21]) + (7.180441e-04) * ((2) * q[i15] * q[i21])
            + (-3.764340e-05) * ((2) * q[i16] * q[i21]) + (-1.431363e-04) * ((2) * q[i19] * q[i21]) + (-2.911553e-05) * ((2) * q[i20] * q[i21])
            + (3.875186e-04) * ((3) * q[i21] * q[i21]) + (5.605760e-05) * ((2) * q[i21] * q[i22]) + (5.699931e-05) * (q[i22] * q[i22])
            + (6.375455e-04) * (q[i0] * q[i1]) + (4.389959e-04) * (q[i0] * q[i2]) + (-3.321404e-04) * (q[i0] * q[i3]) + (1.880298e-05) * (q[i0] * q[i4])
            + (2.820474e-04) * (q[i0] * q[i5]) + (-7.451047e-04) * (q[i0] * q[i6]) + (1.814035e-05) * (q[i0] * q[i7]) + (4.348939e-04) * (q[i0] * q[i8])
            + (-3.066195e-04) * (q[i0] * q[i9]) + (-5.004782e-04) * (q[i0] * q[i10]) + (4.090679e-04) * (q[i0] * q[i11]) + (1.552164e-04) * (q[i0] * q[i12])
            + (2.456465e-04) * (q[i0] * q[i15]) + (2.922396e-05) * (q[i0] * q[i16]) + (-2.073554e-04) * (q[i0] * q[i19]) + (5.198957e-04) * (q[i0] * q[i20])
            + (8.145290e-05) * (q[i0] * q[i22]) + (1.822513e-04) * (q[i1] * q[i2]) + (-2.511312e-05) * (q[i1] * q[i3]) + (-8.889815e-04) * (q[i1] * q[i4])
            + (4.016762e-04) * (q[i1] * q[i5]) + (3.399222e-04) * (q[i1] * q[i6]) + (-8.756166e-04) * (q[i1] * q[i7]) + (1.145841e-04) * (q[i1] * q[i8])
            + (7.667513e-05) * (q[i1] * q[i9]) + (1.233639e-04) * (q[i1] * q[i10]) + (-3.525242e-04) * (q[i1] * q[i11]) + (-2.965218e-04) * (q[i1] * q[i12])
            + (-7.249146e-05) * (q[i1] * q[i15]) + (-3.747243e-04) * (q[i1] * q[i16]) + (-5.847086e-05) * (q[i1] * q[i19]) + (1.523064e-04) * (q[i1] * q[i20])
            + (8.272157e-05) * (q[i1] * q[i22]) + (-8.784506e-04) * (q[i2] * q[i3]) + (5.731572e-04) * (q[i2] * q[i4]) + (8.077700e-04) * (q[i2] * q[i5])
            + (3.222426e-04) * (q[i2] * q[i6]) + (-2.421668e-04) * (q[i2] * q[i7]) + (-3.998145e-04) * (q[i2] * q[i8]) + (-2.424145e-04) * (q[i2] * q[i9])
            + (-7.869362e-05) * (q[i2] * q[i10]) + (2.158425e-04) * (q[i2] * q[i11]) + (-4.512358e-04) * (q[i2] * q[i12]) + (1.149993e-04) * (q[i2] * q[i15])
            + (-2.212101e-04) * (q[i2] * q[i16]) + (-6.358527e-04) * (q[i2] * q[i19]) + (3.801782e-04) * (q[i2] * q[i20]) + (8.527480e-05) * (q[i2] * q[i22])
            + (-5.856657e-04) * (q[i3] * q[i4]) + (9.284001e-05) * (q[i3] * q[i5]) + (-3.234240e-04) * (q[i3] * q[i6]) + (6.429751e-04) * (q[i3] * q[i7])
            + (6.391445e-04) * (q[i3] * q[i8]) + (1.320513e-04) * (q[i3] * q[i9]) + (-1.085302e-04) * (q[i3] * q[i10]) + (-4.160914e-04) * (q[i3] * q[i11])
            + (1.865373e-04) * (q[i3] * q[i12]) + (1.475200e-04) * (q[i3] * q[i15]) + (-8.378908e-05) * (q[i3] * q[i16]) + (5.394042e-04) * (q[i3] * q[i19])
            + (-3.209858e-04) * (q[i3] * q[i20]) + (8.998727e-05) * (q[i3] * q[i22]) + (2.917031e-04) * (q[i4] * q[i5]) + (-3.987744e-04) * (q[i4] * q[i6])
            + (-9.108623e-04) * (q[i4] * q[i7]) + (4.369758e-04) * (q[i4] * q[i8]) + (-1.937380e-04) * (q[i4] * q[i9]) + (3.152262e-04) * (q[i4] * q[i10])
            + (-1.990022e-04) * (q[i4] * q[i11]) + (-2.380424e-04) * (q[i4] * q[i12]) + (4.816924e-04) * (q[i4] * q[i15]) + (-2.286239e-04) * (q[i4] * q[i16])
            + (3.958918e-04) * (q[i4] * q[i19]) + (6.100004e-04) * (q[i4] * q[i20]) + (8.042876e-05) * (q[i4] * q[i22]) + (1.469943e-04) * (q[i5] * q[i6])
            + (1.016375e-04) * (q[i5] * q[i7]) + (3.568588e-04) * (q[i5] * q[i8]) + (3.350566e-05) * (q[i5] * q[i9]) + (-5.018386e-05) * (q[i5] * q[i10])
            + (1.428402e-04) * (q[i5] * q[i11]) + (-6.021615e-04) * (q[i5] * q[i12]) + (-1.647586e-03) * (q[i5] * q[i15]) + (3.135076e-04) * (q[i5] * q[i16])
            + (-1.532837e-03) * (q[i5] * q[i19]) + (-3.402388e-04) * (q[i5] * q[i20]) + (-4.511650e-04) * (q[i5] * q[i22]) + (-8.415065e-04) * (q[i6] * q[i7])
            + (2.789672e-04) * (q[i6] * q[i8]) + (-1.558414e-04) * (q[i6] * q[i9]) + (4.445187e-04) * (q[i6] * q[i10]) + (3.351916e-04) * (q[i6] * q[i11])
            + (-2.481534e-04) * (q[i6] * q[i12]) + (-2.115197e-04) * (q[i6] * q[i15]) + (-4.327594e-04) * (q[i6] * q[i16]) + (-3.073957e-04) * (q[i6] * q[i19])
            + (2.802752e-04) * (q[i6] * q[i20]) + (1.855035e-04) * (q[i6] * q[i22]) + (1.148712e-04) * (q[i7] * q[i8]) + (1.492711e-04) * (q[i7] * q[i9])
            + (-2.008247e-04) * (q[i7] * q[i10]) + (4.271983e-04) * (q[i7] * q[i11]) + (-1.026845e-04) * (q[i7] * q[i12]) + (3.478372e-04) * (q[i7] * q[i15])
            + (1.779659e-04) * (q[i7] * q[i16]) + (-2.169687e-04) * (q[i7] * q[i19]) + (-7.294358e-05) * (q[i7] * q[i20]) + (-1.864748e-04) * (q[i7] * q[i22])
            + (3.364259e-04) * (q[i8] * q[i9]) + (-5.852228e-04) * (q[i8] * q[i10]) + (1.627077e-03) * (q[i8] * q[i11]) + (1.240925e-04) * (q[i8] * q[i12])
            + (2.092922e-04) * (q[i8] * q[i15]) + (3.885455e-05) * (q[i8] * q[i16]) + (-1.010121e-03) * (q[i8] * q[i19]) + (6.841617e-06) * (q[i8] * q[i20])
            + (-2.021385e-07) * (q[i8] * q[i22]) + (1.291903e-05) * (q[i9] * q[i10]) + (2.158245e-04) * (q[i9] * q[i11]) + (3.176837e-05) * (q[i9] * q[i12])
            + (9.140971e-05) * (q[i9] * q[i15]) + (-2.204518e-05) * (q[i9] * q[i16]) + (-6.484038e-05) * (q[i9] * q[i19]) + (8.318399e-05) * (q[i9] * q[i20])
            + (4.165778e-05) * (q[i9] * q[i22]) + (5.205945e-05) * (q[i10] * q[i11]) + (7.259000e-05) * (q[i10] * q[i12]) + (-4.727244e-05) * (q[i10] * q[i15])
            + (-3.285843e-05) * (q[i10] * q[i16]) + (-2.292217e-04) * (q[i10] * q[i19]) + (4.174105e-05) * (q[i10] * q[i20])
            + (-4.007233e-05) * (q[i10] * q[i22]) + (-2.386592e-04) * (q[i11] * q[i12]) + (-4.355309e-04) * (q[i11] * q[i15])
            + (-3.401647e-04) * (q[i11] * q[i16]) + (-1.896972e-03) * (q[i11] * q[i19]) + (2.664331e-05) * (q[i11] * q[i20])
            + (1.066602e-04) * (q[i11] * q[i22]) + (3.056551e-04) * (q[i12] * q[i15]) + (7.957061e-05) * (q[i12] * q[i16]) + (2.692456e-04) * (q[i12] * q[i19])
            + (-1.057655e-04) * (q[i12] * q[i20]) + (1.076934e-04) * (q[i12] * q[i22]) + (-4.443231e-05) * (q[i15] * q[i16])
            + (8.424679e-05) * (q[i15] * q[i19]) + (5.359315e-06) * (q[i15] * q[i20]) + (-2.632293e-05) * (q[i15] * q[i22])
            + (-3.497949e-06) * (q[i16] * q[i19]) + (-9.522871e-05) * (q[i16] * q[i20]) + (2.710906e-05) * (q[i16] * q[i22])
            + (1.342922e-04) * (q[i19] * q[i20]) + (1.658186e-05) * (q[i19] * q[i22]) + (-1.409546e-05) * (q[i20] * q[i22]);
   }

   public void getJQx22(double[] q, double[][] JQ)
   {
      JQ[1][i22] = (-3.452125e-03) * (1) + (-1.955954e-03) * ((2) * q[i22]) + (7.426365e-04) * (q[i0]) + (-1.775374e-03) * (q[i1]) + (-8.366886e-04) * (q[i2])
            + (3.072453e-04) * (q[i3]) + (6.134833e-04) * (q[i4]) + (5.237748e-04) * (q[i5]) + (-4.101186e-04) * (q[i6]) + (-1.333653e-03) * (q[i7])
            + (2.659965e-03) * (q[i8]) + (-8.328203e-05) * (q[i9]) + (3.507823e-04) * (q[i10]) + (-2.830865e-04) * (q[i11]) + (-2.284272e-04) * (q[i12])
            + (-1.155256e-04) * (q[i15]) + (2.038371e-03) * (q[i16]) + (3.017889e-05) * (q[i19]) + (-7.428983e-04) * (q[i20]) + (-5.025617e-06) * (q[i21])
            + (-5.642650e-04) * (q[i0] * q[i0]) + (5.422990e-04) * (q[i1] * q[i1]) + (4.736763e-04) * (q[i2] * q[i2]) + (1.992528e-04) * (q[i3] * q[i3])
            + (-6.013280e-05) * (q[i4] * q[i4]) + (-2.432180e-04) * (q[i5] * q[i5]) + (-1.632641e-05) * (q[i6] * q[i6]) + (-5.552237e-06) * (q[i7] * q[i7])
            + (-1.347525e-04) * (q[i8] * q[i8]) + (-8.623598e-06) * (q[i9] * q[i9]) + (-5.630073e-05) * (q[i10] * q[i10]) + (-9.622360e-05) * (q[i11] * q[i11])
            + (8.008094e-04) * (q[i12] * q[i12]) + (6.319877e-05) * (q[i15] * q[i15]) + (5.458667e-04) * (q[i16] * q[i16]) + (-8.550832e-05) * (q[i19] * q[i19])
            + (6.618400e-04) * (q[i20] * q[i20]) + (5.605760e-05) * (q[i21] * q[i21]) + (4.423919e-06) * ((2) * q[i0] * q[i22])
            + (6.438361e-05) * ((2) * q[i1] * q[i22]) + (1.868505e-04) * ((2) * q[i2] * q[i22]) + (6.041209e-04) * ((2) * q[i3] * q[i22])
            + (4.531129e-05) * ((2) * q[i4] * q[i22]) + (-1.540498e-03) * ((2) * q[i5] * q[i22]) + (3.050303e-04) * ((2) * q[i6] * q[i22])
            + (5.152297e-05) * ((2) * q[i7] * q[i22]) + (-7.910245e-04) * ((2) * q[i8] * q[i22]) + (1.106597e-04) * ((2) * q[i9] * q[i22])
            + (-6.206295e-05) * ((2) * q[i10] * q[i22]) + (5.465612e-05) * ((2) * q[i11] * q[i22]) + (3.705402e-04) * ((2) * q[i12] * q[i22])
            + (4.040670e-05) * ((2) * q[i15] * q[i22]) + (-7.192742e-04) * ((2) * q[i16] * q[i22]) + (3.088854e-05) * ((2) * q[i19] * q[i22])
            + (1.418221e-04) * ((2) * q[i20] * q[i22]) + (5.699931e-05) * ((2) * q[i21] * q[i22]) + (3.898745e-04) * ((3) * q[i22] * q[i22])
            + (6.293637e-04) * (q[i0] * q[i1]) + (1.853998e-04) * (q[i0] * q[i2]) + (-8.913986e-04) * (q[i0] * q[i3]) + (-2.734117e-05) * (q[i0] * q[i4])
            + (4.204164e-04) * (q[i0] * q[i5]) + (8.910689e-04) * (q[i0] * q[i6]) + (-3.477295e-04) * (q[i0] * q[i7]) + (-1.241166e-04) * (q[i0] * q[i8])
            + (-1.234293e-04) * (q[i0] * q[i9]) + (-7.364061e-05) * (q[i0] * q[i10]) + (-3.014434e-04) * (q[i0] * q[i11]) + (-3.450505e-04) * (q[i0] * q[i12])
            + (3.832412e-04) * (q[i0] * q[i15]) + (7.041708e-05) * (q[i0] * q[i16]) + (-1.515405e-04) * (q[i0] * q[i19]) + (5.406636e-05) * (q[i0] * q[i20])
            + (8.145290e-05) * (q[i0] * q[i21]) + (4.438889e-04) * (q[i1] * q[i2]) + (2.563931e-05) * (q[i1] * q[i3]) + (-3.475137e-04) * (q[i1] * q[i4])
            + (2.796101e-04) * (q[i1] * q[i5]) + (-2.211472e-05) * (q[i1] * q[i6]) + (7.638772e-04) * (q[i1] * q[i7]) + (-4.322135e-04) * (q[i1] * q[i8])
            + (5.041266e-04) * (q[i1] * q[i9]) + (2.979040e-04) * (q[i1] * q[i10]) + (1.552551e-04) * (q[i1] * q[i11]) + (4.076207e-04) * (q[i1] * q[i12])
            + (-3.137002e-05) * (q[i1] * q[i15]) + (-2.484144e-04) * (q[i1] * q[i16]) + (-5.140003e-04) * (q[i1] * q[i19]) + (2.056473e-04) * (q[i1] * q[i20])
            + (8.272157e-05) * (q[i1] * q[i21]) + (5.838644e-04) * (q[i2] * q[i3]) + (-8.803963e-04) * (q[i2] * q[i4]) + (7.973751e-04) * (q[i2] * q[i5])
            + (2.455748e-04) * (q[i2] * q[i6]) + (-3.191185e-04) * (q[i2] * q[i7]) + (4.081732e-04) * (q[i2] * q[i8]) + (7.810359e-05) * (q[i2] * q[i9])
            + (2.386402e-04) * (q[i2] * q[i10]) + (-4.507722e-04) * (q[i2] * q[i11]) + (2.083152e-04) * (q[i2] * q[i12]) + (2.249188e-04) * (q[i2] * q[i15])
            + (-1.195472e-04) * (q[i2] * q[i16]) + (-3.780284e-04) * (q[i2] * q[i19]) + (6.258241e-04) * (q[i2] * q[i20]) + (8.527480e-05) * (q[i2] * q[i21])
            + (-5.908238e-04) * (q[i3] * q[i4]) + (2.950487e-04) * (q[i3] * q[i5]) + (9.169762e-04) * (q[i3] * q[i6]) + (4.012098e-04) * (q[i3] * q[i7])
            + (-4.429403e-04) * (q[i3] * q[i8]) + (-3.187391e-04) * (q[i3] * q[i9]) + (1.945716e-04) * (q[i3] * q[i10]) + (-2.337426e-04) * (q[i3] * q[i11])
            + (-2.017979e-04) * (q[i3] * q[i12]) + (2.283656e-04) * (q[i3] * q[i15]) + (-4.900784e-04) * (q[i3] * q[i16]) + (-6.136038e-04) * (q[i3] * q[i19])
            + (-3.933443e-04) * (q[i3] * q[i20]) + (8.998727e-05) * (q[i3] * q[i21]) + (9.782708e-05) * (q[i4] * q[i5]) + (-6.479182e-04) * (q[i4] * q[i6])
            + (3.232710e-04) * (q[i4] * q[i7]) + (-6.370546e-04) * (q[i4] * q[i8]) + (1.095770e-04) * (q[i4] * q[i9]) + (-1.301767e-04) * (q[i4] * q[i10])
            + (1.956478e-04) * (q[i4] * q[i11]) + (-4.228184e-04) * (q[i4] * q[i12]) + (7.991379e-05) * (q[i4] * q[i15]) + (-1.533807e-04) * (q[i4] * q[i16])
            + (3.210236e-04) * (q[i4] * q[i19]) + (-5.392119e-04) * (q[i4] * q[i20]) + (8.042876e-05) * (q[i4] * q[i21]) + (-9.274461e-05) * (q[i5] * q[i6])
            + (-1.583401e-04) * (q[i5] * q[i7]) + (-3.555156e-04) * (q[i5] * q[i8]) + (5.021899e-05) * (q[i5] * q[i9]) + (-4.503147e-05) * (q[i5] * q[i10])
            + (-6.106314e-04) * (q[i5] * q[i11]) + (1.624863e-04) * (q[i5] * q[i12]) + (-3.132046e-04) * (q[i5] * q[i15]) + (1.668104e-03) * (q[i5] * q[i16])
            + (3.425438e-04) * (q[i5] * q[i19]) + (1.535302e-03) * (q[i5] * q[i20]) + (-4.511650e-04) * (q[i5] * q[i21]) + (-8.425326e-04) * (q[i6] * q[i7])
            + (1.065966e-04) * (q[i6] * q[i8]) + (-1.983248e-04) * (q[i6] * q[i9]) + (1.511723e-04) * (q[i6] * q[i10]) + (1.009963e-04) * (q[i6] * q[i11])
            + (-4.363672e-04) * (q[i6] * q[i12]) + (1.826301e-04) * (q[i6] * q[i15]) + (3.514698e-04) * (q[i6] * q[i16]) + (-7.130947e-05) * (q[i6] * q[i19])
            + (-2.261390e-04) * (q[i6] * q[i20]) + (1.855035e-04) * (q[i6] * q[i21]) + (2.757292e-04) * (q[i7] * q[i8]) + (4.424179e-04) * (q[i7] * q[i9])
            + (-1.599506e-04) * (q[i7] * q[i10]) + (2.477358e-04) * (q[i7] * q[i11]) + (-3.307243e-04) * (q[i7] * q[i12]) + (-4.318086e-04) * (q[i7] * q[i15])
            + (-2.200355e-04) * (q[i7] * q[i16]) + (2.842097e-04) * (q[i7] * q[i19]) + (-3.075305e-04) * (q[i7] * q[i20]) + (-1.864748e-04) * (q[i7] * q[i21])
            + (-5.882717e-04) * (q[i8] * q[i9]) + (3.368740e-04) * (q[i8] * q[i10]) + (-1.225490e-04) * (q[i8] * q[i11]) + (-1.640942e-03) * (q[i8] * q[i12])
            + (3.882708e-05) * (q[i8] * q[i15]) + (2.265615e-04) * (q[i8] * q[i16]) + (-1.827928e-07) * (q[i8] * q[i19]) + (-9.942643e-04) * (q[i8] * q[i20])
            + (-2.021385e-07) * (q[i8] * q[i21]) + (1.231432e-05) * (q[i9] * q[i10]) + (-7.281869e-05) * (q[i9] * q[i11]) + (-5.370242e-05) * (q[i9] * q[i12])
            + (-3.221827e-05) * (q[i9] * q[i15]) + (-5.194976e-05) * (q[i9] * q[i16]) + (4.005138e-05) * (q[i9] * q[i19]) + (-2.331863e-04) * (q[i9] * q[i20])
            + (4.165778e-05) * (q[i9] * q[i21]) + (-3.360076e-05) * (q[i10] * q[i11]) + (-2.155244e-04) * (q[i10] * q[i12])
            + (-2.014826e-05) * (q[i10] * q[i15]) + (8.839866e-05) * (q[i10] * q[i16]) + (8.241691e-05) * (q[i10] * q[i19])
            + (-6.584737e-05) * (q[i10] * q[i20]) + (-4.007233e-05) * (q[i10] * q[i21]) + (-2.399177e-04) * (q[i11] * q[i12])
            + (-7.896743e-05) * (q[i11] * q[i15]) + (-3.060890e-04) * (q[i11] * q[i16]) + (1.041701e-04) * (q[i11] * q[i19])
            + (-2.688682e-04) * (q[i11] * q[i20]) + (1.066602e-04) * (q[i11] * q[i21]) + (3.410488e-04) * (q[i12] * q[i15]) + (4.378143e-04) * (q[i12] * q[i16])
            + (-2.546263e-05) * (q[i12] * q[i19]) + (1.885072e-03) * (q[i12] * q[i20]) + (1.076934e-04) * (q[i12] * q[i21])
            + (-4.519659e-05) * (q[i15] * q[i16]) + (-9.699551e-05) * (q[i15] * q[i19]) + (-5.234313e-06) * (q[i15] * q[i20])
            + (-2.632293e-05) * (q[i15] * q[i21]) + (8.001428e-06) * (q[i16] * q[i19]) + (9.050152e-05) * (q[i16] * q[i20]) + (2.710906e-05) * (q[i16] * q[i21])
            + (1.338215e-04) * (q[i19] * q[i20]) + (1.658186e-05) * (q[i19] * q[i21]) + (-1.409546e-05) * (q[i20] * q[i21]);
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
      JQ[2][i0] = (-3.419960e-03) * (1) + (2.105685e-03) * ((2) * q[i0]) + (5.464848e-04) * (q[i1]) + (-2.370456e-04) * (q[i2]) + (-1.042953e-01) * (q[i3])
            + (8.473045e-03) * (q[i4]) + (3.996197e-03) * (q[i5]) + (6.862044e-03) * (q[i6]) + (1.114285e-03) * (q[i7]) + (-1.908835e-03) * (q[i8])
            + (4.586587e-04) * (q[i9]) + (6.756558e-04) * (q[i10]) + (-2.883306e-03) * (q[i11]) + (-9.513398e-04) * (q[i12]) + (3.946335e-03) * (q[i15])
            + (-3.339468e-03) * (q[i16]) + (-6.674361e-04) * (q[i19]) + (-6.149881e-04) * (q[i20]) + (1.730498e-03) * (q[i21]) + (1.026309e-03) * (q[i22])
            + (2.103703e-03) * ((3) * q[i0] * q[i0]) + (9.010064e-04) * ((2) * q[i0] * q[i1]) + (2.649180e-03) * ((2) * q[i0] * q[i2])
            + (-2.712331e-03) * ((2) * q[i0] * q[i3]) + (-1.903058e-03) * ((2) * q[i0] * q[i4]) + (-2.381163e-03) * ((2) * q[i0] * q[i5])
            + (-2.368336e-02) * ((2) * q[i0] * q[i6]) + (-1.687363e-03) * ((2) * q[i0] * q[i7]) + (3.544038e-04) * ((2) * q[i0] * q[i8])
            + (-3.140358e-03) * ((2) * q[i0] * q[i9]) + (-7.828196e-05) * ((2) * q[i0] * q[i10]) + (6.092797e-04) * ((2) * q[i0] * q[i11])
            + (1.224367e-04) * ((2) * q[i0] * q[i12]) + (6.393383e-04) * ((2) * q[i0] * q[i15]) + (-1.798960e-05) * ((2) * q[i0] * q[i16])
            + (9.390891e-05) * ((2) * q[i0] * q[i19]) + (4.563936e-04) * ((2) * q[i0] * q[i20]) + (1.672795e-04) * ((2) * q[i0] * q[i21])
            + (1.590001e-04) * ((2) * q[i0] * q[i22]) + (-8.664399e-04) * (q[i1] * q[i1]) + (1.848346e-03) * (q[i2] * q[i2]) + (7.395034e-03) * (q[i3] * q[i3])
            + (2.391480e-03) * (q[i4] * q[i4]) + (-9.545015e-04) * (q[i5] * q[i5]) + (-4.341833e-03) * (q[i6] * q[i6]) + (8.112818e-04) * (q[i7] * q[i7])
            + (-1.456665e-03) * (q[i8] * q[i8]) + (-1.972233e-04) * (q[i9] * q[i9]) + (3.095510e-04) * (q[i10] * q[i10]) + (2.080339e-04) * (q[i11] * q[i11])
            + (3.540226e-05) * (q[i12] * q[i12]) + (2.058150e-04) * (q[i15] * q[i15]) + (-3.579373e-04) * (q[i16] * q[i16]) + (1.713251e-04) * (q[i19] * q[i19])
            + (-2.899285e-05) * (q[i20] * q[i20]) + (6.082433e-05) * (q[i21] * q[i21]) + (-1.294115e-04) * (q[i22] * q[i22]) + (1.132544e-05) * (q[i1] * q[i2])
            + (-3.003652e-03) * (q[i1] * q[i3]) + (2.966210e-03) * (q[i1] * q[i4]) + (-6.624666e-06) * (q[i1] * q[i5]) + (5.883122e-03) * (q[i1] * q[i6])
            + (5.865579e-03) * (q[i1] * q[i7]) + (2.177889e-03) * (q[i1] * q[i8]) + (1.000905e-03) * (q[i1] * q[i9]) + (9.960849e-04) * (q[i1] * q[i10])
            + (5.025431e-04) * (q[i1] * q[i11]) + (-4.917774e-04) * (q[i1] * q[i12]) + (-2.196508e-04) * (q[i1] * q[i15]) + (-2.161718e-04) * (q[i1] * q[i16])
            + (-8.633194e-04) * (q[i1] * q[i19]) + (-8.676393e-04) * (q[i1] * q[i20]) + (4.477294e-04) * (q[i1] * q[i21]) + (-4.469170e-04) * (q[i1] * q[i22])
            + (-4.418969e-03) * (q[i2] * q[i3]) + (-9.307455e-04) * (q[i2] * q[i4]) + (-1.340187e-03) * (q[i2] * q[i5]) + (-1.042584e-02) * (q[i2] * q[i6])
            + (6.692984e-04) * (q[i2] * q[i7]) + (-4.592705e-03) * (q[i2] * q[i8]) + (-1.144177e-03) * (q[i2] * q[i9]) + (1.910390e-03) * (q[i2] * q[i10])
            + (1.065594e-03) * (q[i2] * q[i11]) + (-1.294444e-04) * (q[i2] * q[i12]) + (2.449058e-04) * (q[i2] * q[i15]) + (2.344968e-04) * (q[i2] * q[i16])
            + (5.742787e-04) * (q[i2] * q[i19]) + (-5.652489e-04) * (q[i2] * q[i20]) + (2.171232e-04) * (q[i2] * q[i21]) + (-8.866820e-04) * (q[i2] * q[i22])
            + (1.369951e-02) * (q[i3] * q[i4]) + (6.985472e-03) * (q[i3] * q[i5]) + (-1.610863e-02) * (q[i3] * q[i6]) + (3.069103e-03) * (q[i3] * q[i7])
            + (-2.993308e-04) * (q[i3] * q[i8]) + (1.178320e-02) * (q[i3] * q[i9]) + (-3.278556e-03) * (q[i3] * q[i10]) + (4.723161e-04) * (q[i3] * q[i11])
            + (-4.202721e-04) * (q[i3] * q[i12]) + (-1.399009e-03) * (q[i3] * q[i15]) + (-4.426984e-03) * (q[i3] * q[i16]) + (-1.230233e-03) * (q[i3] * q[i19])
            + (-4.987277e-04) * (q[i3] * q[i20]) + (-4.396539e-04) * (q[i3] * q[i21]) + (-1.155803e-03) * (q[i3] * q[i22]) + (4.170673e-03) * (q[i4] * q[i5])
            + (-4.381152e-04) * (q[i4] * q[i6]) + (3.905466e-03) * (q[i4] * q[i7]) + (-1.282934e-03) * (q[i4] * q[i8]) + (3.261963e-04) * (q[i4] * q[i9])
            + (-7.603548e-04) * (q[i4] * q[i10]) + (-2.240502e-04) * (q[i4] * q[i11]) + (-8.096605e-04) * (q[i4] * q[i12]) + (3.336301e-05) * (q[i4] * q[i15])
            + (1.829189e-05) * (q[i4] * q[i16]) + (6.015720e-04) * (q[i4] * q[i19]) + (8.366387e-06) * (q[i4] * q[i20]) + (-2.850994e-04) * (q[i4] * q[i21])
            + (-6.946741e-04) * (q[i4] * q[i22]) + (2.766211e-03) * (q[i5] * q[i6]) + (-4.716225e-04) * (q[i5] * q[i7]) + (-2.733731e-05) * (q[i5] * q[i8])
            + (5.309420e-04) * (q[i5] * q[i9]) + (7.873888e-04) * (q[i5] * q[i10]) + (-1.103549e-03) * (q[i5] * q[i11]) + (1.435492e-03) * (q[i5] * q[i12])
            + (-7.085482e-04) * (q[i5] * q[i15]) + (-2.215502e-04) * (q[i5] * q[i16]) + (2.208192e-05) * (q[i5] * q[i19]) + (-6.243246e-04) * (q[i5] * q[i20])
            + (-5.182215e-04) * (q[i5] * q[i21]) + (6.836454e-04) * (q[i5] * q[i22]) + (1.146902e-02) * (q[i6] * q[i7]) + (1.381052e-03) * (q[i6] * q[i8])
            + (-1.529254e-03) * (q[i6] * q[i9]) + (2.113707e-03) * (q[i6] * q[i10]) + (-1.800672e-03) * (q[i6] * q[i11]) + (-1.143893e-03) * (q[i6] * q[i12])
            + (-6.248782e-05) * (q[i6] * q[i15]) + (1.073967e-03) * (q[i6] * q[i16]) + (-1.418968e-03) * (q[i6] * q[i19]) + (1.062149e-03) * (q[i6] * q[i20])
            + (4.109724e-04) * (q[i6] * q[i21]) + (2.203350e-04) * (q[i6] * q[i22]) + (-2.645940e-04) * (q[i7] * q[i8]) + (2.004623e-03) * (q[i7] * q[i9])
            + (8.376530e-05) * (q[i7] * q[i10]) + (1.402177e-04) * (q[i7] * q[i11]) + (-1.169007e-04) * (q[i7] * q[i12]) + (6.449001e-04) * (q[i7] * q[i15])
            + (3.597124e-04) * (q[i7] * q[i16]) + (-1.804753e-04) * (q[i7] * q[i19]) + (6.153951e-05) * (q[i7] * q[i20]) + (-2.018885e-04) * (q[i7] * q[i21])
            + (-5.363128e-04) * (q[i7] * q[i22]) + (1.232822e-03) * (q[i8] * q[i9]) + (1.771793e-04) * (q[i8] * q[i10]) + (-5.257208e-04) * (q[i8] * q[i11])
            + (6.450441e-06) * (q[i8] * q[i12]) + (-2.445200e-04) * (q[i8] * q[i15]) + (6.730540e-04) * (q[i8] * q[i16]) + (1.101120e-04) * (q[i8] * q[i19])
            + (6.534280e-04) * (q[i8] * q[i20]) + (-1.374253e-04) * (q[i8] * q[i21]) + (3.502593e-05) * (q[i8] * q[i22]) + (2.411114e-04) * (q[i9] * q[i10])
            + (8.550443e-05) * (q[i9] * q[i11]) + (-4.645134e-04) * (q[i9] * q[i12]) + (-3.216631e-04) * (q[i9] * q[i15]) + (3.160296e-04) * (q[i9] * q[i16])
            + (-2.978277e-04) * (q[i9] * q[i19]) + (6.484969e-04) * (q[i9] * q[i20]) + (-1.291727e-04) * (q[i9] * q[i21]) + (-1.041448e-04) * (q[i9] * q[i22])
            + (9.875970e-04) * (q[i10] * q[i11]) + (4.411800e-04) * (q[i10] * q[i12]) + (3.368562e-04) * (q[i10] * q[i15]) + (-3.942323e-04) * (q[i10] * q[i16])
            + (3.867514e-05) * (q[i10] * q[i19]) + (3.980839e-05) * (q[i10] * q[i20]) + (-5.573833e-05) * (q[i10] * q[i21])
            + (-2.403846e-04) * (q[i10] * q[i22]) + (1.048394e-04) * (q[i11] * q[i12]) + (1.167051e-04) * (q[i11] * q[i15])
            + (-3.487901e-04) * (q[i11] * q[i16]) + (2.393338e-04) * (q[i11] * q[i19]) + (1.528574e-04) * (q[i11] * q[i20])
            + (-1.610796e-04) * (q[i11] * q[i21]) + (3.191131e-04) * (q[i11] * q[i22]) + (1.398222e-04) * (q[i12] * q[i15])
            + (-7.891911e-05) * (q[i12] * q[i16]) + (-1.259175e-04) * (q[i12] * q[i19]) + (-1.010733e-05) * (q[i12] * q[i20])
            + (-1.028225e-04) * (q[i12] * q[i21]) + (-1.881295e-04) * (q[i12] * q[i22]) + (2.723890e-04) * (q[i15] * q[i16])
            + (-9.798634e-05) * (q[i15] * q[i19]) + (-5.472361e-05) * (q[i15] * q[i20]) + (2.581117e-04) * (q[i15] * q[i21])
            + (8.102358e-05) * (q[i15] * q[i22]) + (-5.580437e-04) * (q[i16] * q[i19]) + (-3.439001e-04) * (q[i16] * q[i20])
            + (-1.036131e-04) * (q[i16] * q[i21]) + (6.974141e-04) * (q[i16] * q[i22]) + (-3.581878e-04) * (q[i19] * q[i20])
            + (-1.261794e-04) * (q[i19] * q[i21]) + (3.998468e-06) * (q[i19] * q[i22]) + (1.815776e-04) * (q[i20] * q[i21])
            + (-2.077979e-04) * (q[i20] * q[i22]) + (-4.586401e-04) * (q[i21] * q[i22]);
   }

   public void getJQy1(double[] q, double[][] JQ)
   {
      JQ[2][i1] = (3.490960e-03) * (1) + (2.094272e-03) * ((2) * q[i1]) + (5.464848e-04) * (q[i0]) + (-2.264149e-04) * (q[i2]) + (8.490752e-03) * (q[i3])
            + (-1.037722e-01) * (q[i4]) + (4.080226e-03) * (q[i5]) + (-1.099342e-03) * (q[i6]) + (-6.925039e-03) * (q[i7]) + (1.898408e-03) * (q[i8])
            + (-6.650083e-04) * (q[i9]) + (-4.832283e-04) * (q[i10]) + (-9.367681e-04) * (q[i11]) + (-2.903912e-03) * (q[i12]) + (3.281194e-03) * (q[i15])
            + (-3.948355e-03) * (q[i16]) + (6.088204e-04) * (q[i19]) + (6.722230e-04) * (q[i20]) + (1.017126e-03) * (q[i21]) + (1.730079e-03) * (q[i22])
            + (9.010064e-04) * (q[i0] * q[i0]) + (-8.664399e-04) * ((2) * q[i0] * q[i1]) + (-2.111054e-03) * ((3) * q[i1] * q[i1])
            + (-2.648981e-03) * ((2) * q[i1] * q[i2]) + (1.908193e-03) * ((2) * q[i1] * q[i3]) + (2.724853e-03) * ((2) * q[i1] * q[i4])
            + (2.403961e-03) * ((2) * q[i1] * q[i5]) + (-1.694718e-03) * ((2) * q[i1] * q[i6]) + (-2.364088e-02) * ((2) * q[i1] * q[i7])
            + (3.581257e-04) * ((2) * q[i1] * q[i8]) + (-7.962768e-05) * ((2) * q[i1] * q[i9]) + (-3.113276e-03) * ((2) * q[i1] * q[i10])
            + (-1.286764e-04) * ((2) * q[i1] * q[i11]) + (-6.013921e-04) * ((2) * q[i1] * q[i12]) + (-2.069750e-05) * ((2) * q[i1] * q[i15])
            + (6.446987e-04) * ((2) * q[i1] * q[i16]) + (4.612640e-04) * ((2) * q[i1] * q[i19]) + (9.646236e-05) * ((2) * q[i1] * q[i20])
            + (-1.517087e-04) * ((2) * q[i1] * q[i21]) + (-1.698214e-04) * ((2) * q[i1] * q[i22]) + (-1.845939e-03) * (q[i2] * q[i2])
            + (-2.389710e-03) * (q[i3] * q[i3]) + (-7.337485e-03) * (q[i4] * q[i4]) + (9.680336e-04) * (q[i5] * q[i5]) + (-8.176774e-04) * (q[i6] * q[i6])
            + (4.321724e-03) * (q[i7] * q[i7]) + (1.468737e-03) * (q[i8] * q[i8]) + (-3.188681e-04) * (q[i9] * q[i9]) + (1.998999e-04) * (q[i10] * q[i10])
            + (-3.758514e-05) * (q[i11] * q[i11]) + (-2.165943e-04) * (q[i12] * q[i12]) + (3.545634e-04) * (q[i15] * q[i15])
            + (-2.042699e-04) * (q[i16] * q[i16]) + (2.274663e-05) * (q[i19] * q[i19]) + (-1.739136e-04) * (q[i20] * q[i20])
            + (1.276385e-04) * (q[i21] * q[i21]) + (-5.942282e-05) * (q[i22] * q[i22]) + (1.132544e-05) * (q[i0] * q[i2]) + (-3.003652e-03) * (q[i0] * q[i3])
            + (2.966210e-03) * (q[i0] * q[i4]) + (-6.624666e-06) * (q[i0] * q[i5]) + (5.883122e-03) * (q[i0] * q[i6]) + (5.865579e-03) * (q[i0] * q[i7])
            + (2.177889e-03) * (q[i0] * q[i8]) + (1.000905e-03) * (q[i0] * q[i9]) + (9.960849e-04) * (q[i0] * q[i10]) + (5.025431e-04) * (q[i0] * q[i11])
            + (-4.917774e-04) * (q[i0] * q[i12]) + (-2.196508e-04) * (q[i0] * q[i15]) + (-2.161718e-04) * (q[i0] * q[i16]) + (-8.633194e-04) * (q[i0] * q[i19])
            + (-8.676393e-04) * (q[i0] * q[i20]) + (4.477294e-04) * (q[i0] * q[i21]) + (-4.469170e-04) * (q[i0] * q[i22]) + (9.468376e-04) * (q[i2] * q[i3])
            + (4.420908e-03) * (q[i2] * q[i4]) + (1.337284e-03) * (q[i2] * q[i5]) + (6.545506e-04) * (q[i2] * q[i6]) + (-1.043075e-02) * (q[i2] * q[i7])
            + (-4.582863e-03) * (q[i2] * q[i8]) + (1.919334e-03) * (q[i2] * q[i9]) + (-1.144844e-03) * (q[i2] * q[i10]) + (1.398578e-04) * (q[i2] * q[i11])
            + (-1.061259e-03) * (q[i2] * q[i12]) + (2.364509e-04) * (q[i2] * q[i15]) + (2.506379e-04) * (q[i2] * q[i16]) + (-5.593445e-04) * (q[i2] * q[i19])
            + (5.744647e-04) * (q[i2] * q[i20]) + (8.894065e-04) * (q[i2] * q[i21]) + (-2.208451e-04) * (q[i2] * q[i22]) + (-1.371216e-02) * (q[i3] * q[i4])
            + (-4.197114e-03) * (q[i3] * q[i5]) + (3.907965e-03) * (q[i3] * q[i6]) + (-4.458325e-04) * (q[i3] * q[i7]) + (-1.308695e-03) * (q[i3] * q[i8])
            + (-7.588419e-04) * (q[i3] * q[i9]) + (3.247102e-04) * (q[i3] * q[i10]) + (8.214066e-04) * (q[i3] * q[i11]) + (2.298352e-04) * (q[i3] * q[i12])
            + (2.560312e-05) * (q[i3] * q[i15]) + (5.096420e-05) * (q[i3] * q[i16]) + (7.963403e-06) * (q[i3] * q[i19]) + (6.092463e-04) * (q[i3] * q[i20])
            + (7.005373e-04) * (q[i3] * q[i21]) + (2.812319e-04) * (q[i3] * q[i22]) + (-7.012161e-03) * (q[i4] * q[i5]) + (3.068551e-03) * (q[i4] * q[i6])
            + (-1.615989e-02) * (q[i4] * q[i7]) + (-2.867031e-04) * (q[i4] * q[i8]) + (-3.326402e-03) * (q[i4] * q[i9]) + (1.165198e-02) * (q[i4] * q[i10])
            + (3.772506e-04) * (q[i4] * q[i11]) + (-4.866641e-04) * (q[i4] * q[i12]) + (-4.366051e-03) * (q[i4] * q[i15]) + (-1.393776e-03) * (q[i4] * q[i16])
            + (-5.003371e-04) * (q[i4] * q[i19]) + (-1.224390e-03) * (q[i4] * q[i20]) + (1.144783e-03) * (q[i4] * q[i21]) + (4.340912e-04) * (q[i4] * q[i22])
            + (-4.758118e-04) * (q[i5] * q[i6]) + (2.736718e-03) * (q[i5] * q[i7]) + (-5.661351e-06) * (q[i5] * q[i8]) + (7.878519e-04) * (q[i5] * q[i9])
            + (5.176418e-04) * (q[i5] * q[i10]) + (-1.469162e-03) * (q[i5] * q[i11]) + (1.040694e-03) * (q[i5] * q[i12]) + (-2.057310e-04) * (q[i5] * q[i15])
            + (-7.129600e-04) * (q[i5] * q[i16]) + (-6.364053e-04) * (q[i5] * q[i19]) + (3.675125e-06) * (q[i5] * q[i20]) + (-6.627542e-04) * (q[i5] * q[i21])
            + (5.078706e-04) * (q[i5] * q[i22]) + (-1.145363e-02) * (q[i6] * q[i7]) + (2.431201e-04) * (q[i6] * q[i8]) + (-9.416981e-05) * (q[i6] * q[i9])
            + (-2.003019e-03) * (q[i6] * q[i10]) + (-1.167237e-04) * (q[i6] * q[i11]) + (1.400456e-04) * (q[i6] * q[i12]) + (-3.584953e-04) * (q[i6] * q[i15])
            + (-6.491164e-04) * (q[i6] * q[i16]) + (-5.207828e-05) * (q[i6] * q[i19]) + (1.743489e-04) * (q[i6] * q[i20]) + (-5.344456e-04) * (q[i6] * q[i21])
            + (-1.932415e-04) * (q[i6] * q[i22]) + (-1.377890e-03) * (q[i7] * q[i8]) + (-2.129486e-03) * (q[i7] * q[i9]) + (1.539917e-03) * (q[i7] * q[i10])
            + (-1.131840e-03) * (q[i7] * q[i11]) + (-1.800183e-03) * (q[i7] * q[i12]) + (-1.062855e-03) * (q[i7] * q[i15]) + (7.710115e-05) * (q[i7] * q[i16])
            + (-1.066193e-03) * (q[i7] * q[i19]) + (1.409631e-03) * (q[i7] * q[i20]) + (2.247653e-04) * (q[i7] * q[i21]) + (4.137149e-04) * (q[i7] * q[i22])
            + (-1.719627e-04) * (q[i8] * q[i9]) + (-1.222720e-03) * (q[i8] * q[i10]) + (3.963438e-05) * (q[i8] * q[i11]) + (-5.528362e-04) * (q[i8] * q[i12])
            + (-6.650239e-04) * (q[i8] * q[i15]) + (2.444242e-04) * (q[i8] * q[i16]) + (-6.624098e-04) * (q[i8] * q[i19]) + (-1.026070e-04) * (q[i8] * q[i20])
            + (2.896848e-05) * (q[i8] * q[i21]) + (-1.338097e-04) * (q[i8] * q[i22]) + (-2.445345e-04) * (q[i9] * q[i10]) + (4.395493e-04) * (q[i9] * q[i11])
            + (9.939138e-04) * (q[i9] * q[i12]) + (3.954244e-04) * (q[i9] * q[i15]) + (-3.384103e-04) * (q[i9] * q[i16]) + (-3.654445e-05) * (q[i9] * q[i19])
            + (-3.530560e-05) * (q[i9] * q[i20]) + (-2.388953e-04) * (q[i9] * q[i21]) + (-5.476873e-05) * (q[i9] * q[i22]) + (-4.684288e-04) * (q[i10] * q[i11])
            + (8.078761e-05) * (q[i10] * q[i12]) + (-3.101763e-04) * (q[i10] * q[i15]) + (3.223845e-04) * (q[i10] * q[i16])
            + (-6.479919e-04) * (q[i10] * q[i19]) + (2.965278e-04) * (q[i10] * q[i20]) + (-1.044877e-04) * (q[i10] * q[i21])
            + (-1.251050e-04) * (q[i10] * q[i22]) + (-1.051395e-04) * (q[i11] * q[i12]) + (-8.438281e-05) * (q[i11] * q[i15])
            + (1.474048e-04) * (q[i11] * q[i16]) + (-1.130834e-05) * (q[i11] * q[i19]) + (-1.253673e-04) * (q[i11] * q[i20])
            + (1.847853e-04) * (q[i11] * q[i21]) + (1.039093e-04) * (q[i11] * q[i22]) + (-3.490332e-04) * (q[i12] * q[i15]) + (1.179321e-04) * (q[i12] * q[i16])
            + (1.497031e-04) * (q[i12] * q[i19]) + (2.454978e-04) * (q[i12] * q[i20]) + (-3.210245e-04) * (q[i12] * q[i21]) + (1.659558e-04) * (q[i12] * q[i22])
            + (-2.649109e-04) * (q[i15] * q[i16]) + (3.470410e-04) * (q[i15] * q[i19]) + (5.617706e-04) * (q[i15] * q[i20]) + (6.902543e-04) * (q[i15] * q[i21])
            + (-1.032416e-04) * (q[i15] * q[i22]) + (4.275607e-05) * (q[i16] * q[i19]) + (1.031505e-04) * (q[i16] * q[i20]) + (7.983962e-05) * (q[i16] * q[i21])
            + (2.655944e-04) * (q[i16] * q[i22]) + (3.652302e-04) * (q[i19] * q[i20]) + (-2.098154e-04) * (q[i19] * q[i21]) + (1.805125e-04) * (q[i19] * q[i22])
            + (6.865431e-06) * (q[i20] * q[i21]) + (-1.258166e-04) * (q[i20] * q[i22]) + (4.540418e-04) * (q[i21] * q[i22]);
   }

   public void getJQy2(double[] q, double[][] JQ)
   {
      JQ[2][i2] = (1.501351e-04) * (1) + (-1.136704e-03) * ((2) * q[i2]) + (-2.370456e-04) * (q[i0]) + (-2.264149e-04) * (q[i1]) + (-2.393672e-02) * (q[i3])
            + (-2.378894e-02) * (q[i4]) + (7.407930e-02) * (q[i5]) + (8.590408e-04) * (q[i6]) + (-8.558570e-04) * (q[i7]) + (-2.238345e-05) * (q[i8])
            + (2.099167e-03) * (q[i9]) + (-2.077411e-03) * (q[i10]) + (-3.180395e-03) * (q[i11]) + (-3.118885e-03) * (q[i12]) + (6.499997e-03) * (q[i15])
            + (-6.603870e-03) * (q[i16]) + (-1.564712e-03) * (q[i19]) + (1.557533e-03) * (q[i20]) + (1.801057e-03) * (q[i21]) + (1.770597e-03) * (q[i22])
            + (2.649180e-03) * (q[i0] * q[i0]) + (-2.648981e-03) * (q[i1] * q[i1]) + (1.848346e-03) * ((2) * q[i0] * q[i2])
            + (-1.845939e-03) * ((2) * q[i1] * q[i2]) + (-1.336101e-05) * ((3) * q[i2] * q[i2]) + (-1.172723e-03) * ((2) * q[i2] * q[i3])
            + (1.181290e-03) * ((2) * q[i2] * q[i4]) + (-9.250102e-06) * ((2) * q[i2] * q[i5]) + (-1.935695e-03) * ((2) * q[i2] * q[i6])
            + (-1.946621e-03) * ((2) * q[i2] * q[i7]) + (-3.025856e-02) * ((2) * q[i2] * q[i8]) + (-1.384378e-03) * ((2) * q[i2] * q[i9])
            + (-1.382920e-03) * ((2) * q[i2] * q[i10]) + (8.277614e-04) * ((2) * q[i2] * q[i11]) + (-8.804478e-04) * ((2) * q[i2] * q[i12])
            + (-5.454598e-05) * ((2) * q[i2] * q[i15]) + (-4.365303e-05) * ((2) * q[i2] * q[i16]) + (6.886514e-04) * ((2) * q[i2] * q[i19])
            + (6.695697e-04) * ((2) * q[i2] * q[i20]) + (1.455581e-04) * ((2) * q[i2] * q[i21]) + (-1.499381e-04) * ((2) * q[i2] * q[i22])
            + (2.295159e-03) * (q[i3] * q[i3]) + (-2.268454e-03) * (q[i4] * q[i4]) + (-2.336732e-05) * (q[i5] * q[i5]) + (-5.110815e-04) * (q[i6] * q[i6])
            + (5.175746e-04) * (q[i7] * q[i7]) + (2.545932e-05) * (q[i8] * q[i8]) + (5.600260e-04) * (q[i9] * q[i9]) + (-5.555149e-04) * (q[i10] * q[i10])
            + (2.278370e-03) * (q[i11] * q[i11]) + (-2.398656e-03) * (q[i12] * q[i12]) + (2.669399e-03) * (q[i15] * q[i15])
            + (-2.696586e-03) * (q[i16] * q[i16]) + (-2.059287e-04) * (q[i19] * q[i19]) + (2.077010e-04) * (q[i20] * q[i20])
            + (3.646947e-04) * (q[i21] * q[i21]) + (-3.571494e-04) * (q[i22] * q[i22]) + (1.132544e-05) * (q[i0] * q[i1]) + (-4.418969e-03) * (q[i0] * q[i3])
            + (-9.307455e-04) * (q[i0] * q[i4]) + (-1.340187e-03) * (q[i0] * q[i5]) + (-1.042584e-02) * (q[i0] * q[i6]) + (6.692984e-04) * (q[i0] * q[i7])
            + (-4.592705e-03) * (q[i0] * q[i8]) + (-1.144177e-03) * (q[i0] * q[i9]) + (1.910390e-03) * (q[i0] * q[i10]) + (1.065594e-03) * (q[i0] * q[i11])
            + (-1.294444e-04) * (q[i0] * q[i12]) + (2.449058e-04) * (q[i0] * q[i15]) + (2.344968e-04) * (q[i0] * q[i16]) + (5.742787e-04) * (q[i0] * q[i19])
            + (-5.652489e-04) * (q[i0] * q[i20]) + (2.171232e-04) * (q[i0] * q[i21]) + (-8.866820e-04) * (q[i0] * q[i22]) + (9.468376e-04) * (q[i1] * q[i3])
            + (4.420908e-03) * (q[i1] * q[i4]) + (1.337284e-03) * (q[i1] * q[i5]) + (6.545506e-04) * (q[i1] * q[i6]) + (-1.043075e-02) * (q[i1] * q[i7])
            + (-4.582863e-03) * (q[i1] * q[i8]) + (1.919334e-03) * (q[i1] * q[i9]) + (-1.144844e-03) * (q[i1] * q[i10]) + (1.398578e-04) * (q[i1] * q[i11])
            + (-1.061259e-03) * (q[i1] * q[i12]) + (2.364509e-04) * (q[i1] * q[i15]) + (2.506379e-04) * (q[i1] * q[i16]) + (-5.593445e-04) * (q[i1] * q[i19])
            + (5.744647e-04) * (q[i1] * q[i20]) + (8.894065e-04) * (q[i1] * q[i21]) + (-2.208451e-04) * (q[i1] * q[i22]) + (-2.037684e-06) * (q[i3] * q[i4])
            + (8.190087e-03) * (q[i3] * q[i5]) + (-3.135415e-03) * (q[i3] * q[i6]) + (1.534195e-03) * (q[i3] * q[i7]) + (1.040342e-03) * (q[i3] * q[i8])
            + (2.559771e-03) * (q[i3] * q[i9]) + (-1.802805e-04) * (q[i3] * q[i10]) + (4.280872e-04) * (q[i3] * q[i11]) + (1.319566e-03) * (q[i3] * q[i12])
            + (7.515078e-04) * (q[i3] * q[i15]) + (-2.011711e-03) * (q[i3] * q[i16]) + (-1.015795e-03) * (q[i3] * q[i19]) + (-2.322656e-04) * (q[i3] * q[i20])
            + (-7.376429e-04) * (q[i3] * q[i21]) + (1.911578e-04) * (q[i3] * q[i22]) + (-8.205304e-03) * (q[i4] * q[i5]) + (1.537182e-03) * (q[i4] * q[i6])
            + (-3.158159e-03) * (q[i4] * q[i7]) + (1.062334e-03) * (q[i4] * q[i8]) + (-2.018845e-04) * (q[i4] * q[i9]) + (2.528187e-03) * (q[i4] * q[i10])
            + (-1.337421e-03) * (q[i4] * q[i11]) + (-4.544515e-04) * (q[i4] * q[i12]) + (-1.998887e-03) * (q[i4] * q[i15]) + (7.697633e-04) * (q[i4] * q[i16])
            + (-2.286704e-04) * (q[i4] * q[i19]) + (-1.016435e-03) * (q[i4] * q[i20]) + (-1.849737e-04) * (q[i4] * q[i21]) + (7.318842e-04) * (q[i4] * q[i22])
            + (-2.330496e-03) * (q[i5] * q[i6]) + (-2.332766e-03) * (q[i5] * q[i7]) + (-1.009762e-02) * (q[i5] * q[i8]) + (5.265570e-03) * (q[i5] * q[i9])
            + (5.205727e-03) * (q[i5] * q[i10]) + (1.832586e-03) * (q[i5] * q[i11]) + (-1.945874e-03) * (q[i5] * q[i12]) + (-6.397119e-03) * (q[i5] * q[i15])
            + (-6.482519e-03) * (q[i5] * q[i16]) + (2.895049e-04) * (q[i5] * q[i19]) + (2.973489e-04) * (q[i5] * q[i20]) + (8.994120e-04) * (q[i5] * q[i21])
            + (-9.153289e-04) * (q[i5] * q[i22]) + (-7.705269e-06) * (q[i6] * q[i7]) + (7.640783e-03) * (q[i6] * q[i8]) + (9.360254e-04) * (q[i6] * q[i9])
            + (-9.274168e-04) * (q[i6] * q[i10]) + (-1.253649e-03) * (q[i6] * q[i11]) + (3.816430e-05) * (q[i6] * q[i12]) + (-6.280712e-04) * (q[i6] * q[i15])
            + (4.216966e-04) * (q[i6] * q[i16]) + (-2.789274e-04) * (q[i6] * q[i19]) + (8.815289e-04) * (q[i6] * q[i20]) + (4.687912e-04) * (q[i6] * q[i21])
            + (-7.638846e-05) * (q[i6] * q[i22]) + (-7.652251e-03) * (q[i7] * q[i8]) + (9.302299e-04) * (q[i7] * q[i9]) + (-9.301464e-04) * (q[i7] * q[i10])
            + (5.285828e-05) * (q[i7] * q[i11]) + (-1.292311e-03) * (q[i7] * q[i12]) + (-4.168032e-04) * (q[i7] * q[i15]) + (6.447545e-04) * (q[i7] * q[i16])
            + (-8.905544e-04) * (q[i7] * q[i19]) + (2.646359e-04) * (q[i7] * q[i20]) + (-7.990804e-05) * (q[i7] * q[i21]) + (4.671243e-04) * (q[i7] * q[i22])
            + (2.822017e-03) * (q[i8] * q[i9]) + (-2.806527e-03) * (q[i8] * q[i10]) + (4.744048e-03) * (q[i8] * q[i11]) + (4.759668e-03) * (q[i8] * q[i12])
            + (5.980369e-05) * (q[i8] * q[i15]) + (-8.999536e-05) * (q[i8] * q[i16]) + (-1.453997e-05) * (q[i8] * q[i19]) + (2.125620e-05) * (q[i8] * q[i20])
            + (-2.847479e-04) * (q[i8] * q[i21]) + (-2.826212e-04) * (q[i8] * q[i22]) + (2.633729e-07) * (q[i9] * q[i10]) + (-2.112597e-04) * (q[i9] * q[i11])
            + (2.393001e-04) * (q[i9] * q[i12]) + (2.405074e-04) * (q[i9] * q[i15]) + (-2.254603e-04) * (q[i9] * q[i16]) + (-3.864767e-04) * (q[i9] * q[i19])
            + (-1.316529e-07) * (q[i9] * q[i20]) + (-2.489416e-04) * (q[i9] * q[i21]) + (-2.295709e-04) * (q[i9] * q[i22]) + (2.285840e-04) * (q[i10] * q[i11])
            + (-2.200809e-04) * (q[i10] * q[i12]) + (2.278704e-04) * (q[i10] * q[i15]) + (-2.443248e-04) * (q[i10] * q[i16])
            + (-5.623259e-06) * (q[i10] * q[i19]) + (3.850246e-04) * (q[i10] * q[i20]) + (-2.336341e-04) * (q[i10] * q[i21])
            + (-2.438366e-04) * (q[i10] * q[i22]) + (4.690415e-06) * (q[i11] * q[i12]) + (-2.637985e-03) * (q[i11] * q[i15])
            + (-6.450256e-04) * (q[i11] * q[i16]) + (8.071551e-04) * (q[i11] * q[i19]) + (-4.746056e-04) * (q[i11] * q[i20])
            + (6.157953e-04) * (q[i11] * q[i21]) + (4.651000e-04) * (q[i11] * q[i22]) + (-6.507329e-04) * (q[i12] * q[i15])
            + (-2.629174e-03) * (q[i12] * q[i16]) + (-4.714424e-04) * (q[i12] * q[i19]) + (7.961283e-04) * (q[i12] * q[i20])
            + (-4.648714e-04) * (q[i12] * q[i21]) + (-6.117027e-04) * (q[i12] * q[i22]) + (-7.984367e-08) * (q[i15] * q[i16])
            + (2.449223e-04) * (q[i15] * q[i19]) + (6.778326e-04) * (q[i15] * q[i20]) + (6.348229e-04) * (q[i15] * q[i21]) + (2.914068e-05) * (q[i15] * q[i22])
            + (-6.885542e-04) * (q[i16] * q[i19]) + (-2.187713e-04) * (q[i16] * q[i20]) + (2.597040e-05) * (q[i16] * q[i21])
            + (6.360450e-04) * (q[i16] * q[i22]) + (7.832015e-06) * (q[i19] * q[i20]) + (-7.995270e-04) * (q[i19] * q[i21]) + (2.315073e-05) * (q[i19] * q[i22])
            + (2.628485e-05) * (q[i20] * q[i21]) + (-7.908005e-04) * (q[i20] * q[i22]) + (-2.930688e-06) * (q[i21] * q[i22]);
   }

   public void getJQy3(double[] q, double[][] JQ)
   {
      JQ[2][i3] = (1.558602e-03) * (1) + (6.271335e-04) * ((2) * q[i3]) + (-1.042953e-01) * (q[i0]) + (8.490752e-03) * (q[i1]) + (-2.393672e-02) * (q[i2])
            + (-2.404377e-03) * (q[i4]) + (5.817271e-04) * (q[i5]) + (-1.221548e-02) * (q[i6]) + (1.650458e-02) * (q[i7]) + (1.532152e-03) * (q[i8])
            + (-5.205169e-03) * (q[i9]) + (4.690330e-03) * (q[i10]) + (-3.008612e-03) * (q[i11]) + (-5.443482e-03) * (q[i12]) + (3.868940e-04) * (q[i15])
            + (1.664877e-03) * (q[i16]) + (-2.862405e-03) * (q[i19]) + (-6.835361e-04) * (q[i20]) + (-1.083523e-03) * (q[i21]) + (1.513340e-04) * (q[i22])
            + (-2.712331e-03) * (q[i0] * q[i0]) + (1.908193e-03) * (q[i1] * q[i1]) + (-1.172723e-03) * (q[i2] * q[i2]) + (7.395034e-03) * ((2) * q[i0] * q[i3])
            + (-2.389710e-03) * ((2) * q[i1] * q[i3]) + (2.295159e-03) * ((2) * q[i2] * q[i3]) + (-2.073203e-04) * ((3) * q[i3] * q[i3])
            + (2.238555e-03) * ((2) * q[i3] * q[i4]) + (1.725926e-03) * ((2) * q[i3] * q[i5]) + (1.004020e-02) * ((2) * q[i3] * q[i6])
            + (-3.008768e-03) * ((2) * q[i3] * q[i7]) + (3.782819e-03) * ((2) * q[i3] * q[i8]) + (1.625013e-03) * ((2) * q[i3] * q[i9])
            + (-1.056831e-03) * ((2) * q[i3] * q[i10]) + (2.469390e-04) * ((2) * q[i3] * q[i11]) + (-1.931397e-04) * ((2) * q[i3] * q[i12])
            + (5.459118e-04) * ((2) * q[i3] * q[i15]) + (-2.540502e-05) * ((2) * q[i3] * q[i16]) + (6.712668e-04) * ((2) * q[i3] * q[i19])
            + (-6.506100e-04) * ((2) * q[i3] * q[i20]) + (-2.625181e-04) * ((2) * q[i3] * q[i21]) + (1.063758e-04) * ((2) * q[i3] * q[i22])
            + (-2.231748e-03) * (q[i4] * q[i4]) + (-4.359289e-04) * (q[i5] * q[i5]) + (-1.290716e-03) * (q[i6] * q[i6]) + (-9.041409e-04) * (q[i7] * q[i7])
            + (-1.144956e-03) * (q[i8] * q[i8]) + (8.883241e-04) * (q[i9] * q[i9]) + (-8.852734e-04) * (q[i10] * q[i10]) + (6.860758e-04) * (q[i11] * q[i11])
            + (4.919326e-04) * (q[i12] * q[i12]) + (6.707951e-05) * (q[i15] * q[i15]) + (4.782192e-05) * (q[i16] * q[i16]) + (-6.031514e-04) * (q[i19] * q[i19])
            + (-6.408954e-05) * (q[i20] * q[i20]) + (1.764318e-04) * (q[i21] * q[i21]) + (-2.551087e-04) * (q[i22] * q[i22]) + (-3.003652e-03) * (q[i0] * q[i1])
            + (-4.418969e-03) * (q[i0] * q[i2]) + (1.369951e-02) * (q[i0] * q[i4]) + (6.985472e-03) * (q[i0] * q[i5]) + (-1.610863e-02) * (q[i0] * q[i6])
            + (3.069103e-03) * (q[i0] * q[i7]) + (-2.993308e-04) * (q[i0] * q[i8]) + (1.178320e-02) * (q[i0] * q[i9]) + (-3.278556e-03) * (q[i0] * q[i10])
            + (4.723161e-04) * (q[i0] * q[i11]) + (-4.202721e-04) * (q[i0] * q[i12]) + (-1.399009e-03) * (q[i0] * q[i15]) + (-4.426984e-03) * (q[i0] * q[i16])
            + (-1.230233e-03) * (q[i0] * q[i19]) + (-4.987277e-04) * (q[i0] * q[i20]) + (-4.396539e-04) * (q[i0] * q[i21]) + (-1.155803e-03) * (q[i0] * q[i22])
            + (9.468376e-04) * (q[i1] * q[i2]) + (-1.371216e-02) * (q[i1] * q[i4]) + (-4.197114e-03) * (q[i1] * q[i5]) + (3.907965e-03) * (q[i1] * q[i6])
            + (-4.458325e-04) * (q[i1] * q[i7]) + (-1.308695e-03) * (q[i1] * q[i8]) + (-7.588419e-04) * (q[i1] * q[i9]) + (3.247102e-04) * (q[i1] * q[i10])
            + (8.214066e-04) * (q[i1] * q[i11]) + (2.298352e-04) * (q[i1] * q[i12]) + (2.560312e-05) * (q[i1] * q[i15]) + (5.096420e-05) * (q[i1] * q[i16])
            + (7.963403e-06) * (q[i1] * q[i19]) + (6.092463e-04) * (q[i1] * q[i20]) + (7.005373e-04) * (q[i1] * q[i21]) + (2.812319e-04) * (q[i1] * q[i22])
            + (-2.037684e-06) * (q[i2] * q[i4]) + (8.190087e-03) * (q[i2] * q[i5]) + (-3.135415e-03) * (q[i2] * q[i6]) + (1.534195e-03) * (q[i2] * q[i7])
            + (1.040342e-03) * (q[i2] * q[i8]) + (2.559771e-03) * (q[i2] * q[i9]) + (-1.802805e-04) * (q[i2] * q[i10]) + (4.280872e-04) * (q[i2] * q[i11])
            + (1.319566e-03) * (q[i2] * q[i12]) + (7.515078e-04) * (q[i2] * q[i15]) + (-2.011711e-03) * (q[i2] * q[i16]) + (-1.015795e-03) * (q[i2] * q[i19])
            + (-2.322656e-04) * (q[i2] * q[i20]) + (-7.376429e-04) * (q[i2] * q[i21]) + (1.911578e-04) * (q[i2] * q[i22]) + (8.074756e-06) * (q[i4] * q[i5])
            + (-3.397184e-03) * (q[i4] * q[i6]) + (-3.358738e-03) * (q[i4] * q[i7]) + (-1.285209e-03) * (q[i4] * q[i8]) + (-1.819951e-03) * (q[i4] * q[i9])
            + (-1.819332e-03) * (q[i4] * q[i10]) + (5.681483e-04) * (q[i4] * q[i11]) + (-6.056154e-04) * (q[i4] * q[i12]) + (-1.645833e-04) * (q[i4] * q[i15])
            + (-1.473190e-04) * (q[i4] * q[i16]) + (1.669453e-03) * (q[i4] * q[i19]) + (1.654502e-03) * (q[i4] * q[i20]) + (-7.266920e-04) * (q[i4] * q[i21])
            + (7.298847e-04) * (q[i4] * q[i22]) + (-6.396838e-03) * (q[i5] * q[i6]) + (2.099809e-03) * (q[i5] * q[i7]) + (-6.112289e-03) * (q[i5] * q[i8])
            + (-1.097910e-03) * (q[i5] * q[i9]) + (-3.088242e-04) * (q[i5] * q[i10]) + (-1.929560e-03) * (q[i5] * q[i11]) + (-1.744299e-03) * (q[i5] * q[i12])
            + (-1.171593e-03) * (q[i5] * q[i15]) + (1.103235e-03) * (q[i5] * q[i16]) + (1.788667e-04) * (q[i5] * q[i19]) + (-1.627474e-03) * (q[i5] * q[i20])
            + (2.501059e-04) * (q[i5] * q[i21]) + (-1.042687e-03) * (q[i5] * q[i22]) + (1.714701e-03) * (q[i6] * q[i7]) + (4.703570e-04) * (q[i6] * q[i8])
            + (1.244003e-03) * (q[i6] * q[i9]) + (-1.035583e-03) * (q[i6] * q[i10]) + (-1.374444e-03) * (q[i6] * q[i11]) + (-3.581392e-04) * (q[i6] * q[i12])
            + (-6.172732e-04) * (q[i6] * q[i15]) + (7.018440e-04) * (q[i6] * q[i16]) + (6.625531e-04) * (q[i6] * q[i19]) + (2.179492e-04) * (q[i6] * q[i20])
            + (-7.358712e-06) * (q[i6] * q[i21]) + (1.187888e-04) * (q[i6] * q[i22]) + (1.178482e-03) * (q[i7] * q[i8]) + (-1.221219e-03) * (q[i7] * q[i9])
            + (-1.806832e-03) * (q[i7] * q[i10]) + (2.956707e-04) * (q[i7] * q[i11]) + (-1.233349e-04) * (q[i7] * q[i12]) + (7.007633e-04) * (q[i7] * q[i15])
            + (-9.436389e-05) * (q[i7] * q[i16]) + (-9.165213e-05) * (q[i7] * q[i19]) + (4.630918e-04) * (q[i7] * q[i20]) + (-9.779688e-05) * (q[i7] * q[i21])
            + (7.096689e-04) * (q[i7] * q[i22]) + (-1.304794e-03) * (q[i8] * q[i9]) + (7.952767e-04) * (q[i8] * q[i10]) + (1.033786e-03) * (q[i8] * q[i11])
            + (8.022163e-04) * (q[i8] * q[i12]) + (-1.804215e-03) * (q[i8] * q[i15]) + (9.682878e-04) * (q[i8] * q[i16]) + (2.773115e-05) * (q[i8] * q[i19])
            + (8.255406e-04) * (q[i8] * q[i20]) + (1.261761e-04) * (q[i8] * q[i21]) + (6.748273e-04) * (q[i8] * q[i22]) + (-4.577861e-04) * (q[i9] * q[i10])
            + (2.531383e-04) * (q[i9] * q[i11]) + (5.033502e-04) * (q[i9] * q[i12]) + (-3.873931e-04) * (q[i9] * q[i15]) + (-6.382520e-04) * (q[i9] * q[i16])
            + (1.814252e-04) * (q[i9] * q[i19]) + (-2.641220e-05) * (q[i9] * q[i20]) + (2.187432e-04) * (q[i9] * q[i21]) + (2.044403e-04) * (q[i9] * q[i22])
            + (-8.193292e-04) * (q[i10] * q[i11]) + (3.973499e-04) * (q[i10] * q[i12]) + (-4.077110e-04) * (q[i10] * q[i15])
            + (4.492608e-04) * (q[i10] * q[i16]) + (2.930732e-04) * (q[i10] * q[i19]) + (4.740704e-04) * (q[i10] * q[i20]) + (-8.036297e-05) * (q[i10] * q[i21])
            + (-1.194938e-04) * (q[i10] * q[i22]) + (-5.889736e-04) * (q[i11] * q[i12]) + (-7.788329e-04) * (q[i11] * q[i15])
            + (3.615160e-04) * (q[i11] * q[i16]) + (-6.880792e-04) * (q[i11] * q[i19]) + (-5.652306e-05) * (q[i11] * q[i20])
            + (-8.779510e-04) * (q[i11] * q[i21]) + (-2.416514e-04) * (q[i11] * q[i22]) + (-8.186351e-05) * (q[i12] * q[i15])
            + (-1.012331e-03) * (q[i12] * q[i16]) + (3.051342e-04) * (q[i12] * q[i19]) + (3.018533e-04) * (q[i12] * q[i20]) + (2.076512e-04) * (q[i12] * q[i21])
            + (2.997615e-04) * (q[i12] * q[i22]) + (-1.571992e-04) * (q[i15] * q[i16]) + (-5.973864e-04) * (q[i15] * q[i19])
            + (-2.824218e-04) * (q[i15] * q[i20]) + (-4.609274e-04) * (q[i15] * q[i21]) + (-3.611729e-04) * (q[i15] * q[i22])
            + (-6.815973e-04) * (q[i16] * q[i19]) + (6.176811e-05) * (q[i16] * q[i20]) + (-1.660415e-05) * (q[i16] * q[i21])
            + (-1.861034e-04) * (q[i16] * q[i22]) + (1.477329e-04) * (q[i19] * q[i20]) + (-1.040570e-04) * (q[i19] * q[i21])
            + (4.657604e-04) * (q[i19] * q[i22]) + (-6.336138e-04) * (q[i20] * q[i21]) + (3.488995e-04) * (q[i20] * q[i22])
            + (1.227214e-04) * (q[i21] * q[i22]);
   }

   public void getJQy4(double[] q, double[][] JQ)
   {
      JQ[2][i4] = (-1.540007e-03) * (1) + (6.097098e-04) * ((2) * q[i4]) + (8.473045e-03) * (q[i0]) + (-1.037722e-01) * (q[i1]) + (-2.378894e-02) * (q[i2])
            + (-2.404377e-03) * (q[i3]) + (5.476178e-04) * (q[i5]) + (-1.650476e-02) * (q[i6]) + (1.214653e-02) * (q[i7]) + (-1.465220e-03) * (q[i8])
            + (-4.747030e-03) * (q[i9]) + (5.152011e-03) * (q[i10]) + (-5.395995e-03) * (q[i11]) + (-3.023244e-03) * (q[i12]) + (-1.631447e-03) * (q[i15])
            + (-3.370856e-04) * (q[i16]) + (6.483261e-04) * (q[i19]) + (2.841136e-03) * (q[i20]) + (1.670042e-04) * (q[i21]) + (-1.072809e-03) * (q[i22])
            + (-1.903058e-03) * (q[i0] * q[i0]) + (2.724853e-03) * (q[i1] * q[i1]) + (1.181290e-03) * (q[i2] * q[i2]) + (2.238555e-03) * (q[i3] * q[i3])
            + (2.391480e-03) * ((2) * q[i0] * q[i4]) + (-7.337485e-03) * ((2) * q[i1] * q[i4]) + (-2.268454e-03) * ((2) * q[i2] * q[i4])
            + (-2.231748e-03) * ((2) * q[i3] * q[i4]) + (1.951299e-04) * ((3) * q[i4] * q[i4]) + (-1.735576e-03) * ((2) * q[i4] * q[i5])
            + (-3.015428e-03) * ((2) * q[i4] * q[i6]) + (1.006275e-02) * ((2) * q[i4] * q[i7]) + (3.800462e-03) * ((2) * q[i4] * q[i8])
            + (-1.062881e-03) * ((2) * q[i4] * q[i9]) + (1.624405e-03) * ((2) * q[i4] * q[i10]) + (1.929410e-04) * ((2) * q[i4] * q[i11])
            + (-2.181179e-04) * ((2) * q[i4] * q[i12]) + (-2.829543e-05) * ((2) * q[i4] * q[i15]) + (5.306661e-04) * ((2) * q[i4] * q[i16])
            + (-6.442424e-04) * ((2) * q[i4] * q[i19]) + (6.694240e-04) * ((2) * q[i4] * q[i20]) + (-1.053468e-04) * ((2) * q[i4] * q[i21])
            + (2.582947e-04) * ((2) * q[i4] * q[i22]) + (4.155623e-04) * (q[i5] * q[i5]) + (9.079333e-04) * (q[i6] * q[i6]) + (1.320553e-03) * (q[i7] * q[i7])
            + (1.125159e-03) * (q[i8] * q[i8]) + (8.941807e-04) * (q[i9] * q[i9]) + (-8.770639e-04) * (q[i10] * q[i10]) + (-4.984141e-04) * (q[i11] * q[i11])
            + (-7.033265e-04) * (q[i12] * q[i12]) + (-4.986179e-05) * (q[i15] * q[i15]) + (-6.367990e-05) * (q[i16] * q[i16])
            + (6.606077e-05) * (q[i19] * q[i19]) + (5.927220e-04) * (q[i20] * q[i20]) + (2.558276e-04) * (q[i21] * q[i21]) + (-1.807228e-04) * (q[i22] * q[i22])
            + (2.966210e-03) * (q[i0] * q[i1]) + (-9.307455e-04) * (q[i0] * q[i2]) + (1.369951e-02) * (q[i0] * q[i3]) + (4.170673e-03) * (q[i0] * q[i5])
            + (-4.381152e-04) * (q[i0] * q[i6]) + (3.905466e-03) * (q[i0] * q[i7]) + (-1.282934e-03) * (q[i0] * q[i8]) + (3.261963e-04) * (q[i0] * q[i9])
            + (-7.603548e-04) * (q[i0] * q[i10]) + (-2.240502e-04) * (q[i0] * q[i11]) + (-8.096605e-04) * (q[i0] * q[i12]) + (3.336301e-05) * (q[i0] * q[i15])
            + (1.829189e-05) * (q[i0] * q[i16]) + (6.015720e-04) * (q[i0] * q[i19]) + (8.366387e-06) * (q[i0] * q[i20]) + (-2.850994e-04) * (q[i0] * q[i21])
            + (-6.946741e-04) * (q[i0] * q[i22]) + (4.420908e-03) * (q[i1] * q[i2]) + (-1.371216e-02) * (q[i1] * q[i3]) + (-7.012161e-03) * (q[i1] * q[i5])
            + (3.068551e-03) * (q[i1] * q[i6]) + (-1.615989e-02) * (q[i1] * q[i7]) + (-2.867031e-04) * (q[i1] * q[i8]) + (-3.326402e-03) * (q[i1] * q[i9])
            + (1.165198e-02) * (q[i1] * q[i10]) + (3.772506e-04) * (q[i1] * q[i11]) + (-4.866641e-04) * (q[i1] * q[i12]) + (-4.366051e-03) * (q[i1] * q[i15])
            + (-1.393776e-03) * (q[i1] * q[i16]) + (-5.003371e-04) * (q[i1] * q[i19]) + (-1.224390e-03) * (q[i1] * q[i20]) + (1.144783e-03) * (q[i1] * q[i21])
            + (4.340912e-04) * (q[i1] * q[i22]) + (-2.037684e-06) * (q[i2] * q[i3]) + (-8.205304e-03) * (q[i2] * q[i5]) + (1.537182e-03) * (q[i2] * q[i6])
            + (-3.158159e-03) * (q[i2] * q[i7]) + (1.062334e-03) * (q[i2] * q[i8]) + (-2.018845e-04) * (q[i2] * q[i9]) + (2.528187e-03) * (q[i2] * q[i10])
            + (-1.337421e-03) * (q[i2] * q[i11]) + (-4.544515e-04) * (q[i2] * q[i12]) + (-1.998887e-03) * (q[i2] * q[i15]) + (7.697633e-04) * (q[i2] * q[i16])
            + (-2.286704e-04) * (q[i2] * q[i19]) + (-1.016435e-03) * (q[i2] * q[i20]) + (-1.849737e-04) * (q[i2] * q[i21]) + (7.318842e-04) * (q[i2] * q[i22])
            + (8.074756e-06) * (q[i3] * q[i5]) + (-3.397184e-03) * (q[i3] * q[i6]) + (-3.358738e-03) * (q[i3] * q[i7]) + (-1.285209e-03) * (q[i3] * q[i8])
            + (-1.819951e-03) * (q[i3] * q[i9]) + (-1.819332e-03) * (q[i3] * q[i10]) + (5.681483e-04) * (q[i3] * q[i11]) + (-6.056154e-04) * (q[i3] * q[i12])
            + (-1.645833e-04) * (q[i3] * q[i15]) + (-1.473190e-04) * (q[i3] * q[i16]) + (1.669453e-03) * (q[i3] * q[i19]) + (1.654502e-03) * (q[i3] * q[i20])
            + (-7.266920e-04) * (q[i3] * q[i21]) + (7.298847e-04) * (q[i3] * q[i22]) + (2.088206e-03) * (q[i5] * q[i6]) + (-6.389809e-03) * (q[i5] * q[i7])
            + (-6.133572e-03) * (q[i5] * q[i8]) + (-3.236227e-04) * (q[i5] * q[i9]) + (-1.076212e-03) * (q[i5] * q[i10]) + (1.749562e-03) * (q[i5] * q[i11])
            + (1.903223e-03) * (q[i5] * q[i12]) + (1.105931e-03) * (q[i5] * q[i15]) + (-1.172226e-03) * (q[i5] * q[i16]) + (-1.633060e-03) * (q[i5] * q[i19])
            + (1.739405e-04) * (q[i5] * q[i20]) + (1.047099e-03) * (q[i5] * q[i21]) + (-2.370044e-04) * (q[i5] * q[i22]) + (-1.735335e-03) * (q[i6] * q[i7])
            + (-1.170429e-03) * (q[i6] * q[i8]) + (1.819040e-03) * (q[i6] * q[i9]) + (1.206311e-03) * (q[i6] * q[i10]) + (-1.265048e-04) * (q[i6] * q[i11])
            + (3.047392e-04) * (q[i6] * q[i12]) + (9.692188e-05) * (q[i6] * q[i15]) + (-7.136097e-04) * (q[i6] * q[i16]) + (-4.620975e-04) * (q[i6] * q[i19])
            + (9.937660e-05) * (q[i6] * q[i20]) + (7.054472e-04) * (q[i6] * q[i21]) + (-1.034558e-04) * (q[i6] * q[i22]) + (-4.796848e-04) * (q[i7] * q[i8])
            + (1.030994e-03) * (q[i7] * q[i9]) + (-1.225452e-03) * (q[i7] * q[i10]) + (-3.746156e-04) * (q[i7] * q[i11]) + (-1.389476e-03) * (q[i7] * q[i12])
            + (-6.976053e-04) * (q[i7] * q[i15]) + (6.232834e-04) * (q[i7] * q[i16]) + (-2.170230e-04) * (q[i7] * q[i19]) + (-6.638632e-04) * (q[i7] * q[i20])
            + (1.071998e-04) * (q[i7] * q[i21]) + (-5.496944e-06) * (q[i7] * q[i22]) + (-7.931301e-04) * (q[i8] * q[i9]) + (1.294842e-03) * (q[i8] * q[i10])
            + (7.959779e-04) * (q[i8] * q[i11]) + (1.067223e-03) * (q[i8] * q[i12]) + (-9.611609e-04) * (q[i8] * q[i15]) + (1.829801e-03) * (q[i8] * q[i16])
            + (-8.232891e-04) * (q[i8] * q[i19]) + (-3.856416e-05) * (q[i8] * q[i20]) + (6.825732e-04) * (q[i8] * q[i21]) + (1.317616e-04) * (q[i8] * q[i22])
            + (4.573698e-04) * (q[i9] * q[i10]) + (3.937067e-04) * (q[i9] * q[i11]) + (-8.228550e-04) * (q[i9] * q[i12]) + (-4.485047e-04) * (q[i9] * q[i15])
            + (3.978682e-04) * (q[i9] * q[i16]) + (-4.595379e-04) * (q[i9] * q[i19]) + (-2.947117e-04) * (q[i9] * q[i20]) + (-1.234635e-04) * (q[i9] * q[i21])
            + (-8.447726e-05) * (q[i9] * q[i22]) + (4.898072e-04) * (q[i10] * q[i11]) + (2.566401e-04) * (q[i10] * q[i12]) + (6.345679e-04) * (q[i10] * q[i15])
            + (3.787644e-04) * (q[i10] * q[i16]) + (2.490965e-05) * (q[i10] * q[i19]) + (-1.768820e-04) * (q[i10] * q[i20]) + (2.002999e-04) * (q[i10] * q[i21])
            + (2.178710e-04) * (q[i10] * q[i22]) + (5.899136e-04) * (q[i11] * q[i12]) + (-1.003587e-03) * (q[i11] * q[i15])
            + (-8.476519e-05) * (q[i11] * q[i16]) + (2.910371e-04) * (q[i11] * q[i19]) + (3.013094e-04) * (q[i11] * q[i20])
            + (-3.079492e-04) * (q[i11] * q[i21]) + (-2.066231e-04) * (q[i11] * q[i22]) + (3.631967e-04) * (q[i12] * q[i15])
            + (-7.855407e-04) * (q[i12] * q[i16]) + (-6.787080e-05) * (q[i12] * q[i19]) + (-6.834487e-04) * (q[i12] * q[i20])
            + (2.436982e-04) * (q[i12] * q[i21]) + (8.872700e-04) * (q[i12] * q[i22]) + (1.578929e-04) * (q[i15] * q[i16]) + (-7.381783e-05) * (q[i15] * q[i19])
            + (6.806984e-04) * (q[i15] * q[i20]) + (-1.828871e-04) * (q[i15] * q[i21]) + (-2.002160e-05) * (q[i15] * q[i22])
            + (2.789459e-04) * (q[i16] * q[i19]) + (5.880863e-04) * (q[i16] * q[i20]) + (-3.602423e-04) * (q[i16] * q[i21])
            + (-4.693425e-04) * (q[i16] * q[i22]) + (-1.476584e-04) * (q[i19] * q[i20]) + (3.561740e-04) * (q[i19] * q[i21])
            + (-6.224446e-04) * (q[i19] * q[i22]) + (4.618507e-04) * (q[i20] * q[i21]) + (-1.077073e-04) * (q[i20] * q[i22])
            + (-1.220856e-04) * (q[i21] * q[i22]);
   }

   public void getJQy5(double[] q, double[][] JQ)
   {
      JQ[2][i5] = (-3.710266e-05) * (1) + (-2.102908e-03) * ((2) * q[i5]) + (3.996197e-03) * (q[i0]) + (4.080226e-03) * (q[i1]) + (7.407930e-02) * (q[i2])
            + (5.817271e-04) * (q[i3]) + (5.476178e-04) * (q[i4]) + (-9.827014e-03) * (q[i6]) + (9.830779e-03) * (q[i7]) + (-1.302839e-05) * (q[i8])
            + (-5.307797e-03) * (q[i9]) + (5.287685e-03) * (q[i10]) + (-3.371008e-03) * (q[i11]) + (-3.296339e-03) * (q[i12]) + (6.842918e-04) * (q[i15])
            + (-7.180200e-04) * (q[i16]) + (-7.658719e-04) * (q[i19]) + (7.757564e-04) * (q[i20]) + (1.357518e-03) * (q[i21]) + (1.335329e-03) * (q[i22])
            + (-2.381163e-03) * (q[i0] * q[i0]) + (2.403961e-03) * (q[i1] * q[i1]) + (-9.250102e-06) * (q[i2] * q[i2]) + (1.725926e-03) * (q[i3] * q[i3])
            + (-1.735576e-03) * (q[i4] * q[i4]) + (-9.545015e-04) * ((2) * q[i0] * q[i5]) + (9.680336e-04) * ((2) * q[i1] * q[i5])
            + (-2.336732e-05) * ((2) * q[i2] * q[i5]) + (-4.359289e-04) * ((2) * q[i3] * q[i5]) + (4.155623e-04) * ((2) * q[i4] * q[i5])
            + (2.164887e-05) * ((3) * q[i5] * q[i5]) + (1.256812e-03) * ((2) * q[i5] * q[i6]) + (1.240186e-03) * ((2) * q[i5] * q[i7])
            + (3.513279e-03) * ((2) * q[i5] * q[i8]) + (4.900245e-04) * ((2) * q[i5] * q[i9]) + (4.921660e-04) * ((2) * q[i5] * q[i10])
            + (-1.414232e-04) * ((2) * q[i5] * q[i11]) + (1.351142e-04) * ((2) * q[i5] * q[i12]) + (-1.570381e-03) * ((2) * q[i5] * q[i15])
            + (-1.588174e-03) * ((2) * q[i5] * q[i16]) + (-7.619498e-04) * ((2) * q[i5] * q[i19]) + (-7.367319e-04) * ((2) * q[i5] * q[i20])
            + (-3.578230e-04) * ((2) * q[i5] * q[i21]) + (3.657845e-04) * ((2) * q[i5] * q[i22]) + (-8.333533e-05) * (q[i6] * q[i6])
            + (8.674062e-05) * (q[i7] * q[i7]) + (4.126801e-06) * (q[i8] * q[i8]) + (1.103610e-03) * (q[i9] * q[i9]) + (-1.104244e-03) * (q[i10] * q[i10])
            + (9.546430e-04) * (q[i11] * q[i11]) + (-1.014978e-03) * (q[i12] * q[i12]) + (-5.327172e-04) * (q[i15] * q[i15])
            + (5.323044e-04) * (q[i16] * q[i16]) + (-6.602141e-04) * (q[i19] * q[i19]) + (6.640720e-04) * (q[i20] * q[i20])
            + (-6.442669e-04) * (q[i21] * q[i21]) + (6.537732e-04) * (q[i22] * q[i22]) + (-6.624666e-06) * (q[i0] * q[i1]) + (-1.340187e-03) * (q[i0] * q[i2])
            + (6.985472e-03) * (q[i0] * q[i3]) + (4.170673e-03) * (q[i0] * q[i4]) + (2.766211e-03) * (q[i0] * q[i6]) + (-4.716225e-04) * (q[i0] * q[i7])
            + (-2.733731e-05) * (q[i0] * q[i8]) + (5.309420e-04) * (q[i0] * q[i9]) + (7.873888e-04) * (q[i0] * q[i10]) + (-1.103549e-03) * (q[i0] * q[i11])
            + (1.435492e-03) * (q[i0] * q[i12]) + (-7.085482e-04) * (q[i0] * q[i15]) + (-2.215502e-04) * (q[i0] * q[i16]) + (2.208192e-05) * (q[i0] * q[i19])
            + (-6.243246e-04) * (q[i0] * q[i20]) + (-5.182215e-04) * (q[i0] * q[i21]) + (6.836454e-04) * (q[i0] * q[i22]) + (1.337284e-03) * (q[i1] * q[i2])
            + (-4.197114e-03) * (q[i1] * q[i3]) + (-7.012161e-03) * (q[i1] * q[i4]) + (-4.758118e-04) * (q[i1] * q[i6]) + (2.736718e-03) * (q[i1] * q[i7])
            + (-5.661351e-06) * (q[i1] * q[i8]) + (7.878519e-04) * (q[i1] * q[i9]) + (5.176418e-04) * (q[i1] * q[i10]) + (-1.469162e-03) * (q[i1] * q[i11])
            + (1.040694e-03) * (q[i1] * q[i12]) + (-2.057310e-04) * (q[i1] * q[i15]) + (-7.129600e-04) * (q[i1] * q[i16]) + (-6.364053e-04) * (q[i1] * q[i19])
            + (3.675125e-06) * (q[i1] * q[i20]) + (-6.627542e-04) * (q[i1] * q[i21]) + (5.078706e-04) * (q[i1] * q[i22]) + (8.190087e-03) * (q[i2] * q[i3])
            + (-8.205304e-03) * (q[i2] * q[i4]) + (-2.330496e-03) * (q[i2] * q[i6]) + (-2.332766e-03) * (q[i2] * q[i7]) + (-1.009762e-02) * (q[i2] * q[i8])
            + (5.265570e-03) * (q[i2] * q[i9]) + (5.205727e-03) * (q[i2] * q[i10]) + (1.832586e-03) * (q[i2] * q[i11]) + (-1.945874e-03) * (q[i2] * q[i12])
            + (-6.397119e-03) * (q[i2] * q[i15]) + (-6.482519e-03) * (q[i2] * q[i16]) + (2.895049e-04) * (q[i2] * q[i19]) + (2.973489e-04) * (q[i2] * q[i20])
            + (8.994120e-04) * (q[i2] * q[i21]) + (-9.153289e-04) * (q[i2] * q[i22]) + (8.074756e-06) * (q[i3] * q[i4]) + (-6.396838e-03) * (q[i3] * q[i6])
            + (2.099809e-03) * (q[i3] * q[i7]) + (-6.112289e-03) * (q[i3] * q[i8]) + (-1.097910e-03) * (q[i3] * q[i9]) + (-3.088242e-04) * (q[i3] * q[i10])
            + (-1.929560e-03) * (q[i3] * q[i11]) + (-1.744299e-03) * (q[i3] * q[i12]) + (-1.171593e-03) * (q[i3] * q[i15]) + (1.103235e-03) * (q[i3] * q[i16])
            + (1.788667e-04) * (q[i3] * q[i19]) + (-1.627474e-03) * (q[i3] * q[i20]) + (2.501059e-04) * (q[i3] * q[i21]) + (-1.042687e-03) * (q[i3] * q[i22])
            + (2.088206e-03) * (q[i4] * q[i6]) + (-6.389809e-03) * (q[i4] * q[i7]) + (-6.133572e-03) * (q[i4] * q[i8]) + (-3.236227e-04) * (q[i4] * q[i9])
            + (-1.076212e-03) * (q[i4] * q[i10]) + (1.749562e-03) * (q[i4] * q[i11]) + (1.903223e-03) * (q[i4] * q[i12]) + (1.105931e-03) * (q[i4] * q[i15])
            + (-1.172226e-03) * (q[i4] * q[i16]) + (-1.633060e-03) * (q[i4] * q[i19]) + (1.739405e-04) * (q[i4] * q[i20]) + (1.047099e-03) * (q[i4] * q[i21])
            + (-2.370044e-04) * (q[i4] * q[i22]) + (1.085024e-05) * (q[i6] * q[i7]) + (1.075929e-03) * (q[i6] * q[i8]) + (1.891117e-03) * (q[i6] * q[i9])
            + (-2.309741e-03) * (q[i6] * q[i10]) + (5.856359e-04) * (q[i6] * q[i11]) + (-1.821301e-03) * (q[i6] * q[i12]) + (1.114467e-03) * (q[i6] * q[i15])
            + (1.956003e-04) * (q[i6] * q[i16]) + (4.064740e-04) * (q[i6] * q[i19]) + (6.454644e-06) * (q[i6] * q[i20]) + (-1.304700e-04) * (q[i6] * q[i21])
            + (1.018957e-04) * (q[i6] * q[i22]) + (-1.076203e-03) * (q[i7] * q[i8]) + (2.316417e-03) * (q[i7] * q[i9]) + (-1.875516e-03) * (q[i7] * q[i10])
            + (-1.821218e-03) * (q[i7] * q[i11]) + (5.918864e-04) * (q[i7] * q[i12]) + (-1.913683e-04) * (q[i7] * q[i15]) + (-1.117954e-03) * (q[i7] * q[i16])
            + (-2.409192e-05) * (q[i7] * q[i19]) + (-3.989730e-04) * (q[i7] * q[i20]) + (9.784063e-05) * (q[i7] * q[i21]) + (-1.198239e-04) * (q[i7] * q[i22])
            + (2.015299e-03) * (q[i8] * q[i9]) + (-2.008047e-03) * (q[i8] * q[i10]) + (1.329208e-03) * (q[i8] * q[i11]) + (1.336449e-03) * (q[i8] * q[i12])
            + (1.798920e-03) * (q[i8] * q[i15]) + (-1.834742e-03) * (q[i8] * q[i16]) + (-6.928540e-05) * (q[i8] * q[i19]) + (7.243788e-05) * (q[i8] * q[i20])
            + (1.485025e-04) * (q[i8] * q[i21]) + (1.510359e-04) * (q[i8] * q[i22]) + (-3.608466e-07) * (q[i9] * q[i10]) + (-6.350464e-04) * (q[i9] * q[i11])
            + (-4.927454e-04) * (q[i9] * q[i12]) + (9.078374e-04) * (q[i9] * q[i15]) + (-2.821584e-04) * (q[i9] * q[i16]) + (5.246723e-04) * (q[i9] * q[i19])
            + (1.785773e-05) * (q[i9] * q[i20]) + (-5.585385e-04) * (q[i9] * q[i21]) + (-2.062318e-04) * (q[i9] * q[i22]) + (-4.946957e-04) * (q[i10] * q[i11])
            + (-6.254464e-04) * (q[i10] * q[i12]) + (2.874019e-04) * (q[i10] * q[i15]) + (-9.036813e-04) * (q[i10] * q[i16])
            + (-1.686565e-05) * (q[i10] * q[i19]) + (-5.345133e-04) * (q[i10] * q[i20]) + (-2.027584e-04) * (q[i10] * q[i21])
            + (-5.498457e-04) * (q[i10] * q[i22]) + (1.301077e-05) * (q[i11] * q[i12]) + (-2.286334e-03) * (q[i11] * q[i15])
            + (-1.740660e-04) * (q[i11] * q[i16]) + (-2.437107e-04) * (q[i11] * q[i19]) + (-2.532027e-04) * (q[i11] * q[i20])
            + (-1.165125e-04) * (q[i11] * q[i21]) + (1.027526e-04) * (q[i11] * q[i22]) + (-1.615513e-04) * (q[i12] * q[i15])
            + (-2.283075e-03) * (q[i12] * q[i16]) + (-2.503387e-04) * (q[i12] * q[i19]) + (-2.516082e-04) * (q[i12] * q[i20])
            + (-1.002124e-04) * (q[i12] * q[i21]) + (1.310858e-04) * (q[i12] * q[i22]) + (-8.659480e-07) * (q[i15] * q[i16])
            + (-7.873197e-04) * (q[i15] * q[i19]) + (-3.741474e-04) * (q[i15] * q[i20]) + (1.155737e-03) * (q[i15] * q[i21])
            + (4.391866e-04) * (q[i15] * q[i22]) + (3.876892e-04) * (q[i16] * q[i19]) + (7.816045e-04) * (q[i16] * q[i20]) + (4.446953e-04) * (q[i16] * q[i21])
            + (1.163210e-03) * (q[i16] * q[i22]) + (2.007867e-06) * (q[i19] * q[i20]) + (8.735954e-04) * (q[i19] * q[i21]) + (-1.434840e-05) * (q[i19] * q[i22])
            + (-7.502974e-06) * (q[i20] * q[i21]) + (8.731083e-04) * (q[i20] * q[i22]) + (-4.321930e-06) * (q[i21] * q[i22]);
   }

   public void getJQy6(double[] q, double[][] JQ)
   {
      JQ[2][i6] = (1.330128e-01) * (1) + (-9.506830e-04) * ((2) * q[i6]) + (6.862044e-03) * (q[i0]) + (-1.099342e-03) * (q[i1]) + (8.590408e-04) * (q[i2])
            + (-1.221548e-02) * (q[i3]) + (-1.650476e-02) * (q[i4]) + (-9.827014e-03) * (q[i5]) + (-3.088028e-03) * (q[i7]) + (1.958398e-03) * (q[i8])
            + (-1.037082e-02) * (q[i9]) + (5.144393e-03) * (q[i10]) + (4.540958e-04) * (q[i11]) + (3.960625e-04) * (q[i12]) + (2.847519e-03) * (q[i15])
            + (5.580052e-03) * (q[i16]) + (-3.673866e-04) * (q[i19]) + (-1.263329e-03) * (q[i20]) + (5.394388e-04) * (q[i21]) + (8.421602e-05) * (q[i22])
            + (-2.368336e-02) * (q[i0] * q[i0]) + (-1.694718e-03) * (q[i1] * q[i1]) + (-1.935695e-03) * (q[i2] * q[i2]) + (1.004020e-02) * (q[i3] * q[i3])
            + (-3.015428e-03) * (q[i4] * q[i4]) + (1.256812e-03) * (q[i5] * q[i5]) + (-4.341833e-03) * ((2) * q[i0] * q[i6])
            + (-8.176774e-04) * ((2) * q[i1] * q[i6]) + (-5.110815e-04) * ((2) * q[i2] * q[i6]) + (-1.290716e-03) * ((2) * q[i3] * q[i6])
            + (9.079333e-04) * ((2) * q[i4] * q[i6]) + (-8.333533e-05) * ((2) * q[i5] * q[i6]) + (-2.142596e-03) * ((3) * q[i6] * q[i6])
            + (-1.413375e-03) * ((2) * q[i6] * q[i7]) + (2.408843e-03) * ((2) * q[i6] * q[i8]) + (-8.203321e-04) * ((2) * q[i6] * q[i9])
            + (-4.106816e-04) * ((2) * q[i6] * q[i10]) + (-4.244388e-04) * ((2) * q[i6] * q[i11]) + (-2.558142e-04) * ((2) * q[i6] * q[i12])
            + (-5.124145e-04) * ((2) * q[i6] * q[i15]) + (-5.331069e-04) * ((2) * q[i6] * q[i16]) + (-8.688893e-04) * ((2) * q[i6] * q[i19])
            + (-2.521222e-04) * ((2) * q[i6] * q[i20]) + (-3.175184e-04) * ((2) * q[i6] * q[i21]) + (4.531219e-04) * ((2) * q[i6] * q[i22])
            + (-1.409038e-03) * (q[i7] * q[i7]) + (1.105897e-03) * (q[i8] * q[i8]) + (-2.560459e-03) * (q[i9] * q[i9]) + (4.343192e-05) * (q[i10] * q[i10])
            + (-6.108288e-04) * (q[i11] * q[i11]) + (8.812535e-05) * (q[i12] * q[i12]) + (6.372224e-05) * (q[i15] * q[i15]) + (7.546963e-04) * (q[i16] * q[i16])
            + (-3.221905e-04) * (q[i19] * q[i19]) + (1.437983e-04) * (q[i20] * q[i20]) + (1.089340e-04) * (q[i21] * q[i21]) + (4.046895e-04) * (q[i22] * q[i22])
            + (5.883122e-03) * (q[i0] * q[i1]) + (-1.042584e-02) * (q[i0] * q[i2]) + (-1.610863e-02) * (q[i0] * q[i3]) + (-4.381152e-04) * (q[i0] * q[i4])
            + (2.766211e-03) * (q[i0] * q[i5]) + (1.146902e-02) * (q[i0] * q[i7]) + (1.381052e-03) * (q[i0] * q[i8]) + (-1.529254e-03) * (q[i0] * q[i9])
            + (2.113707e-03) * (q[i0] * q[i10]) + (-1.800672e-03) * (q[i0] * q[i11]) + (-1.143893e-03) * (q[i0] * q[i12]) + (-6.248782e-05) * (q[i0] * q[i15])
            + (1.073967e-03) * (q[i0] * q[i16]) + (-1.418968e-03) * (q[i0] * q[i19]) + (1.062149e-03) * (q[i0] * q[i20]) + (4.109724e-04) * (q[i0] * q[i21])
            + (2.203350e-04) * (q[i0] * q[i22]) + (6.545506e-04) * (q[i1] * q[i2]) + (3.907965e-03) * (q[i1] * q[i3]) + (3.068551e-03) * (q[i1] * q[i4])
            + (-4.758118e-04) * (q[i1] * q[i5]) + (-1.145363e-02) * (q[i1] * q[i7]) + (2.431201e-04) * (q[i1] * q[i8]) + (-9.416981e-05) * (q[i1] * q[i9])
            + (-2.003019e-03) * (q[i1] * q[i10]) + (-1.167237e-04) * (q[i1] * q[i11]) + (1.400456e-04) * (q[i1] * q[i12]) + (-3.584953e-04) * (q[i1] * q[i15])
            + (-6.491164e-04) * (q[i1] * q[i16]) + (-5.207828e-05) * (q[i1] * q[i19]) + (1.743489e-04) * (q[i1] * q[i20]) + (-5.344456e-04) * (q[i1] * q[i21])
            + (-1.932415e-04) * (q[i1] * q[i22]) + (-3.135415e-03) * (q[i2] * q[i3]) + (1.537182e-03) * (q[i2] * q[i4]) + (-2.330496e-03) * (q[i2] * q[i5])
            + (-7.705269e-06) * (q[i2] * q[i7]) + (7.640783e-03) * (q[i2] * q[i8]) + (9.360254e-04) * (q[i2] * q[i9]) + (-9.274168e-04) * (q[i2] * q[i10])
            + (-1.253649e-03) * (q[i2] * q[i11]) + (3.816430e-05) * (q[i2] * q[i12]) + (-6.280712e-04) * (q[i2] * q[i15]) + (4.216966e-04) * (q[i2] * q[i16])
            + (-2.789274e-04) * (q[i2] * q[i19]) + (8.815289e-04) * (q[i2] * q[i20]) + (4.687912e-04) * (q[i2] * q[i21]) + (-7.638846e-05) * (q[i2] * q[i22])
            + (-3.397184e-03) * (q[i3] * q[i4]) + (-6.396838e-03) * (q[i3] * q[i5]) + (1.714701e-03) * (q[i3] * q[i7]) + (4.703570e-04) * (q[i3] * q[i8])
            + (1.244003e-03) * (q[i3] * q[i9]) + (-1.035583e-03) * (q[i3] * q[i10]) + (-1.374444e-03) * (q[i3] * q[i11]) + (-3.581392e-04) * (q[i3] * q[i12])
            + (-6.172732e-04) * (q[i3] * q[i15]) + (7.018440e-04) * (q[i3] * q[i16]) + (6.625531e-04) * (q[i3] * q[i19]) + (2.179492e-04) * (q[i3] * q[i20])
            + (-7.358712e-06) * (q[i3] * q[i21]) + (1.187888e-04) * (q[i3] * q[i22]) + (2.088206e-03) * (q[i4] * q[i5]) + (-1.735335e-03) * (q[i4] * q[i7])
            + (-1.170429e-03) * (q[i4] * q[i8]) + (1.819040e-03) * (q[i4] * q[i9]) + (1.206311e-03) * (q[i4] * q[i10]) + (-1.265048e-04) * (q[i4] * q[i11])
            + (3.047392e-04) * (q[i4] * q[i12]) + (9.692188e-05) * (q[i4] * q[i15]) + (-7.136097e-04) * (q[i4] * q[i16]) + (-4.620975e-04) * (q[i4] * q[i19])
            + (9.937660e-05) * (q[i4] * q[i20]) + (7.054472e-04) * (q[i4] * q[i21]) + (-1.034558e-04) * (q[i4] * q[i22]) + (1.085024e-05) * (q[i5] * q[i7])
            + (1.075929e-03) * (q[i5] * q[i8]) + (1.891117e-03) * (q[i5] * q[i9]) + (-2.309741e-03) * (q[i5] * q[i10]) + (5.856359e-04) * (q[i5] * q[i11])
            + (-1.821301e-03) * (q[i5] * q[i12]) + (1.114467e-03) * (q[i5] * q[i15]) + (1.956003e-04) * (q[i5] * q[i16]) + (4.064740e-04) * (q[i5] * q[i19])
            + (6.454644e-06) * (q[i5] * q[i20]) + (-1.304700e-04) * (q[i5] * q[i21]) + (1.018957e-04) * (q[i5] * q[i22]) + (-3.365718e-03) * (q[i7] * q[i8])
            + (-2.538085e-04) * (q[i7] * q[i9]) + (-2.458370e-04) * (q[i7] * q[i10]) + (-3.422639e-04) * (q[i7] * q[i11]) + (3.447562e-04) * (q[i7] * q[i12])
            + (4.058894e-04) * (q[i7] * q[i15]) + (4.062233e-04) * (q[i7] * q[i16]) + (7.244586e-04) * (q[i7] * q[i19]) + (7.125068e-04) * (q[i7] * q[i20])
            + (-3.170226e-04) * (q[i7] * q[i21]) + (3.047756e-04) * (q[i7] * q[i22]) + (-4.515444e-04) * (q[i8] * q[i9]) + (-1.345699e-03) * (q[i8] * q[i10])
            + (-3.970293e-04) * (q[i8] * q[i11]) + (-3.383890e-04) * (q[i8] * q[i12]) + (-6.477363e-06) * (q[i8] * q[i15]) + (-2.702556e-04) * (q[i8] * q[i16])
            + (-4.789547e-05) * (q[i8] * q[i19]) + (8.448996e-05) * (q[i8] * q[i20]) + (9.735872e-05) * (q[i8] * q[i21]) + (1.362510e-04) * (q[i8] * q[i22])
            + (-4.095732e-04) * (q[i9] * q[i10]) + (6.405275e-05) * (q[i9] * q[i11]) + (1.787217e-04) * (q[i9] * q[i12]) + (-5.165831e-04) * (q[i9] * q[i15])
            + (-2.863813e-04) * (q[i9] * q[i16]) + (7.655790e-05) * (q[i9] * q[i19]) + (2.288140e-04) * (q[i9] * q[i20]) + (-4.923126e-04) * (q[i9] * q[i21])
            + (-7.035101e-05) * (q[i9] * q[i22]) + (-3.145454e-04) * (q[i10] * q[i11]) + (-1.802055e-04) * (q[i10] * q[i12])
            + (-4.340690e-05) * (q[i10] * q[i15]) + (5.037950e-04) * (q[i10] * q[i16]) + (-1.027714e-04) * (q[i10] * q[i19])
            + (3.982749e-04) * (q[i10] * q[i20]) + (-1.984213e-04) * (q[i10] * q[i21]) + (1.941054e-04) * (q[i10] * q[i22])
            + (-4.465450e-05) * (q[i11] * q[i12]) + (-3.946318e-05) * (q[i11] * q[i15]) + (-2.434706e-05) * (q[i11] * q[i16])
            + (-1.572757e-04) * (q[i11] * q[i19]) + (2.184652e-04) * (q[i11] * q[i20]) + (-6.219832e-05) * (q[i11] * q[i21])
            + (-1.467622e-04) * (q[i11] * q[i22]) + (-7.233044e-05) * (q[i12] * q[i15]) + (-1.767328e-04) * (q[i12] * q[i16])
            + (-2.336362e-04) * (q[i12] * q[i19]) + (1.541868e-04) * (q[i12] * q[i20]) + (2.803087e-04) * (q[i12] * q[i21])
            + (-2.579998e-04) * (q[i12] * q[i22]) + (-6.177311e-05) * (q[i15] * q[i16]) + (-4.611163e-04) * (q[i15] * q[i19])
            + (-3.996412e-04) * (q[i15] * q[i20]) + (7.036249e-05) * (q[i15] * q[i21]) + (7.993119e-05) * (q[i15] * q[i22]) + (4.438298e-04) * (q[i16] * q[i19])
            + (1.861985e-04) * (q[i16] * q[i20]) + (-4.163240e-05) * (q[i16] * q[i21]) + (-5.419059e-04) * (q[i16] * q[i22])
            + (-3.501248e-04) * (q[i19] * q[i20]) + (4.017529e-04) * (q[i19] * q[i21]) + (-1.984777e-04) * (q[i19] * q[i22])
            + (5.300866e-05) * (q[i20] * q[i21]) + (2.983694e-06) * (q[i20] * q[i22]) + (1.353639e-04) * (q[i21] * q[i22]);
   }

   public void getJQy7(double[] q, double[][] JQ)
   {
      JQ[2][i7] = (1.322580e-01) * (1) + (-9.205158e-04) * ((2) * q[i7]) + (1.114285e-03) * (q[i0]) + (-6.925039e-03) * (q[i1]) + (-8.558570e-04) * (q[i2])
            + (1.650458e-02) * (q[i3]) + (1.214653e-02) * (q[i4]) + (9.830779e-03) * (q[i5]) + (-3.088028e-03) * (q[i6]) + (1.917137e-03) * (q[i8])
            + (5.192358e-03) * (q[i9]) + (-1.027846e-02) * (q[i10]) + (-3.654866e-04) * (q[i11]) + (-4.179029e-04) * (q[i12]) + (5.496503e-03) * (q[i15])
            + (2.848602e-03) * (q[i16]) + (-1.261602e-03) * (q[i19]) + (-3.554908e-04) * (q[i20]) + (-8.707822e-05) * (q[i21]) + (-5.215120e-04) * (q[i22])
            + (-1.687363e-03) * (q[i0] * q[i0]) + (-2.364088e-02) * (q[i1] * q[i1]) + (-1.946621e-03) * (q[i2] * q[i2]) + (-3.008768e-03) * (q[i3] * q[i3])
            + (1.006275e-02) * (q[i4] * q[i4]) + (1.240186e-03) * (q[i5] * q[i5]) + (-1.413375e-03) * (q[i6] * q[i6]) + (8.112818e-04) * ((2) * q[i0] * q[i7])
            + (4.321724e-03) * ((2) * q[i1] * q[i7]) + (5.175746e-04) * ((2) * q[i2] * q[i7]) + (-9.041409e-04) * ((2) * q[i3] * q[i7])
            + (1.320553e-03) * ((2) * q[i4] * q[i7]) + (8.674062e-05) * ((2) * q[i5] * q[i7]) + (-1.409038e-03) * ((2) * q[i6] * q[i7])
            + (-2.152157e-03) * ((3) * q[i7] * q[i7]) + (2.430478e-03) * ((2) * q[i7] * q[i8]) + (-4.139885e-04) * ((2) * q[i7] * q[i9])
            + (-8.165828e-04) * ((2) * q[i7] * q[i10]) + (2.617575e-04) * ((2) * q[i7] * q[i11]) + (4.257258e-04) * ((2) * q[i7] * q[i12])
            + (-5.255415e-04) * ((2) * q[i7] * q[i15]) + (-5.192656e-04) * ((2) * q[i7] * q[i16]) + (-2.492046e-04) * ((2) * q[i7] * q[i19])
            + (-8.649150e-04) * ((2) * q[i7] * q[i20]) + (-4.481974e-04) * ((2) * q[i7] * q[i21]) + (3.133449e-04) * ((2) * q[i7] * q[i22])
            + (1.110014e-03) * (q[i8] * q[i8]) + (4.640644e-05) * (q[i9] * q[i9]) + (-2.533437e-03) * (q[i10] * q[i10]) + (8.434333e-05) * (q[i11] * q[i11])
            + (-6.252605e-04) * (q[i12] * q[i12]) + (7.534292e-04) * (q[i15] * q[i15]) + (6.457506e-05) * (q[i16] * q[i16]) + (1.443314e-04) * (q[i19] * q[i19])
            + (-3.164190e-04) * (q[i20] * q[i20]) + (4.041422e-04) * (q[i21] * q[i21]) + (1.068850e-04) * (q[i22] * q[i22]) + (5.865579e-03) * (q[i0] * q[i1])
            + (6.692984e-04) * (q[i0] * q[i2]) + (3.069103e-03) * (q[i0] * q[i3]) + (3.905466e-03) * (q[i0] * q[i4]) + (-4.716225e-04) * (q[i0] * q[i5])
            + (1.146902e-02) * (q[i0] * q[i6]) + (-2.645940e-04) * (q[i0] * q[i8]) + (2.004623e-03) * (q[i0] * q[i9]) + (8.376530e-05) * (q[i0] * q[i10])
            + (1.402177e-04) * (q[i0] * q[i11]) + (-1.169007e-04) * (q[i0] * q[i12]) + (6.449001e-04) * (q[i0] * q[i15]) + (3.597124e-04) * (q[i0] * q[i16])
            + (-1.804753e-04) * (q[i0] * q[i19]) + (6.153951e-05) * (q[i0] * q[i20]) + (-2.018885e-04) * (q[i0] * q[i21]) + (-5.363128e-04) * (q[i0] * q[i22])
            + (-1.043075e-02) * (q[i1] * q[i2]) + (-4.458325e-04) * (q[i1] * q[i3]) + (-1.615989e-02) * (q[i1] * q[i4]) + (2.736718e-03) * (q[i1] * q[i5])
            + (-1.145363e-02) * (q[i1] * q[i6]) + (-1.377890e-03) * (q[i1] * q[i8]) + (-2.129486e-03) * (q[i1] * q[i9]) + (1.539917e-03) * (q[i1] * q[i10])
            + (-1.131840e-03) * (q[i1] * q[i11]) + (-1.800183e-03) * (q[i1] * q[i12]) + (-1.062855e-03) * (q[i1] * q[i15]) + (7.710115e-05) * (q[i1] * q[i16])
            + (-1.066193e-03) * (q[i1] * q[i19]) + (1.409631e-03) * (q[i1] * q[i20]) + (2.247653e-04) * (q[i1] * q[i21]) + (4.137149e-04) * (q[i1] * q[i22])
            + (1.534195e-03) * (q[i2] * q[i3]) + (-3.158159e-03) * (q[i2] * q[i4]) + (-2.332766e-03) * (q[i2] * q[i5]) + (-7.705269e-06) * (q[i2] * q[i6])
            + (-7.652251e-03) * (q[i2] * q[i8]) + (9.302299e-04) * (q[i2] * q[i9]) + (-9.301464e-04) * (q[i2] * q[i10]) + (5.285828e-05) * (q[i2] * q[i11])
            + (-1.292311e-03) * (q[i2] * q[i12]) + (-4.168032e-04) * (q[i2] * q[i15]) + (6.447545e-04) * (q[i2] * q[i16]) + (-8.905544e-04) * (q[i2] * q[i19])
            + (2.646359e-04) * (q[i2] * q[i20]) + (-7.990804e-05) * (q[i2] * q[i21]) + (4.671243e-04) * (q[i2] * q[i22]) + (-3.358738e-03) * (q[i3] * q[i4])
            + (2.099809e-03) * (q[i3] * q[i5]) + (1.714701e-03) * (q[i3] * q[i6]) + (1.178482e-03) * (q[i3] * q[i8]) + (-1.221219e-03) * (q[i3] * q[i9])
            + (-1.806832e-03) * (q[i3] * q[i10]) + (2.956707e-04) * (q[i3] * q[i11]) + (-1.233349e-04) * (q[i3] * q[i12]) + (7.007633e-04) * (q[i3] * q[i15])
            + (-9.436389e-05) * (q[i3] * q[i16]) + (-9.165213e-05) * (q[i3] * q[i19]) + (4.630918e-04) * (q[i3] * q[i20]) + (-9.779688e-05) * (q[i3] * q[i21])
            + (7.096689e-04) * (q[i3] * q[i22]) + (-6.389809e-03) * (q[i4] * q[i5]) + (-1.735335e-03) * (q[i4] * q[i6]) + (-4.796848e-04) * (q[i4] * q[i8])
            + (1.030994e-03) * (q[i4] * q[i9]) + (-1.225452e-03) * (q[i4] * q[i10]) + (-3.746156e-04) * (q[i4] * q[i11]) + (-1.389476e-03) * (q[i4] * q[i12])
            + (-6.976053e-04) * (q[i4] * q[i15]) + (6.232834e-04) * (q[i4] * q[i16]) + (-2.170230e-04) * (q[i4] * q[i19]) + (-6.638632e-04) * (q[i4] * q[i20])
            + (1.071998e-04) * (q[i4] * q[i21]) + (-5.496944e-06) * (q[i4] * q[i22]) + (1.085024e-05) * (q[i5] * q[i6]) + (-1.076203e-03) * (q[i5] * q[i8])
            + (2.316417e-03) * (q[i5] * q[i9]) + (-1.875516e-03) * (q[i5] * q[i10]) + (-1.821218e-03) * (q[i5] * q[i11]) + (5.918864e-04) * (q[i5] * q[i12])
            + (-1.913683e-04) * (q[i5] * q[i15]) + (-1.117954e-03) * (q[i5] * q[i16]) + (-2.409192e-05) * (q[i5] * q[i19]) + (-3.989730e-04) * (q[i5] * q[i20])
            + (9.784063e-05) * (q[i5] * q[i21]) + (-1.198239e-04) * (q[i5] * q[i22]) + (-3.365718e-03) * (q[i6] * q[i8]) + (-2.538085e-04) * (q[i6] * q[i9])
            + (-2.458370e-04) * (q[i6] * q[i10]) + (-3.422639e-04) * (q[i6] * q[i11]) + (3.447562e-04) * (q[i6] * q[i12]) + (4.058894e-04) * (q[i6] * q[i15])
            + (4.062233e-04) * (q[i6] * q[i16]) + (7.244586e-04) * (q[i6] * q[i19]) + (7.125068e-04) * (q[i6] * q[i20]) + (-3.170226e-04) * (q[i6] * q[i21])
            + (3.047756e-04) * (q[i6] * q[i22]) + (-1.356350e-03) * (q[i8] * q[i9]) + (-4.340547e-04) * (q[i8] * q[i10]) + (3.746026e-04) * (q[i8] * q[i11])
            + (3.996764e-04) * (q[i8] * q[i12]) + (-2.735794e-04) * (q[i8] * q[i15]) + (-7.646881e-06) * (q[i8] * q[i16]) + (8.224123e-05) * (q[i8] * q[i19])
            + (-3.455847e-05) * (q[i8] * q[i20]) + (-1.377254e-04) * (q[i8] * q[i21]) + (-8.658386e-05) * (q[i8] * q[i22]) + (-4.172767e-04) * (q[i9] * q[i10])
            + (1.817600e-04) * (q[i9] * q[i11]) + (3.201946e-04) * (q[i9] * q[i12]) + (5.054034e-04) * (q[i9] * q[i15]) + (-3.701206e-05) * (q[i9] * q[i16])
            + (4.038065e-04) * (q[i9] * q[i19]) + (-1.078364e-04) * (q[i9] * q[i20]) + (-1.936129e-04) * (q[i9] * q[i21]) + (1.932345e-04) * (q[i9] * q[i22])
            + (-1.821436e-04) * (q[i10] * q[i11]) + (-7.359981e-05) * (q[i10] * q[i12]) + (-2.776757e-04) * (q[i10] * q[i15])
            + (-5.209704e-04) * (q[i10] * q[i16]) + (2.316224e-04) * (q[i10] * q[i19]) + (7.333906e-05) * (q[i10] * q[i20]) + (7.155125e-05) * (q[i10] * q[i21])
            + (4.919634e-04) * (q[i10] * q[i22]) + (-5.278493e-05) * (q[i11] * q[i12]) + (1.699183e-04) * (q[i11] * q[i15]) + (7.544601e-05) * (q[i11] * q[i16])
            + (-1.569050e-04) * (q[i11] * q[i19]) + (2.369777e-04) * (q[i11] * q[i20]) + (-2.603405e-04) * (q[i11] * q[i21])
            + (2.806518e-04) * (q[i11] * q[i22]) + (2.746136e-05) * (q[i12] * q[i15]) + (4.293758e-05) * (q[i12] * q[i16]) + (-2.168496e-04) * (q[i12] * q[i19])
            + (1.536198e-04) * (q[i12] * q[i20]) + (-1.445232e-04) * (q[i12] * q[i21]) + (-5.375323e-05) * (q[i12] * q[i22])
            + (-6.415069e-05) * (q[i15] * q[i16]) + (1.873814e-04) * (q[i15] * q[i19]) + (4.450782e-04) * (q[i15] * q[i20]) + (5.336785e-04) * (q[i15] * q[i21])
            + (4.000195e-05) * (q[i15] * q[i22]) + (-3.961685e-04) * (q[i16] * q[i19]) + (-4.551602e-04) * (q[i16] * q[i20])
            + (-7.387613e-05) * (q[i16] * q[i21]) + (-6.504092e-05) * (q[i16] * q[i22]) + (-3.432400e-04) * (q[i19] * q[i20])
            + (-3.889145e-06) * (q[i19] * q[i21]) + (-5.439509e-05) * (q[i19] * q[i22]) + (2.009673e-04) * (q[i20] * q[i21])
            + (-3.973034e-04) * (q[i20] * q[i22]) + (1.395011e-04) * (q[i21] * q[i22]);
   }

   public void getJQy8(double[] q, double[][] JQ)
   {
      JQ[2][i8] = (1.077327e-01) * (1) + (-1.279854e-04) * ((2) * q[i8]) + (-1.908835e-03) * (q[i0]) + (1.898408e-03) * (q[i1]) + (-2.238345e-05) * (q[i2])
            + (1.532152e-03) * (q[i3]) + (-1.465220e-03) * (q[i4]) + (-1.302839e-05) * (q[i5]) + (1.958398e-03) * (q[i6]) + (1.917137e-03) * (q[i7])
            + (3.952787e-03) * (q[i9]) + (3.915407e-03) * (q[i10]) + (-5.004001e-04) * (q[i11]) + (3.912603e-04) * (q[i12]) + (-8.566462e-03) * (q[i15])
            + (-8.621797e-03) * (q[i16]) + (-5.022812e-04) * (q[i19]) + (-4.814287e-04) * (q[i20]) + (-2.099477e-03) * (q[i21]) + (2.088474e-03) * (q[i22])
            + (3.544038e-04) * (q[i0] * q[i0]) + (3.581257e-04) * (q[i1] * q[i1]) + (-3.025856e-02) * (q[i2] * q[i2]) + (3.782819e-03) * (q[i3] * q[i3])
            + (3.800462e-03) * (q[i4] * q[i4]) + (3.513279e-03) * (q[i5] * q[i5]) + (2.408843e-03) * (q[i6] * q[i6]) + (2.430478e-03) * (q[i7] * q[i7])
            + (-1.456665e-03) * ((2) * q[i0] * q[i8]) + (1.468737e-03) * ((2) * q[i1] * q[i8]) + (2.545932e-05) * ((2) * q[i2] * q[i8])
            + (-1.144956e-03) * ((2) * q[i3] * q[i8]) + (1.125159e-03) * ((2) * q[i4] * q[i8]) + (4.126801e-06) * ((2) * q[i5] * q[i8])
            + (1.105897e-03) * ((2) * q[i6] * q[i8]) + (1.110014e-03) * ((2) * q[i7] * q[i8]) + (-4.060335e-03) * ((3) * q[i8] * q[i8])
            + (2.916677e-04) * ((2) * q[i8] * q[i9]) + (2.897784e-04) * ((2) * q[i8] * q[i10]) + (5.352001e-04) * ((2) * q[i8] * q[i11])
            + (-5.628161e-04) * ((2) * q[i8] * q[i12]) + (2.523868e-04) * ((2) * q[i8] * q[i15]) + (2.591024e-04) * ((2) * q[i8] * q[i16])
            + (6.735998e-05) * ((2) * q[i8] * q[i19]) + (5.945936e-05) * ((2) * q[i8] * q[i20]) + (-4.600819e-04) * ((2) * q[i8] * q[i21])
            + (4.633328e-04) * ((2) * q[i8] * q[i22]) + (6.011000e-04) * (q[i9] * q[i9]) + (5.970276e-04) * (q[i10] * q[i10])
            + (1.581777e-03) * (q[i11] * q[i11]) + (1.612018e-03) * (q[i12] * q[i12]) + (1.903655e-04) * (q[i15] * q[i15]) + (2.011981e-04) * (q[i16] * q[i16])
            + (-2.638921e-04) * (q[i19] * q[i19]) + (-2.617898e-04) * (q[i20] * q[i20]) + (-8.752838e-04) * (q[i21] * q[i21])
            + (-8.719102e-04) * (q[i22] * q[i22]) + (2.177889e-03) * (q[i0] * q[i1]) + (-4.592705e-03) * (q[i0] * q[i2]) + (-2.993308e-04) * (q[i0] * q[i3])
            + (-1.282934e-03) * (q[i0] * q[i4]) + (-2.733731e-05) * (q[i0] * q[i5]) + (1.381052e-03) * (q[i0] * q[i6]) + (-2.645940e-04) * (q[i0] * q[i7])
            + (1.232822e-03) * (q[i0] * q[i9]) + (1.771793e-04) * (q[i0] * q[i10]) + (-5.257208e-04) * (q[i0] * q[i11]) + (6.450441e-06) * (q[i0] * q[i12])
            + (-2.445200e-04) * (q[i0] * q[i15]) + (6.730540e-04) * (q[i0] * q[i16]) + (1.101120e-04) * (q[i0] * q[i19]) + (6.534280e-04) * (q[i0] * q[i20])
            + (-1.374253e-04) * (q[i0] * q[i21]) + (3.502593e-05) * (q[i0] * q[i22]) + (-4.582863e-03) * (q[i1] * q[i2]) + (-1.308695e-03) * (q[i1] * q[i3])
            + (-2.867031e-04) * (q[i1] * q[i4]) + (-5.661351e-06) * (q[i1] * q[i5]) + (2.431201e-04) * (q[i1] * q[i6]) + (-1.377890e-03) * (q[i1] * q[i7])
            + (-1.719627e-04) * (q[i1] * q[i9]) + (-1.222720e-03) * (q[i1] * q[i10]) + (3.963438e-05) * (q[i1] * q[i11]) + (-5.528362e-04) * (q[i1] * q[i12])
            + (-6.650239e-04) * (q[i1] * q[i15]) + (2.444242e-04) * (q[i1] * q[i16]) + (-6.624098e-04) * (q[i1] * q[i19]) + (-1.026070e-04) * (q[i1] * q[i20])
            + (2.896848e-05) * (q[i1] * q[i21]) + (-1.338097e-04) * (q[i1] * q[i22]) + (1.040342e-03) * (q[i2] * q[i3]) + (1.062334e-03) * (q[i2] * q[i4])
            + (-1.009762e-02) * (q[i2] * q[i5]) + (7.640783e-03) * (q[i2] * q[i6]) + (-7.652251e-03) * (q[i2] * q[i7]) + (2.822017e-03) * (q[i2] * q[i9])
            + (-2.806527e-03) * (q[i2] * q[i10]) + (4.744048e-03) * (q[i2] * q[i11]) + (4.759668e-03) * (q[i2] * q[i12]) + (5.980369e-05) * (q[i2] * q[i15])
            + (-8.999536e-05) * (q[i2] * q[i16]) + (-1.453997e-05) * (q[i2] * q[i19]) + (2.125620e-05) * (q[i2] * q[i20]) + (-2.847479e-04) * (q[i2] * q[i21])
            + (-2.826212e-04) * (q[i2] * q[i22]) + (-1.285209e-03) * (q[i3] * q[i4]) + (-6.112289e-03) * (q[i3] * q[i5]) + (4.703570e-04) * (q[i3] * q[i6])
            + (1.178482e-03) * (q[i3] * q[i7]) + (-1.304794e-03) * (q[i3] * q[i9]) + (7.952767e-04) * (q[i3] * q[i10]) + (1.033786e-03) * (q[i3] * q[i11])
            + (8.022163e-04) * (q[i3] * q[i12]) + (-1.804215e-03) * (q[i3] * q[i15]) + (9.682878e-04) * (q[i3] * q[i16]) + (2.773115e-05) * (q[i3] * q[i19])
            + (8.255406e-04) * (q[i3] * q[i20]) + (1.261761e-04) * (q[i3] * q[i21]) + (6.748273e-04) * (q[i3] * q[i22]) + (-6.133572e-03) * (q[i4] * q[i5])
            + (-1.170429e-03) * (q[i4] * q[i6]) + (-4.796848e-04) * (q[i4] * q[i7]) + (-7.931301e-04) * (q[i4] * q[i9]) + (1.294842e-03) * (q[i4] * q[i10])
            + (7.959779e-04) * (q[i4] * q[i11]) + (1.067223e-03) * (q[i4] * q[i12]) + (-9.611609e-04) * (q[i4] * q[i15]) + (1.829801e-03) * (q[i4] * q[i16])
            + (-8.232891e-04) * (q[i4] * q[i19]) + (-3.856416e-05) * (q[i4] * q[i20]) + (6.825732e-04) * (q[i4] * q[i21]) + (1.317616e-04) * (q[i4] * q[i22])
            + (1.075929e-03) * (q[i5] * q[i6]) + (-1.076203e-03) * (q[i5] * q[i7]) + (2.015299e-03) * (q[i5] * q[i9]) + (-2.008047e-03) * (q[i5] * q[i10])
            + (1.329208e-03) * (q[i5] * q[i11]) + (1.336449e-03) * (q[i5] * q[i12]) + (1.798920e-03) * (q[i5] * q[i15]) + (-1.834742e-03) * (q[i5] * q[i16])
            + (-6.928540e-05) * (q[i5] * q[i19]) + (7.243788e-05) * (q[i5] * q[i20]) + (1.485025e-04) * (q[i5] * q[i21]) + (1.510359e-04) * (q[i5] * q[i22])
            + (-3.365718e-03) * (q[i6] * q[i7]) + (-4.515444e-04) * (q[i6] * q[i9]) + (-1.345699e-03) * (q[i6] * q[i10]) + (-3.970293e-04) * (q[i6] * q[i11])
            + (-3.383890e-04) * (q[i6] * q[i12]) + (-6.477363e-06) * (q[i6] * q[i15]) + (-2.702556e-04) * (q[i6] * q[i16]) + (-4.789547e-05) * (q[i6] * q[i19])
            + (8.448996e-05) * (q[i6] * q[i20]) + (9.735872e-05) * (q[i6] * q[i21]) + (1.362510e-04) * (q[i6] * q[i22]) + (-1.356350e-03) * (q[i7] * q[i9])
            + (-4.340547e-04) * (q[i7] * q[i10]) + (3.746026e-04) * (q[i7] * q[i11]) + (3.996764e-04) * (q[i7] * q[i12]) + (-2.735794e-04) * (q[i7] * q[i15])
            + (-7.646881e-06) * (q[i7] * q[i16]) + (8.224123e-05) * (q[i7] * q[i19]) + (-3.455847e-05) * (q[i7] * q[i20]) + (-1.377254e-04) * (q[i7] * q[i21])
            + (-8.658386e-05) * (q[i7] * q[i22]) + (-1.062765e-04) * (q[i9] * q[i10]) + (-3.790616e-04) * (q[i9] * q[i11]) + (-2.503545e-04) * (q[i9] * q[i12])
            + (-5.608626e-04) * (q[i9] * q[i15]) + (2.161125e-05) * (q[i9] * q[i16]) + (-2.332290e-04) * (q[i9] * q[i19]) + (2.934882e-04) * (q[i9] * q[i20])
            + (1.329135e-04) * (q[i9] * q[i21]) + (-1.614691e-04) * (q[i9] * q[i22]) + (2.480893e-04) * (q[i10] * q[i11]) + (3.753363e-04) * (q[i10] * q[i12])
            + (2.855460e-05) * (q[i10] * q[i15]) + (-5.650428e-04) * (q[i10] * q[i16]) + (2.962574e-04) * (q[i10] * q[i19])
            + (-2.234525e-04) * (q[i10] * q[i20]) + (1.593766e-04) * (q[i10] * q[i21]) + (-1.333644e-04) * (q[i10] * q[i22])
            + (-5.578371e-04) * (q[i11] * q[i12]) + (-7.542863e-04) * (q[i11] * q[i15]) + (3.434920e-05) * (q[i11] * q[i16])
            + (4.733749e-04) * (q[i11] * q[i19]) + (5.237842e-04) * (q[i11] * q[i20]) + (-1.172496e-04) * (q[i11] * q[i21]) + (1.615886e-06) * (q[i11] * q[i22])
            + (-4.209219e-05) * (q[i12] * q[i15]) + (7.420413e-04) * (q[i12] * q[i16]) + (-5.292212e-04) * (q[i12] * q[i19])
            + (-4.641388e-04) * (q[i12] * q[i20]) + (4.989940e-06) * (q[i12] * q[i21]) + (-1.251570e-04) * (q[i12] * q[i22])
            + (-3.166315e-04) * (q[i15] * q[i16]) + (4.024530e-04) * (q[i15] * q[i19]) + (-6.814829e-05) * (q[i15] * q[i20])
            + (-1.336037e-03) * (q[i15] * q[i21]) + (8.554276e-05) * (q[i15] * q[i22]) + (-7.197157e-05) * (q[i16] * q[i19])
            + (4.059137e-04) * (q[i16] * q[i20]) + (-8.746154e-05) * (q[i16] * q[i21]) + (1.347250e-03) * (q[i16] * q[i22]) + (5.735726e-04) * (q[i19] * q[i20])
            + (-7.246433e-04) * (q[i19] * q[i21]) + (-2.846715e-04) * (q[i19] * q[i22]) + (2.859612e-04) * (q[i20] * q[i21])
            + (7.191502e-04) * (q[i20] * q[i22]) + (-9.113027e-05) * (q[i21] * q[i22]);
   }

   public void getJQy9(double[] q, double[][] JQ)
   {
      JQ[2][i9] = (5.828438e-02) * (1) + (-3.456945e-03) * ((2) * q[i9]) + (4.586587e-04) * (q[i0]) + (-6.650083e-04) * (q[i1]) + (2.099167e-03) * (q[i2])
            + (-5.205169e-03) * (q[i3]) + (-4.747030e-03) * (q[i4]) + (-5.307797e-03) * (q[i5]) + (-1.037082e-02) * (q[i6]) + (5.192358e-03) * (q[i7])
            + (3.952787e-03) * (q[i8]) + (3.393551e-03) * (q[i10]) + (-5.751373e-04) * (q[i11]) + (1.103258e-04) * (q[i12]) + (1.040421e-03) * (q[i15])
            + (1.937559e-03) * (q[i16]) + (1.589063e-04) * (q[i19]) + (-1.205392e-04) * (q[i20]) + (-6.516309e-04) * (q[i21]) + (5.353539e-04) * (q[i22])
            + (-3.140358e-03) * (q[i0] * q[i0]) + (-7.962768e-05) * (q[i1] * q[i1]) + (-1.384378e-03) * (q[i2] * q[i2]) + (1.625013e-03) * (q[i3] * q[i3])
            + (-1.062881e-03) * (q[i4] * q[i4]) + (4.900245e-04) * (q[i5] * q[i5]) + (-8.203321e-04) * (q[i6] * q[i6]) + (-4.139885e-04) * (q[i7] * q[i7])
            + (2.916677e-04) * (q[i8] * q[i8]) + (-1.972233e-04) * ((2) * q[i0] * q[i9]) + (-3.188681e-04) * ((2) * q[i1] * q[i9])
            + (5.600260e-04) * ((2) * q[i2] * q[i9]) + (8.883241e-04) * ((2) * q[i3] * q[i9]) + (8.941807e-04) * ((2) * q[i4] * q[i9])
            + (1.103610e-03) * ((2) * q[i5] * q[i9]) + (-2.560459e-03) * ((2) * q[i6] * q[i9]) + (4.640644e-05) * ((2) * q[i7] * q[i9])
            + (6.011000e-04) * ((2) * q[i8] * q[i9]) + (-9.366195e-04) * ((3) * q[i9] * q[i9]) + (-2.172592e-04) * ((2) * q[i9] * q[i10])
            + (3.615142e-05) * ((2) * q[i9] * q[i11]) + (3.414539e-04) * ((2) * q[i9] * q[i12]) + (-9.578398e-05) * ((2) * q[i9] * q[i15])
            + (-2.517296e-04) * ((2) * q[i9] * q[i16]) + (5.892796e-05) * ((2) * q[i9] * q[i19]) + (7.640899e-05) * ((2) * q[i9] * q[i20])
            + (7.150478e-05) * ((2) * q[i9] * q[i21]) + (-1.594535e-04) * ((2) * q[i9] * q[i22]) + (-2.201108e-04) * (q[i10] * q[i10])
            + (-1.448847e-04) * (q[i11] * q[i11]) + (6.421360e-06) * (q[i12] * q[i12]) + (-1.399477e-04) * (q[i15] * q[i15])
            + (-1.199851e-04) * (q[i16] * q[i16]) + (-3.016804e-05) * (q[i19] * q[i19]) + (-1.238043e-04) * (q[i20] * q[i20])
            + (2.229091e-06) * (q[i21] * q[i21]) + (7.763999e-06) * (q[i22] * q[i22]) + (1.000905e-03) * (q[i0] * q[i1]) + (-1.144177e-03) * (q[i0] * q[i2])
            + (1.178320e-02) * (q[i0] * q[i3]) + (3.261963e-04) * (q[i0] * q[i4]) + (5.309420e-04) * (q[i0] * q[i5]) + (-1.529254e-03) * (q[i0] * q[i6])
            + (2.004623e-03) * (q[i0] * q[i7]) + (1.232822e-03) * (q[i0] * q[i8]) + (2.411114e-04) * (q[i0] * q[i10]) + (8.550443e-05) * (q[i0] * q[i11])
            + (-4.645134e-04) * (q[i0] * q[i12]) + (-3.216631e-04) * (q[i0] * q[i15]) + (3.160296e-04) * (q[i0] * q[i16]) + (-2.978277e-04) * (q[i0] * q[i19])
            + (6.484969e-04) * (q[i0] * q[i20]) + (-1.291727e-04) * (q[i0] * q[i21]) + (-1.041448e-04) * (q[i0] * q[i22]) + (1.919334e-03) * (q[i1] * q[i2])
            + (-7.588419e-04) * (q[i1] * q[i3]) + (-3.326402e-03) * (q[i1] * q[i4]) + (7.878519e-04) * (q[i1] * q[i5]) + (-9.416981e-05) * (q[i1] * q[i6])
            + (-2.129486e-03) * (q[i1] * q[i7]) + (-1.719627e-04) * (q[i1] * q[i8]) + (-2.445345e-04) * (q[i1] * q[i10]) + (4.395493e-04) * (q[i1] * q[i11])
            + (9.939138e-04) * (q[i1] * q[i12]) + (3.954244e-04) * (q[i1] * q[i15]) + (-3.384103e-04) * (q[i1] * q[i16]) + (-3.654445e-05) * (q[i1] * q[i19])
            + (-3.530560e-05) * (q[i1] * q[i20]) + (-2.388953e-04) * (q[i1] * q[i21]) + (-5.476873e-05) * (q[i1] * q[i22]) + (2.559771e-03) * (q[i2] * q[i3])
            + (-2.018845e-04) * (q[i2] * q[i4]) + (5.265570e-03) * (q[i2] * q[i5]) + (9.360254e-04) * (q[i2] * q[i6]) + (9.302299e-04) * (q[i2] * q[i7])
            + (2.822017e-03) * (q[i2] * q[i8]) + (2.633729e-07) * (q[i2] * q[i10]) + (-2.112597e-04) * (q[i2] * q[i11]) + (2.393001e-04) * (q[i2] * q[i12])
            + (2.405074e-04) * (q[i2] * q[i15]) + (-2.254603e-04) * (q[i2] * q[i16]) + (-3.864767e-04) * (q[i2] * q[i19]) + (-1.316529e-07) * (q[i2] * q[i20])
            + (-2.489416e-04) * (q[i2] * q[i21]) + (-2.295709e-04) * (q[i2] * q[i22]) + (-1.819951e-03) * (q[i3] * q[i4]) + (-1.097910e-03) * (q[i3] * q[i5])
            + (1.244003e-03) * (q[i3] * q[i6]) + (-1.221219e-03) * (q[i3] * q[i7]) + (-1.304794e-03) * (q[i3] * q[i8]) + (-4.577861e-04) * (q[i3] * q[i10])
            + (2.531383e-04) * (q[i3] * q[i11]) + (5.033502e-04) * (q[i3] * q[i12]) + (-3.873931e-04) * (q[i3] * q[i15]) + (-6.382520e-04) * (q[i3] * q[i16])
            + (1.814252e-04) * (q[i3] * q[i19]) + (-2.641220e-05) * (q[i3] * q[i20]) + (2.187432e-04) * (q[i3] * q[i21]) + (2.044403e-04) * (q[i3] * q[i22])
            + (-3.236227e-04) * (q[i4] * q[i5]) + (1.819040e-03) * (q[i4] * q[i6]) + (1.030994e-03) * (q[i4] * q[i7]) + (-7.931301e-04) * (q[i4] * q[i8])
            + (4.573698e-04) * (q[i4] * q[i10]) + (3.937067e-04) * (q[i4] * q[i11]) + (-8.228550e-04) * (q[i4] * q[i12]) + (-4.485047e-04) * (q[i4] * q[i15])
            + (3.978682e-04) * (q[i4] * q[i16]) + (-4.595379e-04) * (q[i4] * q[i19]) + (-2.947117e-04) * (q[i4] * q[i20]) + (-1.234635e-04) * (q[i4] * q[i21])
            + (-8.447726e-05) * (q[i4] * q[i22]) + (1.891117e-03) * (q[i5] * q[i6]) + (2.316417e-03) * (q[i5] * q[i7]) + (2.015299e-03) * (q[i5] * q[i8])
            + (-3.608466e-07) * (q[i5] * q[i10]) + (-6.350464e-04) * (q[i5] * q[i11]) + (-4.927454e-04) * (q[i5] * q[i12]) + (9.078374e-04) * (q[i5] * q[i15])
            + (-2.821584e-04) * (q[i5] * q[i16]) + (5.246723e-04) * (q[i5] * q[i19]) + (1.785773e-05) * (q[i5] * q[i20]) + (-5.585385e-04) * (q[i5] * q[i21])
            + (-2.062318e-04) * (q[i5] * q[i22]) + (-2.538085e-04) * (q[i6] * q[i7]) + (-4.515444e-04) * (q[i6] * q[i8]) + (-4.095732e-04) * (q[i6] * q[i10])
            + (6.405275e-05) * (q[i6] * q[i11]) + (1.787217e-04) * (q[i6] * q[i12]) + (-5.165831e-04) * (q[i6] * q[i15]) + (-2.863813e-04) * (q[i6] * q[i16])
            + (7.655790e-05) * (q[i6] * q[i19]) + (2.288140e-04) * (q[i6] * q[i20]) + (-4.923126e-04) * (q[i6] * q[i21]) + (-7.035101e-05) * (q[i6] * q[i22])
            + (-1.356350e-03) * (q[i7] * q[i8]) + (-4.172767e-04) * (q[i7] * q[i10]) + (1.817600e-04) * (q[i7] * q[i11]) + (3.201946e-04) * (q[i7] * q[i12])
            + (5.054034e-04) * (q[i7] * q[i15]) + (-3.701206e-05) * (q[i7] * q[i16]) + (4.038065e-04) * (q[i7] * q[i19]) + (-1.078364e-04) * (q[i7] * q[i20])
            + (-1.936129e-04) * (q[i7] * q[i21]) + (1.932345e-04) * (q[i7] * q[i22]) + (-1.062765e-04) * (q[i8] * q[i10]) + (-3.790616e-04) * (q[i8] * q[i11])
            + (-2.503545e-04) * (q[i8] * q[i12]) + (-5.608626e-04) * (q[i8] * q[i15]) + (2.161125e-05) * (q[i8] * q[i16]) + (-2.332290e-04) * (q[i8] * q[i19])
            + (2.934882e-04) * (q[i8] * q[i20]) + (1.329135e-04) * (q[i8] * q[i21]) + (-1.614691e-04) * (q[i8] * q[i22]) + (8.938416e-05) * (q[i10] * q[i11])
            + (-8.833383e-05) * (q[i10] * q[i12]) + (1.485611e-04) * (q[i10] * q[i15]) + (1.512407e-04) * (q[i10] * q[i16])
            + (-3.864186e-05) * (q[i10] * q[i19]) + (-3.865974e-05) * (q[i10] * q[i20]) + (-5.422781e-05) * (q[i10] * q[i21])
            + (5.440258e-05) * (q[i10] * q[i22]) + (3.372127e-04) * (q[i11] * q[i12]) + (3.807597e-05) * (q[i11] * q[i15]) + (-2.562664e-04) * (q[i11] * q[i16])
            + (-2.355869e-04) * (q[i11] * q[i19]) + (4.099562e-04) * (q[i11] * q[i20]) + (-1.556964e-04) * (q[i11] * q[i21])
            + (2.365417e-05) * (q[i11] * q[i22]) + (2.298135e-04) * (q[i12] * q[i15]) + (4.424267e-04) * (q[i12] * q[i16]) + (1.301948e-04) * (q[i12] * q[i19])
            + (2.181249e-04) * (q[i12] * q[i20]) + (-1.273016e-04) * (q[i12] * q[i21]) + (6.292190e-06) * (q[i12] * q[i22]) + (1.216782e-04) * (q[i15] * q[i16])
            + (-2.019897e-04) * (q[i15] * q[i19]) + (5.364623e-05) * (q[i15] * q[i20]) + (6.224095e-05) * (q[i15] * q[i21]) + (5.498908e-05) * (q[i15] * q[i22])
            + (2.454538e-04) * (q[i16] * q[i19]) + (7.094283e-05) * (q[i16] * q[i20]) + (6.556024e-05) * (q[i16] * q[i21]) + (-1.441560e-04) * (q[i16] * q[i22])
            + (4.018498e-05) * (q[i19] * q[i20]) + (7.119420e-05) * (q[i19] * q[i21]) + (5.045095e-06) * (q[i19] * q[i22]) + (2.715758e-05) * (q[i20] * q[i21])
            + (-9.038603e-05) * (q[i20] * q[i22]) + (4.638653e-05) * (q[i21] * q[i22]);
   }

   public void getJQy10(double[] q, double[][] JQ)
   {
      JQ[2][i10] = (5.766145e-02) * (1) + (-3.442015e-03) * ((2) * q[i10]) + (6.756558e-04) * (q[i0]) + (-4.832283e-04) * (q[i1]) + (-2.077411e-03) * (q[i2])
            + (4.690330e-03) * (q[i3]) + (5.152011e-03) * (q[i4]) + (5.287685e-03) * (q[i5]) + (5.144393e-03) * (q[i6]) + (-1.027846e-02) * (q[i7])
            + (3.915407e-03) * (q[i8]) + (3.393551e-03) * (q[i9]) + (-8.699660e-05) * (q[i11]) + (6.043354e-04) * (q[i12]) + (1.905084e-03) * (q[i15])
            + (1.021186e-03) * (q[i16]) + (-1.201700e-04) * (q[i19]) + (1.449772e-04) * (q[i20]) + (-5.336681e-04) * (q[i21]) + (6.467833e-04) * (q[i22])
            + (-7.828196e-05) * (q[i0] * q[i0]) + (-3.113276e-03) * (q[i1] * q[i1]) + (-1.382920e-03) * (q[i2] * q[i2]) + (-1.056831e-03) * (q[i3] * q[i3])
            + (1.624405e-03) * (q[i4] * q[i4]) + (4.921660e-04) * (q[i5] * q[i5]) + (-4.106816e-04) * (q[i6] * q[i6]) + (-8.165828e-04) * (q[i7] * q[i7])
            + (2.897784e-04) * (q[i8] * q[i8]) + (-2.172592e-04) * (q[i9] * q[i9]) + (3.095510e-04) * ((2) * q[i0] * q[i10])
            + (1.998999e-04) * ((2) * q[i1] * q[i10]) + (-5.555149e-04) * ((2) * q[i2] * q[i10]) + (-8.852734e-04) * ((2) * q[i3] * q[i10])
            + (-8.770639e-04) * ((2) * q[i4] * q[i10]) + (-1.104244e-03) * ((2) * q[i5] * q[i10]) + (4.343192e-05) * ((2) * q[i6] * q[i10])
            + (-2.533437e-03) * ((2) * q[i7] * q[i10]) + (5.970276e-04) * ((2) * q[i8] * q[i10]) + (-2.201108e-04) * ((2) * q[i9] * q[i10])
            + (-9.242922e-04) * ((3) * q[i10] * q[i10]) + (-3.427192e-04) * ((2) * q[i10] * q[i11]) + (-3.808000e-05) * ((2) * q[i10] * q[i12])
            + (-2.469388e-04) * ((2) * q[i10] * q[i15]) + (-9.715316e-05) * ((2) * q[i10] * q[i16]) + (7.674123e-05) * ((2) * q[i10] * q[i19])
            + (5.754956e-05) * ((2) * q[i10] * q[i20]) + (1.589391e-04) * ((2) * q[i10] * q[i21]) + (-7.038176e-05) * ((2) * q[i10] * q[i22])
            + (-7.667511e-06) * (q[i11] * q[i11]) + (-1.478634e-04) * (q[i12] * q[i12]) + (-1.180544e-04) * (q[i15] * q[i15])
            + (-1.432445e-04) * (q[i16] * q[i16]) + (-1.226696e-04) * (q[i19] * q[i19]) + (-3.093279e-05) * (q[i20] * q[i20])
            + (8.894858e-06) * (q[i21] * q[i21]) + (6.995825e-07) * (q[i22] * q[i22]) + (9.960849e-04) * (q[i0] * q[i1]) + (1.910390e-03) * (q[i0] * q[i2])
            + (-3.278556e-03) * (q[i0] * q[i3]) + (-7.603548e-04) * (q[i0] * q[i4]) + (7.873888e-04) * (q[i0] * q[i5]) + (2.113707e-03) * (q[i0] * q[i6])
            + (8.376530e-05) * (q[i0] * q[i7]) + (1.771793e-04) * (q[i0] * q[i8]) + (2.411114e-04) * (q[i0] * q[i9]) + (9.875970e-04) * (q[i0] * q[i11])
            + (4.411800e-04) * (q[i0] * q[i12]) + (3.368562e-04) * (q[i0] * q[i15]) + (-3.942323e-04) * (q[i0] * q[i16]) + (3.867514e-05) * (q[i0] * q[i19])
            + (3.980839e-05) * (q[i0] * q[i20]) + (-5.573833e-05) * (q[i0] * q[i21]) + (-2.403846e-04) * (q[i0] * q[i22]) + (-1.144844e-03) * (q[i1] * q[i2])
            + (3.247102e-04) * (q[i1] * q[i3]) + (1.165198e-02) * (q[i1] * q[i4]) + (5.176418e-04) * (q[i1] * q[i5]) + (-2.003019e-03) * (q[i1] * q[i6])
            + (1.539917e-03) * (q[i1] * q[i7]) + (-1.222720e-03) * (q[i1] * q[i8]) + (-2.445345e-04) * (q[i1] * q[i9]) + (-4.684288e-04) * (q[i1] * q[i11])
            + (8.078761e-05) * (q[i1] * q[i12]) + (-3.101763e-04) * (q[i1] * q[i15]) + (3.223845e-04) * (q[i1] * q[i16]) + (-6.479919e-04) * (q[i1] * q[i19])
            + (2.965278e-04) * (q[i1] * q[i20]) + (-1.044877e-04) * (q[i1] * q[i21]) + (-1.251050e-04) * (q[i1] * q[i22]) + (-1.802805e-04) * (q[i2] * q[i3])
            + (2.528187e-03) * (q[i2] * q[i4]) + (5.205727e-03) * (q[i2] * q[i5]) + (-9.274168e-04) * (q[i2] * q[i6]) + (-9.301464e-04) * (q[i2] * q[i7])
            + (-2.806527e-03) * (q[i2] * q[i8]) + (2.633729e-07) * (q[i2] * q[i9]) + (2.285840e-04) * (q[i2] * q[i11]) + (-2.200809e-04) * (q[i2] * q[i12])
            + (2.278704e-04) * (q[i2] * q[i15]) + (-2.443248e-04) * (q[i2] * q[i16]) + (-5.623259e-06) * (q[i2] * q[i19]) + (3.850246e-04) * (q[i2] * q[i20])
            + (-2.336341e-04) * (q[i2] * q[i21]) + (-2.438366e-04) * (q[i2] * q[i22]) + (-1.819332e-03) * (q[i3] * q[i4]) + (-3.088242e-04) * (q[i3] * q[i5])
            + (-1.035583e-03) * (q[i3] * q[i6]) + (-1.806832e-03) * (q[i3] * q[i7]) + (7.952767e-04) * (q[i3] * q[i8]) + (-4.577861e-04) * (q[i3] * q[i9])
            + (-8.193292e-04) * (q[i3] * q[i11]) + (3.973499e-04) * (q[i3] * q[i12]) + (-4.077110e-04) * (q[i3] * q[i15]) + (4.492608e-04) * (q[i3] * q[i16])
            + (2.930732e-04) * (q[i3] * q[i19]) + (4.740704e-04) * (q[i3] * q[i20]) + (-8.036297e-05) * (q[i3] * q[i21]) + (-1.194938e-04) * (q[i3] * q[i22])
            + (-1.076212e-03) * (q[i4] * q[i5]) + (1.206311e-03) * (q[i4] * q[i6]) + (-1.225452e-03) * (q[i4] * q[i7]) + (1.294842e-03) * (q[i4] * q[i8])
            + (4.573698e-04) * (q[i4] * q[i9]) + (4.898072e-04) * (q[i4] * q[i11]) + (2.566401e-04) * (q[i4] * q[i12]) + (6.345679e-04) * (q[i4] * q[i15])
            + (3.787644e-04) * (q[i4] * q[i16]) + (2.490965e-05) * (q[i4] * q[i19]) + (-1.768820e-04) * (q[i4] * q[i20]) + (2.002999e-04) * (q[i4] * q[i21])
            + (2.178710e-04) * (q[i4] * q[i22]) + (-2.309741e-03) * (q[i5] * q[i6]) + (-1.875516e-03) * (q[i5] * q[i7]) + (-2.008047e-03) * (q[i5] * q[i8])
            + (-3.608466e-07) * (q[i5] * q[i9]) + (-4.946957e-04) * (q[i5] * q[i11]) + (-6.254464e-04) * (q[i5] * q[i12]) + (2.874019e-04) * (q[i5] * q[i15])
            + (-9.036813e-04) * (q[i5] * q[i16]) + (-1.686565e-05) * (q[i5] * q[i19]) + (-5.345133e-04) * (q[i5] * q[i20]) + (-2.027584e-04) * (q[i5] * q[i21])
            + (-5.498457e-04) * (q[i5] * q[i22]) + (-2.458370e-04) * (q[i6] * q[i7]) + (-1.345699e-03) * (q[i6] * q[i8]) + (-4.095732e-04) * (q[i6] * q[i9])
            + (-3.145454e-04) * (q[i6] * q[i11]) + (-1.802055e-04) * (q[i6] * q[i12]) + (-4.340690e-05) * (q[i6] * q[i15]) + (5.037950e-04) * (q[i6] * q[i16])
            + (-1.027714e-04) * (q[i6] * q[i19]) + (3.982749e-04) * (q[i6] * q[i20]) + (-1.984213e-04) * (q[i6] * q[i21]) + (1.941054e-04) * (q[i6] * q[i22])
            + (-4.340547e-04) * (q[i7] * q[i8]) + (-4.172767e-04) * (q[i7] * q[i9]) + (-1.821436e-04) * (q[i7] * q[i11]) + (-7.359981e-05) * (q[i7] * q[i12])
            + (-2.776757e-04) * (q[i7] * q[i15]) + (-5.209704e-04) * (q[i7] * q[i16]) + (2.316224e-04) * (q[i7] * q[i19]) + (7.333906e-05) * (q[i7] * q[i20])
            + (7.155125e-05) * (q[i7] * q[i21]) + (4.919634e-04) * (q[i7] * q[i22]) + (-1.062765e-04) * (q[i8] * q[i9]) + (2.480893e-04) * (q[i8] * q[i11])
            + (3.753363e-04) * (q[i8] * q[i12]) + (2.855460e-05) * (q[i8] * q[i15]) + (-5.650428e-04) * (q[i8] * q[i16]) + (2.962574e-04) * (q[i8] * q[i19])
            + (-2.234525e-04) * (q[i8] * q[i20]) + (1.593766e-04) * (q[i8] * q[i21]) + (-1.333644e-04) * (q[i8] * q[i22]) + (8.938416e-05) * (q[i9] * q[i11])
            + (-8.833383e-05) * (q[i9] * q[i12]) + (1.485611e-04) * (q[i9] * q[i15]) + (1.512407e-04) * (q[i9] * q[i16]) + (-3.864186e-05) * (q[i9] * q[i19])
            + (-3.865974e-05) * (q[i9] * q[i20]) + (-5.422781e-05) * (q[i9] * q[i21]) + (5.440258e-05) * (q[i9] * q[i22]) + (3.405033e-04) * (q[i11] * q[i12])
            + (-4.379871e-04) * (q[i11] * q[i15]) + (-2.290909e-04) * (q[i11] * q[i16]) + (-2.225971e-04) * (q[i11] * q[i19])
            + (-1.278631e-04) * (q[i11] * q[i20]) + (2.454380e-06) * (q[i11] * q[i21]) + (-1.242732e-04) * (q[i11] * q[i22])
            + (2.620049e-04) * (q[i12] * q[i15]) + (-3.719982e-05) * (q[i12] * q[i16]) + (-4.135810e-04) * (q[i12] * q[i19])
            + (2.268484e-04) * (q[i12] * q[i20]) + (2.327850e-05) * (q[i12] * q[i21]) + (-1.499846e-04) * (q[i12] * q[i22]) + (1.207736e-04) * (q[i15] * q[i16])
            + (6.720933e-05) * (q[i15] * q[i19]) + (2.440403e-04) * (q[i15] * q[i20]) + (1.405013e-04) * (q[i15] * q[i21]) + (-6.600514e-05) * (q[i15] * q[i22])
            + (5.760996e-05) * (q[i16] * q[i19]) + (-2.040246e-04) * (q[i16] * q[i20]) + (-5.506688e-05) * (q[i16] * q[i21])
            + (-6.104315e-05) * (q[i16] * q[i22]) + (4.563640e-05) * (q[i19] * q[i20]) + (9.115209e-05) * (q[i19] * q[i21])
            + (-2.845212e-05) * (q[i19] * q[i22]) + (-2.515620e-06) * (q[i20] * q[i21]) + (-6.493948e-05) * (q[i20] * q[i22])
            + (4.477543e-05) * (q[i21] * q[i22]);
   }

   public void getJQy11(double[] q, double[][] JQ)
   {
      JQ[2][i11] = (-6.514705e-03) * (1) + (-3.430255e-03) * ((2) * q[i11]) + (-2.883306e-03) * (q[i0]) + (-9.367681e-04) * (q[i1]) + (-3.180395e-03) * (q[i2])
            + (-3.008612e-03) * (q[i3]) + (-5.395995e-03) * (q[i4]) + (-3.371008e-03) * (q[i5]) + (4.540958e-04) * (q[i6]) + (-3.654866e-04) * (q[i7])
            + (-5.004001e-04) * (q[i8]) + (-5.751373e-04) * (q[i9]) + (-8.699660e-05) * (q[i10]) + (6.946383e-05) * (q[i12]) + (8.759169e-04) * (q[i15])
            + (9.139358e-04) * (q[i16]) + (-4.620093e-04) * (q[i19]) + (-6.228889e-04) * (q[i20]) + (-1.495999e-03) * (q[i21]) + (3.249641e-04) * (q[i22])
            + (6.092797e-04) * (q[i0] * q[i0]) + (-1.286764e-04) * (q[i1] * q[i1]) + (8.277614e-04) * (q[i2] * q[i2]) + (2.469390e-04) * (q[i3] * q[i3])
            + (1.929410e-04) * (q[i4] * q[i4]) + (-1.414232e-04) * (q[i5] * q[i5]) + (-4.244388e-04) * (q[i6] * q[i6]) + (2.617575e-04) * (q[i7] * q[i7])
            + (5.352001e-04) * (q[i8] * q[i8]) + (3.615142e-05) * (q[i9] * q[i9]) + (-3.427192e-04) * (q[i10] * q[i10])
            + (2.080339e-04) * ((2) * q[i0] * q[i11]) + (-3.758514e-05) * ((2) * q[i1] * q[i11]) + (2.278370e-03) * ((2) * q[i2] * q[i11])
            + (6.860758e-04) * ((2) * q[i3] * q[i11]) + (-4.984141e-04) * ((2) * q[i4] * q[i11]) + (9.546430e-04) * ((2) * q[i5] * q[i11])
            + (-6.108288e-04) * ((2) * q[i6] * q[i11]) + (8.434333e-05) * ((2) * q[i7] * q[i11]) + (1.581777e-03) * ((2) * q[i8] * q[i11])
            + (-1.448847e-04) * ((2) * q[i9] * q[i11]) + (-7.667511e-06) * ((2) * q[i10] * q[i11]) + (1.164990e-03) * ((3) * q[i11] * q[i11])
            + (-4.182149e-05) * ((2) * q[i11] * q[i12]) + (-2.020515e-03) * ((2) * q[i11] * q[i15]) + (-1.803217e-04) * ((2) * q[i11] * q[i16])
            + (6.447725e-04) * ((2) * q[i11] * q[i19]) + (-8.472641e-05) * ((2) * q[i11] * q[i20]) + (1.849186e-05) * ((2) * q[i11] * q[i21])
            + (6.243942e-05) * ((2) * q[i11] * q[i22]) + (4.737131e-05) * (q[i12] * q[i12]) + (2.029845e-03) * (q[i15] * q[i15])
            + (-1.955120e-04) * (q[i16] * q[i16]) + (-4.295102e-04) * (q[i19] * q[i19]) + (-8.136337e-05) * (q[i20] * q[i20])
            + (-3.551176e-04) * (q[i21] * q[i21]) + (9.220215e-05) * (q[i22] * q[i22]) + (5.025431e-04) * (q[i0] * q[i1]) + (1.065594e-03) * (q[i0] * q[i2])
            + (4.723161e-04) * (q[i0] * q[i3]) + (-2.240502e-04) * (q[i0] * q[i4]) + (-1.103549e-03) * (q[i0] * q[i5]) + (-1.800672e-03) * (q[i0] * q[i6])
            + (1.402177e-04) * (q[i0] * q[i7]) + (-5.257208e-04) * (q[i0] * q[i8]) + (8.550443e-05) * (q[i0] * q[i9]) + (9.875970e-04) * (q[i0] * q[i10])
            + (1.048394e-04) * (q[i0] * q[i12]) + (1.167051e-04) * (q[i0] * q[i15]) + (-3.487901e-04) * (q[i0] * q[i16]) + (2.393338e-04) * (q[i0] * q[i19])
            + (1.528574e-04) * (q[i0] * q[i20]) + (-1.610796e-04) * (q[i0] * q[i21]) + (3.191131e-04) * (q[i0] * q[i22]) + (1.398578e-04) * (q[i1] * q[i2])
            + (8.214066e-04) * (q[i1] * q[i3]) + (3.772506e-04) * (q[i1] * q[i4]) + (-1.469162e-03) * (q[i1] * q[i5]) + (-1.167237e-04) * (q[i1] * q[i6])
            + (-1.131840e-03) * (q[i1] * q[i7]) + (3.963438e-05) * (q[i1] * q[i8]) + (4.395493e-04) * (q[i1] * q[i9]) + (-4.684288e-04) * (q[i1] * q[i10])
            + (-1.051395e-04) * (q[i1] * q[i12]) + (-8.438281e-05) * (q[i1] * q[i15]) + (1.474048e-04) * (q[i1] * q[i16]) + (-1.130834e-05) * (q[i1] * q[i19])
            + (-1.253673e-04) * (q[i1] * q[i20]) + (1.847853e-04) * (q[i1] * q[i21]) + (1.039093e-04) * (q[i1] * q[i22]) + (4.280872e-04) * (q[i2] * q[i3])
            + (-1.337421e-03) * (q[i2] * q[i4]) + (1.832586e-03) * (q[i2] * q[i5]) + (-1.253649e-03) * (q[i2] * q[i6]) + (5.285828e-05) * (q[i2] * q[i7])
            + (4.744048e-03) * (q[i2] * q[i8]) + (-2.112597e-04) * (q[i2] * q[i9]) + (2.285840e-04) * (q[i2] * q[i10]) + (4.690415e-06) * (q[i2] * q[i12])
            + (-2.637985e-03) * (q[i2] * q[i15]) + (-6.450256e-04) * (q[i2] * q[i16]) + (8.071551e-04) * (q[i2] * q[i19]) + (-4.746056e-04) * (q[i2] * q[i20])
            + (6.157953e-04) * (q[i2] * q[i21]) + (4.651000e-04) * (q[i2] * q[i22]) + (5.681483e-04) * (q[i3] * q[i4]) + (-1.929560e-03) * (q[i3] * q[i5])
            + (-1.374444e-03) * (q[i3] * q[i6]) + (2.956707e-04) * (q[i3] * q[i7]) + (1.033786e-03) * (q[i3] * q[i8]) + (2.531383e-04) * (q[i3] * q[i9])
            + (-8.193292e-04) * (q[i3] * q[i10]) + (-5.889736e-04) * (q[i3] * q[i12]) + (-7.788329e-04) * (q[i3] * q[i15]) + (3.615160e-04) * (q[i3] * q[i16])
            + (-6.880792e-04) * (q[i3] * q[i19]) + (-5.652306e-05) * (q[i3] * q[i20]) + (-8.779510e-04) * (q[i3] * q[i21]) + (-2.416514e-04) * (q[i3] * q[i22])
            + (1.749562e-03) * (q[i4] * q[i5]) + (-1.265048e-04) * (q[i4] * q[i6]) + (-3.746156e-04) * (q[i4] * q[i7]) + (7.959779e-04) * (q[i4] * q[i8])
            + (3.937067e-04) * (q[i4] * q[i9]) + (4.898072e-04) * (q[i4] * q[i10]) + (5.899136e-04) * (q[i4] * q[i12]) + (-1.003587e-03) * (q[i4] * q[i15])
            + (-8.476519e-05) * (q[i4] * q[i16]) + (2.910371e-04) * (q[i4] * q[i19]) + (3.013094e-04) * (q[i4] * q[i20]) + (-3.079492e-04) * (q[i4] * q[i21])
            + (-2.066231e-04) * (q[i4] * q[i22]) + (5.856359e-04) * (q[i5] * q[i6]) + (-1.821218e-03) * (q[i5] * q[i7]) + (1.329208e-03) * (q[i5] * q[i8])
            + (-6.350464e-04) * (q[i5] * q[i9]) + (-4.946957e-04) * (q[i5] * q[i10]) + (1.301077e-05) * (q[i5] * q[i12]) + (-2.286334e-03) * (q[i5] * q[i15])
            + (-1.740660e-04) * (q[i5] * q[i16]) + (-2.437107e-04) * (q[i5] * q[i19]) + (-2.532027e-04) * (q[i5] * q[i20]) + (-1.165125e-04) * (q[i5] * q[i21])
            + (1.027526e-04) * (q[i5] * q[i22]) + (-3.422639e-04) * (q[i6] * q[i7]) + (-3.970293e-04) * (q[i6] * q[i8]) + (6.405275e-05) * (q[i6] * q[i9])
            + (-3.145454e-04) * (q[i6] * q[i10]) + (-4.465450e-05) * (q[i6] * q[i12]) + (-3.946318e-05) * (q[i6] * q[i15]) + (-2.434706e-05) * (q[i6] * q[i16])
            + (-1.572757e-04) * (q[i6] * q[i19]) + (2.184652e-04) * (q[i6] * q[i20]) + (-6.219832e-05) * (q[i6] * q[i21]) + (-1.467622e-04) * (q[i6] * q[i22])
            + (3.746026e-04) * (q[i7] * q[i8]) + (1.817600e-04) * (q[i7] * q[i9]) + (-1.821436e-04) * (q[i7] * q[i10]) + (-5.278493e-05) * (q[i7] * q[i12])
            + (1.699183e-04) * (q[i7] * q[i15]) + (7.544601e-05) * (q[i7] * q[i16]) + (-1.569050e-04) * (q[i7] * q[i19]) + (2.369777e-04) * (q[i7] * q[i20])
            + (-2.603405e-04) * (q[i7] * q[i21]) + (2.806518e-04) * (q[i7] * q[i22]) + (-3.790616e-04) * (q[i8] * q[i9]) + (2.480893e-04) * (q[i8] * q[i10])
            + (-5.578371e-04) * (q[i8] * q[i12]) + (-7.542863e-04) * (q[i8] * q[i15]) + (3.434920e-05) * (q[i8] * q[i16]) + (4.733749e-04) * (q[i8] * q[i19])
            + (5.237842e-04) * (q[i8] * q[i20]) + (-1.172496e-04) * (q[i8] * q[i21]) + (1.615886e-06) * (q[i8] * q[i22]) + (8.938416e-05) * (q[i9] * q[i10])
            + (3.372127e-04) * (q[i9] * q[i12]) + (3.807597e-05) * (q[i9] * q[i15]) + (-2.562664e-04) * (q[i9] * q[i16]) + (-2.355869e-04) * (q[i9] * q[i19])
            + (4.099562e-04) * (q[i9] * q[i20]) + (-1.556964e-04) * (q[i9] * q[i21]) + (2.365417e-05) * (q[i9] * q[i22]) + (3.405033e-04) * (q[i10] * q[i12])
            + (-4.379871e-04) * (q[i10] * q[i15]) + (-2.290909e-04) * (q[i10] * q[i16]) + (-2.225971e-04) * (q[i10] * q[i19])
            + (-1.278631e-04) * (q[i10] * q[i20]) + (2.454380e-06) * (q[i10] * q[i21]) + (-1.242732e-04) * (q[i10] * q[i22])
            + (2.104712e-04) * (q[i12] * q[i15]) + (2.136035e-04) * (q[i12] * q[i16]) + (-2.892577e-04) * (q[i12] * q[i19])
            + (-2.910180e-04) * (q[i12] * q[i20]) + (-2.512629e-05) * (q[i12] * q[i21]) + (2.919643e-05) * (q[i12] * q[i22])
            + (1.670981e-04) * (q[i15] * q[i16]) + (-7.325050e-05) * (q[i15] * q[i19]) + (-1.247904e-04) * (q[i15] * q[i20])
            + (-1.182786e-04) * (q[i15] * q[i21]) + (1.123203e-04) * (q[i15] * q[i22]) + (-1.722355e-04) * (q[i16] * q[i19])
            + (-2.718841e-04) * (q[i16] * q[i20]) + (8.398881e-05) * (q[i16] * q[i21]) + (-1.625631e-04) * (q[i16] * q[i22])
            + (1.578352e-04) * (q[i19] * q[i20]) + (9.781332e-05) * (q[i19] * q[i21]) + (-1.896632e-05) * (q[i19] * q[i22])
            + (-7.191706e-05) * (q[i20] * q[i21]) + (-1.398552e-04) * (q[i20] * q[i22]) + (1.085750e-04) * (q[i21] * q[i22]);
   }

   public void getJQy12(double[] q, double[][] JQ)
   {
      JQ[2][i12] = (6.950164e-03) * (1) + (-3.350182e-03) * ((2) * q[i12]) + (-9.513398e-04) * (q[i0]) + (-2.903912e-03) * (q[i1]) + (-3.118885e-03) * (q[i2])
            + (-5.443482e-03) * (q[i3]) + (-3.023244e-03) * (q[i4]) + (-3.296339e-03) * (q[i5]) + (3.960625e-04) * (q[i6]) + (-4.179029e-04) * (q[i7])
            + (3.912603e-04) * (q[i8]) + (1.103258e-04) * (q[i9]) + (6.043354e-04) * (q[i10]) + (6.946383e-05) * (q[i11]) + (-9.042461e-04) * (q[i15])
            + (-9.498034e-04) * (q[i16]) + (6.370554e-04) * (q[i19]) + (4.295278e-04) * (q[i20]) + (3.438006e-04) * (q[i21]) + (-1.516772e-03) * (q[i22])
            + (1.224367e-04) * (q[i0] * q[i0]) + (-6.013921e-04) * (q[i1] * q[i1]) + (-8.804478e-04) * (q[i2] * q[i2]) + (-1.931397e-04) * (q[i3] * q[i3])
            + (-2.181179e-04) * (q[i4] * q[i4]) + (1.351142e-04) * (q[i5] * q[i5]) + (-2.558142e-04) * (q[i6] * q[i6]) + (4.257258e-04) * (q[i7] * q[i7])
            + (-5.628161e-04) * (q[i8] * q[i8]) + (3.414539e-04) * (q[i9] * q[i9]) + (-3.808000e-05) * (q[i10] * q[i10]) + (-4.182149e-05) * (q[i11] * q[i11])
            + (3.540226e-05) * ((2) * q[i0] * q[i12]) + (-2.165943e-04) * ((2) * q[i1] * q[i12]) + (-2.398656e-03) * ((2) * q[i2] * q[i12])
            + (4.919326e-04) * ((2) * q[i3] * q[i12]) + (-7.033265e-04) * ((2) * q[i4] * q[i12]) + (-1.014978e-03) * ((2) * q[i5] * q[i12])
            + (8.812535e-05) * ((2) * q[i6] * q[i12]) + (-6.252605e-04) * ((2) * q[i7] * q[i12]) + (1.612018e-03) * ((2) * q[i8] * q[i12])
            + (6.421360e-06) * ((2) * q[i9] * q[i12]) + (-1.478634e-04) * ((2) * q[i10] * q[i12]) + (4.737131e-05) * ((2) * q[i11] * q[i12])
            + (-1.218456e-03) * ((3) * q[i12] * q[i12]) + (-1.863703e-04) * ((2) * q[i12] * q[i15]) + (-2.013504e-03) * ((2) * q[i12] * q[i16])
            + (-8.000025e-05) * ((2) * q[i12] * q[i19]) + (6.312400e-04) * ((2) * q[i12] * q[i20]) + (-6.533212e-05) * ((2) * q[i12] * q[i21])
            + (-1.222734e-05) * ((2) * q[i12] * q[i22]) + (1.936846e-04) * (q[i15] * q[i15]) + (-2.063879e-03) * (q[i16] * q[i16])
            + (8.348564e-05) * (q[i19] * q[i19]) + (4.310497e-04) * (q[i20] * q[i20]) + (-9.283926e-05) * (q[i21] * q[i21]) + (3.677878e-04) * (q[i22] * q[i22])
            + (-4.917774e-04) * (q[i0] * q[i1]) + (-1.294444e-04) * (q[i0] * q[i2]) + (-4.202721e-04) * (q[i0] * q[i3]) + (-8.096605e-04) * (q[i0] * q[i4])
            + (1.435492e-03) * (q[i0] * q[i5]) + (-1.143893e-03) * (q[i0] * q[i6]) + (-1.169007e-04) * (q[i0] * q[i7]) + (6.450441e-06) * (q[i0] * q[i8])
            + (-4.645134e-04) * (q[i0] * q[i9]) + (4.411800e-04) * (q[i0] * q[i10]) + (1.048394e-04) * (q[i0] * q[i11]) + (1.398222e-04) * (q[i0] * q[i15])
            + (-7.891911e-05) * (q[i0] * q[i16]) + (-1.259175e-04) * (q[i0] * q[i19]) + (-1.010733e-05) * (q[i0] * q[i20]) + (-1.028225e-04) * (q[i0] * q[i21])
            + (-1.881295e-04) * (q[i0] * q[i22]) + (-1.061259e-03) * (q[i1] * q[i2]) + (2.298352e-04) * (q[i1] * q[i3]) + (-4.866641e-04) * (q[i1] * q[i4])
            + (1.040694e-03) * (q[i1] * q[i5]) + (1.400456e-04) * (q[i1] * q[i6]) + (-1.800183e-03) * (q[i1] * q[i7]) + (-5.528362e-04) * (q[i1] * q[i8])
            + (9.939138e-04) * (q[i1] * q[i9]) + (8.078761e-05) * (q[i1] * q[i10]) + (-1.051395e-04) * (q[i1] * q[i11]) + (-3.490332e-04) * (q[i1] * q[i15])
            + (1.179321e-04) * (q[i1] * q[i16]) + (1.497031e-04) * (q[i1] * q[i19]) + (2.454978e-04) * (q[i1] * q[i20]) + (-3.210245e-04) * (q[i1] * q[i21])
            + (1.659558e-04) * (q[i1] * q[i22]) + (1.319566e-03) * (q[i2] * q[i3]) + (-4.544515e-04) * (q[i2] * q[i4]) + (-1.945874e-03) * (q[i2] * q[i5])
            + (3.816430e-05) * (q[i2] * q[i6]) + (-1.292311e-03) * (q[i2] * q[i7]) + (4.759668e-03) * (q[i2] * q[i8]) + (2.393001e-04) * (q[i2] * q[i9])
            + (-2.200809e-04) * (q[i2] * q[i10]) + (4.690415e-06) * (q[i2] * q[i11]) + (-6.507329e-04) * (q[i2] * q[i15]) + (-2.629174e-03) * (q[i2] * q[i16])
            + (-4.714424e-04) * (q[i2] * q[i19]) + (7.961283e-04) * (q[i2] * q[i20]) + (-4.648714e-04) * (q[i2] * q[i21]) + (-6.117027e-04) * (q[i2] * q[i22])
            + (-6.056154e-04) * (q[i3] * q[i4]) + (-1.744299e-03) * (q[i3] * q[i5]) + (-3.581392e-04) * (q[i3] * q[i6]) + (-1.233349e-04) * (q[i3] * q[i7])
            + (8.022163e-04) * (q[i3] * q[i8]) + (5.033502e-04) * (q[i3] * q[i9]) + (3.973499e-04) * (q[i3] * q[i10]) + (-5.889736e-04) * (q[i3] * q[i11])
            + (-8.186351e-05) * (q[i3] * q[i15]) + (-1.012331e-03) * (q[i3] * q[i16]) + (3.051342e-04) * (q[i3] * q[i19]) + (3.018533e-04) * (q[i3] * q[i20])
            + (2.076512e-04) * (q[i3] * q[i21]) + (2.997615e-04) * (q[i3] * q[i22]) + (1.903223e-03) * (q[i4] * q[i5]) + (3.047392e-04) * (q[i4] * q[i6])
            + (-1.389476e-03) * (q[i4] * q[i7]) + (1.067223e-03) * (q[i4] * q[i8]) + (-8.228550e-04) * (q[i4] * q[i9]) + (2.566401e-04) * (q[i4] * q[i10])
            + (5.899136e-04) * (q[i4] * q[i11]) + (3.631967e-04) * (q[i4] * q[i15]) + (-7.855407e-04) * (q[i4] * q[i16]) + (-6.787080e-05) * (q[i4] * q[i19])
            + (-6.834487e-04) * (q[i4] * q[i20]) + (2.436982e-04) * (q[i4] * q[i21]) + (8.872700e-04) * (q[i4] * q[i22]) + (-1.821301e-03) * (q[i5] * q[i6])
            + (5.918864e-04) * (q[i5] * q[i7]) + (1.336449e-03) * (q[i5] * q[i8]) + (-4.927454e-04) * (q[i5] * q[i9]) + (-6.254464e-04) * (q[i5] * q[i10])
            + (1.301077e-05) * (q[i5] * q[i11]) + (-1.615513e-04) * (q[i5] * q[i15]) + (-2.283075e-03) * (q[i5] * q[i16]) + (-2.503387e-04) * (q[i5] * q[i19])
            + (-2.516082e-04) * (q[i5] * q[i20]) + (-1.002124e-04) * (q[i5] * q[i21]) + (1.310858e-04) * (q[i5] * q[i22]) + (3.447562e-04) * (q[i6] * q[i7])
            + (-3.383890e-04) * (q[i6] * q[i8]) + (1.787217e-04) * (q[i6] * q[i9]) + (-1.802055e-04) * (q[i6] * q[i10]) + (-4.465450e-05) * (q[i6] * q[i11])
            + (-7.233044e-05) * (q[i6] * q[i15]) + (-1.767328e-04) * (q[i6] * q[i16]) + (-2.336362e-04) * (q[i6] * q[i19]) + (1.541868e-04) * (q[i6] * q[i20])
            + (2.803087e-04) * (q[i6] * q[i21]) + (-2.579998e-04) * (q[i6] * q[i22]) + (3.996764e-04) * (q[i7] * q[i8]) + (3.201946e-04) * (q[i7] * q[i9])
            + (-7.359981e-05) * (q[i7] * q[i10]) + (-5.278493e-05) * (q[i7] * q[i11]) + (2.746136e-05) * (q[i7] * q[i15]) + (4.293758e-05) * (q[i7] * q[i16])
            + (-2.168496e-04) * (q[i7] * q[i19]) + (1.536198e-04) * (q[i7] * q[i20]) + (-1.445232e-04) * (q[i7] * q[i21]) + (-5.375323e-05) * (q[i7] * q[i22])
            + (-2.503545e-04) * (q[i8] * q[i9]) + (3.753363e-04) * (q[i8] * q[i10]) + (-5.578371e-04) * (q[i8] * q[i11]) + (-4.209219e-05) * (q[i8] * q[i15])
            + (7.420413e-04) * (q[i8] * q[i16]) + (-5.292212e-04) * (q[i8] * q[i19]) + (-4.641388e-04) * (q[i8] * q[i20]) + (4.989940e-06) * (q[i8] * q[i21])
            + (-1.251570e-04) * (q[i8] * q[i22]) + (-8.833383e-05) * (q[i9] * q[i10]) + (3.372127e-04) * (q[i9] * q[i11]) + (2.298135e-04) * (q[i9] * q[i15])
            + (4.424267e-04) * (q[i9] * q[i16]) + (1.301948e-04) * (q[i9] * q[i19]) + (2.181249e-04) * (q[i9] * q[i20]) + (-1.273016e-04) * (q[i9] * q[i21])
            + (6.292190e-06) * (q[i9] * q[i22]) + (3.405033e-04) * (q[i10] * q[i11]) + (2.620049e-04) * (q[i10] * q[i15]) + (-3.719982e-05) * (q[i10] * q[i16])
            + (-4.135810e-04) * (q[i10] * q[i19]) + (2.268484e-04) * (q[i10] * q[i20]) + (2.327850e-05) * (q[i10] * q[i21])
            + (-1.499846e-04) * (q[i10] * q[i22]) + (2.104712e-04) * (q[i11] * q[i15]) + (2.136035e-04) * (q[i11] * q[i16])
            + (-2.892577e-04) * (q[i11] * q[i19]) + (-2.910180e-04) * (q[i11] * q[i20]) + (-2.512629e-05) * (q[i11] * q[i21])
            + (2.919643e-05) * (q[i11] * q[i22]) + (-1.695531e-04) * (q[i15] * q[i16]) + (2.719574e-04) * (q[i15] * q[i19]) + (1.686572e-04) * (q[i15] * q[i20])
            + (-1.579868e-04) * (q[i15] * q[i21]) + (8.610818e-05) * (q[i15] * q[i22]) + (1.297550e-04) * (q[i16] * q[i19]) + (7.923498e-05) * (q[i16] * q[i20])
            + (1.141716e-04) * (q[i16] * q[i21]) + (-1.321650e-04) * (q[i16] * q[i22]) + (-1.579112e-04) * (q[i19] * q[i20])
            + (-1.418660e-04) * (q[i19] * q[i21]) + (-7.484854e-05) * (q[i19] * q[i22]) + (-2.069570e-05) * (q[i20] * q[i21])
            + (1.036630e-04) * (q[i20] * q[i22]) + (-1.105546e-04) * (q[i21] * q[i22]);
   }

   public void getJQy15(double[] q, double[][] JQ)
   {
      JQ[2][i15] = (5.348609e-03) * (1) + (-1.300503e-03) * ((2) * q[i15]) + (3.946335e-03) * (q[i0]) + (3.281194e-03) * (q[i1]) + (6.499997e-03) * (q[i2])
            + (3.868940e-04) * (q[i3]) + (-1.631447e-03) * (q[i4]) + (6.842918e-04) * (q[i5]) + (2.847519e-03) * (q[i6]) + (5.496503e-03) * (q[i7])
            + (-8.566462e-03) * (q[i8]) + (1.040421e-03) * (q[i9]) + (1.905084e-03) * (q[i10]) + (8.759169e-04) * (q[i11]) + (-9.042461e-04) * (q[i12])
            + (-3.944256e-04) * (q[i16]) + (7.792923e-04) * (q[i19]) + (5.134609e-05) * (q[i20]) + (3.841905e-04) * (q[i21]) + (2.592284e-04) * (q[i22])
            + (6.393383e-04) * (q[i0] * q[i0]) + (-2.069750e-05) * (q[i1] * q[i1]) + (-5.454598e-05) * (q[i2] * q[i2]) + (5.459118e-04) * (q[i3] * q[i3])
            + (-2.829543e-05) * (q[i4] * q[i4]) + (-1.570381e-03) * (q[i5] * q[i5]) + (-5.124145e-04) * (q[i6] * q[i6]) + (-5.255415e-04) * (q[i7] * q[i7])
            + (2.523868e-04) * (q[i8] * q[i8]) + (-9.578398e-05) * (q[i9] * q[i9]) + (-2.469388e-04) * (q[i10] * q[i10]) + (-2.020515e-03) * (q[i11] * q[i11])
            + (-1.863703e-04) * (q[i12] * q[i12]) + (2.058150e-04) * ((2) * q[i0] * q[i15]) + (3.545634e-04) * ((2) * q[i1] * q[i15])
            + (2.669399e-03) * ((2) * q[i2] * q[i15]) + (6.707951e-05) * ((2) * q[i3] * q[i15]) + (-4.986179e-05) * ((2) * q[i4] * q[i15])
            + (-5.327172e-04) * ((2) * q[i5] * q[i15]) + (6.372224e-05) * ((2) * q[i6] * q[i15]) + (7.534292e-04) * ((2) * q[i7] * q[i15])
            + (1.903655e-04) * ((2) * q[i8] * q[i15]) + (-1.399477e-04) * ((2) * q[i9] * q[i15]) + (-1.180544e-04) * ((2) * q[i10] * q[i15])
            + (2.029845e-03) * ((2) * q[i11] * q[i15]) + (1.936846e-04) * ((2) * q[i12] * q[i15]) + (3.132070e-04) * ((3) * q[i15] * q[i15])
            + (-8.299577e-05) * ((2) * q[i15] * q[i16]) + (2.742122e-04) * ((2) * q[i15] * q[i19]) + (-1.641042e-04) * ((2) * q[i15] * q[i20])
            + (-2.893991e-04) * ((2) * q[i15] * q[i21]) + (8.124079e-05) * ((2) * q[i15] * q[i22]) + (-8.439846e-05) * (q[i16] * q[i16])
            + (-2.575059e-04) * (q[i19] * q[i19]) + (-3.405803e-05) * (q[i20] * q[i20]) + (-1.400345e-04) * (q[i21] * q[i21])
            + (-4.611770e-05) * (q[i22] * q[i22]) + (-2.196508e-04) * (q[i0] * q[i1]) + (2.449058e-04) * (q[i0] * q[i2]) + (-1.399009e-03) * (q[i0] * q[i3])
            + (3.336301e-05) * (q[i0] * q[i4]) + (-7.085482e-04) * (q[i0] * q[i5]) + (-6.248782e-05) * (q[i0] * q[i6]) + (6.449001e-04) * (q[i0] * q[i7])
            + (-2.445200e-04) * (q[i0] * q[i8]) + (-3.216631e-04) * (q[i0] * q[i9]) + (3.368562e-04) * (q[i0] * q[i10]) + (1.167051e-04) * (q[i0] * q[i11])
            + (1.398222e-04) * (q[i0] * q[i12]) + (2.723890e-04) * (q[i0] * q[i16]) + (-9.798634e-05) * (q[i0] * q[i19]) + (-5.472361e-05) * (q[i0] * q[i20])
            + (2.581117e-04) * (q[i0] * q[i21]) + (8.102358e-05) * (q[i0] * q[i22]) + (2.364509e-04) * (q[i1] * q[i2]) + (2.560312e-05) * (q[i1] * q[i3])
            + (-4.366051e-03) * (q[i1] * q[i4]) + (-2.057310e-04) * (q[i1] * q[i5]) + (-3.584953e-04) * (q[i1] * q[i6]) + (-1.062855e-03) * (q[i1] * q[i7])
            + (-6.650239e-04) * (q[i1] * q[i8]) + (3.954244e-04) * (q[i1] * q[i9]) + (-3.101763e-04) * (q[i1] * q[i10]) + (-8.438281e-05) * (q[i1] * q[i11])
            + (-3.490332e-04) * (q[i1] * q[i12]) + (-2.649109e-04) * (q[i1] * q[i16]) + (3.470410e-04) * (q[i1] * q[i19]) + (5.617706e-04) * (q[i1] * q[i20])
            + (6.902543e-04) * (q[i1] * q[i21]) + (-1.032416e-04) * (q[i1] * q[i22]) + (7.515078e-04) * (q[i2] * q[i3]) + (-1.998887e-03) * (q[i2] * q[i4])
            + (-6.397119e-03) * (q[i2] * q[i5]) + (-6.280712e-04) * (q[i2] * q[i6]) + (-4.168032e-04) * (q[i2] * q[i7]) + (5.980369e-05) * (q[i2] * q[i8])
            + (2.405074e-04) * (q[i2] * q[i9]) + (2.278704e-04) * (q[i2] * q[i10]) + (-2.637985e-03) * (q[i2] * q[i11]) + (-6.507329e-04) * (q[i2] * q[i12])
            + (-7.984367e-08) * (q[i2] * q[i16]) + (2.449223e-04) * (q[i2] * q[i19]) + (6.778326e-04) * (q[i2] * q[i20]) + (6.348229e-04) * (q[i2] * q[i21])
            + (2.914068e-05) * (q[i2] * q[i22]) + (-1.645833e-04) * (q[i3] * q[i4]) + (-1.171593e-03) * (q[i3] * q[i5]) + (-6.172732e-04) * (q[i3] * q[i6])
            + (7.007633e-04) * (q[i3] * q[i7]) + (-1.804215e-03) * (q[i3] * q[i8]) + (-3.873931e-04) * (q[i3] * q[i9]) + (-4.077110e-04) * (q[i3] * q[i10])
            + (-7.788329e-04) * (q[i3] * q[i11]) + (-8.186351e-05) * (q[i3] * q[i12]) + (-1.571992e-04) * (q[i3] * q[i16]) + (-5.973864e-04) * (q[i3] * q[i19])
            + (-2.824218e-04) * (q[i3] * q[i20]) + (-4.609274e-04) * (q[i3] * q[i21]) + (-3.611729e-04) * (q[i3] * q[i22]) + (1.105931e-03) * (q[i4] * q[i5])
            + (9.692188e-05) * (q[i4] * q[i6]) + (-6.976053e-04) * (q[i4] * q[i7]) + (-9.611609e-04) * (q[i4] * q[i8]) + (-4.485047e-04) * (q[i4] * q[i9])
            + (6.345679e-04) * (q[i4] * q[i10]) + (-1.003587e-03) * (q[i4] * q[i11]) + (3.631967e-04) * (q[i4] * q[i12]) + (1.578929e-04) * (q[i4] * q[i16])
            + (-7.381783e-05) * (q[i4] * q[i19]) + (6.806984e-04) * (q[i4] * q[i20]) + (-1.828871e-04) * (q[i4] * q[i21]) + (-2.002160e-05) * (q[i4] * q[i22])
            + (1.114467e-03) * (q[i5] * q[i6]) + (-1.913683e-04) * (q[i5] * q[i7]) + (1.798920e-03) * (q[i5] * q[i8]) + (9.078374e-04) * (q[i5] * q[i9])
            + (2.874019e-04) * (q[i5] * q[i10]) + (-2.286334e-03) * (q[i5] * q[i11]) + (-1.615513e-04) * (q[i5] * q[i12]) + (-8.659480e-07) * (q[i5] * q[i16])
            + (-7.873197e-04) * (q[i5] * q[i19]) + (-3.741474e-04) * (q[i5] * q[i20]) + (1.155737e-03) * (q[i5] * q[i21]) + (4.391866e-04) * (q[i5] * q[i22])
            + (4.058894e-04) * (q[i6] * q[i7]) + (-6.477363e-06) * (q[i6] * q[i8]) + (-5.165831e-04) * (q[i6] * q[i9]) + (-4.340690e-05) * (q[i6] * q[i10])
            + (-3.946318e-05) * (q[i6] * q[i11]) + (-7.233044e-05) * (q[i6] * q[i12]) + (-6.177311e-05) * (q[i6] * q[i16]) + (-4.611163e-04) * (q[i6] * q[i19])
            + (-3.996412e-04) * (q[i6] * q[i20]) + (7.036249e-05) * (q[i6] * q[i21]) + (7.993119e-05) * (q[i6] * q[i22]) + (-2.735794e-04) * (q[i7] * q[i8])
            + (5.054034e-04) * (q[i7] * q[i9]) + (-2.776757e-04) * (q[i7] * q[i10]) + (1.699183e-04) * (q[i7] * q[i11]) + (2.746136e-05) * (q[i7] * q[i12])
            + (-6.415069e-05) * (q[i7] * q[i16]) + (1.873814e-04) * (q[i7] * q[i19]) + (4.450782e-04) * (q[i7] * q[i20]) + (5.336785e-04) * (q[i7] * q[i21])
            + (4.000195e-05) * (q[i7] * q[i22]) + (-5.608626e-04) * (q[i8] * q[i9]) + (2.855460e-05) * (q[i8] * q[i10]) + (-7.542863e-04) * (q[i8] * q[i11])
            + (-4.209219e-05) * (q[i8] * q[i12]) + (-3.166315e-04) * (q[i8] * q[i16]) + (4.024530e-04) * (q[i8] * q[i19]) + (-6.814829e-05) * (q[i8] * q[i20])
            + (-1.336037e-03) * (q[i8] * q[i21]) + (8.554276e-05) * (q[i8] * q[i22]) + (1.485611e-04) * (q[i9] * q[i10]) + (3.807597e-05) * (q[i9] * q[i11])
            + (2.298135e-04) * (q[i9] * q[i12]) + (1.216782e-04) * (q[i9] * q[i16]) + (-2.019897e-04) * (q[i9] * q[i19]) + (5.364623e-05) * (q[i9] * q[i20])
            + (6.224095e-05) * (q[i9] * q[i21]) + (5.498908e-05) * (q[i9] * q[i22]) + (-4.379871e-04) * (q[i10] * q[i11]) + (2.620049e-04) * (q[i10] * q[i12])
            + (1.207736e-04) * (q[i10] * q[i16]) + (6.720933e-05) * (q[i10] * q[i19]) + (2.440403e-04) * (q[i10] * q[i20]) + (1.405013e-04) * (q[i10] * q[i21])
            + (-6.600514e-05) * (q[i10] * q[i22]) + (2.104712e-04) * (q[i11] * q[i12]) + (1.670981e-04) * (q[i11] * q[i16])
            + (-7.325050e-05) * (q[i11] * q[i19]) + (-1.247904e-04) * (q[i11] * q[i20]) + (-1.182786e-04) * (q[i11] * q[i21])
            + (1.123203e-04) * (q[i11] * q[i22]) + (-1.695531e-04) * (q[i12] * q[i16]) + (2.719574e-04) * (q[i12] * q[i19]) + (1.686572e-04) * (q[i12] * q[i20])
            + (-1.579868e-04) * (q[i12] * q[i21]) + (8.610818e-05) * (q[i12] * q[i22]) + (2.415881e-04) * (q[i16] * q[i19]) + (2.378795e-04) * (q[i16] * q[i20])
            + (5.657996e-05) * (q[i16] * q[i21]) + (-5.735289e-05) * (q[i16] * q[i22]) + (1.267131e-04) * (q[i19] * q[i20])
            + (-9.391578e-04) * (q[i19] * q[i21]) + (-1.083471e-04) * (q[i19] * q[i22]) + (4.373890e-05) * (q[i20] * q[i21])
            + (-9.581062e-05) * (q[i20] * q[i22]) + (-3.576275e-05) * (q[i21] * q[i22]);
   }

   public void getJQy16(double[] q, double[][] JQ)
   {
      JQ[2][i16] = (5.286144e-03) * (1) + (-1.319721e-03) * ((2) * q[i16]) + (-3.339468e-03) * (q[i0]) + (-3.948355e-03) * (q[i1]) + (-6.603870e-03) * (q[i2])
            + (1.664877e-03) * (q[i3]) + (-3.370856e-04) * (q[i4]) + (-7.180200e-04) * (q[i5]) + (5.580052e-03) * (q[i6]) + (2.848602e-03) * (q[i7])
            + (-8.621797e-03) * (q[i8]) + (1.937559e-03) * (q[i9]) + (1.021186e-03) * (q[i10]) + (9.139358e-04) * (q[i11]) + (-9.498034e-04) * (q[i12])
            + (-3.944256e-04) * (q[i15]) + (3.609896e-05) * (q[i19]) + (7.638382e-04) * (q[i20]) + (-2.607071e-04) * (q[i21]) + (-3.870956e-04) * (q[i22])
            + (-1.798960e-05) * (q[i0] * q[i0]) + (6.446987e-04) * (q[i1] * q[i1]) + (-4.365303e-05) * (q[i2] * q[i2]) + (-2.540502e-05) * (q[i3] * q[i3])
            + (5.306661e-04) * (q[i4] * q[i4]) + (-1.588174e-03) * (q[i5] * q[i5]) + (-5.331069e-04) * (q[i6] * q[i6]) + (-5.192656e-04) * (q[i7] * q[i7])
            + (2.591024e-04) * (q[i8] * q[i8]) + (-2.517296e-04) * (q[i9] * q[i9]) + (-9.715316e-05) * (q[i10] * q[i10]) + (-1.803217e-04) * (q[i11] * q[i11])
            + (-2.013504e-03) * (q[i12] * q[i12]) + (-8.299577e-05) * (q[i15] * q[i15]) + (-3.579373e-04) * ((2) * q[i0] * q[i16])
            + (-2.042699e-04) * ((2) * q[i1] * q[i16]) + (-2.696586e-03) * ((2) * q[i2] * q[i16]) + (4.782192e-05) * ((2) * q[i3] * q[i16])
            + (-6.367990e-05) * ((2) * q[i4] * q[i16]) + (5.323044e-04) * ((2) * q[i5] * q[i16]) + (7.546963e-04) * ((2) * q[i6] * q[i16])
            + (6.457506e-05) * ((2) * q[i7] * q[i16]) + (2.011981e-04) * ((2) * q[i8] * q[i16]) + (-1.199851e-04) * ((2) * q[i9] * q[i16])
            + (-1.432445e-04) * ((2) * q[i10] * q[i16]) + (-1.955120e-04) * ((2) * q[i11] * q[i16]) + (-2.063879e-03) * ((2) * q[i12] * q[i16])
            + (-8.439846e-05) * ((2) * q[i15] * q[i16]) + (3.163447e-04) * ((3) * q[i16] * q[i16]) + (-1.660521e-04) * ((2) * q[i16] * q[i19])
            + (2.685030e-04) * ((2) * q[i16] * q[i20]) + (-8.377281e-05) * ((2) * q[i16] * q[i21]) + (2.956557e-04) * ((2) * q[i16] * q[i22])
            + (-3.148092e-05) * (q[i19] * q[i19]) + (-2.465934e-04) * (q[i20] * q[i20]) + (-4.668007e-05) * (q[i21] * q[i21])
            + (-1.417031e-04) * (q[i22] * q[i22]) + (-2.161718e-04) * (q[i0] * q[i1]) + (2.344968e-04) * (q[i0] * q[i2]) + (-4.426984e-03) * (q[i0] * q[i3])
            + (1.829189e-05) * (q[i0] * q[i4]) + (-2.215502e-04) * (q[i0] * q[i5]) + (1.073967e-03) * (q[i0] * q[i6]) + (3.597124e-04) * (q[i0] * q[i7])
            + (6.730540e-04) * (q[i0] * q[i8]) + (3.160296e-04) * (q[i0] * q[i9]) + (-3.942323e-04) * (q[i0] * q[i10]) + (-3.487901e-04) * (q[i0] * q[i11])
            + (-7.891911e-05) * (q[i0] * q[i12]) + (2.723890e-04) * (q[i0] * q[i15]) + (-5.580437e-04) * (q[i0] * q[i19]) + (-3.439001e-04) * (q[i0] * q[i20])
            + (-1.036131e-04) * (q[i0] * q[i21]) + (6.974141e-04) * (q[i0] * q[i22]) + (2.506379e-04) * (q[i1] * q[i2]) + (5.096420e-05) * (q[i1] * q[i3])
            + (-1.393776e-03) * (q[i1] * q[i4]) + (-7.129600e-04) * (q[i1] * q[i5]) + (-6.491164e-04) * (q[i1] * q[i6]) + (7.710115e-05) * (q[i1] * q[i7])
            + (2.444242e-04) * (q[i1] * q[i8]) + (-3.384103e-04) * (q[i1] * q[i9]) + (3.223845e-04) * (q[i1] * q[i10]) + (1.474048e-04) * (q[i1] * q[i11])
            + (1.179321e-04) * (q[i1] * q[i12]) + (-2.649109e-04) * (q[i1] * q[i15]) + (4.275607e-05) * (q[i1] * q[i19]) + (1.031505e-04) * (q[i1] * q[i20])
            + (7.983962e-05) * (q[i1] * q[i21]) + (2.655944e-04) * (q[i1] * q[i22]) + (-2.011711e-03) * (q[i2] * q[i3]) + (7.697633e-04) * (q[i2] * q[i4])
            + (-6.482519e-03) * (q[i2] * q[i5]) + (4.216966e-04) * (q[i2] * q[i6]) + (6.447545e-04) * (q[i2] * q[i7]) + (-8.999536e-05) * (q[i2] * q[i8])
            + (-2.254603e-04) * (q[i2] * q[i9]) + (-2.443248e-04) * (q[i2] * q[i10]) + (-6.450256e-04) * (q[i2] * q[i11]) + (-2.629174e-03) * (q[i2] * q[i12])
            + (-7.984367e-08) * (q[i2] * q[i15]) + (-6.885542e-04) * (q[i2] * q[i19]) + (-2.187713e-04) * (q[i2] * q[i20]) + (2.597040e-05) * (q[i2] * q[i21])
            + (6.360450e-04) * (q[i2] * q[i22]) + (-1.473190e-04) * (q[i3] * q[i4]) + (1.103235e-03) * (q[i3] * q[i5]) + (7.018440e-04) * (q[i3] * q[i6])
            + (-9.436389e-05) * (q[i3] * q[i7]) + (9.682878e-04) * (q[i3] * q[i8]) + (-6.382520e-04) * (q[i3] * q[i9]) + (4.492608e-04) * (q[i3] * q[i10])
            + (3.615160e-04) * (q[i3] * q[i11]) + (-1.012331e-03) * (q[i3] * q[i12]) + (-1.571992e-04) * (q[i3] * q[i15]) + (-6.815973e-04) * (q[i3] * q[i19])
            + (6.176811e-05) * (q[i3] * q[i20]) + (-1.660415e-05) * (q[i3] * q[i21]) + (-1.861034e-04) * (q[i3] * q[i22]) + (-1.172226e-03) * (q[i4] * q[i5])
            + (-7.136097e-04) * (q[i4] * q[i6]) + (6.232834e-04) * (q[i4] * q[i7]) + (1.829801e-03) * (q[i4] * q[i8]) + (3.978682e-04) * (q[i4] * q[i9])
            + (3.787644e-04) * (q[i4] * q[i10]) + (-8.476519e-05) * (q[i4] * q[i11]) + (-7.855407e-04) * (q[i4] * q[i12]) + (1.578929e-04) * (q[i4] * q[i15])
            + (2.789459e-04) * (q[i4] * q[i19]) + (5.880863e-04) * (q[i4] * q[i20]) + (-3.602423e-04) * (q[i4] * q[i21]) + (-4.693425e-04) * (q[i4] * q[i22])
            + (1.956003e-04) * (q[i5] * q[i6]) + (-1.117954e-03) * (q[i5] * q[i7]) + (-1.834742e-03) * (q[i5] * q[i8]) + (-2.821584e-04) * (q[i5] * q[i9])
            + (-9.036813e-04) * (q[i5] * q[i10]) + (-1.740660e-04) * (q[i5] * q[i11]) + (-2.283075e-03) * (q[i5] * q[i12]) + (-8.659480e-07) * (q[i5] * q[i15])
            + (3.876892e-04) * (q[i5] * q[i19]) + (7.816045e-04) * (q[i5] * q[i20]) + (4.446953e-04) * (q[i5] * q[i21]) + (1.163210e-03) * (q[i5] * q[i22])
            + (4.062233e-04) * (q[i6] * q[i7]) + (-2.702556e-04) * (q[i6] * q[i8]) + (-2.863813e-04) * (q[i6] * q[i9]) + (5.037950e-04) * (q[i6] * q[i10])
            + (-2.434706e-05) * (q[i6] * q[i11]) + (-1.767328e-04) * (q[i6] * q[i12]) + (-6.177311e-05) * (q[i6] * q[i15]) + (4.438298e-04) * (q[i6] * q[i19])
            + (1.861985e-04) * (q[i6] * q[i20]) + (-4.163240e-05) * (q[i6] * q[i21]) + (-5.419059e-04) * (q[i6] * q[i22]) + (-7.646881e-06) * (q[i7] * q[i8])
            + (-3.701206e-05) * (q[i7] * q[i9]) + (-5.209704e-04) * (q[i7] * q[i10]) + (7.544601e-05) * (q[i7] * q[i11]) + (4.293758e-05) * (q[i7] * q[i12])
            + (-6.415069e-05) * (q[i7] * q[i15]) + (-3.961685e-04) * (q[i7] * q[i19]) + (-4.551602e-04) * (q[i7] * q[i20]) + (-7.387613e-05) * (q[i7] * q[i21])
            + (-6.504092e-05) * (q[i7] * q[i22]) + (2.161125e-05) * (q[i8] * q[i9]) + (-5.650428e-04) * (q[i8] * q[i10]) + (3.434920e-05) * (q[i8] * q[i11])
            + (7.420413e-04) * (q[i8] * q[i12]) + (-3.166315e-04) * (q[i8] * q[i15]) + (-7.197157e-05) * (q[i8] * q[i19]) + (4.059137e-04) * (q[i8] * q[i20])
            + (-8.746154e-05) * (q[i8] * q[i21]) + (1.347250e-03) * (q[i8] * q[i22]) + (1.512407e-04) * (q[i9] * q[i10]) + (-2.562664e-04) * (q[i9] * q[i11])
            + (4.424267e-04) * (q[i9] * q[i12]) + (1.216782e-04) * (q[i9] * q[i15]) + (2.454538e-04) * (q[i9] * q[i19]) + (7.094283e-05) * (q[i9] * q[i20])
            + (6.556024e-05) * (q[i9] * q[i21]) + (-1.441560e-04) * (q[i9] * q[i22]) + (-2.290909e-04) * (q[i10] * q[i11]) + (-3.719982e-05) * (q[i10] * q[i12])
            + (1.207736e-04) * (q[i10] * q[i15]) + (5.760996e-05) * (q[i10] * q[i19]) + (-2.040246e-04) * (q[i10] * q[i20])
            + (-5.506688e-05) * (q[i10] * q[i21]) + (-6.104315e-05) * (q[i10] * q[i22]) + (2.136035e-04) * (q[i11] * q[i12])
            + (1.670981e-04) * (q[i11] * q[i15]) + (-1.722355e-04) * (q[i11] * q[i19]) + (-2.718841e-04) * (q[i11] * q[i20])
            + (8.398881e-05) * (q[i11] * q[i21]) + (-1.625631e-04) * (q[i11] * q[i22]) + (-1.695531e-04) * (q[i12] * q[i15])
            + (1.297550e-04) * (q[i12] * q[i19]) + (7.923498e-05) * (q[i12] * q[i20]) + (1.141716e-04) * (q[i12] * q[i21]) + (-1.321650e-04) * (q[i12] * q[i22])
            + (2.415881e-04) * (q[i15] * q[i19]) + (2.378795e-04) * (q[i15] * q[i20]) + (5.657996e-05) * (q[i15] * q[i21]) + (-5.735289e-05) * (q[i15] * q[i22])
            + (1.244008e-04) * (q[i19] * q[i20]) + (9.520474e-05) * (q[i19] * q[i21]) + (-4.180483e-05) * (q[i19] * q[i22]) + (1.104112e-04) * (q[i20] * q[i21])
            + (9.323239e-04) * (q[i20] * q[i22]) + (-3.703898e-05) * (q[i21] * q[i22]);
   }

   public void getJQy19(double[] q, double[][] JQ)
   {
      JQ[2][i19] = (-1.190229e-03) * (1) + (-1.560022e-04) * ((2) * q[i19]) + (-6.674361e-04) * (q[i0]) + (6.088204e-04) * (q[i1]) + (-1.564712e-03) * (q[i2])
            + (-2.862405e-03) * (q[i3]) + (6.483261e-04) * (q[i4]) + (-7.658719e-04) * (q[i5]) + (-3.673866e-04) * (q[i6]) + (-1.261602e-03) * (q[i7])
            + (-5.022812e-04) * (q[i8]) + (1.589063e-04) * (q[i9]) + (-1.201700e-04) * (q[i10]) + (-4.620093e-04) * (q[i11]) + (6.370554e-04) * (q[i12])
            + (7.792923e-04) * (q[i15]) + (3.609896e-05) * (q[i16]) + (1.147045e-04) * (q[i20]) + (1.353645e-03) * (q[i21]) + (-3.655234e-04) * (q[i22])
            + (9.390891e-05) * (q[i0] * q[i0]) + (4.612640e-04) * (q[i1] * q[i1]) + (6.886514e-04) * (q[i2] * q[i2]) + (6.712668e-04) * (q[i3] * q[i3])
            + (-6.442424e-04) * (q[i4] * q[i4]) + (-7.619498e-04) * (q[i5] * q[i5]) + (-8.688893e-04) * (q[i6] * q[i6]) + (-2.492046e-04) * (q[i7] * q[i7])
            + (6.735998e-05) * (q[i8] * q[i8]) + (5.892796e-05) * (q[i9] * q[i9]) + (7.674123e-05) * (q[i10] * q[i10]) + (6.447725e-04) * (q[i11] * q[i11])
            + (-8.000025e-05) * (q[i12] * q[i12]) + (2.742122e-04) * (q[i15] * q[i15]) + (-1.660521e-04) * (q[i16] * q[i16])
            + (1.713251e-04) * ((2) * q[i0] * q[i19]) + (2.274663e-05) * ((2) * q[i1] * q[i19]) + (-2.059287e-04) * ((2) * q[i2] * q[i19])
            + (-6.031514e-04) * ((2) * q[i3] * q[i19]) + (6.606077e-05) * ((2) * q[i4] * q[i19]) + (-6.602141e-04) * ((2) * q[i5] * q[i19])
            + (-3.221905e-04) * ((2) * q[i6] * q[i19]) + (1.443314e-04) * ((2) * q[i7] * q[i19]) + (-2.638921e-04) * ((2) * q[i8] * q[i19])
            + (-3.016804e-05) * ((2) * q[i9] * q[i19]) + (-1.226696e-04) * ((2) * q[i10] * q[i19]) + (-4.295102e-04) * ((2) * q[i11] * q[i19])
            + (8.348564e-05) * ((2) * q[i12] * q[i19]) + (-2.575059e-04) * ((2) * q[i15] * q[i19]) + (-3.148092e-05) * ((2) * q[i16] * q[i19])
            + (2.139490e-04) * ((3) * q[i19] * q[i19]) + (3.710345e-05) * ((2) * q[i19] * q[i20]) + (-1.694254e-04) * ((2) * q[i19] * q[i21])
            + (3.185691e-05) * ((2) * q[i19] * q[i22]) + (3.678517e-05) * (q[i20] * q[i20]) + (5.828106e-04) * (q[i21] * q[i21])
            + (-9.226347e-05) * (q[i22] * q[i22]) + (-8.633194e-04) * (q[i0] * q[i1]) + (5.742787e-04) * (q[i0] * q[i2]) + (-1.230233e-03) * (q[i0] * q[i3])
            + (6.015720e-04) * (q[i0] * q[i4]) + (2.208192e-05) * (q[i0] * q[i5]) + (-1.418968e-03) * (q[i0] * q[i6]) + (-1.804753e-04) * (q[i0] * q[i7])
            + (1.101120e-04) * (q[i0] * q[i8]) + (-2.978277e-04) * (q[i0] * q[i9]) + (3.867514e-05) * (q[i0] * q[i10]) + (2.393338e-04) * (q[i0] * q[i11])
            + (-1.259175e-04) * (q[i0] * q[i12]) + (-9.798634e-05) * (q[i0] * q[i15]) + (-5.580437e-04) * (q[i0] * q[i16]) + (-3.581878e-04) * (q[i0] * q[i20])
            + (-1.261794e-04) * (q[i0] * q[i21]) + (3.998468e-06) * (q[i0] * q[i22]) + (-5.593445e-04) * (q[i1] * q[i2]) + (7.963403e-06) * (q[i1] * q[i3])
            + (-5.003371e-04) * (q[i1] * q[i4]) + (-6.364053e-04) * (q[i1] * q[i5]) + (-5.207828e-05) * (q[i1] * q[i6]) + (-1.066193e-03) * (q[i1] * q[i7])
            + (-6.624098e-04) * (q[i1] * q[i8]) + (-3.654445e-05) * (q[i1] * q[i9]) + (-6.479919e-04) * (q[i1] * q[i10]) + (-1.130834e-05) * (q[i1] * q[i11])
            + (1.497031e-04) * (q[i1] * q[i12]) + (3.470410e-04) * (q[i1] * q[i15]) + (4.275607e-05) * (q[i1] * q[i16]) + (3.652302e-04) * (q[i1] * q[i20])
            + (-2.098154e-04) * (q[i1] * q[i21]) + (1.805125e-04) * (q[i1] * q[i22]) + (-1.015795e-03) * (q[i2] * q[i3]) + (-2.286704e-04) * (q[i2] * q[i4])
            + (2.895049e-04) * (q[i2] * q[i5]) + (-2.789274e-04) * (q[i2] * q[i6]) + (-8.905544e-04) * (q[i2] * q[i7]) + (-1.453997e-05) * (q[i2] * q[i8])
            + (-3.864767e-04) * (q[i2] * q[i9]) + (-5.623259e-06) * (q[i2] * q[i10]) + (8.071551e-04) * (q[i2] * q[i11]) + (-4.714424e-04) * (q[i2] * q[i12])
            + (2.449223e-04) * (q[i2] * q[i15]) + (-6.885542e-04) * (q[i2] * q[i16]) + (7.832015e-06) * (q[i2] * q[i20]) + (-7.995270e-04) * (q[i2] * q[i21])
            + (2.315073e-05) * (q[i2] * q[i22]) + (1.669453e-03) * (q[i3] * q[i4]) + (1.788667e-04) * (q[i3] * q[i5]) + (6.625531e-04) * (q[i3] * q[i6])
            + (-9.165213e-05) * (q[i3] * q[i7]) + (2.773115e-05) * (q[i3] * q[i8]) + (1.814252e-04) * (q[i3] * q[i9]) + (2.930732e-04) * (q[i3] * q[i10])
            + (-6.880792e-04) * (q[i3] * q[i11]) + (3.051342e-04) * (q[i3] * q[i12]) + (-5.973864e-04) * (q[i3] * q[i15]) + (-6.815973e-04) * (q[i3] * q[i16])
            + (1.477329e-04) * (q[i3] * q[i20]) + (-1.040570e-04) * (q[i3] * q[i21]) + (4.657604e-04) * (q[i3] * q[i22]) + (-1.633060e-03) * (q[i4] * q[i5])
            + (-4.620975e-04) * (q[i4] * q[i6]) + (-2.170230e-04) * (q[i4] * q[i7]) + (-8.232891e-04) * (q[i4] * q[i8]) + (-4.595379e-04) * (q[i4] * q[i9])
            + (2.490965e-05) * (q[i4] * q[i10]) + (2.910371e-04) * (q[i4] * q[i11]) + (-6.787080e-05) * (q[i4] * q[i12]) + (-7.381783e-05) * (q[i4] * q[i15])
            + (2.789459e-04) * (q[i4] * q[i16]) + (-1.476584e-04) * (q[i4] * q[i20]) + (3.561740e-04) * (q[i4] * q[i21]) + (-6.224446e-04) * (q[i4] * q[i22])
            + (4.064740e-04) * (q[i5] * q[i6]) + (-2.409192e-05) * (q[i5] * q[i7]) + (-6.928540e-05) * (q[i5] * q[i8]) + (5.246723e-04) * (q[i5] * q[i9])
            + (-1.686565e-05) * (q[i5] * q[i10]) + (-2.437107e-04) * (q[i5] * q[i11]) + (-2.503387e-04) * (q[i5] * q[i12]) + (-7.873197e-04) * (q[i5] * q[i15])
            + (3.876892e-04) * (q[i5] * q[i16]) + (2.007867e-06) * (q[i5] * q[i20]) + (8.735954e-04) * (q[i5] * q[i21]) + (-1.434840e-05) * (q[i5] * q[i22])
            + (7.244586e-04) * (q[i6] * q[i7]) + (-4.789547e-05) * (q[i6] * q[i8]) + (7.655790e-05) * (q[i6] * q[i9]) + (-1.027714e-04) * (q[i6] * q[i10])
            + (-1.572757e-04) * (q[i6] * q[i11]) + (-2.336362e-04) * (q[i6] * q[i12]) + (-4.611163e-04) * (q[i6] * q[i15]) + (4.438298e-04) * (q[i6] * q[i16])
            + (-3.501248e-04) * (q[i6] * q[i20]) + (4.017529e-04) * (q[i6] * q[i21]) + (-1.984777e-04) * (q[i6] * q[i22]) + (8.224123e-05) * (q[i7] * q[i8])
            + (4.038065e-04) * (q[i7] * q[i9]) + (2.316224e-04) * (q[i7] * q[i10]) + (-1.569050e-04) * (q[i7] * q[i11]) + (-2.168496e-04) * (q[i7] * q[i12])
            + (1.873814e-04) * (q[i7] * q[i15]) + (-3.961685e-04) * (q[i7] * q[i16]) + (-3.432400e-04) * (q[i7] * q[i20]) + (-3.889145e-06) * (q[i7] * q[i21])
            + (-5.439509e-05) * (q[i7] * q[i22]) + (-2.332290e-04) * (q[i8] * q[i9]) + (2.962574e-04) * (q[i8] * q[i10]) + (4.733749e-04) * (q[i8] * q[i11])
            + (-5.292212e-04) * (q[i8] * q[i12]) + (4.024530e-04) * (q[i8] * q[i15]) + (-7.197157e-05) * (q[i8] * q[i16]) + (5.735726e-04) * (q[i8] * q[i20])
            + (-7.246433e-04) * (q[i8] * q[i21]) + (-2.846715e-04) * (q[i8] * q[i22]) + (-3.864186e-05) * (q[i9] * q[i10]) + (-2.355869e-04) * (q[i9] * q[i11])
            + (1.301948e-04) * (q[i9] * q[i12]) + (-2.019897e-04) * (q[i9] * q[i15]) + (2.454538e-04) * (q[i9] * q[i16]) + (4.018498e-05) * (q[i9] * q[i20])
            + (7.119420e-05) * (q[i9] * q[i21]) + (5.045095e-06) * (q[i9] * q[i22]) + (-2.225971e-04) * (q[i10] * q[i11]) + (-4.135810e-04) * (q[i10] * q[i12])
            + (6.720933e-05) * (q[i10] * q[i15]) + (5.760996e-05) * (q[i10] * q[i16]) + (4.563640e-05) * (q[i10] * q[i20]) + (9.115209e-05) * (q[i10] * q[i21])
            + (-2.845212e-05) * (q[i10] * q[i22]) + (-2.892577e-04) * (q[i11] * q[i12]) + (-7.325050e-05) * (q[i11] * q[i15])
            + (-1.722355e-04) * (q[i11] * q[i16]) + (1.578352e-04) * (q[i11] * q[i20]) + (9.781332e-05) * (q[i11] * q[i21])
            + (-1.896632e-05) * (q[i11] * q[i22]) + (2.719574e-04) * (q[i12] * q[i15]) + (1.297550e-04) * (q[i12] * q[i16])
            + (-1.579112e-04) * (q[i12] * q[i20]) + (-1.418660e-04) * (q[i12] * q[i21]) + (-7.484854e-05) * (q[i12] * q[i22])
            + (2.415881e-04) * (q[i15] * q[i16]) + (1.267131e-04) * (q[i15] * q[i20]) + (-9.391578e-04) * (q[i15] * q[i21])
            + (-1.083471e-04) * (q[i15] * q[i22]) + (1.244008e-04) * (q[i16] * q[i20]) + (9.520474e-05) * (q[i16] * q[i21])
            + (-4.180483e-05) * (q[i16] * q[i22]) + (1.039629e-04) * (q[i20] * q[i21]) + (-1.062036e-04) * (q[i20] * q[i22])
            + (1.894616e-05) * (q[i21] * q[i22]);
   }

   public void getJQy20(double[] q, double[][] JQ)
   {
      JQ[2][i20] = (-1.099037e-03) * (1) + (-1.454103e-04) * ((2) * q[i20]) + (-6.149881e-04) * (q[i0]) + (6.722230e-04) * (q[i1]) + (1.557533e-03) * (q[i2])
            + (-6.835361e-04) * (q[i3]) + (2.841136e-03) * (q[i4]) + (7.757564e-04) * (q[i5]) + (-1.263329e-03) * (q[i6]) + (-3.554908e-04) * (q[i7])
            + (-4.814287e-04) * (q[i8]) + (-1.205392e-04) * (q[i9]) + (1.449772e-04) * (q[i10]) + (-6.228889e-04) * (q[i11]) + (4.295278e-04) * (q[i12])
            + (5.134609e-05) * (q[i15]) + (7.638382e-04) * (q[i16]) + (1.147045e-04) * (q[i19]) + (3.754518e-04) * (q[i21]) + (-1.381182e-03) * (q[i22])
            + (4.563936e-04) * (q[i0] * q[i0]) + (9.646236e-05) * (q[i1] * q[i1]) + (6.695697e-04) * (q[i2] * q[i2]) + (-6.506100e-04) * (q[i3] * q[i3])
            + (6.694240e-04) * (q[i4] * q[i4]) + (-7.367319e-04) * (q[i5] * q[i5]) + (-2.521222e-04) * (q[i6] * q[i6]) + (-8.649150e-04) * (q[i7] * q[i7])
            + (5.945936e-05) * (q[i8] * q[i8]) + (7.640899e-05) * (q[i9] * q[i9]) + (5.754956e-05) * (q[i10] * q[i10]) + (-8.472641e-05) * (q[i11] * q[i11])
            + (6.312400e-04) * (q[i12] * q[i12]) + (-1.641042e-04) * (q[i15] * q[i15]) + (2.685030e-04) * (q[i16] * q[i16]) + (3.710345e-05) * (q[i19] * q[i19])
            + (-2.899285e-05) * ((2) * q[i0] * q[i20]) + (-1.739136e-04) * ((2) * q[i1] * q[i20]) + (2.077010e-04) * ((2) * q[i2] * q[i20])
            + (-6.408954e-05) * ((2) * q[i3] * q[i20]) + (5.927220e-04) * ((2) * q[i4] * q[i20]) + (6.640720e-04) * ((2) * q[i5] * q[i20])
            + (1.437983e-04) * ((2) * q[i6] * q[i20]) + (-3.164190e-04) * ((2) * q[i7] * q[i20]) + (-2.617898e-04) * ((2) * q[i8] * q[i20])
            + (-1.238043e-04) * ((2) * q[i9] * q[i20]) + (-3.093279e-05) * ((2) * q[i10] * q[i20]) + (-8.136337e-05) * ((2) * q[i11] * q[i20])
            + (4.310497e-04) * ((2) * q[i12] * q[i20]) + (-3.405803e-05) * ((2) * q[i15] * q[i20]) + (-2.465934e-04) * ((2) * q[i16] * q[i20])
            + (3.678517e-05) * ((2) * q[i19] * q[i20]) + (2.096029e-04) * ((3) * q[i20] * q[i20]) + (-2.987384e-05) * ((2) * q[i20] * q[i21])
            + (1.685686e-04) * ((2) * q[i20] * q[i22]) + (-9.207259e-05) * (q[i21] * q[i21]) + (5.922148e-04) * (q[i22] * q[i22])
            + (-8.676393e-04) * (q[i0] * q[i1]) + (-5.652489e-04) * (q[i0] * q[i2]) + (-4.987277e-04) * (q[i0] * q[i3]) + (8.366387e-06) * (q[i0] * q[i4])
            + (-6.243246e-04) * (q[i0] * q[i5]) + (1.062149e-03) * (q[i0] * q[i6]) + (6.153951e-05) * (q[i0] * q[i7]) + (6.534280e-04) * (q[i0] * q[i8])
            + (6.484969e-04) * (q[i0] * q[i9]) + (3.980839e-05) * (q[i0] * q[i10]) + (1.528574e-04) * (q[i0] * q[i11]) + (-1.010733e-05) * (q[i0] * q[i12])
            + (-5.472361e-05) * (q[i0] * q[i15]) + (-3.439001e-04) * (q[i0] * q[i16]) + (-3.581878e-04) * (q[i0] * q[i19]) + (1.815776e-04) * (q[i0] * q[i21])
            + (-2.077979e-04) * (q[i0] * q[i22]) + (5.744647e-04) * (q[i1] * q[i2]) + (6.092463e-04) * (q[i1] * q[i3]) + (-1.224390e-03) * (q[i1] * q[i4])
            + (3.675125e-06) * (q[i1] * q[i5]) + (1.743489e-04) * (q[i1] * q[i6]) + (1.409631e-03) * (q[i1] * q[i7]) + (-1.026070e-04) * (q[i1] * q[i8])
            + (-3.530560e-05) * (q[i1] * q[i9]) + (2.965278e-04) * (q[i1] * q[i10]) + (-1.253673e-04) * (q[i1] * q[i11]) + (2.454978e-04) * (q[i1] * q[i12])
            + (5.617706e-04) * (q[i1] * q[i15]) + (1.031505e-04) * (q[i1] * q[i16]) + (3.652302e-04) * (q[i1] * q[i19]) + (6.865431e-06) * (q[i1] * q[i21])
            + (-1.258166e-04) * (q[i1] * q[i22]) + (-2.322656e-04) * (q[i2] * q[i3]) + (-1.016435e-03) * (q[i2] * q[i4]) + (2.973489e-04) * (q[i2] * q[i5])
            + (8.815289e-04) * (q[i2] * q[i6]) + (2.646359e-04) * (q[i2] * q[i7]) + (2.125620e-05) * (q[i2] * q[i8]) + (-1.316529e-07) * (q[i2] * q[i9])
            + (3.850246e-04) * (q[i2] * q[i10]) + (-4.746056e-04) * (q[i2] * q[i11]) + (7.961283e-04) * (q[i2] * q[i12]) + (6.778326e-04) * (q[i2] * q[i15])
            + (-2.187713e-04) * (q[i2] * q[i16]) + (7.832015e-06) * (q[i2] * q[i19]) + (2.628485e-05) * (q[i2] * q[i21]) + (-7.908005e-04) * (q[i2] * q[i22])
            + (1.654502e-03) * (q[i3] * q[i4]) + (-1.627474e-03) * (q[i3] * q[i5]) + (2.179492e-04) * (q[i3] * q[i6]) + (4.630918e-04) * (q[i3] * q[i7])
            + (8.255406e-04) * (q[i3] * q[i8]) + (-2.641220e-05) * (q[i3] * q[i9]) + (4.740704e-04) * (q[i3] * q[i10]) + (-5.652306e-05) * (q[i3] * q[i11])
            + (3.018533e-04) * (q[i3] * q[i12]) + (-2.824218e-04) * (q[i3] * q[i15]) + (6.176811e-05) * (q[i3] * q[i16]) + (1.477329e-04) * (q[i3] * q[i19])
            + (-6.336138e-04) * (q[i3] * q[i21]) + (3.488995e-04) * (q[i3] * q[i22]) + (1.739405e-04) * (q[i4] * q[i5]) + (9.937660e-05) * (q[i4] * q[i6])
            + (-6.638632e-04) * (q[i4] * q[i7]) + (-3.856416e-05) * (q[i4] * q[i8]) + (-2.947117e-04) * (q[i4] * q[i9]) + (-1.768820e-04) * (q[i4] * q[i10])
            + (3.013094e-04) * (q[i4] * q[i11]) + (-6.834487e-04) * (q[i4] * q[i12]) + (6.806984e-04) * (q[i4] * q[i15]) + (5.880863e-04) * (q[i4] * q[i16])
            + (-1.476584e-04) * (q[i4] * q[i19]) + (4.618507e-04) * (q[i4] * q[i21]) + (-1.077073e-04) * (q[i4] * q[i22]) + (6.454644e-06) * (q[i5] * q[i6])
            + (-3.989730e-04) * (q[i5] * q[i7]) + (7.243788e-05) * (q[i5] * q[i8]) + (1.785773e-05) * (q[i5] * q[i9]) + (-5.345133e-04) * (q[i5] * q[i10])
            + (-2.532027e-04) * (q[i5] * q[i11]) + (-2.516082e-04) * (q[i5] * q[i12]) + (-3.741474e-04) * (q[i5] * q[i15]) + (7.816045e-04) * (q[i5] * q[i16])
            + (2.007867e-06) * (q[i5] * q[i19]) + (-7.502974e-06) * (q[i5] * q[i21]) + (8.731083e-04) * (q[i5] * q[i22]) + (7.125068e-04) * (q[i6] * q[i7])
            + (8.448996e-05) * (q[i6] * q[i8]) + (2.288140e-04) * (q[i6] * q[i9]) + (3.982749e-04) * (q[i6] * q[i10]) + (2.184652e-04) * (q[i6] * q[i11])
            + (1.541868e-04) * (q[i6] * q[i12]) + (-3.996412e-04) * (q[i6] * q[i15]) + (1.861985e-04) * (q[i6] * q[i16]) + (-3.501248e-04) * (q[i6] * q[i19])
            + (5.300866e-05) * (q[i6] * q[i21]) + (2.983694e-06) * (q[i6] * q[i22]) + (-3.455847e-05) * (q[i7] * q[i8]) + (-1.078364e-04) * (q[i7] * q[i9])
            + (7.333906e-05) * (q[i7] * q[i10]) + (2.369777e-04) * (q[i7] * q[i11]) + (1.536198e-04) * (q[i7] * q[i12]) + (4.450782e-04) * (q[i7] * q[i15])
            + (-4.551602e-04) * (q[i7] * q[i16]) + (-3.432400e-04) * (q[i7] * q[i19]) + (2.009673e-04) * (q[i7] * q[i21]) + (-3.973034e-04) * (q[i7] * q[i22])
            + (2.934882e-04) * (q[i8] * q[i9]) + (-2.234525e-04) * (q[i8] * q[i10]) + (5.237842e-04) * (q[i8] * q[i11]) + (-4.641388e-04) * (q[i8] * q[i12])
            + (-6.814829e-05) * (q[i8] * q[i15]) + (4.059137e-04) * (q[i8] * q[i16]) + (5.735726e-04) * (q[i8] * q[i19]) + (2.859612e-04) * (q[i8] * q[i21])
            + (7.191502e-04) * (q[i8] * q[i22]) + (-3.865974e-05) * (q[i9] * q[i10]) + (4.099562e-04) * (q[i9] * q[i11]) + (2.181249e-04) * (q[i9] * q[i12])
            + (5.364623e-05) * (q[i9] * q[i15]) + (7.094283e-05) * (q[i9] * q[i16]) + (4.018498e-05) * (q[i9] * q[i19]) + (2.715758e-05) * (q[i9] * q[i21])
            + (-9.038603e-05) * (q[i9] * q[i22]) + (-1.278631e-04) * (q[i10] * q[i11]) + (2.268484e-04) * (q[i10] * q[i12]) + (2.440403e-04) * (q[i10] * q[i15])
            + (-2.040246e-04) * (q[i10] * q[i16]) + (4.563640e-05) * (q[i10] * q[i19]) + (-2.515620e-06) * (q[i10] * q[i21])
            + (-6.493948e-05) * (q[i10] * q[i22]) + (-2.910180e-04) * (q[i11] * q[i12]) + (-1.247904e-04) * (q[i11] * q[i15])
            + (-2.718841e-04) * (q[i11] * q[i16]) + (1.578352e-04) * (q[i11] * q[i19]) + (-7.191706e-05) * (q[i11] * q[i21])
            + (-1.398552e-04) * (q[i11] * q[i22]) + (1.686572e-04) * (q[i12] * q[i15]) + (7.923498e-05) * (q[i12] * q[i16])
            + (-1.579112e-04) * (q[i12] * q[i19]) + (-2.069570e-05) * (q[i12] * q[i21]) + (1.036630e-04) * (q[i12] * q[i22])
            + (2.378795e-04) * (q[i15] * q[i16]) + (1.267131e-04) * (q[i15] * q[i19]) + (4.373890e-05) * (q[i15] * q[i21]) + (-9.581062e-05) * (q[i15] * q[i22])
            + (1.244008e-04) * (q[i16] * q[i19]) + (1.104112e-04) * (q[i16] * q[i21]) + (9.323239e-04) * (q[i16] * q[i22]) + (1.039629e-04) * (q[i19] * q[i21])
            + (-1.062036e-04) * (q[i19] * q[i22]) + (1.709314e-05) * (q[i21] * q[i22]);
   }

   public void getJQy21(double[] q, double[][] JQ)
   {
      JQ[2][i21] = (4.107771e-03) * (1) + (-6.221178e-04) * ((2) * q[i21]) + (1.730498e-03) * (q[i0]) + (1.017126e-03) * (q[i1]) + (1.801057e-03) * (q[i2])
            + (-1.083523e-03) * (q[i3]) + (1.670042e-04) * (q[i4]) + (1.357518e-03) * (q[i5]) + (5.394388e-04) * (q[i6]) + (-8.707822e-05) * (q[i7])
            + (-2.099477e-03) * (q[i8]) + (-6.516309e-04) * (q[i9]) + (-5.336681e-04) * (q[i10]) + (-1.495999e-03) * (q[i11]) + (3.438006e-04) * (q[i12])
            + (3.841905e-04) * (q[i15]) + (-2.607071e-04) * (q[i16]) + (1.353645e-03) * (q[i19]) + (3.754518e-04) * (q[i20]) + (-9.737342e-05) * (q[i22])
            + (1.672795e-04) * (q[i0] * q[i0]) + (-1.517087e-04) * (q[i1] * q[i1]) + (1.455581e-04) * (q[i2] * q[i2]) + (-2.625181e-04) * (q[i3] * q[i3])
            + (-1.053468e-04) * (q[i4] * q[i4]) + (-3.578230e-04) * (q[i5] * q[i5]) + (-3.175184e-04) * (q[i6] * q[i6]) + (-4.481974e-04) * (q[i7] * q[i7])
            + (-4.600819e-04) * (q[i8] * q[i8]) + (7.150478e-05) * (q[i9] * q[i9]) + (1.589391e-04) * (q[i10] * q[i10]) + (1.849186e-05) * (q[i11] * q[i11])
            + (-6.533212e-05) * (q[i12] * q[i12]) + (-2.893991e-04) * (q[i15] * q[i15]) + (-8.377281e-05) * (q[i16] * q[i16])
            + (-1.694254e-04) * (q[i19] * q[i19]) + (-2.987384e-05) * (q[i20] * q[i20]) + (6.082433e-05) * ((2) * q[i0] * q[i21])
            + (1.276385e-04) * ((2) * q[i1] * q[i21]) + (3.646947e-04) * ((2) * q[i2] * q[i21]) + (1.764318e-04) * ((2) * q[i3] * q[i21])
            + (2.558276e-04) * ((2) * q[i4] * q[i21]) + (-6.442669e-04) * ((2) * q[i5] * q[i21]) + (1.089340e-04) * ((2) * q[i6] * q[i21])
            + (4.041422e-04) * ((2) * q[i7] * q[i21]) + (-8.752838e-04) * ((2) * q[i8] * q[i21]) + (2.229091e-06) * ((2) * q[i9] * q[i21])
            + (8.894858e-06) * ((2) * q[i10] * q[i21]) + (-3.551176e-04) * ((2) * q[i11] * q[i21]) + (-9.283926e-05) * ((2) * q[i12] * q[i21])
            + (-1.400345e-04) * ((2) * q[i15] * q[i21]) + (-4.668007e-05) * ((2) * q[i16] * q[i21]) + (5.828106e-04) * ((2) * q[i19] * q[i21])
            + (-9.207259e-05) * ((2) * q[i20] * q[i21]) + (-7.129647e-05) * ((3) * q[i21] * q[i21]) + (-1.070164e-05) * ((2) * q[i21] * q[i22])
            + (9.836954e-06) * (q[i22] * q[i22]) + (4.477294e-04) * (q[i0] * q[i1]) + (2.171232e-04) * (q[i0] * q[i2]) + (-4.396539e-04) * (q[i0] * q[i3])
            + (-2.850994e-04) * (q[i0] * q[i4]) + (-5.182215e-04) * (q[i0] * q[i5]) + (4.109724e-04) * (q[i0] * q[i6]) + (-2.018885e-04) * (q[i0] * q[i7])
            + (-1.374253e-04) * (q[i0] * q[i8]) + (-1.291727e-04) * (q[i0] * q[i9]) + (-5.573833e-05) * (q[i0] * q[i10]) + (-1.610796e-04) * (q[i0] * q[i11])
            + (-1.028225e-04) * (q[i0] * q[i12]) + (2.581117e-04) * (q[i0] * q[i15]) + (-1.036131e-04) * (q[i0] * q[i16]) + (-1.261794e-04) * (q[i0] * q[i19])
            + (1.815776e-04) * (q[i0] * q[i20]) + (-4.586401e-04) * (q[i0] * q[i22]) + (8.894065e-04) * (q[i1] * q[i2]) + (7.005373e-04) * (q[i1] * q[i3])
            + (1.144783e-03) * (q[i1] * q[i4]) + (-6.627542e-04) * (q[i1] * q[i5]) + (-5.344456e-04) * (q[i1] * q[i6]) + (2.247653e-04) * (q[i1] * q[i7])
            + (2.896848e-05) * (q[i1] * q[i8]) + (-2.388953e-04) * (q[i1] * q[i9]) + (-1.044877e-04) * (q[i1] * q[i10]) + (1.847853e-04) * (q[i1] * q[i11])
            + (-3.210245e-04) * (q[i1] * q[i12]) + (6.902543e-04) * (q[i1] * q[i15]) + (7.983962e-05) * (q[i1] * q[i16]) + (-2.098154e-04) * (q[i1] * q[i19])
            + (6.865431e-06) * (q[i1] * q[i20]) + (4.540418e-04) * (q[i1] * q[i22]) + (-7.376429e-04) * (q[i2] * q[i3]) + (-1.849737e-04) * (q[i2] * q[i4])
            + (8.994120e-04) * (q[i2] * q[i5]) + (4.687912e-04) * (q[i2] * q[i6]) + (-7.990804e-05) * (q[i2] * q[i7]) + (-2.847479e-04) * (q[i2] * q[i8])
            + (-2.489416e-04) * (q[i2] * q[i9]) + (-2.336341e-04) * (q[i2] * q[i10]) + (6.157953e-04) * (q[i2] * q[i11]) + (-4.648714e-04) * (q[i2] * q[i12])
            + (6.348229e-04) * (q[i2] * q[i15]) + (2.597040e-05) * (q[i2] * q[i16]) + (-7.995270e-04) * (q[i2] * q[i19]) + (2.628485e-05) * (q[i2] * q[i20])
            + (-2.930688e-06) * (q[i2] * q[i22]) + (-7.266920e-04) * (q[i3] * q[i4]) + (2.501059e-04) * (q[i3] * q[i5]) + (-7.358712e-06) * (q[i3] * q[i6])
            + (-9.779688e-05) * (q[i3] * q[i7]) + (1.261761e-04) * (q[i3] * q[i8]) + (2.187432e-04) * (q[i3] * q[i9]) + (-8.036297e-05) * (q[i3] * q[i10])
            + (-8.779510e-04) * (q[i3] * q[i11]) + (2.076512e-04) * (q[i3] * q[i12]) + (-4.609274e-04) * (q[i3] * q[i15]) + (-1.660415e-05) * (q[i3] * q[i16])
            + (-1.040570e-04) * (q[i3] * q[i19]) + (-6.336138e-04) * (q[i3] * q[i20]) + (1.227214e-04) * (q[i3] * q[i22]) + (1.047099e-03) * (q[i4] * q[i5])
            + (7.054472e-04) * (q[i4] * q[i6]) + (1.071998e-04) * (q[i4] * q[i7]) + (6.825732e-04) * (q[i4] * q[i8]) + (-1.234635e-04) * (q[i4] * q[i9])
            + (2.002999e-04) * (q[i4] * q[i10]) + (-3.079492e-04) * (q[i4] * q[i11]) + (2.436982e-04) * (q[i4] * q[i12]) + (-1.828871e-04) * (q[i4] * q[i15])
            + (-3.602423e-04) * (q[i4] * q[i16]) + (3.561740e-04) * (q[i4] * q[i19]) + (4.618507e-04) * (q[i4] * q[i20]) + (-1.220856e-04) * (q[i4] * q[i22])
            + (-1.304700e-04) * (q[i5] * q[i6]) + (9.784063e-05) * (q[i5] * q[i7]) + (1.485025e-04) * (q[i5] * q[i8]) + (-5.585385e-04) * (q[i5] * q[i9])
            + (-2.027584e-04) * (q[i5] * q[i10]) + (-1.165125e-04) * (q[i5] * q[i11]) + (-1.002124e-04) * (q[i5] * q[i12]) + (1.155737e-03) * (q[i5] * q[i15])
            + (4.446953e-04) * (q[i5] * q[i16]) + (8.735954e-04) * (q[i5] * q[i19]) + (-7.502974e-06) * (q[i5] * q[i20]) + (-4.321930e-06) * (q[i5] * q[i22])
            + (-3.170226e-04) * (q[i6] * q[i7]) + (9.735872e-05) * (q[i6] * q[i8]) + (-4.923126e-04) * (q[i6] * q[i9]) + (-1.984213e-04) * (q[i6] * q[i10])
            + (-6.219832e-05) * (q[i6] * q[i11]) + (2.803087e-04) * (q[i6] * q[i12]) + (7.036249e-05) * (q[i6] * q[i15]) + (-4.163240e-05) * (q[i6] * q[i16])
            + (4.017529e-04) * (q[i6] * q[i19]) + (5.300866e-05) * (q[i6] * q[i20]) + (1.353639e-04) * (q[i6] * q[i22]) + (-1.377254e-04) * (q[i7] * q[i8])
            + (-1.936129e-04) * (q[i7] * q[i9]) + (7.155125e-05) * (q[i7] * q[i10]) + (-2.603405e-04) * (q[i7] * q[i11]) + (-1.445232e-04) * (q[i7] * q[i12])
            + (5.336785e-04) * (q[i7] * q[i15]) + (-7.387613e-05) * (q[i7] * q[i16]) + (-3.889145e-06) * (q[i7] * q[i19]) + (2.009673e-04) * (q[i7] * q[i20])
            + (1.395011e-04) * (q[i7] * q[i22]) + (1.329135e-04) * (q[i8] * q[i9]) + (1.593766e-04) * (q[i8] * q[i10]) + (-1.172496e-04) * (q[i8] * q[i11])
            + (4.989940e-06) * (q[i8] * q[i12]) + (-1.336037e-03) * (q[i8] * q[i15]) + (-8.746154e-05) * (q[i8] * q[i16]) + (-7.246433e-04) * (q[i8] * q[i19])
            + (2.859612e-04) * (q[i8] * q[i20]) + (-9.113027e-05) * (q[i8] * q[i22]) + (-5.422781e-05) * (q[i9] * q[i10]) + (-1.556964e-04) * (q[i9] * q[i11])
            + (-1.273016e-04) * (q[i9] * q[i12]) + (6.224095e-05) * (q[i9] * q[i15]) + (6.556024e-05) * (q[i9] * q[i16]) + (7.119420e-05) * (q[i9] * q[i19])
            + (2.715758e-05) * (q[i9] * q[i20]) + (4.638653e-05) * (q[i9] * q[i22]) + (2.454380e-06) * (q[i10] * q[i11]) + (2.327850e-05) * (q[i10] * q[i12])
            + (1.405013e-04) * (q[i10] * q[i15]) + (-5.506688e-05) * (q[i10] * q[i16]) + (9.115209e-05) * (q[i10] * q[i19])
            + (-2.515620e-06) * (q[i10] * q[i20]) + (4.477543e-05) * (q[i10] * q[i22]) + (-2.512629e-05) * (q[i11] * q[i12])
            + (-1.182786e-04) * (q[i11] * q[i15]) + (8.398881e-05) * (q[i11] * q[i16]) + (9.781332e-05) * (q[i11] * q[i19])
            + (-7.191706e-05) * (q[i11] * q[i20]) + (1.085750e-04) * (q[i11] * q[i22]) + (-1.579868e-04) * (q[i12] * q[i15])
            + (1.141716e-04) * (q[i12] * q[i16]) + (-1.418660e-04) * (q[i12] * q[i19]) + (-2.069570e-05) * (q[i12] * q[i20])
            + (-1.105546e-04) * (q[i12] * q[i22]) + (5.657996e-05) * (q[i15] * q[i16]) + (-9.391578e-04) * (q[i15] * q[i19])
            + (4.373890e-05) * (q[i15] * q[i20]) + (-3.576275e-05) * (q[i15] * q[i22]) + (9.520474e-05) * (q[i16] * q[i19]) + (1.104112e-04) * (q[i16] * q[i20])
            + (-3.703898e-05) * (q[i16] * q[i22]) + (1.039629e-04) * (q[i19] * q[i20]) + (1.894616e-05) * (q[i19] * q[i22])
            + (1.709314e-05) * (q[i20] * q[i22]);
   }

   public void getJQy22(double[] q, double[][] JQ)
   {
      JQ[2][i22] = (-4.119158e-03) * (1) + (-6.224303e-04) * ((2) * q[i22]) + (1.026309e-03) * (q[i0]) + (1.730079e-03) * (q[i1]) + (1.770597e-03) * (q[i2])
            + (1.513340e-04) * (q[i3]) + (-1.072809e-03) * (q[i4]) + (1.335329e-03) * (q[i5]) + (8.421602e-05) * (q[i6]) + (-5.215120e-04) * (q[i7])
            + (2.088474e-03) * (q[i8]) + (5.353539e-04) * (q[i9]) + (6.467833e-04) * (q[i10]) + (3.249641e-04) * (q[i11]) + (-1.516772e-03) * (q[i12])
            + (2.592284e-04) * (q[i15]) + (-3.870956e-04) * (q[i16]) + (-3.655234e-04) * (q[i19]) + (-1.381182e-03) * (q[i20]) + (-9.737342e-05) * (q[i21])
            + (1.590001e-04) * (q[i0] * q[i0]) + (-1.698214e-04) * (q[i1] * q[i1]) + (-1.499381e-04) * (q[i2] * q[i2]) + (1.063758e-04) * (q[i3] * q[i3])
            + (2.582947e-04) * (q[i4] * q[i4]) + (3.657845e-04) * (q[i5] * q[i5]) + (4.531219e-04) * (q[i6] * q[i6]) + (3.133449e-04) * (q[i7] * q[i7])
            + (4.633328e-04) * (q[i8] * q[i8]) + (-1.594535e-04) * (q[i9] * q[i9]) + (-7.038176e-05) * (q[i10] * q[i10]) + (6.243942e-05) * (q[i11] * q[i11])
            + (-1.222734e-05) * (q[i12] * q[i12]) + (8.124079e-05) * (q[i15] * q[i15]) + (2.956557e-04) * (q[i16] * q[i16]) + (3.185691e-05) * (q[i19] * q[i19])
            + (1.685686e-04) * (q[i20] * q[i20]) + (-1.070164e-05) * (q[i21] * q[i21]) + (-1.294115e-04) * ((2) * q[i0] * q[i22])
            + (-5.942282e-05) * ((2) * q[i1] * q[i22]) + (-3.571494e-04) * ((2) * q[i2] * q[i22]) + (-2.551087e-04) * ((2) * q[i3] * q[i22])
            + (-1.807228e-04) * ((2) * q[i4] * q[i22]) + (6.537732e-04) * ((2) * q[i5] * q[i22]) + (4.046895e-04) * ((2) * q[i6] * q[i22])
            + (1.068850e-04) * ((2) * q[i7] * q[i22]) + (-8.719102e-04) * ((2) * q[i8] * q[i22]) + (7.763999e-06) * ((2) * q[i9] * q[i22])
            + (6.995825e-07) * ((2) * q[i10] * q[i22]) + (9.220215e-05) * ((2) * q[i11] * q[i22]) + (3.677878e-04) * ((2) * q[i12] * q[i22])
            + (-4.611770e-05) * ((2) * q[i15] * q[i22]) + (-1.417031e-04) * ((2) * q[i16] * q[i22]) + (-9.226347e-05) * ((2) * q[i19] * q[i22])
            + (5.922148e-04) * ((2) * q[i20] * q[i22]) + (9.836954e-06) * ((2) * q[i21] * q[i22]) + (7.309101e-05) * ((3) * q[i22] * q[i22])
            + (-4.469170e-04) * (q[i0] * q[i1]) + (-8.866820e-04) * (q[i0] * q[i2]) + (-1.155803e-03) * (q[i0] * q[i3]) + (-6.946741e-04) * (q[i0] * q[i4])
            + (6.836454e-04) * (q[i0] * q[i5]) + (2.203350e-04) * (q[i0] * q[i6]) + (-5.363128e-04) * (q[i0] * q[i7]) + (3.502593e-05) * (q[i0] * q[i8])
            + (-1.041448e-04) * (q[i0] * q[i9]) + (-2.403846e-04) * (q[i0] * q[i10]) + (3.191131e-04) * (q[i0] * q[i11]) + (-1.881295e-04) * (q[i0] * q[i12])
            + (8.102358e-05) * (q[i0] * q[i15]) + (6.974141e-04) * (q[i0] * q[i16]) + (3.998468e-06) * (q[i0] * q[i19]) + (-2.077979e-04) * (q[i0] * q[i20])
            + (-4.586401e-04) * (q[i0] * q[i21]) + (-2.208451e-04) * (q[i1] * q[i2]) + (2.812319e-04) * (q[i1] * q[i3]) + (4.340912e-04) * (q[i1] * q[i4])
            + (5.078706e-04) * (q[i1] * q[i5]) + (-1.932415e-04) * (q[i1] * q[i6]) + (4.137149e-04) * (q[i1] * q[i7]) + (-1.338097e-04) * (q[i1] * q[i8])
            + (-5.476873e-05) * (q[i1] * q[i9]) + (-1.251050e-04) * (q[i1] * q[i10]) + (1.039093e-04) * (q[i1] * q[i11]) + (1.659558e-04) * (q[i1] * q[i12])
            + (-1.032416e-04) * (q[i1] * q[i15]) + (2.655944e-04) * (q[i1] * q[i16]) + (1.805125e-04) * (q[i1] * q[i19]) + (-1.258166e-04) * (q[i1] * q[i20])
            + (4.540418e-04) * (q[i1] * q[i21]) + (1.911578e-04) * (q[i2] * q[i3]) + (7.318842e-04) * (q[i2] * q[i4]) + (-9.153289e-04) * (q[i2] * q[i5])
            + (-7.638846e-05) * (q[i2] * q[i6]) + (4.671243e-04) * (q[i2] * q[i7]) + (-2.826212e-04) * (q[i2] * q[i8]) + (-2.295709e-04) * (q[i2] * q[i9])
            + (-2.438366e-04) * (q[i2] * q[i10]) + (4.651000e-04) * (q[i2] * q[i11]) + (-6.117027e-04) * (q[i2] * q[i12]) + (2.914068e-05) * (q[i2] * q[i15])
            + (6.360450e-04) * (q[i2] * q[i16]) + (2.315073e-05) * (q[i2] * q[i19]) + (-7.908005e-04) * (q[i2] * q[i20]) + (-2.930688e-06) * (q[i2] * q[i21])
            + (7.298847e-04) * (q[i3] * q[i4]) + (-1.042687e-03) * (q[i3] * q[i5]) + (1.187888e-04) * (q[i3] * q[i6]) + (7.096689e-04) * (q[i3] * q[i7])
            + (6.748273e-04) * (q[i3] * q[i8]) + (2.044403e-04) * (q[i3] * q[i9]) + (-1.194938e-04) * (q[i3] * q[i10]) + (-2.416514e-04) * (q[i3] * q[i11])
            + (2.997615e-04) * (q[i3] * q[i12]) + (-3.611729e-04) * (q[i3] * q[i15]) + (-1.861034e-04) * (q[i3] * q[i16]) + (4.657604e-04) * (q[i3] * q[i19])
            + (3.488995e-04) * (q[i3] * q[i20]) + (1.227214e-04) * (q[i3] * q[i21]) + (-2.370044e-04) * (q[i4] * q[i5]) + (-1.034558e-04) * (q[i4] * q[i6])
            + (-5.496944e-06) * (q[i4] * q[i7]) + (1.317616e-04) * (q[i4] * q[i8]) + (-8.447726e-05) * (q[i4] * q[i9]) + (2.178710e-04) * (q[i4] * q[i10])
            + (-2.066231e-04) * (q[i4] * q[i11]) + (8.872700e-04) * (q[i4] * q[i12]) + (-2.002160e-05) * (q[i4] * q[i15]) + (-4.693425e-04) * (q[i4] * q[i16])
            + (-6.224446e-04) * (q[i4] * q[i19]) + (-1.077073e-04) * (q[i4] * q[i20]) + (-1.220856e-04) * (q[i4] * q[i21]) + (1.018957e-04) * (q[i5] * q[i6])
            + (-1.198239e-04) * (q[i5] * q[i7]) + (1.510359e-04) * (q[i5] * q[i8]) + (-2.062318e-04) * (q[i5] * q[i9]) + (-5.498457e-04) * (q[i5] * q[i10])
            + (1.027526e-04) * (q[i5] * q[i11]) + (1.310858e-04) * (q[i5] * q[i12]) + (4.391866e-04) * (q[i5] * q[i15]) + (1.163210e-03) * (q[i5] * q[i16])
            + (-1.434840e-05) * (q[i5] * q[i19]) + (8.731083e-04) * (q[i5] * q[i20]) + (-4.321930e-06) * (q[i5] * q[i21]) + (3.047756e-04) * (q[i6] * q[i7])
            + (1.362510e-04) * (q[i6] * q[i8]) + (-7.035101e-05) * (q[i6] * q[i9]) + (1.941054e-04) * (q[i6] * q[i10]) + (-1.467622e-04) * (q[i6] * q[i11])
            + (-2.579998e-04) * (q[i6] * q[i12]) + (7.993119e-05) * (q[i6] * q[i15]) + (-5.419059e-04) * (q[i6] * q[i16]) + (-1.984777e-04) * (q[i6] * q[i19])
            + (2.983694e-06) * (q[i6] * q[i20]) + (1.353639e-04) * (q[i6] * q[i21]) + (-8.658386e-05) * (q[i7] * q[i8]) + (1.932345e-04) * (q[i7] * q[i9])
            + (4.919634e-04) * (q[i7] * q[i10]) + (2.806518e-04) * (q[i7] * q[i11]) + (-5.375323e-05) * (q[i7] * q[i12]) + (4.000195e-05) * (q[i7] * q[i15])
            + (-6.504092e-05) * (q[i7] * q[i16]) + (-5.439509e-05) * (q[i7] * q[i19]) + (-3.973034e-04) * (q[i7] * q[i20]) + (1.395011e-04) * (q[i7] * q[i21])
            + (-1.614691e-04) * (q[i8] * q[i9]) + (-1.333644e-04) * (q[i8] * q[i10]) + (1.615886e-06) * (q[i8] * q[i11]) + (-1.251570e-04) * (q[i8] * q[i12])
            + (8.554276e-05) * (q[i8] * q[i15]) + (1.347250e-03) * (q[i8] * q[i16]) + (-2.846715e-04) * (q[i8] * q[i19]) + (7.191502e-04) * (q[i8] * q[i20])
            + (-9.113027e-05) * (q[i8] * q[i21]) + (5.440258e-05) * (q[i9] * q[i10]) + (2.365417e-05) * (q[i9] * q[i11]) + (6.292190e-06) * (q[i9] * q[i12])
            + (5.498908e-05) * (q[i9] * q[i15]) + (-1.441560e-04) * (q[i9] * q[i16]) + (5.045095e-06) * (q[i9] * q[i19]) + (-9.038603e-05) * (q[i9] * q[i20])
            + (4.638653e-05) * (q[i9] * q[i21]) + (-1.242732e-04) * (q[i10] * q[i11]) + (-1.499846e-04) * (q[i10] * q[i12])
            + (-6.600514e-05) * (q[i10] * q[i15]) + (-6.104315e-05) * (q[i10] * q[i16]) + (-2.845212e-05) * (q[i10] * q[i19])
            + (-6.493948e-05) * (q[i10] * q[i20]) + (4.477543e-05) * (q[i10] * q[i21]) + (2.919643e-05) * (q[i11] * q[i12]) + (1.123203e-04) * (q[i11] * q[i15])
            + (-1.625631e-04) * (q[i11] * q[i16]) + (-1.896632e-05) * (q[i11] * q[i19]) + (-1.398552e-04) * (q[i11] * q[i20])
            + (1.085750e-04) * (q[i11] * q[i21]) + (8.610818e-05) * (q[i12] * q[i15]) + (-1.321650e-04) * (q[i12] * q[i16])
            + (-7.484854e-05) * (q[i12] * q[i19]) + (1.036630e-04) * (q[i12] * q[i20]) + (-1.105546e-04) * (q[i12] * q[i21])
            + (-5.735289e-05) * (q[i15] * q[i16]) + (-1.083471e-04) * (q[i15] * q[i19]) + (-9.581062e-05) * (q[i15] * q[i20])
            + (-3.576275e-05) * (q[i15] * q[i21]) + (-4.180483e-05) * (q[i16] * q[i19]) + (9.323239e-04) * (q[i16] * q[i20])
            + (-3.703898e-05) * (q[i16] * q[i21]) + (-1.062036e-04) * (q[i19] * q[i20]) + (1.894616e-05) * (q[i19] * q[i21])
            + (1.709314e-05) * (q[i20] * q[i21]);
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
      JQ[3][i0] = (-1.155305e-01) * (1) + (5.784739e-03) * ((2) * q[i0]) + (2.036691e-05) * (q[i1]) + (1.047292e-03) * (q[i2]) + (-6.074749e-02) * (q[i3])
            + (-4.186895e-02) * (q[i4]) + (-4.252679e-03) * (q[i5]) + (2.489028e-02) * (q[i6]) + (-9.974459e-03) * (q[i7]) + (8.124199e-03) * (q[i8])
            + (-2.182358e-03) * (q[i9]) + (-5.529312e-04) * (q[i10]) + (-7.457680e-04) * (q[i11]) + (-3.888310e-03) * (q[i12]) + (-7.918036e-03) * (q[i15])
            + (-1.004595e-02) * (q[i16]) + (1.977376e-03) * (q[i19]) + (-2.009966e-03) * (q[i20]) + (-3.807450e-03) * (q[i21]) + (-3.142940e-04) * (q[i22])
            + (4.351645e-03) * ((3) * q[i0] * q[i0]) + (-7.216278e-04) * ((2) * q[i0] * q[i1]) + (-1.101905e-03) * ((2) * q[i0] * q[i2])
            + (-6.938060e-03) * ((2) * q[i0] * q[i3]) + (-5.348986e-04) * ((2) * q[i0] * q[i4]) + (1.705448e-04) * ((2) * q[i0] * q[i5])
            + (-1.109336e-02) * ((2) * q[i0] * q[i6]) + (-1.924448e-03) * ((2) * q[i0] * q[i7]) + (-3.690608e-04) * ((2) * q[i0] * q[i8])
            + (-1.714045e-03) * ((2) * q[i0] * q[i9]) + (-1.578440e-03) * ((2) * q[i0] * q[i10]) + (-1.942903e-04) * ((2) * q[i0] * q[i11])
            + (1.862733e-03) * ((2) * q[i0] * q[i12]) + (1.696342e-04) * ((2) * q[i0] * q[i15]) + (-1.053388e-03) * ((2) * q[i0] * q[i16])
            + (7.116120e-04) * ((2) * q[i0] * q[i19]) + (3.588934e-04) * ((2) * q[i0] * q[i20]) + (8.216099e-04) * ((2) * q[i0] * q[i21])
            + (1.011065e-03) * ((2) * q[i0] * q[i22]) + (-7.246073e-04) * (q[i1] * q[i1]) + (4.185170e-03) * (q[i2] * q[i2]) + (-7.665853e-03) * (q[i3] * q[i3])
            + (2.659158e-03) * (q[i4] * q[i4]) + (-1.317582e-03) * (q[i5] * q[i5]) + (-1.145720e-02) * (q[i6] * q[i6]) + (6.246037e-03) * (q[i7] * q[i7])
            + (2.224777e-03) * (q[i8] * q[i8]) + (3.295889e-03) * (q[i9] * q[i9]) + (3.135397e-04) * (q[i10] * q[i10]) + (-2.853861e-04) * (q[i11] * q[i11])
            + (-1.852411e-04) * (q[i12] * q[i12]) + (-1.907664e-03) * (q[i15] * q[i15]) + (-2.086639e-03) * (q[i16] * q[i16])
            + (3.747856e-04) * (q[i19] * q[i19]) + (-6.009789e-04) * (q[i20] * q[i20]) + (-1.631851e-03) * (q[i21] * q[i21])
            + (3.073992e-04) * (q[i22] * q[i22]) + (-8.346531e-04) * (q[i1] * q[i2]) + (1.427112e-03) * (q[i1] * q[i3]) + (1.405527e-03) * (q[i1] * q[i4])
            + (-1.495583e-03) * (q[i1] * q[i5]) + (2.440913e-03) * (q[i1] * q[i6]) + (-2.509785e-03) * (q[i1] * q[i7]) + (3.772037e-05) * (q[i1] * q[i8])
            + (-9.410769e-04) * (q[i1] * q[i9]) + (9.334965e-04) * (q[i1] * q[i10]) + (-8.990105e-04) * (q[i1] * q[i11]) + (-8.987392e-04) * (q[i1] * q[i12])
            + (-1.695188e-03) * (q[i1] * q[i15]) + (1.701661e-03) * (q[i1] * q[i16]) + (-2.723746e-04) * (q[i1] * q[i19]) + (2.669963e-04) * (q[i1] * q[i20])
            + (-9.051953e-04) * (q[i1] * q[i21]) + (-8.994086e-04) * (q[i1] * q[i22]) + (3.394724e-05) * (q[i2] * q[i3]) + (-1.094924e-03) * (q[i2] * q[i4])
            + (3.503458e-03) * (q[i2] * q[i5]) + (-9.255762e-03) * (q[i2] * q[i6]) + (-9.079879e-04) * (q[i2] * q[i7]) + (2.392972e-03) * (q[i2] * q[i8])
            + (-1.221234e-03) * (q[i2] * q[i9]) + (-1.089578e-04) * (q[i2] * q[i10]) + (9.579792e-04) * (q[i2] * q[i11]) + (-7.160882e-04) * (q[i2] * q[i12])
            + (3.870245e-04) * (q[i2] * q[i15]) + (-6.183652e-04) * (q[i2] * q[i16]) + (-1.829261e-04) * (q[i2] * q[i19]) + (1.314575e-04) * (q[i2] * q[i20])
            + (1.036193e-04) * (q[i2] * q[i21]) + (3.914916e-04) * (q[i2] * q[i22]) + (6.346759e-03) * (q[i3] * q[i4]) + (-1.038624e-02) * (q[i3] * q[i5])
            + (-1.448655e-02) * (q[i3] * q[i6]) + (-8.385057e-05) * (q[i3] * q[i7]) + (3.691852e-03) * (q[i3] * q[i8]) + (4.641062e-03) * (q[i3] * q[i9])
            + (-4.279734e-04) * (q[i3] * q[i10]) + (-1.397372e-03) * (q[i3] * q[i11]) + (-1.202876e-03) * (q[i3] * q[i12]) + (2.500203e-03) * (q[i3] * q[i15])
            + (-2.838333e-03) * (q[i3] * q[i16]) + (-2.411072e-03) * (q[i3] * q[i19]) + (-3.702059e-04) * (q[i3] * q[i20]) + (1.223852e-03) * (q[i3] * q[i21])
            + (-1.502629e-03) * (q[i3] * q[i22]) + (2.166891e-03) * (q[i4] * q[i5]) + (2.077978e-03) * (q[i4] * q[i6]) + (-3.874965e-03) * (q[i4] * q[i7])
            + (1.921058e-03) * (q[i4] * q[i8]) + (4.471828e-03) * (q[i4] * q[i9]) + (3.653177e-03) * (q[i4] * q[i10]) + (-7.361672e-05) * (q[i4] * q[i11])
            + (6.362083e-04) * (q[i4] * q[i12]) + (-4.425119e-03) * (q[i4] * q[i15]) + (-1.259237e-03) * (q[i4] * q[i16]) + (1.293175e-03) * (q[i4] * q[i19])
            + (-1.629772e-04) * (q[i4] * q[i20]) + (-2.991066e-04) * (q[i4] * q[i21]) + (-1.857760e-03) * (q[i4] * q[i22]) + (1.889204e-03) * (q[i5] * q[i6])
            + (-1.951168e-03) * (q[i5] * q[i7]) + (4.131932e-03) * (q[i5] * q[i8]) + (1.215783e-03) * (q[i5] * q[i9]) + (5.609378e-04) * (q[i5] * q[i10])
            + (1.385829e-03) * (q[i5] * q[i11]) + (3.532723e-04) * (q[i5] * q[i12]) + (3.217083e-03) * (q[i5] * q[i15]) + (-1.774397e-03) * (q[i5] * q[i16])
            + (-4.666657e-04) * (q[i5] * q[i19]) + (-4.053795e-04) * (q[i5] * q[i20]) + (-1.545775e-03) * (q[i5] * q[i21]) + (-1.651426e-03) * (q[i5] * q[i22])
            + (1.064443e-02) * (q[i6] * q[i7]) + (2.018270e-03) * (q[i6] * q[i8]) + (-8.093356e-03) * (q[i6] * q[i9]) + (4.731231e-03) * (q[i6] * q[i10])
            + (1.256146e-03) * (q[i6] * q[i11]) + (3.351741e-03) * (q[i6] * q[i12]) + (-4.457664e-04) * (q[i6] * q[i15]) + (2.525945e-03) * (q[i6] * q[i16])
            + (-3.448775e-03) * (q[i6] * q[i19]) + (5.469725e-04) * (q[i6] * q[i20]) + (1.146224e-03) * (q[i6] * q[i21]) + (8.591253e-04) * (q[i6] * q[i22])
            + (-5.674026e-04) * (q[i7] * q[i8]) + (7.027701e-04) * (q[i7] * q[i9]) + (6.223994e-03) * (q[i7] * q[i10]) + (6.106089e-05) * (q[i7] * q[i11])
            + (-1.928167e-03) * (q[i7] * q[i12]) + (1.597382e-03) * (q[i7] * q[i15]) + (-2.070965e-03) * (q[i7] * q[i16]) + (-1.237260e-03) * (q[i7] * q[i19])
            + (-1.390933e-04) * (q[i7] * q[i20]) + (-7.828759e-04) * (q[i7] * q[i21]) + (-6.802556e-04) * (q[i7] * q[i22]) + (4.403004e-04) * (q[i8] * q[i9])
            + (-2.309188e-03) * (q[i8] * q[i10]) + (-9.840143e-04) * (q[i8] * q[i11]) + (-1.802440e-05) * (q[i8] * q[i12]) + (-1.237752e-03) * (q[i8] * q[i15])
            + (3.349766e-05) * (q[i8] * q[i16]) + (1.515999e-03) * (q[i8] * q[i19]) + (-1.020506e-03) * (q[i8] * q[i20]) + (6.531737e-04) * (q[i8] * q[i21])
            + (4.179213e-04) * (q[i8] * q[i22]) + (7.059487e-04) * (q[i9] * q[i10]) + (2.171805e-04) * (q[i9] * q[i11]) + (8.206057e-04) * (q[i9] * q[i12])
            + (-2.962509e-04) * (q[i9] * q[i15]) + (3.266708e-04) * (q[i9] * q[i16]) + (-1.340145e-04) * (q[i9] * q[i19]) + (4.477149e-04) * (q[i9] * q[i20])
            + (-9.313780e-05) * (q[i9] * q[i21]) + (-3.259272e-05) * (q[i9] * q[i22]) + (7.615470e-04) * (q[i10] * q[i11]) + (9.834727e-04) * (q[i10] * q[i12])
            + (-1.050272e-03) * (q[i10] * q[i15]) + (5.576206e-04) * (q[i10] * q[i16]) + (-6.144819e-04) * (q[i10] * q[i19])
            + (1.285622e-04) * (q[i10] * q[i20]) + (1.549669e-04) * (q[i10] * q[i21]) + (-3.355837e-04) * (q[i10] * q[i22]) + (2.378756e-05) * (q[i11] * q[i12])
            + (2.347744e-04) * (q[i11] * q[i15]) + (-2.017318e-04) * (q[i11] * q[i16]) + (6.241591e-04) * (q[i11] * q[i19])
            + (-9.831590e-04) * (q[i11] * q[i20]) + (2.935356e-04) * (q[i11] * q[i21]) + (-2.104644e-04) * (q[i11] * q[i22])
            + (9.361919e-04) * (q[i12] * q[i15]) + (-2.527649e-03) * (q[i12] * q[i16]) + (1.297013e-03) * (q[i12] * q[i19]) + (9.315936e-04) * (q[i12] * q[i20])
            + (4.288031e-04) * (q[i12] * q[i21]) + (-3.299089e-05) * (q[i12] * q[i22]) + (2.377203e-06) * (q[i15] * q[i16])
            + (-4.759495e-05) * (q[i15] * q[i19]) + (-3.216543e-04) * (q[i15] * q[i20]) + (-1.686942e-03) * (q[i15] * q[i21])
            + (3.592464e-04) * (q[i15] * q[i22]) + (1.059564e-03) * (q[i16] * q[i19]) + (-2.461122e-04) * (q[i16] * q[i20])
            + (-4.163082e-04) * (q[i16] * q[i21]) + (5.712635e-04) * (q[i16] * q[i22]) + (2.088757e-04) * (q[i19] * q[i20])
            + (-6.207025e-06) * (q[i19] * q[i21]) + (-4.942090e-04) * (q[i19] * q[i22]) + (-7.676975e-04) * (q[i20] * q[i21])
            + (3.850536e-04) * (q[i20] * q[i22]) + (3.870924e-04) * (q[i21] * q[i22]);
   }

   public void getJQz1(double[] q, double[][] JQ)
   {
      JQ[3][i1] = (-1.150095e-01) * (1) + (-5.709468e-03) * ((2) * q[i1]) + (2.036691e-05) * (q[i0]) + (-9.686204e-04) * (q[i2]) + (4.182345e-02) * (q[i3])
            + (6.077392e-02) * (q[i4]) + (4.187169e-03) * (q[i5]) + (-1.000115e-02) * (q[i6]) + (2.483707e-02) * (q[i7]) + (8.120267e-03) * (q[i8])
            + (-5.844495e-04) * (q[i9]) + (-2.087825e-03) * (q[i10]) + (4.029661e-03) * (q[i11]) + (7.970926e-04) * (q[i12]) + (-9.929541e-03) * (q[i15])
            + (-8.015240e-03) * (q[i16]) + (-1.988935e-03) * (q[i19]) + (1.956409e-03) * (q[i20]) + (2.742735e-04) * (q[i21]) + (3.725649e-03) * (q[i22])
            + (-7.216278e-04) * (q[i0] * q[i0]) + (-7.246073e-04) * ((2) * q[i0] * q[i1]) + (4.319157e-03) * ((3) * q[i1] * q[i1])
            + (-1.056982e-03) * ((2) * q[i1] * q[i2]) + (-5.286147e-04) * ((2) * q[i1] * q[i3]) + (-6.889231e-03) * ((2) * q[i1] * q[i4])
            + (1.741400e-04) * ((2) * q[i1] * q[i5]) + (1.946229e-03) * ((2) * q[i1] * q[i6]) + (1.113704e-02) * ((2) * q[i1] * q[i7])
            + (3.509609e-04) * ((2) * q[i1] * q[i8]) + (1.584619e-03) * ((2) * q[i1] * q[i9]) + (1.710830e-03) * ((2) * q[i1] * q[i10])
            + (1.852546e-03) * ((2) * q[i1] * q[i11]) + (-2.041879e-04) * ((2) * q[i1] * q[i12]) + (1.069579e-03) * ((2) * q[i1] * q[i15])
            + (-1.540970e-04) * ((2) * q[i1] * q[i16]) + (-3.622465e-04) * ((2) * q[i1] * q[i19]) + (-6.913918e-04) * ((2) * q[i1] * q[i20])
            + (1.014259e-03) * ((2) * q[i1] * q[i21]) + (7.871221e-04) * ((2) * q[i1] * q[i22]) + (4.196143e-03) * (q[i2] * q[i2])
            + (2.662532e-03) * (q[i3] * q[i3]) + (-7.650379e-03) * (q[i4] * q[i4]) + (-1.370099e-03) * (q[i5] * q[i5]) + (6.238411e-03) * (q[i6] * q[i6])
            + (-1.145755e-02) * (q[i7] * q[i7]) + (2.240031e-03) * (q[i8] * q[i8]) + (3.194498e-04) * (q[i9] * q[i9]) + (3.248302e-03) * (q[i10] * q[i10])
            + (-1.800565e-04) * (q[i11] * q[i11]) + (-3.294122e-04) * (q[i12] * q[i12]) + (-2.048487e-03) * (q[i15] * q[i15])
            + (-1.917866e-03) * (q[i16] * q[i16]) + (-6.184213e-04) * (q[i19] * q[i19]) + (3.759789e-04) * (q[i20] * q[i20])
            + (3.043680e-04) * (q[i21] * q[i21]) + (-1.623037e-03) * (q[i22] * q[i22]) + (-8.346531e-04) * (q[i0] * q[i2]) + (1.427112e-03) * (q[i0] * q[i3])
            + (1.405527e-03) * (q[i0] * q[i4]) + (-1.495583e-03) * (q[i0] * q[i5]) + (2.440913e-03) * (q[i0] * q[i6]) + (-2.509785e-03) * (q[i0] * q[i7])
            + (3.772037e-05) * (q[i0] * q[i8]) + (-9.410769e-04) * (q[i0] * q[i9]) + (9.334965e-04) * (q[i0] * q[i10]) + (-8.990105e-04) * (q[i0] * q[i11])
            + (-8.987392e-04) * (q[i0] * q[i12]) + (-1.695188e-03) * (q[i0] * q[i15]) + (1.701661e-03) * (q[i0] * q[i16]) + (-2.723746e-04) * (q[i0] * q[i19])
            + (2.669963e-04) * (q[i0] * q[i20]) + (-9.051953e-04) * (q[i0] * q[i21]) + (-8.994086e-04) * (q[i0] * q[i22]) + (-1.052106e-03) * (q[i2] * q[i3])
            + (5.408919e-05) * (q[i2] * q[i4]) + (3.533647e-03) * (q[i2] * q[i5]) + (8.852707e-04) * (q[i2] * q[i6]) + (9.255482e-03) * (q[i2] * q[i7])
            + (-2.397383e-03) * (q[i2] * q[i8]) + (1.123347e-04) * (q[i2] * q[i9]) + (1.213317e-03) * (q[i2] * q[i10]) + (-7.466897e-04) * (q[i2] * q[i11])
            + (9.808607e-04) * (q[i2] * q[i12]) + (6.267690e-04) * (q[i2] * q[i15]) + (-3.589615e-04) * (q[i2] * q[i16]) + (-1.329226e-04) * (q[i2] * q[i19])
            + (1.997256e-04) * (q[i2] * q[i20]) + (3.816037e-04) * (q[i2] * q[i21]) + (7.688619e-05) * (q[i2] * q[i22]) + (6.358675e-03) * (q[i3] * q[i4])
            + (2.140390e-03) * (q[i3] * q[i5]) + (3.777589e-03) * (q[i3] * q[i6]) + (-2.093520e-03) * (q[i3] * q[i7]) + (-1.897335e-03) * (q[i3] * q[i8])
            + (-3.666251e-03) * (q[i3] * q[i9]) + (-4.470188e-03) * (q[i3] * q[i10]) + (5.982896e-04) * (q[i3] * q[i11]) + (-1.060551e-04) * (q[i3] * q[i12])
            + (1.236331e-03) * (q[i3] * q[i15]) + (4.454264e-03) * (q[i3] * q[i16]) + (1.676639e-04) * (q[i3] * q[i19]) + (-1.299664e-03) * (q[i3] * q[i20])
            + (-1.828656e-03) * (q[i3] * q[i21]) + (-2.798875e-04) * (q[i3] * q[i22]) + (-1.032987e-02) * (q[i4] * q[i5]) + (7.430544e-05) * (q[i4] * q[i6])
            + (1.461029e-02) * (q[i4] * q[i7]) + (-3.676329e-03) * (q[i4] * q[i8]) + (4.109857e-04) * (q[i4] * q[i9]) + (-4.582681e-03) * (q[i4] * q[i10])
            + (-1.219388e-03) * (q[i4] * q[i11]) + (-1.448425e-03) * (q[i4] * q[i12]) + (2.785258e-03) * (q[i4] * q[i15]) + (-2.476091e-03) * (q[i4] * q[i16])
            + (3.806077e-04) * (q[i4] * q[i19]) + (2.369193e-03) * (q[i4] * q[i20]) + (-1.502616e-03) * (q[i4] * q[i21]) + (1.220969e-03) * (q[i4] * q[i22])
            + (1.989037e-03) * (q[i5] * q[i6]) + (-1.894315e-03) * (q[i5] * q[i7]) + (-4.131924e-03) * (q[i5] * q[i8]) + (-5.376108e-04) * (q[i5] * q[i9])
            + (-1.196318e-03) * (q[i5] * q[i10]) + (3.883544e-04) * (q[i5] * q[i11]) + (1.359287e-03) * (q[i5] * q[i12]) + (1.774251e-03) * (q[i5] * q[i15])
            + (-3.250903e-03) * (q[i5] * q[i16]) + (3.999837e-04) * (q[i5] * q[i19]) + (4.605860e-04) * (q[i5] * q[i20]) + (-1.623950e-03) * (q[i5] * q[i21])
            + (-1.506938e-03) * (q[i5] * q[i22]) + (1.066394e-02) * (q[i6] * q[i7]) + (-5.539103e-04) * (q[i6] * q[i8]) + (6.241155e-03) * (q[i6] * q[i9])
            + (7.006525e-04) * (q[i6] * q[i10]) + (1.968200e-03) * (q[i6] * q[i11]) + (-8.547104e-05) * (q[i6] * q[i12]) + (-2.089086e-03) * (q[i6] * q[i15])
            + (1.587016e-03) * (q[i6] * q[i16]) + (-1.365403e-04) * (q[i6] * q[i19]) + (-1.235479e-03) * (q[i6] * q[i20]) + (6.942189e-04) * (q[i6] * q[i21])
            + (7.825132e-04) * (q[i6] * q[i22]) + (1.990251e-03) * (q[i7] * q[i8]) + (4.768331e-03) * (q[i7] * q[i9]) + (-8.073847e-03) * (q[i7] * q[i10])
            + (-3.302676e-03) * (q[i7] * q[i11]) + (-1.266723e-03) * (q[i7] * q[i12]) + (2.515716e-03) * (q[i7] * q[i15]) + (-4.501706e-04) * (q[i7] * q[i16])
            + (5.669661e-04) * (q[i7] * q[i19]) + (-3.425146e-03) * (q[i7] * q[i20]) + (-8.601597e-04) * (q[i7] * q[i21]) + (-1.137522e-03) * (q[i7] * q[i22])
            + (-2.300075e-03) * (q[i8] * q[i9]) + (4.317355e-04) * (q[i8] * q[i10]) + (3.076759e-05) * (q[i8] * q[i11]) + (9.556149e-04) * (q[i8] * q[i12])
            + (4.747636e-05) * (q[i8] * q[i15]) + (-1.247925e-03) * (q[i8] * q[i16]) + (-1.008339e-03) * (q[i8] * q[i19]) + (1.523498e-03) * (q[i8] * q[i20])
            + (-4.188210e-04) * (q[i8] * q[i21]) + (-6.807220e-04) * (q[i8] * q[i22]) + (7.159339e-04) * (q[i9] * q[i10]) + (-9.588975e-04) * (q[i9] * q[i11])
            + (-7.617789e-04) * (q[i9] * q[i12]) + (5.426493e-04) * (q[i9] * q[i15]) + (-1.040909e-03) * (q[i9] * q[i16]) + (1.209350e-04) * (q[i9] * q[i19])
            + (-6.194793e-04) * (q[i9] * q[i20]) + (3.461663e-04) * (q[i9] * q[i21]) + (-1.466367e-04) * (q[i9] * q[i22]) + (-8.165858e-04) * (q[i10] * q[i11])
            + (-2.143419e-04) * (q[i10] * q[i12]) + (3.368781e-04) * (q[i10] * q[i15]) + (-2.906111e-04) * (q[i10] * q[i16])
            + (4.356600e-04) * (q[i10] * q[i19]) + (-1.314759e-04) * (q[i10] * q[i20]) + (3.046122e-05) * (q[i10] * q[i21]) + (1.084647e-04) * (q[i10] * q[i22])
            + (1.206100e-05) * (q[i11] * q[i12]) + (2.540007e-03) * (q[i11] * q[i15]) + (-9.214786e-04) * (q[i11] * q[i16])
            + (-9.333654e-04) * (q[i11] * q[i19]) + (-1.281242e-03) * (q[i11] * q[i20]) + (-2.952990e-05) * (q[i11] * q[i21])
            + (4.311889e-04) * (q[i11] * q[i22]) + (2.050431e-04) * (q[i12] * q[i15]) + (-2.460959e-04) * (q[i12] * q[i16]) + (9.758899e-04) * (q[i12] * q[i19])
            + (-6.428755e-04) * (q[i12] * q[i20]) + (-2.075818e-04) * (q[i12] * q[i21]) + (2.939070e-04) * (q[i12] * q[i22])
            + (-3.547933e-06) * (q[i15] * q[i16]) + (-2.625495e-04) * (q[i15] * q[i19]) + (1.049304e-03) * (q[i15] * q[i20])
            + (-5.695855e-04) * (q[i15] * q[i21]) + (4.220954e-04) * (q[i15] * q[i22]) + (-3.201155e-04) * (q[i16] * q[i19])
            + (-3.407726e-05) * (q[i16] * q[i20]) + (-3.615603e-04) * (q[i16] * q[i21]) + (1.679648e-03) * (q[i16] * q[i22])
            + (2.035014e-04) * (q[i19] * q[i20]) + (-3.969437e-04) * (q[i19] * q[i21]) + (7.699027e-04) * (q[i19] * q[i22]) + (4.882128e-04) * (q[i20] * q[i21])
            + (1.876201e-05) * (q[i20] * q[i22]) + (3.737421e-04) * (q[i21] * q[i22]);
   }

   public void getJQz2(double[] q, double[][] JQ)
   {
      JQ[3][i2] = (2.023910e-01) * (1) + (8.432394e-06) * ((2) * q[i2]) + (1.047292e-03) * (q[i0]) + (-9.686204e-04) * (q[i1]) + (-1.580835e-02) * (q[i3])
            + (1.576084e-02) * (q[i4]) + (-6.851522e-05) * (q[i5]) + (1.301239e-02) * (q[i6]) + (1.298224e-02) * (q[i7]) + (9.419529e-03) * (q[i8])
            + (5.744760e-05) * (q[i9]) + (1.044127e-04) * (q[i10]) + (3.681198e-03) * (q[i11]) + (-3.495712e-03) * (q[i12]) + (-2.036958e-02) * (q[i15])
            + (-2.057607e-02) * (q[i16]) + (2.362708e-03) * (q[i19]) + (2.297146e-03) * (q[i20]) + (-1.734093e-03) * (q[i21]) + (1.651024e-03) * (q[i22])
            + (-1.101905e-03) * (q[i0] * q[i0]) + (-1.056982e-03) * (q[i1] * q[i1]) + (4.185170e-03) * ((2) * q[i0] * q[i2])
            + (4.196143e-03) * ((2) * q[i1] * q[i2]) + (-3.011909e-03) * ((3) * q[i2] * q[i2]) + (-2.273799e-03) * ((2) * q[i2] * q[i3])
            + (-2.291724e-03) * ((2) * q[i2] * q[i4]) + (4.036315e-03) * ((2) * q[i2] * q[i5]) + (-1.087924e-05) * ((2) * q[i2] * q[i6])
            + (9.450271e-06) * ((2) * q[i2] * q[i7]) + (-4.182231e-06) * ((2) * q[i2] * q[i8]) + (-5.289894e-04) * ((2) * q[i2] * q[i9])
            + (5.168931e-04) * ((2) * q[i2] * q[i10]) + (-7.185042e-04) * ((2) * q[i2] * q[i11]) + (-7.079538e-04) * ((2) * q[i2] * q[i12])
            + (-2.672006e-04) * ((2) * q[i2] * q[i15]) + (2.779448e-04) * ((2) * q[i2] * q[i16]) + (-5.728309e-04) * ((2) * q[i2] * q[i19])
            + (5.599040e-04) * ((2) * q[i2] * q[i20]) + (6.728289e-04) * ((2) * q[i2] * q[i21]) + (6.704102e-04) * ((2) * q[i2] * q[i22])
            + (-1.359760e-03) * (q[i3] * q[i3]) + (-1.354439e-03) * (q[i4] * q[i4]) + (-9.658555e-03) * (q[i5] * q[i5]) + (-6.167978e-03) * (q[i6] * q[i6])
            + (-6.130194e-03) * (q[i7] * q[i7]) + (4.451997e-03) * (q[i8] * q[i8]) + (1.033256e-03) * (q[i9] * q[i9]) + (1.014726e-03) * (q[i10] * q[i10])
            + (-8.427844e-04) * (q[i11] * q[i11]) + (-8.871160e-04) * (q[i12] * q[i12]) + (-4.605599e-03) * (q[i15] * q[i15])
            + (-4.648974e-03) * (q[i16] * q[i16]) + (-4.758407e-04) * (q[i19] * q[i19]) + (-4.632956e-04) * (q[i20] * q[i20])
            + (-1.526069e-03) * (q[i21] * q[i21]) + (-1.514669e-03) * (q[i22] * q[i22]) + (-8.346531e-04) * (q[i0] * q[i1]) + (3.394724e-05) * (q[i0] * q[i3])
            + (-1.094924e-03) * (q[i0] * q[i4]) + (3.503458e-03) * (q[i0] * q[i5]) + (-9.255762e-03) * (q[i0] * q[i6]) + (-9.079879e-04) * (q[i0] * q[i7])
            + (2.392972e-03) * (q[i0] * q[i8]) + (-1.221234e-03) * (q[i0] * q[i9]) + (-1.089578e-04) * (q[i0] * q[i10]) + (9.579792e-04) * (q[i0] * q[i11])
            + (-7.160882e-04) * (q[i0] * q[i12]) + (3.870245e-04) * (q[i0] * q[i15]) + (-6.183652e-04) * (q[i0] * q[i16]) + (-1.829261e-04) * (q[i0] * q[i19])
            + (1.314575e-04) * (q[i0] * q[i20]) + (1.036193e-04) * (q[i0] * q[i21]) + (3.914916e-04) * (q[i0] * q[i22]) + (-1.052106e-03) * (q[i1] * q[i3])
            + (5.408919e-05) * (q[i1] * q[i4]) + (3.533647e-03) * (q[i1] * q[i5]) + (8.852707e-04) * (q[i1] * q[i6]) + (9.255482e-03) * (q[i1] * q[i7])
            + (-2.397383e-03) * (q[i1] * q[i8]) + (1.123347e-04) * (q[i1] * q[i9]) + (1.213317e-03) * (q[i1] * q[i10]) + (-7.466897e-04) * (q[i1] * q[i11])
            + (9.808607e-04) * (q[i1] * q[i12]) + (6.267690e-04) * (q[i1] * q[i15]) + (-3.589615e-04) * (q[i1] * q[i16]) + (-1.329226e-04) * (q[i1] * q[i19])
            + (1.997256e-04) * (q[i1] * q[i20]) + (3.816037e-04) * (q[i1] * q[i21]) + (7.688619e-05) * (q[i1] * q[i22]) + (-3.414844e-03) * (q[i3] * q[i4])
            + (-1.548406e-02) * (q[i3] * q[i5]) + (-4.403246e-03) * (q[i3] * q[i6]) + (-2.472890e-03) * (q[i3] * q[i7]) + (-1.970023e-04) * (q[i3] * q[i8])
            + (1.065266e-03) * (q[i3] * q[i9]) + (-2.928779e-03) * (q[i3] * q[i10]) + (-2.067182e-03) * (q[i3] * q[i11]) + (5.248740e-04) * (q[i3] * q[i12])
            + (3.432405e-03) * (q[i3] * q[i15]) + (6.102899e-04) * (q[i3] * q[i16]) + (-2.403196e-03) * (q[i3] * q[i19]) + (-8.265750e-04) * (q[i3] * q[i20])
            + (8.494775e-04) * (q[i3] * q[i21]) + (-1.339271e-03) * (q[i3] * q[i22]) + (-1.538618e-02) * (q[i4] * q[i5]) + (2.431319e-03) * (q[i4] * q[i6])
            + (4.423881e-03) * (q[i4] * q[i7]) + (2.195300e-04) * (q[i4] * q[i8]) + (2.925550e-03) * (q[i4] * q[i9]) + (-1.013973e-03) * (q[i4] * q[i10])
            + (5.717113e-04) * (q[i4] * q[i11]) + (-2.060311e-03) * (q[i4] * q[i12]) + (-6.114839e-04) * (q[i4] * q[i15]) + (-3.448970e-03) * (q[i4] * q[i16])
            + (8.164762e-04) * (q[i4] * q[i19]) + (2.400144e-03) * (q[i4] * q[i20]) + (-1.327809e-03) * (q[i4] * q[i21]) + (8.300638e-04) * (q[i4] * q[i22])
            + (-2.733054e-03) * (q[i5] * q[i6]) + (2.764813e-03) * (q[i5] * q[i7]) + (-1.664079e-05) * (q[i5] * q[i8]) + (1.866223e-03) * (q[i5] * q[i9])
            + (-1.855923e-03) * (q[i5] * q[i10]) + (-6.183201e-04) * (q[i5] * q[i11]) + (-6.954248e-04) * (q[i5] * q[i12]) + (5.859833e-03) * (q[i5] * q[i15])
            + (-5.913097e-03) * (q[i5] * q[i16]) + (-1.748491e-03) * (q[i5] * q[i19]) + (1.751262e-03) * (q[i5] * q[i20]) + (-1.068426e-03) * (q[i5] * q[i21])
            + (-1.038283e-03) * (q[i5] * q[i22]) + (1.259057e-02) * (q[i6] * q[i7]) + (-7.272094e-03) * (q[i6] * q[i8]) + (-4.764478e-03) * (q[i6] * q[i9])
            + (7.771415e-04) * (q[i6] * q[i10]) + (8.556703e-06) * (q[i6] * q[i11]) + (2.338387e-03) * (q[i6] * q[i12]) + (-1.378391e-03) * (q[i6] * q[i15])
            + (9.848270e-04) * (q[i6] * q[i16]) + (-2.454018e-05) * (q[i6] * q[i19]) + (3.393243e-04) * (q[i6] * q[i20]) + (-9.109117e-04) * (q[i6] * q[i21])
            + (7.190790e-04) * (q[i6] * q[i22]) + (-7.247755e-03) * (q[i7] * q[i8]) + (7.925011e-04) * (q[i7] * q[i9]) + (-4.733092e-03) * (q[i7] * q[i10])
            + (-2.324831e-03) * (q[i7] * q[i11]) + (-3.349756e-06) * (q[i7] * q[i12]) + (9.778492e-04) * (q[i7] * q[i15]) + (-1.405652e-03) * (q[i7] * q[i16])
            + (3.461047e-04) * (q[i7] * q[i19]) + (-1.300624e-05) * (q[i7] * q[i20]) + (-7.129322e-04) * (q[i7] * q[i21]) + (9.094359e-04) * (q[i7] * q[i22])
            + (-2.026978e-03) * (q[i8] * q[i9]) + (-2.007572e-03) * (q[i8] * q[i10]) + (7.690454e-05) * (q[i8] * q[i11]) + (-1.277173e-04) * (q[i8] * q[i12])
            + (-1.013031e-03) * (q[i8] * q[i15]) + (-1.017945e-03) * (q[i8] * q[i16]) + (-1.213964e-03) * (q[i8] * q[i19]) + (-1.202537e-03) * (q[i8] * q[i20])
            + (1.080224e-03) * (q[i8] * q[i21]) + (-1.089511e-03) * (q[i8] * q[i22]) + (4.356090e-04) * (q[i9] * q[i10]) + (-1.279139e-03) * (q[i9] * q[i11])
            + (-3.126844e-04) * (q[i9] * q[i12]) + (1.020416e-03) * (q[i9] * q[i15]) + (-1.131341e-03) * (q[i9] * q[i16]) + (2.814179e-04) * (q[i9] * q[i19])
            + (-4.580965e-04) * (q[i9] * q[i20]) + (-1.082832e-04) * (q[i9] * q[i21]) + (-4.297117e-05) * (q[i9] * q[i22]) + (3.151909e-04) * (q[i10] * q[i11])
            + (1.280565e-03) * (q[i10] * q[i12]) + (-1.123029e-03) * (q[i10] * q[i15]) + (1.026945e-03) * (q[i10] * q[i16])
            + (-4.671196e-04) * (q[i10] * q[i19]) + (2.916659e-04) * (q[i10] * q[i20]) + (4.605087e-05) * (q[i10] * q[i21]) + (1.157773e-04) * (q[i10] * q[i22])
            + (8.537937e-04) * (q[i11] * q[i12]) + (2.076426e-03) * (q[i11] * q[i15]) + (-5.741165e-04) * (q[i11] * q[i16])
            + (-6.186795e-04) * (q[i11] * q[i19]) + (-4.964284e-04) * (q[i11] * q[i20]) + (-5.254483e-05) * (q[i11] * q[i21])
            + (-1.053333e-04) * (q[i11] * q[i22]) + (5.847939e-04) * (q[i12] * q[i15]) + (-2.091056e-03) * (q[i12] * q[i16])
            + (4.856886e-04) * (q[i12] * q[i19]) + (5.954556e-04) * (q[i12] * q[i20]) + (-1.037075e-04) * (q[i12] * q[i21])
            + (-3.184955e-05) * (q[i12] * q[i22]) + (-4.683864e-04) * (q[i15] * q[i16]) + (7.724403e-04) * (q[i15] * q[i19])
            + (4.007489e-04) * (q[i15] * q[i20]) + (-1.745349e-03) * (q[i15] * q[i21]) + (4.714810e-05) * (q[i15] * q[i22]) + (4.203856e-04) * (q[i16] * q[i19])
            + (7.946630e-04) * (q[i16] * q[i20]) + (-3.777557e-05) * (q[i16] * q[i21]) + (1.742322e-03) * (q[i16] * q[i22])
            + (-4.370205e-04) * (q[i19] * q[i20]) + (-1.674023e-03) * (q[i19] * q[i21]) + (1.799295e-05) * (q[i19] * q[i22])
            + (-2.370410e-05) * (q[i20] * q[i21]) + (1.679019e-03) * (q[i20] * q[i22]) + (8.492379e-04) * (q[i21] * q[i22]);
   }

   public void getJQz3(double[] q, double[][] JQ)
   {
      JQ[3][i3] = (1.933653e-02) * (1) + (-4.255974e-03) * ((2) * q[i3]) + (-6.074749e-02) * (q[i0]) + (4.182345e-02) * (q[i1]) + (-1.580835e-02) * (q[i2])
            + (-4.628329e-05) * (q[i4]) + (-3.369455e-03) * (q[i5]) + (1.660043e-02) * (q[i6]) + (2.155436e-02) * (q[i7]) + (-4.288859e-03) * (q[i8])
            + (1.400853e-03) * (q[i9]) + (1.098379e-02) * (q[i10]) + (4.454410e-03) * (q[i11]) + (1.894654e-03) * (q[i12]) + (4.513642e-03) * (q[i15])
            + (9.240801e-04) * (q[i16]) + (-1.019916e-02) * (q[i19]) + (1.230024e-02) * (q[i20]) + (4.898164e-03) * (q[i21]) + (-2.220548e-03) * (q[i22])
            + (-6.938060e-03) * (q[i0] * q[i0]) + (-5.286147e-04) * (q[i1] * q[i1]) + (-2.273799e-03) * (q[i2] * q[i2])
            + (-7.665853e-03) * ((2) * q[i0] * q[i3]) + (2.662532e-03) * ((2) * q[i1] * q[i3]) + (-1.359760e-03) * ((2) * q[i2] * q[i3])
            + (1.195013e-03) * ((3) * q[i3] * q[i3]) + (1.952111e-03) * ((2) * q[i3] * q[i4]) + (1.556399e-03) * ((2) * q[i3] * q[i5])
            + (9.172461e-03) * ((2) * q[i3] * q[i6]) + (4.204319e-03) * ((2) * q[i3] * q[i7]) + (3.506601e-03) * ((2) * q[i3] * q[i8])
            + (1.159972e-03) * ((2) * q[i3] * q[i9]) + (1.046336e-04) * ((2) * q[i3] * q[i10]) + (8.607750e-05) * ((2) * q[i3] * q[i11])
            + (-1.082842e-03) * ((2) * q[i3] * q[i12]) + (-3.190496e-03) * ((2) * q[i3] * q[i15]) + (-1.417495e-03) * ((2) * q[i3] * q[i16])
            + (2.284390e-03) * ((2) * q[i3] * q[i19]) + (-2.929116e-03) * ((2) * q[i3] * q[i20]) + (-1.159429e-04) * ((2) * q[i3] * q[i21])
            + (1.880048e-03) * ((2) * q[i3] * q[i22]) + (1.972930e-03) * (q[i4] * q[i4]) + (1.226351e-03) * (q[i5] * q[i5]) + (-7.821550e-03) * (q[i6] * q[i6])
            + (1.974651e-03) * (q[i7] * q[i7]) + (-2.793614e-03) * (q[i8] * q[i8]) + (7.747477e-04) * (q[i9] * q[i9]) + (-9.855731e-04) * (q[i10] * q[i10])
            + (-9.812237e-04) * (q[i11] * q[i11]) + (-1.043350e-03) * (q[i12] * q[i12]) + (-8.918000e-04) * (q[i15] * q[i15])
            + (4.890517e-04) * (q[i16] * q[i16]) + (-5.499706e-04) * (q[i19] * q[i19]) + (4.830417e-04) * (q[i20] * q[i20])
            + (-2.297460e-04) * (q[i21] * q[i21]) + (1.064119e-03) * (q[i22] * q[i22]) + (1.427112e-03) * (q[i0] * q[i1]) + (3.394724e-05) * (q[i0] * q[i2])
            + (6.346759e-03) * (q[i0] * q[i4]) + (-1.038624e-02) * (q[i0] * q[i5]) + (-1.448655e-02) * (q[i0] * q[i6]) + (-8.385057e-05) * (q[i0] * q[i7])
            + (3.691852e-03) * (q[i0] * q[i8]) + (4.641062e-03) * (q[i0] * q[i9]) + (-4.279734e-04) * (q[i0] * q[i10]) + (-1.397372e-03) * (q[i0] * q[i11])
            + (-1.202876e-03) * (q[i0] * q[i12]) + (2.500203e-03) * (q[i0] * q[i15]) + (-2.838333e-03) * (q[i0] * q[i16]) + (-2.411072e-03) * (q[i0] * q[i19])
            + (-3.702059e-04) * (q[i0] * q[i20]) + (1.223852e-03) * (q[i0] * q[i21]) + (-1.502629e-03) * (q[i0] * q[i22]) + (-1.052106e-03) * (q[i1] * q[i2])
            + (6.358675e-03) * (q[i1] * q[i4]) + (2.140390e-03) * (q[i1] * q[i5]) + (3.777589e-03) * (q[i1] * q[i6]) + (-2.093520e-03) * (q[i1] * q[i7])
            + (-1.897335e-03) * (q[i1] * q[i8]) + (-3.666251e-03) * (q[i1] * q[i9]) + (-4.470188e-03) * (q[i1] * q[i10]) + (5.982896e-04) * (q[i1] * q[i11])
            + (-1.060551e-04) * (q[i1] * q[i12]) + (1.236331e-03) * (q[i1] * q[i15]) + (4.454264e-03) * (q[i1] * q[i16]) + (1.676639e-04) * (q[i1] * q[i19])
            + (-1.299664e-03) * (q[i1] * q[i20]) + (-1.828656e-03) * (q[i1] * q[i21]) + (-2.798875e-04) * (q[i1] * q[i22]) + (-3.414844e-03) * (q[i2] * q[i4])
            + (-1.548406e-02) * (q[i2] * q[i5]) + (-4.403246e-03) * (q[i2] * q[i6]) + (-2.472890e-03) * (q[i2] * q[i7]) + (-1.970023e-04) * (q[i2] * q[i8])
            + (1.065266e-03) * (q[i2] * q[i9]) + (-2.928779e-03) * (q[i2] * q[i10]) + (-2.067182e-03) * (q[i2] * q[i11]) + (5.248740e-04) * (q[i2] * q[i12])
            + (3.432405e-03) * (q[i2] * q[i15]) + (6.102899e-04) * (q[i2] * q[i16]) + (-2.403196e-03) * (q[i2] * q[i19]) + (-8.265750e-04) * (q[i2] * q[i20])
            + (8.494775e-04) * (q[i2] * q[i21]) + (-1.339271e-03) * (q[i2] * q[i22]) + (1.174176e-03) * (q[i4] * q[i5]) + (8.426780e-03) * (q[i4] * q[i6])
            + (-8.359363e-03) * (q[i4] * q[i7]) + (-1.122263e-05) * (q[i4] * q[i8]) + (-8.118305e-04) * (q[i4] * q[i9]) + (8.529342e-04) * (q[i4] * q[i10])
            + (3.597437e-03) * (q[i4] * q[i11]) + (3.627772e-03) * (q[i4] * q[i12]) + (3.793315e-03) * (q[i4] * q[i15]) + (-3.798990e-03) * (q[i4] * q[i16])
            + (-6.057769e-03) * (q[i4] * q[i19]) + (6.060812e-03) * (q[i4] * q[i20]) + (-5.361392e-04) * (q[i4] * q[i21]) + (-5.363010e-04) * (q[i4] * q[i22])
            + (-7.018283e-03) * (q[i5] * q[i6]) + (-3.106559e-03) * (q[i5] * q[i7]) + (-1.497916e-02) * (q[i5] * q[i8]) + (-1.775080e-04) * (q[i5] * q[i9])
            + (1.140135e-03) * (q[i5] * q[i10]) + (-9.524564e-04) * (q[i5] * q[i11]) + (-2.029475e-03) * (q[i5] * q[i12]) + (-1.225580e-03) * (q[i5] * q[i15])
            + (3.048870e-03) * (q[i5] * q[i16]) + (-1.357805e-03) * (q[i5] * q[i19]) + (-1.363373e-03) * (q[i5] * q[i20]) + (1.755457e-03) * (q[i5] * q[i21])
            + (-1.061229e-03) * (q[i5] * q[i22]) + (1.009055e-03) * (q[i6] * q[i7]) + (-2.132648e-03) * (q[i6] * q[i8]) + (-1.094750e-03) * (q[i6] * q[i9])
            + (3.304197e-04) * (q[i6] * q[i10]) + (6.582829e-04) * (q[i6] * q[i11]) + (4.444862e-04) * (q[i6] * q[i12]) + (3.373408e-03) * (q[i6] * q[i15])
            + (3.345002e-03) * (q[i6] * q[i16]) + (-1.681426e-03) * (q[i6] * q[i19]) + (-9.682098e-04) * (q[i6] * q[i20]) + (-2.165034e-03) * (q[i6] * q[i21])
            + (9.942412e-04) * (q[i6] * q[i22]) + (1.143908e-03) * (q[i7] * q[i8]) + (-9.034955e-04) * (q[i7] * q[i9]) + (-2.223610e-03) * (q[i7] * q[i10])
            + (3.926526e-04) * (q[i7] * q[i11]) + (1.952225e-03) * (q[i7] * q[i12]) + (1.778172e-03) * (q[i7] * q[i15]) + (-2.089276e-03) * (q[i7] * q[i16])
            + (1.133573e-03) * (q[i7] * q[i19]) + (4.211431e-03) * (q[i7] * q[i20]) + (2.645970e-03) * (q[i7] * q[i21]) + (-1.204513e-03) * (q[i7] * q[i22])
            + (1.282765e-03) * (q[i8] * q[i9]) + (2.358254e-05) * (q[i8] * q[i10]) + (7.829275e-04) * (q[i8] * q[i11]) + (1.143059e-03) * (q[i8] * q[i12])
            + (-2.119369e-03) * (q[i8] * q[i15]) + (4.871352e-04) * (q[i8] * q[i16]) + (3.770535e-03) * (q[i8] * q[i19]) + (-2.285660e-03) * (q[i8] * q[i20])
            + (6.368776e-04) * (q[i8] * q[i21]) + (1.847937e-03) * (q[i8] * q[i22]) + (-5.890720e-04) * (q[i9] * q[i10]) + (-1.284133e-03) * (q[i9] * q[i11])
            + (-1.399436e-03) * (q[i9] * q[i12]) + (2.049416e-04) * (q[i9] * q[i15]) + (4.223724e-04) * (q[i9] * q[i16]) + (1.406502e-03) * (q[i9] * q[i19])
            + (-1.023814e-03) * (q[i9] * q[i20]) + (-1.350166e-03) * (q[i9] * q[i21]) + (7.085870e-04) * (q[i9] * q[i22]) + (-6.521371e-04) * (q[i10] * q[i11])
            + (5.601645e-04) * (q[i10] * q[i12]) + (3.035723e-04) * (q[i10] * q[i15]) + (-1.849129e-03) * (q[i10] * q[i16]) + (6.035180e-04) * (q[i10] * q[i19])
            + (-1.015407e-03) * (q[i10] * q[i20]) + (-6.701134e-05) * (q[i10] * q[i21]) + (-6.866560e-04) * (q[i10] * q[i22])
            + (4.141827e-04) * (q[i11] * q[i12]) + (8.568046e-05) * (q[i11] * q[i15]) + (-3.801519e-04) * (q[i11] * q[i16]) + (3.596755e-04) * (q[i11] * q[i19])
            + (-2.743292e-03) * (q[i11] * q[i20]) + (-7.055388e-04) * (q[i11] * q[i21]) + (3.205368e-04) * (q[i11] * q[i22])
            + (-7.205732e-04) * (q[i12] * q[i15]) + (-9.703579e-04) * (q[i12] * q[i16]) + (-4.043702e-03) * (q[i12] * q[i19])
            + (-1.148295e-03) * (q[i12] * q[i20]) + (-3.508251e-04) * (q[i12] * q[i21]) + (-1.932807e-05) * (q[i12] * q[i22])
            + (9.508782e-05) * (q[i15] * q[i16]) + (-5.166564e-04) * (q[i15] * q[i19]) + (1.693989e-03) * (q[i15] * q[i20])
            + (-6.911407e-06) * (q[i15] * q[i21]) + (-2.669654e-04) * (q[i15] * q[i22]) + (-2.059293e-03) * (q[i16] * q[i19])
            + (2.422904e-03) * (q[i16] * q[i20]) + (1.136429e-04) * (q[i16] * q[i21]) + (6.164245e-04) * (q[i16] * q[i22]) + (-5.725087e-04) * (q[i19] * q[i20])
            + (3.786057e-04) * (q[i19] * q[i21]) + (-1.428575e-03) * (q[i19] * q[i22]) + (1.415799e-03) * (q[i20] * q[i21]) + (2.036116e-03) * (q[i20] * q[i22])
            + (-1.514408e-04) * (q[i21] * q[i22]);
   }

   public void getJQz4(double[] q, double[][] JQ)
   {
      JQ[3][i4] = (1.929621e-02) * (1) + (4.233750e-03) * ((2) * q[i4]) + (-4.186895e-02) * (q[i0]) + (6.077392e-02) * (q[i1]) + (1.576084e-02) * (q[i2])
            + (-4.628329e-05) * (q[i3]) + (3.369217e-03) * (q[i5]) + (2.152863e-02) * (q[i6]) + (1.642073e-02) * (q[i7]) + (-4.240503e-03) * (q[i8])
            + (1.108682e-02) * (q[i9]) + (1.371122e-03) * (q[i10]) + (-1.862121e-03) * (q[i11]) + (-4.441355e-03) * (q[i12]) + (8.733209e-04) * (q[i15])
            + (4.485459e-03) * (q[i16]) + (1.228612e-02) * (q[i19]) + (-1.017810e-02) * (q[i20]) + (2.181625e-03) * (q[i21]) + (-4.804226e-03) * (q[i22])
            + (-5.348986e-04) * (q[i0] * q[i0]) + (-6.889231e-03) * (q[i1] * q[i1]) + (-2.291724e-03) * (q[i2] * q[i2]) + (1.952111e-03) * (q[i3] * q[i3])
            + (2.659158e-03) * ((2) * q[i0] * q[i4]) + (-7.650379e-03) * ((2) * q[i1] * q[i4]) + (-1.354439e-03) * ((2) * q[i2] * q[i4])
            + (1.972930e-03) * ((2) * q[i3] * q[i4]) + (1.171165e-03) * ((3) * q[i4] * q[i4]) + (1.526571e-03) * ((2) * q[i4] * q[i5])
            + (-4.233610e-03) * ((2) * q[i4] * q[i6]) + (-9.219435e-03) * ((2) * q[i4] * q[i7]) + (-3.510551e-03) * ((2) * q[i4] * q[i8])
            + (-1.080638e-04) * ((2) * q[i4] * q[i9]) + (-1.172863e-03) * ((2) * q[i4] * q[i10]) + (-1.091657e-03) * ((2) * q[i4] * q[i11])
            + (7.709892e-05) * ((2) * q[i4] * q[i12]) + (1.420788e-03) * ((2) * q[i4] * q[i15]) + (3.186542e-03) * ((2) * q[i4] * q[i16])
            + (2.887153e-03) * ((2) * q[i4] * q[i19]) + (-2.277595e-03) * ((2) * q[i4] * q[i20]) + (1.868968e-03) * ((2) * q[i4] * q[i21])
            + (-1.108247e-04) * ((2) * q[i4] * q[i22]) + (1.262891e-03) * (q[i5] * q[i5]) + (1.972131e-03) * (q[i6] * q[i6]) + (-7.832752e-03) * (q[i7] * q[i7])
            + (-2.809552e-03) * (q[i8] * q[i8]) + (-1.002610e-03) * (q[i9] * q[i9]) + (7.347479e-04) * (q[i10] * q[i10]) + (-9.955130e-04) * (q[i11] * q[i11])
            + (-9.536163e-04) * (q[i12] * q[i12]) + (4.768543e-04) * (q[i15] * q[i15]) + (-8.897604e-04) * (q[i16] * q[i16])
            + (4.783957e-04) * (q[i19] * q[i19]) + (-5.486979e-04) * (q[i20] * q[i20]) + (1.052418e-03) * (q[i21] * q[i21])
            + (-2.329584e-04) * (q[i22] * q[i22]) + (1.405527e-03) * (q[i0] * q[i1]) + (-1.094924e-03) * (q[i0] * q[i2]) + (6.346759e-03) * (q[i0] * q[i3])
            + (2.166891e-03) * (q[i0] * q[i5]) + (2.077978e-03) * (q[i0] * q[i6]) + (-3.874965e-03) * (q[i0] * q[i7]) + (1.921058e-03) * (q[i0] * q[i8])
            + (4.471828e-03) * (q[i0] * q[i9]) + (3.653177e-03) * (q[i0] * q[i10]) + (-7.361672e-05) * (q[i0] * q[i11]) + (6.362083e-04) * (q[i0] * q[i12])
            + (-4.425119e-03) * (q[i0] * q[i15]) + (-1.259237e-03) * (q[i0] * q[i16]) + (1.293175e-03) * (q[i0] * q[i19]) + (-1.629772e-04) * (q[i0] * q[i20])
            + (-2.991066e-04) * (q[i0] * q[i21]) + (-1.857760e-03) * (q[i0] * q[i22]) + (5.408919e-05) * (q[i1] * q[i2]) + (6.358675e-03) * (q[i1] * q[i3])
            + (-1.032987e-02) * (q[i1] * q[i5]) + (7.430544e-05) * (q[i1] * q[i6]) + (1.461029e-02) * (q[i1] * q[i7]) + (-3.676329e-03) * (q[i1] * q[i8])
            + (4.109857e-04) * (q[i1] * q[i9]) + (-4.582681e-03) * (q[i1] * q[i10]) + (-1.219388e-03) * (q[i1] * q[i11]) + (-1.448425e-03) * (q[i1] * q[i12])
            + (2.785258e-03) * (q[i1] * q[i15]) + (-2.476091e-03) * (q[i1] * q[i16]) + (3.806077e-04) * (q[i1] * q[i19]) + (2.369193e-03) * (q[i1] * q[i20])
            + (-1.502616e-03) * (q[i1] * q[i21]) + (1.220969e-03) * (q[i1] * q[i22]) + (-3.414844e-03) * (q[i2] * q[i3]) + (-1.538618e-02) * (q[i2] * q[i5])
            + (2.431319e-03) * (q[i2] * q[i6]) + (4.423881e-03) * (q[i2] * q[i7]) + (2.195300e-04) * (q[i2] * q[i8]) + (2.925550e-03) * (q[i2] * q[i9])
            + (-1.013973e-03) * (q[i2] * q[i10]) + (5.717113e-04) * (q[i2] * q[i11]) + (-2.060311e-03) * (q[i2] * q[i12]) + (-6.114839e-04) * (q[i2] * q[i15])
            + (-3.448970e-03) * (q[i2] * q[i16]) + (8.164762e-04) * (q[i2] * q[i19]) + (2.400144e-03) * (q[i2] * q[i20]) + (-1.327809e-03) * (q[i2] * q[i21])
            + (8.300638e-04) * (q[i2] * q[i22]) + (1.174176e-03) * (q[i3] * q[i5]) + (8.426780e-03) * (q[i3] * q[i6]) + (-8.359363e-03) * (q[i3] * q[i7])
            + (-1.122263e-05) * (q[i3] * q[i8]) + (-8.118305e-04) * (q[i3] * q[i9]) + (8.529342e-04) * (q[i3] * q[i10]) + (3.597437e-03) * (q[i3] * q[i11])
            + (3.627772e-03) * (q[i3] * q[i12]) + (3.793315e-03) * (q[i3] * q[i15]) + (-3.798990e-03) * (q[i3] * q[i16]) + (-6.057769e-03) * (q[i3] * q[i19])
            + (6.060812e-03) * (q[i3] * q[i20]) + (-5.361392e-04) * (q[i3] * q[i21]) + (-5.363010e-04) * (q[i3] * q[i22]) + (3.088027e-03) * (q[i5] * q[i6])
            + (7.010348e-03) * (q[i5] * q[i7]) + (1.495923e-02) * (q[i5] * q[i8]) + (-1.137618e-03) * (q[i5] * q[i9]) + (1.745041e-04) * (q[i5] * q[i10])
            + (-1.941690e-03) * (q[i5] * q[i11]) + (-9.799941e-04) * (q[i5] * q[i12]) + (-3.038147e-03) * (q[i5] * q[i15]) + (1.249266e-03) * (q[i5] * q[i16])
            + (1.341935e-03) * (q[i5] * q[i19]) + (1.377475e-03) * (q[i5] * q[i20]) + (-1.064690e-03) * (q[i5] * q[i21]) + (1.727949e-03) * (q[i5] * q[i22])
            + (1.063625e-03) * (q[i6] * q[i7]) + (1.137326e-03) * (q[i6] * q[i8]) + (-2.258961e-03) * (q[i6] * q[i9]) + (-8.680651e-04) * (q[i6] * q[i10])
            + (-1.959240e-03) * (q[i6] * q[i11]) + (-4.171843e-04) * (q[i6] * q[i12]) + (-2.126183e-03) * (q[i6] * q[i15]) + (1.788607e-03) * (q[i6] * q[i16])
            + (4.188873e-03) * (q[i6] * q[i19]) + (1.145180e-03) * (q[i6] * q[i20]) + (1.227218e-03) * (q[i6] * q[i21]) + (-2.634513e-03) * (q[i6] * q[i22])
            + (-2.108197e-03) * (q[i7] * q[i8]) + (3.591907e-04) * (q[i7] * q[i9]) + (-1.167074e-03) * (q[i7] * q[i10]) + (-4.589427e-04) * (q[i7] * q[i11])
            + (-6.756111e-04) * (q[i7] * q[i12]) + (3.312536e-03) * (q[i7] * q[i15]) + (3.367054e-03) * (q[i7] * q[i16]) + (-9.830795e-04) * (q[i7] * q[i19])
            + (-1.690906e-03) * (q[i7] * q[i20]) + (-9.733502e-04) * (q[i7] * q[i21]) + (2.156502e-03) * (q[i7] * q[i22]) + (1.912230e-05) * (q[i8] * q[i9])
            + (1.274215e-03) * (q[i8] * q[i10]) + (-1.178815e-03) * (q[i8] * q[i11]) + (-8.375840e-04) * (q[i8] * q[i12]) + (4.996041e-04) * (q[i8] * q[i15])
            + (-2.130278e-03) * (q[i8] * q[i16]) + (-2.264376e-03) * (q[i8] * q[i19]) + (3.739202e-03) * (q[i8] * q[i20]) + (-1.838598e-03) * (q[i8] * q[i21])
            + (-6.206917e-04) * (q[i8] * q[i22]) + (-5.872507e-04) * (q[i9] * q[i10]) + (-5.744456e-04) * (q[i9] * q[i11]) + (6.472719e-04) * (q[i9] * q[i12])
            + (-1.838502e-03) * (q[i9] * q[i15]) + (3.111986e-04) * (q[i9] * q[i16]) + (-1.010468e-03) * (q[i9] * q[i19]) + (6.126302e-04) * (q[i9] * q[i20])
            + (6.892232e-04) * (q[i9] * q[i21]) + (6.536468e-05) * (q[i9] * q[i22]) + (1.395561e-03) * (q[i10] * q[i11]) + (1.288199e-03) * (q[i10] * q[i12])
            + (4.064886e-04) * (q[i10] * q[i15]) + (2.140959e-04) * (q[i10] * q[i16]) + (-1.031287e-03) * (q[i10] * q[i19]) + (1.394813e-03) * (q[i10] * q[i20])
            + (-7.013622e-04) * (q[i10] * q[i21]) + (1.339857e-03) * (q[i10] * q[i22]) + (4.122995e-04) * (q[i11] * q[i12]) + (9.640143e-04) * (q[i11] * q[i15])
            + (7.220888e-04) * (q[i11] * q[i16]) + (1.144213e-03) * (q[i11] * q[i19]) + (4.028919e-03) * (q[i11] * q[i20]) + (-1.348688e-05) * (q[i11] * q[i21])
            + (-3.507120e-04) * (q[i11] * q[i22]) + (3.887333e-04) * (q[i12] * q[i15]) + (-7.257782e-05) * (q[i12] * q[i16])
            + (2.760975e-03) * (q[i12] * q[i19]) + (-3.517347e-04) * (q[i12] * q[i20]) + (3.232380e-04) * (q[i12] * q[i21])
            + (-7.104490e-04) * (q[i12] * q[i22]) + (1.030262e-04) * (q[i15] * q[i16]) + (2.427859e-03) * (q[i15] * q[i19])
            + (-2.048765e-03) * (q[i15] * q[i20]) + (-6.219502e-04) * (q[i15] * q[i21]) + (-9.526688e-05) * (q[i15] * q[i22])
            + (1.697302e-03) * (q[i16] * q[i19]) + (-5.161785e-04) * (q[i16] * q[i20]) + (2.637954e-04) * (q[i16] * q[i21]) + (3.106142e-05) * (q[i16] * q[i22])
            + (-5.556551e-04) * (q[i19] * q[i20]) + (-2.016544e-03) * (q[i19] * q[i21]) + (-1.405249e-03) * (q[i19] * q[i22])
            + (1.415346e-03) * (q[i20] * q[i21]) + (-3.705879e-04) * (q[i20] * q[i22]) + (-1.554328e-04) * (q[i21] * q[i22]);
   }

   public void getJQz5(double[] q, double[][] JQ)
   {
      JQ[3][i5] = (-1.727993e-02) * (1) + (8.832997e-07) * ((2) * q[i5]) + (-4.252679e-03) * (q[i0]) + (4.187169e-03) * (q[i1]) + (-6.851522e-05) * (q[i2])
            + (-3.369455e-03) * (q[i3]) + (3.369217e-03) * (q[i4]) + (2.376484e-02) * (q[i6]) + (2.352113e-02) * (q[i7]) + (4.538288e-02) * (q[i8])
            + (8.665537e-03) * (q[i9]) + (8.572595e-03) * (q[i10]) + (-4.070402e-03) * (q[i11]) + (4.230532e-03) * (q[i12]) + (-3.330178e-03) * (q[i15])
            + (-3.352356e-03) * (q[i16]) + (4.195663e-03) * (q[i19]) + (4.206466e-03) * (q[i20]) + (-3.430954e-03) * (q[i21]) + (3.408243e-03) * (q[i22])
            + (1.705448e-04) * (q[i0] * q[i0]) + (1.741400e-04) * (q[i1] * q[i1]) + (4.036315e-03) * (q[i2] * q[i2]) + (1.556399e-03) * (q[i3] * q[i3])
            + (1.526571e-03) * (q[i4] * q[i4]) + (-1.317582e-03) * ((2) * q[i0] * q[i5]) + (-1.370099e-03) * ((2) * q[i1] * q[i5])
            + (-9.658555e-03) * ((2) * q[i2] * q[i5]) + (1.226351e-03) * ((2) * q[i3] * q[i5]) + (1.262891e-03) * ((2) * q[i4] * q[i5])
            + (5.050062e-03) * ((3) * q[i5] * q[i5]) + (-1.825219e-03) * ((2) * q[i5] * q[i6]) + (1.814481e-03) * ((2) * q[i5] * q[i7])
            + (6.752906e-05) * ((2) * q[i5] * q[i8]) + (-7.996915e-04) * ((2) * q[i5] * q[i9]) + (7.831560e-04) * ((2) * q[i5] * q[i10])
            + (1.764104e-03) * ((2) * q[i5] * q[i11]) + (1.870958e-03) * ((2) * q[i5] * q[i12]) + (2.081274e-03) * ((2) * q[i5] * q[i15])
            + (-2.095174e-03) * ((2) * q[i5] * q[i16]) + (3.035728e-06) * ((2) * q[i5] * q[i19]) + (1.337627e-06) * ((2) * q[i5] * q[i20])
            + (5.800218e-04) * ((2) * q[i5] * q[i21]) + (5.814167e-04) * ((2) * q[i5] * q[i22]) + (6.330235e-03) * (q[i6] * q[i6])
            + (6.327663e-03) * (q[i7] * q[i7]) + (2.052357e-03) * (q[i8] * q[i8]) + (-1.284342e-03) * (q[i9] * q[i9]) + (-1.257176e-03) * (q[i10] * q[i10])
            + (-5.658690e-05) * (q[i11] * q[i11]) + (-3.241012e-05) * (q[i12] * q[i12]) + (-1.741244e-03) * (q[i15] * q[i15])
            + (-1.767671e-03) * (q[i16] * q[i16]) + (5.838992e-04) * (q[i19] * q[i19]) + (5.822137e-04) * (q[i20] * q[i20]) + (2.817858e-04) * (q[i21] * q[i21])
            + (2.937040e-04) * (q[i22] * q[i22]) + (-1.495583e-03) * (q[i0] * q[i1]) + (3.503458e-03) * (q[i0] * q[i2]) + (-1.038624e-02) * (q[i0] * q[i3])
            + (2.166891e-03) * (q[i0] * q[i4]) + (1.889204e-03) * (q[i0] * q[i6]) + (-1.951168e-03) * (q[i0] * q[i7]) + (4.131932e-03) * (q[i0] * q[i8])
            + (1.215783e-03) * (q[i0] * q[i9]) + (5.609378e-04) * (q[i0] * q[i10]) + (1.385829e-03) * (q[i0] * q[i11]) + (3.532723e-04) * (q[i0] * q[i12])
            + (3.217083e-03) * (q[i0] * q[i15]) + (-1.774397e-03) * (q[i0] * q[i16]) + (-4.666657e-04) * (q[i0] * q[i19]) + (-4.053795e-04) * (q[i0] * q[i20])
            + (-1.545775e-03) * (q[i0] * q[i21]) + (-1.651426e-03) * (q[i0] * q[i22]) + (3.533647e-03) * (q[i1] * q[i2]) + (2.140390e-03) * (q[i1] * q[i3])
            + (-1.032987e-02) * (q[i1] * q[i4]) + (1.989037e-03) * (q[i1] * q[i6]) + (-1.894315e-03) * (q[i1] * q[i7]) + (-4.131924e-03) * (q[i1] * q[i8])
            + (-5.376108e-04) * (q[i1] * q[i9]) + (-1.196318e-03) * (q[i1] * q[i10]) + (3.883544e-04) * (q[i1] * q[i11]) + (1.359287e-03) * (q[i1] * q[i12])
            + (1.774251e-03) * (q[i1] * q[i15]) + (-3.250903e-03) * (q[i1] * q[i16]) + (3.999837e-04) * (q[i1] * q[i19]) + (4.605860e-04) * (q[i1] * q[i20])
            + (-1.623950e-03) * (q[i1] * q[i21]) + (-1.506938e-03) * (q[i1] * q[i22]) + (-1.548406e-02) * (q[i2] * q[i3]) + (-1.538618e-02) * (q[i2] * q[i4])
            + (-2.733054e-03) * (q[i2] * q[i6]) + (2.764813e-03) * (q[i2] * q[i7]) + (-1.664079e-05) * (q[i2] * q[i8]) + (1.866223e-03) * (q[i2] * q[i9])
            + (-1.855923e-03) * (q[i2] * q[i10]) + (-6.183201e-04) * (q[i2] * q[i11]) + (-6.954248e-04) * (q[i2] * q[i12]) + (5.859833e-03) * (q[i2] * q[i15])
            + (-5.913097e-03) * (q[i2] * q[i16]) + (-1.748491e-03) * (q[i2] * q[i19]) + (1.751262e-03) * (q[i2] * q[i20]) + (-1.068426e-03) * (q[i2] * q[i21])
            + (-1.038283e-03) * (q[i2] * q[i22]) + (1.174176e-03) * (q[i3] * q[i4]) + (-7.018283e-03) * (q[i3] * q[i6]) + (-3.106559e-03) * (q[i3] * q[i7])
            + (-1.497916e-02) * (q[i3] * q[i8]) + (-1.775080e-04) * (q[i3] * q[i9]) + (1.140135e-03) * (q[i3] * q[i10]) + (-9.524564e-04) * (q[i3] * q[i11])
            + (-2.029475e-03) * (q[i3] * q[i12]) + (-1.225580e-03) * (q[i3] * q[i15]) + (3.048870e-03) * (q[i3] * q[i16]) + (-1.357805e-03) * (q[i3] * q[i19])
            + (-1.363373e-03) * (q[i3] * q[i20]) + (1.755457e-03) * (q[i3] * q[i21]) + (-1.061229e-03) * (q[i3] * q[i22]) + (3.088027e-03) * (q[i4] * q[i6])
            + (7.010348e-03) * (q[i4] * q[i7]) + (1.495923e-02) * (q[i4] * q[i8]) + (-1.137618e-03) * (q[i4] * q[i9]) + (1.745041e-04) * (q[i4] * q[i10])
            + (-1.941690e-03) * (q[i4] * q[i11]) + (-9.799941e-04) * (q[i4] * q[i12]) + (-3.038147e-03) * (q[i4] * q[i15]) + (1.249266e-03) * (q[i4] * q[i16])
            + (1.341935e-03) * (q[i4] * q[i19]) + (1.377475e-03) * (q[i4] * q[i20]) + (-1.064690e-03) * (q[i4] * q[i21]) + (1.727949e-03) * (q[i4] * q[i22])
            + (-5.513636e-03) * (q[i6] * q[i7]) + (-4.061282e-04) * (q[i6] * q[i8]) + (-2.733507e-03) * (q[i6] * q[i9]) + (2.534805e-05) * (q[i6] * q[i10])
            + (2.757766e-04) * (q[i6] * q[i11]) + (-2.044780e-03) * (q[i6] * q[i12]) + (-3.466603e-03) * (q[i6] * q[i15]) + (-1.234001e-03) * (q[i6] * q[i16])
            + (3.728839e-03) * (q[i6] * q[i19]) + (-2.791426e-03) * (q[i6] * q[i20]) + (2.373988e-03) * (q[i6] * q[i21]) + (7.504333e-04) * (q[i6] * q[i22])
            + (-3.849809e-04) * (q[i7] * q[i8]) + (3.793233e-05) * (q[i7] * q[i9]) + (-2.653394e-03) * (q[i7] * q[i10]) + (2.087253e-03) * (q[i7] * q[i11])
            + (-2.436332e-04) * (q[i7] * q[i12]) + (-1.246730e-03) * (q[i7] * q[i15]) + (-3.479245e-03) * (q[i7] * q[i16]) + (-2.770813e-03) * (q[i7] * q[i19])
            + (3.707763e-03) * (q[i7] * q[i20]) + (-7.583026e-04) * (q[i7] * q[i21]) + (-2.373746e-03) * (q[i7] * q[i22]) + (2.207665e-03) * (q[i8] * q[i9])
            + (2.197406e-03) * (q[i8] * q[i10]) + (-3.876869e-03) * (q[i8] * q[i11]) + (4.035952e-03) * (q[i8] * q[i12]) + (-2.887911e-03) * (q[i8] * q[i15])
            + (-2.914658e-03) * (q[i8] * q[i16]) + (1.483302e-05) * (q[i8] * q[i19]) + (5.371657e-05) * (q[i8] * q[i20]) + (7.720520e-05) * (q[i8] * q[i21])
            + (-1.235063e-04) * (q[i8] * q[i22]) + (1.099198e-03) * (q[i9] * q[i10]) + (3.362156e-04) * (q[i9] * q[i11]) + (-1.014929e-04) * (q[i9] * q[i12])
            + (4.149904e-04) * (q[i9] * q[i15]) + (1.719742e-04) * (q[i9] * q[i16]) + (-6.854388e-04) * (q[i9] * q[i19]) + (-5.408115e-04) * (q[i9] * q[i20])
            + (9.070192e-04) * (q[i9] * q[i21]) + (-8.920332e-04) * (q[i9] * q[i22]) + (1.083697e-04) * (q[i10] * q[i11]) + (-3.021511e-04) * (q[i10] * q[i12])
            + (1.927727e-04) * (q[i10] * q[i15]) + (4.171139e-04) * (q[i10] * q[i16]) + (-5.324809e-04) * (q[i10] * q[i19])
            + (-6.700685e-04) * (q[i10] * q[i20]) + (8.840609e-04) * (q[i10] * q[i21]) + (-9.083725e-04) * (q[i10] * q[i22])
            + (-1.144017e-04) * (q[i11] * q[i12]) + (-3.131738e-03) * (q[i11] * q[i15]) + (8.952596e-04) * (q[i11] * q[i16])
            + (-2.765632e-03) * (q[i11] * q[i19]) + (1.098577e-03) * (q[i11] * q[i20]) + (-1.005917e-03) * (q[i11] * q[i21])
            + (4.014403e-04) * (q[i11] * q[i22]) + (-9.011617e-04) * (q[i12] * q[i15]) + (3.142238e-03) * (q[i12] * q[i16])
            + (-1.107020e-03) * (q[i12] * q[i19]) + (2.738680e-03) * (q[i12] * q[i20]) + (4.047226e-04) * (q[i12] * q[i21])
            + (-9.949296e-04) * (q[i12] * q[i22]) + (2.660128e-04) * (q[i15] * q[i16]) + (1.010086e-04) * (q[i15] * q[i19]) + (1.702609e-05) * (q[i15] * q[i20])
            + (-3.191987e-04) * (q[i15] * q[i21]) + (3.559548e-04) * (q[i15] * q[i22]) + (1.734017e-05) * (q[i16] * q[i19]) + (1.067308e-04) * (q[i16] * q[i20])
            + (-3.566293e-04) * (q[i16] * q[i21]) + (3.086819e-04) * (q[i16] * q[i22]) + (1.162797e-03) * (q[i19] * q[i20]) + (9.746688e-04) * (q[i19] * q[i21])
            + (8.000354e-04) * (q[i19] * q[i22]) + (-7.983432e-04) * (q[i20] * q[i21]) + (-9.931816e-04) * (q[i20] * q[i22])
            + (7.407358e-04) * (q[i21] * q[i22]);
   }

   public void getJQz6(double[] q, double[][] JQ)
   {
      JQ[3][i6] = (8.482997e-02) * (1) + (-2.480046e-03) * ((2) * q[i6]) + (2.489028e-02) * (q[i0]) + (-1.000115e-02) * (q[i1]) + (1.301239e-02) * (q[i2])
            + (1.660043e-02) * (q[i3]) + (2.152863e-02) * (q[i4]) + (2.376484e-02) * (q[i5]) + (-5.543573e-06) * (q[i7]) + (-8.516581e-03) * (q[i8])
            + (-2.719946e-03) * (q[i9]) + (3.286257e-03) * (q[i10]) + (7.732070e-04) * (q[i11]) + (4.117008e-03) * (q[i12]) + (-9.829345e-05) * (q[i15])
            + (9.207379e-03) * (q[i16]) + (-3.224428e-03) * (q[i19]) + (-1.961197e-03) * (q[i20]) + (6.853790e-04) * (q[i21]) + (7.736524e-04) * (q[i22])
            + (-1.109336e-02) * (q[i0] * q[i0]) + (1.946229e-03) * (q[i1] * q[i1]) + (-1.087924e-05) * (q[i2] * q[i2]) + (9.172461e-03) * (q[i3] * q[i3])
            + (-4.233610e-03) * (q[i4] * q[i4]) + (-1.825219e-03) * (q[i5] * q[i5]) + (-1.145720e-02) * ((2) * q[i0] * q[i6])
            + (6.238411e-03) * ((2) * q[i1] * q[i6]) + (-6.167978e-03) * ((2) * q[i2] * q[i6]) + (-7.821550e-03) * ((2) * q[i3] * q[i6])
            + (1.972131e-03) * ((2) * q[i4] * q[i6]) + (6.330235e-03) * ((2) * q[i5] * q[i6]) + (-6.023597e-03) * ((3) * q[i6] * q[i6])
            + (5.546016e-03) * ((2) * q[i6] * q[i7]) + (3.682597e-03) * ((2) * q[i6] * q[i8]) + (-2.824449e-04) * ((2) * q[i6] * q[i9])
            + (2.238024e-03) * ((2) * q[i6] * q[i10]) + (-1.910259e-03) * ((2) * q[i6] * q[i11]) + (-8.133748e-04) * ((2) * q[i6] * q[i12])
            + (1.964660e-03) * ((2) * q[i6] * q[i15]) + (-1.063728e-03) * ((2) * q[i6] * q[i16]) + (-1.662778e-03) * ((2) * q[i6] * q[i19])
            + (3.625732e-04) * ((2) * q[i6] * q[i20]) + (-7.256285e-04) * ((2) * q[i6] * q[i21]) + (-1.595204e-03) * ((2) * q[i6] * q[i22])
            + (-5.540731e-03) * (q[i7] * q[i7]) + (-1.045167e-03) * (q[i8] * q[i8]) + (-1.925311e-03) * (q[i9] * q[i9]) + (-2.083735e-04) * (q[i10] * q[i10])
            + (1.614308e-03) * (q[i11] * q[i11]) + (1.569816e-03) * (q[i12] * q[i12]) + (1.854455e-03) * (q[i15] * q[i15]) + (1.498998e-03) * (q[i16] * q[i16])
            + (1.224872e-03) * (q[i19] * q[i19]) + (1.177861e-03) * (q[i20] * q[i20]) + (1.271774e-03) * (q[i21] * q[i21]) + (1.395523e-03) * (q[i22] * q[i22])
            + (2.440913e-03) * (q[i0] * q[i1]) + (-9.255762e-03) * (q[i0] * q[i2]) + (-1.448655e-02) * (q[i0] * q[i3]) + (2.077978e-03) * (q[i0] * q[i4])
            + (1.889204e-03) * (q[i0] * q[i5]) + (1.064443e-02) * (q[i0] * q[i7]) + (2.018270e-03) * (q[i0] * q[i8]) + (-8.093356e-03) * (q[i0] * q[i9])
            + (4.731231e-03) * (q[i0] * q[i10]) + (1.256146e-03) * (q[i0] * q[i11]) + (3.351741e-03) * (q[i0] * q[i12]) + (-4.457664e-04) * (q[i0] * q[i15])
            + (2.525945e-03) * (q[i0] * q[i16]) + (-3.448775e-03) * (q[i0] * q[i19]) + (5.469725e-04) * (q[i0] * q[i20]) + (1.146224e-03) * (q[i0] * q[i21])
            + (8.591253e-04) * (q[i0] * q[i22]) + (8.852707e-04) * (q[i1] * q[i2]) + (3.777589e-03) * (q[i1] * q[i3]) + (7.430544e-05) * (q[i1] * q[i4])
            + (1.989037e-03) * (q[i1] * q[i5]) + (1.066394e-02) * (q[i1] * q[i7]) + (-5.539103e-04) * (q[i1] * q[i8]) + (6.241155e-03) * (q[i1] * q[i9])
            + (7.006525e-04) * (q[i1] * q[i10]) + (1.968200e-03) * (q[i1] * q[i11]) + (-8.547104e-05) * (q[i1] * q[i12]) + (-2.089086e-03) * (q[i1] * q[i15])
            + (1.587016e-03) * (q[i1] * q[i16]) + (-1.365403e-04) * (q[i1] * q[i19]) + (-1.235479e-03) * (q[i1] * q[i20]) + (6.942189e-04) * (q[i1] * q[i21])
            + (7.825132e-04) * (q[i1] * q[i22]) + (-4.403246e-03) * (q[i2] * q[i3]) + (2.431319e-03) * (q[i2] * q[i4]) + (-2.733054e-03) * (q[i2] * q[i5])
            + (1.259057e-02) * (q[i2] * q[i7]) + (-7.272094e-03) * (q[i2] * q[i8]) + (-4.764478e-03) * (q[i2] * q[i9]) + (7.771415e-04) * (q[i2] * q[i10])
            + (8.556703e-06) * (q[i2] * q[i11]) + (2.338387e-03) * (q[i2] * q[i12]) + (-1.378391e-03) * (q[i2] * q[i15]) + (9.848270e-04) * (q[i2] * q[i16])
            + (-2.454018e-05) * (q[i2] * q[i19]) + (3.393243e-04) * (q[i2] * q[i20]) + (-9.109117e-04) * (q[i2] * q[i21]) + (7.190790e-04) * (q[i2] * q[i22])
            + (8.426780e-03) * (q[i3] * q[i4]) + (-7.018283e-03) * (q[i3] * q[i5]) + (1.009055e-03) * (q[i3] * q[i7]) + (-2.132648e-03) * (q[i3] * q[i8])
            + (-1.094750e-03) * (q[i3] * q[i9]) + (3.304197e-04) * (q[i3] * q[i10]) + (6.582829e-04) * (q[i3] * q[i11]) + (4.444862e-04) * (q[i3] * q[i12])
            + (3.373408e-03) * (q[i3] * q[i15]) + (3.345002e-03) * (q[i3] * q[i16]) + (-1.681426e-03) * (q[i3] * q[i19]) + (-9.682098e-04) * (q[i3] * q[i20])
            + (-2.165034e-03) * (q[i3] * q[i21]) + (9.942412e-04) * (q[i3] * q[i22]) + (3.088027e-03) * (q[i4] * q[i5]) + (1.063625e-03) * (q[i4] * q[i7])
            + (1.137326e-03) * (q[i4] * q[i8]) + (-2.258961e-03) * (q[i4] * q[i9]) + (-8.680651e-04) * (q[i4] * q[i10]) + (-1.959240e-03) * (q[i4] * q[i11])
            + (-4.171843e-04) * (q[i4] * q[i12]) + (-2.126183e-03) * (q[i4] * q[i15]) + (1.788607e-03) * (q[i4] * q[i16]) + (4.188873e-03) * (q[i4] * q[i19])
            + (1.145180e-03) * (q[i4] * q[i20]) + (1.227218e-03) * (q[i4] * q[i21]) + (-2.634513e-03) * (q[i4] * q[i22]) + (-5.513636e-03) * (q[i5] * q[i7])
            + (-4.061282e-04) * (q[i5] * q[i8]) + (-2.733507e-03) * (q[i5] * q[i9]) + (2.534805e-05) * (q[i5] * q[i10]) + (2.757766e-04) * (q[i5] * q[i11])
            + (-2.044780e-03) * (q[i5] * q[i12]) + (-3.466603e-03) * (q[i5] * q[i15]) + (-1.234001e-03) * (q[i5] * q[i16]) + (3.728839e-03) * (q[i5] * q[i19])
            + (-2.791426e-03) * (q[i5] * q[i20]) + (2.373988e-03) * (q[i5] * q[i21]) + (7.504333e-04) * (q[i5] * q[i22]) + (4.983881e-07) * (q[i7] * q[i8])
            + (3.256982e-03) * (q[i7] * q[i9]) + (-3.232325e-03) * (q[i7] * q[i10]) + (9.011420e-04) * (q[i7] * q[i11]) + (8.889800e-04) * (q[i7] * q[i12])
            + (-4.010950e-03) * (q[i7] * q[i15]) + (4.011652e-03) * (q[i7] * q[i16]) + (3.888448e-03) * (q[i7] * q[i19]) + (-3.886024e-03) * (q[i7] * q[i20])
            + (1.910748e-03) * (q[i7] * q[i21]) + (1.899129e-03) * (q[i7] * q[i22]) + (6.320989e-04) * (q[i8] * q[i9]) + (8.281645e-04) * (q[i8] * q[i10])
            + (1.858958e-03) * (q[i8] * q[i11]) + (5.226429e-04) * (q[i8] * q[i12]) + (8.525784e-04) * (q[i8] * q[i15]) + (-3.242813e-03) * (q[i8] * q[i16])
            + (-7.519003e-04) * (q[i8] * q[i19]) + (2.245990e-03) * (q[i8] * q[i20]) + (-2.577027e-03) * (q[i8] * q[i21]) + (5.465064e-05) * (q[i8] * q[i22])
            + (3.559246e-05) * (q[i9] * q[i10]) + (4.895574e-04) * (q[i9] * q[i11]) + (-1.295664e-03) * (q[i9] * q[i12]) + (9.019470e-04) * (q[i9] * q[i15])
            + (-5.992658e-04) * (q[i9] * q[i16]) + (-2.982457e-04) * (q[i9] * q[i19]) + (1.733925e-03) * (q[i9] * q[i20]) + (5.865629e-05) * (q[i9] * q[i21])
            + (-8.798216e-04) * (q[i9] * q[i22]) + (6.311207e-04) * (q[i10] * q[i11]) + (-2.058049e-03) * (q[i10] * q[i12])
            + (-8.672907e-04) * (q[i10] * q[i15]) + (3.818487e-04) * (q[i10] * q[i16]) + (1.426584e-03) * (q[i10] * q[i19]) + (9.999129e-05) * (q[i10] * q[i20])
            + (7.224236e-04) * (q[i10] * q[i21]) + (-1.862710e-04) * (q[i10] * q[i22]) + (-3.388114e-03) * (q[i11] * q[i12])
            + (2.391041e-04) * (q[i11] * q[i15]) + (1.804663e-03) * (q[i11] * q[i16]) + (1.196569e-03) * (q[i11] * q[i19]) + (-8.816488e-04) * (q[i11] * q[i20])
            + (1.821923e-03) * (q[i11] * q[i21]) + (1.017119e-03) * (q[i11] * q[i22]) + (-1.811602e-03) * (q[i12] * q[i15]) + (6.003288e-04) * (q[i12] * q[i16])
            + (4.007865e-05) * (q[i12] * q[i19]) + (-1.552227e-03) * (q[i12] * q[i20]) + (-7.942234e-04) * (q[i12] * q[i21])
            + (3.878863e-04) * (q[i12] * q[i22]) + (2.550293e-04) * (q[i15] * q[i16]) + (-1.196648e-03) * (q[i15] * q[i19]) + (1.111495e-03) * (q[i15] * q[i20])
            + (-1.324569e-03) * (q[i15] * q[i21]) + (1.150947e-03) * (q[i15] * q[i22]) + (-3.283877e-04) * (q[i16] * q[i19])
            + (-2.231642e-04) * (q[i16] * q[i20]) + (4.686602e-04) * (q[i16] * q[i21]) + (-3.558965e-04) * (q[i16] * q[i22])
            + (2.706007e-03) * (q[i19] * q[i20]) + (-5.003558e-04) * (q[i19] * q[i21]) + (7.818853e-04) * (q[i19] * q[i22])
            + (-2.724772e-04) * (q[i20] * q[i21]) + (-2.384966e-03) * (q[i20] * q[i22]) + (-4.921650e-04) * (q[i21] * q[i22]);
   }

   public void getJQz7(double[] q, double[][] JQ)
   {
      JQ[3][i7] = (-8.463776e-02) * (1) + (2.461622e-03) * ((2) * q[i7]) + (-9.974459e-03) * (q[i0]) + (2.483707e-02) * (q[i1]) + (1.298224e-02) * (q[i2])
            + (2.155436e-02) * (q[i3]) + (1.642073e-02) * (q[i4]) + (2.352113e-02) * (q[i5]) + (-5.543573e-06) * (q[i6]) + (8.428740e-03) * (q[i8])
            + (-3.306397e-03) * (q[i9]) + (2.648012e-03) * (q[i10]) + (4.191025e-03) * (q[i11]) + (8.121325e-04) * (q[i12]) + (-9.059258e-03) * (q[i15])
            + (5.056939e-05) * (q[i16]) + (1.962425e-03) * (q[i19]) + (3.172150e-03) * (q[i20]) + (7.724307e-04) * (q[i21]) + (6.615569e-04) * (q[i22])
            + (-1.924448e-03) * (q[i0] * q[i0]) + (1.113704e-02) * (q[i1] * q[i1]) + (9.450271e-06) * (q[i2] * q[i2]) + (4.204319e-03) * (q[i3] * q[i3])
            + (-9.219435e-03) * (q[i4] * q[i4]) + (1.814481e-03) * (q[i5] * q[i5]) + (5.546016e-03) * (q[i6] * q[i6]) + (6.246037e-03) * ((2) * q[i0] * q[i7])
            + (-1.145755e-02) * ((2) * q[i1] * q[i7]) + (-6.130194e-03) * ((2) * q[i2] * q[i7]) + (1.974651e-03) * ((2) * q[i3] * q[i7])
            + (-7.832752e-03) * ((2) * q[i4] * q[i7]) + (6.327663e-03) * ((2) * q[i5] * q[i7]) + (-5.540731e-03) * ((2) * q[i6] * q[i7])
            + (6.023584e-03) * ((3) * q[i7] * q[i7]) + (-3.694758e-03) * ((2) * q[i7] * q[i8]) + (-2.250312e-03) * ((2) * q[i7] * q[i9])
            + (2.785022e-04) * ((2) * q[i7] * q[i10]) + (-8.050146e-04) * ((2) * q[i7] * q[i11]) + (-1.907086e-03) * ((2) * q[i7] * q[i12])
            + (1.072717e-03) * ((2) * q[i7] * q[i15]) + (-1.961758e-03) * ((2) * q[i7] * q[i16]) + (-3.645672e-04) * ((2) * q[i7] * q[i19])
            + (1.653534e-03) * ((2) * q[i7] * q[i20]) + (-1.584634e-03) * ((2) * q[i7] * q[i21]) + (-7.159547e-04) * ((2) * q[i7] * q[i22])
            + (1.055293e-03) * (q[i8] * q[i8]) + (2.180055e-04) * (q[i9] * q[i9]) + (1.908832e-03) * (q[i10] * q[i10]) + (-1.541249e-03) * (q[i11] * q[i11])
            + (-1.602720e-03) * (q[i12] * q[i12]) + (-1.486156e-03) * (q[i15] * q[i15]) + (-1.862477e-03) * (q[i16] * q[i16])
            + (-1.188255e-03) * (q[i19] * q[i19]) + (-1.234309e-03) * (q[i20] * q[i20]) + (-1.401571e-03) * (q[i21] * q[i21])
            + (-1.274019e-03) * (q[i22] * q[i22]) + (-2.509785e-03) * (q[i0] * q[i1]) + (-9.079879e-04) * (q[i0] * q[i2]) + (-8.385057e-05) * (q[i0] * q[i3])
            + (-3.874965e-03) * (q[i0] * q[i4]) + (-1.951168e-03) * (q[i0] * q[i5]) + (1.064443e-02) * (q[i0] * q[i6]) + (-5.674026e-04) * (q[i0] * q[i8])
            + (7.027701e-04) * (q[i0] * q[i9]) + (6.223994e-03) * (q[i0] * q[i10]) + (6.106089e-05) * (q[i0] * q[i11]) + (-1.928167e-03) * (q[i0] * q[i12])
            + (1.597382e-03) * (q[i0] * q[i15]) + (-2.070965e-03) * (q[i0] * q[i16]) + (-1.237260e-03) * (q[i0] * q[i19]) + (-1.390933e-04) * (q[i0] * q[i20])
            + (-7.828759e-04) * (q[i0] * q[i21]) + (-6.802556e-04) * (q[i0] * q[i22]) + (9.255482e-03) * (q[i1] * q[i2]) + (-2.093520e-03) * (q[i1] * q[i3])
            + (1.461029e-02) * (q[i1] * q[i4]) + (-1.894315e-03) * (q[i1] * q[i5]) + (1.066394e-02) * (q[i1] * q[i6]) + (1.990251e-03) * (q[i1] * q[i8])
            + (4.768331e-03) * (q[i1] * q[i9]) + (-8.073847e-03) * (q[i1] * q[i10]) + (-3.302676e-03) * (q[i1] * q[i11]) + (-1.266723e-03) * (q[i1] * q[i12])
            + (2.515716e-03) * (q[i1] * q[i15]) + (-4.501706e-04) * (q[i1] * q[i16]) + (5.669661e-04) * (q[i1] * q[i19]) + (-3.425146e-03) * (q[i1] * q[i20])
            + (-8.601597e-04) * (q[i1] * q[i21]) + (-1.137522e-03) * (q[i1] * q[i22]) + (-2.472890e-03) * (q[i2] * q[i3]) + (4.423881e-03) * (q[i2] * q[i4])
            + (2.764813e-03) * (q[i2] * q[i5]) + (1.259057e-02) * (q[i2] * q[i6]) + (-7.247755e-03) * (q[i2] * q[i8]) + (7.925011e-04) * (q[i2] * q[i9])
            + (-4.733092e-03) * (q[i2] * q[i10]) + (-2.324831e-03) * (q[i2] * q[i11]) + (-3.349756e-06) * (q[i2] * q[i12]) + (9.778492e-04) * (q[i2] * q[i15])
            + (-1.405652e-03) * (q[i2] * q[i16]) + (3.461047e-04) * (q[i2] * q[i19]) + (-1.300624e-05) * (q[i2] * q[i20]) + (-7.129322e-04) * (q[i2] * q[i21])
            + (9.094359e-04) * (q[i2] * q[i22]) + (-8.359363e-03) * (q[i3] * q[i4]) + (-3.106559e-03) * (q[i3] * q[i5]) + (1.009055e-03) * (q[i3] * q[i6])
            + (1.143908e-03) * (q[i3] * q[i8]) + (-9.034955e-04) * (q[i3] * q[i9]) + (-2.223610e-03) * (q[i3] * q[i10]) + (3.926526e-04) * (q[i3] * q[i11])
            + (1.952225e-03) * (q[i3] * q[i12]) + (1.778172e-03) * (q[i3] * q[i15]) + (-2.089276e-03) * (q[i3] * q[i16]) + (1.133573e-03) * (q[i3] * q[i19])
            + (4.211431e-03) * (q[i3] * q[i20]) + (2.645970e-03) * (q[i3] * q[i21]) + (-1.204513e-03) * (q[i3] * q[i22]) + (7.010348e-03) * (q[i4] * q[i5])
            + (1.063625e-03) * (q[i4] * q[i6]) + (-2.108197e-03) * (q[i4] * q[i8]) + (3.591907e-04) * (q[i4] * q[i9]) + (-1.167074e-03) * (q[i4] * q[i10])
            + (-4.589427e-04) * (q[i4] * q[i11]) + (-6.756111e-04) * (q[i4] * q[i12]) + (3.312536e-03) * (q[i4] * q[i15]) + (3.367054e-03) * (q[i4] * q[i16])
            + (-9.830795e-04) * (q[i4] * q[i19]) + (-1.690906e-03) * (q[i4] * q[i20]) + (-9.733502e-04) * (q[i4] * q[i21]) + (2.156502e-03) * (q[i4] * q[i22])
            + (-5.513636e-03) * (q[i5] * q[i6]) + (-3.849809e-04) * (q[i5] * q[i8]) + (3.793233e-05) * (q[i5] * q[i9]) + (-2.653394e-03) * (q[i5] * q[i10])
            + (2.087253e-03) * (q[i5] * q[i11]) + (-2.436332e-04) * (q[i5] * q[i12]) + (-1.246730e-03) * (q[i5] * q[i15]) + (-3.479245e-03) * (q[i5] * q[i16])
            + (-2.770813e-03) * (q[i5] * q[i19]) + (3.707763e-03) * (q[i5] * q[i20]) + (-7.583026e-04) * (q[i5] * q[i21]) + (-2.373746e-03) * (q[i5] * q[i22])
            + (4.983881e-07) * (q[i6] * q[i8]) + (3.256982e-03) * (q[i6] * q[i9]) + (-3.232325e-03) * (q[i6] * q[i10]) + (9.011420e-04) * (q[i6] * q[i11])
            + (8.889800e-04) * (q[i6] * q[i12]) + (-4.010950e-03) * (q[i6] * q[i15]) + (4.011652e-03) * (q[i6] * q[i16]) + (3.888448e-03) * (q[i6] * q[i19])
            + (-3.886024e-03) * (q[i6] * q[i20]) + (1.910748e-03) * (q[i6] * q[i21]) + (1.899129e-03) * (q[i6] * q[i22]) + (-8.157522e-04) * (q[i8] * q[i9])
            + (-6.236508e-04) * (q[i8] * q[i10]) + (4.908779e-04) * (q[i8] * q[i11]) + (1.854941e-03) * (q[i8] * q[i12]) + (3.240030e-03) * (q[i8] * q[i15])
            + (-8.830871e-04) * (q[i8] * q[i16]) + (-2.249042e-03) * (q[i8] * q[i19]) + (7.715338e-04) * (q[i8] * q[i20]) + (5.454105e-05) * (q[i8] * q[i21])
            + (-2.572290e-03) * (q[i8] * q[i22]) + (-3.614415e-05) * (q[i9] * q[i10]) + (-2.056986e-03) * (q[i9] * q[i11]) + (6.258440e-04) * (q[i9] * q[i12])
            + (-3.932300e-04) * (q[i9] * q[i15]) + (8.676878e-04) * (q[i9] * q[i16]) + (-9.565985e-05) * (q[i9] * q[i19]) + (-1.426565e-03) * (q[i9] * q[i20])
            + (-1.815872e-04) * (q[i9] * q[i21]) + (7.214727e-04) * (q[i9] * q[i22]) + (-1.299163e-03) * (q[i10] * q[i11]) + (5.004541e-04) * (q[i10] * q[i12])
            + (5.882058e-04) * (q[i10] * q[i15]) + (-8.908161e-04) * (q[i10] * q[i16]) + (-1.729129e-03) * (q[i10] * q[i19])
            + (2.999787e-04) * (q[i10] * q[i20]) + (-8.749950e-04) * (q[i10] * q[i21]) + (6.418850e-05) * (q[i10] * q[i22]) + (3.371243e-03) * (q[i11] * q[i12])
            + (6.059683e-04) * (q[i11] * q[i15]) + (-1.812968e-03) * (q[i11] * q[i16]) + (-1.546261e-03) * (q[i11] * q[i19])
            + (4.480873e-05) * (q[i11] * q[i20]) + (-3.867935e-04) * (q[i11] * q[i21]) + (7.851733e-04) * (q[i11] * q[i22]) + (1.800135e-03) * (q[i12] * q[i15])
            + (2.377230e-04) * (q[i12] * q[i16]) + (-8.816813e-04) * (q[i12] * q[i19]) + (1.192935e-03) * (q[i12] * q[i20])
            + (-1.011054e-03) * (q[i12] * q[i21]) + (-1.816761e-03) * (q[i12] * q[i22]) + (-2.521683e-04) * (q[i15] * q[i16])
            + (2.188180e-04) * (q[i15] * q[i19]) + (3.129368e-04) * (q[i15] * q[i20]) + (-3.455207e-04) * (q[i15] * q[i21]) + (4.722414e-04) * (q[i15] * q[i22])
            + (-1.106108e-03) * (q[i16] * q[i19]) + (1.191575e-03) * (q[i16] * q[i20]) + (1.154882e-03) * (q[i16] * q[i21])
            + (-1.324447e-03) * (q[i16] * q[i22]) + (-2.697755e-03) * (q[i19] * q[i20]) + (-2.370805e-03) * (q[i19] * q[i21])
            + (-2.713702e-04) * (q[i19] * q[i22]) + (7.750717e-04) * (q[i20] * q[i21]) + (-4.975113e-04) * (q[i20] * q[i22])
            + (4.933090e-04) * (q[i21] * q[i22]);
   }

   public void getJQz8(double[] q, double[][] JQ)
   {
      JQ[3][i8] = (-1.984033e-04) * (1) + (8.717667e-05) * ((2) * q[i8]) + (8.124199e-03) * (q[i0]) + (8.120267e-03) * (q[i1]) + (9.419529e-03) * (q[i2])
            + (-4.288859e-03) * (q[i3]) + (-4.240503e-03) * (q[i4]) + (4.538288e-02) * (q[i5]) + (-8.516581e-03) * (q[i6]) + (8.428740e-03) * (q[i7])
            + (-1.762728e-03) * (q[i9]) + (1.709825e-03) * (q[i10]) + (-2.119828e-03) * (q[i11]) + (-2.327962e-03) * (q[i12]) + (1.183765e-02) * (q[i15])
            + (-1.188977e-02) * (q[i16]) + (3.065881e-03) * (q[i19]) + (-3.045078e-03) * (q[i20]) + (1.498093e-03) * (q[i21]) + (1.520628e-03) * (q[i22])
            + (-3.690608e-04) * (q[i0] * q[i0]) + (3.509609e-04) * (q[i1] * q[i1]) + (-4.182231e-06) * (q[i2] * q[i2]) + (3.506601e-03) * (q[i3] * q[i3])
            + (-3.510551e-03) * (q[i4] * q[i4]) + (6.752906e-05) * (q[i5] * q[i5]) + (3.682597e-03) * (q[i6] * q[i6]) + (-3.694758e-03) * (q[i7] * q[i7])
            + (2.224777e-03) * ((2) * q[i0] * q[i8]) + (2.240031e-03) * ((2) * q[i1] * q[i8]) + (4.451997e-03) * ((2) * q[i2] * q[i8])
            + (-2.793614e-03) * ((2) * q[i3] * q[i8]) + (-2.809552e-03) * ((2) * q[i4] * q[i8]) + (2.052357e-03) * ((2) * q[i5] * q[i8])
            + (-1.045167e-03) * ((2) * q[i6] * q[i8]) + (1.055293e-03) * ((2) * q[i7] * q[i8]) + (-7.670018e-06) * ((3) * q[i8] * q[i8])
            + (-6.465699e-04) * ((2) * q[i8] * q[i9]) + (6.442480e-04) * ((2) * q[i8] * q[i10]) + (-3.818482e-03) * ((2) * q[i8] * q[i11])
            + (-3.917756e-03) * ((2) * q[i8] * q[i12]) + (-3.006871e-03) * ((2) * q[i8] * q[i15]) + (3.063677e-03) * ((2) * q[i8] * q[i16])
            + (1.206806e-04) * ((2) * q[i8] * q[i19]) + (-1.346143e-04) * ((2) * q[i8] * q[i20]) + (1.297573e-03) * ((2) * q[i8] * q[i21])
            + (1.305954e-03) * ((2) * q[i8] * q[i22]) + (-3.126090e-04) * (q[i9] * q[i9]) + (3.205144e-04) * (q[i10] * q[i10])
            + (-3.244212e-03) * (q[i11] * q[i11]) + (3.310083e-03) * (q[i12] * q[i12]) + (-2.453704e-03) * (q[i15] * q[i15])
            + (2.480365e-03) * (q[i16] * q[i16]) + (6.152050e-04) * (q[i19] * q[i19]) + (-5.973724e-04) * (q[i20] * q[i20]) + (2.904282e-04) * (q[i21] * q[i21])
            + (-2.898016e-04) * (q[i22] * q[i22]) + (3.772037e-05) * (q[i0] * q[i1]) + (2.392972e-03) * (q[i0] * q[i2]) + (3.691852e-03) * (q[i0] * q[i3])
            + (1.921058e-03) * (q[i0] * q[i4]) + (4.131932e-03) * (q[i0] * q[i5]) + (2.018270e-03) * (q[i0] * q[i6]) + (-5.674026e-04) * (q[i0] * q[i7])
            + (4.403004e-04) * (q[i0] * q[i9]) + (-2.309188e-03) * (q[i0] * q[i10]) + (-9.840143e-04) * (q[i0] * q[i11]) + (-1.802440e-05) * (q[i0] * q[i12])
            + (-1.237752e-03) * (q[i0] * q[i15]) + (3.349766e-05) * (q[i0] * q[i16]) + (1.515999e-03) * (q[i0] * q[i19]) + (-1.020506e-03) * (q[i0] * q[i20])
            + (6.531737e-04) * (q[i0] * q[i21]) + (4.179213e-04) * (q[i0] * q[i22]) + (-2.397383e-03) * (q[i1] * q[i2]) + (-1.897335e-03) * (q[i1] * q[i3])
            + (-3.676329e-03) * (q[i1] * q[i4]) + (-4.131924e-03) * (q[i1] * q[i5]) + (-5.539103e-04) * (q[i1] * q[i6]) + (1.990251e-03) * (q[i1] * q[i7])
            + (-2.300075e-03) * (q[i1] * q[i9]) + (4.317355e-04) * (q[i1] * q[i10]) + (3.076759e-05) * (q[i1] * q[i11]) + (9.556149e-04) * (q[i1] * q[i12])
            + (4.747636e-05) * (q[i1] * q[i15]) + (-1.247925e-03) * (q[i1] * q[i16]) + (-1.008339e-03) * (q[i1] * q[i19]) + (1.523498e-03) * (q[i1] * q[i20])
            + (-4.188210e-04) * (q[i1] * q[i21]) + (-6.807220e-04) * (q[i1] * q[i22]) + (-1.970023e-04) * (q[i2] * q[i3]) + (2.195300e-04) * (q[i2] * q[i4])
            + (-1.664079e-05) * (q[i2] * q[i5]) + (-7.272094e-03) * (q[i2] * q[i6]) + (-7.247755e-03) * (q[i2] * q[i7]) + (-2.026978e-03) * (q[i2] * q[i9])
            + (-2.007572e-03) * (q[i2] * q[i10]) + (7.690454e-05) * (q[i2] * q[i11]) + (-1.277173e-04) * (q[i2] * q[i12]) + (-1.013031e-03) * (q[i2] * q[i15])
            + (-1.017945e-03) * (q[i2] * q[i16]) + (-1.213964e-03) * (q[i2] * q[i19]) + (-1.202537e-03) * (q[i2] * q[i20]) + (1.080224e-03) * (q[i2] * q[i21])
            + (-1.089511e-03) * (q[i2] * q[i22]) + (-1.122263e-05) * (q[i3] * q[i4]) + (-1.497916e-02) * (q[i3] * q[i5]) + (-2.132648e-03) * (q[i3] * q[i6])
            + (1.143908e-03) * (q[i3] * q[i7]) + (1.282765e-03) * (q[i3] * q[i9]) + (2.358254e-05) * (q[i3] * q[i10]) + (7.829275e-04) * (q[i3] * q[i11])
            + (1.143059e-03) * (q[i3] * q[i12]) + (-2.119369e-03) * (q[i3] * q[i15]) + (4.871352e-04) * (q[i3] * q[i16]) + (3.770535e-03) * (q[i3] * q[i19])
            + (-2.285660e-03) * (q[i3] * q[i20]) + (6.368776e-04) * (q[i3] * q[i21]) + (1.847937e-03) * (q[i3] * q[i22]) + (1.495923e-02) * (q[i4] * q[i5])
            + (1.137326e-03) * (q[i4] * q[i6]) + (-2.108197e-03) * (q[i4] * q[i7]) + (1.912230e-05) * (q[i4] * q[i9]) + (1.274215e-03) * (q[i4] * q[i10])
            + (-1.178815e-03) * (q[i4] * q[i11]) + (-8.375840e-04) * (q[i4] * q[i12]) + (4.996041e-04) * (q[i4] * q[i15]) + (-2.130278e-03) * (q[i4] * q[i16])
            + (-2.264376e-03) * (q[i4] * q[i19]) + (3.739202e-03) * (q[i4] * q[i20]) + (-1.838598e-03) * (q[i4] * q[i21]) + (-6.206917e-04) * (q[i4] * q[i22])
            + (-4.061282e-04) * (q[i5] * q[i6]) + (-3.849809e-04) * (q[i5] * q[i7]) + (2.207665e-03) * (q[i5] * q[i9]) + (2.197406e-03) * (q[i5] * q[i10])
            + (-3.876869e-03) * (q[i5] * q[i11]) + (4.035952e-03) * (q[i5] * q[i12]) + (-2.887911e-03) * (q[i5] * q[i15]) + (-2.914658e-03) * (q[i5] * q[i16])
            + (1.483302e-05) * (q[i5] * q[i19]) + (5.371657e-05) * (q[i5] * q[i20]) + (7.720520e-05) * (q[i5] * q[i21]) + (-1.235063e-04) * (q[i5] * q[i22])
            + (4.983881e-07) * (q[i6] * q[i7]) + (6.320989e-04) * (q[i6] * q[i9]) + (8.281645e-04) * (q[i6] * q[i10]) + (1.858958e-03) * (q[i6] * q[i11])
            + (5.226429e-04) * (q[i6] * q[i12]) + (8.525784e-04) * (q[i6] * q[i15]) + (-3.242813e-03) * (q[i6] * q[i16]) + (-7.519003e-04) * (q[i6] * q[i19])
            + (2.245990e-03) * (q[i6] * q[i20]) + (-2.577027e-03) * (q[i6] * q[i21]) + (5.465064e-05) * (q[i6] * q[i22]) + (-8.157522e-04) * (q[i7] * q[i9])
            + (-6.236508e-04) * (q[i7] * q[i10]) + (4.908779e-04) * (q[i7] * q[i11]) + (1.854941e-03) * (q[i7] * q[i12]) + (3.240030e-03) * (q[i7] * q[i15])
            + (-8.830871e-04) * (q[i7] * q[i16]) + (-2.249042e-03) * (q[i7] * q[i19]) + (7.715338e-04) * (q[i7] * q[i20]) + (5.454105e-05) * (q[i7] * q[i21])
            + (-2.572290e-03) * (q[i7] * q[i22]) + (4.158587e-06) * (q[i9] * q[i10]) + (8.326286e-04) * (q[i9] * q[i11]) + (-4.350046e-04) * (q[i9] * q[i12])
            + (-1.185753e-03) * (q[i9] * q[i15]) + (-2.455940e-03) * (q[i9] * q[i16]) + (-2.689159e-05) * (q[i9] * q[i19]) + (1.587067e-03) * (q[i9] * q[i20])
            + (-7.147197e-04) * (q[i9] * q[i21]) + (2.601415e-04) * (q[i9] * q[i22]) + (-4.307190e-04) * (q[i10] * q[i11]) + (8.374422e-04) * (q[i10] * q[i12])
            + (2.442853e-03) * (q[i10] * q[i15]) + (1.178279e-03) * (q[i10] * q[i16]) + (-1.579250e-03) * (q[i10] * q[i19]) + (3.105124e-05) * (q[i10] * q[i20])
            + (2.566545e-04) * (q[i10] * q[i21]) + (-7.149864e-04) * (q[i10] * q[i22]) + (9.045604e-07) * (q[i11] * q[i12]) + (2.710908e-03) * (q[i11] * q[i15])
            + (-2.294426e-04) * (q[i11] * q[i16]) + (-1.246248e-03) * (q[i11] * q[i19]) + (5.569696e-04) * (q[i11] * q[i20])
            + (-6.084515e-04) * (q[i11] * q[i21]) + (-1.323098e-03) * (q[i11] * q[i22]) + (-2.242985e-04) * (q[i12] * q[i15])
            + (2.704199e-03) * (q[i12] * q[i16]) + (5.618584e-04) * (q[i12] * q[i19]) + (-1.202175e-03) * (q[i12] * q[i20]) + (1.327864e-03) * (q[i12] * q[i21])
            + (6.132567e-04) * (q[i12] * q[i22]) + (3.408042e-06) * (q[i15] * q[i16]) + (3.239668e-04) * (q[i15] * q[i19]) + (-3.857058e-04) * (q[i15] * q[i20])
            + (2.597813e-03) * (q[i15] * q[i21]) + (-5.794559e-05) * (q[i15] * q[i22]) + (3.936908e-04) * (q[i16] * q[i19])
            + (-3.308385e-04) * (q[i16] * q[i20]) + (-5.266935e-05) * (q[i16] * q[i21]) + (2.635654e-03) * (q[i16] * q[i22])
            + (-4.539242e-07) * (q[i19] * q[i20]) + (2.748206e-03) * (q[i19] * q[i21]) + (-7.039320e-04) * (q[i19] * q[i22])
            + (-7.043087e-04) * (q[i20] * q[i21]) + (2.734584e-03) * (q[i20] * q[i22]) + (1.592038e-06) * (q[i21] * q[i22]);
   }

   public void getJQz9(double[] q, double[][] JQ)
   {
      JQ[3][i9] = (3.090555e-02) * (1) + (-2.102622e-03) * ((2) * q[i9]) + (-2.182358e-03) * (q[i0]) + (-5.844495e-04) * (q[i1]) + (5.744760e-05) * (q[i2])
            + (1.400853e-03) * (q[i3]) + (1.108682e-02) * (q[i4]) + (8.665537e-03) * (q[i5]) + (-2.719946e-03) * (q[i6]) + (-3.306397e-03) * (q[i7])
            + (-1.762728e-03) * (q[i8]) + (4.878573e-06) * (q[i10]) + (2.194359e-04) * (q[i11]) + (1.848397e-03) * (q[i12]) + (-3.109319e-03) * (q[i15])
            + (1.225156e-03) * (q[i16]) + (-1.853988e-03) * (q[i19]) + (-1.021110e-03) * (q[i20]) + (7.322895e-04) * (q[i21]) + (1.643148e-03) * (q[i22])
            + (-1.714045e-03) * (q[i0] * q[i0]) + (1.584619e-03) * (q[i1] * q[i1]) + (-5.289894e-04) * (q[i2] * q[i2]) + (1.159972e-03) * (q[i3] * q[i3])
            + (-1.080638e-04) * (q[i4] * q[i4]) + (-7.996915e-04) * (q[i5] * q[i5]) + (-2.824449e-04) * (q[i6] * q[i6]) + (-2.250312e-03) * (q[i7] * q[i7])
            + (-6.465699e-04) * (q[i8] * q[i8]) + (3.295889e-03) * ((2) * q[i0] * q[i9]) + (3.194498e-04) * ((2) * q[i1] * q[i9])
            + (1.033256e-03) * ((2) * q[i2] * q[i9]) + (7.747477e-04) * ((2) * q[i3] * q[i9]) + (-1.002610e-03) * ((2) * q[i4] * q[i9])
            + (-1.284342e-03) * ((2) * q[i5] * q[i9]) + (-1.925311e-03) * ((2) * q[i6] * q[i9]) + (2.180055e-04) * ((2) * q[i7] * q[i9])
            + (-3.126090e-04) * ((2) * q[i8] * q[i9]) + (-9.753753e-04) * ((3) * q[i9] * q[i9]) + (-1.636086e-04) * ((2) * q[i9] * q[i10])
            + (4.127428e-04) * ((2) * q[i9] * q[i11]) + (6.475281e-05) * ((2) * q[i9] * q[i12]) + (2.367505e-04) * ((2) * q[i9] * q[i15])
            + (-4.382164e-04) * ((2) * q[i9] * q[i16]) + (-6.611220e-05) * ((2) * q[i9] * q[i19]) + (6.369948e-04) * ((2) * q[i9] * q[i20])
            + (6.274636e-05) * ((2) * q[i9] * q[i21]) + (-2.526185e-04) * ((2) * q[i9] * q[i22]) + (1.638362e-04) * (q[i10] * q[i10])
            + (1.587662e-04) * (q[i11] * q[i11]) + (3.902955e-04) * (q[i12] * q[i12]) + (-7.700225e-05) * (q[i15] * q[i15])
            + (-3.474733e-04) * (q[i16] * q[i16]) + (3.324393e-04) * (q[i19] * q[i19]) + (-1.292999e-04) * (q[i20] * q[i20])
            + (2.991626e-04) * (q[i21] * q[i21]) + (-7.665134e-05) * (q[i22] * q[i22]) + (-9.410769e-04) * (q[i0] * q[i1]) + (-1.221234e-03) * (q[i0] * q[i2])
            + (4.641062e-03) * (q[i0] * q[i3]) + (4.471828e-03) * (q[i0] * q[i4]) + (1.215783e-03) * (q[i0] * q[i5]) + (-8.093356e-03) * (q[i0] * q[i6])
            + (7.027701e-04) * (q[i0] * q[i7]) + (4.403004e-04) * (q[i0] * q[i8]) + (7.059487e-04) * (q[i0] * q[i10]) + (2.171805e-04) * (q[i0] * q[i11])
            + (8.206057e-04) * (q[i0] * q[i12]) + (-2.962509e-04) * (q[i0] * q[i15]) + (3.266708e-04) * (q[i0] * q[i16]) + (-1.340145e-04) * (q[i0] * q[i19])
            + (4.477149e-04) * (q[i0] * q[i20]) + (-9.313780e-05) * (q[i0] * q[i21]) + (-3.259272e-05) * (q[i0] * q[i22]) + (1.123347e-04) * (q[i1] * q[i2])
            + (-3.666251e-03) * (q[i1] * q[i3]) + (4.109857e-04) * (q[i1] * q[i4]) + (-5.376108e-04) * (q[i1] * q[i5]) + (6.241155e-03) * (q[i1] * q[i6])
            + (4.768331e-03) * (q[i1] * q[i7]) + (-2.300075e-03) * (q[i1] * q[i8]) + (7.159339e-04) * (q[i1] * q[i10]) + (-9.588975e-04) * (q[i1] * q[i11])
            + (-7.617789e-04) * (q[i1] * q[i12]) + (5.426493e-04) * (q[i1] * q[i15]) + (-1.040909e-03) * (q[i1] * q[i16]) + (1.209350e-04) * (q[i1] * q[i19])
            + (-6.194793e-04) * (q[i1] * q[i20]) + (3.461663e-04) * (q[i1] * q[i21]) + (-1.466367e-04) * (q[i1] * q[i22]) + (1.065266e-03) * (q[i2] * q[i3])
            + (2.925550e-03) * (q[i2] * q[i4]) + (1.866223e-03) * (q[i2] * q[i5]) + (-4.764478e-03) * (q[i2] * q[i6]) + (7.925011e-04) * (q[i2] * q[i7])
            + (-2.026978e-03) * (q[i2] * q[i8]) + (4.356090e-04) * (q[i2] * q[i10]) + (-1.279139e-03) * (q[i2] * q[i11]) + (-3.126844e-04) * (q[i2] * q[i12])
            + (1.020416e-03) * (q[i2] * q[i15]) + (-1.131341e-03) * (q[i2] * q[i16]) + (2.814179e-04) * (q[i2] * q[i19]) + (-4.580965e-04) * (q[i2] * q[i20])
            + (-1.082832e-04) * (q[i2] * q[i21]) + (-4.297117e-05) * (q[i2] * q[i22]) + (-8.118305e-04) * (q[i3] * q[i4]) + (-1.775080e-04) * (q[i3] * q[i5])
            + (-1.094750e-03) * (q[i3] * q[i6]) + (-9.034955e-04) * (q[i3] * q[i7]) + (1.282765e-03) * (q[i3] * q[i8]) + (-5.890720e-04) * (q[i3] * q[i10])
            + (-1.284133e-03) * (q[i3] * q[i11]) + (-1.399436e-03) * (q[i3] * q[i12]) + (2.049416e-04) * (q[i3] * q[i15]) + (4.223724e-04) * (q[i3] * q[i16])
            + (1.406502e-03) * (q[i3] * q[i19]) + (-1.023814e-03) * (q[i3] * q[i20]) + (-1.350166e-03) * (q[i3] * q[i21]) + (7.085870e-04) * (q[i3] * q[i22])
            + (-1.137618e-03) * (q[i4] * q[i5]) + (-2.258961e-03) * (q[i4] * q[i6]) + (3.591907e-04) * (q[i4] * q[i7]) + (1.912230e-05) * (q[i4] * q[i8])
            + (-5.872507e-04) * (q[i4] * q[i10]) + (-5.744456e-04) * (q[i4] * q[i11]) + (6.472719e-04) * (q[i4] * q[i12]) + (-1.838502e-03) * (q[i4] * q[i15])
            + (3.111986e-04) * (q[i4] * q[i16]) + (-1.010468e-03) * (q[i4] * q[i19]) + (6.126302e-04) * (q[i4] * q[i20]) + (6.892232e-04) * (q[i4] * q[i21])
            + (6.536468e-05) * (q[i4] * q[i22]) + (-2.733507e-03) * (q[i5] * q[i6]) + (3.793233e-05) * (q[i5] * q[i7]) + (2.207665e-03) * (q[i5] * q[i8])
            + (1.099198e-03) * (q[i5] * q[i10]) + (3.362156e-04) * (q[i5] * q[i11]) + (-1.014929e-04) * (q[i5] * q[i12]) + (4.149904e-04) * (q[i5] * q[i15])
            + (1.719742e-04) * (q[i5] * q[i16]) + (-6.854388e-04) * (q[i5] * q[i19]) + (-5.408115e-04) * (q[i5] * q[i20]) + (9.070192e-04) * (q[i5] * q[i21])
            + (-8.920332e-04) * (q[i5] * q[i22]) + (3.256982e-03) * (q[i6] * q[i7]) + (6.320989e-04) * (q[i6] * q[i8]) + (3.559246e-05) * (q[i6] * q[i10])
            + (4.895574e-04) * (q[i6] * q[i11]) + (-1.295664e-03) * (q[i6] * q[i12]) + (9.019470e-04) * (q[i6] * q[i15]) + (-5.992658e-04) * (q[i6] * q[i16])
            + (-2.982457e-04) * (q[i6] * q[i19]) + (1.733925e-03) * (q[i6] * q[i20]) + (5.865629e-05) * (q[i6] * q[i21]) + (-8.798216e-04) * (q[i6] * q[i22])
            + (-8.157522e-04) * (q[i7] * q[i8]) + (-3.614415e-05) * (q[i7] * q[i10]) + (-2.056986e-03) * (q[i7] * q[i11]) + (6.258440e-04) * (q[i7] * q[i12])
            + (-3.932300e-04) * (q[i7] * q[i15]) + (8.676878e-04) * (q[i7] * q[i16]) + (-9.565985e-05) * (q[i7] * q[i19]) + (-1.426565e-03) * (q[i7] * q[i20])
            + (-1.815872e-04) * (q[i7] * q[i21]) + (7.214727e-04) * (q[i7] * q[i22]) + (4.158587e-06) * (q[i8] * q[i10]) + (8.326286e-04) * (q[i8] * q[i11])
            + (-4.350046e-04) * (q[i8] * q[i12]) + (-1.185753e-03) * (q[i8] * q[i15]) + (-2.455940e-03) * (q[i8] * q[i16]) + (-2.689159e-05) * (q[i8] * q[i19])
            + (1.587067e-03) * (q[i8] * q[i20]) + (-7.147197e-04) * (q[i8] * q[i21]) + (2.601415e-04) * (q[i8] * q[i22]) + (-1.399252e-04) * (q[i10] * q[i11])
            + (-1.486001e-04) * (q[i10] * q[i12]) + (-1.735716e-04) * (q[i10] * q[i15]) + (1.709716e-04) * (q[i10] * q[i16])
            + (2.079621e-04) * (q[i10] * q[i19]) + (-2.078830e-04) * (q[i10] * q[i20]) + (7.075739e-05) * (q[i10] * q[i21]) + (6.639323e-05) * (q[i10] * q[i22])
            + (-6.702575e-04) * (q[i11] * q[i12]) + (4.938244e-04) * (q[i11] * q[i15]) + (2.749411e-04) * (q[i11] * q[i16]) + (1.018745e-04) * (q[i11] * q[i19])
            + (-2.517958e-04) * (q[i11] * q[i20]) + (5.632330e-04) * (q[i11] * q[i21]) + (2.707479e-04) * (q[i11] * q[i22])
            + (-1.756373e-04) * (q[i12] * q[i15]) + (7.380747e-04) * (q[i12] * q[i16]) + (-8.797610e-05) * (q[i12] * q[i19])
            + (-3.981859e-04) * (q[i12] * q[i20]) + (-2.561373e-04) * (q[i12] * q[i21]) + (-2.763025e-04) * (q[i12] * q[i22])
            + (-5.587725e-04) * (q[i15] * q[i16]) + (-6.830786e-05) * (q[i15] * q[i19]) + (3.258650e-04) * (q[i15] * q[i20])
            + (-5.439368e-04) * (q[i15] * q[i21]) + (4.191311e-04) * (q[i15] * q[i22]) + (-2.823206e-04) * (q[i16] * q[i19])
            + (1.364311e-04) * (q[i16] * q[i20]) + (3.010307e-04) * (q[i16] * q[i21]) + (-6.196279e-05) * (q[i16] * q[i22]) + (7.958606e-04) * (q[i19] * q[i20])
            + (-4.528721e-04) * (q[i19] * q[i21]) + (2.472474e-04) * (q[i19] * q[i22]) + (-5.562106e-05) * (q[i20] * q[i21])
            + (-2.840447e-04) * (q[i20] * q[i22]) + (-1.108698e-04) * (q[i21] * q[i22]);
   }

   public void getJQz10(double[] q, double[][] JQ)
   {
      JQ[3][i10] = (-3.070071e-02) * (1) + (2.094341e-03) * ((2) * q[i10]) + (-5.529312e-04) * (q[i0]) + (-2.087825e-03) * (q[i1]) + (1.044127e-04) * (q[i2])
            + (1.098379e-02) * (q[i3]) + (1.371122e-03) * (q[i4]) + (8.572595e-03) * (q[i5]) + (3.286257e-03) * (q[i6]) + (2.648012e-03) * (q[i7])
            + (1.709825e-03) * (q[i8]) + (4.878573e-06) * (q[i9]) + (1.839012e-03) * (q[i11]) + (2.560436e-04) * (q[i12]) + (-1.183486e-03) * (q[i15])
            + (3.101136e-03) * (q[i16]) + (1.017924e-03) * (q[i19]) + (1.831189e-03) * (q[i20]) + (1.629888e-03) * (q[i21]) + (7.292084e-04) * (q[i22])
            + (-1.578440e-03) * (q[i0] * q[i0]) + (1.710830e-03) * (q[i1] * q[i1]) + (5.168931e-04) * (q[i2] * q[i2]) + (1.046336e-04) * (q[i3] * q[i3])
            + (-1.172863e-03) * (q[i4] * q[i4]) + (7.831560e-04) * (q[i5] * q[i5]) + (2.238024e-03) * (q[i6] * q[i6]) + (2.785022e-04) * (q[i7] * q[i7])
            + (6.442480e-04) * (q[i8] * q[i8]) + (-1.636086e-04) * (q[i9] * q[i9]) + (3.135397e-04) * ((2) * q[i0] * q[i10])
            + (3.248302e-03) * ((2) * q[i1] * q[i10]) + (1.014726e-03) * ((2) * q[i2] * q[i10]) + (-9.855731e-04) * ((2) * q[i3] * q[i10])
            + (7.347479e-04) * ((2) * q[i4] * q[i10]) + (-1.257176e-03) * ((2) * q[i5] * q[i10]) + (-2.083735e-04) * ((2) * q[i6] * q[i10])
            + (1.908832e-03) * ((2) * q[i7] * q[i10]) + (3.205144e-04) * ((2) * q[i8] * q[i10]) + (1.638362e-04) * ((2) * q[i9] * q[i10])
            + (9.624813e-04) * ((3) * q[i10] * q[i10]) + (6.127784e-05) * ((2) * q[i10] * q[i11]) + (4.079915e-04) * ((2) * q[i10] * q[i12])
            + (4.309461e-04) * ((2) * q[i10] * q[i15]) + (-2.335088e-04) * ((2) * q[i10] * q[i16]) + (-6.300817e-04) * ((2) * q[i10] * q[i19])
            + (6.601364e-05) * ((2) * q[i10] * q[i20]) + (-2.492530e-04) * ((2) * q[i10] * q[i21]) + (6.552225e-05) * ((2) * q[i10] * q[i22])
            + (-3.784368e-04) * (q[i11] * q[i11]) + (-1.483041e-04) * (q[i12] * q[i12]) + (3.441780e-04) * (q[i15] * q[i15])
            + (7.707608e-05) * (q[i16] * q[i16]) + (1.273070e-04) * (q[i19] * q[i19]) + (-3.326441e-04) * (q[i20] * q[i20]) + (7.848120e-05) * (q[i21] * q[i21])
            + (-3.000039e-04) * (q[i22] * q[i22]) + (9.334965e-04) * (q[i0] * q[i1]) + (-1.089578e-04) * (q[i0] * q[i2]) + (-4.279734e-04) * (q[i0] * q[i3])
            + (3.653177e-03) * (q[i0] * q[i4]) + (5.609378e-04) * (q[i0] * q[i5]) + (4.731231e-03) * (q[i0] * q[i6]) + (6.223994e-03) * (q[i0] * q[i7])
            + (-2.309188e-03) * (q[i0] * q[i8]) + (7.059487e-04) * (q[i0] * q[i9]) + (7.615470e-04) * (q[i0] * q[i11]) + (9.834727e-04) * (q[i0] * q[i12])
            + (-1.050272e-03) * (q[i0] * q[i15]) + (5.576206e-04) * (q[i0] * q[i16]) + (-6.144819e-04) * (q[i0] * q[i19]) + (1.285622e-04) * (q[i0] * q[i20])
            + (1.549669e-04) * (q[i0] * q[i21]) + (-3.355837e-04) * (q[i0] * q[i22]) + (1.213317e-03) * (q[i1] * q[i2]) + (-4.470188e-03) * (q[i1] * q[i3])
            + (-4.582681e-03) * (q[i1] * q[i4]) + (-1.196318e-03) * (q[i1] * q[i5]) + (7.006525e-04) * (q[i1] * q[i6]) + (-8.073847e-03) * (q[i1] * q[i7])
            + (4.317355e-04) * (q[i1] * q[i8]) + (7.159339e-04) * (q[i1] * q[i9]) + (-8.165858e-04) * (q[i1] * q[i11]) + (-2.143419e-04) * (q[i1] * q[i12])
            + (3.368781e-04) * (q[i1] * q[i15]) + (-2.906111e-04) * (q[i1] * q[i16]) + (4.356600e-04) * (q[i1] * q[i19]) + (-1.314759e-04) * (q[i1] * q[i20])
            + (3.046122e-05) * (q[i1] * q[i21]) + (1.084647e-04) * (q[i1] * q[i22]) + (-2.928779e-03) * (q[i2] * q[i3]) + (-1.013973e-03) * (q[i2] * q[i4])
            + (-1.855923e-03) * (q[i2] * q[i5]) + (7.771415e-04) * (q[i2] * q[i6]) + (-4.733092e-03) * (q[i2] * q[i7]) + (-2.007572e-03) * (q[i2] * q[i8])
            + (4.356090e-04) * (q[i2] * q[i9]) + (3.151909e-04) * (q[i2] * q[i11]) + (1.280565e-03) * (q[i2] * q[i12]) + (-1.123029e-03) * (q[i2] * q[i15])
            + (1.026945e-03) * (q[i2] * q[i16]) + (-4.671196e-04) * (q[i2] * q[i19]) + (2.916659e-04) * (q[i2] * q[i20]) + (4.605087e-05) * (q[i2] * q[i21])
            + (1.157773e-04) * (q[i2] * q[i22]) + (8.529342e-04) * (q[i3] * q[i4]) + (1.140135e-03) * (q[i3] * q[i5]) + (3.304197e-04) * (q[i3] * q[i6])
            + (-2.223610e-03) * (q[i3] * q[i7]) + (2.358254e-05) * (q[i3] * q[i8]) + (-5.890720e-04) * (q[i3] * q[i9]) + (-6.521371e-04) * (q[i3] * q[i11])
            + (5.601645e-04) * (q[i3] * q[i12]) + (3.035723e-04) * (q[i3] * q[i15]) + (-1.849129e-03) * (q[i3] * q[i16]) + (6.035180e-04) * (q[i3] * q[i19])
            + (-1.015407e-03) * (q[i3] * q[i20]) + (-6.701134e-05) * (q[i3] * q[i21]) + (-6.866560e-04) * (q[i3] * q[i22]) + (1.745041e-04) * (q[i4] * q[i5])
            + (-8.680651e-04) * (q[i4] * q[i6]) + (-1.167074e-03) * (q[i4] * q[i7]) + (1.274215e-03) * (q[i4] * q[i8]) + (-5.872507e-04) * (q[i4] * q[i9])
            + (1.395561e-03) * (q[i4] * q[i11]) + (1.288199e-03) * (q[i4] * q[i12]) + (4.064886e-04) * (q[i4] * q[i15]) + (2.140959e-04) * (q[i4] * q[i16])
            + (-1.031287e-03) * (q[i4] * q[i19]) + (1.394813e-03) * (q[i4] * q[i20]) + (-7.013622e-04) * (q[i4] * q[i21]) + (1.339857e-03) * (q[i4] * q[i22])
            + (2.534805e-05) * (q[i5] * q[i6]) + (-2.653394e-03) * (q[i5] * q[i7]) + (2.197406e-03) * (q[i5] * q[i8]) + (1.099198e-03) * (q[i5] * q[i9])
            + (1.083697e-04) * (q[i5] * q[i11]) + (-3.021511e-04) * (q[i5] * q[i12]) + (1.927727e-04) * (q[i5] * q[i15]) + (4.171139e-04) * (q[i5] * q[i16])
            + (-5.324809e-04) * (q[i5] * q[i19]) + (-6.700685e-04) * (q[i5] * q[i20]) + (8.840609e-04) * (q[i5] * q[i21]) + (-9.083725e-04) * (q[i5] * q[i22])
            + (-3.232325e-03) * (q[i6] * q[i7]) + (8.281645e-04) * (q[i6] * q[i8]) + (3.559246e-05) * (q[i6] * q[i9]) + (6.311207e-04) * (q[i6] * q[i11])
            + (-2.058049e-03) * (q[i6] * q[i12]) + (-8.672907e-04) * (q[i6] * q[i15]) + (3.818487e-04) * (q[i6] * q[i16]) + (1.426584e-03) * (q[i6] * q[i19])
            + (9.999129e-05) * (q[i6] * q[i20]) + (7.224236e-04) * (q[i6] * q[i21]) + (-1.862710e-04) * (q[i6] * q[i22]) + (-6.236508e-04) * (q[i7] * q[i8])
            + (-3.614415e-05) * (q[i7] * q[i9]) + (-1.299163e-03) * (q[i7] * q[i11]) + (5.004541e-04) * (q[i7] * q[i12]) + (5.882058e-04) * (q[i7] * q[i15])
            + (-8.908161e-04) * (q[i7] * q[i16]) + (-1.729129e-03) * (q[i7] * q[i19]) + (2.999787e-04) * (q[i7] * q[i20]) + (-8.749950e-04) * (q[i7] * q[i21])
            + (6.418850e-05) * (q[i7] * q[i22]) + (4.158587e-06) * (q[i8] * q[i9]) + (-4.307190e-04) * (q[i8] * q[i11]) + (8.374422e-04) * (q[i8] * q[i12])
            + (2.442853e-03) * (q[i8] * q[i15]) + (1.178279e-03) * (q[i8] * q[i16]) + (-1.579250e-03) * (q[i8] * q[i19]) + (3.105124e-05) * (q[i8] * q[i20])
            + (2.566545e-04) * (q[i8] * q[i21]) + (-7.149864e-04) * (q[i8] * q[i22]) + (-1.399252e-04) * (q[i9] * q[i11]) + (-1.486001e-04) * (q[i9] * q[i12])
            + (-1.735716e-04) * (q[i9] * q[i15]) + (1.709716e-04) * (q[i9] * q[i16]) + (2.079621e-04) * (q[i9] * q[i19]) + (-2.078830e-04) * (q[i9] * q[i20])
            + (7.075739e-05) * (q[i9] * q[i21]) + (6.639323e-05) * (q[i9] * q[i22]) + (6.578010e-04) * (q[i11] * q[i12]) + (7.359768e-04) * (q[i11] * q[i15])
            + (-1.702980e-04) * (q[i11] * q[i16]) + (-3.876385e-04) * (q[i11] * q[i19]) + (-7.496709e-05) * (q[i11] * q[i20])
            + (2.774399e-04) * (q[i11] * q[i21]) + (2.573533e-04) * (q[i11] * q[i22]) + (2.738951e-04) * (q[i12] * q[i15]) + (4.983328e-04) * (q[i12] * q[i16])
            + (-2.560389e-04) * (q[i12] * q[i19]) + (1.014790e-04) * (q[i12] * q[i20]) + (-2.674550e-04) * (q[i12] * q[i21])
            + (-5.623680e-04) * (q[i12] * q[i22]) + (5.650456e-04) * (q[i15] * q[i16]) + (-1.323603e-04) * (q[i15] * q[i19])
            + (2.752441e-04) * (q[i15] * q[i20]) + (-5.644669e-05) * (q[i15] * q[i21]) + (3.015409e-04) * (q[i15] * q[i22])
            + (-3.197884e-04) * (q[i16] * q[i19]) + (6.513024e-05) * (q[i16] * q[i20]) + (4.160347e-04) * (q[i16] * q[i21])
            + (-5.392491e-04) * (q[i16] * q[i22]) + (-7.909997e-04) * (q[i19] * q[i20]) + (-2.803785e-04) * (q[i19] * q[i21])
            + (-5.321534e-05) * (q[i19] * q[i22]) + (2.432303e-04) * (q[i20] * q[i21]) + (-4.450342e-04) * (q[i20] * q[i22])
            + (1.143337e-04) * (q[i21] * q[i22]);
   }

   public void getJQz11(double[] q, double[][] JQ)
   {
      JQ[3][i11] = (4.625223e-02) * (1) + (-6.034145e-04) * ((2) * q[i11]) + (-7.457680e-04) * (q[i0]) + (4.029661e-03) * (q[i1]) + (3.681198e-03) * (q[i2])
            + (4.454410e-03) * (q[i3]) + (-1.862121e-03) * (q[i4]) + (-4.070402e-03) * (q[i5]) + (7.732070e-04) * (q[i6]) + (4.191025e-03) * (q[i7])
            + (-2.119828e-03) * (q[i8]) + (2.194359e-04) * (q[i9]) + (1.839012e-03) * (q[i10]) + (-3.334952e-05) * (q[i12]) + (7.124364e-04) * (q[i15])
            + (1.153869e-03) * (q[i16]) + (3.057393e-03) * (q[i19]) + (3.689554e-04) * (q[i20]) + (5.455647e-03) * (q[i21]) + (9.597047e-04) * (q[i22])
            + (-1.942903e-04) * (q[i0] * q[i0]) + (1.852546e-03) * (q[i1] * q[i1]) + (-7.185042e-04) * (q[i2] * q[i2]) + (8.607750e-05) * (q[i3] * q[i3])
            + (-1.091657e-03) * (q[i4] * q[i4]) + (1.764104e-03) * (q[i5] * q[i5]) + (-1.910259e-03) * (q[i6] * q[i6]) + (-8.050146e-04) * (q[i7] * q[i7])
            + (-3.818482e-03) * (q[i8] * q[i8]) + (4.127428e-04) * (q[i9] * q[i9]) + (6.127784e-05) * (q[i10] * q[i10])
            + (-2.853861e-04) * ((2) * q[i0] * q[i11]) + (-1.800565e-04) * ((2) * q[i1] * q[i11]) + (-8.427844e-04) * ((2) * q[i2] * q[i11])
            + (-9.812237e-04) * ((2) * q[i3] * q[i11]) + (-9.955130e-04) * ((2) * q[i4] * q[i11]) + (-5.658690e-05) * ((2) * q[i5] * q[i11])
            + (1.614308e-03) * ((2) * q[i6] * q[i11]) + (-1.541249e-03) * ((2) * q[i7] * q[i11]) + (-3.244212e-03) * ((2) * q[i8] * q[i11])
            + (1.587662e-04) * ((2) * q[i9] * q[i11]) + (-3.784368e-04) * ((2) * q[i10] * q[i11]) + (-9.349521e-04) * ((3) * q[i11] * q[i11])
            + (1.293442e-04) * ((2) * q[i11] * q[i12]) + (1.423557e-03) * ((2) * q[i11] * q[i15]) + (-2.826463e-04) * ((2) * q[i11] * q[i16])
            + (-1.238077e-03) * ((2) * q[i11] * q[i19]) + (2.976851e-04) * ((2) * q[i11] * q[i20]) + (-8.182068e-04) * ((2) * q[i11] * q[i21])
            + (1.101684e-04) * ((2) * q[i11] * q[i22]) + (1.353468e-04) * (q[i12] * q[i12]) + (-5.886223e-03) * (q[i15] * q[i15])
            + (2.513389e-04) * (q[i16] * q[i16]) + (7.898908e-04) * (q[i19] * q[i19]) + (-2.012535e-04) * (q[i20] * q[i20])
            + (-3.634334e-04) * (q[i21] * q[i21]) + (-1.354532e-04) * (q[i22] * q[i22]) + (-8.990105e-04) * (q[i0] * q[i1]) + (9.579792e-04) * (q[i0] * q[i2])
            + (-1.397372e-03) * (q[i0] * q[i3]) + (-7.361672e-05) * (q[i0] * q[i4]) + (1.385829e-03) * (q[i0] * q[i5]) + (1.256146e-03) * (q[i0] * q[i6])
            + (6.106089e-05) * (q[i0] * q[i7]) + (-9.840143e-04) * (q[i0] * q[i8]) + (2.171805e-04) * (q[i0] * q[i9]) + (7.615470e-04) * (q[i0] * q[i10])
            + (2.378756e-05) * (q[i0] * q[i12]) + (2.347744e-04) * (q[i0] * q[i15]) + (-2.017318e-04) * (q[i0] * q[i16]) + (6.241591e-04) * (q[i0] * q[i19])
            + (-9.831590e-04) * (q[i0] * q[i20]) + (2.935356e-04) * (q[i0] * q[i21]) + (-2.104644e-04) * (q[i0] * q[i22]) + (-7.466897e-04) * (q[i1] * q[i2])
            + (5.982896e-04) * (q[i1] * q[i3]) + (-1.219388e-03) * (q[i1] * q[i4]) + (3.883544e-04) * (q[i1] * q[i5]) + (1.968200e-03) * (q[i1] * q[i6])
            + (-3.302676e-03) * (q[i1] * q[i7]) + (3.076759e-05) * (q[i1] * q[i8]) + (-9.588975e-04) * (q[i1] * q[i9]) + (-8.165858e-04) * (q[i1] * q[i10])
            + (1.206100e-05) * (q[i1] * q[i12]) + (2.540007e-03) * (q[i1] * q[i15]) + (-9.214786e-04) * (q[i1] * q[i16]) + (-9.333654e-04) * (q[i1] * q[i19])
            + (-1.281242e-03) * (q[i1] * q[i20]) + (-2.952990e-05) * (q[i1] * q[i21]) + (4.311889e-04) * (q[i1] * q[i22]) + (-2.067182e-03) * (q[i2] * q[i3])
            + (5.717113e-04) * (q[i2] * q[i4]) + (-6.183201e-04) * (q[i2] * q[i5]) + (8.556703e-06) * (q[i2] * q[i6]) + (-2.324831e-03) * (q[i2] * q[i7])
            + (7.690454e-05) * (q[i2] * q[i8]) + (-1.279139e-03) * (q[i2] * q[i9]) + (3.151909e-04) * (q[i2] * q[i10]) + (8.537937e-04) * (q[i2] * q[i12])
            + (2.076426e-03) * (q[i2] * q[i15]) + (-5.741165e-04) * (q[i2] * q[i16]) + (-6.186795e-04) * (q[i2] * q[i19]) + (-4.964284e-04) * (q[i2] * q[i20])
            + (-5.254483e-05) * (q[i2] * q[i21]) + (-1.053333e-04) * (q[i2] * q[i22]) + (3.597437e-03) * (q[i3] * q[i4]) + (-9.524564e-04) * (q[i3] * q[i5])
            + (6.582829e-04) * (q[i3] * q[i6]) + (3.926526e-04) * (q[i3] * q[i7]) + (7.829275e-04) * (q[i3] * q[i8]) + (-1.284133e-03) * (q[i3] * q[i9])
            + (-6.521371e-04) * (q[i3] * q[i10]) + (4.141827e-04) * (q[i3] * q[i12]) + (8.568046e-05) * (q[i3] * q[i15]) + (-3.801519e-04) * (q[i3] * q[i16])
            + (3.596755e-04) * (q[i3] * q[i19]) + (-2.743292e-03) * (q[i3] * q[i20]) + (-7.055388e-04) * (q[i3] * q[i21]) + (3.205368e-04) * (q[i3] * q[i22])
            + (-1.941690e-03) * (q[i4] * q[i5]) + (-1.959240e-03) * (q[i4] * q[i6]) + (-4.589427e-04) * (q[i4] * q[i7]) + (-1.178815e-03) * (q[i4] * q[i8])
            + (-5.744456e-04) * (q[i4] * q[i9]) + (1.395561e-03) * (q[i4] * q[i10]) + (4.122995e-04) * (q[i4] * q[i12]) + (9.640143e-04) * (q[i4] * q[i15])
            + (7.220888e-04) * (q[i4] * q[i16]) + (1.144213e-03) * (q[i4] * q[i19]) + (4.028919e-03) * (q[i4] * q[i20]) + (-1.348688e-05) * (q[i4] * q[i21])
            + (-3.507120e-04) * (q[i4] * q[i22]) + (2.757766e-04) * (q[i5] * q[i6]) + (2.087253e-03) * (q[i5] * q[i7]) + (-3.876869e-03) * (q[i5] * q[i8])
            + (3.362156e-04) * (q[i5] * q[i9]) + (1.083697e-04) * (q[i5] * q[i10]) + (-1.144017e-04) * (q[i5] * q[i12]) + (-3.131738e-03) * (q[i5] * q[i15])
            + (8.952596e-04) * (q[i5] * q[i16]) + (-2.765632e-03) * (q[i5] * q[i19]) + (1.098577e-03) * (q[i5] * q[i20]) + (-1.005917e-03) * (q[i5] * q[i21])
            + (4.014403e-04) * (q[i5] * q[i22]) + (9.011420e-04) * (q[i6] * q[i7]) + (1.858958e-03) * (q[i6] * q[i8]) + (4.895574e-04) * (q[i6] * q[i9])
            + (6.311207e-04) * (q[i6] * q[i10]) + (-3.388114e-03) * (q[i6] * q[i12]) + (2.391041e-04) * (q[i6] * q[i15]) + (1.804663e-03) * (q[i6] * q[i16])
            + (1.196569e-03) * (q[i6] * q[i19]) + (-8.816488e-04) * (q[i6] * q[i20]) + (1.821923e-03) * (q[i6] * q[i21]) + (1.017119e-03) * (q[i6] * q[i22])
            + (4.908779e-04) * (q[i7] * q[i8]) + (-2.056986e-03) * (q[i7] * q[i9]) + (-1.299163e-03) * (q[i7] * q[i10]) + (3.371243e-03) * (q[i7] * q[i12])
            + (6.059683e-04) * (q[i7] * q[i15]) + (-1.812968e-03) * (q[i7] * q[i16]) + (-1.546261e-03) * (q[i7] * q[i19]) + (4.480873e-05) * (q[i7] * q[i20])
            + (-3.867935e-04) * (q[i7] * q[i21]) + (7.851733e-04) * (q[i7] * q[i22]) + (8.326286e-04) * (q[i8] * q[i9]) + (-4.307190e-04) * (q[i8] * q[i10])
            + (9.045604e-07) * (q[i8] * q[i12]) + (2.710908e-03) * (q[i8] * q[i15]) + (-2.294426e-04) * (q[i8] * q[i16]) + (-1.246248e-03) * (q[i8] * q[i19])
            + (5.569696e-04) * (q[i8] * q[i20]) + (-6.084515e-04) * (q[i8] * q[i21]) + (-1.323098e-03) * (q[i8] * q[i22]) + (-1.399252e-04) * (q[i9] * q[i10])
            + (-6.702575e-04) * (q[i9] * q[i12]) + (4.938244e-04) * (q[i9] * q[i15]) + (2.749411e-04) * (q[i9] * q[i16]) + (1.018745e-04) * (q[i9] * q[i19])
            + (-2.517958e-04) * (q[i9] * q[i20]) + (5.632330e-04) * (q[i9] * q[i21]) + (2.707479e-04) * (q[i9] * q[i22]) + (6.578010e-04) * (q[i10] * q[i12])
            + (7.359768e-04) * (q[i10] * q[i15]) + (-1.702980e-04) * (q[i10] * q[i16]) + (-3.876385e-04) * (q[i10] * q[i19])
            + (-7.496709e-05) * (q[i10] * q[i20]) + (2.774399e-04) * (q[i10] * q[i21]) + (2.573533e-04) * (q[i10] * q[i22])
            + (-3.694582e-04) * (q[i12] * q[i15]) + (3.527266e-04) * (q[i12] * q[i16]) + (1.785714e-04) * (q[i12] * q[i19])
            + (-1.718031e-04) * (q[i12] * q[i20]) + (-1.055158e-04) * (q[i12] * q[i21]) + (-1.007766e-04) * (q[i12] * q[i22])
            + (-1.547781e-04) * (q[i15] * q[i16]) + (6.189937e-04) * (q[i15] * q[i19]) + (-5.593809e-04) * (q[i15] * q[i20])
            + (1.712960e-03) * (q[i15] * q[i21]) + (2.039076e-04) * (q[i15] * q[i22]) + (1.117545e-04) * (q[i16] * q[i19]) + (-2.979092e-04) * (q[i16] * q[i20])
            + (1.930622e-04) * (q[i16] * q[i21]) + (-6.535191e-05) * (q[i16] * q[i22]) + (2.243249e-04) * (q[i19] * q[i20]) + (1.306839e-03) * (q[i19] * q[i21])
            + (4.637868e-04) * (q[i19] * q[i22]) + (2.009443e-05) * (q[i20] * q[i21]) + (-1.081577e-04) * (q[i20] * q[i22])
            + (1.321018e-04) * (q[i21] * q[i22]);
   }

   public void getJQz12(double[] q, double[][] JQ)
   {
      JQ[3][i12] = (4.647797e-02) * (1) + (8.135552e-04) * ((2) * q[i12]) + (-3.888310e-03) * (q[i0]) + (7.970926e-04) * (q[i1]) + (-3.495712e-03) * (q[i2])
            + (1.894654e-03) * (q[i3]) + (-4.441355e-03) * (q[i4]) + (4.230532e-03) * (q[i5]) + (4.117008e-03) * (q[i6]) + (8.121325e-04) * (q[i7])
            + (-2.327962e-03) * (q[i8]) + (1.848397e-03) * (q[i9]) + (2.560436e-04) * (q[i10]) + (-3.334952e-05) * (q[i11]) + (1.139107e-03) * (q[i15])
            + (6.336331e-04) * (q[i16]) + (3.944362e-04) * (q[i19]) + (3.032000e-03) * (q[i20]) + (-9.732808e-04) * (q[i21]) + (-5.564965e-03) * (q[i22])
            + (1.862733e-03) * (q[i0] * q[i0]) + (-2.041879e-04) * (q[i1] * q[i1]) + (-7.079538e-04) * (q[i2] * q[i2]) + (-1.082842e-03) * (q[i3] * q[i3])
            + (7.709892e-05) * (q[i4] * q[i4]) + (1.870958e-03) * (q[i5] * q[i5]) + (-8.133748e-04) * (q[i6] * q[i6]) + (-1.907086e-03) * (q[i7] * q[i7])
            + (-3.917756e-03) * (q[i8] * q[i8]) + (6.475281e-05) * (q[i9] * q[i9]) + (4.079915e-04) * (q[i10] * q[i10]) + (1.293442e-04) * (q[i11] * q[i11])
            + (-1.852411e-04) * ((2) * q[i0] * q[i12]) + (-3.294122e-04) * ((2) * q[i1] * q[i12]) + (-8.871160e-04) * ((2) * q[i2] * q[i12])
            + (-1.043350e-03) * ((2) * q[i3] * q[i12]) + (-9.536163e-04) * ((2) * q[i4] * q[i12]) + (-3.241012e-05) * ((2) * q[i5] * q[i12])
            + (1.569816e-03) * ((2) * q[i6] * q[i12]) + (-1.602720e-03) * ((2) * q[i7] * q[i12]) + (3.310083e-03) * ((2) * q[i8] * q[i12])
            + (3.902955e-04) * ((2) * q[i9] * q[i12]) + (-1.483041e-04) * ((2) * q[i10] * q[i12]) + (1.353468e-04) * ((2) * q[i11] * q[i12])
            + (-9.692197e-04) * ((3) * q[i12] * q[i12]) + (2.788632e-04) * ((2) * q[i12] * q[i15]) + (-1.424586e-03) * ((2) * q[i12] * q[i16])
            + (-2.953930e-04) * ((2) * q[i12] * q[i19]) + (1.201686e-03) * ((2) * q[i12] * q[i20]) + (1.117641e-04) * ((2) * q[i12] * q[i21])
            + (-8.072142e-04) * ((2) * q[i12] * q[i22]) + (2.535798e-04) * (q[i15] * q[i15]) + (-5.953830e-03) * (q[i16] * q[i16])
            + (-2.068457e-04) * (q[i19] * q[i19]) + (7.762578e-04) * (q[i20] * q[i20]) + (-1.363000e-04) * (q[i21] * q[i21])
            + (-3.473568e-04) * (q[i22] * q[i22]) + (-8.987392e-04) * (q[i0] * q[i1]) + (-7.160882e-04) * (q[i0] * q[i2]) + (-1.202876e-03) * (q[i0] * q[i3])
            + (6.362083e-04) * (q[i0] * q[i4]) + (3.532723e-04) * (q[i0] * q[i5]) + (3.351741e-03) * (q[i0] * q[i6]) + (-1.928167e-03) * (q[i0] * q[i7])
            + (-1.802440e-05) * (q[i0] * q[i8]) + (8.206057e-04) * (q[i0] * q[i9]) + (9.834727e-04) * (q[i0] * q[i10]) + (2.378756e-05) * (q[i0] * q[i11])
            + (9.361919e-04) * (q[i0] * q[i15]) + (-2.527649e-03) * (q[i0] * q[i16]) + (1.297013e-03) * (q[i0] * q[i19]) + (9.315936e-04) * (q[i0] * q[i20])
            + (4.288031e-04) * (q[i0] * q[i21]) + (-3.299089e-05) * (q[i0] * q[i22]) + (9.808607e-04) * (q[i1] * q[i2]) + (-1.060551e-04) * (q[i1] * q[i3])
            + (-1.448425e-03) * (q[i1] * q[i4]) + (1.359287e-03) * (q[i1] * q[i5]) + (-8.547104e-05) * (q[i1] * q[i6]) + (-1.266723e-03) * (q[i1] * q[i7])
            + (9.556149e-04) * (q[i1] * q[i8]) + (-7.617789e-04) * (q[i1] * q[i9]) + (-2.143419e-04) * (q[i1] * q[i10]) + (1.206100e-05) * (q[i1] * q[i11])
            + (2.050431e-04) * (q[i1] * q[i15]) + (-2.460959e-04) * (q[i1] * q[i16]) + (9.758899e-04) * (q[i1] * q[i19]) + (-6.428755e-04) * (q[i1] * q[i20])
            + (-2.075818e-04) * (q[i1] * q[i21]) + (2.939070e-04) * (q[i1] * q[i22]) + (5.248740e-04) * (q[i2] * q[i3]) + (-2.060311e-03) * (q[i2] * q[i4])
            + (-6.954248e-04) * (q[i2] * q[i5]) + (2.338387e-03) * (q[i2] * q[i6]) + (-3.349756e-06) * (q[i2] * q[i7]) + (-1.277173e-04) * (q[i2] * q[i8])
            + (-3.126844e-04) * (q[i2] * q[i9]) + (1.280565e-03) * (q[i2] * q[i10]) + (8.537937e-04) * (q[i2] * q[i11]) + (5.847939e-04) * (q[i2] * q[i15])
            + (-2.091056e-03) * (q[i2] * q[i16]) + (4.856886e-04) * (q[i2] * q[i19]) + (5.954556e-04) * (q[i2] * q[i20]) + (-1.037075e-04) * (q[i2] * q[i21])
            + (-3.184955e-05) * (q[i2] * q[i22]) + (3.627772e-03) * (q[i3] * q[i4]) + (-2.029475e-03) * (q[i3] * q[i5]) + (4.444862e-04) * (q[i3] * q[i6])
            + (1.952225e-03) * (q[i3] * q[i7]) + (1.143059e-03) * (q[i3] * q[i8]) + (-1.399436e-03) * (q[i3] * q[i9]) + (5.601645e-04) * (q[i3] * q[i10])
            + (4.141827e-04) * (q[i3] * q[i11]) + (-7.205732e-04) * (q[i3] * q[i15]) + (-9.703579e-04) * (q[i3] * q[i16]) + (-4.043702e-03) * (q[i3] * q[i19])
            + (-1.148295e-03) * (q[i3] * q[i20]) + (-3.508251e-04) * (q[i3] * q[i21]) + (-1.932807e-05) * (q[i3] * q[i22]) + (-9.799941e-04) * (q[i4] * q[i5])
            + (-4.171843e-04) * (q[i4] * q[i6]) + (-6.756111e-04) * (q[i4] * q[i7]) + (-8.375840e-04) * (q[i4] * q[i8]) + (6.472719e-04) * (q[i4] * q[i9])
            + (1.288199e-03) * (q[i4] * q[i10]) + (4.122995e-04) * (q[i4] * q[i11]) + (3.887333e-04) * (q[i4] * q[i15]) + (-7.257782e-05) * (q[i4] * q[i16])
            + (2.760975e-03) * (q[i4] * q[i19]) + (-3.517347e-04) * (q[i4] * q[i20]) + (3.232380e-04) * (q[i4] * q[i21]) + (-7.104490e-04) * (q[i4] * q[i22])
            + (-2.044780e-03) * (q[i5] * q[i6]) + (-2.436332e-04) * (q[i5] * q[i7]) + (4.035952e-03) * (q[i5] * q[i8]) + (-1.014929e-04) * (q[i5] * q[i9])
            + (-3.021511e-04) * (q[i5] * q[i10]) + (-1.144017e-04) * (q[i5] * q[i11]) + (-9.011617e-04) * (q[i5] * q[i15]) + (3.142238e-03) * (q[i5] * q[i16])
            + (-1.107020e-03) * (q[i5] * q[i19]) + (2.738680e-03) * (q[i5] * q[i20]) + (4.047226e-04) * (q[i5] * q[i21]) + (-9.949296e-04) * (q[i5] * q[i22])
            + (8.889800e-04) * (q[i6] * q[i7]) + (5.226429e-04) * (q[i6] * q[i8]) + (-1.295664e-03) * (q[i6] * q[i9]) + (-2.058049e-03) * (q[i6] * q[i10])
            + (-3.388114e-03) * (q[i6] * q[i11]) + (-1.811602e-03) * (q[i6] * q[i15]) + (6.003288e-04) * (q[i6] * q[i16]) + (4.007865e-05) * (q[i6] * q[i19])
            + (-1.552227e-03) * (q[i6] * q[i20]) + (-7.942234e-04) * (q[i6] * q[i21]) + (3.878863e-04) * (q[i6] * q[i22]) + (1.854941e-03) * (q[i7] * q[i8])
            + (6.258440e-04) * (q[i7] * q[i9]) + (5.004541e-04) * (q[i7] * q[i10]) + (3.371243e-03) * (q[i7] * q[i11]) + (1.800135e-03) * (q[i7] * q[i15])
            + (2.377230e-04) * (q[i7] * q[i16]) + (-8.816813e-04) * (q[i7] * q[i19]) + (1.192935e-03) * (q[i7] * q[i20]) + (-1.011054e-03) * (q[i7] * q[i21])
            + (-1.816761e-03) * (q[i7] * q[i22]) + (-4.350046e-04) * (q[i8] * q[i9]) + (8.374422e-04) * (q[i8] * q[i10]) + (9.045604e-07) * (q[i8] * q[i11])
            + (-2.242985e-04) * (q[i8] * q[i15]) + (2.704199e-03) * (q[i8] * q[i16]) + (5.618584e-04) * (q[i8] * q[i19]) + (-1.202175e-03) * (q[i8] * q[i20])
            + (1.327864e-03) * (q[i8] * q[i21]) + (6.132567e-04) * (q[i8] * q[i22]) + (-1.486001e-04) * (q[i9] * q[i10]) + (-6.702575e-04) * (q[i9] * q[i11])
            + (-1.756373e-04) * (q[i9] * q[i15]) + (7.380747e-04) * (q[i9] * q[i16]) + (-8.797610e-05) * (q[i9] * q[i19]) + (-3.981859e-04) * (q[i9] * q[i20])
            + (-2.561373e-04) * (q[i9] * q[i21]) + (-2.763025e-04) * (q[i9] * q[i22]) + (6.578010e-04) * (q[i10] * q[i11]) + (2.738951e-04) * (q[i10] * q[i15])
            + (4.983328e-04) * (q[i10] * q[i16]) + (-2.560389e-04) * (q[i10] * q[i19]) + (1.014790e-04) * (q[i10] * q[i20])
            + (-2.674550e-04) * (q[i10] * q[i21]) + (-5.623680e-04) * (q[i10] * q[i22]) + (-3.694582e-04) * (q[i11] * q[i15])
            + (3.527266e-04) * (q[i11] * q[i16]) + (1.785714e-04) * (q[i11] * q[i19]) + (-1.718031e-04) * (q[i11] * q[i20])
            + (-1.055158e-04) * (q[i11] * q[i21]) + (-1.007766e-04) * (q[i11] * q[i22]) + (-1.589321e-04) * (q[i15] * q[i16])
            + (-3.018339e-04) * (q[i15] * q[i19]) + (1.066293e-04) * (q[i15] * q[i20]) + (6.021189e-05) * (q[i15] * q[i21])
            + (-2.014812e-04) * (q[i15] * q[i22]) + (-5.592825e-04) * (q[i16] * q[i19]) + (6.358665e-04) * (q[i16] * q[i20])
            + (-2.030599e-04) * (q[i16] * q[i21]) + (-1.759297e-03) * (q[i16] * q[i22]) + (2.304211e-04) * (q[i19] * q[i20])
            + (1.028135e-04) * (q[i19] * q[i21]) + (-1.867936e-05) * (q[i19] * q[i22]) + (-4.651508e-04) * (q[i20] * q[i21])
            + (-1.291617e-03) * (q[i20] * q[i22]) + (1.320225e-04) * (q[i21] * q[i22]);
   }

   public void getJQz15(double[] q, double[][] JQ)
   {
      JQ[3][i15] = (1.086699e-02) * (1) + (3.708246e-03) * ((2) * q[i15]) + (-7.918036e-03) * (q[i0]) + (-9.929541e-03) * (q[i1]) + (-2.036958e-02) * (q[i2])
            + (4.513642e-03) * (q[i3]) + (8.733209e-04) * (q[i4]) + (-3.330178e-03) * (q[i5]) + (-9.829345e-05) * (q[i6]) + (-9.059258e-03) * (q[i7])
            + (1.183765e-02) * (q[i8]) + (-3.109319e-03) * (q[i9]) + (-1.183486e-03) * (q[i10]) + (7.124364e-04) * (q[i11]) + (1.139107e-03) * (q[i12])
            + (9.802463e-07) * (q[i16]) + (5.300136e-03) * (q[i19]) + (-1.283342e-03) * (q[i20]) + (3.836362e-03) * (q[i21]) + (-8.313458e-05) * (q[i22])
            + (1.696342e-04) * (q[i0] * q[i0]) + (1.069579e-03) * (q[i1] * q[i1]) + (-2.672006e-04) * (q[i2] * q[i2]) + (-3.190496e-03) * (q[i3] * q[i3])
            + (1.420788e-03) * (q[i4] * q[i4]) + (2.081274e-03) * (q[i5] * q[i5]) + (1.964660e-03) * (q[i6] * q[i6]) + (1.072717e-03) * (q[i7] * q[i7])
            + (-3.006871e-03) * (q[i8] * q[i8]) + (2.367505e-04) * (q[i9] * q[i9]) + (4.309461e-04) * (q[i10] * q[i10]) + (1.423557e-03) * (q[i11] * q[i11])
            + (2.788632e-04) * (q[i12] * q[i12]) + (-1.907664e-03) * ((2) * q[i0] * q[i15]) + (-2.048487e-03) * ((2) * q[i1] * q[i15])
            + (-4.605599e-03) * ((2) * q[i2] * q[i15]) + (-8.918000e-04) * ((2) * q[i3] * q[i15]) + (4.768543e-04) * ((2) * q[i4] * q[i15])
            + (-1.741244e-03) * ((2) * q[i5] * q[i15]) + (1.854455e-03) * ((2) * q[i6] * q[i15]) + (-1.486156e-03) * ((2) * q[i7] * q[i15])
            + (-2.453704e-03) * ((2) * q[i8] * q[i15]) + (-7.700225e-05) * ((2) * q[i9] * q[i15]) + (3.441780e-04) * ((2) * q[i10] * q[i15])
            + (-5.886223e-03) * ((2) * q[i11] * q[i15]) + (2.535798e-04) * ((2) * q[i12] * q[i15]) + (-3.729371e-04) * ((3) * q[i15] * q[i15])
            + (-1.696360e-04) * ((2) * q[i15] * q[i16]) + (7.238608e-04) * ((2) * q[i15] * q[i19]) + (1.245525e-04) * ((2) * q[i15] * q[i20])
            + (1.243250e-03) * ((2) * q[i15] * q[i21]) + (-5.914336e-05) * ((2) * q[i15] * q[i22]) + (1.689590e-04) * (q[i16] * q[i16])
            + (2.784032e-04) * (q[i19] * q[i19]) + (2.665662e-04) * (q[i20] * q[i20]) + (2.829903e-04) * (q[i21] * q[i21]) + (1.136117e-04) * (q[i22] * q[i22])
            + (-1.695188e-03) * (q[i0] * q[i1]) + (3.870245e-04) * (q[i0] * q[i2]) + (2.500203e-03) * (q[i0] * q[i3]) + (-4.425119e-03) * (q[i0] * q[i4])
            + (3.217083e-03) * (q[i0] * q[i5]) + (-4.457664e-04) * (q[i0] * q[i6]) + (1.597382e-03) * (q[i0] * q[i7]) + (-1.237752e-03) * (q[i0] * q[i8])
            + (-2.962509e-04) * (q[i0] * q[i9]) + (-1.050272e-03) * (q[i0] * q[i10]) + (2.347744e-04) * (q[i0] * q[i11]) + (9.361919e-04) * (q[i0] * q[i12])
            + (2.377203e-06) * (q[i0] * q[i16]) + (-4.759495e-05) * (q[i0] * q[i19]) + (-3.216543e-04) * (q[i0] * q[i20]) + (-1.686942e-03) * (q[i0] * q[i21])
            + (3.592464e-04) * (q[i0] * q[i22]) + (6.267690e-04) * (q[i1] * q[i2]) + (1.236331e-03) * (q[i1] * q[i3]) + (2.785258e-03) * (q[i1] * q[i4])
            + (1.774251e-03) * (q[i1] * q[i5]) + (-2.089086e-03) * (q[i1] * q[i6]) + (2.515716e-03) * (q[i1] * q[i7]) + (4.747636e-05) * (q[i1] * q[i8])
            + (5.426493e-04) * (q[i1] * q[i9]) + (3.368781e-04) * (q[i1] * q[i10]) + (2.540007e-03) * (q[i1] * q[i11]) + (2.050431e-04) * (q[i1] * q[i12])
            + (-3.547933e-06) * (q[i1] * q[i16]) + (-2.625495e-04) * (q[i1] * q[i19]) + (1.049304e-03) * (q[i1] * q[i20]) + (-5.695855e-04) * (q[i1] * q[i21])
            + (4.220954e-04) * (q[i1] * q[i22]) + (3.432405e-03) * (q[i2] * q[i3]) + (-6.114839e-04) * (q[i2] * q[i4]) + (5.859833e-03) * (q[i2] * q[i5])
            + (-1.378391e-03) * (q[i2] * q[i6]) + (9.778492e-04) * (q[i2] * q[i7]) + (-1.013031e-03) * (q[i2] * q[i8]) + (1.020416e-03) * (q[i2] * q[i9])
            + (-1.123029e-03) * (q[i2] * q[i10]) + (2.076426e-03) * (q[i2] * q[i11]) + (5.847939e-04) * (q[i2] * q[i12]) + (-4.683864e-04) * (q[i2] * q[i16])
            + (7.724403e-04) * (q[i2] * q[i19]) + (4.007489e-04) * (q[i2] * q[i20]) + (-1.745349e-03) * (q[i2] * q[i21]) + (4.714810e-05) * (q[i2] * q[i22])
            + (3.793315e-03) * (q[i3] * q[i4]) + (-1.225580e-03) * (q[i3] * q[i5]) + (3.373408e-03) * (q[i3] * q[i6]) + (1.778172e-03) * (q[i3] * q[i7])
            + (-2.119369e-03) * (q[i3] * q[i8]) + (2.049416e-04) * (q[i3] * q[i9]) + (3.035723e-04) * (q[i3] * q[i10]) + (8.568046e-05) * (q[i3] * q[i11])
            + (-7.205732e-04) * (q[i3] * q[i12]) + (9.508782e-05) * (q[i3] * q[i16]) + (-5.166564e-04) * (q[i3] * q[i19]) + (1.693989e-03) * (q[i3] * q[i20])
            + (-6.911407e-06) * (q[i3] * q[i21]) + (-2.669654e-04) * (q[i3] * q[i22]) + (-3.038147e-03) * (q[i4] * q[i5]) + (-2.126183e-03) * (q[i4] * q[i6])
            + (3.312536e-03) * (q[i4] * q[i7]) + (4.996041e-04) * (q[i4] * q[i8]) + (-1.838502e-03) * (q[i4] * q[i9]) + (4.064886e-04) * (q[i4] * q[i10])
            + (9.640143e-04) * (q[i4] * q[i11]) + (3.887333e-04) * (q[i4] * q[i12]) + (1.030262e-04) * (q[i4] * q[i16]) + (2.427859e-03) * (q[i4] * q[i19])
            + (-2.048765e-03) * (q[i4] * q[i20]) + (-6.219502e-04) * (q[i4] * q[i21]) + (-9.526688e-05) * (q[i4] * q[i22]) + (-3.466603e-03) * (q[i5] * q[i6])
            + (-1.246730e-03) * (q[i5] * q[i7]) + (-2.887911e-03) * (q[i5] * q[i8]) + (4.149904e-04) * (q[i5] * q[i9]) + (1.927727e-04) * (q[i5] * q[i10])
            + (-3.131738e-03) * (q[i5] * q[i11]) + (-9.011617e-04) * (q[i5] * q[i12]) + (2.660128e-04) * (q[i5] * q[i16]) + (1.010086e-04) * (q[i5] * q[i19])
            + (1.702609e-05) * (q[i5] * q[i20]) + (-3.191987e-04) * (q[i5] * q[i21]) + (3.559548e-04) * (q[i5] * q[i22]) + (-4.010950e-03) * (q[i6] * q[i7])
            + (8.525784e-04) * (q[i6] * q[i8]) + (9.019470e-04) * (q[i6] * q[i9]) + (-8.672907e-04) * (q[i6] * q[i10]) + (2.391041e-04) * (q[i6] * q[i11])
            + (-1.811602e-03) * (q[i6] * q[i12]) + (2.550293e-04) * (q[i6] * q[i16]) + (-1.196648e-03) * (q[i6] * q[i19]) + (1.111495e-03) * (q[i6] * q[i20])
            + (-1.324569e-03) * (q[i6] * q[i21]) + (1.150947e-03) * (q[i6] * q[i22]) + (3.240030e-03) * (q[i7] * q[i8]) + (-3.932300e-04) * (q[i7] * q[i9])
            + (5.882058e-04) * (q[i7] * q[i10]) + (6.059683e-04) * (q[i7] * q[i11]) + (1.800135e-03) * (q[i7] * q[i12]) + (-2.521683e-04) * (q[i7] * q[i16])
            + (2.188180e-04) * (q[i7] * q[i19]) + (3.129368e-04) * (q[i7] * q[i20]) + (-3.455207e-04) * (q[i7] * q[i21]) + (4.722414e-04) * (q[i7] * q[i22])
            + (-1.185753e-03) * (q[i8] * q[i9]) + (2.442853e-03) * (q[i8] * q[i10]) + (2.710908e-03) * (q[i8] * q[i11]) + (-2.242985e-04) * (q[i8] * q[i12])
            + (3.408042e-06) * (q[i8] * q[i16]) + (3.239668e-04) * (q[i8] * q[i19]) + (-3.857058e-04) * (q[i8] * q[i20]) + (2.597813e-03) * (q[i8] * q[i21])
            + (-5.794559e-05) * (q[i8] * q[i22]) + (-1.735716e-04) * (q[i9] * q[i10]) + (4.938244e-04) * (q[i9] * q[i11]) + (-1.756373e-04) * (q[i9] * q[i12])
            + (-5.587725e-04) * (q[i9] * q[i16]) + (-6.830786e-05) * (q[i9] * q[i19]) + (3.258650e-04) * (q[i9] * q[i20]) + (-5.439368e-04) * (q[i9] * q[i21])
            + (4.191311e-04) * (q[i9] * q[i22]) + (7.359768e-04) * (q[i10] * q[i11]) + (2.738951e-04) * (q[i10] * q[i12]) + (5.650456e-04) * (q[i10] * q[i16])
            + (-1.323603e-04) * (q[i10] * q[i19]) + (2.752441e-04) * (q[i10] * q[i20]) + (-5.644669e-05) * (q[i10] * q[i21])
            + (3.015409e-04) * (q[i10] * q[i22]) + (-3.694582e-04) * (q[i11] * q[i12]) + (-1.547781e-04) * (q[i11] * q[i16])
            + (6.189937e-04) * (q[i11] * q[i19]) + (-5.593809e-04) * (q[i11] * q[i20]) + (1.712960e-03) * (q[i11] * q[i21]) + (2.039076e-04) * (q[i11] * q[i22])
            + (-1.589321e-04) * (q[i12] * q[i16]) + (-3.018339e-04) * (q[i12] * q[i19]) + (1.066293e-04) * (q[i12] * q[i20])
            + (6.021189e-05) * (q[i12] * q[i21]) + (-2.014812e-04) * (q[i12] * q[i22]) + (3.772262e-04) * (q[i16] * q[i19])
            + (-3.754970e-04) * (q[i16] * q[i20]) + (-1.755289e-04) * (q[i16] * q[i21]) + (-1.772365e-04) * (q[i16] * q[i22])
            + (-1.729091e-04) * (q[i19] * q[i20]) + (3.298283e-04) * (q[i19] * q[i21]) + (-3.501043e-04) * (q[i19] * q[i22])
            + (-7.171487e-05) * (q[i20] * q[i21]) + (8.631228e-05) * (q[i20] * q[i22]) + (1.734183e-04) * (q[i21] * q[i22]);
   }

   public void getJQz16(double[] q, double[][] JQ)
   {
      JQ[3][i16] = (-1.112881e-02) * (1) + (-3.722115e-03) * ((2) * q[i16]) + (-1.004595e-02) * (q[i0]) + (-8.015240e-03) * (q[i1]) + (-2.057607e-02) * (q[i2])
            + (9.240801e-04) * (q[i3]) + (4.485459e-03) * (q[i4]) + (-3.352356e-03) * (q[i5]) + (9.207379e-03) * (q[i6]) + (5.056939e-05) * (q[i7])
            + (-1.188977e-02) * (q[i8]) + (1.225156e-03) * (q[i9]) + (3.101136e-03) * (q[i10]) + (1.153869e-03) * (q[i11]) + (6.336331e-04) * (q[i12])
            + (9.802463e-07) * (q[i15]) + (1.305496e-03) * (q[i19]) + (-5.222701e-03) * (q[i20]) + (-8.271314e-05) * (q[i21]) + (3.876542e-03) * (q[i22])
            + (-1.053388e-03) * (q[i0] * q[i0]) + (-1.540970e-04) * (q[i1] * q[i1]) + (2.779448e-04) * (q[i2] * q[i2]) + (-1.417495e-03) * (q[i3] * q[i3])
            + (3.186542e-03) * (q[i4] * q[i4]) + (-2.095174e-03) * (q[i5] * q[i5]) + (-1.063728e-03) * (q[i6] * q[i6]) + (-1.961758e-03) * (q[i7] * q[i7])
            + (3.063677e-03) * (q[i8] * q[i8]) + (-4.382164e-04) * (q[i9] * q[i9]) + (-2.335088e-04) * (q[i10] * q[i10]) + (-2.826463e-04) * (q[i11] * q[i11])
            + (-1.424586e-03) * (q[i12] * q[i12]) + (-1.696360e-04) * (q[i15] * q[i15]) + (-2.086639e-03) * ((2) * q[i0] * q[i16])
            + (-1.917866e-03) * ((2) * q[i1] * q[i16]) + (-4.648974e-03) * ((2) * q[i2] * q[i16]) + (4.890517e-04) * ((2) * q[i3] * q[i16])
            + (-8.897604e-04) * ((2) * q[i4] * q[i16]) + (-1.767671e-03) * ((2) * q[i5] * q[i16]) + (1.498998e-03) * ((2) * q[i6] * q[i16])
            + (-1.862477e-03) * ((2) * q[i7] * q[i16]) + (2.480365e-03) * ((2) * q[i8] * q[i16]) + (-3.474733e-04) * ((2) * q[i9] * q[i16])
            + (7.707608e-05) * ((2) * q[i10] * q[i16]) + (2.513389e-04) * ((2) * q[i11] * q[i16]) + (-5.953830e-03) * ((2) * q[i12] * q[i16])
            + (1.689590e-04) * ((2) * q[i15] * q[i16]) + (3.780196e-04) * ((3) * q[i16] * q[i16]) + (-1.198530e-04) * ((2) * q[i16] * q[i19])
            + (-7.271708e-04) * ((2) * q[i16] * q[i20]) + (-5.805207e-05) * ((2) * q[i16] * q[i21]) + (1.255390e-03) * ((2) * q[i16] * q[i22])
            + (-2.679767e-04) * (q[i19] * q[i19]) + (-2.648461e-04) * (q[i20] * q[i20]) + (-1.146371e-04) * (q[i21] * q[i21])
            + (-2.800429e-04) * (q[i22] * q[i22]) + (1.701661e-03) * (q[i0] * q[i1]) + (-6.183652e-04) * (q[i0] * q[i2]) + (-2.838333e-03) * (q[i0] * q[i3])
            + (-1.259237e-03) * (q[i0] * q[i4]) + (-1.774397e-03) * (q[i0] * q[i5]) + (2.525945e-03) * (q[i0] * q[i6]) + (-2.070965e-03) * (q[i0] * q[i7])
            + (3.349766e-05) * (q[i0] * q[i8]) + (3.266708e-04) * (q[i0] * q[i9]) + (5.576206e-04) * (q[i0] * q[i10]) + (-2.017318e-04) * (q[i0] * q[i11])
            + (-2.527649e-03) * (q[i0] * q[i12]) + (2.377203e-06) * (q[i0] * q[i15]) + (1.059564e-03) * (q[i0] * q[i19]) + (-2.461122e-04) * (q[i0] * q[i20])
            + (-4.163082e-04) * (q[i0] * q[i21]) + (5.712635e-04) * (q[i0] * q[i22]) + (-3.589615e-04) * (q[i1] * q[i2]) + (4.454264e-03) * (q[i1] * q[i3])
            + (-2.476091e-03) * (q[i1] * q[i4]) + (-3.250903e-03) * (q[i1] * q[i5]) + (1.587016e-03) * (q[i1] * q[i6]) + (-4.501706e-04) * (q[i1] * q[i7])
            + (-1.247925e-03) * (q[i1] * q[i8]) + (-1.040909e-03) * (q[i1] * q[i9]) + (-2.906111e-04) * (q[i1] * q[i10]) + (-9.214786e-04) * (q[i1] * q[i11])
            + (-2.460959e-04) * (q[i1] * q[i12]) + (-3.547933e-06) * (q[i1] * q[i15]) + (-3.201155e-04) * (q[i1] * q[i19]) + (-3.407726e-05) * (q[i1] * q[i20])
            + (-3.615603e-04) * (q[i1] * q[i21]) + (1.679648e-03) * (q[i1] * q[i22]) + (6.102899e-04) * (q[i2] * q[i3]) + (-3.448970e-03) * (q[i2] * q[i4])
            + (-5.913097e-03) * (q[i2] * q[i5]) + (9.848270e-04) * (q[i2] * q[i6]) + (-1.405652e-03) * (q[i2] * q[i7]) + (-1.017945e-03) * (q[i2] * q[i8])
            + (-1.131341e-03) * (q[i2] * q[i9]) + (1.026945e-03) * (q[i2] * q[i10]) + (-5.741165e-04) * (q[i2] * q[i11]) + (-2.091056e-03) * (q[i2] * q[i12])
            + (-4.683864e-04) * (q[i2] * q[i15]) + (4.203856e-04) * (q[i2] * q[i19]) + (7.946630e-04) * (q[i2] * q[i20]) + (-3.777557e-05) * (q[i2] * q[i21])
            + (1.742322e-03) * (q[i2] * q[i22]) + (-3.798990e-03) * (q[i3] * q[i4]) + (3.048870e-03) * (q[i3] * q[i5]) + (3.345002e-03) * (q[i3] * q[i6])
            + (-2.089276e-03) * (q[i3] * q[i7]) + (4.871352e-04) * (q[i3] * q[i8]) + (4.223724e-04) * (q[i3] * q[i9]) + (-1.849129e-03) * (q[i3] * q[i10])
            + (-3.801519e-04) * (q[i3] * q[i11]) + (-9.703579e-04) * (q[i3] * q[i12]) + (9.508782e-05) * (q[i3] * q[i15]) + (-2.059293e-03) * (q[i3] * q[i19])
            + (2.422904e-03) * (q[i3] * q[i20]) + (1.136429e-04) * (q[i3] * q[i21]) + (6.164245e-04) * (q[i3] * q[i22]) + (1.249266e-03) * (q[i4] * q[i5])
            + (1.788607e-03) * (q[i4] * q[i6]) + (3.367054e-03) * (q[i4] * q[i7]) + (-2.130278e-03) * (q[i4] * q[i8]) + (3.111986e-04) * (q[i4] * q[i9])
            + (2.140959e-04) * (q[i4] * q[i10]) + (7.220888e-04) * (q[i4] * q[i11]) + (-7.257782e-05) * (q[i4] * q[i12]) + (1.030262e-04) * (q[i4] * q[i15])
            + (1.697302e-03) * (q[i4] * q[i19]) + (-5.161785e-04) * (q[i4] * q[i20]) + (2.637954e-04) * (q[i4] * q[i21]) + (3.106142e-05) * (q[i4] * q[i22])
            + (-1.234001e-03) * (q[i5] * q[i6]) + (-3.479245e-03) * (q[i5] * q[i7]) + (-2.914658e-03) * (q[i5] * q[i8]) + (1.719742e-04) * (q[i5] * q[i9])
            + (4.171139e-04) * (q[i5] * q[i10]) + (8.952596e-04) * (q[i5] * q[i11]) + (3.142238e-03) * (q[i5] * q[i12]) + (2.660128e-04) * (q[i5] * q[i15])
            + (1.734017e-05) * (q[i5] * q[i19]) + (1.067308e-04) * (q[i5] * q[i20]) + (-3.566293e-04) * (q[i5] * q[i21]) + (3.086819e-04) * (q[i5] * q[i22])
            + (4.011652e-03) * (q[i6] * q[i7]) + (-3.242813e-03) * (q[i6] * q[i8]) + (-5.992658e-04) * (q[i6] * q[i9]) + (3.818487e-04) * (q[i6] * q[i10])
            + (1.804663e-03) * (q[i6] * q[i11]) + (6.003288e-04) * (q[i6] * q[i12]) + (2.550293e-04) * (q[i6] * q[i15]) + (-3.283877e-04) * (q[i6] * q[i19])
            + (-2.231642e-04) * (q[i6] * q[i20]) + (4.686602e-04) * (q[i6] * q[i21]) + (-3.558965e-04) * (q[i6] * q[i22]) + (-8.830871e-04) * (q[i7] * q[i8])
            + (8.676878e-04) * (q[i7] * q[i9]) + (-8.908161e-04) * (q[i7] * q[i10]) + (-1.812968e-03) * (q[i7] * q[i11]) + (2.377230e-04) * (q[i7] * q[i12])
            + (-2.521683e-04) * (q[i7] * q[i15]) + (-1.106108e-03) * (q[i7] * q[i19]) + (1.191575e-03) * (q[i7] * q[i20]) + (1.154882e-03) * (q[i7] * q[i21])
            + (-1.324447e-03) * (q[i7] * q[i22]) + (-2.455940e-03) * (q[i8] * q[i9]) + (1.178279e-03) * (q[i8] * q[i10]) + (-2.294426e-04) * (q[i8] * q[i11])
            + (2.704199e-03) * (q[i8] * q[i12]) + (3.408042e-06) * (q[i8] * q[i15]) + (3.936908e-04) * (q[i8] * q[i19]) + (-3.308385e-04) * (q[i8] * q[i20])
            + (-5.266935e-05) * (q[i8] * q[i21]) + (2.635654e-03) * (q[i8] * q[i22]) + (1.709716e-04) * (q[i9] * q[i10]) + (2.749411e-04) * (q[i9] * q[i11])
            + (7.380747e-04) * (q[i9] * q[i12]) + (-5.587725e-04) * (q[i9] * q[i15]) + (-2.823206e-04) * (q[i9] * q[i19]) + (1.364311e-04) * (q[i9] * q[i20])
            + (3.010307e-04) * (q[i9] * q[i21]) + (-6.196279e-05) * (q[i9] * q[i22]) + (-1.702980e-04) * (q[i10] * q[i11]) + (4.983328e-04) * (q[i10] * q[i12])
            + (5.650456e-04) * (q[i10] * q[i15]) + (-3.197884e-04) * (q[i10] * q[i19]) + (6.513024e-05) * (q[i10] * q[i20]) + (4.160347e-04) * (q[i10] * q[i21])
            + (-5.392491e-04) * (q[i10] * q[i22]) + (3.527266e-04) * (q[i11] * q[i12]) + (-1.547781e-04) * (q[i11] * q[i15])
            + (1.117545e-04) * (q[i11] * q[i19]) + (-2.979092e-04) * (q[i11] * q[i20]) + (1.930622e-04) * (q[i11] * q[i21])
            + (-6.535191e-05) * (q[i11] * q[i22]) + (-1.589321e-04) * (q[i12] * q[i15]) + (-5.592825e-04) * (q[i12] * q[i19])
            + (6.358665e-04) * (q[i12] * q[i20]) + (-2.030599e-04) * (q[i12] * q[i21]) + (-1.759297e-03) * (q[i12] * q[i22])
            + (3.772262e-04) * (q[i15] * q[i19]) + (-3.754970e-04) * (q[i15] * q[i20]) + (-1.755289e-04) * (q[i15] * q[i21])
            + (-1.772365e-04) * (q[i15] * q[i22]) + (1.727719e-04) * (q[i19] * q[i20]) + (8.513549e-05) * (q[i19] * q[i21])
            + (-7.545387e-05) * (q[i19] * q[i22]) + (-3.464083e-04) * (q[i20] * q[i21]) + (3.163393e-04) * (q[i20] * q[i22])
            + (-1.737211e-04) * (q[i21] * q[i22]);
   }

   public void getJQz19(double[] q, double[][] JQ)
   {
      JQ[3][i19] = (1.147939e-02) * (1) + (-1.825673e-03) * ((2) * q[i19]) + (1.977376e-03) * (q[i0]) + (-1.988935e-03) * (q[i1]) + (2.362708e-03) * (q[i2])
            + (-1.019916e-02) * (q[i3]) + (1.228612e-02) * (q[i4]) + (4.195663e-03) * (q[i5]) + (-3.224428e-03) * (q[i6]) + (1.962425e-03) * (q[i7])
            + (3.065881e-03) * (q[i8]) + (-1.853988e-03) * (q[i9]) + (1.017924e-03) * (q[i10]) + (3.057393e-03) * (q[i11]) + (3.944362e-04) * (q[i12])
            + (5.300136e-03) * (q[i15]) + (1.305496e-03) * (q[i16]) + (-1.310761e-06) * (q[i20]) + (-5.192123e-03) * (q[i21]) + (-1.044434e-03) * (q[i22])
            + (7.116120e-04) * (q[i0] * q[i0]) + (-3.622465e-04) * (q[i1] * q[i1]) + (-5.728309e-04) * (q[i2] * q[i2]) + (2.284390e-03) * (q[i3] * q[i3])
            + (2.887153e-03) * (q[i4] * q[i4]) + (3.035728e-06) * (q[i5] * q[i5]) + (-1.662778e-03) * (q[i6] * q[i6]) + (-3.645672e-04) * (q[i7] * q[i7])
            + (1.206806e-04) * (q[i8] * q[i8]) + (-6.611220e-05) * (q[i9] * q[i9]) + (-6.300817e-04) * (q[i10] * q[i10]) + (-1.238077e-03) * (q[i11] * q[i11])
            + (-2.953930e-04) * (q[i12] * q[i12]) + (7.238608e-04) * (q[i15] * q[i15]) + (-1.198530e-04) * (q[i16] * q[i16])
            + (3.747856e-04) * ((2) * q[i0] * q[i19]) + (-6.184213e-04) * ((2) * q[i1] * q[i19]) + (-4.758407e-04) * ((2) * q[i2] * q[i19])
            + (-5.499706e-04) * ((2) * q[i3] * q[i19]) + (4.783957e-04) * ((2) * q[i4] * q[i19]) + (5.838992e-04) * ((2) * q[i5] * q[i19])
            + (1.224872e-03) * ((2) * q[i6] * q[i19]) + (-1.188255e-03) * ((2) * q[i7] * q[i19]) + (6.152050e-04) * ((2) * q[i8] * q[i19])
            + (3.324393e-04) * ((2) * q[i9] * q[i19]) + (1.273070e-04) * ((2) * q[i10] * q[i19]) + (7.898908e-04) * ((2) * q[i11] * q[i19])
            + (-2.068457e-04) * ((2) * q[i12] * q[i19]) + (2.784032e-04) * ((2) * q[i15] * q[i19]) + (-2.679767e-04) * ((2) * q[i16] * q[i19])
            + (-3.305556e-04) * ((3) * q[i19] * q[i19]) + (1.467082e-05) * ((2) * q[i19] * q[i20]) + (-1.887431e-03) * ((2) * q[i19] * q[i21])
            + (8.010234e-05) * ((2) * q[i19] * q[i22]) + (-2.308392e-05) * (q[i20] * q[i20]) + (-1.248131e-03) * (q[i21] * q[i21])
            + (2.622147e-05) * (q[i22] * q[i22]) + (-2.723746e-04) * (q[i0] * q[i1]) + (-1.829261e-04) * (q[i0] * q[i2]) + (-2.411072e-03) * (q[i0] * q[i3])
            + (1.293175e-03) * (q[i0] * q[i4]) + (-4.666657e-04) * (q[i0] * q[i5]) + (-3.448775e-03) * (q[i0] * q[i6]) + (-1.237260e-03) * (q[i0] * q[i7])
            + (1.515999e-03) * (q[i0] * q[i8]) + (-1.340145e-04) * (q[i0] * q[i9]) + (-6.144819e-04) * (q[i0] * q[i10]) + (6.241591e-04) * (q[i0] * q[i11])
            + (1.297013e-03) * (q[i0] * q[i12]) + (-4.759495e-05) * (q[i0] * q[i15]) + (1.059564e-03) * (q[i0] * q[i16]) + (2.088757e-04) * (q[i0] * q[i20])
            + (-6.207025e-06) * (q[i0] * q[i21]) + (-4.942090e-04) * (q[i0] * q[i22]) + (-1.329226e-04) * (q[i1] * q[i2]) + (1.676639e-04) * (q[i1] * q[i3])
            + (3.806077e-04) * (q[i1] * q[i4]) + (3.999837e-04) * (q[i1] * q[i5]) + (-1.365403e-04) * (q[i1] * q[i6]) + (5.669661e-04) * (q[i1] * q[i7])
            + (-1.008339e-03) * (q[i1] * q[i8]) + (1.209350e-04) * (q[i1] * q[i9]) + (4.356600e-04) * (q[i1] * q[i10]) + (-9.333654e-04) * (q[i1] * q[i11])
            + (9.758899e-04) * (q[i1] * q[i12]) + (-2.625495e-04) * (q[i1] * q[i15]) + (-3.201155e-04) * (q[i1] * q[i16]) + (2.035014e-04) * (q[i1] * q[i20])
            + (-3.969437e-04) * (q[i1] * q[i21]) + (7.699027e-04) * (q[i1] * q[i22]) + (-2.403196e-03) * (q[i2] * q[i3]) + (8.164762e-04) * (q[i2] * q[i4])
            + (-1.748491e-03) * (q[i2] * q[i5]) + (-2.454018e-05) * (q[i2] * q[i6]) + (3.461047e-04) * (q[i2] * q[i7]) + (-1.213964e-03) * (q[i2] * q[i8])
            + (2.814179e-04) * (q[i2] * q[i9]) + (-4.671196e-04) * (q[i2] * q[i10]) + (-6.186795e-04) * (q[i2] * q[i11]) + (4.856886e-04) * (q[i2] * q[i12])
            + (7.724403e-04) * (q[i2] * q[i15]) + (4.203856e-04) * (q[i2] * q[i16]) + (-4.370205e-04) * (q[i2] * q[i20]) + (-1.674023e-03) * (q[i2] * q[i21])
            + (1.799295e-05) * (q[i2] * q[i22]) + (-6.057769e-03) * (q[i3] * q[i4]) + (-1.357805e-03) * (q[i3] * q[i5]) + (-1.681426e-03) * (q[i3] * q[i6])
            + (1.133573e-03) * (q[i3] * q[i7]) + (3.770535e-03) * (q[i3] * q[i8]) + (1.406502e-03) * (q[i3] * q[i9]) + (6.035180e-04) * (q[i3] * q[i10])
            + (3.596755e-04) * (q[i3] * q[i11]) + (-4.043702e-03) * (q[i3] * q[i12]) + (-5.166564e-04) * (q[i3] * q[i15]) + (-2.059293e-03) * (q[i3] * q[i16])
            + (-5.725087e-04) * (q[i3] * q[i20]) + (3.786057e-04) * (q[i3] * q[i21]) + (-1.428575e-03) * (q[i3] * q[i22]) + (1.341935e-03) * (q[i4] * q[i5])
            + (4.188873e-03) * (q[i4] * q[i6]) + (-9.830795e-04) * (q[i4] * q[i7]) + (-2.264376e-03) * (q[i4] * q[i8]) + (-1.010468e-03) * (q[i4] * q[i9])
            + (-1.031287e-03) * (q[i4] * q[i10]) + (1.144213e-03) * (q[i4] * q[i11]) + (2.760975e-03) * (q[i4] * q[i12]) + (2.427859e-03) * (q[i4] * q[i15])
            + (1.697302e-03) * (q[i4] * q[i16]) + (-5.556551e-04) * (q[i4] * q[i20]) + (-2.016544e-03) * (q[i4] * q[i21]) + (-1.405249e-03) * (q[i4] * q[i22])
            + (3.728839e-03) * (q[i5] * q[i6]) + (-2.770813e-03) * (q[i5] * q[i7]) + (1.483302e-05) * (q[i5] * q[i8]) + (-6.854388e-04) * (q[i5] * q[i9])
            + (-5.324809e-04) * (q[i5] * q[i10]) + (-2.765632e-03) * (q[i5] * q[i11]) + (-1.107020e-03) * (q[i5] * q[i12]) + (1.010086e-04) * (q[i5] * q[i15])
            + (1.734017e-05) * (q[i5] * q[i16]) + (1.162797e-03) * (q[i5] * q[i20]) + (9.746688e-04) * (q[i5] * q[i21]) + (8.000354e-04) * (q[i5] * q[i22])
            + (3.888448e-03) * (q[i6] * q[i7]) + (-7.519003e-04) * (q[i6] * q[i8]) + (-2.982457e-04) * (q[i6] * q[i9]) + (1.426584e-03) * (q[i6] * q[i10])
            + (1.196569e-03) * (q[i6] * q[i11]) + (4.007865e-05) * (q[i6] * q[i12]) + (-1.196648e-03) * (q[i6] * q[i15]) + (-3.283877e-04) * (q[i6] * q[i16])
            + (2.706007e-03) * (q[i6] * q[i20]) + (-5.003558e-04) * (q[i6] * q[i21]) + (7.818853e-04) * (q[i6] * q[i22]) + (-2.249042e-03) * (q[i7] * q[i8])
            + (-9.565985e-05) * (q[i7] * q[i9]) + (-1.729129e-03) * (q[i7] * q[i10]) + (-1.546261e-03) * (q[i7] * q[i11]) + (-8.816813e-04) * (q[i7] * q[i12])
            + (2.188180e-04) * (q[i7] * q[i15]) + (-1.106108e-03) * (q[i7] * q[i16]) + (-2.697755e-03) * (q[i7] * q[i20]) + (-2.370805e-03) * (q[i7] * q[i21])
            + (-2.713702e-04) * (q[i7] * q[i22]) + (-2.689159e-05) * (q[i8] * q[i9]) + (-1.579250e-03) * (q[i8] * q[i10]) + (-1.246248e-03) * (q[i8] * q[i11])
            + (5.618584e-04) * (q[i8] * q[i12]) + (3.239668e-04) * (q[i8] * q[i15]) + (3.936908e-04) * (q[i8] * q[i16]) + (-4.539242e-07) * (q[i8] * q[i20])
            + (2.748206e-03) * (q[i8] * q[i21]) + (-7.039320e-04) * (q[i8] * q[i22]) + (2.079621e-04) * (q[i9] * q[i10]) + (1.018745e-04) * (q[i9] * q[i11])
            + (-8.797610e-05) * (q[i9] * q[i12]) + (-6.830786e-05) * (q[i9] * q[i15]) + (-2.823206e-04) * (q[i9] * q[i16]) + (7.958606e-04) * (q[i9] * q[i20])
            + (-4.528721e-04) * (q[i9] * q[i21]) + (2.472474e-04) * (q[i9] * q[i22]) + (-3.876385e-04) * (q[i10] * q[i11]) + (-2.560389e-04) * (q[i10] * q[i12])
            + (-1.323603e-04) * (q[i10] * q[i15]) + (-3.197884e-04) * (q[i10] * q[i16]) + (-7.909997e-04) * (q[i10] * q[i20])
            + (-2.803785e-04) * (q[i10] * q[i21]) + (-5.321534e-05) * (q[i10] * q[i22]) + (1.785714e-04) * (q[i11] * q[i12])
            + (6.189937e-04) * (q[i11] * q[i15]) + (1.117545e-04) * (q[i11] * q[i16]) + (2.243249e-04) * (q[i11] * q[i20]) + (1.306839e-03) * (q[i11] * q[i21])
            + (4.637868e-04) * (q[i11] * q[i22]) + (-3.018339e-04) * (q[i12] * q[i15]) + (-5.592825e-04) * (q[i12] * q[i16])
            + (2.304211e-04) * (q[i12] * q[i20]) + (1.028135e-04) * (q[i12] * q[i21]) + (-1.867936e-05) * (q[i12] * q[i22]) + (3.772262e-04) * (q[i15] * q[i16])
            + (-1.729091e-04) * (q[i15] * q[i20]) + (3.298283e-04) * (q[i15] * q[i21]) + (-3.501043e-04) * (q[i15] * q[i22])
            + (1.727719e-04) * (q[i16] * q[i20]) + (8.513549e-05) * (q[i16] * q[i21]) + (-7.545387e-05) * (q[i16] * q[i22])
            + (-1.371422e-04) * (q[i20] * q[i21]) + (-1.367320e-04) * (q[i20] * q[i22]) + (-6.391255e-05) * (q[i21] * q[i22]);
   }

   public void getJQz20(double[] q, double[][] JQ)
   {
      JQ[3][i20] = (-1.124376e-02) * (1) + (1.814904e-03) * ((2) * q[i20]) + (-2.009966e-03) * (q[i0]) + (1.956409e-03) * (q[i1]) + (2.297146e-03) * (q[i2])
            + (1.230024e-02) * (q[i3]) + (-1.017810e-02) * (q[i4]) + (4.206466e-03) * (q[i5]) + (-1.961197e-03) * (q[i6]) + (3.172150e-03) * (q[i7])
            + (-3.045078e-03) * (q[i8]) + (-1.021110e-03) * (q[i9]) + (1.831189e-03) * (q[i10]) + (3.689554e-04) * (q[i11]) + (3.032000e-03) * (q[i12])
            + (-1.283342e-03) * (q[i15]) + (-5.222701e-03) * (q[i16]) + (-1.310761e-06) * (q[i19]) + (-1.033220e-03) * (q[i21]) + (-5.224340e-03) * (q[i22])
            + (3.588934e-04) * (q[i0] * q[i0]) + (-6.913918e-04) * (q[i1] * q[i1]) + (5.599040e-04) * (q[i2] * q[i2]) + (-2.929116e-03) * (q[i3] * q[i3])
            + (-2.277595e-03) * (q[i4] * q[i4]) + (1.337627e-06) * (q[i5] * q[i5]) + (3.625732e-04) * (q[i6] * q[i6]) + (1.653534e-03) * (q[i7] * q[i7])
            + (-1.346143e-04) * (q[i8] * q[i8]) + (6.369948e-04) * (q[i9] * q[i9]) + (6.601364e-05) * (q[i10] * q[i10]) + (2.976851e-04) * (q[i11] * q[i11])
            + (1.201686e-03) * (q[i12] * q[i12]) + (1.245525e-04) * (q[i15] * q[i15]) + (-7.271708e-04) * (q[i16] * q[i16]) + (1.467082e-05) * (q[i19] * q[i19])
            + (-6.009789e-04) * ((2) * q[i0] * q[i20]) + (3.759789e-04) * ((2) * q[i1] * q[i20]) + (-4.632956e-04) * ((2) * q[i2] * q[i20])
            + (4.830417e-04) * ((2) * q[i3] * q[i20]) + (-5.486979e-04) * ((2) * q[i4] * q[i20]) + (5.822137e-04) * ((2) * q[i5] * q[i20])
            + (1.177861e-03) * ((2) * q[i6] * q[i20]) + (-1.234309e-03) * ((2) * q[i7] * q[i20]) + (-5.973724e-04) * ((2) * q[i8] * q[i20])
            + (-1.292999e-04) * ((2) * q[i9] * q[i20]) + (-3.326441e-04) * ((2) * q[i10] * q[i20]) + (-2.012535e-04) * ((2) * q[i11] * q[i20])
            + (7.762578e-04) * ((2) * q[i12] * q[i20]) + (2.665662e-04) * ((2) * q[i15] * q[i20]) + (-2.648461e-04) * ((2) * q[i16] * q[i20])
            + (-2.308392e-05) * ((2) * q[i19] * q[i20]) + (3.206918e-04) * ((3) * q[i20] * q[i20]) + (8.445291e-05) * ((2) * q[i20] * q[i21])
            + (-1.884380e-03) * ((2) * q[i20] * q[i22]) + (-2.553853e-05) * (q[i21] * q[i21]) + (1.266536e-03) * (q[i22] * q[i22])
            + (2.669963e-04) * (q[i0] * q[i1]) + (1.314575e-04) * (q[i0] * q[i2]) + (-3.702059e-04) * (q[i0] * q[i3]) + (-1.629772e-04) * (q[i0] * q[i4])
            + (-4.053795e-04) * (q[i0] * q[i5]) + (5.469725e-04) * (q[i0] * q[i6]) + (-1.390933e-04) * (q[i0] * q[i7]) + (-1.020506e-03) * (q[i0] * q[i8])
            + (4.477149e-04) * (q[i0] * q[i9]) + (1.285622e-04) * (q[i0] * q[i10]) + (-9.831590e-04) * (q[i0] * q[i11]) + (9.315936e-04) * (q[i0] * q[i12])
            + (-3.216543e-04) * (q[i0] * q[i15]) + (-2.461122e-04) * (q[i0] * q[i16]) + (2.088757e-04) * (q[i0] * q[i19]) + (-7.676975e-04) * (q[i0] * q[i21])
            + (3.850536e-04) * (q[i0] * q[i22]) + (1.997256e-04) * (q[i1] * q[i2]) + (-1.299664e-03) * (q[i1] * q[i3]) + (2.369193e-03) * (q[i1] * q[i4])
            + (4.605860e-04) * (q[i1] * q[i5]) + (-1.235479e-03) * (q[i1] * q[i6]) + (-3.425146e-03) * (q[i1] * q[i7]) + (1.523498e-03) * (q[i1] * q[i8])
            + (-6.194793e-04) * (q[i1] * q[i9]) + (-1.314759e-04) * (q[i1] * q[i10]) + (-1.281242e-03) * (q[i1] * q[i11]) + (-6.428755e-04) * (q[i1] * q[i12])
            + (1.049304e-03) * (q[i1] * q[i15]) + (-3.407726e-05) * (q[i1] * q[i16]) + (2.035014e-04) * (q[i1] * q[i19]) + (4.882128e-04) * (q[i1] * q[i21])
            + (1.876201e-05) * (q[i1] * q[i22]) + (-8.265750e-04) * (q[i2] * q[i3]) + (2.400144e-03) * (q[i2] * q[i4]) + (1.751262e-03) * (q[i2] * q[i5])
            + (3.393243e-04) * (q[i2] * q[i6]) + (-1.300624e-05) * (q[i2] * q[i7]) + (-1.202537e-03) * (q[i2] * q[i8]) + (-4.580965e-04) * (q[i2] * q[i9])
            + (2.916659e-04) * (q[i2] * q[i10]) + (-4.964284e-04) * (q[i2] * q[i11]) + (5.954556e-04) * (q[i2] * q[i12]) + (4.007489e-04) * (q[i2] * q[i15])
            + (7.946630e-04) * (q[i2] * q[i16]) + (-4.370205e-04) * (q[i2] * q[i19]) + (-2.370410e-05) * (q[i2] * q[i21]) + (1.679019e-03) * (q[i2] * q[i22])
            + (6.060812e-03) * (q[i3] * q[i4]) + (-1.363373e-03) * (q[i3] * q[i5]) + (-9.682098e-04) * (q[i3] * q[i6]) + (4.211431e-03) * (q[i3] * q[i7])
            + (-2.285660e-03) * (q[i3] * q[i8]) + (-1.023814e-03) * (q[i3] * q[i9]) + (-1.015407e-03) * (q[i3] * q[i10]) + (-2.743292e-03) * (q[i3] * q[i11])
            + (-1.148295e-03) * (q[i3] * q[i12]) + (1.693989e-03) * (q[i3] * q[i15]) + (2.422904e-03) * (q[i3] * q[i16]) + (-5.725087e-04) * (q[i3] * q[i19])
            + (1.415799e-03) * (q[i3] * q[i21]) + (2.036116e-03) * (q[i3] * q[i22]) + (1.377475e-03) * (q[i4] * q[i5]) + (1.145180e-03) * (q[i4] * q[i6])
            + (-1.690906e-03) * (q[i4] * q[i7]) + (3.739202e-03) * (q[i4] * q[i8]) + (6.126302e-04) * (q[i4] * q[i9]) + (1.394813e-03) * (q[i4] * q[i10])
            + (4.028919e-03) * (q[i4] * q[i11]) + (-3.517347e-04) * (q[i4] * q[i12]) + (-2.048765e-03) * (q[i4] * q[i15]) + (-5.161785e-04) * (q[i4] * q[i16])
            + (-5.556551e-04) * (q[i4] * q[i19]) + (1.415346e-03) * (q[i4] * q[i21]) + (-3.705879e-04) * (q[i4] * q[i22]) + (-2.791426e-03) * (q[i5] * q[i6])
            + (3.707763e-03) * (q[i5] * q[i7]) + (5.371657e-05) * (q[i5] * q[i8]) + (-5.408115e-04) * (q[i5] * q[i9]) + (-6.700685e-04) * (q[i5] * q[i10])
            + (1.098577e-03) * (q[i5] * q[i11]) + (2.738680e-03) * (q[i5] * q[i12]) + (1.702609e-05) * (q[i5] * q[i15]) + (1.067308e-04) * (q[i5] * q[i16])
            + (1.162797e-03) * (q[i5] * q[i19]) + (-7.983432e-04) * (q[i5] * q[i21]) + (-9.931816e-04) * (q[i5] * q[i22]) + (-3.886024e-03) * (q[i6] * q[i7])
            + (2.245990e-03) * (q[i6] * q[i8]) + (1.733925e-03) * (q[i6] * q[i9]) + (9.999129e-05) * (q[i6] * q[i10]) + (-8.816488e-04) * (q[i6] * q[i11])
            + (-1.552227e-03) * (q[i6] * q[i12]) + (1.111495e-03) * (q[i6] * q[i15]) + (-2.231642e-04) * (q[i6] * q[i16]) + (2.706007e-03) * (q[i6] * q[i19])
            + (-2.724772e-04) * (q[i6] * q[i21]) + (-2.384966e-03) * (q[i6] * q[i22]) + (7.715338e-04) * (q[i7] * q[i8]) + (-1.426565e-03) * (q[i7] * q[i9])
            + (2.999787e-04) * (q[i7] * q[i10]) + (4.480873e-05) * (q[i7] * q[i11]) + (1.192935e-03) * (q[i7] * q[i12]) + (3.129368e-04) * (q[i7] * q[i15])
            + (1.191575e-03) * (q[i7] * q[i16]) + (-2.697755e-03) * (q[i7] * q[i19]) + (7.750717e-04) * (q[i7] * q[i21]) + (-4.975113e-04) * (q[i7] * q[i22])
            + (1.587067e-03) * (q[i8] * q[i9]) + (3.105124e-05) * (q[i8] * q[i10]) + (5.569696e-04) * (q[i8] * q[i11]) + (-1.202175e-03) * (q[i8] * q[i12])
            + (-3.857058e-04) * (q[i8] * q[i15]) + (-3.308385e-04) * (q[i8] * q[i16]) + (-4.539242e-07) * (q[i8] * q[i19]) + (-7.043087e-04) * (q[i8] * q[i21])
            + (2.734584e-03) * (q[i8] * q[i22]) + (-2.078830e-04) * (q[i9] * q[i10]) + (-2.517958e-04) * (q[i9] * q[i11]) + (-3.981859e-04) * (q[i9] * q[i12])
            + (3.258650e-04) * (q[i9] * q[i15]) + (1.364311e-04) * (q[i9] * q[i16]) + (7.958606e-04) * (q[i9] * q[i19]) + (-5.562106e-05) * (q[i9] * q[i21])
            + (-2.840447e-04) * (q[i9] * q[i22]) + (-7.496709e-05) * (q[i10] * q[i11]) + (1.014790e-04) * (q[i10] * q[i12]) + (2.752441e-04) * (q[i10] * q[i15])
            + (6.513024e-05) * (q[i10] * q[i16]) + (-7.909997e-04) * (q[i10] * q[i19]) + (2.432303e-04) * (q[i10] * q[i21])
            + (-4.450342e-04) * (q[i10] * q[i22]) + (-1.718031e-04) * (q[i11] * q[i12]) + (-5.593809e-04) * (q[i11] * q[i15])
            + (-2.979092e-04) * (q[i11] * q[i16]) + (2.243249e-04) * (q[i11] * q[i19]) + (2.009443e-05) * (q[i11] * q[i21])
            + (-1.081577e-04) * (q[i11] * q[i22]) + (1.066293e-04) * (q[i12] * q[i15]) + (6.358665e-04) * (q[i12] * q[i16]) + (2.304211e-04) * (q[i12] * q[i19])
            + (-4.651508e-04) * (q[i12] * q[i21]) + (-1.291617e-03) * (q[i12] * q[i22]) + (-3.754970e-04) * (q[i15] * q[i16])
            + (-1.729091e-04) * (q[i15] * q[i19]) + (-7.171487e-05) * (q[i15] * q[i21]) + (8.631228e-05) * (q[i15] * q[i22])
            + (1.727719e-04) * (q[i16] * q[i19]) + (-3.464083e-04) * (q[i16] * q[i21]) + (3.163393e-04) * (q[i16] * q[i22])
            + (-1.371422e-04) * (q[i19] * q[i21]) + (-1.367320e-04) * (q[i19] * q[i22]) + (6.553016e-05) * (q[i21] * q[i22]);
   }

   public void getJQz21(double[] q, double[][] JQ)
   {
      JQ[3][i21] = (6.412442e-03) * (1) + (6.871610e-04) * ((2) * q[i21]) + (-3.807450e-03) * (q[i0]) + (2.742735e-04) * (q[i1]) + (-1.734093e-03) * (q[i2])
            + (4.898164e-03) * (q[i3]) + (2.181625e-03) * (q[i4]) + (-3.430954e-03) * (q[i5]) + (6.853790e-04) * (q[i6]) + (7.724307e-04) * (q[i7])
            + (1.498093e-03) * (q[i8]) + (7.322895e-04) * (q[i9]) + (1.629888e-03) * (q[i10]) + (5.455647e-03) * (q[i11]) + (-9.732808e-04) * (q[i12])
            + (3.836362e-03) * (q[i15]) + (-8.271314e-05) * (q[i16]) + (-5.192123e-03) * (q[i19]) + (-1.033220e-03) * (q[i20]) + (-5.880013e-06) * (q[i22])
            + (8.216099e-04) * (q[i0] * q[i0]) + (1.014259e-03) * (q[i1] * q[i1]) + (6.728289e-04) * (q[i2] * q[i2]) + (-1.159429e-04) * (q[i3] * q[i3])
            + (1.868968e-03) * (q[i4] * q[i4]) + (5.800218e-04) * (q[i5] * q[i5]) + (-7.256285e-04) * (q[i6] * q[i6]) + (-1.584634e-03) * (q[i7] * q[i7])
            + (1.297573e-03) * (q[i8] * q[i8]) + (6.274636e-05) * (q[i9] * q[i9]) + (-2.492530e-04) * (q[i10] * q[i10]) + (-8.182068e-04) * (q[i11] * q[i11])
            + (1.117641e-04) * (q[i12] * q[i12]) + (1.243250e-03) * (q[i15] * q[i15]) + (-5.805207e-05) * (q[i16] * q[i16])
            + (-1.887431e-03) * (q[i19] * q[i19]) + (8.445291e-05) * (q[i20] * q[i20]) + (-1.631851e-03) * ((2) * q[i0] * q[i21])
            + (3.043680e-04) * ((2) * q[i1] * q[i21]) + (-1.526069e-03) * ((2) * q[i2] * q[i21]) + (-2.297460e-04) * ((2) * q[i3] * q[i21])
            + (1.052418e-03) * ((2) * q[i4] * q[i21]) + (2.817858e-04) * ((2) * q[i5] * q[i21]) + (1.271774e-03) * ((2) * q[i6] * q[i21])
            + (-1.401571e-03) * ((2) * q[i7] * q[i21]) + (2.904282e-04) * ((2) * q[i8] * q[i21]) + (2.991626e-04) * ((2) * q[i9] * q[i21])
            + (7.848120e-05) * ((2) * q[i10] * q[i21]) + (-3.634334e-04) * ((2) * q[i11] * q[i21]) + (-1.363000e-04) * ((2) * q[i12] * q[i21])
            + (2.829903e-04) * ((2) * q[i15] * q[i21]) + (-1.146371e-04) * ((2) * q[i16] * q[i21]) + (-1.248131e-03) * ((2) * q[i19] * q[i21])
            + (-2.553853e-05) * ((2) * q[i20] * q[i21]) + (-2.864912e-04) * ((3) * q[i21] * q[i21]) + (9.754488e-05) * ((2) * q[i21] * q[i22])
            + (9.629361e-05) * (q[i22] * q[i22]) + (-9.051953e-04) * (q[i0] * q[i1]) + (1.036193e-04) * (q[i0] * q[i2]) + (1.223852e-03) * (q[i0] * q[i3])
            + (-2.991066e-04) * (q[i0] * q[i4]) + (-1.545775e-03) * (q[i0] * q[i5]) + (1.146224e-03) * (q[i0] * q[i6]) + (-7.828759e-04) * (q[i0] * q[i7])
            + (6.531737e-04) * (q[i0] * q[i8]) + (-9.313780e-05) * (q[i0] * q[i9]) + (1.549669e-04) * (q[i0] * q[i10]) + (2.935356e-04) * (q[i0] * q[i11])
            + (4.288031e-04) * (q[i0] * q[i12]) + (-1.686942e-03) * (q[i0] * q[i15]) + (-4.163082e-04) * (q[i0] * q[i16]) + (-6.207025e-06) * (q[i0] * q[i19])
            + (-7.676975e-04) * (q[i0] * q[i20]) + (3.870924e-04) * (q[i0] * q[i22]) + (3.816037e-04) * (q[i1] * q[i2]) + (-1.828656e-03) * (q[i1] * q[i3])
            + (-1.502616e-03) * (q[i1] * q[i4]) + (-1.623950e-03) * (q[i1] * q[i5]) + (6.942189e-04) * (q[i1] * q[i6]) + (-8.601597e-04) * (q[i1] * q[i7])
            + (-4.188210e-04) * (q[i1] * q[i8]) + (3.461663e-04) * (q[i1] * q[i9]) + (3.046122e-05) * (q[i1] * q[i10]) + (-2.952990e-05) * (q[i1] * q[i11])
            + (-2.075818e-04) * (q[i1] * q[i12]) + (-5.695855e-04) * (q[i1] * q[i15]) + (-3.615603e-04) * (q[i1] * q[i16]) + (-3.969437e-04) * (q[i1] * q[i19])
            + (4.882128e-04) * (q[i1] * q[i20]) + (3.737421e-04) * (q[i1] * q[i22]) + (8.494775e-04) * (q[i2] * q[i3]) + (-1.327809e-03) * (q[i2] * q[i4])
            + (-1.068426e-03) * (q[i2] * q[i5]) + (-9.109117e-04) * (q[i2] * q[i6]) + (-7.129322e-04) * (q[i2] * q[i7]) + (1.080224e-03) * (q[i2] * q[i8])
            + (-1.082832e-04) * (q[i2] * q[i9]) + (4.605087e-05) * (q[i2] * q[i10]) + (-5.254483e-05) * (q[i2] * q[i11]) + (-1.037075e-04) * (q[i2] * q[i12])
            + (-1.745349e-03) * (q[i2] * q[i15]) + (-3.777557e-05) * (q[i2] * q[i16]) + (-1.674023e-03) * (q[i2] * q[i19]) + (-2.370410e-05) * (q[i2] * q[i20])
            + (8.492379e-04) * (q[i2] * q[i22]) + (-5.361392e-04) * (q[i3] * q[i4]) + (1.755457e-03) * (q[i3] * q[i5]) + (-2.165034e-03) * (q[i3] * q[i6])
            + (2.645970e-03) * (q[i3] * q[i7]) + (6.368776e-04) * (q[i3] * q[i8]) + (-1.350166e-03) * (q[i3] * q[i9]) + (-6.701134e-05) * (q[i3] * q[i10])
            + (-7.055388e-04) * (q[i3] * q[i11]) + (-3.508251e-04) * (q[i3] * q[i12]) + (-6.911407e-06) * (q[i3] * q[i15]) + (1.136429e-04) * (q[i3] * q[i16])
            + (3.786057e-04) * (q[i3] * q[i19]) + (1.415799e-03) * (q[i3] * q[i20]) + (-1.514408e-04) * (q[i3] * q[i22]) + (-1.064690e-03) * (q[i4] * q[i5])
            + (1.227218e-03) * (q[i4] * q[i6]) + (-9.733502e-04) * (q[i4] * q[i7]) + (-1.838598e-03) * (q[i4] * q[i8]) + (6.892232e-04) * (q[i4] * q[i9])
            + (-7.013622e-04) * (q[i4] * q[i10]) + (-1.348688e-05) * (q[i4] * q[i11]) + (3.232380e-04) * (q[i4] * q[i12]) + (-6.219502e-04) * (q[i4] * q[i15])
            + (2.637954e-04) * (q[i4] * q[i16]) + (-2.016544e-03) * (q[i4] * q[i19]) + (1.415346e-03) * (q[i4] * q[i20]) + (-1.554328e-04) * (q[i4] * q[i22])
            + (2.373988e-03) * (q[i5] * q[i6]) + (-7.583026e-04) * (q[i5] * q[i7]) + (7.720520e-05) * (q[i5] * q[i8]) + (9.070192e-04) * (q[i5] * q[i9])
            + (8.840609e-04) * (q[i5] * q[i10]) + (-1.005917e-03) * (q[i5] * q[i11]) + (4.047226e-04) * (q[i5] * q[i12]) + (-3.191987e-04) * (q[i5] * q[i15])
            + (-3.566293e-04) * (q[i5] * q[i16]) + (9.746688e-04) * (q[i5] * q[i19]) + (-7.983432e-04) * (q[i5] * q[i20]) + (7.407358e-04) * (q[i5] * q[i22])
            + (1.910748e-03) * (q[i6] * q[i7]) + (-2.577027e-03) * (q[i6] * q[i8]) + (5.865629e-05) * (q[i6] * q[i9]) + (7.224236e-04) * (q[i6] * q[i10])
            + (1.821923e-03) * (q[i6] * q[i11]) + (-7.942234e-04) * (q[i6] * q[i12]) + (-1.324569e-03) * (q[i6] * q[i15]) + (4.686602e-04) * (q[i6] * q[i16])
            + (-5.003558e-04) * (q[i6] * q[i19]) + (-2.724772e-04) * (q[i6] * q[i20]) + (-4.921650e-04) * (q[i6] * q[i22]) + (5.454105e-05) * (q[i7] * q[i8])
            + (-1.815872e-04) * (q[i7] * q[i9]) + (-8.749950e-04) * (q[i7] * q[i10]) + (-3.867935e-04) * (q[i7] * q[i11]) + (-1.011054e-03) * (q[i7] * q[i12])
            + (-3.455207e-04) * (q[i7] * q[i15]) + (1.154882e-03) * (q[i7] * q[i16]) + (-2.370805e-03) * (q[i7] * q[i19]) + (7.750717e-04) * (q[i7] * q[i20])
            + (4.933090e-04) * (q[i7] * q[i22]) + (-7.147197e-04) * (q[i8] * q[i9]) + (2.566545e-04) * (q[i8] * q[i10]) + (-6.084515e-04) * (q[i8] * q[i11])
            + (1.327864e-03) * (q[i8] * q[i12]) + (2.597813e-03) * (q[i8] * q[i15]) + (-5.266935e-05) * (q[i8] * q[i16]) + (2.748206e-03) * (q[i8] * q[i19])
            + (-7.043087e-04) * (q[i8] * q[i20]) + (1.592038e-06) * (q[i8] * q[i22]) + (7.075739e-05) * (q[i9] * q[i10]) + (5.632330e-04) * (q[i9] * q[i11])
            + (-2.561373e-04) * (q[i9] * q[i12]) + (-5.439368e-04) * (q[i9] * q[i15]) + (3.010307e-04) * (q[i9] * q[i16]) + (-4.528721e-04) * (q[i9] * q[i19])
            + (-5.562106e-05) * (q[i9] * q[i20]) + (-1.108698e-04) * (q[i9] * q[i22]) + (2.774399e-04) * (q[i10] * q[i11]) + (-2.674550e-04) * (q[i10] * q[i12])
            + (-5.644669e-05) * (q[i10] * q[i15]) + (4.160347e-04) * (q[i10] * q[i16]) + (-2.803785e-04) * (q[i10] * q[i19])
            + (2.432303e-04) * (q[i10] * q[i20]) + (1.143337e-04) * (q[i10] * q[i22]) + (-1.055158e-04) * (q[i11] * q[i12]) + (1.712960e-03) * (q[i11] * q[i15])
            + (1.930622e-04) * (q[i11] * q[i16]) + (1.306839e-03) * (q[i11] * q[i19]) + (2.009443e-05) * (q[i11] * q[i20]) + (1.321018e-04) * (q[i11] * q[i22])
            + (6.021189e-05) * (q[i12] * q[i15]) + (-2.030599e-04) * (q[i12] * q[i16]) + (1.028135e-04) * (q[i12] * q[i19])
            + (-4.651508e-04) * (q[i12] * q[i20]) + (1.320225e-04) * (q[i12] * q[i22]) + (-1.755289e-04) * (q[i15] * q[i16])
            + (3.298283e-04) * (q[i15] * q[i19]) + (-7.171487e-05) * (q[i15] * q[i20]) + (1.734183e-04) * (q[i15] * q[i22]) + (8.513549e-05) * (q[i16] * q[i19])
            + (-3.464083e-04) * (q[i16] * q[i20]) + (-1.737211e-04) * (q[i16] * q[i22]) + (-1.371422e-04) * (q[i19] * q[i20])
            + (-6.391255e-05) * (q[i19] * q[i22]) + (6.553016e-05) * (q[i20] * q[i22]);
   }

   public void getJQz22(double[] q, double[][] JQ)
   {
      JQ[3][i22] = (6.439572e-03) * (1) + (-7.022029e-04) * ((2) * q[i22]) + (-3.142940e-04) * (q[i0]) + (3.725649e-03) * (q[i1]) + (1.651024e-03) * (q[i2])
            + (-2.220548e-03) * (q[i3]) + (-4.804226e-03) * (q[i4]) + (3.408243e-03) * (q[i5]) + (7.736524e-04) * (q[i6]) + (6.615569e-04) * (q[i7])
            + (1.520628e-03) * (q[i8]) + (1.643148e-03) * (q[i9]) + (7.292084e-04) * (q[i10]) + (9.597047e-04) * (q[i11]) + (-5.564965e-03) * (q[i12])
            + (-8.313458e-05) * (q[i15]) + (3.876542e-03) * (q[i16]) + (-1.044434e-03) * (q[i19]) + (-5.224340e-03) * (q[i20]) + (-5.880013e-06) * (q[i21])
            + (1.011065e-03) * (q[i0] * q[i0]) + (7.871221e-04) * (q[i1] * q[i1]) + (6.704102e-04) * (q[i2] * q[i2]) + (1.880048e-03) * (q[i3] * q[i3])
            + (-1.108247e-04) * (q[i4] * q[i4]) + (5.814167e-04) * (q[i5] * q[i5]) + (-1.595204e-03) * (q[i6] * q[i6]) + (-7.159547e-04) * (q[i7] * q[i7])
            + (1.305954e-03) * (q[i8] * q[i8]) + (-2.526185e-04) * (q[i9] * q[i9]) + (6.552225e-05) * (q[i10] * q[i10]) + (1.101684e-04) * (q[i11] * q[i11])
            + (-8.072142e-04) * (q[i12] * q[i12]) + (-5.914336e-05) * (q[i15] * q[i15]) + (1.255390e-03) * (q[i16] * q[i16])
            + (8.010234e-05) * (q[i19] * q[i19]) + (-1.884380e-03) * (q[i20] * q[i20]) + (9.754488e-05) * (q[i21] * q[i21])
            + (3.073992e-04) * ((2) * q[i0] * q[i22]) + (-1.623037e-03) * ((2) * q[i1] * q[i22]) + (-1.514669e-03) * ((2) * q[i2] * q[i22])
            + (1.064119e-03) * ((2) * q[i3] * q[i22]) + (-2.329584e-04) * ((2) * q[i4] * q[i22]) + (2.937040e-04) * ((2) * q[i5] * q[i22])
            + (1.395523e-03) * ((2) * q[i6] * q[i22]) + (-1.274019e-03) * ((2) * q[i7] * q[i22]) + (-2.898016e-04) * ((2) * q[i8] * q[i22])
            + (-7.665134e-05) * ((2) * q[i9] * q[i22]) + (-3.000039e-04) * ((2) * q[i10] * q[i22]) + (-1.354532e-04) * ((2) * q[i11] * q[i22])
            + (-3.473568e-04) * ((2) * q[i12] * q[i22]) + (1.136117e-04) * ((2) * q[i15] * q[i22]) + (-2.800429e-04) * ((2) * q[i16] * q[i22])
            + (2.622147e-05) * ((2) * q[i19] * q[i22]) + (1.266536e-03) * ((2) * q[i20] * q[i22]) + (9.629361e-05) * ((2) * q[i21] * q[i22])
            + (-2.891843e-04) * ((3) * q[i22] * q[i22]) + (-8.994086e-04) * (q[i0] * q[i1]) + (3.914916e-04) * (q[i0] * q[i2])
            + (-1.502629e-03) * (q[i0] * q[i3]) + (-1.857760e-03) * (q[i0] * q[i4]) + (-1.651426e-03) * (q[i0] * q[i5]) + (8.591253e-04) * (q[i0] * q[i6])
            + (-6.802556e-04) * (q[i0] * q[i7]) + (4.179213e-04) * (q[i0] * q[i8]) + (-3.259272e-05) * (q[i0] * q[i9]) + (-3.355837e-04) * (q[i0] * q[i10])
            + (-2.104644e-04) * (q[i0] * q[i11]) + (-3.299089e-05) * (q[i0] * q[i12]) + (3.592464e-04) * (q[i0] * q[i15]) + (5.712635e-04) * (q[i0] * q[i16])
            + (-4.942090e-04) * (q[i0] * q[i19]) + (3.850536e-04) * (q[i0] * q[i20]) + (3.870924e-04) * (q[i0] * q[i21]) + (7.688619e-05) * (q[i1] * q[i2])
            + (-2.798875e-04) * (q[i1] * q[i3]) + (1.220969e-03) * (q[i1] * q[i4]) + (-1.506938e-03) * (q[i1] * q[i5]) + (7.825132e-04) * (q[i1] * q[i6])
            + (-1.137522e-03) * (q[i1] * q[i7]) + (-6.807220e-04) * (q[i1] * q[i8]) + (-1.466367e-04) * (q[i1] * q[i9]) + (1.084647e-04) * (q[i1] * q[i10])
            + (4.311889e-04) * (q[i1] * q[i11]) + (2.939070e-04) * (q[i1] * q[i12]) + (4.220954e-04) * (q[i1] * q[i15]) + (1.679648e-03) * (q[i1] * q[i16])
            + (7.699027e-04) * (q[i1] * q[i19]) + (1.876201e-05) * (q[i1] * q[i20]) + (3.737421e-04) * (q[i1] * q[i21]) + (-1.339271e-03) * (q[i2] * q[i3])
            + (8.300638e-04) * (q[i2] * q[i4]) + (-1.038283e-03) * (q[i2] * q[i5]) + (7.190790e-04) * (q[i2] * q[i6]) + (9.094359e-04) * (q[i2] * q[i7])
            + (-1.089511e-03) * (q[i2] * q[i8]) + (-4.297117e-05) * (q[i2] * q[i9]) + (1.157773e-04) * (q[i2] * q[i10]) + (-1.053333e-04) * (q[i2] * q[i11])
            + (-3.184955e-05) * (q[i2] * q[i12]) + (4.714810e-05) * (q[i2] * q[i15]) + (1.742322e-03) * (q[i2] * q[i16]) + (1.799295e-05) * (q[i2] * q[i19])
            + (1.679019e-03) * (q[i2] * q[i20]) + (8.492379e-04) * (q[i2] * q[i21]) + (-5.363010e-04) * (q[i3] * q[i4]) + (-1.061229e-03) * (q[i3] * q[i5])
            + (9.942412e-04) * (q[i3] * q[i6]) + (-1.204513e-03) * (q[i3] * q[i7]) + (1.847937e-03) * (q[i3] * q[i8]) + (7.085870e-04) * (q[i3] * q[i9])
            + (-6.866560e-04) * (q[i3] * q[i10]) + (3.205368e-04) * (q[i3] * q[i11]) + (-1.932807e-05) * (q[i3] * q[i12]) + (-2.669654e-04) * (q[i3] * q[i15])
            + (6.164245e-04) * (q[i3] * q[i16]) + (-1.428575e-03) * (q[i3] * q[i19]) + (2.036116e-03) * (q[i3] * q[i20]) + (-1.514408e-04) * (q[i3] * q[i21])
            + (1.727949e-03) * (q[i4] * q[i5]) + (-2.634513e-03) * (q[i4] * q[i6]) + (2.156502e-03) * (q[i4] * q[i7]) + (-6.206917e-04) * (q[i4] * q[i8])
            + (6.536468e-05) * (q[i4] * q[i9]) + (1.339857e-03) * (q[i4] * q[i10]) + (-3.507120e-04) * (q[i4] * q[i11]) + (-7.104490e-04) * (q[i4] * q[i12])
            + (-9.526688e-05) * (q[i4] * q[i15]) + (3.106142e-05) * (q[i4] * q[i16]) + (-1.405249e-03) * (q[i4] * q[i19]) + (-3.705879e-04) * (q[i4] * q[i20])
            + (-1.554328e-04) * (q[i4] * q[i21]) + (7.504333e-04) * (q[i5] * q[i6]) + (-2.373746e-03) * (q[i5] * q[i7]) + (-1.235063e-04) * (q[i5] * q[i8])
            + (-8.920332e-04) * (q[i5] * q[i9]) + (-9.083725e-04) * (q[i5] * q[i10]) + (4.014403e-04) * (q[i5] * q[i11]) + (-9.949296e-04) * (q[i5] * q[i12])
            + (3.559548e-04) * (q[i5] * q[i15]) + (3.086819e-04) * (q[i5] * q[i16]) + (8.000354e-04) * (q[i5] * q[i19]) + (-9.931816e-04) * (q[i5] * q[i20])
            + (7.407358e-04) * (q[i5] * q[i21]) + (1.899129e-03) * (q[i6] * q[i7]) + (5.465064e-05) * (q[i6] * q[i8]) + (-8.798216e-04) * (q[i6] * q[i9])
            + (-1.862710e-04) * (q[i6] * q[i10]) + (1.017119e-03) * (q[i6] * q[i11]) + (3.878863e-04) * (q[i6] * q[i12]) + (1.150947e-03) * (q[i6] * q[i15])
            + (-3.558965e-04) * (q[i6] * q[i16]) + (7.818853e-04) * (q[i6] * q[i19]) + (-2.384966e-03) * (q[i6] * q[i20]) + (-4.921650e-04) * (q[i6] * q[i21])
            + (-2.572290e-03) * (q[i7] * q[i8]) + (7.214727e-04) * (q[i7] * q[i9]) + (6.418850e-05) * (q[i7] * q[i10]) + (7.851733e-04) * (q[i7] * q[i11])
            + (-1.816761e-03) * (q[i7] * q[i12]) + (4.722414e-04) * (q[i7] * q[i15]) + (-1.324447e-03) * (q[i7] * q[i16]) + (-2.713702e-04) * (q[i7] * q[i19])
            + (-4.975113e-04) * (q[i7] * q[i20]) + (4.933090e-04) * (q[i7] * q[i21]) + (2.601415e-04) * (q[i8] * q[i9]) + (-7.149864e-04) * (q[i8] * q[i10])
            + (-1.323098e-03) * (q[i8] * q[i11]) + (6.132567e-04) * (q[i8] * q[i12]) + (-5.794559e-05) * (q[i8] * q[i15]) + (2.635654e-03) * (q[i8] * q[i16])
            + (-7.039320e-04) * (q[i8] * q[i19]) + (2.734584e-03) * (q[i8] * q[i20]) + (1.592038e-06) * (q[i8] * q[i21]) + (6.639323e-05) * (q[i9] * q[i10])
            + (2.707479e-04) * (q[i9] * q[i11]) + (-2.763025e-04) * (q[i9] * q[i12]) + (4.191311e-04) * (q[i9] * q[i15]) + (-6.196279e-05) * (q[i9] * q[i16])
            + (2.472474e-04) * (q[i9] * q[i19]) + (-2.840447e-04) * (q[i9] * q[i20]) + (-1.108698e-04) * (q[i9] * q[i21]) + (2.573533e-04) * (q[i10] * q[i11])
            + (-5.623680e-04) * (q[i10] * q[i12]) + (3.015409e-04) * (q[i10] * q[i15]) + (-5.392491e-04) * (q[i10] * q[i16])
            + (-5.321534e-05) * (q[i10] * q[i19]) + (-4.450342e-04) * (q[i10] * q[i20]) + (1.143337e-04) * (q[i10] * q[i21])
            + (-1.007766e-04) * (q[i11] * q[i12]) + (2.039076e-04) * (q[i11] * q[i15]) + (-6.535191e-05) * (q[i11] * q[i16])
            + (4.637868e-04) * (q[i11] * q[i19]) + (-1.081577e-04) * (q[i11] * q[i20]) + (1.321018e-04) * (q[i11] * q[i21])
            + (-2.014812e-04) * (q[i12] * q[i15]) + (-1.759297e-03) * (q[i12] * q[i16]) + (-1.867936e-05) * (q[i12] * q[i19])
            + (-1.291617e-03) * (q[i12] * q[i20]) + (1.320225e-04) * (q[i12] * q[i21]) + (-1.772365e-04) * (q[i15] * q[i16])
            + (-3.501043e-04) * (q[i15] * q[i19]) + (8.631228e-05) * (q[i15] * q[i20]) + (1.734183e-04) * (q[i15] * q[i21])
            + (-7.545387e-05) * (q[i16] * q[i19]) + (3.163393e-04) * (q[i16] * q[i20]) + (-1.737211e-04) * (q[i16] * q[i21])
            + (-1.367320e-04) * (q[i19] * q[i20]) + (-6.391255e-05) * (q[i19] * q[i21]) + (6.553016e-05) * (q[i20] * q[i21]);
   }

   //==========================================================================================================================================
}
