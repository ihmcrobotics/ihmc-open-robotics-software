package us.ihmc.atlas.hikSim;

import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.AtlasWholeBodyIK;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.BambooPlan;
import us.ihmc.utilities.code.agileTesting.BambooPlanType;
import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;
import us.ihmc.utilities.humanoidRobot.partNames.LegJointName;
import us.ihmc.utilities.humanoidRobot.partNames.SpineJointName;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.wholeBodyController.WholeBodyHalfCylinderTargetParameters;
import us.ihmc.wholeBodyController.WholeBodyIkSolver;
import us.ihmc.wholeBodyController.WholeBodyIkSolverTest;
import us.ihmc.wholeBodyController.WholeBodyIkSolverTestHelper;

//import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;

@BambooPlan(planType = {BambooPlanType.Fast})
public class AtlasWholeBodyIkSolverTest extends WholeBodyIkSolverTest
{
   // private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   // final static private boolean VISUALIZE_GUI =  simulationTestingParameters.getKeepSCSUp(); // do not commit this to true! will break bamboo
   // private static final boolean VISUALIZE_RANDOMLY_GENERATED_POSES = (true & VISUALIZE_GUI);

   private final ArrayList<Matrix4d> rightHandToWorldArray = new ArrayList<Matrix4d>();
   private final ArrayList<Matrix4d> leftHandToWorldArray = new ArrayList<Matrix4d>();
   private final ArrayList<Matrix4d> rightHandToFootArray = new ArrayList<Matrix4d>();
   private final ArrayList<Matrix4d> leftHandToFootArray = new ArrayList<Matrix4d>();

   @Override
   public WholeBodyIkSolverTestHelper getWholeBodyIkSolverTestHelper(boolean generateRandomHandPoses)
   {
      AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_DUAL_ROBOTIQ, AtlasRobotModel.AtlasTarget.SIM, false);
      SDFFullRobotModel actualRobotModel = atlasRobotModel.createFullRobotModel();
      WholeBodyIkSolver wholeBodySolver = new AtlasWholeBodyIK(atlasRobotModel);

      initializeFullRobotModelJointAngles(actualRobotModel, atlasRobotModel);

      wholeBodySolver.setNumberOfMaximumAutomaticReseeds(20);    // "I am NOT Feeling Lucky"" ( Davide Faconti)

      Robot robot = atlasRobotModel.createSdfRobot(false);

      WholeBodyHalfCylinderTargetParameters wholeBodyHalfCylinderTargetParameters = new WholeBodyHalfCylinderTargetParameters();
      wholeBodyHalfCylinderTargetParameters.setMaxReachRadius(0.8);
      wholeBodyHalfCylinderTargetParameters.setMaxHeight(1.5);
      wholeBodyHalfCylinderTargetParameters.setMaxTheta(Math.PI * 3.0 / 4.0);
      wholeBodyHalfCylinderTargetParameters.setRadiusIncrements(4);
      wholeBodyHalfCylinderTargetParameters.setHeightIncrements(4);
      wholeBodyHalfCylinderTargetParameters.setThetaIncrements(4);

      WholeBodyIkSolverTestHelper ret = new WholeBodyIkSolverTestHelper(atlasRobotModel, actualRobotModel, wholeBodySolver, robot,
                                           wholeBodyHalfCylinderTargetParameters);

      int numberOfPoses = ret.getNumberOfRegressionPoses();

      if (generateRandomHandPoses)
         ret.setRandomHandPosesForRegression(getRandomHandPosesForRegression(numberOfPoses, wholeBodySolver));

      createHandTargetArrays();

      ret.setLeftHandToFootArray(leftHandToFootArray);
      ret.setLeftHandToWorldArray(leftHandToWorldArray);
      ret.setRightHandToFootArray(rightHandToFootArray);
      ret.setRightHandToWorldArray(rightHandToWorldArray);

      return ret;
   }

   private void initializeFullRobotModelJointAngles(SDFFullRobotModel fullRobotModelToInitialize, AtlasRobotModel atlasRobotModel)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.HIP_YAW)).setQ(0.0);    // leg_hpz
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide,
                 LegJointName.HIP_ROLL)).setQ(robotSide.negateIfRightSide(0.1));    // leg_hpx
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.HIP_PITCH)).setQ(-0.5);    // leg_hpy
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.KNEE)).setQ(1.0);    // leg_kny
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.ANKLE_PITCH)).setQ(-0.5);    // leg_aky
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide,
                 LegJointName.ANKLE_ROLL)).setQ(robotSide.negateIfRightSide(-0.1));    // leg_akx
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.SHOULDER_YAW)).setQ(0.500);    // arm_shy
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide,
                 ArmJointName.SHOULDER_ROLL)).setQ(robotSide.negateIfRightSide(-1.0));    // arm_shx
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.ELBOW_PITCH)).setQ(2.00);    // arm_ely
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide,
                 ArmJointName.ELBOW_ROLL)).setQ(robotSide.negateIfRightSide(0.9));    // arm_elx
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.WRIST_PITCH)).setQ(0.000);    // arm_wry
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide,
                 ArmJointName.WRIST_ROLL)).setQ(robotSide.negateIfRightSide(0));    // arm_wrx
      }
   }

   private ArrayList<Pair<FramePose, FramePose>> getRandomHandPosesForRegression(int numberOfPoses, WholeBodyIkSolver wholeBodySolver)
   {
      ArrayList<Pair<FramePose, FramePose>> handTargetArrayListToReturn = new ArrayList<Pair<FramePose, FramePose>>();

      Random random = new Random(10L);
      while (handTargetArrayListToReturn.size() < numberOfPoses)
      {
         AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_DUAL_ROBOTIQ, AtlasRobotModel.AtlasTarget.SIM, false);
         SDFFullRobotModel actualRobotModel = atlasRobotModel.createFullRobotModel();

         Vector3d rootPosition = new Vector3d(0.0, 0.0, 0.93);
         actualRobotModel.getRootJoint().setPosition(rootPosition);
         actualRobotModel.getRootJoint().setRotation(0.0, 0.0, 0.0);

       
         for (RobotSide robotSide : RobotSide.values)
         {
            actualRobotModel.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.HIP_YAW)).setQ(0.0);    // leg_hpz
            actualRobotModel.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide,
                    LegJointName.HIP_ROLL)).setQ(robotSide.negateIfRightSide(0.1));    // leg_hpx
            actualRobotModel.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.HIP_PITCH)).setQ(-0.5);    // leg_hpy
            actualRobotModel.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.KNEE)).setQ(1.0);    // leg_kny
            actualRobotModel.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.ANKLE_PITCH)).setQ(-0.5);    // leg_aky
            actualRobotModel.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide,
                    LegJointName.ANKLE_ROLL)).setQ(robotSide.negateIfRightSide(-0.1));    // leg_akx

            for (ArmJointName armJointName : atlasRobotModel.getJointMap().getArmJointNames())
            {
               OneDoFJoint armJoint = actualRobotModel.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, armJointName));
               double randomJointAngle = RandomTools.generateRandomDouble(random, armJoint.getJointLimitLower(), armJoint.getJointLimitUpper());
               armJoint.setQ(randomJointAngle);
            }
         }

         for (SpineJointName jointName : atlasRobotModel.getJointMap().getSpineJointNames())
         {
            OneDoFJoint spineJoint = actualRobotModel.getOneDoFJointByName(atlasRobotModel.getJointMap().getSpineJointName(jointName));
            spineJoint.setQ(RandomTools.generateRandomDouble(random, spineJoint.getJointLimitUpper(), spineJoint.getJointLimitLower()));
         }

         actualRobotModel.updateFrames();

         double error = wholeBodySolver.calculateCenterOfMassError(actualRobotModel);
         double EPS = 0.01;    // 1 cm tolerance

         boolean isSelfColliding = wholeBodySolver.checkCollisions(actualRobotModel);

         // if CoM is close enough to stable and there are no self collisions then accept the proposed pose as feasible and add it to the list of targets to try with the IK solver.
       if (error < EPS & !isSelfColliding)
//         if ((error > 0.07 &  error < 0.10)& !isSelfColliding)
         {
            Vector3d vector = new Vector3d();
            Quat4d rotation = new Quat4d();

            ReferenceFrame leftHandFrameDesired = actualRobotModel.getHandControlFrame(RobotSide.LEFT);
            ReferenceFrame rightHandFrameDesired = actualRobotModel.getHandControlFrame(RobotSide.RIGHT);

            leftHandFrameDesired.getTransformToWorldFrame().getTranslation(vector);
            leftHandFrameDesired.getTransformToWorldFrame().getRotation(rotation);
            Point3d point = new Point3d(vector);
            FramePose leftHandTarget = new FramePose(ReferenceFrame.getWorldFrame(), point, rotation);

            rightHandFrameDesired.getTransformToWorldFrame().getTranslation(vector);
            rightHandFrameDesired.getTransformToWorldFrame().getRotation(rotation);
            point = new Point3d(vector);
            FramePose rightHandTarget = new FramePose(ReferenceFrame.getWorldFrame(), point, rotation);

            Pair<FramePose, FramePose> pairToPack = new Pair<FramePose, FramePose>(leftHandTarget, rightHandTarget);

            handTargetArrayListToReturn.add(pairToPack);
            //System.out.println("getRandomHandPosesForRegression(): handTargetArrayListToReturn.size()=" + handTargetArrayListToReturn.size());
         }




      }

      return handTargetArrayListToReturn;

   }



   private void createHandTargetArrays()
   {
      // This is a set of 12 hand target points for both left and right hands. A 3p3r IK solution should be possible for each left and right hand set.
      rightHandToWorldArray.add(new Matrix4d(0.823812375151399, -0.49201146856663575, -0.28152777011569086, -0.03534793675069671, 0.10841406974041566,
              -0.350722427138408, 0.93018286835679, -0.4728269985295789, -0.5563987419375882, -0.7968177294093343, -0.2355885100534878, 0.8147717429551471,
              0.0, 0.0, 0.0, 1.0));
      rightHandToFootArray.add(new Matrix4d(0.8285584107789775, -0.4848525860468195, -0.2800159454944078, 0.02197274730901636, 0.1088405782378846,
              -0.351099454817649, 0.9299908071349918, -0.30884210002654444, -0.5492218936425044, -0.8010288026222663, -0.23813477048360707, 0.73037954594352,
              0.0, 0.0, 0.0, 1.0));
      leftHandToWorldArray.add(new Matrix4d(0.8238069750939127, 0.49193912159919667, -0.28166996365751984, 0.3509129855323839, -0.10828065484392745,
              -0.351177963434722, -0.930026525312268, 0.23313046544665755, -0.5564327161240502, 0.796661746689303, -0.23603536554669144, 0.9448569553916497,
              0.0, 0.0, 0.0, 1.0));
      leftHandToFootArray.add(new Matrix4d(0.8286811499838438, 0.48519569941549456, -0.27905677725180533, 0.40667059115801185, -0.10785411411137716,
              -0.3508008687359308, -0.9302183832645783, 0.39735758798886095, -0.5492313189637901, 0.8009518610770716, -0.23837171498831422, 0.8637406088751843,
              0.0, 0.0, 0.0, 1.0));


      rightHandToWorldArray.add(new Matrix4d(0.09729712524901879, -0.9732400037290739, -0.2081758020513136, 0.06571155613444146, -0.26842752614614573,
              -0.22707820814017668, 0.9361528457441688, -0.3960506036061289, -0.9583735871909951, -0.03520486513647136, -0.28333846340619806,
              0.23232813704760774, 0.0, 0.0, 0.0, 1.0));
      rightHandToFootArray.add(new Matrix4d(0.10579079122824, -0.9727626895057886, -0.20625483848086268, 0.1280510603373005, -0.2684738310894408,
              -0.22765713886382505, 0.9359989471919796, -0.2320691094686739, -0.9574602396504308, -0.04364604254454758, -0.28524535484164804,
              0.1488285903572634, 0.0, 0.0, 0.0, 1.0));
      leftHandToWorldArray.add(new Matrix4d(0.8234576381075394, 0.49233921465373304, -0.28199222676609026, 0.35126686192402107, -0.10857213263380854,
              -0.35109017813783555, -0.9300256871885259, 0.23313480067853337, -0.5568928175669887, 0.7964532531977557, -0.2356535703379271, 0.9448630983096067,
              0.0, 0.0, 0.0, 1.0));
      leftHandToFootArray.add(new Matrix4d(0.8283360112991677, 0.48559754369386343, -0.279382315015593, 0.4070243799717608, -0.10814582045333604,
              -0.35071284963896204, -0.930217704957605, 0.3973621493333298, -0.5496944004658046, 0.8007468530419621, -0.23799273820840497, 0.8637498126084475,
              0.0, 0.0, 0.0, 1.0));


      rightHandToWorldArray.add(new Matrix4d(0.9262962932798531, -0.34766073879298726, -0.1452831296397451, 0.7017669433357538, 0.1369722589653251,
              -0.04850782796630113, 0.9893864719612491, -0.37241226658860294, -0.35101820085280566, -0.9363647800552442, 0.0026872555057877888,
              1.0002852795019088, 0.0, 0.0, 0.0, 1.0));
      rightHandToFootArray.add(new Matrix4d(0.9292345308866479, -0.33947141327031244, -0.1458847016658299, 0.7573861633753205, 0.13748158433967098,
              -0.04881473308654128, 0.9893007307189982, -0.20797169012979475, -0.3429606399797584, -0.9393488603314247, 0.0013107321344006768,
              0.92228834942038, 0.0, 0.0, 0.0, 1.0));
      leftHandToWorldArray.add(new Matrix4d(0.8239226893623445, 0.49143581245958035, -0.28220957493698545, -0.1833665376399064, -0.10893557345232126,
              -0.3513533494343883, -0.9298837909533914, 0.4242157840193429, -0.5561334756967394, 0.7968950157162665, -0.2359531545371569, 0.8147577190511159,
              0.0, 0.0, 0.0, 1.0));
      leftHandToFootArray.add(new Matrix4d(0.8287946465623791, 0.4846904804488559, -0.2795971601998102, -0.12656933770285975, -0.10850892729612338,
              -0.35097652270300944, -0.9300759609883209, 0.588113050126055, -0.5489310034299264, 0.8011807652918044, -0.2382942189811855, 0.7289757417308455,
              0.0, 0.0, 0.0, 1.0));


      rightHandToWorldArray.add(new Matrix4d(0.8240134372415762, -0.49196186837862654, -0.2810255776735166, 0.5101310953187642, 0.10790599956838462,
              -0.35066175782186026, 0.9302648154468864, -0.15304384922889058, -0.5561997397540479, -0.7968750539844331, -0.2358643632150384,
              0.8849012650972204, 0.0, 0.0, 0.0, 1.0));
      rightHandToFootArray.add(new Matrix4d(0.8287580354284896, -0.4848025047860898, -0.2795114488994039, 0.5666321597006573, 0.10833267424403875,
              -0.35103878154548807, 0.930073010866455, 0.01127095511713566, -0.5490210837516586, -0.801085704031842, -0.2384062591291638, 0.8052163499258987,
              0.0, 0.0, 0.0, 1.0));
      leftHandToWorldArray.add(new Matrix4d(0.20166858583880823, 0.9379976455087098, -0.28194006190304216, -0.040803308277168805, 0.18286684748424611,
              -0.3188513115440312, -0.9299965361321638, 0.49061746276734475, -0.9622315197378166, 0.1359935959770406, -0.23583096547375665,
              0.22356000118775649, 0.0, 0.0, 0.0, 1.0));
      leftHandToFootArray.add(new Matrix4d(0.20992558500347203, 0.9369667904467358, -0.27932862789354834, 0.021093402684903985, 0.182881722785262,
              -0.31828218965165117, -0.9301885417600257, 0.6545349000105645, -0.9604610998015715, 0.144186273100084, -0.2381696756882849, 0.1390333189430167,
              0.0, 0.0, 0.0, 1.0));


      rightHandToWorldArray.add(new Matrix4d(-0.1695119814411824, -0.14296021199354705, -0.975104130816111, 0.5960608206600292, 0.9846118539875215,
              -0.06722184184049022, -0.16130939515981937, 0.0199591160133695, -0.04248747033113124, -0.987442961272362, 0.15215522698392353,
              0.9783755035882715, 0.0, 0.0, 0.0, 1.0));
      rightHandToFootArray.add(new Matrix4d(-0.1697167508033204, -0.13432318711105826, -0.9762958086057169, 0.6516432102763037, 0.9845068988502403,
              -0.06741331461258626, -0.16186911728067832, 0.18433478814747659, -0.04407256077249267, -0.988641859531181, 0.14368327310331944,
              0.8994150715788026, 0.0, 0.0, 0.0, 1.0));
      leftHandToWorldArray.add(new Matrix4d(0.823251608865808, 0.49280466138852846, -0.2817806845999154, -0.03458377393004691, -0.10823806780859671,
              -0.3510037354245366, -0.9300972521167257, 0.4633403152767809, -0.5572623342528371, 0.7962034560536395, -0.23562416555039403, 0.8148764671234034,
              0.0, 0.0, 0.0, 1.0));
      leftHandToFootArray.add(new Matrix4d(0.828133004544372, 0.4860650903537192, -0.2791710134017186, 0.022183646431709687, -0.1078119331646557,
              -0.350626169370374, -0.9302891359249327, 0.627325509764975, -0.5500657359367256, 0.8005011038668773, -0.23796148607768414, 0.7303845846355767,
              0.0, 0.0, 0.0, 1.0));

      rightHandToWorldArray.add(new Matrix4d(0.6276473623123782, -0.4925578662261432, -0.6028644433046811, 0.39071993990493853, 0.49089230137492734,
              -0.35063093989636274, 0.7975479248535607, -0.17197957876090733, -0.604221430464976, -0.7965203653429974, 0.02172027994101293, 1.0128246755110586,
              0.0, 0.0, 0.0, 1.0));
      rightHandToFootArray.add(new Matrix4d(0.632591125744892, -0.48540145775144217, -0.6035013607619996, 0.44612369993820733, 0.49119789175305684,
              -0.3510084576212115, 0.7971936363367002, -0.0077213165731045506, -0.598793035001379, -0.8007362159232924, 0.01638333734878842,
              0.9320980099528423, 0.0, 0.0, 0.0, 1.0));
      leftHandToWorldArray.add(new Matrix4d(0.6167714771970556, 0.49198106001792985, -0.6144490064275442, 0.345027813750304, -0.5052683887278109,
              -0.35110863501137296, -0.7883061472377676, 0.25455012272989685, -0.6035700458675562, 0.7966664063484069, 0.03202868600725344, 1.0127852202414223,
              0.0, 0.0, 0.0, 1.0));
      leftHandToFootArray.add(new Matrix4d(0.6222980948084975, 0.48523769226295277, -0.6142389300630808, 0.400181842297594, -0.5049689790948718,
              -0.3507313157368292, -0.7886658825595262, 0.41878128396875286, -0.5981232409375158, 0.8009568815915357, 0.026770552507668313, 0.9316125713405345,
              0.0, 0.0, 0.0, 1.0));

      rightHandToWorldArray.add(new Matrix4d(0.8236516130241986, -0.4925343900078735, -0.2810834307144546, 0.13611541227932822, 0.10783136954603127,
              -0.3505882405103596, 0.9303011777686179, -0.7065096797442678, -0.5567498685266689, -0.7965536769580704, -0.23565191198060526, 0.8147145145019492,
              0.0, 0.0, 0.0, 1.0));
      rightHandToFootArray.add(new Matrix4d(0.8284010565903457, -0.4853778560882402, -0.27957114704343594, 0.19356794090219648, 0.10825775196464857,
              -0.3509655523176656, 0.930109370034469, -0.5424234174384698, -0.549574333989097, -0.8007693287742353, -0.23819431881323477, 0.7318406692471304,
              0.0, 0.0, 0.0, 1.0));
      leftHandToWorldArray.add(new Matrix4d(0.8234671945389558, 0.49266712279171887, -0.28139098357671477, 0.18328374403935727, -0.1079866990755339,
              -0.35079949697113094, -0.9302035184558122, 0.6313352493780738, -0.5569925065392817, 0.7963785651591405, -0.23567038128317566, 0.8150466887897028,
              0.0, 0.0, 0.0, 1.0));
      leftHandToFootArray.add(new Matrix4d(0.8283460843456485, 0.4859259110853176, -0.2787808700128757, 0.2398832911555768, -0.10756041297978683,
              -0.3504219991162724, -0.9303951741571824, 0.7954491620018408, -0.549794072456997, 0.8006749849166387, -0.23800429916262264, 0.7324200552910664,
              0.0, 0.0, 0.0, 1.0));

      rightHandToWorldArray.add(new Matrix4d(0.8236912600011814, -0.49258441750056015, -0.28087951123800464, 0.203399628512669, 0.10760982620217245,
              -0.35055175776725606, 0.9303405776547776, -0.4432755214542159, -0.5567340779065035, -0.7965387980267633, -0.2357394954938064, 0.814689404737201,
              0.0, 0.0, 0.0, 1.0));
      rightHandToFootArray.add(new Matrix4d(0.8284406944204825, -0.4854280401581863, -0.27936648627974586, 0.2606945125018025, 0.10803622358665432,
              -0.3509290920704725, 0.9301488841748552, -0.2791495533136561, -0.549558177285393, -0.8007548876945904, -0.2382801284454504, 0.7323711031889578,
              0.0, 0.0, 0.0, 1.0));
      leftHandToWorldArray.add(new Matrix4d(0.08545439619835085, 0.06402979088621721, -0.9942825212430534, 0.45850577509321533, -0.9959216236141816,
              0.034478064857394075, -0.08337495224215669, 0.26317903430126854, 0.02894245649677772, 0.9973522190895034, 0.06671495551355418, 1.176316490154357,
              0.0, 0.0, 0.0, 1.0));
      leftHandToFootArray.add(new Matrix4d(0.08578687295117883, 0.05532886982226411, -0.994776019309598, 0.512227589192328, -0.9958678099399496,
              0.034623970125056025, -0.08395526081305299, 0.4274948115607019, 0.0297979454772781, 0.9978676750235826, 0.05807052254253431, 1.0961240237271637,
              0.0, 0.0, 0.0, 1.0));

      rightHandToWorldArray.add(new Matrix4d(-0.030655672262984407, -0.2937921723890779, -0.9553776160246852, 0.3876194248996444, 0.999097442585265,
              0.019110555330001996, -0.037935298793630494, -0.1449246734889309, 0.02940289063489455, -0.9556782649604864, 0.29294116116453706,
              1.1264249067894079, 0.0, 0.0, 0.0, 1.0));
      rightHandToFootArray.add(new Matrix4d(-0.03149978737670745, -0.28547674433961523, -0.9578678363096218, 0.4420188217359727, 0.9990823327764906,
              0.01883338034132465, -0.03846811821242208, 0.019343884767590667, 0.02902164242609446, -0.9582005699362599, 0.2846215242119244, 1.045663849100821,
              0.0, 0.0, 0.0, 1.0));
      leftHandToWorldArray.add(new Matrix4d(0.08547442186680355, 0.06439195271871097, -0.9942574111524678, 0.4347797612325888, -0.9959257431634769,
              0.034307907974750904, -0.08339593248278204, 0.20607894151509376, 0.028740864823668916, 0.9973347702128247, 0.06706205196473093,
              1.176577518780454, 0.0, 0.0, 0.0, 1.0));
      leftHandToFootArray.add(new Matrix4d(0.085808661645227, 0.0556912724827643, -0.9947539171855048, 0.4885338947403559, -0.9958719388802268,
              0.03445402577935903, -0.08397619578507794, 0.3703807398884125, 0.029596535905295103, 0.9978533971866039, 0.05841782934653435, 1.0961850712627446,
              0.0, 0.0, 0.0, 1.0));

      rightHandToWorldArray.add(new Matrix4d(0.8236065291283172, -0.4926239743883254, -0.2810585437859112, 0.1985281854213744, 0.10782518840548785,
              -0.35051404878128883, 0.9303298502962637, -0.3118582005016258, -0.5568177564720144, -0.7965309293836605, -0.23556838627598942, 1.134665633483067,
              0.0, 0.0, 0.0, 1.0));
      rightHandToFootArray.add(new Matrix4d(0.8283565651959268, -0.4854676841074383, -0.2795470060368509, 0.25296155685397553, 0.10825153615591393,
              -0.3508914115590835, 0.9301380662105803, -0.1477004591750844, -0.5496426164488081, -0.8007473665154534, -0.23811058187714845, 1.052277902003901,
              0.0, 0.0, 0.0, 1.0));
      leftHandToWorldArray.add(new Matrix4d(0.823486455317593, 0.49232886183689556, -0.2819261422976824, 0.017874646251496115, -0.10836330916554024,
              -0.3512870342111075, -0.929975705500879, 0.32460274424062546, -0.5568908790195433, 0.7963728469740484, -0.23592972993518488, 0.8148627051060824,
              0.0, 0.0, 0.0, 1.0));
      leftHandToFootArray.add(new Matrix4d(0.8283646786743211, 0.4855880055235528, -0.27931388797631834, 0.04897374001488973, -0.10793700127829914,
              -0.3509097362983995, -0.9301677056993733, 0.4886037440387385, -0.5496922437872284, 0.8006663761284385, -0.23826831777959195, 0.7306189091917638,
              0.0, 0.0, 0.0, 1.0));

   }




// 
// 
// 
//


   /*
    * public void initializeSDFRobotlJointAngles(SDFRobot scsRobot) { for
    * (RobotSide robotSide : RobotSide.values) {
    * scsRobot.getOneDegreeOfFreedomJoint
    * (atlasRobotModel.getJointMap().getLegJointName(robotSide,
    * LegJointName.HIP_YAW)).setQ(0.0); //leg_hpz
    * scsRobot.getOneDegreeOfFreedomJoint
    * (atlasRobotModel.getJointMap().getLegJointName(robotSide,
    * LegJointName.HIP_ROLL)).setQ( robotSide.negateIfRightSide(0.1)); //leg_hpx
    * scsRobot
    * .getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().getLegJointName
    * (robotSide, LegJointName.HIP_PITCH)).setQ(-0.4); //leg_hpy
    * scsRobot.getOneDegreeOfFreedomJoint
    * (atlasRobotModel.getJointMap().getLegJointName(robotSide,
    * LegJointName.KNEE)).setQ(0.8); //leg_kny
    * scsRobot.getOneDegreeOfFreedomJoint
    * (atlasRobotModel.getJointMap().getLegJointName(robotSide,
    * LegJointName.ANKLE_PITCH)).setQ(-0.4); //leg_aky
    * scsRobot.getOneDegreeOfFreedomJoint
    * (atlasRobotModel.getJointMap().getLegJointName(robotSide,
    * LegJointName.ANKLE_ROLL)).setQ( robotSide.negateIfRightSide(-0.1));
    * //leg_akx
    *
    * scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().
    * getArmJointName(robotSide, ArmJointName.SHOULDER_YAW)).setQ(0.300);
    * //arm_shy
    * scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap()
    * .getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL)).setQ(
    * robotSide.negateIfRightSide(-1.3)); //arm_shx
    * scsRobot.getOneDegreeOfFreedomJoint
    * (atlasRobotModel.getJointMap().getArmJointName(robotSide,
    * ArmJointName.ELBOW_PITCH)).setQ(2.00); //arm_ely
    * scsRobot.getOneDegreeOfFreedomJoint
    * (atlasRobotModel.getJointMap().getArmJointName(robotSide,
    * ArmJointName.ELBOW_ROLL)).setQ( robotSide.negateIfRightSide( -0.8));
    * //arm_elx
    * scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap()
    * .getArmJointName(robotSide, ArmJointName.WRIST_PITCH)).setQ(0.000);
    * //arm_wry
    * scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap()
    * .getArmJointName(robotSide, ArmJointName.WRIST_ROLL)).setQ(0.000);
    * //arm_wrx } }
    */

}
