package us.ihmc.atlas.hikSim;

import java.util.ArrayList;

import javax.vecmath.Matrix4d;

import org.junit.Test;

import us.ihmc.SdfLoader.FullRobotModelVisualizer;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.AtlasWholeBodyIK;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.code.unitTesting.BambooAnnotations.AverageDuration;
import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;
import us.ihmc.utilities.humanoidRobot.partNames.LegJointName;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.wholeBodyController.WholeBodyIkSolver;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ControlledDoF;
import us.ihmc.wholeBodyController.WholeBodyIkSolverTestFactory;

public class AtlasWholeBodyIkSolverTest extends WholeBodyIkSolverTestFactory
{
   static private final AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_DUAL_ROBOTIQ, AtlasRobotModel.AtlasTarget.SIM, false);
   static private SDFFullRobotModel actualRobotModel = atlasRobotModel.createFullRobotModel();
   static private WholeBodyIkSolver wholeBodySolver = new AtlasWholeBodyIK(atlasRobotModel);

   static private SimulationConstructionSet scs;
   static private boolean VISUALIZE_GUI = false;
   static FullRobotModelVisualizer modelVisualizer;
   private ArrayList<Matrix4d> RightHandToWorldArray = new ArrayList<Matrix4d>();
   private ArrayList<Matrix4d> LeftHandToWorldArray = new ArrayList<Matrix4d>();
   private ArrayList<Matrix4d> RightHandToFootArray = new ArrayList<Matrix4d>();
   private ArrayList<Matrix4d> LeftHandToFootArray = new ArrayList<Matrix4d>();

   public AtlasWholeBodyIkSolverTest() throws InterruptedException
   {
      super(actualRobotModel, wholeBodySolver);

      createHandTargetArrays();
      
      if (scs == null && VISUALIZE_GUI)
      {
         scs = new SimulationConstructionSet(atlasRobotModel.createSdfRobot(false));
         modelVisualizer = new FullRobotModelVisualizer(scs, actualRobotModel, 0.01);
         scs.startOnAThread();

         Thread.sleep(3000);
      }
   }

   private void createHandTargetArrays()
   {
      // This is a set of 12 hand target points for both left and right hands. A 3p3r IK solution should be possible for each left and right hand set.
      RightHandToWorldArray.add(new Matrix4d( 0.823812375151399, -0.49201146856663575, -0.28152777011569086, -0.03534793675069671, 0.10841406974041566, -0.350722427138408, 0.93018286835679, -0.4728269985295789, -0.5563987419375882, -0.7968177294093343, -0.2355885100534878, 0.8147717429551471,             0.0, 0.0, 0.0, 1.0         ));     
      RightHandToFootArray.add(new Matrix4d( 0.8285584107789775, -0.4848525860468195, -0.2800159454944078, 0.02197274730901636, 0.1088405782378846, -0.351099454817649, 0.9299908071349918, -0.30884210002654444, -0.5492218936425044, -0.8010288026222663, -0.23813477048360707, 0.73037954594352,               0.0, 0.0, 0.0, 1.0             ));
      LeftHandToWorldArray.add(new Matrix4d( 0.8238069750939127, 0.49193912159919667, -0.28166996365751984, 0.3509129855323839, -0.10828065484392745, -0.351177963434722, -0.930026525312268, 0.23313046544665755, -0.5564327161240502, 0.796661746689303, -0.23603536554669144, 0.9448569553916497,             0.0, 0.0, 0.0, 1.0          ));
      LeftHandToFootArray.add(new Matrix4d( 0.8286811499838438, 0.48519569941549456, -0.27905677725180533, 0.40667059115801185, -0.10785411411137716, -0.3508008687359308, -0.9302183832645783, 0.39735758798886095, -0.5492313189637901, 0.8009518610770716, -0.23837171498831422, 0.8637406088751843,          0.0, 0.0, 0.0, 1.0       ));
      
      RightHandToWorldArray.add(new Matrix4d( 0.09729712524901879, -0.9732400037290739, -0.2081758020513136, 0.06571155613444146, -0.26842752614614573, -0.22707820814017668, 0.9361528457441688, -0.3960506036061289, -0.9583735871909951, -0.03520486513647136, -0.28333846340619806, 0.23232813704760774,   0.0, 0.0, 0.0, 1.0 ));
      RightHandToFootArray.add(new Matrix4d( 0.10579079122824, -0.9727626895057886, -0.20625483848086268, 0.1280510603373005, -0.2684738310894408, -0.22765713886382505, 0.9359989471919796, -0.2320691094686739, -0.9574602396504308, -0.04364604254454758, -0.28524535484164804, 0.1488285903572634,            0.0, 0.0, 0.0, 1.0        ));
      LeftHandToWorldArray.add(new Matrix4d( 0.8234576381075394, 0.49233921465373304, -0.28199222676609026, 0.35126686192402107, -0.10857213263380854, -0.35109017813783555, -0.9300256871885259, 0.23313480067853337, -0.5568928175669887, 0.7964532531977557, -0.2356535703379271, 0.9448630983096067,       0.0, 0.0, 0.0, 1.0      ));
      LeftHandToFootArray.add(new Matrix4d( 0.8283360112991677, 0.48559754369386343, -0.279382315015593, 0.4070243799717608, -0.10814582045333604, -0.35071284963896204, -0.930217704957605, 0.3973621493333298, -0.5496944004658046, 0.8007468530419621, -0.23799273820840497, 0.8637498126084475,           0.0, 0.0, 0.0, 1.0           ));
      
      RightHandToWorldArray.add(new Matrix4d( 0.9262962932798531, -0.34766073879298726, -0.1452831296397451, 0.7017669433357538, 0.1369722589653251, -0.04850782796630113, 0.9893864719612491, -0.37241226658860294, -0.35101820085280566, -0.9363647800552442, 0.0026872555057877888, 1.0002852795019088,    0.0, 0.0, 0.0, 1.0    ));
      RightHandToFootArray.add(new Matrix4d( 0.9292345308866479, -0.33947141327031244, -0.1458847016658299, 0.7573861633753205, 0.13748158433967098, -0.04881473308654128, 0.9893007307189982, -0.20797169012979475, -0.3429606399797584, -0.9393488603314247, 0.0013107321344006768, 0.92228834942038,       0.0, 0.0, 0.0, 1.0         ));
      LeftHandToWorldArray.add(new Matrix4d( 0.8239226893623445, 0.49143581245958035, -0.28220957493698545, -0.1833665376399064, -0.10893557345232126, -0.3513533494343883, -0.9298837909533914, 0.4242157840193429, -0.5561334756967394, 0.7968950157162665, -0.2359531545371569, 0.8147577190511159,       0.0, 0.0, 0.0, 1.0        ));
      LeftHandToFootArray.add(new Matrix4d( 0.8287946465623791, 0.4846904804488559, -0.2795971601998102, -0.12656933770285975, -0.10850892729612338, -0.35097652270300944, -0.9300759609883209, 0.588113050126055, -0.5489310034299264, 0.8011807652918044, -0.2382942189811855, 0.7289757417308455,         0.0, 0.0, 0.0, 1.0          ));
 
      RightHandToWorldArray.add(new Matrix4d( 0.8240134372415762, -0.49196186837862654, -0.2810255776735166, 0.5101310953187642, 0.10790599956838462, -0.35066175782186026, 0.9302648154468864, -0.15304384922889058, -0.5561997397540479, -0.7968750539844331, -0.2358643632150384, 0.8849012650972204,     0.0, 0.0, 0.0, 1.0      ));
      RightHandToFootArray.add(new Matrix4d( 0.8287580354284896, -0.4848025047860898, -0.2795114488994039, 0.5666321597006573, 0.10833267424403875, -0.35103878154548807, 0.930073010866455, 0.01127095511713566, -0.5490210837516586, -0.801085704031842, -0.2384062591291638, 0.8052163499258987,          0.0, 0.0, 0.0, 1.0           ));
      LeftHandToWorldArray.add(new Matrix4d( 0.20166858583880823, 0.9379976455087098, -0.28194006190304216, -0.040803308277168805, 0.18286684748424611, -0.3188513115440312, -0.9299965361321638, 0.49061746276734475, -0.9622315197378166, 0.1359935959770406, -0.23583096547375665, 0.22356000118775649,    0.0, 0.0, 0.0, 1.0   ));
      LeftHandToFootArray.add(new Matrix4d( 0.20992558500347203, 0.9369667904467358, -0.27932862789354834, 0.021093402684903985, 0.182881722785262, -0.31828218965165117, -0.9301885417600257, 0.6545349000105645, -0.9604610998015715, 0.144186273100084, -0.2381696756882849, 0.1390333189430167,          0.0, 0.0, 0.0, 1.0           ));
                                                                                                                                                                                                                                                                                                                            
      RightHandToWorldArray.add(new Matrix4d( 0.8237657601646099, -0.4921807164142774, -0.28136829027157756, 0.2003427696597494, 0.10778159587711125, -0.35128904251118603, 0.9300425453718535, -0.36261359001684684, -0.5565906035593893, -0.7964635277283586, -0.23633186206665424, 1.3649267574362924,    0.0, 0.0, 0.0, 1.0     ));
      RightHandToFootArray.add(new Matrix4d( 0.82851384270625, -0.48502456101055946, -0.27984993775349887, 0.2528024659968673, 0.10820807157754725, -0.351666143722229, 0.929850491533453, -0.1984298096293975, -0.5494140748921451, -0.8006760259780793, -0.23887669567029315, 1.2825519401707857,          0.0, 0.0, 0.0, 1.0           ));
      LeftHandToWorldArray.add(new Matrix4d( 0.20167327966568876, 0.9379883295949463, -0.2819676963990144, 0.20704582212416844, 0.18274047973956692, -0.3188619889744166, -0.9300177143752859, 0.45757069857009924, -0.962254542900909, 0.13603281049423474, -0.23571438041749024, 0.2228019671343296,       0.0, 0.0, 0.0, 1.0        ));
      LeftHandToFootArray.add(new Matrix4d( 0.20993055442655228, 0.9369571360413465, -0.27935727579455927, 0.2689592072870208, 0.18275535356153746, -0.3182928807193786, -0.9302097198094395, 0.6216345275692433, -0.9604840670530446, 0.1442254045047404, -0.23805333358868497, 0.1404356250148577,         0.0, 0.0, 0.0, 1.0          ));
      
      RightHandToWorldArray.add(new Matrix4d( 0.8236403439446706, -0.49260895954582773, -0.28098575907309215, 0.1873812367498332, 0.1077019878277068, -0.3505798581772309, 0.9303193241346718, -0.347368499624809, -0.5567915818730247, -0.7965112529121035, -0.23569675080434405, 1.4248767829807938,         0.0, 0.0, 0.0, 1.0        ));
      RightHandToFootArray.add(new Matrix4d( 0.8283902267760537, -0.48545280178349465, -0.27947309248382707, 0.23931079396582594, 0.10812835163091136, -0.35095720617625875, 0.9301275713613356, -0.18318589235096683, -0.5496161312730008, -0.8007275545861459, -0.23823830836158416, 1.342385174067586,     0.0, 0.0, 0.0, 1.0      ));
      LeftHandToWorldArray.add(new Matrix4d( 0.3568174342852245, -0.8907943951285795, -0.2813657125479534, 0.03986091896162129, -0.32967952676525963, 0.16173540165864497, -0.9301360489101319, 0.5007933276670093, 0.8740667756081018, 0.4246492734690886, -0.23596666357820784, 1.4444050336875836,         0.0, 0.0, 0.0, 1.0         ));
      LeftHandToFootArray.add(new Matrix4d( 0.34939294831351814, -0.8945508893151992, -0.2787530701071713, 0.09112577742653717, -0.3293738959492636, 0.16125498637480967, -0.9303277196969162, 0.6648907230304257, 0.8771758115315257, 0.41686392959171487, -0.23830035641887456, 1.360532827565789,          0.0, 0.0, 0.0, 1.0           ));
      
      RightHandToWorldArray.add(new Matrix4d( -0.1695119814411824, -0.14296021199354705, -0.975104130816111, 0.5960608206600292, 0.9846118539875215, -0.06722184184049022, -0.16130939515981937, 0.0199591160133695, -0.04248747033113124, -0.987442961272362, 0.15215522698392353, 0.9783755035882715,      0.0, 0.0, 0.0, 1.0       ));
      RightHandToFootArray.add(new Matrix4d( -0.1697167508033204, -0.13432318711105826, -0.9762958086057169, 0.6516432102763037, 0.9845068988502403, -0.06741331461258626, -0.16186911728067832, 0.18433478814747659, -0.04407256077249267, -0.988641859531181, 0.14368327310331944, 0.8994150715788026,     0.0, 0.0, 0.0, 1.0      ));
      LeftHandToWorldArray.add(new Matrix4d( 0.823251608865808, 0.49280466138852846, -0.2817806845999154, -0.03458377393004691, -0.10823806780859671, -0.3510037354245366, -0.9300972521167257, 0.4633403152767809, -0.5572623342528371, 0.7962034560536395, -0.23562416555039403, 0.8148764671234034,       0.0, 0.0, 0.0, 1.0        ));
      LeftHandToFootArray.add(new Matrix4d( 0.828133004544372, 0.4860650903537192, -0.2791710134017186, 0.022183646431709687, -0.1078119331646557, -0.350626169370374, -0.9302891359249327, 0.627325509764975, -0.5500657359367256, 0.8005011038668773, -0.23796148607768414, 0.7303845846355767,            0.0, 0.0, 0.0, 1.0             ));
      
      RightHandToWorldArray.add(new Matrix4d( 0.6276473623123782, -0.4925578662261432, -0.6028644433046811, 0.39071993990493853, 0.49089230137492734, -0.35063093989636274, 0.7975479248535607, -0.17197957876090733, -0.604221430464976, -0.7965203653429974, 0.02172027994101293, 1.0128246755110586,      0.0, 0.0, 0.0, 1.0       ));
      RightHandToFootArray.add(new Matrix4d( 0.632591125744892, -0.48540145775144217, -0.6035013607619996, 0.44612369993820733, 0.49119789175305684, -0.3510084576212115, 0.7971936363367002, -0.0077213165731045506, -0.598793035001379, -0.8007362159232924, 0.01638333734878842, 0.9320980099528423,      0.0, 0.0, 0.0, 1.0       ));
      LeftHandToWorldArray.add(new Matrix4d( 0.6167714771970556, 0.49198106001792985, -0.6144490064275442, 0.345027813750304, -0.5052683887278109, -0.35110863501137296, -0.7883061472377676, 0.25455012272989685, -0.6035700458675562, 0.7966664063484069, 0.03202868600725344, 1.0127852202414223,         0.0, 0.0, 0.0, 1.0          ));
      LeftHandToFootArray.add(new Matrix4d( 0.6222980948084975, 0.48523769226295277, -0.6142389300630808, 0.400181842297594, -0.5049689790948718, -0.3507313157368292, -0.7886658825595262, 0.41878128396875286, -0.5981232409375158, 0.8009568815915357, 0.026770552507668313, 0.9316125713405345,          0.0, 0.0, 0.0, 1.0           ));
      
      RightHandToWorldArray.add(new Matrix4d( 0.8236516130241986, -0.4925343900078735, -0.2810834307144546, 0.13611541227932822, 0.10783136954603127, -0.3505882405103596, 0.9303011777686179, -0.7065096797442678, -0.5567498685266689, -0.7965536769580704, -0.23565191198060526, 0.8147145145019492,       0.0, 0.0, 0.0, 1.0       ));
      RightHandToFootArray.add(new Matrix4d( 0.8284010565903457, -0.4853778560882402, -0.27957114704343594, 0.19356794090219648, 0.10825775196464857, -0.3509655523176656, 0.930109370034469, -0.5424234174384698, -0.549574333989097, -0.8007693287742353, -0.23819431881323477, 0.7318406692471304,         0.0, 0.0, 0.0, 1.0         ));
      LeftHandToWorldArray.add(new Matrix4d( 0.8234671945389558, 0.49266712279171887, -0.28139098357671477, 0.18328374403935727, -0.1079866990755339, -0.35079949697113094, -0.9302035184558122, 0.7313352493780738, -0.5569925065392817, 0.7963785651591405, -0.23567038128317566, 0.8150466887897028,       0.0, 0.0, 0.0, 1.0       ));
      LeftHandToFootArray.add(new Matrix4d( 0.8283460843456485, 0.4859259110853176, -0.2787808700128757, 0.2398832911555768, -0.10756041297978683, -0.3504219991162724, -0.9303951741571824, 0.8954491620018408, -0.549794072456997, 0.8006749849166387, -0.23800429916262264, 0.7324200552910664,             0.0, 0.0, 0.0, 1.0            ));
      
      RightHandToWorldArray.add(new Matrix4d( 0.8236912600011814, -0.49258441750056015, -0.28087951123800464, 0.203399628512669, 0.10760982620217245, -0.35055175776725606, 0.9303405776547776, -0.4432755214542159, -0.5567340779065035, -0.7965387980267633, -0.2357394954938064, 0.814689404737201,        0.0, 0.0, 0.0, 1.0         ));
      RightHandToFootArray.add(new Matrix4d( 0.8284406944204825, -0.4854280401581863, -0.27936648627974586, 0.2606945125018025, 0.10803622358665432, -0.3509290920704725, 0.9301488841748552, -0.2791495533136561, -0.549558177285393, -0.8007548876945904, -0.2382801284454504, 0.7323711031889578,           0.0, 0.0, 0.0, 1.0          ));
      LeftHandToWorldArray.add(new Matrix4d( 0.08545439619835085, 0.06402979088621721, -0.9942825212430534, 0.45850577509321533, -0.9959216236141816, 0.034478064857394075, -0.08337495224215669, 0.26317903430126854, 0.02894245649677772, 0.9973522190895034, 0.06671495551355418, 1.176316490154357,       0.0, 0.0, 0.0, 1.0        ));
      LeftHandToFootArray.add(new Matrix4d( 0.08578687295117883, 0.05532886982226411, -0.994776019309598, 0.512227589192328, -0.9958678099399496, 0.034623970125056025, -0.08395526081305299, 0.4274948115607019, 0.0297979454772781, 0.9978676750235826, 0.05807052254253431, 1.0961240237271637,             0.0, 0.0, 0.0, 1.0            ));
 
      RightHandToWorldArray.add(new Matrix4d( -0.030655672262984407, -0.2937921723890779, -0.9553776160246852, 0.3876194248996444, 0.999097442585265, 0.019110555330001996, -0.037935298793630494, -0.1449246734889309, 0.02940289063489455, -0.9556782649604864, 0.29294116116453706, 1.1264249067894079,        0.0, 0.0, 0.0, 1.0    ));
      RightHandToFootArray.add(new Matrix4d( -0.03149978737670745, -0.28547674433961523, -0.9578678363096218, 0.4420188217359727, 0.9990823327764906, 0.01883338034132465, -0.03846811821242208, 0.019343884767590667, 0.02902164242609446, -0.9582005699362599, 0.2846215242119244, 1.045663849100821,          0.0, 0.0, 0.0, 1.0        ));
      LeftHandToWorldArray.add(new Matrix4d( 0.08547442186680355, 0.06439195271871097, -0.9942574111524678, 0.4347797612325888, -0.9959257431634769, 0.034307907974750904, -0.08339593248278204, 0.20607894151509376, 0.028740864823668916, 0.9973347702128247, 0.06706205196473093, 1.176577518780454,          0.0, 0.0, 0.0, 1.0        ));
      LeftHandToFootArray.add(new Matrix4d( 0.085808661645227, 0.0556912724827643, -0.9947539171855048, 0.4885338947403559, -0.9958719388802268, 0.03445402577935903, -0.08397619578507794, 0.3703807398884125, 0.029596535905295103, 0.9978533971866039, 0.05841782934653435, 1.0961850712627446,               0.0, 0.0, 0.0, 1.0            ));
      
      RightHandToWorldArray.add(new Matrix4d( 0.8236065291283172, -0.4926239743883254, -0.2810585437859112, 0.1985281854213744, 0.10782518840548785, -0.35051404878128883, 0.9303298502962637, -0.3118582005016258, -0.5568177564720144, -0.7965309293836605, -0.23556838627598942, 1.134665633483067,           0.0, 0.0, 0.0, 1.0         ));
      RightHandToFootArray.add(new Matrix4d( 0.8283565651959268, -0.4854676841074383, -0.2795470060368509, 0.25296155685397553, 0.10825153615591393, -0.3508914115590835, 0.9301380662105803, -0.1477004591750844, -0.5496426164488081, -0.8007473665154534, -0.23811058187714845, 1.052277902003901,            0.0, 0.0, 0.0, 1.0          ));
      LeftHandToWorldArray.add(new Matrix4d( 0.823486455317593, 0.49232886183689556, -0.2819261422976824, -0.007874646251496115, -0.10836330916554024, -0.3512870342111075, -0.929975705500879, 0.32460274424062546, -0.5568908790195433, 0.7963728469740484, -0.23592972993518488, 0.8148627051060824,          0.0, 0.0, 0.0, 1.0       ));
      LeftHandToFootArray.add(new Matrix4d( 0.8283646786743211, 0.4855880055235528, -0.27931388797631834, 0.04897374001488973, -0.10793700127829914, -0.3509097362983995, -0.9301677056993733, 0.4886037440387385, -0.5496922437872284, 0.8006663761284385, -0.23826831777959195, 0.7306189091917638,            0.0, 0.0, 0.0, 1.0         ));

   }

   @org.junit.AfterClass
   static public void keepAliveTheGUI()
   {
      if (scs != null)
      {
         ThreadTools.sleepForever();
      }
   }

   @Override
   public WholeBodyControllerParameters getRobotModel()
   {
      return atlasRobotModel;
   }

   @Override
   public FullRobotModelVisualizer getFullRobotModelVisualizer()
   {
      return modelVisualizer;
   }

   @Override
   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return scs;
   }

	@AverageDuration
	@Test(timeout = 150000)
   public void testRightHandIn3PMode()
   {
//      ArrayList<Pair<ReferenceFrame, ReferenceFrame>> handTargetArray = createHalfCylinderOfTargetPoints(RobotSide.RIGHT);
      ArrayList<Pair<ReferenceFrame, ReferenceFrame>> handTargetArray = createManualReferenceFramesPairArrayList(null, RightHandToWorldArray);
      executeHandTargetTest(ControlledDoF.DOF_NONE, ControlledDoF.DOF_3P, handTargetArray, false);
   }

	@AverageDuration
	@Test(timeout = 150000)
   public void testLeftHandIn3PMode()
   {
//      ArrayList<Pair<ReferenceFrame, ReferenceFrame>> handTargetArray = createHalfCylinderOfTargetPoints(RobotSide.LEFT);
      ArrayList<Pair<ReferenceFrame, ReferenceFrame>> handTargetArray = createManualReferenceFramesPairArrayList(LeftHandToWorldArray, null);
      this.executeHandTargetTest(ControlledDoF.DOF_3P, ControlledDoF.DOF_NONE, handTargetArray, false);
   }

	@AverageDuration
	@Test(timeout = 150000)
   public void testBothHandsIn3PMode()
   {
      ArrayList<Pair<ReferenceFrame, ReferenceFrame>> handTargetArray = createManualReferenceFramesPairArrayList(LeftHandToWorldArray, RightHandToWorldArray);
      this.executeHandTargetTest(ControlledDoF.DOF_3P, ControlledDoF.DOF_3P, handTargetArray, false);
   }

	@AverageDuration
	@Test(timeout = 150000)
   public void testRightHandIn3P2RMode()
   {
      ArrayList<Pair<ReferenceFrame, ReferenceFrame>> handTargetArray = createManualReferenceFramesPairArrayList(null, RightHandToWorldArray);
      this.executeHandTargetTest(ControlledDoF.DOF_NONE, ControlledDoF.DOF_3P2R, handTargetArray, false);
   }

	@AverageDuration
	@Test(timeout = 150000)
   public void testLeftHandIn3P2RMode()
   {
      ArrayList<Pair<ReferenceFrame, ReferenceFrame>> handTargetArray = createManualReferenceFramesPairArrayList(LeftHandToWorldArray, null);
      this.executeHandTargetTest(ControlledDoF.DOF_3P2R, ControlledDoF.DOF_NONE, handTargetArray, false);
   }

	@AverageDuration
	@Test(timeout = 150000)
   public void testBothHandsIn3P2RMode()
   {
      ArrayList<Pair<ReferenceFrame, ReferenceFrame>> handTargetArray = createManualReferenceFramesPairArrayList(LeftHandToWorldArray, RightHandToWorldArray);
      this.executeHandTargetTest(ControlledDoF.DOF_3P2R, ControlledDoF.DOF_3P2R, handTargetArray, false);
   }

	@AverageDuration
	@Test(timeout = 150000)
   public void testRightHandIn3P3RMode()
   {
      ArrayList<Pair<ReferenceFrame, ReferenceFrame>> handTargetArray = createManualReferenceFramesPairArrayList(null, RightHandToWorldArray);
      this.executeHandTargetTest(ControlledDoF.DOF_NONE, ControlledDoF.DOF_3P3R, handTargetArray, false);
   }

	@AverageDuration
	@Test(timeout = 150000)
   public void testLeftHandIn3P3RMode()
   {
      ArrayList<Pair<ReferenceFrame, ReferenceFrame>> handTargetArray = createManualReferenceFramesPairArrayList(LeftHandToWorldArray, null);
      this.executeHandTargetTest(ControlledDoF.DOF_3P3R, ControlledDoF.DOF_NONE, handTargetArray, false);
   }

	@AverageDuration
	@Test(timeout = 150000)
   public void testBothHandsIn3P3RMode()
   {
      ArrayList<Pair<ReferenceFrame, ReferenceFrame>> handTargetArray = createManualReferenceFramesPairArrayList(LeftHandToWorldArray, RightHandToWorldArray);
      this.executeHandTargetTest(ControlledDoF.DOF_3P3R, ControlledDoF.DOF_3P3R, handTargetArray, false);
   }
   

   public void initializeFullRobotModelJointAngles(SDFFullRobotModel fullRobotModelToInitialize)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.HIP_YAW)).setQ(0.0); //leg_hpz
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.HIP_ROLL)).setQ(
               robotSide.negateIfRightSide(0.1)); //leg_hpx
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.HIP_PITCH)).setQ(-0.3); //leg_hpy
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.KNEE)).setQ(0.6); //leg_kny
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.ANKLE_PITCH)).setQ(-0.3); //leg_aky
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.ANKLE_ROLL)).setQ(
               robotSide.negateIfRightSide(-0.1)); //leg_akx
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.SHOULDER_YAW)).setQ(0.500); //arm_shy
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL)).setQ(
               robotSide.negateIfRightSide(-1.0)); //arm_shx
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.ELBOW_PITCH)).setQ(2.00); //arm_ely
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.ELBOW_ROLL)).setQ(
               robotSide.negateIfRightSide(0.6)); //arm_elx
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.WRIST_PITCH)).setQ(0.000); //arm_wry
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.WRIST_ROLL)).setQ(
               robotSide.negateIfRightSide(0)); //arm_wrx
      }
   }

   public void initializeSDFRobotlJointAngles(SDFRobot scsRobot)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.HIP_YAW)).setQ(0.0); //leg_hpz
         scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.HIP_ROLL)).setQ(
               robotSide.negateIfRightSide(0.062)); //leg_hpx
         scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.HIP_PITCH)).setQ(-0.233); //leg_hpy
         scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.KNEE)).setQ(0.518); //leg_kny
         scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.ANKLE_PITCH)).setQ(-0.276); //leg_aky
         scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.ANKLE_ROLL)).setQ(
               robotSide.negateIfRightSide(-0.062)); //leg_akx
         scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.SHOULDER_YAW)).setQ(0.300); //arm_shy
         scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL)).setQ(
               robotSide.negateIfRightSide(-1.30)); //arm_shx
         scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.ELBOW_PITCH)).setQ(2.00); //arm_ely
         scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.ELBOW_ROLL)).setQ(
               robotSide.negateIfRightSide(0.498)); //arm_elx
         scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.WRIST_PITCH)).setQ(0.000); //arm_wry
         scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.WRIST_ROLL)).setQ(
               robotSide.negateIfRightSide(-0.004)); //arm_wrx
      }
   }
}
