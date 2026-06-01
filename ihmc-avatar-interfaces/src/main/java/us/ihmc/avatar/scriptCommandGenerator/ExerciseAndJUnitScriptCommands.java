package us.ihmc.avatar.scriptCommandGenerator;

import controller_msgs.FootstepDataListMessage;
import controller_msgs.FootstepDataMessage;
import controller_msgs.PauseWalkingMessage;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.Arrays;

public final class ExerciseAndJUnitScriptCommands
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private ExerciseAndJUnitScriptCommands()
   {
   }

   public static void run(ExerciseAndJUnitScript script, ScriptBasedControllerCommandGenerator generator, ReferenceFrame referenceFrame)
   {
      RigidBodyTransform transformToWorld = referenceFrame.getTransformToDesiredFrame(worldFrame);
      switch (script)
      {
         case DRC_TRIALS_SLOPE_LEFT_FOOT_POSE:
            submitFootsteps(generator, transformToWorld, 0.0, 0.0,
                                      new FootstepSpec[] {
               new FootstepSpec(RobotSide.RIGHT, 0.3819996816139285, -0.21714809916017686, 0.005708466297193751, 0.0033380751509319547, 0.00600115961289335, 0.003552884292981071, 0.9999701097286798),
               new FootstepSpec(RobotSide.LEFT, 0.7784631444456441, 0.04199123249539027, 0.005923403262909133, -0.00011651165434174242, 0.0015395393915390602, 0.0035742441276513295, 0.9999924204824816),
               new FootstepSpec(RobotSide.RIGHT, 0.7802491333256277, -0.20800245509025075, 0.0060222067606626045, 0.0025136373658113335, -0.00041334771937638414, 0.0035755624498098487, 0.9999903630155761),
                                      });
            submitPause(generator, false);
            submitFootsteps(generator, transformToWorld, 0.0, 0.0,
                                      new FootstepSpec[] {
               new FootstepSpec(RobotSide.LEFT, 0.8312655479266958, 0.18655253435174066, 0.007494269975721529, 0.02542766163880979, -0.026269017796840415, -0.033140482008324645, 0.998781798582567),
               new FootstepSpec(RobotSide.RIGHT, 0.948581230470679, -0.15702886514699466, 0.007923681573495572, 0.002001072509080683, -0.03457796465628506, -0.041942675299671606, 0.9985194900740538),
               new FootstepSpec(RobotSide.LEFT, 1.1555000821363999, 0.08031631152755948, 0.0482495296363287, 0.054249338760572426, -0.15664097158605744, -0.03194261322336017, 0.9856471400684321),
               new FootstepSpec(RobotSide.RIGHT, 1.2621620127949849, -0.17634321104275688, 0.07168588934429558, -0.012298185810682694, -0.14433945567362388, -0.04249505904158686, 0.9885388440110282),
               new FootstepSpec(RobotSide.LEFT, 1.4405297701904973, 0.05951096531876969, 0.12471127357379348, 0.0054519688139955755, -0.12385807730221086, -0.040391385614190436, 0.9914625503221071),
               new FootstepSpec(RobotSide.RIGHT, 1.4194208657031027, -0.18959823681238225, 0.1351486028211072, -0.0018950853997859727, -0.12854159942701113, -0.04123485906414261, 0.9908446660632703),
                                      });
            submitPause(generator, false);
            submitFootsteps(generator, transformToWorld, 0.0, 0.0,
                                      new FootstepSpec[] {
               new FootstepSpec(RobotSide.RIGHT, 1.4451124971048381, -0.18364121109022982, 0.13513301830548063, -0.002536873056239693, -0.1283752583645514, 0.013809198431849605, 0.9916262720166308),
               new FootstepSpec(RobotSide.LEFT, 1.7155463558612245, 0.2014165326903834, 0.14255682103825632, 0.017451017801187206, 0.12049555722275646, 0.04585244712748365, 0.9915007996767836),
               new FootstepSpec(RobotSide.RIGHT, 1.7675150528827661, -0.08623057793686688, 0.14243009855772565, 0.012519722504541865, 0.13724016152471977, 0.04591007410758001, 0.9893940871606797),
               new FootstepSpec(RobotSide.LEFT, 1.915341557681539, 0.17952341290209656, 0.09240604032183408, 0.009774099443702756, 0.14205428088971067, 0.04613860215464631, 0.9887346851669497),
               new FootstepSpec(RobotSide.RIGHT, 1.939753101976663, -0.06928154472499164, 0.09238318872711514, 0.005064921099470476, 0.1453070660929657, 0.046722280630632426, 0.9882698172110606),
                                      });
            submitPause(generator, false);
            submitFootsteps(generator, transformToWorld, 0.0, 0.0,
                                      new FootstepSpec[] {
               new FootstepSpec(RobotSide.RIGHT, 2.0231511380015452, -0.03947862261886354, 0.06269974712057401, -0.03996137793337269, 0.16362751490552138, 0.0010410711145800156, 0.9857119461640119),
               new FootstepSpec(RobotSide.LEFT, 2.3336702500942383, 0.23774729901937414, 0.005537805488121966, 0.009031237629051279, 0.00978788936302821, -0.005867111523041889, 0.9998940998781226),
               new FootstepSpec(RobotSide.RIGHT, 2.358332434903211, 0.0005668582576247691, 0.006072105520890575, -0.00579815967405266, -0.0012900691047702051, -0.00578373033686396, 0.9999656321741711),
               new FootstepSpec(RobotSide.LEFT, 2.52797611782022, 0.24862252440648447, 0.00551233910474444, 0.004613736234819086, 0.010144442876422046, -0.00582535424795535, 0.999920931356359),
               new FootstepSpec(RobotSide.RIGHT, 2.5250956321911935, -0.0013601689742313613, 0.005509216457219593, 0.000509622949998805, 0.010161212697416818, -0.00578366401508213, 0.999931517290823),
                                      });
            submitPause(generator, false);
            break;

         case DRC_TRIALS_ZIGZAG_HURDLES_LEFT_FOOT_POSE:
            submitFootsteps(generator, transformToWorld, 0.0, 0.0,
                                      new FootstepSpec[] {
               new FootstepSpec(RobotSide.LEFT, 0.225, -0.1, 0.02099999999999999, 0.0, 0.0, 0.0, 1.0),
               new FootstepSpec(RobotSide.RIGHT, 0.435, -0.52, 0.14100000000000001, 0.0, 0.0, 0.0, 1.0),
               new FootstepSpec(RobotSide.LEFT, 0.725, -0.13, 0.14100000000000001, 0.0, 0.0, 0.0, 1.0),
               new FootstepSpec(RobotSide.RIGHT, 0.9568021800344485, -0.5173231118026214, 0.02099999999999999, 0.0, 0.0, 0.0, 1.0),
               new FootstepSpec(RobotSide.LEFT, 1.3412268406702583, -0.20091956082893264, 0.02099999999999999, 0.0, 0.0, 0.0, 1.0),
               new FootstepSpec(RobotSide.RIGHT, 1.5691048863799313, -0.4279145573420012, 0.02099999999999999, 0.0, 0.0, 0.0, 1.0),
               new FootstepSpec(RobotSide.LEFT, 1.59044003680388, -0.21746384957600445, 0.02099999999999999, 0.0, 0.0, 0.0, 1.0),
                                      });
            submitPause(generator, false);
            break;

         case LONG_SIDE_STEPS_LEFT:
            submitFootsteps(generator, transformToWorld, 0.0, 0.0,
                                      new FootstepSpec[] {
               new FootstepSpec(RobotSide.LEFT, 0.017065668895015117, 0.23404433166049693, -2.110685839805393e-05, -0.00015913047654473808, -0.0002581897453172097, 0.001475917116189346, 0.9999988648414622),
               new FootstepSpec(RobotSide.RIGHT, 0.01743763302656967, 0.10803711391963246, 2.0930724626166963e-05, -0.0002256309062557937, -0.00024697718662275106, 0.001475899512879994, 0.99999885490614),
               new FootstepSpec(RobotSide.LEFT, 0.0160358498913055, 0.5829238621349261, -0.00011348784364163811, -0.00015913047654515338, -0.0002581897453165707, 0.001475917116189346, 0.9999988648414622),
               new FootstepSpec(RobotSide.RIGHT, 0.016405150114233692, 0.45781324763357384, -8.035938289094324e-05, -0.0001591304765451549, -0.0002581897453174589, 0.0014759171161893457, 0.9999988648414622),
               new FootstepSpec(RobotSide.LEFT, 0.015003384296461966, 0.9326999981961956, -0.00020610624409457545, -0.00015913047654568437, -0.00025818974531876285, 0.001475917116189346, 0.9999988648414622),
               new FootstepSpec(RobotSide.RIGHT, 0.015372684519390159, 0.8075893836948435, -0.00017297778334422753, -0.00015913047654526544, -0.00025818974531718154, 0.001475917116189346, 0.9999988648414622),
               new FootstepSpec(RobotSide.LEFT, 0.014487151499040197, 1.1075880662268303, -0.0002524154443218213, -0.0001591304765448219, -0.0002581897453175427, 0.001475917116189346, 0.9999988648414622),
                                      });
            submitPause(generator, false);
            break;

         case LONG_STEPS_MAX_HEIGHT_PAUSE_AND_RESTART_LEFT_FOOT_TEST:
            submitFootsteps(generator, transformToWorld, 0.0, 0.0,
                                      new FootstepSpec[] {
               new FootstepSpec(RobotSide.LEFT, 0.4587796761983862, 0.035775123781215404, -0.005089890266117628, -0.00014226198976730503, 0.0006658530697307656, -0.007153881555249982, 0.9999741788565889),
               new FootstepSpec(RobotSide.RIGHT, 1.034584897695115, -0.11697898558017215, -0.005698294619403674, -0.00014226198980663387, 0.0006658530697223544, -0.007153881614314441, 0.9999741788561664),
                                      });
            submitPause(generator, false);
            submitPause(generator, false);
            submitPause(generator, false);
            submitPause(generator, false);
            submitPause(generator, false);
            submitFootsteps(generator, transformToWorld, 0.0, 0.0,
                                      new FootstepSpec[] {
               new FootstepSpec(RobotSide.LEFT, 1.5773135634335025, 0.020352014569407495, -0.006577447151677199, -0.00014226198980663105, 0.0006658530697223714, -0.007153881614314441, 0.9999741788561664),
               new FootstepSpec(RobotSide.RIGHT, 1.8932137385535026, -0.24085070347943513, -0.006720463800491655, -0.00014226198976730514, 0.0006658530697307572, -0.007153881555249982, 0.9999741788565889),
               new FootstepSpec(RobotSide.LEFT, 2.3761560835479787, 0.0022643634899831883, -0.00764690034716195, -0.00023887948916003418, 0.000683161321498182, -0.007153817863394783, 0.9999741492244566),
               new FootstepSpec(RobotSide.RIGHT, 2.3725795842070396, -0.24770997514554816, -0.0073470798338264864, -7.764063142704572e-05, 0.0006140671590071872, -0.007153914168063466, 0.999974218870433),
                                      });
            break;

         case SIMPLE_FLAT_GROUND_SCRIPT:
            submitFootsteps(generator, transformToWorld, 0.0, 0.0,
                                      new FootstepSpec[] {
               new FootstepSpec(RobotSide.LEFT, 0.025715511329624546, 9.3055952595988e-06, -2.2439086531933494e-05, 1.2481815200629632e-06, -0.0008137147287229978, -1.0223284361872986e-06, 0.9999996689328138),
               new FootstepSpec(RobotSide.RIGHT, 0.025684358269919508, -0.1780019927119301, 2.524572399430214e-05, 9.229303571609209e-07, -0.0008137605655951133, -0.00017150588213870268, 0.9999996541892514),
                                      });
            submitFootsteps(generator, transformToWorld, 0.0, 0.0,
                                      new FootstepSpec[] {
               new FootstepSpec(RobotSide.RIGHT, 0.3457222961770401, -0.15536002576452945, -0.0007238077382609037, -0.00013242825830794223, 0.00039014612211330364, 4.504852714257658e-05, 0.9999999141096914),
               new FootstepSpec(RobotSide.LEFT, 0.5851867241463046, 0.04888244861654454, -0.0009646669302214167, -0.00012653364241641769, 0.0003900546579935504, 4.5046238988566765e-05, 0.9999999149087151),
               new FootstepSpec(RobotSide.RIGHT, 0.6120742342599089, -0.11063740111230903, -0.0009436932974718065, -0.00013825531878260852, 0.00038992198854206515, 4.505082625406707e-05, 0.9999999134083626),
                                      });
            submitPause(generator, false);
            submitFootsteps(generator, transformToWorld, 0.0, 0.0,
                                      new FootstepSpec[] {
               new FootstepSpec(RobotSide.RIGHT, 0.6265729778327734, -0.21694828497668536, -0.0009269280335787999, -0.00013363049959868003, 0.00038768065267928047, -0.018203498014978472, 0.999834218511559),
               new FootstepSpec(RobotSide.LEFT, 0.6350360465565287, 0.015394096639087937, -0.0009950783072789071, -0.00013363049959856728, 0.00038768065267919937, -0.018203498014978472, 0.999834218511559),
               new FootstepSpec(RobotSide.RIGHT, 0.6241157916517974, -0.2844070715486974, -0.0009079372004641012, -0.00013363049959858207, 0.00038768065268000396, -0.018203498014978472, 0.999834218511559),
                                      });
            submitPause(generator, false);
            submitFootsteps(generator, transformToWorld, 0.0, 0.0,
                                      new FootstepSpec[] {
               new FootstepSpec(RobotSide.RIGHT, 0.8800154506490467, -0.2189350828355645, -0.0011241351646976844, -0.00013736653046558456, 0.00038637266362333706, -0.02785398632614362, 0.9996119183424856),
               new FootstepSpec(RobotSide.LEFT, 1.1895403229024335, 0.03129863416791767, -0.0014316715245196593, -0.0001373665304634808, 0.00038637266361956355, -0.02785398632614362, 0.9996119183424856),
               new FootstepSpec(RobotSide.RIGHT, 1.1756187632261719, -0.21831343342621237, -0.0013548920918210888, -0.0001373665304647557, 0.00038637266361947275, -0.027853986326143627, 0.9996119183424856),
                                      });
            submitPause(generator, false);
            break;

         case SIMPLE_SINGLE_FOOT_TRAJECTORY_SCRIPT:
            break;

         case SIMPLE_SINGLE_HAND_TRAJECTORY_SCRIPT:
            break;

         case SIMPLE_SINGLE_PELVIS_HEIGHT_SCRIPT:
            break;

         case SIMPLE_SINGLE_STEP_SCRIPT:
            submitFootsteps(generator, transformToWorld, 0.0, 0.0,
                                      new FootstepSpec[] {
               new FootstepSpec(RobotSide.LEFT, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
                                      });
            break;

         case TWO_CINDER_BLOCKS_STEP_ON_LEFT_FOOT_TEST:
            submitFootsteps(generator, transformToWorld, 0.0, 0.0,
                                      new FootstepSpec[] {
               new FootstepSpec(RobotSide.RIGHT, 0.27134708390623097, -0.15565001575425902, -0.0017864603085800124, 0.01089603089710632, 0.01116810841180155, -0.00012150622143424914, 0.9998782601404181),
               new FootstepSpec(RobotSide.LEFT, 0.5658686945004905, 0.09695148340679087, -0.0033226950327895632, -0.00012961171578895604, 0.0008301744138303921, 3.000606007459321e-06, 0.9999996470010588),
               new FootstepSpec(RobotSide.RIGHT, 0.8756173675969479, -0.18381490765691977, 0.2991554862322814, 0.030387169822316894, 0.0003547918480583096, -7.827287139449683e-06, 0.9995381403286558),
               new FootstepSpec(RobotSide.LEFT, 1.1541445436624365, 0.20623577550635594, 0.3038620854552176, -0.00012961178576678537, 0.0008301744029050385, 2.9163131120500995e-06, 0.9999996470013082),
                                      });
            submitPause(generator, false);
            break;

         case TWO_CINDER_BLOCKS_STEP_ON_LEFT_FOOT_TEST_SLOW:
            submitFootsteps(generator, transformToWorld, 0.0, 0.0,
                                      new FootstepSpec[] {
               new FootstepSpec(RobotSide.RIGHT, 0.27134708390623097, -0.15565001575425902, -0.0017864603085800124, 0.01089603089710632, 0.01116810841180155, -0.00012150622143424914, 0.9998782601404181),
               new FootstepSpec(RobotSide.LEFT, 0.5658686945004905, 0.09695148340679087, -0.0033226950327895632, -0.00012961171578895604, 0.0008301744138303921, 3.000606007459321e-06, 0.9999996470010588),
               new FootstepSpec(RobotSide.RIGHT, 0.8756173675969479, -0.18381490765691977, 0.2991554862322814, 0.030387169822316894, 0.0003547918480583096, -7.827287139449683e-06, 0.9995381403286558),
               new FootstepSpec(RobotSide.LEFT, 1.1541445436624365, 0.20623577550635594, 0.3038620854552176, -0.00012961178576678537, 0.0008301744029050385, 2.9163131120500995e-06, 0.9999996470013082),
                                      });
            submitPause(generator, false);
            break;

         default:
            throw new IllegalArgumentException("Unknown script: " + script);
      }
   }

   private record FootstepSpec(RobotSide side, double x, double y, double z, double qx, double qy, double qz, double qs) {}

   private static void submitFootsteps(ScriptBasedControllerCommandGenerator generator,
                                       RigidBodyTransform transformToWorld,
                                       double defaultSwingDuration,
                                       double defaultTransferDuration,
                                       FootstepSpec[] footsteps)
   {
      FootstepDataMessage[] footstepMessages = new FootstepDataMessage[footsteps.length];
      for (int i = 0; i < footsteps.length; i++)
      {
         FootstepSpec spec = footsteps[i];
         Pose3D pose = new Pose3D(new Point3D(spec.x, spec.y, spec.z), new Quaternion(spec.qx, spec.qy, spec.qz, spec.qs));
         footstepMessages[i] = HumanoidMessageTools.createFootstepDataMessage(spec.side, pose);
         MessageTransformer.transform(footstepMessages[i], transformToWorld);
      }
      FootstepDataListMessage message = HumanoidMessageTools.createFootstepDataListMessage(Arrays.asList(footstepMessages),
                                                                                            defaultSwingDuration,
                                                                                            defaultTransferDuration,
                                                                                            ExecutionMode.OVERRIDE);
      generator.submitMessage(message);
   }

   private static void submitPause(ScriptBasedControllerCommandGenerator generator, boolean pause)
   {
      PauseWalkingMessage message = new PauseWalkingMessage();
      message.setPause(pause);
      message.setClearRemainingFootstepQueue(false);
      generator.submitMessage(message);
   }
}
