package us.ihmc.simulationConstructionSetTools.util.environments;

import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceTexture;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;

/**
 * SCS environments defined from RDX JSON-defined environments. See RDXEnvironmentExporter to generate method
 */
public enum RDXEnvironments
{
   LOOK_AND_STEP_EASY,
   LOOK_AND_STEP_HARD,
   FOOTSTEP_PLANNER_TRAINING_TERRAIN_GENERATED;

   public CommonAvatarEnvironmentInterface getEnvironment()
   {
      CombinedTerrainObject3D terrain = new CombinedTerrainObject3D(getClass().getSimpleName());
      YoAppearanceTexture groundTexture = new YoAppearanceTexture("Textures/gridGroundProfile.png");

      Box3D ground = new Box3D(50.0, 50.0, 1.0);
      ground.getPosition().setZ(-0.5);
      terrain.addTerrainObject(new RotatableBoxTerrainObject(ground, groundTexture));

      switch (this)
      {
         case LOOK_AND_STEP_EASY -> addLookAndStepSimple(terrain);
         case LOOK_AND_STEP_HARD -> addLookAndStepHard(terrain);
         case FOOTSTEP_PLANNER_TRAINING_TERRAIN_GENERATED -> addFootstepPlannerTrainingTerrainGenerated(terrain);
      }

      terrain.addTerrainObject(terrain);
      return () -> terrain;
   }
   
   private static void addLookAndStepSimple(CombinedTerrainObject3D terrain)
   {
      AppearanceDefinition appearance = YoAppearance.DarkGray();

      Point3D position0 = new Point3D(2.9991922858880433, 0.17738506103097837, 0.07749255480474931);
      Quaternion orientation0 = new Quaternion(3.591011281523848E-4, -4.404093410555292E-4, -0.7140274385984735, 0.7001174858654047);
      Vector3D size0 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box0 = new Box3D(position0, orientation0, size0);
      terrain.addRotatableBox(box0, appearance);

      Point3D position1 = new Point3D(4.010313695756378, 0.1616056331805633, 0.07664394760851354);
      Quaternion orientation1 = new Quaternion(0.0010814627453136847, 0.0011373925186028025, -0.7128015132552289,0.7013640563057426);
      Vector3D size1 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box1 = new Box3D(position1, orientation1, size1);
      terrain.addRotatableBox(box1, appearance);

      Point3D position2 = new Point3D(1.538532083708091, 0.6741701950310802, 0.05131964350890132);
      Quaternion orientation2 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size2 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box2 = new Box3D(position2, orientation2, size2);
      terrain.addRotatableBox(box2, appearance);

      Point3D position3 = new Point3D(1.5377122418466396, 0.4871995187212327, 0.0453699739307727);
      Quaternion orientation3 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size3 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box3 = new Box3D(position3, orientation3, size3);
      terrain.addRotatableBox(box3, appearance);

      Point3D position4 = new Point3D(1.8318418085847752, 0.5832432291386377, 0.05026246658367628);
      Quaternion orientation4 = new Quaternion(0.0, 0.0, -0.7032798171527006,0.7109131443331624);
      Vector3D size4 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box4 = new Box3D(position4, orientation4, size4);
      terrain.addRotatableBox(box4, appearance);

      Point3D position5 = new Point3D(2.0226812492006916, 0.5816220959595908, 0.058535600491900285);
      Quaternion orientation5 = new Quaternion(0.0, 0.0, -0.7033244049950812,0.7108690324794822);
      Vector3D size5 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box5 = new Box3D(position5, orientation5, size5);
      terrain.addRotatableBox(box5, appearance);

      Point3D position6 = new Point3D(1.9258075527472558, 0.2916058787804267, 0.04914770991489481);
      Quaternion orientation6 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size6 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box6 = new Box3D(position6, orientation6, size6);
      terrain.addRotatableBox(box6, appearance);

      Point3D position7 = new Point3D(1.9221267562656839, 0.0971054615729663, 0.048374686104427383);
      Quaternion orientation7 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size7 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box7 = new Box3D(position7, orientation7, size7);
      terrain.addRotatableBox(box7, appearance);

      Point3D position8 = new Point3D(1.6391139250203717, 0.20003028808817114, 0.03909051303520233);
      Quaternion orientation8 = new Quaternion(4.2221998162823653E-4, -0.0021771612644779165, -0.7083346233855763,0.7058733193798512);
      Vector3D size8 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box8 = new Box3D(position8, orientation8, size8);
      terrain.addRotatableBox(box8, appearance);

      Point3D position9 = new Point3D(1.4508258759747759, 0.19187378577710404, 0.03772504674415519);
      Quaternion orientation9 = new Quaternion(0.0, 0.0, 0.7038510074598656,0.710347632710726);
      Vector3D size9 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box9 = new Box3D(position9, orientation9, size9);
      terrain.addRotatableBox(box9, appearance);

      Point3D position10 = new Point3D(1.544254053576254, -0.09368499925145309, 0.032965432002729965);
      Quaternion orientation10 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size10 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box10 = new Box3D(position10, orientation10, size10);
      terrain.addRotatableBox(box10, appearance);

      Point3D position11 = new Point3D(1.5421604510537665, -0.2847681953194701, 0.03169062655032244);
      Quaternion orientation11 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size11 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box11 = new Box3D(position11, orientation11, size11);
      terrain.addRotatableBox(box11, appearance);

      Point3D position12 = new Point3D(1.8302064340875628, -0.18772505668791095, 0.03233411894842265);
      Quaternion orientation12 = new Quaternion(0.0, 0.0, -0.7041659007794275,0.7100354809300009);
      Vector3D size12 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box12 = new Box3D(position12, orientation12, size12);
      terrain.addRotatableBox(box12, appearance);

      Point3D position13 = new Point3D(2.020076111592974, -0.19450183861517623, 0.032951829300953354);
      Quaternion orientation13 = new Quaternion(0.0, 0.0, 0.7094730567086567,0.7047325604826807);
      Vector3D size13 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box13 = new Box3D(position13, orientation13, size13);
      terrain.addRotatableBox(box13, appearance);

      Point3D position14 = new Point3D(2.3017726687557736, 0.6659211869311393, 0.050732948560174566);
      Quaternion orientation14 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size14 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box14 = new Box3D(position14, orientation14, size14);
      terrain.addRotatableBox(box14, appearance);

      Point3D position15 = new Point3D(2.3045613478518816, 0.4744061119031097, 0.04678489901068887);
      Quaternion orientation15 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size15 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box15 = new Box3D(position15, orientation15, size15);
      terrain.addRotatableBox(box15, appearance);

      Point3D position16 = new Point3D(2.214159178501207, 0.19092562262741894, 0.04546863118006792);
      Quaternion orientation16 = new Quaternion(0.0, 0.0, -0.7100452702091272,0.7041560297644605);
      Vector3D size16 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box16 = new Box3D(position16, orientation16, size16);
      terrain.addRotatableBox(box16, appearance);

      Point3D position17 = new Point3D(2.3882625449800243, 0.1854857053173351, 0.04358061639227621);
      Quaternion orientation17 = new Quaternion(0.0, 0.0, 0.7032305077721187,0.710961920878023);
      Vector3D size17 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box17 = new Box3D(position17, orientation17, size17);
      terrain.addRotatableBox(box17, appearance);

      Point3D position18 = new Point3D(2.305334951328886, -0.09669125069873875, 0.04026317060808564);
      Quaternion orientation18 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size18 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box18 = new Box3D(position18, orientation18, size18);
      terrain.addRotatableBox(box18, appearance);

      Point3D position19 = new Point3D(2.3035618801058884, -0.28517226345878166, 0.043838376674458175);
      Quaternion orientation19 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size19 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box19 = new Box3D(position19, orientation19, size19);
      terrain.addRotatableBox(box19, appearance);

      Point3D position20 = new Point3D(5.022827545101327, 0.14359272115178368, 0.06929944023803199);
      Quaternion orientation20 = new Quaternion(0.0, 0.0, -0.7146326573404541,0.6994999392870033);
      Vector3D size20 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box20 = new Box3D(position20, orientation20, size20);
      terrain.addRotatableBox(box20, appearance);

      Point3D position21 = new Point3D(6.0368497952581395, 0.10384214494564183, 0.06172802598521781);
      Quaternion orientation21 = new Quaternion(0.0056138542832086386, 0.00558201305529614, -0.7143726400604126,0.6997206992103732);
      Vector3D size21 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box21 = new Box3D(position21, orientation21, size21);
      terrain.addRotatableBox(box21, appearance);

      Point3D position22 = new Point3D(7.050523277840206, 0.08289463608396092, 0.06864253875907342);
      Quaternion orientation22 = new Quaternion(0.0, 0.0, 0.7017961773791696,0.7123777968297335);
      Vector3D size22 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box22 = new Box3D(position22, orientation22, size22);
      terrain.addRotatableBox(box22, appearance);

      Point3D position23 = new Point3D(5.023846234292196, 0.13613703474961703, 0.22602295943394107);
      Quaternion orientation23 = new Quaternion(0.0, 0.0, 0.7009087265528494,0.7132509775962896);
      Vector3D size23 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box23 = new Box3D(position23, orientation23, size23);
      terrain.addRotatableBox(box23, appearance);

      Point3D position24 = new Point3D(2.6900015009577163, 0.6436366016069017, 0.19963009876119422);
      Quaternion orientation24 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size24 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box24 = new Box3D(position24, orientation24, size24);
      terrain.addRotatableBox(box24, appearance);

      Point3D position25 = new Point3D(2.6890564650088904, 0.4543195036924751, 0.1998904919894186);
      Quaternion orientation25 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size25 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box25 = new Box3D(position25, orientation25, size25);
      terrain.addRotatableBox(box25, appearance);

      Point3D position26 = new Point3D(2.7797616190377004, 0.16496187008841534, 0.19985815206905305);
      Quaternion orientation26 = new Quaternion(0.0, 0.0, -0.7091191010590019,0.7050887181860686);
      Vector3D size26 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box26 = new Box3D(position26, orientation26, size26);
      terrain.addRotatableBox(box26, appearance);

      Point3D position27 = new Point3D(2.593463551704851, 0.1621908804922451, 0.19471440676353352);
      Quaternion orientation27 = new Quaternion(0.0, 0.0, 0.7049836505970685,0.7092235560039095);
      Vector3D size27 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box27 = new Box3D(position27, orientation27, size27);
      terrain.addRotatableBox(box27, appearance);

      Point3D position28 = new Point3D(2.684024192233144, -0.13061899475743363, 0.19603696340587146);
      Quaternion orientation28 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size28 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box28 = new Box3D(position28, orientation28, size28);
      terrain.addRotatableBox(box28, appearance);

      Point3D position29 = new Point3D(2.6860150703200567, -0.320451962389396, 0.19828815283388773);
      Quaternion orientation29 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size29 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box29 = new Box3D(position29, orientation29, size29);
      terrain.addRotatableBox(box29, appearance);

      Point3D position30 = new Point3D(3.1711383025454465, 0.5462180514407013, 0.22616424914213662);
      Quaternion orientation30 = new Quaternion(0.0, 0.0, -0.7038282851139497,0.7103701465247234);
      Vector3D size30 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box30 = new Box3D(position30, orientation30, size30);
      terrain.addRotatableBox(box30, appearance);

      Point3D position31 = new Point3D(2.9818424101161205, 0.5424769634680712, 0.22464735511398604);
      Quaternion orientation31 = new Quaternion(0.0, 0.0, 0.709774201169097,0.7044292607173344);
      Vector3D size31 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box31 = new Box3D(position31, orientation31, size31);
      terrain.addRotatableBox(box31, appearance);

      Point3D position32 = new Point3D(3.0756668786094545, 0.2573398848240338, 0.22430098450189007);
      Quaternion orientation32 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size32 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box32 = new Box3D(position32, orientation32, size32);
      terrain.addRotatableBox(box32, appearance);

      Point3D position33 = new Point3D(3.0722024139657997, 0.06783466183720784, 0.2251948202315323);
      Quaternion orientation33 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size33 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box33 = new Box3D(position33, orientation33, size33);
      terrain.addRotatableBox(box33, appearance);

      Point3D position34 = new Point3D(3.165664362999164, -0.22522204408849275, 0.22681213465404784);
      Quaternion orientation34 = new Quaternion(0.0, 0.0, -0.7083652138817587,0.7058461048714872);
      Vector3D size34 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box34 = new Box3D(position34, orientation34, size34);
      terrain.addRotatableBox(box34, appearance);

      Point3D position35 = new Point3D(2.974840053678399, -0.22519875771755868, 0.22595308610329148);
      Quaternion orientation35 = new Quaternion(0.0, 0.0, 0.7081687952141784,0.7060431697034532);
      Vector3D size35 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box35 = new Box3D(position35, orientation35, size35);
      terrain.addRotatableBox(box35, appearance);

      Point3D position36 = new Point3D(3.359147869089035, 0.550879534175364, 0.22486106332834727);
      Quaternion orientation36 = new Quaternion(0.0, 0.0, -0.7071194347376093,0.7070941274090488);
      Vector3D size36 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box36 = new Box3D(position36, orientation36, size36);
      terrain.addRotatableBox(box36, appearance);

      Point3D position37 = new Point3D(3.354472458529534, 0.15783397615504235, 0.22395977136362658);
      Quaternion orientation37 = new Quaternion(0.0, 0.0, -0.7058785319428242,0.7083329006492944);
      Vector3D size37 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box37 = new Box3D(position37, orientation37, size37);
      terrain.addRotatableBox(box37, appearance);

      Point3D position38 = new Point3D(3.350402623341776, -0.22691663331160317, 0.22640085739147484);
      Quaternion orientation38 = new Quaternion(0.0, 0.0, 0.7062242020231959,0.7079882601263245);
      Vector3D size38 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box38 = new Box3D(position38, orientation38, size38);
      terrain.addRotatableBox(box38, appearance);

      Point3D position39 = new Point3D(3.68866762247556, 0.6236265834842826, 0.2503418207870058);
      Quaternion orientation39 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size39 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box39 = new Box3D(position39, orientation39, size39);
      terrain.addRotatableBox(box39, appearance);

      Point3D position40 = new Point3D(3.7706107394082076, 0.15186184665978525, 0.2572179895864939);
      Quaternion orientation40 = new Quaternion(0.0, 0.0, -0.7053501510530733,0.708859058212144);
      Vector3D size40 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box40 = new Box3D(position40, orientation40, size40);
      terrain.addRotatableBox(box40, appearance);

      Point3D position41 = new Point3D(3.68965927337154, 0.4388934247654853, 0.24918503610056578);
      Quaternion orientation41 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size41 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box41 = new Box3D(position41, orientation41, size41);
      terrain.addRotatableBox(box41, appearance);

      Point3D position42 = new Point3D(3.588806285529547, 0.15484013654183854, 0.24678723679981535);
      Quaternion orientation42 = new Quaternion(0.006622056925947793, 0.006517902589659019, 0.7126585328208642,0.7014495569216039);
      Vector3D size42 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box42 = new Box3D(position42, orientation42, size42);
      terrain.addRotatableBox(box42, appearance);

      Point3D position43 = new Point3D(3.6964906211913506, -0.14386449366407408, 0.24588881335187482);
      Quaternion orientation43 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size43 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box43 = new Box3D(position43, orientation43, size43);
      terrain.addRotatableBox(box43, appearance);

      Point3D position44 = new Point3D(3.6860421913290207, -0.33336370841482466, 0.24832633958306163);
      Quaternion orientation44 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size44 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box44 = new Box3D(position44, orientation44, size44);
      terrain.addRotatableBox(box44, appearance);

      Point3D position45 = new Point3D(3.9745492483528864, -0.23633221329182877, 0.2477310686455436);
      Quaternion orientation45 = new Quaternion(0.0, 0.0, 0.7074123606850091,0.70680106957337);
      Vector3D size45 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box45 = new Box3D(position45, orientation45, size45);
      terrain.addRotatableBox(box45, appearance);

      Point3D position46 = new Point3D(4.1570571372942995, 0.5351500725177174, 0.24857256496015911);
      Quaternion orientation46 = new Quaternion(0.0, 0.0, -0.7031379195773428,0.7110534902891946);
      Vector3D size46 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box46 = new Box3D(position46, orientation46, size46);
      terrain.addRotatableBox(box46, appearance);

      Point3D position47 = new Point3D(3.976449906945254, 0.5313406371863271, 0.25024361307815357);
      Quaternion orientation47 = new Quaternion(0.0, 0.0, 0.7053477339779689,0.7088614633143371);
      Vector3D size47 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box47 = new Box3D(position47, orientation47, size47);
      terrain.addRotatableBox(box47, appearance);

      Point3D position48 = new Point3D(4.060087891099629, 0.2392964224398486, 0.24882388075031026);
      Quaternion orientation48 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size48 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box48 = new Box3D(position48, orientation48, size48);
      terrain.addRotatableBox(box48, appearance);

      Point3D position49 = new Point3D(4.063602710036821, 0.052570508322739594, 0.2470906830235654);
      Quaternion orientation49 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size49 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box49 = new Box3D(position49, orientation49, size49);
      terrain.addRotatableBox(box49, appearance);

      Point3D position50 = new Point3D(4.147655172749554, -0.2365996880994438, 0.2511546494352128);
      Quaternion orientation50 = new Quaternion(0.0, 0.0, -0.7069099517221287,0.7073035558769786);
      Vector3D size50 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box50 = new Box3D(position50, orientation50, size50);
      terrain.addRotatableBox(box50, appearance);

      Point3D position51 = new Point3D(4.343356717444665, -0.23511809278230003, 0.24226611243093557);
      Quaternion orientation51 = new Quaternion(-0.011094912138101922, -0.016513086855919283, 0.7006319373826693,0.7132454761206297);
      Vector3D size51 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box51 = new Box3D(position51, orientation51, size51);
      terrain.addRotatableBox(box51, appearance);

      Point3D position52 = new Point3D(4.349272116834419, 0.15033640054496056, 0.23830329314841697);
      Quaternion orientation52 = new Quaternion(0.0, 0.0, -0.711526223471408,0.7026595429598301);
      Vector3D size52 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box52 = new Box3D(position52, orientation52, size52);
      terrain.addRotatableBox(box52, appearance);

      Point3D position53 = new Point3D(4.346481880050672, 0.5267456748407474, 0.24375773924450508);
      Quaternion orientation53 = new Quaternion(0.0, 0.0, -0.706513253073592,0.7076998115241873);
      Vector3D size53 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box53 = new Box3D(position53, orientation53, size53);
      terrain.addRotatableBox(box53, appearance);

      Point3D position54 = new Point3D(4.702380013634424, 0.6079336148133885, 0.3479183895982199);
      Quaternion orientation54 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size54 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box54 = new Box3D(position54, orientation54, size54);
      terrain.addRotatableBox(box54, appearance);

      Point3D position55 = new Point3D(4.703890280086529, 0.4155963177652986, 0.34604430953711496);
      Quaternion orientation55 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size55 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box55 = new Box3D(position55, orientation55, size55);
      terrain.addRotatableBox(box55, appearance);

      Point3D position56 = new Point3D(4.792888242703473, 0.12489420821974778, 0.3479451232275179);
      Quaternion orientation56 = new Quaternion(0.0, 0.0, -0.7035865871495103,0.7106095372166822);
      Vector3D size56 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box56 = new Box3D(position56, orientation56, size56);
      terrain.addRotatableBox(box56, appearance);

      Point3D position57 = new Point3D(4.610354981743812, 0.12911728605722894, 0.3475132664494318);
      Quaternion orientation57 = new Quaternion(0.0, 0.0, -0.7132505655401294,0.7009091458646314);
      Vector3D size57 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box57 = new Box3D(position57, orientation57, size57);
      terrain.addRotatableBox(box57, appearance);

      Point3D position58 = new Point3D(4.712970930242387, -0.1602010896506567, 0.34378735053868653);
      Quaternion orientation58 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size58 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box58 = new Box3D(position58, orientation58, size58);
      terrain.addRotatableBox(box58, appearance);

      Point3D position59 = new Point3D(4.707914463288, -0.34860534277151983, 0.34697375743836595);
      Quaternion orientation59 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size59 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box59 = new Box3D(position59, orientation59, size59);
      terrain.addRotatableBox(box59, appearance);

      Point3D position60 = new Point3D(5.1727760970934185, 0.5112340340750825, 0.3478127861059397);
      Quaternion orientation60 = new Quaternion(0.0, 0.0, -0.7027188610745567,0.7114676396647134);
      Vector3D size60 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box60 = new Box3D(position60, orientation60, size60);
      terrain.addRotatableBox(box60, appearance);

      Point3D position61 = new Point3D(4.986578998410523, 0.5045728830128698, 0.35723923234878907);
      Quaternion orientation61 = new Quaternion(0.0, 0.0, 0.7072488450361675,0.7069646887893388);
      Vector3D size61 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box61 = new Box3D(position61, orientation61, size61);
      terrain.addRotatableBox(box61, appearance);

      Point3D position62 = new Point3D(5.0805297951234305, 0.21436861966465973, 0.3490365620576039);
      Quaternion orientation62 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size62 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box62 = new Box3D(position62, orientation62, size62);
      terrain.addRotatableBox(box62, appearance);

      Point3D position63 = new Point3D(5.082643679829409, 0.025622949614205873, 0.34617777218674023);
      Quaternion orientation63 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size63 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box63 = new Box3D(position63, orientation63, size63);
      terrain.addRotatableBox(box63, appearance);

      Point3D position64 = new Point3D(5.181134977089534, -0.2648254795801891, 0.34319902751110637);
      Quaternion orientation64 = new Quaternion(0.0, 0.0, -0.7087044927674657,0.705505451383057);
      Vector3D size64 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box64 = new Box3D(position64, orientation64, size64);
      terrain.addRotatableBox(box64, appearance);

      Point3D position65 = new Point3D(5.001168160091194, -0.26897331063196755, 0.3442518243348339);
      Quaternion orientation65 = new Quaternion(0.0, 0.0, 0.7085911489028206,0.7056192909045078);
      Vector3D size65 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box65 = new Box3D(position65, orientation65, size65);
      terrain.addRotatableBox(box65, appearance);

      Point3D position66 = new Point3D(5.361423516842161, -0.26268163874752193, 0.3467424072643232);
      Quaternion orientation66 = new Quaternion(0.0, 0.0, -0.702830138774019,0.7113577131309495);
      Vector3D size66 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box66 = new Box3D(position66, orientation66, size66);
      terrain.addRotatableBox(box66, appearance);

      Point3D position67 = new Point3D(5.368768670021752, 0.1171590670986501, 0.3453752030174442);
      Quaternion orientation67 = new Quaternion(0.0, 0.0, -0.7086738484704129,0.7055362332964443);
      Vector3D size67 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box67 = new Box3D(position67, orientation67, size67);
      terrain.addRotatableBox(box67, appearance);

      Point3D position68 = new Point3D(5.359430458701423, 0.5074604944131372, 0.3480032395435422);
      Quaternion orientation68 = new Quaternion(0.0, 0.0, 0.7069593494590138,0.7072541821809802);
      Vector3D size68 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box68 = new Box3D(position68, orientation68, size68);
      terrain.addRotatableBox(box68, appearance);

      Point3D position69 = new Point3D(5.75258973537161, -0.3504907819884919, 0.227418511493009);
      Quaternion orientation69 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size69 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box69 = new Box3D(position69, orientation69, size69);
      terrain.addRotatableBox(box69, appearance);

      Point3D position70 = new Point3D(5.753494342409687, -0.1684521815005659, 0.2278999120441889);
      Quaternion orientation70 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size70 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box70 = new Box3D(position70, orientation70, size70);
      terrain.addRotatableBox(box70, appearance);

      Point3D position71 = new Point3D(5.648721454782096, 0.12274996284824707, 0.22909123665228792);
      Quaternion orientation71 = new Quaternion(0.0, 0.0, -0.7107650931702626,0.7034294437473249);
      Vector3D size71 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box71 = new Box3D(position71, orientation71, size71);
      terrain.addRotatableBox(box71, appearance);

      Point3D position72 = new Point3D(5.840403719567024, 0.12495103853557084, 0.2307081992117488);
      Quaternion orientation72 = new Quaternion(0.0, 0.0, 0.705341788004333,0.7088673797649667);
      Vector3D size72 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box72 = new Box3D(position72, orientation72, size72);
      terrain.addRotatableBox(box72, appearance);

      Point3D position73 = new Point3D(5.7577226491388185, 0.4116362919569683, 0.23430691862402694);
      Quaternion orientation73 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size73 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box73 = new Box3D(position73, orientation73, size73);
      terrain.addRotatableBox(box73, appearance);

      Point3D position74 = new Point3D(5.764448795159363, 0.6034155096950473, 0.24155583755511856);
      Quaternion orientation74 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size74 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box74 = new Box3D(position74, orientation74, size74);
      terrain.addRotatableBox(box74, appearance);

      Point3D position75 = new Point3D(6.2352107198329065, -0.25880611548136095, 0.22412120969808713);
      Quaternion orientation75 = new Quaternion(0.0, 0.0, 0.7050698828375693,0.7091378288565752);
      Vector3D size75 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box75 = new Box3D(position75, orientation75, size75);
      terrain.addRotatableBox(box75, appearance);

      Point3D position76 = new Point3D(6.041424771298938, -0.2617975766406595, 0.23217475146219316);
      Quaternion orientation76 = new Quaternion(0.0, 0.0, -0.7079095834255491,0.7063030664624527);
      Vector3D size76 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box76 = new Box3D(position76, orientation76, size76);
      terrain.addRotatableBox(box76, appearance);

      Point3D position77 = new Point3D(6.124353907177331, 0.21270454422081966, 0.23404920128215498);
      Quaternion orientation77 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size77 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box77 = new Box3D(position77, orientation77, size77);
      terrain.addRotatableBox(box77, appearance);

      Point3D position78 = new Point3D(6.421349657210157, -0.2658011220542116, 0.2311280740993358);
      Quaternion orientation78 = new Quaternion(0.0, 0.0, 0.7008708717943496,0.7132881753332446);
      Vector3D size78 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box78 = new Box3D(position78, orientation78, size78);
      terrain.addRotatableBox(box78, appearance);

      Point3D position79 = new Point3D(6.2386546198104, 0.5019196938501862, 0.24330105462308782);
      Quaternion orientation79 = new Quaternion(0.0, 0.0, 0.7008772020274565,0.7132819552450227);
      Vector3D size79 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box79 = new Box3D(position79, orientation79, size79);
      terrain.addRotatableBox(box79, appearance);

      Point3D position80 = new Point3D(6.051805117872733, 0.5075380042699142, 0.2462076993245688);
      Quaternion orientation80 = new Quaternion(-0.003546939645528666, -0.003543267978614838, 0.7074639704526886,0.7067316286841134);
      Vector3D size80 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box80 = new Box3D(position80, orientation80, size80);
      terrain.addRotatableBox(box80, appearance);

      Point3D position81 = new Point3D(6.122908361470702, 0.020253416476653883, 0.2312384964958033);
      Quaternion orientation81 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size81 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box81 = new Box3D(position81, orientation81, size81);
      terrain.addRotatableBox(box81, appearance);

      Point3D position82 = new Point3D(6.409268200039842, 0.11934128728302902, 0.23604260025687943);
      Quaternion orientation82 = new Quaternion(0.0, 0.0, 0.7016890425161328,0.7124833244454162);
      Vector3D size82 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box82 = new Box3D(position82, orientation82, size82);
      terrain.addRotatableBox(box82, appearance);

      Point3D position83 = new Point3D(6.429966481857306, 0.500996096184173, 0.24551820736884225);
      Quaternion orientation83 = new Quaternion(0.0, 0.0, 0.7027244324751094,0.7114621367314882);
      Vector3D size83 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box83 = new Box3D(position83, orientation83, size83);
      terrain.addRotatableBox(box83, appearance);

      Point3D position84 = new Point3D(6.699741918264115, -0.3631569365460855, 0.21492116687566903);
      Quaternion orientation84 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size84 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box84 = new Box3D(position84, orientation84, size84);
      terrain.addRotatableBox(box84, appearance);

      Point3D position85 = new Point3D(6.698630441512664, -0.17503310876007605, 0.21724682814725804);
      Quaternion orientation85 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size85 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box85 = new Box3D(position85, orientation85, size85);
      terrain.addRotatableBox(box85, appearance);

      Point3D position86 = new Point3D(6.797501566111999, 0.11234335851675088, 0.2166656716058247);
      Quaternion orientation86 = new Quaternion(0.0, 0.0, 0.6984451166535963,0.7156636214191308);
      Vector3D size86 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box86 = new Box3D(position86, orientation86, size86);
      terrain.addRotatableBox(box86, appearance);

      Point3D position87 = new Point3D(6.61320792222252, 0.11105183808493378, 0.21603636247814803);
      Quaternion orientation87 = new Quaternion(-0.006736059144041472, 0.007390816837914501, 0.7020028580344309,0.7121039170269788);
      Vector3D size87 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box87 = new Box3D(position87, orientation87, size87);
      terrain.addRotatableBox(box87, appearance);

      Point3D position88 = new Point3D(6.7127304242037615, 0.4038501715863721, 0.21934622785676056);
      Quaternion orientation88 = new Quaternion(0.0, 0.0, -0.003814431622533768,0.9999927250292361);
      Vector3D size88 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box88 = new Box3D(position88, orientation88, size88);
      terrain.addRotatableBox(box88, appearance);

      Point3D position89 = new Point3D(6.718241475052683, 0.5961877211864752, 0.214411381592355);
      Quaternion orientation89 = new Quaternion(0.0, 0.0, -0.009312552251322236,0.9999566372451187);
      Vector3D size89 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box89 = new Box3D(position89, orientation89, size89);
      terrain.addRotatableBox(box89, appearance);

      Point3D position90 = new Point3D(6.983093459558476, -0.2800509790382789, 0.21557870456215963);
      Quaternion orientation90 = new Quaternion(0.0, 0.0, 0.703035834272026,0.7111544246712078);
      Vector3D size90 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box90 = new Box3D(position90, orientation90, size90);
      terrain.addRotatableBox(box90, appearance);

      Point3D position91 = new Point3D(6.988683516017568, 0.11099979053069783, 0.21687826978610028);
      Quaternion orientation91 = new Quaternion(0.0, 0.0, 0.6996747373431306,0.7144615188544595);
      Vector3D size91 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box91 = new Box3D(position91, orientation91, size91);
      terrain.addRotatableBox(box91, appearance);

      Point3D position92 = new Point3D(7.00489079733141, 0.5035039836470866, 0.2167604197386731);
      Quaternion orientation92 = new Quaternion(0.0, 0.0, 0.7019672015374098,0.7122092725917974);
      Vector3D size92 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box92 = new Box3D(position92, orientation92, size92);
      terrain.addRotatableBox(box92, appearance);

      Point3D position93 = new Point3D(7.345613673095441, -0.3621818158794301, 0.18851614172025807);
      Quaternion orientation93 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size93 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box93 = new Box3D(position93, orientation93, size93);
      terrain.addRotatableBox(box93, appearance);

      Point3D position94 = new Point3D(7.346728260451079, -0.17617424269482315, 0.19148708763994512);
      Quaternion orientation94 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size94 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box94 = new Box3D(position94, orientation94, size94);
      terrain.addRotatableBox(box94, appearance);

      Point3D position95 = new Point3D(7.256713781322195, 0.11746933621526329, 0.19034824545194998);
      Quaternion orientation95 = new Quaternion(0.0, 0.0, -0.7109847545862545,0.7032074222773266);
      Vector3D size95 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box95 = new Box3D(position95, orientation95, size95);
      terrain.addRotatableBox(box95, appearance);

      Point3D position96 = new Point3D(7.436520392537069, 0.1116348793188018, 0.18978383802880525);
      Quaternion orientation96 = new Quaternion(0.0, 0.0, 0.7079054853446145,0.7063071738408194);
      Vector3D size96 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box96 = new Box3D(position96, orientation96, size96);
      terrain.addRotatableBox(box96, appearance);

      Point3D position97 = new Point3D(7.356717455086338, 0.40091839642545996, 0.1894719544174172);
      Quaternion orientation97 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size97 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box97 = new Box3D(position97, orientation97, size97);
      terrain.addRotatableBox(box97, appearance);

      Point3D position98 = new Point3D(7.3566302205035035, 0.587809313874047, 0.1878985545160597);
      Quaternion orientation98 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size98 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box98 = new Box3D(position98, orientation98, size98);
      terrain.addRotatableBox(box98, appearance);

      Point3D position99 = new Point3D(7.770134742471376, -0.40055264320453654, 0.047256945955076055);
      Quaternion orientation99 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size99 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box99 = new Box3D(position99, orientation99, size99);
      terrain.addRotatableBox(box99, appearance);

      Point3D position100 = new Point3D(7.770217973789706, -0.2137607588909391, 0.04430979308496823);
      Quaternion orientation100 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size100 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box100 = new Box3D(position100, orientation100, size100);
      terrain.addRotatableBox(box100, appearance);

      Point3D position101 = new Point3D(7.869196761927994, 0.07594085472191646, 0.04358384009610045);
      Quaternion orientation101 = new Quaternion(0.0, 0.0, 0.7034682501294439,0.7107266852031212);
      Vector3D size101 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box101 = new Box3D(position101, orientation101, size101);
      terrain.addRotatableBox(box101, appearance);

      Point3D position102 = new Point3D(7.682970194739092, 0.07705257011165333, 0.047271829008987146);
      Quaternion orientation102 = new Quaternion(0.0, 0.0, 0.7034450413479779,0.7107496562102162);
      Vector3D size102 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box102 = new Box3D(position102, orientation102, size102);
      terrain.addRotatableBox(box102, appearance);

      Point3D position103 = new Point3D(7.779256902384008, 0.3659919336394692, 0.04432955446553308);
      Quaternion orientation103 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size103 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box103 = new Box3D(position103, orientation103, size103);
      terrain.addRotatableBox(box103, appearance);

      Point3D position104 = new Point3D(7.785783227158354, 0.5557054734046523, 0.045068700704124554);
      Quaternion orientation104 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size104 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box104 = new Box3D(position104, orientation104, size104);
      terrain.addRotatableBox(box104, appearance);

      Point3D position105 = new Point3D(8.252153182891886, -0.3056293119640772, 0.04285320203865197);
      Quaternion orientation105 = new Quaternion(0.0, 0.0, 0.7041815061508303,0.7100200042218157);
      Vector3D size105 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box105 = new Box3D(position105, orientation105, size105);
      terrain.addRotatableBox(box105, appearance);

      Point3D position106 = new Point3D(8.064675866921874, -0.2987549941396834, 0.046016306708178786);
      Quaternion orientation106 = new Quaternion(0.0, 0.0, 0.7060501535261506,0.7081618322853174);
      Vector3D size106 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box106 = new Box3D(position106, orientation106, size106);
      terrain.addRotatableBox(box106, appearance);

      Point3D position107 = new Point3D(8.152449598918341, -0.020698347797655257, 0.047082419865977894);
      Quaternion orientation107 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size107 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box107 = new Box3D(position107, orientation107, size107);
      terrain.addRotatableBox(box107, appearance);

      Point3D position108 = new Point3D(8.15561274022191, 0.16606239445418075, 0.04851321195122305);
      Quaternion orientation108 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size108 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box108 = new Box3D(position108, orientation108, size108);
      terrain.addRotatableBox(box108, appearance);

      Point3D position109 = new Point3D(8.06995087947249, 0.4533393922315608, 0.04470427328368063);
      Quaternion orientation109 = new Quaternion(0.0, 0.0, 0.7089663975812708,0.7052422612837637);
      Vector3D size109 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box109 = new Box3D(position109, orientation109, size109);
      terrain.addRotatableBox(box109, appearance);

      Point3D position110 = new Point3D(8.256712637978552, 0.45548448733570646, 0.0465446546756541);
      Quaternion orientation110 = new Quaternion(0.0, 0.0, 0.7057265405863627,0.708484332862771);
      Vector3D size110 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box110 = new Box3D(position110, orientation110, size110);
      terrain.addRotatableBox(box110, appearance);

      Point3D position111 = new Point3D(8.534695414460673, -0.40215227023449407, 0.0436624434311707);
      Quaternion orientation111 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size111 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box111 = new Box3D(position111, orientation111, size111);
      terrain.addRotatableBox(box111, appearance);

      Point3D position112 = new Point3D(8.540900973445865, -0.21132852297590346, 0.04400585133181643);
      Quaternion orientation112 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size112 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box112 = new Box3D(position112, orientation112, size112);
      terrain.addRotatableBox(box112, appearance);

      Point3D position113 = new Point3D(8.446166935149789, 0.07613432146555155, 0.04558588339207196);
      Quaternion orientation113 = new Quaternion(0.0, 0.0, 0.7016853567916598,0.7124869543115587);
      Vector3D size113 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box113 = new Box3D(position113, orientation113, size113);
      terrain.addRotatableBox(box113, appearance);

      Point3D position114 = new Point3D(8.629100011417771, 0.078953161543596, 0.0401527710286485);
      Quaternion orientation114 = new Quaternion(0.0, 0.0, 0.7096844738128036,0.7045196573758916);
      Vector3D size114 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box114 = new Box3D(position114, orientation114, size114);
      terrain.addRotatableBox(box114, appearance);

      Point3D position115 = new Point3D(8.544205652116847, 0.363513099370666, 0.04861356986695628);
      Quaternion orientation115 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size115 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box115 = new Box3D(position115, orientation115, size115);
      terrain.addRotatableBox(box115, appearance);

      Point3D position116 = new Point3D(8.5399389891589, 0.555300276983753, 0.04766115185873509);
      Quaternion orientation116 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size116 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box116 = new Box3D(position116, orientation116, size116);
      terrain.addRotatableBox(box116, appearance);
   }

   private static void addLookAndStepHard(CombinedTerrainObject3D terrain)
   {
      AppearanceDefinition appearance = YoAppearance.DarkGray();

      Point3D position0 = new Point3D(2.9991922858880433, 0.17738506103097837, 0.07749255480474931);
      Quaternion orientation0 = new Quaternion(3.591011281523848E-4, -4.404093410555292E-4, -0.7140274385984735,0.7001174858654047);
      Vector3D size0 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box0 = new Box3D(position0, orientation0, size0);
      terrain.addRotatableBox(box0, appearance);

      Point3D position1 = new Point3D(4.010313695756378, 0.1616056331805633, 0.07664394760851354);
      Quaternion orientation1 = new Quaternion(0.0010814627453136847, 0.0011373925186028025, -0.7128015132552289,0.7013640563057426);
      Vector3D size1 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box1 = new Box3D(position1, orientation1, size1);
      terrain.addRotatableBox(box1, appearance);

      Point3D position2 = new Point3D(1.538532083708091, 0.6741701950310802, 0.05131964350890132);
      Quaternion orientation2 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size2 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box2 = new Box3D(position2, orientation2, size2);
      terrain.addRotatableBox(box2, appearance);

      Point3D position3 = new Point3D(1.5377122418466396, 0.4871995187212327, 0.0453699739307727);
      Quaternion orientation3 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size3 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box3 = new Box3D(position3, orientation3, size3);
      terrain.addRotatableBox(box3, appearance);

      Point3D position4 = new Point3D(1.8318418085847752, 0.5832432291386377, 0.05026246658367628);
      Quaternion orientation4 = new Quaternion(0.0, 0.0, -0.7032798171527006,0.7109131443331625);
      Vector3D size4 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box4 = new Box3D(position4, orientation4, size4);
      terrain.addRotatableBox(box4, appearance);

      Point3D position5 = new Point3D(2.0226812492006916, 0.5816220959595908, 0.058535600491900285);
      Quaternion orientation5 = new Quaternion(0.0, 0.0, -0.7033244049950813,0.7108690324794821);
      Vector3D size5 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box5 = new Box3D(position5, orientation5, size5);
      terrain.addRotatableBox(box5, appearance);

      Point3D position6 = new Point3D(1.9258075527472558, 0.2916058787804267, 0.04914770991489481);
      Quaternion orientation6 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size6 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box6 = new Box3D(position6, orientation6, size6);
      terrain.addRotatableBox(box6, appearance);

      Point3D position7 = new Point3D(1.9221267562656839, 0.0971054615729663, 0.048374686104427383);
      Quaternion orientation7 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size7 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box7 = new Box3D(position7, orientation7, size7);
      terrain.addRotatableBox(box7, appearance);

      Point3D position8 = new Point3D(1.6391139250203717, 0.20003028808817114, 0.03909051303520233);
      Quaternion orientation8 = new Quaternion(4.2221998162823653E-4, -0.0021771612644779165, -0.7083346233855763,0.7058733193798512);
      Vector3D size8 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box8 = new Box3D(position8, orientation8, size8);
      terrain.addRotatableBox(box8, appearance);

      Point3D position9 = new Point3D(1.4508258759747759, 0.19187378577710404, 0.03772504674415519);
      Quaternion orientation9 = new Quaternion(0.0, 0.0, 0.7038510074598656,0.7103476327107258);
      Vector3D size9 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box9 = new Box3D(position9, orientation9, size9);
      terrain.addRotatableBox(box9, appearance);

      Point3D position10 = new Point3D(1.544254053576254, -0.09368499925145309, 0.032965432002729965);
      Quaternion orientation10 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size10 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box10 = new Box3D(position10, orientation10, size10);
      terrain.addRotatableBox(box10, appearance);

      Point3D position11 = new Point3D(1.5421604510537665, -0.2847681953194701, 0.03169062655032244);
      Quaternion orientation11 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size11 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box11 = new Box3D(position11, orientation11, size11);
      terrain.addRotatableBox(box11, appearance);

      Point3D position12 = new Point3D(1.8302064340875628, -0.18772505668791095, 0.03233411894842265);
      Quaternion orientation12 = new Quaternion(0.0, 0.0, -0.7041659007794283,0.7100354809300001);
      Vector3D size12 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box12 = new Box3D(position12, orientation12, size12);
      terrain.addRotatableBox(box12, appearance);

      Point3D position13 = new Point3D(2.020076111592974, -0.19450183861517623, 0.032951829300953354);
      Quaternion orientation13 = new Quaternion(0.0, 0.0, 0.7094730567086567,0.7047325604826807);
      Vector3D size13 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box13 = new Box3D(position13, orientation13, size13);
      terrain.addRotatableBox(box13, appearance);

      Point3D position14 = new Point3D(2.3017726687557736, 0.6659211869311393, 0.050732948560174566);
      Quaternion orientation14 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size14 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box14 = new Box3D(position14, orientation14, size14);
      terrain.addRotatableBox(box14, appearance);

      Point3D position15 = new Point3D(2.3045613478518816, 0.4744061119031097, 0.04678489901068887);
      Quaternion orientation15 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size15 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box15 = new Box3D(position15, orientation15, size15);
      terrain.addRotatableBox(box15, appearance);

      Point3D position16 = new Point3D(2.214159178501207, 0.19092562262741894, 0.04546863118006792);
      Quaternion orientation16 = new Quaternion(0.0, 0.0, -0.710045270209128,0.7041560297644597);
      Vector3D size16 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box16 = new Box3D(position16, orientation16, size16);
      terrain.addRotatableBox(box16, appearance);

      Point3D position17 = new Point3D(2.3882625449800243, 0.1854857053173351, 0.04358061639227621);
      Quaternion orientation17 = new Quaternion(0.0, 0.0, 0.7032305077721192,0.7109619208780226);
      Vector3D size17 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box17 = new Box3D(position17, orientation17, size17);
      terrain.addRotatableBox(box17, appearance);

      Point3D position18 = new Point3D(2.305334951328886, -0.09669125069873875, 0.04026317060808564);
      Quaternion orientation18 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size18 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box18 = new Box3D(position18, orientation18, size18);
      terrain.addRotatableBox(box18, appearance);

      Point3D position19 = new Point3D(2.3035618801058884, -0.28517226345878166, 0.043838376674458175);
      Quaternion orientation19 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size19 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box19 = new Box3D(position19, orientation19, size19);
      terrain.addRotatableBox(box19, appearance);

      Point3D position20 = new Point3D(5.022827545101327, 0.14359272115178368, 0.06929944023803199);
      Quaternion orientation20 = new Quaternion(0.0, 0.0, -0.7146326573404546,0.6994999392870028);
      Vector3D size20 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box20 = new Box3D(position20, orientation20, size20);
      terrain.addRotatableBox(box20, appearance);

      Point3D position21 = new Point3D(6.0368497952581395, 0.10384214494564183, 0.06172802598521781);
      Quaternion orientation21 = new Quaternion(0.005613854283208643, 0.0055820130552961455, -0.7143726400604131,0.6997206992103728);
      Vector3D size21 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box21 = new Box3D(position21, orientation21, size21);
      terrain.addRotatableBox(box21, appearance);

      Point3D position22 = new Point3D(7.050523277840206, 0.08289463608396092, 0.06864253875907342);
      Quaternion orientation22 = new Quaternion(0.0, 0.0, 0.7017961773791698,0.7123777968297335);
      Vector3D size22 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box22 = new Box3D(position22, orientation22, size22);
      terrain.addRotatableBox(box22, appearance);

      Point3D position23 = new Point3D(5.023846234292196, 0.13613703474961703, 0.22602295943394107);
      Quaternion orientation23 = new Quaternion(0.0, 0.0, 0.7009087265528491,0.71325097759629);
      Vector3D size23 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box23 = new Box3D(position23, orientation23, size23);
      terrain.addRotatableBox(box23, appearance);

      Point3D position24 = new Point3D(2.6900015009577163, 0.6436366016069017, 0.19963009876119422);
      Quaternion orientation24 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size24 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box24 = new Box3D(position24, orientation24, size24);
      terrain.addRotatableBox(box24, appearance);

      Point3D position25 = new Point3D(2.6890564650088904, 0.4543195036924751, 0.1998904919894186);
      Quaternion orientation25 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size25 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box25 = new Box3D(position25, orientation25, size25);
      terrain.addRotatableBox(box25, appearance);

      Point3D position26 = new Point3D(2.7797616190377004, 0.16496187008841534, 0.24403572659127876);
      Quaternion orientation26 = new Quaternion(0.09461018209558089, 0.09407245118837755, -0.7027793486796466,0.6987849987232575);
      Vector3D size26 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box26 = new Box3D(position26, orientation26, size26);
      terrain.addRotatableBox(box26, appearance);

      Point3D position27 = new Point3D(2.593463551704851, 0.1621908804922451, 0.24443600486584452);
      Quaternion orientation27 = new Quaternion(0.09234953713579733, -0.09290494477039353, 0.698908799916006,0.7031121700183992);
      Vector3D size27 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box27 = new Box3D(position27, orientation27, size27);
      terrain.addRotatableBox(box27, appearance);

      Point3D position28 = new Point3D(2.684024192233144, -0.13061899475743363, 0.19603696340587146);
      Quaternion orientation28 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size28 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box28 = new Box3D(position28, orientation28, size28);
      terrain.addRotatableBox(box28, appearance);

      Point3D position29 = new Point3D(2.6860150703200567, -0.320451962389396, 0.19828815283388773);
      Quaternion orientation29 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size29 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box29 = new Box3D(position29, orientation29, size29);
      terrain.addRotatableBox(box29, appearance);

      Point3D position30 = new Point3D(3.1711383025454465, 0.5462180514407013, 0.28166428275520083);
      Quaternion orientation30 = new Quaternion(0.10793831101703633, 0.10894156349569274, -0.6955023910392171,0.7019668658965574);
      Vector3D size30 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box30 = new Box3D(position30, orientation30, size30);
      terrain.addRotatableBox(box30, appearance);

      Point3D position31 = new Point3D(2.9818424101161205, 0.5424769634680712, 0.28083800074621);
      Quaternion orientation31 = new Quaternion(0.11223315066187517, -0.11138798116147307, 0.7008445880063211,0.6955668918281909);
      Vector3D size31 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box31 = new Box3D(position31, orientation31, size31);
      terrain.addRotatableBox(box31, appearance);

      Point3D position32 = new Point3D(3.0756668786094545, 0.2573398848240338, 0.22430098450189007);
      Quaternion orientation32 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size32 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box32 = new Box3D(position32, orientation32, size32);
      terrain.addRotatableBox(box32, appearance);

      Point3D position33 = new Point3D(3.0722024139657997, 0.06783466183720784, 0.2251948202315323);
      Quaternion orientation33 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size33 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box33 = new Box3D(position33, orientation33, size33);
      terrain.addRotatableBox(box33, appearance);

      Point3D position34 = new Point3D(3.165664362999164, -0.22522204408849275, 0.27735751751742505);
      Quaternion orientation34 = new Quaternion(0.09437843324707647, 0.09404280191325493, -0.7020498469308203,0.6995531968124677);
      Vector3D size34 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box34 = new Box3D(position34, orientation34, size34);
      terrain.addRotatableBox(box34, appearance);

      Point3D position35 = new Point3D(2.974840053678399, -0.22519875771755868, 0.2788927721576176);
      Quaternion orientation35 = new Quaternion(0.09994070761295208, -0.09964072755297639, 0.7010812345776699,0.6989768829493477);
      Vector3D size35 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box35 = new Box3D(position35, orientation35, size35);
      terrain.addRotatableBox(box35, appearance);

      Point3D position36 = new Point3D(3.359147869089035, 0.550879534175364, 0.22486106332834727);
      Quaternion orientation36 = new Quaternion(0.0, 0.0, -0.7071194347376097,0.7070941274090483);
      Vector3D size36 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box36 = new Box3D(position36, orientation36, size36);
      terrain.addRotatableBox(box36, appearance);

      Point3D position37 = new Point3D(3.354472458529534, 0.15783397615504235, 0.22395977136362658);
      Quaternion orientation37 = new Quaternion(0.0, 0.0, -0.7058785319428242,0.7083329006492944);
      Vector3D size37 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box37 = new Box3D(position37, orientation37, size37);
      terrain.addRotatableBox(box37, appearance);

      Point3D position38 = new Point3D(3.350402623341776, -0.22691663331160317, 0.22640085739147484);
      Quaternion orientation38 = new Quaternion(0.0, 0.0, 0.7062242020231959,0.7079882601263245);
      Vector3D size38 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box38 = new Box3D(position38, orientation38, size38);
      terrain.addRotatableBox(box38, appearance);

      Point3D position39 = new Point3D(3.68866762247556, 0.6236265834842826, 0.3010889371484357);
      Quaternion orientation39 = new Quaternion(-5.839236425510448E-34, 0.14469060323807462, 8.859744206110364E-18,0.9894769473487506);
      Vector3D size39 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box39 = new Box3D(position39, orientation39, size39);
      terrain.addRotatableBox(box39, appearance);

      Point3D position40 = new Point3D(3.7706107394082076, 0.15186184665978525, 0.2940640116728258);
      Quaternion orientation40 = new Quaternion(-0.08864451472761907, -0.08908549481655237, -0.6997578049577696,0.7032388918587189);
      Vector3D size40 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box40 = new Box3D(position40, orientation40, size40);
      terrain.addRotatableBox(box40, appearance);

      Point3D position41 = new Point3D(3.68965927337154, 0.4388934247654853, 0.30028645012494665);
      Quaternion orientation41 = new Quaternion(1.9454724300576844E-34, 0.14138358070604393, 8.657247478182429E-18,0.989954889430189);
      Vector3D size41 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box41 = new Box3D(position41, orientation41, size41);
      terrain.addRotatableBox(box41, appearance);

      Point3D position42 = new Point3D(3.588806285529547, 0.15484013654183854, 0.2930951718708826);
      Quaternion orientation42 = new Quaternion(-0.08427871374180397, 0.09588259957957021, 0.7076885857849924,0.6948960289550722);
      Vector3D size42 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box42 = new Box3D(position42, orientation42, size42);
      terrain.addRotatableBox(box42, appearance);

      Point3D position43 = new Point3D(3.6964906211913506, -0.14386449366407408, 0.24588881335187482);
      Quaternion orientation43 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size43 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box43 = new Box3D(position43, orientation43, size43);
      terrain.addRotatableBox(box43, appearance);

      Point3D position44 = new Point3D(3.6860421913290207, -0.33336370841482466, 0.24832633958306163);
      Quaternion orientation44 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size44 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box44 = new Box3D(position44, orientation44, size44);
      terrain.addRotatableBox(box44, appearance);

      Point3D position45 = new Point3D(3.9745492483528864, -0.23633221329182877, 0.3000158195677145);
      Quaternion orientation45 = new Quaternion(0.09976532226249955, -0.09967911277827533, 0.7003421510403348,0.6997369694576615);
      Vector3D size45 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box45 = new Box3D(position45, orientation45, size45);
      terrain.addRotatableBox(box45, appearance);

      Point3D position46 = new Point3D(4.1570571372942995, 0.5351500725177174, 0.3009665879051755);
      Quaternion orientation46 = new Quaternion(0.0, 0.0, -0.7031379195773428,0.7110534902891948);
      Vector3D size46 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box46 = new Box3D(position46, orientation46, size46);
      terrain.addRotatableBox(box46, appearance);

      Point3D position47 = new Point3D(3.976449906945254, 0.5313406371863271, 0.30163369997832445);
      Quaternion orientation47 = new Quaternion(0.0, 0.0, 0.7053477339779695,0.7088614633143374);
      Vector3D size47 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box47 = new Box3D(position47, orientation47, size47);
      terrain.addRotatableBox(box47, appearance);

      Point3D position48 = new Point3D(4.060087891099629, 0.2392964224398486, 0.24882388075031026);
      Quaternion orientation48 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size48 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box48 = new Box3D(position48, orientation48, size48);
      terrain.addRotatableBox(box48, appearance);

      Point3D position49 = new Point3D(4.063602710036821, 0.052570508322739594, 0.2470906830235654);
      Quaternion orientation49 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size49 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box49 = new Box3D(position49, orientation49, size49);
      terrain.addRotatableBox(box49, appearance);

      Point3D position50 = new Point3D(4.147655172749554, -0.2365996880994438, 0.2989374948751531);
      Quaternion orientation50 = new Quaternion(0.09713879505240461, 0.09719288147916494, -0.7002040662118151,0.7005939365609611);
      Vector3D size50 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box50 = new Box3D(position50, orientation50, size50);
      terrain.addRotatableBox(box50, appearance);

      Point3D position51 = new Point3D(4.343356717444665, -0.23511809278230003, 0.30128052384367776);
      Quaternion orientation51 = new Quaternion(0.0844889112032617, -0.11355828784903144, 0.6956075277336033,0.7043410441716627);
      Vector3D size51 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box51 = new Box3D(position51, orientation51, size51);
      terrain.addRotatableBox(box51, appearance);

      Point3D position52 = new Point3D(4.349272116834419, 0.15033640054496056, 0.23830329314841697);
      Quaternion orientation52 = new Quaternion(0.0, 0.0, -0.7115262234714076,0.7026595429598296);
      Vector3D size52 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box52 = new Box3D(position52, orientation52, size52);
      terrain.addRotatableBox(box52, appearance);

      Point3D position53 = new Point3D(4.346481880050672, 0.5267456748407474, 0.3000392880382346);
      Quaternion orientation53 = new Quaternion(0.0, 0.0, -0.7065132530735921,0.7076998115241875);
      Vector3D size53 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box53 = new Box3D(position53, orientation53, size53);
      terrain.addRotatableBox(box53, appearance);

      Point3D position54 = new Point3D(4.702380013634424, 0.6079336148133885, 0.3797638118070186);
      Quaternion orientation54 = new Quaternion(1.6449470919030792E-33, 0.09792926395956664, 5.9964379825470255E-18,0.9951933778217867);
      Vector3D size54 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box54 = new Box3D(position54, orientation54, size54);
      terrain.addRotatableBox(box54, appearance);

      Point3D position55 = new Point3D(4.703890280086529, 0.4155963177652986, 0.37962892936597686);
      Quaternion orientation55 = new Quaternion(1.9358735945371405E-34, 0.10122575159006256, 6.1982896338026795E-18,0.9948634816973769);
      Vector3D size55 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box55 = new Box3D(position55, orientation55, size55);
      terrain.addRotatableBox(box55, appearance);

      Point3D position56 = new Point3D(4.792888242703473, 0.12489420821974778, 0.3479451232275179);
      Quaternion orientation56 = new Quaternion(0.0, 0.0, -0.7035865871495094,0.7106095372166812);
      Vector3D size56 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box56 = new Box3D(position56, orientation56, size56);
      terrain.addRotatableBox(box56, appearance);

      Point3D position57 = new Point3D(4.610354981743812, 0.12911728605722894, 0.3475132664494318);
      Quaternion orientation57 = new Quaternion(0.0, 0.0, -0.7132505655401289,0.7009091458646308);
      Vector3D size57 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box57 = new Box3D(position57, orientation57, size57);
      terrain.addRotatableBox(box57, appearance);

      Point3D position58 = new Point3D(4.712970930242387, -0.1602010896506567, 0.3912550730752458);
      Quaternion orientation58 = new Quaternion(-3.880765412410759E-34, -0.12182502599465175, -7.459631407019642E-18,0.9925515921308083);
      Vector3D size58 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box58 = new Box3D(position58, orientation58, size58);
      terrain.addRotatableBox(box58, appearance);

      Point3D position59 = new Point3D(4.707914463288, -0.34860534277151983, 0.3891170604695318);
      Quaternion orientation59 = new Quaternion(2.3291328065487197E-33, -0.1241412431733023, -7.601458804717881E-18,0.9922645573350823);
      Vector3D size59 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box59 = new Box3D(position59, orientation59, size59);
      terrain.addRotatableBox(box59, appearance);

      Point3D position60 = new Point3D(5.1727760970934185, 0.5112340340750825, 0.3967895082089883);
      Quaternion orientation60 = new Quaternion(0.09472131621407967, 0.09590058699963736, -0.6963057302396933,0.7049746660009598);
      Vector3D size60 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box60 = new Box3D(position60, orientation60, size60);
      terrain.addRotatableBox(box60, appearance);

      Point3D position61 = new Point3D(4.986578998410523, 0.5045728830128698, 0.3944322980633405);
      Quaternion orientation61 = new Quaternion(0.09774852819681804, -0.09770925509640942, 0.7004613865448716,0.700179957342037);
      Vector3D size61 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box61 = new Box3D(position61, orientation61, size61);
      terrain.addRotatableBox(box61, appearance);

      Point3D position62 = new Point3D(5.181946498191193, 0.11593862123374693, 0.3935301841017318);
      Quaternion orientation62 = new Quaternion(-0.09454705714016172, 0.09446453379644658, 0.7010687973196257,0.7004568846585553);
      Vector3D size62 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box62 = new Box3D(position62, orientation62, size62);
      terrain.addRotatableBox(box62, appearance);

      Point3D position63 = new Point3D(4.999908472643262, 0.12271227322859121, 0.3936477908269693);
      Quaternion orientation63 = new Quaternion(-0.0943438948778358, -0.0947342877428095, -0.6993100325946019,0.7022037614095209);
      Vector3D size63 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box63 = new Box3D(position63, orientation63, size63);
      terrain.addRotatableBox(box63, appearance);

      Point3D position64 = new Point3D(5.181134977089534, -0.2648254795801891, 0.34319902751110637);
      Quaternion orientation64 = new Quaternion(0.0, 0.0, -0.7087044927674649,0.7055054513830564);
      Vector3D size64 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box64 = new Box3D(position64, orientation64, size64);
      terrain.addRotatableBox(box64, appearance);

      Point3D position65 = new Point3D(5.001168160091194, -0.26897331063196755, 0.3442518243348339);
      Quaternion orientation65 = new Quaternion(0.0, 0.0, 0.7085911489028207,0.7056192909045079);
      Vector3D size65 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box65 = new Box3D(position65, orientation65, size65);
      terrain.addRotatableBox(box65, appearance);

      Point3D position66 = new Point3D(5.361423516842161, -0.26268163874752193, 0.3467424072643232);
      Quaternion orientation66 = new Quaternion(0.0, 0.0, -0.702830138774019,0.7113577131309489);
      Vector3D size66 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box66 = new Box3D(position66, orientation66, size66);
      terrain.addRotatableBox(box66, appearance);

      Point3D position67 = new Point3D(5.368768670021752, 0.1171590670986501, 0.38910279983955076);
      Quaternion orientation67 = new Quaternion(-0.09458307226381174, -0.09416431082175267, -0.7023337283279424,0.6992241836933095);
      Vector3D size67 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box67 = new Box3D(position67, orientation67, size67);
      terrain.addRotatableBox(box67, appearance);

      Point3D position68 = new Point3D(5.359430458701423, 0.5074604944131372, 0.3951426086328146);
      Quaternion orientation68 = new Quaternion(0.09529196339253196, -0.09533170427575388, 0.7005076469963307,0.7007997890784077);
      Vector3D size68 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box68 = new Box3D(position68, orientation68, size68);
      terrain.addRotatableBox(box68, appearance);

      Point3D position69 = new Point3D(5.75258973537161, -0.3504907819884919, 0.227418511493009);
      Quaternion orientation69 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size69 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box69 = new Box3D(position69, orientation69, size69);
      terrain.addRotatableBox(box69, appearance);

      Point3D position70 = new Point3D(5.753494342409687, -0.1684521815005659, 0.2278999120441889);
      Quaternion orientation70 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size70 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box70 = new Box3D(position70, orientation70, size70);
      terrain.addRotatableBox(box70, appearance);

      Point3D position71 = new Point3D(5.648721454782096, 0.12274996284824707, 0.27732476658080424);
      Quaternion orientation71 = new Quaternion(0.0, 0.0, -0.7107650931702618,0.7034294437473235);
      Vector3D size71 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box71 = new Box3D(position71, orientation71, size71);
      terrain.addRotatableBox(box71, appearance);

      Point3D position72 = new Point3D(5.840403719567024, 0.12495103853557084, 0.27650382371337645);
      Quaternion orientation72 = new Quaternion(0.0, 0.0, 0.7053417880043341,0.7088673797649665);
      Vector3D size72 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box72 = new Box3D(position72, orientation72, size72);
      terrain.addRotatableBox(box72, appearance);

      Point3D position73 = new Point3D(5.7577226491388185, 0.4116362919569683, 0.28389741291306736);
      Quaternion orientation73 = new Quaternion(0.0, 0.11262021465784755, 6.89599927000101E-18,0.993638106782555);
      Vector3D size73 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box73 = new Box3D(position73, orientation73, size73);
      terrain.addRotatableBox(box73, appearance);

      Point3D position74 = new Point3D(5.764448795159363, 0.6034155096950473, 0.28467265927325774);
      Quaternion orientation74 = new Quaternion(-1.0658594972039655E-33, 0.11109613212788319, 6.802676130403094E-18,0.9938096645868483);
      Vector3D size74 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box74 = new Box3D(position74, orientation74, size74);
      terrain.addRotatableBox(box74, appearance);

      Point3D position75 = new Point3D(6.2352107198329065, -0.25880611548136095, 0.2684214567166878);
      Quaternion orientation75 = new Quaternion(-0.09099327976522753, 0.09151827134292549, 0.6991736284516058,0.7032075556518044);
      Vector3D size75 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box75 = new Box3D(position75, orientation75, size75);
      terrain.addRotatableBox(box75, appearance);

      Point3D position76 = new Point3D(6.041424771298938, -0.2617975766406595, 0.2701155633865791);
      Quaternion orientation76 = new Quaternion(-0.09049938070702158, -0.09029400308019075, -0.7021010186557202,0.7005076835424435);
      Vector3D size76 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box76 = new Box3D(position76, orientation76, size76);
      terrain.addRotatableBox(box76, appearance);

      Point3D position77 = new Point3D(6.124353907177331, 0.21270454422081966, 0.23404920128215498);
      Quaternion orientation77 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size77 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box77 = new Box3D(position77, orientation77, size77);
      terrain.addRotatableBox(box77, appearance);

      Point3D position78 = new Point3D(6.421349657210157, -0.2658011220542116, 0.26733468034992436);
      Quaternion orientation78 = new Quaternion(-0.09224861237772161, 0.09388297766102086, 0.6947734684371486,0.7070827445042763);
      Vector3D size78 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box78 = new Box3D(position78, orientation78, size78);
      terrain.addRotatableBox(box78, appearance);

      Point3D position79 = new Point3D(6.2386546198104, 0.5019196938501862, 0.2618318096377476);
      Quaternion orientation79 = new Quaternion(-0.0924918376852269, 0.09412884116858322, 0.6947475169322426,0.7070437814862335);
      Vector3D size79 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box79 = new Box3D(position79, orientation79, size79);
      terrain.addRotatableBox(box79, appearance);

      Point3D position80 = new Point3D(6.051805117872733, 0.5075380042699142, 0.26005353871268083);
      Quaternion orientation80 = new Quaternion(-0.09694408237307763, 0.08981925984154682, 0.7007993258860914,0.7010097362319289);
      Vector3D size80 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box80 = new Box3D(position80, orientation80, size80);
      terrain.addRotatableBox(box80, appearance);

      Point3D position81 = new Point3D(6.122908361470702, 0.020253416476653883, 0.2312384964958033);
      Quaternion orientation81 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size81 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box81 = new Box3D(position81, orientation81, size81);
      terrain.addRotatableBox(box81, appearance);

      Point3D position82 = new Point3D(6.409268200039842, 0.11934128728302902, 0.23604260025687943);
      Quaternion orientation82 = new Quaternion(0.0, 0.0, 0.7016890425161331,0.7124833244454164);
      Vector3D size82 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box82 = new Box3D(position82, orientation82, size82);
      terrain.addRotatableBox(box82, appearance);

      Point3D position83 = new Point3D(6.429966481857306, 0.500996096184173, 0.26300251335578917);
      Quaternion orientation83 = new Quaternion(-0.09163052675672367, 0.092769864463877, 0.6967248198275796,0.7053879246555514);
      Vector3D size83 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box83 = new Box3D(position83, orientation83, size83);
      terrain.addRotatableBox(box83, appearance);

      Point3D position84 = new Point3D(6.699741918264115, -0.3631569365460855, 0.2550180515425791);
      Quaternion orientation84 = new Quaternion(1.7479183742545392E-33, 0.1288981738241679, 7.892736799485345E-18,0.9916578345300332);
      Vector3D size84 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box84 = new Box3D(position84, orientation84, size84);
      terrain.addRotatableBox(box84, appearance);

      Point3D position85 = new Point3D(6.698630441512664, -0.17503310876007605, 0.2526101274005956);
      Quaternion orientation85 = new Quaternion(-3.884427274751654E-34, 0.1292203005277809, 7.912461371310276E-18,0.9916159104872764);
      Vector3D size85 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box85 = new Box3D(position85, orientation85, size85);
      terrain.addRotatableBox(box85, appearance);

      Point3D position86 = new Point3D(6.797501566111999, 0.11234335851675088, 0.2633885198693278);
      Quaternion orientation86 = new Quaternion(0.09819637521077755, -0.10061717351576993, 0.691507811143679,0.708555293125695);
      Vector3D size86 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box86 = new Box3D(position86, orientation86, size86);
      terrain.addRotatableBox(box86, appearance);

      Point3D position87 = new Point3D(6.61320792222252, 0.11105183808493378, 0.2609523583831561);
      Quaternion orientation87 = new Quaternion(0.09905634457071809, -0.09993018618950954, 0.6950116745649737,0.7050961428818118);
      Vector3D size87 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box87 = new Box3D(position87, orientation87, size87);
      terrain.addRotatableBox(box87, appearance);

      Point3D position88 = new Point3D(6.7127304242037615, 0.4038501715863721, 0.21934622785676056);
      Quaternion orientation88 = new Quaternion(0.0, 0.0, -0.0038144316225337672,0.9999927250292359);
      Vector3D size88 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box88 = new Box3D(position88, orientation88, size88);
      terrain.addRotatableBox(box88, appearance);

      Point3D position89 = new Point3D(6.718241475052683, 0.5961877211864752, 0.214411381592355);
      Quaternion orientation89 = new Quaternion(0.0, 0.0, -0.009312552251322237,0.999956637245119);
      Vector3D size89 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box89 = new Box3D(position89, orientation89, size89);
      terrain.addRotatableBox(box89, appearance);

      Point3D position90 = new Point3D(6.983093459558476, -0.2800509790382789, 0.21557870456215963);
      Quaternion orientation90 = new Quaternion(0.0, 0.0, 0.7030358342720265,0.7111544246712073);
      Vector3D size90 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box90 = new Box3D(position90, orientation90, size90);
      terrain.addRotatableBox(box90, appearance);

      Point3D position91 = new Point3D(6.988683516017568, 0.11099979053069783, 0.25958187507798225);
      Quaternion orientation91 = new Quaternion(0.10036878834372606, -0.10248996160407609, 0.6924383325630104,0.7070721813889761);
      Vector3D size91 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box91 = new Box3D(position91, orientation91, size91);
      terrain.addRotatableBox(box91, appearance);

      Point3D position92 = new Point3D(7.00489079733141, 0.5035039836470866, 0.2167604197386731);
      Quaternion orientation92 = new Quaternion(0.0, 0.0, 0.7019672015374099,0.7122092725917976);
      Vector3D size92 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box92 = new Box3D(position92, orientation92, size92);
      terrain.addRotatableBox(box92, appearance);

      Point3D position93 = new Point3D(7.345613673095441, -0.3621818158794301, 0.18851614172025807);
      Quaternion orientation93 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size93 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box93 = new Box3D(position93, orientation93, size93);
      terrain.addRotatableBox(box93, appearance);

      Point3D position94 = new Point3D(7.346728260451079, -0.17617424269482315, 0.19148708763994512);
      Quaternion orientation94 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size94 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box94 = new Box3D(position94, orientation94, size94);
      terrain.addRotatableBox(box94, appearance);

      Point3D position95 = new Point3D(7.256713781322195, 0.11746933621526329, 0.19034824545194998);
      Quaternion orientation95 = new Quaternion(0.0, 0.0, -0.7109847545862554,0.7032074222773266);
      Vector3D size95 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box95 = new Box3D(position95, orientation95, size95);
      terrain.addRotatableBox(box95, appearance);

      Point3D position96 = new Point3D(7.436520392537069, 0.1116348793188018, 0.18978383802880525);
      Quaternion orientation96 = new Quaternion(0.0, 0.0, 0.7079054853446147,0.7063071738408194);
      Vector3D size96 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box96 = new Box3D(position96, orientation96, size96);
      terrain.addRotatableBox(box96, appearance);

      Point3D position97 = new Point3D(7.356717455086338, 0.40091839642545996, 0.22230115969485045);
      Quaternion orientation97 = new Quaternion(0.0, 0.11441875733241588, 7.006128246478086E-18,0.9934326086708176);
      Vector3D size97 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box97 = new Box3D(position97, orientation97, size97);
      terrain.addRotatableBox(box97, appearance);

      Point3D position98 = new Point3D(7.3566302205035035, 0.587809313874047, 0.22224355113151553);
      Quaternion orientation98 = new Quaternion(-1.5505289284134202E-33, 0.11216790604418489, 6.8683033552035814E-18,0.9936892677561043);
      Vector3D size98 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box98 = new Box3D(position98, orientation98, size98);
      terrain.addRotatableBox(box98, appearance);

      Point3D position99 = new Point3D(7.770134742471376, -0.40055264320453654, 0.047256945955076055);
      Quaternion orientation99 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size99 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box99 = new Box3D(position99, orientation99, size99);
      terrain.addRotatableBox(box99, appearance);

      Point3D position100 = new Point3D(7.770217973789706, -0.2137607588909391, 0.04430979308496823);
      Quaternion orientation100 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size100 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box100 = new Box3D(position100, orientation100, size100);
      terrain.addRotatableBox(box100, appearance);

      Point3D position101 = new Point3D(7.869196761927994, 0.07594085472191646, 0.04358384009610045);
      Quaternion orientation101 = new Quaternion(0.0, 0.0, 0.703468250129445,0.7107266852031212);
      Vector3D size101 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box101 = new Box3D(position101, orientation101, size101);
      terrain.addRotatableBox(box101, appearance);

      Point3D position102 = new Point3D(7.682970194739092, 0.07705257011165333, 0.047271829008987146);
      Quaternion orientation102 = new Quaternion(0.0, 0.0, 0.7034450413479783,0.7107496562102167);
      Vector3D size102 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box102 = new Box3D(position102, orientation102, size102);
      terrain.addRotatableBox(box102, appearance);

      Point3D position103 = new Point3D(7.779256902384008, 0.3659919336394692, 0.04432955446553308);
      Quaternion orientation103 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size103 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box103 = new Box3D(position103, orientation103, size103);
      terrain.addRotatableBox(box103, appearance);

      Point3D position104 = new Point3D(7.785783227158354, 0.5557054734046523, 0.045068700704124554);
      Quaternion orientation104 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size104 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box104 = new Box3D(position104, orientation104, size104);
      terrain.addRotatableBox(box104, appearance);

      Point3D position105 = new Point3D(8.252153182891886, -0.3056293119640772, 0.04285320203865197);
      Quaternion orientation105 = new Quaternion(0.0, 0.0, 0.704181506150831,0.7100200042218158);
      Vector3D size105 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box105 = new Box3D(position105, orientation105, size105);
      terrain.addRotatableBox(box105, appearance);

      Point3D position106 = new Point3D(8.064675866921874, -0.2987549941396834, 0.046016306708178786);
      Quaternion orientation106 = new Quaternion(0.0, 0.0, 0.7060501535261512,0.7081618322853177);
      Vector3D size106 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box106 = new Box3D(position106, orientation106, size106);
      terrain.addRotatableBox(box106, appearance);

      Point3D position107 = new Point3D(8.152449598918341, -0.020698347797655257, 0.047082419865977894);
      Quaternion orientation107 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size107 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box107 = new Box3D(position107, orientation107, size107);
      terrain.addRotatableBox(box107, appearance);

      Point3D position108 = new Point3D(8.15561274022191, 0.16606239445418075, 0.04851321195122305);
      Quaternion orientation108 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size108 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box108 = new Box3D(position108, orientation108, size108);
      terrain.addRotatableBox(box108, appearance);

      Point3D position109 = new Point3D(8.06995087947249, 0.4533393922315608, 0.04470427328368063);
      Quaternion orientation109 = new Quaternion(0.0, 0.0, 0.7089663975812704,0.7052422612837633);
      Vector3D size109 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box109 = new Box3D(position109, orientation109, size109);
      terrain.addRotatableBox(box109, appearance);

      Point3D position110 = new Point3D(8.256712637978552, 0.45548448733570646, 0.0465446546756541);
      Quaternion orientation110 = new Quaternion(0.0, 0.0, 0.705726540586363,0.7084843328627703);
      Vector3D size110 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box110 = new Box3D(position110, orientation110, size110);
      terrain.addRotatableBox(box110, appearance);

      Point3D position111 = new Point3D(8.534695414460673, -0.40215227023449407, 0.0436624434311707);
      Quaternion orientation111 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size111 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box111 = new Box3D(position111, orientation111, size111);
      terrain.addRotatableBox(box111, appearance);

      Point3D position112 = new Point3D(8.540900973445865, -0.21132852297590346, 0.04400585133181643);
      Quaternion orientation112 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size112 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box112 = new Box3D(position112, orientation112, size112);
      terrain.addRotatableBox(box112, appearance);

      Point3D position113 = new Point3D(8.446166935149789, 0.07613432146555155, 0.04558588339207196);
      Quaternion orientation113 = new Quaternion(0.0, 0.0, 0.7016853567916603,0.7124869543115583);
      Vector3D size113 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box113 = new Box3D(position113, orientation113, size113);
      terrain.addRotatableBox(box113, appearance);

      Point3D position114 = new Point3D(8.629100011417771, 0.078953161543596, 0.0401527710286485);
      Quaternion orientation114 = new Quaternion(0.0, 0.0, 0.7096844738128042,0.7045196573758914);
      Vector3D size114 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box114 = new Box3D(position114, orientation114, size114);
      terrain.addRotatableBox(box114, appearance);

      Point3D position115 = new Point3D(8.544205652116847, 0.363513099370666, 0.04861356986695628);
      Quaternion orientation115 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size115 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box115 = new Box3D(position115, orientation115, size115);
      terrain.addRotatableBox(box115, appearance);

      Point3D position116 = new Point3D(8.5399389891589, 0.555300276983753, 0.04766115185873509);
      Quaternion orientation116 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size116 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box116 = new Box3D(position116, orientation116, size116);
      terrain.addRotatableBox(box116, appearance);
   }

   private static void addFootstepPlannerTrainingTerrainGenerated(CombinedTerrainObject3D terrain)
   {
      Point3D position0 = new Point3D(2.9991922858880433, 0.17738506103097837, 0.07749255480474931);
      Quaternion orientation0 = new Quaternion(3.591011281523848E-4, -4.404093410555292E-4, -0.7140274385984735,0.7001174858654047);
      Vector3D size0 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box0 = new Box3D(position0, orientation0, size0);
      terrain.addRotatableBox(box0, YoAppearance.DarkGray());

      Point3D position1 = new Point3D(4.010313695756378, 0.1616056331805633, 0.07664394760851354);
      Quaternion orientation1 = new Quaternion(0.0010814627453136847, 0.0011373925186028025, -0.7128015132552289,0.7013640563057426);
      Vector3D size1 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box1 = new Box3D(position1, orientation1, size1);
      terrain.addRotatableBox(box1, YoAppearance.DarkGray());

      Point3D position2 = new Point3D(1.538532083708091, 0.6741701950310802, 0.05131964350890132);
      Quaternion orientation2 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size2 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box2 = new Box3D(position2, orientation2, size2);
      terrain.addRotatableBox(box2, YoAppearance.DarkGray());

      Point3D position3 = new Point3D(1.5377122418466396, 0.4871995187212327, 0.0453699739307727);
      Quaternion orientation3 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size3 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box3 = new Box3D(position3, orientation3, size3);
      terrain.addRotatableBox(box3, YoAppearance.DarkGray());

      Point3D position4 = new Point3D(1.8318418085847752, 0.5832432291386377, 0.05026246658367628);
      Quaternion orientation4 = new Quaternion(0.0, 0.0, -0.7032798171527006,0.7109131443331625);
      Vector3D size4 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box4 = new Box3D(position4, orientation4, size4);
      terrain.addRotatableBox(box4, YoAppearance.DarkGray());

      Point3D position5 = new Point3D(2.0226812492006916, 0.5816220959595908, 0.058535600491900285);
      Quaternion orientation5 = new Quaternion(0.0, 0.0, -0.7033244049950813,0.7108690324794821);
      Vector3D size5 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box5 = new Box3D(position5, orientation5, size5);
      terrain.addRotatableBox(box5, YoAppearance.DarkGray());

      Point3D position6 = new Point3D(1.9258075527472558, 0.2916058787804267, 0.04914770991489481);
      Quaternion orientation6 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size6 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box6 = new Box3D(position6, orientation6, size6);
      terrain.addRotatableBox(box6, YoAppearance.DarkGray());

      Point3D position7 = new Point3D(1.9221267562656839, 0.0971054615729663, 0.048374686104427383);
      Quaternion orientation7 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size7 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box7 = new Box3D(position7, orientation7, size7);
      terrain.addRotatableBox(box7, YoAppearance.DarkGray());

      Point3D position8 = new Point3D(1.6391139250203717, 0.20003028808817114, 0.03909051303520233);
      Quaternion orientation8 = new Quaternion(4.2221998162823653E-4, -0.0021771612644779165, -0.7083346233855763,0.7058733193798512);
      Vector3D size8 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box8 = new Box3D(position8, orientation8, size8);
      terrain.addRotatableBox(box8, YoAppearance.DarkGray());

      Point3D position9 = new Point3D(1.4508258759747759, 0.19187378577710404, 0.03772504674415519);
      Quaternion orientation9 = new Quaternion(0.0, 0.0, 0.7038510074598656,0.7103476327107258);
      Vector3D size9 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box9 = new Box3D(position9, orientation9, size9);
      terrain.addRotatableBox(box9, YoAppearance.DarkGray());

      Point3D position10 = new Point3D(1.544254053576254, -0.09368499925145309, 0.032965432002729965);
      Quaternion orientation10 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size10 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box10 = new Box3D(position10, orientation10, size10);
      terrain.addRotatableBox(box10, YoAppearance.DarkGray());

      Point3D position11 = new Point3D(1.5421604510537665, -0.2847681953194701, 0.03169062655032244);
      Quaternion orientation11 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size11 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box11 = new Box3D(position11, orientation11, size11);
      terrain.addRotatableBox(box11, YoAppearance.DarkGray());

      Point3D position12 = new Point3D(1.8302064340875628, -0.18772505668791095, 0.03233411894842265);
      Quaternion orientation12 = new Quaternion(0.0, 0.0, -0.7041659007794283,0.7100354809300001);
      Vector3D size12 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box12 = new Box3D(position12, orientation12, size12);
      terrain.addRotatableBox(box12, YoAppearance.DarkGray());

      Point3D position13 = new Point3D(2.020076111592974, -0.19450183861517623, 0.032951829300953354);
      Quaternion orientation13 = new Quaternion(0.0, 0.0, 0.7094730567086567,0.7047325604826807);
      Vector3D size13 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box13 = new Box3D(position13, orientation13, size13);
      terrain.addRotatableBox(box13, YoAppearance.DarkGray());

      Point3D position14 = new Point3D(2.3017726687557736, 0.6659211869311393, 0.050732948560174566);
      Quaternion orientation14 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size14 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box14 = new Box3D(position14, orientation14, size14);
      terrain.addRotatableBox(box14, YoAppearance.DarkGray());

      Point3D position15 = new Point3D(2.3045613478518816, 0.4744061119031097, 0.04678489901068887);
      Quaternion orientation15 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size15 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box15 = new Box3D(position15, orientation15, size15);
      terrain.addRotatableBox(box15, YoAppearance.DarkGray());

      Point3D position16 = new Point3D(2.214159178501207, 0.19092562262741894, 0.04546863118006792);
      Quaternion orientation16 = new Quaternion(0.0, 0.0, -0.710045270209128,0.7041560297644597);
      Vector3D size16 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box16 = new Box3D(position16, orientation16, size16);
      terrain.addRotatableBox(box16, YoAppearance.DarkGray());

      Point3D position17 = new Point3D(2.3882625449800243, 0.1854857053173351, 0.04358061639227621);
      Quaternion orientation17 = new Quaternion(0.0, 0.0, 0.7032305077721192,0.7109619208780226);
      Vector3D size17 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box17 = new Box3D(position17, orientation17, size17);
      terrain.addRotatableBox(box17, YoAppearance.DarkGray());

      Point3D position18 = new Point3D(2.305334951328886, -0.09669125069873875, 0.04026317060808564);
      Quaternion orientation18 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size18 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box18 = new Box3D(position18, orientation18, size18);
      terrain.addRotatableBox(box18, YoAppearance.DarkGray());

      Point3D position19 = new Point3D(2.3035618801058884, -0.28517226345878166, 0.043838376674458175);
      Quaternion orientation19 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size19 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box19 = new Box3D(position19, orientation19, size19);
      terrain.addRotatableBox(box19, YoAppearance.DarkGray());

      Point3D position20 = new Point3D(5.022827545101327, 0.14359272115178368, 0.06929944023803199);
      Quaternion orientation20 = new Quaternion(0.0, 0.0, -0.7146326573404546,0.6994999392870028);
      Vector3D size20 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box20 = new Box3D(position20, orientation20, size20);
      terrain.addRotatableBox(box20, YoAppearance.DarkGray());

      Point3D position21 = new Point3D(6.0368497952581395, 0.10384214494564183, 0.06172802598521781);
      Quaternion orientation21 = new Quaternion(0.005613854283208643, 0.0055820130552961455, -0.7143726400604131,0.6997206992103728);
      Vector3D size21 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box21 = new Box3D(position21, orientation21, size21);
      terrain.addRotatableBox(box21, YoAppearance.DarkGray());

      Point3D position22 = new Point3D(7.050523277840206, 0.08289463608396092, 0.06864253875907342);
      Quaternion orientation22 = new Quaternion(0.0, 0.0, 0.7017961773791698,0.7123777968297335);
      Vector3D size22 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box22 = new Box3D(position22, orientation22, size22);
      terrain.addRotatableBox(box22, YoAppearance.DarkGray());

      Point3D position23 = new Point3D(5.023846234292196, 0.13613703474961703, 0.22602295943394107);
      Quaternion orientation23 = new Quaternion(0.0, 0.0, 0.7009087265528491,0.71325097759629);
      Vector3D size23 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box23 = new Box3D(position23, orientation23, size23);
      terrain.addRotatableBox(box23, YoAppearance.DarkGray());

      Point3D position24 = new Point3D(2.6900015009577163, 0.6436366016069017, 0.19963009876119422);
      Quaternion orientation24 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size24 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box24 = new Box3D(position24, orientation24, size24);
      terrain.addRotatableBox(box24, YoAppearance.DarkGray());

      Point3D position25 = new Point3D(2.6890564650088904, 0.4543195036924751, 0.1998904919894186);
      Quaternion orientation25 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size25 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box25 = new Box3D(position25, orientation25, size25);
      terrain.addRotatableBox(box25, YoAppearance.DarkGray());

      Point3D position26 = new Point3D(2.7797616190377004, 0.16496187008841534, 0.24403572659127876);
      Quaternion orientation26 = new Quaternion(0.09461018209558089, 0.09407245118837755, -0.7027793486796466,0.6987849987232575);
      Vector3D size26 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box26 = new Box3D(position26, orientation26, size26);
      terrain.addRotatableBox(box26, YoAppearance.DarkGray());

      Point3D position27 = new Point3D(2.593463551704851, 0.1621908804922451, 0.24443600486584452);
      Quaternion orientation27 = new Quaternion(0.09234953713579733, -0.09290494477039353, 0.698908799916006,0.7031121700183992);
      Vector3D size27 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box27 = new Box3D(position27, orientation27, size27);
      terrain.addRotatableBox(box27, YoAppearance.DarkGray());

      Point3D position28 = new Point3D(2.684024192233144, -0.13061899475743363, 0.19603696340587146);
      Quaternion orientation28 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size28 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box28 = new Box3D(position28, orientation28, size28);
      terrain.addRotatableBox(box28, YoAppearance.DarkGray());

      Point3D position29 = new Point3D(2.6860150703200567, -0.320451962389396, 0.19828815283388773);
      Quaternion orientation29 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size29 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box29 = new Box3D(position29, orientation29, size29);
      terrain.addRotatableBox(box29, YoAppearance.DarkGray());

      Point3D position30 = new Point3D(3.1711383025454465, 0.5462180514407013, 0.28166428275520083);
      Quaternion orientation30 = new Quaternion(0.10793831101703633, 0.10894156349569274, -0.6955023910392171,0.7019668658965574);
      Vector3D size30 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box30 = new Box3D(position30, orientation30, size30);
      terrain.addRotatableBox(box30, YoAppearance.DarkGray());

      Point3D position31 = new Point3D(2.9818424101161205, 0.5424769634680712, 0.28083800074621);
      Quaternion orientation31 = new Quaternion(0.11223315066187517, -0.11138798116147307, 0.7008445880063211,0.6955668918281909);
      Vector3D size31 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box31 = new Box3D(position31, orientation31, size31);
      terrain.addRotatableBox(box31, YoAppearance.DarkGray());

      Point3D position32 = new Point3D(3.0756668786094545, 0.2573398848240338, 0.22430098450189007);
      Quaternion orientation32 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size32 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box32 = new Box3D(position32, orientation32, size32);
      terrain.addRotatableBox(box32, YoAppearance.DarkGray());

      Point3D position33 = new Point3D(3.0722024139657997, 0.06783466183720784, 0.2251948202315323);
      Quaternion orientation33 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size33 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box33 = new Box3D(position33, orientation33, size33);
      terrain.addRotatableBox(box33, YoAppearance.DarkGray());

      Point3D position34 = new Point3D(3.165664362999164, -0.22522204408849275, 0.27735751751742505);
      Quaternion orientation34 = new Quaternion(0.09437843324707647, 0.09404280191325493, -0.7020498469308203,0.6995531968124677);
      Vector3D size34 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box34 = new Box3D(position34, orientation34, size34);
      terrain.addRotatableBox(box34, YoAppearance.DarkGray());

      Point3D position35 = new Point3D(2.974840053678399, -0.22519875771755868, 0.2788927721576176);
      Quaternion orientation35 = new Quaternion(0.09994070761295208, -0.09964072755297639, 0.7010812345776699,0.6989768829493477);
      Vector3D size35 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box35 = new Box3D(position35, orientation35, size35);
      terrain.addRotatableBox(box35, YoAppearance.DarkGray());

      Point3D position36 = new Point3D(3.359147869089035, 0.550879534175364, 0.22486106332834727);
      Quaternion orientation36 = new Quaternion(0.0, 0.0, -0.7071194347376097,0.7070941274090483);
      Vector3D size36 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box36 = new Box3D(position36, orientation36, size36);
      terrain.addRotatableBox(box36, YoAppearance.DarkGray());

      Point3D position37 = new Point3D(3.354472458529534, 0.15783397615504235, 0.22395977136362658);
      Quaternion orientation37 = new Quaternion(0.0, 0.0, -0.7058785319428242,0.7083329006492944);
      Vector3D size37 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box37 = new Box3D(position37, orientation37, size37);
      terrain.addRotatableBox(box37, YoAppearance.DarkGray());

      Point3D position38 = new Point3D(3.350402623341776, -0.22691663331160317, 0.22640085739147484);
      Quaternion orientation38 = new Quaternion(0.0, 0.0, 0.7062242020231959,0.7079882601263245);
      Vector3D size38 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box38 = new Box3D(position38, orientation38, size38);
      terrain.addRotatableBox(box38, YoAppearance.DarkGray());

      Point3D position39 = new Point3D(3.68866762247556, 0.6236265834842826, 0.3010889371484357);
      Quaternion orientation39 = new Quaternion(-5.839236425510448E-34, 0.14469060323807462, 8.859744206110364E-18,0.9894769473487506);
      Vector3D size39 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box39 = new Box3D(position39, orientation39, size39);
      terrain.addRotatableBox(box39, YoAppearance.DarkGray());

      Point3D position40 = new Point3D(3.7706107394082076, 0.15186184665978525, 0.2940640116728258);
      Quaternion orientation40 = new Quaternion(-0.08864451472761907, -0.08908549481655237, -0.6997578049577696,0.7032388918587189);
      Vector3D size40 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box40 = new Box3D(position40, orientation40, size40);
      terrain.addRotatableBox(box40, YoAppearance.DarkGray());

      Point3D position41 = new Point3D(3.68965927337154, 0.4388934247654853, 0.30028645012494665);
      Quaternion orientation41 = new Quaternion(1.9454724300576844E-34, 0.14138358070604393, 8.657247478182429E-18,0.989954889430189);
      Vector3D size41 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box41 = new Box3D(position41, orientation41, size41);
      terrain.addRotatableBox(box41, YoAppearance.DarkGray());

      Point3D position42 = new Point3D(3.588806285529547, 0.15484013654183854, 0.2930951718708826);
      Quaternion orientation42 = new Quaternion(-0.08427871374180397, 0.09588259957957021, 0.7076885857849924,0.6948960289550722);
      Vector3D size42 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box42 = new Box3D(position42, orientation42, size42);
      terrain.addRotatableBox(box42, YoAppearance.DarkGray());

      Point3D position43 = new Point3D(3.6964906211913506, -0.14386449366407408, 0.24588881335187482);
      Quaternion orientation43 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size43 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box43 = new Box3D(position43, orientation43, size43);
      terrain.addRotatableBox(box43, YoAppearance.DarkGray());

      Point3D position44 = new Point3D(3.6860421913290207, -0.33336370841482466, 0.24832633958306163);
      Quaternion orientation44 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size44 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box44 = new Box3D(position44, orientation44, size44);
      terrain.addRotatableBox(box44, YoAppearance.DarkGray());

      Point3D position45 = new Point3D(3.9745492483528864, -0.23633221329182877, 0.3000158195677145);
      Quaternion orientation45 = new Quaternion(0.09976532226249955, -0.09967911277827533, 0.7003421510403348,0.6997369694576615);
      Vector3D size45 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box45 = new Box3D(position45, orientation45, size45);
      terrain.addRotatableBox(box45, YoAppearance.DarkGray());

      Point3D position46 = new Point3D(4.1570571372942995, 0.5351500725177174, 0.3009665879051755);
      Quaternion orientation46 = new Quaternion(0.0, 0.0, -0.7031379195773428,0.7110534902891948);
      Vector3D size46 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box46 = new Box3D(position46, orientation46, size46);
      terrain.addRotatableBox(box46, YoAppearance.DarkGray());

      Point3D position47 = new Point3D(3.976449906945254, 0.5313406371863271, 0.30163369997832445);
      Quaternion orientation47 = new Quaternion(0.0, 0.0, 0.7053477339779695,0.7088614633143374);
      Vector3D size47 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box47 = new Box3D(position47, orientation47, size47);
      terrain.addRotatableBox(box47, YoAppearance.DarkGray());

      Point3D position48 = new Point3D(4.060087891099629, 0.2392964224398486, 0.24882388075031026);
      Quaternion orientation48 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size48 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box48 = new Box3D(position48, orientation48, size48);
      terrain.addRotatableBox(box48, YoAppearance.DarkGray());

      Point3D position49 = new Point3D(4.063602710036821, 0.052570508322739594, 0.2470906830235654);
      Quaternion orientation49 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size49 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box49 = new Box3D(position49, orientation49, size49);
      terrain.addRotatableBox(box49, YoAppearance.DarkGray());

      Point3D position50 = new Point3D(4.147655172749554, -0.2365996880994438, 0.2989374948751531);
      Quaternion orientation50 = new Quaternion(0.09713879505240461, 0.09719288147916494, -0.7002040662118151,0.7005939365609611);
      Vector3D size50 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box50 = new Box3D(position50, orientation50, size50);
      terrain.addRotatableBox(box50, YoAppearance.DarkGray());

      Point3D position51 = new Point3D(4.343356717444665, -0.23511809278230003, 0.30128052384367776);
      Quaternion orientation51 = new Quaternion(0.0844889112032617, -0.11355828784903144, 0.6956075277336033,0.7043410441716627);
      Vector3D size51 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box51 = new Box3D(position51, orientation51, size51);
      terrain.addRotatableBox(box51, YoAppearance.DarkGray());

      Point3D position52 = new Point3D(4.349272116834419, 0.15033640054496056, 0.23830329314841697);
      Quaternion orientation52 = new Quaternion(0.0, 0.0, -0.7115262234714076,0.7026595429598296);
      Vector3D size52 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box52 = new Box3D(position52, orientation52, size52);
      terrain.addRotatableBox(box52, YoAppearance.DarkGray());

      Point3D position53 = new Point3D(4.346481880050672, 0.5267456748407474, 0.3000392880382346);
      Quaternion orientation53 = new Quaternion(0.0, 0.0, -0.7065132530735921,0.7076998115241875);
      Vector3D size53 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box53 = new Box3D(position53, orientation53, size53);
      terrain.addRotatableBox(box53, YoAppearance.DarkGray());

      Point3D position54 = new Point3D(4.702380013634424, 0.6079336148133885, 0.3797638118070186);
      Quaternion orientation54 = new Quaternion(1.6449470919030792E-33, 0.09792926395956664, 5.9964379825470255E-18,0.9951933778217867);
      Vector3D size54 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box54 = new Box3D(position54, orientation54, size54);
      terrain.addRotatableBox(box54, YoAppearance.DarkGray());

      Point3D position55 = new Point3D(4.703890280086529, 0.4155963177652986, 0.37962892936597686);
      Quaternion orientation55 = new Quaternion(1.9358735945371405E-34, 0.10122575159006256, 6.1982896338026795E-18,0.9948634816973769);
      Vector3D size55 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box55 = new Box3D(position55, orientation55, size55);
      terrain.addRotatableBox(box55, YoAppearance.DarkGray());

      Point3D position56 = new Point3D(4.792888242703473, 0.12489420821974778, 0.3479451232275179);
      Quaternion orientation56 = new Quaternion(0.0, 0.0, -0.7035865871495094,0.7106095372166812);
      Vector3D size56 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box56 = new Box3D(position56, orientation56, size56);
      terrain.addRotatableBox(box56, YoAppearance.DarkGray());

      Point3D position57 = new Point3D(4.610354981743812, 0.12911728605722894, 0.3475132664494318);
      Quaternion orientation57 = new Quaternion(0.0, 0.0, -0.7132505655401289,0.7009091458646308);
      Vector3D size57 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box57 = new Box3D(position57, orientation57, size57);
      terrain.addRotatableBox(box57, YoAppearance.DarkGray());

      Point3D position58 = new Point3D(4.712970930242387, -0.1602010896506567, 0.3912550730752458);
      Quaternion orientation58 = new Quaternion(-3.880765412410759E-34, -0.12182502599465175, -7.459631407019642E-18,0.9925515921308083);
      Vector3D size58 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box58 = new Box3D(position58, orientation58, size58);
      terrain.addRotatableBox(box58, YoAppearance.DarkGray());

      Point3D position59 = new Point3D(4.707914463288, -0.34860534277151983, 0.3891170604695318);
      Quaternion orientation59 = new Quaternion(2.3291328065487197E-33, -0.1241412431733023, -7.601458804717881E-18,0.9922645573350823);
      Vector3D size59 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box59 = new Box3D(position59, orientation59, size59);
      terrain.addRotatableBox(box59, YoAppearance.DarkGray());

      Point3D position60 = new Point3D(5.1727760970934185, 0.5112340340750825, 0.3967895082089883);
      Quaternion orientation60 = new Quaternion(0.09472131621407967, 0.09590058699963736, -0.6963057302396933,0.7049746660009598);
      Vector3D size60 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box60 = new Box3D(position60, orientation60, size60);
      terrain.addRotatableBox(box60, YoAppearance.DarkGray());

      Point3D position61 = new Point3D(4.986578998410523, 0.5045728830128698, 0.3944322980633405);
      Quaternion orientation61 = new Quaternion(0.09774852819681804, -0.09770925509640942, 0.7004613865448716,0.700179957342037);
      Vector3D size61 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box61 = new Box3D(position61, orientation61, size61);
      terrain.addRotatableBox(box61, YoAppearance.DarkGray());

      Point3D position62 = new Point3D(5.181946498191193, 0.11593862123374693, 0.3935301841017318);
      Quaternion orientation62 = new Quaternion(-0.09454705714016172, 0.09446453379644658, 0.7010687973196257,0.7004568846585553);
      Vector3D size62 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box62 = new Box3D(position62, orientation62, size62);
      terrain.addRotatableBox(box62, YoAppearance.DarkGray());

      Point3D position63 = new Point3D(4.999908472643262, 0.12271227322859121, 0.3936477908269693);
      Quaternion orientation63 = new Quaternion(-0.0943438948778358, -0.0947342877428095, -0.6993100325946019,0.7022037614095209);
      Vector3D size63 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box63 = new Box3D(position63, orientation63, size63);
      terrain.addRotatableBox(box63, YoAppearance.DarkGray());

      Point3D position64 = new Point3D(5.181134977089534, -0.2648254795801891, 0.34319902751110637);
      Quaternion orientation64 = new Quaternion(0.0, 0.0, -0.7087044927674649,0.7055054513830564);
      Vector3D size64 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box64 = new Box3D(position64, orientation64, size64);
      terrain.addRotatableBox(box64, YoAppearance.DarkGray());

      Point3D position65 = new Point3D(5.001168160091194, -0.26897331063196755, 0.3442518243348339);
      Quaternion orientation65 = new Quaternion(0.0, 0.0, 0.7085911489028207,0.7056192909045079);
      Vector3D size65 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box65 = new Box3D(position65, orientation65, size65);
      terrain.addRotatableBox(box65, YoAppearance.DarkGray());

      Point3D position66 = new Point3D(5.361423516842161, -0.26268163874752193, 0.3467424072643232);
      Quaternion orientation66 = new Quaternion(0.0, 0.0, -0.702830138774019,0.7113577131309489);
      Vector3D size66 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box66 = new Box3D(position66, orientation66, size66);
      terrain.addRotatableBox(box66, YoAppearance.DarkGray());

      Point3D position67 = new Point3D(5.368768670021752, 0.1171590670986501, 0.38910279983955076);
      Quaternion orientation67 = new Quaternion(-0.09458307226381174, -0.09416431082175267, -0.7023337283279424,0.6992241836933095);
      Vector3D size67 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box67 = new Box3D(position67, orientation67, size67);
      terrain.addRotatableBox(box67, YoAppearance.DarkGray());

      Point3D position68 = new Point3D(5.359430458701423, 0.5074604944131372, 0.3951426086328146);
      Quaternion orientation68 = new Quaternion(0.09529196339253196, -0.09533170427575388, 0.7005076469963307,0.7007997890784077);
      Vector3D size68 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box68 = new Box3D(position68, orientation68, size68);
      terrain.addRotatableBox(box68, YoAppearance.DarkGray());

      Point3D position69 = new Point3D(5.75258973537161, -0.3504907819884919, 0.227418511493009);
      Quaternion orientation69 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size69 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box69 = new Box3D(position69, orientation69, size69);
      terrain.addRotatableBox(box69, YoAppearance.DarkGray());

      Point3D position70 = new Point3D(5.753494342409687, -0.1684521815005659, 0.2278999120441889);
      Quaternion orientation70 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size70 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box70 = new Box3D(position70, orientation70, size70);
      terrain.addRotatableBox(box70, YoAppearance.DarkGray());

      Point3D position71 = new Point3D(5.648721454782096, 0.12274996284824707, 0.27732476658080424);
      Quaternion orientation71 = new Quaternion(0.0, 0.0, -0.7107650931702618,0.7034294437473235);
      Vector3D size71 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box71 = new Box3D(position71, orientation71, size71);
      terrain.addRotatableBox(box71, YoAppearance.DarkGray());

      Point3D position72 = new Point3D(5.840403719567024, 0.12495103853557084, 0.27650382371337645);
      Quaternion orientation72 = new Quaternion(0.0, 0.0, 0.7053417880043341,0.7088673797649665);
      Vector3D size72 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box72 = new Box3D(position72, orientation72, size72);
      terrain.addRotatableBox(box72, YoAppearance.DarkGray());

      Point3D position73 = new Point3D(5.7577226491388185, 0.4116362919569683, 0.28389741291306736);
      Quaternion orientation73 = new Quaternion(0.0, 0.11262021465784755, 6.89599927000101E-18,0.993638106782555);
      Vector3D size73 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box73 = new Box3D(position73, orientation73, size73);
      terrain.addRotatableBox(box73, YoAppearance.DarkGray());

      Point3D position74 = new Point3D(5.764448795159363, 0.6034155096950473, 0.28467265927325774);
      Quaternion orientation74 = new Quaternion(-1.0658594972039655E-33, 0.11109613212788319, 6.802676130403094E-18,0.9938096645868483);
      Vector3D size74 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box74 = new Box3D(position74, orientation74, size74);
      terrain.addRotatableBox(box74, YoAppearance.DarkGray());

      Point3D position75 = new Point3D(6.2352107198329065, -0.25880611548136095, 0.2684214567166878);
      Quaternion orientation75 = new Quaternion(-0.09099327976522753, 0.09151827134292549, 0.6991736284516058,0.7032075556518044);
      Vector3D size75 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box75 = new Box3D(position75, orientation75, size75);
      terrain.addRotatableBox(box75, YoAppearance.DarkGray());

      Point3D position76 = new Point3D(6.041424771298938, -0.2617975766406595, 0.2701155633865791);
      Quaternion orientation76 = new Quaternion(-0.09049938070702158, -0.09029400308019075, -0.7021010186557202,0.7005076835424435);
      Vector3D size76 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box76 = new Box3D(position76, orientation76, size76);
      terrain.addRotatableBox(box76, YoAppearance.DarkGray());

      Point3D position77 = new Point3D(6.124353907177331, 0.21270454422081966, 0.23404920128215498);
      Quaternion orientation77 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size77 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box77 = new Box3D(position77, orientation77, size77);
      terrain.addRotatableBox(box77, YoAppearance.DarkGray());

      Point3D position78 = new Point3D(6.421349657210157, -0.2658011220542116, 0.26733468034992436);
      Quaternion orientation78 = new Quaternion(-0.09224861237772161, 0.09388297766102086, 0.6947734684371486,0.7070827445042763);
      Vector3D size78 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box78 = new Box3D(position78, orientation78, size78);
      terrain.addRotatableBox(box78, YoAppearance.DarkGray());

      Point3D position79 = new Point3D(6.2386546198104, 0.5019196938501862, 0.2618318096377476);
      Quaternion orientation79 = new Quaternion(-0.0924918376852269, 0.09412884116858322, 0.6947475169322426,0.7070437814862335);
      Vector3D size79 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box79 = new Box3D(position79, orientation79, size79);
      terrain.addRotatableBox(box79, YoAppearance.DarkGray());

      Point3D position80 = new Point3D(6.051805117872733, 0.5075380042699142, 0.26005353871268083);
      Quaternion orientation80 = new Quaternion(-0.09694408237307763, 0.08981925984154682, 0.7007993258860914,0.7010097362319289);
      Vector3D size80 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box80 = new Box3D(position80, orientation80, size80);
      terrain.addRotatableBox(box80, YoAppearance.DarkGray());

      Point3D position81 = new Point3D(6.122908361470702, 0.020253416476653883, 0.2312384964958033);
      Quaternion orientation81 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size81 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box81 = new Box3D(position81, orientation81, size81);
      terrain.addRotatableBox(box81, YoAppearance.DarkGray());

      Point3D position82 = new Point3D(6.409268200039842, 0.11934128728302902, 0.23604260025687943);
      Quaternion orientation82 = new Quaternion(0.0, 0.0, 0.7016890425161331,0.7124833244454164);
      Vector3D size82 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box82 = new Box3D(position82, orientation82, size82);
      terrain.addRotatableBox(box82, YoAppearance.DarkGray());

      Point3D position83 = new Point3D(6.429966481857306, 0.500996096184173, 0.26300251335578917);
      Quaternion orientation83 = new Quaternion(-0.09163052675672367, 0.092769864463877, 0.6967248198275796,0.7053879246555514);
      Vector3D size83 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box83 = new Box3D(position83, orientation83, size83);
      terrain.addRotatableBox(box83, YoAppearance.DarkGray());

      Point3D position84 = new Point3D(6.699741918264115, -0.3631569365460855, 0.2550180515425791);
      Quaternion orientation84 = new Quaternion(1.7479183742545392E-33, 0.1288981738241679, 7.892736799485345E-18,0.9916578345300332);
      Vector3D size84 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box84 = new Box3D(position84, orientation84, size84);
      terrain.addRotatableBox(box84, YoAppearance.DarkGray());

      Point3D position85 = new Point3D(6.698630441512664, -0.17503310876007605, 0.2526101274005956);
      Quaternion orientation85 = new Quaternion(-3.884427274751654E-34, 0.1292203005277809, 7.912461371310276E-18,0.9916159104872764);
      Vector3D size85 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box85 = new Box3D(position85, orientation85, size85);
      terrain.addRotatableBox(box85, YoAppearance.DarkGray());

      Point3D position86 = new Point3D(6.797501566111999, 0.11234335851675088, 0.2633885198693278);
      Quaternion orientation86 = new Quaternion(0.09819637521077755, -0.10061717351576993, 0.691507811143679,0.708555293125695);
      Vector3D size86 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box86 = new Box3D(position86, orientation86, size86);
      terrain.addRotatableBox(box86, YoAppearance.DarkGray());

      Point3D position87 = new Point3D(6.61320792222252, 0.11105183808493378, 0.2609523583831561);
      Quaternion orientation87 = new Quaternion(0.09905634457071809, -0.09993018618950954, 0.6950116745649737,0.7050961428818118);
      Vector3D size87 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box87 = new Box3D(position87, orientation87, size87);
      terrain.addRotatableBox(box87, YoAppearance.DarkGray());

      Point3D position88 = new Point3D(6.7127304242037615, 0.4038501715863721, 0.21934622785676056);
      Quaternion orientation88 = new Quaternion(0.0, 0.0, -0.0038144316225337672,0.9999927250292359);
      Vector3D size88 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box88 = new Box3D(position88, orientation88, size88);
      terrain.addRotatableBox(box88, YoAppearance.DarkGray());

      Point3D position89 = new Point3D(6.718241475052683, 0.5961877211864752, 0.214411381592355);
      Quaternion orientation89 = new Quaternion(0.0, 0.0, -0.009312552251322237,0.999956637245119);
      Vector3D size89 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box89 = new Box3D(position89, orientation89, size89);
      terrain.addRotatableBox(box89, YoAppearance.DarkGray());

      Point3D position90 = new Point3D(6.983093459558476, -0.2800509790382789, 0.21557870456215963);
      Quaternion orientation90 = new Quaternion(0.0, 0.0, 0.7030358342720265,0.7111544246712073);
      Vector3D size90 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box90 = new Box3D(position90, orientation90, size90);
      terrain.addRotatableBox(box90, YoAppearance.DarkGray());

      Point3D position91 = new Point3D(6.988683516017568, 0.11099979053069783, 0.25958187507798225);
      Quaternion orientation91 = new Quaternion(0.10036878834372606, -0.10248996160407609, 0.6924383325630104,0.7070721813889761);
      Vector3D size91 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box91 = new Box3D(position91, orientation91, size91);
      terrain.addRotatableBox(box91, YoAppearance.DarkGray());

      Point3D position92 = new Point3D(7.00489079733141, 0.5035039836470866, 0.2167604197386731);
      Quaternion orientation92 = new Quaternion(0.0, 0.0, 0.7019672015374099,0.7122092725917976);
      Vector3D size92 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box92 = new Box3D(position92, orientation92, size92);
      terrain.addRotatableBox(box92, YoAppearance.DarkGray());

      Point3D position93 = new Point3D(7.345613673095441, -0.3621818158794301, 0.18851614172025807);
      Quaternion orientation93 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size93 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box93 = new Box3D(position93, orientation93, size93);
      terrain.addRotatableBox(box93, YoAppearance.DarkGray());

      Point3D position94 = new Point3D(7.346728260451079, -0.17617424269482315, 0.19148708763994512);
      Quaternion orientation94 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size94 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box94 = new Box3D(position94, orientation94, size94);
      terrain.addRotatableBox(box94, YoAppearance.DarkGray());

      Point3D position95 = new Point3D(7.256713781322195, 0.11746933621526329, 0.19034824545194998);
      Quaternion orientation95 = new Quaternion(0.0, 0.0, -0.7109847545862554,0.7032074222773266);
      Vector3D size95 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box95 = new Box3D(position95, orientation95, size95);
      terrain.addRotatableBox(box95, YoAppearance.DarkGray());

      Point3D position96 = new Point3D(7.436520392537069, 0.1116348793188018, 0.18978383802880525);
      Quaternion orientation96 = new Quaternion(0.0, 0.0, 0.7079054853446147,0.7063071738408194);
      Vector3D size96 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box96 = new Box3D(position96, orientation96, size96);
      terrain.addRotatableBox(box96, YoAppearance.DarkGray());

      Point3D position97 = new Point3D(7.356717455086338, 0.40091839642545996, 0.22230115969485045);
      Quaternion orientation97 = new Quaternion(0.0, 0.11441875733241588, 7.006128246478086E-18,0.9934326086708176);
      Vector3D size97 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box97 = new Box3D(position97, orientation97, size97);
      terrain.addRotatableBox(box97, YoAppearance.DarkGray());

      Point3D position98 = new Point3D(7.3566302205035035, 0.587809313874047, 0.22224355113151553);
      Quaternion orientation98 = new Quaternion(-1.5505289284134202E-33, 0.11216790604418489, 6.8683033552035814E-18,0.9936892677561043);
      Vector3D size98 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box98 = new Box3D(position98, orientation98, size98);
      terrain.addRotatableBox(box98, YoAppearance.DarkGray());

      Point3D position99 = new Point3D(7.770134742471376, -0.40055264320453654, 0.047256945955076055);
      Quaternion orientation99 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size99 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box99 = new Box3D(position99, orientation99, size99);
      terrain.addRotatableBox(box99, YoAppearance.DarkGray());

      Point3D position100 = new Point3D(7.770217973789706, -0.2137607588909391, 0.04430979308496823);
      Quaternion orientation100 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size100 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box100 = new Box3D(position100, orientation100, size100);
      terrain.addRotatableBox(box100, YoAppearance.DarkGray());

      Point3D position101 = new Point3D(7.869196761927994, 0.07594085472191646, 0.04358384009610045);
      Quaternion orientation101 = new Quaternion(0.0, 0.0, 0.703468250129445,0.7107266852031212);
      Vector3D size101 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box101 = new Box3D(position101, orientation101, size101);
      terrain.addRotatableBox(box101, YoAppearance.DarkGray());

      Point3D position102 = new Point3D(7.682970194739092, 0.07705257011165333, 0.047271829008987146);
      Quaternion orientation102 = new Quaternion(0.0, 0.0, 0.7034450413479783,0.7107496562102167);
      Vector3D size102 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box102 = new Box3D(position102, orientation102, size102);
      terrain.addRotatableBox(box102, YoAppearance.DarkGray());

      Point3D position103 = new Point3D(7.779256902384008, 0.3659919336394692, 0.04432955446553308);
      Quaternion orientation103 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size103 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box103 = new Box3D(position103, orientation103, size103);
      terrain.addRotatableBox(box103, YoAppearance.DarkGray());

      Point3D position104 = new Point3D(7.785783227158354, 0.5557054734046523, 0.045068700704124554);
      Quaternion orientation104 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size104 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box104 = new Box3D(position104, orientation104, size104);
      terrain.addRotatableBox(box104, YoAppearance.DarkGray());

      Point3D position105 = new Point3D(8.252153182891886, -0.3056293119640772, 0.04285320203865197);
      Quaternion orientation105 = new Quaternion(0.0, 0.0, 0.704181506150831,0.7100200042218158);
      Vector3D size105 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box105 = new Box3D(position105, orientation105, size105);
      terrain.addRotatableBox(box105, YoAppearance.DarkGray());

      Point3D position106 = new Point3D(8.064675866921874, -0.2987549941396834, 0.046016306708178786);
      Quaternion orientation106 = new Quaternion(0.0, 0.0, 0.7060501535261512,0.7081618322853177);
      Vector3D size106 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box106 = new Box3D(position106, orientation106, size106);
      terrain.addRotatableBox(box106, YoAppearance.DarkGray());

      Point3D position107 = new Point3D(8.152449598918341, -0.020698347797655257, 0.047082419865977894);
      Quaternion orientation107 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size107 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box107 = new Box3D(position107, orientation107, size107);
      terrain.addRotatableBox(box107, YoAppearance.DarkGray());

      Point3D position108 = new Point3D(8.15561274022191, 0.16606239445418075, 0.04851321195122305);
      Quaternion orientation108 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size108 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box108 = new Box3D(position108, orientation108, size108);
      terrain.addRotatableBox(box108, YoAppearance.DarkGray());

      Point3D position109 = new Point3D(8.06995087947249, 0.4533393922315608, 0.04470427328368063);
      Quaternion orientation109 = new Quaternion(0.0, 0.0, 0.7089663975812704,0.7052422612837633);
      Vector3D size109 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box109 = new Box3D(position109, orientation109, size109);
      terrain.addRotatableBox(box109, YoAppearance.DarkGray());

      Point3D position110 = new Point3D(8.256712637978552, 0.45548448733570646, 0.0465446546756541);
      Quaternion orientation110 = new Quaternion(0.0, 0.0, 0.705726540586363,0.7084843328627703);
      Vector3D size110 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box110 = new Box3D(position110, orientation110, size110);
      terrain.addRotatableBox(box110, YoAppearance.DarkGray());

      Point3D position111 = new Point3D(8.534695414460673, -0.40215227023449407, 0.0436624434311707);
      Quaternion orientation111 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size111 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box111 = new Box3D(position111, orientation111, size111);
      terrain.addRotatableBox(box111, YoAppearance.DarkGray());

      Point3D position112 = new Point3D(8.540900973445865, -0.21132852297590346, 0.04400585133181643);
      Quaternion orientation112 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size112 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box112 = new Box3D(position112, orientation112, size112);
      terrain.addRotatableBox(box112, YoAppearance.DarkGray());

      Point3D position113 = new Point3D(8.446166935149789, 0.07613432146555155, 0.04558588339207196);
      Quaternion orientation113 = new Quaternion(0.0, 0.0, 0.7016853567916603,0.7124869543115583);
      Vector3D size113 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box113 = new Box3D(position113, orientation113, size113);
      terrain.addRotatableBox(box113, YoAppearance.DarkGray());

      Point3D position114 = new Point3D(8.629100011417771, 0.078953161543596, 0.0401527710286485);
      Quaternion orientation114 = new Quaternion(0.0, 0.0, 0.7096844738128042,0.7045196573758914);
      Vector3D size114 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box114 = new Box3D(position114, orientation114, size114);
      terrain.addRotatableBox(box114, YoAppearance.DarkGray());

      Point3D position115 = new Point3D(8.544205652116847, 0.363513099370666, 0.04861356986695628);
      Quaternion orientation115 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size115 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box115 = new Box3D(position115, orientation115, size115);
      terrain.addRotatableBox(box115, YoAppearance.DarkGray());

      Point3D position116 = new Point3D(8.5399389891589, 0.555300276983753, 0.04766115185873509);
      Quaternion orientation116 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size116 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box116 = new Box3D(position116, orientation116, size116);
      terrain.addRotatableBox(box116, YoAppearance.DarkGray());

      Point3D position117 = new Point3D(2.9991922858880433, 1.3773850610309784, 0.07749255480474931);
      Quaternion orientation117 = new Quaternion(3.591011281523848E-4, -4.404093410555292E-4, -0.7140274385984735,0.7001174858654047);
      Vector3D size117 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box117 = new Box3D(position117, orientation117, size117);
      terrain.addRotatableBox(box117, YoAppearance.DarkGray());

      Point3D position118 = new Point3D(4.010313695756378, 1.3616056331805633, 0.07664394760851354);
      Quaternion orientation118 = new Quaternion(0.0010814627453136847, 0.0011373925186028025, -0.7128015132552289,0.7013640563057426);
      Vector3D size118 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box118 = new Box3D(position118, orientation118, size118);
      terrain.addRotatableBox(box118, YoAppearance.DarkGray());

      Point3D position119 = new Point3D(1.538532083708091, 1.87417019503108, 0.05131964350890132);
      Quaternion orientation119 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size119 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box119 = new Box3D(position119, orientation119, size119);
      terrain.addRotatableBox(box119, YoAppearance.DarkGray());

      Point3D position120 = new Point3D(1.5377122418466396, 1.6871995187212327, 0.0453699739307727);
      Quaternion orientation120 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size120 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box120 = new Box3D(position120, orientation120, size120);
      terrain.addRotatableBox(box120, YoAppearance.DarkGray());

      Point3D position121 = new Point3D(1.8318418085847752, 1.7832432291386375, 0.05026246658367628);
      Quaternion orientation121 = new Quaternion(0.0, 0.0, -0.7032798171527006,0.7109131443331625);
      Vector3D size121 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box121 = new Box3D(position121, orientation121, size121);
      terrain.addRotatableBox(box121, YoAppearance.DarkGray());

      Point3D position122 = new Point3D(2.0226812492006916, 1.7816220959595908, 0.058535600491900285);
      Quaternion orientation122 = new Quaternion(0.0, 0.0, -0.7033244049950813,0.7108690324794821);
      Vector3D size122 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box122 = new Box3D(position122, orientation122, size122);
      terrain.addRotatableBox(box122, YoAppearance.DarkGray());

      Point3D position123 = new Point3D(1.9258075527472558, 1.4916058787804267, 0.04914770991489481);
      Quaternion orientation123 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size123 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box123 = new Box3D(position123, orientation123, size123);
      terrain.addRotatableBox(box123, YoAppearance.DarkGray());

      Point3D position124 = new Point3D(1.9221267562656839, 1.2971054615729662, 0.048374686104427383);
      Quaternion orientation124 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size124 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box124 = new Box3D(position124, orientation124, size124);
      terrain.addRotatableBox(box124, YoAppearance.DarkGray());

      Point3D position125 = new Point3D(1.6391139250203717, 1.4000302880881712, 0.03909051303520233);
      Quaternion orientation125 = new Quaternion(4.2221998162823653E-4, -0.0021771612644779165, -0.7083346233855763,0.7058733193798512);
      Vector3D size125 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box125 = new Box3D(position125, orientation125, size125);
      terrain.addRotatableBox(box125, YoAppearance.DarkGray());

      Point3D position126 = new Point3D(1.4508258759747759, 1.391873785777104, 0.03772504674415519);
      Quaternion orientation126 = new Quaternion(0.0, 0.0, 0.7038510074598656,0.7103476327107258);
      Vector3D size126 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box126 = new Box3D(position126, orientation126, size126);
      terrain.addRotatableBox(box126, YoAppearance.DarkGray());

      Point3D position127 = new Point3D(1.544254053576254, 1.106315000748547, 0.032965432002729965);
      Quaternion orientation127 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size127 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box127 = new Box3D(position127, orientation127, size127);
      terrain.addRotatableBox(box127, YoAppearance.DarkGray());

      Point3D position128 = new Point3D(1.5421604510537665, 0.9152318046805299, 0.03169062655032244);
      Quaternion orientation128 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size128 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box128 = new Box3D(position128, orientation128, size128);
      terrain.addRotatableBox(box128, YoAppearance.DarkGray());

      Point3D position129 = new Point3D(1.8302064340875628, 1.012274943312089, 0.03233411894842265);
      Quaternion orientation129 = new Quaternion(0.0, 0.0, -0.7041659007794283,0.7100354809300001);
      Vector3D size129 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box129 = new Box3D(position129, orientation129, size129);
      terrain.addRotatableBox(box129, YoAppearance.DarkGray());

      Point3D position130 = new Point3D(2.020076111592974, 1.0054981613848237, 0.032951829300953354);
      Quaternion orientation130 = new Quaternion(0.0, 0.0, 0.7094730567086567,0.7047325604826807);
      Vector3D size130 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box130 = new Box3D(position130, orientation130, size130);
      terrain.addRotatableBox(box130, YoAppearance.DarkGray());

      Point3D position131 = new Point3D(2.3017726687557736, 1.8659211869311392, 0.050732948560174566);
      Quaternion orientation131 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size131 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box131 = new Box3D(position131, orientation131, size131);
      terrain.addRotatableBox(box131, YoAppearance.DarkGray());

      Point3D position132 = new Point3D(2.3045613478518816, 1.6744061119031097, 0.04678489901068887);
      Quaternion orientation132 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size132 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box132 = new Box3D(position132, orientation132, size132);
      terrain.addRotatableBox(box132, YoAppearance.DarkGray());

      Point3D position133 = new Point3D(2.214159178501207, 1.390925622627419, 0.04546863118006792);
      Quaternion orientation133 = new Quaternion(0.0, 0.0, -0.710045270209128,0.7041560297644597);
      Vector3D size133 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box133 = new Box3D(position133, orientation133, size133);
      terrain.addRotatableBox(box133, YoAppearance.DarkGray());

      Point3D position134 = new Point3D(2.3882625449800243, 1.385485705317335, 0.04358061639227621);
      Quaternion orientation134 = new Quaternion(0.0, 0.0, 0.7032305077721192,0.7109619208780226);
      Vector3D size134 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box134 = new Box3D(position134, orientation134, size134);
      terrain.addRotatableBox(box134, YoAppearance.DarkGray());

      Point3D position135 = new Point3D(2.305334951328886, 1.1033087493012612, 0.04026317060808564);
      Quaternion orientation135 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size135 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box135 = new Box3D(position135, orientation135, size135);
      terrain.addRotatableBox(box135, YoAppearance.DarkGray());

      Point3D position136 = new Point3D(2.3035618801058884, 0.9148277365412183, 0.043838376674458175);
      Quaternion orientation136 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size136 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box136 = new Box3D(position136, orientation136, size136);
      terrain.addRotatableBox(box136, YoAppearance.DarkGray());

      Point3D position137 = new Point3D(5.022827545101327, 1.3435927211517837, 0.06929944023803199);
      Quaternion orientation137 = new Quaternion(0.0, 0.0, -0.7146326573404546,0.6994999392870028);
      Vector3D size137 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box137 = new Box3D(position137, orientation137, size137);
      terrain.addRotatableBox(box137, YoAppearance.DarkGray());

      Point3D position138 = new Point3D(6.0368497952581395, 1.3038421449456419, 0.06172802598521781);
      Quaternion orientation138 = new Quaternion(0.005613854283208643, 0.0055820130552961455, -0.7143726400604131,0.6997206992103728);
      Vector3D size138 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box138 = new Box3D(position138, orientation138, size138);
      terrain.addRotatableBox(box138, YoAppearance.DarkGray());

      Point3D position139 = new Point3D(7.050523277840206, 1.2828946360839608, 0.06864253875907342);
      Quaternion orientation139 = new Quaternion(0.0, 0.0, 0.7017961773791698,0.7123777968297335);
      Vector3D size139 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box139 = new Box3D(position139, orientation139, size139);
      terrain.addRotatableBox(box139, YoAppearance.DarkGray());

      Point3D position140 = new Point3D(5.023846234292196, 1.3361370347496169, 0.22602295943394107);
      Quaternion orientation140 = new Quaternion(0.0, 0.0, 0.7009087265528491,0.71325097759629);
      Vector3D size140 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box140 = new Box3D(position140, orientation140, size140);
      terrain.addRotatableBox(box140, YoAppearance.DarkGray());

      Point3D position141 = new Point3D(2.6900015009577163, 1.8436366016069017, 0.19963009876119422);
      Quaternion orientation141 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size141 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box141 = new Box3D(position141, orientation141, size141);
      terrain.addRotatableBox(box141, YoAppearance.DarkGray());

      Point3D position142 = new Point3D(2.6890564650088904, 1.654319503692475, 0.1998904919894186);
      Quaternion orientation142 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size142 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box142 = new Box3D(position142, orientation142, size142);
      terrain.addRotatableBox(box142, YoAppearance.DarkGray());

      Point3D position143 = new Point3D(2.7797616190377004, 1.3649618700884152, 0.24403572659127876);
      Quaternion orientation143 = new Quaternion(0.09461018209558089, 0.09407245118837755, -0.7027793486796466,0.6987849987232575);
      Vector3D size143 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box143 = new Box3D(position143, orientation143, size143);
      terrain.addRotatableBox(box143, YoAppearance.DarkGray());

      Point3D position144 = new Point3D(2.593463551704851, 1.3621908804922451, 0.24443600486584452);
      Quaternion orientation144 = new Quaternion(0.09234953713579733, -0.09290494477039353, 0.698908799916006,0.7031121700183992);
      Vector3D size144 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box144 = new Box3D(position144, orientation144, size144);
      terrain.addRotatableBox(box144, YoAppearance.DarkGray());

      Point3D position145 = new Point3D(2.684024192233144, 1.0693810052425663, 0.19603696340587146);
      Quaternion orientation145 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size145 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box145 = new Box3D(position145, orientation145, size145);
      terrain.addRotatableBox(box145, YoAppearance.DarkGray());

      Point3D position146 = new Point3D(2.6860150703200567, 0.879548037610604, 0.19828815283388773);
      Quaternion orientation146 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size146 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box146 = new Box3D(position146, orientation146, size146);
      terrain.addRotatableBox(box146, YoAppearance.DarkGray());

      Point3D position147 = new Point3D(3.1711383025454465, 1.7462180514407013, 0.28166428275520083);
      Quaternion orientation147 = new Quaternion(0.10793831101703633, 0.10894156349569274, -0.6955023910392171,0.7019668658965574);
      Vector3D size147 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box147 = new Box3D(position147, orientation147, size147);
      terrain.addRotatableBox(box147, YoAppearance.DarkGray());

      Point3D position148 = new Point3D(2.9818424101161205, 1.742476963468071, 0.28083800074621);
      Quaternion orientation148 = new Quaternion(0.11223315066187517, -0.11138798116147307, 0.7008445880063211,0.6955668918281909);
      Vector3D size148 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box148 = new Box3D(position148, orientation148, size148);
      terrain.addRotatableBox(box148, YoAppearance.DarkGray());

      Point3D position149 = new Point3D(3.0756668786094545, 1.4573398848240338, 0.22430098450189007);
      Quaternion orientation149 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size149 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box149 = new Box3D(position149, orientation149, size149);
      terrain.addRotatableBox(box149, YoAppearance.DarkGray());

      Point3D position150 = new Point3D(3.0722024139657997, 1.2678346618372078, 0.2251948202315323);
      Quaternion orientation150 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size150 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box150 = new Box3D(position150, orientation150, size150);
      terrain.addRotatableBox(box150, YoAppearance.DarkGray());

      Point3D position151 = new Point3D(3.165664362999164, 0.9747779559115072, 0.27735751751742505);
      Quaternion orientation151 = new Quaternion(0.09437843324707647, 0.09404280191325493, -0.7020498469308203,0.6995531968124677);
      Vector3D size151 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box151 = new Box3D(position151, orientation151, size151);
      terrain.addRotatableBox(box151, YoAppearance.DarkGray());

      Point3D position152 = new Point3D(2.974840053678399, 0.9748012422824412, 0.2788927721576176);
      Quaternion orientation152 = new Quaternion(0.09994070761295208, -0.09964072755297639, 0.7010812345776699,0.6989768829493477);
      Vector3D size152 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box152 = new Box3D(position152, orientation152, size152);
      terrain.addRotatableBox(box152, YoAppearance.DarkGray());

      Point3D position153 = new Point3D(3.359147869089035, 1.7508795341753638, 0.22486106332834727);
      Quaternion orientation153 = new Quaternion(0.0, 0.0, -0.7071194347376097,0.7070941274090483);
      Vector3D size153 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box153 = new Box3D(position153, orientation153, size153);
      terrain.addRotatableBox(box153, YoAppearance.DarkGray());

      Point3D position154 = new Point3D(3.354472458529534, 1.3578339761550424, 0.22395977136362658);
      Quaternion orientation154 = new Quaternion(0.0, 0.0, -0.7058785319428242,0.7083329006492944);
      Vector3D size154 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box154 = new Box3D(position154, orientation154, size154);
      terrain.addRotatableBox(box154, YoAppearance.DarkGray());

      Point3D position155 = new Point3D(3.350402623341776, 0.9730833666883968, 0.22640085739147484);
      Quaternion orientation155 = new Quaternion(0.0, 0.0, 0.7062242020231959,0.7079882601263245);
      Vector3D size155 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box155 = new Box3D(position155, orientation155, size155);
      terrain.addRotatableBox(box155, YoAppearance.DarkGray());

      Point3D position156 = new Point3D(3.68866762247556, 1.8236265834842826, 0.3010889371484357);
      Quaternion orientation156 = new Quaternion(-5.839236425510448E-34, 0.14469060323807462, 8.859744206110364E-18,0.9894769473487506);
      Vector3D size156 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box156 = new Box3D(position156, orientation156, size156);
      terrain.addRotatableBox(box156, YoAppearance.DarkGray());

      Point3D position157 = new Point3D(3.7706107394082076, 1.3518618466597851, 0.2940640116728258);
      Quaternion orientation157 = new Quaternion(-0.08864451472761907, -0.08908549481655237, -0.6997578049577696,0.7032388918587189);
      Vector3D size157 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box157 = new Box3D(position157, orientation157, size157);
      terrain.addRotatableBox(box157, YoAppearance.DarkGray());

      Point3D position158 = new Point3D(3.68965927337154, 1.6388934247654854, 0.30028645012494665);
      Quaternion orientation158 = new Quaternion(1.9454724300576844E-34, 0.14138358070604393, 8.657247478182429E-18,0.989954889430189);
      Vector3D size158 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box158 = new Box3D(position158, orientation158, size158);
      terrain.addRotatableBox(box158, YoAppearance.DarkGray());

      Point3D position159 = new Point3D(3.588806285529547, 1.3548401365418385, 0.2930951718708826);
      Quaternion orientation159 = new Quaternion(-0.08427871374180397, 0.09588259957957021, 0.7076885857849924,0.6948960289550722);
      Vector3D size159 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box159 = new Box3D(position159, orientation159, size159);
      terrain.addRotatableBox(box159, YoAppearance.DarkGray());

      Point3D position160 = new Point3D(3.6964906211913506, 1.056135506335926, 0.24588881335187482);
      Quaternion orientation160 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size160 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box160 = new Box3D(position160, orientation160, size160);
      terrain.addRotatableBox(box160, YoAppearance.DarkGray());

      Point3D position161 = new Point3D(3.6860421913290207, 0.8666362915851753, 0.24832633958306163);
      Quaternion orientation161 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size161 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box161 = new Box3D(position161, orientation161, size161);
      terrain.addRotatableBox(box161, YoAppearance.DarkGray());

      Point3D position162 = new Point3D(3.9745492483528864, 0.9636677867081712, 0.3000158195677145);
      Quaternion orientation162 = new Quaternion(0.09976532226249955, -0.09967911277827533, 0.7003421510403348,0.6997369694576615);
      Vector3D size162 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box162 = new Box3D(position162, orientation162, size162);
      terrain.addRotatableBox(box162, YoAppearance.DarkGray());

      Point3D position163 = new Point3D(4.1570571372942995, 1.7351500725177174, 0.3009665879051755);
      Quaternion orientation163 = new Quaternion(0.0, 0.0, -0.7031379195773428,0.7110534902891948);
      Vector3D size163 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box163 = new Box3D(position163, orientation163, size163);
      terrain.addRotatableBox(box163, YoAppearance.DarkGray());

      Point3D position164 = new Point3D(3.976449906945254, 1.7313406371863271, 0.30163369997832445);
      Quaternion orientation164 = new Quaternion(0.0, 0.0, 0.7053477339779695,0.7088614633143374);
      Vector3D size164 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box164 = new Box3D(position164, orientation164, size164);
      terrain.addRotatableBox(box164, YoAppearance.DarkGray());

      Point3D position165 = new Point3D(4.060087891099629, 1.4392964224398486, 0.24882388075031026);
      Quaternion orientation165 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size165 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box165 = new Box3D(position165, orientation165, size165);
      terrain.addRotatableBox(box165, YoAppearance.DarkGray());

      Point3D position166 = new Point3D(4.063602710036821, 1.2525705083227396, 0.2470906830235654);
      Quaternion orientation166 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size166 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box166 = new Box3D(position166, orientation166, size166);
      terrain.addRotatableBox(box166, YoAppearance.DarkGray());

      Point3D position167 = new Point3D(4.147655172749554, 0.9634003119005562, 0.2989374948751531);
      Quaternion orientation167 = new Quaternion(0.09713879505240461, 0.09719288147916494, -0.7002040662118151,0.7005939365609611);
      Vector3D size167 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box167 = new Box3D(position167, orientation167, size167);
      terrain.addRotatableBox(box167, YoAppearance.DarkGray());

      Point3D position168 = new Point3D(4.343356717444665, 0.9648819072177, 0.30128052384367776);
      Quaternion orientation168 = new Quaternion(0.0844889112032617, -0.11355828784903144, 0.6956075277336033,0.7043410441716627);
      Vector3D size168 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box168 = new Box3D(position168, orientation168, size168);
      terrain.addRotatableBox(box168, YoAppearance.DarkGray());

      Point3D position169 = new Point3D(4.349272116834419, 1.3503364005449605, 0.23830329314841697);
      Quaternion orientation169 = new Quaternion(0.0, 0.0, -0.7115262234714076,0.7026595429598296);
      Vector3D size169 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box169 = new Box3D(position169, orientation169, size169);
      terrain.addRotatableBox(box169, YoAppearance.DarkGray());

      Point3D position170 = new Point3D(4.346481880050672, 1.7267456748407475, 0.3000392880382346);
      Quaternion orientation170 = new Quaternion(0.0, 0.0, -0.7065132530735921,0.7076998115241875);
      Vector3D size170 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box170 = new Box3D(position170, orientation170, size170);
      terrain.addRotatableBox(box170, YoAppearance.DarkGray());

      Point3D position171 = new Point3D(4.702380013634424, 1.8079336148133884, 0.3797638118070186);
      Quaternion orientation171 = new Quaternion(1.6449470919030792E-33, 0.09792926395956664, 5.9964379825470255E-18,0.9951933778217867);
      Vector3D size171 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box171 = new Box3D(position171, orientation171, size171);
      terrain.addRotatableBox(box171, YoAppearance.DarkGray());

      Point3D position172 = new Point3D(4.703890280086529, 1.6155963177652986, 0.37962892936597686);
      Quaternion orientation172 = new Quaternion(1.9358735945371405E-34, 0.10122575159006256, 6.1982896338026795E-18,0.9948634816973769);
      Vector3D size172 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box172 = new Box3D(position172, orientation172, size172);
      terrain.addRotatableBox(box172, YoAppearance.DarkGray());

      Point3D position173 = new Point3D(4.792888242703473, 1.3248942082197477, 0.3479451232275179);
      Quaternion orientation173 = new Quaternion(0.0, 0.0, -0.7035865871495094,0.7106095372166812);
      Vector3D size173 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box173 = new Box3D(position173, orientation173, size173);
      terrain.addRotatableBox(box173, YoAppearance.DarkGray());

      Point3D position174 = new Point3D(4.610354981743812, 1.329117286057229, 0.3475132664494318);
      Quaternion orientation174 = new Quaternion(0.0, 0.0, -0.7132505655401289,0.7009091458646308);
      Vector3D size174 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box174 = new Box3D(position174, orientation174, size174);
      terrain.addRotatableBox(box174, YoAppearance.DarkGray());

      Point3D position175 = new Point3D(4.712970930242387, 1.0397989103493432, 0.3912550730752458);
      Quaternion orientation175 = new Quaternion(-3.880765412410759E-34, -0.12182502599465175, -7.459631407019642E-18,0.9925515921308083);
      Vector3D size175 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box175 = new Box3D(position175, orientation175, size175);
      terrain.addRotatableBox(box175, YoAppearance.DarkGray());

      Point3D position176 = new Point3D(4.707914463288, 0.8513946572284801, 0.3891170604695318);
      Quaternion orientation176 = new Quaternion(2.3291328065487197E-33, -0.1241412431733023, -7.601458804717881E-18,0.9922645573350823);
      Vector3D size176 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box176 = new Box3D(position176, orientation176, size176);
      terrain.addRotatableBox(box176, YoAppearance.DarkGray());

      Point3D position177 = new Point3D(5.1727760970934185, 1.7112340340750825, 0.3967895082089883);
      Quaternion orientation177 = new Quaternion(0.09472131621407967, 0.09590058699963736, -0.6963057302396933,0.7049746660009598);
      Vector3D size177 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box177 = new Box3D(position177, orientation177, size177);
      terrain.addRotatableBox(box177, YoAppearance.DarkGray());

      Point3D position178 = new Point3D(4.986578998410523, 1.7045728830128697, 0.3944322980633405);
      Quaternion orientation178 = new Quaternion(0.09774852819681804, -0.09770925509640942, 0.7004613865448716,0.700179957342037);
      Vector3D size178 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box178 = new Box3D(position178, orientation178, size178);
      terrain.addRotatableBox(box178, YoAppearance.DarkGray());

      Point3D position179 = new Point3D(5.181946498191193, 1.315938621233747, 0.3935301841017318);
      Quaternion orientation179 = new Quaternion(-0.09454705714016172, 0.09446453379644658, 0.7010687973196257,0.7004568846585553);
      Vector3D size179 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box179 = new Box3D(position179, orientation179, size179);
      terrain.addRotatableBox(box179, YoAppearance.DarkGray());

      Point3D position180 = new Point3D(4.999908472643262, 1.3227122732285912, 0.3936477908269693);
      Quaternion orientation180 = new Quaternion(-0.0943438948778358, -0.0947342877428095, -0.6993100325946019,0.7022037614095209);
      Vector3D size180 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box180 = new Box3D(position180, orientation180, size180);
      terrain.addRotatableBox(box180, YoAppearance.DarkGray());

      Point3D position181 = new Point3D(5.181134977089534, 0.9351745204198109, 0.34319902751110637);
      Quaternion orientation181 = new Quaternion(0.0, 0.0, -0.7087044927674649,0.7055054513830564);
      Vector3D size181 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box181 = new Box3D(position181, orientation181, size181);
      terrain.addRotatableBox(box181, YoAppearance.DarkGray());

      Point3D position182 = new Point3D(5.001168160091194, 0.9310266893680323, 0.3442518243348339);
      Quaternion orientation182 = new Quaternion(0.0, 0.0, 0.7085911489028207,0.7056192909045079);
      Vector3D size182 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box182 = new Box3D(position182, orientation182, size182);
      terrain.addRotatableBox(box182, YoAppearance.DarkGray());

      Point3D position183 = new Point3D(5.361423516842161, 0.937318361252478, 0.3467424072643232);
      Quaternion orientation183 = new Quaternion(0.0, 0.0, -0.702830138774019,0.7113577131309489);
      Vector3D size183 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box183 = new Box3D(position183, orientation183, size183);
      terrain.addRotatableBox(box183, YoAppearance.DarkGray());

      Point3D position184 = new Point3D(5.368768670021752, 1.31715906709865, 0.38910279983955076);
      Quaternion orientation184 = new Quaternion(-0.09458307226381174, -0.09416431082175267, -0.7023337283279424,0.6992241836933095);
      Vector3D size184 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box184 = new Box3D(position184, orientation184, size184);
      terrain.addRotatableBox(box184, YoAppearance.DarkGray());

      Point3D position185 = new Point3D(5.359430458701423, 1.7074604944131373, 0.3951426086328146);
      Quaternion orientation185 = new Quaternion(0.09529196339253196, -0.09533170427575388, 0.7005076469963307,0.7007997890784077);
      Vector3D size185 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box185 = new Box3D(position185, orientation185, size185);
      terrain.addRotatableBox(box185, YoAppearance.DarkGray());

      Point3D position186 = new Point3D(5.75258973537161, 0.849509218011508, 0.227418511493009);
      Quaternion orientation186 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size186 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box186 = new Box3D(position186, orientation186, size186);
      terrain.addRotatableBox(box186, YoAppearance.DarkGray());

      Point3D position187 = new Point3D(5.753494342409687, 1.031547818499434, 0.2278999120441889);
      Quaternion orientation187 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size187 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box187 = new Box3D(position187, orientation187, size187);
      terrain.addRotatableBox(box187, YoAppearance.DarkGray());

      Point3D position188 = new Point3D(5.648721454782096, 1.322749962848247, 0.27732476658080424);
      Quaternion orientation188 = new Quaternion(0.0, 0.0, -0.7107650931702618,0.7034294437473235);
      Vector3D size188 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box188 = new Box3D(position188, orientation188, size188);
      terrain.addRotatableBox(box188, YoAppearance.DarkGray());

      Point3D position189 = new Point3D(5.840403719567024, 1.3249510385355707, 0.27650382371337645);
      Quaternion orientation189 = new Quaternion(0.0, 0.0, 0.7053417880043341,0.7088673797649665);
      Vector3D size189 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box189 = new Box3D(position189, orientation189, size189);
      terrain.addRotatableBox(box189, YoAppearance.DarkGray());

      Point3D position190 = new Point3D(5.7577226491388185, 1.6116362919569682, 0.28389741291306736);
      Quaternion orientation190 = new Quaternion(0.0, 0.11262021465784755, 6.89599927000101E-18,0.993638106782555);
      Vector3D size190 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box190 = new Box3D(position190, orientation190, size190);
      terrain.addRotatableBox(box190, YoAppearance.DarkGray());

      Point3D position191 = new Point3D(5.764448795159363, 1.8034155096950473, 0.28467265927325774);
      Quaternion orientation191 = new Quaternion(-1.0658594972039655E-33, 0.11109613212788319, 6.802676130403094E-18,0.9938096645868483);
      Vector3D size191 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box191 = new Box3D(position191, orientation191, size191);
      terrain.addRotatableBox(box191, YoAppearance.DarkGray());

      Point3D position192 = new Point3D(6.2352107198329065, 0.941193884518639, 0.2684214567166878);
      Quaternion orientation192 = new Quaternion(-0.09099327976522753, 0.09151827134292549, 0.6991736284516058,0.7032075556518044);
      Vector3D size192 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box192 = new Box3D(position192, orientation192, size192);
      terrain.addRotatableBox(box192, YoAppearance.DarkGray());

      Point3D position193 = new Point3D(6.041424771298938, 0.9382024233593405, 0.2701155633865791);
      Quaternion orientation193 = new Quaternion(-0.09049938070702158, -0.09029400308019075, -0.7021010186557202,0.7005076835424435);
      Vector3D size193 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box193 = new Box3D(position193, orientation193, size193);
      terrain.addRotatableBox(box193, YoAppearance.DarkGray());

      Point3D position194 = new Point3D(6.124353907177331, 1.4127045442208197, 0.23404920128215498);
      Quaternion orientation194 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size194 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box194 = new Box3D(position194, orientation194, size194);
      terrain.addRotatableBox(box194, YoAppearance.DarkGray());

      Point3D position195 = new Point3D(6.421349657210157, 0.9341988779457884, 0.26733468034992436);
      Quaternion orientation195 = new Quaternion(-0.09224861237772161, 0.09388297766102086, 0.6947734684371486,0.7070827445042763);
      Vector3D size195 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box195 = new Box3D(position195, orientation195, size195);
      terrain.addRotatableBox(box195, YoAppearance.DarkGray());

      Point3D position196 = new Point3D(6.2386546198104, 1.7019196938501862, 0.2618318096377476);
      Quaternion orientation196 = new Quaternion(-0.0924918376852269, 0.09412884116858322, 0.6947475169322426,0.7070437814862335);
      Vector3D size196 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box196 = new Box3D(position196, orientation196, size196);
      terrain.addRotatableBox(box196, YoAppearance.DarkGray());

      Point3D position197 = new Point3D(6.051805117872733, 1.7075380042699142, 0.26005353871268083);
      Quaternion orientation197 = new Quaternion(-0.09694408237307763, 0.08981925984154682, 0.7007993258860914,0.7010097362319289);
      Vector3D size197 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box197 = new Box3D(position197, orientation197, size197);
      terrain.addRotatableBox(box197, YoAppearance.DarkGray());

      Point3D position198 = new Point3D(6.122908361470702, 1.2202534164766539, 0.2312384964958033);
      Quaternion orientation198 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size198 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box198 = new Box3D(position198, orientation198, size198);
      terrain.addRotatableBox(box198, YoAppearance.DarkGray());

      Point3D position199 = new Point3D(6.409268200039842, 1.319341287283029, 0.23604260025687943);
      Quaternion orientation199 = new Quaternion(0.0, 0.0, 0.7016890425161331,0.7124833244454164);
      Vector3D size199 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box199 = new Box3D(position199, orientation199, size199);
      terrain.addRotatableBox(box199, YoAppearance.DarkGray());

      Point3D position200 = new Point3D(6.429966481857306, 1.700996096184173, 0.26300251335578917);
      Quaternion orientation200 = new Quaternion(-0.09163052675672367, 0.092769864463877, 0.6967248198275796,0.7053879246555514);
      Vector3D size200 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box200 = new Box3D(position200, orientation200, size200);
      terrain.addRotatableBox(box200, YoAppearance.DarkGray());

      Point3D position201 = new Point3D(6.699741918264115, 0.8368430634539145, 0.2550180515425791);
      Quaternion orientation201 = new Quaternion(1.7479183742545392E-33, 0.1288981738241679, 7.892736799485345E-18,0.9916578345300332);
      Vector3D size201 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box201 = new Box3D(position201, orientation201, size201);
      terrain.addRotatableBox(box201, YoAppearance.DarkGray());

      Point3D position202 = new Point3D(6.698630441512664, 1.024966891239924, 0.2526101274005956);
      Quaternion orientation202 = new Quaternion(-3.884427274751654E-34, 0.1292203005277809, 7.912461371310276E-18,0.9916159104872764);
      Vector3D size202 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box202 = new Box3D(position202, orientation202, size202);
      terrain.addRotatableBox(box202, YoAppearance.DarkGray());

      Point3D position203 = new Point3D(6.797501566111999, 1.3123433585167508, 0.2633885198693278);
      Quaternion orientation203 = new Quaternion(0.09819637521077755, -0.10061717351576993, 0.691507811143679,0.708555293125695);
      Vector3D size203 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box203 = new Box3D(position203, orientation203, size203);
      terrain.addRotatableBox(box203, YoAppearance.DarkGray());

      Point3D position204 = new Point3D(6.61320792222252, 1.3110518380849336, 0.2609523583831561);
      Quaternion orientation204 = new Quaternion(0.09905634457071809, -0.09993018618950954, 0.6950116745649737,0.7050961428818118);
      Vector3D size204 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box204 = new Box3D(position204, orientation204, size204);
      terrain.addRotatableBox(box204, YoAppearance.DarkGray());

      Point3D position205 = new Point3D(6.7127304242037615, 1.603850171586372, 0.21934622785676056);
      Quaternion orientation205 = new Quaternion(0.0, 0.0, -0.0038144316225337672,0.9999927250292359);
      Vector3D size205 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box205 = new Box3D(position205, orientation205, size205);
      terrain.addRotatableBox(box205, YoAppearance.DarkGray());

      Point3D position206 = new Point3D(6.718241475052683, 1.7961877211864752, 0.214411381592355);
      Quaternion orientation206 = new Quaternion(0.0, 0.0, -0.009312552251322237,0.999956637245119);
      Vector3D size206 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box206 = new Box3D(position206, orientation206, size206);
      terrain.addRotatableBox(box206, YoAppearance.DarkGray());

      Point3D position207 = new Point3D(6.983093459558476, 0.919949020961721, 0.21557870456215963);
      Quaternion orientation207 = new Quaternion(0.0, 0.0, 0.7030358342720265,0.7111544246712073);
      Vector3D size207 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box207 = new Box3D(position207, orientation207, size207);
      terrain.addRotatableBox(box207, YoAppearance.DarkGray());

      Point3D position208 = new Point3D(6.988683516017568, 1.3109997905306978, 0.25958187507798225);
      Quaternion orientation208 = new Quaternion(0.10036878834372606, -0.10248996160407609, 0.6924383325630104,0.7070721813889761);
      Vector3D size208 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box208 = new Box3D(position208, orientation208, size208);
      terrain.addRotatableBox(box208, YoAppearance.DarkGray());

      Point3D position209 = new Point3D(7.00489079733141, 1.7035039836470864, 0.2167604197386731);
      Quaternion orientation209 = new Quaternion(0.0, 0.0, 0.7019672015374099,0.7122092725917976);
      Vector3D size209 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box209 = new Box3D(position209, orientation209, size209);
      terrain.addRotatableBox(box209, YoAppearance.DarkGray());

      Point3D position210 = new Point3D(7.345613673095441, 0.8378181841205699, 0.18851614172025807);
      Quaternion orientation210 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size210 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box210 = new Box3D(position210, orientation210, size210);
      terrain.addRotatableBox(box210, YoAppearance.DarkGray());

      Point3D position211 = new Point3D(7.346728260451079, 1.0238257573051768, 0.19148708763994512);
      Quaternion orientation211 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size211 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box211 = new Box3D(position211, orientation211, size211);
      terrain.addRotatableBox(box211, YoAppearance.DarkGray());

      Point3D position212 = new Point3D(7.256713781322195, 1.3174693362152632, 0.19034824545194998);
      Quaternion orientation212 = new Quaternion(0.0, 0.0, -0.7109847545862554,0.7032074222773266);
      Vector3D size212 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box212 = new Box3D(position212, orientation212, size212);
      terrain.addRotatableBox(box212, YoAppearance.DarkGray());

      Point3D position213 = new Point3D(7.436520392537069, 1.3116348793188017, 0.18978383802880525);
      Quaternion orientation213 = new Quaternion(0.0, 0.0, 0.7079054853446147,0.7063071738408194);
      Vector3D size213 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box213 = new Box3D(position213, orientation213, size213);
      terrain.addRotatableBox(box213, YoAppearance.DarkGray());

      Point3D position214 = new Point3D(7.356717455086338, 1.6009183964254599, 0.22230115969485045);
      Quaternion orientation214 = new Quaternion(0.0, 0.11441875733241588, 7.006128246478086E-18,0.9934326086708176);
      Vector3D size214 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box214 = new Box3D(position214, orientation214, size214);
      terrain.addRotatableBox(box214, YoAppearance.DarkGray());

      Point3D position215 = new Point3D(7.3566302205035035, 1.787809313874047, 0.22224355113151553);
      Quaternion orientation215 = new Quaternion(-1.5505289284134202E-33, 0.11216790604418489, 6.8683033552035814E-18,0.9936892677561043);
      Vector3D size215 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box215 = new Box3D(position215, orientation215, size215);
      terrain.addRotatableBox(box215, YoAppearance.DarkGray());

      Point3D position216 = new Point3D(7.770134742471376, 0.7994473567954634, 0.047256945955076055);
      Quaternion orientation216 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size216 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box216 = new Box3D(position216, orientation216, size216);
      terrain.addRotatableBox(box216, YoAppearance.DarkGray());

      Point3D position217 = new Point3D(7.770217973789706, 0.9862392411090608, 0.04430979308496823);
      Quaternion orientation217 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size217 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box217 = new Box3D(position217, orientation217, size217);
      terrain.addRotatableBox(box217, YoAppearance.DarkGray());

      Point3D position218 = new Point3D(7.869196761927994, 1.2759408547219164, 0.04358384009610045);
      Quaternion orientation218 = new Quaternion(0.0, 0.0, 0.703468250129445,0.7107266852031212);
      Vector3D size218 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box218 = new Box3D(position218, orientation218, size218);
      terrain.addRotatableBox(box218, YoAppearance.DarkGray());

      Point3D position219 = new Point3D(7.682970194739092, 1.2770525701116533, 0.047271829008987146);
      Quaternion orientation219 = new Quaternion(0.0, 0.0, 0.7034450413479783,0.7107496562102167);
      Vector3D size219 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box219 = new Box3D(position219, orientation219, size219);
      terrain.addRotatableBox(box219, YoAppearance.DarkGray());

      Point3D position220 = new Point3D(7.779256902384008, 1.5659919336394692, 0.04432955446553308);
      Quaternion orientation220 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size220 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box220 = new Box3D(position220, orientation220, size220);
      terrain.addRotatableBox(box220, YoAppearance.DarkGray());

      Point3D position221 = new Point3D(7.785783227158354, 1.7557054734046522, 0.045068700704124554);
      Quaternion orientation221 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size221 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box221 = new Box3D(position221, orientation221, size221);
      terrain.addRotatableBox(box221, YoAppearance.DarkGray());

      Point3D position222 = new Point3D(8.252153182891886, 0.8943706880359228, 0.04285320203865197);
      Quaternion orientation222 = new Quaternion(0.0, 0.0, 0.704181506150831,0.7100200042218158);
      Vector3D size222 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box222 = new Box3D(position222, orientation222, size222);
      terrain.addRotatableBox(box222, YoAppearance.DarkGray());

      Point3D position223 = new Point3D(8.064675866921874, 0.9012450058603165, 0.046016306708178786);
      Quaternion orientation223 = new Quaternion(0.0, 0.0, 0.7060501535261512,0.7081618322853177);
      Vector3D size223 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box223 = new Box3D(position223, orientation223, size223);
      terrain.addRotatableBox(box223, YoAppearance.DarkGray());

      Point3D position224 = new Point3D(8.152449598918341, 1.1793016522023447, 0.047082419865977894);
      Quaternion orientation224 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size224 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box224 = new Box3D(position224, orientation224, size224);
      terrain.addRotatableBox(box224, YoAppearance.DarkGray());

      Point3D position225 = new Point3D(8.15561274022191, 1.3660623944541808, 0.04851321195122305);
      Quaternion orientation225 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size225 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box225 = new Box3D(position225, orientation225, size225);
      terrain.addRotatableBox(box225, YoAppearance.DarkGray());

      Point3D position226 = new Point3D(8.06995087947249, 1.6533393922315607, 0.04470427328368063);
      Quaternion orientation226 = new Quaternion(0.0, 0.0, 0.7089663975812704,0.7052422612837633);
      Vector3D size226 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box226 = new Box3D(position226, orientation226, size226);
      terrain.addRotatableBox(box226, YoAppearance.DarkGray());

      Point3D position227 = new Point3D(8.256712637978552, 1.6554844873357064, 0.0465446546756541);
      Quaternion orientation227 = new Quaternion(0.0, 0.0, 0.705726540586363,0.7084843328627703);
      Vector3D size227 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box227 = new Box3D(position227, orientation227, size227);
      terrain.addRotatableBox(box227, YoAppearance.DarkGray());

      Point3D position228 = new Point3D(8.534695414460673, 0.7978477297655059, 0.0436624434311707);
      Quaternion orientation228 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size228 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box228 = new Box3D(position228, orientation228, size228);
      terrain.addRotatableBox(box228, YoAppearance.DarkGray());

      Point3D position229 = new Point3D(8.540900973445865, 0.9886714770240965, 0.04400585133181643);
      Quaternion orientation229 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size229 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box229 = new Box3D(position229, orientation229, size229);
      terrain.addRotatableBox(box229, YoAppearance.DarkGray());

      Point3D position230 = new Point3D(8.446166935149789, 1.2761343214655514, 0.04558588339207196);
      Quaternion orientation230 = new Quaternion(0.0, 0.0, 0.7016853567916603,0.7124869543115583);
      Vector3D size230 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box230 = new Box3D(position230, orientation230, size230);
      terrain.addRotatableBox(box230, YoAppearance.DarkGray());

      Point3D position231 = new Point3D(8.629100011417771, 1.2789531615435958, 0.0401527710286485);
      Quaternion orientation231 = new Quaternion(0.0, 0.0, 0.7096844738128042,0.7045196573758914);
      Vector3D size231 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box231 = new Box3D(position231, orientation231, size231);
      terrain.addRotatableBox(box231, YoAppearance.DarkGray());

      Point3D position232 = new Point3D(8.544205652116847, 1.5635130993706658, 0.04861356986695628);
      Quaternion orientation232 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size232 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box232 = new Box3D(position232, orientation232, size232);
      terrain.addRotatableBox(box232, YoAppearance.DarkGray());

      Point3D position233 = new Point3D(8.5399389891589, 1.755300276983753, 0.04766115185873509);
      Quaternion orientation233 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size233 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box233 = new Box3D(position233, orientation233, size233);
      terrain.addRotatableBox(box233, YoAppearance.DarkGray());

      Point3D position234 = new Point3D(2.9991922858880433, 2.577385061030978, 0.07749255480474931);
      Quaternion orientation234 = new Quaternion(3.591011281523848E-4, -4.404093410555292E-4, -0.7140274385984735,0.7001174858654047);
      Vector3D size234 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box234 = new Box3D(position234, orientation234, size234);
      terrain.addRotatableBox(box234, YoAppearance.DarkGray());

      Point3D position235 = new Point3D(4.010313695756378, 2.5616056331805632, 0.07664394760851354);
      Quaternion orientation235 = new Quaternion(0.0010814627453136847, 0.0011373925186028025, -0.7128015132552289,0.7013640563057426);
      Vector3D size235 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box235 = new Box3D(position235, orientation235, size235);
      terrain.addRotatableBox(box235, YoAppearance.DarkGray());

      Point3D position236 = new Point3D(1.538532083708091, 3.07417019503108, 0.05131964350890132);
      Quaternion orientation236 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size236 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box236 = new Box3D(position236, orientation236, size236);
      terrain.addRotatableBox(box236, YoAppearance.DarkGray());

      Point3D position237 = new Point3D(1.5377122418466396, 2.887199518721233, 0.0453699739307727);
      Quaternion orientation237 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size237 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box237 = new Box3D(position237, orientation237, size237);
      terrain.addRotatableBox(box237, YoAppearance.DarkGray());

      Point3D position238 = new Point3D(1.8318418085847752, 2.9832432291386377, 0.05026246658367628);
      Quaternion orientation238 = new Quaternion(0.0, 0.0, -0.7032798171527006,0.7109131443331625);
      Vector3D size238 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box238 = new Box3D(position238, orientation238, size238);
      terrain.addRotatableBox(box238, YoAppearance.DarkGray());

      Point3D position239 = new Point3D(2.0226812492006916, 2.9816220959595907, 0.058535600491900285);
      Quaternion orientation239 = new Quaternion(0.0, 0.0, -0.7033244049950813,0.7108690324794821);
      Vector3D size239 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box239 = new Box3D(position239, orientation239, size239);
      terrain.addRotatableBox(box239, YoAppearance.DarkGray());

      Point3D position240 = new Point3D(1.9258075527472558, 2.6916058787804267, 0.04914770991489481);
      Quaternion orientation240 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size240 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box240 = new Box3D(position240, orientation240, size240);
      terrain.addRotatableBox(box240, YoAppearance.DarkGray());

      Point3D position241 = new Point3D(1.9221267562656839, 2.497105461572966, 0.048374686104427383);
      Quaternion orientation241 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size241 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box241 = new Box3D(position241, orientation241, size241);
      terrain.addRotatableBox(box241, YoAppearance.DarkGray());

      Point3D position242 = new Point3D(1.6391139250203717, 2.6000302880881714, 0.03909051303520233);
      Quaternion orientation242 = new Quaternion(4.2221998162823653E-4, -0.0021771612644779165, -0.7083346233855763,0.7058733193798512);
      Vector3D size242 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box242 = new Box3D(position242, orientation242, size242);
      terrain.addRotatableBox(box242, YoAppearance.DarkGray());

      Point3D position243 = new Point3D(1.4508258759747759, 2.591873785777104, 0.03772504674415519);
      Quaternion orientation243 = new Quaternion(0.0, 0.0, 0.7038510074598656,0.7103476327107258);
      Vector3D size243 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box243 = new Box3D(position243, orientation243, size243);
      terrain.addRotatableBox(box243, YoAppearance.DarkGray());

      Point3D position244 = new Point3D(1.544254053576254, 2.3063150007485467, 0.032965432002729965);
      Quaternion orientation244 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size244 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box244 = new Box3D(position244, orientation244, size244);
      terrain.addRotatableBox(box244, YoAppearance.DarkGray());

      Point3D position245 = new Point3D(1.5421604510537665, 2.11523180468053, 0.03169062655032244);
      Quaternion orientation245 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size245 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box245 = new Box3D(position245, orientation245, size245);
      terrain.addRotatableBox(box245, YoAppearance.DarkGray());

      Point3D position246 = new Point3D(1.8302064340875628, 2.2122749433120887, 0.03233411894842265);
      Quaternion orientation246 = new Quaternion(0.0, 0.0, -0.7041659007794283,0.7100354809300001);
      Vector3D size246 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box246 = new Box3D(position246, orientation246, size246);
      terrain.addRotatableBox(box246, YoAppearance.DarkGray());

      Point3D position247 = new Point3D(2.020076111592974, 2.2054981613848237, 0.032951829300953354);
      Quaternion orientation247 = new Quaternion(0.0, 0.0, 0.7094730567086567,0.7047325604826807);
      Vector3D size247 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box247 = new Box3D(position247, orientation247, size247);
      terrain.addRotatableBox(box247, YoAppearance.DarkGray());

      Point3D position248 = new Point3D(2.3017726687557736, 3.0659211869311394, 0.050732948560174566);
      Quaternion orientation248 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size248 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box248 = new Box3D(position248, orientation248, size248);
      terrain.addRotatableBox(box248, YoAppearance.DarkGray());

      Point3D position249 = new Point3D(2.3045613478518816, 2.8744061119031095, 0.04678489901068887);
      Quaternion orientation249 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size249 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box249 = new Box3D(position249, orientation249, size249);
      terrain.addRotatableBox(box249, YoAppearance.DarkGray());

      Point3D position250 = new Point3D(2.214159178501207, 2.590925622627419, 0.04546863118006792);
      Quaternion orientation250 = new Quaternion(0.0, 0.0, -0.710045270209128,0.7041560297644597);
      Vector3D size250 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box250 = new Box3D(position250, orientation250, size250);
      terrain.addRotatableBox(box250, YoAppearance.DarkGray());

      Point3D position251 = new Point3D(2.3882625449800243, 2.585485705317335, 0.04358061639227621);
      Quaternion orientation251 = new Quaternion(0.0, 0.0, 0.7032305077721192,0.7109619208780226);
      Vector3D size251 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box251 = new Box3D(position251, orientation251, size251);
      terrain.addRotatableBox(box251, YoAppearance.DarkGray());

      Point3D position252 = new Point3D(2.305334951328886, 2.3033087493012614, 0.04026317060808564);
      Quaternion orientation252 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size252 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box252 = new Box3D(position252, orientation252, size252);
      terrain.addRotatableBox(box252, YoAppearance.DarkGray());

      Point3D position253 = new Point3D(2.3035618801058884, 2.1148277365412183, 0.043838376674458175);
      Quaternion orientation253 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size253 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box253 = new Box3D(position253, orientation253, size253);
      terrain.addRotatableBox(box253, YoAppearance.DarkGray());

      Point3D position254 = new Point3D(5.022827545101327, 2.543592721151784, 0.06929944023803199);
      Quaternion orientation254 = new Quaternion(0.0, 0.0, -0.7146326573404546,0.6994999392870028);
      Vector3D size254 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box254 = new Box3D(position254, orientation254, size254);
      terrain.addRotatableBox(box254, YoAppearance.DarkGray());

      Point3D position255 = new Point3D(6.0368497952581395, 2.503842144945642, 0.06172802598521781);
      Quaternion orientation255 = new Quaternion(0.005613854283208643, 0.0055820130552961455, -0.7143726400604131,0.6997206992103728);
      Vector3D size255 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box255 = new Box3D(position255, orientation255, size255);
      terrain.addRotatableBox(box255, YoAppearance.DarkGray());

      Point3D position256 = new Point3D(7.050523277840206, 2.4828946360839605, 0.06864253875907342);
      Quaternion orientation256 = new Quaternion(0.0, 0.0, 0.7017961773791698,0.7123777968297335);
      Vector3D size256 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box256 = new Box3D(position256, orientation256, size256);
      terrain.addRotatableBox(box256, YoAppearance.DarkGray());

      Point3D position257 = new Point3D(5.023846234292196, 2.5361370347496166, 0.22602295943394107);
      Quaternion orientation257 = new Quaternion(0.0, 0.0, 0.7009087265528491,0.71325097759629);
      Vector3D size257 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box257 = new Box3D(position257, orientation257, size257);
      terrain.addRotatableBox(box257, YoAppearance.DarkGray());

      Point3D position258 = new Point3D(2.6900015009577163, 3.0436366016069014, 0.19963009876119422);
      Quaternion orientation258 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size258 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box258 = new Box3D(position258, orientation258, size258);
      terrain.addRotatableBox(box258, YoAppearance.DarkGray());

      Point3D position259 = new Point3D(2.6890564650088904, 2.8543195036924747, 0.1998904919894186);
      Quaternion orientation259 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size259 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box259 = new Box3D(position259, orientation259, size259);
      terrain.addRotatableBox(box259, YoAppearance.DarkGray());

      Point3D position260 = new Point3D(2.7797616190377004, 2.564961870088415, 0.24403572659127876);
      Quaternion orientation260 = new Quaternion(0.09461018209558089, 0.09407245118837755, -0.7027793486796466,0.6987849987232575);
      Vector3D size260 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box260 = new Box3D(position260, orientation260, size260);
      terrain.addRotatableBox(box260, YoAppearance.DarkGray());

      Point3D position261 = new Point3D(2.593463551704851, 2.562190880492245, 0.24443600486584452);
      Quaternion orientation261 = new Quaternion(0.09234953713579733, -0.09290494477039353, 0.698908799916006,0.7031121700183992);
      Vector3D size261 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box261 = new Box3D(position261, orientation261, size261);
      terrain.addRotatableBox(box261, YoAppearance.DarkGray());

      Point3D position262 = new Point3D(2.684024192233144, 2.2693810052425665, 0.19603696340587146);
      Quaternion orientation262 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size262 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box262 = new Box3D(position262, orientation262, size262);
      terrain.addRotatableBox(box262, YoAppearance.DarkGray());

      Point3D position263 = new Point3D(2.6860150703200567, 2.079548037610604, 0.19828815283388773);
      Quaternion orientation263 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size263 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box263 = new Box3D(position263, orientation263, size263);
      terrain.addRotatableBox(box263, YoAppearance.DarkGray());

      Point3D position264 = new Point3D(3.1711383025454465, 2.9462180514407015, 0.28166428275520083);
      Quaternion orientation264 = new Quaternion(0.10793831101703633, 0.10894156349569274, -0.6955023910392171,0.7019668658965574);
      Vector3D size264 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box264 = new Box3D(position264, orientation264, size264);
      terrain.addRotatableBox(box264, YoAppearance.DarkGray());

      Point3D position265 = new Point3D(2.9818424101161205, 2.9424769634680707, 0.28083800074621);
      Quaternion orientation265 = new Quaternion(0.11223315066187517, -0.11138798116147307, 0.7008445880063211,0.6955668918281909);
      Vector3D size265 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box265 = new Box3D(position265, orientation265, size265);
      terrain.addRotatableBox(box265, YoAppearance.DarkGray());

      Point3D position266 = new Point3D(3.0756668786094545, 2.657339884824034, 0.22430098450189007);
      Quaternion orientation266 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size266 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box266 = new Box3D(position266, orientation266, size266);
      terrain.addRotatableBox(box266, YoAppearance.DarkGray());

      Point3D position267 = new Point3D(3.0722024139657997, 2.467834661837208, 0.2251948202315323);
      Quaternion orientation267 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size267 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box267 = new Box3D(position267, orientation267, size267);
      terrain.addRotatableBox(box267, YoAppearance.DarkGray());

      Point3D position268 = new Point3D(3.165664362999164, 2.174777955911507, 0.27735751751742505);
      Quaternion orientation268 = new Quaternion(0.09437843324707647, 0.09404280191325493, -0.7020498469308203,0.6995531968124677);
      Vector3D size268 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box268 = new Box3D(position268, orientation268, size268);
      terrain.addRotatableBox(box268, YoAppearance.DarkGray());

      Point3D position269 = new Point3D(2.974840053678399, 2.174801242282441, 0.2788927721576176);
      Quaternion orientation269 = new Quaternion(0.09994070761295208, -0.09964072755297639, 0.7010812345776699,0.6989768829493477);
      Vector3D size269 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box269 = new Box3D(position269, orientation269, size269);
      terrain.addRotatableBox(box269, YoAppearance.DarkGray());

      Point3D position270 = new Point3D(3.359147869089035, 2.950879534175364, 0.22486106332834727);
      Quaternion orientation270 = new Quaternion(0.0, 0.0, -0.7071194347376097,0.7070941274090483);
      Vector3D size270 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box270 = new Box3D(position270, orientation270, size270);
      terrain.addRotatableBox(box270, YoAppearance.DarkGray());

      Point3D position271 = new Point3D(3.354472458529534, 2.5578339761550426, 0.22395977136362658);
      Quaternion orientation271 = new Quaternion(0.0, 0.0, -0.7058785319428242,0.7083329006492944);
      Vector3D size271 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box271 = new Box3D(position271, orientation271, size271);
      terrain.addRotatableBox(box271, YoAppearance.DarkGray());

      Point3D position272 = new Point3D(3.350402623341776, 2.1730833666883966, 0.22640085739147484);
      Quaternion orientation272 = new Quaternion(0.0, 0.0, 0.7062242020231959,0.7079882601263245);
      Vector3D size272 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box272 = new Box3D(position272, orientation272, size272);
      terrain.addRotatableBox(box272, YoAppearance.DarkGray());

      Point3D position273 = new Point3D(3.68866762247556, 3.0236265834842824, 0.3010889371484357);
      Quaternion orientation273 = new Quaternion(-5.839236425510448E-34, 0.14469060323807462, 8.859744206110364E-18,0.9894769473487506);
      Vector3D size273 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box273 = new Box3D(position273, orientation273, size273);
      terrain.addRotatableBox(box273, YoAppearance.DarkGray());

      Point3D position274 = new Point3D(3.7706107394082076, 2.551861846659785, 0.2940640116728258);
      Quaternion orientation274 = new Quaternion(-0.08864451472761907, -0.08908549481655237, -0.6997578049577696,0.7032388918587189);
      Vector3D size274 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box274 = new Box3D(position274, orientation274, size274);
      terrain.addRotatableBox(box274, YoAppearance.DarkGray());

      Point3D position275 = new Point3D(3.68965927337154, 2.8388934247654856, 0.30028645012494665);
      Quaternion orientation275 = new Quaternion(1.9454724300576844E-34, 0.14138358070604393, 8.657247478182429E-18,0.989954889430189);
      Vector3D size275 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box275 = new Box3D(position275, orientation275, size275);
      terrain.addRotatableBox(box275, YoAppearance.DarkGray());

      Point3D position276 = new Point3D(3.588806285529547, 2.5548401365418387, 0.2930951718708826);
      Quaternion orientation276 = new Quaternion(-0.08427871374180397, 0.09588259957957021, 0.7076885857849924,0.6948960289550722);
      Vector3D size276 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box276 = new Box3D(position276, orientation276, size276);
      terrain.addRotatableBox(box276, YoAppearance.DarkGray());

      Point3D position277 = new Point3D(3.6964906211913506, 2.2561355063359256, 0.24588881335187482);
      Quaternion orientation277 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size277 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box277 = new Box3D(position277, orientation277, size277);
      terrain.addRotatableBox(box277, YoAppearance.DarkGray());

      Point3D position278 = new Point3D(3.6860421913290207, 2.0666362915851755, 0.24832633958306163);
      Quaternion orientation278 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size278 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box278 = new Box3D(position278, orientation278, size278);
      terrain.addRotatableBox(box278, YoAppearance.DarkGray());

      Point3D position279 = new Point3D(3.9745492483528864, 2.1636677867081713, 0.3000158195677145);
      Quaternion orientation279 = new Quaternion(0.09976532226249955, -0.09967911277827533, 0.7003421510403348,0.6997369694576615);
      Vector3D size279 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box279 = new Box3D(position279, orientation279, size279);
      terrain.addRotatableBox(box279, YoAppearance.DarkGray());

      Point3D position280 = new Point3D(4.1570571372942995, 2.9351500725177173, 0.3009665879051755);
      Quaternion orientation280 = new Quaternion(0.0, 0.0, -0.7031379195773428,0.7110534902891948);
      Vector3D size280 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box280 = new Box3D(position280, orientation280, size280);
      terrain.addRotatableBox(box280, YoAppearance.DarkGray());

      Point3D position281 = new Point3D(3.976449906945254, 2.9313406371863273, 0.30163369997832445);
      Quaternion orientation281 = new Quaternion(0.0, 0.0, 0.7053477339779695,0.7088614633143374);
      Vector3D size281 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box281 = new Box3D(position281, orientation281, size281);
      terrain.addRotatableBox(box281, YoAppearance.DarkGray());

      Point3D position282 = new Point3D(4.060087891099629, 2.6392964224398483, 0.24882388075031026);
      Quaternion orientation282 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size282 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box282 = new Box3D(position282, orientation282, size282);
      terrain.addRotatableBox(box282, YoAppearance.DarkGray());

      Point3D position283 = new Point3D(4.063602710036821, 2.4525705083227396, 0.2470906830235654);
      Quaternion orientation283 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size283 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box283 = new Box3D(position283, orientation283, size283);
      terrain.addRotatableBox(box283, YoAppearance.DarkGray());

      Point3D position284 = new Point3D(4.147655172749554, 2.1634003119005563, 0.2989374948751531);
      Quaternion orientation284 = new Quaternion(0.09713879505240461, 0.09719288147916494, -0.7002040662118151,0.7005939365609611);
      Vector3D size284 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box284 = new Box3D(position284, orientation284, size284);
      terrain.addRotatableBox(box284, YoAppearance.DarkGray());

      Point3D position285 = new Point3D(4.343356717444665, 2.1648819072177, 0.30128052384367776);
      Quaternion orientation285 = new Quaternion(0.0844889112032617, -0.11355828784903144, 0.6956075277336033,0.7043410441716627);
      Vector3D size285 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box285 = new Box3D(position285, orientation285, size285);
      terrain.addRotatableBox(box285, YoAppearance.DarkGray());

      Point3D position286 = new Point3D(4.349272116834419, 2.5503364005449605, 0.23830329314841697);
      Quaternion orientation286 = new Quaternion(0.0, 0.0, -0.7115262234714076,0.7026595429598296);
      Vector3D size286 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box286 = new Box3D(position286, orientation286, size286);
      terrain.addRotatableBox(box286, YoAppearance.DarkGray());

      Point3D position287 = new Point3D(4.346481880050672, 2.926745674840747, 0.3000392880382346);
      Quaternion orientation287 = new Quaternion(0.0, 0.0, -0.7065132530735921,0.7076998115241875);
      Vector3D size287 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box287 = new Box3D(position287, orientation287, size287);
      terrain.addRotatableBox(box287, YoAppearance.DarkGray());

      Point3D position288 = new Point3D(4.702380013634424, 3.007933614813388, 0.3797638118070186);
      Quaternion orientation288 = new Quaternion(1.6449470919030792E-33, 0.09792926395956664, 5.9964379825470255E-18,0.9951933778217867);
      Vector3D size288 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box288 = new Box3D(position288, orientation288, size288);
      terrain.addRotatableBox(box288, YoAppearance.DarkGray());

      Point3D position289 = new Point3D(4.703890280086529, 2.8155963177652987, 0.37962892936597686);
      Quaternion orientation289 = new Quaternion(1.9358735945371405E-34, 0.10122575159006256, 6.1982896338026795E-18,0.9948634816973769);
      Vector3D size289 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box289 = new Box3D(position289, orientation289, size289);
      terrain.addRotatableBox(box289, YoAppearance.DarkGray());

      Point3D position290 = new Point3D(4.792888242703473, 2.524894208219748, 0.3479451232275179);
      Quaternion orientation290 = new Quaternion(0.0, 0.0, -0.7035865871495094,0.7106095372166812);
      Vector3D size290 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box290 = new Box3D(position290, orientation290, size290);
      terrain.addRotatableBox(box290, YoAppearance.DarkGray());

      Point3D position291 = new Point3D(4.610354981743812, 2.529117286057229, 0.3475132664494318);
      Quaternion orientation291 = new Quaternion(0.0, 0.0, -0.7132505655401289,0.7009091458646308);
      Vector3D size291 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box291 = new Box3D(position291, orientation291, size291);
      terrain.addRotatableBox(box291, YoAppearance.DarkGray());

      Point3D position292 = new Point3D(4.712970930242387, 2.239798910349343, 0.3912550730752458);
      Quaternion orientation292 = new Quaternion(-3.880765412410759E-34, -0.12182502599465175, -7.459631407019642E-18,0.9925515921308083);
      Vector3D size292 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box292 = new Box3D(position292, orientation292, size292);
      terrain.addRotatableBox(box292, YoAppearance.DarkGray());

      Point3D position293 = new Point3D(4.707914463288, 2.05139465722848, 0.3891170604695318);
      Quaternion orientation293 = new Quaternion(2.3291328065487197E-33, -0.1241412431733023, -7.601458804717881E-18,0.9922645573350823);
      Vector3D size293 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box293 = new Box3D(position293, orientation293, size293);
      terrain.addRotatableBox(box293, YoAppearance.DarkGray());

      Point3D position294 = new Point3D(5.1727760970934185, 2.9112340340750826, 0.3967895082089883);
      Quaternion orientation294 = new Quaternion(0.09472131621407967, 0.09590058699963736, -0.6963057302396933,0.7049746660009598);
      Vector3D size294 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box294 = new Box3D(position294, orientation294, size294);
      terrain.addRotatableBox(box294, YoAppearance.DarkGray());

      Point3D position295 = new Point3D(4.986578998410523, 2.90457288301287, 0.3944322980633405);
      Quaternion orientation295 = new Quaternion(0.09774852819681804, -0.09770925509640942, 0.7004613865448716,0.700179957342037);
      Vector3D size295 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box295 = new Box3D(position295, orientation295, size295);
      terrain.addRotatableBox(box295, YoAppearance.DarkGray());

      Point3D position296 = new Point3D(5.181946498191193, 2.515938621233747, 0.3935301841017318);
      Quaternion orientation296 = new Quaternion(-0.09454705714016172, 0.09446453379644658, 0.7010687973196257,0.7004568846585553);
      Vector3D size296 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box296 = new Box3D(position296, orientation296, size296);
      terrain.addRotatableBox(box296, YoAppearance.DarkGray());

      Point3D position297 = new Point3D(4.999908472643262, 2.522712273228591, 0.3936477908269693);
      Quaternion orientation297 = new Quaternion(-0.0943438948778358, -0.0947342877428095, -0.6993100325946019,0.7022037614095209);
      Vector3D size297 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box297 = new Box3D(position297, orientation297, size297);
      terrain.addRotatableBox(box297, YoAppearance.DarkGray());

      Point3D position298 = new Point3D(5.181134977089534, 2.135174520419811, 0.34319902751110637);
      Quaternion orientation298 = new Quaternion(0.0, 0.0, -0.7087044927674649,0.7055054513830564);
      Vector3D size298 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box298 = new Box3D(position298, orientation298, size298);
      terrain.addRotatableBox(box298, YoAppearance.DarkGray());

      Point3D position299 = new Point3D(5.001168160091194, 2.1310266893680323, 0.3442518243348339);
      Quaternion orientation299 = new Quaternion(0.0, 0.0, 0.7085911489028207,0.7056192909045079);
      Vector3D size299 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box299 = new Box3D(position299, orientation299, size299);
      terrain.addRotatableBox(box299, YoAppearance.DarkGray());

      Point3D position300 = new Point3D(5.361423516842161, 2.137318361252478, 0.3467424072643232);
      Quaternion orientation300 = new Quaternion(0.0, 0.0, -0.702830138774019,0.7113577131309489);
      Vector3D size300 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box300 = new Box3D(position300, orientation300, size300);
      terrain.addRotatableBox(box300, YoAppearance.DarkGray());

      Point3D position301 = new Point3D(5.368768670021752, 2.5171590670986497, 0.38910279983955076);
      Quaternion orientation301 = new Quaternion(-0.09458307226381174, -0.09416431082175267, -0.7023337283279424,0.6992241836933095);
      Vector3D size301 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box301 = new Box3D(position301, orientation301, size301);
      terrain.addRotatableBox(box301, YoAppearance.DarkGray());

      Point3D position302 = new Point3D(5.359430458701423, 2.9074604944131375, 0.3951426086328146);
      Quaternion orientation302 = new Quaternion(0.09529196339253196, -0.09533170427575388, 0.7005076469963307,0.7007997890784077);
      Vector3D size302 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box302 = new Box3D(position302, orientation302, size302);
      terrain.addRotatableBox(box302, YoAppearance.DarkGray());

      Point3D position303 = new Point3D(5.75258973537161, 2.049509218011508, 0.227418511493009);
      Quaternion orientation303 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size303 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box303 = new Box3D(position303, orientation303, size303);
      terrain.addRotatableBox(box303, YoAppearance.DarkGray());

      Point3D position304 = new Point3D(5.753494342409687, 2.231547818499434, 0.2278999120441889);
      Quaternion orientation304 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size304 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box304 = new Box3D(position304, orientation304, size304);
      terrain.addRotatableBox(box304, YoAppearance.DarkGray());

      Point3D position305 = new Point3D(5.648721454782096, 2.522749962848247, 0.27732476658080424);
      Quaternion orientation305 = new Quaternion(0.0, 0.0, -0.7107650931702618,0.7034294437473235);
      Vector3D size305 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box305 = new Box3D(position305, orientation305, size305);
      terrain.addRotatableBox(box305, YoAppearance.DarkGray());

      Point3D position306 = new Point3D(5.840403719567024, 2.524951038535571, 0.27650382371337645);
      Quaternion orientation306 = new Quaternion(0.0, 0.0, 0.7053417880043341,0.7088673797649665);
      Vector3D size306 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box306 = new Box3D(position306, orientation306, size306);
      terrain.addRotatableBox(box306, YoAppearance.DarkGray());

      Point3D position307 = new Point3D(5.7577226491388185, 2.811636291956968, 0.28389741291306736);
      Quaternion orientation307 = new Quaternion(0.0, 0.11262021465784755, 6.89599927000101E-18,0.993638106782555);
      Vector3D size307 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box307 = new Box3D(position307, orientation307, size307);
      terrain.addRotatableBox(box307, YoAppearance.DarkGray());

      Point3D position308 = new Point3D(5.764448795159363, 3.0034155096950474, 0.28467265927325774);
      Quaternion orientation308 = new Quaternion(-1.0658594972039655E-33, 0.11109613212788319, 6.802676130403094E-18,0.9938096645868483);
      Vector3D size308 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box308 = new Box3D(position308, orientation308, size308);
      terrain.addRotatableBox(box308, YoAppearance.DarkGray());

      Point3D position309 = new Point3D(6.2352107198329065, 2.141193884518639, 0.2684214567166878);
      Quaternion orientation309 = new Quaternion(-0.09099327976522753, 0.09151827134292549, 0.6991736284516058,0.7032075556518044);
      Vector3D size309 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box309 = new Box3D(position309, orientation309, size309);
      terrain.addRotatableBox(box309, YoAppearance.DarkGray());

      Point3D position310 = new Point3D(6.041424771298938, 2.13820242335934, 0.2701155633865791);
      Quaternion orientation310 = new Quaternion(-0.09049938070702158, -0.09029400308019075, -0.7021010186557202,0.7005076835424435);
      Vector3D size310 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box310 = new Box3D(position310, orientation310, size310);
      terrain.addRotatableBox(box310, YoAppearance.DarkGray());

      Point3D position311 = new Point3D(6.124353907177331, 2.6127045442208194, 0.23404920128215498);
      Quaternion orientation311 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size311 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box311 = new Box3D(position311, orientation311, size311);
      terrain.addRotatableBox(box311, YoAppearance.DarkGray());

      Point3D position312 = new Point3D(6.421349657210157, 2.1341988779457886, 0.26733468034992436);
      Quaternion orientation312 = new Quaternion(-0.09224861237772161, 0.09388297766102086, 0.6947734684371486,0.7070827445042763);
      Vector3D size312 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box312 = new Box3D(position312, orientation312, size312);
      terrain.addRotatableBox(box312, YoAppearance.DarkGray());

      Point3D position313 = new Point3D(6.2386546198104, 2.901919693850186, 0.2618318096377476);
      Quaternion orientation313 = new Quaternion(-0.0924918376852269, 0.09412884116858322, 0.6947475169322426,0.7070437814862335);
      Vector3D size313 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box313 = new Box3D(position313, orientation313, size313);
      terrain.addRotatableBox(box313, YoAppearance.DarkGray());

      Point3D position314 = new Point3D(6.051805117872733, 2.907538004269914, 0.26005353871268083);
      Quaternion orientation314 = new Quaternion(-0.09694408237307763, 0.08981925984154682, 0.7007993258860914,0.7010097362319289);
      Vector3D size314 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box314 = new Box3D(position314, orientation314, size314);
      terrain.addRotatableBox(box314, YoAppearance.DarkGray());

      Point3D position315 = new Point3D(6.122908361470702, 2.420253416476654, 0.2312384964958033);
      Quaternion orientation315 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size315 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box315 = new Box3D(position315, orientation315, size315);
      terrain.addRotatableBox(box315, YoAppearance.DarkGray());

      Point3D position316 = new Point3D(6.409268200039842, 2.519341287283029, 0.23604260025687943);
      Quaternion orientation316 = new Quaternion(0.0, 0.0, 0.7016890425161331,0.7124833244454164);
      Vector3D size316 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box316 = new Box3D(position316, orientation316, size316);
      terrain.addRotatableBox(box316, YoAppearance.DarkGray());

      Point3D position317 = new Point3D(6.429966481857306, 2.9009960961841728, 0.26300251335578917);
      Quaternion orientation317 = new Quaternion(-0.09163052675672367, 0.092769864463877, 0.6967248198275796,0.7053879246555514);
      Vector3D size317 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box317 = new Box3D(position317, orientation317, size317);
      terrain.addRotatableBox(box317, YoAppearance.DarkGray());

      Point3D position318 = new Point3D(6.699741918264115, 2.0368430634539143, 0.2550180515425791);
      Quaternion orientation318 = new Quaternion(1.7479183742545392E-33, 0.1288981738241679, 7.892736799485345E-18,0.9916578345300332);
      Vector3D size318 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box318 = new Box3D(position318, orientation318, size318);
      terrain.addRotatableBox(box318, YoAppearance.DarkGray());

      Point3D position319 = new Point3D(6.698630441512664, 2.224966891239924, 0.2526101274005956);
      Quaternion orientation319 = new Quaternion(-3.884427274751654E-34, 0.1292203005277809, 7.912461371310276E-18,0.9916159104872764);
      Vector3D size319 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box319 = new Box3D(position319, orientation319, size319);
      terrain.addRotatableBox(box319, YoAppearance.DarkGray());

      Point3D position320 = new Point3D(6.797501566111999, 2.512343358516751, 0.2633885198693278);
      Quaternion orientation320 = new Quaternion(0.09819637521077755, -0.10061717351576993, 0.691507811143679,0.708555293125695);
      Vector3D size320 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box320 = new Box3D(position320, orientation320, size320);
      terrain.addRotatableBox(box320, YoAppearance.DarkGray());

      Point3D position321 = new Point3D(6.61320792222252, 2.5110518380849336, 0.2609523583831561);
      Quaternion orientation321 = new Quaternion(0.09905634457071809, -0.09993018618950954, 0.6950116745649737,0.7050961428818118);
      Vector3D size321 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box321 = new Box3D(position321, orientation321, size321);
      terrain.addRotatableBox(box321, YoAppearance.DarkGray());

      Point3D position322 = new Point3D(6.7127304242037615, 2.803850171586372, 0.21934622785676056);
      Quaternion orientation322 = new Quaternion(0.0, 0.0, -0.0038144316225337672,0.9999927250292359);
      Vector3D size322 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box322 = new Box3D(position322, orientation322, size322);
      terrain.addRotatableBox(box322, YoAppearance.DarkGray());

      Point3D position323 = new Point3D(6.718241475052683, 2.996187721186475, 0.214411381592355);
      Quaternion orientation323 = new Quaternion(0.0, 0.0, -0.009312552251322237,0.999956637245119);
      Vector3D size323 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box323 = new Box3D(position323, orientation323, size323);
      terrain.addRotatableBox(box323, YoAppearance.DarkGray());

      Point3D position324 = new Point3D(6.983093459558476, 2.119949020961721, 0.21557870456215963);
      Quaternion orientation324 = new Quaternion(0.0, 0.0, 0.7030358342720265,0.7111544246712073);
      Vector3D size324 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box324 = new Box3D(position324, orientation324, size324);
      terrain.addRotatableBox(box324, YoAppearance.DarkGray());

      Point3D position325 = new Point3D(6.988683516017568, 2.5109997905306978, 0.25958187507798225);
      Quaternion orientation325 = new Quaternion(0.10036878834372606, -0.10248996160407609, 0.6924383325630104,0.7070721813889761);
      Vector3D size325 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box325 = new Box3D(position325, orientation325, size325);
      terrain.addRotatableBox(box325, YoAppearance.DarkGray());

      Point3D position326 = new Point3D(7.00489079733141, 2.903503983647086, 0.2167604197386731);
      Quaternion orientation326 = new Quaternion(0.0, 0.0, 0.7019672015374099,0.7122092725917976);
      Vector3D size326 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box326 = new Box3D(position326, orientation326, size326);
      terrain.addRotatableBox(box326, YoAppearance.DarkGray());

      Point3D position327 = new Point3D(7.345613673095441, 2.03781818412057, 0.18851614172025807);
      Quaternion orientation327 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size327 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box327 = new Box3D(position327, orientation327, size327);
      terrain.addRotatableBox(box327, YoAppearance.DarkGray());

      Point3D position328 = new Point3D(7.346728260451079, 2.2238257573051765, 0.19148708763994512);
      Quaternion orientation328 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size328 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box328 = new Box3D(position328, orientation328, size328);
      terrain.addRotatableBox(box328, YoAppearance.DarkGray());

      Point3D position329 = new Point3D(7.256713781322195, 2.517469336215263, 0.19034824545194998);
      Quaternion orientation329 = new Quaternion(0.0, 0.0, -0.7109847545862554,0.7032074222773266);
      Vector3D size329 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box329 = new Box3D(position329, orientation329, size329);
      terrain.addRotatableBox(box329, YoAppearance.DarkGray());

      Point3D position330 = new Point3D(7.436520392537069, 2.5116348793188017, 0.18978383802880525);
      Quaternion orientation330 = new Quaternion(0.0, 0.0, 0.7079054853446147,0.7063071738408194);
      Vector3D size330 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box330 = new Box3D(position330, orientation330, size330);
      terrain.addRotatableBox(box330, YoAppearance.DarkGray());

      Point3D position331 = new Point3D(7.356717455086338, 2.80091839642546, 0.22230115969485045);
      Quaternion orientation331 = new Quaternion(0.0, 0.11441875733241588, 7.006128246478086E-18,0.9934326086708176);
      Vector3D size331 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box331 = new Box3D(position331, orientation331, size331);
      terrain.addRotatableBox(box331, YoAppearance.DarkGray());

      Point3D position332 = new Point3D(7.3566302205035035, 2.987809313874047, 0.22224355113151553);
      Quaternion orientation332 = new Quaternion(-1.5505289284134202E-33, 0.11216790604418489, 6.8683033552035814E-18,0.9936892677561043);
      Vector3D size332 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box332 = new Box3D(position332, orientation332, size332);
      terrain.addRotatableBox(box332, YoAppearance.DarkGray());

      Point3D position333 = new Point3D(7.770134742471376, 1.9994473567954634, 0.047256945955076055);
      Quaternion orientation333 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size333 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box333 = new Box3D(position333, orientation333, size333);
      terrain.addRotatableBox(box333, YoAppearance.DarkGray());

      Point3D position334 = new Point3D(7.770217973789706, 2.1862392411090608, 0.04430979308496823);
      Quaternion orientation334 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size334 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box334 = new Box3D(position334, orientation334, size334);
      terrain.addRotatableBox(box334, YoAppearance.DarkGray());

      Point3D position335 = new Point3D(7.869196761927994, 2.4759408547219164, 0.04358384009610045);
      Quaternion orientation335 = new Quaternion(0.0, 0.0, 0.703468250129445,0.7107266852031212);
      Vector3D size335 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box335 = new Box3D(position335, orientation335, size335);
      terrain.addRotatableBox(box335, YoAppearance.DarkGray());

      Point3D position336 = new Point3D(7.682970194739092, 2.4770525701116535, 0.047271829008987146);
      Quaternion orientation336 = new Quaternion(0.0, 0.0, 0.7034450413479783,0.7107496562102167);
      Vector3D size336 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box336 = new Box3D(position336, orientation336, size336);
      terrain.addRotatableBox(box336, YoAppearance.DarkGray());

      Point3D position337 = new Point3D(7.779256902384008, 2.7659919336394694, 0.04432955446553308);
      Quaternion orientation337 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size337 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box337 = new Box3D(position337, orientation337, size337);
      terrain.addRotatableBox(box337, YoAppearance.DarkGray());

      Point3D position338 = new Point3D(7.785783227158354, 2.9557054734046524, 0.045068700704124554);
      Quaternion orientation338 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size338 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box338 = new Box3D(position338, orientation338, size338);
      terrain.addRotatableBox(box338, YoAppearance.DarkGray());

      Point3D position339 = new Point3D(8.252153182891886, 2.094370688035923, 0.04285320203865197);
      Quaternion orientation339 = new Quaternion(0.0, 0.0, 0.704181506150831,0.7100200042218158);
      Vector3D size339 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box339 = new Box3D(position339, orientation339, size339);
      terrain.addRotatableBox(box339, YoAppearance.DarkGray());

      Point3D position340 = new Point3D(8.064675866921874, 2.1012450058603163, 0.046016306708178786);
      Quaternion orientation340 = new Quaternion(0.0, 0.0, 0.7060501535261512,0.7081618322853177);
      Vector3D size340 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box340 = new Box3D(position340, orientation340, size340);
      terrain.addRotatableBox(box340, YoAppearance.DarkGray());

      Point3D position341 = new Point3D(8.152449598918341, 2.3793016522023445, 0.047082419865977894);
      Quaternion orientation341 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size341 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box341 = new Box3D(position341, orientation341, size341);
      terrain.addRotatableBox(box341, YoAppearance.DarkGray());

      Point3D position342 = new Point3D(8.15561274022191, 2.566062394454181, 0.04851321195122305);
      Quaternion orientation342 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size342 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box342 = new Box3D(position342, orientation342, size342);
      terrain.addRotatableBox(box342, YoAppearance.DarkGray());

      Point3D position343 = new Point3D(8.06995087947249, 2.8533393922315606, 0.04470427328368063);
      Quaternion orientation343 = new Quaternion(0.0, 0.0, 0.7089663975812704,0.7052422612837633);
      Vector3D size343 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box343 = new Box3D(position343, orientation343, size343);
      terrain.addRotatableBox(box343, YoAppearance.DarkGray());

      Point3D position344 = new Point3D(8.256712637978552, 2.8554844873357066, 0.0465446546756541);
      Quaternion orientation344 = new Quaternion(0.0, 0.0, 0.705726540586363,0.7084843328627703);
      Vector3D size344 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box344 = new Box3D(position344, orientation344, size344);
      terrain.addRotatableBox(box344, YoAppearance.DarkGray());

      Point3D position345 = new Point3D(8.534695414460673, 1.997847729765506, 0.0436624434311707);
      Quaternion orientation345 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size345 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box345 = new Box3D(position345, orientation345, size345);
      terrain.addRotatableBox(box345, YoAppearance.DarkGray());

      Point3D position346 = new Point3D(8.540900973445865, 2.1886714770240965, 0.04400585133181643);
      Quaternion orientation346 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size346 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box346 = new Box3D(position346, orientation346, size346);
      terrain.addRotatableBox(box346, YoAppearance.DarkGray());

      Point3D position347 = new Point3D(8.446166935149789, 2.476134321465551, 0.04558588339207196);
      Quaternion orientation347 = new Quaternion(0.0, 0.0, 0.7016853567916603,0.7124869543115583);
      Vector3D size347 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box347 = new Box3D(position347, orientation347, size347);
      terrain.addRotatableBox(box347, YoAppearance.DarkGray());

      Point3D position348 = new Point3D(8.629100011417771, 2.478953161543596, 0.0401527710286485);
      Quaternion orientation348 = new Quaternion(0.0, 0.0, 0.7096844738128042,0.7045196573758914);
      Vector3D size348 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box348 = new Box3D(position348, orientation348, size348);
      terrain.addRotatableBox(box348, YoAppearance.DarkGray());

      Point3D position349 = new Point3D(8.544205652116847, 2.763513099370666, 0.04861356986695628);
      Quaternion orientation349 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size349 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box349 = new Box3D(position349, orientation349, size349);
      terrain.addRotatableBox(box349, YoAppearance.DarkGray());

      Point3D position350 = new Point3D(8.5399389891589, 2.9553002769837526, 0.04766115185873509);
      Quaternion orientation350 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size350 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box350 = new Box3D(position350, orientation350, size350);
      terrain.addRotatableBox(box350, YoAppearance.DarkGray());

      Point3D position351 = new Point3D(2.9991922858880433, 3.7773850610309783, 0.07749255480474931);
      Quaternion orientation351 = new Quaternion(3.591011281523848E-4, -4.404093410555292E-4, -0.7140274385984735,0.7001174858654047);
      Vector3D size351 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box351 = new Box3D(position351, orientation351, size351);
      terrain.addRotatableBox(box351, YoAppearance.DarkGray());

      Point3D position352 = new Point3D(4.010313695756378, 3.7616056331805634, 0.07664394760851354);
      Quaternion orientation352 = new Quaternion(0.0010814627453136847, 0.0011373925186028025, -0.7128015132552289,0.7013640563057426);
      Vector3D size352 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box352 = new Box3D(position352, orientation352, size352);
      terrain.addRotatableBox(box352, YoAppearance.DarkGray());

      Point3D position353 = new Point3D(1.538532083708091, 4.27417019503108, 0.05131964350890132);
      Quaternion orientation353 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size353 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box353 = new Box3D(position353, orientation353, size353);
      terrain.addRotatableBox(box353, YoAppearance.DarkGray());

      Point3D position354 = new Point3D(1.5377122418466396, 4.087199518721233, 0.0453699739307727);
      Quaternion orientation354 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size354 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box354 = new Box3D(position354, orientation354, size354);
      terrain.addRotatableBox(box354, YoAppearance.DarkGray());

      Point3D position355 = new Point3D(1.8318418085847752, 4.183243229138638, 0.05026246658367628);
      Quaternion orientation355 = new Quaternion(0.0, 0.0, -0.7032798171527006,0.7109131443331625);
      Vector3D size355 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box355 = new Box3D(position355, orientation355, size355);
      terrain.addRotatableBox(box355, YoAppearance.DarkGray());

      Point3D position356 = new Point3D(2.0226812492006916, 4.1816220959595904, 0.058535600491900285);
      Quaternion orientation356 = new Quaternion(0.0, 0.0, -0.7033244049950813,0.7108690324794821);
      Vector3D size356 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box356 = new Box3D(position356, orientation356, size356);
      terrain.addRotatableBox(box356, YoAppearance.DarkGray());

      Point3D position357 = new Point3D(1.9258075527472558, 3.8916058787804264, 0.04914770991489481);
      Quaternion orientation357 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size357 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box357 = new Box3D(position357, orientation357, size357);
      terrain.addRotatableBox(box357, YoAppearance.DarkGray());

      Point3D position358 = new Point3D(1.9221267562656839, 3.697105461572966, 0.048374686104427383);
      Quaternion orientation358 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size358 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box358 = new Box3D(position358, orientation358, size358);
      terrain.addRotatableBox(box358, YoAppearance.DarkGray());

      Point3D position359 = new Point3D(1.6391139250203717, 3.8000302880881716, 0.03909051303520233);
      Quaternion orientation359 = new Quaternion(4.2221998162823653E-4, -0.0021771612644779165, -0.7083346233855763,0.7058733193798512);
      Vector3D size359 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box359 = new Box3D(position359, orientation359, size359);
      terrain.addRotatableBox(box359, YoAppearance.DarkGray());

      Point3D position360 = new Point3D(1.4508258759747759, 3.7918737857771037, 0.03772504674415519);
      Quaternion orientation360 = new Quaternion(0.0, 0.0, 0.7038510074598656,0.7103476327107258);
      Vector3D size360 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box360 = new Box3D(position360, orientation360, size360);
      terrain.addRotatableBox(box360, YoAppearance.DarkGray());

      Point3D position361 = new Point3D(1.544254053576254, 3.506315000748547, 0.032965432002729965);
      Quaternion orientation361 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size361 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box361 = new Box3D(position361, orientation361, size361);
      terrain.addRotatableBox(box361, YoAppearance.DarkGray());

      Point3D position362 = new Point3D(1.5421604510537665, 3.31523180468053, 0.03169062655032244);
      Quaternion orientation362 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size362 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box362 = new Box3D(position362, orientation362, size362);
      terrain.addRotatableBox(box362, YoAppearance.DarkGray());

      Point3D position363 = new Point3D(1.8302064340875628, 3.412274943312089, 0.03233411894842265);
      Quaternion orientation363 = new Quaternion(0.0, 0.0, -0.7041659007794283,0.7100354809300001);
      Vector3D size363 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box363 = new Box3D(position363, orientation363, size363);
      terrain.addRotatableBox(box363, YoAppearance.DarkGray());

      Point3D position364 = new Point3D(2.020076111592974, 3.4054981613848234, 0.032951829300953354);
      Quaternion orientation364 = new Quaternion(0.0, 0.0, 0.7094730567086567,0.7047325604826807);
      Vector3D size364 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box364 = new Box3D(position364, orientation364, size364);
      terrain.addRotatableBox(box364, YoAppearance.DarkGray());

      Point3D position365 = new Point3D(2.3017726687557736, 4.2659211869311395, 0.050732948560174566);
      Quaternion orientation365 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size365 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box365 = new Box3D(position365, orientation365, size365);
      terrain.addRotatableBox(box365, YoAppearance.DarkGray());

      Point3D position366 = new Point3D(2.3045613478518816, 4.07440611190311, 0.04678489901068887);
      Quaternion orientation366 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size366 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box366 = new Box3D(position366, orientation366, size366);
      terrain.addRotatableBox(box366, YoAppearance.DarkGray());

      Point3D position367 = new Point3D(2.214159178501207, 3.7909256226274186, 0.04546863118006792);
      Quaternion orientation367 = new Quaternion(0.0, 0.0, -0.710045270209128,0.7041560297644597);
      Vector3D size367 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box367 = new Box3D(position367, orientation367, size367);
      terrain.addRotatableBox(box367, YoAppearance.DarkGray());

      Point3D position368 = new Point3D(2.3882625449800243, 3.7854857053173347, 0.04358061639227621);
      Quaternion orientation368 = new Quaternion(0.0, 0.0, 0.7032305077721192,0.7109619208780226);
      Vector3D size368 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box368 = new Box3D(position368, orientation368, size368);
      terrain.addRotatableBox(box368, YoAppearance.DarkGray());

      Point3D position369 = new Point3D(2.305334951328886, 3.5033087493012616, 0.04026317060808564);
      Quaternion orientation369 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size369 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box369 = new Box3D(position369, orientation369, size369);
      terrain.addRotatableBox(box369, YoAppearance.DarkGray());

      Point3D position370 = new Point3D(2.3035618801058884, 3.3148277365412184, 0.043838376674458175);
      Quaternion orientation370 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size370 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box370 = new Box3D(position370, orientation370, size370);
      terrain.addRotatableBox(box370, YoAppearance.DarkGray());

      Point3D position371 = new Point3D(5.022827545101327, 3.743592721151784, 0.06929944023803199);
      Quaternion orientation371 = new Quaternion(0.0, 0.0, -0.7146326573404546,0.6994999392870028);
      Vector3D size371 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box371 = new Box3D(position371, orientation371, size371);
      terrain.addRotatableBox(box371, YoAppearance.DarkGray());

      Point3D position372 = new Point3D(6.0368497952581395, 3.703842144945642, 0.06172802598521781);
      Quaternion orientation372 = new Quaternion(0.005613854283208643, 0.0055820130552961455, -0.7143726400604131,0.6997206992103728);
      Vector3D size372 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box372 = new Box3D(position372, orientation372, size372);
      terrain.addRotatableBox(box372, YoAppearance.DarkGray());

      Point3D position373 = new Point3D(7.050523277840206, 3.6828946360839607, 0.06864253875907342);
      Quaternion orientation373 = new Quaternion(0.0, 0.0, 0.7017961773791698,0.7123777968297335);
      Vector3D size373 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box373 = new Box3D(position373, orientation373, size373);
      terrain.addRotatableBox(box373, YoAppearance.DarkGray());

      Point3D position374 = new Point3D(5.023846234292196, 3.736137034749617, 0.22602295943394107);
      Quaternion orientation374 = new Quaternion(0.0, 0.0, 0.7009087265528491,0.71325097759629);
      Vector3D size374 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box374 = new Box3D(position374, orientation374, size374);
      terrain.addRotatableBox(box374, YoAppearance.DarkGray());

      Point3D position375 = new Point3D(2.6900015009577163, 4.243636601606902, 0.19963009876119422);
      Quaternion orientation375 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size375 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box375 = new Box3D(position375, orientation375, size375);
      terrain.addRotatableBox(box375, YoAppearance.DarkGray());

      Point3D position376 = new Point3D(2.6890564650088904, 4.054319503692475, 0.1998904919894186);
      Quaternion orientation376 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size376 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box376 = new Box3D(position376, orientation376, size376);
      terrain.addRotatableBox(box376, YoAppearance.DarkGray());

      Point3D position377 = new Point3D(2.7797616190377004, 3.764961870088415, 0.24403572659127876);
      Quaternion orientation377 = new Quaternion(0.09461018209558089, 0.09407245118837755, -0.7027793486796466,0.6987849987232575);
      Vector3D size377 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box377 = new Box3D(position377, orientation377, size377);
      terrain.addRotatableBox(box377, YoAppearance.DarkGray());

      Point3D position378 = new Point3D(2.593463551704851, 3.762190880492245, 0.24443600486584452);
      Quaternion orientation378 = new Quaternion(0.09234953713579733, -0.09290494477039353, 0.698908799916006,0.7031121700183992);
      Vector3D size378 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box378 = new Box3D(position378, orientation378, size378);
      terrain.addRotatableBox(box378, YoAppearance.DarkGray());

      Point3D position379 = new Point3D(2.684024192233144, 3.4693810052425667, 0.19603696340587146);
      Quaternion orientation379 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size379 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box379 = new Box3D(position379, orientation379, size379);
      terrain.addRotatableBox(box379, YoAppearance.DarkGray());

      Point3D position380 = new Point3D(2.6860150703200567, 3.2795480376106037, 0.19828815283388773);
      Quaternion orientation380 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size380 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box380 = new Box3D(position380, orientation380, size380);
      terrain.addRotatableBox(box380, YoAppearance.DarkGray());

      Point3D position381 = new Point3D(3.1711383025454465, 4.146218051440702, 0.28166428275520083);
      Quaternion orientation381 = new Quaternion(0.10793831101703633, 0.10894156349569274, -0.6955023910392171,0.7019668658965574);
      Vector3D size381 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box381 = new Box3D(position381, orientation381, size381);
      terrain.addRotatableBox(box381, YoAppearance.DarkGray());

      Point3D position382 = new Point3D(2.9818424101161205, 4.142476963468071, 0.28083800074621);
      Quaternion orientation382 = new Quaternion(0.11223315066187517, -0.11138798116147307, 0.7008445880063211,0.6955668918281909);
      Vector3D size382 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box382 = new Box3D(position382, orientation382, size382);
      terrain.addRotatableBox(box382, YoAppearance.DarkGray());

      Point3D position383 = new Point3D(3.0756668786094545, 3.857339884824034, 0.22430098450189007);
      Quaternion orientation383 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size383 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box383 = new Box3D(position383, orientation383, size383);
      terrain.addRotatableBox(box383, YoAppearance.DarkGray());

      Point3D position384 = new Point3D(3.0722024139657997, 3.667834661837208, 0.2251948202315323);
      Quaternion orientation384 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size384 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box384 = new Box3D(position384, orientation384, size384);
      terrain.addRotatableBox(box384, YoAppearance.DarkGray());

      Point3D position385 = new Point3D(3.165664362999164, 3.374777955911507, 0.27735751751742505);
      Quaternion orientation385 = new Quaternion(0.09437843324707647, 0.09404280191325493, -0.7020498469308203,0.6995531968124677);
      Vector3D size385 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box385 = new Box3D(position385, orientation385, size385);
      terrain.addRotatableBox(box385, YoAppearance.DarkGray());

      Point3D position386 = new Point3D(2.974840053678399, 3.374801242282441, 0.2788927721576176);
      Quaternion orientation386 = new Quaternion(0.09994070761295208, -0.09964072755297639, 0.7010812345776699,0.6989768829493477);
      Vector3D size386 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box386 = new Box3D(position386, orientation386, size386);
      terrain.addRotatableBox(box386, YoAppearance.DarkGray());

      Point3D position387 = new Point3D(3.359147869089035, 4.150879534175364, 0.22486106332834727);
      Quaternion orientation387 = new Quaternion(0.0, 0.0, -0.7071194347376097,0.7070941274090483);
      Vector3D size387 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box387 = new Box3D(position387, orientation387, size387);
      terrain.addRotatableBox(box387, YoAppearance.DarkGray());

      Point3D position388 = new Point3D(3.354472458529534, 3.7578339761550428, 0.22395977136362658);
      Quaternion orientation388 = new Quaternion(0.0, 0.0, -0.7058785319428242,0.7083329006492944);
      Vector3D size388 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box388 = new Box3D(position388, orientation388, size388);
      terrain.addRotatableBox(box388, YoAppearance.DarkGray());

      Point3D position389 = new Point3D(3.350402623341776, 3.3730833666883964, 0.22640085739147484);
      Quaternion orientation389 = new Quaternion(0.0, 0.0, 0.7062242020231959,0.7079882601263245);
      Vector3D size389 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box389 = new Box3D(position389, orientation389, size389);
      terrain.addRotatableBox(box389, YoAppearance.DarkGray());

      Point3D position390 = new Point3D(3.68866762247556, 4.2236265834842825, 0.3010889371484357);
      Quaternion orientation390 = new Quaternion(-5.839236425510448E-34, 0.14469060323807462, 8.859744206110364E-18,0.9894769473487506);
      Vector3D size390 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box390 = new Box3D(position390, orientation390, size390);
      terrain.addRotatableBox(box390, YoAppearance.DarkGray());

      Point3D position391 = new Point3D(3.7706107394082076, 3.751861846659785, 0.2940640116728258);
      Quaternion orientation391 = new Quaternion(-0.08864451472761907, -0.08908549481655237, -0.6997578049577696,0.7032388918587189);
      Vector3D size391 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box391 = new Box3D(position391, orientation391, size391);
      terrain.addRotatableBox(box391, YoAppearance.DarkGray());

      Point3D position392 = new Point3D(3.68965927337154, 4.038893424765486, 0.30028645012494665);
      Quaternion orientation392 = new Quaternion(1.9454724300576844E-34, 0.14138358070604393, 8.657247478182429E-18,0.989954889430189);
      Vector3D size392 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box392 = new Box3D(position392, orientation392, size392);
      terrain.addRotatableBox(box392, YoAppearance.DarkGray());

      Point3D position393 = new Point3D(3.588806285529547, 3.754840136541839, 0.2930951718708826);
      Quaternion orientation393 = new Quaternion(-0.08427871374180397, 0.09588259957957021, 0.7076885857849924,0.6948960289550722);
      Vector3D size393 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box393 = new Box3D(position393, orientation393, size393);
      terrain.addRotatableBox(box393, YoAppearance.DarkGray());

      Point3D position394 = new Point3D(3.6964906211913506, 3.456135506335926, 0.24588881335187482);
      Quaternion orientation394 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size394 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box394 = new Box3D(position394, orientation394, size394);
      terrain.addRotatableBox(box394, YoAppearance.DarkGray());

      Point3D position395 = new Point3D(3.6860421913290207, 3.2666362915851757, 0.24832633958306163);
      Quaternion orientation395 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size395 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box395 = new Box3D(position395, orientation395, size395);
      terrain.addRotatableBox(box395, YoAppearance.DarkGray());

      Point3D position396 = new Point3D(3.9745492483528864, 3.363667786708171, 0.3000158195677145);
      Quaternion orientation396 = new Quaternion(0.09976532226249955, -0.09967911277827533, 0.7003421510403348,0.6997369694576615);
      Vector3D size396 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box396 = new Box3D(position396, orientation396, size396);
      terrain.addRotatableBox(box396, YoAppearance.DarkGray());

      Point3D position397 = new Point3D(4.1570571372942995, 4.1351500725177175, 0.3009665879051755);
      Quaternion orientation397 = new Quaternion(0.0, 0.0, -0.7031379195773428,0.7110534902891948);
      Vector3D size397 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box397 = new Box3D(position397, orientation397, size397);
      terrain.addRotatableBox(box397, YoAppearance.DarkGray());

      Point3D position398 = new Point3D(3.976449906945254, 4.1313406371863275, 0.30163369997832445);
      Quaternion orientation398 = new Quaternion(0.0, 0.0, 0.7053477339779695,0.7088614633143374);
      Vector3D size398 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box398 = new Box3D(position398, orientation398, size398);
      terrain.addRotatableBox(box398, YoAppearance.DarkGray());

      Point3D position399 = new Point3D(4.060087891099629, 3.8392964224398485, 0.24882388075031026);
      Quaternion orientation399 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size399 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box399 = new Box3D(position399, orientation399, size399);
      terrain.addRotatableBox(box399, YoAppearance.DarkGray());

      Point3D position400 = new Point3D(4.063602710036821, 3.6525705083227393, 0.2470906830235654);
      Quaternion orientation400 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size400 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box400 = new Box3D(position400, orientation400, size400);
      terrain.addRotatableBox(box400, YoAppearance.DarkGray());

      Point3D position401 = new Point3D(4.147655172749554, 3.3634003119005564, 0.2989374948751531);
      Quaternion orientation401 = new Quaternion(0.09713879505240461, 0.09719288147916494, -0.7002040662118151,0.7005939365609611);
      Vector3D size401 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box401 = new Box3D(position401, orientation401, size401);
      terrain.addRotatableBox(box401, YoAppearance.DarkGray());

      Point3D position402 = new Point3D(4.343356717444665, 3.3648819072176996, 0.30128052384367776);
      Quaternion orientation402 = new Quaternion(0.0844889112032617, -0.11355828784903144, 0.6956075277336033,0.7043410441716627);
      Vector3D size402 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box402 = new Box3D(position402, orientation402, size402);
      terrain.addRotatableBox(box402, YoAppearance.DarkGray());

      Point3D position403 = new Point3D(4.349272116834419, 3.7503364005449606, 0.23830329314841697);
      Quaternion orientation403 = new Quaternion(0.0, 0.0, -0.7115262234714076,0.7026595429598296);
      Vector3D size403 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box403 = new Box3D(position403, orientation403, size403);
      terrain.addRotatableBox(box403, YoAppearance.DarkGray());

      Point3D position404 = new Point3D(4.346481880050672, 4.126745674840747, 0.3000392880382346);
      Quaternion orientation404 = new Quaternion(0.0, 0.0, -0.7065132530735921,0.7076998115241875);
      Vector3D size404 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box404 = new Box3D(position404, orientation404, size404);
      terrain.addRotatableBox(box404, YoAppearance.DarkGray());

      Point3D position405 = new Point3D(4.702380013634424, 4.207933614813388, 0.3797638118070186);
      Quaternion orientation405 = new Quaternion(1.6449470919030792E-33, 0.09792926395956664, 5.9964379825470255E-18,0.9951933778217867);
      Vector3D size405 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box405 = new Box3D(position405, orientation405, size405);
      terrain.addRotatableBox(box405, YoAppearance.DarkGray());

      Point3D position406 = new Point3D(4.703890280086529, 4.015596317765299, 0.37962892936597686);
      Quaternion orientation406 = new Quaternion(1.9358735945371405E-34, 0.10122575159006256, 6.1982896338026795E-18,0.9948634816973769);
      Vector3D size406 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box406 = new Box3D(position406, orientation406, size406);
      terrain.addRotatableBox(box406, YoAppearance.DarkGray());

      Point3D position407 = new Point3D(4.792888242703473, 3.724894208219748, 0.3479451232275179);
      Quaternion orientation407 = new Quaternion(0.0, 0.0, -0.7035865871495094,0.7106095372166812);
      Vector3D size407 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box407 = new Box3D(position407, orientation407, size407);
      terrain.addRotatableBox(box407, YoAppearance.DarkGray());

      Point3D position408 = new Point3D(4.610354981743812, 3.729117286057229, 0.3475132664494318);
      Quaternion orientation408 = new Quaternion(0.0, 0.0, -0.7132505655401289,0.7009091458646308);
      Vector3D size408 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box408 = new Box3D(position408, orientation408, size408);
      terrain.addRotatableBox(box408, YoAppearance.DarkGray());

      Point3D position409 = new Point3D(4.712970930242387, 3.439798910349343, 0.3912550730752458);
      Quaternion orientation409 = new Quaternion(-3.880765412410759E-34, -0.12182502599465175, -7.459631407019642E-18,0.9925515921308083);
      Vector3D size409 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box409 = new Box3D(position409, orientation409, size409);
      terrain.addRotatableBox(box409, YoAppearance.DarkGray());

      Point3D position410 = new Point3D(4.707914463288, 3.25139465722848, 0.3891170604695318);
      Quaternion orientation410 = new Quaternion(2.3291328065487197E-33, -0.1241412431733023, -7.601458804717881E-18,0.9922645573350823);
      Vector3D size410 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box410 = new Box3D(position410, orientation410, size410);
      terrain.addRotatableBox(box410, YoAppearance.DarkGray());

      Point3D position411 = new Point3D(5.1727760970934185, 4.111234034075083, 0.3967895082089883);
      Quaternion orientation411 = new Quaternion(0.09472131621407967, 0.09590058699963736, -0.6963057302396933,0.7049746660009598);
      Vector3D size411 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box411 = new Box3D(position411, orientation411, size411);
      terrain.addRotatableBox(box411, YoAppearance.DarkGray());

      Point3D position412 = new Point3D(4.986578998410523, 4.10457288301287, 0.3944322980633405);
      Quaternion orientation412 = new Quaternion(0.09774852819681804, -0.09770925509640942, 0.7004613865448716,0.700179957342037);
      Vector3D size412 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box412 = new Box3D(position412, orientation412, size412);
      terrain.addRotatableBox(box412, YoAppearance.DarkGray());

      Point3D position413 = new Point3D(5.181946498191193, 3.7159386212337466, 0.3935301841017318);
      Quaternion orientation413 = new Quaternion(-0.09454705714016172, 0.09446453379644658, 0.7010687973196257,0.7004568846585553);
      Vector3D size413 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box413 = new Box3D(position413, orientation413, size413);
      terrain.addRotatableBox(box413, YoAppearance.DarkGray());

      Point3D position414 = new Point3D(4.999908472643262, 3.7227122732285913, 0.3936477908269693);
      Quaternion orientation414 = new Quaternion(-0.0943438948778358, -0.0947342877428095, -0.6993100325946019,0.7022037614095209);
      Vector3D size414 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box414 = new Box3D(position414, orientation414, size414);
      terrain.addRotatableBox(box414, YoAppearance.DarkGray());

      Point3D position415 = new Point3D(5.181134977089534, 3.3351745204198107, 0.34319902751110637);
      Quaternion orientation415 = new Quaternion(0.0, 0.0, -0.7087044927674649,0.7055054513830564);
      Vector3D size415 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box415 = new Box3D(position415, orientation415, size415);
      terrain.addRotatableBox(box415, YoAppearance.DarkGray());

      Point3D position416 = new Point3D(5.001168160091194, 3.3310266893680325, 0.3442518243348339);
      Quaternion orientation416 = new Quaternion(0.0, 0.0, 0.7085911489028207,0.7056192909045079);
      Vector3D size416 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box416 = new Box3D(position416, orientation416, size416);
      terrain.addRotatableBox(box416, YoAppearance.DarkGray());

      Point3D position417 = new Point3D(5.361423516842161, 3.3373183612524784, 0.3467424072643232);
      Quaternion orientation417 = new Quaternion(0.0, 0.0, -0.702830138774019,0.7113577131309489);
      Vector3D size417 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box417 = new Box3D(position417, orientation417, size417);
      terrain.addRotatableBox(box417, YoAppearance.DarkGray());

      Point3D position418 = new Point3D(5.368768670021752, 3.71715906709865, 0.38910279983955076);
      Quaternion orientation418 = new Quaternion(-0.09458307226381174, -0.09416431082175267, -0.7023337283279424,0.6992241836933095);
      Vector3D size418 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box418 = new Box3D(position418, orientation418, size418);
      terrain.addRotatableBox(box418, YoAppearance.DarkGray());

      Point3D position419 = new Point3D(5.359430458701423, 4.107460494413138, 0.3951426086328146);
      Quaternion orientation419 = new Quaternion(0.09529196339253196, -0.09533170427575388, 0.7005076469963307,0.7007997890784077);
      Vector3D size419 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box419 = new Box3D(position419, orientation419, size419);
      terrain.addRotatableBox(box419, YoAppearance.DarkGray());

      Point3D position420 = new Point3D(5.75258973537161, 3.249509218011508, 0.227418511493009);
      Quaternion orientation420 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size420 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box420 = new Box3D(position420, orientation420, size420);
      terrain.addRotatableBox(box420, YoAppearance.DarkGray());

      Point3D position421 = new Point3D(5.753494342409687, 3.4315478184994337, 0.2278999120441889);
      Quaternion orientation421 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size421 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box421 = new Box3D(position421, orientation421, size421);
      terrain.addRotatableBox(box421, YoAppearance.DarkGray());

      Point3D position422 = new Point3D(5.648721454782096, 3.7227499628482468, 0.27732476658080424);
      Quaternion orientation422 = new Quaternion(0.0, 0.0, -0.7107650931702618,0.7034294437473235);
      Vector3D size422 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box422 = new Box3D(position422, orientation422, size422);
      terrain.addRotatableBox(box422, YoAppearance.DarkGray());

      Point3D position423 = new Point3D(5.840403719567024, 3.724951038535571, 0.27650382371337645);
      Quaternion orientation423 = new Quaternion(0.0, 0.0, 0.7053417880043341,0.7088673797649665);
      Vector3D size423 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box423 = new Box3D(position423, orientation423, size423);
      terrain.addRotatableBox(box423, YoAppearance.DarkGray());

      Point3D position424 = new Point3D(5.7577226491388185, 4.011636291956968, 0.28389741291306736);
      Quaternion orientation424 = new Quaternion(0.0, 0.11262021465784755, 6.89599927000101E-18,0.993638106782555);
      Vector3D size424 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box424 = new Box3D(position424, orientation424, size424);
      terrain.addRotatableBox(box424, YoAppearance.DarkGray());

      Point3D position425 = new Point3D(5.764448795159363, 4.203415509695048, 0.28467265927325774);
      Quaternion orientation425 = new Quaternion(-1.0658594972039655E-33, 0.11109613212788319, 6.802676130403094E-18,0.9938096645868483);
      Vector3D size425 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box425 = new Box3D(position425, orientation425, size425);
      terrain.addRotatableBox(box425, YoAppearance.DarkGray());

      Point3D position426 = new Point3D(6.2352107198329065, 3.3411938845186393, 0.2684214567166878);
      Quaternion orientation426 = new Quaternion(-0.09099327976522753, 0.09151827134292549, 0.6991736284516058,0.7032075556518044);
      Vector3D size426 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box426 = new Box3D(position426, orientation426, size426);
      terrain.addRotatableBox(box426, YoAppearance.DarkGray());

      Point3D position427 = new Point3D(6.041424771298938, 3.3382024233593404, 0.2701155633865791);
      Quaternion orientation427 = new Quaternion(-0.09049938070702158, -0.09029400308019075, -0.7021010186557202,0.7005076835424435);
      Vector3D size427 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box427 = new Box3D(position427, orientation427, size427);
      terrain.addRotatableBox(box427, YoAppearance.DarkGray());

      Point3D position428 = new Point3D(6.124353907177331, 3.8127045442208196, 0.23404920128215498);
      Quaternion orientation428 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size428 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box428 = new Box3D(position428, orientation428, size428);
      terrain.addRotatableBox(box428, YoAppearance.DarkGray());

      Point3D position429 = new Point3D(6.421349657210157, 3.3341988779457887, 0.26733468034992436);
      Quaternion orientation429 = new Quaternion(-0.09224861237772161, 0.09388297766102086, 0.6947734684371486,0.7070827445042763);
      Vector3D size429 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box429 = new Box3D(position429, orientation429, size429);
      terrain.addRotatableBox(box429, YoAppearance.DarkGray());

      Point3D position430 = new Point3D(6.2386546198104, 4.101919693850186, 0.2618318096377476);
      Quaternion orientation430 = new Quaternion(-0.0924918376852269, 0.09412884116858322, 0.6947475169322426,0.7070437814862335);
      Vector3D size430 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box430 = new Box3D(position430, orientation430, size430);
      terrain.addRotatableBox(box430, YoAppearance.DarkGray());

      Point3D position431 = new Point3D(6.051805117872733, 4.107538004269914, 0.26005353871268083);
      Quaternion orientation431 = new Quaternion(-0.09694408237307763, 0.08981925984154682, 0.7007993258860914,0.7010097362319289);
      Vector3D size431 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box431 = new Box3D(position431, orientation431, size431);
      terrain.addRotatableBox(box431, YoAppearance.DarkGray());

      Point3D position432 = new Point3D(6.122908361470702, 3.620253416476654, 0.2312384964958033);
      Quaternion orientation432 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size432 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box432 = new Box3D(position432, orientation432, size432);
      terrain.addRotatableBox(box432, YoAppearance.DarkGray());

      Point3D position433 = new Point3D(6.409268200039842, 3.7193412872830294, 0.23604260025687943);
      Quaternion orientation433 = new Quaternion(0.0, 0.0, 0.7016890425161331,0.7124833244454164);
      Vector3D size433 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box433 = new Box3D(position433, orientation433, size433);
      terrain.addRotatableBox(box433, YoAppearance.DarkGray());

      Point3D position434 = new Point3D(6.429966481857306, 4.100996096184173, 0.26300251335578917);
      Quaternion orientation434 = new Quaternion(-0.09163052675672367, 0.092769864463877, 0.6967248198275796,0.7053879246555514);
      Vector3D size434 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box434 = new Box3D(position434, orientation434, size434);
      terrain.addRotatableBox(box434, YoAppearance.DarkGray());

      Point3D position435 = new Point3D(6.699741918264115, 3.2368430634539145, 0.2550180515425791);
      Quaternion orientation435 = new Quaternion(1.7479183742545392E-33, 0.1288981738241679, 7.892736799485345E-18,0.9916578345300332);
      Vector3D size435 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box435 = new Box3D(position435, orientation435, size435);
      terrain.addRotatableBox(box435, YoAppearance.DarkGray());

      Point3D position436 = new Point3D(6.698630441512664, 3.424966891239924, 0.2526101274005956);
      Quaternion orientation436 = new Quaternion(-3.884427274751654E-34, 0.1292203005277809, 7.912461371310276E-18,0.9916159104872764);
      Vector3D size436 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box436 = new Box3D(position436, orientation436, size436);
      terrain.addRotatableBox(box436, YoAppearance.DarkGray());

      Point3D position437 = new Point3D(6.797501566111999, 3.712343358516751, 0.2633885198693278);
      Quaternion orientation437 = new Quaternion(0.09819637521077755, -0.10061717351576993, 0.691507811143679,0.708555293125695);
      Vector3D size437 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box437 = new Box3D(position437, orientation437, size437);
      terrain.addRotatableBox(box437, YoAppearance.DarkGray());

      Point3D position438 = new Point3D(6.61320792222252, 3.7110518380849333, 0.2609523583831561);
      Quaternion orientation438 = new Quaternion(0.09905634457071809, -0.09993018618950954, 0.6950116745649737,0.7050961428818118);
      Vector3D size438 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box438 = new Box3D(position438, orientation438, size438);
      terrain.addRotatableBox(box438, YoAppearance.DarkGray());

      Point3D position439 = new Point3D(6.7127304242037615, 4.003850171586372, 0.21934622785676056);
      Quaternion orientation439 = new Quaternion(0.0, 0.0, -0.0038144316225337672,0.9999927250292359);
      Vector3D size439 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box439 = new Box3D(position439, orientation439, size439);
      terrain.addRotatableBox(box439, YoAppearance.DarkGray());

      Point3D position440 = new Point3D(6.718241475052683, 4.196187721186475, 0.214411381592355);
      Quaternion orientation440 = new Quaternion(0.0, 0.0, -0.009312552251322237,0.999956637245119);
      Vector3D size440 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box440 = new Box3D(position440, orientation440, size440);
      terrain.addRotatableBox(box440, YoAppearance.DarkGray());

      Point3D position441 = new Point3D(6.983093459558476, 3.3199490209617206, 0.21557870456215963);
      Quaternion orientation441 = new Quaternion(0.0, 0.0, 0.7030358342720265,0.7111544246712073);
      Vector3D size441 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box441 = new Box3D(position441, orientation441, size441);
      terrain.addRotatableBox(box441, YoAppearance.DarkGray());

      Point3D position442 = new Point3D(6.988683516017568, 3.710999790530698, 0.25958187507798225);
      Quaternion orientation442 = new Quaternion(0.10036878834372606, -0.10248996160407609, 0.6924383325630104,0.7070721813889761);
      Vector3D size442 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box442 = new Box3D(position442, orientation442, size442);
      terrain.addRotatableBox(box442, YoAppearance.DarkGray());

      Point3D position443 = new Point3D(7.00489079733141, 4.103503983647086, 0.2167604197386731);
      Quaternion orientation443 = new Quaternion(0.0, 0.0, 0.7019672015374099,0.7122092725917976);
      Vector3D size443 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box443 = new Box3D(position443, orientation443, size443);
      terrain.addRotatableBox(box443, YoAppearance.DarkGray());

      Point3D position444 = new Point3D(7.345613673095441, 3.23781818412057, 0.18851614172025807);
      Quaternion orientation444 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size444 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box444 = new Box3D(position444, orientation444, size444);
      terrain.addRotatableBox(box444, YoAppearance.DarkGray());

      Point3D position445 = new Point3D(7.346728260451079, 3.4238257573051767, 0.19148708763994512);
      Quaternion orientation445 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size445 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box445 = new Box3D(position445, orientation445, size445);
      terrain.addRotatableBox(box445, YoAppearance.DarkGray());

      Point3D position446 = new Point3D(7.256713781322195, 3.717469336215263, 0.19034824545194998);
      Quaternion orientation446 = new Quaternion(0.0, 0.0, -0.7109847545862554,0.7032074222773266);
      Vector3D size446 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box446 = new Box3D(position446, orientation446, size446);
      terrain.addRotatableBox(box446, YoAppearance.DarkGray());

      Point3D position447 = new Point3D(7.436520392537069, 3.711634879318802, 0.18978383802880525);
      Quaternion orientation447 = new Quaternion(0.0, 0.0, 0.7079054853446147,0.7063071738408194);
      Vector3D size447 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box447 = new Box3D(position447, orientation447, size447);
      terrain.addRotatableBox(box447, YoAppearance.DarkGray());

      Point3D position448 = new Point3D(7.356717455086338, 4.00091839642546, 0.22230115969485045);
      Quaternion orientation448 = new Quaternion(0.0, 0.11441875733241588, 7.006128246478086E-18,0.9934326086708176);
      Vector3D size448 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box448 = new Box3D(position448, orientation448, size448);
      terrain.addRotatableBox(box448, YoAppearance.DarkGray());

      Point3D position449 = new Point3D(7.3566302205035035, 4.187809313874047, 0.22224355113151553);
      Quaternion orientation449 = new Quaternion(-1.5505289284134202E-33, 0.11216790604418489, 6.8683033552035814E-18,0.9936892677561043);
      Vector3D size449 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box449 = new Box3D(position449, orientation449, size449);
      terrain.addRotatableBox(box449, YoAppearance.DarkGray());

      Point3D position450 = new Point3D(7.770134742471376, 3.1994473567954635, 0.047256945955076055);
      Quaternion orientation450 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size450 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box450 = new Box3D(position450, orientation450, size450);
      terrain.addRotatableBox(box450, YoAppearance.DarkGray());

      Point3D position451 = new Point3D(7.770217973789706, 3.3862392411090605, 0.04430979308496823);
      Quaternion orientation451 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size451 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box451 = new Box3D(position451, orientation451, size451);
      terrain.addRotatableBox(box451, YoAppearance.DarkGray());

      Point3D position452 = new Point3D(7.869196761927994, 3.675940854721916, 0.04358384009610045);
      Quaternion orientation452 = new Quaternion(0.0, 0.0, 0.703468250129445,0.7107266852031212);
      Vector3D size452 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box452 = new Box3D(position452, orientation452, size452);
      terrain.addRotatableBox(box452, YoAppearance.DarkGray());

      Point3D position453 = new Point3D(7.682970194739092, 3.6770525701116537, 0.047271829008987146);
      Quaternion orientation453 = new Quaternion(0.0, 0.0, 0.7034450413479783,0.7107496562102167);
      Vector3D size453 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box453 = new Box3D(position453, orientation453, size453);
      terrain.addRotatableBox(box453, YoAppearance.DarkGray());

      Point3D position454 = new Point3D(7.779256902384008, 3.9659919336394696, 0.04432955446553308);
      Quaternion orientation454 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size454 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box454 = new Box3D(position454, orientation454, size454);
      terrain.addRotatableBox(box454, YoAppearance.DarkGray());

      Point3D position455 = new Point3D(7.785783227158354, 4.155705473404653, 0.045068700704124554);
      Quaternion orientation455 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size455 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box455 = new Box3D(position455, orientation455, size455);
      terrain.addRotatableBox(box455, YoAppearance.DarkGray());

      Point3D position456 = new Point3D(8.252153182891886, 3.294370688035923, 0.04285320203865197);
      Quaternion orientation456 = new Quaternion(0.0, 0.0, 0.704181506150831,0.7100200042218158);
      Vector3D size456 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box456 = new Box3D(position456, orientation456, size456);
      terrain.addRotatableBox(box456, YoAppearance.DarkGray());

      Point3D position457 = new Point3D(8.064675866921874, 3.3012450058603164, 0.046016306708178786);
      Quaternion orientation457 = new Quaternion(0.0, 0.0, 0.7060501535261512,0.7081618322853177);
      Vector3D size457 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box457 = new Box3D(position457, orientation457, size457);
      terrain.addRotatableBox(box457, YoAppearance.DarkGray());

      Point3D position458 = new Point3D(8.152449598918341, 3.5793016522023446, 0.047082419865977894);
      Quaternion orientation458 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size458 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box458 = new Box3D(position458, orientation458, size458);
      terrain.addRotatableBox(box458, YoAppearance.DarkGray());

      Point3D position459 = new Point3D(8.15561274022191, 3.766062394454181, 0.04851321195122305);
      Quaternion orientation459 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size459 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box459 = new Box3D(position459, orientation459, size459);
      terrain.addRotatableBox(box459, YoAppearance.DarkGray());

      Point3D position460 = new Point3D(8.06995087947249, 4.05333939223156, 0.04470427328368063);
      Quaternion orientation460 = new Quaternion(0.0, 0.0, 0.7089663975812704,0.7052422612837633);
      Vector3D size460 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box460 = new Box3D(position460, orientation460, size460);
      terrain.addRotatableBox(box460, YoAppearance.DarkGray());

      Point3D position461 = new Point3D(8.256712637978552, 4.055484487335707, 0.0465446546756541);
      Quaternion orientation461 = new Quaternion(0.0, 0.0, 0.705726540586363,0.7084843328627703);
      Vector3D size461 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box461 = new Box3D(position461, orientation461, size461);
      terrain.addRotatableBox(box461, YoAppearance.DarkGray());

      Point3D position462 = new Point3D(8.534695414460673, 3.197847729765506, 0.0436624434311707);
      Quaternion orientation462 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size462 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box462 = new Box3D(position462, orientation462, size462);
      terrain.addRotatableBox(box462, YoAppearance.DarkGray());

      Point3D position463 = new Point3D(8.540900973445865, 3.388671477024096, 0.04400585133181643);
      Quaternion orientation463 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size463 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box463 = new Box3D(position463, orientation463, size463);
      terrain.addRotatableBox(box463, YoAppearance.DarkGray());

      Point3D position464 = new Point3D(8.446166935149789, 3.6761343214655513, 0.04558588339207196);
      Quaternion orientation464 = new Quaternion(0.0, 0.0, 0.7016853567916603,0.7124869543115583);
      Vector3D size464 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box464 = new Box3D(position464, orientation464, size464);
      terrain.addRotatableBox(box464, YoAppearance.DarkGray());

      Point3D position465 = new Point3D(8.629100011417771, 3.678953161543596, 0.0401527710286485);
      Quaternion orientation465 = new Quaternion(0.0, 0.0, 0.7096844738128042,0.7045196573758914);
      Vector3D size465 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box465 = new Box3D(position465, orientation465, size465);
      terrain.addRotatableBox(box465, YoAppearance.DarkGray());

      Point3D position466 = new Point3D(8.544205652116847, 3.963513099370666, 0.04861356986695628);
      Quaternion orientation466 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size466 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box466 = new Box3D(position466, orientation466, size466);
      terrain.addRotatableBox(box466, YoAppearance.DarkGray());

      Point3D position467 = new Point3D(8.5399389891589, 4.155300276983753, 0.04766115185873509);
      Quaternion orientation467 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size467 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box467 = new Box3D(position467, orientation467, size467);
      terrain.addRotatableBox(box467, YoAppearance.DarkGray());

      Point3D position468 = new Point3D(2.9991922858880433, 4.977385061030978, 0.07749255480474931);
      Quaternion orientation468 = new Quaternion(3.591011281523848E-4, -4.404093410555292E-4, -0.7140274385984735,0.7001174858654047);
      Vector3D size468 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box468 = new Box3D(position468, orientation468, size468);
      terrain.addRotatableBox(box468, YoAppearance.DarkGray());

      Point3D position469 = new Point3D(4.010313695756378, 4.961605633180564, 0.07664394760851354);
      Quaternion orientation469 = new Quaternion(0.0010814627453136847, 0.0011373925186028025, -0.7128015132552289,0.7013640563057426);
      Vector3D size469 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box469 = new Box3D(position469, orientation469, size469);
      terrain.addRotatableBox(box469, YoAppearance.DarkGray());

      Point3D position470 = new Point3D(1.538532083708091, 5.47417019503108, 0.05131964350890132);
      Quaternion orientation470 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size470 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box470 = new Box3D(position470, orientation470, size470);
      terrain.addRotatableBox(box470, YoAppearance.DarkGray());

      Point3D position471 = new Point3D(1.5377122418466396, 5.287199518721233, 0.0453699739307727);
      Quaternion orientation471 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size471 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box471 = new Box3D(position471, orientation471, size471);
      terrain.addRotatableBox(box471, YoAppearance.DarkGray());

      Point3D position472 = new Point3D(1.8318418085847752, 5.383243229138638, 0.05026246658367628);
      Quaternion orientation472 = new Quaternion(0.0, 0.0, -0.7032798171527006,0.7109131443331625);
      Vector3D size472 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box472 = new Box3D(position472, orientation472, size472);
      terrain.addRotatableBox(box472, YoAppearance.DarkGray());

      Point3D position473 = new Point3D(2.0226812492006916, 5.381622095959591, 0.058535600491900285);
      Quaternion orientation473 = new Quaternion(0.0, 0.0, -0.7033244049950813,0.7108690324794821);
      Vector3D size473 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box473 = new Box3D(position473, orientation473, size473);
      terrain.addRotatableBox(box473, YoAppearance.DarkGray());

      Point3D position474 = new Point3D(1.9258075527472558, 5.091605878780427, 0.04914770991489481);
      Quaternion orientation474 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size474 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box474 = new Box3D(position474, orientation474, size474);
      terrain.addRotatableBox(box474, YoAppearance.DarkGray());

      Point3D position475 = new Point3D(1.9221267562656839, 4.897105461572966, 0.048374686104427383);
      Quaternion orientation475 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size475 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box475 = new Box3D(position475, orientation475, size475);
      terrain.addRotatableBox(box475, YoAppearance.DarkGray());

      Point3D position476 = new Point3D(1.6391139250203717, 5.000030288088172, 0.03909051303520233);
      Quaternion orientation476 = new Quaternion(4.2221998162823653E-4, -0.0021771612644779165, -0.7083346233855763,0.7058733193798512);
      Vector3D size476 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box476 = new Box3D(position476, orientation476, size476);
      terrain.addRotatableBox(box476, YoAppearance.DarkGray());

      Point3D position477 = new Point3D(1.4508258759747759, 4.991873785777104, 0.03772504674415519);
      Quaternion orientation477 = new Quaternion(0.0, 0.0, 0.7038510074598656,0.7103476327107258);
      Vector3D size477 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box477 = new Box3D(position477, orientation477, size477);
      terrain.addRotatableBox(box477, YoAppearance.DarkGray());

      Point3D position478 = new Point3D(1.544254053576254, 4.706315000748547, 0.032965432002729965);
      Quaternion orientation478 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size478 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box478 = new Box3D(position478, orientation478, size478);
      terrain.addRotatableBox(box478, YoAppearance.DarkGray());

      Point3D position479 = new Point3D(1.5421604510537665, 4.51523180468053, 0.03169062655032244);
      Quaternion orientation479 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size479 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box479 = new Box3D(position479, orientation479, size479);
      terrain.addRotatableBox(box479, YoAppearance.DarkGray());

      Point3D position480 = new Point3D(1.8302064340875628, 4.612274943312089, 0.03233411894842265);
      Quaternion orientation480 = new Quaternion(0.0, 0.0, -0.7041659007794283,0.7100354809300001);
      Vector3D size480 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box480 = new Box3D(position480, orientation480, size480);
      terrain.addRotatableBox(box480, YoAppearance.DarkGray());

      Point3D position481 = new Point3D(2.020076111592974, 4.605498161384824, 0.032951829300953354);
      Quaternion orientation481 = new Quaternion(0.0, 0.0, 0.7094730567086567,0.7047325604826807);
      Vector3D size481 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box481 = new Box3D(position481, orientation481, size481);
      terrain.addRotatableBox(box481, YoAppearance.DarkGray());

      Point3D position482 = new Point3D(2.3017726687557736, 5.46592118693114, 0.050732948560174566);
      Quaternion orientation482 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size482 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box482 = new Box3D(position482, orientation482, size482);
      terrain.addRotatableBox(box482, YoAppearance.DarkGray());

      Point3D position483 = new Point3D(2.3045613478518816, 5.27440611190311, 0.04678489901068887);
      Quaternion orientation483 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size483 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box483 = new Box3D(position483, orientation483, size483);
      terrain.addRotatableBox(box483, YoAppearance.DarkGray());

      Point3D position484 = new Point3D(2.214159178501207, 4.990925622627419, 0.04546863118006792);
      Quaternion orientation484 = new Quaternion(0.0, 0.0, -0.710045270209128,0.7041560297644597);
      Vector3D size484 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box484 = new Box3D(position484, orientation484, size484);
      terrain.addRotatableBox(box484, YoAppearance.DarkGray());

      Point3D position485 = new Point3D(2.3882625449800243, 4.985485705317335, 0.04358061639227621);
      Quaternion orientation485 = new Quaternion(0.0, 0.0, 0.7032305077721192,0.7109619208780226);
      Vector3D size485 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box485 = new Box3D(position485, orientation485, size485);
      terrain.addRotatableBox(box485, YoAppearance.DarkGray());

      Point3D position486 = new Point3D(2.305334951328886, 4.703308749301262, 0.04026317060808564);
      Quaternion orientation486 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size486 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box486 = new Box3D(position486, orientation486, size486);
      terrain.addRotatableBox(box486, YoAppearance.DarkGray());

      Point3D position487 = new Point3D(2.3035618801058884, 4.514827736541219, 0.043838376674458175);
      Quaternion orientation487 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size487 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box487 = new Box3D(position487, orientation487, size487);
      terrain.addRotatableBox(box487, YoAppearance.DarkGray());

      Point3D position488 = new Point3D(5.022827545101327, 4.943592721151784, 0.06929944023803199);
      Quaternion orientation488 = new Quaternion(0.0, 0.0, -0.7146326573404546,0.6994999392870028);
      Vector3D size488 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box488 = new Box3D(position488, orientation488, size488);
      terrain.addRotatableBox(box488, YoAppearance.DarkGray());

      Point3D position489 = new Point3D(6.0368497952581395, 4.903842144945642, 0.06172802598521781);
      Quaternion orientation489 = new Quaternion(0.005613854283208643, 0.0055820130552961455, -0.7143726400604131,0.6997206992103728);
      Vector3D size489 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box489 = new Box3D(position489, orientation489, size489);
      terrain.addRotatableBox(box489, YoAppearance.DarkGray());

      Point3D position490 = new Point3D(7.050523277840206, 4.882894636083961, 0.06864253875907342);
      Quaternion orientation490 = new Quaternion(0.0, 0.0, 0.7017961773791698,0.7123777968297335);
      Vector3D size490 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box490 = new Box3D(position490, orientation490, size490);
      terrain.addRotatableBox(box490, YoAppearance.DarkGray());

      Point3D position491 = new Point3D(5.023846234292196, 4.936137034749617, 0.22602295943394107);
      Quaternion orientation491 = new Quaternion(0.0, 0.0, 0.7009087265528491,0.71325097759629);
      Vector3D size491 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box491 = new Box3D(position491, orientation491, size491);
      terrain.addRotatableBox(box491, YoAppearance.DarkGray());

      Point3D position492 = new Point3D(2.6900015009577163, 5.443636601606902, 0.19963009876119422);
      Quaternion orientation492 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size492 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box492 = new Box3D(position492, orientation492, size492);
      terrain.addRotatableBox(box492, YoAppearance.DarkGray());

      Point3D position493 = new Point3D(2.6890564650088904, 5.254319503692475, 0.1998904919894186);
      Quaternion orientation493 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size493 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box493 = new Box3D(position493, orientation493, size493);
      terrain.addRotatableBox(box493, YoAppearance.DarkGray());

      Point3D position494 = new Point3D(2.7797616190377004, 4.964961870088415, 0.24403572659127876);
      Quaternion orientation494 = new Quaternion(0.09461018209558089, 0.09407245118837755, -0.7027793486796466,0.6987849987232575);
      Vector3D size494 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box494 = new Box3D(position494, orientation494, size494);
      terrain.addRotatableBox(box494, YoAppearance.DarkGray());

      Point3D position495 = new Point3D(2.593463551704851, 4.962190880492245, 0.24443600486584452);
      Quaternion orientation495 = new Quaternion(0.09234953713579733, -0.09290494477039353, 0.698908799916006,0.7031121700183992);
      Vector3D size495 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box495 = new Box3D(position495, orientation495, size495);
      terrain.addRotatableBox(box495, YoAppearance.DarkGray());

      Point3D position496 = new Point3D(2.684024192233144, 4.669381005242567, 0.19603696340587146);
      Quaternion orientation496 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size496 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box496 = new Box3D(position496, orientation496, size496);
      terrain.addRotatableBox(box496, YoAppearance.DarkGray());

      Point3D position497 = new Point3D(2.6860150703200567, 4.479548037610604, 0.19828815283388773);
      Quaternion orientation497 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size497 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box497 = new Box3D(position497, orientation497, size497);
      terrain.addRotatableBox(box497, YoAppearance.DarkGray());

      Point3D position498 = new Point3D(3.1711383025454465, 5.346218051440702, 0.28166428275520083);
      Quaternion orientation498 = new Quaternion(0.10793831101703633, 0.10894156349569274, -0.6955023910392171,0.7019668658965574);
      Vector3D size498 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box498 = new Box3D(position498, orientation498, size498);
      terrain.addRotatableBox(box498, YoAppearance.DarkGray());

      Point3D position499 = new Point3D(2.9818424101161205, 5.342476963468071, 0.28083800074621);
      Quaternion orientation499 = new Quaternion(0.11223315066187517, -0.11138798116147307, 0.7008445880063211,0.6955668918281909);
      Vector3D size499 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box499 = new Box3D(position499, orientation499, size499);
      terrain.addRotatableBox(box499, YoAppearance.DarkGray());

      Point3D position500 = new Point3D(3.0756668786094545, 5.057339884824034, 0.22430098450189007);
      Quaternion orientation500 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size500 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box500 = new Box3D(position500, orientation500, size500);
      terrain.addRotatableBox(box500, YoAppearance.DarkGray());

      Point3D position501 = new Point3D(3.0722024139657997, 4.867834661837208, 0.2251948202315323);
      Quaternion orientation501 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size501 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box501 = new Box3D(position501, orientation501, size501);
      terrain.addRotatableBox(box501, YoAppearance.DarkGray());

      Point3D position502 = new Point3D(3.165664362999164, 4.574777955911507, 0.27735751751742505);
      Quaternion orientation502 = new Quaternion(0.09437843324707647, 0.09404280191325493, -0.7020498469308203,0.6995531968124677);
      Vector3D size502 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box502 = new Box3D(position502, orientation502, size502);
      terrain.addRotatableBox(box502, YoAppearance.DarkGray());

      Point3D position503 = new Point3D(2.974840053678399, 4.574801242282441, 0.2788927721576176);
      Quaternion orientation503 = new Quaternion(0.09994070761295208, -0.09964072755297639, 0.7010812345776699,0.6989768829493477);
      Vector3D size503 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box503 = new Box3D(position503, orientation503, size503);
      terrain.addRotatableBox(box503, YoAppearance.DarkGray());

      Point3D position504 = new Point3D(3.359147869089035, 5.350879534175364, 0.22486106332834727);
      Quaternion orientation504 = new Quaternion(0.0, 0.0, -0.7071194347376097,0.7070941274090483);
      Vector3D size504 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box504 = new Box3D(position504, orientation504, size504);
      terrain.addRotatableBox(box504, YoAppearance.DarkGray());

      Point3D position505 = new Point3D(3.354472458529534, 4.957833976155043, 0.22395977136362658);
      Quaternion orientation505 = new Quaternion(0.0, 0.0, -0.7058785319428242,0.7083329006492944);
      Vector3D size505 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box505 = new Box3D(position505, orientation505, size505);
      terrain.addRotatableBox(box505, YoAppearance.DarkGray());

      Point3D position506 = new Point3D(3.350402623341776, 4.5730833666883965, 0.22640085739147484);
      Quaternion orientation506 = new Quaternion(0.0, 0.0, 0.7062242020231959,0.7079882601263245);
      Vector3D size506 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box506 = new Box3D(position506, orientation506, size506);
      terrain.addRotatableBox(box506, YoAppearance.DarkGray());

      Point3D position507 = new Point3D(3.68866762247556, 5.423626583484283, 0.3010889371484357);
      Quaternion orientation507 = new Quaternion(-5.839236425510448E-34, 0.14469060323807462, 8.859744206110364E-18,0.9894769473487506);
      Vector3D size507 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box507 = new Box3D(position507, orientation507, size507);
      terrain.addRotatableBox(box507, YoAppearance.DarkGray());

      Point3D position508 = new Point3D(3.7706107394082076, 4.951861846659785, 0.2940640116728258);
      Quaternion orientation508 = new Quaternion(-0.08864451472761907, -0.08908549481655237, -0.6997578049577696,0.7032388918587189);
      Vector3D size508 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box508 = new Box3D(position508, orientation508, size508);
      terrain.addRotatableBox(box508, YoAppearance.DarkGray());

      Point3D position509 = new Point3D(3.68965927337154, 5.238893424765486, 0.30028645012494665);
      Quaternion orientation509 = new Quaternion(1.9454724300576844E-34, 0.14138358070604393, 8.657247478182429E-18,0.989954889430189);
      Vector3D size509 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box509 = new Box3D(position509, orientation509, size509);
      terrain.addRotatableBox(box509, YoAppearance.DarkGray());

      Point3D position510 = new Point3D(3.588806285529547, 4.954840136541839, 0.2930951718708826);
      Quaternion orientation510 = new Quaternion(-0.08427871374180397, 0.09588259957957021, 0.7076885857849924,0.6948960289550722);
      Vector3D size510 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box510 = new Box3D(position510, orientation510, size510);
      terrain.addRotatableBox(box510, YoAppearance.DarkGray());

      Point3D position511 = new Point3D(3.6964906211913506, 4.656135506335926, 0.24588881335187482);
      Quaternion orientation511 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size511 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box511 = new Box3D(position511, orientation511, size511);
      terrain.addRotatableBox(box511, YoAppearance.DarkGray());

      Point3D position512 = new Point3D(3.6860421913290207, 4.466636291585176, 0.24832633958306163);
      Quaternion orientation512 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size512 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box512 = new Box3D(position512, orientation512, size512);
      terrain.addRotatableBox(box512, YoAppearance.DarkGray());

      Point3D position513 = new Point3D(3.9745492483528864, 4.563667786708171, 0.3000158195677145);
      Quaternion orientation513 = new Quaternion(0.09976532226249955, -0.09967911277827533, 0.7003421510403348,0.6997369694576615);
      Vector3D size513 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box513 = new Box3D(position513, orientation513, size513);
      terrain.addRotatableBox(box513, YoAppearance.DarkGray());

      Point3D position514 = new Point3D(4.1570571372942995, 5.335150072517718, 0.3009665879051755);
      Quaternion orientation514 = new Quaternion(0.0, 0.0, -0.7031379195773428,0.7110534902891948);
      Vector3D size514 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box514 = new Box3D(position514, orientation514, size514);
      terrain.addRotatableBox(box514, YoAppearance.DarkGray());

      Point3D position515 = new Point3D(3.976449906945254, 5.331340637186328, 0.30163369997832445);
      Quaternion orientation515 = new Quaternion(0.0, 0.0, 0.7053477339779695,0.7088614633143374);
      Vector3D size515 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box515 = new Box3D(position515, orientation515, size515);
      terrain.addRotatableBox(box515, YoAppearance.DarkGray());

      Point3D position516 = new Point3D(4.060087891099629, 5.039296422439849, 0.24882388075031026);
      Quaternion orientation516 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size516 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box516 = new Box3D(position516, orientation516, size516);
      terrain.addRotatableBox(box516, YoAppearance.DarkGray());

      Point3D position517 = new Point3D(4.063602710036821, 4.8525705083227395, 0.2470906830235654);
      Quaternion orientation517 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size517 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box517 = new Box3D(position517, orientation517, size517);
      terrain.addRotatableBox(box517, YoAppearance.DarkGray());

      Point3D position518 = new Point3D(4.147655172749554, 4.563400311900557, 0.2989374948751531);
      Quaternion orientation518 = new Quaternion(0.09713879505240461, 0.09719288147916494, -0.7002040662118151,0.7005939365609611);
      Vector3D size518 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box518 = new Box3D(position518, orientation518, size518);
      terrain.addRotatableBox(box518, YoAppearance.DarkGray());

      Point3D position519 = new Point3D(4.343356717444665, 4.5648819072177, 0.30128052384367776);
      Quaternion orientation519 = new Quaternion(0.0844889112032617, -0.11355828784903144, 0.6956075277336033,0.7043410441716627);
      Vector3D size519 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box519 = new Box3D(position519, orientation519, size519);
      terrain.addRotatableBox(box519, YoAppearance.DarkGray());

      Point3D position520 = new Point3D(4.349272116834419, 4.950336400544961, 0.23830329314841697);
      Quaternion orientation520 = new Quaternion(0.0, 0.0, -0.7115262234714076,0.7026595429598296);
      Vector3D size520 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box520 = new Box3D(position520, orientation520, size520);
      terrain.addRotatableBox(box520, YoAppearance.DarkGray());

      Point3D position521 = new Point3D(4.346481880050672, 5.326745674840748, 0.3000392880382346);
      Quaternion orientation521 = new Quaternion(0.0, 0.0, -0.7065132530735921,0.7076998115241875);
      Vector3D size521 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box521 = new Box3D(position521, orientation521, size521);
      terrain.addRotatableBox(box521, YoAppearance.DarkGray());

      Point3D position522 = new Point3D(4.702380013634424, 5.4079336148133885, 0.3797638118070186);
      Quaternion orientation522 = new Quaternion(1.6449470919030792E-33, 0.09792926395956664, 5.9964379825470255E-18,0.9951933778217867);
      Vector3D size522 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box522 = new Box3D(position522, orientation522, size522);
      terrain.addRotatableBox(box522, YoAppearance.DarkGray());

      Point3D position523 = new Point3D(4.703890280086529, 5.215596317765299, 0.37962892936597686);
      Quaternion orientation523 = new Quaternion(1.9358735945371405E-34, 0.10122575159006256, 6.1982896338026795E-18,0.9948634816973769);
      Vector3D size523 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box523 = new Box3D(position523, orientation523, size523);
      terrain.addRotatableBox(box523, YoAppearance.DarkGray());

      Point3D position524 = new Point3D(4.792888242703473, 4.924894208219748, 0.3479451232275179);
      Quaternion orientation524 = new Quaternion(0.0, 0.0, -0.7035865871495094,0.7106095372166812);
      Vector3D size524 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box524 = new Box3D(position524, orientation524, size524);
      terrain.addRotatableBox(box524, YoAppearance.DarkGray());

      Point3D position525 = new Point3D(4.610354981743812, 4.929117286057229, 0.3475132664494318);
      Quaternion orientation525 = new Quaternion(0.0, 0.0, -0.7132505655401289,0.7009091458646308);
      Vector3D size525 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box525 = new Box3D(position525, orientation525, size525);
      terrain.addRotatableBox(box525, YoAppearance.DarkGray());

      Point3D position526 = new Point3D(4.712970930242387, 4.639798910349343, 0.3912550730752458);
      Quaternion orientation526 = new Quaternion(-3.880765412410759E-34, -0.12182502599465175, -7.459631407019642E-18,0.9925515921308083);
      Vector3D size526 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box526 = new Box3D(position526, orientation526, size526);
      terrain.addRotatableBox(box526, YoAppearance.DarkGray());

      Point3D position527 = new Point3D(4.707914463288, 4.45139465722848, 0.3891170604695318);
      Quaternion orientation527 = new Quaternion(2.3291328065487197E-33, -0.1241412431733023, -7.601458804717881E-18,0.9922645573350823);
      Vector3D size527 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box527 = new Box3D(position527, orientation527, size527);
      terrain.addRotatableBox(box527, YoAppearance.DarkGray());

      Point3D position528 = new Point3D(5.1727760970934185, 5.311234034075083, 0.3967895082089883);
      Quaternion orientation528 = new Quaternion(0.09472131621407967, 0.09590058699963736, -0.6963057302396933,0.7049746660009598);
      Vector3D size528 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box528 = new Box3D(position528, orientation528, size528);
      terrain.addRotatableBox(box528, YoAppearance.DarkGray());

      Point3D position529 = new Point3D(4.986578998410523, 5.30457288301287, 0.3944322980633405);
      Quaternion orientation529 = new Quaternion(0.09774852819681804, -0.09770925509640942, 0.7004613865448716,0.700179957342037);
      Vector3D size529 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box529 = new Box3D(position529, orientation529, size529);
      terrain.addRotatableBox(box529, YoAppearance.DarkGray());

      Point3D position530 = new Point3D(5.181946498191193, 4.915938621233747, 0.3935301841017318);
      Quaternion orientation530 = new Quaternion(-0.09454705714016172, 0.09446453379644658, 0.7010687973196257,0.7004568846585553);
      Vector3D size530 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box530 = new Box3D(position530, orientation530, size530);
      terrain.addRotatableBox(box530, YoAppearance.DarkGray());

      Point3D position531 = new Point3D(4.999908472643262, 4.9227122732285915, 0.3936477908269693);
      Quaternion orientation531 = new Quaternion(-0.0943438948778358, -0.0947342877428095, -0.6993100325946019,0.7022037614095209);
      Vector3D size531 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box531 = new Box3D(position531, orientation531, size531);
      terrain.addRotatableBox(box531, YoAppearance.DarkGray());

      Point3D position532 = new Point3D(5.181134977089534, 4.535174520419811, 0.34319902751110637);
      Quaternion orientation532 = new Quaternion(0.0, 0.0, -0.7087044927674649,0.7055054513830564);
      Vector3D size532 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box532 = new Box3D(position532, orientation532, size532);
      terrain.addRotatableBox(box532, YoAppearance.DarkGray());

      Point3D position533 = new Point3D(5.001168160091194, 4.531026689368033, 0.3442518243348339);
      Quaternion orientation533 = new Quaternion(0.0, 0.0, 0.7085911489028207,0.7056192909045079);
      Vector3D size533 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box533 = new Box3D(position533, orientation533, size533);
      terrain.addRotatableBox(box533, YoAppearance.DarkGray());

      Point3D position534 = new Point3D(5.361423516842161, 4.537318361252479, 0.3467424072643232);
      Quaternion orientation534 = new Quaternion(0.0, 0.0, -0.702830138774019,0.7113577131309489);
      Vector3D size534 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box534 = new Box3D(position534, orientation534, size534);
      terrain.addRotatableBox(box534, YoAppearance.DarkGray());

      Point3D position535 = new Point3D(5.368768670021752, 4.91715906709865, 0.38910279983955076);
      Quaternion orientation535 = new Quaternion(-0.09458307226381174, -0.09416431082175267, -0.7023337283279424,0.6992241836933095);
      Vector3D size535 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box535 = new Box3D(position535, orientation535, size535);
      terrain.addRotatableBox(box535, YoAppearance.DarkGray());

      Point3D position536 = new Point3D(5.359430458701423, 5.307460494413138, 0.3951426086328146);
      Quaternion orientation536 = new Quaternion(0.09529196339253196, -0.09533170427575388, 0.7005076469963307,0.7007997890784077);
      Vector3D size536 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box536 = new Box3D(position536, orientation536, size536);
      terrain.addRotatableBox(box536, YoAppearance.DarkGray());

      Point3D position537 = new Point3D(5.75258973537161, 4.449509218011508, 0.227418511493009);
      Quaternion orientation537 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size537 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box537 = new Box3D(position537, orientation537, size537);
      terrain.addRotatableBox(box537, YoAppearance.DarkGray());

      Point3D position538 = new Point3D(5.753494342409687, 4.631547818499434, 0.2278999120441889);
      Quaternion orientation538 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size538 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box538 = new Box3D(position538, orientation538, size538);
      terrain.addRotatableBox(box538, YoAppearance.DarkGray());

      Point3D position539 = new Point3D(5.648721454782096, 4.922749962848247, 0.27732476658080424);
      Quaternion orientation539 = new Quaternion(0.0, 0.0, -0.7107650931702618,0.7034294437473235);
      Vector3D size539 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box539 = new Box3D(position539, orientation539, size539);
      terrain.addRotatableBox(box539, YoAppearance.DarkGray());

      Point3D position540 = new Point3D(5.840403719567024, 4.924951038535571, 0.27650382371337645);
      Quaternion orientation540 = new Quaternion(0.0, 0.0, 0.7053417880043341,0.7088673797649665);
      Vector3D size540 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box540 = new Box3D(position540, orientation540, size540);
      terrain.addRotatableBox(box540, YoAppearance.DarkGray());

      Point3D position541 = new Point3D(5.7577226491388185, 5.211636291956968, 0.28389741291306736);
      Quaternion orientation541 = new Quaternion(0.0, 0.11262021465784755, 6.89599927000101E-18,0.993638106782555);
      Vector3D size541 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box541 = new Box3D(position541, orientation541, size541);
      terrain.addRotatableBox(box541, YoAppearance.DarkGray());

      Point3D position542 = new Point3D(5.764448795159363, 5.403415509695048, 0.28467265927325774);
      Quaternion orientation542 = new Quaternion(-1.0658594972039655E-33, 0.11109613212788319, 6.802676130403094E-18,0.9938096645868483);
      Vector3D size542 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box542 = new Box3D(position542, orientation542, size542);
      terrain.addRotatableBox(box542, YoAppearance.DarkGray());

      Point3D position543 = new Point3D(6.2352107198329065, 4.5411938845186395, 0.2684214567166878);
      Quaternion orientation543 = new Quaternion(-0.09099327976522753, 0.09151827134292549, 0.6991736284516058,0.7032075556518044);
      Vector3D size543 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box543 = new Box3D(position543, orientation543, size543);
      terrain.addRotatableBox(box543, YoAppearance.DarkGray());

      Point3D position544 = new Point3D(6.041424771298938, 4.5382024233593405, 0.2701155633865791);
      Quaternion orientation544 = new Quaternion(-0.09049938070702158, -0.09029400308019075, -0.7021010186557202,0.7005076835424435);
      Vector3D size544 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box544 = new Box3D(position544, orientation544, size544);
      terrain.addRotatableBox(box544, YoAppearance.DarkGray());

      Point3D position545 = new Point3D(6.124353907177331, 5.01270454422082, 0.23404920128215498);
      Quaternion orientation545 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size545 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box545 = new Box3D(position545, orientation545, size545);
      terrain.addRotatableBox(box545, YoAppearance.DarkGray());

      Point3D position546 = new Point3D(6.421349657210157, 4.534198877945789, 0.26733468034992436);
      Quaternion orientation546 = new Quaternion(-0.09224861237772161, 0.09388297766102086, 0.6947734684371486,0.7070827445042763);
      Vector3D size546 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box546 = new Box3D(position546, orientation546, size546);
      terrain.addRotatableBox(box546, YoAppearance.DarkGray());

      Point3D position547 = new Point3D(6.2386546198104, 5.301919693850186, 0.2618318096377476);
      Quaternion orientation547 = new Quaternion(-0.0924918376852269, 0.09412884116858322, 0.6947475169322426,0.7070437814862335);
      Vector3D size547 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box547 = new Box3D(position547, orientation547, size547);
      terrain.addRotatableBox(box547, YoAppearance.DarkGray());

      Point3D position548 = new Point3D(6.051805117872733, 5.307538004269914, 0.26005353871268083);
      Quaternion orientation548 = new Quaternion(-0.09694408237307763, 0.08981925984154682, 0.7007993258860914,0.7010097362319289);
      Vector3D size548 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box548 = new Box3D(position548, orientation548, size548);
      terrain.addRotatableBox(box548, YoAppearance.DarkGray());

      Point3D position549 = new Point3D(6.122908361470702, 4.820253416476654, 0.2312384964958033);
      Quaternion orientation549 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size549 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box549 = new Box3D(position549, orientation549, size549);
      terrain.addRotatableBox(box549, YoAppearance.DarkGray());

      Point3D position550 = new Point3D(6.409268200039842, 4.9193412872830296, 0.23604260025687943);
      Quaternion orientation550 = new Quaternion(0.0, 0.0, 0.7016890425161331,0.7124833244454164);
      Vector3D size550 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box550 = new Box3D(position550, orientation550, size550);
      terrain.addRotatableBox(box550, YoAppearance.DarkGray());

      Point3D position551 = new Point3D(6.429966481857306, 5.300996096184173, 0.26300251335578917);
      Quaternion orientation551 = new Quaternion(-0.09163052675672367, 0.092769864463877, 0.6967248198275796,0.7053879246555514);
      Vector3D size551 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box551 = new Box3D(position551, orientation551, size551);
      terrain.addRotatableBox(box551, YoAppearance.DarkGray());

      Point3D position552 = new Point3D(6.699741918264115, 4.436843063453915, 0.2550180515425791);
      Quaternion orientation552 = new Quaternion(1.7479183742545392E-33, 0.1288981738241679, 7.892736799485345E-18,0.9916578345300332);
      Vector3D size552 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box552 = new Box3D(position552, orientation552, size552);
      terrain.addRotatableBox(box552, YoAppearance.DarkGray());

      Point3D position553 = new Point3D(6.698630441512664, 4.624966891239924, 0.2526101274005956);
      Quaternion orientation553 = new Quaternion(-3.884427274751654E-34, 0.1292203005277809, 7.912461371310276E-18,0.9916159104872764);
      Vector3D size553 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box553 = new Box3D(position553, orientation553, size553);
      terrain.addRotatableBox(box553, YoAppearance.DarkGray());

      Point3D position554 = new Point3D(6.797501566111999, 4.912343358516751, 0.2633885198693278);
      Quaternion orientation554 = new Quaternion(0.09819637521077755, -0.10061717351576993, 0.691507811143679,0.708555293125695);
      Vector3D size554 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box554 = new Box3D(position554, orientation554, size554);
      terrain.addRotatableBox(box554, YoAppearance.DarkGray());

      Point3D position555 = new Point3D(6.61320792222252, 4.9110518380849335, 0.2609523583831561);
      Quaternion orientation555 = new Quaternion(0.09905634457071809, -0.09993018618950954, 0.6950116745649737,0.7050961428818118);
      Vector3D size555 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box555 = new Box3D(position555, orientation555, size555);
      terrain.addRotatableBox(box555, YoAppearance.DarkGray());

      Point3D position556 = new Point3D(6.7127304242037615, 5.203850171586372, 0.21934622785676056);
      Quaternion orientation556 = new Quaternion(0.0, 0.0, -0.0038144316225337672,0.9999927250292359);
      Vector3D size556 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box556 = new Box3D(position556, orientation556, size556);
      terrain.addRotatableBox(box556, YoAppearance.DarkGray());

      Point3D position557 = new Point3D(6.718241475052683, 5.396187721186475, 0.214411381592355);
      Quaternion orientation557 = new Quaternion(0.0, 0.0, -0.009312552251322237,0.999956637245119);
      Vector3D size557 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box557 = new Box3D(position557, orientation557, size557);
      terrain.addRotatableBox(box557, YoAppearance.DarkGray());

      Point3D position558 = new Point3D(6.983093459558476, 4.519949020961721, 0.21557870456215963);
      Quaternion orientation558 = new Quaternion(0.0, 0.0, 0.7030358342720265,0.7111544246712073);
      Vector3D size558 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box558 = new Box3D(position558, orientation558, size558);
      terrain.addRotatableBox(box558, YoAppearance.DarkGray());

      Point3D position559 = new Point3D(6.988683516017568, 4.910999790530698, 0.25958187507798225);
      Quaternion orientation559 = new Quaternion(0.10036878834372606, -0.10248996160407609, 0.6924383325630104,0.7070721813889761);
      Vector3D size559 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box559 = new Box3D(position559, orientation559, size559);
      terrain.addRotatableBox(box559, YoAppearance.DarkGray());

      Point3D position560 = new Point3D(7.00489079733141, 5.3035039836470865, 0.2167604197386731);
      Quaternion orientation560 = new Quaternion(0.0, 0.0, 0.7019672015374099,0.7122092725917976);
      Vector3D size560 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box560 = new Box3D(position560, orientation560, size560);
      terrain.addRotatableBox(box560, YoAppearance.DarkGray());

      Point3D position561 = new Point3D(7.345613673095441, 4.43781818412057, 0.18851614172025807);
      Quaternion orientation561 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size561 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box561 = new Box3D(position561, orientation561, size561);
      terrain.addRotatableBox(box561, YoAppearance.DarkGray());

      Point3D position562 = new Point3D(7.346728260451079, 4.623825757305177, 0.19148708763994512);
      Quaternion orientation562 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size562 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box562 = new Box3D(position562, orientation562, size562);
      terrain.addRotatableBox(box562, YoAppearance.DarkGray());

      Point3D position563 = new Point3D(7.256713781322195, 4.917469336215263, 0.19034824545194998);
      Quaternion orientation563 = new Quaternion(0.0, 0.0, -0.7109847545862554,0.7032074222773266);
      Vector3D size563 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box563 = new Box3D(position563, orientation563, size563);
      terrain.addRotatableBox(box563, YoAppearance.DarkGray());

      Point3D position564 = new Point3D(7.436520392537069, 4.911634879318802, 0.18978383802880525);
      Quaternion orientation564 = new Quaternion(0.0, 0.0, 0.7079054853446147,0.7063071738408194);
      Vector3D size564 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box564 = new Box3D(position564, orientation564, size564);
      terrain.addRotatableBox(box564, YoAppearance.DarkGray());

      Point3D position565 = new Point3D(7.356717455086338, 5.20091839642546, 0.22230115969485045);
      Quaternion orientation565 = new Quaternion(0.0, 0.11441875733241588, 7.006128246478086E-18,0.9934326086708176);
      Vector3D size565 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box565 = new Box3D(position565, orientation565, size565);
      terrain.addRotatableBox(box565, YoAppearance.DarkGray());

      Point3D position566 = new Point3D(7.3566302205035035, 5.3878093138740475, 0.22224355113151553);
      Quaternion orientation566 = new Quaternion(-1.5505289284134202E-33, 0.11216790604418489, 6.8683033552035814E-18,0.9936892677561043);
      Vector3D size566 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box566 = new Box3D(position566, orientation566, size566);
      terrain.addRotatableBox(box566, YoAppearance.DarkGray());

      Point3D position567 = new Point3D(7.770134742471376, 4.399447356795464, 0.047256945955076055);
      Quaternion orientation567 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size567 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box567 = new Box3D(position567, orientation567, size567);
      terrain.addRotatableBox(box567, YoAppearance.DarkGray());

      Point3D position568 = new Point3D(7.770217973789706, 4.586239241109061, 0.04430979308496823);
      Quaternion orientation568 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size568 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box568 = new Box3D(position568, orientation568, size568);
      terrain.addRotatableBox(box568, YoAppearance.DarkGray());

      Point3D position569 = new Point3D(7.869196761927994, 4.875940854721916, 0.04358384009610045);
      Quaternion orientation569 = new Quaternion(0.0, 0.0, 0.703468250129445,0.7107266852031212);
      Vector3D size569 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box569 = new Box3D(position569, orientation569, size569);
      terrain.addRotatableBox(box569, YoAppearance.DarkGray());

      Point3D position570 = new Point3D(7.682970194739092, 4.877052570111654, 0.047271829008987146);
      Quaternion orientation570 = new Quaternion(0.0, 0.0, 0.7034450413479783,0.7107496562102167);
      Vector3D size570 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box570 = new Box3D(position570, orientation570, size570);
      terrain.addRotatableBox(box570, YoAppearance.DarkGray());

      Point3D position571 = new Point3D(7.779256902384008, 5.16599193363947, 0.04432955446553308);
      Quaternion orientation571 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size571 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box571 = new Box3D(position571, orientation571, size571);
      terrain.addRotatableBox(box571, YoAppearance.DarkGray());

      Point3D position572 = new Point3D(7.785783227158354, 5.355705473404653, 0.045068700704124554);
      Quaternion orientation572 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size572 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box572 = new Box3D(position572, orientation572, size572);
      terrain.addRotatableBox(box572, YoAppearance.DarkGray());

      Point3D position573 = new Point3D(8.252153182891886, 4.494370688035923, 0.04285320203865197);
      Quaternion orientation573 = new Quaternion(0.0, 0.0, 0.704181506150831,0.7100200042218158);
      Vector3D size573 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box573 = new Box3D(position573, orientation573, size573);
      terrain.addRotatableBox(box573, YoAppearance.DarkGray());

      Point3D position574 = new Point3D(8.064675866921874, 4.501245005860317, 0.046016306708178786);
      Quaternion orientation574 = new Quaternion(0.0, 0.0, 0.7060501535261512,0.7081618322853177);
      Vector3D size574 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box574 = new Box3D(position574, orientation574, size574);
      terrain.addRotatableBox(box574, YoAppearance.DarkGray());

      Point3D position575 = new Point3D(8.152449598918341, 4.779301652202345, 0.047082419865977894);
      Quaternion orientation575 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size575 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box575 = new Box3D(position575, orientation575, size575);
      terrain.addRotatableBox(box575, YoAppearance.DarkGray());

      Point3D position576 = new Point3D(8.15561274022191, 4.966062394454181, 0.04851321195122305);
      Quaternion orientation576 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size576 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box576 = new Box3D(position576, orientation576, size576);
      terrain.addRotatableBox(box576, YoAppearance.DarkGray());

      Point3D position577 = new Point3D(8.06995087947249, 5.2533393922315605, 0.04470427328368063);
      Quaternion orientation577 = new Quaternion(0.0, 0.0, 0.7089663975812704,0.7052422612837633);
      Vector3D size577 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box577 = new Box3D(position577, orientation577, size577);
      terrain.addRotatableBox(box577, YoAppearance.DarkGray());

      Point3D position578 = new Point3D(8.256712637978552, 5.255484487335707, 0.0465446546756541);
      Quaternion orientation578 = new Quaternion(0.0, 0.0, 0.705726540586363,0.7084843328627703);
      Vector3D size578 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box578 = new Box3D(position578, orientation578, size578);
      terrain.addRotatableBox(box578, YoAppearance.DarkGray());

      Point3D position579 = new Point3D(8.534695414460673, 4.397847729765506, 0.0436624434311707);
      Quaternion orientation579 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size579 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box579 = new Box3D(position579, orientation579, size579);
      terrain.addRotatableBox(box579, YoAppearance.DarkGray());

      Point3D position580 = new Point3D(8.540900973445865, 4.588671477024096, 0.04400585133181643);
      Quaternion orientation580 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size580 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box580 = new Box3D(position580, orientation580, size580);
      terrain.addRotatableBox(box580, YoAppearance.DarkGray());

      Point3D position581 = new Point3D(8.446166935149789, 4.8761343214655515, 0.04558588339207196);
      Quaternion orientation581 = new Quaternion(0.0, 0.0, 0.7016853567916603,0.7124869543115583);
      Vector3D size581 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box581 = new Box3D(position581, orientation581, size581);
      terrain.addRotatableBox(box581, YoAppearance.DarkGray());

      Point3D position582 = new Point3D(8.629100011417771, 4.878953161543596, 0.0401527710286485);
      Quaternion orientation582 = new Quaternion(0.0, 0.0, 0.7096844738128042,0.7045196573758914);
      Vector3D size582 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box582 = new Box3D(position582, orientation582, size582);
      terrain.addRotatableBox(box582, YoAppearance.DarkGray());

      Point3D position583 = new Point3D(8.544205652116847, 5.163513099370666, 0.04861356986695628);
      Quaternion orientation583 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size583 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box583 = new Box3D(position583, orientation583, size583);
      terrain.addRotatableBox(box583, YoAppearance.DarkGray());

      Point3D position584 = new Point3D(8.5399389891589, 5.355300276983753, 0.04766115185873509);
      Quaternion orientation584 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size584 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box584 = new Box3D(position584, orientation584, size584);
      terrain.addRotatableBox(box584, YoAppearance.DarkGray());
   }
}
