package us.ihmc.darpaRoboticsChallenge;

import com.yobotics.simulationconstructionset.DynamicIntegrationMethod;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.util.LinearGroundContactModel;
import com.yobotics.simulationconstructionset.util.ground.TerrainObject;

import us.ihmc.commonAvatarInterfaces.CommonAvatarEnvironmentInterface;
import us.ihmc.commonWalkingControlModules.terrain.CommonTerrain;
import us.ihmc.graphics3DAdapter.GroundProfile;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.jme.JMEGraphics3dAdapter;

public class DRCDemo01EnvironmentViewer
{
   private final boolean SHOWGRAPHICS = true;//false;//
   private final boolean SHOWCOLLISIONTERRAIN = false;//true;
   
   
   Robot robot = new Robot("NotARobot");
   private double gravity = -9.81;
   CommonTerrain commonTerrain;
   
   
   DRCDemo01EnvironmentViewer()
   {
      double simulateDT = .001;
      int initialBufferSize = 16342;

      CommonAvatarEnvironmentInterface commonAvatarEnvironmentInterface = new DRCDemo01NavigationEnvironment();
//      Link staticLink=new Link("StaticLink");
//      robot.addStaticLink(staticLink);
//      robot.addStaticLinkGraphics(commonAvatarEnvironmentInterface.getTerrainObject().getLinkGraphics());
      
      GroundProfile groundProfile = commonAvatarEnvironmentInterface.getTerrainObject();
      commonTerrain = new CommonTerrain(groundProfile);

      
      robot.setGravity(gravity);

    double groundKz = 500;
    double groundBz = 300.0;
    double groundKxy = 50000.0;
    double groundBxy = 2000.0;

    LinearGroundContactModel groundContactModel = new LinearGroundContactModel(robot, groundKxy, groundBxy, groundKz, groundBz,
          robot.getRobotsYoVariableRegistry());

    groundContactModel.setGroundProfile(commonTerrain.getGroundProfile());

    robot.setGroundContactModel(groundContactModel);
    robot.setDynamicIntegrationMethod(DynamicIntegrationMethod.EULER_DOUBLE_STEPS);
//      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(false, false);
       JMEGraphics3dAdapter graphicsAdapter=new JMEGraphics3dAdapter();
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, graphicsAdapter, initialBufferSize);

      scs.setDT(simulateDT, 16342);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(linkGraphics);
      
//      scs.setCameraPosition(6.0, -2.0, 4.5);
//      scs.setCameraFix(-0.44, -0.17, 0.75);
//      scs.setCameraPosition(-11.5, -2,4.8);
//      scs.setCameraFix(3.5,1.5,0);
//      scs.setCameraPosition(-3, -7, 3);
//      scs.setCameraFix(4,4,0);
      //look at end of walking obstacle course
      scs.setCameraFix(17,17,0);
      scs.setCameraPosition(10, 24, 20);
      
      //look at trials qualification course
      scs.setCameraFix(5,-5,0);
      scs.setCameraPosition(0, -10, 20);
      


//      scs.setCameraTracking(false, true, true, false);
//      scs.setCameraDolly(false, true, true, false);

//      scs.setGroundAppearance(YoAppearance.EarthTexture());
//      scs.setGroundAppearance(YoAppearance.Blue());
      
      
      scs.setGroundVisible(SHOWCOLLISIONTERRAIN);

      if (SHOWGRAPHICS)
      {
         TerrainObject environmentTerrain = commonAvatarEnvironmentInterface.getTerrainObject();
         scs.addStaticLinkGraphics(environmentTerrain.getLinkGraphics());
      }

      Thread simThread = new Thread(scs, "SCS simulation thread");
     
    
    
    simThread.start();
     
   }
   
   
   
   public static void main(String[] args)
   {
      new DRCDemo01EnvironmentViewer();
   }

}
