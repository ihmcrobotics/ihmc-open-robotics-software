package us.ihmc.darpaRoboticsChallenge;

import javax.swing.JButton;

import us.ihmc.graphics3DAdapter.Graphics3DAdapter;
import us.ihmc.graphics3DAdapter.camera.CameraConfiguration;
import us.ihmc.graphics3DAdapter.jme.JMEGraphics3dAdapter;
import us.ihmc.projectM.R2Sim02.DataExporter;
import us.ihmc.projectM.R2Sim02.R2Robot;

import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.SupportedGraphics3DAdapter;

public class DRCGuiInitialSetup
{
   private SupportedGraphics3DAdapter graphics3dAdapter = SupportedGraphics3DAdapter.JAVA_MONKEY_ENGINE;

   public void initializeGUI(SimulationConstructionSet scs, Robot robot)
   {
      setUpGUI(scs);

      setupSliderBoard(scs);

      CameraConfiguration behindPelvis = new CameraConfiguration("BehindPelvis");
      behindPelvis.setCameraTracking(false, true, true, false);
      behindPelvis.setCameraDolly(false, true, true, false);
      behindPelvis.setCameraFix(0.0, 0.0, 1.0);
      behindPelvis.setCameraPosition(-2.5, 0.0, 1.0);
      behindPelvis.setCameraTrackingVars("q_x", "q_y", "q_z");
      scs.setupCamera(behindPelvis);

      CameraConfiguration camera5 = new CameraConfiguration("left_camera_sensor");
      camera5.setCameraMount("left_camera_sensor");
      scs.setupCamera(camera5);
      CameraConfiguration camera6 = new CameraConfiguration("right_camera_sensor");
      camera6.setCameraMount("right_camera_sensor");
      scs.setupCamera(camera6);
   }

   private void setupSliderBoard(SimulationConstructionSet scs)
   {
      //    EvolutionUC33ESetup setup = new EvolutionUC33ESetup(scs);
      //    setup.setupEvolutionForDrivingUpperBodyDesiredPosition();
   }

   private void setUpGUI(SimulationConstructionSet scs)
   {
   }

   public void initializeGUI(SimulationConstructionSet scs, R2Robot robot)
   {
      JButton exportTorqueAndSpeedButton = new JButton("Export Torque And Speed");
      DataExporter dataExporter = new DataExporter(scs, robot);
      exportTorqueAndSpeedButton.addActionListener(dataExporter);
      scs.addButton(exportTorqueAndSpeedButton);
   }

   public SupportedGraphics3DAdapter getGraphics3dAdapter()
   {
      return graphics3dAdapter;
   }

   public void setGraphics3dAdapter(SupportedGraphics3DAdapter graphics3dAdapter)
   {
      this.graphics3dAdapter = graphics3dAdapter;
   }
}
