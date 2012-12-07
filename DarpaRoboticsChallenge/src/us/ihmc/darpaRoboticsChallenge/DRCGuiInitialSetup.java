package us.ihmc.darpaRoboticsChallenge;

import javax.swing.JButton;

import us.ihmc.graphics3DAdapter.Graphics3DAdapter;
import us.ihmc.graphics3DAdapter.camera.CameraConfiguration;
import us.ihmc.graphics3DAdapter.jme.JMEGraphics3dAdapter;
import us.ihmc.projectM.R2Sim02.DataExporter;
import us.ihmc.projectM.R2Sim02.R2Parameters;
import us.ihmc.projectM.R2Sim02.R2Robot;

import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;

public class DRCGuiInitialSetup
{
   private Graphics3DAdapter graphics3dAdapter = new JMEGraphics3dAdapter();

   public void initializeGUI(SimulationConstructionSet scs, Robot robot)
   {
      setUpGUI(scs);

      if (R2Parameters.USE_SLIDER_BOARD)
         setupSliderBoard(scs);

      CameraConfiguration behindPelvis = new CameraConfiguration("BehindPelvis");
      behindPelvis.setCameraTracking(false, true, true, false);
      behindPelvis.setCameraDolly(false, true, true, false);
      behindPelvis.setCameraFix(0.0, 0.0, 1.0);
      behindPelvis.setCameraPosition(-2.5, 0.0, 1.0);
      behindPelvis.setCameraTrackingVars("q_x", "q_y", "q_z");
      scs.setupCamera(behindPelvis);

      CameraConfiguration camera5 = new CameraConfiguration("HMD_LeftEye");
      camera5.setCameraMount("leftEye");
      scs.setupCamera(camera5);
      CameraConfiguration camera6 = new CameraConfiguration("HMD_RightEye");
      camera6.setCameraMount("rightEye");
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

   public Graphics3DAdapter getGraphics3dAdapter()
   {
      return graphics3dAdapter;
   }

   public void setGraphics3dAdapter(Graphics3DAdapter graphics3dAdapter)
   {
      this.graphics3dAdapter = graphics3dAdapter;
   }
}
