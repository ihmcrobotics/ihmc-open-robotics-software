package us.ihmc.darpaRoboticsChallenge;

import java.awt.Component;

import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.automaticSimulationRunner.AutomaticSimulationRunner;
import us.ihmc.commonWalkingControlModules.controllers.ControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.FlatGroundWalkingHighLevelHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.terrain.TerrainType;
import us.ihmc.darpaRoboticsChallenge.controllers.DRCRobotMomentumBasedControllerFactory;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.initialSetup.SquaredUpDRCRobotInitialSetup;
import us.ihmc.graphics3DAdapter.graphics.LinkGraphics;
import us.ihmc.utilities.math.geometry.FramePoint;

import com.martiansoftware.jsap.JSAPException;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicCheckBoxMenuItem;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.inputdevices.MidiSliderBoard;

public class DRCFlatGroundWalkingTrack
{
   private final DRCSimulation drcSimulation;
   private final DRCDemo01Environment environment;
   private final DRCDemo0SelectedListener selectedListener;

   public DRCFlatGroundWalkingTrack(DRCGuiInitialSetup guiInitialSetup, AutomaticSimulationRunner automaticSimulationRunner, double timePerRecordTick,
                   int simulationDataBufferSize, boolean doChestOrientationControl, boolean showInstructions)
   {
      DRCSCSInitialSetup scsInitialSetup;
      DRCRobotInitialSetup drcRobotInitialSetup;

      drcRobotInitialSetup = new SquaredUpDRCRobotInitialSetup();
      
      
      
      scsInitialSetup = new DRCSCSInitialSetup(TerrainType.FLAT);
      scsInitialSetup.setSimulationDataBufferSize(simulationDataBufferSize);

      double dt = scsInitialSetup.getDT();
      int recordFrequency = (int) Math.round(timePerRecordTick / dt);
      if (recordFrequency < 1)
         recordFrequency = 1;
      scsInitialSetup.setRecordFrequency(recordFrequency);



      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();
      YoVariableRegistry registry = new YoVariableRegistry("adjustableParabolicTrajectoryDemoSimRegistry");
      selectedListener = new DRCDemo0SelectedListener(dynamicGraphicObjectsListRegistry, registry);

      double desiredCoMHeight = 0.9;
      HighLevelHumanoidControllerFactory highLevelHumanoidControllerFactory = new FlatGroundWalkingHighLevelHumanoidControllerFactory(desiredCoMHeight);
      ControllerFactory controllerFactory = new DRCRobotMomentumBasedControllerFactory(highLevelHumanoidControllerFactory);

      environment = new DRCDemo01Environment(dynamicGraphicObjectsListRegistry);
      environment.addSelectableListenerToSelectables(selectedListener);

//      r2Simulation = new R2Simulation(environment, r2InitialSetup, sensorNoiseInitialSetup, controllerFactory, scsInitialSetup, guiInitialSetup);
      drcSimulation = new DRCSimulation(drcRobotInitialSetup, controllerFactory, scsInitialSetup, guiInitialSetup);

      // attach slider board for apex control
//    final DoubleYoVariable desiredApex = new DoubleYoVariable("apexValue", registry);
//    desiredApex.set(1.5);
//    desiredApex.addVariableChangedListener(new VariableChangedListener()
//    {
//       public void variableChanged(YoVariable variable)
//       {
//          if (variable == desiredApex)
//          {
//             selectedListener.setApex(desiredApex.getDoubleValue());
//          }
//       }
//    });

      SimulationConstructionSet simulationConstructionSet = drcSimulation.getSimulationConstructionSet();
      MidiSliderBoard sliderBoard = new MidiSliderBoard(simulationConstructionSet);
      int i = 1;

      // TODO: get these from CommonAvatarUserInterface once it exists:
      sliderBoard.setSlider(i++, "desiredICPParameterX", getSimulationConstructionSet(), 0.0, 1.0);
      sliderBoard.setSlider(i++, "desiredICPParameterY", getSimulationConstructionSet(), 0.0, 1.0);
      sliderBoard.setSlider(i++, "desiredHeadingFinal", getSimulationConstructionSet(), Math.toRadians(-30.0), Math.toRadians(30.0));
      sliderBoard.setSlider(i++, "desiredPelvisPitch", getSimulationConstructionSet(), Math.toRadians(-20.0), Math.toRadians(20.0));
      sliderBoard.setSlider(i++, "desiredPelvisRoll", getSimulationConstructionSet(), Math.toRadians(-20.0), Math.toRadians(20.0));
      sliderBoard.setSlider(i++, "desiredCenterOfMassHeightFinal", getSimulationConstructionSet(), 0.42, 1.5);

      setUpJoyStick(getSimulationConstructionSet());

      // add other registries
      drcSimulation.addAdditionalDynamicGraphicObjectsListRegistries(dynamicGraphicObjectsListRegistry);
      drcSimulation.addAdditionalYoVariableRegistriesToSCS(registry);

      // attach release listener
      simulationConstructionSet.attachClicked3DPointForReleaseListener(selectedListener);
      setUpDemoButtons(simulationConstructionSet);

      simulationConstructionSet.setCameraPosition(6.0, -2.0, 4.5);
      simulationConstructionSet.setCameraFix(-0.44, -0.17, 0.75);
      hideDynamicGraphicsForActualDemo(showInstructions);

      setUpInstructionsBoard(simulationConstructionSet, showInstructions);

      if (automaticSimulationRunner != null)
      {
         drcSimulation.start(automaticSimulationRunner);
      }
      else
      {
         drcSimulation.start(null);
      }
   }

   private void hideDynamicGraphicsForActualDemo(boolean showInstructions)
   {
      if (showInstructions)
      {
         Component[] components = drcSimulation.getSimulationConstructionSet().getDynamicGraphicMenuManager().getjMenu().getMenuComponents();
         for (int j = 0; j < components.length; j++)
         {
            Component component = components[j];
            if (component instanceof DynamicGraphicCheckBoxMenuItem)
            {
               DynamicGraphicCheckBoxMenuItem jCheckBox = (DynamicGraphicCheckBoxMenuItem) component;
               if (jCheckBox.getText().equals("trajectoryEndPoint"))
               {
                  jCheckBox.setSelected(true);
               }
               else
               {
                  jCheckBox.setSelected(false);
               }
            }
         }
      }
   }

   private void setUpJoyStick(SimulationConstructionSet simulationConstructionSet)
   {
      try
      {
         new DRCJoystickController(simulationConstructionSet);
      }
      catch (RuntimeException e)
      {
         System.out.println("Could not connect to joystick");
      }
   }

   public static void main(String[] args) throws JSAPException
   {
      DRCDemo0ArgumentParser drcDemo0ArgumentParser = new DRCDemo0ArgumentParser(args);

      boolean showInstructions = drcDemo0ArgumentParser.getShowInstructions();

      AutomaticSimulationRunner automaticSimulationRunner = null;

      

      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup();

      new DRCFlatGroundWalkingTrack(guiInitialSetup, automaticSimulationRunner, 0.005, 16000, true, showInstructions);
   }


   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return drcSimulation.getSimulationConstructionSet();
   }

   public DRCDemo01Environment getEnvironment()
   {
      return environment;
   }

   public DRCDemo0SelectedListener getSelectedListener()
   {
      return selectedListener;
   }

   private void setUpDemoButtons(SimulationConstructionSet scs)
   {
//      // an angel loses its wings :(
//      final BooleanYoVariable doBoxPickAction = (BooleanYoVariable) scs.getVariable("doBoxPickAction");
//
//      if (doBoxPickAction != null)
//      {
//         final JButton button = new JButton("Start Action");
//         button.addActionListener(new ActionListener()
//         {
//            public void actionPerformed(ActionEvent arg0)
//            {
//               doBoxPickAction.set(!doBoxPickAction.getBooleanValue());
//            }
//         });
//         doBoxPickAction.addVariableChangedListener(new VariableChangedListener()
//         {
//            public void variableChanged(YoVariable v)
//            {
//               if (((BooleanYoVariable) v).getBooleanValue())
//               {
//                  button.setText("Stop Action");
//                  button.setEnabled(true);
//               }
//               else
//               {
//                  button.setText("Start Action");
//                  button.setEnabled(true);
//               }
//
//            }
//         });
//         scs.addButton(button);
//      }
//      else
//         System.err.println("DrcDemo.setUpDemoButtons: the variable doBoxPickAction has been changed");
   }

   private void setUpInstructionsBoard(SimulationConstructionSet scs, boolean showInstructions)
   {
      if (showInstructions)
      {
         if (showInstructions)
         {
            LinkGraphics linkGraphics = new LinkGraphics();
            linkGraphics.translate(new Vector3d(-1.0, -1.0, 0.0));
            linkGraphics.rotate(Math.toRadians(90), LinkGraphics.Z);
            linkGraphics.scale(0.2);
            linkGraphics.addModelFile(getClass().getResource("3ds/billboard.3DS"));
            scs.addStaticLinkGraphics(linkGraphics);
         }
      }
   }

   public void setTrajectoryEndPoint(FramePoint endPoint)
   {
      selectedListener.setTrajectoryEndPoint(endPoint);
   }
}
