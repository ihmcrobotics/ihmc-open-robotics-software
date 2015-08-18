package us.ihmc.valkyrie;

import java.awt.BorderLayout;
import java.awt.Canvas;
import java.awt.Container;

import javax.swing.JFrame;
import javax.swing.JPanel;

import us.ihmc.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.commonWalkingControlModules.visualizer.CommonInertiaEllipsoidsVisualizer;
import us.ihmc.simulationconstructionset.InverseDynamicsMechanismReferenceFrameVisualizer;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.graphics.GraphicsRobot;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

public class ValkyrieFullRobotModelVisualizer
{
   public ValkyrieFullRobotModelVisualizer()
   {
//      ValkyrieRobotInterface robotInterface = new ValkyrieRobotInterface();
//      
//      
//      SDFFullRobotModel sdfFullRobotModel = robotInterface.getFullRobotModelFactory().create();
//      GeneralizedSDFRobotModel generalizedSDFRobotModel = robotInterface.getGeneralizedSDFRobotModel();
      
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(false, false);
      
      
      GeneralizedSDFRobotModel generalizedSDFRobotModel = robotModel.getGeneralizedRobotModel();
//      jaxbSDFLoader.createRobot(jointMap, false);
      SDFFullRobotModel sdfFullRobotModel = robotModel.createFullRobotModel();
      
//      sdfFullRobotModel.getRootJoint().setRotation(0.5, 1.0, 0.8);
//      for(OneDoFJoint joint : sdfFullRobotModel.getOneDoFJoints())
//      {
//         joint.setQ(0.0);
//         
//      }

//      for(RobotSide robotSide : RobotSide.values)
//      {
//         sdfFullRobotModel.getArmJoint(robotSide, ArmJointName.SHOULDER_ROLL).setQ(-0.2);
//         sdfFullRobotModel.getArmJoint(robotSide, ArmJointName.SHOULDER_PITCH).setQ(0.5);
//         sdfFullRobotModel.getArmJoint(robotSide, ArmJointName.ELBOW_PITCH).setQ(-1.57);
//         
//         System.out.println(robotSide);
//         sdfFullRobotModel.updateFrames();
//         
//         System.out.println(sdfFullRobotModel.getEndEffectorFrame(robotSide, LimbName.ARM).getTransformToDesiredFrame(ReferenceFrame.getWorldFrame()));
//      }
      
//      ((OneDoFJoint)sdfFullRobotModel.getChest().getParentJoint()).setQ(1.0);
      
      GraphicsRobot robotGraphics = new GraphicsRobot(generalizedSDFRobotModel.getName(), sdfFullRobotModel.getElevator(), generalizedSDFRobotModel, false);
      robotGraphics.update();
      
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      CommonInertiaEllipsoidsVisualizer inertiaVis = new CommonInertiaEllipsoidsVisualizer(sdfFullRobotModel.getElevator(), yoGraphicsListRegistry);
      inertiaVis.update();
      
      InverseDynamicsMechanismReferenceFrameVisualizer referenceFrameVis = new InverseDynamicsMechanismReferenceFrameVisualizer(sdfFullRobotModel.getElevator(), yoGraphicsListRegistry, 0.5);
      referenceFrameVis.doControl();
      
      SimulationConstructionSet scs = new SimulationConstructionSet();
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      
      scs.getGraphics3dAdapter().addRootNode(robotGraphics.getRootNode());
      scs.setGroundVisible(false);
      scs.startOnAThread();

//      JMEGraphics3dAdapter graphicsAdapter = new JMEGraphics3dAdapter();
//      graphicsAdapter.addRootNode(robotGraphics.getRootNode());
//      createNewWindow(graphicsAdapter.createNewViewport(null, true, false).getCanvas());
   }
   
   
   public void createNewWindow(Canvas canvas)
   {
      JPanel panel = new JPanel(new BorderLayout());
      panel.add("Center", canvas);
      
      JFrame jFrame = new JFrame("Example One");
      jFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      Container contentPane = jFrame.getContentPane();
      contentPane.setLayout(new BorderLayout());
      contentPane.add("Center", panel);
      
      jFrame.pack();
      jFrame.setVisible(true);
      jFrame.setSize(800, 600);
   }
   
   public static void main(String[] args)
   {
      new ValkyrieFullRobotModelVisualizer();
   }
}
