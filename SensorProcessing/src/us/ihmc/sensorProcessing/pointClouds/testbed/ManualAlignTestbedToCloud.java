package us.ihmc.sensorProcessing.pointClouds.testbed;

import georegression.struct.point.Point3D_F64;
import us.ihmc.sensorProcessing.pointClouds.DisplayPointCloudApp;
import us.ihmc.userInterface.BasicUserInterface;
import us.ihmc.userInterface.ThirdPersonPerspective;
import us.ihmc.userInterface.dataStructures.quadTree.JMELidarSpriteGenerator;
import us.ihmc.userInterface.util.TimestampedPoint;

import javax.swing.*;
import javax.swing.border.BevelBorder;
import java.awt.*;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * User: Matt Date: 1/14/13
 */
public class ManualAlignTestbedToCloud extends BasicUserInterface
{
   protected ThirdPersonPerspective thirdPersonPerspective;

   private JMELidarSpriteGenerator world;
   private ArrayList<TimestampedPoint> points = new ArrayList<TimestampedPoint>();


   public ManualAlignTestbedToCloud()
   {
   }

   public void simpleInitApp()
   {
      super.simpleInitApp();

      thirdPersonPerspective = new ThirdPersonPerspective(this);

      setUpGrid();

      // selectionInputManager.registerModifiableObject(new ModifiableBox(this));
      // selectionInputManager.registerModifiableObject(new ModifiableSphere(this));
      Random r = new Random(3);



//     
//
//    for (int i = 0; i < 1; i++)
//    {
//       Node tmp = new Node();
//       Material objectMaterial = new Material(getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
//       objectMaterial.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Alpha);
//       objectMaterial.setColor("Color", ColorRGBA.randomColor());
//       Sphere s = new Sphere(5, 5, 0.025f);
//       Geometry g = new Geometry("g" + i, s);
//       tmp.attachChild(g);
//       getZUpNode().attachChild(tmp);
//       g.setLocalTranslation(r.nextFloat() * 5.0f - 2.5f, r.nextFloat() * 5.0f - 2.5f, r.nextFloat() * 5.0f - 2.5f);
//       g.setMaterial(objectMaterial);
//       g.setShadowMode(ShadowMode.CastAndReceive);
//    }

//    //      

//      for (int i = 0; i < 1; i++)
//      {
//         ModifiablePoint p = new ModifiablePoint(this, "p" + i);
//         getGraphicsObjectIO().createGraphicObjectAtLocation(p,
//                 new Vector3f(r.nextFloat() * 10.0f - 5.0f, r.nextFloat() * 10.0f - 5.0f, r.nextFloat() * 10.0f - 5.0f));
//
//      }

      world = new JMELidarSpriteGenerator(this, thirdPersonPerspective.getViewPort());

      points.clear();

      java.util.List<Point3D_F64> input = DisplayPointCloudApp.loadCloud("../SensorProcessing/data/testbed/2014-07-10/cloud01.txt");

      // if these points change, move this code to simpleUpdate
      for (int i = 0; i < input.size(); i++)
      {
         Point3D_F64 p = input.get(i);
         points.add(new TimestampedPoint((float)p.x,(float)p.z,(float)-p.y, System.currentTimeMillis()));
      }

      world.setSource(points);
      getZUpNode().attachChild(world);

      buildGUI();
   }

   private void buildGUI()
   {
      JFrame frame = new JFrame("Virtual World Interface");
      frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

      Container mainContainer = frame.getContentPane();
      mainContainer.setLayout(new BorderLayout());

      JPanel fpvPanel = new JPanel(new BorderLayout());
      fpvPanel.setBorder(BorderFactory.createBevelBorder(BevelBorder.LOWERED));
      fpvPanel.setPreferredSize(new Dimension(800, 600));
      fpvPanel.add(getCanvas(), BorderLayout.CENTER);
      mainContainer.add(fpvPanel, BorderLayout.WEST);

      frame.pack();
      frame.setLocationRelativeTo(null);
      frame.setVisible(true);
   }



   public void simpleUpdate(float tpf)
   {
      super.simpleUpdate(tpf);

   }

   public static void main(String[] args)
   {
      Logger.getLogger("").setLevel(Level.WARNING);

      new ManualAlignTestbedToCloud();
   }
}
