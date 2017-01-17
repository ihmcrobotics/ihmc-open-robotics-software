package us.ihmc.jMonkeyEngineToolkit.jme;

import java.awt.BorderLayout;
import java.awt.Canvas;
import java.awt.Color;
import java.awt.Container;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.geom.Rectangle2D;
import java.awt.image.BufferedImage;

import javax.swing.JFrame;
import javax.swing.JPanel;

import us.ihmc.graphicsDescription.structure.Graphics3DNodeType;
import us.ihmc.jMonkeyEngineToolkit.camera.ClassicCameraController;
import us.ihmc.jMonkeyEngineToolkit.camera.ViewportAdapter;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEAppearanceMaterial;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEAssetLocator;
import us.ihmc.jMonkeyEngineToolkit.jme.JMERenderer;
import us.ihmc.jMonkeyEngineToolkit.jme.util.JME3DLoaderUtils;
import us.ihmc.jMonkeyEngineToolkit.utils.GraphicsDemoTools.PanBackAndForthTrackingAndDollyPositionHolder;

import com.jme3.math.FastMath;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.system.AppSettings;
import com.jme3.system.awt.AwtPanelsContext;

public class JMEBufferedImageAppearanceTester extends JMERenderer
{
   public JMEBufferedImageAppearanceTester()
   {
      super(RenderType.AWTPANELS);
      // TODO Auto-generated constructor stub
   }

   int counter = 0;
   

   Geometry footIconGeometryLeft;
   public void simpleInitApp()
   {
      super.simpleInitApp();
//      Graphics3DObject graphics = new Graphics3DObject();
//      graphics.setChangeable(true);
//      instruction = graphics.addModelFile(getClass().getResource("Models/LeftFootPathIcon.obj").getFile(), YoAppearance.Blue());
     
      footIconGeometryLeft = JME3DLoaderUtils.extractFirstGeometry(getClass().getResource("Models/LeftFootPathIcon.obj").getFile(),
            new JMEAssetLocator(assetManager), Graphics3DNodeType.VISUALIZATION);
      
//      JMEGraphicsObject jmeGraphicsObject = new JMEGraphicsObject(this, new JMEAssetLocator(assetManager), graphics);
      Node rotation = new Node();
      rotation.rotate(FastMath.HALF_PI, 0, 0);
      rotation.attachChild(footIconGeometryLeft);
      getZUpNode().attachChild(rotation);
      
   }
   
   public int index = 0;
   @Override
   public void simpleUpdate(float tpf)

   {
      
      super.simpleUpdate(tpf);

      if(counter % 100 == 0)
      {
         double footWidth = 0.10;
         double footHeight = 0.25;
         
         double footEdgeStart = 0.007;
         double footEdgeEnd = 0.055;
         
         int width = 100;
         int height = (int) (footWidth/footHeight * 100.0);
         BufferedImage appearance = new BufferedImage(width, height, BufferedImage.TYPE_4BYTE_ABGR);
         Graphics2D g = (Graphics2D) appearance.getGraphics();
   
         g.setColor(new Color(1.0f, 0.0f, 0.0f, 0.4f));
         g.fillRect(0, 0, width, height);
         g.setColor(Color.black);
         
         
         
         
         Font font = new Font("Arial", Font.PLAIN, 18);
         FontMetrics metrics = g.getFontMetrics(font);
         
         String text = String.valueOf(index);
         Rectangle2D bounds = metrics.getStringBounds(text, g);
         
         
         int xStart = (int) (height * (footEdgeStart/footWidth));
         int xEnd = (int) (height * (footEdgeEnd/footWidth));
         
         int y = ((xEnd-xStart) - (int)bounds.getWidth())/2 + xStart;
         int x = (width - (int)bounds.getHeight())/2;
         g.translate(x, y);
         g.rotate(Math.PI/2.0);
               
         g.setFont(font);
         g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
         g.drawString(text, 0, 0);
         g.dispose();
         footIconGeometryLeft.setMaterial(JMEAppearanceMaterial.createMaterialFromBufferedImage(new JMEAssetLocator(assetManager), appearance));
         footIconGeometryLeft.setQueueBucket(RenderQueue.Bucket.Opaque);
         index++;
      }
      
      
      
      
      counter++;
   }
   
   public static void main(String[] args)
   {
      JMEBufferedImageAppearanceTester jmeChangeAppearanceTester = new JMEBufferedImageAppearanceTester();
      AppSettings appSettings = new AppSettings(true);
      appSettings.setCustomRenderer(AwtPanelsContext.class);
      appSettings.setResolution(800, 600);
      jmeChangeAppearanceTester.setSettings(appSettings);


      jmeChangeAppearanceTester.setShowSettings(false);
      jmeChangeAppearanceTester.setPauseOnLostFocus(false);
      
      
      

      PanBackAndForthTrackingAndDollyPositionHolder cameraTrackAndDollyVariablesHolder = new PanBackAndForthTrackingAndDollyPositionHolder(0.0, 2.0, 0.2);

      ViewportAdapter viewportAdapter = jmeChangeAppearanceTester.createNewViewport(null, false, false);
      ClassicCameraController classicCameraController = ClassicCameraController.createClassicCameraControllerAndAddListeners(viewportAdapter, cameraTrackAndDollyVariablesHolder, jmeChangeAppearanceTester);
      viewportAdapter.setCameraController(classicCameraController);
      Canvas canvas = viewportAdapter.getCanvas();
      createNewWindow(canvas);
   }


   private static void createNewWindow(Canvas canvas)
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
}
