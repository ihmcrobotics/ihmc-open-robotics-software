package us.ihmc.jMonkeyEngineToolkit.jme;
 
import java.awt.BorderLayout;
import java.awt.Canvas;
import java.awt.Container;

import javax.swing.JFrame;
import javax.swing.JPanel;

import com.jme3.app.SimpleApplication;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.scene.Geometry;
import com.jme3.scene.shape.Box;
import com.jme3.system.AppSettings;
import com.jme3.system.awt.AwtPanel;
import com.jme3.system.awt.AwtPanelsContext;
import com.jme3.system.awt.PaintMode;

import us.ihmc.jMonkeyEngineToolkit.jme.context.PBOAwtPanel;
import us.ihmc.jMonkeyEngineToolkit.jme.context.PBOAwtPanelsContext;
 
/** Sample 4 - how to trigger repeating actions from the main update loop.
 * In this example, we make the player character rotate. */
public class AWTPanelsTest extends SimpleApplication {
 
   private static final boolean USE_PBO = true;
 
   
    protected Geometry player;
 
    @Override
    public void simpleInitApp() {
 
        Box b = new Box(1, 1, 1);
        player = new Geometry("blue cube", b);
        Material mat = new Material(assetManager,
          "Common/MatDefs/Misc/Unshaded.j3md");
        mat.setColor("Color", ColorRGBA.Blue);
        player.setMaterial(mat);
        rootNode.attachChild(player);
        
        flyCam.setDragToRotate(true);
        
        if(context instanceof AwtPanelsContext)
        {
           AwtPanelsContext awtPanelsContext = (AwtPanelsContext) context;
           AwtPanel panel = awtPanelsContext.createPanel(PaintMode.Accelerated);
           panel.attachTo(true, viewPort, guiViewPort);
           createNewWindow(panel);
        }
        else
        {
           PBOAwtPanelsContext awtPanelsContext = (PBOAwtPanelsContext) context;
           PBOAwtPanel panel = awtPanelsContext.createPanel();
           panel.attachTo(true, viewPort, guiViewPort);
           createNewWindow(panel);
        }
        
    }
 
    /* This is the update loop */
    @Override
    public void simpleUpdate(float tpf) {
        // make the player rotate
        player.rotate(0, 2*tpf, 0); 
    }
    
    private static void createNewWindow(Canvas canvas)
    {
       JPanel panel = new JPanel(new BorderLayout());
       panel.add("Center", canvas);
       
       JFrame jFrame = new JFrame("AWTPanelsTest");
       jFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
       Container contentPane = jFrame.getContentPane();
       contentPane.setLayout(new BorderLayout());
       contentPane.add("Center", panel);
       
       jFrame.pack();
       jFrame.setVisible(true);
       jFrame.setSize(1920, 1080);
    }
    
    public static void main(String[] argv)
    {
       AppSettings appSettings = new AppSettings(true);
       if(USE_PBO)
       {
          appSettings.setCustomRenderer(PBOAwtPanelsContext.class);
             
       }
       else
       {
          appSettings.setCustomRenderer(AwtPanelsContext.class);
       }
       
       AWTPanelsTest test = new AWTPanelsTest();
       test.setSettings(appSettings);
       
       test.setShowSettings(false);
       
       test.start();
    }
    
    
}