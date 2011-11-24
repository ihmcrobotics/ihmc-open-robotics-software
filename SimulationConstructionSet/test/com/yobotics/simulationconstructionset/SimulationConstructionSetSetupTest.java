package com.yobotics.simulationconstructionset;

import java.applet.Applet;
import java.awt.BorderLayout;
import java.awt.GraphicsConfiguration;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.media.j3d.Alpha;
import javax.media.j3d.BoundingSphere;
import javax.media.j3d.BranchGroup;
import javax.media.j3d.Canvas3D;
import javax.media.j3d.RotationInterpolator;
import javax.media.j3d.Transform3D;
import javax.media.j3d.TransformGroup;
import javax.swing.JButton;
import javax.swing.JWindow;
import javax.vecmath.Point3d;

import org.junit.Ignore;
import org.junit.Test;

import com.sun.j3d.utils.applet.MainFrame;
import com.sun.j3d.utils.geometry.ColorCube;
import com.sun.j3d.utils.universe.SimpleUniverse;
import com.yobotics.simulationconstructionset.gui.SplashPanel;

public class SimulationConstructionSetSetupTest
{

   @Test
   public void testHelloUniverse()
   {
      boolean WAIT_FOR_HUMAN_TO_PUSH_BUTTON = false;

      JButton jButton = new JButton("Close");
      final boolean[] buttonWasPushed = new boolean[1];

      ActionListener listener = new ActionListener()
      {
         public void actionPerformed(ActionEvent e)
         {
            buttonWasPushed[0] = true;
         }
      };

      jButton.addActionListener(listener);

      HelloUniverse helloUniverse = new HelloUniverse(jButton);

      MainFrame mainFrame = new MainFrame(helloUniverse, 256, 256);

      sleep(5000); 

      while (WAIT_FOR_HUMAN_TO_PUSH_BUTTON && !buttonWasPushed[0])
      {
         sleep(1000);
      }

      mainFrame.dispose();
   }

   @Test
   public void testSplashScreen()
   {
      SplashPanel splashPanel = new SplashPanel();
      JWindow window = splashPanel.showSplashScreen();

      sleep(5000);
      window.dispose();
   }

   @Ignore
   public void testSimulationConstructionSet()
   {
      SimulationConstructionSet scs = new SimulationConstructionSet();
      Thread thread = new Thread(scs);
      thread.start();

      sleep(5000);
      scs.closeAndDispose();
   }

   private void sleep(long sleepMillis)
   {
      try
      {
         Thread.sleep(sleepMillis);
      } catch (InterruptedException e)
      {
      }
   }

   private static class HelloUniverse extends Applet
   {
      private static final long serialVersionUID = 6392083952777727371L;
      private SimpleUniverse simpleUniverse = null;
      private final JButton jButton;

      public HelloUniverse()
      {
         this(null);
      }

      public HelloUniverse(JButton jButton)
      {
         this.jButton = jButton;
      }

      public BranchGroup createSceneGraph()
      {
         // Create the root of the branch graph
         BranchGroup objRoot = new BranchGroup();

         // Create the TransformGroup node and initialize it to the
         // identity. Enable the TRANSFORM_WRITE capability so that
         // our behavior code can modify it at run time. Add it to
         // the root of the subgraph.
         TransformGroup objTrans = new TransformGroup();
         objTrans.setCapability(TransformGroup.ALLOW_TRANSFORM_WRITE);
         objRoot.addChild(objTrans);

         // Create a simple Shape3D node; add it to the scene graph.
         objTrans.addChild(new ColorCube(0.4));

         // Create a new Behavior object that will perform the
         // desired operation on the specified transform and add
         // it into the scene graph.
         Transform3D yAxis = new Transform3D();
         Alpha rotationAlpha = new Alpha(-1, 4000);

         RotationInterpolator rotator = new RotationInterpolator(rotationAlpha, objTrans, yAxis, 0.0f, (float) Math.PI * 2.0f);
         BoundingSphere bounds = new BoundingSphere(new Point3d(0.0, 0.0, 0.0), 100.0);
         rotator.setSchedulingBounds(bounds);
         objRoot.addChild(rotator);

         // Have Java 3D perform optimizations on this scene graph.
         objRoot.compile();

         return objRoot;
      }

      public void init()
      {
         setLayout(new BorderLayout());
         GraphicsConfiguration config = SimpleUniverse.getPreferredConfiguration();

         Canvas3D c = new Canvas3D(config);
         add("Center", c);

         if (jButton != null)
         {
            add("North", jButton);
         }

         // Create a simple scene and attach it to the virtual universe
         BranchGroup scene = createSceneGraph();
         simpleUniverse = new SimpleUniverse(c);

         // This will move the ViewPlatform back a bit so the
         // objects in the scene can be viewed.
         simpleUniverse.getViewingPlatform().setNominalViewingTransform();

         simpleUniverse.addBranchGraph(scene);
      }

      public void destroy()
      {
         simpleUniverse.cleanup();
      }

   }

}
