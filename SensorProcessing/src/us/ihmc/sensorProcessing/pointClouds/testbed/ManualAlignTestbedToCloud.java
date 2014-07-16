/*
 * Copyright (c) 2013-2014, Peter Abeles. All Rights Reserved.
 *
 * This file is part of Project BUBO.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package us.ihmc.sensorProcessing.pointClouds.testbed;

import boofcv.gui.image.ShowImages;
import com.jme3.app.SimpleApplication;
import com.jme3.asset.AssetManager;
import com.jme3.asset.plugins.FileLocator;
import com.jme3.input.KeyInput;
import com.jme3.input.MouseInput;
import com.jme3.input.controls.ActionListener;
import com.jme3.input.controls.AnalogListener;
import com.jme3.input.controls.KeyTrigger;
import com.jme3.input.controls.MouseButtonTrigger;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.*;
import com.jme3.system.AppSettings;
import com.jme3.system.JmeCanvasContext;
import com.jme3.texture.plugins.AWTLoader;
import com.jme3.util.TangentBinormalGenerator;
import com.thoughtworks.xstream.XStream;
import georegression.geometry.RotationMatrixGenerator;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.struct.so.Quaternion_F64;
import jme3dae.ColladaLoader;
import org.jmonkeyengine.scene.plugins.ogre.MaterialLoader;
import us.ihmc.graphics3DAdapter.jme.JMEAssetLocator;
import us.ihmc.graphics3DAdapter.jme.util.JME3DLoaderUtils;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNodeType;
import us.ihmc.sensorProcessing.pointClouds.DisplayPointCloudApp;
import us.ihmc.sensorProcessing.pointClouds.GeometryOps;
import us.ihmc.utilities.math.MatrixTools;

import javax.swing.*;
import java.awt.*;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.net.URL;
import java.util.*;
import java.util.List;
import java.util.concurrent.Callable;

/**
 * @author Peter Abeles
 */
// TODO improve rotation alignment
// TODO continuously move translations
public class ManualAlignTestbedToCloud extends SimpleApplication {

   public static final String OBJ_LEFT = "OBJ_LEFT";
   public static final String OBJ_RIGHT = "OBJ_RIGHT";
   public static final String OBJ_UP = "OBJ_UP";
   public static final String OBJ_DOWN = "OBJ_DOWN";
   public static final String OBJ_FORWARDS = "OBJ_FORWARDS";
   public static final String OBJ_BACKWARDS = "OBJ_BACKWARDS";
   public static final String SLOW_DOWN = "SLOW_DOWN";
   public static final String SPEED_UP = "SPEED_UP";

   public static final String SAVE_TRANFORM = "SAVE_TRANFORM";
   public static final String LOAD_TRANFORM = "LOAD_TRANFORM";


   public static final String OBJ_ROTATE = "OBJ_ROTATE";

   Canvas canvas;
   JMEAssetLocator assetLocator;

   Node testbedTransform = new Node("TestbedTransform");

   float mouseInitX,mouseInitY;

   public ManualAlignTestbedToCloud() {
      AppSettings settings = new AppSettings(true);
      settings.setWidth(800);
      settings.setHeight(800);
      settings.setFrameRate(30);

      setDisplayStatView(false);
      setDisplayFps(false);
      setPauseOnLostFocus(false);
      setSettings(settings);
      createCanvas();
      startCanvas();

      JmeCanvasContext context = (JmeCanvasContext)getContext();
      canvas = context.getCanvas();
      canvas.setSize(settings.getWidth(), settings.getHeight());
   }

   public Canvas getCanvas()
   {
      return canvas;
   }


   public void addPoints(java.util.List<Point3D_F64> points, final int color , final float size ) {
      final float[] buf = convertPointsToArray(points);

      enqueue(new Callable<Void>() {
         public Void call() {
            float alpha = ((color >> 24) & 0xFF) / 255.0f;
            float red = ((color >> 16) & 0xFF) / 255.0f;
            float green = ((color >> 8) & 0xFF) / 255.0f;
            float blue = (color & 0xFF) / 255.0f;

            Material mat = new Material(getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
            mat.setColor("Color", new ColorRGBA(red, green, blue, alpha));
            mat.getAdditionalRenderState().setFaceCullMode(RenderState.FaceCullMode.Off);

            Mesh m = new Mesh();
            m.setBuffer(VertexBuffer.Type.Position, 3, buf);
            m.setMode(Mesh.Mode.Points);
            m.setPointSize(size);
            m.setStatic();
            m.updateBound();

            Geometry g = new Geometry("Point Cloud", m);
            g.setShadowMode(RenderQueue.ShadowMode.Off);
            g.setQueueBucket(RenderQueue.Bucket.Opaque);
            g.setMaterial(mat);
            getRootNode().attachChild(g);
            return null;
         }
      });
   }

   public void addTestBedModel() {
      enqueue(new Callable<Void>() {
         public Void call() {
            Spatial object3DSGraphics = JME3DLoaderUtils.load3DModel("models/ManualTestBed.obj", assetLocator, Graphics3DNodeType.VISUALIZATION);
            TangentBinormalGenerator.generate(object3DSGraphics);

            Material mat = new Material(assetManager, "Common/MatDefs/Light/Lighting.j3md");
            mat.setTexture("DiffuseMap", assetManager.loadTexture("Textures/Terrain/Pond/Pond.jpg"));
            mat.setTexture("NormalMap",
                    assetManager.loadTexture("Textures/Terrain/Pond/Pond_normal.png"));
            mat.setBoolean("UseMaterialColors", true);
            mat.setColor("Diffuse", ColorRGBA.White);  // minimum material color
            mat.setColor("Specular", ColorRGBA.White); // for shininess
            mat.setFloat("Shininess", 64f); // [1,128] for shininess

//            Material mat = new Material(getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
//            mat.setColor("Color", new ColorRGBA(0, 1.0f, 0,0.5f));
//            mat.getAdditionalRenderState().setFaceCullMode(RenderState.FaceCullMode.Off);


            object3DSGraphics.setShadowMode(RenderQueue.ShadowMode.CastAndReceive);
            object3DSGraphics.setMaterial(mat);

            testbedTransform.attachChild(object3DSGraphics);
            return null;
         }
      });
   }

   @Override
   public void simpleInitApp() {
      flyCam.setDragToRotate(true);
      flyCam.setMoveSpeed(5);
      assetLocator = new JMEAssetLocator(assetManager);

      getRootNode().attachChild(testbedTransform);

      DirectionalLight sun = new DirectionalLight();
      sun.setColor(ColorRGBA.White);
      sun.setDirection(new Vector3f(-.5f,-.5f,-.5f).normalizeLocal());
      rootNode.addLight(sun);

      sun = new DirectionalLight();
      sun.setColor(ColorRGBA.White);
      sun.setDirection(new Vector3f(.5f,.5f,.5f).normalizeLocal());
      rootNode.addLight(sun);

      inputManager.addMapping(OBJ_LEFT,  new KeyTrigger(KeyInput.KEY_G));
      inputManager.addMapping(OBJ_RIGHT,   new KeyTrigger(KeyInput.KEY_J));
      inputManager.addMapping(OBJ_FORWARDS,  new KeyTrigger(KeyInput.KEY_Y));
      inputManager.addMapping(OBJ_BACKWARDS,  new KeyTrigger(KeyInput.KEY_H));
      inputManager.addMapping(OBJ_UP,  new KeyTrigger(KeyInput.KEY_T));
      inputManager.addMapping(OBJ_DOWN,  new KeyTrigger(KeyInput.KEY_U));
      inputManager.addMapping(SLOW_DOWN,  new KeyTrigger(KeyInput.KEY_DOWN));
      inputManager.addMapping(SPEED_UP,  new KeyTrigger(KeyInput.KEY_UP));
      inputManager.addMapping(SAVE_TRANFORM,  new KeyTrigger(KeyInput.KEY_RETURN));
      inputManager.addMapping(LOAD_TRANFORM,  new KeyTrigger(KeyInput.KEY_PGUP));

      inputManager.addMapping(OBJ_ROTATE, new MouseButtonTrigger(MouseInput.BUTTON_RIGHT));

      inputManager.addListener(new ObjectMoveListener(),OBJ_LEFT,OBJ_RIGHT,OBJ_FORWARDS,OBJ_BACKWARDS,OBJ_UP,
              OBJ_DOWN,OBJ_ROTATE,SAVE_TRANFORM,LOAD_TRANFORM,SPEED_UP,SLOW_DOWN);



      inputManager.addListener(new ObjectRotateListener(),OBJ_ROTATE);
   }

   private float[] convertPointsToArray(List<Point3D_F64> points) {
      int N = points.size();

      final float buf[] = new float[N*3];
      for( int i = 0; i < N; i++ ) {
         Point3D_F64 p = points.get(i);
         buf[i*3+0] = (float)p.x;
         buf[i*3+1] = (float)p.y;
         buf[i*3+2] = (float)p.z;
      }
      return buf;
   }

   private class ObjectMoveListener implements ActionListener {
      public void onAction(String name, boolean keyPressed, float tpf) {

         Transform transform = testbedTransform.getLocalTransform();

         if( name.equals(OBJ_ROTATE)) {
            if( keyPressed ) {
               mouseInitX = getInputManager().getCursorPosition().x;
               mouseInitY = getInputManager().getCursorPosition().y;
            }
         } else {

            if (!keyPressed)
               return;

            if (name.equals(OBJ_LEFT)) {
               transform.getTranslation().x -= 0.05*speed;
            } else if (name.equals(OBJ_RIGHT)) {
               transform.getTranslation().x += 0.05*speed;
            } else if (name.equals(OBJ_FORWARDS)) {
               transform.getTranslation().z -= 0.05*speed;
            } else if (name.equals(OBJ_BACKWARDS)) {
               transform.getTranslation().z += 0.05*speed;
            } else if (name.equals(OBJ_UP)) {
               transform.getTranslation().y += 0.05*speed;
            } else if (name.equals(OBJ_DOWN)) {
               transform.getTranslation().y -= 0.05*speed;
            } else if (name.equals(SPEED_UP)) {
               speed *= 1.1;
               System.out.println("speed = "+speed);
            } else if (name.equals(SLOW_DOWN)) {
               speed *= 0.9;
               System.out.println("speed = "+speed);
            } else if (name.equals(SAVE_TRANFORM)) {
               System.out.println("Saving transform");
               Se3_F64 transformGR = GeometryOps.convert(transform,null);

               try {
                  new XStream().toXML(transformGR,new FileOutputStream("testbedToWorld.xml"));
               } catch (FileNotFoundException e) {
                  throw new RuntimeException(e);
               }
            } else if (name.equals(LOAD_TRANFORM)) {
               System.out.println("Loading transform");
               try {
                  Se3_F64 transformGR = (Se3_F64)new XStream().fromXML(new FileInputStream("testbedToWorld.xml"));
                  transform = GeometryOps.convert(transformGR,null);
                  testbedTransform.setLocalTransform(transform);
               } catch (FileNotFoundException e) {
                  throw new RuntimeException(e);
               }
            }
         }

         testbedTransform.setLocalTransform(transform);
      }
   }

   private class ObjectRotateListener implements AnalogListener {

      @Override
      public void onAnalog(String name, float value, float tpf) {
         float foo = 0.01f;

         if( name.equals(OBJ_ROTATE)) {
            float mouseX = getInputManager().getCursorPosition().x;
            float mouseY = getInputManager().getCursorPosition().y;

            float rotX = mouseX-mouseInitX;
            float rotY = mouseY-mouseInitY;

            testbedTransform.rotate(rotX*speed*foo, 0, 0);
            testbedTransform.rotate(0, rotY*speed*foo, 0);

            mouseInitX = mouseX;
            mouseInitY = mouseY;
         }
      }
   }

   public static void main(String[] args) {
      List<Point3D_F64> cloud = DisplayPointCloudApp.loadCloud("../SensorProcessing/data/testbed/2014-07-10/cloud01.txt");
      ManualAlignTestbedToCloud app = new ManualAlignTestbedToCloud();
      app.addPoints(cloud,0xFF0000,1);
      app.addTestBedModel();


      JPanel gui = new JPanel();
      gui.add( app.getCanvas() );
      gui.setPreferredSize( new Dimension(800,800));

      ShowImages.showWindow(gui,"Testbed Stuff");
      gui.requestFocus();

   }
}
