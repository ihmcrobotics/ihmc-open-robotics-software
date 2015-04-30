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

import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;

import java.awt.Canvas;
import java.awt.Dimension;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.util.List;
import java.util.concurrent.Callable;

import javax.swing.JPanel;

import us.ihmc.graphics3DAdapter.jme.JMEAssetLocator;
import us.ihmc.graphics3DAdapter.jme.util.JME3DLoaderUtils;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNodeType;
import us.ihmc.sensorProcessing.pointClouds.GeometryOps;
import boofcv.gui.image.ShowImages;

import com.jme3.app.SimpleApplication;
import com.jme3.input.KeyInput;
import com.jme3.input.MouseInput;
import com.jme3.input.controls.ActionListener;
import com.jme3.input.controls.AnalogListener;
import com.jme3.input.controls.KeyTrigger;
import com.jme3.input.controls.MouseButtonTrigger;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.VertexBuffer;
import com.jme3.system.AppSettings;
import com.jme3.system.JmeCanvasContext;
import com.jme3.util.TangentBinormalGenerator;
import com.thoughtworks.xstream.XStream;

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

   public static final String ROTATE_X = "ROTATE_X";
   public static final String ROTATE_Y = "ROTATE_Y";
   public static final String ROTATE_Z = "ROTATE_Z";

   public static final String SAVE_TRANFORM = "SAVE_TRANFORM";
   public static final String LOAD_TRANFORM = "LOAD_TRANFORM";


   public static final String OBJ_ROTATE = "OBJ_ROTATE";

   int axisRotation = 0;

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
            g.setQueueBucket(RenderQueue.Bucket.Translucent);
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

   public void setTestbedToWorld( final Se3_F64 testbedToWorld ) {
      enqueue(new Callable<Void>() {
         public Void call() {
            Transform transform = GeometryOps.convert(testbedToWorld, null);
            testbedTransform.setLocalTransform(transform);
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

      inputManager.addMapping(ROTATE_X,  new KeyTrigger(KeyInput.KEY_NUMPAD1));
      inputManager.addMapping(ROTATE_Y,  new KeyTrigger(KeyInput.KEY_NUMPAD2));
      inputManager.addMapping(ROTATE_Z,  new KeyTrigger(KeyInput.KEY_NUMPAD3));

      inputManager.addMapping(OBJ_ROTATE, new MouseButtonTrigger(MouseInput.BUTTON_RIGHT));

      inputManager.addListener(new ObjectMoveListener(),OBJ_LEFT,OBJ_RIGHT,OBJ_FORWARDS,OBJ_BACKWARDS,OBJ_UP,
              OBJ_DOWN,OBJ_ROTATE,SAVE_TRANFORM,LOAD_TRANFORM,SPEED_UP,SLOW_DOWN,ROTATE_X,ROTATE_Y,ROTATE_Z);



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
            } else if (name.equals(ROTATE_X)) {
               System.out.println("Rotate X");
               axisRotation = 0;
            } else if (name.equals(ROTATE_Y)) {
               System.out.println("Rotate Y");
               axisRotation = 1;
            } else if (name.equals(ROTATE_Z)) {
               System.out.println("Rotate Z");
               axisRotation = 2;
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
                  new XStream().toXML(transformGR, new FileOutputStream("modelTestbedToWorld.xml"));
               } catch (FileNotFoundException e) {
                  throw new RuntimeException(e);
               }
            } else if (name.equals(LOAD_TRANFORM)) {
               System.out.println("Loading transform");
               try {
                  Se3_F64 transformGR = (Se3_F64)new XStream().fromXML(new FileInputStream("modelTestbedToWorld.xml"));
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
//            float mouseY = getInputManager().getCursorPosition().y;

            float rotX = mouseX-mouseInitX;
//            float rotY = mouseY-mouseInitY;

            switch(axisRotation) {
               case 0:
                  testbedTransform.rotate(rotX*speed*foo, 0, 0);
                  break;

               case 1:
                  testbedTransform.rotate(0, rotX*speed*foo, 0);
                  break;

               case 2:
                  testbedTransform.rotate(0, 0, rotX*speed*foo);
                  break;

            }
//            testbedTransform.rotate(0, rotY*speed*foo, 0);

            mouseInitX = mouseX;
//            mouseInitY = mouseY;
         }
      }
   }

   public static void main(String[] args) {
      List<Point3D_F64> cloud = GeometryOps.loadCloud("../SensorProcessing/data/testbed/2014-08-01/cloud02.txt");
      ManualAlignTestbedToCloud app = new ManualAlignTestbedToCloud();
      app.addPoints(cloud,0xFF0000,2);
      app.addTestBedModel();


      JPanel gui = new JPanel();
      gui.add( app.getCanvas() );
      gui.setPreferredSize( new Dimension(800,800));

      ShowImages.showWindow(gui,"Testbed Stuff");
      gui.requestFocus();

   }
}
