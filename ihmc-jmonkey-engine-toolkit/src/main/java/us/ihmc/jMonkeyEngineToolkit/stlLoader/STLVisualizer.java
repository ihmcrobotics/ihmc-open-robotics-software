/*
 *   Copyright 2014 Florida Institute for Human and Machine Cognition (IHMC)
 *    
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *    
 *    http://www.apache.org/licenses/LICENSE-2.0
 *    
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 *    
 *    Written by Jesper Smith with assistance from IHMC team members
 */
package us.ihmc.jMonkeyEngineToolkit.stlLoader;

import com.jme3.app.SimpleApplication;
import com.jme3.asset.plugins.ClasspathLocator;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.scene.Spatial;

/**
 * Visualizer class to show STL files in a JMonkeyEngine application
 * 
 * @author Jesper Smith
 *
 */
public class STLVisualizer extends SimpleApplication
{
   private final String filename;
   public STLVisualizer(String filename)
   {
      this.filename = filename;
   }

   @Override
   public void simpleInitApp()
   {
      assetManager.registerLocator("/", ClasspathLocator.class);
      assetManager.registerLoader(STLLoader.class, "stl");
      flyCam.setDragToRotate(true);
      flyCam.setZoomSpeed(10.0f);
      flyCam.setMoveSpeed(10.0f);

      Spatial model = assetManager.loadModel(filename);

      Material mat = new Material(assetManager, "Common/MatDefs/Light/Lighting.j3md");
      mat.setBoolean("UseMaterialColors", true);
      mat.setColor("Diffuse", ColorRGBA.White);
      mat.setColor("Specular", ColorRGBA.White);
      mat.setFloat("Shininess", 64f);
//      mat.getAdditionalRenderState().setFaceCullMode(FaceCullMode.Off);
      model.setMaterial(mat);

      model.scale(0.01f);
      rootNode.attachChild(model);
      
      DirectionalLight sun = new DirectionalLight();
      sun.setDirection(new Vector3f(1,0,-2).normalizeLocal());
      sun.setColor(ColorRGBA.White);
      rootNode.addLight(sun);
      
      
      
   }

   public static void main(String[] args)
   {
      String filename;
      if(args.length == 1)
      {
         filename = args[1];
      }
      else
      {
         filename = "teapotBinary.STL";
      }
      STLVisualizer stlVisualizer = new STLVisualizer(filename);
      stlVisualizer.setShowSettings(false);
      stlVisualizer.start();
   }
}
