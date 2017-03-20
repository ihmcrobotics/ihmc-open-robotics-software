package us.ihmc.simulationconstructionset.util.ground;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.tools.thread.ThreadTools;

public class RollingGroundProfile extends GroundProfileFromHeightMap
{
   private static final double xMinDefault = -20.0, xMaxDefault = 20.0, yMinDefault = -20.0, yMaxDefault = 20.0;
   private static final double amplitudeDefault = 0.1, frequencyDefault = 0.3, offsetDefault = 0.0;
   
   private final BoundingBox3D boundingBox;
   

   protected final double amplitude, frequency, offset;

   public RollingGroundProfile()
   {
      this(amplitudeDefault, frequencyDefault, offsetDefault);  
   }

   public RollingGroundProfile(double amplitude, double frequency, double offset)
   {
      this(amplitude, frequency, offset, xMinDefault, xMaxDefault, yMinDefault, yMaxDefault);
   }

   public RollingGroundProfile(double amplitude, double frequency, double offset, double xMin, double xMax, double yMin, double yMax)
   {
      this.amplitude = amplitude;
      this.frequency = frequency;
      this.offset = offset;
      
      double zMin = Double.NEGATIVE_INFINITY; //-100.0;
      double zMax = Math.abs(amplitude) + 1e-4;
            
      boundingBox = new BoundingBox3D(new Point3D(xMin, yMin, zMin), new Point3D(xMax, yMax, zMax));
   }

   @Override
   public BoundingBox3D getBoundingBox()
   {
      return boundingBox;
   }

   @Override
   public double heightAt(double x, double y, double z)
   {
      double height = amplitude * Math.sin(2.0 * Math.PI * frequency * (x + offset));
      return height;
   }


   public void surfaceNormalAt(double x, double y, double z, Vector3D normal)
   {
      double dzdx = 0.0;

      dzdx = amplitude * 2.0 * Math.PI * frequency * Math.cos(2.0 * Math.PI * frequency * (x + offset));

      normal.setX(-dzdx);
      normal.setY(0.0);
      normal.setZ(1.0);

      normal.normalize();
   }
   
   @Override
   public double heightAndNormalAt(double x, double y, double z, Vector3D normalToPack)
   {
      double heightAt = heightAt(x, y, z);
      surfaceNormalAt(x, y, z, normalToPack);
      return heightAt;
   }

   public static void main(String[] args)
   {
      RollingGroundProfile rollingGroundProfile = new RollingGroundProfile();

      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Null"));
      scs.setGroundVisible(false);
      scs.startOnAThread();
      
      ThreadTools.sleep(1000);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(new Vector3D(0.0, 0.0, 1.0));
      linkGraphics.addSphere(0.5);
      scs.addStaticLinkGraphics(linkGraphics);
      
      
      MeshDataHolder meshData = MeshDataGenerator.Cone(0.8, 0.4, 20);
      Graphics3DObject meshLinkGraphics = new Graphics3DObject();
      meshLinkGraphics.translate(2.0, 0.0, 0.0);
      meshLinkGraphics.addMeshData(meshData, YoAppearance.Green());
      scs.addStaticLinkGraphics(meshLinkGraphics);
      
      Graphics3DObject groundLinkGraphics = new Graphics3DObject();
      groundLinkGraphics.addCoordinateSystem(1.0);
      
      HeightMap heightMap = rollingGroundProfile;
      groundLinkGraphics.addHeightMap(heightMap, 300, 300, YoAppearance.Red());
      scs.addStaticLinkGraphics(groundLinkGraphics);    
   }
   
}
