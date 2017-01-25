package us.ihmc.modelFileLoaders.SdfLoader.xmlDescription;

import java.util.List;

import javax.xml.bind.annotation.XmlElement;

public class SDFGeometry
{
   private Box box;
   private Sphere sphere;
   private Cylinder cylinder;
   private Mesh mesh;
   private Plane plane;
   private GeometryImage image;
   private HeightMap heightMap;

   public Box getBox()
   {
      return box;
   }

   @XmlElement(name = "box")
   public void setBox(Box box)
   {
      this.box = box;
   }

   public Sphere getSphere()
   {
      return sphere;
   }

   @XmlElement(name = "sphere")
   public void setSphere(Sphere sphere)
   {
      this.sphere = sphere;
   }

   public Cylinder getCylinder()
   {
      return cylinder;
   }

   @XmlElement(name = "cylinder")
   public void setCylinder(Cylinder cylinder)
   {
      this.cylinder = cylinder;
   }

   public Mesh getMesh()
   {
      return mesh;
   }

   @XmlElement(name = "mesh")
   public void setMesh(Mesh mesh)
   {
      this.mesh = mesh;
   }

   public Plane getPlane()
   {
      return plane;
   }

   @XmlElement(name = "plane")
   public void setPlane(Plane plane)
   {
      this.plane = plane;
   }

   public GeometryImage getImage()
   {
      return image;
   }

   @XmlElement(name = "image")
   public void setImage(GeometryImage image)
   {
      this.image = image;
   }

   public HeightMap getHeightMap()
   {
      return heightMap;
   }

   @XmlElement(name = "heightmap")
   public void setHeightMap(HeightMap heightMap)
   {
      this.heightMap = heightMap;
   }

   public static class Box
   {
      private String size;

      public String getSize()
      {
         return size;
      }

      @XmlElement(name = "size")
      public void setSize(String size)
      {
         this.size = size;
      }
   }

   public static class Sphere
   {
      private String radius;

      public String getRadius()
      {
         return radius;
      }

      @XmlElement(name = "radius")
      public void setRadius(String radius)
      {
         this.radius = radius;
      }
   }

   public static class Cylinder
   {
      private String radius;
      private String length;

      public String getRadius()
      {
         return radius;
      }

      @XmlElement(name = "radius")
      public void setRadius(String radius)
      {
         this.radius = radius;
      }

      public String getLength()
      {
         return length;
      }

      @XmlElement(name = "length")
      public void setLength(String length)
      {
         this.length = length;
      }
   }

   public static class Mesh
   {
      private String uri;
      private String scale;
      private SubMesh submesh;

      public String getUri()
      {
         return uri;
      }

      @XmlElement(name = "uri")
      public void setUri(String uri)
      {
         this.uri = uri;
      }

      public String getScale()
      {
         return scale;
      }

      @XmlElement(name = "scale")
      public void setScale(String scale)
      {
         this.scale = scale;
      }

      public SubMesh getSubmesh()
      {
         return submesh;
      }

      @XmlElement(name = "submesh")
      public void setSubmesh(SubMesh submesh)
      {
         this.submesh = submesh;
      }

      public static class SubMesh
      {
         private String name;
         private String center;

         public String getName()
         {
            return name;
         }

         public String getCenter()
         {
            return center;
         }

         @XmlElement(name = "name")
         public void setName(String name)
         {
            this.name = name;
         }

         @XmlElement(name = "center")
         public void setCenter(String center)
         {
            this.center = center;
         }

      }

   }

   public static class Plane
   {
      private String normal;
      private String size;

      public String getNormal()
      {
         return normal;
      }

      @XmlElement(name = "normal")
      public void setNormal(String normal)
      {
         this.normal = normal;
      }

      public String getSize()
      {
         return size;
      }

      @XmlElement(name = "size")
      public void setSize(String size)
      {
         this.size = size;
      }
   }

   public static class GeometryImage
   {
      private String uri;
      private String scale;
      private String threshold;
      private String height;
      private String granularity;

      public String getUri()
      {
         return uri;
      }

      @XmlElement(name = "uri")
      public void setUri(String uri)
      {
         this.uri = uri;
      }

      public String getScale()
      {
         return scale;
      }

      @XmlElement(name = "scale")
      public void setScale(String scale)
      {
         this.scale = scale;
      }

      public String getThreshold()
      {
         return threshold;
      }

      @XmlElement(name = "threshold")
      public void setThreshold(String threshold)
      {
         this.threshold = threshold;
      }

      public String getHeight()
      {
         return height;
      }

      @XmlElement(name = "height")
      public void setHeight(String height)
      {
         this.height = height;
      }

      public String getGranularity()
      {
         return granularity;
      }

      @XmlElement(name = "granularity")
      public void setGranularity(String granularity)
      {
         this.granularity = granularity;
      }

   }

   public static class HeightMap
   {
      private String uri;
      private String size;
      private String pos;

      private List<Texture> textures;
      private List<Blend> blends;

      public String getUri()
      {
         return uri;
      }

      @XmlElement(name = "uri")
      public void setUri(String uri)
      {
         this.uri = uri;
      }

      public String getSize()
      {
         return size;
      }

      @XmlElement(name = "size")
      public void setSize(String size)
      {
         this.size = size;
      }

      public String getPos()
      {
         return pos;
      }

      @XmlElement(name = "pos")
      public void setPos(String pos)
      {
         this.pos = pos;
      }

      public List<Texture> getTextures()
      {
         return textures;
      }

      @XmlElement(name = "texture")
      public void setTextures(List<Texture> textures)
      {
         this.textures = textures;
      }

      public List<Blend> getBlends()
      {
         return blends;
      }

      @XmlElement(name = "blend")
      public void setBlends(List<Blend> blends)
      {
         this.blends = blends;
      }

      public static class Texture
      {
         private String size;
         private String diffuse;
         private String normal;

         public String getSize()
         {
            return size;
         }

         @XmlElement(name = "size")
         public void setSize(String size)
         {
            this.size = size;
         }

         public String getDiffuse()
         {
            return diffuse;
         }

         @XmlElement(name = "diffuse")
         public void setDiffuse(String diffuse)
         {
            this.diffuse = diffuse;
         }

         public String getNormal()
         {
            return normal;
         }

         @XmlElement(name = "normal")
         public void setNormal(String normal)
         {
            this.normal = normal;
         }

      }

      public static class Blend
      {
         private String minHeight;
         private String fadeDist;

         public String getMinHeight()
         {
            return minHeight;
         }

         @XmlElement(name = "min_height")
         public void setMinHeight(String minHeight)
         {
            this.minHeight = minHeight;
         }

         public String getFadeDist()
         {
            return fadeDist;
         }

         @XmlElement(name = "fade_dist")
         public void setFadeDist(String fadeDist)
         {
            this.fadeDist = fadeDist;
         }

      }
   }

}