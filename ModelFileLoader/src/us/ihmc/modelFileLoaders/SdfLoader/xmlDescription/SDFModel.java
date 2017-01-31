package us.ihmc.modelFileLoaders.SdfLoader.xmlDescription;

import java.util.List;

import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElement;

public class SDFModel
{
   private String name;

   private String pose;

   private List<SDFLink> links;

   private List<SDFJoint> joints;

   public String getName()
   {
      return name;
   }

   @XmlAttribute
   public void setName(String name)
   {
      this.name = name;
   }

   public String getPose()
   {
      return pose;
   }

   @XmlElement(name = "pose")
   public void setPose(String pose)
   {
      this.pose = pose;
   }

   public List<SDFLink> getLinks()
   {
      return links;
   }

   @XmlElement(name = "link")
   public void setLinks(List<SDFLink> links)
   {
      this.links = links;
   }

   public List<SDFJoint> getJoints()
   {
      return joints;
   }

   @XmlElement(name = "joint")
   public void setJoints(List<SDFJoint> joints)
   {
      this.joints = joints;
   }

   public String toString()
   {
      return "\n\tJoints : " + (joints!=null?joints.toString():"[]") + "\n\tLinks: " + (links!=null?links.toString():"[]");
   }
}