/*
 * 
 */
package us.ihmc.robotics.hierarchicalKinematics;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;
import us.ihmc.robotics.geometry.Capsule;
import us.ihmc.robotics.geometry.RigidBodyTransform;

import javax.vecmath.Point3d;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.LinkedList;

/**
 * The Class CapsuleCollisionCheck checks the minimum distance between
 * the capsules associated to a pair of links.
 * <p>
 * Additionally, it has a method to parse a URDF file and create the list of capsules.
 * <p>
 */
public class CapsuleCollisionCheck
{
   final public HashMap<String, LinkedList<Capsule>> capsulesByLink = new HashMap<String, LinkedList<Capsule>>();
   private int numCapsules = 0;


   public int getNumCapsules() { 
      return numCapsules;
   }
    
   /**
    * Gets the closest points between the capsule(s) associated to linkA and those of
    * linkB. Links are identified by their name. 
    * A transform for both links need to be provided. These transform must be expressed in the same 
    * reference frame.
    *
    * @param linkNameA name of the first link (linkA).
    * @param transformA pose of linkA.
    * @param linkNameB  name of the first link (linkB).
    * @param transformB pose of linkB.
    * @param closestPointsToReturn this is the output. Null if you want to ignore this information.
    * @return the distance between the closest points.
    */
   public double getClosestPoints(String linkNameA, RigidBodyTransform transformA, 
         String linkNameB, RigidBodyTransform transformB,
         ImmutablePair<Point3d, Point3d> closestPointsToReturn) throws Exception
   {
      
      LinkedList<Capsule> listA = capsulesByLink.get(linkNameA);
      LinkedList<Capsule> listB = capsulesByLink.get(linkNameB);
      
      if (listA == null){
         throw new Exception(" cant find link with name " + linkNameA);
      }

      if (listB == null){
         throw new Exception(" cant find link with name " + linkNameB);
      }

      Point3d p1 = new Point3d();
      Point3d p2 = new Point3d();
      double dist_min = Double.MAX_VALUE;

      for (int A = 0; A < listA.size(); A++)
      {
         Capsule c1 = new Capsule(listA.get(A));
         
         if( transformA != null)
              c1.transform(transformA);
         
         for (int B = 0; B < listB.size(); B++)
         {          
            Capsule c2 = new Capsule(listB.get(B));  

            if( transformB != null)
               c2.transform(transformB);

            double dist = Capsule.distanceQuery(c1, c2, p1, p2);
            if (dist < dist_min)
            {
               dist_min = dist;
               if( closestPointsToReturn != null)
               {
                  closestPointsToReturn.getLeft().set(p1);
                  closestPointsToReturn.getRight().set(p2);
               }
            }
         }
      }
      return dist_min;
   }

   /**
    * Load the capsules associated to a certain link of the robot.
    */
   public void loadFromURDF(File filein) throws ParserConfigurationException, SAXException, IOException
   {
      DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
      DocumentBuilder builder = factory.newDocumentBuilder();
      Document document = builder.parse(filein);
      loadFromURDF(document);
   }

   /**
    * Load the capsules associated to a certain link of the robot.
    * <p>
    * The format of the XML should be something like:
    * <pre>
    * {@code
    * 
    * <link name="name_of_link">
    *     ...
    *    <visual  name="capsule_p1">
    *     <geometry>  <sphere radius="0.12" />  </geometry>
    *        <origin xyz="0.02  0.01 0.04"  />
    *    </visual>   
    *    <visual  name="capsule_p2">
    *     <geometry>  <sphere radius="0.12" />  </geometry>
    *        <origin xyz="0.04  -0.03 0.29"  />
    *    </visual>
    * </link> 
    * 
    * }
    * </pre>  
    */
   public void loadFromURDF(Document document) throws ParserConfigurationException, SAXException, IOException
   {
      NodeList nodeList = document.getDocumentElement().getChildNodes();

      for (int i = 0; i < nodeList.getLength(); i++)
      {
         Node link_node = nodeList.item(i);

         if (link_node.getNodeType() == Node.ELEMENT_NODE && link_node.getNodeName().matches("link"))
         {
            Element elem = (Element) link_node;

            String link_name = link_node.getAttributes().getNamedItem("name").getNodeValue();

            Point3d p1 = new Point3d();
            Point3d p2 = new Point3d();
            double radius_p1 = -1;

            NodeList visuals = elem.getElementsByTagName("visual");
            boolean prev_point_p1 = false;

            LinkedList<Capsule> capsule_list     = new LinkedList<Capsule>();

            for (int c = 0; c < visuals.getLength(); c++)
            {
               Node vis_node = visuals.item(c);
               Element vis_element = (Element) vis_node;
               String vis_name = vis_element.getAttribute("name");
               Element origin = (Element) vis_element.getElementsByTagName("origin").item(0);

               if (vis_name != null && origin != null)
               {
                  Element geometry = (Element) vis_element.getElementsByTagName("geometry").item(0);
                  Element sphere = (Element) geometry.getElementsByTagName("sphere").item(0);
                  String[] xyz = origin.getAttribute("xyz").trim().split("\\s+");

                  if (xyz.length != 3)
                  {
                     throw new IOException("error in visual -> origin -> xyz");
                  }

                  if (!prev_point_p1) // expecting "capsule_p1"
                  {
                     if (vis_name.equals("capsule_p1"))
                     {
                        prev_point_p1 = true;
                        radius_p1 = Double.parseDouble(sphere.getAttribute("radius"));
                        if (radius_p1 <= 0)
                        {
                           throw new IOException("radius must be > 0");
                        }

                        p1.set(Double.parseDouble(xyz[0]), Double.parseDouble(xyz[1]), Double.parseDouble(xyz[2]));
                     }
                     else if (vis_name.equals("capsule_p2"))
                     {
                        throw new IOException("error in the order of capsule_p1 and capsule_p2");
                     }
                  }
                  else
                  {
                     if (vis_name.equals("capsule_p2"))
                     {
                        prev_point_p1 = false;
                        double radius_p2 = Double.parseDouble(sphere.getAttribute("radius"));
                        if (radius_p2 != radius_p1)
                        {
                           throw new IOException("radius of capsule_p2 must be the same as capsule_p1");
                        }
                        p2.set(Double.parseDouble(xyz[0]), Double.parseDouble(xyz[1]), Double.parseDouble(xyz[2]));

                        capsule_list.addLast(new Capsule(p1, p2, radius_p1));

                        numCapsules++;
                     }
                     else if (vis_name.equals("capsule_p1"))
                     {
                        throw new IOException("two consecutive capsule_p1 found");
                     }
                  }
               }
            } // end of for

            if (prev_point_p1)
            {
               throw new IOException("found capsule_p1 without capsule_p2");
            }
            if (capsule_list.size() > 0)
            {
               capsulesByLink.put(link_name, capsule_list);
            }
         }
      }
   }

}
