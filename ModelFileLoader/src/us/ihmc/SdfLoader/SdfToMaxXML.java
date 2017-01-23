package us.ihmc.SdfLoader;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.StringTokenizer;

import us.ihmc.tools.io.xml.XMLReaderUtility;


public class SdfToMaxXML
{
   String finalXML = "";
   int currentLocation = 0;

   public SdfToMaxXML()
   {
      // Create a file chooser
//    final JFileChooser fc = new JFileChooser();

      // In response to a button click:
//    int returnVal = fc.showOpenDialog(new JFrame());

//    if (returnVal == JFileChooser.APPROVE_OPTION)
      {
//       File file = fc.getSelectedFile();
         File file = new File("C:/RobotControl/SDFLoader/Models/Polaris_ranger/polaris_ranger_ev/model.sdf");

         String xml = loadFile(file);

         finalXML += "<Root>\n";
         if (!xml.equals(""))
            convert(xml);
         finalXML += "\n</Root>";

       System.out.println(finalXML);
         System.exit(0);
      }
   }



   private String loadFile(File sdfFile)
   {
      if (sdfFile == null)
         return "";

      try
      {
         if (sdfFile.exists())
         {
            BufferedReader reader = new BufferedReader(new FileReader(sdfFile));
            String xmlRepresentation = "";
            String tempLine;
            String fileEnding = ".xml";
            String name = sdfFile.getName().substring(0, sdfFile.getName().length() - fileEnding.length());

            while ((tempLine = reader.readLine()) != null)
            {
               xmlRepresentation += tempLine;
            }

            reader.close();

            return xmlRepresentation;

         }
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }

      return "";
   }

   private void convert(String xmlRepresentation)
   {
//    int collisionStart = XMLReaderUtility.getEndIndexOfSubString(currentLocation, xmlRepresentation, "collision");
      String tmp = "";
      do
      {
         tmp = XMLReaderUtility.getMiddleString(currentLocation, xmlRepresentation, "<collision", "</collision>");

         currentLocation = XMLReaderUtility.getEndIndexOfSubString(currentLocation, xmlRepresentation, "</collision>");

         // System.out.println(tmp);

         if (tmp != null)
         {
            if (tmp.contains("<box>"))
            {
               createBox(tmp);
            }

            if (tmp.contains("<cylinder>"))
            {
               createCylinder(tmp);
            }
         }
      }

      while (tmp != null);

   }

   private void createBox(String xmlForBox)
   {
      String name = XMLReaderUtility.getMiddleString(0, xmlForBox, "name=", ">");

      String xmlToAdd = "";
      xmlToAdd += "<node ";

      if ((name != null) && !name.equals(""))
      {
         xmlToAdd += "name=" + name + " ";
      }

      xmlToAdd += " class=\"Box\" ";

      String pose = XMLReaderUtility.getMiddleString(0, xmlForBox, "<pose>", "</pose>");
      if (pose != null)
      {
         pose.trim();

//       pose.replaceAll("  ", " ");
         StringTokenizer poseTok = new StringTokenizer(pose, " ");


         if (poseTok.countTokens() == 6)
         {
            xmlToAdd += "x=\"" + poseTok.nextToken().trim() + "\" ";
            xmlToAdd += "y=\"" + poseTok.nextToken().trim() + "\" ";
            xmlToAdd += "z=\"" + poseTok.nextToken().trim() + "\" ";
            xmlToAdd += "yaw=\"" + poseTok.nextToken().trim() + "\" ";
            xmlToAdd += "pitch=\"" + poseTok.nextToken().trim() + "\" ";
            xmlToAdd += "roll=\"" + poseTok.nextToken().trim() + "\" ";
         }
         else
         {
            System.out.println("invalid post values " + pose);
         }
      }
      else
      {
         xmlToAdd += "x=\"" + 0.0 + "\" ";
         xmlToAdd += "y=\"" + 0.0 + "\" ";
         xmlToAdd += "z=\"" + 0.0 + "\" ";
         xmlToAdd += "yaw=\"" + 0.0 + "\" ";
         xmlToAdd += "pitch=\"" + 0.0 + "\" ";
         xmlToAdd += "roll=\"" + 0.0 + "\" ";
      }

      xmlToAdd += " ";
      String size = XMLReaderUtility.getMiddleString(0, xmlForBox, "<size>", "</size>");
      size.trim();


      StringTokenizer sizeTok = new StringTokenizer(size, " ");

      if (sizeTok.countTokens() == 3)
      {
         xmlToAdd += "sx=\"" + sizeTok.nextToken().trim() + "\" ";
         xmlToAdd += "sy=\"" + sizeTok.nextToken().trim() + "\" ";
         xmlToAdd += "sz=\"" + sizeTok.nextToken().trim() + "\" ";
      }
      else
      {
         System.out.println("invalid size values " + size);
      }




      xmlToAdd += "/>\n";

      finalXML += xmlToAdd;



   }

   private void createCylinder(String xmlForCylinder)
   {
      String name = XMLReaderUtility.getMiddleString(0, xmlForCylinder, "name=", ">");

      String xmlToAdd = "";
      xmlToAdd += "<node ";

      if ((name != null) && !name.equals(""))
      {
         xmlToAdd += "name=" + name + " ";
      }

      xmlToAdd += " class=\"Cylinder\" ";

      String pose = XMLReaderUtility.getMiddleString(0, xmlForCylinder, "<pose>", "</pose>");

      if (pose != null)
      {
         pose.trim();

//       pose.replaceAll("  ", " ");
         StringTokenizer poseTok = new StringTokenizer(pose, " ");


         if (poseTok.countTokens() == 6)
         {
            xmlToAdd += "x=\"" + poseTok.nextToken().trim() + "\" ";
            xmlToAdd += "y=\"" + poseTok.nextToken().trim() + "\" ";
            xmlToAdd += "z=\"" + poseTok.nextToken().trim() + "\" ";
            xmlToAdd += "yaw=\"" + poseTok.nextToken().trim() + "\" ";
            xmlToAdd += "pitch=\"" + poseTok.nextToken().trim() + "\" ";
            xmlToAdd += "roll=\"" + poseTok.nextToken().trim() + "\" ";
         }
         else
         {
            System.out.println("invalid post values " + pose);
         }
      }
      else
      {
         xmlToAdd += "x=\"" + 0.0 + "\" ";
         xmlToAdd += "y=\"" + 0.0 + "\" ";
         xmlToAdd += "z=\"" + 0.0 + "\" ";
         xmlToAdd += "yaw=\"" + 0.0 + "\" ";
         xmlToAdd += "pitch=\"" + 0.0 + "\" ";
         xmlToAdd += "roll=\"" + 0.0 + "\" ";
      }


      xmlToAdd += " ";
      String size = XMLReaderUtility.getMiddleString(0, xmlForCylinder, "<radius>", "</radius>");
      size.trim();

      xmlToAdd += "radius=\"" + size.trim() + "\" ";

      xmlToAdd += " ";
      String length = XMLReaderUtility.getMiddleString(0, xmlForCylinder, "<length>", "</length>");
      length.trim();

      xmlToAdd += "length=\"" + length.trim() + "\" ";



      xmlToAdd += "/>\n";

      finalXML += xmlToAdd;



   }


   public static void main(String[] args)
   {
      new SdfToMaxXML();
   }
}
