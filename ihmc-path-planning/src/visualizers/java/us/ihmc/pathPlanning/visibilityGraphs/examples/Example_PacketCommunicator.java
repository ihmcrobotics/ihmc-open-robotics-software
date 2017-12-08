package us.ihmc.pathPlanning.visibilityGraphs.examples;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JPanel;

import javafx.application.Application;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage.RequestType;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.robotEnvironmentAwareness.LidarBasedREAStandaloneLauncher;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationKryoNetClassLists;
import us.ihmc.robotEnvironmentAwareness.simulation.LidarFastSimulation;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.tools.processManagement.JavaProcessSpawner;
import us.ihmc.commons.thread.ThreadTools;

/**
 * User: Matt Date: 1/14/13
 */
public class Example_PacketCommunicator extends Application
{
   ArrayList<Cluster> clusters = new ArrayList<>();
   ArrayList<PlanarRegion> regions = new ArrayList<>();
   ArrayList<PlanarRegion> accesibleRegions = new ArrayList<>();
   ArrayList<PlanarRegion> obstacleRegions = new ArrayList<>();

   PacketCommunicator packetCommunicator;
   private final String networkManagerHost = NetworkParameters.getHost(NetworkParameterKeys.networkManager);
   private PlanarRegionsList planarRegionsList;

   public Example_PacketCommunicator()
   {
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   
   {
      packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorClient(networkManagerHost, NetworkPorts.VISIBILITY_GRAPHS,
                                                                                REACommunicationKryoNetClassLists.getPublicNetClassList());
      packetCommunicator.connect();
      packetCommunicator.attachListener(PlanarRegionsListMessage.class, createPlanarRegionConsumer());
//      new AnimationTimer()
//      {
//
//         @Override
//         public void handle(long now)
//         {
//            packetCommunicator.send(new RequestPlanarRegionsListMessage(RequestType.CONTINUOUS_UPDATE, PacketDestination.REA_MODULE));
//
//         }
//      }.start();

      View3DFactory view3dFactory = new View3DFactory(640, 480);
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);

      TextureColorPalette colorPalette = new TextureColorAdaptivePalette();
      JavaFXMultiColorMeshBuilder javaFXMultiColorMeshBuilder = new JavaFXMultiColorMeshBuilder(colorPalette);

      MeshView meshView = new MeshView(javaFXMultiColorMeshBuilder.generateMesh());
      meshView.setMaterial(javaFXMultiColorMeshBuilder.generateMaterial());
      view3dFactory.addNodeToView(meshView);

      primaryStage.setScene(view3dFactory.getScene());
      primaryStage.show();

      JButton saveRegions = new JButton("Save Regions");
      saveRegions.addActionListener(new ActionListener()
      {

         @Override
         public void actionPerformed(ActionEvent arg0)
         {
            Thread thread = new Thread()
            {
               public void run()
               {
                  if (planarRegionsList != null)
                  {
                     System.out.println("Saving planar regions to file");
                     String data = "";

                     String filename = "PlanarRegions_";
                     if (filename.length() < 1)
                     {
                        filename = "LidarDefault_";
                     }

                     filename = filename + new SimpleDateFormat("yyyyMMddhhmm'.txt'").format(new Date());

                     File file = new File(filename);

                     try
                     {
                        // if file doesnt exists, then create it
                        if (!file.exists())
                        {
                           file.createNewFile();
                        }

                        FileWriter fw = new FileWriter(file.getAbsoluteFile());
                        BufferedWriter bw = new BufferedWriter(fw);

                        for (int j = 0; j < planarRegionsList.getNumberOfPlanarRegions(); j++) //planarRegionsList.getNumberOfPlanarRegions()
                        {
                           ConvexPolygon2D cp2d = planarRegionsList.getPlanarRegion(j).getConvexHull();
                           RigidBodyTransform transformToWorld = new RigidBodyTransform();
                           planarRegionsList.getPlanarRegion(j).getTransformToWorld(transformToWorld);

                           bw.write("PR_" + j);
                           bw.write(System.getProperty("line.separator"));

                           Vector3D translation = new Vector3D();
                           transformToWorld.getTranslation(translation);

                           Quaternion quat = new Quaternion();
                           transformToWorld.getRotation(quat);

                           bw.write("RBT," + translation + ", " + quat);
                           bw.write(System.getProperty("line.separator"));

                           for (int i = 0; i < cp2d.getNumberOfVertices(); i++)
                           {
                              Point2DReadOnly pt = cp2d.getVertexCCW(i);

                              FramePoint3D fpt = new FramePoint3D();
                              fpt.set(pt.getX(), pt.getY(), 0);
                              //                              fpt.applyTransform(transformToWorld);
                              data = fpt.getX() + ", " + fpt.getY() + ", " + fpt.getZ();
                              bw.write(data);
                              bw.write(System.getProperty("line.separator"));
                           }
                        }

                        bw.close();
                     }
                     catch (IOException e1)
                     {
                        // TODO Auto-generated catch block
                        e1.printStackTrace();
                     }
                  }
               }
            };

            thread.start();

         }
      });
      
      JButton requestPlanarRegions = new JButton("Request Regions");
      requestPlanarRegions.addActionListener(new ActionListener()
      {
         
         @Override
         public void actionPerformed(ActionEvent e)
         {
            System.out.println("Requesting LIDAR");
            packetCommunicator.send(new RequestPlanarRegionsListMessage(RequestType.SINGLE_UPDATE, PacketDestination.REA_MODULE));
         }
      });
      
      JPanel panel = new JPanel();
      panel.add(saveRegions);
      panel.add(requestPlanarRegions);
      
      JFrame frame = new JFrame("JFrame Source Demo");
      frame.setPreferredSize(new Dimension(175, 100));
      frame.getContentPane().add(panel, BorderLayout.CENTER);
      frame.pack();
      frame.setVisible(true);

   }

   private PacketConsumer<PlanarRegionsListMessage> createPlanarRegionConsumer()
   {
      return new PacketConsumer<PlanarRegionsListMessage>()
      {
         @Override
         public void receivedPacket(PlanarRegionsListMessage packet)
         {
            System.out.println("Received " + packet.getPlanarRegions().size() + " regions");
            planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(packet);
         }
      };
   }

   private void classifyRegions(ArrayList<PlanarRegion> regions)
   {
      for (PlanarRegion region : regions)
      {
         Vector3D normal = calculateNormal(region);

         if (normal != null)
         {
            if (Math.abs(normal.getZ()) < 0.5)
            {
               obstacleRegions.add(region);
               Cluster cluster = new Cluster();
               clusters.add(cluster);
               cluster.setObserverInLocal(clusters.get(0).getCentroidInLocal());

               for (int i = 0; i < region.getConvexHull().getNumberOfVertices(); i++)
               {
                  Point2D point2D = (Point2D) region.getConvexHull().getVertex(i);
                  Point3D point3D = new Point3D(point2D.getX(), point2D.getY(), 0);
                  FramePoint3D fpt = new FramePoint3D();
                  fpt.set(point3D);
                  RigidBodyTransform transToWorld = new RigidBodyTransform();
                  region.getTransformToWorld(transToWorld);
                  fpt.applyTransform(transToWorld);
                  Point3D pointToProject = fpt.getPoint();

                  cluster.addRawPointInWorld(pointToProject);
               }
            }
            else
            {
               accesibleRegions.add(region);
            }
         }
      }
   }

   private Vector3D calculateNormal(PlanarRegion region)
   {
      if (!region.getConvexHull().isEmpty())
      {
         FramePoint3D fpt1 = new FramePoint3D();
         fpt1.set(new Point3D(region.getConvexHull().getVertex(0).getX(), region.getConvexHull().getVertex(0).getY(), 0));
         RigidBodyTransform trans = new RigidBodyTransform();
         region.getTransformToWorld(trans);
         fpt1.applyTransform(trans);

         FramePoint3D fpt2 = new FramePoint3D();
         fpt2.set(new Point3D(region.getConvexHull().getVertex(1).getX(), region.getConvexHull().getVertex(1).getY(), 0));
         fpt2.applyTransform(trans);

         FramePoint3D fpt3 = new FramePoint3D();
         fpt3.set(new Point3D(region.getConvexHull().getVertex(2).getX(), region.getConvexHull().getVertex(2).getY(), 0));
         fpt3.applyTransform(trans);

         Vector3D normal = EuclidGeometryTools.normal3DFromThreePoint3Ds(fpt1.getPoint(), fpt2.getPoint(), fpt3.getPoint());
         return normal;
      }
      return null;
   }

   public ArrayList<Point3D> loadPointCloudFromFile(String fileName)
   {
      ArrayList<Cluster> clusters = new ArrayList<>();
      BufferedReader br = null;
      FileReader fr = null;

      try
      {

         //br = new BufferedReader(new FileReader(FILENAME));
         fr = new FileReader(fileName);
         br = new BufferedReader(fr);

         String sCurrentLine;

         double averageX = 0.0;
         double averageY = 0.0;
         double averageZ = 0.0;

         int index = 0;

         Cluster cluster = new Cluster();
         int nPacketsRead = 0;

         ArrayList<Point3D> pointsTemp = new ArrayList<>();

         while ((sCurrentLine = br.readLine()) != null)
         {
            //            System.out.println(sCurrentLine);

            if (sCurrentLine.contains("PR_"))
            {
               if (!pointsTemp.isEmpty())
               {
                  cluster.addRawPointsInWorld(pointsTemp, true);
                  pointsTemp.clear();
               }

               cluster = new Cluster();
               clusters.add(cluster);
               nPacketsRead++;
               //               System.out.println("New cluster created");
            }

            else if (sCurrentLine.contains("RBT,"))
            {
               //               System.out.println("Transformation read");
               sCurrentLine = sCurrentLine.substring(sCurrentLine.indexOf(",") + 1, sCurrentLine.length());

               sCurrentLine = sCurrentLine.replace("(", "");
               sCurrentLine = sCurrentLine.replace(")", "");

               String[] points = sCurrentLine.split(",");

               float x = (float) Double.parseDouble(points[0]);
               float y = (float) Double.parseDouble(points[1]);
               float z = (float) Double.parseDouble(points[2]);
               Vector3D translation = new Vector3D(x, y, z);

               float qx = (float) Double.parseDouble(points[3]);
               float qy = (float) Double.parseDouble(points[4]);
               float qz = (float) Double.parseDouble(points[5]);
               float qs = (float) Double.parseDouble(points[6]);
               Quaternion quat = new Quaternion(qx, qy, qz, qs);

               RigidBodyTransform rigidBodyTransform = new RigidBodyTransform(quat, translation);
               cluster.setTransformToWorld(rigidBodyTransform);
            }
            else
            {
               //               System.out.println("adding point");

               String[] points = sCurrentLine.split(",");

               float x = (float) Double.parseDouble(points[0]);
               float y = (float) Double.parseDouble(points[1]);
               float z = (float) Double.parseDouble(points[2]);

               pointsTemp.add(new Point3D(x, y, z));

               averageX = averageX + x;
               averageY = averageY + y;
               averageZ = averageZ + z;

               index++;
            }
         }

         for (Cluster cluster1 : clusters)
         {
            ArrayList<Point2D> vertices = new ArrayList<>();

            for (Point3D pt : cluster1.getRawPointsInWorld())
            {
               vertices.add(new Point2D(pt.getX(), pt.getY()));
            }

            ConvexPolygon2D convexPolygon = new ConvexPolygon2D(vertices);

            PlanarRegion planarRegion = new PlanarRegion(cluster1.getTransformToWorld(), convexPolygon);

            regions.add(planarRegion);
         }

         System.out.println("Loaded " + regions.size() + " regions");
      }
      catch (IOException e)
      {

         e.printStackTrace();

      } finally
      {

         try
         {

            if (br != null)
               br.close();

            if (fr != null)
               fr.close();

         }
         catch (IOException ex)
         {

            ex.printStackTrace();
         }

      }
      return null;

   }

   public static void main(String[] args)
   {
      ThreadTools.startAsDaemon(() -> startLidarFastSimulation(), "LidarSimStarter");
      startREA();
      launch();
   }

   private static void startLidarFastSimulation()
   {
      try
      {
         LidarFastSimulation lidarFastSimulation = new LidarFastSimulation();
         lidarFastSimulation.registerServer(PacketDestination.VISIBILITY_GRAPHS, NetworkPorts.VISIBILITY_GRAPHS,
                                            REACommunicationKryoNetClassLists.getPublicNetClassList());
         lidarFastSimulation.startSimulation();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   private static void startREA()
   {
      JavaProcessSpawner spawner = new JavaProcessSpawner(true);
      spawner.spawn(LidarBasedREAStandaloneLauncher.class);
   }
}