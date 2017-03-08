package us.ihmc.manipulation.planning.rrt;

import java.util.ArrayList;
import java.util.Random;

import us.ihmc.tools.io.printing.PrintTools;

/**
 * <RRTPiecewisePath> class a node path divides into @param sizeOfPiecewise.
 * ArrayList<RRTNode> piecewisePath is initialized when this class is initiated.
 * @method updateShortCutPath() will return validity of the shortcut path after updating shortcutPath from the piecewisePath and linearPath.
 * 
 * @author InhoLee 170224
 *
 */
public class RRTPiecewisePath
{
   private ArrayList<RRTNode> linearPath = new ArrayList<RRTNode>();
   private ArrayList<RRTNode> piecewisePath = new ArrayList<RRTNode>();
   ArrayList<RRTNode> shortcutPath = new ArrayList<RRTNode>();
   private int[] pieceIndex;
   private int sizeOfPiecewisePath;

   private RRTNode nodeCreator;

   public RRTPiecewisePath(ArrayList<RRTNode> linearPath, int sizeOfPicewise)
   {
      this.linearPath = linearPath;
      this.sizeOfPiecewisePath = sizeOfPicewise;
      pieceIndex = new int[this.sizeOfPiecewisePath];
      nodeCreator = linearPath.get(0).createNode();
      initializePiecewisePath();
   }
   
   private void initializePiecewisePath()
   {
      double totalLength = 0;
      double pieceLength;

      for (int i = 0; i < linearPath.size() - 1; i++)
      {
         totalLength = totalLength + linearPath.get(i).getDistance(linearPath.get(i + 1));
      }

      pieceLength = totalLength / (sizeOfPiecewisePath - 1);

      RRTNode curPathNode;
      RRTNode nextPathNode;
      RRTNode precedentNode = linearPath.get(0);
      RRTNode succedentNode;

      // initial
      int curPiece = 0;
      curPathNode = linearPath.get(curPiece);
      nextPathNode = linearPath.get(curPiece + 1);
      addPathPiecewise(precedentNode, curPiece);

      for (int i = 1; i < sizeOfPiecewisePath - 1; i++)
      {
         succedentNode = nodeCreator.createNode();

         double nomalizedDistance;
         nomalizedDistance = precedentNode.getDistance(nextPathNode);
         double distanceToGo;

         if (nomalizedDistance >= pieceLength)
         {
            distanceToGo = pieceLength;
         }
         else
         {
            distanceToGo = pieceLength - nomalizedDistance;

            curPiece++;
            curPathNode = linearPath.get(curPiece);
            nextPathNode = linearPath.get(curPiece + 1);
            precedentNode = curPathNode;
         }

         setNodeDataInterpolation(precedentNode, nextPathNode, succedentNode, distanceToGo);

         addPathPiecewise(succedentNode, curPiece);
         precedentNode = succedentNode;
      }

      // final
      succedentNode = nodeCreator.createNode();
      for (int i = 0; i < precedentNode.getDimensionOfNodeData(); i++)
      {
         succedentNode.setNodeData(i, linearPath.get(linearPath.size() - 1).getNodeData(i));
      }
      addPathPiecewise(succedentNode, curPiece);
   }

   private void setNodeDataInterpolation(RRTNode nodeOne, RRTNode nodeTwo, RRTNode targetNode, double desLength)
   {
      double normailizedDistance = nodeOne.getDistance(nodeTwo);
      for (int i = 0; i < targetNode.getDimensionOfNodeData(); i++)
      {
         double dataOne = nodeOne.getNodeData(i);
         double dataTwo = nodeTwo.getNodeData(i);
         double interpolatedData = desLength / normailizedDistance * (dataTwo - dataOne) + dataOne;
         targetNode.setNodeData(i, interpolatedData);
      }
   }

   private void addPathPiecewise(RRTNode node, int idOfPiece)
   {
      piecewisePath.add(node);
      pieceIndex[piecewisePath.size() - 1] = idOfPiece;
   }

   private int getRandomNumber(int minIndex, int maxIndex)
   {
      Random randomManager = new Random();
      if (minIndex == maxIndex)
      {
         return minIndex;
      }
      else
      {
         return (randomManager.nextInt(maxIndex - minIndex) + minIndex);
      }
   }

   private int getRandomNumber(int minIndex, int maxIndex, int exceptNumber)
   {
      Random randomManager = new Random();
      int ret;
      while (true)
      {
         ret = randomManager.nextInt(maxIndex - minIndex) + minIndex;
         if (ret != exceptNumber)
         {
            break;
         }
      }
      return ret;
   }

   private int getRandomPiecewiseNodeIndex(int piece)
   {
      int minIndex = 0, maxIndex = 0;
      for (int i = 0; i < piecewisePath.size(); i++)
      {
         if (pieceIndex[i] == piece)
         {
            maxIndex = i;
         }
      }
      for (int i = piecewisePath.size() - 1; i > -1; i--)
      {
         if (pieceIndex[i] == piece)
         {
            minIndex = i;
         }
      }

      if (minIndex == 0 && maxIndex == 0)
      {
         return -1;
      }
      return getRandomNumber(minIndex, maxIndex);
   }

   private int[] getPiecePair()
   {
      int[] retPair = new int[2];

      retPair[0] = getRandomNumber(0, linearPath.size() - 1);
      retPair[1] = getRandomNumber(0, linearPath.size() - 1, retPair[0]);

      if (retPair[0] > retPair[1])
      {
         int pieceTemp = retPair[0];
         retPair[0] = retPair[1];
         retPair[1] = pieceTemp;
      }

      return retPair;
   }

   private boolean isValidPath(ArrayList<RRTNode> nodePath)
   {
      RRTPiecewisePath newPiecewisePath = new RRTPiecewisePath(nodePath, this.sizeOfPiecewisePath);

      for (int i = 0; i < newPiecewisePath.piecewisePath.size(); i++)
      {
         if (newPiecewisePath.piecewisePath.get(i).isValidNode() == false)
         {
            return false;
         }
      }
      return true;
   }

   public boolean updateShortCutPath()
   {
      shortcutPath.clear();

      int[] linearPiecePair;
      int[] piecewisePair = new int[2];

      while (true)
      {
         linearPiecePair = getPiecePair();
         piecewisePair[0] = getRandomPiecewiseNodeIndex(linearPiecePair[0]);
         piecewisePair[1] = getRandomPiecewiseNodeIndex(linearPiecePair[1]);
         if (piecewisePair[0] != -1 && piecewisePair[1] != -1)
         {
            break;
         }
      }

      for (int i = 0; i <= linearPiecePair[0]; i++)
      {
         shortcutPath.add(linearPath.get(i));
      }
      shortcutPath.add(piecewisePath.get(piecewisePair[0]));
      shortcutPath.add(piecewisePath.get(piecewisePair[1]));

      for (int i = linearPiecePair[1] + 1; i < linearPath.size(); i++)
      {
         shortcutPath.add(linearPath.get(i));
      }
     
      return isValidPath(shortcutPath);
   }
   
   public ArrayList<RRTNode> getLinearPath()
   {
      return linearPath;
   }
   
   public ArrayList<RRTNode> getPiecewisePath()
   {
      return piecewisePath;
   }

}
